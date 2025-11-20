#ifdef ARDUINO_ARCH_RP2040
#include "ST77xxPIO.h"
#include "ST77xxDMA.h" // Для доступа к ST77xxConfig
#include <hardware/irq.h>
#include <pico/stdlib.h>
#include <hardware/clocks.h>

// Экземпляр для обработчика прерывания
static ST77xxPIO* _pio_instance = nullptr;

// ISR-обертка
extern "C" void dma_irh_for_pio_chain() {
    if (_pio_instance) _pio_instance->dma_irh();
}

// PIO программа для 16bpp (MSB-first), без автопула, 2 пикселя (32 бита) на 1 pull.
// Адаптирована под произвольные пины: 1-битный sideset (только SCLK), CS ведём через SET pins.
// setup: set x,9; mov isr<-x; in zeroes,10; mov x<-isr; set pins,0 (CS=0)
// цикл: pull (32 бита: [pix_hi(31:16)|pix_lo(15:0)]);
//       set y,15; 16x: out pins,1 | sideset 0/1 (выдаём верхние 16 бит => первый пиксель);
//       mov isr, osr; in null,16; mov osr, isr  (сдвиг оставшихся 16 бит в MSB);
//       set y,15; 16x: out pins,1 | sideset 0/1 (выдаём нижние 16 бит => второй пиксель);
//       jmp к pull
static const uint16_t pio_stream_program_instructions[] = {
    (uint16_t)pio_encode_set(pio_x, 9),                        // X <- 0x09
    (uint16_t)pio_encode_mov(pio_isr, pio_x),                  // ISR = X
    (uint16_t)pio_encode_in(pio_null, 10),                     // набить 10 нулей
    (uint16_t)pio_encode_mov(pio_x, pio_isr),                  // X <- ISR (нечётное)
    // Установим CS=0 через SET PINS (база set_pins = CS, ширина=1)
    (uint16_t)pio_encode_set(pio_pins, 0),
    // pull_n_go:
    (uint16_t)pio_encode_pull(true, true),                     // блокирующий pull (32 бита)
    // Первый пиксель: верхние 16 бит
    (uint16_t)pio_encode_set(pio_y, 15),                       // Y = 15 (16 бит)
    // more_bits_hi:
    (uint16_t)(pio_encode_out(pio_pins, 1) | pio_encode_sideset(1, 0)), // CLK=0
    (uint16_t)(pio_encode_jmp_y_dec(8)     | pio_encode_sideset(1, 1)), // CLK=1 (петля на out)
    // Сдвиг OSR на 16 бит влево (надёжно):
    (uint16_t)pio_encode_mov(pio_isr, pio_osr),                // ISR = OSR
    (uint16_t)pio_encode_in(pio_null, 16),                     // ISR <<= 16
    (uint16_t)pio_encode_mov(pio_osr, pio_isr),                // OSR = ISR (нижние 16 в MSB)
    // Второй пиксель: нижние 16 бит
    (uint16_t)pio_encode_set(pio_y, 15),                       // Y = 15 (16 бит)
    // more_bits_lo:
    (uint16_t)(pio_encode_out(pio_pins, 1) | pio_encode_sideset(1, 0)), // CLK=0
    (uint16_t)(pio_encode_jmp_y_dec(13)    | pio_encode_sideset(1, 1)), // CLK=1 (петля на out)
    (uint16_t)(pio_encode_jmp(5)           | pio_encode_sideset(1, 0)), // CLK=0 -> к pull_n_go
};

static const struct pio_program pio_stream_program = {
    .instructions = pio_stream_program_instructions,
    .length = sizeof(pio_stream_program_instructions) / sizeof(uint16_t),
    .origin = -1,
};

// PIO программа для префикса (ровно по 1 байту на PULL):
//   pull  block
//   out   x, 8
//   mov   osr, x
//   out   null, 24
//   set   y, 7
//   [8 циклов тактирования SCLK с выдачей бита]
//   jmp   0
static const uint16_t pio_cmd_program_instructions[] = {
    //           /     .wrap_target
    (uint16_t)pio_encode_pull(true, false),
    (uint16_t)pio_encode_out(pio_x, 8),
    (uint16_t)pio_encode_mov(pio_osr, pio_x),
    (uint16_t)pio_encode_out(pio_null, 24),
    (uint16_t)pio_encode_set(pio_y, 7),
    (uint16_t)(pio_encode_out(pio_pins, 1) | pio_encode_sideset(1, 0)),
    (uint16_t)(pio_encode_nop() | pio_encode_sideset(1, 1)),
    (uint16_t)(pio_encode_jmp_y_dec(5) | pio_encode_sideset(1, 0)),
    (uint16_t)pio_encode_jmp(0),
    //           \     .wrap
};

static const struct pio_program pio_cmd_program = {
    .instructions = pio_cmd_program_instructions,
    .length = sizeof(pio_cmd_program_instructions) / sizeof(uint16_t),
    .origin = -1,
};

// (удалено) Ранее здесь был черновой код подготовки пикселей на второй SM.

ST77xxPIO::ST77xxPIO(const ST77xxConfig& config) : 
    _mosi(config.mosi), _sclk(config.sclk), _dc(config.dc), _cs(config.cs), _rst(config.rst),
    _debug(config.debug), _spi_hz(config.spi_hz)
{
    _pio_instance = this;
}

ST77xxPIO::~ST77xxPIO() {
    end();
}

void ST77xxPIO::begin() {
    if (_debug) printf("ST77xxPIO::begin() PIO SPI on pins: MOSI=%d, SCLK=%d, DC=%d, CS=%d\n", _mosi, _sclk, _dc, _cs);

    // 1) Инициализация GPIO
    pio_gpio_init(_pio, _mosi);
    pio_gpio_init(_pio, _sclk);
    if (_dc >= 0) { pio_gpio_init(_pio, _dc); }
    // CS теперь ведётся PIO через SET, поэтому переведём пин в функцию PIO
    if (_cs >= 0) { pio_gpio_init(_pio, _cs); }
    if (_rst >= 0) {
        gpio_init(_rst); gpio_set_dir(_rst, GPIO_OUT);
        gpio_put(_rst, 1); sleep_ms(5);
        gpio_put(_rst, 0); sleep_ms(20);
        gpio_put(_rst, 1); sleep_ms(150);
    }

    // 2) Захват SM и загрузка программы
    _sm_output = pio_claim_unused_sm(_pio, true);
    // Сначала добавим программу для получения свободного offset
    _pio_output_program_offset = pio_add_program(_pio, &pio_stream_program);
    // Пересоберём инструкции со скорректированными адресами переходов (absolute = offset + index)
    uint16_t prog[sizeof(pio_stream_program_instructions)/sizeof(uint16_t)];
    // Индексы внутри pio_stream_program_instructions (только те, что реально используются)
    const uint8_t IDX_PULL          = 5;
    const uint8_t IDX_OUT_HI        = 7;  // out pins,1 (hi)
    const uint8_t IDX_JMPDEC_HI     = 8;  // jmp y-- -> OUT_HI
    const uint8_t IDX_OUT_LO        = 13; // out pins,1 (lo)
    const uint8_t IDX_JMPDEC_LO     = 14; // jmp y-- -> OUT_LO
    const uint8_t IDX_JMP_PULL      = 15; // jmp -> PULL

    // Скопируем основу
    for (size_t i = 0; i < sizeof(prog)/sizeof(prog[0]); ++i) prog[i] = pio_stream_program_instructions[i];
    // Исправим jmp-ы на абсолютные адреса (sideset non-optional: 1 бит, значения 0/1)
    prog[IDX_JMPDEC_HI] = (uint16_t)(pio_encode_jmp_y_dec(_pio_output_program_offset + IDX_OUT_HI) | pio_encode_sideset(1, 1));
    prog[IDX_JMPDEC_LO] = (uint16_t)(pio_encode_jmp_y_dec(_pio_output_program_offset + IDX_OUT_LO) | pio_encode_sideset(1, 1));
    prog[IDX_JMP_PULL]  = (uint16_t)(pio_encode_jmp(_pio_output_program_offset + IDX_PULL)         | pio_encode_sideset(1, 0));

    // Удалим ранее добавленную версию и загрузим скорректированную точно в тот же offset
    pio_remove_program(_pio, &pio_stream_program, _pio_output_program_offset);
    pio_add_program_at_offset(_pio, &pio_stream_program, _pio_output_program_offset);
    // Перезапишем скорректированные слова непосредственно в память инструкций PIO
    for (size_t i = 0; i < sizeof(prog)/sizeof(prog[0]); ++i) {
        _pio->instr_mem[_pio_output_program_offset + i] = prog[i];
    }
    int cmd_program_offset = pio_add_program(_pio, &pio_cmd_program);
    _sm_cmd = pio_claim_unused_sm(_pio, true);

    // Больше не требуется соседство CS/SCLK: 1-битный sideset только на SCLK, CS ведём через SET pins

    // 3) Конфигурация SM (поток пикселей)
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, _pio_output_program_offset + 0, _pio_output_program_offset + pio_stream_program.length - 1);
    // sideset: 1 бит (SCLK), optional=FALSE (каждая инструкция несёт значение SCLK)
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, _sclk);
    // MOSI как out pins шириной 1 бит
    sm_config_set_out_pins(&c, _mosi, 1);
    // Направления пинов: MOSI,SCLK,CS — на выход
    pio_sm_set_consecutive_pindirs(_pio, _sm_output, _mosi, 1, true);
    if (_sclk >= 0) pio_sm_set_consecutive_pindirs(_pio, _sm_output, _sclk, 1, true);
    if (_cs   >= 0) pio_sm_set_consecutive_pindirs(_pio, _sm_output, _cs,   1, true);
    // CS как set pins (PIO управляет уровнем CS через SET)
    if (_cs >= 0) sm_config_set_set_pins(&c, _cs, 1);
    // MSB-first: сдвиг влево, БЕЗ автопула (pull в программе), порог 32
    sm_config_set_out_shift(&c, false, false, 32);
    // Нормализуем частоту и зададим безопасное значение по умолчанию
    uint32_t target_hz = _spi_hz;
    if (target_hz == 0) target_hz = 1000000u; // 1 МГц по умолчанию
    if (target_hz <= 512) target_hz *= 1000000u; // шортхенд МГц
    // Частота тактирования
    float clkdiv = (float)clock_get_hz(clk_sys) / (float)target_hz;
    if (_debug) printf("PIO clkdiv=%.2f (sys=%lu Hz, target=%lu Hz)\n", clkdiv, (unsigned long)clock_get_hz(clk_sys), (unsigned long)target_hz);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_init(_pio, _sm_output, _pio_output_program_offset + 0, &c);
    // Не включаем SM вывода до подготовки X и DMA

    // Конфигурация SM для командного префикса (без автопула)
    pio_sm_config cc = pio_get_default_sm_config();
    sm_config_set_wrap(&cc, cmd_program_offset + 0, cmd_program_offset + pio_cmd_program.length - 1);
    // Для команд используем 1-битный sideset на SCLK
    sm_config_set_sideset(&cc, 1, false, false);
    sm_config_set_sideset_pins(&cc, _sclk);
    sm_config_set_out_pins(&cc, _mosi, 1);
    pio_sm_set_consecutive_pindirs(_pio, _sm_cmd, _mosi, 1, true);
    pio_sm_set_consecutive_pindirs(_pio, _sm_cmd, _sclk, 1, true);
    // DC как set pins в командном SM
    if (_dc >= 0) { sm_config_set_set_pins(&cc, _dc, 1); pio_sm_set_consecutive_pindirs(_pio, _sm_cmd, _dc, 1, true); }
    // CS как set pins в командном SM (чтобы удерживать CS низким в префиксе без включения потокового SM)
    if (_cs >= 0) { sm_config_set_set_pins(&cc, _cs, 1); pio_sm_set_consecutive_pindirs(_pio, _sm_cmd, _cs, 1, true); }
    // При желании можно добавить и CS как set_pins для командного SM, но CS удерживается низким SM потока
    sm_config_set_out_shift(&cc, false, false, 32); // autopull=0
    sm_config_set_clkdiv(&cc, clkdiv);
    pio_sm_init(_pio, _sm_cmd, cmd_program_offset + 0, &cc);
    pio_sm_set_enabled(_pio, _sm_cmd, true);

    // 4) DMA каналы и IRQ
    _dma_ch_cmd   = dma_claim_unused_channel(true);
    _dma_ch_pixel = dma_claim_unused_channel(true);
    _dma_ch_ctrl2 = dma_claim_unused_channel(true);

    // IRQ0 обработчик
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irh_for_pio_chain);
    // Очистим возможные висячие флаги, сбросим счетчик кадров и включим прерывания на канале PIX
    dma_hw->ints0 = dma_hw->ints0; // ack все флаги
    hwFrameCount = 0;
    dma_channel_set_irq0_enabled(_dma_ch_pixel, true);
    irq_set_enabled(DMA_IRQ_0, true);
}

void ST77xxPIO::end() {
    if (_sm_output >= 0) pio_sm_set_enabled(_pio, _sm_output, false);
    if (_dma_ch_cmd >= 0) dma_channel_abort(_dma_ch_cmd);
    if (_dma_ch_pixel >= 0) dma_channel_abort(_dma_ch_pixel);
    if (_dma_ch_ctrl2 >= 0) dma_channel_abort(_dma_ch_ctrl2);

    irq_set_enabled(DMA_IRQ_0, false);

    if (_dma_ch_cmd >= 0) dma_channel_unclaim(_dma_ch_cmd);
    if (_dma_ch_pixel >= 0) dma_channel_unclaim(_dma_ch_pixel);
    if (_dma_ch_ctrl2 >= 0) dma_channel_unclaim(_dma_ch_ctrl2);
    if (_sm_cmd >= 0) pio_sm_unclaim(_pio, _sm_cmd);
    if (_sm_output >= 0) pio_sm_unclaim(_pio, _sm_output);
}

void ST77xxPIO::send_command(uint8_t cmd, const uint8_t* data, size_t len) {
    // Отправка одиночной команды: DC переключаем через PIO (SM команд), байты через DMA в SM.
    // CS удерживается PIO: потоковый SM уже поставил CS=0 и блокируется на pull.
    if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 0)); // DC=0

    // Передаем 1 байт команды (по одному 32-бит слову на байт в SM префикса)
    uint32_t cmd_word = ((uint32_t)cmd) << 24; // байт в [31:24]
    dma_channel_config cfg = dma_channel_get_default_config(_dma_ch_cmd);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_dreq(&cfg, pio_get_dreq(_pio, _sm_cmd, true));
    dma_channel_configure(_dma_ch_cmd, &cfg,
                          &_pio->txf[_sm_cmd], &cmd_word, 1, true);
    dma_channel_wait_for_finish_blocking(_dma_ch_cmd);

    if (len && data) {
        if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 1)); // DC=1
        // Упакуем данные по 4 байта в слова
        uint32_t tmp[8]; // хватает для коротких команд; при необходимости сделать циклом
        size_t sent = 0;
        while (sent < len) {
            size_t chunk = min((size_t)32, len - sent); // до 32 байт за раз
            size_t wcount = (chunk + 3) / 4;
            for (size_t i = 0; i < wcount; ++i) {
                uint32_t w = 0;
                for (size_t b = 0; b < 4; ++b) {
                    size_t idx = sent + i * 4 + b;
                    uint8_t v = (idx < sent + chunk) ? data[idx] : 0;
                    w |= (uint32_t)v << (24 - b * 8);
                }
                tmp[i] = w;
            }
            dma_channel_config cfgd = dma_channel_get_default_config(_dma_ch_cmd);
            channel_config_set_transfer_data_size(&cfgd, DMA_SIZE_32);
            channel_config_set_dreq(&cfgd, pio_get_dreq(_pio, _sm_cmd, true));
            dma_channel_configure(_dma_ch_cmd, &cfgd,
                                  &_pio->txf[_sm_cmd], tmp, wcount, true);
            dma_channel_wait_for_finish_blocking(_dma_ch_cmd);
            sent += chunk;
        }
    }
    // CS не трогаем — управляет PIO (SM потока)
}

void ST77xxPIO::startContinuousRefreshHW(uint16_t* framebuffer, uint16_t w, uint16_t h, uint8_t x, uint8_t y) {
    // 0) Перед префиксом НЕ включаем потоковый SM, чтобы избежать конфликта на SCLK.
    //    CS удержим низким через командный SM (PIO set-pins).

    // Для теста PIO канала: заполним буфер тестовым паттерном (белый прямоугольник 50x50 в центре)
    if (framebuffer) {
        // Заполним весь буфер черным
        uint16_t black_be = ST77xx::toBE(ST77xx::C::Black);
        for (size_t i = 0; i < (size_t)w * h; ++i) framebuffer[i] = black_be;
        // Белый прямоугольник 50x50 в центре
        int rect_w = 50, rect_h = 50;
        int start_x = (w - rect_w) / 2;
        int start_y = (h - rect_h) / 2;
        uint16_t white_be = ST77xx::toBE(ST77xx::C::White);
        for (int yy = start_y; yy < start_y + rect_h; ++yy) {
            for (int xx = start_x; xx < start_x + rect_w; ++xx) {
                if (xx >= 0 && xx < w && yy >= 0 && yy < h) {
                    framebuffer[yy * w + xx] = white_be;
                }
            }
        }
    }

    // 1) Префикс окна и RAMWR (полностью через DMA->PIO, через SM команд), DC переключаем PIO.
    //    ВАЖНО: CS низкий — его удерживает командный SM (PIO), GPIO не трогаем.
    if (_cs >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 0)); // CS=0
    uint16_t x_end = x + w - 1;
    uint16_t y_end = y + h - 1;
    uint8_t caset_cmd = 0x2A;
    uint8_t caset_args[4] = { (uint8_t)(x >> 8), (uint8_t)(x & 0xFF), (uint8_t)(x_end >> 8), (uint8_t)(x_end & 0xFF) };
    uint8_t raset_cmd = 0x2B;
    uint8_t raset_args[4] = { (uint8_t)(y >> 8), (uint8_t)(y & 0xFF), (uint8_t)(y_end >> 8), (uint8_t)(y_end & 0xFF) };
    uint8_t ramwr_cmd = 0x2C;

    // CASET (DC=0 затем DC=1 для аргументов)
    if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 0));
    {
        uint32_t w = ((uint32_t)caset_cmd) << 24;
        dma_channel_config c = dma_channel_get_default_config(_dma_ch_cmd);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_dreq(&c, pio_get_dreq(_pio, _sm_cmd, true));
        dma_channel_configure(_dma_ch_cmd, &c, &_pio->txf[_sm_cmd], &w, 1, true);
        dma_channel_wait_for_finish_blocking(_dma_ch_cmd);
    }
    if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 1));
    {
        uint32_t buf[1];
        buf[0] = ((uint32_t)caset_args[0] << 24) | ((uint32_t)caset_args[1] << 16) | ((uint32_t)caset_args[2] << 8) | caset_args[3];
        dma_channel_config c = dma_channel_get_default_config(_dma_ch_cmd);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_dreq(&c, pio_get_dreq(_pio, _sm_cmd, true));
        dma_channel_configure(_dma_ch_cmd, &c, &_pio->txf[_sm_cmd], buf, 1, true);
        dma_channel_wait_for_finish_blocking(_dma_ch_cmd);
    }

    // RASET
    if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 0));
    {
        uint32_t w = ((uint32_t)raset_cmd) << 24;
        dma_channel_config c = dma_channel_get_default_config(_dma_ch_cmd);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_dreq(&c, pio_get_dreq(_pio, _sm_cmd, true));
        dma_channel_configure(_dma_ch_cmd, &c, &_pio->txf[_sm_cmd], &w, 1, true);
        dma_channel_wait_for_finish_blocking(_dma_ch_cmd);
    }
    if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 1));
    {
        uint32_t buf[1];
        buf[0] = ((uint32_t)raset_args[0] << 24) | ((uint32_t)raset_args[1] << 16) | ((uint32_t)raset_args[2] << 8) | raset_args[3];
        dma_channel_config c = dma_channel_get_default_config(_dma_ch_cmd);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_dreq(&c, pio_get_dreq(_pio, _sm_cmd, true));
        dma_channel_configure(_dma_ch_cmd, &c, &_pio->txf[_sm_cmd], buf, 1, true);
        dma_channel_wait_for_finish_blocking(_dma_ch_cmd);
    }

    // RAMWR
    if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 0));
    {
        uint32_t w = ((uint32_t)ramwr_cmd) << 24;
        dma_channel_config c = dma_channel_get_default_config(_dma_ch_cmd);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_dreq(&c, pio_get_dreq(_pio, _sm_cmd, true));
        dma_channel_configure(_dma_ch_cmd, &c, &_pio->txf[_sm_cmd], &w, 1, true);
        dma_channel_wait_for_finish_blocking(_dma_ch_cmd);
    }
    if (_dc >= 0) pio_sm_exec(_pio, _sm_cmd, pio_encode_set(pio_pins, 1)); // DC=1 зафиксируем через командный SM
    // Дальше CS будет удерживаться низким PIO (SET) в SM вывода пикселей
    // Отключаем SM команд, чтобы не было конфликта вождения линий с SM потока
    if (_sm_cmd >= 0) pio_sm_set_enabled(_pio, _sm_cmd, false);

    // 2) Запомним адрес и длину кадра для перезапуска
    _dma_pixel_read_addr_ptr = framebuffer;
    size_t pixel_count = (size_t)w * (size_t)h;              // количество пикселей (16-бит)
    // В PIO потоке теперь 2 пикселя (32 бита) на 1 pull. Передаём 32-битные слова DMA.
    // Убедимся, что число пикселей чётное (для ST77xx типично так и есть: 170x320 и т.п.).
    size_t word_count = pixel_count / 2;                      // количество 32-битных слов

    // Если адрес буфера выровнен на 2, но не на 4 (addr % 4 == 2), 32-битный DMA не стартует.
    // В этом случае заранее выталкиваем первое 32-битное слово вручную (pix0|pix1),
    // а источник DMA сдвигаем на +1 пиксель до 4-байтового выравнивания.
    if ((((uintptr_t)_dma_pixel_read_addr_ptr) & 0x3u) == 0x2u && pixel_count >= 2) {
        const uint16_t* fb16 = (const uint16_t*)_dma_pixel_read_addr_ptr;
        uint32_t wpair = ((uint32_t)fb16[0] << 16) | fb16[1];
        // Запишем в TX FIFO до старта SM, чтобы не было гонки
        pio_sm_put_blocking(_pio, _sm_output, wpair);
        _dma_pixel_read_addr_ptr = fb16 + 2; // смещение на 1 слово (2 пикселя) -> адрес кратен 4
        if (word_count) --word_count;
    }

    // 3) Настройка DMA каналов (32-бит) с явным рестартом
    // Канал PIX: word (32b) -> TXF SM, DREQ на SM TX, incr_read, no incr_write, chain_to CTRL2
    dma_channel_config cfg_pix = dma_channel_get_default_config(_dma_ch_pixel);
    channel_config_set_transfer_data_size(&cfg_pix, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_pix, true);
    channel_config_set_write_increment(&cfg_pix, false);
    channel_config_set_dreq(&cfg_pix, pio_get_dreq(_pio, _sm_output, true));
    // По завершению PIX запускаем CTRL2 (он перезарядит TRANS_COUNT и триггернёт READ_ADDR)
    channel_config_set_chain_to(&cfg_pix, _dma_ch_ctrl2);
    if (word_count > 0) {
        dma_channel_configure(_dma_ch_pixel, &cfg_pix,
                              &_pio->txf[_sm_output],                 // dst
                              (const uint32_t*)_dma_pixel_read_addr_ptr, // src (uint32_t words)
                              word_count,                               // count (words)
                              false);
    }
    // Снимем сформированный CTRL PIX и добавим EN для перезапуска по CTRL2
    dma_channel_hw_t* pix_hw = &dma_hw->ch[_dma_ch_pixel];
    _pix_ctrl_en = pix_hw->al1_ctrl | DMA_CH0_CTRL_TRIG_EN_BITS;

    // Канал CTRL2: 3 слова подряд пишет в регистры PIX:
    //   [0] -> CHx.AL1_CTRL (с установленным EN)
    //   [1] -> CHx.AL2_TRANSFER_COUNT  (число 32-бит слов word_count)
    //   [2] -> CHx.AL3_READ_ADDR_TRIG  (адрес framebuffer, триггер запуска)
    // Источник: _ctrl2_words = { _pix_ctrl_en, word_count, (uint32_t)_dma_pixel_read_addr_ptr }
    _ctrl2_words[0] = (uint32_t)_pix_ctrl_en;
    _ctrl2_words[1] = (uint32_t)word_count;
    _ctrl2_words[2] = (uint32_t)_dma_pixel_read_addr_ptr;
    dma_channel_config cfg_ctrl2 = dma_channel_get_default_config(_dma_ch_ctrl2);
    channel_config_set_transfer_data_size(&cfg_ctrl2, DMA_SIZE_32);
    channel_config_set_dreq(&cfg_ctrl2, DREQ_FORCE);
    channel_config_set_read_increment(&cfg_ctrl2, true);
    channel_config_set_write_increment(&cfg_ctrl2, true);
    // Не делаем self-loop! По завершении CTRL2 чейним на PIX, хотя READ_ADDR_TRIG уже запускает PIX.
    channel_config_set_chain_to(&cfg_ctrl2, _dma_ch_pixel);
    dma_channel_configure(_dma_ch_ctrl2, &cfg_ctrl2,
                          &pix_hw->al1_ctrl,   // dst (начать с CTRL)
                          _ctrl2_words,        // src ([CTRL_EN],[COUNT],[ADDR_TRIG])
                          3,                   // три слова подряд
                          false);

    // 4) Подготовить SM вывода: сбросить FIFO/PC
    pio_sm_set_enabled(_pio, _sm_output, false);
    pio_sm_clear_fifos(_pio, _sm_output);
    pio_sm_exec(_pio, _sm_output, pio_encode_jmp(_pio_output_program_offset + 0));

    // 5) Запуск: ПРИМАЕМ PIX один раз (стартуем передачу), чтобы заранее наполнить TX FIFO,
    //    затем включаем SM вывода. CTRL2 не запускаем вручную — он пойдёт по chain.
    // Тестовый прогон: заполним TX FIFO первыми словами кадра напрямую, чтобы исключить проблемы с DMA
    if (_debug && framebuffer) {
        size_t prim_words = (word_count > 32) ? 32 : word_count;
        const uint32_t* fb32 = (const uint32_t*)framebuffer;
        for (size_t i = 0; i < prim_words; ++i) {
            pio_sm_put_blocking(_pio, _sm_output, fb32[i]);
        }
    }
    dma_channel_start(_dma_ch_pixel);   // первичный старт потока пикселей (prefill TXF до включения SM)
    pio_sm_set_enabled(_pio, _sm_output, true);
}

 void ST77xxPIO::dma_irh() {
    // Прерывание от канала пикселей означает конец кадра (циклическая передача перезапущена chain)
    if (dma_channel_get_irq0_status(_dma_ch_pixel)) {
        dma_channel_acknowledge_irq0(_dma_ch_pixel);
        hwFrameCount++;
    }
}

#endif // ARDUINO_ARCH_RP2040
