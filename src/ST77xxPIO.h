#pragma once
#ifdef ARDUINO_ARCH_RP2040

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/dma.h>

struct ST77xxConfig; // Forward declaration

class ST77xxPIO {
public:
    explicit ST77xxPIO(const ST77xxConfig& config);
    ~ST77xxPIO();

    void begin();
    void end();

    // Отправка команд инициализации
    void send_command(uint8_t cmd, const uint8_t* data = nullptr, size_t len = 0);

    // Запуск непрерывного обновления экрана из фреймбуфера для заданной области
    void startContinuousRefreshHW(uint16_t* framebuffer, uint16_t w, uint16_t h, uint8_t x = 0, uint8_t y = 0);

    // Обработчик прерывания DMA (публичный для C-обертки)
    void dma_irh();

    volatile uint32_t hwFrameCount = 0;

private:
    // Конфигурация пинов
    int8_t _mosi, _sclk, _dc, _cs, _rst;
    bool _debug;
    int _spi_hz; // SPI frequency in Hz

    // PIO ресурсы
    PIO _pio = pio0;
    int _sm_output = -1;   // SM для потока пикселей
    int _sm_cmd    = -1;   // SM для коротких командных пакетов (префикс)
    int _sm_cs     = -1;   // SM для управления CS (через set pins)
    int _sm_rst    = -1;   // SM для управления RST (через set pins)
    uint _pio_output_program_offset = 0; // общий код для обеих SPI-SM

    // DMA ресурсы
    int _dma_ch_cmd = -1;    // Канал для команд и префикса кадра
    int _dma_ch_pixel = -1;  // Канал для пикселей
    int _dma_ch_ctrl2 = -1;  // Канал для перезапуска (CTRL->TRANSFER_COUNT->chain PIX)

    // Буфер для команд и префикса (CASET, RASET, RAMWR)
    static const size_t CMD_BUFFER_SIZE = 16;
    uint8_t _dma_command_buffer[CMD_BUFFER_SIZE];

    // Указатель на начало пиксельных данных для перезапуска DMA
    const void* _dma_pixel_read_addr_ptr = nullptr;

    // Постоянный буфер слов для CTRL-канала (READ_ADDR)
    uint32_t _ctrl_words[2] = {0, 0};
    // Постоянный буфер для CTRL2: [AL1_CTRL(EN)][AL2_TRANSFER_COUNT][AL3_READ_ADDR_TRIG]
    uint32_t _ctrl2_words[3] = {0, 0, 0};
    // Сохраненное значение CTRL с установленным EN для PIX
    uint32_t _pix_ctrl_en = 0;
};

#endif // ARDUINO_ARCH_RP2040
