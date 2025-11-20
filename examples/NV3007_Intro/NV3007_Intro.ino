#include <Arduino.h>
#include <ST77xxDMA.h>
#include <SPI.h>
#if defined(ARDUINO_ARCH_RP2040)
#include <pico/multicore.h>
#include <hardware/sync.h>
#endif
// Разрешаем короткий вызов setBrightness(...)
using ST77xx::setBrightness;
// Шрифт для HUD (Adafruit GFX формат)
#include <Fonts/GFXFF/font_mod/kurze/times9pt7b.h>
// ---- Выбор панели ----
static const ST77xxPanel PANEL = ST77xxPanel::NV3007_428x142;
 
// RP2040: используем два ядра для ровного распределения расчёта кадра
// и RP2040 и  esp32 s3 и только они из-за двух ядерного режима раоты!
// --- Пинмапы по платформам ---
#if defined(ARDUINO_ARCH_ESP32)
  #define PIN_MOSI 11
  #define PIN_MISO -1
  #define PIN_SCLK 12
  #define PIN_CS    10
  #define PIN_DC    9
  #define PIN_RST   8
  #define PIN_BL   5
#elif defined(ESP8266)
  #define PIN_MOSI D7
  #define PIN_MISO -1
  #define PIN_SCLK D5
  #define PIN_CS   D8
  #define PIN_DC   D0
  #define PIN_RST  D2
  #define PIN_BL   D1
#elif defined(ARDUINO_ARCH_RP2040)
  #define PIN_MOSI 7
  #define PIN_MISO -1
  #define PIN_SCLK 6
  #define PIN_CS   10
  #define PIN_DC   9
  #define PIN_RST  8
  #define PIN_BL   11
#else
  #define PIN_MOSI 7
  #define PIN_MISO -1
  #define PIN_SCLK 6
  #define PIN_CS   10
  #define PIN_DC    9
  #define PIN_RST   8
  #define PIN_BL    11
#endif

ST77xxDMA* lcd = nullptr;
static uint16_t* g_backbuf = nullptr; // указатель на внутренний framebuffer (RGB565 LE)
static int g_w = 0, g_h = 0;
static uint32_t g_fps_t0 = 0;
static uint32_t g_fps_frames = 0;
// Профилирование загрузки ядер (RP2040)
#if defined(ARDUINO_ARCH_RP2040)
static volatile uint32_t g_acc_core0_us = 0;
static volatile uint32_t g_acc_core1_us = 0;
static uint8_t g_cpu0_pct = 0, g_cpu1_pct = 0;
static uint32_t g_win_start_us = 0;
#endif
static char g_hud_text[64] = {0};
static uint32_t g_start_ms = 0; // момент запуска (для фазы радуги)

#if defined(ARDUINO_ARCH_RP2040)
// --- Multicore sync for split compute ---
static volatile bool g_frame_req = false;     // core0 -> core1: новый кадр (посчитать нижнюю половину)
static volatile bool g_core1_done = false;    // core1 -> core0: нижняя половина готова
static volatile uint32_t g_frame_time_ms = 0; // timestamp кадра (общий для обеих половин)

static uint16_t colorWheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) return ST77xx::RGB(255 - WheelPos * 3, 0, WheelPos * 3);
  if(WheelPos < 170) { WheelPos -= 85; return ST77xx::RGB(0, WheelPos * 3, 255 - WheelPos * 3); }
  WheelPos -= 170; return ST77xx::RGB(WheelPos * 3, 255 - WheelPos * 3, 0);
}

static inline void drawPlasmaSegment(uint16_t* buf, int w, int y0, int y1, uint32_t time_ms)
{
  float t = time_ms / 500.0f;
  for (int y = y0; y < y1; ++y) {
    uint16_t* row = buf + (size_t)y * w;
    for (int x = 0; x < w; ++x) {
      float v = sinf(x / 16.0f + t)
              + sinf((x + y) / 16.0f + t);
      int color_index = int((v + 4.0f) * 32.0f) & 255;
      row[x] = colorWheel((byte)color_index); // RGB565 LE в framebuffer
    }
  }
}

static void core1_worker() {
  for (;;) {
    if (g_frame_req) {
      int y0 = g_h / 2; int y1 = g_h; // нижняя половина
      uint32_t t0 = micros();
      // Убрал плазму, простой чёрный фон для нижней половины
  memset(g_backbuf + (size_t)g_w * y0, 0, (size_t)g_w * (y1 - y0) * 2);
      g_acc_core1_us += (micros() - t0);
      g_core1_done = true;
      // ждём, пока core0 сбросит флаг запроса (начало следующего кадра)
      while (g_frame_req) { tight_loop_contents(); }
    } else {
      tight_loop_contents();
    }
  }
}
#endif
// FPS выводим раз в секунду
static void fps_tick() {
  ++g_fps_frames;
  uint32_t t = millis();
  if (t - g_fps_t0 >= 1000) {
#if defined(ARDUINO_ARCH_RP2040)
    // Расчёт процентов загрузки за окно
    uint32_t now_us = micros();
    uint32_t win_us = now_us - g_win_start_us;
    if (win_us == 0) win_us = 1;
    g_cpu0_pct = (uint8_t)min<uint32_t>(99, (g_acc_core0_us * 100UL) / win_us);
    g_cpu1_pct = (uint8_t)min<uint32_t>(99, (g_acc_core1_us * 100UL) / win_us);
    g_acc_core0_us = 0; g_acc_core1_us = 0; g_win_start_us = now_us;
    snprintf(g_hud_text, sizeof(g_hud_text), "FPS:%lu C0:%u%% C1:%u%%",
             (unsigned long)g_fps_frames, (unsigned)g_cpu0_pct, (unsigned)g_cpu1_pct);
    Serial.print(F("[HUD] ")); Serial.println(g_hud_text);
#else
    snprintf(g_hud_text, sizeof(g_hud_text), "FPS:%lu", (unsigned long)g_fps_frames);
    Serial.print(F("[HUD] ")); Serial.println(g_hud_text);
#endif
    g_fps_frames = 0;
    g_fps_t0 = t;
  }
}

// Белая 1px рамка по периметру в backbuffer
static inline void drawWhiteBorder()
{
  const uint16_t W = g_w, H = g_h;
  const uint16_t white = ST77xxDMA::toBE(ST77xxDMA::Colors::White); // framebuffer хранит BE
  // Верх и низ
  uint16_t* top = g_backbuf;
  uint16_t* bot = g_backbuf + (size_t)(H - 1) * W;
  for (uint16_t x=0; x<W; ++x) { top[x] = white; bot[x] = white; }
  // Левый и правый столбцы
  for (uint16_t y=0; y<H; ++y) {
    g_backbuf[(size_t)y * W + 0] = white;
    g_backbuf[(size_t)y * W + (W - 1)] = white;
  }
}


static inline void drawSevenColorBars(uint16_t* buf, int w, int h, uint32_t time_ms)
{
  using C = ST77xxDMA::Colors;
  const uint16_t bars[8] = {
    ST77xxDMA::toBE(C::Black),
    ST77xxDMA::toBE(C::Blue),
    ST77xxDMA::toBE(C::Red),
    ST77xxDMA::toBE(C::Magenta),
    ST77xxDMA::toBE(C::Green),
    ST77xxDMA::toBE(C::Cyan),
    ST77xxDMA::toBE(C::Yellow),
    ST77xxDMA::toBE(C::White)
  };
  int offset = (time_ms / 100) % 8; // скорость и зацикливание
  int barW = w / 8; if (barW <= 0) barW = 1;
  int x = 0;
  for (int i = 0; i < 8; ++i) {
    int x0 = x;
    int x1 = (i == 7) ? (w) : (x + barW); // последняя полоса забирает остаток
    if (x1 > w) x1 = w;
    uint16_t color = bars[(i + offset) % 8];
    for (int yy = 0; yy < h; ++yy) {
      uint16_t* row = buf + (size_t)yy * w;
      for (int xx = x0; xx < x1; ++xx) row[xx] = color;
    }
    x = x1;
  }
}

void setup() {
  Serial.begin(115200);
  for (int i=0; i<10 && !Serial; ++i) delay(100);
  Serial.println(F("--- NV3007 Performance Test ---"));

  ST77xxConfig cfg;
  cfg.panel = PANEL;
  cfg.mosi = PIN_MOSI; 
  cfg.miso = PIN_MISO; 
  cfg.sclk = PIN_SCLK;
  cfg.cs   = PIN_CS;   
  cfg.dc   = PIN_DC;   
  cfg.rst  = PIN_RST;
  cfg.bl   = PIN_BL;
  cfg.bl_start = 155;
  cfg.rotation   = 1;
  // Платформо-зависимая стартовая частота SPI
#if defined(ARDUINO_ARCH_RP2040)
  cfg.spi_hz     = 80;    // 80 МГц — обычно стабильно на хороших проводах
#elif defined(ARDUINO_ARCH_ESP32)
  cfg.spi_hz     = 80;    // до 80 МГц (HSPI/VSPI)
#elif defined(ESP8266)
  cfg.spi_hz     = 40;    // 40 МГц — более совместимо (80 МГц может быть на грани)
#elif defined(ARDUINO_ARCH_STM32)
  cfg.spi_hz     = 80;    // ~54 МГц (проверьте ограничение вашей платы/ядра)
#else
  cfg.spi_hz     = 40;    // дефолт для прочих платформ
#endif
  cfg.bl_freq_hz = 10000;
  cfg.debug      = false;
  cfg.allocate_framebuffer = true;  // включаем внутренний framebuffer для flush()
  cfg.use_hw_spi = true;
  cfg.sw_spi_use_asm = true;       // зарезервированно: ASM-вариант (если будет доступен на платформе)
  cfg.sw_spi_direct_gpio = true;    // true: быстрые прямые регистры GPIO; false: portable digitalWrite() (медленнее, но максимально совместимо)
  cfg.bl_invert = false;            // Инверсия подсветки
  cfg.bgr        =false;          // set true if colors swapped  
  // RP2040-специфичные поля — только на RP2040
#if defined(ARDUINO_ARCH_RP2040)
  cfg.rp2040_use_pio = false; // SPI+DMA путь (PIO находится в статусе разработки — не включать)
  cfg.rp2040_auto_raise_clk_peri = true;
  cfg.rp2040_target_sysclk_khz = 160000; 
#endif
   
  Serial.println(F("setup(): create LCD"));
  lcd = new ST77xxDMA(cfg);

  if (!lcd->begin()) {
    Serial.println("[ERROR] LCD begin() failed");
    while (1) delay(1000);
  }
  Serial.println(F("setup(): lcd.begin() OK"));
  // Задать дефолтный дисплей для короткого API и установить яркость
  ST77xx::setDefault(lcd);
  setBrightness(155);
  // Установить нормальный шрифт для HUD
  ST77xx::setGFXFont(&times9pt7b);
  lcd->fillScreen(ST77xxDMA::Colors::Black);
  g_fps_t0 = millis();
#if defined(ARDUINO_ARCH_RP2040)
  g_win_start_us = micros();
#endif
  lcd->setTextWrap(false);
  g_start_ms = millis();
  // Показать HUD сразу, до первого секундного тика
  snprintf(g_hud_text, sizeof(g_hud_text), "FPS:0");

  // Backbuffer под весь кадр (RGB565 BE)
  g_w = lcd->width(); g_h = lcd->height();
  g_backbuf = lcd->framebuffer(); // используем внутренний framebuffer
  if (!g_backbuf) {
    Serial.println(F("[ERROR] framebuffer() returned null"));
    while (1) delay(1000);
  }

#if defined(ARDUINO_ARCH_RP2040)
  multicore_launch_core1(core1_worker);
  Serial.println(F("setup(): core1 worker started"));
#endif
  Serial.println(F("setup(): init OK"));
}

void loop() {
  uint32_t now = millis();
  // Непрерывно рисуем бары + рамку + HUD
  drawSevenColorBars(g_backbuf, g_w, g_h, now);
  drawWhiteBorder();
  // Подложка под HUD
  lcd->drawText(4, 30, g_hud_text, ST77xxDMA::Colors::Blue, ST77xxDMA::Colors::Black, 2);
  lcd->flush();
  fps_tick();
}