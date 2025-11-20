#include <Arduino.h>
#include <ST77xxDMA.h>
// Панель как в текущем проекте
static const ST77xxPanel PANEL = ST77xxPanel::ST7789_170x320;
// Пины как в FpsTest для RP2040 (SPI0)
#if defined(ARDUINO_ARCH_RP2040)
  #define PIN_MOSI 7
  #define PIN_MISO -1
  #define PIN_SCLK 6
  #define PIN_CS   10
  #define PIN_DC   9
  #define PIN_RST  8
  #define PIN_BL   11
#else
  #error "Этот пример предназначен для RP2040"
#endif
ST77xxConfig cfg;
ST77xxDMA*   tft = nullptr;
// Импорт короткого API (минимум нужных функций)
using ST77xx::setBrightness;
using ST77xx::setTextColor;
using ST77xx::setCursor;
using ST77xx::print;
using ST77xx::fillRect;
using ST77xx::drawRect;
using ST77xx::width;
using ST77xx::height;
// Цвета
namespace C = ST77xx::C;
void setup() {
  Serial.begin(115200);
  delay(1500);
  cfg.panel      = PANEL;
  cfg.mosi       = PIN_MOSI;
  cfg.miso       = PIN_MISO;
  cfg.sclk       = PIN_SCLK;
  cfg.cs         = PIN_CS;
  cfg.dc         = PIN_DC;
  cfg.rst        = PIN_RST;
  cfg.bl         = PIN_BL;
  cfg.use_hw_spi = true;
  cfg.spi_hz     = 50;        // MHz shorthand (см. обновление API)
  cfg.rotation   = 1;
  cfg.bl_invert  = false;
  // Критично для чистого теста 9-бит PIO+DMA: без полного framebuffer
  cfg.allocate_framebuffer = false;
  cfg.rp2040_use_pio              = true;
  cfg.rp2040_pio_lsb_first        = true;  // MSB-first
  cfg.rp2040_spi_9bit             = false;   // тестируем 9-бит режим
  cfg.rp2040_pio_post_ramwr_us    = 10;
  cfg.rp2040_auto_raise_clk_peri  = true;
  cfg.rp2040_target_sysclk_khz    = 200000; // 200 МГц sysclk
  cfg.debug = true; // логи в Serial для диагностики
  tft = new ST77xxDMA(cfg);
  if (!tft->begin()) {
    Serial.println("[ERR] ST77xxDMA begin() failed");
    while (1) delay(1);
  }
  // Установим экземпляр по умолчанию для короткого API
  ST77xx::setDefault(tft);
  // Яркость (через BL pin или панельный регистр)
  setBrightness(155);
  // Фон чёрный
  fillRect(0, 0, width(), height(), C::Black);
  // Белая рамка по контуру
  drawRect(0, 0, width(), height(), C::White);
  // Пара строк текста (ASCII-билтин шрифт, без GFX)
  setTextColor(C::White, C::Black);
  setCursor(4, 16);
  print("RP2040 PIO+DMA 9-bit");
  setCursor(4, 32);
  print("ST7789 170x320 test");
  Serial.println("[OK] Init done. Minimal 9-bit PIO+DMA sketch running.");
}
void loop() {
  // Ничего не делаем: одноразовый вывод
  delay(1000);
}
