#include <Arduino.h>
#include <ST77xxDMA.h>
#include <TEMT6000.h>

// Simple RP2040 (Pico) example for ST77xxDMA:
// - Uses SPI0 with user-provided pins
// - Draws a green border along the screen edges
// - Fills inner area with vertical color bars
//
// Adjust PANEL below to match your glass (e.g. ST7789_240x240, ST7789_135x240, ST7735_80x160).

#if !defined(ARDUINO_ARCH_RP2040)
#error "This example is intended for RP2040 boards (Raspberry Pi Pico etc.)"
#endif

// ---- RP2040 SPI0 pin mapping from user ----
// SCL  -> SCLK
// SDA  -> MOSI
// CS   -> chip select
// DC   -> data/command
// RST  -> reset
// BL   -> backlight (PWM capable pin is recommended)
#define PIN_MOSI 7   // GPIO7
#define PIN_MISO -1  // not used
#define PIN_SCLK 6   // GPIO6
#define PIN_CS   10  // GPIO10
#define PIN_DC    9  // GPIO9
#define PIN_RST   8  // GPIO8
#define PIN_BL   11  // GPIO11

// ---- Select panel preset here ----
// Change this line if you test another module.
static const ST77xxPanel PANEL = ST77xxPanel::ST7789_240x240;

static ST77xxDMA* tft = nullptr;
using C = ST77xxDMA::Colors;

static TEMT6000 g_lux(26, 3.3f, 4095);  // TEMT6000 on GPIO26 (ADC0), 3.3V, 12-bit ADC

static void draw_border_and_bars() {
  if (!tft) return;
  uint16_t w = tft->width();
  uint16_t h = tft->height();
  if (w == 0 || h == 0) return;

  // Clear screen
  tft->fillScreen(C::Black);

  // Green border along the very edge
  uint16_t borderColor = C::Green;
  tft->drawRect(0, 0, w, h, borderColor);

  // Inner area (inside 1px border on all sides)
  int16_t x0 = 1;
  int16_t y0 = 1;
  int16_t innerW = (w > 2) ? (w - 2) : 0;
  int16_t innerH = (h > 2) ? (h - 2) : 0;
  if (innerW <= 0 || innerH <= 0) return;

  const uint16_t cols[8] = {
    C::White, C::Yellow, C::Cyan, C::Green,
    C::Magenta, C::Red, C::Blue, C::Black
  };
  const int bars = 8;
  int bw = innerW / bars;
  if (bw <= 0) bw = 1;

  int x = x0;
  for (int i = 0; i < bars; ++i) {
    int ww = (i == bars - 1) ? (x0 + innerW - x) : bw; // last bar gets remainder
    if (ww <= 0) break;
    tft->fillRect(x, y0, ww, innerH, cols[i]);
    x += ww;
    if (x >= x0 + innerW) break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

#if defined(ARDUINO_ARCH_RP2040)
  // Use full 12-bit ADC resolution (0..4095) for the light sensor
  analogReadResolution(12);
#endif

  ST77xxConfig cfg;
  cfg.panel = PANEL;
  cfg.controller = ST77xxController::ST7789; // will be overridden by applyPanelPreset()

  cfg.mosi = PIN_MOSI;
  cfg.miso = PIN_MISO;
  cfg.sclk = PIN_SCLK;
  cfg.cs   = PIN_CS;
  cfg.dc   = PIN_DC;
  cfg.rst  = PIN_RST;
  cfg.bl   = PIN_BL;

  cfg.use_hw_spi = true;        // Use RP2040 hardware SPI0
  cfg.spi_hz     = 80;          // 80 MHz (internally treated as MHz shorthand)
  cfg.rotation   = 1;           // Start in landscape; change 0..3 to test offsets
  cfg.bgr        = false;       // set true if colors swapped
  cfg.debug      = false;

  cfg.allocate_framebuffer = false; // simple immediate-mode drawing is enough here

  // RP2040 backend options
  cfg.rp2040_use_pio = false;                // SPI+DMA backend (set true to try PIO+DMA)
  cfg.rp2040_auto_raise_clk_peri = true;     // tie clk_peri to current clk_sys
  cfg.rp2040_target_sysclk_khz   = 200000;   // optional: target 160 MHz system clock

  tft = new ST77xxDMA(cfg);
  if (!tft || !tft->begin()) {
    Serial.println("Failed to init ST77xxDMA");
    while (1) delay(1);
  }

  // Short-form API helpers
  ST77xx::setDefault(tft);

  // Set backlight/brightness (either via BL pin or panel-side if BL < 0)
  ST77xx::setBrightness(180); // 0..255

  draw_border_and_bars();
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 100) {
    last = now;
    // Average 100 individual measurements to suppress 50 Hz flicker/noise
    uint32_t sum = 0;
    for (int i = 0; i < 100; ++i) {
      sum += analogRead(26); // same ADC pin as TEMT6000 (GPIO26 / ADC0)
    }
    uint16_t raw = (uint16_t)(sum / 100);   // averaged 0..4095
    if (raw > 4095) raw = 4095;
    // 12-bit -> 8-bit: 4096 steps -> 256 levels
    uint8_t level = (uint8_t)(raw >> 4);  // divide by 16

    Serial.print("raw=");  Serial.print(raw);
    Serial.print(" level="); Serial.println(level);

    ST77xx::setBrightness(level);
  }
}
