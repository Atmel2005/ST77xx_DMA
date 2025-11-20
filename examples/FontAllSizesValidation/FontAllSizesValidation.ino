#include <Arduino.h>
#include <ST77xxDMA.h>
// GFX fonts
#include <Fonts/GFXFF/TomThumb.h>
#include <Fonts/GFXFF/FreeSans9pt7b.h>
#include <Fonts/GFXFF/FreeSans12pt7b.h>
#include <Fonts/GFXFF/FreeSans18pt7b.h>
#include <Fonts/GFXFF/FreeSans24pt7b.h>
#include <Fonts/GFXFF/FreeSerif9pt7b.h>
#include <Fonts/GFXFF/FreeSerif12pt7b.h>
#include <Fonts/GFXFF/FreeSerif18pt7b.h>
#include <Fonts/GFXFF/FreeSerif24pt7b.h>
#include <Fonts/GFXFF/FreeMono9pt7b.h>
#include <Fonts/GFXFF/FreeMono12pt7b.h>
#include <Fonts/GFXFF/FreeMono18pt7b.h>
#include <Fonts/GFXFF/FreeMono24pt7b.h>
#include <string.h>
#include <stdio.h>
// Times (regular/bold) наборы
#include <Fonts/GFXFF/font_mod/TimesAll.h>

// Optional: RUS mono fonts aggregator (fallback for Cyrillic)
#ifndef __has_include
  #define __has_include(x) 0
#endif
#if __has_include(<Fonts/GFXFF/font_mod/RUS/TimesAllRU.h>)
  #include <Fonts/GFXFF/font_mod/RUS/TimesAllRU.h>
  #define ST77XX_HAVE_RUS_TIMES 1
#else
  #define ST77XX_HAVE_RUS_TIMES 0
#endif
#if __has_include(<Fonts/GFXFF/font_mod/RUS/FontMonoAll.h>)
  #include <Fonts/GFXFF/font_mod/RUS/FontMonoAll.h>
  #define ST77XX_HAVE_RUS 1
#else
  #define ST77XX_HAVE_RUS 0
#endif

// Options: ST77xxPanel::ST7735_128x160, ST7789_240x320, ST7789_172x320, ST7789_76x284, ST7789_170x320, NV3007_428x142
static const ST77xxPanel PANEL = ST77xxPanel::NV3007_428x142; // change as needed
 
// ---- Pins per platform (adjust for your wiring) ----
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 WROOM / S3 default example pins (adjust to your board)
  #define PIN_MOSI 11
  #define PIN_MISO -1
  #define PIN_SCLK 12
  #define PIN_CS    10
  #define PIN_DC    9
  #define PIN_RST   8
  #define PIN_BL   5 // 5
#elif defined(ESP8266)
  
  #define PIN_MOSI D7    // GPIO13 — общий для HW и SW (в SW будет использован)
  #define PIN_MISO -1    // или D6 (GPIO12), если чтение нужно
  #define PIN_SCLK D5    // GPIO14 — общий для HW и SW (в SW будет использован)
  #define PIN_CS   D8    // GPIO15 — OK для CS
  #define PIN_DC   D0    // GPIO5 — безопаснее, чем GPIO0/1/2
  #define PIN_RST  D2    // GPIO4 (или D4=GPIO2, но это бут-стрэп HIGH)
  #define PIN_BL   D1    // GPIO2 (или D1/D2), PWM работает через analogWrite
#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 (Pico/Zero) using SPI0 default pins
  #define PIN_MOSI 7
  #define PIN_MISO -1
  #define PIN_SCLK 6
  #define PIN_CS   10
  #define PIN_DC   9
  #define PIN_RST  8
  #define PIN_BL   11
#else 
  // Fallback generic
  #define PIN_MOSI 7
  #define PIN_MISO -1
  #define PIN_SCLK 6
  #define PIN_CS   10
  #define PIN_DC    9
  #define PIN_RST   8
  #define PIN_BL    11
#endif

// Import short-form API into this translation unit
using ST77xx::setTextSize;
using ST77xx::setTextWrap;
using ST77xx::setBrightness;
using ST77xx::setTextColor;
using ST77xx::setCursor;
using ST77xx::print;
using ST77xx::drawTextUTF8;
using ST77xx::flush;
using ST77xx::fillRect;
using ST77xx::drawText;
using ST77xx::framebuffer;
using ST77xx::width;
using ST77xx::height;
using ST77xx::setGFXFont;
using ST77xx::textSize;
using ST77xx::RGB;
using ST77xx::toBE;
using ST77xx::fillScreen;

namespace C = ST77xx::C; // color aliases

ST77xxConfig cfg;
ST77xxDMA*   tft = nullptr;

// ---------------- Font sets to iterate ----------------
enum FontMode : uint8_t {
  TM_REG = 0,   // Times Regular, px 5..40
  TM_BOLD,      // Times Bold,    px 5..40
  FS_SANS,      // FreeSans  {9,12,18,24} pt
  FS_SERIF,     // FreeSerif {9,12,18,24} pt
  FS_MONO,      // FreeMono  {9,12,18,24} pt
  FM_TOMTHUMB,  // TomThumb tiny
  FONT_MODE_COUNT
};

static const GFXfont* kFreeSansFonts[]  = { &FreeSans9pt7b,  &FreeSans12pt7b,  &FreeSans18pt7b,  &FreeSans24pt7b };
static const char*     kFreeSansNames[]  = { "FreeSans 9pt",  "FreeSans 12pt",  "FreeSans 18pt",  "FreeSans 24pt" };
static const GFXfont* kFreeSerifFonts[] = { &FreeSerif9pt7b, &FreeSerif12pt7b, &FreeSerif18pt7b, &FreeSerif24pt7b };
static const char*     kFreeSerifNames[] = { "FreeSerif 9pt", "FreeSerif 12pt", "FreeSerif 18pt", "FreeSerif 24pt" };
static const GFXfont* kFreeMonoFonts[]  = { &FreeMono9pt7b,  &FreeMono12pt7b,  &FreeMono18pt7b,  &FreeMono24pt7b };
static const char*     kFreeMonoNames[]  = { "FreeMono 9pt",  "FreeMono 12pt",  "FreeMono 18pt",  "FreeMono 24pt" };

static uint8_t g_mode = TM_REG;
static uint8_t g_px   = 5;   // for Times ranges 5..40
static uint8_t g_idx  = 0;   // for Free* arrays 0..3

static uint8_t applyFontAndTitle(char* title, size_t title_sz) {
  switch (g_mode) {
    case TM_REG:
      setGFXFont(TimesFF::regular(g_px));
      snprintf(title, title_sz, "Times Regular ~%u px", (unsigned)g_px);
      break;
    case TM_BOLD:
      setGFXFont(TimesFF::bold(g_px));
      snprintf(title, title_sz, "Times Bold ~%u px", (unsigned)g_px);
      break;
    case FS_SANS:
      setGFXFont(kFreeSansFonts[g_idx]);
      snprintf(title, title_sz, "%s", kFreeSansNames[g_idx]);
      break;
    case FS_SERIF:
      setGFXFont(kFreeSerifFonts[g_idx]);
      snprintf(title, title_sz, "%s", kFreeSerifNames[g_idx]);
      break;
    case FS_MONO:
      setGFXFont(kFreeMonoFonts[g_idx]);
      snprintf(title, title_sz, "%s", kFreeMonoNames[g_idx]);
      break;
    default: // FM_TOMTHUMB
      setGFXFont(&TomThumb);
      snprintf(title, title_sz, "TomThumb");
      break;
  }
  // Prefer Times RU fallback (matching weight) if available, else Mono RU
  {
    uint8_t px = g_px;
    if (px == 0) px = 12;
    if (px > 40) px = 40;
#if ST77XX_HAVE_RUS_TIMES
    if (g_mode == TM_BOLD) {
      ST77xx::setGFXFontFallback(TimesRUFF::bold(px));
    } else {
      ST77xx::setGFXFontFallback(TimesRUFF::regular(px));
    }
#elif ST77XX_HAVE_RUS
    ST77xx::setGFXFontFallback(FontMonoFF::mono(px));
#endif
  }
  if (tft && tft->gfxFont()) return tft->gfxFont()->yAdvance;
  return 8;
}

static void drawValidationPage() {
  char title[48];
  const uint8_t h = applyFontAndTitle(title, sizeof(title));
  fillScreen(C::Black);
  setTextColor(C::White, C::Black);
  setTextSize(1);
  setTextWrap(false);

  const int x = 4;
  // Start at baseline for first line
  setCursor(x, 4 + (int)h);

  // Title
  setTextColor(C::Yellow, C::Black);
  print(title);
  print("\n");
  {
    int16_t cx, cy; ST77xx::getCursor(cx, cy);
    setCursor(x, cy + 4);
  }

  // ASCII
  setTextColor(C::White, C::Black);
  print("ASCII: Hello, 0123 !@#?");
  print("\n");
  {
    int16_t cx, cy; ST77xx::getCursor(cx, cy);
    setCursor(x, cy + 4);
  }

  print("UPPER: THE QUICK BROWN FOX JUMPS");
  print("\n");
  {
    int16_t cx, cy; ST77xx::getCursor(cx, cy);
    setCursor(x, cy + 4);
  }

  print("lower: the quick brown fox jumps");
  print("\n");
  {
    int16_t cx, cy; ST77xx::getCursor(cx, cy);
    setCursor(x, cy + 4);
  }

  setTextColor(RGB(200,255,200), C::Black);
  print("Digits: 1234567890");
  print("\n");
  {
    int16_t cx, cy; ST77xx::getCursor(cx, cy);
    setCursor(x, cy + 4);
  }

  // Cyrillic (UTF-8)
  setTextColor(C::White, C::Black);
  print((const char*)u8"Кириллица: Привет, мир! Ё/ё Й/й Ю/ю Я/я");
  print("\n");
  {
    int16_t cx, cy; ST77xx::getCursor(cx, cy);
    setCursor(x, cy + 4);
  }

  print((const char*)u8"АБВГДЕЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ");
  print("\n");
  {
    int16_t cx, cy; ST77xx::getCursor(cx, cy);
    setCursor(x, cy + 4);
  }

  print((const char*)u8"абвгдежзийклмнопрстуфхцчшщъыьэюя");

  flush();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  cfg.panel      = PANEL;
  cfg.mosi = PIN_MOSI;
  cfg.miso = PIN_MISO;
  cfg.sclk = PIN_SCLK;
  cfg.cs   = PIN_CS;
  cfg.dc   = PIN_DC;
  cfg.rst  = PIN_RST;
  cfg.bl   = PIN_BL;
  cfg.bl_start = 155;
  cfg.rotation   = 1;
  cfg.spi_hz     = 80;   
  cfg.bl_freq_hz = 10000;
  cfg.debug      = false;
  cfg.allocate_framebuffer = true;  // включаем внутренний framebuffer для flush()
  cfg.use_hw_spi = true;
  cfg.sw_spi_use_asm = true;       // зарезервированно: ASM-вариант (если будет доступен на платформе)
  cfg.sw_spi_direct_gpio = true;    // true: быстрые прямые регистры GPIO; false: portable digitalWrite() (медленнее, но максимально совместимо)
  cfg.bl_invert = false;            // Инверсия подсветки
  cfg.bgr        =false;          // set true if colors swapped  
  cfg.rp2040_use_pio = false; // SPI+DMA путь (pio не реализовано)
  cfg.rp2040_auto_raise_clk_peri = true;
  cfg.rp2040_target_sysclk_khz = 160000; 


  tft = new ST77xxDMA(cfg);
  if (!tft->begin()) {
    Serial.println("Failed to init ST77xxDMA");
    while (1) delay(1);
  }
  ST77xx::setDefault(tft);
  setBrightness(50);
}

void loop() {
  drawValidationPage();
  delay(1000);

  // Advance state
  switch (g_mode) {
    case TM_REG:
    case TM_BOLD:
      if (g_px >= 40) { g_px = 5; g_mode = (FontMode)(g_mode + 1); }
      else { ++g_px; }
      break;
    case FS_SANS:
    case FS_SERIF:
    case FS_MONO:
      if (++g_idx >= 4) { g_idx = 0; g_mode = (FontMode)(g_mode + 1); }
      break;
    default: // FM_TOMTHUMB
      g_mode = (FontMode)(g_mode + 1);
      break;
  }
  if (g_mode >= FONT_MODE_COUNT) { g_mode = TM_REG; }
}
