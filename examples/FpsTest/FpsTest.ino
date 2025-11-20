#include <Arduino.h>
// Core display
#define ST77XX_DEBUG
#include <ST77xxDMA.h>
// Use only needed GFX fonts: Times 12pt EN and (optionally) Times RU 12pt
#include <Fonts/GFXFF/font_mod/kurze/times12pt7b.h>
// PROGMEM helpers for ESP8266 font field reads
#include <pgmspace.h>
#ifndef __has_include
  #define __has_include(x) 0
#endif
#if __has_include(<Fonts/GFXFF/font_mod/RUS/regular/tnr_ru_12pt7b.h>)
  #include <Fonts/GFXFF/font_mod/RUS/regular/tnr_ru_12pt7b.h>
  #define HAVE_RU_12PT 1
#else
  #define HAVE_RU_12PT 0
#endif
#include <string.h>
#include <stdio.h>
// RP2040: access to clock configuration (to raise clk_peri for fast SPI)
#if defined(ARDUINO_ARCH_RP2040)
  #include <hardware/clocks.h>
#endif
// ---- Select panel here ----
// Options: ST77xxPanel::ST7735_128x160, ST7789_240x320, ST7789_172x320, ST7789_76x284, ST7789_170x320, ST7789_135x240, ST7789_240x240, ST7735_80x160, NV3007_428x142
static const ST77xxPanel PANEL = ST77xxPanel::ST7789_240x240; // change as needed
 
// ---- Pins per platform (adjust for your wiring) ----
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 WROOM / S3 default example pins (adjust to your board)
  #define PIN_MOSI 11 //11  - 6
  #define PIN_MISO -1
  #define PIN_SCLK 12 // 12 - 4
  #define PIN_CS    10//10 - 7
  #define PIN_DC    9 // 9 - 5 
  #define PIN_RST   8 // 8 - 3
  #define PIN_BL   5 // 5 - 10
#elif defined(ESP8266)
  // ВАЖНО (HW SPI на ESP8266): аппаратные пины SPI фиксированы ядром:
  //   SCLK=D5(GPIO14), MISO=D6(GPIO12), MOSI=D7(GPIO13)
  // Эти #define нужны:
  //   - всегда: для CS/DC/RST/BL (выбираете любые доступные GPIO),
  //   - для SW SPI: когда cfg.use_hw_spi=false (тогда будут использованы ваши MOSI/SCLK).
  #define PIN_MOSI D7    // GPIO13 — общий для HW и SW (в SW будет использован)
  #define PIN_MISO -1    // или D6 (GPIO12), если чтение нужно
  #define PIN_SCLK D5    // GPIO14 — общий для HW и SW (в SW будет использован)
  #define PIN_CS   D8//D8    // GPIO15 — OK для CS
  #define PIN_DC   D0    // GPIO5 — безопаснее, чем GPIO0/1/2
  #define PIN_RST  D2    // GPIO4 (или D4=GPIO2, но это бут-стрэп HIGH)
  #define PIN_BL   D1 //D1    // GPIO2 (или D1/D2), PWM работает через analogWrite
#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 (Pico/Zero) using SPI0 default pins
  #define PIN_MOSI 7 // 7  (11)
  #define PIN_MISO -1
  #define PIN_SCLK 6  //6 (10)
  #define PIN_CS   10 //10  (29)
  #define PIN_DC   9 //9 (0)
  #define PIN_RST  8 //8 (9)
  #define PIN_BL   11 // 11 (-1)
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

// ---- Config ----
ST77xxConfig cfg;
ST77xxDMA*   tft = nullptr;
// Import short-form API into this translation unit
using ST77xx::setTextSize;
using ST77xx::setTextWrap;
using ST77xx::setBrightness;
using ST77xx::setTextColor;
using ST77xx::setCursor;
using ST77xx::print;
using ST77xx::drawTextUTF8;
using ST77xx::flush;
using ST77xx::flushRect;
using ST77xx::fillRect;
using ST77xx::drawText;
using ST77xx::framebuffer;
using ST77xx::width;
using ST77xx::height;
using ST77xx::setGFXFont;
using ST77xx::textSize;
using ST77xx::RGB;
using ST77xx::toBE;
// Short alias for color constants
namespace C = ST77xx::C;

// --- One-bit toggle for animated background (0 = static 8 bars, 1 = animated 16 bars) ---
#ifndef FPS_TEST_ANIM
#define FPS_TEST_ANIM 1
#endif
#ifndef FPS_TEST_ANIM_SPEED
#define FPS_TEST_ANIM_SPEED 1 // pixels per frame
#endif

// --- Layout helpers for evenly spaced text lines ---
static int g_lineY[7] = {4, 20, 36, 52, 68, 84, 100}; // [0]=FPS, [1]=ASCII, [2]=UTF8, [3..6]=alphabets
static const int g_lineCount = 7;
static int g_marginTop = 8;
static int g_marginBottom = 8;
static int g_lineH = 8;          // base font height in pixels (built-in/6x8 -> 8)
static uint8_t g_textSize = 1;   // text scale factor
static bool g_rebuild_bg_each_frame = true; // true: перерисовывать фон полностью каждый кадр

#if FPS_TEST_ANIM
static uint16_t s_anim_px = 0; // глобальная фаза анимации, чтобы использовать в нескольких местах
#endif

static void computeLayout() {
  if (!tft) return;
  const int H = (int)ST77xx::height();
  const int lh = (int)g_textSize * g_lineH;
  // Ensure the first baseline is at least one line-height from the top
  int baselineStart = g_marginTop;
  if (baselineStart < lh - 1) baselineStart = lh - 1;
  int freeSpace = H - baselineStart - g_marginBottom - g_lineCount * lh;
  if (freeSpace < 0) freeSpace = 0;
  const int gap = (g_lineCount > 1) ? (freeSpace / (g_lineCount - 1)) : 0;
  for (int i = 0; i < g_lineCount; ++i) {
    g_lineY[i] = baselineStart + i * (lh + gap);
  }
}

void build_color_bars(uint16_t* fb, uint16_t w, uint16_t h) {
  const uint16_t cols[8] = {
    C::White, C::Yellow, C::Cyan, C::Green,
    C::Magenta, C::Red, C::Blue, C::Black
  };
  const int bars = 8;
  int bw  = w / bars;
  int rem = w - bw*bars;
  for (uint16_t y=0; y<h; ++y) {
    int x = 0;
    for (int i=0; i<bars; ++i) {
      int ww = bw + (i==bars-1 ? rem : 0);
      uint16_t be = toBE(cols[i]);
      for (int k=0; k<ww; ++k) {
        fb[y*w + x + k] = be;
      }
      x += ww;
    }
  }
}
// Восстановить исходный фон (цветные полосы) в прямоугольнике [x..x+rw), [y..y+rh)
static __attribute__((unused)) void restore_color_bars_rect(uint16_t* fb, uint16_t w, uint16_t h,
                                    int x, int y, int rw, int rh) {
  if (!fb || w==0 || h==0 || rw<=0 || rh<=0) return;
  if (x < 0) { rw += x; x = 0; }
  if (y < 0) { rh += y; y = 0; }
  if (x >= (int)w || y >= (int)h) return;
  if (x + rw > (int)w) rw = (int)w - x;
  if (y + rh > (int)h) rh = (int)h - y;
  const uint16_t cols[8] = {
    C::White, C::Yellow, C::Cyan, C::Green,
    C::Magenta, C::Red, C::Blue, C::Black
  };
  const int bars = 8;
  const int bw   = w / bars;         // базовая ширина полосы
  const int cut  = bw * (bars - 1);  // до этой X идут ровные полосы, дальше — последняя с остатком
  for (int yy = y; yy < y + rh; ++yy) {
    uint16_t* row = fb + yy * w;
    for (int xx = x; xx < x + rw; ++xx) {
      int barIdx = (xx < cut) ? (xx / bw) : (bars - 1);
      uint16_t be = toBE(cols[barIdx]);
      row[xx] = be;
    }
  }
}

// --- Немедленная (immediate) отрисовка цветных полос поверх экрана, без использования framebuffer ---
static void draw_color_bars_immediate(uint16_t w, uint16_t h) {
  const uint16_t cols[8] = {
    C::White, C::Yellow, C::Cyan, C::Green,
    C::Magenta, C::Red, C::Blue, C::Black
  };
  const int bars = 8;
  const int bw   = (int)w / bars;
  const int rem  = (int)w - bw*bars;
  int x = 0;
  for (int i = 0; i < bars; ++i) {
    int ww = bw + (i == bars-1 ? rem : 0);
    if (ww > 0) fillRect(x, 0, ww, h, cols[i]);
    x += ww;
  }
}

// Восстановить участок "фона-полос" немедленным способом (перерисовать пересекающиеся полосы
static __attribute__((unused)) void draw_color_bars_rect_immediate(int x, int y, int rw, int rh, uint16_t w, uint16_t h) {
  if (rw <= 0 || rh <= 0) return;
  if (x < 0) { rw += x; x = 0; }
  if (y < 0) { rh += y; y = 0; }
  if (x >= (int)w || y >= (int)h) return;
  if (x + rw > (int)w) rw = (int)w - x;
  if (y + rh > (int)h) rh = (int)h - y;
  const uint16_t cols[8] = {
    C::White, C::Yellow, C::Cyan, C::Green,
    C::Magenta, C::Red, C::Blue, C::Black
  };
  const int bars = 8;
  const int bw   = (int)w / bars;
  // Перебираем все полосы и рисуем пересечение с прямоугольником
  int barX = 0;
  for (int i = 0; i < bars; ++i) {
    int ww = (i == bars-1) ? ((int)w - barX) : bw; // последняя полоса получает остаток
    int ix0 = (x > barX) ? x : barX;
    int ix1 = ((x + rw) < (barX + ww)) ? (x + rw) : (barX + ww);
    int iw = ix1 - ix0;
    if (iw > 0) {
      fillRect(ix0, y, iw, rh, cols[i]);
    }
    barX += ww;
  }
}

// --- Animated 16-color bars, scrolling left-to-right. Immediate mode, no framebuffer. ---
#if FPS_TEST_ANIM
static void draw_color_bars_anim_immediate(uint16_t w, uint16_t h, uint16_t anim_px) {
  const uint16_t cols[16] = {
    C::White, C::Yellow, C::Cyan, C::Green,
    C::Magenta, C::Red, C::Blue, C::Black,
    RGB(255,128,0),   /* Orange */
    RGB(128,0,128),   /* Purple */
    RGB(0,255,0),     /* Lime */
    RGB(0,128,128),   /* Teal */
    RGB(0,0,128),     /* Navy */
    RGB(128,0,0),     /* Maroon */
    RGB(128,128,0),   /* Olive */
    RGB(128,128,128)  /* Gray */
  };
  const int bars = 16;
  if (w == 0 || h == 0) return;
  int bw = (int)w / bars;
  if (bw <= 0) {
    // Edge case: screen narrower than bars; draw 1px stripes cycling colors
    for (int x = 0; x < (int)w; ++x) {
      int idx = ((x + (int)anim_px) * bars) / (int)w; idx &= (bars - 1);
      fillRect(x, 0, 1, h, cols[idx]);
    }
    return;
  }
  int phase = (int)anim_px % bw;               // shift in pixels within one bar
  int rot   = ((int)anim_px / bw) % bars;      // rotate bar colors when phase wraps
  // Leftmost partial (wrap from the last bar)
  if (phase > 0) {
    int w0 = phase;
    int cidx = (rot + bars - 1) % bars; // color of the bar that wraps from the right
    fillRect(0, 0, w0, h, cols[cidx]);
  }
  int x = phase;
  for (int i = 0; i < bars; ++i) {
    int ww = (i == bars-1) ? ((int)w - x) : bw;
    if (ww <= 0) break;
    int cidx = (i + rot) % bars;
    fillRect(x, 0, ww, h, cols[cidx]);
    x += ww;
    if (x >= (int)w) break;
  }
}

// --- Animated bars into framebuffer (rect), to overlay text correctly when animating ---
static void draw_color_bars_anim_rect_to_fb(uint16_t* fb, uint16_t w, uint16_t h,
                                            int x, int y, int rw, int rh, uint16_t anim_px) {
  if (!fb || w == 0 || h == 0 || rw <= 0 || rh <= 0) return;
  if (x < 0) { rw += x; x = 0; }
  if (y < 0) { rh += y; y = 0; }
  if (x >= (int)w || y >= (int)h) return;
  if (x + rw > (int)w) rw = (int)w - x;
  if (y + rh > (int)h) rh = (int)h - y;
  const uint16_t cols[16] = {
    C::White, C::Yellow, C::Cyan, C::Green,
    C::Magenta, C::Red, C::Blue, C::Black,
    RGB(255,128,0), RGB(128,0,128), RGB(0,255,0), RGB(0,128,128),
    RGB(0,0,128), RGB(128,0,0), RGB(128,128,0), RGB(128,128,128)
  };
  const int bars = 16;
  int bw = (int)w / bars;
  if (bw <= 0) {
    // 1px stripes cycling colors
    for (int yy = y; yy < y + rh; ++yy) {
      uint16_t* row = fb + (size_t)yy * w;
      for (int xx = x; xx < x + rw; ++xx) {
        int idx = ((xx + (int)anim_px) * bars) / (int)w; idx &= (bars - 1);
        row[xx] = toBE(cols[idx]);
      }
    }
    return;
  }
  int phase = (int)anim_px % bw;
  int rot   = ((int)anim_px / bw) % bars;

  // Helper to fill intersection of a vertical strip [sx, sx+sw) with our rect
  auto fill_strip = [&](int sx, int sw, uint16_t color){
    int ix0 = (x > sx) ? x : sx;
    int ix1 = ((x + rw) < (sx + sw)) ? (x + rw) : (sx + sw);
    int iw = ix1 - ix0;
    if (iw <= 0) return;
    uint16_t cbe = toBE(color);
    for (int yy = y; yy < y + rh; ++yy) {
      uint16_t* row = fb + (size_t)yy * w + ix0;
      for (int xx = 0; xx < iw; ++xx) row[xx] = cbe;
    }
  };

  // Leftmost partial
  if (phase > 0) {
    int cidx = (rot + bars - 1) % bars;
    fill_strip(0, phase, cols[cidx]);
  }
  int sx = phase;
  for (int i = 0; i < bars; ++i) {
    int sw = (i == bars - 1) ? ((int)w - sx) : bw;
    if (sw <= 0) break;
    int cidx = (i + rot) % bars;
    fill_strip(sx, sw, cols[cidx]);
    sx += sw;
    if (sx >= (int)w) break;
  }
}
#endif // FPS_TEST_ANIM

void setup() {
  Serial.begin(115200);
  delay(1000);
  cfg.panel = PANEL;
  cfg.mosi = PIN_MOSI;
  cfg.miso = PIN_MISO;
  cfg.sclk = PIN_SCLK;
  cfg.cs   = PIN_CS; 
  cfg.dc   = PIN_DC;
  cfg.rst  = PIN_RST;
  cfg.bl   = PIN_BL;

  //cfg.bl_start = 155;
  cfg.debug = false;               // enable runtime debug logs  
  cfg.use_hw_spi = true;         // ESP8266: true=HW SPI (D5/D6/D7 фиксированы), false=SW SPI по пинам выше
  // Software SPI runtime options (активны, когда use_hw_spi=false)
  cfg.sw_spi_use_asm = true;       // зарезервированно: ASM-вариант (если будет доступен на платформе)
  cfg.sw_spi_direct_gpio = true;    // true: быстрые прямые регистры GPIO; false: portable digitalWrite() (медленнее, но максимально совместимо)
  cfg.spi_hz     = 80;             // shorthand in MHz; ниже для стабильности ESP8266
  cfg.rotation   =1;             // Поворот экрана (0-3)
  cfg.bl_invert = false;            // Инверсия подсветки
  cfg.bgr        = false;          // set true if colors swapped  
  cfg.allocate_framebuffer = true; 
  // moved: setBrightness() вызывается после ST77xx::setDefault(tft)
#if defined(ARDUINO_ARCH_RP2040)
  cfg.rp2040_use_pio = false; // PIO+DMA или если выкл - SPI+DMA
  // Привязать clk_peri к текущему clk_sys (без изменения clk_sys)
  cfg.rp2040_auto_raise_clk_peri = true;
  // Либо явно задать частоту системы (кГц) и привязать clk_peri:
   cfg.rp2040_target_sysclk_khz = 160000; // 350 МГц -> целевая SPI ≈ 125 МГц
#endif
   
  tft = new ST77xxDMA(cfg);
  if (!tft->begin()) {
    Serial.println("Failed to init ST77xxDMA"); 
    while (1) delay(1);
  }
  // Set default display instance for short-form API
  ST77xx::setDefault(tft);
  setBrightness(155); // максимум: через BL pin или panel-side (при BL=-1)
  // Select a GFX font (Times, ~12 px)
  setGFXFont(&times12pt7b);
  // RU fallback (same ~12 px), if header is available
#if HAVE_RU_12PT
  ST77xx::setGFXFontFallback(&times12pt8b);
#endif
  // Фон: если есть framebuffer — заполним его полосами; иначе рисуем немедленно
  if (framebuffer()) {
    build_color_bars(framebuffer(), width(), height());
    flush();
  } else {
    // Если не используем перерисовку фона каждый кадр, стартуем со статичного фона,
    // чтобы не было «узкой полосы мелких полос» при выключенной анимации.
#if FPS_TEST_ANIM
    if (g_rebuild_bg_each_frame) {
      draw_color_bars_anim_immediate(width(), height(), 0);
    } else {
      draw_color_bars_immediate(width(), height());
    }
#else
    draw_color_bars_immediate(width(), height());
#endif
  }
  // Draw some text (ASCII via Print and UTF-8 via drawTextUTF8) с равномерным вертикальным шагом
  // Прозрачный фон текста: setTextColor(fg) и в drawTextUTF8 передаём bg=fg
  setTextColor(C::White);
  setTextSize(1);
  // Update line height from the selected GFX font
  if (tft && tft->gfxFont()) {
    g_lineH = pgm_read_byte(&tft->gfxFont()->yAdvance);
  }
  setTextWrap(true);
  g_textSize = textSize();
  computeLayout();
  setCursor(4, g_lineY[1]);
  print("ASCII: Hello!");
  // Нормальное использование: C++20 u8-литерал (см. перегрузку const char8_t* в библиотеке)
  drawTextUTF8(4, g_lineY[2], u8"UTF-8: Привет!", C::Yellow, C::Yellow);
  // Алфавиты: 1-я строка EN, 2-я строка RU (верхний регистр)
  drawTextUTF8(4, g_lineY[3], "ABCDEFGHIJKLMNOPQRSTUVWXYZ", C::White, C::White);
  drawTextUTF8(4, g_lineY[4], u8"АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ", C::White, C::White);
  // Никаких flush() — мы рисуем немедленно

#if defined(ARDUINO_ARCH_RP2040)
  // RP2040: аппаратное непрерывное обновление (PIO+DMA, ноль CPU)
  if (cfg.rp2040_use_pio && framebuffer()) {
    if (tft->startContinuousRefreshHW()) {
      Serial.println("[ST77xx][PIO] HW continuous refresh started");
    } else {
      Serial.println("[ST77xx][PIO] HW continuous refresh FAILED; using manual flush");
    }
  }
#endif
 
  Serial.print("Panel: ");
  switch (PANEL) {
    case ST77xxPanel::ST7735_128x160: Serial.print("ST7735 128x160"); break;
    case ST77xxPanel::ST7789_240x320: Serial.print("ST7789 240x320"); break;
    case ST77xxPanel::ST7789_172x320: Serial.print("ST7789 172x320"); break;
    case ST77xxPanel::ST7789_76x284:  Serial.print("ST7789 76x284");  break;
    case ST77xxPanel::ST7789_170x320: Serial.print("ST7789 170x320"); break;
    case ST77xxPanel::NV3007_428x142: Serial.print("NV3007 428x142"); break;
  }
  Serial.print(", SPI MHz: "); Serial.println(cfg.spi_hz);
}

void loop() {
  static uint32_t t0 = millis();
  static uint32_t frames = 0; // используется только в ручном (SW) режиме
  static char buf[24];
  // При необходимости полностью перерисовываем фон каждый кадр (для визуального подтверждения)
  // Рисуем фон немедленными примитивами только если включен режим перерисовки,
  // но фазу анимации двигаем всегда (чтобы FPS-линия не застывала при выключенном фоне)
#if FPS_TEST_ANIM
  if (g_rebuild_bg_each_frame) {
    draw_color_bars_anim_immediate(width(), height(), s_anim_px);
  }
  s_anim_px = (uint16_t)(s_anim_px + FPS_TEST_ANIM_SPEED);
#else
  if (g_rebuild_bg_each_frame) {
    draw_color_bars_immediate(width(), height());
  }
#endif
  // В стресс-режиме перерисовываем текстовые строки поверх фона каждый кадр
  if (g_rebuild_bg_each_frame) {
    setTextColor(C::White);
#if FPS_TEST_ANIM
    const int lh = (int)g_textSize * g_lineH;
    const int fullw = (int)width();
    const int fullh = (int)height();
    // ASCII line
    {
      int yb = g_lineY[1];
      int y0 = yb - lh + 1; if (y0 < 0) y0 = 0; if (y0 + lh > fullh) y0 = fullh - lh;
      if (framebuffer()) draw_color_bars_anim_rect_to_fb(framebuffer(), width(), height(), 0, y0, fullw, lh, s_anim_px);
      setCursor(4, g_lineY[1]);
      print("ASCII: Hello!");
      if (framebuffer()) flushRect(0, y0, fullw, lh);
    }
    // UTF-8 line
    {
      int yb = g_lineY[2];
      int y0 = yb - lh + 1; if (y0 < 0) y0 = 0; if (y0 + lh > fullh) y0 = fullh - lh;
      if (framebuffer()) draw_color_bars_anim_rect_to_fb(framebuffer(), width(), height(), 0, y0, fullw, lh, s_anim_px);
      drawTextUTF8(4, g_lineY[2], u8"UTF-8: Привет!", C::Yellow, C::Yellow);
      if (framebuffer()) flushRect(0, y0, fullw, lh);
    }
    // Alphabets EN
    {
      int yb = g_lineY[3];
      int y0 = yb - lh + 1; if (y0 < 0) y0 = 0; if (y0 + lh > fullh) y0 = fullh - lh;
      if (framebuffer()) draw_color_bars_anim_rect_to_fb(framebuffer(), width(), height(), 0, y0, fullw, lh, s_anim_px);
      drawTextUTF8(4, g_lineY[3], "ABCDEFGHIJKLMNOPQRSTUVWXYZ", C::White, C::White);
      if (framebuffer()) flushRect(0, y0, fullw, lh);
    }
    // Alphabets RU
    {
      int yb = g_lineY[4];
      int y0 = yb - lh + 1; if (y0 < 0) y0 = 0; if (y0 + lh > fullh) y0 = fullh - lh;
      if (framebuffer()) draw_color_bars_anim_rect_to_fb(framebuffer(), width(), height(), 0, y0, fullw, lh, s_anim_px);
      drawTextUTF8(4, g_lineY[4], u8"АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ", C::White, C::White);
      if (framebuffer()) flushRect(0, y0, fullw, lh);
    }
#else
    setCursor(4, g_lineY[1]);
    print("ASCII: Hello!");
    drawTextUTF8(4, g_lineY[2], u8"UTF-8: Привет!", C::Yellow, C::Yellow);
    drawTextUTF8(4, g_lineY[3], "ABCDEFGHIJKLMNOPQRSTUVWXYZ", C::White, C::White);
    drawTextUTF8(4, g_lineY[4], u8"АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ", C::White, C::White);
#endif
  }
  // Полноэкранный вывод кадра каждый цикл — честный FPS по количеству flush в секунду (только с framebuffer)
  bool hw = false;
#if defined(ARDUINO_ARCH_RP2040)
  hw = (tft && tft->isContinuousRefreshHW());
#endif
  // Если есть framebuffer и нет HW-обновления, будем считать FPS по числу flush
  uint32_t t = millis();
  if (t - t0 >= 1000) {
    // Подготовить строку FPS
#if defined(ARDUINO_ARCH_RP2040)
    if (hw) {
      unsigned long hwf = (unsigned long)tft->rp2040HwFrameCount();
      snprintf(buf, sizeof(buf), "FPS(HW): %lu", hwf);
      tft->rp2040HwFrameCountReset();
    } else
#endif
    {
      snprintf(buf, sizeof(buf), "FPS: %lu", (unsigned long)frames);
      frames = 0;
    }
    Serial.println(buf);
    t0 = t;
  }
  // Рисуем FPS поверх каждый кадр, используя последнюю строку buf
  const int x = 4;
  const int y_base = g_lineY[0];
  setTextColor(C::Blue);
  {
#if FPS_TEST_ANIM
    if (framebuffer()) {
      const int lh = (int)g_textSize * g_lineH;
      const int fullw = (int)width();
      const int fullh = (int)height();
      int y0 = y_base - lh + 1; if (y0 < 0) y0 = 0; if (y0 + lh > fullh) y0 = fullh - lh;
      // Подкладываем анимированный фон только если фон кадра тоже перерисовывается
      if (g_rebuild_bg_each_frame) {
        draw_color_bars_anim_rect_to_fb(framebuffer(), width(), height(), 0, y0, fullw, lh, s_anim_px);
      }
      setCursor(x, y_base);
      print(buf);
      flushRect(0, y0, fullw, lh);
    } else {
      setCursor(x, y_base);
      print(buf);
    }
#else
    setCursor(x, y_base);
    print(buf);
#endif
  }
  // Push the frame if using framebuffer without HW continuous refresh
  if (!hw && framebuffer()) {
#if FPS_TEST_ANIM 
    // Анимация рисуется немедленно — не заливаем поверх framebuffer'ом
    frames++;
#else
    flush();
    frames++;
#endif
  }
}
