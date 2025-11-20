#pragma once
#include <Arduino.h>
#include <Print.h>
#include <stdint.h>
#include <initializer_list>

// Shared GFX core (GFXglyph/GFXfont)
#include "GFX_Core.h"

// RP2040 DMA availability check (only when Pico SDK headers are present).
#if defined(ARDUINO_ARCH_RP2040)
#  if defined(__has_include)
#    if __has_include(<hardware/dma.h>) && __has_include(<hardware/spi.h>)
#      define ST77XX_HAVE_RP2040_DMA 1
#    else
#      define ST77XX_HAVE_RP2040_DMA 0
#    endif
#  else
#    define ST77XX_HAVE_RP2040_DMA 0
#  endif
#else
#  define ST77XX_HAVE_RP2040_DMA 0
#endif

// RP2040 PIO availability check (requires PIO + DMA headers)
#if defined(ARDUINO_ARCH_RP2040)
#  if defined(__has_include)
#    if __has_include(<hardware/pio.h>) && __has_include(<hardware/dma.h>)
#      define ST77XX_HAVE_RP2040_PIO 1
#    else
#      define ST77XX_HAVE_RP2040_PIO 0
#    endif
#  else
#    define ST77XX_HAVE_RP2040_PIO 0
#  endif
#else
#  define ST77XX_HAVE_RP2040_PIO 0
#endif

// Forward declaration to avoid pulling SDK headers in public header
#if ST77XX_HAVE_RP2040_PIO
class ST77xxPIO;
#endif



// Controllers
enum class ST77xxController : uint8_t {
  ST7735,
  ST7789,
  NV3007
};

// Panel presets
enum class ST77xxPanel : uint8_t {
  ST7735_128x160,   // 1.8" 128x160
  ST7789_240x320,   // 2.0" 240x320
  ST7789_172x320,   // 1.47" 172x320
  ST7789_76x284,    // 2.25"  76x284
  ST7789_170x320,   // 1.9"  170x320
  ST7789_135x240,   // 
  ST7789_240x240,   // 
  ST7735_80x160,    // 
  NV3007_428x142    // 1.65" 428x142 
};

struct ST77xxConfig {
  ST77xxController controller = ST77xxController::ST7735;
  ST77xxPanel panel           = ST77xxPanel::ST7735_128x160;
  // Dimensions and offsets (0 -> use preset)
  uint16_t width  = 0;
  uint16_t height = 0;
  uint8_t  colstart = 0;
  uint8_t  rowstart = 0;
  bool     bgr = false;
  uint8_t  rotation = 0;       // 0..3

  // Pins
  int8_t mosi = -1;
  int8_t miso = -1;
  int8_t sclk = -1;
  int8_t cs   = -1;            
  int8_t dc   = -1;
  int8_t rst  = -1;
  int8_t bl   = -1;            // Backlight pin (PWM)
  bool   bl_invert = false;    // Invert PWM duty (true: 0 -> max)
  uint32_t bl_freq_hz = 10000; // PWM frequency for backlight (ESP32 LEDC; others use analogWrite default)
  uint8_t  bl_ledc_channel = 0; // ESP32 LEDC channel to use (0..15 depending on core)
  uint8_t  bl_start   = 127;   // Initial backlight level (0..255)
  uint8_t  panel_brightness_start = 127; // Initial panel-side brightness (0..255) when using DCS (e.g., ST7789)


  // SPI
  bool     use_hw_spi = true;  // false -> software SPI bit-bang
  uint32_t spi_hz     = 80u*1000u*1000u; // target SPI clock (Hz). If <=512, treated as MHz shorthand.
  // Software SPI (bit-bang) runtime options
  // When use_hw_spi == false, these control the SW SPI implementation variant.
  // sw_spi_use_asm: prefer inline-assembly tuned variant when available on the platform.
  //                 If not available, the library falls back to direct register writes.
  // sw_spi_direct_gpio: when true (default), use direct GPIO register access where supported;
  //                     when false, fall back to portable digitalWrite() (slower, but safest).
  bool     sw_spi_use_asm = false;
  bool     sw_spi_direct_gpio = true;
  // RP2040: optionally use PIO+DMA for pixel stream (commands still via SPI)
  bool     rp2040_use_pio = false;
  // RP2040: optionally raise system/peripheral clock to meet high SPI targets.
  // By default, the library will NOT change clocks. Set one of the below to opt-in.
  bool     rp2040_auto_raise_clk_peri = false; // if true, tie clk_peri to current clk_sys (no sysclk change)
  uint32_t rp2040_target_sysclk_khz = 0;       // explicit sys clock target in kHz; when >0, set clk_sys and tie clk_peri
  // RP2040: explicitly select SPI instance (auto by default). -1:auto, 0:SPI0, 1:SPI1
  int8_t   rp2040_spi_index = -1;
  // RP2040: force GPIO function(GPIO_FUNC_SPI) on SCLK/MOSI/MISO pins for the selected SPI instance
  bool     rp2040_force_gpio_func = true;

  // Buffer
  bool allocate_framebuffer = true; // full-frame buffer
  // Debug prints at runtime (Serial). Off by default.
  bool debug = false;
  // Optional reusable transmit buffer sized as width * txbuf_lines pixels.
  // This is a small, line-based staging buffer useful on low-RAM MCUs (e.g., ESP8266)
  // when a full framebuffer cannot be allocated. Immediate-mode drawing paths will
  // use this buffer to batch larger SPI writes and avoid large stack arrays.
  // Set to 0 to disable (default).
  uint16_t txbuf_lines = 0;
};

class ST77xxDMA : public Print {
public:
  explicit ST77xxDMA(const ST77xxConfig& cfg);
  ~ST77xxDMA();

  bool begin();
  void end();
  // Enable/disable verbose debug prints (Serial)
  void setDebug(bool enable);

  void setRotation(uint8_t r);
  void setBGR(bool bgr);
  // Toggle panel pixel inversion (INVON/INVOFF)
  void setInversion(bool enable);
  void setOffsets(uint8_t colstart, uint8_t rowstart);
  // Отзеркаливание изображения (независимо от поворота)
  void setMirror(bool mirrorX, bool mirrorY);

  // --- TE (Tearing Effect) control
  // Enable/disable TE signal generation. When enabling, 'mode' selects TE mode (commonly 0x00 = V-blank only).
  void setTE(bool enable, uint8_t mode = 0x00);
  // Set TE scanline (where TE is asserted), if supported (e.g., ST7789, NV3007).
  void setTEScanline(uint16_t line);

  // --- Vertical scroll (MIPI DCS)
  // Set scroll definition (top fixed area, vertical scroll area, bottom fixed area). Not supported on all controllers.
  void setScrollDefinition(uint16_t topFixed, uint16_t scrollArea, uint16_t bottomFixed);
  // Set vertical scroll start address (row offset inside scrolling area).
  void setScroll(uint16_t y);

  // --- Partial display mode
  // Define partial area (inclusive). Use together with setPartialMode(true).
  void setPartialArea(uint16_t y0, uint16_t y1);
  // Enable/disable partial mode (PTLON/NORON).
  void setPartialMode(bool enable);

  // NV3007-specific: choose RAM write command (0x2C RAMWR or 0x3C RAMWRC)
  // Default is RAMWR (0x2C). Enable only if your NV3007 glass prefers RAMWRC.
  void setNV3007UseRAMWRC(bool enable);
  // NV3007-specific: some glasses expect COLMOD=0x55 (16bpp) instead of vendor 0x05.
  // Enable this if you see a vertical line at the left and no image, which typically indicates COLMOD mismatch.
  void setNV3007UseCOLMOD55(bool enable);

  // "Contrast": backlight brightness via PWM (0..255)
  void setBacklight(uint8_t level);
  // Unified brightness API: prefers hardware PWM if BL pin is available,
  // otherwise uses panel-side brightness (e.g., ST7789 WRDISBV). Range 0..255.
  void setBrightness(uint8_t level);

  // Panel-side brightness (aka "contrast") via MIPI DCS for controllers that support it (e.g. ST7789)
  // Effective when panel wiring uses internal brightness control instead of an external BL pin.
  // Range 0..255. No-op on unsupported controllers.
  void setPanelBrightness(uint8_t level);
  void enablePanelBrightness(bool enable);
  // Alias, for convenience
  inline void setContrast(uint8_t level) { setPanelBrightness(level); }

  // Change SPI frequency (takes effect after reinit on some MCUs)
  bool setFrequency(uint32_t hz);

  // Framebuffer API
  uint16_t* framebuffer();
  size_t framebufferSize() const;
  void flush();

  // Immediate API
  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void startRAMWR();
  void writePixelsDMA(const uint16_t* data_be, size_t count);
  void endRAMWR();

#if ST77XX_HAVE_RP2040_PIO
  // RP2040 PIO continuous refresh control and telemetry (proxy to ST77xxPIO)
  bool startContinuousRefreshHW();
  void stopContinuousRefreshHW();
  uint32_t hwFrameCount() const;
  void hwFrameCountReset();
  // Query if HW continuous refresh (PIO+DMA) is currently active
  bool isContinuousRefreshHW() const;
  // Compatibility wrappers (legacy example names)
  inline uint32_t rp2040HwFrameCount() const { return hwFrameCount(); }
  inline void rp2040HwFrameCountReset() { hwFrameCountReset(); }
#endif
#if !ST77XX_HAVE_RP2040_PIO
  // Fallback stubs when PIO backend is not compiled in
  inline bool startContinuousRefreshHW() { return false; }
  inline void stopContinuousRefreshHW() {}
  inline uint32_t hwFrameCount() const { return 0; }
  inline void hwFrameCountReset() {}
  inline bool isContinuousRefreshHW() const { return false; }
  inline uint32_t rp2040HwFrameCount() const { return 0; }
  inline void rp2040HwFrameCountReset() {}
#endif
  inline uint16_t width() const { return _lcdW; }
  inline uint16_t height() const { return _lcdH; }

  // --- Примитивы рисования (RGB565 цвет)
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void fillScreen(uint16_t color);

  // Расширенные примитивы
  void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
  void drawTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t color);
  void fillTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t color);
  void drawRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t color);
  void fillRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t color);

  // --- Текст (минимальный API, задел под шрифты)
  void drawChar(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
  void drawText(int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size);
  void drawTextUTF8(int16_t x, int16_t y, const char* utf8, uint16_t color, uint16_t bg, uint8_t size);
#if defined(__cpp_char8_t)
  // Overload for C++20 u8"..." literals (const char8_t*)
  void drawTextUTF8(int16_t x, int16_t y, const char8_t* utf8, uint16_t color, uint16_t bg, uint8_t size);
#endif

  // Print-интерфейс и параметры текста
  virtual size_t write(uint8_t b) override; // потоковый вывод (UTF-8)
  void setCursor(int16_t x, int16_t y) { _cursorX = x; _cursorY = y; }
  void getCursor(int16_t& x, int16_t& y) const { x = _cursorX; y = _cursorY; }
  void setTextColor(uint16_t fg) { _textColor = fg; _textBg = fg; }
  void setTextColor(uint16_t fg, uint16_t bg) { _textColor = fg; _textBg = bg; }
  // Convenience helpers (explicitly named to avoid overload ambiguity)
  void setTextColor24(uint32_t fg24) { _textColor = rgb565((fg24>>16)&0xFF, (fg24>>8)&0xFF, fg24&0xFF); _textBg = _textColor; }
  void setTextColor24(uint32_t fg24, uint32_t bg24) {
    _textColor = rgb565((fg24>>16)&0xFF, (fg24>>8)&0xFF, fg24&0xFF);
    _textBg    = rgb565((bg24>>16)&0xFF, (bg24>>8)&0xFF, bg24&0xFF);
  }
  void setTextColorRGB(uint8_t r, uint8_t g, uint8_t b) { _textColor = rgb565(r,g,b); _textBg = _textColor; }
  void setTextColorRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t br, uint8_t bg, uint8_t bb) {
    _textColor = rgb565(r,g,b); _textBg = rgb565(br,bg,bb);
  }
  void setTextSize(uint8_t s) { _textSize = s ? s : 1; }
  void setTextWrap(bool w) { _textWrap = w; }
  uint8_t textSize() const { return _textSize; }
  bool textWrap() const { return _textWrap; }
  // Additional text layout control: extra pixels between characters (can be negative). Default 0.
  void setLetterSpacing(int8_t px) { _letterSpacing = px; }
  int8_t letterSpacing() const { return _letterSpacing; }

  void setGFXFont(const GFXfont* font);
  inline const GFXfont* gfxFont() const { return _gfxFont; }
  // Фолбэк-шрифт: будет использован для кодпоинтов, отсутствующих в основном
  void setGFXFontFallback(const GFXfont* font);
  inline const GFXfont* gfxFontFallback() const { return _gfxFontFallback; }
  // Установить оба сразу (удобство)
  void setFonts(const GFXfont* primary, const GFXfont* fallback);

  // Clip-rect
  void setClipRect(int16_t x, int16_t y, int16_t w, int16_t h);
  void resetClipRect();

  // Частичные обновления из фреймбуфера
  void flushRect(int16_t x, int16_t y, int16_t w, int16_t h);

  // Транзакции (зарезервировано под продвинутый немедленный режим)
  void beginWrite();
  void endWrite();

  // Битмапы
  void drawBitmap1(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t* bits, uint16_t color, uint16_t bg, bool transparent_bg);
  void drawRGBBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* rgb565_le);

  static constexpr uint16_t rgb565(uint8_t r,uint8_t g,uint8_t b) {
    return (uint16_t)(((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3));
  }
  static constexpr uint16_t toBE(uint16_t le) { return (uint16_t)((le<<8)|(le>>8)); }

  // Common color constants (like other gfx libs)
  struct Colors {
    static constexpr uint16_t Black   = 0x0000;
    static constexpr uint16_t White   = 0xFFFF;
    static constexpr uint16_t Red     = 0xF800;
    static constexpr uint16_t Green   = 0x07E0;
    static constexpr uint16_t Blue    = 0x001F;
    static constexpr uint16_t Yellow  = 0xFFE0;
    static constexpr uint16_t Cyan    = 0x07FF;
    static constexpr uint16_t Magenta = 0xF81F;
    static constexpr uint16_t Gray    = 0x8410;
    static constexpr uint16_t Orange  = 0xFD20;
    static constexpr uint16_t Purple  = 0x8010;
  };

private:
  ST77xxConfig _cfg;
  uint16_t _lcdW = 0, _lcdH = 0;
  uint8_t  _colstart = 0, _rowstart = 0;
  // Effective offsets for the current rotation (applied in address window commands)
  uint8_t  _xstart = 0, _ystart = 0;
  bool     _bgr = false;
  uint8_t  _rotation = 0;
  bool     _mirrorX = false;
  bool     _mirrorY = false;

  // Clip
  int16_t _clipX1 = 0, _clipY1 = 0, _clipX2 = 0, _clipY2 = 0; // [x1,x2] inclusive
  bool    _clipEnabled = false;

  // Текст/шрифт (теперь только GFX/TFT_eSPI)
  const GFXfont* _gfxFont = nullptr; // пропорциональный шрифт формата Adafruit GFX/TFT_eSPI
  const GFXfont* _gfxFontFallback = nullptr; // запасной шрифт для отсутствующих глифов

  // Состояние транзакции
  bool _inWrite = false;

  // Текст/Print state
  int16_t _cursorX = 0, _cursorY = 0;
  uint16_t _textColor = 0xFFFF; // white
  uint16_t _textBg    = 0x0000; // black
  uint8_t  _textSize  = 1;
  bool     _textWrap  = true;
  int8_t   _letterSpacing = 0; // extra pixels between characters (scaled by size)
  // UTF-8 streaming decoder state for Print::write
  uint8_t  _utf8_state = 0; // 0..3 remaining bytes
  uint32_t _utf8_cp    = 0;

  // Buffer
  uint16_t* _frame = nullptr;
  size_t    _frame_bytes = 0;
  size_t    _half_bytes  = 0;
  // Optional reusable transmit buffer (partial, line-based staging)
  uint16_t* _txbuf = nullptr;     // BE-order pixels
  size_t    _txbuf_words = 0;     // capacity in pixels (words)
  uint16_t  _txbuf_lines = 0;     // number of lines the buffer can hold (cfg-driven)

  // Backend SPI
  void backendInitSPI();
  void backendDeinitSPI();

  void sendCmd(uint8_t cmd);
  void sendData(const void* data, size_t len);
  void sendCmdData(uint8_t cmd, const void* data, size_t len);

  void writeWindowPixels(const void* data, size_t len);

  // Panel init
  void applyPanelPreset();
  void panelInitBasic();
  void panelInitNV3007();
  void writeMADCTL();
  void writeCOLMOD();

  // Address window
  void setWindowFull();
  void setWindowY(uint16_t y0, uint16_t y1);
  void setPin(uint8_t pin, uint8_t val);
  void setPins(uint32_t pins, uint8_t val);
  void sendCmdList(const uint8_t *addr);
  // Select RAM write opcode depending on controller and NV3007 preference
  uint8_t ramwrCmd() const;

  // GPIO helpers
  inline void csLow();
  inline void csHigh();
  inline void dcCmd();
  inline void dcData();

  void resetPanel();

  // Backlight
  void setupBacklight();
  void applyBacklight(uint8_t level);

  // Platform specifics
  void hwSend(const void* data, size_t len, bool cmd);
  void swSend(const void* data, size_t len, bool cmd);
  void hwWritePixels(const void* data, size_t len);

  // Текст: вспомогательные
  // Нарисовать глиф из конкретного шрифта (без выбора фолбэка)
  void drawGlyphGFX(int16_t x, int16_t y, const GFXfont* font, uint32_t cp, uint16_t color, uint16_t bg, uint8_t size);

  bool _begun = false;
  bool _debug = false; // runtime debug flag
  // One-time log guard: print flush mode only once per session
  bool _flushModeLogged = false;
  // Suppress nested debug prints (e.g., when sendCmdData() calls sendCmd()/sendData())
  bool _dbg_suppress = false;

  // Debug log de-duplication for address window commands
  // Cache last 4-byte payloads for CASET/RASET and suppress repeated identical logs
  bool    _log_have_caset = false;
  uint8_t _log_last_caset[4] = {0,0,0,0};
  uint32_t _log_dup_caset = 0;
  bool    _log_have_raset = false;
  uint8_t _log_last_raset[4] = {0,0,0,0};
  uint32_t _log_dup_raset = 0;

  // NV3007 options (do not affect other controllers)
  bool _nv3007_use_ramwrc = false;
  bool _nv3007_colmod_55 = false;

  // Backend selection flags
  bool _rp2040_use_pio = false; // mirror of cfg.rp2040_use_pio
#if ST77XX_HAVE_RP2040_PIO
  ST77xxPIO* _rp2040_pio = nullptr; // RP2040 PIO+DMA backend (optional)
  bool _rp2040_hw_running = false;  // true while HW continuous refresh is active
#endif

#if defined(ARDUINO_ARCH_ESP32)
  void* _spi_dev = nullptr; // spi_device_handle_t
  int   _spi_host = 0;      // VSPI_HOST/SPI2_HOST/SPI3_HOST
#endif

#if ST77XX_HAVE_RP2040_DMA
  // RP2040 DMA state (Pico SDK). We keep types opaque here to avoid including SDK headers in the public header.
  int   _rp2040_dma_ch = -1;   // DMA channel index
  void* _rp2040_spi    = nullptr; // spi_inst_t*
  uint32_t _rp2040_dreq = 0;   // DREQ_SPIx_TX value
  uint32_t _rp2040_spi_actual_hz = 0; // actual SPI baud after setup

  // RP2040 DMA helpers (implemented in .cpp)
  void rp2040InitDMA();
  void rp2040DeinitDMA();
  void rp2040DMASend(const uint8_t* data, size_t len);
#endif
};

namespace ST77xx {
  // Set/get default display instance (defined in ST77xxDMA.cpp)
  void setDefault(ST77xxDMA* d);
  ST77xxDMA* defaultDisplay();

  // Helpers
  inline bool has() { return defaultDisplay() != nullptr; }

  // Core state/info
  inline uint16_t width()  { auto d = defaultDisplay(); return d ? d->width()  : 0; }
  inline uint16_t height() { auto d = defaultDisplay(); return d ? d->height() : 0; }
  inline uint16_t* framebuffer() { auto d = defaultDisplay(); return d ? d->framebuffer() : nullptr; }
  inline size_t framebufferSize() { auto d = defaultDisplay(); return d ? d->framebufferSize() : 0; }

  // Control
  inline void flush() { if (auto d = defaultDisplay()) d->flush(); }
  inline bool setFrequency(uint32_t hz) { auto d = defaultDisplay(); return d ? d->setFrequency(hz) : false; }
  inline void setRotation(uint8_t r) { if (auto d = defaultDisplay()) d->setRotation(r); }
  inline void setMirror(bool mx, bool my) { if (auto d = defaultDisplay()) d->setMirror(mx, my); }
  inline void setBrightness(uint8_t level) { if (auto d = defaultDisplay()) d->setBrightness(level); }
  inline void setBacklight(uint8_t level) { if (auto d = defaultDisplay()) d->setBacklight(level); }
  inline void setInversion(bool en) { if (auto d = defaultDisplay()) d->setInversion(en); }
  inline void setTE(bool en, uint8_t mode = 0x00) { if (auto d = defaultDisplay()) d->setTE(en, mode); }
  inline void setTEScanline(uint16_t line) { if (auto d = defaultDisplay()) d->setTEScanline(line); }
  // Additional control helpers
  inline void setBGR(bool b) { if (auto d = defaultDisplay()) d->setBGR(b); }
  inline void setOffsets(uint8_t colstart, uint8_t rowstart) { if (auto d = defaultDisplay()) d->setOffsets(colstart, rowstart); }
  inline void setScrollDefinition(uint16_t tfa, uint16_t vsa, uint16_t bfa) { if (auto d = defaultDisplay()) d->setScrollDefinition(tfa, vsa, bfa); }
  inline void setScroll(uint16_t y) { if (auto d = defaultDisplay()) d->setScroll(y); }
  inline void setPartialArea(uint16_t y0, uint16_t y1) { if (auto d = defaultDisplay()) d->setPartialArea(y0, y1); }
  inline void setPartialMode(bool en) { if (auto d = defaultDisplay()) d->setPartialMode(en); }
  inline void setClipRect(int16_t x, int16_t y, int16_t w, int16_t h) { if (auto d = defaultDisplay()) d->setClipRect(x, y, w, h); }
  inline void resetClipRect() { if (auto d = defaultDisplay()) d->resetClipRect(); }
  inline void flushRect(int16_t x, int16_t y, int16_t w, int16_t h) { if (auto d = defaultDisplay()) d->flushRect(x, y, w, h); }
  inline void enablePanelBrightness(bool en) { if (auto d = defaultDisplay()) d->enablePanelBrightness(en); }
  inline void setPanelBrightness(uint8_t level) { if (auto d = defaultDisplay()) d->setPanelBrightness(level); }

  // Drawing (RGB565)
  inline void fillScreen(uint16_t c) { if (auto d = defaultDisplay()) d->fillScreen(c); }
  inline void drawPixel(int16_t x, int16_t y, uint16_t c) { if (auto d = defaultDisplay()) d->drawPixel(x,y,c); }
  inline void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t c) { if (auto d = defaultDisplay()) d->drawFastHLine(x,y,w,c); }
  inline void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t c) { if (auto d = defaultDisplay()) d->drawFastVLine(x,y,h,c); }
  inline void drawLine(int16_t x0,int16_t y0,int16_t x1,int16_t y1,uint16_t c) { if (auto d = defaultDisplay()) d->drawLine(x0,y0,x1,y1,c); }
  inline void drawRect(int16_t x,int16_t y,int16_t w,int16_t h,uint16_t c) { if (auto d = defaultDisplay()) d->drawRect(x,y,w,h,c); }
  inline void fillRect(int16_t x,int16_t y,int16_t w,int16_t h,uint16_t c) { if (auto d = defaultDisplay()) d->fillRect(x,y,w,h,c); }
  inline void drawCircle(int16_t x0,int16_t y0,int16_t r,uint16_t c) { if (auto d = defaultDisplay()) d->drawCircle(x0,y0,r,c); }
  inline void fillCircle(int16_t x0,int16_t y0,int16_t r,uint16_t c) { if (auto d = defaultDisplay()) d->fillCircle(x0,y0,r,c); }
  inline void drawTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t c) { if (auto d = defaultDisplay()) d->drawTriangle(x0,y0,x1,y1,x2,y2,c); }
  inline void fillTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t c) { if (auto d = defaultDisplay()) d->fillTriangle(x0,y0,x1,y1,x2,y2,c); }
  inline void drawRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t c) { if (auto d = defaultDisplay()) d->drawRoundRect(x,y,w,h,r,c); }
  inline void fillRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t c) { if (auto d = defaultDisplay()) d->fillRoundRect(x,y,w,h,r,c); }

  // Text state
  inline void setCursor(int16_t x, int16_t y) { if (auto d = defaultDisplay()) d->setCursor(x,y); }
  inline void getCursor(int16_t& x, int16_t& y) { if (auto d = defaultDisplay()) d->getCursor(x,y); }
  inline void setTextColor(uint16_t fg) { if (auto d = defaultDisplay()) d->setTextColor(fg); }
  inline void setTextColor(uint16_t fg, uint16_t bg) { if (auto d = defaultDisplay()) d->setTextColor(fg,bg); }
  inline void setTextColor24(uint32_t fg24) { if (auto d = defaultDisplay()) d->setTextColor24(fg24); }
  inline void setTextColor24(uint32_t fg24, uint32_t bg24) { if (auto d = defaultDisplay()) d->setTextColor24(fg24,bg24); }
  inline void setTextColorRGB(uint8_t r,uint8_t g,uint8_t b) { if (auto d = defaultDisplay()) d->setTextColorRGB(r,g,b); }
  inline void setTextColorRGB(uint8_t r,uint8_t g,uint8_t b,uint8_t br,uint8_t bg,uint8_t bb) { if (auto d = defaultDisplay()) d->setTextColorRGB(r,g,b,br,bg,bb); }
  inline void setTextSize(uint8_t s) { if (auto d = defaultDisplay()) d->setTextSize(s); }
  inline void setTextWrap(bool w) { if (auto d = defaultDisplay()) d->setTextWrap(w); }
  inline uint8_t textSize() { auto d = defaultDisplay(); return d ? d->textSize() : 1; }
  inline bool textWrap() { auto d = defaultDisplay(); return d ? d->textWrap() : true; }
  inline void setLetterSpacing(int8_t px) { if (auto d = defaultDisplay()) d->setLetterSpacing(px); }
  inline int8_t letterSpacing() { auto d = defaultDisplay(); return d ? d->letterSpacing() : 0; }

  // Fonts
  inline void setGFXFont(const GFXfont* f) { if (auto d = defaultDisplay()) d->setGFXFont(f); }
  inline void setGFXFontFallback(const GFXfont* f) { if (auto d = defaultDisplay()) d->setGFXFontFallback(f); }
  inline void setFonts(const GFXfont* primary, const GFXfont* fallback) { if (auto d = defaultDisplay()) d->setFonts(primary, fallback); }

  // Text output helpers
  inline void drawText(int16_t x, int16_t y, const char* s, uint16_t fg, uint16_t bg, uint8_t size=1) { if (auto d = defaultDisplay()) d->drawText(x,y,s,fg,bg,size); }
  inline void drawTextUTF8(int16_t x, int16_t y, const char* s, uint16_t fg, uint16_t bg, uint8_t size=1) { if (auto d = defaultDisplay()) d->drawTextUTF8(x,y,s,fg,bg,size); }
#if defined(__cpp_char8_t)
  inline void drawTextUTF8(int16_t x, int16_t y, const char8_t* s, uint16_t fg, uint16_t bg, uint8_t size=1) { if (auto d = defaultDisplay()) d->drawTextUTF8(x,y,s,fg,bg,size); }
#endif

  // Immediate mode helpers
  inline void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) { if (auto d = defaultDisplay()) d->setAddrWindow(x0, y0, x1, y1); }
  inline void startRAMWR() { if (auto d = defaultDisplay()) d->startRAMWR(); }
  inline void writePixelsDMA(const uint16_t* data_be, size_t count) { if (auto d = defaultDisplay()) d->writePixelsDMA(data_be, count); }
  inline void endRAMWR() { if (auto d = defaultDisplay()) d->endRAMWR(); }
  // Print-like shortcuts
  inline size_t print(const char* s) { auto d = defaultDisplay(); return d ? d->print(s) : 0; }
  inline size_t println(const char* s) { auto d = defaultDisplay(); return d ? d->println(s) : 0; }

  // --------------------------------------------------------
  // Convenience color helpers and friendly wrappers
  // Make user code shorter and clearer (no need to call ST77xxDMA::rgb565 directly)
  inline constexpr uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
    return ST77xxDMA::rgb565(r, g, b);
  }
  inline constexpr uint16_t color24(uint32_t rgb24) {
    return ST77xxDMA::rgb565((uint8_t)(rgb24 >> 16), (uint8_t)(rgb24 >> 8), (uint8_t)rgb24);
  }
  inline constexpr uint16_t toBE(uint16_t le) {
    return ST77xxDMA::toBE(le);
  }

  // Color constants alias (like C::Black, C::Green, ...)
  namespace C {
    static constexpr uint16_t Black   = ST77xxDMA::Colors::Black;
    static constexpr uint16_t White   = ST77xxDMA::Colors::White;
    static constexpr uint16_t Red     = ST77xxDMA::Colors::Red;
    static constexpr uint16_t Green   = ST77xxDMA::Colors::Green;
    static constexpr uint16_t Blue    = ST77xxDMA::Colors::Blue;
    static constexpr uint16_t Yellow  = ST77xxDMA::Colors::Yellow;
    static constexpr uint16_t Cyan    = ST77xxDMA::Colors::Cyan;
    static constexpr uint16_t Magenta = ST77xxDMA::Colors::Magenta;
    static constexpr uint16_t Gray    = ST77xxDMA::Colors::Gray;
    static constexpr uint16_t Orange  = ST77xxDMA::Colors::Orange;
    static constexpr uint16_t Purple  = ST77xxDMA::Colors::Purple;
  }

  // Friendly wrappers for RGB/24-bit arguments
  inline void fillRectRGB(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t r, uint8_t g, uint8_t b) {
    if (auto d = defaultDisplay()) d->fillRect(x, y, w, h, ST77xxDMA::rgb565(r, g, b));
  }
  inline void fillRect24(int16_t x, int16_t y, int16_t w, int16_t h, uint32_t rgb24) {
    if (auto d = defaultDisplay()) d->fillRect(x, y, w, h, color24(rgb24));
  }

  inline void drawText24(int16_t x, int16_t y, const char* s, uint32_t fg24, uint32_t bg24, uint8_t size = 1) {
    if (auto d = defaultDisplay()) d->drawText(x, y, s, color24(fg24), color24(bg24), size);
  }
  inline void drawTextRGB(int16_t x, int16_t y, const char* s,
                          uint8_t fr, uint8_t fg, uint8_t fb,
                          uint8_t br, uint8_t bg, uint8_t bb,
                          uint8_t size = 1) {
    if (auto d = defaultDisplay()) d->drawText(x, y, s,
      ST77xxDMA::rgb565(fr, fg, fb), ST77xxDMA::rgb565(br, bg, bb), size);
  }
}
