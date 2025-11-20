# ST77xxDMA – Schnelle TFT‑Displays mit ST77xx‑Controllern (DMA)

Die Bibliothek **ST77xxDMA** dient zur Ansteuerung von Farb‑TFT‑Displays
mit Controllern der Familie **ST77xx** (z. B. ST7735, ST7789, NV3007).
Schwerpunkt sind **hohe Geschwindigkeit**, **DMA‑Unterstützung** und
komfortable Bedienung (inkl. UTF‑8‑Text mit kyrillischen Zeichen).

## Funktionen im Überblick

- **Unterstützte Controller**
  - `ST7735`
  - `ST7789`
  - `NV3007`

- **Vordefinierte Panel‑Presets** (`ST77xxPanel`)
  - `ST7735_128x160` – 1.8" 128×160
  - `ST7735_80x160`  – 0.96" 80×160
  - `ST7789_240x320` – 2.0" 240×320
  - `ST7789_240x240` – 1.54" 240×240
  - `ST7789_170x320` – 1.9" 170×320
  - `ST7789_172x320` – 1.47" 172×320
  - `ST7789_76x284`  – 2.25" 76×284
  - `NV3007_428x142` – 1.65" 428×142

- **SPI‑Anbindung**
  - Hardware‑SPI mit frei wählbarer Taktfrequenz (bis in den zweistelligen MHz‑Bereich,
    je nach MCU und Panel).
  - Software‑SPI (Bit‑Bang) mit optimierten Pfaden für verschiedene Architekturen.

- **DMA und Framebuffer**
  - Optionaler voller **Frame­buffer** im RGB565‑Format.
  - Komplettes Senden des Framebuffers per `flush()` über SPI/DMA.
  - Teilaktualisierung mit `flushRect(x, y, w, h)`.

- **Hintergrundbeleuchtung & Helligkeit**
  - Externer Backlight‑Pin `BL` (PWM) für alle unterstützten Controller.
  - Panel‑seitige Helligkeit (MIPI DCS `WRDISBV`) für ST7789‑Module ohne BL‑Pin.
  - Einheitliches API `setBrightness(0..255)`, das je nach Konfiguration
    automatisch PWM oder Panel‑DCS verwendet.

- **Text & Fonts**
  - Unterstützung von Fonts im Format **Adafruit GFX / TFT_eSPI GFXFF**.
  - UTF‑8‑Ausgabe (inkl. Kyrillisch) über `Print`‑Interface und `drawTextUTF8`.
  - Einstellbarer Zeichenabstand (`setLetterSpacing`).

- **Zeichen‑Primitive**
  - Pixel, Linien, Rechtecke, gefüllte Flächen.
  - Kreise, Dreiecke, abgerundete Rechtecke.
  - 1‑Bit‑Bitmaps und RGB565‑Bitmaps.

- **Weitere Features**
  - Rotation `setRotation(0..3)` und Spiegelung `setMirror()`.
  - Farb­inversion `setInversion(true/false)`.
  - TE‑Signal (Tearing Effect), vertikales Scrollen, Partial‑Display‑Mode.
  - Erweiterte Optionen für **RP2040** (PIO+DMA, Anhebung von System‑/Peripherie‑Takt).


## Installation

### Arduino IDE

1. Dieses Repository klonen oder als ZIP herunterladen und in den Ordner
   `libraries` der Arduino‑Umgebung entpacken, z. B.:

   `Dokumente/Arduino/libraries/ST77xxDMA`

2. Arduino IDE neu starten.
3. Die Bibliothek erscheint nun im Menü **Sketch → Bibliothek einbinden**.

### PlatformIO

In `platformio.ini` als Git‑Abhängigkeit eintragen:

```ini
lib_deps =
    https://github.com/Atmel2005/ST77xx_DMA.git
```

oder die Bibliothek in den Projektordner `lib/` kopieren.


## Schnellstart

### 1. Grundinitialisierung

Ein einfaches Beispiel für ein **ST7789‑Display 170×320**
(analog für andere Panel‑Presets):

```cpp
#include <ST77xxDMA.h>

ST77xxDMA* tft;

void setup() {
  ST77xxConfig cfg;

  // Panel auswählen (siehe Liste oben)
  cfg.panel = ST77xxPanel::ST7789_170x320;

  // SPI
  cfg.use_hw_spi = true;   // Hardware‑SPI verwenden
  cfg.spi_hz     = 80;     // 80 MHz (Werte <=512 werden als MHz interpretiert)

  // Ausrichtung & Basisoptionen
  cfg.rotation   = 1;      // 0..3
  cfg.allocate_framebuffer = true; // Vollbild‑Framebuffer für DMA

  // Pins (Beispiele — an eigene Hardware anpassen)
  cfg.dc  = PIN_DC;        // D/C (Pflicht)
  cfg.cs  = PIN_CS;        // CS (empfohlen, kein Auto‑CS)
  cfg.rst = PIN_RST;       // RST (-1, falls nicht genutzt)
  cfg.bl  = PIN_BL;        // Backlight‑Pin (>=0, falls vorhanden)

  tft = new ST77xxDMA(cfg);
  tft->begin();

  // Kurz‑API über Namespace ST77xx
  ST77xx::setDefault(tft);

  ST77xx::setRotation(cfg.rotation);
  ST77xx::setBGR(cfg.bgr);
  ST77xx::setBrightness(160);   // Gesamt­helligkeit 0..255

  ST77xx::fillScreen(ST77xx::C::Black);
  ST77xx::flush();
}

void loop() {
  // Benutzer‑Code
}
```

### 2. Panel auswählen

Das wichtigste Feld ist `cfg.panel`:

```cpp
cfg.panel = ST77xxPanel::ST7789_240x320; // 2.0" 240×320
// oder
cfg.panel = ST77xxPanel::ST7789_240x240; // 1.54" 240×240
// oder
cfg.panel = ST77xxPanel::ST7789_135x240; // 1.14" 135×240
// oder
cfg.panel = ST77xxPanel::ST7735_80x160;  // 0.96" 80×160
```

Der Controller (`cfg.controller`) wird automatisch zum Panel passend gesetzt.
Falls Ihr Display besondere RAM‑Offsets besitzt, können
`width/height/colstart/rowstart` manuell überschrieben werden.


## Konfiguration: `ST77xxConfig`

`ST77xxConfig` fasst alle relevanten Einstellungen zusammen.
Nur ein Teil der Felder ist im Alltag wichtig — für den Rest siehe
`src/ST77xxDMA.h` oder die russische Dokumentation `ST77xxDMA_USAGE_RU.txt`.

### Grundlegende Felder

- `ST77xxController controller` – Display‑Controller
  (`ST7735`, `ST7789`, `NV3007`). Wird in der Regel automatisch gesetzt.
- `ST77xxPanel panel` – Panel‑Preset (siehe oben).
- `uint16_t width`, `height` – logische Auflösung; `0` = Werte aus Preset.
- `uint8_t colstart`, `rowstart` – RAM‑Offsets für das sichtbare Fenster.
- `bool bgr` – Farb­reihenfolge (false = RGB, true = BGR).
- `uint8_t rotation` – Drehung 0..3.

### Pins

- `int8_t mosi`, `miso`, `sclk` – SPI‑Pins (für SW‑SPI und einige HW‑SPI‑Plattformen).
- `int8_t cs`  – Chip‑Select des Displays.
- `int8_t dc`  – Data/Command.
- `int8_t rst` – Reset‑Pin (-1, falls unbenutzt).
- `int8_t bl`  – Backlight‑Pin (PWM). `-1`, wenn kein externer BL‑Pin vorhanden ist.

### Backlight (PWM über BL‑Pin)

- `bool bl_invert` – invertiertes PWM (true: 0 = maximale Helligkeit).
- `uint32_t bl_freq_hz` – PWM‑Frequenz (ESP32/ESP8266/RP2040).
- `uint8_t bl_ledc_channel` – LEDC‑Kanal auf ESP32.
- `uint8_t bl_start` – Starthelligkeit 0..255.

Nach `begin()`:

```cpp
ST77xx::setBrightness(200); // bevorzugtes API (wählt PWM oder DCS automatisch)
// oder bei Bedarf direkter PWM‑Zugriff:
// ST77xx::setBacklight(200);
```

### Panel‑seitige Helligkeit (ST7789 ohne BL‑Pin)

Wenn `cfg.bl < 0` und der Controller `ST7789` ist, nutzt die Bibliothek
MIPI‑DCS‑Kommandos zur Regelung der Hintergrundbeleuchtung im Panel selbst:

- `cfg.panel_brightness_start` – Startwert 0..255.

Im Code:

```cpp
ST77xx::setBrightness(120);      // steuert die Panelhelligkeit
// Low‑Level bei Bedarf:
// ST77xx::enablePanelBrightness(true);
// ST77xx::setPanelBrightness(120);
```

### SPI

- `bool use_hw_spi` – `true` für Hardware‑SPI, `false` für SW‑SPI.
- `uint32_t spi_hz` – Ziel‑SPI‑Takt in Hz;
  - Werte ≤512 werden als MHz interpretiert (80 → 80 MHz).
- `bool sw_spi_use_asm` – ASM‑optimierter SW‑SPI‑Pfad (falls verfügbar).
- `bool sw_spi_direct_gpio` – direkte GPIO‑Register statt `digitalWrite`.

### RP2040 / PIO

- `bool rp2040_use_pio` – PIO+DMA‑Backend (experimentell).
- `bool rp2040_auto_raise_clk_peri` – `clk_peri` auf `clk_sys` anheben.
- `uint32_t rp2040_target_sysclk_khz` – expliziter `clk_sys` in kHz.
- `int8_t rp2040_spi_index` – Auswahl SPI‑Instanz (-1 = auto, 0 = SPI0, 1 = SPI1).

### Speicher & Debug

- `bool allocate_framebuffer` – Vollbild‑Framebuffer anlegen (empfohlen für DMA).
- `uint16_t txbuf_lines` – kleiner Zeilenpuffer (v. a. für ESP8266).
- `bool debug` – ausführliche Debug‑Ausgaben auf `Serial`.


## Zeichnen & Text

Nach `ST77xx::setDefault(tft)` steht ein kompakter Global‑API zur Verfügung.

### Zeichen‑Primitive

```cpp
using namespace ST77xx;

fillScreen(C::Black);
drawPixel(10, 10, C::Red);
drawFastHLine(0, 20, 100, C::Green);
drawRect(10, 30, 50, 20, C::Yellow);
fillRect(70, 30, 40, 20, C::Blue);
flush();
```

### Textausgabe

```cpp
using namespace ST77xx;

setCursor(10, 50);
setTextColor(C::White, C::Black);
setTextSize(2);
print("Hello, ST77xx!");
flush();
```

Für UTF‑8‑Ausgabe und GFX‑Fonts (z. B. Times, FreeSans, kyrillische Fonts)
stehen zusätzliche Funktionen wie `setGFXFont()` und `drawTextUTF8()` zur Verfügung.
Beispiele dazu finden Sie in `examples/` und in der Datei
`ST77xxDMA_USAGE_RU.txt`.


## Erweiterte Display‑Funktionen

- **TE (Tearing Effect)**
  - `setTE(bool enable, uint8_t mode = 0x00);`
  - `setTEScanline(uint16_t line);`

- **Vertikales Scrollen**
  - `setScrollDefinition(tfa, vsa, bfa);`
  - `setScroll(y);`

- **Partial Display Mode**
  - `setPartialArea(y0, y1);`
  - `setPartialMode(true/false);`

- **Clip‑Rechteck**
  - `setClipRect(x, y, w, h);`
  - `resetClipRect();`

Alle Funktionen existieren sowohl als Methoden des Klassenobjekts
`ST77xxDMA` als auch als Wrapper im `namespace ST77xx`.


## Beispiele

Im Ordner `examples/` befinden sich mehrere Demos:

- **FpsTest** – Performance‑/Stress‑Test mit animierten Balken und Text.
- **NV3007_Intro** – Demonstration eines NV3007‑Panels (Plasma, HUD, FPS‑Anzeige).
- **FontAllSizesValidation** – Überprüfung aller verfügbaren Fontgrößen.
- **RP2040_PIO_DMA_9bit_Min** – Beispiel für RP2040 mit PIO+DMA.

Ein guter Einstieg ist **FpsTest**; passen Sie dort die
`ST77xxConfig`‑Einstellungen (Panel, Pins, SPI‑Takt, Helligkeit)
an Ihre Hardware an.

---

Weitere Details (insbesondere auf Russisch) finden Sie in der Datei
`ST77xxDMA_USAGE_RU.txt` im Wurzelverzeichnis des Repositories.
