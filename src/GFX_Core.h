#pragma once
#include <stdint.h>

// Shared GFX core structures (Adafruit GFX / TFT_eSPI compatible)
// Centralized here to avoid duplication across display drivers.
struct GFXglyph {
  uint16_t bitmapOffset; // Offset into GFXfont::bitmap
  uint8_t  width;        // Bitmap dimensions in pixels
  uint8_t  height;       // Bitmap dimensions in pixels
  uint8_t  xAdvance;     // Distance to advance cursor (x axis)
  int8_t   xOffset;      // X dist from cursor pos to UL corner
  int8_t   yOffset;      // Y dist from cursor pos to UL corner
};

struct GFXfont {
  const uint8_t*  bitmap; // Glyph bitmaps, concatenated
  const GFXglyph* glyph;  // Glyph array
  uint16_t first;         // ASCII extents (first char)
  uint16_t last;          // ASCII extents (last char)
  uint8_t  yAdvance;      // Newline distance (y axis)
};
