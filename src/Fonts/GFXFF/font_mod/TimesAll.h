#pragma once
#include <ST77xxDMA.h>

// Собрание всех шрифтов Times (regular/bold) из font_mod
// Предоставляет функции выбора по желаемому размеру в пикселях (5..40):
//   TimesFF::regular(px)
//   TimesFF::bold(px)
// px будет зажат в диапазон 5..40. Для отсутствующих размеров подбирается ближайший доступный.

// Regular (kurze)
#include <Fonts/GFXFF/font_mod/kurze/times5pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times6pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times7pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times8pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times9pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times10pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times11pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times12pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times13pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times14pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times15pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times16pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times17pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times18pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times19pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times20pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times21pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times22pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times23pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times24pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times25pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times26pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times27pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times28pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times29pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times30pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times31pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times32pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times33pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times34pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times35pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times36pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times37pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times38pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times39pt7b.h>
#include <Fonts/GFXFF/font_mod/kurze/times40pt7b.h>

// Bold (lange)
#include <Fonts/GFXFF/font_mod/lange/timesbd5pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd6pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd7pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd8pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd9pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd10pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd11pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd12pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd13pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd14pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd15pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd16pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd17pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd18pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd19pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd20pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd21pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd22pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd23pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd24pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd25pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd26pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd27pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd28pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd29pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd30pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd31pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd32pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd33pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd34pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd35pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd36pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd37pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd38pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd39pt7b.h>
#include <Fonts/GFXFF/font_mod/lange/timesbd40pt7b.h>

namespace TimesFF {
  inline uint8_t clampPx(uint8_t px) {
    if (px < 5) return 5;
    if (px > 40) return 40;
    return px;
  }

  inline const GFXfont* regular(uint8_t px) {
    switch (clampPx(px)) {
      case 5:  return &times5pt7b;
      case 6:  return &times6pt7b;
      case 7:  return &times7pt7b;
      case 8:  return &times8pt7b;
      case 9:  return &times9pt7b;
      case 10: return &times10pt7b;
      case 11: return &times11pt7b;
      case 12: return &times12pt7b;
      case 13: return &times13pt7b;
      case 14: return &times14pt7b;
      case 15: return &times15pt7b;
      case 16: return &times16pt7b;
      case 17: return &times17pt7b;
      case 18: return &times18pt7b;
      case 19: return &times19pt7b;
      case 20: return &times20pt7b;
      case 21: return &times21pt7b;
      case 22: return &times22pt7b;
      case 23: return &times23pt7b;
      case 24: return &times24pt7b;
      case 25: return &times25pt7b;
      case 26: return &times26pt7b;
      case 27: return &times27pt7b;
      case 28: return &times28pt7b;
      case 29: return &times29pt7b;
      case 30: return &times30pt7b;
      case 31: return &times31pt7b;
      case 32: return &times32pt7b;
      case 33: return &times33pt7b;
      case 34: return &times34pt7b;
      case 35: return &times35pt7b;
      case 36: return &times36pt7b;
      case 37: return &times37pt7b;
      case 38: return &times38pt7b;
      case 39: return &times39pt7b;
      case 40: return &times40pt7b;
      default: return &times40pt7b;
    }
  }

  inline const GFXfont* bold(uint8_t px) {
    switch (clampPx(px)) {
      case 5:  return &timesbd5pt7b;
      case 6:  return &timesbd6pt7b;
      case 7:  return &timesbd7pt7b;
      case 8:  return &timesbd8pt7b;
      case 9:  return &timesbd9pt7b;
      case 10: return &timesbd10pt7b;
      case 11: return &timesbd11pt7b;
      case 12: return &timesbd12pt7b;
      case 13: return &timesbd13pt7b;
      case 14: return &timesbd14pt7b;
      case 15: return &timesbd15pt7b;
      case 16: return &timesbd16pt7b;
      case 17: return &timesbd17pt7b;
      case 18: return &timesbd18pt7b;
      case 19: return &timesbd19pt7b;
      case 20: return &timesbd20pt7b;
      case 21: return &timesbd21pt7b;
      case 22: return &timesbd22pt7b;
      case 23: return &timesbd23pt7b;
      case 24: return &timesbd24pt7b;
      case 25: return &timesbd25pt7b;
      case 26: return &timesbd26pt7b;
      case 27: return &timesbd27pt7b;
      case 28: return &timesbd28pt7b;
      case 29: return &timesbd29pt7b; 
      case 30: return &timesbd30pt7b;
      case 31: return &timesbd31pt7b;
      case 32: return &timesbd32pt7b;
      case 33: return &timesbd33pt7b;
      case 34: return &timesbd34pt7b;
      case 35: return &timesbd35pt7b;
      case 36: return &timesbd36pt7b;
      case 37: return &timesbd37pt7b;
      case 38: return &timesbd38pt7b;
      case 39: return &timesbd39pt7b;
      case 40: return &timesbd40pt7b;
      default: return &timesbd40pt7b;
    }
  }
}
