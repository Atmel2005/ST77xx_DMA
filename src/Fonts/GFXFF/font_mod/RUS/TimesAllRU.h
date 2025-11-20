#pragma once
#include <ST77xxDMA.h>

// Агрегатор кириллических шрифтов Times (Regular/Bold) в формате GFXFF.
// Диапазон размеров: 5..40 pt (вызывается как px; здесь соответствует point размеру шрифта в файлах GFXFF).
// Автоматически сгенерированные include-файлы для всех размеров Regular и Bold.

// Regular Times RU
#include "regular/tnr_ru_5pt7b.h"
#include "regular/tnr_ru_6pt7b.h"
#include "regular/tnr_ru_7pt7b.h"
#include "regular/tnr_ru_8pt7b.h"
#include "regular/tnr_ru_9pt7b.h"
#include "regular/tnr_ru_10pt7b.h"
#include "regular/tnr_ru_11pt7b.h"
#include "regular/tnr_ru_12pt7b.h"
#include "regular/tnr_ru_13pt7b.h"
#include "regular/tnr_ru_14pt7b.h"
#include "regular/tnr_ru_15pt7b.h"
#include "regular/tnr_ru_16pt7b.h"
#include "regular/tnr_ru_17pt7b.h"
#include "regular/tnr_ru_18pt7b.h"
#include "regular/tnr_ru_19pt7b.h"
#include "regular/tnr_ru_20pt7b.h"
#include "regular/tnr_ru_21pt7b.h"
#include "regular/tnr_ru_22pt7b.h"
#include "regular/tnr_ru_23pt7b.h"
#include "regular/tnr_ru_24pt7b.h"
#include "regular/tnr_ru_25pt7b.h"
#include "regular/tnr_ru_26pt7b.h"
#include "regular/tnr_ru_27pt7b.h"
#include "regular/tnr_ru_28pt7b.h"
#include "regular/tnr_ru_29pt7b.h"
#include "regular/tnr_ru_30pt7b.h"
#include "regular/tnr_ru_31pt7b.h"
#include "regular/tnr_ru_32pt7b.h"
#include "regular/tnr_ru_33pt7b.h"
#include "regular/tnr_ru_34pt7b.h"
#include "regular/tnr_ru_35pt7b.h"
#include "regular/tnr_ru_36pt7b.h"
#include "regular/tnr_ru_37pt7b.h"
#include "regular/tnr_ru_38pt7b.h"
#include "regular/tnr_ru_39pt7b.h"
#include "regular/tnr_ru_40pt7b.h"

// Bold Times RU
#include "bold/tnr_rubd_5pt7b.h"
#include "bold/tnr_rubd_6pt7b.h"
#include "bold/tnr_rubd_7pt7b.h"
#include "bold/tnr_rubd_8pt7b.h"
#include "bold/tnr_rubd_9pt7b.h"
#include "bold/tnr_rubd_10pt7b.h"
#include "bold/tnr_rubd_11pt7b.h"
#include "bold/tnr_rubd_12pt7b.h"
#include "bold/tnr_rubd_13pt7b.h"
#include "bold/tnr_rubd_14pt7b.h"
#include "bold/tnr_rubd_15pt7b.h"
#include "bold/tnr_rubd_16pt7b.h"
#include "bold/tnr_rubd_17pt7b.h"
#include "bold/tnr_rubd_18pt7b.h"
#include "bold/tnr_rubd_19pt7b.h"
#include "bold/tnr_rubd_20pt7b.h"
#include "bold/tnr_rubd_21pt7b.h"
#include "bold/tnr_rubd_22pt7b.h"
#include "bold/tnr_rubd_23pt7b.h"
#include "bold/tnr_rubd_24pt7b.h"
#include "bold/tnr_rubd_25pt7b.h"
#include "bold/tnr_rubd_26pt7b.h"
#include "bold/tnr_rubd_27pt7b.h"
#include "bold/tnr_rubd_28pt7b.h"
#include "bold/tnr_rubd_29pt7b.h"
#include "bold/tnr_rubd_30pt7b.h"
#include "bold/tnr_rubd_31pt7b.h"
#include "bold/tnr_rubd_32pt7b.h"
#include "bold/tnr_rubd_33pt7b.h"
#include "bold/tnr_rubd_34pt7b.h"
#include "bold/tnr_rubd_35pt7b.h"
#include "bold/tnr_rubd_36pt7b.h"
#include "bold/tnr_rubd_37pt7b.h"
#include "bold/tnr_rubd_38pt7b.h"
#include "bold/tnr_rubd_39pt7b.h"
#include "bold/tnr_rubd_40pt7b.h"

namespace TimesRUFF {
  inline uint8_t clampPx(uint8_t px) {
    if (px < 5) return 5;
    if (px > 40) return 40;
    return px;
  }

  // Возвращает указатель на GFXfont для Times Regular RU требуемого размера, либо nullptr если недоступно
  inline const GFXfont* regular(uint8_t px) {
    px = clampPx(px);
    switch (px) {
      case 5:  return &times5pt8b;
      case 6:  return &times6pt8b;
      case 7:  return &times7pt8b;
      case 8:  return &times8pt8b;
      case 9:  return &times9pt8b;
      case 10: return &times10pt8b;
      case 11: return &times11pt8b;
      case 12: return &times12pt8b;
      case 13: return &times13pt8b;
      case 14: return &times14pt8b;
      case 15: return &times15pt8b;
      case 16: return &times16pt8b;
      case 17: return &times17pt8b;
      case 18: return &times18pt8b;
      case 19: return &times19pt8b;
      case 20: return &times20pt8b;
      case 21: return &times21pt8b;
      case 22: return &times22pt8b;
      case 23: return &times23pt8b;
      case 24: return &times24pt8b;
      case 25: return &times25pt8b;
      case 26: return &times26pt8b;
      case 27: return &times27pt8b;
      case 28: return &times28pt8b;
      case 29: return &times29pt8b;
      case 30: return &times30pt8b;
      case 31: return &times31pt8b;
      case 32: return &times32pt8b;
      case 33: return &times33pt8b;
      case 34: return &times34pt8b;
      case 35: return &times35pt8b;
      case 36: return &times36pt8b;
      case 37: return &times37pt8b;
      case 38: return &times38pt8b;
      case 39: return &times39pt8b;
      case 40: return &times40pt8b;
      default: return nullptr;
    }
  }

  // Возвращает указатель на GFXfont для Times Bold RU требуемого размера, либо nullptr если недоступно
  inline const GFXfont* bold(uint8_t px) {
    px = clampPx(px);
    switch (px) {
      case 5:  return &timesbd5pt8b;
      case 6:  return &timesbd6pt8b;
      case 7:  return &timesbd7pt8b;
      case 8:  return &timesbd8pt8b;
      case 9:  return &timesbd9pt8b;
      case 10: return &timesbd10pt8b;
      case 11: return &timesbd11pt8b;
      case 12: return &timesbd12pt8b;
      case 13: return &timesbd13pt8b;
      case 14: return &timesbd14pt8b;
      case 15: return &timesbd15pt8b;
      case 16: return &timesbd16pt8b;
      case 17: return &timesbd17pt8b;
      case 18: return &timesbd18pt8b;
      case 19: return &timesbd19pt8b;
      case 20: return &timesbd20pt8b;
      case 21: return &timesbd21pt8b;
      case 22: return &timesbd22pt8b;
      case 23: return &timesbd23pt8b;
      case 24: return &timesbd24pt8b;
      case 25: return &timesbd25pt8b;
      case 26: return &timesbd26pt8b;
      case 27: return &timesbd27pt8b;
      case 28: return &timesbd28pt8b;
      case 29: return &timesbd29pt8b;
      case 30: return &timesbd30pt8b;
      case 31: return &timesbd31pt8b;
      case 32: return &timesbd32pt8b;
      case 33: return &timesbd33pt8b;
      case 34: return &timesbd34pt8b;
      case 35: return &timesbd35pt8b;
      case 36: return &timesbd36pt8b;
      case 37: return &timesbd37pt8b;
      case 38: return &timesbd38pt8b;
      case 39: return &timesbd39pt8b;
      case 40: return &timesbd40pt8b;
      default: return nullptr;
    }
  }
}

