#include "ST77xxDMA.h"
#include <string.h>

#if defined(ESP8266) || defined(ESP32)
  #include <pgmspace.h>
#elif defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#endif

#ifndef pgm_read_byte
  #define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif
#ifndef pgm_read_word
  #define pgm_read_word(addr) (*(const uint16_t *)(addr))
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <esp32-hal-ledc.h>
#endif

#if ST77XX_HAVE_RP2040_PIO
#include "ST77xxPIO.h"
#endif

#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/gpio.h>
#include <hardware/structs/sio.h>
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include "soc/gpio_struct.h"
#include "soc/gpio_periph.h"
#endif

// Hint hot paths to IRAM on ESP32 to avoid flash stalls
#if !defined(ATTR_IRAM)
#  if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#    define ATTR_IRAM IRAM_ATTR
#  else
#    define ATTR_IRAM
#  endif
#endif

namespace {
  static ST77xxDMA* g_default_display = nullptr;

  // --- Software SPI helpers (bit-bang) ---
  // Centralize GPIO toggling so we can optimize later (e.g., direct port writes)
  inline void ATTR_IRAM sw_write_byte(int sclk, int mosi, uint8_t b, bool use_asm, bool direct_gpio) {
    // Runtime fallback: if user disabled direct GPIO, use portable digitalWrite path
    if (!direct_gpio) {
      for (int8_t i = 7; i >= 0; --i) {
        digitalWrite(sclk, LOW);
        digitalWrite(mosi, (b >> i) & 1);
        digitalWrite(sclk, HIGH);
      }
      return;
    }
#if defined(ARDUINO_ARCH_RP2040)
    // RP2040: use SIO atomic set/clear registers
    const uint32_t s_mask = (1u << (uint32_t)sclk);
    const uint32_t m_mask = (1u << (uint32_t)mosi);
    if (use_asm) {
      // Real inline-ASM path for RP2040 (ARMv6-M, Thumb). Writes directly to SIO set/clear.
      volatile uint32_t* const clr = &sio_hw->gpio_clr;
      volatile uint32_t* const set = &sio_hw->gpio_set;
      // Bit 7
      if (b & (1u << 7)) {
        asm volatile(
          "str %2, [%0]\n\t"   // SCLK LOW   -> *clr = s_mask
          "str %3, [%1]\n\t"   // MOSI HIGH  -> *set = m_mask
          "str %2, [%1]\n\t"   // SCLK HIGH  -> *set = s_mask
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"   // SCLK LOW
          "str %3, [%0]\n\t"   // MOSI LOW   -> *clr = m_mask
          "str %2, [%1]\n\t"   // SCLK HIGH
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 6
      if (b & (1u << 6)) {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%1]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%0]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 5
      if (b & (1u << 5)) {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%1]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%0]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 4
      if (b & (1u << 4)) {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%1]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%0]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 3
      if (b & (1u << 3)) {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%1]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%0]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 2
      if (b & (1u << 2)) {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%1]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%0]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 1
      if (b & (1u << 1)) {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%1]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%0]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 0
      if (b & (1u << 0)) {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%1]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "str %2, [%0]\n\t"
          "str %3, [%0]\n\t"
          "str %2, [%1]\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      return;
    }
    for (int8_t i = 7; i >= 0; --i) {
      sio_hw->gpio_clr = s_mask;                 // SCLK LOW
      if (b & (1u << i)) sio_hw->gpio_set = m_mask; else sio_hw->gpio_clr = m_mask; // MOSI
      sio_hw->gpio_set = s_mask;                 // SCLK HIGH (latch)
    }
#elif defined(__AVR__)
    // AVR: direct port writes
    volatile uint8_t* s_port = portOutputRegister(digitalPinToPort(sclk));
    const uint8_t     s_bit  = digitalPinToBitMask(sclk);
    volatile uint8_t* m_port = portOutputRegister(digitalPinToPort(mosi));
    const uint8_t     m_bit  = digitalPinToBitMask(mosi);
    if (use_asm) {
      // Tuned/unrolled sequence as the "ASM" variant on AVR
      auto emit_bit = [&](int i){
        *s_port &= (uint8_t)~s_bit;                                  // SCLK LOW
        if (b & (1u << i)) *m_port |= m_bit; else *m_port &= (uint8_t)~m_bit; // MOSI
        *s_port |= s_bit;                                            // SCLK HIGH
        asm volatile("nop;");                                       // small hold after clock high
        asm volatile("nop;");
      };
      emit_bit(7); emit_bit(6); emit_bit(5); emit_bit(4); emit_bit(3); emit_bit(2); emit_bit(1); emit_bit(0);
      return;
    }
    for (int8_t i = 7; i >= 0; --i) {
      *s_port &= (uint8_t)~s_bit;                // SCLK LOW
      if (b & (1u << i)) *m_port |= m_bit; else *m_port &= (uint8_t)~m_bit; // MOSI
      *s_port |= s_bit;                          // SCLK HIGH
    }
#elif defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    // ESP32/S2/S3/C3: use fast GPIO on Xtensa; on ESP32-C3 (RISC-V) use direct GPIO registers (no ASM)
    #if defined(__XTENSA__)
      auto gpio_clr = [](int pin){
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
        if (pin < 32) GPIO.out_w1tc = (1UL << (uint32_t)pin); else GPIO.out1_w1tc.val = (1UL << (uint32_t)(pin - 32));
#else
        GPIO.out_w1tc = (1UL << (uint32_t)pin);
#endif
      };
      auto gpio_set = [](int pin){
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
        if (pin < 32) GPIO.out_w1ts = (1UL << (uint32_t)pin); else GPIO.out1_w1ts.val = (1UL << (uint32_t)(pin - 32));
#else
        GPIO.out_w1ts = (1UL << (uint32_t)pin);
#endif
      };
    #else
      // RISC-V targets (e.g., ESP32-C3) have only one GPIO port (<32 pins). Use .val fields.
      auto gpio_clr = [](int pin){
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
        if (pin < 32) GPIO.out_w1tc.val = (1UL << (uint32_t)pin); else GPIO.out1_w1tc.val = (1UL << (uint32_t)(pin - 32));
#else
        GPIO.out_w1tc.val = (1UL << (uint32_t)pin);
#endif
      };
      auto gpio_set = [](int pin){
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
        if (pin < 32) GPIO.out_w1ts.val = (1UL << (uint32_t)pin); else GPIO.out1_w1ts.val = (1UL << (uint32_t)(pin - 32));
#else
        GPIO.out_w1ts.val = (1UL << (uint32_t)pin);
#endif
      };
    #endif
    if (use_asm) {
      // True inline-ASM for ESP32 Xtensa (ESP32/ESP32S2/ESP32S3) and RISC-V (ESP32-C3).
      #if defined(__XTENSA__)
        volatile uint32_t* sclk_clr = (sclk < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val;
        volatile uint32_t* sclk_set = (sclk < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val;
        volatile uint32_t* mosi_clr = (mosi < 32) ? &GPIO.out_w1tc : &GPIO.out1_w1tc.val;
        volatile uint32_t* mosi_set = (mosi < 32) ? &GPIO.out_w1ts : &GPIO.out1_w1ts.val;
        const uint32_t sclk_mask = (sclk < 32) ? (1UL << (uint32_t)sclk) : (1UL << (uint32_t)(sclk - 32));
        const uint32_t mosi_mask = (mosi < 32) ? (1UL << (uint32_t)mosi) : (1UL << (uint32_t)(mosi - 32));
        // Unroll 8 bits, each as a single asm block of 3 stores; no barriers or extra nops
        if (b & (1u<<7)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"  // SCLK LOW
            "s32i %3, %1, 0\n\t"  // MOSI HIGH
            "s32i %2, %4, 0\n\t"  // SCLK HIGH
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"  // SCLK LOW
            "s32i %3, %1, 0\n\t"  // MOSI LOW
            "s32i %2, %4, 0\n\t"  // SCLK HIGH
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<6)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<5)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<4)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<3)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<2)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<1)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<0)) {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "s32i %2, %0, 0\n\t"
            "s32i %3, %1, 0\n\t"
            "s32i %2, %4, 0\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        return;
      #elif defined(__riscv)
        // RISC-V (ESP32-C3): inline ASM using 'sw' to W1TC/W1TS registers
        volatile uint32_t* sclk_clr;
        volatile uint32_t* sclk_set;
        volatile uint32_t* mosi_clr;
        volatile uint32_t* mosi_set;
        const uint32_t sclk_mask =
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
          (sclk < 32) ? (1UL << (uint32_t)sclk) : (1UL << (uint32_t)(sclk - 32));
#else
          (1UL << (uint32_t)sclk);
#endif
        const uint32_t mosi_mask =
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
          (mosi < 32) ? (1UL << (uint32_t)mosi) : (1UL << (uint32_t)(mosi - 32));
#else
          (1UL << (uint32_t)mosi);
#endif

#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
        sclk_clr = (sclk < 32) ? &GPIO.out_w1tc.val : &GPIO.out1_w1tc.val;
        sclk_set = (sclk < 32) ? &GPIO.out_w1ts.val : &GPIO.out1_w1ts.val;
        mosi_clr = (mosi < 32) ? &GPIO.out_w1tc.val : &GPIO.out1_w1tc.val;
        mosi_set = (mosi < 32) ? &GPIO.out_w1ts.val : &GPIO.out1_w1ts.val;
#else
        sclk_clr = &GPIO.out_w1tc.val;
        sclk_set = &GPIO.out_w1ts.val;
        mosi_clr = &GPIO.out_w1tc.val;
        mosi_set = &GPIO.out_w1ts.val;
#endif
        // Unroll 8 bits, each as three stores: SCLK LOW -> MOSI -> SCLK HIGH
        if (b & (1u<<7)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<6)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<5)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<4)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<3)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<2)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<1)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        if (b & (1u<<0)) {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_set), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        } else {
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(mosi_clr), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        return;
      #else
        // Other arches: use the unrolled C path
        auto emit_bit = [&](int i){
          gpio_clr(sclk);
          if (b & (1u << i)) gpio_set(mosi); else gpio_clr(mosi);
          gpio_set(sclk);
        };
        emit_bit(7); emit_bit(6); emit_bit(5); emit_bit(4); emit_bit(3); emit_bit(2); emit_bit(1); emit_bit(0);
        return;
      #endif
    }
    for (int8_t i = 7; i >= 0; --i) {
      gpio_clr(sclk);                              // SCLK LOW
      if (b & (1u << i)) gpio_set(mosi); else gpio_clr(mosi); // MOSI
      gpio_set(sclk);                              // SCLK HIGH
    }
#elif defined(ESP8266) && defined(GPOS) && defined(GPOC)
    // ESP8266: fast GPIO set/clear registers
    const uint32_t s_mask = (1UL << (uint32_t)sclk);
    const uint32_t m_mask = (1UL << (uint32_t)mosi);
    if (use_asm) {
      // True inline-ASM path for ESP8266 (Xtensa LX106). Use direct stores to GPOS/GPOC.
      volatile uint32_t* const set = &GPOS; // write-1-to-set
      volatile uint32_t* const clr = &GPOC; // write-1-to-clear
      // Bit 7
      if (b & (1u << 7)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"  // SCLK LOW   -> *clr = s_mask
          "s32i %3, %1, 0\n\t"  // MOSI HIGH  -> *set = m_mask
          "s32i %2, %1, 0\n\t"  // SCLK HIGH  -> *set = s_mask
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"  // SCLK LOW
          "s32i %3, %0, 0\n\t"  // MOSI LOW   -> *clr = m_mask
          "s32i %2, %1, 0\n\t"  // SCLK HIGH
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 6
      if (b & (1u << 6)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %1, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %0, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 5
      if (b & (1u << 5)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %1, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %0, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 4
      if (b & (1u << 4)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %1, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %0, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 3
      if (b & (1u << 3)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %1, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %0, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 2
      if (b & (1u << 2)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %1, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %0, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 1
      if (b & (1u << 1)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %1, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %0, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      // Bit 0
      if (b & (1u << 0)) {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %1, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      } else {
        asm volatile(
          "s32i %2, %0, 0\n\t"
          "s32i %3, %0, 0\n\t"
          "s32i %2, %1, 0\n\t"
          "nop\n\t"
          "nop\n\t"
          : : "r"(clr), "r"(set), "r"(s_mask), "r"(m_mask) : "memory");
      }
      return;
    }
    for (int8_t i = 7; i >= 0; --i) {
      GPOC = s_mask;                              // SCLK LOW
      if (b & (1u << i)) GPOS = m_mask; else GPOC = m_mask; // MOSI
      GPOS = s_mask;                              // SCLK HIGH
    }
#else
    // Fallback: standard digitalWrite (portable, slower)
    for (int8_t i = 7; i >= 0; --i) {
      digitalWrite(sclk, LOW);
      digitalWrite(mosi, (b >> i) & 1);
      digitalWrite(sclk, HIGH);
    }
#endif
  }

  inline void ATTR_IRAM sw_write_block(int sclk, int mosi, const uint8_t* p, size_t n, bool use_asm, bool direct_gpio) {
    if (!direct_gpio) {
      for (size_t idx = 0; idx < n; ++idx) {
        sw_write_byte(sclk, mosi, p[idx], use_asm, direct_gpio);
#if defined(ESP8266)
        if ((idx & 0x3Fu) == 0) { yield(); }
#endif
      }
      return;
    }

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    // Fast bulk path for ESP32 family
    #if defined(__riscv)
      if (use_asm) {
        // Precompute register pointers and bit masks once
        volatile uint32_t* sclk_clr;
        volatile uint32_t* sclk_set;
        volatile uint32_t* mosi_clr;
        volatile uint32_t* mosi_set;
        const uint32_t sclk_mask =
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
          (sclk < 32) ? (1UL << (uint32_t)sclk) : (1UL << (uint32_t)(sclk - 32));
#else
          (1UL << (uint32_t)sclk);
#endif
        const uint32_t mosi_mask =
#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
          (mosi < 32) ? (1UL << (uint32_t)mosi) : (1UL << (uint32_t)(mosi - 32));
#else
          (1UL << (uint32_t)mosi);
#endif

#if (defined(SOC_GPIO_PIN_COUNT) && (SOC_GPIO_PIN_COUNT > 32))
        sclk_clr = (sclk < 32) ? &GPIO.out_w1tc.val : &GPIO.out1_w1tc.val;
        sclk_set = (sclk < 32) ? &GPIO.out_w1ts.val : &GPIO.out1_w1ts.val;
        mosi_clr = (mosi < 32) ? &GPIO.out_w1tc.val : &GPIO.out1_w1tc.val;
        mosi_set = (mosi < 32) ? &GPIO.out_w1ts.val : &GPIO.out1_w1ts.val;
#else
        sclk_clr = &GPIO.out_w1tc.val;
        sclk_set = &GPIO.out_w1ts.val;
        mosi_clr = &GPIO.out_w1tc.val;
        mosi_set = &GPIO.out_w1ts.val;
#endif
        for (size_t idx = 0; idx < n; ++idx) {
          uint8_t b = p[idx];
          // Bit 7..0
          volatile uint32_t* pm;
          pm = (b & (1u<<7)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
          pm = (b & (1u<<6)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
          pm = (b & (1u<<5)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
          pm = (b & (1u<<4)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
          pm = (b & (1u<<3)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
          pm = (b & (1u<<2)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
          pm = (b & (1u<<1)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
          pm = (b & (1u<<0)) ? mosi_set : mosi_clr;
          asm volatile(
            "sw %2, 0(%0)\n\t"
            "sw %3, 0(%1)\n\t"
            "sw %2, 0(%4)\n\t"
            : : "r"(sclk_clr), "r"(pm), "r"(sclk_mask), "r"(mosi_mask), "r"(sclk_set) : "memory");
        }
        return;
      }
    #endif
#endif

    // Default: reuse byte writer
    for (size_t idx = 0; idx < n; ++idx) {
      sw_write_byte(sclk, mosi, p[idx], use_asm, direct_gpio);
#if defined(ESP8266)
      if ((idx & 0x3Fu) == 0) { yield(); }
#endif
    }
  }
}
namespace ST77xx {
  void setDefault(ST77xxDMA* d) { g_default_display = d; }
  ST77xxDMA* defaultDisplay() { return g_default_display; }
}
// ------------ Ctor/Dtor ------------
ST77xxDMA::ST77xxDMA(const ST77xxConfig& cfg) : _cfg(cfg) {
  _lcdW = cfg.width;
  _lcdH = cfg.height;
  _colstart = cfg.colstart;
  _rowstart = cfg.rowstart;
  _bgr = cfg.bgr;
  _rotation = (cfg.rotation & 3);
  // Backend selection
  _rp2040_use_pio = cfg.rp2040_use_pio;
  // Runtime debug
  _debug = cfg.debug;
  // NV3007 defaults: most glasses require COLMOD=0x55 (16bpp standard) and RAMWRC disabled.
  // Make these defaults automatic when panel/controller is NV3007 so user code doesn't need to set them.
  if (_cfg.panel == ST77xxPanel::NV3007_428x142 || _cfg.controller == ST77xxController::NV3007) {
    _nv3007_colmod_55 = true;   // use 0x55 by default
    _nv3007_use_ramwrc = false; // keep RAMWR (0x2C) by default
  }
}

#if defined(__cpp_char8_t)
void ST77xxDMA::drawTextUTF8(int16_t x, int16_t y, const char8_t* utf8, uint16_t color, uint16_t bg, uint8_t size) {
  // Forward by reinterpret_cast to const char*; UTF-8 byte layout is identical
  drawTextUTF8(x, y, reinterpret_cast<const char*>(utf8), color, bg, size);
}
#endif

// Helper: choose font (primary/fallback) and mapped codepoint (original or '?')
static bool pickFontForGlyph(const GFXfont* primary, const GFXfont* fallback,
                             uint32_t cp, const GFXfont*& chosen, uint32_t& mapped) {
  chosen = nullptr; mapped = cp;
  if (primary) {
    uint16_t pf = pgm_read_word(&primary->first);
    uint16_t pl = pgm_read_word(&primary->last);
    if (cp >= pf && cp <= pl) { chosen = primary; mapped = cp; return true; }
  }
  if (fallback) {
    uint16_t ff = pgm_read_word(&fallback->first);
    uint16_t fl = pgm_read_word(&fallback->last);
    if (cp >= ff && cp <= fl) { chosen = fallback; mapped = cp; return true; }
  }
  // Try '?'
  uint32_t q = (uint32_t)'?';
  if (primary) {
    uint16_t pf = pgm_read_word(&primary->first);
    uint16_t pl = pgm_read_word(&primary->last);
    if (q >= pf && q <= pl) { chosen = primary; mapped = q; return true; }
  }
  if (fallback) {
    uint16_t ff = pgm_read_word(&fallback->first);
    uint16_t fl = pgm_read_word(&fallback->last);
    if (q >= ff && q <= fl) { chosen = fallback; mapped = q; return true; }
  }
  return false;
}

void ST77xxDMA::drawGlyphGFX(int16_t x, int16_t y, const GFXfont* f, uint32_t cp, uint16_t color, uint16_t bg, uint8_t size) {
  (void)bg; // Background fill is typically not applied for GFX fonts
  if (size == 0 || !f) return;
  uint16_t first = pgm_read_word(&f->first);
  uint16_t last  = pgm_read_word(&f->last);
  if (cp < first || cp > last) {
    // Fallback to '?'
    cp = (uint32_t)'?';
    if (cp < first || cp > last) return;
  }
  uint16_t gi = (uint16_t)(cp - first);
  const GFXglyph* glyphBase = (const GFXglyph*)pgm_read_ptr(&f->glyph);
  const GFXglyph* gptr = &glyphBase[gi];
  uint16_t bo = pgm_read_word(&gptr->bitmapOffset);
  uint8_t  w  = pgm_read_byte(&gptr->width);
  uint8_t  h  = pgm_read_byte(&gptr->height);
  int8_t   xo = (int8_t)pgm_read_byte(&gptr->xOffset);
  int8_t   yo = (int8_t)pgm_read_byte(&gptr->yOffset);

  if (w == 0 || h == 0) return; // nothing to draw

  int16_t xs = x + (int16_t)xo * size;
  int16_t ys = y + (int16_t)yo * size; // y is baseline

  const uint8_t* bitmap = (const uint8_t*)pgm_read_ptr(&f->bitmap);
  uint8_t bits = 0, bitMask = 0;
  for (uint8_t yy = 0; yy < h; ++yy) {
    for (uint8_t xx = 0; xx < w; ++xx) {
      if (bitMask == 0) { bits = pgm_read_byte(bitmap + bo++); bitMask = 0x80; }
      bool on = (bits & bitMask) != 0;
      if (on) {
        if (size == 1) drawPixel(xs + xx, ys + yy, color);
        else fillRect(xs + (int16_t)xx * size, ys + (int16_t)yy * size, size, size, color);
      }
      bitMask >>= 1;
    }
  }
}

void ST77xxDMA::drawChar(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
  const GFXfont* useF = nullptr; uint32_t mapped = 0;
  if (!pickFontForGlyph(_gfxFont, _gfxFontFallback, (uint8_t)c, useF, mapped)) return;
  drawGlyphGFX(x, y, useF, mapped, color, bg, size ? size : 1);
}

void ST77xxDMA::drawText(int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size) {
  // Treat input as UTF-8 and delegate to the UTF-8 path for consistent behavior
  drawTextUTF8(x, y, str, color, bg, size);
}

void ST77xxDMA::setGFXFont(const GFXfont* font) {
  _gfxFont = font;
}

void ST77xxDMA::setGFXFontFallback(const GFXfont* font) {
  _gfxFontFallback = font;
}

void ST77xxDMA::setFonts(const GFXfont* primary, const GFXfont* fallback) {
  _gfxFont = primary; _gfxFontFallback = fallback;
}

void ST77xxDMA::drawTextUTF8(int16_t x, int16_t y, const char* utf8, uint16_t color, uint16_t bg, uint8_t size) {
  if (!utf8) return;
  if (!_gfxFont && !_gfxFontFallback) return; // Need at least one font
  int16_t cx = x, cy = y;
  uint8_t state = 0; // remaining continuation bytes
  uint32_t cp = 0;

  auto emit = [&](uint32_t code){
    // Map Numero sign U+2116 to ASCII 'N' as requested
    if (code == 0x2116u) code = (uint32_t)'N';
    if (code == '\r') return;
    if (code == '\n') {
      const GFXfont* lf = _gfxFont ? _gfxFont : _gfxFontFallback;
      uint8_t lh = lf ? pgm_read_byte(&lf->yAdvance) : 0;
      cx = x; cy += (int16_t)size * lh; return;
    }
    const GFXfont* gf = nullptr; uint32_t code2 = 0;
    if (!pickFontForGlyph(_gfxFont, _gfxFontFallback, code, gf, code2)) return;
    uint16_t gi = (uint16_t)(code2 - pgm_read_word(&gf->first));
    const GFXglyph* glyphBase = (const GFXglyph*)pgm_read_ptr(&gf->glyph);
    const GFXglyph* gptr = &glyphBase[gi];
    uint8_t xAdv = pgm_read_byte(&gptr->xAdvance);
    // compute advance including configurable letter spacing (can be negative)
    int16_t adv = (int16_t)size * ((int16_t)xAdv + (int16_t)_letterSpacing);
    // wrap
    if (_textWrap && (cx + adv > (int16_t)_lcdW)) {
      cx = x; cy += (int16_t)size * pgm_read_byte(&gf->yAdvance);
    }
    drawGlyphGFX(cx, cy, gf, code2, color, bg, size);
    cx += adv;
  };

  while (*utf8) {
    uint8_t b = (uint8_t)*utf8++;
    if (state == 0) {
      if (b < 0x80) {
        emit(b);
      } else if ((b & 0xE0) == 0xC0) {
        cp = b & 0x1F; state = 1;
      } else if ((b & 0xF0) == 0xE0) {
        cp = b & 0x0F; state = 2;
      } else if ((b & 0xF8) == 0xF0) {
        cp = b & 0x07; state = 3;
      } else {
        emit('?');
      }
    } else {
      if ((b & 0xC0) != 0x80) {
        state = 0; emit('?');
      } else {
        cp = (cp << 6) | (b & 0x3F);
        if (--state == 0) emit(cp);
      }
    }
  }
}

// --- Print/UTF-8 streaming ---
size_t ST77xxDMA::write(uint8_t b) {
  auto emitCP = [&](uint32_t code){
    // Map Numero sign U+2116 to ASCII 'N' as requested
    if (code == 0x2116u) code = (uint32_t)'N';
    if (code == '\r') return (size_t)1;
    if (code == '\n') {
      _cursorX = 0;
      const GFXfont* lf = _gfxFont ? _gfxFont : _gfxFontFallback;
      if (lf) { _cursorY += (int16_t)_textSize * pgm_read_byte(&lf->yAdvance); }
      return (size_t)1;
    }
    // Use any available font (primary or fallback). If none, just consume.
    const GFXfont* gf = nullptr; uint32_t mapped = 0;
    if (!pickFontForGlyph(_gfxFont, _gfxFontFallback, code, gf, mapped)) return (size_t)1;
    uint16_t first = pgm_read_word(&gf->first);
    uint16_t gi = (uint16_t)(mapped - first);
    const GFXglyph* glyphBase = (const GFXglyph*)pgm_read_ptr(&gf->glyph);
    const GFXglyph* gptr = &glyphBase[gi];
    uint8_t xAdv = pgm_read_byte(&gptr->xAdvance);
    int16_t adv = (int16_t)_textSize * ((int16_t)xAdv + (int16_t)_letterSpacing);
    if (_textWrap && (_cursorX + adv > (int16_t)_lcdW)) {
      _cursorX = 0; _cursorY += (int16_t)_textSize * pgm_read_byte(&gf->yAdvance);
    }
    drawGlyphGFX(_cursorX, _cursorY, gf, mapped, _textColor, _textBg, _textSize);
    _cursorX += adv;
    return (size_t)1;
  };

  if (_utf8_state == 0) {
    if (b < 0x80) return emitCP(b);
    if ((b & 0xE0) == 0xC0) { _utf8_cp = b & 0x1F; _utf8_state = 1; return 1; }
    if ((b & 0xF0) == 0xE0) { _utf8_cp = b & 0x0F; _utf8_state = 2; return 1; }
    if ((b & 0xF8) == 0xF0) { _utf8_cp = b & 0x07; _utf8_state = 3; return 1; }
    // invalid starter
    return emitCP('?');
  } else {
    if ((b & 0xC0) != 0x80) { // invalid continuation
      _utf8_state = 0;
      return emitCP('?');
    }
    _utf8_cp = (_utf8_cp << 6) | (b & 0x3F);
    if (--_utf8_state == 0) return emitCP(_utf8_cp);
    return 1;
  }
}

ST77xxDMA::~ST77xxDMA() {
  end();
}

// Commands
#define ST77XX_SWRESET 0x01
#define ST77XX_SLPOUT  0x11
#define ST77XX_NORON   0x13
#define ST77XX_INVOFF  0x20
#define ST77XX_INVON   0x21
#define ST77XX_DISPON  0x29
#define ST77XX_CASET   0x2A
#define ST77XX_RASET   0x2B
#define ST77XX_RAMWR   0x2C
#define ST77XX_MADCTL  0x36
#define ST77XX_COLMOD  0x3A
// MIPI DCS brightness / control (for ST7789 etc.)
#define ST77XX_WRDISBV 0x51 // Write Display Brightness
#define ST77XX_WRCTRLD 0x53 // Write CTRL Display (BCTRL, BL, DD)

// Extended MIPI DCS set used by cross-controller APIs
#define ST77XX_TEOFF    0x34 // Tearing Effect Line OFF
#define ST77XX_TEON     0x35 // Tearing Effect Line ON
#define ST77XX_VSCRDEF  0x33 // Set Vertical Scroll Definition
#define ST77XX_VSCRSADD 0x37 // Set Vertical Scroll Start Address
#define ST77XX_PTLAR    0x30 // Set Partial Area
#define ST77XX_PTLON    0x12 // Partial Mode ON
#define ST77XX_TESCAN   0x44 // Set Tear Scanline

// MADCTL bits
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_BGR 0x08

#if defined(ARDUINO_ARCH_ESP32)
  #include <driver/spi_master.h>
  #include <driver/gpio.h>
  #include <esp_heap_caps.h>
#endif

#if !defined(ARDUINO_ARCH_ESP32)
  #include <SPI.h>
#endif

#if ST77XX_HAVE_RP2040_DMA
  #include <hardware/spi.h>
  #include <hardware/dma.h>
  #include <hardware/clocks.h>
#endif

// ------------ GPIO helpers (inline declared in header) ------------
inline void ST77xxDMA::csLow()  { if (_cfg.cs >= 0) digitalWrite(_cfg.cs, LOW); }
inline void ST77xxDMA::csHigh() { if (_cfg.cs >= 0) digitalWrite(_cfg.cs, HIGH); }
inline void ST77xxDMA::dcCmd()  { if (_cfg.dc >= 0) digitalWrite(_cfg.dc, LOW); }
inline void ST77xxDMA::dcData() { if (_cfg.dc >= 0) digitalWrite(_cfg.dc, HIGH); }

// ------------ Presets ------------
void ST77xxDMA::applyPanelPreset() {
  switch (_cfg.panel) {
    case ST77xxPanel::ST7735_128x160:
      _cfg.controller = ST77xxController::ST7735;
      _lcdW = 128; _lcdH = 160; _colstart = 0; _rowstart = 0; break;
    case ST77xxPanel::ST7789_240x320:
      _cfg.controller = ST77xxController::ST7789;
      _lcdW = 240; _lcdH = 320; _colstart = 0; _rowstart = 0; break;
    case ST77xxPanel::ST7789_172x320:
      _cfg.controller = ST77xxController::ST7789;
      _lcdW = 172; _lcdH = 320; _colstart = 0; _rowstart = 0; _bgr = true; break;
    case ST77xxPanel::ST7789_76x284:
      _cfg.controller = ST77xxController::ST7789;
      _lcdW = 76; _lcdH = 284; _colstart = 0; _rowstart = 0; break;
    case ST77xxPanel::ST7789_170x320:
      _cfg.controller = ST77xxController::ST7789;
      _lcdW = 170; _lcdH = 320; _colstart = 0; _rowstart = 0; break;
    case ST77xxPanel::ST7789_135x240:
      _cfg.controller = ST77xxController::ST7789;
      _lcdW = 135; _lcdH = 240; _colstart = 0; _rowstart = 0; break;
    case ST77xxPanel::ST7789_240x240:
      _cfg.controller = ST77xxController::ST7789;
      _lcdW = 240; _lcdH = 240; _colstart = 0; _rowstart = 0; break;
    case ST77xxPanel::ST7735_80x160:
      _cfg.controller = ST77xxController::ST7735;
      _lcdW = 80; _lcdH = 160; _colstart = 0; _rowstart = 0; _cfg.bgr = true; break;
    case ST77xxPanel::NV3007_428x142:
      _cfg.controller = ST77xxController::NV3007;
      _lcdW = 428; _lcdH = 142; _colstart = 0; _rowstart = 0; _bgr = false; break;
  }
  // allow explicit overrides
  if (_cfg.width)  _lcdW = _cfg.width;
  if (_cfg.height) _lcdH = _cfg.height;
  _colstart = _cfg.colstart;
  _rowstart = _cfg.rowstart;
  _bgr      = _cfg.bgr;
  _rotation = _cfg.rotation & 3;

  // Default mirror for specific panels/orientations to match real-world modules
  // ST7789 240x320: many modules are mounted upside-down relative to rotation=1.
  // To keep example sketches with rotation=1, mirror both axes by default in that case.
  if (_cfg.panel == ST77xxPanel::ST7789_240x320) {
    if ((_rotation & 3) == 1) { _mirrorX = true; _mirrorY = true; }
    else { _mirrorX = false; _mirrorY = false; }
  } else if (_cfg.panel == ST77xxPanel::ST7789_170x320) {
    // Many 1.9" 170x320 modules require horizontal + vertical mirror at rotation=1
    // to avoid upside-down text while keeping left origin.
    if ((_rotation & 3) == 1) { _mirrorX = true; _mirrorY = true; }
    else { _mirrorX = false; _mirrorY = false; }
  } else if (_cfg.panel == ST77xxPanel::ST7789_172x320) {
    // Default: no mirroring for 172x320 modules
    _mirrorX = false; _mirrorY = false;
  } else if (_cfg.panel == ST77xxPanel::ST7789_76x284) {
    // Default: always mirror both axes for 76x284 modules
    _mirrorX = true; _mirrorY = true;
  }
}

// ------------ Backlight ------------
void ST77xxDMA::setupBacklight() {
  if (_cfg.bl < 0) return;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  // Configure LEDC for backlight
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // Core v3: use pin-based attach with explicit channel selection
    ledcAttachChannel((uint8_t)_cfg.bl, (uint32_t)_cfg.bl_freq_hz, 8, (uint8_t)_cfg.bl_ledc_channel);
  #else
    // Older cores: channel-based setup + attach pin
    ledcSetup(_cfg.bl_ledc_channel, _cfg.bl_freq_hz, 8);
    ledcAttachPin(_cfg.bl, _cfg.bl_ledc_channel);
  #endif
#elif defined(ESP8266)
  pinMode(_cfg.bl, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(_cfg.bl_freq_hz);
#elif defined(ARDUINO_ARCH_RP2040)
  pinMode(_cfg.bl, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(_cfg.bl_freq_hz);
#else
  pinMode(_cfg.bl, OUTPUT);
#endif
  // Set initial brightness from config (default 127), respecting inversion
  applyBacklight(_cfg.bl_start);
}

void ST77xxDMA::applyBacklight(uint8_t level) {
  if (_cfg.bl < 0) return;
  uint8_t duty = _cfg.bl_invert ? (uint8_t)(255 - level) : level;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  // ESP32 core v3 uses pin-based ledcWrite(pin, duty). Older cores used channel.
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite((uint8_t)_cfg.bl, duty);
  #else
    ledcWrite(_cfg.bl_ledc_channel, duty);
  #endif
#else
  analogWrite(_cfg.bl, duty);
#endif
}

// ------------ Panel init helpers ------------
// Debug helpers: decode common DCS/command names
static inline const __FlashStringHelper* st77xxCmdName(uint8_t cmd) {
  switch (cmd) {
    case 0x2A: return F("CASET");
    case 0x2B: return F("RASET");
    case 0x2C: return F("RAMWR");
    case 0x3C: return F("RAMWRC");
    case 0x36: return F("MADCTL");
    case 0x3A: return F("COLMOD");
    case 0x11: return F("SLPOUT");
    case 0x29: return F("DISPON");
    case 0x13: return F("NORON");
    case 0x21: return F("INVON");
    case 0x20: return F("INVOFF");
    case 0x35: return F("TEON");
    case 0x34: return F("TEOFF");
    case 0x44: return F("TEAREA");
    case 0xFF: return F("VENDOR_LOCK"); // NV3007 vendor lock/unlock
    default:   return nullptr;
  }
}

void ST77xxDMA::sendCmd(uint8_t cmd) {
  if (_debug && !_dbg_suppress) {
    Serial.print(F("[ST77xx][CMD] 0x"));
    if (cmd < 16) Serial.print('0');
    Serial.println(cmd, HEX);
  }
  dcCmd(); csLow();
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  if (!_cfg.use_hw_spi) {
    // ESP32: Software SPI (bit-bang)
    sw_write_byte(_cfg.sclk, _cfg.mosi, cmd, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
  } else {
    // Guard against invalid SPI device handle
    if (_spi_dev == nullptr) {
      if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 sendCmd: _spi_dev is null; skipping"));
      csHigh();
      return;
    }
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
  }
#else
  if (_cfg.use_hw_spi) {
#if ST77XX_HAVE_RP2040_DMA
    if (_rp2040_spi != nullptr) {
      spi_write_blocking((spi_inst_t*)_rp2040_spi, &cmd, 1);
    } else {
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      SPI.transfer(cmd);
      SPI.endTransaction();
    }
#else
    SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
    SPI.transfer(cmd);
    SPI.endTransaction();
#endif
  } else {
    // SW SPI
    sw_write_byte(_cfg.sclk, _cfg.mosi, cmd, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
  }
#endif
  csHigh();
}

void ST77xxDMA::sendData(const void* data, size_t len) {
  if (!len) return;
  if (_debug && !_dbg_suppress) {
    Serial.print(F("[ST77xx][DAT] len="));
    Serial.print(len);
    if (len <= 32) {
      Serial.print(F(" bytes="));
      const uint8_t* p_dbg = (const uint8_t*)data;
      for (size_t i = 0; i < len; ++i) {
        Serial.print(F("0x"));
        if (p_dbg[i] < 16) Serial.print('0');
        Serial.print(p_dbg[i], HEX);
        if (i + 1 < len) Serial.print(' ');
      }
    } else {
      Serial.print(F(" (omitted)"));
    }
    Serial.println();
  }
  dcData(); csLow();
#if defined(ARDUINO_ARCH_ESP32)
  if (!_cfg.use_hw_spi) {
    // ESP32: Software SPI (bit-bang)
    const uint8_t* p = (const uint8_t*)data;
    sw_write_block(_cfg.sclk, _cfg.mosi, p, len, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
  } else {
    // Guard against invalid SPI device handle
    if (_spi_dev == nullptr) {
      if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 sendData: _spi_dev is null; skipping"));
      csHigh();
      return;
    }
    // Some ESP-IDF targets enforce a maximum transfer size per transaction.
    // Split large payloads into chunks no larger than the bus-configured max.
    const uint8_t* p = (const uint8_t*)data;
    size_t left = len;
    // Use half-frame as a conservative upper bound configured in backendInitSPI().
    size_t max_bytes = _half_bytes ? _half_bytes : (size_t)4096;
    while (left) {
      size_t n = (left > max_bytes) ? max_bytes : left;
      spi_transaction_t t = {};
      t.length = n * 8;     // bits
      t.tx_buffer = p;
      // blocking transmit is fine here; CS is already asserted
      spi_device_transmit((spi_device_handle_t)_spi_dev, &t);
      p += n;
      left -= n;
    }
  }
#else
  if (_cfg.use_hw_spi) {
#if ST77XX_HAVE_RP2040_DMA
    // RP2040: Prefer DMA for larger payloads, even without a framebuffer
    if (_rp2040_dma_ch >= 0 && _rp2040_spi != nullptr) {
      // Small writes are faster via blocking SPI; large writes via DMA
      const size_t DMA_THRESHOLD = 64; // bytes
      if (len >= DMA_THRESHOLD) {
        rp2040DMASend((const uint8_t*)data, len);
      } else {
        // Use the same RP2040 SPI instance as DMA to avoid instance mismatch
        spi_write_blocking((spi_inst_t*)_rp2040_spi, (const uint8_t*)data, len);
      }
    } else {
      // Fallback to blocking SPI if DMA not initialized
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      const uint8_t* p = (const uint8_t*)data;
      for (size_t i=0;i<len;++i) SPI.transfer(p[i]);
      SPI.endTransaction();
    }
#else
    SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
    const uint8_t* p = (const uint8_t*)data;
    for (size_t i=0;i<len;++i) SPI.transfer(p[i]);
    SPI.endTransaction();
#endif
  } else {
    const uint8_t* p = (const uint8_t*)data;
    sw_write_block(_cfg.sclk, _cfg.mosi, p, len, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
  }
#endif
  csHigh();
}

void ST77xxDMA::sendCmdData(uint8_t cmd, const void* data, size_t len) {
  // Duplicate log suppression for CASET/RASET with 4-byte payloads
  bool suppress_log = false;
  bool print_prev_note = false;
  uint32_t prev_repeats = 0;
  const uint8_t* b = (const uint8_t*)data;
  if (b && len == 4) {
    if (cmd == 0x2A) { // CASET
      if (_log_have_caset &&
          _log_last_caset[0]==b[0] && _log_last_caset[1]==b[1] &&
          _log_last_caset[2]==b[2] && _log_last_caset[3]==b[3]) {
        _log_dup_caset++;
        suppress_log = true;
      } else {
        if (_log_have_caset && _log_dup_caset > 0) { print_prev_note = true; prev_repeats = _log_dup_caset; }
        _log_have_caset = true;
        _log_last_caset[0]=b[0]; _log_last_caset[1]=b[1]; _log_last_caset[2]=b[2]; _log_last_caset[3]=b[3];
        _log_dup_caset = 0;
      }
    } else if (cmd == 0x2B) { // RASET
      if (_log_have_raset &&
          _log_last_raset[0]==b[0] && _log_last_raset[1]==b[1] &&
          _log_last_raset[2]==b[2] && _log_last_raset[3]==b[3]) {
        _log_dup_raset++;
        suppress_log = true;
      } else {
        if (_log_have_raset && _log_dup_raset > 0) { print_prev_note = true; prev_repeats = _log_dup_raset; }
        _log_have_raset = true;
        _log_last_raset[0]=b[0]; _log_last_raset[1]=b[1]; _log_last_raset[2]=b[2]; _log_last_raset[3]=b[3];
        _log_dup_raset = 0;
      }
    }
  }

  if (_debug && !suppress_log) {
    // If previous identical values were suppressed, report it briefly now
    if (print_prev_note) {
      Serial.print(F("[ST77xx][DBG] "));
      Serial.print(cmd == 0x2A ? F("CASET") : F("RASET"));
      Serial.print(F(" repeated "));
      Serial.print(prev_repeats);
      Serial.println(F("x (suppressed)"));
    }

    Serial.print(F("[ST77xx][TX] "));
    if (auto n = st77xxCmdName(cmd)) {
      Serial.print(n);
      Serial.print(F(" "));
    }
    Serial.print(F("(0x")); if (cmd < 16) Serial.print('0'); Serial.print(cmd, HEX); Serial.print(F(")"));
    Serial.print(F(" len=")); Serial.print(len);
    // Parse common payloads for readability
    if ((cmd == 0x2A || cmd == 0x2B) && len == 4 && b) {
      uint16_t s = (uint16_t)((b[0] << 8) | b[1]);
      uint16_t e = (uint16_t)((b[2] << 8) | b[3]);
      Serial.print(F(" range=")); Serial.print(s); Serial.print(F("..")); Serial.print(e);
    } else if (cmd == 0x36 && len >= 1 && b) { // MADCTL
      uint8_t v = b[0];
      Serial.print(F(" madctl="));
      if (v & 0x80) Serial.print(F("MY "));
      if (v & 0x40) Serial.print(F("MX "));
      if (v & 0x20) Serial.print(F("MV "));
      if (v & 0x08) Serial.print(F("BGR "));
      Serial.print(F("(0x")); if (v < 16) Serial.print('0'); Serial.print(v, HEX); Serial.print(F(")"));
    } else if (cmd == 0x3A && len >= 1 && b) { // COLMOD
      uint8_t v = b[0];
      Serial.print(F(" colmod=0x")); if (v < 16) Serial.print('0'); Serial.print(v, HEX);
      if (v == 0x55 || v == 0x05) Serial.print(F(" (16bpp)"));
      else if (v == 0x66) Serial.print(F(" (18bpp)"));
    } else if (cmd == 0xFF && len >= 1 && b) { // NV vendor lock
      Serial.print(F(" lock=")); Serial.print(b[0] == 0xA5 ? F("A5 (LOCK)") : F("00 (UNLOCK)"));
    } else if (len && b && len <= 16) {
      Serial.print(F(" data="));
      for (size_t i = 0; i < len; ++i) {
        Serial.print(F("0x")); if (b[i] < 16) Serial.print('0'); Serial.print(b[i], HEX);
        if (i + 1 < len) Serial.print(' ');
      }
    }
    Serial.println();
  }
  // Suppress nested logs from sendCmd()/sendData()
  bool prev = _dbg_suppress; _dbg_suppress = true;
  sendCmd(cmd);
  if (len) sendData(data, len);
  _dbg_suppress = prev;
}

void ST77xxDMA::setDebug(bool enable) {
  _debug = enable;
  if (_debug) {
    Serial.println(F("[ST77xx] Debug: ON"));
  } else {
    Serial.println(F("[ST77xx] Debug: OFF"));
  }
}

void ST77xxDMA::writeMADCTL() {
  uint8_t mad = 0;
  if (_cfg.controller == ST77xxController::NV3007) {
    // NV3007 specific mapping (per vendor example):
    // rot0: 0x00, rot1: 0x60 (MX|MV), rot2: 0xC0 (MX|MY), rot3: 0xA0 (MY|MV)
    switch (_rotation & 3) {
      case 0: mad = 0;                                          break;
      case 1: mad = MADCTL_MX | MADCTL_MV;                      break;
      case 2: mad = MADCTL_MX | MADCTL_MY;                      break;
      case 3: mad = MADCTL_MY | MADCTL_MV;                      break;
    }
  } else {
    if (_cfg.controller == ST77xxController::ST7789) {
      // ST7789: mirror TFT_eSPI's rotation mapping so CGRAM offsets match exactly.
      // rotation 0: 0, 1: MX|MV, 2: MX|MY, 3: MV|MY
      switch (_rotation & 3) {
        case 0: mad = 0;                                   break;
        case 1: mad = MADCTL_MX | MADCTL_MV;               break;
        case 2: mad = MADCTL_MX | MADCTL_MY;               break;
        case 3: mad = MADCTL_MV | MADCTL_MY;               break;
      }
    } else {
      // Generic ST77xx mapping (kept for ST7735 and others to preserve behavior)
      switch (_rotation & 3) {
        // 0: MY|MX, 1: MY|MV, 2: 0, 3: MX|MV
        case 0: mad = MADCTL_MX | MADCTL_MY;               break;
        case 1: mad = MADCTL_MY | MADCTL_MV;               break;
        case 2: mad = 0;                                   break;
        case 3: mad = MADCTL_MX | MADCTL_MV;               break;
      }
    }
  }
  // Применить пользовательское зеркалирование поверх текущей ориентации
  // В режимах с MV=1 (rotation 1/3) визуальные оси X/Y меняются местами,
  // поэтому горизонтальное зеркало должно инвертировать MY, а вертикальное — MX.
  bool mv = (mad & MADCTL_MV) != 0;
  if (_mirrorX) mad ^= mv ? MADCTL_MY : MADCTL_MX;
  if (_mirrorY) mad ^= mv ? MADCTL_MX : MADCTL_MY;
  if (_bgr) mad |= MADCTL_BGR;
  sendCmdData(ST77XX_MADCTL, &mad, 1);
}

void ST77xxDMA::writeCOLMOD() {
  // Pixel format: many ST77xx use 0x55 (16bpp). NV3007 vendor init uses 0x05 for 16bpp.
  uint8_t colmod = 0x55;
  if (_cfg.controller == ST77xxController::NV3007) {
    colmod = _nv3007_colmod_55 ? 0x55 : 0x05;
  }
  sendCmdData(ST77XX_COLMOD, &colmod, 1);
}

void ST77xxDMA::panelInitBasic() {
  resetPanel();
  sendCmd(ST77XX_SWRESET); delay(120);
  sendCmd(ST77XX_SLPOUT);  delay(120);
  writeCOLMOD();           delay(10);
  writeMADCTL();           delay(10);
  // Default inversion policy:
  // - NV3007 glasses typically require INVON for correct white/black balance — enable by default
  // - Some ST7789 variants (170x320/172x320/240x320) also need INVON
  // - Otherwise, default to INVOFF
  if (_cfg.controller == ST77xxController::NV3007 ||
      (_cfg.controller == ST77xxController::ST7789 &&
       (_cfg.panel == ST77xxPanel::ST7789_170x320 || _cfg.panel == ST77xxPanel::ST7789_172x320 || _cfg.panel == ST77xxPanel::ST7789_240x320 ||
        _cfg.panel == ST77xxPanel::ST7789_135x240 || _cfg.panel == ST77xxPanel::ST7789_240x240))) {
    sendCmd(ST77XX_INVON);
  } else {
    sendCmd(ST77XX_INVOFF);
  }
  delay(10);
  sendCmd(ST77XX_NORON);   delay(10);
  sendCmd(ST77XX_DISPON);  delay(120);
}

// Helper to send a command list (from PROGMEM)
void ST77xxDMA::sendCmdList(const uint8_t *addr) {
  uint8_t numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);
  if (_debug) { Serial.print(F("[ST77xx][LIST] num=")); Serial.println(numCommands); }
  while (numCommands--) {
    uint8_t cmd = pgm_read_byte(addr++);
    numArgs = pgm_read_byte(addr++);
    ms = numArgs & 0x80; // Check for delay bit
    numArgs &= ~0x80;
    // Read payload (if any) from PROGMEM
    if (numArgs > 0) {
      uint8_t data_buf[numArgs];
      for (int i=0; i<numArgs; ++i) data_buf[i] = pgm_read_byte(addr++);
      sendCmdData(cmd, data_buf, numArgs);
    } else {
      sendCmdData(cmd, nullptr, 0);
    }
    if (ms) {
      ms = pgm_read_byte(addr++);
      if (ms == 255) ms = 500;
      if (_debug) {
        Serial.print(F("[ST77xx][DELAY] "));
        Serial.print(ms);
        Serial.println(F(" ms"));
      }
      delay(ms);
    }
  }
}

static const uint8_t nv3007_init_cmds[] PROGMEM = {
    98, // 98 commands total
    0xff, 1, 0xa5,
    0x9a, 1, 0x08,
    0x9b, 1, 0x08,
    0x9c, 1, 0xb0,
    0x9d, 1, 0x17,
    0x9e, 1, 0xc2,  // <-- Этот был пропущен!
    0x8f, 1, 0x22,
    0x84, 1, 0x90,
    0x83, 1, 0x7B,
    0x85, 1, 0x4F,
    0x6e, 1, 0x0f,
    0x7e, 1, 0x0f,
    0x60, 1, 0x00,
    0x70, 1, 0x00,
    0x6d, 1, 0x39,
    0x7d, 1, 0x31,
    0x61, 1, 0x0A,
    0x71, 1, 0x0A,
    0x6c, 1, 0x35,
    0x7c, 1, 0x29,
    0x62, 1, 0x0F,
    0x72, 1, 0x0F,
    0x68, 1, 0x4f,
    0x78, 1, 0x45,
    0x66, 1, 0x33,
    0x76, 1, 0x33,
    0x6b, 1, 0x14,
    0x7b, 1, 0x14,
    0x63, 1, 0x09,
    0x73, 1, 0x09,
    0x6a, 1, 0x13,
    0x7a, 1, 0x16,
    0x64, 1, 0x08,
    0x74, 1, 0x08,
    0x69, 1, 0x07,
    0x79, 1, 0x0d,
    0x65, 1, 0x05,
    0x75, 1, 0x05,
    0x67, 1, 0x33,
    0x77, 1, 0x33,
    0x6f, 1, 0x00,
    0x7f, 1, 0x00,
    0x50, 1, 0x00,
    0x52, 1, 0xd6,
    0x53, 1, 0x04,
    0x54, 1, 0x04,
    0x55, 1, 0x1b,
    0x56, 1, 0x1b,
    0xa0, 3, 0x2a, 0x24, 0x00,
    0xa1, 1, 0x84,
    0xa2, 1, 0x85,
    0xa8, 1, 0x34,
    0xa9, 1, 0x80,
    0xaa, 1, 0x73,
    0xAB, 2, 0x03, 0x61,
    0xAC, 2, 0x03, 0x65,
    0xAD, 2, 0x03, 0x60,
    0xAE, 2, 0x03, 0x64,
    0xB9, 1, 0x82,
    0xBA, 1, 0x83,
    0xBB, 1, 0x80,
    0xBC, 1, 0x81,
    0xBD, 1, 0x02,
    0xBE, 1, 0x01,
    0xBF, 1, 0x04,
    0xC0, 1, 0x03,
    0xc4, 1, 0x33,
    0xc5, 1, 0x80,
    0xc6, 1, 0x73,
    0xc7, 1, 0x00,
    0xC8, 2, 0x33, 0x33,
    0xC9, 1, 0x5b,
    0xCA, 1, 0x5a,
    0xCB, 1, 0x5d,
    0xCC, 1, 0x5c,
    0xCD, 2, 0x33, 0x33,
    0xCE, 1, 0x5f,
    0xCF, 1, 0x5e,
    0xD0, 1, 0x61,
    0xD1, 1, 0x60,
    0xB0, 4, 0x3a, 0x3a, 0x00, 0x00,
    0xB6, 1, 0x32,
    0xB7, 1, 0x80,
    0xB8, 1, 0x73,
    0xe0, 1, 0x00,
    0xe1, 2, 0x03, 0x0f,
    0xe2, 1, 0x04,
    0xe3, 1, 0x01,
    0xe4, 1, 0x0e,
    0xe5, 1, 0x01,
    0xe6, 1, 0x19,
    0xe7, 1, 0x10,
    0xe8, 1, 0x10,
    0xe9, 1, 0x21,
    0xea, 1, 0x12,
    0xeb, 1, 0xd0,
    0xec, 1, 0x04,
    0xed, 1, 0x07,
    0xee, 1, 0x07,
    0xef, 1, 0x09,
    0xF0, 1, 0xD0,
    0xF1, 1, 0x0E,
    0xF9, 1, 0x56,
    0xf2, 4, 0x26, 0x1b, 0x0b, 0x20,
    0xec, 1, 0x04,
    0x35, 1, 0x00,
    0x44, 2, 0x00, 0x10,
    0x46, 1, 0x10,
    0xff, 1, 0x00,
    0x3a, 1, 0x05,
    0x36, 1, 0x00,
    // Duplicate init commands
    0xff, 1, 0xa5,
    0x9a, 1, 0x08,
    0x9b, 1, 0x08,
    0x9c, 1, 0xb0,
    0x9d, 1, 0x17,
    // SLPOUT and DISPON moved to the start of panelInitNV3007() to ensure panel is on.
    // Keep a final command here to properly terminate the list.
    0xff, 1, 0x00
};

void ST77xxDMA::panelInitNV3007() {
  if (_debug) Serial.println(F("[ST77xx][NV3007] init start"));
  resetPanel();
  // For NV3007, it's critical to exit sleep and turn display on immediately after reset.
  // The original init list had these at the end, which likely failed.
  sendCmd(ST77XX_SLPOUT); delay(120);
  sendCmd(ST77XX_DISPON); delay(100);
  sendCmdList(nv3007_init_cmds);
  // Set pixel format and MADCTL after main init
  writeCOLMOD();
  writeMADCTL();
  // Default inversion for NV3007: many glasses require INVON for proper white/black
  sendCmd(ST77XX_INVON);
  delay(10);
  if (_debug) Serial.println(F("[ST77xx][NV3007] init done"));
}

void ST77xxDMA::resetPanel() {
  if (_cfg.rst >= 0) {
    pinMode(_cfg.rst, OUTPUT);
    digitalWrite(_cfg.rst, HIGH); delay(5);
    digitalWrite(_cfg.rst, LOW);  delay(20);
    digitalWrite(_cfg.rst, HIGH); delay(120);
  }
}

void ST77xxDMA::setWindowFull() {
  uint16_t xs = 0 + _xstart;
  uint16_t xe = (_lcdW - 1) + _xstart;
  uint16_t ys = 0 + _ystart;
  uint16_t ye = (_lcdH - 1) + _ystart;
  uint8_t b[4];
  b[0]=xs>>8; b[1]=xs&0xFF; b[2]=xe>>8; b[3]=xe&0xFF; sendCmdData(ST77XX_CASET, b, 4);
  b[0]=ys>>8; b[1]=ys&0xFF; b[2]=ye>>8; b[3]=ye&0xFF; sendCmdData(ST77XX_RASET, b, 4);
  // NV3007 may need a tiny settle time before RAM write
  if (_cfg.controller == ST77xxController::NV3007) { delayMicroseconds(5); }
}

void ST77xxDMA::setWindowY(uint16_t y0, uint16_t y1) {
  uint16_t xs = 0 + _xstart;
  uint16_t xe = (_lcdW - 1) + _xstart;
  uint16_t ys = y0 + _ystart;
  uint16_t ye = y1 + _ystart;
  uint8_t b[4];
  b[0]=xs>>8; b[1]=xs&0xFF; b[2]=xe>>8; b[3]=xe&0xFF; sendCmdData(ST77XX_CASET, b, 4);
  b[0]=ys>>8; b[1]=ys&0xFF; b[2]=ye>>8; b[3]=ye&0xFF; sendCmdData(ST77XX_RASET, b, 4);
  if (_cfg.controller == ST77xxController::NV3007) { delayMicroseconds(5); }
}

void ST77xxDMA::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint16_t xs = x0 + _xstart;
  uint16_t xe = x1 + _xstart;
  uint16_t ys = y0 + _ystart;
  uint16_t ye = y1 + _ystart;
  uint8_t b[4];
  b[0]=xs>>8; b[1]=xs&0xFF; b[2]=xe>>8; b[3]=xe&0xFF; sendCmdData(ST77XX_CASET, b, 4);
  b[0]=ys>>8; b[1]=ys&0xFF; b[2]=ye>>8; b[3]=ye&0xFF; sendCmdData(ST77XX_RASET, b, 4);
  if (_cfg.controller == ST77xxController::NV3007) { delayMicroseconds(5); }
}

void ST77xxDMA::startRAMWR() { sendCmdData(ramwrCmd(), nullptr, 0); }
void ST77xxDMA::endRAMWR()   { /* no-op */ }

void ST77xxDMA::writeWindowPixels(const void* data, size_t len) {
  startRAMWR();
  sendData(data, len);
}

// NV3007 control: choose RAM write opcode at runtime
void ST77xxDMA::setNV3007UseRAMWRC(bool enable) {
  _nv3007_use_ramwrc = enable;
}

uint8_t ST77xxDMA::ramwrCmd() const {
  if (_cfg.controller == ST77xxController::NV3007 && _nv3007_use_ramwrc) return 0x3C; // RAMWRC
  return ST77XX_RAMWR; // default
}

void ST77xxDMA::setNV3007UseCOLMOD55(bool enable) {
  _nv3007_colmod_55 = enable;
  // If display is already initialized, re-apply COLMOD immediately.
  // This is critical for NV3007 where examples may toggle the preference after begin().
  if (_begun && _cfg.controller == ST77xxController::NV3007) {
    if (_debug) Serial.println(F("[ST77xx][NV3007] Re-applying COLMOD due to runtime toggle"));
    writeCOLMOD();
  }
}

// ------------ SPI backend ------------
void ST77xxDMA::backendInitSPI() {
  // pins
  if (_cfg.cs  >= 0) { pinMode(_cfg.cs,  OUTPUT); digitalWrite(_cfg.cs, HIGH);} 
  if (_cfg.dc  >= 0) { pinMode(_cfg.dc,  OUTPUT); digitalWrite(_cfg.dc, HIGH);} 
  if (_cfg.mosi>= 0)  pinMode(_cfg.mosi, OUTPUT);
  if (_cfg.sclk>= 0)  pinMode(_cfg.sclk, OUTPUT);
  if (_cfg.miso>= 0)  pinMode(_cfg.miso, INPUT);

#if defined(ARDUINO_ARCH_ESP32)
  // ESP32-family: use IDF SPI master
  // Reset ESP32 SPI bookkeeping early to avoid stale handles on re-init
  _spi_dev = nullptr;
  _spi_host = 0;
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = _cfg.mosi;
  buscfg.miso_io_num = _cfg.miso;
  buscfg.sclk_io_num = _cfg.sclk;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  // Half-frame as safe max transfer
  size_t full_bytes = (size_t)_lcdW * _lcdH * 2;
  _frame_bytes = full_bytes;
  _half_bytes  = full_bytes / 2;
  size_t max_tx = _half_bytes + 32;
  buscfg.max_transfer_sz = max_tx;

  // Optional: Software SPI on ESP32 when use_hw_spi==false
  if (!_cfg.use_hw_spi) {
    // Ensure idle levels
    if (_cfg.sclk >= 0) digitalWrite(_cfg.sclk, LOW);
    if (_cfg.mosi >= 0) digitalWrite(_cfg.mosi, LOW);
    if (_debug) Serial.println(F("[ST77xx] ESP32: using Software SPI (bit-bang)"));
    return; // Skip ESP-IDF SPI init
  }

  // Select host per target (robust across core/IDF versions)
  // Prefer SPI3 when available, else SPI2; as legacy fallbacks use VSPI/HSPI; otherwise default to SPI2 numeric.
  #if defined(SPI3_HOST)
    _spi_host = SPI3_HOST;
  #elif defined(SPI2_HOST)
    _spi_host = SPI2_HOST;
  #elif defined(VSPI_HOST)
    _spi_host = VSPI_HOST;
  #elif defined(HSPI_HOST)
    _spi_host = HSPI_HOST;
  #else
    _spi_host = 1; // SPI2 default
  #endif

  ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)_spi_host, &buscfg, SPI_DMA_CH_AUTO));

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = (int)_cfg.spi_hz;
  devcfg.mode = 0;
  devcfg.spics_io_num = -1; // manual CS
  devcfg.queue_size = 1;
  devcfg.flags = SPI_DEVICE_HALFDUPLEX;

  spi_device_handle_t dev = nullptr;
  ESP_ERROR_CHECK(spi_bus_add_device((spi_host_device_t)_spi_host, &devcfg, &dev));
  _spi_dev = dev;
  if (_debug) {
    Serial.print(F("[ST77xx] ESP32 SPI init ok: host=")); Serial.print(_spi_host);
    Serial.print(F(" dev=0x")); Serial.println((uintptr_t)_spi_dev, HEX);
    Serial.print(F("[ST77xx] ESP32 max_transfer_sz=")); Serial.println((unsigned long)buscfg.max_transfer_sz);
  }

#else
  // Non-ESP32: use Arduino SPI if requested
  if (_cfg.use_hw_spi) {
    #if defined(ARDUINO_ARCH_RP2040)
      if (_cfg.sclk >= 0) SPI.setSCK((uint8_t)_cfg.sclk);
      if (_cfg.mosi >= 0) SPI.setTX((uint8_t)_cfg.mosi);
      if (_cfg.miso >= 0) SPI.setRX((uint8_t)_cfg.miso);
      SPI.begin();
      // RP2040: усилить драйв и фронты на выводах SPI, отключить подтяжки
      {
        auto set_drive = [&](int pin){
          if (pin < 0) return;
          gpio_set_drive_strength((uint)pin, GPIO_DRIVE_STRENGTH_12MA);
          gpio_set_slew_rate((uint)pin, GPIO_SLEW_RATE_FAST);
          gpio_disable_pulls((uint)pin);
        };
        set_drive(_cfg.sclk);
        set_drive(_cfg.mosi);
        set_drive(_cfg.cs);
        set_drive(_cfg.dc);
        set_drive(_cfg.rst);
      }
      #if ST77XX_HAVE_RP2040_DMA
        rp2040InitDMA();
      #endif
    #elif defined(ESP8266)
      // ESP8266: SPI.begin() has no overload with pin arguments; hardware SPI pins are fixed.
      // CS is controlled manually using _cfg.cs.
      SPI.begin();
    #else
      SPI.begin();
    #endif
  } else {
    // SW SPI: already set pinModes above
    digitalWrite(_cfg.sclk, LOW);
    digitalWrite(_cfg.mosi, LOW);
    #if defined(ARDUINO_ARCH_RP2040)
      // RP2040: усилить драйв и фронты для SW SPI линий
      auto set_drive = [&](int pin){
        if (pin < 0) return;
        gpio_set_drive_strength((uint)pin, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate((uint)pin, GPIO_SLEW_RATE_FAST);
        gpio_disable_pulls((uint)pin);
      };
      set_drive(_cfg.sclk);
      set_drive(_cfg.mosi);
      set_drive(_cfg.cs);
      set_drive(_cfg.dc);
      set_drive(_cfg.rst);
    #endif
  }
  // compute sizes for buffer even here
  _frame_bytes = (size_t)_lcdW * _lcdH * 2;
  _half_bytes  = _frame_bytes / 2;
#endif
}

void ST77xxDMA::backendDeinitSPI() {
#if defined(ARDUINO_ARCH_ESP32)
  if (_spi_dev) {
    spi_device_handle_t dev = (spi_device_handle_t)_spi_dev;
    spi_bus_remove_device(dev);
    _spi_dev = nullptr;
  }
  if (_spi_host) {
    spi_bus_free((spi_host_device_t)_spi_host);
    _spi_host = 0;
  }
#else
  #if ST77XX_HAVE_RP2040_DMA
    rp2040DeinitDMA();
  #endif
  if (_cfg.use_hw_spi) SPI.end();
#endif
}

#if ST77XX_HAVE_RP2040_DMA
// --- RP2040 DMA helpers ---
static void rp2040_pick_spi_from_sck(int sck_pin, spi_inst_t** out_spi, uint32_t* out_dreq) {
  // Heuristic: for common Pico wiring, SCK 10/14 map to SPI1; otherwise SPI0.
  // This can be extended if вы используете альтернативные группы.
  bool use_spi1 = (sck_pin == 10) || (sck_pin == 14);
  if (use_spi1) {
    *out_spi = spi1;
    *out_dreq = DREQ_SPI1_TX;
  } else {
    *out_spi = spi0;
    *out_dreq = DREQ_SPI0_TX;
  }
}

void ST77xxDMA::rp2040InitDMA() {
  // Try to pick SPI instance by selected SCK pin
  int sck = (_cfg.sclk >= 0) ? _cfg.sclk :
#ifdef PIN_SPI_SCK
    (int)PIN_SPI_SCK
#else
    18
#endif
  ;
  spi_inst_t* spi_inst = nullptr;
  uint32_t dreq = 0;
  // Honor explicit user selection first
  if (_cfg.rp2040_spi_index == 0) {
    spi_inst = spi0; dreq = DREQ_SPI0_TX;
  } else if (_cfg.rp2040_spi_index == 1) {
    spi_inst = spi1; dreq = DREQ_SPI1_TX;
  } else {
    rp2040_pick_spi_from_sck(sck, &spi_inst, &dreq);
  }
  _rp2040_spi = (void*)spi_inst;
  _rp2040_dreq = dreq;
  // Optional RP2040 clock management: ensure clk_peri (and clk_sys) are high enough for target SPI
  {
    uint32_t target_spi_hz = _cfg.spi_hz ? _cfg.spi_hz : 1000000u;
    uint32_t sys_hz_before = clock_get_hz(clk_sys);
    uint32_t peri_hz_before = clock_get_hz(clk_peri);
    if (_cfg.rp2040_target_sysclk_khz) {
      // Explicit target: raise clk_sys to requested value, then tie clk_peri to clk_sys (no divider)
      uint32_t target_sys_khz = _cfg.rp2040_target_sysclk_khz;
      if (_debug) {
        Serial.print(F("[ST77xx] RP2040: setting clk_sys to "));
        Serial.print((unsigned long)target_sys_khz);
        Serial.println(F(" kHz and tying clk_peri=clk_sys/1"));
      }
      set_sys_clock_khz(target_sys_khz, false);
      uint32_t sys_hz_now = clock_get_hz(clk_sys);
      clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, sys_hz_now, sys_hz_now);
      if (_debug) {
        Serial.print(F("[ST77xx] RP2040 clocks after: clk_sys=")); Serial.print((unsigned long)clock_get_hz(clk_sys));
        Serial.print(F(" clk_peri=")); Serial.println((unsigned long)clock_get_hz(clk_peri));
      }
    } else if (_cfg.rp2040_auto_raise_clk_peri) {
      // Only tie clk_peri to current clk_sys; do not change clk_sys automatically
      uint32_t sys_hz_now = sys_hz_before;
      if (_debug) Serial.println(F("[ST77xx] RP2040: tying clk_peri to current clk_sys (no sysclk change)"));
      clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, sys_hz_now, sys_hz_now);
      if (_debug) {
        Serial.print(F("[ST77xx] RP2040 clocks: clk_sys=")); Serial.print((unsigned long)clock_get_hz(clk_sys));
        Serial.print(F(" clk_peri=")); Serial.println((unsigned long)clock_get_hz(clk_peri));
      }
    } else if (_debug) {
      // Warn if user requested high SPI but clk_peri is too low to derive it reliably
      if (target_spi_hz && peri_hz_before < target_spi_hz * 2u) {
        Serial.print(F("[ST77xx] Warning: clk_peri=")); Serial.print((unsigned long)peri_hz_before);
        Serial.print(F(" < ~2x target SPI=")); Serial.print((unsigned long)target_spi_hz);
        Serial.println(F(". Consider cfg.rp2040_auto_raise_clk_peri=true or set rp2040_target_sysclk_khz."));
      }
    }
  }
  // Optionally force GPIO functions for selected pins to SPI
  if (_cfg.rp2040_force_gpio_func) {
    if (_cfg.sclk >= 0) gpio_set_function((uint)_cfg.sclk, GPIO_FUNC_SPI);
    if (_cfg.mosi >= 0) gpio_set_function((uint)_cfg.mosi, GPIO_FUNC_SPI);
    if (_cfg.miso >= 0) gpio_set_function((uint)_cfg.miso, GPIO_FUNC_SPI);
  }
  // Ensure SPI instance is initialized before setting format/baud
  // Use requested baud (or a safe default) to initialize the peripheral
  spi_init(spi_inst, _cfg.spi_hz ? _cfg.spi_hz : 1000000u);
  // Configure baudrate precisely if provided
  if (_cfg.spi_hz) {
    _rp2040_spi_actual_hz = spi_set_baudrate((spi_inst_t*)_rp2040_spi, _cfg.spi_hz);
  }
  // Ensure 8-bit, mode 0, MSB-first
  spi_set_format((spi_inst_t*)_rp2040_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  // Claim a DMA channel
  _rp2040_dma_ch = dma_claim_unused_channel(true);
  if (_debug) {
    Serial.print(F("[ST77xx] clk_sys=")); Serial.print((unsigned long)clock_get_hz(clk_sys));
    Serial.print(F(" clk_peri=")); Serial.println((unsigned long)clock_get_hz(clk_peri));
    Serial.print(F("[ST77xx] RP2040 SPI instance: "));
    Serial.print((_rp2040_spi == (void*)spi0) ? F("SPI0") : F("SPI1"));
    Serial.print(F(" DREQ=0x")); Serial.println(_rp2040_dreq, HEX);
    Serial.print(F("[ST77xx] DMA CH=")); Serial.println(_rp2040_dma_ch);
    Serial.print(F("[ST77xx] Pins: SCK=")); Serial.print(_cfg.sclk);
    Serial.print(F(" MOSI=")); Serial.print(_cfg.mosi);
    Serial.print(F(" MISO=")); Serial.println(_cfg.miso);
    if (_cfg.spi_hz) { Serial.print(F("[ST77xx] SPI target Hz=")); Serial.println((unsigned long)_cfg.spi_hz); }
  }
}

void ST77xxDMA::rp2040DeinitDMA() {
  if (_rp2040_dma_ch >= 0) {
    dma_channel_unclaim(_rp2040_dma_ch);
    _rp2040_dma_ch = -1;
  }
}

void ST77xxDMA::rp2040DMASend(const uint8_t* data, size_t len) {
  if (_rp2040_dma_ch < 0 || !data || len == 0) return;
  spi_inst_t* spi = (spi_inst_t*)_rp2040_spi;
  volatile void* dst = (volatile void*)&spi_get_hw(spi)->dr;

  const size_t chunk = 65520; // ~64KB chunks reduce reconfig overhead
  while (len) {
    size_t n = (len > chunk) ? chunk : len;

    dma_channel_config c = dma_channel_get_default_config(_rp2040_dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, _rp2040_dreq);

    dma_channel_configure(
      _rp2040_dma_ch,
      &c,
      dst,          // write address (SPI TX FIFO)
      data,         // read address (memory buffer)
      n,            // number of bytes
      false         // don't start yet
    );

    dma_start_channel_mask(1u << _rp2040_dma_ch);
    dma_channel_wait_for_finish_blocking(_rp2040_dma_ch);

    data += n; len -= n;
  }

  // Ensure SPI has shifted out the FIFO contents
  while (spi_is_busy(spi)) { /* busy wait */ }
}
#endif

// ------------ Public API ------------
bool ST77xxDMA::begin() {
  applyPanelPreset();
  // Normalize SPI frequency: if small value (<=512), treat as MHz (e.g. 80 -> 80MHz)
  if (_cfg.spi_hz > 0 && _cfg.spi_hz <= 512u) {
    _cfg.spi_hz *= 1000000u;
  }
  // New session: allow one-time flush mode logging again
  _flushModeLogged = false;
#if defined(ARDUINO_ARCH_RP2040)
#if !ST77XX_HAVE_RP2040_PIO
  if (_rp2040_use_pio) {
    // PIO not compiled in; force-disable
    _rp2040_use_pio = false;
    if (_debug) Serial.println(F("[ST77xx] RP2040 PIO requested but not available at build time, falling back to SPI+DMA"));
  }
#endif
#endif
  backendInitSPI();
  // Native demo sets BL pin OUTPUT and HIGH before init. Mirror that here for NV3007 only.
  if (_cfg.controller == ST77xxController::NV3007 && _cfg.bl >= 0) {
    pinMode(_cfg.bl, OUTPUT);
    // Respect inversion flag if used in config
    digitalWrite(_cfg.bl, _cfg.bl_invert ? LOW : HIGH);
    if (_debug) Serial.println(F("[ST77xx][NV3007] Backlight enabled (pinMode+digitalWrite)"));
  }
  setupBacklight();

  if (_cfg.controller == ST77xxController::NV3007) panelInitNV3007();
  else panelInitBasic();
  setRotation(_rotation);

#if ST77XX_HAVE_RP2040_PIO
  // Initialize RP2040 PIO backend if requested and available
  if (_rp2040_use_pio) {
    // Recreate backend to match current rotation/geometry
    if (_rp2040_pio) { _rp2040_pio->end(); delete _rp2040_pio; _rp2040_pio = nullptr; }
    ST77xxConfig pcfg;
    pcfg.width  = _lcdW;
    pcfg.height = _lcdH;
    pcfg.colstart = _xstart;
    pcfg.rowstart = _ystart;
    pcfg.mosi = _cfg.mosi;
    pcfg.sclk = _cfg.sclk;
    pcfg.dc   = _cfg.dc;
    pcfg.cs   = _cfg.cs;
    pcfg.rst  = _cfg.rst;
    // CS is assumed active-low by the PIO backend; no explicit config flag required here
    pcfg.spi_hz = _cfg.spi_hz; // already normalized to Hz
    pcfg.debug  = _debug;
    pcfg.rp2040_auto_raise_clk_peri = _cfg.rp2040_auto_raise_clk_peri;
    pcfg.rp2040_target_sysclk_khz   = _cfg.rp2040_target_sysclk_khz;

    if (_debug) {
      Serial.print(F("[ST77xx] PIO pin map (from cfg): MOSI=")); Serial.print(_cfg.mosi);
      Serial.print(F(" SCLK=")); Serial.print(_cfg.sclk);
      Serial.print(F(" DC="));   Serial.print(_cfg.dc);
      Serial.print(F(" CS="));   Serial.print(_cfg.cs);
      Serial.print(F(" RST="));  Serial.println(_cfg.rst);
    }

    _rp2040_pio = new ST77xxPIO(pcfg);
    if (!_rp2040_pio) {
      if (_debug) Serial.println(F("[ST77xx] RP2040 PIO init failed, falling back to SPI+DMA"));
      _rp2040_use_pio = false;
    } else {
      // Ensure default display is set before enabling DMA IRQs in PIO backend
      if (!ST77xx::defaultDisplay()) {
        ST77xx::setDefault(this);
      }
      _rp2040_pio->begin();
    }
  }
#endif

  // If panel has no external BL pin and controller supports panel-side brightness, enable it
  if (_cfg.bl < 0 && _cfg.controller == ST77xxController::ST7789) {
    enablePanelBrightness(true);
    setPanelBrightness(_cfg.panel_brightness_start);
  }

  if (_cfg.allocate_framebuffer) {
#if defined(ARDUINO_ARCH_ESP32)
    _frame = (uint16_t*)heap_caps_malloc(_frame_bytes, MALLOC_CAP_DMA);
#else
    _frame = (uint16_t*)malloc(_frame_bytes);
#endif
    if (!_frame) {
      // Low-RAM fallback (e.g., ESP8266): proceed without a framebuffer
      if (_debug) Serial.println(F("[ST77xx][WARN] framebuffer allocation failed; falling back to immediate mode (no FB)"));
      _cfg.allocate_framebuffer = false;
    } else {
      memset(_frame, 0, _frame_bytes);
    }
  }
  // Optional small line-based TX buffer (partial framebuffer style)
  _txbuf = nullptr; _txbuf_words = 0; _txbuf_lines = 0;
  // Decide desired TX buffer lines:
  // - If user specified txbuf_lines > 0, honor it (clamped to height)
  // - Else if we have no framebuffer (either by user choice or FB alloc fallback), pick a sane default
  uint16_t desired_lines = 0;

  // Gate TX buffer usage to ESP8266 only for now. On other MCUs keep it disabled
  // to avoid unintended interactions; immediate paths will fall back to on-stack
  // chunks or full framebuffer when available.
#if defined(ESP8266)
  if (_cfg.txbuf_lines > 0) {
    desired_lines = _cfg.txbuf_lines;
  } else if (!_cfg.allocate_framebuffer) {
    // Auto-pick 4 lines (or fewer if panel shorter). This improves throughput in immediate mode.
    desired_lines = (_lcdH < 4) ? _lcdH : 4;
    if (_debug) {
      Serial.print(F("[ST77xx][DBG] no framebuffer active; auto-selecting TX buffer lines="));
      Serial.println(desired_lines);
    }
  }
#else
  if (_cfg.txbuf_lines > 0) {
    if (_debug) Serial.println(F("[ST77xx][DBG] TX buffer requested but disabled on this MCU (supported on ESP8266 only); ignoring"));
  }
  _cfg.txbuf_lines = 0; // force-disable on non-ESP8266
#endif
  if (desired_lines > 0) {
    // Cap to panel height
    if (desired_lines > _lcdH) desired_lines = _lcdH;

    // Try to allocate, with graceful degradation (e.g., 4 -> 2 -> 1 lines)
    uint16_t try_lines = desired_lines;
    while (try_lines > 0 && !_txbuf) {
      size_t words = (size_t)_lcdW * try_lines;
      size_t bytes = words * sizeof(uint16_t);
#if defined(ARDUINO_ARCH_ESP32)
      _txbuf = (uint16_t*)heap_caps_malloc(bytes, MALLOC_CAP_DMA);
#else
      _txbuf = (uint16_t*)malloc(bytes);
#endif
      if (_txbuf) {
        _txbuf_words = words;
        _txbuf_lines = try_lines;
        if (_debug) {
          Serial.print(F("[ST77xx] TX buffer allocated: lines=")); Serial.print(try_lines);
          Serial.print(F(" words=")); Serial.println((unsigned long)words);
        }
        // Reflect actual size back into cfg for user inspection
        _cfg.txbuf_lines = try_lines;
        break;
      } else {
        if (_debug) {
          Serial.print(F("[ST77xx][WARN] TX buffer allocation failed for lines="));
          Serial.print(try_lines);
          Serial.println(F(", trying smaller"));
        }
        // Degrade: prefer 2 or 1 line as last resort
        if (try_lines > 2) try_lines = 2; else try_lines = try_lines - 1; // 2 -> 1 -> 0
      }
    }
    if (!_txbuf && _debug) {
      Serial.println(F("[ST77xx][WARN] Unable to allocate any TX buffer; continuing without it"));
    }
  }
  _begun = true;
#if 1
  // Fill screen black after NV3007 init to match native demo behavior
  if (_cfg.controller == ST77xxController::NV3007) {
    if (_debug) Serial.println(F("[ST77xx][NV3007] Clearing screen to black after init"));
    fillScreen(ST77xxDMA::Colors::Black);
  }
#endif
#if defined(ARDUINO_ARCH_RP2040)
  if (_debug) {
    Serial.print(F("[ST77xx] RP2040 backend: "));
    Serial.println(_rp2040_use_pio ? F("PIO+DMA") : F("SPI+DMA"));
    Serial.print(F("[ST77xx] SPI target Hz="));
    Serial.print((unsigned long)_cfg.spi_hz);
#if ST77XX_HAVE_RP2040_DMA
    Serial.print(F(" actual="));
    Serial.println((unsigned long)_rp2040_spi_actual_hz);
#else
    Serial.println();
#endif
  }
#endif
  return true;
}

void ST77xxDMA::end() {
#if ST77XX_HAVE_RP2040_PIO
  if (_rp2040_pio) {
    _rp2040_pio->end();
    delete _rp2040_pio;
    _rp2040_pio = nullptr;
    _rp2040_hw_running = false;
  }
#endif
  if (_frame) { free(_frame); _frame = nullptr; }
  if (_txbuf)  { free(_txbuf);  _txbuf = nullptr; _txbuf_words = 0; _txbuf_lines = 0; }
  backendDeinitSPI();
  _begun = false;
  _flushModeLogged = false; // reset for next session
}

void ST77xxDMA::setRotation(uint8_t r) {
  _rotation = r & 3;
  // Recompute logical width/height from panel preset + overrides
  uint16_t baseW = 0, baseH = 0;
  switch (_cfg.panel) {
    case ST77xxPanel::ST7735_128x160: baseW = 128; baseH = 160; break;
    case ST77xxPanel::ST7789_240x320: baseW = 240; baseH = 320; break;
    case ST77xxPanel::ST7789_172x320: baseW = 172; baseH = 320; break;
    case ST77xxPanel::ST7789_76x284:  baseW =  76; baseH = 284; break;
    case ST77xxPanel::ST7789_170x320: baseW = 170; baseH = 320; break;
    case ST77xxPanel::ST7789_135x240: baseW = 135; baseH = 240; break;
    case ST77xxPanel::ST7789_240x240: baseW = 240; baseH = 240; break;
    case ST77xxPanel::ST7735_80x160:  baseW =  80; baseH = 160; break;
    case ST77xxPanel::NV3007_428x142: baseW = 428; baseH = 142; break;
  }
  if (_cfg.width)  baseW = _cfg.width;
  if (_cfg.height) baseH = _cfg.height;
  // For NV3007, swap logical W/H on rotations 0 and 2 (landscape orientations)
  // per module behavior; keep 1 and 3 without swap.
  if (_cfg.controller == ST77xxController::NV3007) {
    if ((_rotation == 0) || (_rotation == 2)) {
      _lcdW = baseH;
      _lcdH = baseW;
    } else {
      _lcdW = baseW;
      _lcdH = baseH;
    }
  } else {
    if (_rotation & 1) {
      _lcdW = baseH;
      _lcdH = baseW;
    } else {
      _lcdW = baseW;
      _lcdH = baseH;
    }
  }

  // Compute effective start offsets per rotation (center crops for narrow panels)
  _xstart = _colstart;
  _ystart = _rowstart;

  // NV3007 reference has no offsets; force to zero
  if (_cfg.controller == ST77xxController::NV3007) {
    _xstart = 0;
    _ystart = 0;
  }

  if (_cfg.controller == ST77xxController::ST7789) {
    if (_cfg.panel == ST77xxPanel::ST7789_170x320) {
      // Panel memory is 240x320; visible area 170x320 -> center with 35px margin horizontally
      const uint8_t xoff = 35, yoff = 0;
      switch (_rotation & 3) {
        case 0: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
        case 1: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break; // axes swapped by MV
        case 2: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
        case 3: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break;
      }
    } else if (_cfg.panel == ST77xxPanel::ST7789_172x320) {
      // 172x320 -> 34px total crop horizontally (17 each side), but many modules center to 34 left margin with MADCTL
      const uint8_t xoff = 34, yoff = 0; // common setting
      switch (_rotation & 3) {
        case 0: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
        case 1: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break;
        case 2: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
        case 3: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break;
      }
    } else if (_cfg.panel == ST77xxPanel::ST7789_76x284) {
      // Panel RAM is 240x320; visible is 76x284. Center: x (240-76)/2=82, y (320-284)/2=18
      const uint8_t xoff = 82, yoff = 18;
      switch (_rotation & 3) {
        case 0: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
        case 1: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break; // swap for MV
        case 2: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
        case 3: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break;
      }
    } else if (_cfg.panel == ST77xxPanel::ST7789_240x240) {
      // Many 240x240 ST7789 modules use a 240x320 RAM with a vertical offset of 80 pixels
      switch (_rotation & 3) {
        case 0: _xstart = _colstart + 0;  _ystart = _rowstart + 0;  break; // portrait
        case 1: _xstart = _colstart + 0;  _ystart = _rowstart + 0;  break; // landscape
        case 2: _xstart = _colstart + 0;  _ystart = _rowstart + 80; break; // inverted portrait
        case 3: _xstart = _colstart + 80; _ystart = _rowstart + 0;  break; // inverted landscape
      }
    } else if (_cfg.panel == ST77xxPanel::ST7789_135x240) {
      // 1.14" 135x240 ST7789: typical RAM is 240x320 with offsets ~52x40
      switch (_rotation & 3) {
        case 0: _xstart = _colstart + 52; _ystart = _rowstart + 40; break; // portrait
        case 1: _xstart = _colstart + 40; _ystart = _rowstart + 53; break; // landscape
        case 2: _xstart = _colstart + 53; _ystart = _rowstart + 40; break; // inverted portrait
        case 3: _xstart = _colstart + 40; _ystart = _rowstart + 52; break; // inverted landscape
      }
    }
  } else if (_cfg.controller == ST77xxController::NV3007) {
    // NV3007 glass (e.g., 428x142) requires fixed RAM offsets per rotation
    // per working vendor example (see src/NV3007/Graphics_test/TFTM1.65-2.cpp):
    // rotation 0: x+=12, y+=0
    // rotation 1: x+=0,  y+=14
    // rotation 2: x+=14, y+=0
    // rotation 3: x+=0,  y+=12
    switch (_rotation & 3) {
      case 0: _xstart = _colstart + 12; _ystart = _rowstart + 0;  break;
      case 1: _xstart = _colstart + 0;  _ystart = _rowstart + 14; break;
      case 2: _xstart = _colstart + 14; _ystart = _rowstart + 0;  break;
      case 3: _xstart = _colstart + 0;  _ystart = _rowstart + 12; break;
    }
  }

  if (_cfg.controller == ST77xxController::ST7735 &&
      _cfg.panel == ST77xxPanel::ST7735_80x160) {
    // Many 0.96" 80x160 ST7735S modules have a 132x162 RAM; visible window is centered
    const uint8_t xoff = 24, yoff = 0;
    switch (_rotation & 3) {
      case 0: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
      case 1: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break;
      case 2: _xstart = _colstart + xoff; _ystart = _rowstart + yoff; break;
      case 3: _xstart = _colstart + yoff; _ystart = _rowstart + xoff; break;
    }
  }

  writeMADCTL();
}

void ST77xxDMA::setBGR(bool bgr){ _bgr = bgr; writeMADCTL(); }
void ST77xxDMA::setOffsets(uint8_t c, uint8_t r){ _colstart=c; _rowstart=r; }

// Toggle display inversion (pixel polarity)
void ST77xxDMA::setInversion(bool enable) {
  sendCmd(enable ? ST77XX_INVON : ST77XX_INVOFF);
}

void ST77xxDMA::setBacklight(uint8_t level) { applyBacklight(level); }

// Unified brightness: use hardware BL pin if present, otherwise try panel-side brightness
void ST77xxDMA::setBrightness(uint8_t level) {
  if (_cfg.bl >= 0) {
    applyBacklight(level);
    return;
  }
  if (_cfg.controller == ST77xxController::ST7789) {
    enablePanelBrightness(true);
    setPanelBrightness(level);
  }
}

// Отзеркаливание изображения (независимо от поворота)
void ST77xxDMA::setMirror(bool mirrorX, bool mirrorY) {
  _mirrorX = mirrorX;
  _mirrorY = mirrorY;
  writeMADCTL();
}

// ---- Tearing Effect (TE) control ----
// enable = true -> TEON (0x35). If 'mode' is provided, send as parameter (e.g., 0x00 V-blank only, 0x01 V-blank + H-blank)
// enable = false -> TEOFF (0x34)
void ST77xxDMA::setTE(bool enable, uint8_t mode) {
  if (enable) {
    // Some controllers accept TEON without parameter; honor mode if user specified.
    sendCmdData(ST77XX_TEON, &mode, 1);
  } else {
    sendCmd(ST77XX_TEOFF);
  }
}

// Set TE scanline (MIPI DCS 0x44)
void ST77xxDMA::setTEScanline(uint16_t line) {
  uint8_t d[2] = { (uint8_t)(line >> 8), (uint8_t)(line & 0xFF) };
  sendCmdData(ST77XX_TESCAN, d, 2);
}

// ---- Vertical scrolling ----
// Define scroll regions (top fixed area, scroll area height, bottom fixed area)
void ST77xxDMA::setScrollDefinition(uint16_t topFixed, uint16_t scrollArea, uint16_t bottomFixed) {
  uint8_t d[6] = {
    (uint8_t)(topFixed >> 8),   (uint8_t)(topFixed & 0xFF),
    (uint8_t)(scrollArea >> 8), (uint8_t)(scrollArea & 0xFF),
    (uint8_t)(bottomFixed >> 8),(uint8_t)(bottomFixed & 0xFF)
  };
  sendCmdData(ST77XX_VSCRDEF, d, 6);
}

// Set vertical scroll start address
void ST77xxDMA::setScroll(uint16_t y) {
  uint8_t d[2] = { (uint8_t)(y >> 8), (uint8_t)(y & 0xFF) };
  sendCmdData(ST77XX_VSCRSADD, d, 2);
}

// ---- Partial display mode ----
// Set partial area (inclusive). Apply current RAM y-offset so API uses logical coordinates.
void ST77xxDMA::setPartialArea(uint16_t y0, uint16_t y1) {
  uint16_t ys = (uint16_t)(y0 + _ystart);
  uint16_t ye = (uint16_t)(y1 + _ystart);
  uint8_t d[4] = { (uint8_t)(ys >> 8), (uint8_t)(ys & 0xFF), (uint8_t)(ye >> 8), (uint8_t)(ye & 0xFF) };
  sendCmdData(ST77XX_PTLAR, d, 4);
}

// Enable/disable partial mode (PTLON/NORON)
void ST77xxDMA::setPartialMode(bool enable) {
  sendCmd(enable ? ST77XX_PTLON : ST77XX_NORON);
}

// ---- Panel-side brightness (aka "contrast") ----
void ST77xxDMA::enablePanelBrightness(bool enable) {
  if (_cfg.controller != ST77xxController::ST7789) return;
  uint8_t v = enable ? 0x24 : 0x00; // BCTRL=1, BL=1 when enabled; 0 to disable
  sendCmdData(ST77XX_WRCTRLD, &v, 1);
}

void ST77xxDMA::setPanelBrightness(uint8_t level) {
  if (_cfg.controller != ST77xxController::ST7789) return;
  // Make sure control is enabled
  uint8_t ctrl = 0x24; // BCTRL=1, BL=1
  sendCmdData(ST77XX_WRCTRLD, &ctrl, 1);
  // Write brightness level (0..255)
  sendCmdData(ST77XX_WRDISBV, &level, 1);
}

bool ST77xxDMA::setFrequency(uint32_t hz) {
  if (hz > 0 && hz <= 512u) hz *= 1000000u; // interpret small values as MHz
  _cfg.spi_hz = hz; // On ESP32 need re-add device; keep simple: reinit
  if (!_begun) return true;
  backendDeinitSPI();
  backendInitSPI();
  writeMADCTL(); writeCOLMOD();
  return true;
}

uint16_t* ST77xxDMA::framebuffer(){ return _frame; }
size_t ST77xxDMA::framebufferSize() const { return _frame ? _frame_bytes : 0; }

void ST77xxDMA::flush() {
  if (!_frame) return;
  // Prevent any transfers after end() or before begin()
  if (!_begun) {
    if (_debug) Serial.println(F("[ST77xx][WARN] flush() called while not begun; ignoring"));
    return;
  }
#if defined(ARDUINO_ARCH_ESP32)
  // Software SPI path on ESP32
  if (!_cfg.use_hw_spi) {
    // Stream entire framebuffer using bit-bang SPI, no IDF calls at all
    setWindowFull();
    const uint8_t ramwr = ramwrCmd();
    const uint8_t* p = (const uint8_t*)_frame;
    size_t left = _frame_bytes;
    const size_t chunk = 4096; // safe chunk size
    if (_debug && !_flushModeLogged) { Serial.println(F("[ST77xx][DBG] ESP32 flush: SW SPI bit-bang path")); _flushModeLogged = true; }

    // Begin transfer
    dcCmd(); csLow();
    // Send command byte
    sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
    dcData();
    // Stream data bytes
    while (left) {
      size_t n = (left > chunk) ? chunk : left;
      sw_write_block(_cfg.sclk, _cfg.mosi, p, n, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      p += n; left -= n;
      yield();
    }
    csHigh();
    return;
  }

  // Hardware SPI path on ESP32
  // Validate device handle prior to any nested command/data writes
  if (_spi_dev == nullptr) {
    if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 flush: _spi_dev is null; aborting"));
    return;
  }
  // Two half-frame DMA transfers to avoid size limits
  uint8_t cmd = ramwrCmd();

  // Top half
  setWindowY(0, (_lcdH/2)-1);
  dcCmd(); csLow();
  spi_transaction_t t = {};
  t.length = 8; t.tx_buffer = &cmd; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
  dcData();
  // Stream in smaller chunks to satisfy hardware/driver constraints
  {
    const uint8_t* p = (const uint8_t*)_frame;
    size_t left = _half_bytes;
    const size_t chunk = 4092; // safe chunk size across ESP32 variants
    while (left) {
      size_t n = (left > chunk) ? chunk : left;
      spi_transaction_t td = {};
      td.length = n * 8;
      td.tx_buffer = p;
      spi_device_transmit((spi_device_handle_t)_spi_dev, &td);
      p += n; left -= n;
    }
  }
  csHigh();

  // Bottom half
  setWindowY(_lcdH/2, _lcdH-1);
  dcCmd(); csLow();
  t.length = 8; t.tx_buffer = &cmd; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
  dcData();
  {
    const uint8_t* p = (const uint8_t*)((uintptr_t)_frame + _half_bytes);
    size_t left = _half_bytes;
    const size_t chunk = 4092; // safe chunk size across ESP32 variants
    while (left) {
      size_t n = (left > chunk) ? chunk : left;
      spi_transaction_t td = {};
      td.length = n * 8;
      td.tx_buffer = p;
      spi_device_transmit((spi_device_handle_t)_spi_dev, &td);
      p += n; left -= n;
    }
  }
  csHigh();

#elif ST77XX_HAVE_RP2040_DMA
  // RP2040 path: prefer SPI+DMA if initialized; otherwise fallback to generic SW/HW SPI
  if (_rp2040_spi != nullptr && _rp2040_dma_ch >= 0) {
    setWindowFull();
    const uint8_t ramwr = ramwrCmd();
    dcCmd(); csLow();
    // Send RAMWR command via SDK SPI
    spi_write_blocking((spi_inst_t*)_rp2040_spi, &ramwr, 1);
    dcData();
    // For flush() we use the SPI+DMA path; PIO is used for continuous refresh HW mode.
    rp2040DMASend((const uint8_t*)_frame, _frame_bytes);
    csHigh();
    // Single yield after full frame to keep system responsive
    yield();
  } else {
    if (_debug && !_flushModeLogged) { Serial.println(F("[ST77xx] RP2040 DMA not initialized; falling back to SW/HW SPI flush")); _flushModeLogged = true; }
    // Generic (RP2040, ESP8266, etc.): keep CS low across the whole frame
    // and stream bytes in chunks via HW/SW SPI without intermediate CS toggles.
    setWindowFull();
    const uint8_t ramwr = ramwrCmd();
    const uint8_t* p = (const uint8_t*)_frame;
    size_t left = _frame_bytes;
    const size_t chunk = 4096; // safe chunk size

    // Begin transfer
    dcCmd(); csLow();
    if (_cfg.use_hw_spi) {
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      SPI.transfer(ramwr);
      dcData();
      while (left) {
        size_t n = (left > chunk) ? chunk : left;
        for (size_t i = 0; i < n; ++i) SPI.transfer(p[i]);
        p += n; left -= n;
        // Yield to keep system responsive (ESP8266, others)
        yield();
      }
      SPI.endTransaction();
    } else {
      // Software SPI bit-bang
      // Send command byte
      sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      dcData();
      // Stream data bytes
      while (left) {
        size_t n = (left > chunk) ? chunk : left;
        sw_write_block(_cfg.sclk, _cfg.mosi, p, n, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
        p += n; left -= n;
        yield();
      }
    }
    csHigh();
  }

#else
  // Generic (RP2040, ESP8266, etc.): keep CS low across the whole frame
  // and stream bytes in chunks via HW/SW SPI without intermediate CS toggles.
  setWindowFull();
  const uint8_t ramwr = ramwrCmd();
  const uint8_t* p = (const uint8_t*)_frame;
  size_t left = _frame_bytes;
  const size_t chunk = 4096; // safe chunk size

  // Begin transfer
  dcCmd(); csLow();
  if (_cfg.use_hw_spi) {
    SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
    SPI.transfer(ramwr);
    dcData();
    while (left) {
      size_t n = (left > chunk) ? chunk : left;
      for (size_t i = 0; i < n; ++i) SPI.transfer(p[i]);
      p += n; left -= n;
      // Yield to keep system responsive (ESP8266, others)
      yield();
    }
    SPI.endTransaction();
  } else {
    // Software SPI bit-bang via helpers
    // Send command byte
    sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
    dcData();
    // Stream data bytes
    while (left) {
      size_t n = (left > chunk) ? chunk : left;
      sw_write_block(_cfg.sclk, _cfg.mosi, p, n, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      p += n; left -= n;
      yield();
    }
  }
  csHigh();
#endif
}

void ST77xxDMA::flushRect(int16_t x, int16_t y, int16_t w, int16_t h) {
  if (!_frame) return;
  if (!_begun) {
    if (_debug) Serial.println(F("[ST77xx][WARN] flushRect() called while not begun; ignoring"));
    return;
  }
  // Clip to screen bounds
  if (w <= 0 || h <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= (int16_t)_lcdW || y >= (int16_t)_lcdH) return;
  if (x + w > (int16_t)_lcdW) w = _lcdW - x;
  if (y + h > (int16_t)_lcdH) h = _lcdH - y;
  if (w <= 0 || h <= 0) return;

  // Set address window for the target rectangle
  setAddrWindow(x, y, x + w - 1, y + h - 1);

  const uint8_t ramwr = ramwrCmd();
  dcCmd(); csLow();
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  if (_cfg.use_hw_spi) {
    if (_spi_dev == nullptr) { if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 flushRect: _spi_dev is null; skipping")); csHigh(); return; }
    // Send RAMWR command
    spi_transaction_t t = {};
    t.length = 8; t.tx_buffer = &ramwr; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
    dcData();
    // Stream framebuffer bytes row-by-row
    for (int16_t yy = 0; yy < h; ++yy) {
      const uint8_t* row = (const uint8_t*)(_frame + (size_t)(y + yy) * _lcdW + x);
      size_t left = (size_t)w * 2;
      const size_t chunk = 4092; // safe chunk size
      while (left) {
        size_t n = (left > chunk) ? chunk : left;
        spi_transaction_t td = {};
        td.length = n * 8;
        td.tx_buffer = row;
        spi_device_transmit((spi_device_handle_t)_spi_dev, &td);
        row += n; left -= n;
      }
    }
  } else {
    // Software SPI on ESP32
    sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
    dcData();
    for (int16_t yy = 0; yy < h; ++yy) {
      const uint8_t* row = (const uint8_t*)(_frame + (size_t)(y + yy) * _lcdW + x);
      sw_write_block(_cfg.sclk, _cfg.mosi, row, (size_t)w * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
    }
  }
#elif ST77XX_HAVE_RP2040_DMA
  // RP2040: prefer SPI+DMA if initialized; otherwise fallback to generic HW/SW SPI
  if (_rp2040_spi != nullptr && _rp2040_dma_ch >= 0 && _cfg.use_hw_spi) {
    // Send RAMWR using SDK SPI, then DMA each row of the rectangle
    spi_write_blocking((spi_inst_t*)_rp2040_spi, &ramwr, 1);
    dcData();
    for (int16_t yy = 0; yy < h; ++yy) {
      const uint8_t* row = (const uint8_t*)(_frame + (size_t)(y + yy) * _lcdW + x);
      rp2040DMASend(row, (size_t)w * 2);
    }
  } else {
    if (_cfg.use_hw_spi) {
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      SPI.transfer(ramwr);
      dcData();
      for (int16_t yy = 0; yy < h; ++yy) {
        const uint8_t* row = (const uint8_t*)(_frame + (size_t)(y + yy) * _lcdW + x);
        for (size_t i = 0, n = (size_t)w * 2; i < n; ++i) SPI.transfer(row[i]);
      }
      SPI.endTransaction();
    } else {
      // Software SPI
      sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      dcData();
      for (int16_t yy = 0; yy < h; ++yy) {
        const uint8_t* row = (const uint8_t*)(_frame + (size_t)(y + yy) * _lcdW + x);
        sw_write_block(_cfg.sclk, _cfg.mosi, row, (size_t)w * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      }
    }
  }
#else
  // Generic path (ESP8266, AVR, etc.)
  if (_cfg.use_hw_spi) {
    SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
    SPI.transfer(ramwr);
    dcData();
    for (int16_t yy = 0; yy < h; ++yy) {
      const uint8_t* row = (const uint8_t*)(_frame + (size_t)(y + yy) * _lcdW + x);
      for (size_t i = 0, n = (size_t)w * 2; i < n; ++i) SPI.transfer(row[i]);
    }
    SPI.endTransaction();
  } else {
    // Software SPI
    sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
    dcData();
    for (int16_t yy = 0; yy < h; ++yy) {
      const uint8_t* row = (const uint8_t*)(_frame + (size_t)(y + yy) * _lcdW + x);
      sw_write_block(_cfg.sclk, _cfg.mosi, row, (size_t)w * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
    }
  }
#endif
  csHigh();
}

#if ST77XX_HAVE_RP2040_PIO
// ---- RP2040 PIO continuous refresh control & telemetry (proxies) ----
bool ST77xxDMA::startContinuousRefreshHW() {
  if (_rp2040_pio && _frame) {
    _rp2040_pio->startContinuousRefreshHW(_frame, _lcdW, _lcdH);
    _rp2040_hw_running = true;
    return true;
  }
  _rp2040_hw_running = false;
  return false;
}

void ST77xxDMA::stopContinuousRefreshHW() {
  _rp2040_hw_running = false;
}

uint32_t ST77xxDMA::hwFrameCount() const {
  if (_rp2040_pio) return _rp2040_pio->hwFrameCount;
  return 0;
}

void ST77xxDMA::hwFrameCountReset() {
  if (_rp2040_pio) _rp2040_pio->hwFrameCount = 0;
}

bool ST77xxDMA::isContinuousRefreshHW() const {
  return _rp2040_hw_running;
}
#endif

void ST77xxDMA::writePixelsDMA(const uint16_t* data_be, size_t count) {
  // count pixels in BE layout
  const uint8_t* p = (const uint8_t*)data_be;
  size_t bytes = count * 2;
  sendData(p, bytes);
}

// ------------ Примитивы и текст ------------
void ST77xxDMA::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if (x < 0 || y < 0 || x >= (int16_t)_lcdW || y >= (int16_t)_lcdH) return;
  if (_clipEnabled && (x < _clipX1 || y < _clipY1 || x > _clipX2 || y > _clipY2)) return;
  uint16_t cbe = ST77xxDMA::toBE(color);
  if (_frame) {
    _frame[(size_t)y * _lcdW + x] = cbe;
  } else {
    setAddrWindow(x, y, x, y);
    const uint8_t ramwr = ramwrCmd();
    dcCmd(); csLow();
    if (_cfg.use_hw_spi) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
      if (_spi_dev == nullptr) { if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 drawPixel: _spi_dev is null; skipping")); csHigh(); return; }
      spi_transaction_t t = {};
      t.length = 8; t.tx_buffer = &ramwr; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
      dcData();
      t = {};
      t.length = 16; t.tx_buffer = &cbe; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
#else
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      SPI.transfer(ramwr);
      dcData();
      const uint8_t* p = (const uint8_t*)&cbe;
      SPI.transfer(p[0]);
      SPI.transfer(p[1]);
      SPI.endTransaction();
#endif
    } else {
      // Software SPI
      sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      dcData();
      sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)&cbe, 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
    }
    csHigh();
  }
}

void ST77xxDMA::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  if (y < 0 || y >= (int16_t)_lcdH || w <= 0) return;
  if (_clipEnabled && (y < _clipY1 || y > _clipY2)) return;
  if (x < 0) { w += x; x = 0; }
  if (x + w > (int16_t)_lcdW) w = _lcdW - x;
  if (_clipEnabled) {
    if (x < _clipX1) { int16_t d = _clipX1 - x; x += d; w -= d; }
    int16_t maxX = _clipX2;
    if (x + w - 1 > maxX) w = maxX - x + 1;
  }
  if (w <= 0) return;
  uint16_t cbe = ST77xxDMA::toBE(color);
  if (_frame) {
    uint16_t* row = _frame + (size_t)y * _lcdW + x;
    for (int16_t i = 0; i < w; ++i) row[i] = cbe;
  } else {
    setAddrWindow(x, y, x + w - 1, y);
    const bool use_tx = (_txbuf && _txbuf_words > 0);
    const size_t tx_chunk = use_tx ? _txbuf_words : (size_t)128;
    uint16_t buf[128];
    if (!use_tx) { for (size_t i = 0; i < 128; ++i) buf[i] = cbe; }
    else { for (size_t i = 0; i < tx_chunk; ++i) _txbuf[i] = cbe; }
    int16_t left = w;
    const uint8_t ramwr = ramwrCmd();
    dcCmd(); csLow();
    if (_cfg.use_hw_spi) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
      if (_spi_dev == nullptr) { if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 drawFastHLine: _spi_dev is null; skipping")); csHigh(); return; }
      spi_transaction_t t = {};
      t.length = 8; t.tx_buffer = &ramwr; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
          t = {};
          t.length = (size_t)n * 16; t.tx_buffer = _txbuf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
          left -= n;
        }
      } else {
        while (left > 0) {
          int16_t n = left > 128 ? 128 : left;
          t = {};
          t.length = (size_t)n * 16; t.tx_buffer = buf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
          left -= n;
        }
      }
#else
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      SPI.transfer(ramwr);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
          const uint8_t* pb = (const uint8_t*)_txbuf;
          for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
          left -= n;
        }
      } else {
        while (left > 0) {
          int16_t n = left > 128 ? 128 : left;
          const uint8_t* pb = (const uint8_t*)buf;
          for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
          left -= n;
        }
      }
      SPI.endTransaction();
#endif
    } else {
      // Software SPI
      sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
          sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)_txbuf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
          left -= n;
        }
      } else {
        while (left > 0) {
          int16_t n = left > 128 ? 128 : left;
          sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)buf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
          left -= n;
        }
      }
    }
    csHigh();
  }
}

void ST77xxDMA::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  if (x < 0 || x >= (int16_t)_lcdW || h <= 0) return;
  if (_clipEnabled && (x < _clipX1 || x > _clipX2)) return;
  if (y < 0) { h += y; y = 0; }
  if (y + h > (int16_t)_lcdH) h = _lcdH - y;
  if (_clipEnabled) {
    if (y < _clipY1) { int16_t d = _clipY1 - y; y += d; h -= d; }
    int16_t maxY = _clipY2;
    if (y + h - 1 > maxY) h = maxY - y + 1;
  }
  if (h <= 0) return;
  uint16_t cbe = ST77xxDMA::toBE(color);
  if (_frame) {
    uint16_t* p = _frame + (size_t)y * _lcdW + x;
    for (int16_t i = 0; i < h; ++i) { *p = cbe; p += _lcdW; }
  } else {
    setAddrWindow(x, y, x, y + h - 1);
    const bool use_tx = (_txbuf && _txbuf_words > 0);
    const size_t tx_chunk = use_tx ? _txbuf_words : (size_t)128;
    uint16_t buf[128];
    if (!use_tx) { for (size_t i = 0; i < 128; ++i) buf[i] = cbe; }
    else { for (size_t i = 0; i < tx_chunk; ++i) _txbuf[i] = cbe; }
    int16_t left = h;
    const uint8_t ramwr = ramwrCmd();
    dcCmd(); csLow();
    if (_cfg.use_hw_spi) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
      if (_spi_dev == nullptr) { if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 drawFastVLine: _spi_dev is null; skipping")); csHigh(); return; }
      spi_transaction_t t = {};
      t.length = 8; t.tx_buffer = &ramwr; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
          t = {};
          t.length = (size_t)n * 16; t.tx_buffer = _txbuf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
          left -= n;
        }
      } else {
        while (left > 0) {
          int16_t n = left > 128 ? 128 : left;
          t = {};
          t.length = (size_t)n * 16; t.tx_buffer = buf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
          left -= n;
        }
      }
#else
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      SPI.transfer(ramwr);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
          const uint8_t* pb = (const uint8_t*)_txbuf;
          for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
          left -= n;
        }
      } else {
        while (left > 0) {
          int16_t n = left > 128 ? 128 : left;
          const uint8_t* pb = (const uint8_t*)buf;
          for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
          left -= n;
        }
      }
      SPI.endTransaction();
#endif
    } else {
      // Software SPI
      sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
          sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)_txbuf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
          left -= n;
        }
      } else {
        while (left > 0) {
          int16_t n = left > 128 ? 128 : left;
          sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)buf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
          left -= n;
        }
      }
    }
    csHigh();
  }
}

void ST77xxDMA::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int16_t err = dx + dy, e2;
  for (;;) {
    drawPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void ST77xxDMA::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w <= 0 || h <= 0) return;
  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y + h - 1, w, color);
  if (h > 2) {
    drawFastVLine(x, y + 1, h - 2, color);
    drawFastVLine(x + w - 1, y + 1, h - 2, color);
  }
}

void ST77xxDMA::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (w <= 0 || h <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= (int16_t)_lcdW || y >= (int16_t)_lcdH) return;
  if (x + w > (int16_t)_lcdW) w = _lcdW - x;
  if (y + h > (int16_t)_lcdH) h = _lcdH - y;
  if (_clipEnabled) {
    int16_t rx1 = x, ry1 = y, rx2 = x + w - 1, ry2 = y + h - 1;
    if (rx1 < _clipX1) rx1 = _clipX1;
    if (ry1 < _clipY1) ry1 = _clipY1;
    if (rx2 > _clipX2) rx2 = _clipX2;
    if (ry2 > _clipY2) ry2 = _clipY2;
    x = rx1; y = ry1; w = rx2 - rx1 + 1; h = ry2 - ry1 + 1;
    if (w <= 0 || h <= 0) return;
  }
  if (w <= 0 || h <= 0) return;
  uint16_t cbe = ST77xxDMA::toBE(color);
  if (_frame) {
    for (int16_t yy = 0; yy < h; ++yy) {
      uint16_t* row = _frame + (size_t)(y + yy) * _lcdW + x;
      for (int16_t xx = 0; xx < w; ++xx) row[xx] = cbe;
    }
  } else {
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    const bool use_tx = (_txbuf && _txbuf_words > 0);
    const size_t tx_chunk = use_tx ? _txbuf_words : (size_t)128;
    uint16_t buf[128];
    if (!use_tx) { for (size_t i = 0; i < 128; ++i) buf[i] = cbe; }
    else { for (size_t i = 0; i < tx_chunk; ++i) _txbuf[i] = cbe; }
    int32_t left = (int32_t)w * h;
    const uint8_t ramwr = ramwrCmd();
    dcCmd(); csLow();
    if (_cfg.use_hw_spi) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
      if (_spi_dev == nullptr) { if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 fillRect: _spi_dev is null; skipping")); csHigh(); return; }
      spi_transaction_t t = {};
      t.length = 8; t.tx_buffer = &ramwr; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int32_t n = left > (int32_t)tx_chunk ? (int32_t)tx_chunk : left;
          t = {};
          t.length = (size_t)n * 16; t.tx_buffer = _txbuf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
          left -= n;
        }
      } else {
        while (left > 0) {
          int32_t n = left > 128 ? 128 : left;
          t = {};
          t.length = (size_t)n * 16; t.tx_buffer = buf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
          left -= n;
        }
      }
#else
      SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
      SPI.transfer(ramwr);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int32_t n = left > (int32_t)tx_chunk ? (int32_t)tx_chunk : left;
          const uint8_t* pb = (const uint8_t*)_txbuf;
          for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
          left -= n;
        }
      } else {
        while (left > 0) {
          int32_t n = left > 128 ? 128 : left;
          const uint8_t* pb = (const uint8_t*)buf;
          for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
          left -= n;
        }
      }
      SPI.endTransaction();
#endif
    } else {
      // Software SPI
      sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
      dcData();
      if (use_tx) {
        while (left > 0) {
          int32_t n = left > (int32_t)tx_chunk ? (int32_t)tx_chunk : left;
          sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)_txbuf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
          left -= n;
        }
      } else {
        while (left > 0) {
          int32_t n = left > 128 ? 128 : left;
          sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)buf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
          left -= n;
        }
      }
    }
    csHigh();
  }
}

void ST77xxDMA::fillScreen(uint16_t color) {
  fillRect(0, 0, _lcdW, _lcdH, color);
}

void ST77xxDMA::drawRGBBitmap(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* rgb565_le) {
  if (w <= 0 || h <= 0) return;
  // Клип к экрану (без clip-rect тут, так как drawPixel уже учитывает)
  for (int16_t yy = 0; yy < h; ++yy) {
    int16_t cy = y + yy;
    if (cy < 0 || cy >= (int16_t)_lcdH) continue;
    // Apply clip-rect on Y
    if (_clipEnabled && (cy < _clipY1 || cy > _clipY2)) continue;

    int16_t runX = x;
    int16_t runW = w;
    if (runX < 0) { runW += runX; runX = 0; }
    if (runX + runW > (int16_t)_lcdW) runW = _lcdW - runX;
    // Apply clip-rect on X
    if (_clipEnabled) {
      if (runX < _clipX1) { int16_t d = _clipX1 - runX; runX += d; runW -= d; }
      int16_t maxX = _clipX2;
      if (runX + runW - 1 > maxX) runW = maxX - runX + 1;
    }
    if (runW <= 0) continue;
    if (_frame) {
      uint16_t* dst = _frame + (size_t)cy * _lcdW + runX;
      const uint16_t* src = rgb565_le + (size_t)yy * w + (runX - x);
      for (int16_t i = 0; i < runW; ++i) dst[i] = toBE(src[i]);
    } else {
      setAddrWindow(runX, cy, runX + runW - 1, cy);
      const uint16_t* src = rgb565_le + (size_t)yy * w + (runX - x);
      const bool use_tx = (_txbuf && _txbuf_words > 0);
      const size_t tx_chunk = use_tx ? _txbuf_words : (size_t)128;
      uint16_t buf[128];
      int16_t left = runW;
      const uint8_t ramwr = ramwrCmd();
      dcCmd(); csLow();
      if (_cfg.use_hw_spi) {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        if (_spi_dev == nullptr) { if (_debug) Serial.println(F("[ST77xx][ERR] ESP32 drawRGBBitmap row: _spi_dev is null; skipping")); csHigh(); return; }
        spi_transaction_t t = {};
        t.length = 8; t.tx_buffer = &ramwr; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
        dcData();
        if (use_tx) {
          while (left > 0) {
            int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
            for (int16_t i = 0; i < n; ++i) _txbuf[i] = toBE(src[i]);
            t = {};
            t.length = (size_t)n * 16; t.tx_buffer = _txbuf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
            src += n; left -= n;
          }
        } else {
          while (left > 0) {
            int16_t n = left > 128 ? 128 : left;
            for (int16_t i = 0; i < n; ++i) buf[i] = toBE(src[i]);
            t = {};
            t.length = (size_t)n * 16; t.tx_buffer = buf; spi_device_polling_transmit((spi_device_handle_t)_spi_dev, &t);
            src += n; left -= n;
          }
        }
#else
        SPI.beginTransaction(SPISettings(_cfg.spi_hz, MSBFIRST, SPI_MODE0));
        SPI.transfer(ramwr);
        dcData();
        if (use_tx) {
          while (left > 0) {
            int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
            for (int16_t i = 0; i < n; ++i) _txbuf[i] = toBE(src[i]);
            const uint8_t* pb = (const uint8_t*)_txbuf;
            for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
            src += n; left -= n;
          }
        } else {
          while (left > 0) {
            int16_t n = left > 128 ? 128 : left;
            for (int16_t i = 0; i < n; ++i) buf[i] = toBE(src[i]);
            const uint8_t* pb = (const uint8_t*)buf;
            for (int32_t i = 0; i < (int32_t)n * 2; ++i) SPI.transfer(pb[i]);
            src += n; left -= n;
          }
        }
        SPI.endTransaction();
#endif
      } else {
        // Software SPI
        sw_write_byte(_cfg.sclk, _cfg.mosi, ramwr, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
        dcData();
        if (use_tx) {
          while (left > 0) {
            int16_t n = left > (int16_t)tx_chunk ? (int16_t)tx_chunk : left;
            for (int16_t i = 0; i < n; ++i) _txbuf[i] = toBE(src[i]);
            sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)_txbuf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
            src += n; left -= n;
          }
        } else {
          while (left > 0) {
            int16_t n = left > 128 ? 128 : left;
            for (int16_t i = 0; i < n; ++i) buf[i] = toBE(src[i]);
            sw_write_block(_cfg.sclk, _cfg.mosi, (const uint8_t*)buf, (size_t)n * 2, _cfg.sw_spi_use_asm, _cfg.sw_spi_direct_gpio);
            src += n; left -= n;
          }
        }
      }
      csHigh();
    }
  }
}

void ST77xxDMA::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  if (r <= 0) return;
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  drawPixel(x0, y0 + r, color);
  drawPixel(x0, y0 - r, color);
  drawPixel(x0 + r, y0, color);
  drawPixel(x0 - r, y0, color);
  while (x < y) {
    if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
    x++; ddF_x += 2; f += ddF_x;
    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void ST77xxDMA::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  if (r <= 0) return;
  drawFastVLine(x0, y0 - r, 2 * r + 1, color);
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  while (x < y) {
    if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
    x++; ddF_x += 2; f += ddF_x;
    drawFastVLine(x0 + x, y0 - y, 2 * y + 1, color);
    drawFastVLine(x0 - x, y0 - y, 2 * y + 1, color);
    drawFastVLine(x0 + y, y0 - x, 2 * x + 1, color);
    drawFastVLine(x0 - y, y0 - x, 2 * x + 1, color);
  }
}

static inline void _swap_int16(int16_t& a, int16_t& b){ int16_t t=a; a=b; b=t; }

void ST77xxDMA::drawTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t color) {
  drawLine(x0,y0,x1,y1,color);
  drawLine(x1,y1,x2,y2,color);
  drawLine(x2,y2,x0,y0,color);
}

void ST77xxDMA::fillTriangle(int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t x2,int16_t y2,uint16_t color) {
  if (y0 > y1) { _swap_int16(y0,y1); _swap_int16(x0,x1);} 
  if (y1 > y2) { _swap_int16(y1,y2); _swap_int16(x1,x2);} 
  if (y0 > y1) { _swap_int16(y0,y1); _swap_int16(x0,x1);} 
  auto interp = [](int16_t x0,int16_t y0,int16_t x1,int16_t y1,int16_t y){
    if (y1==y0) return x0;
    return (int16_t)(x0 + (int32_t)(x1 - x0) * (y - y0) / (y1 - y0));
  };
  int16_t y;
  for (y = y0; y <= y1; ++y) {
    int16_t xa = interp(x0,y0,x2,y2,y);
    int16_t xb = interp(x0,y0,x1,y1,y);
    if (xa > xb) _swap_int16(xa, xb);
    drawFastHLine(xa, y, xb - xa + 1, color);
  }
  for (; y <= y2; ++y) {
    int16_t xa = interp(x0,y0,x2,y2,y);
    int16_t xb = interp(x1,y1,x2,y2,y);
    if (xa > xb) _swap_int16(xa, xb);
    drawFastHLine(xa, y, xb - xa + 1, color);
  }
}

void ST77xxDMA::drawRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t color) {
  if (w <= 0 || h <= 0) return;
  if (r < 0) r = 0;
  if (r > w/2) r = w/2;
  if (r > h/2) r = h/2;
  // sides
  drawFastHLine(x + r, y, w - 2*r, color);
  drawFastHLine(x + r, y + h - 1, w - 2*r, color);
  drawFastVLine(x, y + r, h - 2*r, color);
  drawFastVLine(x + w - 1, y + r, h - 2*r, color);
  // corners
  int16_t x0 = 0, y0 = r; int16_t f = 1 - r, ddF_x = 1, ddF_y = -2*r;
  while (x0 < y0) {
    if (f >= 0) { y0--; ddF_y += 2; f += ddF_y; }
    x0++; ddF_x += 2; f += ddF_x;
    drawPixel(x + r - y0, y + r - x0, color);
    drawPixel(x + w - r - 1 + y0, y + r - x0, color);
    drawPixel(x + r - y0, y + h - r - 1 + x0, color);
    drawPixel(x + w - r - 1 + y0, y + h - r - 1 + x0, color);
  }
}

void ST77xxDMA::fillRoundRect(int16_t x,int16_t y,int16_t w,int16_t h,int16_t r,uint16_t color) {
  if (w <= 0 || h <= 0) return;
  if (r < 0) r = 0;
  if (r > w/2) r = w/2;
  if (r > h/2) r = h/2;
  fillRect(x + r, y, w - 2*r, h, color);
  int16_t x0 = 0, y0 = r; int16_t f = 1 - r, ddF_x = 1, ddF_y = -2*r;
  while (x0 < y0) {
    if (f >= 0) { y0--; ddF_y += 2; f += ddF_y; }
    x0++; ddF_x += 2; f += ddF_x;
    drawFastHLine(x + r - y0, y + r - x0, y0*2 + (w - 2*r), color);
    drawFastHLine(x + r - y0, y + h - r - 1 + x0, y0*2 + (w - 2*r), color);
  }
}

// legacy text/font implementation was consolidated earlier with font registry