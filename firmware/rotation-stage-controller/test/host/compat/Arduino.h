// MIT License — HackerFab Open Micro-Manipulator
// Minimal Arduino compatibility shim for host-native unit tests of the
// rotation-stage-controller firmware (STM32 / STM32Duino target).
// Provides only the subset of the Arduino API used by temperature.cpp and vacuum.cpp.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

using byte = uint8_t;

// ---- GPIO constants --------------------------------------------------------
#define INPUT   0
#define OUTPUT  1
#define HIGH    1
#define LOW     0

// ---- STM32Duino pin aliases used by config.h -------------------------------
// These must be defined before config.h is processed.
#define PA0  0
#define PA1  1
#define PA2  2
#define PA3  3
#define PA4  4
#define PA5  5
#define PA6  6
#define PA7  7
#define PA8  8
#define PA9  9
#define PA10 10
#define PA11 11
#define PA12 12
#define PA13 13
#define PA14 14
#define PA15 15

// PA_x variants (STM32Duino underscore style used in config.h)
#define PA_0  PA0
#define PA_1  PA1
#define PA_2  PA2
#define PA_3  PA3

#define PB0  16
#define PB1  17
#define PB2  18
#define PB3  19
#define PB4  20
#define PB5  21
#define PB6  22
#define PB7  23
#define PB8  24
#define PB9  25

// ---- PWM resolution constant (STM32Duino) — only used as a numeric tag ----
#define RESOLUTION_12B_COMPARE_FORMAT 12

// ---- Constrain macro (identical to Arduino's) ------------------------------
#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// ---- Analog stubs ----------------------------------------------------------
// analogRead always returns 0 so tests can exercise the arithmetic paths
// without real ADC hardware.
inline int analogRead(int /*pin*/) { return 0; }
inline void analogWrite(int /*pin*/, int /*value*/) {}

// ---- GPIO stubs ------------------------------------------------------------
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return LOW; }

// ---- Timing (no-ops on host) -----------------------------------------------
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned long) {}
inline unsigned long millis() { return 0; }

// ---- Serial stub -----------------------------------------------------------
struct SerialClass {
    void begin(unsigned long) {}
    explicit operator bool() const { return true; }
    template <typename T> void print(T) {}
    template <typename T> void print(T, int) {}
    template <typename T> void println(T) {}
    void println() {}
};

inline SerialClass Serial;
