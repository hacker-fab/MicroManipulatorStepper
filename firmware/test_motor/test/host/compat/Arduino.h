// MIT License — HackerFab Open Micro-Manipulator
// Minimal Arduino compatibility shim for host-native unit tests.
// Provides only the subset of the Arduino API used by test_motor source files.
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

// ---- SPI constants ---------------------------------------------------------
#define MSBFIRST  1
#define SPI_MODE3 3

// ---- Timing (no-ops on host) -----------------------------------------------
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned long) {}
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }

// ---- GPIO stubs ------------------------------------------------------------
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return LOW; }

// ---- Serial stub -----------------------------------------------------------
struct SerialClass {
    void begin(unsigned long) {}
    explicit operator bool() const { return true; }
    template <typename T> void print(T)   {}
    template <typename T> void print(T, int) {}
    template <typename T> void println(T) {}
    void println() {}
};

inline SerialClass Serial;
