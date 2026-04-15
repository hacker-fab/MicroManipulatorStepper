// =============================================================================
// Arduino.h — Compatibility shim for Pico SDK CMake builds (test_motor)
// Provides the subset of Arduino API used by test_motor programs.
// =============================================================================
#pragma once

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <algorithm>

// ---- Math constants --------------------------------------------------------
#ifndef PI
#define PI M_PI
#endif
#ifndef HALF_PI
#define HALF_PI (M_PI / 2.0)
#endif
#ifndef TWO_PI
#define TWO_PI (2.0 * M_PI)
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif

// ---- Timing ----------------------------------------------------------------
inline void delay(unsigned long ms) { sleep_ms(ms); }
inline void delayMicroseconds(unsigned long us) { busy_wait_us(us); }
inline unsigned long millis() { return to_ms_since_boot(get_absolute_time()); }
inline unsigned long micros() { return static_cast<unsigned long>(time_us_64()); }

// ---- GPIO ------------------------------------------------------------------
#ifndef INPUT
#define INPUT  0
#endif
#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW  0
#endif

inline void pinMode(uint pin, uint mode) {
    gpio_init(pin);
    gpio_set_dir(pin, mode == OUTPUT);
}

inline void digitalWrite(uint pin, uint val) {
    gpio_put(pin, val);
}

inline int digitalRead(uint pin) {
    return gpio_get(pin) ? HIGH : LOW;
}

// ---- Format bases ----------------------------------------------------------
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

// ---- LED_BUILTIN -----------------------------------------------------------
#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// ---- Arch defines ----------------------------------------------------------
#define ARDUINO_ARCH_RP2040 1

// ---- Serial ----------------------------------------------------------------
class SerialClass {
public:
    void begin(unsigned long baud) {
        (void)baud;
        m_initialized = true;
    }

    int available() {
        if (m_buffered >= 0) return 1;
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) { m_buffered = c; return 1; }
        return 0;
    }

    int read() {
        if (m_buffered >= 0) { int c = m_buffered; m_buffered = -1; return c; }
        int c = getchar_timeout_us(0);
        return (c == PICO_ERROR_TIMEOUT) ? -1 : c;
    }

    size_t write(uint8_t c) { putchar_raw(c); return 1; }

    void print(const char* s)     { ::printf("%s", s); }
    void print(int v)             { ::printf("%d", v); }
    void print(unsigned int v)    { ::printf("%u", v); }
    void print(long v)            { ::printf("%ld", v); }
    void print(unsigned long v)   { ::printf("%lu", v); }
    void print(float v, int prec = 2) {
        char fmt[16];
        snprintf(fmt, sizeof(fmt), "%%.%df", prec);
        ::printf(fmt, (double)v);
    }
    void print(double v)          { ::printf("%f", v); }
    void print(int32_t v, int base) {
        if (base == HEX) ::printf("%X", (unsigned)v);
        else ::printf("%d", v);
    }
    void print(uint8_t v, int base) {
        if (base == HEX) ::printf("%02X", v);
        else ::printf("%u", v);
    }

    void println()                   { putchar_raw('\n'); }
    void println(const char* s)      { ::printf("%s\n", s); }
    void println(int v)              { ::printf("%d\n", v); }
    void println(unsigned int v)     { ::printf("%u\n", v); }
    void println(long v)             { ::printf("%ld\n", v); }
    void println(unsigned long v)    { ::printf("%lu\n", v); }
    void println(float v, int prec = 2) { print(v, prec); putchar_raw('\n'); }
    void println(double v)           { ::printf("%f\n", v); }
    void println(uint8_t v, int b)   { print(v, b); putchar_raw('\n'); }
    void println(int32_t v, int b)   { print(v, b); putchar_raw('\n'); }

    int printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        va_list args;
        va_start(args, fmt);
        int ret = vprintf(fmt, args);
        va_end(args);
        return ret;
    }

    explicit operator bool() const { return m_initialized; }

private:
    bool m_initialized = false;
    int  m_buffered    = -1;
};

extern SerialClass Serial;
