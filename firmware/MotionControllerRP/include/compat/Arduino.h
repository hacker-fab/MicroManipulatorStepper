// =============================================================================
// Arduino.h — Compatibility shim for Pico SDK CMake builds
// Provides the subset of Arduino API used by MotionControllerRP.
// Under PlatformIO/Arduino builds this file is never included (the real
// Arduino.h takes priority via the framework's include path).
// =============================================================================
#pragma once

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <algorithm>

// ---- Math constants (Arduino defines) --------------------------------------

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

// ---- GPIO constants --------------------------------------------------------

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

// ---- Format bases for print() ----------------------------------------------

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

// ---- Serial ----------------------------------------------------------------

class SerialClass {
public:
    void begin(unsigned long baud) {
        (void)baud;
        // stdio_init_all() is called from the CMake entry point before setup()
        m_initialized = true;
    }

    int available() {
        if (m_buffered >= 0) return 1;
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            m_buffered = c;
            return 1;
        }
        return 0;
    }

    int read() {
        if (m_buffered >= 0) {
            int c = m_buffered;
            m_buffered = -1;
            return c;
        }
        int c = getchar_timeout_us(0);
        return (c == PICO_ERROR_TIMEOUT) ? -1 : c;
    }

    size_t write(uint8_t c) {
        putchar_raw(c);
        return 1;
    }

    size_t write(const char* str) {
        if (!str) return 0;
        size_t len = strlen(str);
        for (size_t i = 0; i < len; i++) putchar_raw(str[i]);
        return len;
    }

    size_t write(const uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; i++) putchar_raw(buf[i]);
        return len;
    }

    // --- print family ---

    void print(const char* s)     { ::printf("%s", s); }
    void print(int v)             { ::printf("%d", v); }
    void print(unsigned int v)    { ::printf("%u", v); }
    void print(long v)            { ::printf("%ld", v); }
    void print(unsigned long v)   { ::printf("%lu", v); }
    void print(float v)           { ::printf("%f", (double)v); }
    void print(double v)          { ::printf("%f", v); }

    void print(uint8_t val, int base) {
        switch (base) {
            case BIN:
                for (int i = 7; i >= 0; i--) putchar_raw('0' + ((val >> i) & 1));
                break;
            case HEX:
                ::printf("%02X", val);
                break;
            default:
                ::printf("%u", val);
                break;
        }
    }

    void println()                   { putchar_raw('\n'); }
    void println(const char* s)      { ::printf("%s\n", s); }
    void println(int v)              { ::printf("%d\n", v); }
    void println(unsigned int v)     { ::printf("%u\n", v); }
    void println(uint8_t v, int b)   { print(v, b); putchar_raw('\n'); }

    int printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        va_list args;
        va_start(args, fmt);
        int ret = vprintf(fmt, args);
        va_end(args);
        return ret;
    }

    // `while (!Serial)` — returns true once begin() has been called.
    // Under USB CDC the host connection state is checked at the executable
    // level via pico_enable_stdio_usb; this shim just tracks init state.
    explicit operator bool() const {
        return m_initialized;
    }

private:
    bool m_initialized = false;
    int  m_buffered    = -1;
};

extern SerialClass Serial;

// ---- String type alias (minimal) -------------------------------------------
// The project uses std::string, not Arduino String, so this is not needed.
// If needed: #include <string> and typedef std::string String;
