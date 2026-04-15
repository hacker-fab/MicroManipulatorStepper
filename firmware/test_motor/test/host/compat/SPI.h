// MIT License — HackerFab Open Micro-Manipulator
// Minimal SPI compatibility shim for host-native unit tests.
#pragma once

#include <cstdint>
#include <cstddef>

struct SPISettings {
    SPISettings() = default;
    SPISettings(uint32_t /*clock*/, int /*bitOrder*/, int /*mode*/) {}
};

struct SPIClass {
    void begin() {}
    void end()   {}
    void beginTransaction(const SPISettings&) {}
    void endTransaction() {}
    // RP2040 / Arduino-Pico pin-assignment extensions
    void setSCK(int /*pin*/) {}
    void setRX(int /*pin*/)  {}
    void setTX(int /*pin*/)  {}
    void setCS(int /*pin*/)  {}
    uint8_t transfer(uint8_t) { return 0; }
    void transfer(uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; ++i) buf[i] = 0;
    }
};

inline SPIClass SPI;
