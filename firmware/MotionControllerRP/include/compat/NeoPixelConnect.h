// =============================================================================
// NeoPixelConnect.h — Stub for CMake builds (no-op LED driver)
// The real NeoPixelConnect library is only used via PlatformIO.
// =============================================================================
#pragma once

#include <cstdint>

class NeoPixelConnect {
public:
    NeoPixelConnect(uint8_t /*pin*/, uint16_t /*count*/) {}

    void neoPixelSetValue(uint16_t /*idx*/, uint8_t /*r*/, uint8_t /*g*/,
                          uint8_t /*b*/, bool /*show*/ = false) {}
    void neoPixelShow() {}
    void neoPixelClear(bool /*show*/ = false) {}
    void neoPixelFill(uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/,
                      bool /*show*/ = false) {}
};
