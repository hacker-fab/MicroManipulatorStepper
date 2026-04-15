// =============================================================================
// arduino_entry.cpp — Provides main() for Pico SDK CMake builds.
// Calls the Arduino-style setup() once then loop() forever, exactly like
// the Arduino framework does.  Under PlatformIO the real Arduino core
// supplies main(), so this file compiles to nothing.
// =============================================================================

#ifndef ARDUINO  // Only active for CMake/Pico SDK builds

#include "pico/stdlib.h"

// Defined in the existing main.cpp
extern void setup();
extern void loop();

int main() {
    stdio_init_all();
    setup();
    for (;;) {
        loop();
    }
    return 0;
}

#endif // ARDUINO
