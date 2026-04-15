// Arduino compat globals + main() entry point for test_motor CMake builds
#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "pico/stdlib.h"

SerialClass Serial;
SPIClass SPI;
TwoWire Wire;

// Each test program provides its own setup() / loop()
extern void setup();
extern void loop();

int main() {
    stdio_init_all();
    setup();
    for (;;) loop();
    return 0;
}
