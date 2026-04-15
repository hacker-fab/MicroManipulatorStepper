// =============================================================================
// arduino_compat.cpp — Global object definitions for the Arduino compat shim.
// Provides the `Serial` and `Wire` singletons expected by existing source.
// =============================================================================

#ifndef ARDUINO  // Only active for CMake/Pico SDK builds

#include "Arduino.h"
#include "Wire.h"

SerialClass Serial;
TwoWire     Wire;

#endif // ARDUINO
