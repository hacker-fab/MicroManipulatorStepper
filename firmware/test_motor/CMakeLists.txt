# =============================================================================
# test_motor — CMake Build
# Target: Raspberry Pi Pico 2 (RP2350), Pico SDK
#
# Builds 6 independent test executables:
#   blink, encoder, stepper, stepper_and_encoder, one_shot_encoder, peripheral_test
# =============================================================================
cmake_minimum_required(VERSION 3.14)

set(PICO_BOARD pico2 CACHE STRING "Target board")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Target platform")

include(cmake/pico_sdk_import.cmake)

project(test_motor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# --- Arduino compatibility shim ---
add_library(arduino_compat STATIC
    src/compat/arduino_main.cpp
)
target_include_directories(arduino_compat PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/compat
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(arduino_compat PUBLIC
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_sync
)

# --- Helper function to add a test executable ---
function(add_test_firmware NAME SOURCE_FILE)
    add_executable(${NAME} ${SOURCE_FILE})
    target_include_directories(${NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/compat
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(${NAME} PRIVATE
        arduino_compat
        pico_stdlib
        hardware_spi
        hardware_i2c
        hardware_pwm
        hardware_gpio
        hardware_sync
    )
    pico_enable_stdio_usb(${NAME} 1)
    pico_enable_stdio_uart(${NAME} 0)
    set_target_properties(${NAME} PROPERTIES SUFFIX ".elf")
    pico_add_extra_outputs(${NAME})
endfunction()

# --- Test executables ---
add_test_firmware(blink                 src/blink.cpp)
add_test_firmware(encoder               src/encoder.cpp)
add_test_firmware(stepper               src/stepper.cpp)
add_test_firmware(stepper_and_encoder   src/stepper_and_encoder.cpp)
add_test_firmware(one_shot_encoder      src/one_shot_encoder.cpp)
add_test_firmware(peripheral_test       src/peripheral_test.cpp)
