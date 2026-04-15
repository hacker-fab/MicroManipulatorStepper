# =============================================================================
# MotionControllerRP — CMake Build
# Target: Raspberry Pi Pico 2 (RP2350), Pico SDK
# =============================================================================
cmake_minimum_required(VERSION 3.14)

# Board selection — must be set BEFORE project() so the toolchain file picks
# up the correct platform (RP2350) and compiler triple.
set(PICO_BOARD pico2 CACHE STRING "Target board")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Target platform")

# Import Pico SDK (must come before project())
include(cmake/pico_sdk_import.cmake)

project(MotionControllerRP C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# --- LittleFS C library (fetched from upstream) ---
include(FetchContent)
FetchContent_Declare(littlefs
    GIT_REPOSITORY https://github.com/littlefs-project/littlefs.git
    GIT_TAG        v2.9.3
)
FetchContent_MakeAvailable(littlefs)

add_library(littlefs_lib STATIC
    ${littlefs_SOURCE_DIR}/lfs.c
    ${littlefs_SOURCE_DIR}/lfs_util.c
)
target_include_directories(littlefs_lib PUBLIC ${littlefs_SOURCE_DIR})

# --- Arduino compatibility shim library ---
add_library(arduino_compat STATIC
    src/compat/arduino_compat.cpp
    src/compat/pico_littlefs.cpp
)
target_include_directories(arduino_compat PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/compat
)
target_link_libraries(arduino_compat PUBLIC
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_flash
    hardware_sync
    littlefs_lib
)
# Filesystem layout: last 0.5 MB of flash reserved for LittleFS
target_compile_definitions(arduino_compat PRIVATE
    LFS_FLASH_OFFSET=0x180000
    LFS_FLASH_SIZE=0x80000
    FLASH_SECTOR_SIZE=4096
    FLASH_PAGE_SIZE=256
)

# --- Main firmware executable ---
add_executable(motion_controller
    src/compat/arduino_entry.cpp
    src/main.cpp
    src/robot.cpp
    src/demo_gcode_generator.cpp
    src/command_parser/command_parser.cpp
    src/hardware/MT6701_encoder.cpp
    src/hardware/MT6835_encoder.cpp
    src/hardware/spi_interface.cpp
    src/hardware/TB6612_motor_driver.cpp
    src/kinematic_models/kinematic_model_base.cpp
    src/kinematic_models/kinematic_model_delta3d.cpp
    src/motion_control/motion_controller.cpp
    src/motion_control/path_planner.cpp
    src/motion_control/path_segment.cpp
    src/peripheral_control/peripheral.cpp
    src/robot_joint/robot_joint.cpp
    src/robot_tool/pwm_tool.cpp
    src/servo_control/actuator_calibration.cpp
    src/servo_control/homing_controller.cpp
    src/servo_control/pid.cpp
    src/servo_control/servo_controller.cpp
    src/utilities/logging.cpp
    src/utilities/lookup_table.cpp
    src/utilities/utilities.cpp
    src/utilities/waveforms.cpp
)

target_include_directories(motion_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/compat
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motion_controller PRIVATE
    arduino_compat
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_i2c
    hardware_pwm
    hardware_timer
    hardware_clocks
    hardware_vreg
    hardware_sync
    hardware_dma
    hardware_gpio
    hardware_pll
    hardware_flash
    hardware_pio
    pico_unique_id
)

# USB serial output (not UART)
pico_enable_stdio_usb(motion_controller 1)
pico_enable_stdio_uart(motion_controller 0)

# Ensure the ELF has the .elf extension (required for picotool post-processing)
set_target_properties(motion_controller PROPERTIES SUFFIX ".elf")

# Generate .uf2 for drag-and-drop upload
pico_add_extra_outputs(motion_controller)

# Flash clock divider for 250 MHz overclock
target_compile_definitions(motion_controller PRIVATE
    PICO_FLASH_SPI_CLKDIV=4
    CFG_TUD_CDC_RX_BUFSIZE=256
    CFG_TUD_CDC_TX_BUFSIZE=256
)
