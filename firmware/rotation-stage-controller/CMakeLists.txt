# =============================================================================
# rotation-stage-controller — CMake Build
# Target: STM32F103C8 (Blue Pill), Cortex-M3
#
# This project heavily uses STM32 Arduino framework APIs (analogReadResolution,
# analogWriteFrequency, pwm_start, INPUT_FLOATING, etc.). The CMake build uses
# the STM32Cube HAL via stm32-cmake for bare-metal compilation.
#
# Prerequisites:
#   - arm-none-eabi-gcc toolchain
#   - STM32_CUBE_F1_PATH set to STM32CubeF1 SDK, OR it will be fetched
#
# For now, this CMakeLists.txt provides the project skeleton. The Arduino
# framework APIs used by this firmware require either:
#   (a) STM32duino core ported to CMake, or
#   (b) Rewriting the source to use STM32 HAL directly
#
# Until one of those is completed, use PlatformIO for this target:
#   cd firmware/rotation-stage-controller && pio run
# =============================================================================
cmake_minimum_required(VERSION 3.14)

# --- Cross-compilation toolchain ---
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

find_program(ARM_CC arm-none-eabi-gcc REQUIRED)
find_program(ARM_CXX arm-none-eabi-g++ REQUIRED)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(ARM_SIZE arm-none-eabi-size REQUIRED)

set(CMAKE_C_COMPILER ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})
set(CMAKE_ASM_COMPILER ${ARM_CC})
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(rotation_stage_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- MCU flags for STM32F103C8 (Cortex-M3, 64KB Flash, 20KB RAM) ---
set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MCU_FLAGS} -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} -specs=nosys.specs -specs=nano.specs -Wl,--gc-sections")

# --- Fetch STM32CubeF1 HAL ---
include(FetchContent)
FetchContent_Declare(stm32cubef1
    GIT_REPOSITORY https://github.com/STMicroelectronics/STM32CubeF1.git
    GIT_TAG        v1.8.5
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)

option(FETCH_STM32_HAL "Download STM32CubeF1 HAL (large, ~500MB)" OFF)

if(FETCH_STM32_HAL)
    FetchContent_MakeAvailable(stm32cubef1)

    set(HAL_DIR ${stm32cubef1_SOURCE_DIR}/Drivers)

    # --- HAL/LL sources (minimal set) ---
    set(HAL_SOURCES
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
    )

    add_library(stm32_hal STATIC ${HAL_SOURCES})
    target_include_directories(stm32_hal PUBLIC
        ${HAL_DIR}/STM32F1xx_HAL_Driver/Inc
        ${HAL_DIR}/CMSIS/Device/ST/STM32F1xx/Include
        ${HAL_DIR}/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_compile_definitions(stm32_hal PUBLIC
        STM32F103xB
        USE_HAL_DRIVER
    )

    # --- Firmware executable ---
    add_executable(rotation_stage
        src/main.cpp
        src/rotation.cpp
        src/temperature.cpp
        src/vacuum.cpp
        src/i2c_parser.cpp
    )

    target_include_directories(rotation_stage PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(rotation_stage PRIVATE stm32_hal)
    set_target_properties(rotation_stage PROPERTIES SUFFIX ".elf")

    # Generate .bin and .hex
    add_custom_command(TARGET rotation_stage POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O binary $<TARGET_FILE:rotation_stage> ${CMAKE_BINARY_DIR}/rotation_stage.bin
        COMMAND ${ARM_OBJCOPY} -O ihex   $<TARGET_FILE:rotation_stage> ${CMAKE_BINARY_DIR}/rotation_stage.hex
        COMMAND ${ARM_SIZE} $<TARGET_FILE:rotation_stage>
        COMMENT "Generating .bin and .hex"
    )
else()
    message(STATUS "")
    message(STATUS "=================================================================")
    message(STATUS "rotation-stage-controller: STM32 HAL fetch disabled.")
    message(STATUS "")
    message(STATUS "This firmware uses STM32 Arduino APIs that require either:")
    message(STATUS "  1. STM32 HAL + Arduino compat layer (pass -DFETCH_STM32_HAL=ON)")
    message(STATUS "  2. PlatformIO: cd firmware/rotation-stage-controller && pio run")
    message(STATUS "=================================================================")
    message(STATUS "")
endif()
