# =============================================================================
# MotionControllerRP — Host-native unit tests with GoogleTest
# This is a separate CMake project for host compilation (not cross-compiled).
# =============================================================================
cmake_minimum_required(VERSION 3.14)

project(motion_controller_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# Firmware modules that are pure C++ (no hardware/Pico/Arduino dependencies)
set(FIRMWARE_SOURCES
    ${SRC_DIR}/command_parser/command_parser.cpp
    ${SRC_DIR}/servo_control/pid.cpp
    ${SRC_DIR}/utilities/waveforms.cpp
)

# Explicitly list real test files (excludes placeholder smoke_test.cpp)
set(TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_gcode_command.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_math3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ringbuffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_waveforms.cpp
)

if(TEST_SOURCES)
    add_executable(motion_controller_tests
        ${TEST_SOURCES}
        ${FIRMWARE_SOURCES}
    )
    target_include_directories(motion_controller_tests PRIVATE
        ${SRC_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    )
    target_link_libraries(motion_controller_tests PRIVATE
        GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(motion_controller_tests)
endif()
