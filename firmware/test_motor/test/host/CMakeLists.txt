# =============================================================================
# test_motor — Host-native unit tests with GoogleTest
# This is a separate CMake project for host compilation (not cross-compiled).
# =============================================================================
cmake_minimum_required(VERSION 3.14)

project(test_motor_tests CXX)
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

# Firmware source files that are pure C++ (hardware deps satisfied by compat stubs)
set(FIRMWARE_SOURCES
    ${SRC_DIR}/encoder.cpp
)

# Explicitly list real test files (excludes placeholder smoke_test.cpp)
set(TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_encoder.cpp
)

if(TEST_SOURCES)
    add_executable(test_motor_tests ${TEST_SOURCES} ${FIRMWARE_SOURCES})
    target_include_directories(test_motor_tests PRIVATE
        # compat/ must come first so <Arduino.h> and <SPI.h> resolve to our stubs
        ${CMAKE_CURRENT_SOURCE_DIR}/compat
        ${SRC_DIR}
    )
    target_link_libraries(test_motor_tests PRIVATE
        GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(test_motor_tests)
endif()
