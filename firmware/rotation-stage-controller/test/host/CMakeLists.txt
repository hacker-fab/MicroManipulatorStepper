# =============================================================================
# rotation-stage-controller — Host-native unit tests with GoogleTest
# This is a separate CMake project for host compilation (not cross-compiled).
# =============================================================================
cmake_minimum_required(VERSION 3.14)

project(rotation_stage_tests CXX)
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

# Firmware source files that are pure enough to compile on the host
# (hardware calls are satisfied by the Arduino compat stub above).
set(FIRMWARE_SOURCES
    ${SRC_DIR}/temperature.cpp
    ${SRC_DIR}/vacuum.cpp
)

# Explicitly list real test files (excludes placeholder smoke_test.cpp)
set(TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rotation_stage.cpp
)

if(TEST_SOURCES)
    add_executable(rotation_stage_tests ${TEST_SOURCES} ${FIRMWARE_SOURCES})
    target_include_directories(rotation_stage_tests PRIVATE
        # compat/ must come first so <Arduino.h> resolves to our stub
        ${CMAKE_CURRENT_SOURCE_DIR}/compat
        ${SRC_DIR}
    )
    target_link_libraries(rotation_stage_tests PRIVATE
        GTest::gtest_main
    )
    include(GoogleTest)
    gtest_discover_tests(rotation_stage_tests)
endif()
