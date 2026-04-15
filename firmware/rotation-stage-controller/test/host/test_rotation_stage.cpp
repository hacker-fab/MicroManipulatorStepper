// MIT License — HackerFab Open Micro-Manipulator
// Host-native unit tests for rotation-stage-controller firmware.
//   - positive_mod()         : utils.h integer modulo helper
//   - temperature_set/get()  : temperature module setpoint management
//   - temperature_poll()     : PID arithmetic paths (stub ADC returns 0)
//   - vacuum_set/get()       : vacuum duty state tracking
#include <gtest/gtest.h>
#include <cmath>

#include "utils.h"
#include "temperature.h"
#include "vacuum.h"

// ===========================================================================
// positive_mod macro (utils.h)
// ===========================================================================

TEST(PositiveMod, PositiveValueLessThanMod) {
    EXPECT_EQ(positive_mod(2, 5), 2);
}

TEST(PositiveMod, PositiveValueEqualToMod) {
    EXPECT_EQ(positive_mod(5, 5), 0);
}

TEST(PositiveMod, PositiveValueGreaterThanMod) {
    EXPECT_EQ(positive_mod(7, 3), 1);
    EXPECT_EQ(positive_mod(9, 3), 0);
}

TEST(PositiveMod, NegativeValueMinusOne) {
    EXPECT_EQ(positive_mod(-1, 5), 4);
    EXPECT_EQ(positive_mod(-1, 3), 2);
}

TEST(PositiveMod, NegativeValueEqualToNegMod) {
    EXPECT_EQ(positive_mod(-5, 5), 0);
    EXPECT_EQ(positive_mod(-3, 3), 0);
}

TEST(PositiveMod, NegativeValueLargerMagnitude) {
    EXPECT_EQ(positive_mod(-7, 3), 2);
    EXPECT_EQ(positive_mod(-8, 5), 2);
}

TEST(PositiveMod, ZeroValue) {
    EXPECT_EQ(positive_mod(0, 7), 0);
    EXPECT_EQ(positive_mod(0, 1), 0);
}

TEST(PositiveMod, ResultAlwaysNonNegative) {
    for (int v = -20; v <= 20; ++v) {
        EXPECT_GE(positive_mod(v, 7), 0) << "v=" << v;
        EXPECT_LT(positive_mod(v, 7), 7) << "v=" << v;
    }
}

// ===========================================================================
// Temperature module — setpoint management (temperature.h / temperature.cpp)
// ===========================================================================

TEST(Temperature, DefaultSetpointIsZero) {
    // Static module state initialises to 0.0f at program start
    EXPECT_FLOAT_EQ(temperature_target_get(), 0.0f);
}

TEST(Temperature, SetpointSetAndGet) {
    temperature_set(60.0f);
    EXPECT_FLOAT_EQ(temperature_target_get(), 60.0f);
}

TEST(Temperature, SetpointUpdatedToNewValue) {
    temperature_set(25.0f);
    temperature_set(75.0f);
    EXPECT_FLOAT_EQ(temperature_target_get(), 75.0f);
}

TEST(Temperature, SetpointZeroIsValid) {
    temperature_set(0.0f);
    EXPECT_FLOAT_EQ(temperature_target_get(), 0.0f);
}

TEST(Temperature, SetpointNegativeIsStored) {
    // Module does no clamping of the setpoint itself
    temperature_set(-10.0f);
    EXPECT_FLOAT_EQ(temperature_target_get(), -10.0f);
}

TEST(Temperature, PollDoesNotCrash) {
    // With analogRead() stubbed to 0, poll() should execute cleanly.
    // raw_temp = (0 / 4095.0 * 3.3 - 0.4) / 0.0195 ≈ -20.5 °C
    // The rolling average and PID update run without hardware.
    temperature_set(0.0f);
    EXPECT_NO_THROW(temperature_poll(10000));  // dt = 10 ms
}

TEST(Temperature, PollWithHighSetpointDoesNotCrash) {
    temperature_set(100.0f);
    EXPECT_NO_THROW(temperature_poll(10000));
    temperature_set(0.0f);  // restore neutral state for later tests
}

TEST(Temperature, RollingAverageConvergesWithRepeatedPolls) {
    // After many polls with ADC=0 (→ raw ≈ -20.5°C), the rolling average
    // should converge towards that value and remain finite.
    temperature_set(0.0f);
    for (int i = 0; i < 200; ++i) {
        temperature_poll(10000);
    }
    float avg = temperature_get();
    EXPECT_TRUE(std::isfinite(avg));
    EXPECT_LT(avg, 0.0f);   // expected to settle below 0 (cold stub reading)
}

// ===========================================================================
// Vacuum module — duty state tracking (vacuum.h / vacuum.cpp)
// ===========================================================================

TEST(Vacuum, DefaultDutyIsZero) {
    // Static vacuum_state initialises to 0
    EXPECT_EQ(vacuum_get(), 0);
}

TEST(Vacuum, SetAndGetDuty) {
    vacuum_set(50);
    EXPECT_EQ(vacuum_get(), 50);
}

TEST(Vacuum, SetDutyToMax) {
    vacuum_set(100);
    EXPECT_EQ(vacuum_get(), 100);
}

TEST(Vacuum, SetDutyToZero) {
    vacuum_set(75);
    vacuum_set(0);
    EXPECT_EQ(vacuum_get(), 0);
}

TEST(Vacuum, SetDutyMultipleTimes) {
    vacuum_set(10);
    vacuum_set(30);
    vacuum_set(80);
    EXPECT_EQ(vacuum_get(), 80);
}

TEST(Vacuum, SetCallsAnalogWriteNoOp) {
    // With analogWrite stubbed to no-op this must not throw or crash
    EXPECT_NO_THROW(vacuum_set(60));
    EXPECT_EQ(vacuum_get(), 60);
}
