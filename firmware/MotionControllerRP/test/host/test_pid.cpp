// Tests for PIDController and LowpassFilter (servo_control/pid.h)
#include <gtest/gtest.h>
#include "servo_control/pid.h"
#include <cmath>

static constexpr float kEps = 1e-5f;
static constexpr float dt   = 0.001f;   // 1 ms timestep
static constexpr float odt  = 1.0f / dt;

// ---------------------------------------------------------------------------
// PIDController — proportional only
// ---------------------------------------------------------------------------
TEST(PIDController, ProportionalOnlyOutput) {
    PIDController pid;
    pid.set_parameter(2.0f, 0.0f, 0.0f, 100.0f, 100.0f);
    float out = pid.compute(5.0f, dt, odt);
    EXPECT_NEAR(out, 10.0f, kEps);  // kP * error = 2 * 5
}

TEST(PIDController, ProportionalNegativeError) {
    PIDController pid;
    pid.set_parameter(3.0f, 0.0f, 0.0f, 100.0f, 100.0f);
    float out = pid.compute(-4.0f, dt, odt);
    EXPECT_NEAR(out, -12.0f, kEps);
}

// ---------------------------------------------------------------------------
// PIDController — output clamping
// ---------------------------------------------------------------------------
TEST(PIDController, OutputClampedToLimit) {
    PIDController pid;
    pid.set_parameter(100.0f, 0.0f, 0.0f, 10.0f, 10.0f);  // limit = 10
    float out = pid.compute(5.0f, dt, odt);   // kP*error = 500, but clamped
    EXPECT_NEAR(out, 10.0f, kEps);
}

TEST(PIDController, OutputClampedNegative) {
    PIDController pid;
    pid.set_parameter(100.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    float out = pid.compute(-5.0f, dt, odt);
    EXPECT_NEAR(out, -10.0f, kEps);
}

// ---------------------------------------------------------------------------
// PIDController — integral accumulation
// ---------------------------------------------------------------------------
TEST(PIDController, IntegralAccumulates) {
    PIDController pid;
    pid.set_parameter(0.0f, 1.0f, 0.0f, 1000.0f, 1000.0f);  // I-only
    // After N steps with constant error=1, integral grows
    float sum = 0.0f;
    for (int i = 0; i < 100; ++i)
        sum = pid.compute(1.0f, dt, odt);
    EXPECT_GT(sum, 0.0f);  // positive error → positive accumulation
}

TEST(PIDController, IntegralWindupClamped) {
    PIDController pid;
    // Tiny windup_limit so the integrator saturates quickly
    pid.set_parameter(0.0f, 100.0f, 0.0f, 1000.0f, 0.5f);
    float out = 0.0f;
    for (int i = 0; i < 1000; ++i)
        out = pid.compute(1.0f, dt, odt);
    EXPECT_LE(out, 0.5f + kEps);   // integral cannot exceed windup_limit
}

// ---------------------------------------------------------------------------
// PIDController — derivative
// ---------------------------------------------------------------------------
TEST(PIDController, DerivativeBrakesOnDecreasingError) {
    PIDController pid;
    pid.set_parameter(0.0f, 0.0f, 1.0f, 1000.0f, 1000.0f);  // D-only
    // First step: error = 1, prev = 0 → derivative = (1 - 0) / dt > 0
    float out1 = pid.compute(1.0f, dt, odt);
    EXPECT_GT(out1, 0.0f);
    // Second step: error decreases to 0 → derivative = (0 - 1) / dt < 0
    float out2 = pid.compute(0.0f, dt, odt);
    EXPECT_LT(out2, 0.0f);
}

// ---------------------------------------------------------------------------
// PIDController — reset
// ---------------------------------------------------------------------------
TEST(PIDController, ResetClearsIntegralAndError) {
    PIDController pid;
    pid.set_parameter(1.0f, 1.0f, 0.0f, 1000.0f, 1000.0f);
    for (int i = 0; i < 50; ++i)
        pid.compute(1.0f, dt, odt);
    pid.reset();
    // After reset, first compute step should give P only (I = 0, prev_err = 0)
    float out = pid.compute(1.0f, dt, odt);
    EXPECT_NEAR(out, 1.0f + 1.0f * 0.5f * dt * 1.0f, 1e-3f);
}

TEST(PIDController, ZeroErrorGivesZeroOutput) {
    PIDController pid;
    pid.set_parameter(5.0f, 5.0f, 5.0f, 100.0f, 100.0f);
    for (int i = 0; i < 100; ++i)
        pid.compute(0.0f, dt, odt);
    float out = pid.compute(0.0f, dt, odt);
    EXPECT_NEAR(out, 0.0f, kEps);
}

// ---------------------------------------------------------------------------
// LowpassFilter — basic behaviour
// ---------------------------------------------------------------------------
TEST(LowpassFilter, StepResponseConverges) {
    LowpassFilter lpf;
    lpf.set_time_constant(0.1f);  // 100 ms
    lpf.reset(0.0f);

    // Drive with value = 1 for 10 time constants
    float out = 0.0f;
    for (int i = 0; i < 10000; ++i)
        out = lpf.update(1.0f, 0.0001f);  // 0.1 ms steps
    EXPECT_NEAR(out, 1.0f, 0.01f);
}

TEST(LowpassFilter, ResetSetInitialValue) {
    LowpassFilter lpf;
    lpf.set_time_constant(1.0f);
    lpf.reset(5.0f);
    // First update: output should be between reset value and new input
    float out = lpf.update(0.0f, 0.001f);
    EXPECT_LT(out, 5.0f);
    EXPECT_GT(out, 0.0f);
}

TEST(LowpassFilter, SlowTimeConstantHardlyMoves) {
    LowpassFilter lpf;
    lpf.set_time_constant(100.0f);  // very slow
    lpf.reset(0.0f);
    float out = lpf.update(1.0f, 0.001f);  // single step
    EXPECT_LT(out, 0.001f);  // barely moved
}

TEST(LowpassFilter, FastTimeConstantPassesThrough) {
    LowpassFilter lpf;
    lpf.set_time_constant(1e-9f);  // essentially no filtering
    lpf.reset(0.0f);
    float out = lpf.update(42.0f, 1.0f);
    EXPECT_NEAR(out, 42.0f, 0.01f);
}
