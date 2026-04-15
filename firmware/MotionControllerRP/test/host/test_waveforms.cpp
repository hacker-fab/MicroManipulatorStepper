// Tests for waveform generators (utilities/waveforms.h)
#include <gtest/gtest.h>
#include "utilities/waveforms.h"
#include "utilities/math_constants.h"
#include <cmath>

static constexpr float kEps  = 1e-4f;
static constexpr float TWO_PI = Constants::TWO_PI_F;

// ---------------------------------------------------------------------------
// triangle_wave: period = 2π, range = [-1, 1]
//   phase 0       → 0
//   phase π/2     → +1
//   phase π       → 0
//   phase 3π/2    → -1
// ---------------------------------------------------------------------------
TEST(TriangleWave, AtOrigin) {
    EXPECT_NEAR(triangle_wave(0.0f), 0.0f, kEps);
}

TEST(TriangleWave, AtQuarterPeriod) {
    EXPECT_NEAR(triangle_wave(TWO_PI * 0.25f), 1.0f, kEps);
}

TEST(TriangleWave, AtHalfPeriod) {
    EXPECT_NEAR(triangle_wave(TWO_PI * 0.5f), 0.0f, kEps);
}

TEST(TriangleWave, AtThreeQuarterPeriod) {
    EXPECT_NEAR(triangle_wave(TWO_PI * 0.75f), -1.0f, kEps);
}

TEST(TriangleWave, IsPeriodic) {
    for (float x = 0.0f; x < TWO_PI; x += 0.3f)
        EXPECT_NEAR(triangle_wave(x), triangle_wave(x + TWO_PI), kEps);
}

TEST(TriangleWave, OutputBounds) {
    for (float x = 0.0f; x < 10.0f * TWO_PI; x += 0.05f) {
        float v = triangle_wave(x);
        EXPECT_GE(v, -1.0f - kEps);
        EXPECT_LE(v,  1.0f + kEps);
    }
}

TEST(TriangleWave, IsLinearBetweenExtrema) {
    // Between 0 and π/2 the wave rises linearly from 0 to 1
    float x0 = 0.1f, x1 = TWO_PI * 0.2f;
    float slope0 = (triangle_wave(x0 + 1e-4f) - triangle_wave(x0)) / 1e-4f;
    float slope1 = (triangle_wave(x1 + 1e-4f) - triangle_wave(x1)) / 1e-4f;
    EXPECT_NEAR(slope0, slope1, 0.01f);  // same slope → linear
}

// ---------------------------------------------------------------------------
// trapezoidal_wave: period = 2π, range = [-1, 1]
// Default plateau_fraction = 0.2
// ---------------------------------------------------------------------------
TEST(TrapezoidalWave, OutputBounds) {
    for (float x = 0.0f; x < 10.0f * TWO_PI; x += 0.05f) {
        float v = trapezoidal_wave(x);
        EXPECT_GE(v, -1.0f - kEps);
        EXPECT_LE(v,  1.0f + kEps);
    }
}

TEST(TrapezoidalWave, HasPositivePlateau) {
    // Around the top plateau the value should be exactly +1
    float plateau_start = TWO_PI * 0.2f + TWO_PI * 0.3f;  // after first ramp
    EXPECT_NEAR(trapezoidal_wave(plateau_start + 0.01f), 1.0f, kEps);
}

TEST(TrapezoidalWave, HasNegativePlateau) {
    // starts at x=0 (plateau_fraction=0.2 means bottom plateau from 0 to 0.2*2π)
    EXPECT_NEAR(trapezoidal_wave(0.0f), -1.0f, kEps);
    EXPECT_NEAR(trapezoidal_wave(0.1f * TWO_PI), -1.0f, kEps);
}

TEST(TrapezoidalWave, IsPeriodic) {
    for (float x = 0.0f; x < TWO_PI; x += 0.3f)
        EXPECT_NEAR(trapezoidal_wave(x), trapezoidal_wave(x + TWO_PI), kEps);
}

TEST(TrapezoidalWave, ZeroPlateau_DegradestoTriangle) {
    // With plateau_fraction = 0 the flat sections vanish — it becomes a triangle
    for (float x = 0.05f; x < TWO_PI - 0.05f; x += 0.2f) {
        float trap = trapezoidal_wave(x, 0.0f);
        EXPECT_GE(trap, -1.0f - kEps);
        EXPECT_LE(trap,  1.0f + kEps);
    }
}

// ---------------------------------------------------------------------------
// triangle_with_plateau: period = 2π, range = [-1, 1], has flat sections
// ---------------------------------------------------------------------------
TEST(TriangleWithPlateau, OutputBounds) {
    for (float x = 0.0f; x < 10.0f * TWO_PI; x += 0.05f) {
        float v = triangle_with_plateau(x);
        EXPECT_GE(v, -1.0f - kEps);
        EXPECT_LE(v,  1.0f + kEps);
    }
}

TEST(TriangleWithPlateau, IsPeriodic) {
    for (float x = 0.0f; x < TWO_PI; x += 0.3f)
        EXPECT_NEAR(triangle_with_plateau(x), triangle_with_plateau(x + TWO_PI), kEps);
}

TEST(TriangleWithPlateau, HasZeroPlateau) {
    // Segment 2 to 3 (i.e. 2/6 * 2π to 3/6 * 2π) should be 0
    float seg_start = TWO_PI * 2.0f / 6.0f + 0.01f;
    float seg_end   = TWO_PI * 3.0f / 6.0f - 0.01f;
    EXPECT_NEAR(triangle_with_plateau(seg_start), 0.0f, kEps);
    EXPECT_NEAR(triangle_with_plateau(seg_end),   0.0f, kEps);
}

TEST(TriangleWithPlateau, PeakIsPositiveOne) {
    // Peak (value = +1) is at the boundary between segments 0 and 1
    float peak = TWO_PI * 1.0f / 6.0f;
    EXPECT_NEAR(triangle_with_plateau(peak), 1.0f, kEps);
}

TEST(TriangleWithPlateau, TroughIsNegativeOne) {
    // Trough (value = -1) is at the boundary between segments 3 and 4
    float trough = TWO_PI * 4.0f / 6.0f;
    EXPECT_NEAR(triangle_with_plateau(trough), -1.0f, kEps);
}
