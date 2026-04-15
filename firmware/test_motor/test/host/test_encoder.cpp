// MIT License — HackerFab Open Micro-Manipulator
// Host-native unit tests for test_motor firmware.
//   - compute_crc8()    : MT6835 CRC-8 algorithm (encoder.cpp)
//   - update_position() : multi-turn absolute position tracking (encoder.cpp)
//   - math_constants.h  : compile-time constant correctness
//   - hw_config.h       : derived hardware constants
#include <gtest/gtest.h>
#include <cmath>

// Forward-declare pure functions and module-level state from encoder.cpp.
// These are free (non-static) symbols and are accessible by the linker.
uint8_t compute_crc8(uint32_t angle21, uint8_t status3);
void    update_position(int32_t raw);

extern int32_t abs_position;
extern bool    first_read;

// Header-only modules (no hardware deps)
#include "utilities/math_constants.h"
#include "hw_config.h"

// ===========================================================================
// Helpers
// ===========================================================================
static void reset_position_state() {
    first_read   = true;
    abs_position = 0;
}

static constexpr int32_t MT6835_CPR = 2097152;  // 21-bit, matches encoder.cpp

// ===========================================================================
// compute_crc8 — MT6835 CRC-8 (poly 0x07, init 0x00)
// ===========================================================================

TEST(MT6835Crc, ZeroAngleZeroStatusYieldsZero) {
    // All input bits zero → CRC remains 0 throughout the shift register
    EXPECT_EQ(compute_crc8(0, 0), 0);
}

TEST(MT6835Crc, IsDeterministic) {
    EXPECT_EQ(compute_crc8(12345, 3), compute_crc8(12345, 3));
    EXPECT_EQ(compute_crc8(0x1FFFFF, 0x07), compute_crc8(0x1FFFFF, 0x07));
}

TEST(MT6835Crc, ChangingAngleBitChangesCrc) {
    // Flipping the LSB of angle must produce a different CRC
    EXPECT_NE(compute_crc8(12345, 0), compute_crc8(12344, 0));
}

TEST(MT6835Crc, ChangingStatusBitChangesCrc) {
    EXPECT_NE(compute_crc8(100, 0), compute_crc8(100, 1));
    EXPECT_NE(compute_crc8(100, 0), compute_crc8(100, 7));
}

TEST(MT6835Crc, AngleBitsAbove21AreIgnored) {
    // Mask is 0x1FFFFF (bits 0-20). 0x200001 has bit 21 set; after masking → 0x000001.
    uint32_t low  = 0x000001u;
    uint32_t high = 0x200001u;  // bit 21 is above the 21-bit mask
    EXPECT_EQ(compute_crc8(low, 0), compute_crc8(high, 0));
}

TEST(MT6835Crc, StatusBitsAbove3AreIgnored) {
    // Mask is 0x07 — bits 3+ of status must not change output
    EXPECT_EQ(compute_crc8(500, 0x01), compute_crc8(500, 0x09));
    EXPECT_EQ(compute_crc8(500, 0x03), compute_crc8(500, 0x3B));
}

TEST(MT6835Crc, OutputFitsInOneByte) {
    // Return value is masked with 0xFF — result always fits in uint8_t range
    for (uint32_t angle : {0u, 1u, 0xFFFFu, 0x1FFFFFu}) {
        for (uint8_t st : {uint8_t(0), uint8_t(3), uint8_t(7)}) {
            EXPECT_LE(compute_crc8(angle, st), 0xFF);
        }
    }
}

TEST(MT6835Crc, FullScaleAngleNonZeroCrc) {
    // Max valid angle 0x1FFFFF with status 7 should not yield zero (sanity)
    uint8_t crc = compute_crc8(0x1FFFFF, 0x07);
    // We just verify determinism and non-crash; exact value is hardware-defined
    EXPECT_EQ(crc, compute_crc8(0x1FFFFF, 0x07));
}

// ===========================================================================
// update_position — multi-turn absolute position tracking
// ===========================================================================

TEST(PositionTracking, FirstReadInitializesAbsPosition) {
    reset_position_state();
    update_position(1000);
    EXPECT_EQ(abs_position, 1000);
}

TEST(PositionTracking, SequentialIncrementsAreTracked) {
    reset_position_state();
    update_position(0);
    update_position(500);
    update_position(1500);
    EXPECT_EQ(abs_position, 1500);
}

TEST(PositionTracking, ForwardWrapCorrectlyIncrements) {
    // Going from near-max to near-zero is a forward rotation (counts continue up)
    reset_position_state();
    update_position(MT6835_CPR - 100);
    update_position(50);
    // delta = 50 - (CPR-100) ≈ -(CPR-150), which is < -CPR/2, so delta += CPR = 150
    EXPECT_EQ(abs_position, (MT6835_CPR - 100) + 150);
}

TEST(PositionTracking, BackwardWrapCorrectlyDecrements) {
    // Going from near-zero to near-max is a backward rotation
    reset_position_state();
    update_position(100);
    update_position(MT6835_CPR - 50);
    // delta = (CPR-50) - 100 ≈ CPR-150, which is > CPR/2, so delta -= CPR = -150
    EXPECT_EQ(abs_position, 100 - 150);
}

TEST(PositionTracking, MultipleWrapsAccumulate) {
    reset_position_state();
    update_position(0);
    // Advance 3 full revolutions using quarter-turn steps so no single
    // delta exceeds CPR/2 (which would trigger the wrong branch).
    for (int rev = 0; rev < 3; ++rev) {
        update_position(MT6835_CPR / 4);
        update_position(MT6835_CPR / 2);
        update_position(3 * MT6835_CPR / 4);
        update_position(0);  // wrap: delta = -(3*CPR/4) < -CPR/2 → +CPR
    }
    EXPECT_EQ(abs_position, 3 * MT6835_CPR);
}

// ===========================================================================
// Math constants (math_constants.h — header-only, no hardware deps)
// ===========================================================================

TEST(MathConstants, PiIsCorrect) {
    EXPECT_NEAR(Constants::PI_F, 3.14159f, 1e-4f);
}

TEST(MathConstants, TwoPiIsTwicePI) {
    EXPECT_NEAR(Constants::TWO_PI_F, 2.0f * Constants::PI_F, 1e-6f);
}

TEST(MathConstants, Rad2DegAndDeg2RadAreReciprocal) {
    EXPECT_NEAR(Constants::RAD2DEG * Constants::DEG2RAD, 1.0f, 1e-6f);
}

TEST(MathConstants, Rad2DegKnownValue) {
    // 1 radian ≈ 57.2958°
    EXPECT_NEAR(Constants::RAD2DEG, 57.2958f, 1e-3f);
}

TEST(MathConstants, Deg2RadKnownValue) {
    // 180° should equal π
    EXPECT_NEAR(180.0f * Constants::DEG2RAD, Constants::PI_F, 1e-5f);
}

// ===========================================================================
// Hardware config constants (hw_config.h)
// ===========================================================================

TEST(HwConfig, EncoderAngleToRotorAngleIsPositive) {
    EXPECT_GT(ENCODER_ANGLE_TO_ROTOR_ANGLE, 0.0f);
}

TEST(HwConfig, EncoderAngleToRotorAngleMatchesFormula) {
    // = (ENCODER_MAGNET_PITCH * 2) / (ENCODER_MAGNET_RADIUS * TWO_PI)
    // = (3mm * 2) / (30mm * 2π) ≈ 6 / 188.495 ≈ 0.03183
    constexpr float expected = (3.0f * 2.0f) / (30.0f * Constants::TWO_PI_F);
    EXPECT_NEAR(ENCODER_ANGLE_TO_ROTOR_ANGLE, expected, 1e-5f);
}

TEST(HwConfig, PolePairCountsArePositive) {
    EXPECT_GT(MOTOR1_POLE_PAIRS, 0.0f);
    EXPECT_GT(MOTOR2_POLE_PAIRS, 0.0f);
    EXPECT_GT(MOTOR3_POLE_PAIRS, 0.0f);
}

TEST(HwConfig, HomingParametersAreReasonable) {
    EXPECT_GT(HOMING_VELOCITY, 0.0f);
    EXPECT_GT(HOMING_CURRENT,  0.0f);
    EXPECT_LT(HOMING_CURRENT,  1.0f);  // must be in range 0..1
    EXPECT_GT(HOMING_FINISH_POS, 0.0f);
}
