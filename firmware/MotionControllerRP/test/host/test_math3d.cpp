// Tests for Vec3F and QuaternionF (math3d.h)
#include <gtest/gtest.h>
#include "utilities/math3d.h"
#include <cmath>

static constexpr float kEps = 1e-5f;

// ---------------------------------------------------------------------------
// Vec3F: arithmetic
// ---------------------------------------------------------------------------
TEST(Vec3F, DefaultConstructorIsZero) {
    Vec3F v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3F, AdditionAndSubtraction) {
    Vec3F a(1, 2, 3);
    Vec3F b(4, 5, 6);
    Vec3F sum = a + b;
    EXPECT_FLOAT_EQ(sum.x, 5.0f);
    EXPECT_FLOAT_EQ(sum.y, 7.0f);
    EXPECT_FLOAT_EQ(sum.z, 9.0f);

    Vec3F diff = b - a;
    EXPECT_FLOAT_EQ(diff.x, 3.0f);
    EXPECT_FLOAT_EQ(diff.y, 3.0f);
    EXPECT_FLOAT_EQ(diff.z, 3.0f);
}

TEST(Vec3F, ScalarMultiplyAndDivide) {
    Vec3F v(2.0f, 4.0f, 6.0f);
    Vec3F scaled = v * 2.0f;
    EXPECT_FLOAT_EQ(scaled.x, 4.0f);
    EXPECT_FLOAT_EQ(scaled.y, 8.0f);
    EXPECT_FLOAT_EQ(scaled.z, 12.0f);

    Vec3F halved = v / 2.0f;
    EXPECT_FLOAT_EQ(halved.x, 1.0f);
    EXPECT_FLOAT_EQ(halved.y, 2.0f);
    EXPECT_FLOAT_EQ(halved.z, 3.0f);
}

// ---------------------------------------------------------------------------
// Vec3F: length / dot / cross / normalized
// ---------------------------------------------------------------------------
TEST(Vec3F, LengthOfUnitX) {
    EXPECT_FLOAT_EQ(Vec3F(1, 0, 0).length(), 1.0f);
}

TEST(Vec3F, Length345) {
    // 3-4-5 right triangle generalised to 3D
    EXPECT_NEAR(Vec3F(3, 4, 0).length(), 5.0f, kEps);
}

TEST(Vec3F, DotProduct) {
    Vec3F a(1, 0, 0);
    Vec3F b(0, 1, 0);
    EXPECT_FLOAT_EQ(a.dot(b), 0.0f);

    Vec3F c(1, 2, 3);
    Vec3F d(4, 5, 6);
    EXPECT_FLOAT_EQ(c.dot(d), 32.0f);  // 4+10+18
}

TEST(Vec3F, CrossProduct) {
    Vec3F x(1, 0, 0);
    Vec3F y(0, 1, 0);
    Vec3F z = x.cross(y);
    EXPECT_NEAR(z.x, 0.0f, kEps);
    EXPECT_NEAR(z.y, 0.0f, kEps);
    EXPECT_NEAR(z.z, 1.0f, kEps);

    // Anti-commutativity
    Vec3F neg_z = y.cross(x);
    EXPECT_NEAR(neg_z.z, -1.0f, kEps);
}

TEST(Vec3F, NormalizedHasUnitLength) {
    Vec3F v(3.0f, 4.0f, 0.0f);
    Vec3F n = v.normalized();
    EXPECT_NEAR(n.length(), 1.0f, kEps);
    EXPECT_NEAR(n.x, 0.6f, kEps);
    EXPECT_NEAR(n.y, 0.8f, kEps);
}

TEST(Vec3F, NormalizedZeroVectorReturnsZero) {
    Vec3F n = Vec3F(0, 0, 0).normalized();
    EXPECT_FLOAT_EQ(n.length(), 0.0f);
}

// ---------------------------------------------------------------------------
// QuaternionF: identity
// ---------------------------------------------------------------------------
TEST(QuaternionF, DefaultIsIdentity) {
    QuaternionF q;
    EXPECT_FLOAT_EQ(q.w, 1.0f);
    EXPECT_FLOAT_EQ(q.x, 0.0f);
    EXPECT_FLOAT_EQ(q.y, 0.0f);
    EXPECT_FLOAT_EQ(q.z, 0.0f);
}

TEST(QuaternionF, IdentityRotatesVectorUnchanged) {
    QuaternionF q;  // identity
    Vec3F v(1.0f, 2.0f, 3.0f);
    Vec3F r = q.rotate(v);
    EXPECT_NEAR(r.x, v.x, kEps);
    EXPECT_NEAR(r.y, v.y, kEps);
    EXPECT_NEAR(r.z, v.z, kEps);
}

// ---------------------------------------------------------------------------
// QuaternionF: from_axis_angle
// ---------------------------------------------------------------------------
TEST(QuaternionF, RotateXAxis90DegAroundZ) {
    // Rotate (1,0,0) by 90° around Z → should give (0,1,0)
    Vec3F axis(0, 0, 1);
    float angle = static_cast<float>(M_PI) / 2.0f;
    QuaternionF q = QuaternionF::from_axis_angle(axis, angle);

    Vec3F v(1.0f, 0.0f, 0.0f);
    Vec3F r = q.rotate(v);
    EXPECT_NEAR(r.x, 0.0f, kEps);
    EXPECT_NEAR(r.y, 1.0f, kEps);
    EXPECT_NEAR(r.z, 0.0f, kEps);
}

TEST(QuaternionF, RotateYAxis180DegAroundZ) {
    // Rotate (0,1,0) by 180° around Z → should give (0,-1,0)
    Vec3F axis(0, 0, 1);
    float angle = static_cast<float>(M_PI);
    QuaternionF q = QuaternionF::from_axis_angle(axis, angle);

    Vec3F v(0.0f, 1.0f, 0.0f);
    Vec3F r = q.rotate(v);
    EXPECT_NEAR(r.x,  0.0f, kEps);
    EXPECT_NEAR(r.y, -1.0f, kEps);
    EXPECT_NEAR(r.z,  0.0f, kEps);
}

TEST(QuaternionF, ZeroAngleRotationIsIdentity) {
    Vec3F axis(0, 0, 1);
    QuaternionF q = QuaternionF::from_axis_angle(axis, 0.0f);
    Vec3F v(1.0f, 2.0f, 3.0f);
    Vec3F r = q.rotate(v);
    EXPECT_NEAR(r.x, v.x, kEps);
    EXPECT_NEAR(r.y, v.y, kEps);
    EXPECT_NEAR(r.z, v.z, kEps);
}

// ---------------------------------------------------------------------------
// QuaternionF: normalized_inverse undoes rotation
// ---------------------------------------------------------------------------
TEST(QuaternionF, NormalizedInverseUndoesRotation) {
    Vec3F axis(0, 1, 0);
    float angle = 1.2f;
    QuaternionF q = QuaternionF::from_axis_angle(axis, angle);
    QuaternionF qi = q.normalized_inverse();

    Vec3F v(3.0f, 1.0f, -2.0f);
    Vec3F rotated = q.rotate(v);
    Vec3F restored = qi.rotate(rotated);

    EXPECT_NEAR(restored.x, v.x, kEps);
    EXPECT_NEAR(restored.y, v.y, kEps);
    EXPECT_NEAR(restored.z, v.z, kEps);
}

// ---------------------------------------------------------------------------
// QuaternionF: composition
// ---------------------------------------------------------------------------
TEST(QuaternionF, CompositionTwoHalfTurnsIsFullTurn) {
    // Two 180° rotations around Z should bring (1,0,0) back to (1,0,0)
    Vec3F axis(0, 0, 1);
    QuaternionF q = QuaternionF::from_axis_angle(axis, static_cast<float>(M_PI));
    QuaternionF q2 = (q * q).normalized();

    Vec3F v(1.0f, 0.0f, 0.0f);
    Vec3F r = q2.rotate(v);
    EXPECT_NEAR(r.x, 1.0f, kEps);
    EXPECT_NEAR(r.y, 0.0f, kEps);
    EXPECT_NEAR(r.z, 0.0f, kEps);
}

// ---------------------------------------------------------------------------
// QuaternionF: slerp
// ---------------------------------------------------------------------------
TEST(QuaternionF, SlerpAtT0ReturnsQ1) {
    QuaternionF q1 = QuaternionF::from_axis_angle(Vec3F(0,0,1), 0.0f);
    QuaternionF q2 = QuaternionF::from_axis_angle(Vec3F(0,0,1), 1.0f);
    QuaternionF r = q1.slerp(q2, 0.0f);
    EXPECT_NEAR(r.w, q1.w, kEps);
    EXPECT_NEAR(r.x, q1.x, kEps);
}

TEST(QuaternionF, SlerpAtT1ReturnsQ2) {
    QuaternionF q1 = QuaternionF::from_axis_angle(Vec3F(0,0,1), 0.0f);
    QuaternionF q2 = QuaternionF::from_axis_angle(Vec3F(0,0,1), 1.0f);
    QuaternionF r = q1.slerp(q2, 1.0f);
    EXPECT_NEAR(r.w, q2.w, kEps);
    EXPECT_NEAR(r.z, q2.z, kEps);
}

TEST(QuaternionF, SlerpAtMidpointHasHalfAngle) {
    Vec3F axis(0, 0, 1);
    QuaternionF q1 = QuaternionF::from_axis_angle(axis, 0.0f);
    QuaternionF q2 = QuaternionF::from_axis_angle(axis, 1.0f);  // 1 rad
    QuaternionF mid = q1.slerp(q2, 0.5f);

    Vec3F v(1.0f, 0.0f, 0.0f);
    Vec3F r = mid.rotate(v);
    EXPECT_NEAR(r.x, std::cos(0.5f), kEps);
    EXPECT_NEAR(r.y, std::sin(0.5f), kEps);
}
