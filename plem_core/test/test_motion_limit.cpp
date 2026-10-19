// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <type_traits>
#include "plem/core/types/motion_limit.hpp"

using namespace plem::core;

// Layout contract: the RT controllers store MotionLimit in pre-allocated
// std::array buffers and copy them around the 1kHz path.
static_assert(std::is_trivially_copyable_v<MotionLimit>);

// Validation Tests

TEST(MotionLimitValidationTest, ValidLimit) {
    MotionLimit limit{-3.14, 3.14, 2.0, 200.0};
    EXPECT_TRUE(limit.is_valid());
}

TEST(MotionLimitValidationTest, InvalidLowerGreaterThanUpper) {
    MotionLimit limit{3.14, -3.14, 2.0, 200.0};  // lower > upper
    EXPECT_FALSE(limit.is_valid());
}

TEST(MotionLimitValidationTest, InvalidEqualLimits) {
    MotionLimit limit{1.0, 1.0, 2.0, 200.0};  // lower == upper
    EXPECT_FALSE(limit.is_valid());
}

TEST(MotionLimitValidationTest, InvalidZeroVelocity) {
    MotionLimit limit{-3.14, 3.14, 0.0, 200.0};  // velocity_limit == 0
    EXPECT_FALSE(limit.is_valid());
}

TEST(MotionLimitValidationTest, InvalidNegativeVelocity) {
    MotionLimit limit{-3.14, 3.14, -2.0, 200.0};  // velocity_limit < 0
    EXPECT_FALSE(limit.is_valid());
}

TEST(MotionLimitValidationTest, InvalidZeroEffort) {
    // effort_limit == 0 would silently disable the torque clamp; a limits
    // object without a real per-joint effort (URDF <limit effort>) is invalid.
    MotionLimit limit{-3.14, 3.14, 2.0, 0.0};
    EXPECT_FALSE(limit.is_valid());
}

TEST(MotionLimitValidationTest, InvalidNegativeEffort) {
    MotionLimit limit{-3.14, 3.14, 2.0, -50.0};
    EXPECT_FALSE(limit.is_valid());
}

// Check Limit Tests

TEST(MotionLimitCheckTest, WithinLimits) {
    MotionLimit limit{-3.14, 3.14, 2.0, 200.0};
    EXPECT_TRUE(limit.check_limit(0.0, 0.0));
    EXPECT_TRUE(limit.check_limit(1.0, 1.0));
    EXPECT_TRUE(limit.check_limit(-1.0, -1.0));
}

TEST(MotionLimitCheckTest, AtBoundaries) {
    MotionLimit limit{-3.14, 3.14, 2.0, 200.0};
    EXPECT_TRUE(limit.check_limit(-3.14, 0.0));  // at lower position limit
    EXPECT_TRUE(limit.check_limit(3.14, 0.0));   // at upper position limit
    EXPECT_TRUE(limit.check_limit(0.0, 2.0));    // at positive velocity limit
    EXPECT_TRUE(limit.check_limit(0.0, -2.0));   // at negative velocity limit
}

TEST(MotionLimitCheckTest, OutsideLimits) {
    MotionLimit limit{-3.14, 3.14, 2.0, 200.0};
    EXPECT_FALSE(limit.check_limit(-4.0, 0.0));  // below lower position limit
    EXPECT_FALSE(limit.check_limit(4.0, 0.0));   // above upper position limit
    EXPECT_FALSE(limit.check_limit(0.0, 3.0));   // above velocity limit
    EXPECT_FALSE(limit.check_limit(0.0, -3.0));  // below velocity limit
}

// Clamp Tests

TEST(MotionLimitClampTest, ClampPositionAndVelocity) {
    MotionLimit limit{-3.14, 3.14, 2.0, 200.0};

    // Position: below lower -> lower, above upper -> upper, within -> unchanged.
    EXPECT_DOUBLE_EQ(limit.clamp_position(-4.0), -3.14);
    EXPECT_DOUBLE_EQ(limit.clamp_position(4.0), 3.14);
    EXPECT_DOUBLE_EQ(limit.clamp_position(1.0), 1.0);
    // At exact bounds -> bound.
    EXPECT_DOUBLE_EQ(limit.clamp_position(-3.14), -3.14);
    EXPECT_DOUBLE_EQ(limit.clamp_position(3.14), 3.14);

    // Velocity: below -vel -> -vel, above +vel -> +vel, within -> unchanged.
    EXPECT_DOUBLE_EQ(limit.clamp_velocity(-3.0), -2.0);
    EXPECT_DOUBLE_EQ(limit.clamp_velocity(3.0), 2.0);
    EXPECT_DOUBLE_EQ(limit.clamp_velocity(1.0), 1.0);
    // At exact ±vel -> ±vel.
    EXPECT_DOUBLE_EQ(limit.clamp_velocity(-2.0), -2.0);
    EXPECT_DOUBLE_EQ(limit.clamp_velocity(2.0), 2.0);
}

TEST(MotionLimitClampTest, ClampTorqueSymmetricEffortBand) {
    MotionLimit limit{-3.14, 3.14, 2.0, 80.0};

    // Above +effort -> +effort, below -effort -> -effort, within -> unchanged.
    EXPECT_DOUBLE_EQ(limit.clamp_torque(200.0), 80.0);
    EXPECT_DOUBLE_EQ(limit.clamp_torque(-200.0), -80.0);
    EXPECT_DOUBLE_EQ(limit.clamp_torque(50.0), 50.0);
    // At exact ±effort -> ±effort.
    EXPECT_DOUBLE_EQ(limit.clamp_torque(80.0), 80.0);
    EXPECT_DOUBLE_EQ(limit.clamp_torque(-80.0), -80.0);
}
