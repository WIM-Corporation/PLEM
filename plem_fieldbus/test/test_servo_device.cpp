// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/servo/device.hpp"
#include "plem/fieldbus/ethercat/utils/servo_conversions.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

using namespace plem::fieldbus::ethercat;
using namespace plem::core::units;

// Device-level tests on the production constructor and the fake master's
// slave config (no bus). Registration and initialize() run in
// test_ethercat_bus_fake_master.cpp.

class DeviceServoTest : public ::testing::Test {
  protected:
    void SetUp() override {
        plem::fieldbus::test::FakeEcrt::reset();
        servo_info_.vendor_id = plem::fieldbus::test::kSyntheticVendorId;
        servo_info_.product_code = plem::fieldbus::test::kSyntheticProductCode;
        servo_info_.factors = plem::fieldbus::ethercat::make_axis_factors(plem::fieldbus::test::kSyntheticServoSpec);
        servo_info_.flip = false;
        servo_info_.joint_offset = Pulse(1000);
        servo_info_.cia402_profile = plem::fieldbus::kNotReadyShutdownCia402Profile;
        servo_info_.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
        servo_info_.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;
    }

    ServoInfo servo_info_;

    static std::unique_ptr<DeviceServo> make(const ServoInfo& info) {
        return std::make_unique<DeviceServo>(plem::fieldbus::test::FakeEcrt::master(), 0, info);
    }
};

// Carry pin: get_modes_of_operation() reports the declared mode
// (ServoInfo::modes_of_operation), the 0x6060 command byte both constructors
// initialize, not a constant. This pins the carry only; whether the bus
// accepts such a ServoInfo is its own boundary (is_valid(): the three cyclic
// synchronous modes), pinned in test_servo_info / the fake-master suite.
TEST_F(DeviceServoTest, ReportsTheDeclaredOperationMode) {
    for (const auto mode : {OperationMode::CYCLIC_SYNC_POSITION, OperationMode::CYCLIC_SYNC_VELOCITY,
                            OperationMode::CYCLIC_SYNC_TORQUE}) {
        SCOPED_TRACE("mode " + std::to_string(static_cast<int>(mode)));
        auto info = servo_info_;
        info.modes_of_operation = mode;
        auto servo = make(info);
        EXPECT_EQ(servo->get_modes_of_operation(), mode);
        EXPECT_EQ(servo->get_modes_of_operation_display(), 0) << "the display is the drive's report, untouched";
    }
}

TEST_F(DeviceServoTest, FaultHandling) {
    auto servo = make(servo_info_);

    EXPECT_FALSE(servo->is_faulted());

    // Reset fault should return false when not faulted
    bool result = servo->reset_fault();
    EXPECT_FALSE(result);
}
