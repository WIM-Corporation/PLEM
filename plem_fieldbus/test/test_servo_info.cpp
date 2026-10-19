// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

#include <limits>
#include <sstream>
#include <string>

#include "plem/fieldbus/ethercat/config/configuration.hpp"

using namespace plem::fieldbus::ethercat;
using namespace plem::core::units;

// Struct validation tests

// A ServoInfo that declares no mode is a torque axis.
TEST(ServoInfoValidationTest, DefaultsToCyclicSyncTorque) {
    EXPECT_EQ(ServoInfo{}.modes_of_operation, plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE);
}

// A null pdo_mapping is the scanned policy (the bus reads the slave's mapping
// from the master at initialize), not an absence; it is a valid ServoInfo.
TEST(ServoInfoValidationTest, NullPdoLayoutIsValidAsScanned) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    EXPECT_TRUE(info.is_valid()) << "pdo_mapping defaults to null = scanned";
    EXPECT_NE(info.dump().find("pdo_mapping=scanned"), std::string::npos) << info.dump();
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;
    EXPECT_TRUE(info.is_valid());
    EXPECT_NE(info.dump().find("pdo_mapping=declared"), std::string::npos) << info.dump();
}

TEST(ServoInfoValidationTest, ValidInfo) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;  // mandatory
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    EXPECT_TRUE(info.is_valid());
}

// ecrt addresses a slave by a uint16 ring position: a wider value would be
// narrowed on the way to the master and configure slave (value mod 65536).
TEST(ServoInfoValidationTest, RejectsASlavePositionTheMasterCannotAddress) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    info.slave_position = 65535;
    EXPECT_TRUE(info.is_valid());
    info.slave_position = 65536;
    EXPECT_FALSE(info.is_valid());
    info.slave_position = -1;  // auto
    EXPECT_TRUE(info.is_valid());
    info.slave_position = -2;
    EXPECT_FALSE(info.is_valid());
}

// Release admission reads mode_display_timing; a servo that never declared its
// 0x6061 reporting timing must not reach the bus (no default exists).
TEST(ServoInfoValidationTest, UndeclaredModeDisplayTimingIsInvalid) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;  // mandatory
    EXPECT_FALSE(info.is_valid()) << "UNDECLARED is the absence sentinel";
    info.mode_display_timing = static_cast<plem::fieldbus::ModeDisplayTiming>(3);
    EXPECT_FALSE(info.is_valid()) << "unknown numbers are refused like the sentinel";
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::POST_ENABLE;
    EXPECT_TRUE(info.is_valid());
}

// An endless axis counts from the drive's own word: an offset would be a
// second origin the int64 count never applies.
TEST(ServoInfoValidationTest, AnEndlessAxisCarriesNoJointOffset) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), true, Pulse(0)};
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    info.position_endless = true;
    EXPECT_TRUE(info.is_valid());
    EXPECT_NE(info.dump().find("position_endless=true"), std::string::npos) << info.dump();
    info.joint_offset = Pulse(1);
    EXPECT_FALSE(info.is_valid());
    info.position_endless = false;
    EXPECT_TRUE(info.is_valid()) << "a bounded axis keeps its offset";
}

TEST(ServoInfoValidationTest, InvalidVendorId) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{0, plem::fieldbus::test::kSyntheticProductCode, make_axis_factors(spec), false,
                   Pulse(0)};  // vendor_id = 0
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    EXPECT_FALSE(info.is_valid());
}

TEST(ServoInfoValidationTest, InvalidProductCode) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, 0, make_axis_factors(spec), false,
                   Pulse(0)};  // product_code = 0
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    EXPECT_FALSE(info.is_valid());
}

TEST(ServoInfoValidationTest, InvalidSpec) {
    ServoSpec spec{0.0, 0.12, 65536, 50.0};  // invalid spec
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;  // mandatory
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    EXPECT_FALSE(info.is_valid());
}

TEST(ServoInfoValidationTest, InvalidTorqueLimits) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;  // mandatory
    info.max_torque_nm = 100.0;
    info.min_torque_nm = 200.0;  // min > max
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    EXPECT_FALSE(info.is_valid());
}

// An infinite torque limit is ordered (min < max) yet defeats the set_command
// clamp, so a non-finite bound must be rejected at validation.
TEST(ServoInfoValidationTest, RejectsNonFiniteTorqueLimit) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;  // mandatory
    info.max_torque_nm = std::numeric_limits<double>::infinity();    // no effective torque cap
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    EXPECT_FALSE(info.is_valid());
}

// A stop writes a zero torque command through the same clamp; a band that
// leaves out 0 would turn it into a nonzero torque.
TEST(ServoInfoValidationTest, RejectsATorqueBandThatLeavesOutZero) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;  // mandatory
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    info.min_torque_nm = 10.0;
    info.max_torque_nm = 20.0;
    EXPECT_FALSE(info.is_valid());
    info.min_torque_nm = -20.0;
    info.max_torque_nm = -10.0;
    EXPECT_FALSE(info.is_valid());
    info.min_torque_nm = 0.0;  // zero at the edge is inside
    info.max_torque_nm = 20.0;
    EXPECT_TRUE(info.is_valid());
}

// The declared mode of operation is the 0x6060 byte the device commands from its
// first frame. The bus serves the three cyclic synchronous modes
// (a higher layer may narrow that at its own gate); is_valid() refuses
// every other value at the bus boundary (direct ServoInfo injection included).
TEST(ServoInfoValidationTest, AcceptsTheThreeCyclicSyncModesOnly) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), false, Pulse(0)};
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;  // mandatory
    info.mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    ASSERT_TRUE(info.is_valid()) << "the default declaration is CST";
    for (const auto mode :
         {plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY}) {
        SCOPED_TRACE("mode " + std::to_string(static_cast<int>(mode)));
        info.modes_of_operation = mode;
        EXPECT_TRUE(info.is_valid());
    }
    info.modes_of_operation = plem::fieldbus::OperationMode::NO_MODE;
    EXPECT_FALSE(info.is_valid());
    info.modes_of_operation = static_cast<plem::fieldbus::OperationMode>(1);  // profile position: no command path
    EXPECT_FALSE(info.is_valid());
}

TEST(ServoInfoValidationTest, DumpFormat) {
    ServoSpec spec{100.0, 0.12, 65536, 50.0};
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode,
                   make_axis_factors(spec), true, Pulse(-100)};
    std::string dump = info.dump();
    EXPECT_TRUE(dump.find("ServoInfo") != std::string::npos);
    EXPECT_TRUE(dump.find("vendor_id=0xc0ffee") != std::string::npos);
    EXPECT_TRUE(dump.find("flip=true") != std::string::npos);
}

// The channel map is what ServoInfo validates

TEST(AxisChannelsValidationTest, ComposedFromSpecIsValidAndCarriesLiterals) {
    // Literal expectations (not re-read from the spec), so a helper that
    // crossed two fields would not equal itself.
    const auto c = make_axis_factors(ServoSpec{100.0, 0.12, 65536, 50.0, 750.3});
    EXPECT_TRUE(validate_axis_factors(c).is_ok()) << validate_axis_factors(c).message();
    EXPECT_EQ(c.torque.unit, TorqueUnit::SCALED_CURRENT);
    EXPECT_DOUBLE_EQ(c.torque.counts_per_unit, 50.0);
    EXPECT_DOUBLE_EQ(c.torque.torque_constant, 0.12);
    EXPECT_DOUBLE_EQ(c.torque.gear_ratio, 100.0);
    EXPECT_EQ(c.position.unit, MotionUnit::ROTARY_MOTOR_ENCODER);
    EXPECT_EQ(c.position.encoder_resolution, 65536u);
    EXPECT_DOUBLE_EQ(c.position.gear_ratio, 100.0);
    EXPECT_EQ(c.velocity.unit, MotionUnit::ROTARY_MOTOR_ENCODER);
    EXPECT_EQ(c.velocity.encoder_resolution, 65536u);
    EXPECT_DOUBLE_EQ(c.velocity.gear_ratio, 100.0);
    EXPECT_DOUBLE_EQ(c.velocity.velocity_factor, 750.3);
}

TEST(AxisChannelsValidationTest, DefaultConstructedIsUndeclaredAndRefused) {
    // No default unit exists: a channel map nobody composed is refused, channel by channel.
    const AxisFactors none{};
    EXPECT_NE(validate_axis_factors(none).message().find("torque channel unit is UNDECLARED"), std::string::npos);
    ServoInfo info{plem::fieldbus::test::kSyntheticVendorId, plem::fieldbus::test::kSyntheticProductCode, none, false,
                   Pulse(0)};
    EXPECT_FALSE(info.is_valid());
}

TEST(AxisChannelsValidationTest, UnknownUnitNumbersAreRefusedNotReadAsAnImplementedUnit) {
    auto c = make_axis_factors(ServoSpec{100.0, 0.12, 65536, 50.0});
    c.torque.unit = static_cast<TorqueUnit>(7);
    EXPECT_NE(validate_axis_factors(c).message().find("torque channel unit holds an out-of-range value (7)"),
              std::string::npos);
    c = make_axis_factors(ServoSpec{100.0, 0.12, 65536, 50.0});
    c.position.unit = static_cast<MotionUnit>(200);
    EXPECT_NE(validate_axis_factors(c).message().find("position channel unit holds an out-of-range value (200)"),
              std::string::npos);
    c = make_axis_factors(ServoSpec{100.0, 0.12, 65536, 50.0});
    c.velocity.unit = MotionUnit::UNDECLARED;
    EXPECT_NE(validate_axis_factors(c).message().find("velocity channel unit is UNDECLARED"), std::string::npos);
}

TEST(AxisChannelsValidationTest, EveryInvalidParameterIsRefusedWithItsOwnDiagnostic) {
    // The diagnostic projection of each channel's defect(), over a mutation
    // grid: each parameter of each channel (0, negative, NaN, inf), encoder 0,
    // and each unit (the sentinel and an out-of-range number). The message
    // must name the channel, the unit and the offending parameter with its
    // value; a validator that reported another field, or a different defect's
    // wording, fails here.
    //
    // axis_factors_valid() is asserted on the same grid: what defect()
    // unified is the per-CHANNEL rule, while axis_factors_valid is the
    // three-way conjunction above it, still separate code. Because the grid
    // mutates one channel at a time, a mutation dropping any one conjunct
    // (say `c.velocity.is_valid()`) leaves a sample this expectation refuses
    // and the conjunction accepts. The conversions' debug precondition reads
    // that bool, so it must not be covered by the string path alone.
    const auto good = make_axis_factors(ServoSpec{100.0, 0.12, 65536, 50.0, 750.3});
    EXPECT_TRUE(axis_factors_valid(good));
    EXPECT_TRUE(validate_axis_factors(good).is_ok()) << validate_axis_factors(good).message();

    auto refused = [](const AxisFactors& c, const std::string& fragment) {
        EXPECT_FALSE(axis_factors_valid(c)) << fragment;
        const std::string d = validate_axis_factors(c).message();
        EXPECT_NE(d.find(fragment), std::string::npos) << d;
    };
    // The parameter diagnostic names the value too ("gear_ratio=0 must be
    // finite and > 0", "torque_constant=inf", "velocity_factor=-1"):
    // compose the expectation with the same stream formatting the validator
    // uses, so a formatter that printed another field would not pass.
    auto param = [](const char* channel, double v) {
        std::ostringstream ss;
        ss << channel << "=" << v << " must be finite and > 0";
        return ss.str();
    };
    const double bad[] = {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()};
    for (double v : bad) {
        AxisFactors c = good;
        c.torque.counts_per_unit = v;
        refused(c, param("torque channel (SCALED_CURRENT) counts_per_unit", v));
        c = good;
        c.torque.torque_constant = v;
        refused(c, param("torque channel (SCALED_CURRENT) torque_constant", v));
        c = good;
        c.torque.gear_ratio = v;  // only the torque channel; position keeps its gear
        refused(c, param("torque channel (SCALED_CURRENT) gear_ratio", v));
        c = good;
        c.position.gear_ratio = v;
        refused(c, param("position channel (ROTARY_MOTOR_ENCODER) gear_ratio", v));
        c = good;
        c.velocity.gear_ratio = v;
        refused(c, param("velocity channel (ROTARY_MOTOR_ENCODER) gear_ratio", v));
        c = good;
        c.velocity.velocity_factor = v;
        refused(c, param("velocity channel (ROTARY_MOTOR_ENCODER) velocity_factor", v));
    }
    AxisFactors c = good;
    c.position.encoder_resolution = 0;
    refused(c, "position channel (ROTARY_MOTOR_ENCODER) encoder_resolution=0 must be > 0");
    c = good;
    c.velocity.encoder_resolution = 0;
    refused(c, "velocity channel (ROTARY_MOTOR_ENCODER) encoder_resolution=0 must be > 0");
    c = good;
    c.torque.unit = TorqueUnit::UNDECLARED;
    refused(c, "torque channel unit is UNDECLARED");
    c = good;
    c.torque.unit = static_cast<TorqueUnit>(5);
    refused(c, "torque channel unit holds an out-of-range value (5)");
    c = good;
    c.position.unit = static_cast<MotionUnit>(5);
    refused(c, "position channel unit holds an out-of-range value (5)");
    c = good;
    c.velocity.unit = MotionUnit::UNDECLARED;
    refused(c, "velocity channel unit is UNDECLARED");
}

// RATED_PER_MILLE: the CiA 402 standard torque unit

TEST(AxisChannelsValidationTest, RatedPerMilleIsScaledCurrentWithScale1000) {
    // The unit reuses the two fields: counts_per_unit is the fixed 1000 and
    // torque_constant the motor rated torque. The diagnostic names the unit
    // the channel declares, not a hardcoded one.
    auto c = make_axis_factors(ServoSpec{100.0, 0.12, 65536, 50.0});
    c.torque = {TorqueUnit::RATED_PER_MILLE, TorqueChannel::kPerMilleScale, 1.27, 100.0};
    EXPECT_TRUE(axis_factors_valid(c));
    EXPECT_TRUE(validate_axis_factors(c).is_ok()) << validate_axis_factors(c).message();

    for (const double scale : {999.0, 50.0, 0.0, std::numeric_limits<double>::quiet_NaN()}) {
        SCOPED_TRACE(scale);
        auto bad = c;
        bad.torque.counts_per_unit = scale;
        EXPECT_FALSE(axis_factors_valid(bad));
        std::ostringstream says;
        says << "torque channel (RATED_PER_MILLE) counts_per_unit=" << scale << " must be 1000";
        EXPECT_NE(validate_axis_factors(bad).message().find(says.str()), std::string::npos)
            << validate_axis_factors(bad).message();
    }
    auto bad = c;
    bad.torque.torque_constant = 0.0;
    EXPECT_NE(validate_axis_factors(bad).message().find(
                  "torque channel (RATED_PER_MILLE) torque_constant=0 must be finite and > 0"),
              std::string::npos)
        << validate_axis_factors(bad).message();
    // An out-of-range number lists both implemented torque units.
    bad = c;
    bad.torque.unit = static_cast<TorqueUnit>(7);
    EXPECT_NE(validate_axis_factors(bad).message().find("only SCALED_CURRENT or RATED_PER_MILLE is implemented"),
              std::string::npos)
        << validate_axis_factors(bad).message();
}
