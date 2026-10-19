// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_host.cpp
/// @brief DriveModelSpec<N> value gate, the spec→fieldbus carry, the enable
/// decision and DriveHost::create's no-side-effect refusal. Pure pins only:
/// this target links the real libethercat, so the loop is proven on the
/// fieldbus fake-master target and the assembly on the bench.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <plem/fieldbus/ethercat/bus.hpp>
#include <plem/fieldbus/test/ethercat_test_probes.hpp>

#include "bus_from_spec.hpp"
#include "drive_host_rules.hpp"
#include "drive_model_spec_fixture.hpp"
#include "models/value_gate.hpp"
#include "plem/drive/bus_setup.hpp"
#include "plem/drive/drive_host.hpp"
#include "plem/drive/models/drive_model_spec.hpp"

namespace plem::drive {
namespace {

using models::Approval;
using models::DriveModelSpec;

DriveModelSpec<1> one_drive() {
    return test::make_n_drives<1>();
}

DriveModelSpec<2> two_drives() {
    return test::make_n_drives<2>();
}

// value gate
TEST(DriveModelSpecValidation, ValidSpecPasses) {
    const auto valid = detail::validate_drive_model_spec(one_drive());
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

// The drive layer serves the three cyclic synchronous modes,
// mixed on one bus; a mode without a command path is refused by the shared gate.
TEST(DriveModelSpecValidation, AcceptsMixedCyclicSyncModesAndRefusesTheRest) {
    auto spec = test::make_n_drives<3>();
    test::set_mode(spec, 1, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    test::set_mode(spec, 2, fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    spec.axes[2].modes_of_operation = fieldbus::OperationMode::NO_MODE;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message(),
              "[DriveHost] model spec axes[2].modes_of_operation (0) is not a cyclic synchronous mode — the bus serves "
              "CST (10), CSP (8) and CSV (9)");
}

TEST(DriveModelSpecValidation, RejectsSlaveCountOutsideOneToN) {
    auto spec = one_drive();
    spec.slave_count = 0;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message(),
              "[DriveHost] model spec slave_count (0) must be 1..1 — one SlaveSpec per physical drive slave, at "
              "most one per axis");
}

TEST(DriveModelSpecValidation, SharedSlaveRulesReportUnderTheDriveHostTag) {
    auto spec = one_drive();
    spec.slaves[0].vendor_id = 0;
    const auto error = detail::validate_drive_model_spec(spec).message();
    EXPECT_EQ(error.rfind("[DriveHost] model spec slaves[0] identity is incomplete", 0), 0u) << error;
}

TEST(DriveModelSpecValidation, RejectsARingPositionTheMasterCannotAddress) {
    auto spec = one_drive();
    spec.slaves[0].ring_position = 65535;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    spec.slaves[0].ring_position = 65536;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message(),
              "[DriveHost] model spec slaves[0].ring_position (65536) exceeds 65535 — the master addresses a "
              "slave by a 16-bit ring position");
}

TEST(DriveModelSpecValidation, RejectsSafetyDeclarations) {
    auto spec = one_drive();
    spec.safety[0].position_max_rad = spec.safety[0].position_min_rad;
    EXPECT_NE(
        detail::validate_drive_model_spec(spec).message().find("axes[0].limits.position must be finite with min < max"),
        std::string::npos);
    spec = one_drive();
    spec.safety[0].velocity_rad_s = 0.0;
    EXPECT_NE(
        detail::validate_drive_model_spec(spec).message().find("axes[0].limits.velocity_rad_s must be finite and > 0"),
        std::string::npos);
    spec = one_drive();
    spec.safety[0].torque_min_nm = 0.0;  // a clamp that cannot carry 0 is refused
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                  "axes[0].limits.torque_nm must be finite with min < 0 < max"),
              std::string::npos);
    spec = one_drive();
    spec.safety[0].torque_max_nm = std::numeric_limits<double>::infinity();
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("axes[0].limits.torque_nm"), std::string::npos);
}

// The torque limit clamps cst commands only: a csp or csv axis declares none
// (0/0), and one it declares is refused rather than carried with no effect.
TEST(DriveModelSpecValidation, TheTorqueLimitBelongsToACstAxisOnly) {
    for (const auto mode :
         {fieldbus::OperationMode::CYCLIC_SYNC_POSITION, fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY}) {
        auto spec = one_drive();
        spec.axes[0].modes_of_operation = mode;
        EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                      "axes[0].limits.torque_nm is declared on an axis whose mode is not cst"),
                  std::string::npos);
        spec.safety[0].torque_min_nm = 0.0;
        spec.safety[0].torque_max_nm = 0.0;
        spec.safety[0].acceleration_rad_s2 = test::kStopDecelerationRadS2;
        const auto valid = detail::validate_drive_model_spec(spec);
        EXPECT_TRUE(valid.is_ok()) << valid.message();
    }
}

// An axis without end stops declares no travel (file: `position: null`, held
// as ±inf): endless in every mode, csp included. It counts from the drive's
// raw zero; an offset would be a second origin the endless count never applies.
TEST(DriveModelSpecValidation, AnUnboundedTravelCountsFromTheRawZeroInEveryMode) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto spec = one_drive();
    spec.safety[0].position_min_rad = -kInf;
    spec.safety[0].position_max_rad = kInf;
    spec.safety[0].zero_offset_count = 0;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();

    auto offset = spec;
    offset.safety[0].zero_offset_count = 5;
    EXPECT_NE(detail::validate_drive_model_spec(offset).message().find("set zero_offset_count: 0"), std::string::npos)
        << detail::validate_drive_model_spec(offset).message();

    for (const auto mode :
         {fieldbus::OperationMode::CYCLIC_SYNC_POSITION, fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY}) {
        auto endless = spec;
        test::set_mode(endless, 0, mode);
        const auto valid2 = detail::validate_drive_model_spec(endless);
        EXPECT_TRUE(valid2.is_ok()) << valid2.message() << static_cast<int>(mode);
        endless.safety[0].zero_offset_count = -5;
        EXPECT_NE(detail::validate_drive_model_spec(endless).message().find("set zero_offset_count: 0"),
                  std::string::npos);
    }

    auto half = one_drive();
    half.safety[0].position_min_rad = -kInf;  // no file spells this
    EXPECT_NE(
        detail::validate_drive_model_spec(half).message().find("axes[0].limits.position must be finite with min < max"),
        std::string::npos);
}

// An endless csp axis writes 0x607A modulo 2^32: one cycle's largest step at
// the velocity limit, plus two counts of rounding, stays under 2^31 counts.
// Bounded csp axes and endless cst/csv axes have no such step.
TEST(DriveModelSpecValidation, AnEndlessCspStepStaysUnderHalfTheWordPeriod) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto spec = one_drive();
    test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    spec.safety[0].position_min_rad = -kInf;
    spec.safety[0].position_max_rad = kInf;
    spec.safety[0].zero_offset_count = 0;
    const double count_rad = fieldbus::ethercat::to_angle(core::units::Pulse(1), spec.axes[0].factors.position).value();
    const double cycle_s = spec.cycle_time_us * 1e-6;
    const double half_period = 2147483648.0;

    auto ok = spec;
    ok.safety[0].velocity_rad_s = (half_period - 2.5) * count_rad / cycle_s;
    const auto valid = detail::validate_drive_model_spec(ok);
    EXPECT_TRUE(valid.is_ok()) << valid.message();

    auto over = spec;
    over.safety[0].velocity_rad_s = (half_period - 1.5) * count_rad / cycle_s;  // + 2 rounding counts ≥ 2^31
    EXPECT_NE(detail::validate_drive_model_spec(over).message().find("one cycle's step must stay under 2^31 counts"),
              std::string::npos)
        << detail::validate_drive_model_spec(over).message();

    auto bounded = over;
    bounded.safety[0].position_min_rad = -1.0;
    bounded.safety[0].position_max_rad = 1.0;
    bounded.safety[0].zero_offset_count = 1234;
    const auto valid2 = detail::validate_drive_model_spec(bounded);
    EXPECT_TRUE(valid2.is_ok()) << valid2.message();
    auto cst = over;
    test::set_mode(cst, 0, fieldbus::OperationMode::CYCLIC_SYNC_TORQUE);
    const auto valid3 = detail::validate_drive_model_spec(cst);
    EXPECT_TRUE(valid3.is_ok()) << valid3.message();
}

// A csp axis's velocity limit within the count-rounding margin per cycle leaves a
// planner no speed to command (velocity_cap_rad_s <= 0): refused, with the cap it
// would get. Only csp has the step; csv and cst axes keep the limit as declared.
TEST(DriveModelSpecValidation, ACspVelocityLimitWithinTheRoundingMarginIsRefused) {
    auto spec = one_drive();
    test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    const double count_rad = fieldbus::ethercat::to_angle(core::units::Pulse(1), spec.axes[0].factors.position).value();
    const double cycle_s = spec.cycle_time_us * 1e-6;
    const double one_count_per_cycle = count_rad / cycle_s;

    auto ok = spec;
    ok.safety[0].velocity_rad_s = 1.5 * one_count_per_cycle;  // half a count per cycle is left to command
    const auto valid = detail::validate_drive_model_spec(ok);
    EXPECT_TRUE(valid.is_ok()) << valid.message();

    auto low = spec;
    low.safety[0].velocity_rad_s = one_count_per_cycle;  // the cap is exactly 0
    const auto refused = detail::validate_drive_model_spec(low);
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("axes[0].limits.velocity_rad_s"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("leaves a planner no speed to command"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("raise limits.velocity_rad_s"), std::string::npos) << refused.message();
    auto lower = spec;
    lower.safety[0].velocity_rad_s = 0.5 * one_count_per_cycle;  // a negative cap
    EXPECT_FALSE(detail::validate_drive_model_spec(lower).is_ok());

    // An axis without end stops keeps two counts: two per cycle is refused, two and a half passes.
    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto endless = spec;
    endless.safety[0].position_min_rad = -kInf;
    endless.safety[0].position_max_rad = kInf;
    endless.safety[0].zero_offset_count = 0;
    endless.safety[0].velocity_rad_s = 2.0 * one_count_per_cycle;
    EXPECT_FALSE(detail::validate_drive_model_spec(endless).is_ok());
    endless.safety[0].velocity_rad_s = 2.5 * one_count_per_cycle;
    const auto valid3 = detail::validate_drive_model_spec(endless);
    EXPECT_TRUE(valid3.is_ok()) << valid3.message();

    auto csv = low;
    test::set_mode(csv, 0, fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    const auto valid2 = detail::validate_drive_model_spec(csv);
    EXPECT_TRUE(valid2.is_ok()) << valid2.message();
}

// An endless axis decodes counts up to the domain edge (2^48), not just int32:
// a factor finite at int32 counts but not at the edge is refused there only.
TEST(DriveModelSpecValidation, AnEndlessPositionChannelKeepsTheDomainFinite) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto spec = one_drive();
    auto& channels = spec.axes[0].factors;
    channels.position.encoder_resolution = channels.velocity.encoder_resolution = 1;
    channels.torque.gear_ratio = channels.position.gear_ratio = channels.velocity.gear_ratio = 1e-298;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message().find("factors maps"), std::string::npos)
        << "finite over int32 counts: " << detail::validate_drive_model_spec(spec).message();
    spec.safety[0].position_min_rad = -kInf;
    spec.safety[0].position_max_rad = kInf;
    spec.safety[0].zero_offset_count = 0;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("factors maps the drive's endless count range"),
              std::string::npos)
        << detail::validate_drive_model_spec(spec).message();
}

// A csp axis's velocity cap keeps one count per cycle under the velocity
// limit; an endless axis keeps a second for binary64 error far past int32 counts.
TEST(DriveHostCarry, TheCspVelocityCapKeepsOneCountPerCycleTwoWhenEndless) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto spec = one_drive();
    test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    const double count_rad = fieldbus::ethercat::to_angle(core::units::Pulse(1), spec.axes[0].factors.position).value();
    const double cycle_s = spec.cycle_time_us * 1e-6;
    EXPECT_EQ(detail::drive_csp_velocity_cap(spec, 0), spec.safety[0].velocity_rad_s - count_rad / cycle_s);

    auto endless = spec;
    endless.safety[0].position_min_rad = -kInf;
    endless.safety[0].position_max_rad = kInf;
    EXPECT_EQ(detail::drive_csp_velocity_cap(endless, 0), spec.safety[0].velocity_rad_s - 2.0 * count_rad / cycle_s);
}

// The position channel must map the whole int32 count range to a finite,
// non-zero travel: an overflowing product decodes every count to 0 (motion
// unseen), an underflowing one decodes a count to ±inf (passes any bound).
TEST(DriveModelSpecValidation, APositionChannelKeepsTheCountRangeFiniteAndNonzero) {
    // 2π·2^-993 on a 1-count encoder: INT32_MAX lands just below DBL_MAX,
    // INT32_MIN (one count further) overflows; the negative end is the larger.
    for (const auto& [resolution, gear] :
         {std::pair<uint32_t, double>{65536, 1e308}, std::pair<uint32_t, double>{65536, 1e-307},
          std::pair<uint32_t, double>{1, std::ldexp(2.0 * M_PI, -993)}}) {
        SCOPED_TRACE(gear);
        auto spec = one_drive();
        auto& channels = spec.axes[0].factors;  // one factor group: every copy carries it
        channels.position.encoder_resolution = channels.velocity.encoder_resolution = resolution;
        channels.torque.gear_ratio = channels.position.gear_ratio = channels.velocity.gear_ratio = gear;
        EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("axes[0].factors maps"), std::string::npos)
            << detail::validate_drive_model_spec(spec).message();
    }
}

// The factor group is checked in the file's terms before the shared channel
// check, whose wording is the robot tier's ("torque channel (SCALED_CURRENT) …").
TEST(DriveModelSpecValidation, AFactorIsJudgedInTheFilesTerms) {
    auto spec = one_drive();
    spec.axes[0].factors.torque.counts_per_unit = 0.0;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message(),
              "[DriveHost] model spec axes[0].factors.torque.counts_per_amp (0) must be finite and > 0");
    spec = one_drive();
    auto& channels = spec.axes[0].factors;
    channels.torque.gear_ratio = channels.position.gear_ratio = channels.velocity.gear_ratio = -2.0;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message(),
              "[DriveHost] model spec axes[0].factors.gear_ratio (-2) must be finite and > 0");
    spec = one_drive();
    spec.axes[0].factors.velocity.velocity_factor = 0.0;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message(),
              "[DriveHost] model spec axes[0].factors.velocity.velocity_factor (0) must be finite and > 0");
    spec = one_drive();
    spec.axes[0].factors.position.encoder_resolution = spec.axes[0].factors.velocity.encoder_resolution = 0;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("axes[0].factors.encoder_resolution is 0"),
              std::string::npos)
        << detail::validate_drive_model_spec(spec).message();
}

// A rated_per_mille axis names its rated torque by the file key; a scale other
// than 1000 has no file key (the loader writes 1000) and keeps the shared wording.
TEST(DriveModelSpecValidation, APerMilleFactorIsJudgedInItsOwnTerms) {
    using fieldbus::ethercat::TorqueUnit;
    auto spec = one_drive();
    auto& torque = spec.axes[0].factors.torque;
    torque = {TorqueUnit::RATED_PER_MILLE, 1000.0, 1.27, torque.gear_ratio};
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    torque.torque_constant = 0.0;
    EXPECT_EQ(detail::validate_drive_model_spec(spec).message(),
              "[DriveHost] model spec axes[0].factors.torque.motor_rated_torque_nm (0) must be finite and > 0");
    torque = {TorqueUnit::RATED_PER_MILLE, 999.0, 1.27, torque.gear_ratio};
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                  "torque channel (RATED_PER_MILLE) counts_per_unit=999 must be 1000"),
              std::string::npos)
        << detail::validate_drive_model_spec(spec).message();
}

TEST(DriveModelSpecValidation, RejectsAxisNames) {
    auto spec = two_drives();
    spec.axis_names[1] = {};
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("axes[1].name is empty"), std::string::npos);

    spec = two_drives();
    std::memcpy(spec.axis_names[1].data(), "J1", 3);
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("must match [a-z_][a-z0-9_]*"), std::string::npos);

    spec = two_drives();
    std::memcpy(spec.axis_names[1].data(), "1j", 3);
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("must match [a-z_][a-z0-9_]*"), std::string::npos);

    spec = two_drives();
    spec.axis_names[1] = spec.axis_names[0];
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("repeats axes[0].name"), std::string::npos);

    spec = two_drives();
    spec.axis_names[1].fill('a');
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("is not NUL-terminated"), std::string::npos);
}

TEST(DriveModelSpecValidation, RequiresAnOutputSyncManagerWatchdog) {
    auto spec = one_drive();
    for (auto& sync : spec.slaves[0].pdo_mapping.syncs) {
        sync.watchdog = false;
    }
    const std::string refusal = detail::validate_drive_model_spec(spec).message();
    EXPECT_NE(refusal.find("no output sync manager with the watchdog enabled"), std::string::npos) << refusal;
    EXPECT_NE(refusal.find("enable the watchdog on an output sync manager"), std::string::npos) << refusal;
}

TEST(DriveModelSpecValidation, AScannedSlaveIsNotJudgedForTheWatchdog) {
    // sync_count 0 = scanned: the core writes no sync manager, so there is no
    // declaration to hold to the watchdog rule (the drive runs on its own SM
    // setting, which is not checked).
    auto spec = one_drive();
    spec.slaves[0].pdo_mapping = models::SlavePdoMappingSpec{};
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(DriveModelSpecValidation, RejectsPeriodOutsideTheEnvelopeAndUndeclaredApproval) {
    auto spec = one_drive();
    spec.cycle_time_us = 100;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                  "bus.cycle_time_ns (100000) is outside the range this release accepts 200000..1000000 ns"),
              std::string::npos)
        << detail::validate_drive_model_spec(spec).message();
    spec = one_drive();
    spec.approval = Approval::UNDECLARED;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("approval is undeclared"), std::string::npos);
    spec = one_drive();
    spec.approval = Approval::UNAPPROVED;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message() << "UNAPPROVED assembles (enable is refused later)";
}

TEST(DriveModelSpecValidation, RejectsPacingUnderNoneAndBlankName) {
    auto spec = one_drive();
    spec.fault_reset_pacing = fieldbus::ethercat::FaultResetPacing{};
    EXPECT_NE(
        detail::validate_drive_model_spec(spec).message().find("fault_reset_policy is NONE but fault_reset_pacing"),
        std::string::npos);
    spec = one_drive();
    spec.model_name.fill(' ');
    spec.model_name[31] = '\0';
    EXPECT_NE(
        detail::validate_drive_model_spec(spec).message().find("[DriveHost] model spec model_name is empty or blank"),
        std::string::npos);
}

TEST(DriveModelSpecValidation, DcEnableRequiresEverySlaveToDeclareDc) {
    auto spec = one_drive();
    spec.slaves[0].dc_assign_activate = 0;
    spec.dc_enable = false;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message() << "DC off: declarations are carried but unused";
    spec.dc_enable = true;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                  "slaves[0] declares no DC (dc.assign_activate 0) while bus.sync_mode is dc_sync0"),
              std::string::npos);
    spec.slaves[0].dc_assign_activate = 0x0300;
    const auto valid2 = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid2.is_ok()) << valid2.message();
}

TEST(DriveModelSpecValidation, WriteCycleTimeNeedsDc) {
    auto spec = one_drive();
    spec.slaves[0].dc_assign_activate = 0x0300;
    spec.slaves[0].dc_write_cycle_time = true;
    spec.dc_enable = true;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    spec.dc_enable = false;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                  "slaves[0] declares dc.write_cycle_time while bus.sync_mode is drive_default"),
              std::string::npos);
}

TEST(DriveModelSpecValidation, TwoSlavesTwoAxesPassAndCarryPerSlave) {
    const auto spec = two_drives();
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    auto graphs = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, detail::kDriveHostTag).value();
    ASSERT_EQ(graphs.layouts.size(), 2u);
    const auto infos = detail::drive_servo_infos(spec, graphs.layouts);
    ASSERT_EQ(infos.size(), 2u);
    EXPECT_EQ(infos[0].pdo_mapping, graphs.layouts[0]);
    EXPECT_EQ(infos[1].pdo_mapping, graphs.layouts[1]);
    EXPECT_EQ(infos[1].slave_position, 1);
    EXPECT_EQ(infos[1].joint_offset.value(), -77);
    auto bad = spec;
    bad.axes[1].slave_index = 2;
    EXPECT_NE(detail::validate_drive_model_spec(bad).message().find("axes[1].slave_index (2) is outside"),
              std::string::npos);
}

// The direction and the unit conversion are per axis: a flipped axis next to
// an unflipped one, each with its own encoder, reaches the bus as declared.
TEST(DriveHostCarry, EachAxisCarriesItsOwnFlipAndFactors) {
    auto spec = two_drives();
    spec.axes[1].flip = true;
    spec.axes[1].factors.position.encoder_resolution = 4096;
    spec.axes[1].factors.velocity.encoder_resolution = 4096;
    const auto valid = detail::validate_drive_model_spec(spec);
    ASSERT_TRUE(valid.is_ok()) << valid.message();
    auto graphs = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, detail::kDriveHostTag).value();
    const auto infos = detail::drive_servo_infos(spec, graphs.layouts);
    ASSERT_EQ(infos.size(), 2u);
    EXPECT_FALSE(infos[0].flip);
    EXPECT_TRUE(infos[1].flip);
    EXPECT_EQ(infos[0].factors.position.encoder_resolution, 65536u);
    EXPECT_EQ(infos[0].factors.velocity.encoder_resolution, 65536u);
    EXPECT_EQ(infos[1].factors.position.encoder_resolution, 4096u);
    EXPECT_EQ(infos[1].factors.velocity.encoder_resolution, 4096u);
    EXPECT_DOUBLE_EQ(infos[1].factors.position.gear_ratio, 100.0) << "the shared gear, untouched";
}

// bus setup
static_assert(!std::is_copy_constructible_v<PdoMappingStorage> && !std::is_move_constructible_v<PdoMappingStorage>,
              "its layout points into the object itself: a copy or a move would point into the original");

TEST(BusSetup, BuildPdoMappingsNamesTheSlaveWhoseCountsDoNotFit) {
    auto spec = two_drives();
    spec.slaves[1].pdo_mapping.syncs[0].pdo_count = models::SyncManagerAssignmentSpec::kMaxPdos + 1;
    const auto refused = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, "Host");
    ASSERT_FALSE(refused.is_ok()) << "a pdo_count beyond the storage was accepted";
    EXPECT_EQ(refused.message(),
              "[Host] model spec slaves[1].pdo_mapping: syncs[0].pdo_count (5) exceeds the fixed capacity (4)");
}

// The whole text of each capacity refusal, from the spec check and from the
// graph builder, so a shared count walk cannot change either wording.
TEST(BusSetup, CapacityRefusalsReadInFull) {
    const auto built = [](const models::SlavePdoMappingSpec& value) -> std::string {
        auto spec = one_drive();
        spec.slaves[0].pdo_mapping = value;
        const auto built = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, "Host");
        return built.is_ok() ? std::string("accepted") : built.message();
    };
    const auto checked = [](const models::SlavePdoMappingSpec& value) {
        auto spec = one_drive();
        spec.slaves[0].pdo_mapping = value;
        return models::check_slaves({spec.slaves.data(), spec.slave_count}, {spec.axes.data(), 1}, "Host").message();
    };

    models::SlavePdoMappingSpec value{};
    value.sync_count = models::SlavePdoMappingSpec::kMaxSyncManagers + 1;
    EXPECT_EQ(built(value),
              "[Host] model spec slaves[0].pdo_mapping: "
              "sync_count (5) exceeds the fixed capacity (4)");
    EXPECT_EQ(checked(value), "[Host] model spec slaves[0].pdo_mapping.sync_count (5) exceeds the fixed capacity (4)");

    value = {};
    value.sync_count = 2;
    value.syncs[1].pdo_count = models::SyncManagerAssignmentSpec::kMaxPdos + 1;
    EXPECT_EQ(built(value),
              "[Host] model spec slaves[0].pdo_mapping: "
              "syncs[1].pdo_count (5) exceeds the fixed capacity (4)");
    EXPECT_EQ(checked(value),
              "[Host] model spec slaves[0].pdo_mapping syncs[1].pdo_count (5) exceeds the fixed capacity (4)");

    value = {};
    value.sync_count = 1;
    value.syncs[0].pdo_count = 2;
    value.syncs[0].pdos[1].entry_count = models::PdoMappingSpec::kMaxEntries + 1;
    EXPECT_EQ(built(value),
              "[Host] model spec slaves[0].pdo_mapping: "
              "syncs[0].pdos[1].entry_count (17) exceeds the fixed capacity (16)");
    EXPECT_EQ(checked(value),
              "[Host] model spec slaves[0].pdo_mapping syncs[0].pdos[1].entry_count (17) exceeds the fixed capacity "
              "(16)");
}

// carry
TEST(DriveHostCarry, ServoInfoTakesTheSpecsOffsetAndTorqueClamp) {
    auto spec = one_drive();
    spec.fault_reset_pulse_ms = 20;
    auto graphs = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, detail::kDriveHostTag).value();
    const auto infos = detail::drive_servo_infos(spec, graphs.layouts);
    ASSERT_EQ(infos.size(), 1u);
    EXPECT_TRUE(infos[0].is_valid()) << infos[0].dump();
    EXPECT_EQ(infos[0].joint_offset.value(), 1234);
    EXPECT_DOUBLE_EQ(infos[0].min_torque_nm, -30.0);
    EXPECT_DOUBLE_EQ(infos[0].max_torque_nm, 40.0);
    EXPECT_EQ(infos[0].fault_reset_pulse_ms, 20u) << "the model scalar, on every axis";
    EXPECT_EQ(infos[0].slave_position, 0) << "never the -1 derivation";
    EXPECT_EQ(infos[0].pdo_mapping, graphs.layouts[0]);
    EXPECT_FALSE(infos[0].position_endless) << "bounded travel";
    const auto limits = detail::drive_limits(spec);
    EXPECT_DOUBLE_EQ(limits[0].lower_limit, -1.0);
    EXPECT_DOUBLE_EQ(limits[0].upper_limit, 1.5);
    EXPECT_DOUBLE_EQ(limits[0].velocity_limit, 2.0);
    EXPECT_DOUBLE_EQ(limits[0].effort_limit, 40.0);
}

// A csp or csv axis declares no torque limit, and none is applied (the clamp
// acts on cst commands only): its ServoInfo carries the widest finite band so
// it stays valid, and its MotionLimit no effort bound.
TEST(DriveHostCarry, AnAxisWithoutATorqueLimitCarriesNoBound) {
    auto spec = one_drive();
    test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    spec.safety[0].torque_min_nm = 0.0;
    spec.safety[0].torque_max_nm = 0.0;
    const auto valid = detail::validate_drive_model_spec(spec);
    ASSERT_TRUE(valid.is_ok()) << valid.message();
    auto graphs = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, detail::kDriveHostTag).value();
    const auto infos = detail::drive_servo_infos(spec, graphs.layouts);
    EXPECT_TRUE(infos[0].is_valid()) << infos[0].dump();
    EXPECT_EQ(infos[0].min_torque_nm, std::numeric_limits<double>::lowest());
    EXPECT_EQ(infos[0].max_torque_nm, std::numeric_limits<double>::max());
    EXPECT_EQ(detail::drive_limits(spec)[0].effort_limit, std::numeric_limits<double>::infinity());
}

// Only an axis without end stops is endless.
TEST(DriveHostCarry, OnlyANullPositionIsCarriedEndless) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    auto spec = two_drives();
    spec.safety[1].position_min_rad = -kInf;
    spec.safety[1].position_max_rad = kInf;
    spec.safety[1].zero_offset_count = 0;
    const auto valid = detail::validate_drive_model_spec(spec);
    ASSERT_TRUE(valid.is_ok()) << valid.message();
    auto graphs = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, detail::kDriveHostTag).value();
    const auto infos = detail::drive_servo_infos(spec, graphs.layouts);
    EXPECT_FALSE(infos[0].position_endless);
    EXPECT_TRUE(infos[1].position_endless);
    EXPECT_TRUE(infos[1].is_valid()) << infos[1].dump();
}

// A csp or csv axis's stop deceleration is its own; a cst axis carries none.
TEST(DriveHostCarry, EachAxisStopsAtItsOwnDeceleration) {
    auto spec = two_drives();
    test::set_mode(spec, 1, fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    spec.safety[1].acceleration_rad_s2 = 6.5;
    const auto decelerations = detail::drive_stop_decelerations(spec);
    EXPECT_EQ(decelerations[0], 0.0);
    EXPECT_EQ(decelerations[1], 6.5);
    EXPECT_TRUE(detail::drive_streams_torque(spec));
    test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    EXPECT_FALSE(detail::drive_streams_torque(spec));
}

// A csp or csv axis's stop escalates to CiA 402 Quick stop: its profile must
// declare that row (transition 11), or the escalation could not be commanded.
TEST(DriveModelSpecValidation, AStoppableAxisDeclaresTheQuickStopRow) {
    auto spec = one_drive();
    test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    const auto valid = detail::validate_drive_model_spec(spec);
    ASSERT_TRUE(valid.is_ok()) << valid.message() << "the standard table has it";
    auto& profile = spec.axes[0].cia402_profile;
    const auto row = std::find_if(profile.transitions.begin(), profile.transitions.begin() + profile.transition_count,
                                  [](const fieldbus::Cia402Transition& t) {
                                      return t.from == fieldbus::ServoState::OPERATION_ENABLED &&
                                             t.to == fieldbus::ServoState::QUICK_STOP_ACTIVE;
                                  });
    ASSERT_NE(row, profile.transitions.begin() + profile.transition_count);
    std::copy(row + 1, profile.transitions.begin() + profile.transition_count, row);
    --profile.transition_count;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("Quick stop"), std::string::npos);
    test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_TORQUE);
    const auto valid2 = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid2.is_ok()) << valid2.message() << "a cst axis stops by its drive's disable reaction";
}

// stop decision
TEST(DriveHostStop, OnlyAReleasedRunWaitsToConfirmItsBrakes) {
    using detail::StopAction;
    EXPECT_EQ(detail::decide_stop(/*started=*/false, false, false), StopAction::NOTHING);
    EXPECT_EQ(detail::decide_stop(true, /*stopped=*/true, true), StopAction::NOTHING) << "stop() runs once";
    // Started, never released: a slave short of OP never answers the
    // confirmation, so a stop there would wait out every quiesce attempt.
    EXPECT_EQ(detail::decide_stop(true, false, /*released=*/false), StopAction::STOP_LOOP);
    EXPECT_EQ(detail::decide_stop(true, false, true), StopAction::QUIESCE);
}

// A stop with nothing released observed nothing: the outcome says every joint
// is engaged (the startup state) and that no release happened, so it is never
// read as a confirmation.
TEST(DriveHostStop, AStopWithNothingReleasedSaysSoInsteadOfConfirming) {
    const fieldbus::ethercat::BrakeQuiesceOutcome outcome = detail::unreleased_stop_outcome(2);
    EXPECT_TRUE(outcome.never_released);
    EXPECT_EQ(outcome.all_joints_mask, 0x03);
    EXPECT_EQ(outcome.engaged_mask, 0x03);
    EXPECT_EQ(outcome.timeout_mask, 0x00);
    EXPECT_FALSE(fieldbus::ethercat::BrakeQuiesceOutcome{}.never_released) << "a quiesce outcome is a release's";
}

// enable decision
TEST(DriveHostEnable, DecisionTable) {
    using detail::EnableAction;
    using detail::EnableInputs;
    const EnableInputs armed{.started = true,
                             .stopped = false,
                             .approval = Approval::APPROVED,
                             .protective_stop = false,
                             .command_fresh = true,
                             .brake_engaged = 0x01,
                             .brake_operation_pending = 0,
                             .streams_torque = true};
    EXPECT_EQ(detail::decide_enable(armed), EnableAction::RELEASE);
    auto in = armed;
    in.started = false;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::NOT_STARTED);
    in = armed;
    in.stopped = true;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::NOT_STARTED);
    in = armed;
    in.approval = Approval::UNAPPROVED;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::NOT_APPROVED);
    in = armed;
    in.protective_stop = true;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::PROTECTIVE_STOP);
    in = armed;
    in.command_fresh = false;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::STALE_COMMAND);
    in = armed;
    in.brake_engaged = 0;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::ALREADY_ENABLED);
    in.brake_operation_pending = 0x01;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::RELEASE) << "an unsettled brake op is not 'enabled'";
    // A forced engage (P-STOP) leaves the brake engaged: once the stop clears
    // the next enable re-posts the release.
    in = armed;
    in.brake_engaged = 0x01;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::RELEASE);
    // The bench bootstrap opens an unapproved model and changes nothing else.
    in = armed;
    in.approval = Approval::UNAPPROVED;
    in.bench_unapproved = true;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::RELEASE);
    in.protective_stop = true;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::PROTECTIVE_STOP);
    in.protective_stop = false;
    in.command_fresh = false;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::STALE_COMMAND);
    in = armed;
    in.bench_unapproved = true;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::RELEASE) << "approved + bench: the flag is moot";
    // Without a cst axis the loop holds the position it finds: no command needed first.
    in = armed;
    in.streams_torque = false;
    in.command_fresh = false;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::RELEASE);
    // A stop still running finishes first; engaging is its end.
    in = armed;
    in.stopping = true;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::STOPPING);
    in.brake_engaged = 0;
    EXPECT_EQ(detail::decide_enable(in), EnableAction::STOPPING);
}

// endless axis: count past the drive's int32
TEST(DriveHostEndless, TurnsPastInt32OnlyOutsideTheRange) {
    constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
    constexpr int64_t kMin = std::numeric_limits<int32_t>::min();
    EXPECT_FALSE(detail::endless_turns_past_int32(0, 1000));
    EXPECT_FALSE(detail::endless_turns_past_int32(kMax, 1000)) << "upper bound is inside";
    EXPECT_FALSE(detail::endless_turns_past_int32(kMin, 1000)) << "lower bound is inside";
    EXPECT_DOUBLE_EQ(detail::endless_turns_past_int32(kMax + 1, 1000).value(), 0.001) << "+2^31";
    EXPECT_DOUBLE_EQ(detail::endless_turns_past_int32(kMin - 1, 1000).value(), 0.001) << "-2^31-1";
    EXPECT_DOUBLE_EQ(detail::endless_turns_past_int32(kMax + 2900 * int64_t{1 << 20}, 1 << 20).value(), 2900.0);
    EXPECT_DOUBLE_EQ(detail::endless_turns_past_int32(kMin - 3 * int64_t{1 << 17}, 1 << 17).value(), 3.0);
}

// create: value gate before any bus contact
TEST(DriveHostCreate, RefusesAnInvalidSpecBeforeTouchingTheBus) {
    auto spec = one_drive();
    spec.approval = Approval::UNDECLARED;
    auto bus = std::make_shared<fieldbus::ethercat::EtherCATBus>(0);
    auto result = DriveHost<1>::create(spec, bus, DriveHostParams{});
    ASSERT_FALSE(result);
    EXPECT_NE(result.message().find("approval is undeclared"), std::string::npos) << result.message();
    EXPECT_EQ(bus->servo_count(), 0u) << "no injection happened";
}

TEST(DriveHostCreate, RefusesANullBus) {
    auto result = DriveHost<1>::create(one_drive(), nullptr, DriveHostParams{});
    ASSERT_FALSE(result);
    EXPECT_NE(result.message().find("requires an EtherCAT bus"), std::string::npos) << result.message();
}

// The graph gate refuses a malformed declared mapping before any injection and
// names it slaves[s].pdo_mapping (the SlaveSpec field).
TEST(DriveHostCreate, NamesAMalformedDeclaredMappingByItsFileKey) {
    auto spec = one_drive();
    spec.slaves[0].pdo_mapping.syncs[2].pdos[0].index = 0x1A01;  // a TxPDO number under the output SM
    auto bus = std::make_shared<fieldbus::ethercat::EtherCATBus>(0);
    auto result = DriveHost<1>::create(spec, bus, DriveHostParams{});
    ASSERT_FALSE(result);
    EXPECT_NE(result.message().find("slaves[0].pdo_mapping: "), std::string::npos) << result.message();
    EXPECT_EQ(bus->servo_count(), 0u) << "the graph gate runs before any injection";
}

// startup SDO (schema §4.7)
TEST(DriveModelSpecValidation, AcceptsStartupSdosTheMasterCanReserve) {
    auto spec = two_drives();
    spec.startup_sdo[0].writes[0] = {0x2000, 0x01, 1, 0xFF};
    spec.startup_sdo[0].writes[1] = {0x2000, 0x01, 1, 0x03};  // repetition is data
    spec.startup_sdo[0].count = 2;
    spec.startup_sdo[1].writes[0] = {0x2001, 0x00, 4, 0xFFFFFFFF};
    spec.startup_sdo[1].count = 1;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(DriveModelSpecValidation, RejectsStartupSdosTheMasterCannotReserve) {
    auto spec = one_drive();
    spec.startup_sdo[0].count = models::StartupSdoList::kMaxWrites + 1;
    EXPECT_NE(
        detail::validate_drive_model_spec(spec).message().find("slaves[0].startup_sdo.count (17) exceeds the capacity"),
        std::string::npos);
    spec = one_drive();
    spec.startup_sdo[0].count = 1;
    spec.startup_sdo[0].writes[0] = {0x2000, 0x00, 3, 0};
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("the size must be 1, 2 or 4 bytes"),
              std::string::npos);
    spec.startup_sdo[0].writes[0] = {0x2000, 0x00, 1, 0x100};
    const auto error = detail::validate_drive_model_spec(spec).message();
    EXPECT_NE(error.find("slaves[0].startup_sdo[0] (0x2000:00) has size 1 and value 0x100"), std::string::npos)
        << error;
}

TEST(DriveModelSpecValidation, RejectsStartupSdosOnObjectsTheCoreOwns) {
    auto spec = one_drive();
    spec.startup_sdo[0].count = 1;
    for (const uint16_t index :
         std::initializer_list<uint16_t>{0x1C10, 0x1C2F, 0x1600, 0x17FF, 0x1A00, 0x1BFF, 0x1C32, 0x1C33}) {
        spec.startup_sdo[0].writes[0] = {index, 0x00, 1, 0};
        EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                      "is a PDO assignment/mapping or sync-manager object"),
                  std::string::npos)
            << index;
    }
    for (const uint16_t index :
         std::initializer_list<uint16_t>{0x6040, 0x6060, 0x6071, 0x607A, 0x60FF, 0x6072, 0x607C, 0x607D, 0x6080}) {
        spec.startup_sdo[0].writes[0] = {index, 0x00, 1, 0};
        EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("is axes[0]'s"), std::string::npos) << index;
    }
    // Checked at the axis's own channel (base + od_offset), as the schema's
    // example: an axis on channel 2 owns 0x6860, not 0x6060.
    spec.axes[0].od_offset = 0x800;
    spec.startup_sdo[0].writes[0] = {0x6860, 0x00, 1, 8};
    const auto error = detail::validate_drive_model_spec(spec).message();
    EXPECT_NE(error.find("(0x6860:00) is axes[0]'s 0x6060 (+ od_offset 0x0800)"), std::string::npos) << error;
    spec.startup_sdo[0].writes[0] = {0x6060, 0x00, 1, 8};
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(DriveModelSpecValidation, JudgesStartupSdosOnlyAgainstTheirSlavesAxes) {
    auto spec = two_drives();
    spec.axes[0].od_offset = 0x800;  // slave 0's axis owns 0x6860; slave 1's axis owns 0x6060
    spec.startup_sdo[1].writes[0] = {0x6860, 0x00, 1, 8};
    spec.startup_sdo[1].count = 1;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(DriveModelSpecValidation, RejectsStartupSdosForASlavePastSlaveCount) {
    auto spec = two_drives();
    spec.slave_count = 1;  // both axes on slave 0, channels 1 and 2
    spec.axes[1].slave_index = 0;
    spec.axes[1].od_offset = 0x800;
    const auto valid = detail::validate_drive_model_spec(spec);
    ASSERT_TRUE(valid.is_ok()) << valid.message();
    spec.startup_sdo[1].writes[0] = {0x2000, 0x00, 1, 0};
    spec.startup_sdo[1].count = 1;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find(
                  "slaves[1].startup_sdo declares writes for a slave past"),
              std::string::npos);
}

TEST(DriveHostCarry, StartupSdosTravelPerSlaveInWriteOrder) {
    auto spec = two_drives();
    spec.slaves[1].ring_position = 5;
    spec.startup_sdo[1].writes[0] = {0x2000, 0x01, 1, 0xAA};
    spec.startup_sdo[1].writes[1] = {0x2000, 0x01, 1, 0x03};
    spec.startup_sdo[1].count = 2;
    const auto lists = detail::drive_sdo_lists(spec, spec.startup_sdo);
    ASSERT_EQ(lists.size(), 2u);
    EXPECT_EQ(lists[0].slave_position, 0u);
    EXPECT_TRUE(lists[0].sdos.empty());
    EXPECT_EQ(lists[1].slave_position, 5u) << "the ring position, not the slave index";
    ASSERT_EQ(lists[1].sdos.size(), 2u) << "count writes, not the capacity";
    EXPECT_EQ(lists[1].sdos[0].value, 0xAAu);
    EXPECT_EQ(lists[1].sdos[1].value, 0x03u);
}

/// Stops create() at initialize() (this target has no master) after reading
/// what the assembly had injected.
class StartupSdoProbeBus : public fieldbus::test::EtherCATBusTestProbe {
  public:
    using fieldbus::test::EtherCATBusTestProbe::EtherCATBusTestProbe;
    core::Result<void> initialize() override {
        lists_at_initialize = injected_startup_sdos().size();
        stored_lists_at_initialize = injected_stored_sdos().size();
        identities_at_initialize = injected_expected_identity().size();
        return core::Result<void>::error("probe: no master");
    }
    size_t lists_at_initialize = 0;
    size_t stored_lists_at_initialize = 0;
    size_t identities_at_initialize = 0;
};

// stored SDO (schema §4.9)
TEST(DriveModelSpecValidation, AcceptsStoredSdosOnAnyObject) {
    // No static refusal: an object the bus exchanges every cycle is skipped
    // (with a warning) by the bus against the actual layout, not here.
    auto spec = two_drives();
    spec.stored_sdo[0].writes[0] = {0x6040, 0x00, 2, 0};
    spec.stored_sdo[0].writes[1] = {0x60B8, 0x00, 2, 0};
    spec.stored_sdo[0].writes[2] = {0x6072, 0x00, 2, 1000};
    spec.stored_sdo[0].writes[3] = {0x1600, 0x01, 4, 0x60400010};
    spec.stored_sdo[0].count = 4;
    spec.stored_sdo[1].writes[0] = {0x3401, 0x00, 4, 0};
    spec.stored_sdo[1].count = 1;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(DriveModelSpecValidation, RejectsMalformedStoredSdos) {
    auto spec = one_drive();
    spec.stored_sdo[0].count = models::StoredSdoList::kMaxWrites + 1;
    EXPECT_NE(
        detail::validate_drive_model_spec(spec).message().find("slaves[0].stored_sdo.count (17) exceeds the capacity"),
        std::string::npos);
    spec = one_drive();
    spec.stored_sdo[0].count = 1;
    spec.stored_sdo[0].writes[0] = {0x3015, 0x00, 1, 0x100};
    auto error = detail::validate_drive_model_spec(spec).message();
    EXPECT_NE(error.find("slaves[0].stored_sdo[0] (0x3015:00) has size 1 and value 0x100"), std::string::npos) << error;
    spec.stored_sdo[0].writes[0] = {0x3015, 0x00, 3, 0};
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("the size must be 1, 2 or 4 bytes"),
              std::string::npos);
    // Two expected values for one object contradict each other.
    spec.stored_sdo[0].writes[0] = {0x3015, 0x00, 2, 2};
    spec.stored_sdo[0].writes[1] = {0x3015, 0x00, 2, 3};
    spec.stored_sdo[0].count = 2;
    error = detail::validate_drive_model_spec(spec).message();
    EXPECT_NE(error.find("slaves[0].stored_sdo[1] (0x3015:00) is listed twice"), std::string::npos) << error;
    spec.stored_sdo[0].writes[1] = {0x3015, 0x01, 2, 3};
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message() << "another subindex is another object";
}

TEST(DriveModelSpecValidation, RejectsAStoredSdoTheSlavesStartupSdoWrites) {
    auto spec = two_drives();
    spec.startup_sdo[0].writes[0] = {0x3015, 0x00, 2, 2};
    spec.startup_sdo[0].count = 1;
    spec.stored_sdo[0].writes[0] = {0x3015, 0x00, 2, 2};
    spec.stored_sdo[0].count = 1;
    const auto error = detail::validate_drive_model_spec(spec).message();
    EXPECT_NE(error.find("slaves[0].stored_sdo[0] (0x3015:00) is also written by this slave's startup_sdo"),
              std::string::npos)
        << error;
    EXPECT_NE(error.find("reading it proves nothing; drop it from stored_sdo"), std::string::npos) << error;
    // Another slave's startup write is another drive's RAM.
    spec.stored_sdo[0].count = 0;
    spec.stored_sdo[1].writes[0] = {0x3015, 0x00, 2, 2};
    spec.stored_sdo[1].count = 1;
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(DriveModelSpecValidation, RejectsStoredSdosForASlavePastSlaveCount) {
    auto spec = two_drives();
    spec.slave_count = 1;
    spec.axes[1].slave_index = 0;
    spec.axes[1].od_offset = 0x800;
    const auto valid = detail::validate_drive_model_spec(spec);
    ASSERT_TRUE(valid.is_ok()) << valid.message();
    spec.stored_sdo[1].writes[0] = {0x3015, 0x00, 2, 2};
    spec.stored_sdo[1].count = 1;
    EXPECT_NE(
        detail::validate_drive_model_spec(spec).message().find("slaves[1].stored_sdo lists values for a slave past"),
        std::string::npos);
}

// The whole text of each list-shape refusal the two SDO lists share, so the
// shared wording cannot drift between them.
TEST(DriveModelSpecValidation, SdoListShapeRefusalsReadInFull) {
    auto past = two_drives();
    past.slave_count = 1;
    past.axes[1].slave_index = 0;
    past.axes[1].od_offset = 0x800;
    past.startup_sdo[1].writes[0] = {0x2000, 0x00, 1, 0};
    past.startup_sdo[1].count = 1;
    EXPECT_EQ(detail::validate_drive_model_spec(past).message(),
              "[DriveHost] model spec slaves[1].startup_sdo declares writes for a slave past slave_count (1) — "
              "nothing would reserve them");
    past.startup_sdo[1].count = 0;
    past.stored_sdo[1].writes[0] = {0x3015, 0x00, 2, 2};
    past.stored_sdo[1].count = 1;
    EXPECT_EQ(detail::validate_drive_model_spec(past).message(),
              "[DriveHost] model spec slaves[1].stored_sdo lists values for a slave past slave_count (1) — nothing "
              "would read them");

    auto full = one_drive();
    full.startup_sdo[0].count = models::StartupSdoList::kMaxWrites + 1;
    EXPECT_EQ(detail::validate_drive_model_spec(full).message(),
              "[DriveHost] model spec slaves[0].startup_sdo.count (17) exceeds the capacity of 16 writes");
    full = one_drive();
    full.stored_sdo[0].count = models::StoredSdoList::kMaxWrites + 1;
    EXPECT_EQ(detail::validate_drive_model_spec(full).message(),
              "[DriveHost] model spec slaves[0].stored_sdo.count (17) exceeds the capacity of 16 values");

    auto bad = one_drive();
    bad.startup_sdo[0].writes[0] = {0x2000, 0x01, 3, 0x7};
    bad.startup_sdo[0].count = 1;
    EXPECT_EQ(detail::validate_drive_model_spec(bad).message(),
              "[DriveHost] model spec slaves[0].startup_sdo[0] (0x2000:01) has size 3 and value 0x7 — the size must be "
              "1, 2 or 4 bytes and the value must fit in it");
    bad = one_drive();
    bad.stored_sdo[0].writes[0] = {0x3015, 0x02, 1, 0x1ff};
    bad.stored_sdo[0].count = 1;
    EXPECT_EQ(
        detail::validate_drive_model_spec(bad).message(),
        "[DriveHost] model spec slaves[0].stored_sdo[0] (0x3015:02) has size 1 and value 0x1ff — the size must be "
        "1, 2 or 4 bytes and the value must fit in it");
}

TEST(DriveHostCarry, StoredSdosTravelPerSlaveByBusPosition) {
    auto spec = two_drives();
    spec.slaves[1].ring_position = 5;
    spec.stored_sdo[1].writes[0] = {0x3015, 0x00, 2, 2};
    spec.stored_sdo[1].writes[1] = {0x3401, 0x00, 4, 0};
    spec.stored_sdo[1].count = 2;
    const auto lists = detail::drive_sdo_lists(spec, spec.stored_sdo);
    ASSERT_EQ(lists.size(), 2u);
    EXPECT_TRUE(lists[0].sdos.empty());
    EXPECT_EQ(lists[1].slave_position, 5u) << "the ring position, not the slave index";
    ASSERT_EQ(lists[1].sdos.size(), 2u) << "count values, not the capacity";
    EXPECT_EQ(lists[1].sdos[0], spec.stored_sdo[1].writes[0]);
    EXPECT_EQ(lists[1].sdos[1], spec.stored_sdo[1].writes[1]);
}

TEST(DriveHostCarry, ExpectedIdentitiesTravelPerDeclaredSlaveByBusPosition) {
    auto spec = two_drives();
    spec.slaves[1].ring_position = 5;
    EXPECT_TRUE(detail::drive_expected_identities(spec).empty()) << "nothing declared, nothing read";
    spec.expected_identity[1].software_version = std::array<char, 32>{'A', '.', '6', '0'};
    const auto out = detail::drive_expected_identities(spec);
    ASSERT_EQ(out.size(), 1u) << "slave 0 declares nothing";
    EXPECT_EQ(out[0].slave_position, 5u) << "the ring position, not the slave index";
    EXPECT_EQ(out[0].identity, spec.expected_identity[1]);
    spec.expected_identity[0].serial_number = 0;  // 0 is a declared value
    EXPECT_EQ(detail::drive_expected_identities(spec).size(), 2u);
    // A slave whose record claims only the scanned mapping's digest is handed
    // over too: the bus compares it; a filter that dropped it would let the
    // slave pass uncompared.
    auto digest_only = two_drives();
    std::array<char, fieldbus::ethercat::SlaveIdentity::kDigestLength> digest{};
    std::copy_n("sha256:63f608fc09430a5f804e8c5506e06212013d43b91bdd252031105b90c28e5d8f",
                fieldbus::ethercat::SlaveIdentity::kDigestLength - 1, digest.begin());
    digest_only.expected_identity[1].pdo_mapping_digest = digest;
    const auto handed = detail::drive_expected_identities(digest_only);
    ASSERT_EQ(handed.size(), 1u) << "a digest alone is a declared value";
    EXPECT_EQ(handed[0].identity.pdo_mapping_digest, digest);
    auto revision_only = two_drives();
    revision_only.expected_identity[0].revision_number = 0;  // 0 is a declared value
    ASSERT_EQ(detail::drive_expected_identities(revision_only).size(), 1u) << "a revision alone is a declared value";
}

// One create() carries all three lists: each is on the bus before initialize().
TEST(DriveHostCreate, HandsTheSdoListsAndIdentitiesToTheBusBeforeInitialize) {
    auto spec = one_drive();
    spec.startup_sdo[0].writes[0] = {0x2000, 0x01, 1, 0xAA};
    spec.startup_sdo[0].count = 1;
    spec.stored_sdo[0].writes[0] = {0x3015, 0x00, 2, 2};
    spec.stored_sdo[0].count = 1;
    spec.expected_identity[0].hardware_version = std::array<char, 32>{'0', '.', '2'};
    auto bus = std::make_shared<StartupSdoProbeBus>(0u);
    ASSERT_FALSE(DriveHost<1>::create(spec, bus, DriveHostParams{})) << "the probe refuses initialize()";
    ASSERT_EQ(bus->lists_at_initialize, 1u);
    ASSERT_EQ(bus->injected_startup_sdos()[0].sdos.size(), 1u);
    EXPECT_EQ(bus->injected_startup_sdos()[0].sdos[0].index, 0x2000);
    ASSERT_EQ(bus->stored_lists_at_initialize, 1u);
    ASSERT_EQ(bus->injected_stored_sdos()[0].sdos.size(), 1u);
    EXPECT_EQ(bus->injected_stored_sdos()[0].sdos[0].index, 0x3015);
    ASSERT_EQ(bus->identities_at_initialize, 1u);
    EXPECT_EQ(bus->injected_expected_identity()[0].identity, spec.expected_identity[0]);
}

}  // namespace
}  // namespace plem::drive
