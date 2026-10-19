// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file cia402_connection_fixture.hpp
/// @brief One complete CiA 402 bus input shared by the two halves of the
/// assembly→bus connection contract: the producer test (whatever assembly
/// layer builds the bus from model constants → EtherCATBus injection) pins
/// that the assembly hands the bus exactly this data; the consumer test
/// (plem_fieldbus fake master: EtherCATBus::initialize() → ecrt effects)
/// feeds exactly this data in. Each compares at its own boundary, so together
/// they pin the connection without linking a fake master into the assembly
/// layer (whose tests link the real fieldbus library).
///
/// Test-support header: installed only when BUILD_TESTING is ON (like every
/// header under test/include/, never part of the release surface). Data and pure builders
/// only: no assembly logic (synthesis, materialization, retention) lives here.
///
/// Every value is synthetic and belongs to no vendor. The shape is the
/// hardest one the assembly has to preserve: three two-axis slaves at ring
/// positions {5, 0, 2} (not in joint order), each hosting two NON-ADJACENT
/// joints (0/3, 1/4, 2/5) with channel 2 at +0x800. So the IO slave lands at
/// 6 (highest servo slave + 1) and the DC reference clock is the lowest ring
/// position, 0 (joints 1/4). Every per-joint value is distinct, so a joint swap inside a
/// slave pair is visible.
///
/// Torque limits are deliberately not part of this contract: the producer
/// takes them from its URDF (its test compares them against the loader), the
/// consumer leaves ServoInfo's defaults.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/cia402_types.hpp"
#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/pdo_layout.hpp"

namespace plem::fieldbus::test::connection {

using core::units::Pulse;
using ethercat::IOInfo;
using ethercat::make_axis_factors;
using ethercat::PdoEntryMapping;
using ethercat::PdoMapping;
using ethercat::ServoInfo;
using ethercat::ServoSpec;
using ethercat::SlavePdoMapping;
using ethercat::SyncManagerAssignment;

// Slave identity / distributed clocks (the three slaves are one device type)
inline constexpr uint32_t kVendorId = 0x00C0DE01;
inline constexpr uint32_t kProductCode = 0x00C0DE02;
inline constexpr uint16_t kDcAssignActivate = 0x0300;
inline constexpr int32_t kDcSync0ShiftNs = 250'000;
inline constexpr uint32_t kDcSync0CycleNs = 2'000'000;

// Per-joint values: all six distinct
inline constexpr std::array<ServoSpec, 6> kServoSpecs = {{
    {11.0, 0.051, 4096, 1.5, 2.0},
    {12.0, 0.052, 8192, 2.5, 4.0},
    {13.0, 0.053, 16384, 3.5, 0.5},
    {14.0, 0.054, 2048, 4.5, 8.0},
    {15.0, 0.055, 1024, 5.5, 0.25},
    {16.0, 0.056, 65536, 6.5, 16.0},
}};
inline constexpr std::array<bool, 6> kFlipJoints = {false, true, true, false, true, false};
inline constexpr std::array<int32_t, 6> kJointOffsets = {1234, -2345, 3456, -4567, 5678, -6789};

// Placement: joint → (slave ring position, CiA 402 channel base)
inline constexpr std::array<int32_t, 6> kSlavePositions = {5, 0, 2, 5, 0, 2};
inline constexpr std::array<uint16_t, 6> kChannelOffsets = {0x0000, 0x0000, 0x0000, 0x0800, 0x0800, 0x0800};
inline constexpr uint32_t kIoSlavePosition = 6;   ///< highest servo slave (5) + 1
inline constexpr uint32_t kDcReferenceSlave = 0;  ///< the lowest ring position (joint 0 sits on slave 5)

// ErrorCode role (channel-1 base)
inline constexpr PdoEntryMapping kErrorCodePdo{0x603F, 0x00, 16};

// CiA 402 transition profile, deliberately not kNotReadyShutdownCia402Profile:
// 12 rows in a different declaration order, without the optional
// READY_TO_SWITCH_ON → OPERATION_ENABLED row. The carry this fixture pins
// must survive a profile that differs from the shipped one in row set and
// order, so referencing the standard value here would stop exercising that.
inline constexpr Cia402TransitionProfile kCia402Profile{
    .transitions = {{
        // Standard CiA 402 walk: compatibility row 1' (NRTSO→RTSO, this FSM
        // starts there) + transitions 2-12.
        {ServoState::NOT_READY_TO_SWITCH_ON, ServoState::READY_TO_SWITCH_ON, 0x0006},
        {ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON, 0x0006},
        {ServoState::READY_TO_SWITCH_ON, ServoState::SWITCHED_ON, 0x0007},
        {ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED, 0x0000},
        {ServoState::SWITCHED_ON, ServoState::OPERATION_ENABLED, 0x000F},
        {ServoState::SWITCHED_ON, ServoState::READY_TO_SWITCH_ON, 0x0006},
        {ServoState::SWITCHED_ON, ServoState::SWITCH_ON_DISABLED, 0x0000},
        {ServoState::OPERATION_ENABLED, ServoState::SWITCHED_ON, 0x0007},
        {ServoState::OPERATION_ENABLED, ServoState::READY_TO_SWITCH_ON, 0x0006},
        {ServoState::OPERATION_ENABLED, ServoState::SWITCH_ON_DISABLED, 0x0000},
        {ServoState::OPERATION_ENABLED, ServoState::QUICK_STOP_ACTIVE, 0x0002},
        {ServoState::QUICK_STOP_ACTIVE, ServoState::SWITCH_ON_DISABLED, 0x0000},
    }},
    .transition_count = 12,
};

// Declared PDO layout: CST-complete on both channels (every role the RT
// loop reads/writes), ErrorCode mapped on both, target velocity absent
// (optional), one LOAD-BEARING 8-bit gap per PDO, mailbox SM0/SM1
namespace layout {

inline constexpr PdoEntryMapping kRx1[] = {
    {0x6040, 0x00, 16},  // Controlword
    {0x6060, 0x00, 8},   // Modes of operation
    {0x0000, 0x00, 8},   // gap: declared padding, carried verbatim (Target position at bit 32)
    {0x607A, 0x00, 32},  // Target position
    {0x6071, 0x00, 16},  // Target torque
};
inline constexpr PdoEntryMapping kRx2[] = {
    {0x6840, 0x00, 16}, {0x6860, 0x00, 8}, {0x0000, 0x00, 8}, {0x687A, 0x00, 32}, {0x6871, 0x00, 16},
};
inline constexpr PdoEntryMapping kTx1[] = {
    {0x6041, 0x00, 16},  // Statusword
    {0x6061, 0x00, 8},   // Modes of operation display
    {0x0000, 0x00, 8},   // gap
    {0x6064, 0x00, 32},  // Position actual value
    {0x606C, 0x00, 32},  // Velocity actual value
    {0x6077, 0x00, 16},  // Torque actual value
    {0x603F, 0x00, 16},  // Error code
};
inline constexpr PdoEntryMapping kTx2[] = {
    {0x6841, 0x00, 16}, {0x6861, 0x00, 8},  {0x0000, 0x00, 8},  {0x6864, 0x00, 32},
    {0x686C, 0x00, 32}, {0x6877, 0x00, 16}, {0x683F, 0x00, 16},
};
inline constexpr PdoMapping kRxPdos[] = {{0x1600, kRx1, 5}, {0x1601, kRx2, 5}};
inline constexpr PdoMapping kTxPdos[] = {{0x1A00, kTx1, 7}, {0x1A01, kTx2, 7}};
inline constexpr SyncManagerAssignment kSyncs[] = {
    {0, true, nullptr, 0, false},
    {1, false, nullptr, 0, false},
    {2, true, kRxPdos, 2, true},
    {3, false, kTxPdos, 2, false},
};

}  // namespace layout

inline constexpr SlavePdoMapping kLayout{layout::kSyncs, 4};

// One IO slave (served by the injected factory at position 6)
inline constexpr uint32_t kIoVendorId = 0x00C0DE10;
inline constexpr uint32_t kIoProductCode = 0x00C0DE11;

/// The IOInfo the bus must receive for the fixture's IO slave.
inline IOInfo io_info() {
    return IOInfo{.vendor_id = kIoVendorId, .product_code = kIoProductCode};
}

/// The six ServoInfos the bus must receive, in joint order. `pdo_mapping`
/// points at kLayout for every axis (the producer's own graph is compared by
/// content and by same-slave sharing, never by address). Torque limits stay
/// at ServoInfo's defaults; see the file comment.
inline std::vector<ServoInfo> servo_infos() {
    std::vector<ServoInfo> infos;
    infos.reserve(kServoSpecs.size());
    for (size_t i = 0; i < kServoSpecs.size(); ++i) {
        ServoInfo info{
            .vendor_id = kVendorId,
            .product_code = kProductCode,
            .factors = make_axis_factors(kServoSpecs[i]),
            .flip = kFlipJoints[i],
            .joint_offset = Pulse(kJointOffsets[i]),
            .dc_assign_activate = kDcAssignActivate,
            .dc_sync0_shift_ns = kDcSync0ShiftNs,
            .dc_sync0_cycle_ns = kDcSync0CycleNs,
            .cia402_profile = kCia402Profile,
            .mode_display_timing = ModeDisplayTiming::PRE_ENABLE,
            .od_offset = kChannelOffsets[i],
            .slave_position = kSlavePositions[i],
            .pdo_mapping = &kLayout,
            .error_code_pdo = kErrorCodePdo,
        };
        infos.push_back(std::move(info));
    }
    return infos;
}

}  // namespace plem::fieldbus::test::connection
