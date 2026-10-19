// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_model_spec.hpp
/// @brief Drive-layer model spec: N CiA 402 drives with
/// no URDF, no torque stack: the value bundle DriveHost consumes. Composed
/// from the same leaves a higher-layer host's model spec uses (SlaveSpec /
/// AxisSpec) plus the per-axis safety declaration every drive model must
/// state (schema cia402-drive-model.md §4.4; no default exists
/// for any of them), an optional per-slave startup SDO list (§4.7) and an
/// optional per-slave list of values the drive must have stored (§4.9).
/// Pointer-free POD: assembled in C++, or loaded from a drive model file
/// (drive_model_file.hpp).
///
/// N is the exact axis count (a DriveHost<N> drives N axes; no count
/// field); slaves carry a count (≤ N: a multi-axis slave hosts several axes).
/// How a stopped axis is held, what a lost bus does and whether brakes are
/// shared are not declared at all: in v1 they are premises of the drive layer
/// (schema §4.4: power-coupled brakes, protective stop, independent brakes).

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include <plem/core/constants.hpp>
#include <plem/fieldbus/ethercat/brake_logic.hpp>
#include <plem/fieldbus/ethercat/config/configuration.hpp>

#include "plem/drive/models/ethercat_slave_spec.hpp"

namespace plem::drive {

/// Drive-layer axis bound: DriveModelSpec<N> exists for every N in
/// 1..kMaxAxes, every axis count one bus carries.
inline constexpr size_t kMaxAxes = core::kMaxBusAxes;

}  // namespace plem::drive

namespace plem::drive::models {

/// Storage width of an axis label, NUL included (schema §4.5): 31 bytes of
/// text.
inline constexpr size_t kMaxAxisNameLength = 32;

/// Storage width of the model name, NUL included: 31 bytes of text.
inline constexpr size_t kMaxModelNameLength = 32;

/// @brief Per-axis physical safety declaration (schema §4.4). Coordinates:
/// `zero_offset_count` is the hardware count at q = 0 (flip applied
/// separately, AxisSpec::flip); the position/velocity limits are in
/// the SI joint frame the loop publishes (after offset and flip); the torque
/// clamp is the drive-side limit (ServoInfo::min/max_torque_nm; min < 0 <
/// max so a stop command is representable).
/// `acceleration_rad_s2` is a csp or csv axis's stop deceleration (the file's
/// `limits.acceleration_rad_s2`): a requested stop ramps its command to rest
/// at it. 0 on a cst axis, which stops by its drive's own disable reaction.
struct AxisSafety {
    int32_t zero_offset_count = 0;
    double position_min_rad = 0.0;
    double position_max_rad = 0.0;
    double velocity_rad_s = 0.0;
    double acceleration_rad_s2 = 0.0;
    double torque_min_nm = 0.0;
    double torque_max_nm = 0.0;
    constexpr bool operator==(const AxisSafety&) const = default;
};

/// @brief One slave's startup SDO writes (schema §4.7), in write order,
/// repetition kept, never merged or sorted. DriveHost::create hands them to
/// the bus, which reserves them before activation, so they reach the drive
/// before any motion gate: reservation is assembly, an UNAPPROVED spec writes
/// too (the bus logs every write), and what a vendor object does is the
/// declarer's responsibility. A new host re-validates and re-reserves them
/// all (one-shot lifecycle).
struct StartupSdoList {
    static constexpr size_t kMaxWrites = 16;
    std::array<fieldbus::ethercat::StartupSdo, kMaxWrites> writes{};
    size_t count = 0;
    constexpr bool operator==(const StartupSdoList&) const = default;
};

/// @brief One slave's stored-value expectations (schema §4.9): the same
/// {index, subindex, size, value} elements, read as "the drive holds this".
/// The bus reads each before activation and warns on a difference; it never
/// writes one.
using StoredSdoList = StartupSdoList;

template <size_t N>
struct DriveModelSpec {
    static_assert(N >= 1 && N <= kMaxAxes, "DriveModelSpec: 1..kMaxAxes axes (uint8 joint masks)");

    std::array<SlaveSpec, N> slaves{};
    size_t slave_count = 0;
    /// Joint order; axes[i].slave_index < slave_count.
    std::array<AxisSpec, N> axes{};
    std::array<AxisSafety, N> safety{};
    /// Joint labels in axes[] order. The drive layer has no URDF, so these are
    /// the only names the node can publish; the rules are schema §4.5, enforced
    /// by validate_drive_model_spec.
    std::array<std::array<char, kMaxAxisNameLength>, N> axis_names{};
    /// Per slave, in slaves[] order. validate_drive_model_spec refuses a write
    /// the master cannot reserve and one on a core-owned object (schema §4.7).
    std::array<StartupSdoList, N> startup_sdo{};
    /// Per slave, in slaves[] order: the values the drive must have stored.
    /// validate_drive_model_spec refuses only malformed lists (schema §4.9).
    /// Not part of the config digest.
    std::array<StoredSdoList, N> stored_sdo{};
    /// Control period [us], the only source (the host has no separate knob). The
    /// file declares it in ns (`bus.cycle_time_ns`, a whole number of us).
    uint32_t cycle_time_us = 0;
    /// Distributed clocks (DC) deployment knob, the same split as a higher-layer host's
    /// `dc_enable`: a slave declares its DC capability (SlaveSpec::dc_*), the
    /// model chooses whether to run on it (the file's `bus.sync_mode`:
    /// `drive_default` = false, `dc_sync0` = true). false = the drive's own
    /// synchronization setting (the shipped default; PLEM turns no DC on),
    /// the declarations are carried but unused. true requires every
    /// slave to declare DC (validator); the bus fails fast on the rest.
    bool dc_enable = false;
    fieldbus::ethercat::FaultResetPolicy fault_reset = fieldbus::ethercat::FaultResetPolicy::NONE;
    std::optional<fieldbus::ethercat::FaultResetPacing> fault_reset_pacing{};
    /// Fault-reset pulse width [ms], the file's `fault_reset.pulse_ms`
    /// (mandatory there, 0..kMaxFaultResetPulseMs; 0 = one frame).
    uint32_t fault_reset_pulse_ms = 0;
    /// Motion admission: APPROVED only. UNAPPROVED = assembly and start()
    /// allowed, enable() refused. UNDECLARED is refused by the validator.
    Approval approval = Approval::UNDECLARED;
    std::array<char, kMaxModelNameLength> model_name{};
    /// Per slave, in slaves[] order: what the approved record claims each
    /// slave is (filled only by the file loader from a record it accepted).
    /// DriveHost::create hands the declared ones to the bus, which refuses to
    /// initialize on a mismatch. Not part of the config digest.
    std::array<fieldbus::ethercat::SlaveIdentity, N> expected_identity{};
    constexpr bool operator==(const DriveModelSpec&) const = default;
};

static_assert(std::is_trivially_copyable_v<DriveModelSpec<1>>,
              "DriveModelSpec must stay a fixed-size VALUE bundle (no resources, no parsing)");

}  // namespace plem::drive::models
