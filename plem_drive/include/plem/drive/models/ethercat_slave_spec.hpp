// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file ethercat_slave_spec.hpp
/// @brief Value-owned EtherCAT slave / CiA 402 axis building blocks: the
/// leaves every EtherCAT model spec is composed from (PDO layout mirrors,
/// SlaveSpec, AxisSpec), so any model spec can compose the same leaves.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <type_traits>

#include <plem/fieldbus/cia402_profile.hpp>
#include <plem/fieldbus/ethercat/brake_logic.hpp>
#include <plem/fieldbus/ethercat/config/configuration.hpp>

namespace plem::drive::models {

// PDO-layout value representation. The spec holds no pointers, so
// to_pdo_mapping_spec() flattens a model's constexpr SlavePdoMapping graph into
// these fixed-size mirrors, and create() rebuilds a stable pointer graph for
// the fieldbus lifetime contract. Field meanings, including the gap entries
// (index 0x0000 still occupies image space) and the per-SM watchdog, are
// defined in plem/fieldbus/ethercat/pdo_layout.hpp.

/// @brief One PDO and its entries, value-owned (mirror of fieldbus PdoMapping).
struct PdoMappingSpec {
    /// Fixed capacity for entries within one PDO (fixed max + count idiom).
    static constexpr size_t kMaxEntries = 16;

    uint16_t index = 0;  ///< PDO index (0 only in unused array slots)
    std::array<fieldbus::ethercat::PdoEntryMapping, kMaxEntries> entries{};
    uint8_t entry_count = 0;
    constexpr bool operator==(const PdoMappingSpec&) const = default;
};

/// @brief One sync manager and its PDOs, value-owned (mirror of fieldbus
/// SyncManagerAssignment).
struct SyncManagerAssignmentSpec {
    /// Fixed capacity for PDOs assigned to one sync manager.
    static constexpr size_t kMaxPdos = 4;

    uint8_t sm_index = 0;  ///< Sync manager number (0..fieldbus::ethercat::kSyncManagerCount-1)
    bool output = false;   ///< true = master->slave (Rx)
    std::array<PdoMappingSpec, kMaxPdos> pdos{};
    uint8_t pdo_count = 0;
    bool watchdog = false;  ///< Enable the sync-manager watchdog for this SM
    constexpr bool operator==(const SyncManagerAssignmentSpec&) const = default;
};

/// @brief A slave's complete sync-manager / PDO configuration, value-owned.
/// sync_count == 0 is the scanned policy (a model's `kPdoLayout = nullptr`,
/// a file's `pdo_mapping: scanned`): no graph is materialized and the bus
/// reads the slave's mapping from the master at initialize.
struct SlavePdoMappingSpec {
    /// Storage capacity of this value spec: how many SyncManagerAssignmentSpecs a
    /// model may declare (the shipped shapes use SM 0..3). Not the master's
    /// sm_index range, which is fieldbus::ethercat::kSyncManagerCount.
    static constexpr size_t kMaxSyncManagers = 4;

    std::array<SyncManagerAssignmentSpec, kMaxSyncManagers> syncs{};
    uint8_t sync_count = 0;
    constexpr bool operator==(const SlavePdoMappingSpec&) const = default;
};

/// @brief Flatten a model's constexpr SlavePdoMapping pointer graph into the
/// spec's value representation. constexpr on purpose: evaluated in a constant
/// expression, a layout exceeding a fixed capacity (or a count with a null
/// list) fails the build instead of a runtime assembly.
constexpr SlavePdoMappingSpec to_pdo_mapping_spec(const fieldbus::ethercat::SlavePdoMapping& layout) {
    SlavePdoMappingSpec out{};
    if (layout.sync_count > SlavePdoMappingSpec::kMaxSyncManagers) {
        throw std::length_error("SlavePdoMapping sync_count exceeds SlavePdoMappingSpec::kMaxSyncManagers");
    }
    out.sync_count = layout.sync_count;
    for (size_t s = 0; s < layout.sync_count; ++s) {
        const auto& sync = layout.syncs[s];
        auto& sync_out = out.syncs[s];
        sync_out.sm_index = sync.sm_index;
        sync_out.output = sync.output;
        sync_out.watchdog = sync.watchdog;
        if (sync.pdo_count > SyncManagerAssignmentSpec::kMaxPdos) {
            throw std::length_error("SyncManagerAssignment pdo_count exceeds SyncManagerAssignmentSpec::kMaxPdos");
        }
        if (sync.pdo_count > 0 && sync.pdos == nullptr) {
            throw std::invalid_argument("SyncManagerAssignment declares pdo_count > 0 with a null pdo list");
        }
        sync_out.pdo_count = sync.pdo_count;
        for (size_t p = 0; p < sync.pdo_count; ++p) {
            const auto& pdo = sync.pdos[p];
            auto& pdo_out = sync_out.pdos[p];
            pdo_out.index = pdo.index;
            if (pdo.entry_count > PdoMappingSpec::kMaxEntries) {
                throw std::length_error("PdoMapping entry_count exceeds PdoMappingSpec::kMaxEntries");
            }
            if (pdo.entry_count > 0 && pdo.entries == nullptr) {
                throw std::invalid_argument("PdoMapping declares entry_count > 0 with a null entry list");
            }
            pdo_out.entry_count = pdo.entry_count;
            for (size_t e = 0; e < pdo.entry_count; ++e) {
                pdo_out.entries[e] = pdo.entries[e];
            }
        }
    }
    return out;
}

/// Accepted control-period range (µs); the validator refuses a bus cycle
/// outside it. 200 µs (5 kHz) is the hardest point it admits; the hardware
/// runs recorded in docs/hardware-evidence.md are at 1 ms only, so the other
/// periods are for the bench to show on the user's setup.
/// Widen only with a new hardware soak at the new boundary.
inline constexpr int kMinCycleTimeUs = 200;
inline constexpr int kMaxCycleTimeUs = 1000;

/// @brief A drive model's approval state (the model file's `approval`).
/// UNDECLARED (the zero default) = not stated; the drive model validator
/// refuses it; APPROVED / UNAPPROVED = the approval state DriveHost
/// gates enable() on. The values are fixed (uint8_t 0/1/2).
enum class Approval : uint8_t {
    UNDECLARED = 0,
    APPROVED = 1,
    UNAPPROVED = 2,
};

/// @brief One physical drive slave, one ec_slave_config_t on the bus: its
/// ring position, identity, the PDO layout to enforce (or scanned) and its
/// distributed clocks (DC) properties. IO slaves are not here (a higher layer declares them). Value-owned like
/// everything in the spec; the fieldbus pointer graph is re-materialized per
/// slave inside create().
struct SlaveSpec {
    /// Bus ring position (the value the master addresses). The same int32_t
    /// as ServoInfo::slave_position, but never negative here: a caller that
    /// derives positions (ServoInfo's -1) resolves them before filling this field.
    int32_t ring_position = 0;
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    /// sync_count 0 = scanned (the bus reads the slave's mapping); > 0 = the
    /// declared layout the bus writes. models::check_slaves checks capacities only.
    SlavePdoMappingSpec pdo_mapping{};
    // Distributed clocks (0 = unregistered; enabling DC then fails at bus
    // init).
    uint16_t dc_assign_activate = 0;
    int32_t dc_sync0_shift_ns = 0;
    uint32_t dc_sync0_cycle_ns = 0;
    /// The drive takes its cycle time from the master: with DC on, the bus
    /// writes dc_sync0_cycle_ns to its 0x1C32:02 in PREOP (drive-model schema §5).
    bool dc_write_cycle_time = false;
    constexpr bool operator==(const SlaveSpec&) const = default;
};

/// @brief One CiA 402 axis (= one joint) and the slave hosting it. The axes
/// array is in joint order and is never re-sorted by slave or channel (joint 0
/// may sit on channel 2 of slave 0): everything indexed by joint (servo
/// spec, flip, zero offset, limits, brake mask) keeps its number.
struct AxisSpec {
    /// Index into the owning model spec's `slaves`, not a ring position.
    size_t slave_index = 0;
    /// CiA 402 channel base added to every standard object index this axis
    /// registers (0 = channel 1, 0x800 = channel 2, …).
    uint16_t od_offset = 0;
    /// The axis's channel map: per channel, the unit its counts are in and
    /// that unit's parameters; make_axis_factors() composes it from a
    /// ServoSpec; default = every unit UNDECLARED, which the spec gate
    /// refuses.
    fieldbus::ethercat::AxisFactors factors{};
    bool flip = false;
    /// Mandatory per axis (value-owned, exactly as ServoInfo carries it; no
    /// fallback table exists; see plem/fieldbus/cia402_profile.hpp).
    fieldbus::Cia402TransitionProfile cia402_profile{};
    /// CiA 402 mode of operation the axis is declared to run in: the 0x6060
    /// command from the first frame on, and what 0x6061 must display while
    /// enabled and, subject to `mode_display_timing` below, before release admission.
    /// Default CST. The shared spec check accepts CST, CSP and CSV
    /// (fieldbus::operation_mode_supported) and refuses anything else; a
    /// higher-layer host's own gate may be narrower (CST only).
    fieldbus::OperationMode modes_of_operation = fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
    /// When the drive first reports the declared mode on 0x6061 (release
    /// admission); mandatory per axis, no default: the zero value is the
    /// absence sentinel and the spec gate refuses it. Set it on every axis.
    fieldbus::ModeDisplayTiming mode_display_timing = fieldbus::ModeDisplayTiming::UNDECLARED;
    /// ErrorCode optional-PDO role: channel-1 base, index 0 = undeclared.
    fieldbus::ethercat::PdoEntryMapping error_code_pdo{0, 0, 0};
    /// Object holding the position before the drive's first enable
    /// (ServoInfo::position_before_enable_index); 0 = none. Declared by a
    /// robot model; a drive model file does not carry it.
    uint16_t position_before_enable_index = 0;
    constexpr bool operator==(const AxisSpec&) const = default;
};

static_assert(std::is_trivially_copyable_v<std::optional<fieldbus::ethercat::FaultResetPacing>>,
              "fault_reset_pacing must stay a plain optional VALUE — its payload is two integers");
static_assert(std::is_trivially_copyable_v<fieldbus::ethercat::AxisFactors>,
              "AxisFactors must stay a plain VALUE bundle — three channels of unit + numbers");

}  // namespace plem::drive::models
