// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file bus_from_spec.hpp
/// @brief What a drive model spec hands the bus before initialize() (not
/// installed): the servo carries, the startup and stored SDO lists and the
/// expected identities, and prepare_drive_bus, which DriveHost::create and
/// the drive probe both call, so the probe checks the drives exactly as the
/// host's start-up does.

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <plem/core/result.hpp>
#include <plem/core/units/pulse.hpp>
#include <plem/fieldbus/ethercat/bus.hpp>
#include <plem/fieldbus/ethercat/config/configuration.hpp>
#include <plem/fieldbus/ethercat/pdo_layout.hpp>

#include "drive_host_rules.hpp"
#include "models/value_gate.hpp"
#include "plem/drive/bus_setup.hpp"
#include "plem/drive/models/drive_model_spec.hpp"

namespace plem::drive::detail {

/// ServoInfo per axis: the shared carry with the spec's own zero offset and
/// torque clamp (no default reaches the bus), and an axis without end stops
/// (limits.position null → ±inf) flagged endless. `layouts[s]` = slave s's graph.
/// A csp or csv axis has no torque clamp (it sends no torque command): it
/// carries the widest finite band, which ServoInfo::is_valid accepts.
template <size_t N>
[[nodiscard]] std::vector<fieldbus::ethercat::ServoInfo> drive_servo_infos(
    const models::DriveModelSpec<N>& spec, std::span<const fieldbus::ethercat::SlavePdoMapping* const> layouts) {
    std::vector<fieldbus::ethercat::ServoInfo> out;
    out.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        const auto& axis = spec.axes[i];
        const auto& s = spec.safety[i];
        const bool cst = axis.modes_of_operation == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
        out.push_back(make_servo_info(
            spec.slaves[axis.slave_index], axis, layouts[axis.slave_index], core::units::Pulse(s.zero_offset_count),
            cst ? s.torque_min_nm : std::numeric_limits<double>::lowest(),
            cst ? s.torque_max_nm : std::numeric_limits<double>::max(), spec.fault_reset_pulse_ms));
        out.back().position_endless = axis_endless(s.position_min_rad, s.position_max_rad);
    }
    return out;
}

/// Per-slave SDO lists keyed by ring position: `count` elements of each, in
/// order: `lists` = spec.startup_sdo for EtherCATBus::set_startup_sdos,
/// spec.stored_sdo for set_stored_sdos.
template <size_t N>
[[nodiscard]] std::vector<fieldbus::ethercat::SlaveSdos> drive_sdo_lists(
    const models::DriveModelSpec<N>& spec, const std::array<models::StartupSdoList, N>& lists) {
    std::vector<fieldbus::ethercat::SlaveSdos> out;
    out.reserve(spec.slave_count);
    for (size_t s = 0; s < spec.slave_count; ++s) {
        const auto& list = lists[s];
        out.push_back({static_cast<uint32_t>(spec.slaves[s].ring_position),
                       {list.writes.begin(), list.writes.begin() + static_cast<std::ptrdiff_t>(list.count)}});
    }
    return out;
}

/// What the approved record claims each slave is, keyed by ring position, for
/// EtherCATBus::set_expected_slave_identity: only slaves with at least one
/// declared value, the PDO mapping digest included (an undeclared slave is not
/// read at all; a slave declaring only a digest is handed over; the bus
/// compares it, never skips it).
template <size_t N>
[[nodiscard]] std::vector<fieldbus::ethercat::SlaveExpectedIdentity> drive_expected_identities(
    const models::DriveModelSpec<N>& spec) {
    std::vector<fieldbus::ethercat::SlaveExpectedIdentity> out;
    for (size_t s = 0; s < spec.slave_count; ++s) {
        const fieldbus::ethercat::SlaveIdentity& id = spec.expected_identity[s];
        if (id.serial_number || id.revision_number || id.hardware_version || id.software_version ||
            id.pdo_mapping_digest) {
            out.push_back({.slave_position = static_cast<uint32_t>(spec.slaves[s].ring_position), .identity = id});
        }
    }
    return out;
}

/// Everything DriveHost::create hands the bus before initialize(), derived
/// from the spec in one place: `graphs` (the declared slaves' PDO layouts,
/// retained by the bus), the servo carries pointing into them, the startup and
/// stored SDO lists and the expected identities.
struct DriveBusInjection {
    SlavePdoMappings graphs;
    std::vector<fieldbus::ethercat::ServoInfo> servos;
    std::vector<fieldbus::ethercat::SlaveSdos> startup;
    std::vector<fieldbus::ethercat::SlaveSdos> stored;
    std::vector<fieldbus::ethercat::SlaveExpectedIdentity> expected;
};

/// The bus injection for `spec`, or the graph gate's refusal (build_pdo_mappings)
/// on a structural PDO layout defect.
template <size_t N>
[[nodiscard]] core::Result<DriveBusInjection> drive_bus_injection(const models::DriveModelSpec<N>& spec) {
    using Result = core::Result<DriveBusInjection>;
    auto graphs = build_pdo_mappings({spec.slaves.data(), spec.slave_count}, kDriveHostTag);
    if (!graphs) {
        return Result::error(graphs.message());
    }
    DriveBusInjection out;
    out.graphs = std::move(graphs).value();
    out.servos = drive_servo_infos(spec, out.graphs.layouts);
    out.startup = drive_sdo_lists(spec, spec.startup_sdo);
    out.stored = drive_sdo_lists(spec, spec.stored_sdo);
    out.expected = drive_expected_identities(spec);
    return Result::ok(std::move(out));
}

/// "[DriveHost <model name>]": the tag a host's messages lead with.
inline std::string drive_host_tag(const std::array<char, models::kMaxModelNameLength>& model_name) {
    const auto nul = std::find(model_name.begin(), model_name.end(), '\0');
    return "[DriveHost " + std::string(model_name.data(), static_cast<size_t>(nul - model_name.begin())) + "]";
}

/// The value gate (no side effect), the bus check, then the graph gate and the
/// bus injection: `bus` is ready for initialize().
template <size_t N>
[[nodiscard]] core::Result<void> prepare_drive_bus(const models::DriveModelSpec<N>& spec,
                                                   fieldbus::ethercat::EtherCATBus* bus) {
    if (core::Result<void> valid = validate_drive_model_spec(spec); !valid) {
        return valid;
    }
    if (bus == nullptr) {
        return core::Result<void>::error(drive_host_tag(spec.model_name) +
                                         " create() requires an EtherCAT bus (got nullptr)");
    }
    auto injected = drive_bus_injection(spec);
    if (!injected) {
        return core::Result<void>::error(injected.message());
    }
    DriveBusInjection injection = std::move(injected).value();
    configure_bus(*bus, std::chrono::microseconds(spec.cycle_time_us), spec.dc_enable, std::move(injection.servos),
                  std::move(injection.graphs.storage));
    bus->set_startup_sdos(std::move(injection.startup));
    bus->set_stored_sdos(std::move(injection.stored));
    bus->set_expected_slave_identity(std::move(injection.expected));
    return core::Result<void>::ok();
}

}  // namespace plem::drive::detail
