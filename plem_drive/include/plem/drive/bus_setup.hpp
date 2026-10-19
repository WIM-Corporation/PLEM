// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file bus_setup.hpp
/// @brief From a drive spec to a configured EtherCAT bus: the PDO mappings the
/// bus reads, one ServoInfo per axis, and the bus settings before initialize().
///
/// DriveHost uses these functions, and so can another program that runs its
/// own EtherCAT bus from the same spec types (models/ethercat_slave_spec.hpp).
/// Call order: check the spec (models/validation.hpp), build_pdo_mappings,
/// make_servo_info per axis, configure_bus, then the bus's own initialize().
/// A spec field these functions do not copy never reaches the bus: a new
/// field in SlaveSpec or AxisSpec gets its line in make_servo_info.

#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <plem/core/result.hpp>
#include <plem/core/units/pulse.hpp>
#include <plem/fieldbus/ethercat/bus.hpp>
#include <plem/fieldbus/ethercat/config/configuration.hpp>
#include <plem/fieldbus/ethercat/pdo_layout.hpp>

#include "plem/drive/models/ethercat_slave_spec.hpp"

namespace plem::drive {

/// @brief One slave's PDO mapping in the form the fieldbus reads, with the
/// storage it points into. `layout` points at this object's own members, so
/// the object stays where it was built (on the heap, held by pointer) and
/// outlives the bus that reads `layout`; configure_bus hands it to the bus,
/// which keeps it until the bus is destroyed.
struct PdoMappingStorage {
    PdoMappingStorage() = default;
    PdoMappingStorage(const PdoMappingStorage&) = delete;  // no copy or move: `layout` points into this object
    PdoMappingStorage& operator=(const PdoMappingStorage&) = delete;

    models::SlavePdoMappingSpec value;  ///< owned copy; entry storage lives here
    std::array<fieldbus::ethercat::PdoMapping,
               models::SlavePdoMappingSpec::kMaxSyncManagers * models::SyncManagerAssignmentSpec::kMaxPdos>
        pdo_nodes{};
    std::array<fieldbus::ethercat::SyncManagerAssignment, models::SlavePdoMappingSpec::kMaxSyncManagers> sync_nodes{};
    fieldbus::ethercat::SlavePdoMapping layout{nullptr, 0};
};

/// @brief Every slave's PDO mapping, in slave order: `layouts[s]` is slave
/// s's mapping (the pointer every axis of slave s carries), or nullptr for a
/// scanned slave (sync_count 0: the bus reads the drive's own mapping at
/// initialize). `storage` owns the declared mappings; the bus keeps them for
/// its lifetime (see PdoMappingStorage).
struct SlavePdoMappings {
    std::vector<std::shared_ptr<const PdoMappingStorage>> storage;
    std::vector<const fieldbus::ethercat::SlavePdoMapping*> layouts;
};

/// @brief Builds each declared slave mapping and checks its SM/PDO structure
/// once (fieldbus validate_pdo_mapping with an empty axis span; what each
/// entry means is checked by the bus's own initialize()). Refuses with
/// "[<where>] model spec slaves[s].pdo_mapping: …" (the SlaveSpec field, also
/// the drive model file's key) on the first count beyond its array or
/// structural defect — the same Result channel as the spec check. Runs before
/// any bus contact.
[[nodiscard]] core::Result<SlavePdoMappings> build_pdo_mappings(std::span<const models::SlaveSpec> slaves,
                                                                std::string_view where);

/// @brief One axis as the fieldbus takes it: the slave/axis spec fields copied
/// verbatim plus the values no spec field holds: the zero offset, the
/// drive-side torque clamp and the model's fault-reset pulse width. All are
/// mandatory arguments so no default (ServoInfo's ±200 Nm, its 0 ms) can
/// reach the bus: each caller passes its own zero offset and torque bounds.
/// `layout` is the hosting slave's mapping
/// (SlavePdoMappings::layouts[axis.slave_index]), nullptr for a scanned slave.
[[nodiscard]] fieldbus::ethercat::ServoInfo make_servo_info(const models::SlaveSpec& slave,
                                                            const models::AxisSpec& axis,
                                                            const fieldbus::ethercat::SlavePdoMapping* layout,
                                                            core::units::Pulse zero_offset, double torque_min_nm,
                                                            double torque_max_nm, uint32_t fault_reset_pulse_ms);

/// @brief The bus settings before initialize(): control period, DC on or off,
/// the PDO mappings the bus keeps (see PdoMappingStorage) and the servos.
/// Stops there: IO config, IO factory, initialize() and the drive-count check
/// stay with the caller.
void configure_bus(fieldbus::ethercat::EtherCATBus& bus, std::chrono::microseconds cycle_time, bool dc_enable,
                   std::vector<fieldbus::ethercat::ServoInfo> servo_infos,
                   std::vector<std::shared_ptr<const PdoMappingStorage>> mappings);

}  // namespace plem::drive
