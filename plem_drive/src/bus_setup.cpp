// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file bus_setup.cpp
/// @brief From a drive spec to a configured EtherCAT bus; the contract is in
/// bus_setup.hpp.

#include "plem/drive/bus_setup.hpp"

#include <string>
#include <utility>

#include <plem/fieldbus/ethercat/pdo_layout_validation.hpp>

#include "models/pdo_capacity.hpp"

namespace plem::drive {

namespace {

/// One slave's declared PDO mapping, built for the fieldbus. The counts index
/// fixed arrays here and in the fieldbus: the caller refused one beyond them
/// (first_capacity_excess) before this reads anything. The SM/PDO structure is
/// not checked here.
std::unique_ptr<const PdoMappingStorage> build_pdo_mapping(const models::SlavePdoMappingSpec& value) {
    auto out = std::make_unique<PdoMappingStorage>();
    out->value = value;  // entry storage the graph points into
    size_t pdo_node = 0;
    for (size_t s = 0; s < out->value.sync_count; ++s) {
        const auto& sync = out->value.syncs[s];
        out->sync_nodes[s] = fieldbus::ethercat::SyncManagerAssignment{
            .sm_index = sync.sm_index,
            .output = sync.output,
            .pdos = sync.pdo_count > 0 ? &out->pdo_nodes[pdo_node] : nullptr,
            .pdo_count = sync.pdo_count,
            .watchdog = sync.watchdog,
        };
        for (size_t p = 0; p < sync.pdo_count; ++p) {
            out->pdo_nodes[pdo_node++] = fieldbus::ethercat::PdoMapping{
                .index = sync.pdos[p].index,
                .entries = sync.pdos[p].entries.data(),
                .entry_count = sync.pdos[p].entry_count,
            };
        }
    }
    out->layout = fieldbus::ethercat::SlavePdoMapping{out->sync_nodes.data(), out->value.sync_count};
    return out;
}

}  // namespace

core::Result<SlavePdoMappings> build_pdo_mappings(std::span<const models::SlaveSpec> slaves, std::string_view where) {
    using Result = core::Result<SlavePdoMappings>;
    SlavePdoMappings out;
    out.layouts.assign(slaves.size(), nullptr);
    for (size_t s = 0; s < slaves.size(); ++s) {
        // Invariant for every caller: `sync_count 0` ⇔ the axes'
        // ServoInfo::pdo_mapping is nullptr (scanned). No graph is made
        // for such a slave: a materialized empty spec would be a non-null
        // `SlavePdoMapping{data, 0}` that the bus takes for a declaration and
        // refuses for its missing roles.
        if (slaves[s].pdo_mapping.sync_count == 0) {
            continue;
        }
        const auto refuse = [&](const std::string& why) {
            return Result::error("[" + std::string(where) + "] model spec slaves[" + std::to_string(s) +
                                 "].pdo_mapping: " + why);
        };
        if (const auto over = models::first_capacity_excess(slaves[s].pdo_mapping)) {
            return refuse(over->field + " (" + std::to_string(over->count) + ") exceeds the fixed capacity (" +
                          std::to_string(over->capacity) + ")");
        }
        std::unique_ptr<const PdoMappingStorage> graph = build_pdo_mapping(slaves[s].pdo_mapping);
        if (const core::Result<void> structure = fieldbus::ethercat::validate_pdo_mapping(graph->layout, {});
            !structure) {
            return refuse(structure.message());
        }
        out.layouts[s] = &graph->layout;
        out.storage.push_back(std::move(graph));
    }
    return Result::ok(std::move(out));
}

fieldbus::ethercat::ServoInfo make_servo_info(const models::SlaveSpec& slave, const models::AxisSpec& axis,
                                              const fieldbus::ethercat::SlavePdoMapping* layout,
                                              core::units::Pulse zero_offset, double torque_min_nm,
                                              double torque_max_nm, uint32_t fault_reset_pulse_ms) {
    return fieldbus::ethercat::ServoInfo{
        .vendor_id = slave.vendor_id,
        .product_code = slave.product_code,
        .factors = axis.factors,
        .flip = axis.flip,
        .joint_offset = zero_offset,
        .max_torque_nm = torque_max_nm,
        .min_torque_nm = torque_min_nm,
        .dc_assign_activate = slave.dc_assign_activate,
        .dc_sync0_shift_ns = slave.dc_sync0_shift_ns,
        .dc_sync0_cycle_ns = slave.dc_sync0_cycle_ns,
        .dc_write_cycle_time = slave.dc_write_cycle_time,
        .cia402_profile = axis.cia402_profile,
        .modes_of_operation = axis.modes_of_operation,
        .mode_display_timing = axis.mode_display_timing,
        .od_offset = axis.od_offset,
        .slave_position = slave.ring_position,
        .pdo_mapping = layout,
        .error_code_pdo = axis.error_code_pdo,
        .fault_reset_pulse_ms = fault_reset_pulse_ms,
        .position_before_enable_index = axis.position_before_enable_index,
    };
}

void configure_bus(fieldbus::ethercat::EtherCATBus& bus, std::chrono::microseconds cycle_time, bool dc_enable,
                   std::vector<fieldbus::ethercat::ServoInfo> servo_infos,
                   std::vector<std::shared_ptr<const PdoMappingStorage>> mappings) {
    // Inject the active control period so the master send interval (and
    // DC's frame-alignment check; SYNC0 itself is the drive's constant)
    // tracks the deployed rate instead of the bus's 1 ms default.
    bus.set_cycle_time_ns(static_cast<uint64_t>(cycle_time.count()) * 1000ULL);
    // DC deployment knob. DC off: PLEM configures no synchronization; the
    // drive runs its own setting.
    // The bus fail-fasts at initialize() on enable + unregistered
    // dc_assign_activate / missing injected period.
    bus.set_dc_enable(dc_enable);
    // The bus retains every declared mapping until destruction because the bus
    // may outlive the caller (see PdoMappingStorage in bus_setup.hpp).
    for (auto& mapping : mappings) {
        bus.retain_pdo_mapping_storage(std::move(mapping));
    }
    if (!servo_infos.empty()) {
        bus.set_servo_config(std::move(servo_infos));
    }
}

}  // namespace plem::drive
