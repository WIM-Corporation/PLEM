// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file pointer_free_fields.cpp
/// @brief Compile-time only: every field of the model value types, and every
/// leaf type a field aggregates, holds no pointer. The headers assert that the
/// types are trivially copyable; this adds the per-field check those cannot
/// make (a raw pointer is trivially copyable). Enumeration duty: a field added
/// to a spec struct gets its line here in the same edit.

#include <type_traits>

#include "plem/drive/models/drive_model_spec.hpp"
#include "plem/drive/models/ethercat_slave_spec.hpp"

namespace plem::drive::models {
namespace {

// A plain value field: no object pointer, no member(-function) pointer
// (std::is_member_pointer_v covers member-function pointers too). Integer-typed
// handles cannot be detected here; reviewers check them.
template <typename T>
inline constexpr bool is_pointer_free_field_v = !std::is_pointer_v<T> && !std::is_member_pointer_v<T>;

#define PLEM_SPEC_FIELD_POINTER_FREE(owner, field)                 \
    static_assert(is_pointer_free_field_v<decltype(owner::field)>, \
                  #owner "::" #field " must be a plain value: the spec holds no pointers or pointer-typed handles")

// ethercat_slave_spec.hpp
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, ring_position);
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, vendor_id);
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, product_code);
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, pdo_mapping);
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, dc_assign_activate);
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, dc_sync0_shift_ns);
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, dc_sync0_cycle_ns);
PLEM_SPEC_FIELD_POINTER_FREE(SlaveSpec, dc_write_cycle_time);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, slave_index);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, od_offset);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, factors);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, flip);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, cia402_profile);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, modes_of_operation);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, mode_display_timing);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, error_code_pdo);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSpec, position_before_enable_index);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::FaultResetPacing, interval_ms);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::FaultResetPacing, max_attempts);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::Cia402TransitionProfile, transitions);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::Cia402TransitionProfile, transition_count);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::Cia402Transition, from);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::Cia402Transition, to);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::Cia402Transition, controlword);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::AxisFactors, torque);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::AxisFactors, position);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::AxisFactors, velocity);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::TorqueChannel, unit);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::TorqueChannel, counts_per_unit);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::TorqueChannel, torque_constant);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::TorqueChannel, gear_ratio);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::PositionChannel, unit);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::PositionChannel, encoder_resolution);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::PositionChannel, gear_ratio);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::VelocityChannel, unit);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::VelocityChannel, encoder_resolution);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::VelocityChannel, gear_ratio);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::VelocityChannel, velocity_factor);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::PdoEntryMapping, index);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::PdoEntryMapping, subindex);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::PdoEntryMapping, bit_length);
PLEM_SPEC_FIELD_POINTER_FREE(PdoMappingSpec, index);
PLEM_SPEC_FIELD_POINTER_FREE(PdoMappingSpec, entries);
PLEM_SPEC_FIELD_POINTER_FREE(PdoMappingSpec, entry_count);
PLEM_SPEC_FIELD_POINTER_FREE(SyncManagerAssignmentSpec, sm_index);
PLEM_SPEC_FIELD_POINTER_FREE(SyncManagerAssignmentSpec, output);
PLEM_SPEC_FIELD_POINTER_FREE(SyncManagerAssignmentSpec, pdos);
PLEM_SPEC_FIELD_POINTER_FREE(SyncManagerAssignmentSpec, pdo_count);
PLEM_SPEC_FIELD_POINTER_FREE(SyncManagerAssignmentSpec, watchdog);
PLEM_SPEC_FIELD_POINTER_FREE(SlavePdoMappingSpec, syncs);
PLEM_SPEC_FIELD_POINTER_FREE(SlavePdoMappingSpec, sync_count);

// drive_model_spec.hpp
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, slaves);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, slave_count);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, axes);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, safety);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, axis_names);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, startup_sdo);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, stored_sdo);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, cycle_time_us);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, dc_enable);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, fault_reset);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, fault_reset_pacing);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, fault_reset_pulse_ms);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, approval);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, model_name);
PLEM_SPEC_FIELD_POINTER_FREE(DriveModelSpec<1>, expected_identity);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::SlaveIdentity, serial_number);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::SlaveIdentity, revision_number);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::SlaveIdentity, hardware_version);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::SlaveIdentity, software_version);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::SlaveIdentity, pdo_mapping_digest);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSafety, zero_offset_count);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSafety, position_min_rad);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSafety, position_max_rad);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSafety, velocity_rad_s);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSafety, acceleration_rad_s2);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSafety, torque_min_nm);
PLEM_SPEC_FIELD_POINTER_FREE(AxisSafety, torque_max_nm);
PLEM_SPEC_FIELD_POINTER_FREE(StartupSdoList, writes);
PLEM_SPEC_FIELD_POINTER_FREE(StartupSdoList, count);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::StartupSdo, index);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::StartupSdo, subindex);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::StartupSdo, size);
PLEM_SPEC_FIELD_POINTER_FREE(fieldbus::ethercat::StartupSdo, value);

#undef PLEM_SPEC_FIELD_POINTER_FREE

}  // namespace
}  // namespace plem::drive::models
