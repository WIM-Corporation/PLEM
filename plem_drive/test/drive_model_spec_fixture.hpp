// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_model_spec_fixture.hpp
/// @brief The drive-layer spec the DriveModelSpec<N> pins start from: N drives
/// on N slaves, one axis each. Test-local (not installed). Shared by the value
/// gate pins and the model-file loader's goldens, so a new spec field is
/// filled in one place and a file that should load to it is compared whole.

#pragma once

#include <cstdint>
#include <cstring>
#include <iterator>
#include <string>

#include <plem/fieldbus/cia402_profile.hpp>
#include <plem/fieldbus/ethercat/config/configuration.hpp>
#include <plem/fieldbus/test/single_axis_cst_layout.hpp>

#include "plem/drive/models/drive_model_spec.hpp"

namespace plem::drive::test {

/// The cst torque clamp every fixture axis declares (output-shaft Nm).
inline constexpr double kTorqueMinNm = -30.0;
inline constexpr double kTorqueMaxNm = 40.0;
/// The stop deceleration set_mode gives a csp or csv axis (rad/s²).
inline constexpr double kStopDecelerationRadS2 = 4.0;
/// Per-axis offsets: distinct, and axis 1 negative, so a mis-indexed or
/// sign-dropping carry cannot pass.
inline constexpr int32_t kAxisZeroOffsets[] = {1234, -77, 305, -406, 507, -608, 709, -810};

/// N drives on N slaves, one axis each: the shape every spec pin starts from.
/// One builder, so a new spec field cannot be filled at N = 1 and forgotten at
/// N = 2.
template <size_t N>
models::DriveModelSpec<N> make_n_drives() {
    static_assert(N <= std::size(kAxisZeroOffsets), "add offsets before widening the fixture");
    models::DriveModelSpec<N> spec{};
    spec.slave_count = N;
    for (size_t i = 0; i < N; ++i) {
        auto& slave = spec.slaves[i];
        slave.ring_position = static_cast<int32_t>(i);
        slave.vendor_id = fieldbus::test::kSyntheticVendorId;
        slave.product_code = fieldbus::test::kSyntheticProductCode;
        slave.pdo_mapping = models::to_pdo_mapping_spec(fieldbus::test::kSingleAxisCstLayout);
        slave.dc_assign_activate = 0;

        auto& axis = spec.axes[i];
        axis.slave_index = i;
        axis.od_offset = 0;
        axis.factors = fieldbus::ethercat::make_axis_factors(fieldbus::test::kSyntheticServoSpec);
        axis.cia402_profile = fieldbus::kStandardCia402Profile;
        axis.mode_display_timing = fieldbus::ModeDisplayTiming::PRE_ENABLE;

        spec.safety[i] = models::AxisSafety{.zero_offset_count = kAxisZeroOffsets[i],
                                            .position_min_rad = -1.0,
                                            .position_max_rad = 1.5,
                                            .velocity_rad_s = 2.0,
                                            .torque_min_nm = kTorqueMinNm,
                                            .torque_max_nm = kTorqueMaxNm};

        const std::string axis_name = "j" + std::to_string(i);
        std::memcpy(spec.axis_names[i].data(), axis_name.c_str(), axis_name.size() + 1);
    }
    spec.cycle_time_us = 1000;
    spec.approval = models::Approval::APPROVED;
    const char name[] = "bench_drive";
    std::memcpy(spec.model_name.data(), name, sizeof(name));
    return spec;
}

/// Axis `i` in `mode` with the torque limit that mode takes: the fixture
/// clamp on a cst axis, none (0/0) on a csp or csv axis; the value gate
/// refuses a clamp on any mode but cst.
template <size_t N>
void set_mode(models::DriveModelSpec<N>& spec, size_t i, fieldbus::OperationMode mode) {
    spec.axes[i].modes_of_operation = mode;
    const bool cst = mode == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
    spec.safety[i].torque_min_nm = cst ? kTorqueMinNm : 0.0;
    spec.safety[i].torque_max_nm = cst ? kTorqueMaxNm : 0.0;
    spec.safety[i].acceleration_rad_s2 = cst ? 0.0 : kStopDecelerationRadS2;
}

}  // namespace plem::drive::test
