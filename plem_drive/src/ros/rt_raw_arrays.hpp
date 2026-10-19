// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file rt_raw_arrays.hpp
/// @brief The arrays of plem_rt_msgs/RtRawBatch, listed once for the batcher
/// and the relay (not installed).

#pragma once

#include <plem/core/monitoring/rt_sample.hpp>
#include <plem_rt_msgs/msg/rt_raw_batch.hpp>

namespace plem::drive::ros {

/// Calls `per_sample(name, batch_array, sample_field)` for every array with
/// one entry per sample, then `per_joint(name, batch_array, sample_field)` for
/// every array with axis_count entries per sample (sample-major); each pair is
/// a member pointer into RtRawBatch and the RtSample field it batches. A new
/// message array is added here, in this order (the relay reports the first
/// array whose length is wrong).
template <typename PerSample, typename PerJoint>
void for_each_rt_raw_array(PerSample&& per_sample, PerJoint&& per_joint) {
    using B = plem_rt_msgs::msg::RtRawBatch;
    using S = plem::core::monitoring::RtSample;
    per_sample("monotonic_ns", &B::monotonic_ns, &S::monotonic_ns);
    per_sample("sequence", &B::sequence, &S::sequence);
    per_sample("loop_exec_us", &B::loop_exec_us, &S::loop_exec_us);
    per_sample("loop_jitter_us", &B::loop_jitter_us, &S::loop_jitter_us);
    per_sample("deadline_miss", &B::deadline_miss, &S::deadline_miss);
    per_sample("wkc", &B::wkc, &S::wkc);
    per_sample("wkc_mismatch", &B::wkc_mismatch, &S::wkc_mismatch);
    per_sample("dc_sync_monitor_ns", &B::dc_sync_monitor_ns, &S::dc_sync_monitor_ns);
    per_joint("actual_position", &B::actual_position, &S::actual_position);
    per_joint("actual_velocity", &B::actual_velocity, &S::actual_velocity);
    per_joint("actual_torque", &B::actual_torque, &S::actual_torque);
    per_joint("command_position", &B::command_position, &S::command_position);
    per_joint("command_velocity", &B::command_velocity, &S::command_velocity);
    per_joint("command_torque", &B::command_torque, &S::command_torque);
    per_joint("applied_command", &B::applied_command, &S::applied_command);
    per_joint("statusword", &B::statusword, &S::statusword);
    per_joint("modes_of_operation_display", &B::modes_of_operation_display, &S::modes_of_operation_display);
}

}  // namespace plem::drive::ros
