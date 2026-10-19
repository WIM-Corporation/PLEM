// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <array>
#include <cstdint>
#include <type_traits>

#include "plem/core/constants.hpp"

namespace plem::core::monitoring {

/**
 * @brief RT control-cycle diagnostics: windowed timing statistics.
 *
 * Produced and published by a higher-layer host; the drive packages allocate no
 * queue for it (nothing in them fills it). The rest describes that host.
 *
 * Emitted ~once per second (window size derived from the control period by the
 * higher-layer host's control thread) on that thread's own SPSC queue, drained by the
 * non-RT publisher into the diagnostics message on the volatile /rt_diagnostics
 * topic. Separate from the retained /rt_events safety channel; this is
 * measurement-only, never a safety trip.
 *
 * The bus_* cycle-health fields on the wire message are intentionally not here:
 * the publisher folds them from the EtherCAT RtSample drain (the bus thread's
 * own per-cycle samples), so this POD carries only what the control thread produces.
 *
 * @note Must be trivially copyable for RT-safe SPSC queue operations.
 * @note Robot tier: internal, not part of the public API; may change without notice.
 * @realtime_safe No dynamic allocation, trivially copyable.
 */
struct PLEM_CORE_PUBLIC RtDiagnostics {
    uint64_t monotonic_ns;  ///< Window-end timestamp [ns] (CLOCK_MONOTONIC; emit() auto-fills if zero)
    uint64_t sequence;      ///< Sequence number (always set by emit())

    // Aggregation Window
    uint64_t window_start_monotonic_ns;  ///< Window start [ns] (CLOCK_MONOTONIC)
    uint64_t window_end_monotonic_ns;    ///< Window end [ns] (CLOCK_MONOTONIC)
    uint32_t window_cycles;              ///< Cycles folded into this window
    uint32_t cycle_period_target_us;     ///< Configured cycle period [us] (constant)

    // Control thread: full-cycle + pure-compute timing
    float ctrl_cycle_duration_max_us;     ///< Full cycle (read+safety+compute+write, sleep excluded), max
    float ctrl_cycle_duration_mean_us;    ///< Full cycle, mean
    float ctrl_compute_duration_max_us;   ///< Pure controller compute, max (0 if none)
    float ctrl_compute_duration_mean_us;  ///< Pure controller compute, mean
    float ctrl_cycle_period_max_us;       ///< Control-thread wake-to-wake period, max
    uint32_t ctrl_cycle_overrun_count;    ///< cycle duration > target, count per window

    // Reference command chain
    float ctrl_reference_update_ratio;         ///< Fraction of cycles the producer stamp advanced [0,1]
    float ctrl_reference_age_mean_us;          ///< ROS-to-control-thread reference latency, mean
    float ctrl_reference_age_max_us;           ///< ROS-to-control-thread reference latency, max
    float ctrl_reference_interarrival_max_us;  ///< Producer publish gap, max

    // Lumped-disturbance residual (measurement-only)
    // Per-joint host-estimated external torque tau_ext [Nm], the input a higher-layer
    // host's overload guard trips on. This is the raw signed peak, not the margin to
    // the budget. The budget rises with the joint's own speed, so comparing this peak
    // against it needs the matching joint velocity (published on /joint_states).
    //
    // First axis_count slots, same as RtSample.
    uint8_t axis_count;                                               ///< Joints this window carries (1..kMaxBusAxes)
    std::array<float, kMaxBusAxes> ctrl_disturbance_residual_max_nm;  ///< signed value of the largest |.|
    /// Cycles in this window in which the residual was measured. 0 = not measured (no gravity
    /// model, no observer-backed controller, or the host stayed in its braked mode);
    /// this distinguishes "no instrument" from a genuine zero residual, which the value
    /// field alone cannot.
    uint32_t ctrl_disturbance_residual_cycles;
};

// Compile-time verification that RtDiagnostics is suitable for RT-safe queue operations
static_assert(std::is_trivially_copyable_v<RtDiagnostics>,
              "RtDiagnostics must be trivially copyable for RT-safe queue operations");

}  // namespace plem::core::monitoring
