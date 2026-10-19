// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <plem/core/threading/seqlock.hpp>
#include <plem/core/types/joint_state.hpp>

namespace plem::core {

/**
 * @brief Shared data structure for RT-safe inter-thread communication
 *
 * Data shared in-process between EtherCATController and the host's control
 * thread.
 *
 * Data flow:
 *   EtherCATController --[joint_actual]--> control thread
 *   control thread --[command]--> EtherCATController
 *   the host's motion API --[joint_target]--> control thread
 *   DriveHost::write --[command]--> EtherCATController (drive layer: no control thread)
 *
 * @tparam DOF Degrees of freedom
 *
 * @thread_safety All members use seqlock or atomic operations
 * @realtime_safe All operations are O(1) with bounded spin
 */
template <std::size_t DOF>
struct SharedJointData {
    // Joint Data (seqlock-protected for multi-word atomicity)

    /// Actual joint values from servos (EtherCAT -> Controller)
    threading::SeqlockData<JointActualValues<DOF>> joint_actual;

    /// Target joint values (the host's motion API -> Controller)
    threading::SeqlockData<JointTargetValues<DOF>> joint_target;

    /// Joint commands (Controller or DriveHost -> EtherCAT)
    threading::SeqlockData<JointCommand<DOF>> command;

    /// Phase-lock epoch channel: the EtherCAT bus loop
    /// stamps its wake time here every cycle (ns, CLOCK_MONOTONIC); the
    /// control thread one-shot aligns its timer to epoch + phase_offset_ns.
    /// 0 = bus never stamped (free-run fallback). Independent scalar timestamp;
    /// relaxed ordering is sufficient (no data is published through it).
    std::atomic<int64_t> last_cycle_ns{0};
    static_assert(std::atomic<int64_t>::is_always_lock_free,
                  "the phase-lock epoch channel must be a lock-free atomic (RT writer/reader)");
};

}  // namespace plem::core
