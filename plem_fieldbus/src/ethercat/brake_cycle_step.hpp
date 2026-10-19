// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <plem/core/types/tri_state.hpp>

#include "ethercat/internal_visibility.hpp"
#include "plem/fieldbus/ethercat/brake_logic.hpp"

namespace plem::fieldbus::ethercat {

// Package-private brake cycle-step shell (deliberately not an installed
// header): the one place that fixes the per-cycle ordering around the pure
// decisions in brake_logic.hpp. Templated so the ordering itself is testable
// with scripted fakes (two-sample toggles are non-deterministic on the
// fake-master harness); the only production instantiation lives in
// controller.cpp (real ControlModeFlags / std::atomic masks / inline apply;
// no virtuals, no std::function, no allocation).

/// @brief Per-cycle inputs of one brake cycle step
struct BrakeCycleInputs {
    core::TriState motion_possible{core::TriState::UNDEFINED};  ///< EtherCATBus::motion_possible()
    /// Release-admission sample for this cycle (the request leg's
    /// consume-time re-check; a close landing mid-cycle is applied from the
    /// next cycle at the latest).
    bool release_admission_closed{false};
    /// In-service trip onset (mode trip or enable loss) detected by the
    /// previous sequencing step (see BrakeControlInputs::mode_trip); the
    /// controller hands it over once and
    /// clears it. Control leg only: the request leg sees its consequence
    /// through the control decision's engage_pending, as for any trigger.
    bool mode_trip{false};
};

/// @brief Production MasksLike: references to the controller's atomic
/// request masks and busy mirror (release/engage/busy member names are the
/// template contract).
struct BrakeRequestMaskRefs {
    std::atomic<uint8_t>& release;
    std::atomic<uint8_t>& engage;
    /// Settled-mirror (controller brake_operation_busy_mask_): the consume must
    /// publish a bit here before clearing it from the request mask.
    std::atomic<uint8_t>& busy;
};

/// @brief One brake step per bus cycle, fixed observable order:
///
///   1st stop sample -> decide_control -> apply(control effects)
///   -> consume release, engage (load, busy.fetch_or, fetch_and of the
///      loaded bits)                                       (unconditional)
///   -> 2nd stop sample + mode sample + readiness(consumed release)
///   -> decide_requested -> apply(request effects)
///
/// Ordering contracts (each pinned by test_brake_cycle_step.cpp):
/// - Control effects apply before the consume: a same-cycle mask request
///   is consumed after the safety-forced engage landed, so a requested
///   engage overwrites that sequence, never the reverse.
/// - The masks are consumed unconditionally, release before engage; a
///   suppressed release is discarded, never re-queued.
/// - The protective-stop flag is sampled twice (before decide_control, after
///   the consume): a stop landing between the samples is observable
///   behavior, not an artifact — collapsing to one snapshot is a concurrency
///   semantics change, not a refactor. The committed mode is sampled once,
///   after the consume; only the request leg reads it.
///
/// @tparam FlagsLike shaped like plem::core::ControlModeFlags
///                   (is_protective_stop_active(), the RT-latched stop
///                   level; mode identity does not come from flags)
/// @tparam ModeLike callable `CommittedModeSample()` returning the committed
///                   mode. Production reads the mode state machine's CAS word
///                   through the controller's probe, so motion and the protective stop
///                   recovery lane come from one load. Sampled once, after the
///                   consume.
/// @tparam ReadyLike callable `uint8_t(uint8_t consumed_release) noexcept`:
///                   the release readiness mask (BrakeRequestInputs::
///                   release_ready, closed over the brake groups; 0 =
///                   fail-closed), asked once after the consume with the
///                   release bits this step actually consumed, so the
///                   readiness can be checked for exactly that request.
/// @tparam MasksLike shaped like BrakeRequestMaskRefs (atomic release, engage
///                   and busy members)
/// @tparam ApplyLike inline callable applying one BrakeEffect (template, no
///                   std::function). Must not throw: the production apply
///                   chain (apply_brake_effect: sequence starts, PLEM_EMIT) is
///                   noexcept end-to-end; a throwing
///                   apply terminates (RT fail-fast).
/// @return the control decision's next state (the request leg never updates it)
template <typename FlagsLike, typename ModeLike, typename ReadyLike, typename MasksLike, typename ApplyLike>
[[nodiscard]] PLEM_FIELDBUS_INTERNAL BrakeLogicState run_brake_cycle_step(
    const BrakeCycleInputs& inputs, FlagsLike& flags, ModeLike&& mode, ReadyLike&& ready, MasksLike& masks,
    const BrakeLogicState& state, uint8_t all_joints_mask, ApplyLike&& apply) noexcept {
    const bool protective_stop_first = flags.is_protective_stop_active();

    const BrakeControlDecision control = decide_control_brake({.motion_possible = inputs.motion_possible,
                                                               .protective_stop_active = protective_stop_first,
                                                               .mode_trip = inputs.mode_trip},
                                                              state, all_joints_mask);
    for (size_t i = 0; i < control.count; ++i) {
        apply(control.effects[i]);
    }

    // Linearized consume: a bit must be visible in the busy mirror
    // before it leaves the request mask; with a plain exchange a poller
    // could read request=0 (already consumed) ∧ busy=0 (mirror published
    // only at cycle end) and conclude "settled" at the exact moment an op
    // begins. Load → publish to busy → clear exactly the loaded bits.
    //
    // Late-post contract (level mailbox, not a counter): a post of a
    // bit that was 0 at the load stays queued for the next cycle (never
    // lost, never invisible: still in the request mask the pending getter
    // reads first). A re-post of a bit already being consumed this cycle is
    // merged into that consume by the fetch_and: same joint, same
    // direction, already executing: the request is level-encoded, so "make
    // it so" twice is one op. Distinct-direction posts are separate words
    // and unaffected.
    const uint8_t release_mask = masks.release.load(std::memory_order_acquire);
    const uint8_t engage_mask = masks.engage.load(std::memory_order_acquire);
    masks.busy.fetch_or(static_cast<uint8_t>(release_mask | engage_mask), std::memory_order_release);
    masks.release.fetch_and(static_cast<uint8_t>(~release_mask), std::memory_order_acq_rel);
    masks.engage.fetch_and(static_cast<uint8_t>(~engage_mask), std::memory_order_acq_rel);

    const bool protective_stop_second = flags.is_protective_stop_active();
    const CommittedModeSample committed = mode();
    const uint8_t release_ready = ready(release_mask);

    const BrakeRequestDecision requested =
        decide_requested_brake({.release_mask = release_mask,
                                .engage_mask = engage_mask,
                                .protective_stop_active = protective_stop_second,
                                .protective_stop_recovery = committed.protective_stop_recovery,
                                .motion_enabled = committed.motion,
                                .engage_pending = control.next_state.engage_pending,
                                .release_admission_closed = inputs.release_admission_closed,
                                .release_ready = release_ready});
    for (size_t i = 0; i < requested.count; ++i) {
        apply(requested.effects[i]);
    }

    return control.next_state;
}

}  // namespace plem::fieldbus::ethercat
