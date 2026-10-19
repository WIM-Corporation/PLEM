// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <type_traits>

#include "ethercat/internal_visibility.hpp"
#include "plem/fieldbus/ethercat/brake_logic.hpp"

namespace plem::fieldbus::ethercat {

// Package-private per-joint sequence-step decision (deliberately not an
// installed header): the one place that owns the per-cycle priority of the
// CiA 402 walk (arrival > declared fault reset > proven engage fallback >
// transition command), plus every stall/reset counter edge. The executor
// (EtherCATController::execute_state_transitions) samples the drive into
// values, calls the decision, and performs the effects; group settlement
// physics and event emission stay on the executor side.
//
// Pure constexpr decision with POD structs and an enum; no virtuals or
// allocation. Table-tested in test_sequence_step.cpp; golden traces pin
// cycle-exact end-to-end behavior.

/// @brief Per-joint mutable walk state: the four controller counters/latches
/// as one value. The controller's arrays remain the single source of truth;
/// this struct is a one-cycle working copy (same idiom as WatchdogState).
struct SequenceStepState {
    uint32_t stall_cycles{0};    ///< cycles off the target step this episode
    bool stall_reported{false};  ///< episode latch, one BRAKE_SEQUENCE_STALLED per episode
    uint32_t reset_cycles{0};    ///< cycles toward the next fault-reset attempt
    uint32_t reset_attempts{0};  ///< accepted reset requests this episode, not resets performed
                                 ///< and not drive recoveries (max+1 = give-up emitted)
};

/// @brief One joint's cycle sample, taken before deciding. The PDO-derived
/// flags (initialized/at_target/is_faulted/can_transition/
/// engage_settle_proven) all come from this cycle's single PDO read; the
/// sequence facts (release_target/policy/state) ride along so the decision
/// touches no controller state.
///
/// Claim scope: `can_transition` is the profile answer FOR THE CURRENT
/// TARGET only — the decision never asks about any other state, and widening
/// that is an interface change, not an executor detail. `engage_settle_proven`
/// is the composed fact `status_recognized() && proves_brake_closed(current)`
/// (recognition gates only this fallback; arrival and transition run
/// on the held state).
struct SequenceStepInputs {
    bool initialized{false};
    bool at_target{false};  ///< held state == the current target
    bool is_faulted{false};
    bool can_transition{false};        ///< can_transition_to(target), current target only
    bool engage_settle_proven{false};  ///< recognized ∧ proven-closed (composed)
    bool release_target{false};        ///< this sequence releases (true) or engages (false)
    FaultResetPolicy policy{FaultResetPolicy::NONE};
    SequenceStepState state{};
};

/// @brief What the executor performs this cycle. Exactly one action; the
/// stall report is not an action; it can coexist with any of them.
enum class SequenceStepAction : uint8_t {
    HOLD,                  ///< nothing to perform (waiting, undeclared fault, mid-interval)
    ADVANCE,               ///< consume the step; the executor settles a terminal consume
                           ///< (release/engage group settlement)
    SETTLE_ENGAGED_EARLY,  ///< conservative engage fallback: consume + settle engaged
    TRY_FAULT_RESET,       ///< request a fault-reset pulse; then fold commit_fault_reset(next,
                           ///< accepted) and, only if accepted, emit SERVO_FAULT_RESET
                           ///< (value = attempt #). A refused request emits nothing
    GIVE_UP_FAULT_RESET,   ///< allowance spent: emit SERVO_FAULT_RESET_EXHAUSTED once
    COMMAND_TRANSITION,    ///< transition_state(target); a wire refusal is per-cycle retry
};

/// @brief The decision plus the unconditional next counter/latch values.
///
/// `next` is complete except for one field the decision cannot know: a
/// TRY_FAULT_RESET attempt counts only if the device accepted the request,
/// which it does iff that axis is in a recognized FAULT. So the executor
/// folds `commit_fault_reset(next, accepted)` after the effect (two-phase).
/// "Accepted" is about the request, not the wire and not the drive; see
/// DeviceServo::reset_fault. An accepted request can still fail to publish:
/// if the frame owes a falling edge it is held one cycle, and a discard on
/// that next cycle spends the attempt without ever putting bit 7 on the wire.
struct SequenceStepDecision {
    SequenceStepAction action{SequenceStepAction::HOLD};
    SequenceStepState next{};
    /// Emit BRAKE_SEQUENCE_STALLED this cycle. Not exclusive with the action;
    /// the executor emits the stall first (pinned end-to-end by
    /// StallReportOutrunsFaultResetInTheSameCycle).
    bool emit_stall{false};
};

/// @brief Decide one joint's sequence step for this cycle. Pure: the
/// branch priority of execute_state_transitions as a testable table.
///
/// Behavior pinned by characterization tests:
/// - The stall check runs above the initialization gate: a never-initialized
///   axis with an active sequence is exactly the silent stall the event
///   names. The counter freezes once reported; arrival re-arms the episode.
/// - Arrival zeroes reset_cycles but not reset_attempts; the healthy
///   COMMAND_TRANSITION path zeroes reset_attempts but carries reset_cycles
///   (a short recovery leaves the elapsed interval in place; the next
///   episode's first reset arrives early).
/// - threshold 0 disables stall counting entirely.
[[nodiscard]] PLEM_FIELDBUS_INTERNAL constexpr SequenceStepDecision decide_sequence_step(
    const SequenceStepInputs& in, uint32_t stall_threshold_cycles, uint32_t reset_interval_cycles,
    uint32_t reset_max_attempts) noexcept {
    SequenceStepDecision d;
    d.next = in.state;

    const bool at_step = in.initialized && in.at_target;
    if (!at_step) {
        if (!d.next.stall_reported && stall_threshold_cycles != 0 && ++d.next.stall_cycles >= stall_threshold_cycles) {
            d.next.stall_reported = true;
            d.emit_stall = true;
        }
    } else {
        d.next.stall_cycles = 0;
        d.next.stall_reported = false;
    }

    if (!in.initialized) {
        return d;  // HOLD
    }

    if (in.at_target) {
        d.next.reset_cycles = 0;
        d.action = SequenceStepAction::ADVANCE;
    } else if (in.policy == FaultResetPolicy::BOUNDED_AUTO_RESET && in.is_faulted) {
        if (++d.next.reset_cycles >= reset_interval_cycles) {
            d.next.reset_cycles = 0;
            if (in.state.reset_attempts >= reset_max_attempts) {
                if (in.state.reset_attempts == reset_max_attempts) {
                    ++d.next.reset_attempts;  // sentinel: the give-up is one-shot
                    d.action = SequenceStepAction::GIVE_UP_FAULT_RESET;
                }  // else: already gave up, so stop asking (HOLD). The axis
                   // may read FRA or an unrecognized word when the cap is
                   // crossed.
            } else {
                d.action = SequenceStepAction::TRY_FAULT_RESET;
            }
        }  // else: mid-interval (HOLD)
    } else if (!in.release_target && in.engage_settle_proven) {
        d.action = SequenceStepAction::SETTLE_ENGAGED_EARLY;
    } else if (in.can_transition) {
        d.next.reset_attempts = 0;  // a healthy step ends the fault episode
        d.action = SequenceStepAction::COMMAND_TRANSITION;
    }
    return d;
}

/// @brief Second fold of a TRY_FAULT_RESET: count the attempt only if the
/// device accepted the request (a refused request consumes no allowance;
/// measured behavior). Nothing about the wire is known here: an accepted
/// request that meets an owed falling edge is held a frame and can still be
/// dropped before it is ever published.
[[nodiscard]] PLEM_FIELDBUS_INTERNAL constexpr SequenceStepState commit_fault_reset(SequenceStepState next,
                                                                                    bool reset_accepted) noexcept {
    if (reset_accepted) {
        ++next.reset_attempts;
    }
    return next;
}

static_assert(std::is_trivially_copyable_v<SequenceStepInputs>);
static_assert(std::is_trivially_copyable_v<SequenceStepDecision>);

}  // namespace plem::fieldbus::ethercat
