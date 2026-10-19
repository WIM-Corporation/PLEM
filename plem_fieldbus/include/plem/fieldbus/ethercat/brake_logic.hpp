// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>

#include <plem/core/result.hpp>
#include <plem/core/types/tri_state.hpp>

#include "plem/fieldbus/cia402_types.hpp"

namespace plem::fieldbus::ethercat {

/// @brief Persistent brake-decision state (RT-safe: trivially copyable)
///
/// Owned by the EtherCAT cycle shell; threaded through decide_control_brake()
/// once per cycle. The request-mask per-joint brake API never updates this state.
struct BrakeLogicState {
    /// Previous cycle's protective-stop level (rising-edge detection).
    bool last_protective_stop_active{false};

    /// A safety-forced engage (protective stop rising edge or a mode trip) is still
    /// completing; every release path stays suppressed until the pending
    /// flag resolves: motion observed no longer possible (any axis seen off
    /// OPERATION_ENABLED, not a closure barrier), or the stop cleared first.
    bool engage_pending{false};
};

/// @brief One ordered brake effect (RT-safe POD)
///
/// Effects are an ordered list, never an OR-merged mask: the same cycle can
/// legally carry a safety-forced engage and a request-leg engage, and the emit
/// count, the sequence-overwrite order, and the event order are all part of
/// the observable contract.
enum class BrakeEffectKind : uint8_t {
    /// Start the per-servo engage sequence for `mask` (the sequence start
    /// itself emits BRAKE_ENGAGE). A user / request-mask engage: an axis
    /// already walking an engage restarts from the first step.
    START_ENGAGE,
    /// The safety-forced engage (protective stop rising edge, or a mode trip):
    /// same closure, busy publication and BRAKE_ENGAGE as
    /// START_ENGAGE, but an axis already walking an engage keeps its head,
    /// remaining steps and stall history (only axes with an active release
    /// or no sequence are loaded); a new stop intent must never walk a
    /// closing drive back up, nor starve a healthy axis of its next step
    /// when trips keep arriving.
    START_FORCED_ENGAGE,
    /// Start the per-servo release sequence for `mask` (emits BRAKE_RELEASE).
    START_RELEASE,
    /// Emit BRAKE_ENGAGE only: motion is no longer possible after a forced
    /// engage. No sequence is touched.
    EMIT_BRAKE_ENGAGE_ONLY,
    /// Emit PROTECTIVE_STOP, once per trigger: the aggregate rising edge, or
    /// a mode trip arriving under an already-active level.
    EMIT_PROTECTIVE_STOP,
    /// Emit PROTECTIVE_STOP_CLEAR (pending resolved by the stop clearing
    /// before the engage was observed complete).
    EMIT_PROTECTIVE_STOP_CLEAR,
    /// A release that every other permit admitted was refused at the
    /// readiness gate (some joint of the request is not admitted; see
    /// BrakeRequestInputs::release_ready). The request is consumed and
    /// discarded, never re-queued; `mask` is the requested joints
    /// (pre-closure) so the apply can report every axis of the closure that
    /// fails the gate (SLAVE_NOT_OPERATIONAL or MODES_OF_OPERATION_MISMATCH per
    /// axis).
    EMIT_RELEASE_REFUSED,
};

struct BrakeEffect {
    BrakeEffectKind kind{BrakeEffectKind::START_ENGAGE};
    uint8_t mask{0};  ///< joints for Start* and EMIT_RELEASE_REFUSED; 0 for the other Emit*

    constexpr bool operator==(const BrakeEffect&) const = default;
};

/// @brief One projection of the committed mode word: motion and
/// the protective-stop recovery lane taken from a single atomic load of the host's
/// committed mode state, never from separately-stored flags, whose store
/// windows a stale writer can interleave.
struct CommittedModeSample {
    bool motion{false};  ///< committed mode is motion-capable
    /// Committed protective-stop recovery lane: safety mode protective stop and mode FREEDRIVE,
    /// from the same snapshot word (see
    /// EtherCATController::set_committed_mode_probe).
    bool protective_stop_recovery{false};
};

/// @brief Inputs to the control-leg brake decision (first safety sample)
struct BrakeControlInputs {
    /// EtherCATBus::motion_possible() (servo-level fact). Anything but YES,
    /// including UNDEFINED, counts as "motion not possible".
    core::TriState motion_possible{core::TriState::UNDEFINED};
    /// First protective-stop sample of the cycle (INTERNAL | EXTERNAL level).
    bool protective_stop_active{false};
    /// In-service trip (mode trip or enable loss): a new
    /// per-axis onset detected by the previous sequencing step: an enabled
    /// axis seen leaving its declared mode of operation, or a released
    /// axis seen leaving OPERATION_ENABLED outside a walking sequence (the
    /// controller reports each axis once per episode, by cause).
    /// Treated as a stop trigger exactly like the aggregate protective stop rising
    /// edge, through the same branch, so the forced engage starts even while
    /// the aggregate level is already active (a protective-stop recovery freedrive
    /// with the brakes open has no rising edge left to fire). Coinciding
    /// with a rising edge it fires the trigger once.
    bool mode_trip{false};
};

/// Static effect bound for the control leg: pending-resolution emit (1) +
/// trigger pair (2); the control leg's only inputs are the trigger and
/// pending-resolution facts, so this is its whole output. test_brake_logic.cpp checks this bound over
/// the full input domain at compile time, so decide has no runtime overflow
/// branch.
inline constexpr size_t kMaxControlBrakeEffects = 3;

struct BrakeControlDecision {
    std::array<BrakeEffect, kMaxControlBrakeEffects> effects{};
    size_t count{0};
    BrakeLogicState next_state{};
};

/// @brief Inputs to the request-mask brake decision (second safety sample)
struct BrakeRequestInputs {
    /// Release request bits consumed this cycle (cleared from the source mask).
    uint8_t release_mask{0};
    /// Engage request bits consumed this cycle (cleared from the source mask).
    uint8_t engage_mask{0};
    /// Second protective-stop sample, taken after the mask consume. The
    /// re-sample (not a reuse of the control-leg sample) is observable
    /// behavior: a stop rising between the two samples must suppress this
    /// cycle's requested release.
    bool protective_stop_active{false};
    /// Recovery-lane sample after the mask consume, from the same
    /// committed-word load as motion_enabled.
    bool protective_stop_recovery{false};
    /// Motion-committed sample after the mask consume from the host's
    /// committed mode word. Release
    /// requires it: outside a motion mode the safety logic suspends torque to
    /// zero, so an enabled joint would be unheld the moment its brake opens.
    /// This consume-time sample decides; a check at post time can be raced.
    /// Engage is never gated.
    bool motion_enabled{false};
    /// The control decision's next_state.engage_pending (same-cycle value;
    /// a protective stop edge this cycle suppresses this cycle's requested release).
    bool engage_pending{false};
    /// Release-admission sample (the RT consume-time re-check): true =
    /// admission closed (shutdown quiesce): no release effect may be emitted
    /// this cycle. Engage is never gated (the safe direction). A queued
    /// release consumed after the close is discarded here; the TOCTOU
    /// ("posted before close, consumed after") closes at this exact point.
    bool release_admission_closed{false};
    /// Release readiness mask (the mode-acceptance gate, checked before
    /// the enable sequence starts): bit i set = joint i and every joint of
    /// its brake group is admitted this cycle: its slave is operational (in
    /// OP with this configuration, per the master's last state sample) and
    /// the axis is accepted in its declared mode of operation (initialized,
    /// recognized statusword, 0x6061 display == declaration, or, for a
    /// POST_ENABLE axis, the cold zero-display exception of
    /// EtherCATController::release_admissible()). Recomputed every cycle by
    /// the controller and already closed over the brake groups. The consumed
    /// release is compared as passed: `release_mask ⊆ release_ready` starts
    /// it, anything else refuses the whole request
    /// (EMIT_RELEASE_REFUSED; no partial release: the group closure the
    /// apply performs would otherwise power an unready partner), consumed
    /// and discarded like every other suppression, never held for readiness
    /// to recover. Reversible, unlike the one-way admission close above,
    /// which is why it is a separate input. Bits above the configured DOF
    /// are checked like any other bit; production clips requests at the
    /// posting boundary. Default 0 = nothing ready: a caller that forgets to
    /// wire it refuses every release (fail-closed), never opens the gate.
    uint8_t release_ready{0};
};

/// Static effect bound for the request leg: at most one release start or
/// refusal + one engage start (release first, the consume order at the source).
inline constexpr size_t kMaxRequestBrakeEffects = 2;

struct BrakeRequestDecision {
    std::array<BrakeEffect, kMaxRequestBrakeEffects> effects{};
    size_t count{0};
    // Deliberately no next_state: the request-mask API never updates
    // BrakeLogicState.
};

/// @brief Pure function: control-leg (safety-trigger) brake decision for one RT cycle
///
/// 1. On protective stop trigger (rising edge, or a mode trip) -> always
///    force-engage all brakes and mark the engage pending
/// 2. While the forced engage is pending -> resolve it once motion is no
///    longer possible (BRAKE_ENGAGE), or once the stop cleared first
///    (PROTECTIVE_STOP_CLEAR). Releases belong to the request leg, which
///    reads this cycle's pending flag and stays suppressed while it is set.
///
/// Effect order is the contract: pending resolution precedes a same-cycle
/// re-trigger, and a trigger emits START_FORCED_ENGAGE before
/// EMIT_PROTECTIVE_STOP.
///
/// @param all_joints_mask Mask covering every configured joint (forced
///                        engages address all joints)
/// @thread_safety Thread-safe: pure function of its value arguments.
/// @realtime_safe Yes: constexpr noexcept, no allocation, no exceptions.
[[nodiscard]] constexpr BrakeControlDecision decide_control_brake(const BrakeControlInputs& in,
                                                                  const BrakeLogicState& state,
                                                                  uint8_t all_joints_mask) noexcept {
    BrakeControlDecision decision;
    decision.next_state = state;

    // The aggregate level's rising edge, or a mode trip (a new stop cause under
    // a level that may already be active); one trigger either way.
    const bool triggered = (in.protective_stop_active && !state.last_protective_stop_active) || in.mode_trip;
    decision.next_state.last_protective_stop_active = in.protective_stop_active;
    const bool motion_possible = (in.motion_possible == core::TriState::YES);

    // Pending resolution first. Priority is part of the contract: with the
    // stop already cleared but brakes still not fully engaged, the engage
    // observation wins; PROTECTIVE_STOP_CLEAR is only emitted when the stop
    // cleared while motion was still possible.
    if (decision.next_state.engage_pending) {
        if (!motion_possible) {
            decision.effects[decision.count++] = {BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY, 0};
            decision.next_state.engage_pending = false;
        } else if (!in.protective_stop_active) {
            decision.effects[decision.count++] = {BrakeEffectKind::EMIT_PROTECTIVE_STOP_CLEAR, 0};
            decision.next_state.engage_pending = false;
        }
    }

    if (triggered) {
        // Trigger: always force-engage all brakes immediately (even in
        // freedrive); the forced engage keeps an engage already walking on
        // an axis. Sequence start first, then the stop event; the emit
        // order is pinned by the device-level characterization.
        decision.effects[decision.count++] = {BrakeEffectKind::START_FORCED_ENGAGE, all_joints_mask};
        decision.effects[decision.count++] = {BrakeEffectKind::EMIT_PROTECTIVE_STOP, 0};
        decision.next_state.engage_pending = true;
    }

    return decision;
}

/// @brief Pure function: request-mask brake decision for one RT cycle
///
/// The queued masks were consumed unconditionally at the source (the loaded
/// bits cleared):
/// a release suppressed here is discarded, never re-queued: releasing brakes
/// later from a stale request the moment the stop clears would be a safety
/// regression. A release the permits admit but the readiness gate refuses
/// is discarded the same way, reported as EMIT_RELEASE_REFUSED for the
/// whole request. Release applies before engage (consume order at the source).
///
/// @thread_safety Thread-safe: pure function of its value arguments.
/// @realtime_safe Yes: constexpr noexcept, no allocation, no exceptions.
[[nodiscard]] constexpr BrakeRequestDecision decide_requested_brake(const BrakeRequestInputs& in) noexcept {
    BrakeRequestDecision decision;

    const bool release_allowed = !in.release_admission_closed && in.motion_enabled && !in.engage_pending &&
                                 (!in.protective_stop_active || in.protective_stop_recovery);
    if (release_allowed && in.release_mask != 0x00) {
        const bool ready = (in.release_mask & static_cast<uint8_t>(~in.release_ready)) == 0;
        decision.effects[decision.count++] = {
            ready ? BrakeEffectKind::START_RELEASE : BrakeEffectKind::EMIT_RELEASE_REFUSED, in.release_mask};
    }
    if (in.engage_mask != 0x00) {
        decision.effects[decision.count++] = {BrakeEffectKind::START_ENGAGE, in.engage_mask};
    }

    return decision;
}

static_assert(std::is_trivially_copyable_v<BrakeLogicState>);
static_assert(std::is_trivially_copyable_v<BrakeEffect>);
static_assert(std::is_trivially_copyable_v<BrakeControlDecision>);
static_assert(std::is_trivially_copyable_v<BrakeRequestDecision>);

/// @brief What the brake sequence does with a drive that faults mid-transition.
///
/// A faulted drive has no CiA 402 path to any target (can_transition_to()
/// is false forever), and the only exit is a controlword bit-7 reset. Whether
/// the sequence issues that reset is declared per drive family by the model,
/// independently of its brake groups. Declaring BOUNDED_AUTO_RESET says the model wants the reset;
/// it does not certify that the drive is known to fault.
enum class FaultResetPolicy : uint8_t {
    /// The sequence waits and never resets by itself (the default when
    /// undeclared). This policy governs the automatic path only; it does not
    /// prohibit a manual request, which is checked on its own merits at
    /// publication time like any other.
    NONE = 0,
    /// Rate-limited, capped bit-7 reset requests, then the sequence stops
    /// asking (the cap and the interval are the model's FaultResetPacing).
    /// The axis is then left in whatever state it reports, which need not be
    /// a settled FAULT.
    BOUNDED_AUTO_RESET,
};

/// @brief Pacing of BOUNDED_AUTO_RESET: a drive-family property the model
/// declares next to its policy: the nominal accumulated fault-cycle
/// window between two accepted automatic requests, and how many accepted
/// requests one software fault episode (per axis) gets before the sequence
/// stops asking. Which cycles the counter accumulates (is_faulted: FAULT and
/// FAULT_REACTION_ACTIVE, a held unrecognized word included), which requests
/// are accepted (a recognized FAULT only) and where an episode starts and
/// ends live in sequence_step.hpp / device.cpp, not here. Time is the source
/// of truth: the controller converts the interval to cycles when configured
/// (cycles_at_least, rounded up to the period), so the window does not shrink
/// with the control period. Not a wall-clock guarantee. Declared only with
/// BOUNDED_AUTO_RESET; NONE carries no pacing.
/// Claim scope: an accepted request is not a published pulse, and neither
/// is a drive reset (SERVO_FAULT_RESET_RESULT reports what reached domain
/// memory; the drive's answer is its statusword).
struct FaultResetPacing {
    uint32_t interval_ms;   ///< nominal FAULT dwell between accepted requests (> 0)
    uint32_t max_attempts;  ///< accepted requests per episode, 1..kMaxSupportedFaultResetAttempts
    constexpr bool operator==(const FaultResetPacing&) const = default;
};
static_assert(std::is_trivially_copyable_v<FaultResetPacing>);

/// @brief Supported ceiling on FaultResetPacing::max_attempts — an
/// engineering bound on the COUNT, not a measured drive limit and not a
/// time bound (a long interval with a small cap can still ask for days —
/// no outer sequence timeout exists; the cap is the only thing that ends
/// the asking). 10'000 keeps the give-up sentinel (cap + 1) far from uint32
/// wrap and both event values (attempt #, cap) exactly representable in the
/// float payload; no drive measurement backs the number.
inline constexpr uint32_t kMaxSupportedFaultResetAttempts = 10'000;

/// @brief Policy × pacing validity: the one rule, shared by the model-spec
/// gate and the controller setter (each adds its own error context). ok =
/// valid. Presence is part of the rule: NONE must carry no pacing (a
/// declared pacing with no policy would be silently ignored), and
/// BOUNDED_AUTO_RESET must carry one (no default pacing exists). The
/// period-dependent range of the cycle conversion is not here; only the
/// setter knows its period.
[[nodiscard]] inline core::Result<void> validate_fault_reset(FaultResetPolicy policy,
                                                             const std::optional<FaultResetPacing>& pacing) {
    switch (policy) {
        case FaultResetPolicy::NONE:
            if (pacing.has_value()) {
                return core::Result<void>::error(
                    "fault_reset_policy is NONE but fault_reset_pacing is declared — pacing belongs to "
                    "BOUNDED_AUTO_RESET only; drop the pacing or declare the policy");
            }
            return core::Result<void>::ok();
        case FaultResetPolicy::BOUNDED_AUTO_RESET:
            if (!pacing.has_value()) {
                return core::Result<void>::error(
                    "fault_reset_policy is BOUNDED_AUTO_RESET but no fault_reset_pacing is declared — "
                    "declare {interval_ms, max_attempts} (no default pacing exists)");
            }
            if (pacing->interval_ms == 0) {
                return core::Result<void>::error(
                    "fault_reset_pacing.interval_ms is 0 — the nominal FAULT dwell between accepted requests "
                    "must be > 0");
            }
            if (pacing->max_attempts == 0 || pacing->max_attempts > kMaxSupportedFaultResetAttempts) {
                return core::Result<void>::error("fault_reset_pacing.max_attempts (" +
                                                 std::to_string(pacing->max_attempts) + ") is outside 1.." +
                                                 std::to_string(kMaxSupportedFaultResetAttempts) +
                                                 " — the supported ceiling on accepted requests per fault episode");
            }
            return core::Result<void>::ok();
    }
    return core::Result<void>::error("fault_reset_policy holds an out-of-range value (" +
                                     std::to_string(static_cast<int>(policy)) +
                                     ") — only NONE/BOUNDED_AUTO_RESET exist");
}

// The brake-release step
//
// THE ASSUMPTION THIS SEQUENCING IS BUILT ON: every supported axis's holding
// brake FOLLOWS DRIVE POWER. A drive's brake may be open from SWITCHED_ON
// upward on the enable sequence and is proven closed only strictly below it,
// so the enable sequence is the release, the group's closure evidence settles
// the engage, and a declared group map puts the shutdown quiesce first. A
// drive family whose brake is not power-coupled (a drive-owned 0x60FE output,
// no brake, an external brake) reads CiA 402 states as brake evidence
// wrongly here; supporting one takes a new sequencing path, not a parameter.
//
// One fact, two projections. The two predicates below are that fact asked
// from each side; keep them agreeing (test_brake_logic.cpp pins it over every
// enumerator) rather than re-deriving the step at a call site.
//
// SWITCHED_ON is the conservative bound across the supported families, not a
// per-model measurement: some drives open the brake at SWITCHED_ON, others are
// only known to open at OPERATION_ENABLED. Tightening this bound for a family
// needs a measurement that its brake stays closed at SWITCHED_ON.
//
// QUICK_STOP_ACTIVE, FAULT, FAULT_REACTION_ACTIVE and UNKNOWN sit on neither
// side: they say nothing about brake power, so they neither prime torque nor
// prove closure.

/// @brief May this step hold the brake open? Torque priming starts here, so a
/// brake that opens meets a compensating command.
[[nodiscard]] constexpr bool may_hold_brake_open(ServoState state) noexcept {
    return state == ServoState::SWITCHED_ON || state == ServoState::OPERATION_ENABLED;
}

/// @brief Does this step prove the brake closed? Engage settlement accepts
/// only these (an allowlist: a step that might hold the brake
/// open must block a "closed" verdict).
[[nodiscard]] constexpr bool proves_brake_closed(ServoState state) noexcept {
    return state == ServoState::NOT_READY_TO_SWITCH_ON || state == ServoState::SWITCH_ON_DISABLED ||
           state == ServoState::READY_TO_SWITCH_ON;
}

}  // namespace plem::fieldbus::ethercat
