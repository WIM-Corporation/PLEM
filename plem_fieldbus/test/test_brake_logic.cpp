// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_brake_logic.cpp
/// @brief Truth-table pins for the pure brake decisions.
///
/// Every row of the preserved-edge matrix
/// gets an explicit expected-effect-sequence test; the full input domain is
/// additionally swept at constexpr time, so an out-of-bounds effect emission
/// (array overflow) fails compilation: the static bound proof the header's
/// kMax* constants rely on (no runtime overflow branch exists by design).

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <plem/fieldbus/ethercat/brake_logic.hpp>

using plem::core::TriState;
using plem::fieldbus::ServoState;
using plem::fieldbus::ethercat::BrakeControlDecision;
using plem::fieldbus::ethercat::BrakeControlInputs;
using plem::fieldbus::ethercat::BrakeEffect;
using plem::fieldbus::ethercat::BrakeEffectKind;
using plem::fieldbus::ethercat::BrakeLogicState;
using plem::fieldbus::ethercat::BrakeRequestDecision;
using plem::fieldbus::ethercat::BrakeRequestInputs;
using plem::fieldbus::ethercat::decide_control_brake;
using plem::fieldbus::ethercat::decide_requested_brake;
using plem::fieldbus::ethercat::FaultResetPacing;
using plem::fieldbus::ethercat::FaultResetPolicy;
using plem::fieldbus::ethercat::kMaxControlBrakeEffects;
using plem::fieldbus::ethercat::kMaxRequestBrakeEffects;
using plem::fieldbus::ethercat::kMaxSupportedFaultResetAttempts;
using plem::fieldbus::ethercat::may_hold_brake_open;
using plem::fieldbus::ethercat::proves_brake_closed;
using plem::fieldbus::ethercat::validate_fault_reset;

namespace {

constexpr uint8_t kAll = 0x3F;  // all-six-axes mask

std::vector<BrakeEffect> effects_of(const BrakeControlDecision& d) {
    return {d.effects.begin(), d.effects.begin() + static_cast<std::ptrdiff_t>(d.count)};
}
std::vector<BrakeEffect> effects_of(const BrakeRequestDecision& d) {
    return {d.effects.begin(), d.effects.begin() + static_cast<std::ptrdiff_t>(d.count)};
}

// RT contract: pure decisions are constexpr noexcept on POD in/out.
static_assert(noexcept(decide_control_brake({}, {}, kAll)));
static_assert(noexcept(decide_requested_brake({})));

// Static bound proof + decision oracle
// Constant evaluation sweeps: any effects[] overflow is UB and therefore a
// compile error inside static_assert; this is the static bound proof. Every row is
// also checked against an oracle: the expected effect sequence (kind, mask,
// order) and next state, written out from the contract independently of the
// decision's branches: a dropped release candidate, a refusal without an
// edge, a duplicated or misplaced effect, a wrong mask or a wrong edge state
// all fail the sweep, not only an overflow. Both sweeps enumerate every
// bool/TriState field; the masks are taken by equivalence class relative to
// the all-joints request, not over the raw 8-bit domain (readiness: fully
// ready / none / part / only bits outside the request unready / above-DOF;
// request: zero / single-bit / multi-bit / all-joints / above-DOF); the
// decisions only test masks against 0x00 and against `request ⊆ ready`, so
// the classes are exhaustive for those two tests. The oracle is a second
// writing of the same contract, not an independent proof: a wrong
// requirement fails both together. This pins BrakeEffect capacity and the
// decisions' effect order, masks and next state, not the controller's
// wiring, nor how many Events the apply emits per effect.
constexpr TriState kTri[] = {TriState::NO, TriState::YES, TriState::UNDEFINED};

/// Readiness masks by their relation to the all-six-axes request.
constexpr uint8_t kReadyClasses[] = {0x00, kAll, 0x3E, 0x01, 0xFF, 0x80};

/// The oracle's effect list (the larger of the two static bounds).
struct ExpectedEffects {
    std::array<BrakeEffect, kMaxControlBrakeEffects> effects{};
    size_t count{0};
    constexpr void add(BrakeEffectKind kind, uint8_t mask) {
        effects[count++] = {kind, mask};
    }
};

constexpr bool same_effects(const auto& decision, const ExpectedEffects& expected) {
    if (decision.count != expected.count) {
        return false;
    }
    for (size_t i = 0; i < expected.count; ++i) {
        if (!(decision.effects[i] == expected.effects[i])) {
            return false;
        }
    }
    return true;
}

/// Control-leg oracle: the contract of decide_control_brake as expectations.
/// 1. Pending resolution first: brakes observed engaged (motion not possible)
///    wins over the stop having cleared while motion was still possible.
/// 2. A trigger (rising edge or mode trip) forces the engage of every joint,
///    emits the stop and leaves the engage pending.
/// Nothing else: releases and user engages belong to the request leg.
constexpr bool control_row_matches_oracle(const BrakeControlInputs& in, const BrakeLogicState& state) {
    const auto d = decide_control_brake(in, state, kAll);

    const bool triggered = (in.protective_stop_active && !state.last_protective_stop_active) || in.mode_trip;
    const bool motion_possible = in.motion_possible == TriState::YES;
    const bool engage_seen = state.engage_pending && !motion_possible;
    const bool stop_cleared = state.engage_pending && motion_possible && !in.protective_stop_active;
    const bool still_pending = state.engage_pending && !engage_seen && !stop_cleared;

    ExpectedEffects expected;
    if (engage_seen) {
        expected.add(BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY, 0);
    } else if (stop_cleared) {
        expected.add(BrakeEffectKind::EMIT_PROTECTIVE_STOP_CLEAR, 0);
    }
    if (triggered) {
        expected.add(BrakeEffectKind::START_FORCED_ENGAGE, kAll);
        expected.add(BrakeEffectKind::EMIT_PROTECTIVE_STOP, 0);
    }

    return same_effects(d, expected) && d.next_state.last_protective_stop_active == in.protective_stop_active &&
           d.next_state.engage_pending == (triggered || still_pending);
}

constexpr bool control_domain_matches_oracle() {
    for (auto motion : kTri) {
        for (int protective_stop = 0; protective_stop < 2; ++protective_stop) {
            for (int last_protective_stop = 0; last_protective_stop < 2; ++last_protective_stop) {
                for (int pending = 0; pending < 2; ++pending) {
                    for (int trip = 0; trip < 2; ++trip) {
                        const BrakeControlInputs in{.motion_possible = motion,
                                                    .protective_stop_active = protective_stop != 0,
                                                    .mode_trip = trip != 0};
                        const BrakeLogicState state{last_protective_stop != 0, pending != 0};
                        if (!control_row_matches_oracle(in, state)) {
                            return false;
                        }
                    }
                }
            }
        }
    }
    return true;
}
static_assert(control_domain_matches_oracle(),
              "decide_control_brake domain sweep: effect bound / oracle (sequence, mask, next state) violated");

/// Request-leg oracle: a requested release the permits admit (admission open,
/// motion enabled, no forced engage pending, no stop without recovery) is
/// exactly one of start / refusal by readiness, carrying the original mask;
/// a suppressed release is dropped silently; the engage is never gated;
/// release before engage.
constexpr bool request_row_matches_oracle(const BrakeRequestInputs& in) {
    const auto d = decide_requested_brake(in);
    const bool allowed = !in.release_admission_closed && in.motion_enabled && !in.engage_pending &&
                         (!in.protective_stop_active || in.protective_stop_recovery);
    const bool ready = (in.release_mask & static_cast<uint8_t>(~in.release_ready)) == 0;

    ExpectedEffects expected;
    if (allowed && in.release_mask != 0x00) {
        expected.add(ready ? BrakeEffectKind::START_RELEASE : BrakeEffectKind::EMIT_RELEASE_REFUSED, in.release_mask);
    }
    if (in.engage_mask != 0x00) {
        expected.add(BrakeEffectKind::START_ENGAGE, in.engage_mask);
    }
    return d.count <= kMaxRequestBrakeEffects && same_effects(d, expected);
}

constexpr bool request_domain_matches_oracle() {
    constexpr uint8_t kMasks[] = {0x00, 0x01, 0x2A, 0x3F, 0xFF};
    for (auto release_mask : kMasks) {
        for (auto engage_mask : kMasks) {
            for (int protective_stop = 0; protective_stop < 2; ++protective_stop) {
                for (int recovery = 0; recovery < 2; ++recovery) {
                    for (int pending = 0; pending < 2; ++pending) {
                        for (int motion_en = 0; motion_en < 2; ++motion_en) {
                            for (int closed = 0; closed < 2; ++closed) {
                                for (auto ready : kReadyClasses) {
                                    if (!request_row_matches_oracle({.release_mask = release_mask,
                                                                     .engage_mask = engage_mask,
                                                                     .protective_stop_active = protective_stop != 0,
                                                                     .protective_stop_recovery = recovery != 0,
                                                                     .motion_enabled = motion_en != 0,
                                                                     .engage_pending = pending != 0,
                                                                     .release_admission_closed = closed != 0,
                                                                     .release_ready = ready})) {
                                        return false;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}
static_assert(request_domain_matches_oracle(),
              "decide_requested_brake domain sweep: effect bound / oracle (sequence, mask) violated");

}  // namespace

// Control leg: preserved-edge matrix rows

TEST(BrakeControlDecide, RisingEdgeEngagesAllThenEmitsProtectiveStop) {
    // The effect order (sequence start, then stop event) is the contract.
    const auto d =
        decide_control_brake({.motion_possible = TriState::YES, .protective_stop_active = true}, {false, false}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_FORCED_ENGAGE, kAll},
                                               {BrakeEffectKind::EMIT_PROTECTIVE_STOP, 0}};
    EXPECT_EQ(effects_of(d), expected);
    EXPECT_TRUE(d.next_state.last_protective_stop_active);
    EXPECT_TRUE(d.next_state.engage_pending);
}

TEST(BrakeControlDecide, PendingResolvesViaEngageObservedComplete) {
    // Sustained stop, motion no longer possible: only EMIT_BRAKE_ENGAGE_ONLY,
    // which resolves the pending engage.
    const auto d =
        decide_control_brake({.motion_possible = TriState::NO, .protective_stop_active = true}, {true, true}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY, 0}};
    EXPECT_EQ(effects_of(d), expected);
    EXPECT_FALSE(d.next_state.engage_pending);
}

TEST(BrakeControlDecide, MotionPossibleUndefinedCountsAsNotPossible) {
    // UNDEFINED counts as NO for the pending resolution.
    const auto d = decide_control_brake({.motion_possible = TriState::UNDEFINED, .protective_stop_active = true},
                                        {true, true}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY, 0}};
    EXPECT_EQ(effects_of(d), expected);
    EXPECT_FALSE(d.next_state.engage_pending);
}

TEST(BrakeControlDecide, PendingResolvesViaStopClearWhileMotionStillPossible) {
    // Stop cleared before the engage completed: PROTECTIVE_STOP_CLEAR.
    const auto d = decide_control_brake({.motion_possible = TriState::YES}, {true, true}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::EMIT_PROTECTIVE_STOP_CLEAR, 0}};
    EXPECT_EQ(effects_of(d), expected);
    EXPECT_FALSE(d.next_state.engage_pending);
}

TEST(BrakeControlDecide, StopClearWithMotionNotPossibleEmitsEngageNotClear) {
    // if/else priority row: pending + stop cleared + motion not possible ->
    // BRAKE_ENGAGE only, never PROTECTIVE_STOP_CLEAR.
    const auto d = decide_control_brake({.motion_possible = TriState::NO}, {true, true}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY, 0}};
    EXPECT_EQ(effects_of(d), expected);
    EXPECT_FALSE(d.next_state.engage_pending);
}

TEST(BrakeControlDecide, ReRisingWhilePendingOrdersResolutionBeforeTrigger) {
    // Shell-unreachable but domain-total row: pending resolution precedes the
    // re-trigger pair: the 3-effect worst case that sizes the static bound.
    const auto d =
        decide_control_brake({.motion_possible = TriState::NO, .protective_stop_active = true}, {false, true}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY, 0},
                                               {BrakeEffectKind::START_FORCED_ENGAGE, kAll},
                                               {BrakeEffectKind::EMIT_PROTECTIVE_STOP, 0}};
    EXPECT_EQ(effects_of(d), expected);
    EXPECT_TRUE(d.next_state.engage_pending);
}

// The mode trip is a stop trigger like the rising edge

TEST(BrakeControlDecide, ModeTripTriggersTheForcedEngageUnderAnActiveStop) {
    // The aggregate level is already active (no rising edge left): a
    // P-STOP-recovery freedrive with the brakes open. A mode trip is a fresh
    // trigger: forced engage + PROTECTIVE_STOP, pending set.
    const auto d = decide_control_brake(
        {.motion_possible = TriState::YES, .protective_stop_active = true, .mode_trip = true}, {true, false}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_FORCED_ENGAGE, kAll},
                                               {BrakeEffectKind::EMIT_PROTECTIVE_STOP, 0}};
    EXPECT_EQ(effects_of(d), expected);
    EXPECT_TRUE(d.next_state.engage_pending);
}

TEST(BrakeControlDecide, ModeTripCoincidingWithTheRisingEdgeTriggersOnce) {
    const auto d = decide_control_brake(
        {.motion_possible = TriState::YES, .protective_stop_active = true, .mode_trip = true}, {false, false}, kAll);
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_FORCED_ENGAGE, kAll},
                                               {BrakeEffectKind::EMIT_PROTECTIVE_STOP, 0}};
    EXPECT_EQ(effects_of(d), expected) << "one trigger, one pair — never two forced engages";
}

// Direct leg

TEST(BrakeRequestDecide, ReleaseThenEngageInConsumeOrder) {
    const auto d = decide_requested_brake(
        {.release_mask = 0x01, .engage_mask = 0x02, .motion_enabled = true, .release_ready = kAll});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_RELEASE, 0x01},
                                               {BrakeEffectKind::START_ENGAGE, 0x02}};
    EXPECT_EQ(effects_of(d), expected);
}

TEST(BrakeRequestDecide, SuppressesReleaseDuringActiveStopWithoutFreedrive) {
    const auto d = decide_requested_brake({.release_mask = 0x3F,
                                           .engage_mask = 0x00,
                                           .protective_stop_active = true,
                                           .motion_enabled = true,
                                           .release_ready = kAll});
    EXPECT_EQ(d.count, 0u) << "suppressed release must be discarded, not emitted";
}

TEST(BrakeRequestDecide, SuppressesReleaseWhileEngagePendingEvenInFreedrive) {
    const auto d = decide_requested_brake({.release_mask = 0x3F,
                                           .engage_mask = 0x00,
                                           .protective_stop_active = true,
                                           .protective_stop_recovery = true,
                                           .motion_enabled = true,
                                           .engage_pending = true,
                                           .release_ready = kAll});
    EXPECT_EQ(d.count, 0u) << "the P-STOP-forced engage must complete before any recovery release";
}

TEST(BrakeRequestDecide, AllowsReleaseDuringStopWithFreedriveRecovery) {
    const auto d = decide_requested_brake({.release_mask = 0x3F,
                                           .engage_mask = 0x00,
                                           .protective_stop_active = true,
                                           .protective_stop_recovery = true,
                                           .motion_enabled = true,
                                           .release_ready = kAll});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_RELEASE, 0x3F}};
    EXPECT_EQ(effects_of(d), expected);
}

TEST(BrakeRequestDecide, EngageIsNeverGated) {
    const auto d = decide_requested_brake({.release_mask = 0x00,
                                           .engage_mask = 0x3F,
                                           .protective_stop_active = true,
                                           .engage_pending = true,
                                           .release_ready = kAll});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_ENGAGE, 0x3F}};
    EXPECT_EQ(effects_of(d), expected);
}

TEST(BrakeRequestDecide, EmptyMasksProduceNoEffects) {
    const auto d = decide_requested_brake(
        {.release_mask = 0x00, .engage_mask = 0x00, .motion_enabled = true, .release_ready = kAll});
    EXPECT_EQ(d.count, 0u);
}

// Release-admission suppression (consume-time
// re-check: the pure half; the controller samples the flag once per cycle).

TEST(BrakeRequestDecide, ClosedAdmissionDiscardsReleaseKeepsEngage) {
    const auto d = decide_requested_brake({.release_mask = 0x03,
                                           .engage_mask = 0x30,
                                           .motion_enabled = true,
                                           .release_admission_closed = true,
                                           .release_ready = kAll});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_ENGAGE, 0x30}};
    EXPECT_EQ(effects_of(d), expected) << "release discarded, engage untouched (the safe direction is never gated)";
}

TEST(BrakeRequestDecide, MotionDisabledDiscardsReleaseKeepsEngage) {
    // The consume-time authority: whatever raced the poster
    // (a stale mode snapshot, a safety-forced BRAKED commit),
    // the cycle that would start the sequence decides on the committed-mode
    // sample's motion bit. Outside a motion mode the safety logic commands
    // zero torque: an enabled joint would be unheld the moment its brake
    // opens. Discard-not-requeue; engage is never gated.
    const auto d = decide_requested_brake({.release_mask = 0x03, .engage_mask = 0x30, .release_ready = kAll});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_ENGAGE, 0x30}};
    EXPECT_EQ(effects_of(d), expected);
}

// The release-readiness gate (mode acceptance before the enable
// sequence starts). A release every other permit admits is either started
// (whole request ready) or refused (whole request, never partial); the
// refusal is a consumed candidate like every suppression, so readiness
// recovering later revives nothing. Default readiness is fail-closed.

TEST(BrakeRequestDecide, RefusalIsWholeRequestNeverPartial) {
    // Joints 0 and 1 requested, only joint 0 ready: the request is refused
    // as a whole (the apply's group closure would otherwise power an unready
    // partner) and carries the original mask; the same-cycle engage is
    // untouched (never gated).
    const auto d = decide_requested_brake(
        {.release_mask = 0x03, .engage_mask = 0x30, .motion_enabled = true, .release_ready = 0x01});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::EMIT_RELEASE_REFUSED, 0x03},
                                               {BrakeEffectKind::START_ENGAGE, 0x30}};
    EXPECT_EQ(effects_of(d), expected);
}

TEST(BrakeRequestDecide, ReadinessOutsideTheRequestDoesNotMatter) {
    // Only joint 0 requested and only joint 0 ready: the release starts;
    // unready joints outside the request (and outside its closure, which the
    // ready mask already folds in) are not the request's business.
    const auto d = decide_requested_brake(
        {.release_mask = 0x01, .engage_mask = 0x00, .motion_enabled = true, .release_ready = 0x01});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::START_RELEASE, 0x01}};
    EXPECT_EQ(effects_of(d), expected);
}

TEST(BrakeRequestDecide, RefusalOnlyWhenTheOtherPermitsAdmit) {
    // Unready under an active stop without recovery: the stop owns the
    // discard: nothing is emitted (no refusal for a suppressed release).
    const auto d = decide_requested_brake({.release_mask = 0x3F,
                                           .engage_mask = 0x00,
                                           .protective_stop_active = true,
                                           .motion_enabled = true,
                                           .release_ready = 0x00});
    EXPECT_EQ(d.count, 0u);
}

TEST(BrakeRequestDecide, ReadinessDefaultIsFailClosed) {
    // Every other permit satisfied, a release requested, readiness not
    // stated: refused, not started.
    const auto d = decide_requested_brake({.release_mask = 0x01, .engage_mask = 0x00, .motion_enabled = true});
    const std::vector<BrakeEffect> expected = {{BrakeEffectKind::EMIT_RELEASE_REFUSED, 0x01}};
    EXPECT_EQ(effects_of(d), expected);
}

TEST(BrakeRequestDecide, AboveDofBitsAreComparedAsPassed) {
    // The pure function knows no DOF: a request bit above the configured
    // joints is checked against the ready mask like any other bit (production
    // clips requests at the posting boundary, never here).
    const auto started = decide_requested_brake(
        {.release_mask = 0x80, .engage_mask = 0x00, .motion_enabled = true, .release_ready = 0x80});
    const std::vector<BrakeEffect> expected_start = {{BrakeEffectKind::START_RELEASE, 0x80}};
    EXPECT_EQ(effects_of(started), expected_start);
    const auto refused = decide_requested_brake(
        {.release_mask = 0x80, .engage_mask = 0x00, .motion_enabled = true, .release_ready = kAll});
    const std::vector<BrakeEffect> expected_refusal = {{BrakeEffectKind::EMIT_RELEASE_REFUSED, 0x80}};
    EXPECT_EQ(effects_of(refused), expected_refusal);
}

// The brake-release step: one fact, two projections

TEST(BrakeReleaseRung, TorquePrimingStartsAtSwitchedOn) {
    EXPECT_TRUE(may_hold_brake_open(ServoState::SWITCHED_ON));
    EXPECT_TRUE(may_hold_brake_open(ServoState::OPERATION_ENABLED));
    EXPECT_FALSE(may_hold_brake_open(ServoState::READY_TO_SWITCH_ON));
}

TEST(BrakeReleaseRung, OnlyUnpoweredRungsProveClosure) {
    EXPECT_TRUE(proves_brake_closed(ServoState::NOT_READY_TO_SWITCH_ON));
    EXPECT_TRUE(proves_brake_closed(ServoState::SWITCH_ON_DISABLED));
    EXPECT_TRUE(proves_brake_closed(ServoState::READY_TO_SWITCH_ON));
    EXPECT_FALSE(proves_brake_closed(ServoState::SWITCHED_ON));
}

TEST(BrakeReleaseRung, NoRungAnswersBoth) {
    // Two "yes" answers would be two different release steps.
    for (const auto state :
         {ServoState::NOT_READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON,
          ServoState::SWITCHED_ON, ServoState::OPERATION_ENABLED, ServoState::QUICK_STOP_ACTIVE, ServoState::FAULT,
          ServoState::FAULT_REACTION_ACTIVE, ServoState::UNKNOWN}) {
        SCOPED_TRACE(static_cast<int>(state));
        EXPECT_FALSE(may_hold_brake_open(state) && proves_brake_closed(state));
    }
}

TEST(BrakeReleaseRung, InconclusiveStatesAnswerNeither) {
    for (const auto state :
         {ServoState::QUICK_STOP_ACTIVE, ServoState::FAULT, ServoState::FAULT_REACTION_ACTIVE, ServoState::UNKNOWN}) {
        SCOPED_TRACE(static_cast<int>(state));
        EXPECT_FALSE(may_hold_brake_open(state));
        EXPECT_FALSE(proves_brake_closed(state));
    }
}

// Fault-reset policy × pacing: the one shared rule
//
// Presence is part of the rule: a declared pacing without the policy would be
// silently ignored, and the policy without a pacing has no default to fall
// back on. The period-dependent cycle range is the setter's, not tested here.

TEST(FaultResetPacingValidation, NoneWithoutPacingIsTheDefault) {
    EXPECT_TRUE(validate_fault_reset(FaultResetPolicy::NONE, std::nullopt).is_ok());
}

TEST(FaultResetPacingValidation, NoneWithAnyPacingIsRefused) {
    // Even {0, 0}: an engaged optional is a declaration, not an absence.
    EXPECT_NE(validate_fault_reset(FaultResetPolicy::NONE, FaultResetPacing{0, 0}).message().find("is NONE but"),
              std::string::npos);
    EXPECT_FALSE(validate_fault_reset(FaultResetPolicy::NONE, FaultResetPacing{200, 25}).is_ok());
}

TEST(FaultResetPacingValidation, BoundedWithoutPacingIsRefused) {
    EXPECT_NE(validate_fault_reset(FaultResetPolicy::BOUNDED_AUTO_RESET, std::nullopt)
                  .message()
                  .find("no fault_reset_pacing"),
              std::string::npos);
}

TEST(FaultResetPacingValidation, BoundedRefusesZeroIntervalAndOutOfRangeAttempts) {
    const auto bounded = FaultResetPolicy::BOUNDED_AUTO_RESET;
    EXPECT_NE(validate_fault_reset(bounded, FaultResetPacing{0, 25}).message().find("interval_ms is 0"),
              std::string::npos);
    EXPECT_NE(validate_fault_reset(bounded, FaultResetPacing{200, 0}).message().find("max_attempts (0)"),
              std::string::npos);
    EXPECT_NE(validate_fault_reset(bounded, FaultResetPacing{200, kMaxSupportedFaultResetAttempts + 1})
                  .message()
                  .find("max_attempts (10001)"),
              std::string::npos);
    EXPECT_FALSE(validate_fault_reset(bounded, FaultResetPacing{200, UINT32_MAX}).is_ok())
        << "UINT32_MAX would wrap the give-up sentinel (cap + 1) to 0";
    // The ceiling itself and the floor are inside the range.
    EXPECT_TRUE(validate_fault_reset(bounded, FaultResetPacing{1, 1}).is_ok());
    EXPECT_TRUE(validate_fault_reset(bounded, FaultResetPacing{200, kMaxSupportedFaultResetAttempts}).is_ok());
    EXPECT_TRUE(validate_fault_reset(bounded, FaultResetPacing{200, 25}).is_ok());
}

TEST(FaultResetPacingValidation, OutOfRangePolicyIsRefusedWhateverThePacing) {
    const auto bogus = static_cast<FaultResetPolicy>(7);
    EXPECT_NE(validate_fault_reset(bogus, std::nullopt).message().find("out-of-range value (7)"), std::string::npos);
    EXPECT_NE(validate_fault_reset(bogus, FaultResetPacing{200, 25}).message().find("out-of-range value (7)"),
              std::string::npos);
}
