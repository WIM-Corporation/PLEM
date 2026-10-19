// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_sequence_step.cpp
/// @brief Decision table for the per-joint sequence step.
/// Pins the priority (arrival > declared fault reset > proven engage fallback
/// > transition command), every counter/latch edge, and the two-phase fault
/// reset fold: the pieces the golden traces exercise only end-to-end.

#include <gtest/gtest.h>

#include "ethercat/sequence_step.hpp"

using plem::fieldbus::ethercat::commit_fault_reset;
using plem::fieldbus::ethercat::decide_sequence_step;
using plem::fieldbus::ethercat::FaultResetPolicy;
using plem::fieldbus::ethercat::SequenceStepAction;
using plem::fieldbus::ethercat::SequenceStepInputs;
using plem::fieldbus::ethercat::SequenceStepState;

namespace {

// Production limits (controller.hpp kFaultReset* / stall threshold derived
// from the 2 s window at 1 kHz); the table uses small stand-ins on purpose:
// the decision is limit-agnostic and small numbers keep edges readable.
constexpr uint32_t kStallThr = 5;
constexpr uint32_t kInterval = 4;
constexpr uint32_t kMaxAttempts = 3;

SequenceStepInputs healthy_walk() {
    SequenceStepInputs in;
    in.initialized = true;
    in.at_target = false;
    in.is_faulted = false;
    in.can_transition = true;
    in.engage_settle_proven = false;
    in.release_target = true;
    in.policy = FaultResetPolicy::NONE;
    return in;
}

auto decide(const SequenceStepInputs& in) {
    return decide_sequence_step(in, kStallThr, kInterval, kMaxAttempts);
}

// Priority

TEST(SequenceStepDecision, ArrivalOutranksEverythingAndZeroesTheInterval) {
    auto in = healthy_walk();
    in.at_target = true;
    in.is_faulted = true;  // even a fault word at the target step is an arrival
    in.policy = FaultResetPolicy::BOUNDED_AUTO_RESET;
    in.state.reset_cycles = 3;
    in.state.stall_cycles = 4;
    in.state.stall_reported = true;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::ADVANCE);
    EXPECT_EQ(d.next.reset_cycles, 0u) << "arrival re-arms the reset interval";
    EXPECT_EQ(d.next.reset_attempts, 0u) << "attempts stayed 0 — ADVANCE does not touch them";
    EXPECT_EQ(d.next.stall_cycles, 0u) << "arrival re-arms the stall episode";
    EXPECT_FALSE(d.next.stall_reported);
    EXPECT_FALSE(d.emit_stall);
}

TEST(SequenceStepDecision, ArrivalKeepsAttemptAllowanceUntouched) {
    auto in = healthy_walk();
    in.at_target = true;
    in.state.reset_attempts = 2;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::ADVANCE);
    EXPECT_EQ(d.next.reset_attempts, 2u) << "only the healthy TRANSITION path pre-resets attempts";
}

TEST(SequenceStepDecision, DeclaredFaultOutranksTransitionCommand) {
    auto in = healthy_walk();
    in.is_faulted = true;
    in.can_transition = true;  // a faulted drive offering a row must still be reset, not stepped
    in.policy = FaultResetPolicy::BOUNDED_AUTO_RESET;
    in.state.reset_cycles = kInterval - 1;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::TRY_FAULT_RESET);
}

TEST(SequenceStepDecision, UndeclaredPolicyStallsOnFault) {
    auto in = healthy_walk();
    in.is_faulted = true;
    in.can_transition = false;
    in.policy = FaultResetPolicy::NONE;
    in.state.reset_cycles = 100;  // whatever accumulated: NONE never resets
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::HOLD);
    EXPECT_EQ(d.next.reset_cycles, 100u) << "NONE does not even run the interval";
}

TEST(SequenceStepDecision, ProvenEngageFallbackOutranksTransitionCommand) {
    auto in = healthy_walk();
    in.release_target = false;
    in.engage_settle_proven = true;
    in.can_transition = true;  // A proven-resting drive must not power up a stage
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::SETTLE_ENGAGED_EARLY);
}

TEST(SequenceStepDecision, ReleaseSequenceNeverTakesTheEngageFallback) {
    auto in = healthy_walk();
    in.release_target = true;
    in.engage_settle_proven = true;  // proven-closed is meaningless for a RELEASE
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::COMMAND_TRANSITION);
}

TEST(SequenceStepDecision, UnprovenEngageHoldsAliveInsteadOfSettling) {
    // Self leg: an unrecognized/unproven word must neither settle nor
    // consume; HOLD keeps the sequence alive for the next sample.
    auto in = healthy_walk();
    in.release_target = false;
    in.engage_settle_proven = false;
    in.can_transition = false;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::HOLD);
}

TEST(SequenceStepDecision, TransitionCommandPreResetsTheAttemptAllowance) {
    auto in = healthy_walk();
    in.can_transition = true;
    in.state.reset_attempts = 2;
    in.state.reset_cycles = 3;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::COMMAND_TRANSITION);
    EXPECT_EQ(d.next.reset_attempts, 0u) << "a healthy step ends the fault episode";
    EXPECT_EQ(d.next.reset_cycles, 3u) << "the elapsed interval is deliberately CARRIED (measured behavior)";
}

TEST(SequenceStepDecision, UninitializedAxisHoldsButStillCountsTheStall) {
    // The stall check sits above the initialization gate on purpose: an axis
    // that never produced a statusword while its sequence is active is
    // exactly the silent stall the event exists to name.
    auto in = healthy_walk();
    in.initialized = false;
    in.state.stall_cycles = kStallThr - 1;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::HOLD);
    EXPECT_TRUE(d.emit_stall);
    EXPECT_TRUE(d.next.stall_reported);
}

// Stall episode edges

TEST(SequenceStepDecision, StallCountsUpToTheWindowAndReportsOnce) {
    auto in = healthy_walk();
    in.can_transition = false;
    SequenceStepState s{};
    for (uint32_t cycle = 1; cycle < kStallThr; ++cycle) {
        in.state = s;
        const auto d = decide(in);
        EXPECT_FALSE(d.emit_stall) << "cycle " << cycle;
        EXPECT_EQ(d.next.stall_cycles, cycle);
        s = d.next;
    }
    in.state = s;
    const auto at_window = decide(in);
    EXPECT_TRUE(at_window.emit_stall) << "exactly at the window";
    EXPECT_TRUE(at_window.next.stall_reported);

    in.state = at_window.next;
    const auto after = decide(in);
    EXPECT_FALSE(after.emit_stall) << "one report per episode";
    EXPECT_EQ(after.next.stall_cycles, at_window.next.stall_cycles) << "the counter freezes once reported";
}

TEST(SequenceStepDecision, ZeroThresholdDisablesStallCounting) {
    auto in = healthy_walk();
    in.can_transition = false;
    in.state.stall_cycles = 1000;
    const auto d = decide_sequence_step(in, /*stall_threshold_cycles=*/0, kInterval, kMaxAttempts);
    EXPECT_FALSE(d.emit_stall);
    EXPECT_EQ(d.next.stall_cycles, 1000u) << "threshold 0 = the counter does not even run";
}

TEST(SequenceStepDecision, StallReportCoexistsWithTheSameCycleFaultReset) {
    // Both windows can elapse on one cycle; the report is not exclusive with
    // the action (the executor emits the stall first, pinned end-to-end by
    // StallReportOutrunsFaultResetInTheSameCycle).
    auto in = healthy_walk();
    in.is_faulted = true;
    in.can_transition = false;
    in.policy = FaultResetPolicy::BOUNDED_AUTO_RESET;
    in.state.stall_cycles = kStallThr - 1;
    in.state.reset_cycles = kInterval - 1;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::TRY_FAULT_RESET);
    EXPECT_TRUE(d.emit_stall);
}

// Fault-reset interval, cap and the two-phase fold

TEST(SequenceStepDecision, ResetWaitsOutTheFullIntervalThenRearms) {
    auto in = healthy_walk();
    in.is_faulted = true;
    in.can_transition = false;
    in.policy = FaultResetPolicy::BOUNDED_AUTO_RESET;
    SequenceStepState s{};
    for (uint32_t cycle = 1; cycle < kInterval; ++cycle) {
        in.state = s;
        const auto d = decide(in);
        EXPECT_EQ(d.action, SequenceStepAction::HOLD) << "cycle " << cycle;
        EXPECT_EQ(d.next.reset_cycles, cycle);
        s = d.next;
    }
    in.state = s;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::TRY_FAULT_RESET);
    EXPECT_EQ(d.next.reset_cycles, 0u) << "the interval re-arms whether or not the request is accepted";
    EXPECT_EQ(d.next.reset_attempts, in.state.reset_attempts)
        << "the DECISION never bumps attempts — only the commit fold may (device-acceptance dependent)";
}

TEST(SequenceStepDecision, CommitFoldConfirmsOnlyAcceptedResets) {
    SequenceStepState next{};
    next.reset_attempts = 1;
    const auto accepted = commit_fault_reset(next, /*reset_accepted=*/true);
    EXPECT_EQ(accepted.reset_attempts, 2u);
    const auto refused = commit_fault_reset(next, /*reset_accepted=*/false);
    EXPECT_EQ(refused.reset_attempts, 1u) << "a refused REQUEST consumes no allowance (measured behavior)";
}

TEST(SequenceStepDecision, CapCrossingGivesUpExactlyOnceThenStaysSilent) {
    auto in = healthy_walk();
    in.is_faulted = true;
    in.can_transition = false;
    in.policy = FaultResetPolicy::BOUNDED_AUTO_RESET;
    in.state.reset_cycles = kInterval - 1;
    in.state.reset_attempts = kMaxAttempts;  // allowance spent
    const auto give_up = decide(in);
    EXPECT_EQ(give_up.action, SequenceStepAction::GIVE_UP_FAULT_RESET);
    EXPECT_EQ(give_up.next.reset_attempts, kMaxAttempts + 1) << "the sentinel makes the give-up one-shot";

    in.state = give_up.next;
    in.state.reset_cycles = kInterval - 1;
    const auto after = decide(in);
    EXPECT_EQ(after.action, SequenceStepAction::HOLD) << "a stable fault: no more diagnostics";
    EXPECT_EQ(after.next.reset_cycles, 0u) << "the interval still runs; it just decides nothing";
}

TEST(SequenceStepDecision, EngageWithoutProofStillCommandsWhenARowExists) {
    // Recognition gates only the fallback: an engage whose drive offers a
    // legal row keeps walking even while the settle proof is absent.
    auto in = healthy_walk();
    in.release_target = false;
    in.engage_settle_proven = false;
    in.can_transition = true;
    EXPECT_EQ(decide(in).action, SequenceStepAction::COMMAND_TRANSITION);
}

TEST(SequenceStepDecision, EngageFallbackCoexistsWithTheStallReport) {
    // The fallback can fire on the exact cycle the stall window elapses;
    // the report rides along, it never suppresses the settle.
    auto in = healthy_walk();
    in.release_target = false;
    in.engage_settle_proven = true;
    in.can_transition = false;
    in.state.stall_cycles = kStallThr - 1;
    const auto d = decide(in);
    EXPECT_EQ(d.action, SequenceStepAction::SETTLE_ENGAGED_EARLY);
    EXPECT_TRUE(d.emit_stall);
}

// The decision is a constant expression: usable in static contexts and
// guaranteed side-effect-free (a pure function over POD).
static_assert(decide_sequence_step(SequenceStepInputs{}, 0, 4, 3).action == SequenceStepAction::HOLD);

}  // namespace
