// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/types/control_mode_flags.hpp>

namespace plem::core {
namespace {

// Compile-time pins.
// evaluate_internal_protective_stop: violation present → set_protective_stop fires
static_assert(evaluate_internal_protective_stop(false, false, false, false).set_protective_stop);
// evaluate_internal_protective_stop: reset + within limits + no persistent fault → clear fires, not rejected
static_assert(evaluate_internal_protective_stop(true, true, true, false).clear_protective_stop);
static_assert(!evaluate_internal_protective_stop(true, true, true, false).clear_rejected);
// evaluate_internal_protective_stop: a persistent internal fault vetoes the clear even at a safe pose
static_assert(!evaluate_internal_protective_stop(true, true, true, true).clear_protective_stop);
static_assert(evaluate_internal_protective_stop(true, true, true, true).clear_rejected);

TEST(EvaluateInternalProtectiveStop, SetsOnLimitViolation) {
    auto d =
        evaluate_internal_protective_stop(/*initial=*/false, /*all_within_limits=*/false, /*reset_requested=*/false,
                                          /*persistent_fault=*/false);
    EXPECT_TRUE(d.set_protective_stop);
    EXPECT_FALSE(d.clear_protective_stop);
    EXPECT_FALSE(d.clear_rejected);
}

TEST(EvaluateInternalProtectiveStop, RefiresSetEveryCycleWhileLatched) {
    // initial_internal_protective_stop carries the latch; set re-fires even with limits OK.
    auto d = evaluate_internal_protective_stop(/*initial=*/true, /*all_within_limits=*/true, /*reset_requested=*/false,
                                               /*persistent_fault=*/false);
    EXPECT_TRUE(d.set_protective_stop);
}

TEST(EvaluateInternalProtectiveStop, ClearsOnResetWhenWithinLimits) {
    auto d = evaluate_internal_protective_stop(/*initial=*/true, /*all_within_limits=*/true, /*reset_requested=*/true,
                                               /*persistent_fault=*/false);
    EXPECT_TRUE(d.clear_protective_stop);
    EXPECT_FALSE(d.clear_rejected);
}

TEST(EvaluateInternalProtectiveStop, RejectsResetWhenLimitsStillViolated) {
    auto d = evaluate_internal_protective_stop(/*initial=*/true, /*all_within_limits=*/false, /*reset_requested=*/true,
                                               /*persistent_fault=*/false);
    EXPECT_TRUE(d.clear_rejected);
    EXPECT_FALSE(d.clear_protective_stop);
    // clear_protective_stop and clear_rejected are mutually exclusive (within vs. !within).
}

TEST(EvaluateInternalProtectiveStop, PersistentFaultVetoesClearAtSafePose) {
    // A source that re-asserts independent of pose (faulted freedrive law):
    // clearing would be a phantom recovery edge; the reset must be rejected
    // and the latch kept asserted.
    auto d = evaluate_internal_protective_stop(/*initial=*/true, /*all_within_limits=*/true, /*reset_requested=*/true,
                                               /*persistent_fault=*/true);
    EXPECT_TRUE(d.set_protective_stop);
    EXPECT_FALSE(d.clear_protective_stop);
    EXPECT_TRUE(d.clear_rejected);
}

TEST(EvaluateInternalProtectiveStop, PersistentFaultAssertsSetWithoutReset) {
    // While the fault holds, the INTERNAL latch stays asserted from the
    // decision layer itself, not only from the freedrive leg's own set.
    auto d = evaluate_internal_protective_stop(/*initial=*/false, /*all_within_limits=*/true, /*reset_requested=*/false,
                                               /*persistent_fault=*/true);
    EXPECT_TRUE(d.set_protective_stop);
    EXPECT_FALSE(d.clear_protective_stop);
    EXPECT_FALSE(d.clear_rejected);
}

TEST(EvaluateInternalProtectiveStop, KeepsLatchWhenNoResetRequested) {
    auto d = evaluate_internal_protective_stop(/*initial=*/true, /*all_within_limits=*/true, /*reset_requested=*/false,
                                               /*persistent_fault=*/false);
    EXPECT_FALSE(d.clear_protective_stop);
    EXPECT_FALSE(d.clear_rejected);
    EXPECT_TRUE(d.set_protective_stop);  // latch held
}

TEST(EvaluateInternalProtectiveStop, FiresSetBeforeClearOnTheClearingCycle) {
    // On the clearing cycle both flags are true; the caller applies set before
    // clear, so the set call still happens.
    auto d = evaluate_internal_protective_stop(/*initial=*/true, /*all_within_limits=*/true, /*reset_requested=*/true,
                                               /*persistent_fault=*/false);
    EXPECT_TRUE(d.set_protective_stop);
    EXPECT_TRUE(d.clear_protective_stop);
}

}  // namespace
}  // namespace plem::core
