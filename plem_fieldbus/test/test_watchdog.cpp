// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "plem/fieldbus/watchdog.hpp"

namespace plem::fieldbus {
namespace {

TEST(Watchdog, ResetsCounterOnSuccessfulCycle) {
    WatchdogState state{.mismatch_count = 3, .triggered = false, .armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/true, 5);
    EXPECT_EQ(result.next_state.mismatch_count, 0u);
    EXPECT_FALSE(result.next_state.triggered);
    EXPECT_FALSE(result.protective_stop_active);
}

TEST(Watchdog, IncrementsCounterOnMismatch) {
    WatchdogState state{.mismatch_count = 0, .triggered = false, .armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/false, 5);
    EXPECT_EQ(result.next_state.mismatch_count, 1u);
    EXPECT_FALSE(result.protective_stop_active);
}

TEST(Watchdog, TriggersProtectiveStopAfterThresholdConsecutiveMismatches) {
    WatchdogState state{.mismatch_count = 4, .triggered = false, .armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/false, 5);
    EXPECT_EQ(result.next_state.mismatch_count, 5u);
    EXPECT_TRUE(result.next_state.triggered);
    EXPECT_TRUE(result.protective_stop_active);
}

TEST(Watchdog, MismatchCountFreezesOnceTriggered) {
    WatchdogState state{.mismatch_count = 5, .triggered = true, .armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/false, 5);
    EXPECT_EQ(result.next_state.mismatch_count, 5u);
}

TEST(Watchdog, RecoveryAfterSuccessfulCyclePostTrigger) {
    WatchdogState state{.mismatch_count = 5, .triggered = true, .armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/true, 5);
    EXPECT_EQ(result.next_state.mismatch_count, 0u);
    EXPECT_FALSE(result.next_state.triggered);
}

TEST(Watchdog, ThresholdOneTriggersImmediately) {
    WatchdogState state{.armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/false, 1);
    EXPECT_TRUE(result.protective_stop_active);
}

// protective_stop_active: level view of the watchdog
// protective_stop_active stays asserted every cycle while the bus remains degraded beyond threshold. The level view
// lets the controller re-assert the EXTERNAL protective stop after an optimistic
// operator reset, so a reset can only stick once the bus is healthy again.

TEST(Watchdog, ProtectiveStopActiveAssertsAtThresholdAndWhileDegraded) {
    WatchdogState state{.mismatch_count = 4, .triggered = false, .armed = true};
    auto at_threshold = evaluate_watchdog(state, /*wkc_ok=*/false, 5);
    EXPECT_TRUE(at_threshold.protective_stop_active);

    auto still_degraded = evaluate_watchdog(at_threshold.next_state, /*wkc_ok=*/false, 5);
    EXPECT_TRUE(still_degraded.protective_stop_active);  // level keeps asserting
}

TEST(Watchdog, ProtectiveStopActiveFalseBeforeThreshold) {
    WatchdogState state{.armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/false, 5);
    EXPECT_FALSE(result.protective_stop_active);
}

TEST(Watchdog, ProtectiveStopActiveDropsOnRecovery) {
    WatchdogState state{.mismatch_count = 5, .triggered = true, .armed = true};
    auto result = evaluate_watchdog(state, /*wkc_ok=*/true, 5);
    EXPECT_FALSE(result.protective_stop_active);
}

// Boot arming: EtherCAT bring-up produces WKC mismatches while the slaves
// transition to OP. The watchdog detects degradation (healthy to unhealthy), so
// mismatch counting begins only after the bus has been healthy once. A bus that
// never comes up never enables the servos (no torque path, brakes hold).

TEST(Watchdog, SustainedBootMismatchesNeverTrigger) {
    WatchdogState state{};  // boot: the bus has never been healthy
    for (int cycle = 0; cycle < 100; ++cycle) {
        auto result = evaluate_watchdog(state, /*wkc_ok=*/false, 5);
        ASSERT_FALSE(result.protective_stop_active) << "bring-up mismatch must not trigger (cycle " << cycle << ")";
        ASSERT_FALSE(result.next_state.triggered);
        state = result.next_state;
    }
    EXPECT_FALSE(state.armed) << "mismatches must not start the watch (armed stays false)";
}

TEST(Watchdog, ArmsOnFirstHealthyCycleThenDetectsDegradation) {
    WatchdogState state{};
    auto armed = evaluate_watchdog(state, /*wkc_ok=*/true, 1);
    EXPECT_TRUE(armed.next_state.armed);

    auto degraded = evaluate_watchdog(armed.next_state, /*wkc_ok=*/false, 1);
    EXPECT_TRUE(degraded.protective_stop_active);
}

TEST(Watchdog, ArmingSurvivesRecovery) {
    // Recovery resets the count and the trigger but not the arming: a bus
    // that was healthy once stays in operation; later degradation must trigger.
    WatchdogState state{.mismatch_count = 5, .triggered = true, .armed = true};
    auto recovered = evaluate_watchdog(state, /*wkc_ok=*/true, 5);
    EXPECT_TRUE(recovered.next_state.armed);
    EXPECT_FALSE(recovered.next_state.triggered);
    EXPECT_EQ(recovered.next_state.mismatch_count, 0u);
}

// Host command writer → EtherCAT command freshness

namespace {
constexpr int64_t kBudget = kDefaultCommandStaleTimeoutNs;
}

TEST(CommandFreshness, UnarmedZeroStampNeverTrips) {
    // Bring-up gate: a writer that never stamped never produced torque either.
    CommandFreshnessState state{};
    auto result = evaluate_command_freshness(state, /*update_ns=*/0, /*now_ns=*/100 * kBudget, kBudget);
    EXPECT_FALSE(result.next_state.armed);
    EXPECT_FALSE(result.should_protective_stop);
    EXPECT_FALSE(result.protective_stop_active);
}

TEST(CommandFreshness, ArmsOnFirstStampAndStaysFreshWithinBudget) {
    CommandFreshnessState state{};
    auto result = evaluate_command_freshness(state, /*update_ns=*/1'000'000, /*now_ns=*/2'000'000, kBudget);
    EXPECT_TRUE(result.next_state.armed);
    EXPECT_FALSE(result.next_state.triggered);
    EXPECT_FALSE(result.should_protective_stop);
    EXPECT_FALSE(result.protective_stop_active);
}

TEST(CommandFreshness, ExactBudgetBoundaryIsNotStale) {
    // Strict greater-than, mirroring is_stopping_backstop_expired.
    CommandFreshnessState state{.triggered = false, .armed = true};
    auto result = evaluate_command_freshness(state, /*update_ns=*/1, /*now_ns=*/1 + kBudget, kBudget);
    EXPECT_FALSE(result.protective_stop_active);
    EXPECT_FALSE(result.next_state.triggered);
}

TEST(CommandFreshness, TripsOnceWhenBudgetExceeded) {
    CommandFreshnessState state{.triggered = false, .armed = true};
    auto result = evaluate_command_freshness(state, /*update_ns=*/1, /*now_ns=*/2 + kBudget, kBudget);
    EXPECT_TRUE(result.should_protective_stop);  // edge fires exactly at the crossing
    EXPECT_TRUE(result.protective_stop_active);
    EXPECT_TRUE(result.next_state.triggered);
}

TEST(CommandFreshness, LevelStaysAssertedWhileStaleWithoutReEdge) {
    CommandFreshnessState state{.triggered = true, .armed = true};
    auto result = evaluate_command_freshness(state, /*update_ns=*/1, /*now_ns=*/10 * kBudget, kBudget);
    EXPECT_FALSE(result.should_protective_stop);  // no event storm
    EXPECT_TRUE(result.protective_stop_active);   // re-assert defeats an optimistic clear
}

TEST(CommandFreshness, RecoversWhenWriterResumes) {
    CommandFreshnessState state{.triggered = true, .armed = true};
    auto result = evaluate_command_freshness(state, /*update_ns=*/10 * kBudget, /*now_ns=*/10 * kBudget + 1, kBudget);
    EXPECT_FALSE(result.should_protective_stop);
    EXPECT_FALSE(result.protective_stop_active);
    EXPECT_FALSE(result.next_state.triggered);
    EXPECT_TRUE(result.next_state.armed);  // arming survives recovery (watchdog contract)
}

TEST(CommandFreshness, AlreadyStaleFirstStampTripsImmediately) {
    // Writer wrote once and hung before this loop started observing: the
    // arming cycle itself must evaluate staleness, not grant a free pass.
    CommandFreshnessState state{};
    auto result = evaluate_command_freshness(state, /*update_ns=*/1, /*now_ns=*/2 + kBudget, kBudget);
    EXPECT_TRUE(result.next_state.armed);
    EXPECT_TRUE(result.should_protective_stop);
    EXPECT_TRUE(result.protective_stop_active);
}

// (The budget floor pin lives header-side in watchdog.hpp, next to the constant,
// so it fires in every consumer TU, not only when this test target builds.)

// constexpr anchors: the decision is computable at compile time (RT contract).
static_assert(!evaluate_command_freshness({}, 0, 1'000'000'000, kDefaultCommandStaleTimeoutNs).protective_stop_active,
              "unarmed zero stamp must not trip");
static_assert(evaluate_command_freshness({.triggered = false, .armed = true}, 1, 2 + kDefaultCommandStaleTimeoutNs,
                                         kDefaultCommandStaleTimeoutNs)
                  .should_protective_stop,
              "budget crossing must fire the edge");

}  // namespace
}  // namespace plem::fieldbus
