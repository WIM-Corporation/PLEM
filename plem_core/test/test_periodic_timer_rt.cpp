// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <limits>
#include <plem/core/timing/periodic_timer_rt.hpp>
#include <plem/core/timing/timespec.hpp>
#include <thread>
#include <type_traits>

using namespace plem::core;

/// PeriodicTimerRT regression tests: overrun rebase and phase alignment.
class PeriodicTimerRTTest : public ::testing::Test {};

// The timer owns its implementation: a copy would share it and free it twice.
static_assert(!std::is_copy_constructible_v<PeriodicTimerRT>);
static_assert(!std::is_copy_assignable_v<PeriodicTimerRT>);

TEST_F(PeriodicTimerRTTest, OverrunDoesNotRunCatchUpCyclesBackToBack) {
    constexpr unsigned overrun_period_us = 20'000;  // 20ms; large enough to avoid scheduler noise.
    PeriodicTimerRT timer(overrun_period_us);

    timer.wait_period();
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    const auto before = std::chrono::steady_clock::now();
    timer.wait_period();
    const auto elapsed = std::chrono::steady_clock::now() - before;

    EXPECT_GT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 5)
        << "After an overrun, the timer must rebase to a future period instead of "
           "running immediate catch-up cycles; catch-up cycles can make EtherCAT "
           "datagrams get queued too soon and appear as SKIPPED/UNMATCHED.";
}

// ── start_at(): one-shot phase-alignment seam ──

TEST_F(PeriodicTimerRTTest, StartAtSetsFirstWake) {
    // Contract: the next wait_period() after start_at(t) wakes at t (absolute
    // CLOCK_MONOTONIC), not at "start time + period". 20 ms period keeps the
    // assertions well above scheduler noise (same rationale as the overrun test).
    constexpr unsigned period_us = 20'000;
    constexpr int64_t period_ns = static_cast<int64_t>(period_us) * 1000;
    PeriodicTimerRT timer(period_us);

    const int64_t target = now_ns() + 3 * period_ns + period_ns / 2;  // future, mid-phase
    timer.start_at(target);

    timer.wait_period();
    const int64_t wake = now_ns();

    EXPECT_GE(wake, target) << "woke before the requested first wake";
    EXPECT_LT(wake - target, period_ns / 4) << "first wake is not at the requested time";
}

TEST_F(PeriodicTimerRTTest, StartAtPastTimeRebasesPhasePreserving) {
    // Contract pinned for the phase-lock align branch: a past target is normalized
    // forward by the existing overrun rebase (whole periods): the first wake lands
    // on the earliest future point phase-equal to the target, never an immediate
    // catch-up wake and never a phase shift. This is why the align caller needs no
    // min-future guard of its own.
    constexpr unsigned period_us = 20'000;
    constexpr int64_t period_ns = static_cast<int64_t>(period_us) * 1000;
    PeriodicTimerRT timer(period_us);

    const int64_t before = now_ns();
    const int64_t target = before - 3 * period_ns - period_ns / 2;  // past, mid-phase
    timer.start_at(target);
    timer.wait_period();
    const int64_t wake = now_ns();

    EXPECT_GT(wake, before) << "must not return immediately into a catch-up cycle";
    const int64_t phase = (wake - target) % period_ns;
    EXPECT_LT(phase, period_ns / 4) << "rebase shifted the phase off the target's period grid";
    EXPECT_LT(wake - before, period_ns) << "slept past the earliest future phase point";
}

// INT64_MIN is a legal (if pointless) past target: it is taken at its phase (the
// earliest non-negative time on the period grid), so neither the seed nor the
// rebase can overflow, with a period under and over one second (a one-second
// period once put tv_sec * 1e9 past int64).
TEST_F(PeriodicTimerRTTest, StartAtTakesANegativeTargetAtItsPhase) {
    for (const unsigned period_us : {3'000u, 20'000u, 1'000'000u}) {  // 3 ms does not divide 1 s
        SCOPED_TRACE(period_us);
        const int64_t period_ns = static_cast<int64_t>(period_us) * 1000;
        PeriodicTimerRT timer(period_us);
        const int64_t target = std::numeric_limits<int64_t>::min();
        const int64_t residue = target % period_ns + period_ns;  // the contract's phase point in (0, period]

        const int64_t before = now_ns();
        timer.start_at(target);
        timer.wait_period();
        const int64_t wake = now_ns();

        EXPECT_LT(wake - before, period_ns + period_ns / 4) << "slept past the earliest future phase point";
        EXPECT_LT((wake - residue) % period_ns, period_ns / 4) << "the first wake left the target's period grid";
    }
}
