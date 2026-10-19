// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <limits>

#include <plem/fieldbus/ethercat/drive_safety.hpp>

namespace plem::fieldbus::ethercat {
namespace {

// (motion_enabled, all_drives_enabled, protective_stop) → torque reaches the drive.
// Exactly one open row: everything the host intends, every drive confirmed, no stop.
static_assert(drive_torque_allowed(true, true, false));
static_assert(!drive_torque_allowed(false, true, false));  // host disabled (disable/stop)
static_assert(!drive_torque_allowed(true, false, false));  // a drive not yet OE (SO priming window)
static_assert(!drive_torque_allowed(true, true, true));    // any P-STOP source
static_assert(!drive_torque_allowed(false, false, false));
static_assert(!drive_torque_allowed(false, false, true));
static_assert(!drive_torque_allowed(false, true, true));
static_assert(!drive_torque_allowed(true, false, true));

TEST(DriveTorqueGate, ExactlyOneOfEightStatesIsOpen) {
    int open = 0;
    for (int bits = 0; bits < 8; ++bits) {
        open += drive_torque_allowed((bits & 1) != 0, (bits & 2) != 0, (bits & 4) != 0) ? 1 : 0;
    }
    EXPECT_EQ(open, 1);
}

// CSP/CSV command checks. Limit ±1 rad, 2 rad/s; the cycle is
// 2^-10 s so one cycle's travel (2^-9 rad) and the boundary step are exact in
// binary. The bounds themselves are admitted; NaN is not.
constexpr core::MotionLimit kLimit{.lower_limit = -1.0, .upper_limit = 1.0, .velocity_limit = 2.0, .effort_limit = 5.0};
constexpr double kCycle = 0.0009765625;
constexpr bool csp_target_allowed(double target, double reference, const core::MotionLimit& limit, double cycle_s) {
    return csp_target_cause(target, reference, limit, cycle_s).reason == core::monitoring::LimitStopReason::NONE;
}
constexpr bool csv_target_allowed(double target, const core::MotionLimit& limit) {
    return csv_target_cause(target, limit).reason == core::monitoring::LimitStopReason::NONE;
}
static_assert(csp_target_allowed(0.5, 0.499, kLimit, kCycle));
static_assert(csp_target_allowed(1.0, 1.0, kLimit, kCycle));    // at the upper bound
static_assert(csp_target_allowed(-1.0, -1.0, kLimit, kCycle));  // at the lower bound
static_assert(!csp_target_allowed(1.0001, 1.0, kLimit, kCycle));
static_assert(!csp_target_allowed(-1.0001, -1.0, kLimit, kCycle));
static_assert(!csp_target_allowed(0.5, 0.49, kLimit, kCycle));        // 0.01 rad in one cycle
static_assert(!csp_target_allowed(0.49, 0.5, kLimit, kCycle));        // either direction
static_assert(csp_target_allowed(0.5, 0.498046875, kLimit, kCycle));  // exactly one cycle's travel
static_assert(csv_target_allowed(2.0, kLimit));
static_assert(csv_target_allowed(-2.0, kLimit));
static_assert(!csv_target_allowed(2.0001, kLimit));
static_assert(!csv_target_allowed(-2.0001, kLimit));

TEST(DriveCommandChecks, NonFiniteIsNeverAllowed) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(csp_target_allowed(nan, 0.0, kLimit, kCycle));
    EXPECT_FALSE(csp_target_allowed(0.0, nan, kLimit, kCycle));
    EXPECT_FALSE(csv_target_allowed(nan, kLimit));
}

// The cause each check names (LIMIT_STOP error_code); the bool checks
// are these with no reason. Feedback names an undecodable word first; its
// position is a saturated value.
using core::monitoring::LimitStopReason;
// The numbers are the wire contract (never reused), pinned here, not by name.
static_assert(static_cast<uint16_t>(LimitStopReason::NONE) == 0 &&
              static_cast<uint16_t>(LimitStopReason::POSITION) == 1 &&
              static_cast<uint16_t>(LimitStopReason::VELOCITY) == 2 &&
              static_cast<uint16_t>(LimitStopReason::UNDECODABLE) == 3 &&
              static_cast<uint16_t>(LimitStopReason::CSP_RANGE) == 4 &&
              static_cast<uint16_t>(LimitStopReason::CSP_STEP) == 5 &&
              static_cast<uint16_t>(LimitStopReason::CSV_VELOCITY) == 6 &&
              static_cast<uint16_t>(LimitStopReason::UNENCODABLE) == 7);
static_assert(csp_target_cause(1.0001, 1.0, kLimit, kCycle).reason == LimitStopReason::CSP_RANGE);
static_assert(csp_target_cause(0.5, 0.49, kLimit, kCycle).reason == LimitStopReason::CSP_STEP);
static_assert(csp_target_cause(0.5, 0.49, kLimit, kCycle).value == static_cast<float>(0.5 - 0.49));
static_assert(csv_target_cause(2.5, kLimit).reason == LimitStopReason::CSV_VELOCITY);
static_assert(feedback_stop_cause(0.0, 0.0, false, kLimit).reason == LimitStopReason::NONE);
static_assert(feedback_stop_cause(1.5, 0.0, false, kLimit).reason == LimitStopReason::POSITION);
static_assert(feedback_stop_cause(1.5, 0.0, false, kLimit).value == 1.5f);
static_assert(feedback_stop_cause(0.0, -3.0, false, kLimit).reason == LimitStopReason::VELOCITY);
static_assert(feedback_stop_cause(1.5, 3.0, true, kLimit).reason == LimitStopReason::UNDECODABLE);
// Overlaps name the first cause in the order above; each value is its own quantity.
static_assert(feedback_stop_cause(1.5, 3.0, false, kLimit).reason == LimitStopReason::POSITION);
static_assert(feedback_stop_cause(0.0, -3.0, false, kLimit).value == -3.0f);
static_assert(csp_target_cause(1.5, 0.0, kLimit, kCycle).reason == LimitStopReason::CSP_RANGE);
static_assert(csp_target_cause(1.5, 0.0, kLimit, kCycle).value == 1.5f);
static_assert(csv_target_cause(2.5, kLimit).value == 2.5f);
static_assert(csp_range_cause(1.0001, kLimit).reason == LimitStopReason::CSP_RANGE);
static_assert(csp_range_cause(1.0001, kLimit).value == 1.0001f);
static_assert(csp_range_cause(1.0, kLimit).reason == LimitStopReason::NONE);

// Requested stop

TEST(DriveStop, TheSpeedFallsByTheDecelerationEachCycleAndRestsWithoutReversing) {
    // 1 rad/s at 4 rad/s² on a 2^-10 s cycle: 2^-8 rad/s a cycle, 256 cycles.
    double velocity = 1.0;
    int cycles = 0;
    while (velocity != 0.0 && cycles < 1000) {
        const double next = stop_velocity_step(velocity, 4.0, kCycle);
        EXPECT_EQ(velocity - next, 0.00390625) << "cycle " << cycles;
        velocity = next;
        ++cycles;
    }
    EXPECT_EQ(cycles, 256);
    EXPECT_EQ(stop_velocity_step(-0.001, 4.0, kCycle), 0.0) << "under one step: rest, never the other way";
    EXPECT_EQ(stop_velocity_step(-0.5, 4.0, kCycle), -0.5 + 0.00390625);
    EXPECT_EQ(stop_velocity_step(0.0, 4.0, kCycle), 0.0);
}

constexpr int64_t kMs = 1'000'000;

RampStopDecision ramp_stop(RampStopState state, bool requested, bool live, bool at_rest, bool settled, bool all_closed,
                           int64_t now_ns, int64_t decelerate_ns = 0) {
    return decide_ramp_stop(state, RampStopInputs{.requested = requested,
                                                  .live = live,
                                                  .at_rest = at_rest,
                                                  .settled = settled,
                                                  .all_closed = all_closed,
                                                  .now_ns = now_ns,
                                                  .decelerate_ns = decelerate_ns});
}

TEST(DriveStop, ARequestWithMotionLiveDeceleratesUntilItsPlannedTimePlusTheSettleMargin) {
    const auto d = ramp_stop({}, true, true, false, false, false, 10 * kMs, 300 * kMs);
    EXPECT_EQ(d.next.phase, RampStopPhase::DECELERATING);
    EXPECT_EQ(d.next.deadline_ns, 10 * kMs + 300 * kMs + kRampStopSettleTimeoutNs);
    EXPECT_EQ(d.action, RampStopAction::NONE);
}

TEST(DriveStop, ARequestWithNothingMovingEngagesAtOnce) {
    const auto d = ramp_stop({}, true, false, false, false, false, 10 * kMs);
    EXPECT_EQ(d.next.phase, RampStopPhase::IDLE);
    EXPECT_EQ(d.action, RampStopAction::ENGAGE);
}

TEST(DriveStop, NoRequestNoAction) {
    const auto d = ramp_stop({}, false, true, true, true, false, 10 * kMs);
    EXPECT_EQ(d.next.phase, RampStopPhase::IDLE);
    EXPECT_EQ(d.action, RampStopAction::NONE);
}

TEST(DriveStop, TheBrakesEngageOnlyOnceTheCommandRestsAndTheAxesHaveSettled) {
    const RampStopState decelerating{RampStopPhase::DECELERATING, 500 * kMs};
    EXPECT_EQ(ramp_stop(decelerating, false, true, false, true, false, 100 * kMs).action, RampStopAction::NONE);
    EXPECT_EQ(ramp_stop(decelerating, false, true, true, false, false, 100 * kMs).action, RampStopAction::NONE)
        << "commanded at rest, an axis still moving";
    const auto done = ramp_stop(decelerating, false, true, true, true, false, 100 * kMs);
    EXPECT_EQ(done.action, RampStopAction::ENGAGE);
    EXPECT_EQ(done.next.phase, RampStopPhase::IDLE);
    EXPECT_EQ(ramp_stop(decelerating, true, true, false, false, false, 100 * kMs).next, decelerating)
        << "a second request changes nothing";
}

TEST(DriveStop, AStopNotConfirmedByItsDeadlineEscalatesToQuickStop) {
    const RampStopState decelerating{RampStopPhase::DECELERATING, 500 * kMs};
    EXPECT_EQ(ramp_stop(decelerating, false, true, true, false, false, 500 * kMs).action, RampStopAction::NONE)
        << "the deadline itself is still inside";
    const auto d = ramp_stop(decelerating, false, true, true, false, false, 500 * kMs + 1);
    EXPECT_EQ(d.action, RampStopAction::QUICK_STOP);
    EXPECT_EQ(d.next.phase, RampStopPhase::QUICK_STOP);
    EXPECT_EQ(d.next.deadline_ns, 500 * kMs + 1 + kQuickStopTimeoutNs);
}

TEST(DriveStop, MotionThatEndsMidDecelerationEngages) {
    // A protective stop closed the gate: its own forced engage runs; the stop still ends with motion off.
    const auto d = ramp_stop({RampStopPhase::DECELERATING, 500 * kMs}, false, false, false, false, false, 100 * kMs);
    EXPECT_EQ(d.action, RampStopAction::ENGAGE);
    EXPECT_EQ(d.next.phase, RampStopPhase::IDLE);
}

// The planned ramp time feeds an int64 deadline: a tiny declared deceleration or a
// NaN speed must saturate instead of converting out of range.
TEST(DriveStop, RampDurationSaturatesInsteadOfOverflowing) {
    EXPECT_EQ(ramp_stop_duration_ns(1.0, 2.0), 500'000'000);
    EXPECT_EQ(ramp_stop_duration_ns(-1.0, 2.0), 500'000'000);
    EXPECT_EQ(ramp_stop_duration_ns(1.0, 1e-10), 1'000'000'000'000'000'000) << "1e10 s saturates at 1e9 s";
    EXPECT_EQ(ramp_stop_duration_ns(std::numeric_limits<double>::quiet_NaN(), 2.0), 1'000'000'000'000'000'000);
    EXPECT_EQ(ramp_stop_duration_ns(0.0, 2.0), 0);
}

TEST(DriveStop, AQuickStopWaitsForTheDrivesToFinishTheirOwnRampThenEngages) {
    const RampStopState quick{RampStopPhase::QUICK_STOP, 3000 * kMs};
    EXPECT_EQ(ramp_stop(quick, false, false, false, false, false, 1000 * kMs).action, RampStopAction::NONE)
        << "the drives left Operation enabled for Quick stop active: not done";
    const auto closed = ramp_stop(quick, false, false, false, false, true, 1000 * kMs);
    EXPECT_EQ(closed.action, RampStopAction::ENGAGE);
    EXPECT_EQ(closed.next.phase, RampStopPhase::IDLE);
    const auto late = ramp_stop(quick, false, false, false, false, false, 3000 * kMs + 1);
    EXPECT_EQ(late.action, RampStopAction::ENGAGE) << "past its deadline the stop ends anyway";
}

}  // namespace
}  // namespace plem::fieldbus::ethercat
