// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_safety.hpp
/// @brief Drive-layer RT safety decision: a pure constexpr predicate
/// (decision = pure function, effect = caller). Consumed by EtherCATController's RT loop when a
/// host declares drive limits (set_drive_limits): a host without a control
/// thread of its own (DriveHost) cannot own the torque gate and the INTERNAL
/// protective stop, so the loop owns both. A host that declares no drive limits gates
/// and zeroes torque in its own control thread.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <plem/core/monitoring/event.hpp>
#include <plem/core/types/motion_limit.hpp>

namespace plem::fieldbus::ethercat {

/// Drive-layer torque gate: the three live facts read at the apply point of
/// the same cycle (execute_state_transitions has already published
/// all_drives_enabled). true = the host's torque command reaches the drive;
/// false = 0 Nm is written instead, so disable / stop / protective stop hold 0 even
/// when the drive ignores its engage sequence. A host that declared no drive
/// limits must gate torque upstream in its own control thread with the same
/// three terms (motion_enabled && all_drives_enabled && !protective_stop); the loop then
/// never evaluates this (constant open, no flag loads). Emitted torque is primed from
/// SWITCHED_ON on (may_hold_brake_open), so the gate also covers the Switched on
/// priming window until every drive is enabled.
[[nodiscard]] constexpr bool drive_torque_allowed(bool motion_enabled, bool all_drives_enabled,
                                                  bool protective_stop) noexcept {
    return motion_enabled && all_drives_enabled && !protective_stop;
}

/// Why a drive-layer check refuses (LIMIT_STOP's error_code and value);
/// NONE = passes.
struct LimitStopCause {
    core::monitoring::LimitStopReason reason = core::monitoring::LimitStopReason::NONE;
    float value = 0.0f;
};

/// Measured feedback against the drive limits. An undecodable word is named
/// first: its position is a saturated value, not where the axis is.
[[nodiscard]] constexpr LimitStopCause feedback_stop_cause(double position_rad, double velocity_rad_s, bool undecodable,
                                                           const core::MotionLimit& limit) noexcept {
    using core::monitoring::LimitStopReason;
    if (undecodable) {
        return {LimitStopReason::UNDECODABLE, std::numeric_limits<float>::quiet_NaN()};
    }
    if (!(position_rad >= limit.lower_limit && position_rad <= limit.upper_limit)) {
        return {LimitStopReason::POSITION, static_cast<float>(position_rad)};
    }
    if (!(velocity_rad_s <= limit.velocity_limit && -velocity_rad_s <= limit.velocity_limit)) {
        return {LimitStopReason::VELOCITY, static_cast<float>(velocity_rad_s)};
    }
    return {};
}

/// CSP range check: inside the position limits. NaN is refused.
[[nodiscard]] constexpr LimitStopCause csp_range_cause(double target_rad, const core::MotionLimit& limit) noexcept {
    if (!(target_rad >= limit.lower_limit && target_rad <= limit.upper_limit)) {
        return {core::monitoring::LimitStopReason::CSP_RANGE, static_cast<float>(target_rad)};
    }
    return {};
}

/// CSP target check: inside the position limits and at most
/// one cycle's travel at the velocity limit from `reference_rad`, the target
/// this axis applied the cycle before, or its actual position on the first
/// application after any cycle that applied nothing. Checks the angle the
/// encoded count decodes back to (CheckedTarget::si), not the request. NaN
/// anywhere is refused (every comparison is false).
[[nodiscard]] constexpr LimitStopCause csp_target_cause(double target_rad, double reference_rad,
                                                        const core::MotionLimit& limit, double cycle_s) noexcept {
    using core::monitoring::LimitStopReason;
    if (const LimitStopCause range = csp_range_cause(target_rad, limit); range.reason != LimitStopReason::NONE) {
        return range;
    }
    const double travel = limit.velocity_limit * cycle_s;
    const double step = target_rad - reference_rad;
    if (!(step <= travel && -step <= travel)) {
        return {LimitStopReason::CSP_STEP, static_cast<float>(step)};
    }
    return {};
}

/// Requested stop, one cycle of a csp or csv axis's commanded speed: it falls
/// by `deceleration` × cycle toward 0 and rests there, never past 0, so a stop
/// does not come back. A csp axis advances its target by the returned speed.
/// deceleration and cycle_s > 0 (caller's contract).
[[nodiscard]] constexpr double stop_velocity_step(double velocity, double deceleration, double cycle_s) noexcept {
    const double dv = deceleration * cycle_s;
    return velocity > dv ? velocity - dv : velocity < -dv ? velocity + dv : 0.0;
}

/// After the planned deceleration, how long the axes get to settle before the
/// stop counts as not confirmed.
inline constexpr int64_t kRampStopSettleTimeoutNs = 500'000'000;
/// How long the drives get to finish their own Quick stop ramp (0x6085, then
/// 0x605A's end state) before the stop ends anyway.
inline constexpr int64_t kQuickStopTimeoutNs = 2'000'000'000;

/// Nanoseconds a ramp from |velocity| to rest takes at `deceleration` (> 0),
/// saturated at 1e9 s so the int64 conversion is defined for any velocity and
/// any deceleration > 0 (a tiny declared deceleration, or a NaN speed, cannot
/// overflow it).
[[nodiscard]] inline int64_t ramp_stop_duration_ns(double velocity, double deceleration) noexcept {
    constexpr double kMaxSeconds = 1e9;  // far past any plan; min(kMax, NaN) is kMax
    return static_cast<int64_t>(std::min(kMaxSeconds, std::abs(velocity) / deceleration) * 1e9);
}

/// A requested stop (EtherCATController::request_ramp_stop): DECELERATING while the
/// loop ramps every csp and csv command to rest, QUICK_STOP once that was not
/// confirmed in time and the drives ramp down on their own.
enum class RampStopPhase : uint8_t {
    IDLE,
    DECELERATING,
    QUICK_STOP
};

struct RampStopState {
    RampStopPhase phase = RampStopPhase::IDLE;
    int64_t deadline_ns = 0;
    constexpr bool operator==(const RampStopState&) const = default;
};

struct RampStopInputs {
    bool requested;   ///< a stop request taken this cycle
    bool live;        ///< the torque gate is open this cycle (commands reach the drives)
    bool at_rest;     ///< every stopping command has reached speed 0
    bool settled;     ///< every axis's measured speed is inside its settle band
    bool all_closed;  ///< every axis reads a state that proves its brake closed
    int64_t now_ns;
    int64_t decelerate_ns;  ///< the planned deceleration from this cycle's speeds (read at a request)
};

enum class RampStopAction : uint8_t {
    NONE,
    ENGAGE,      ///< motion off and every brake engaged; the stop is over
    QUICK_STOP,  ///< command Quick stop to every enabled drive
};

struct RampStopDecision {
    RampStopState next;
    RampStopAction action = RampStopAction::NONE;
};

/// One cycle of a requested stop. Engages once the commands rest and the axes
/// have settled; if the stop is not confirmed within the planned deceleration plus
/// kRampStopSettleTimeoutNs, it commands Quick stop to every enabled drive and waits for the drives to
/// reach a brake-closed state (0x605A = 2: Switch on disabled) or
/// kQuickStopTimeoutNs, then engages. Motion that ends otherwise (a protective
/// stop closed the gate) ends the stop too. A request with nothing moving
/// engages at once.
[[nodiscard]] constexpr RampStopDecision decide_ramp_stop(RampStopState state, const RampStopInputs& in) noexcept {
    switch (state.phase) {
        case RampStopPhase::IDLE:
            if (!in.requested) {
                return {state};
            }
            if (!in.live) {
                return {{}, RampStopAction::ENGAGE};
            }
            return {{RampStopPhase::DECELERATING, in.now_ns + in.decelerate_ns + kRampStopSettleTimeoutNs}};
        case RampStopPhase::DECELERATING:
            if (!in.live || (in.at_rest && in.settled)) {
                return {{}, RampStopAction::ENGAGE};
            }
            if (in.now_ns > state.deadline_ns) {
                return {{RampStopPhase::QUICK_STOP, in.now_ns + kQuickStopTimeoutNs}, RampStopAction::QUICK_STOP};
            }
            return {state};
        case RampStopPhase::QUICK_STOP:
            if (in.all_closed || in.now_ns > state.deadline_ns) {
                return {{}, RampStopAction::ENGAGE};
            }
            return {state};
    }
    return {state};
}

/// CSV target check: |target| within the velocity limit (the decoded value of
/// the encoded count). NaN is refused.
[[nodiscard]] constexpr LimitStopCause csv_target_cause(double target_rad_s, const core::MotionLimit& limit) noexcept {
    if (!(target_rad_s <= limit.velocity_limit && -target_rad_s <= limit.velocity_limit)) {
        return {core::monitoring::LimitStopReason::CSV_VELOCITY, static_cast<float>(target_rad_s)};
    }
    return {};
}

}  // namespace plem::fieldbus::ethercat
