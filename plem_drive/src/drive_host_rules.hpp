// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_host_rules.hpp
/// @brief DriveHost's pure rules (not installed): the enable and stop
/// decisions, the endless-axis reads, the limits and stop decelerations it
/// hands the controller and the csp velocity cap.
/// Pure so test_drive_host pins them without a bus (that target links the
/// real libethercat and cannot run a fake master).

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>

#include <plem/core/types/motion_limit.hpp>
#include <plem/core/units/pulse.hpp>
#include <plem/fieldbus/ethercat/controller.hpp>
#include <plem/fieldbus/ethercat/utils/servo_conversions.hpp>

#include "plem/drive/models/drive_model_spec.hpp"

namespace plem::drive::detail {

inline constexpr std::string_view kDriveHostTag = "DriveHost";

/// Why enable() refuses, or what it does instead.
enum class EnableAction : uint8_t {
    RELEASE,         ///< post the brake release (motion_enabled ← true first)
    ALREADY_ENABLED,  ///< every brake released and settled: no-op, ok
    NOT_STARTED,      ///< start() has not run (or stop() has)
    NOT_APPROVED,     ///< approval != APPROVED: motion refused
    PROTECTIVE_STOP,  ///< a protective stop source is latched: reset() first, wait for clear
    STALE_COMMAND,    ///< a cst bus without a host write() inside the freshness window: torque would start from a stale
                      ///< or missing command
    STOPPING,        ///< a stop (ramp_stop) is still running; it ends with the brakes engaged; enable after it
};

struct EnableInputs {
    bool started;
    bool stopped;
    models::Approval approval;
    bool protective_stop;
    bool command_fresh;
    uint8_t brake_engaged;            ///< bit i = axis i engaged
    uint8_t brake_operation_pending;  ///< bit i = axis i release/engage unsettled
    /// The bench bootstrap (DriveHostParams::bench_unapproved): an unapproved
    /// model may open motion so the bench can produce the record that
    /// approves it. Default false: the only path an unapproved model has.
    bool bench_unapproved = false;
    /// A cst axis on the bus: torque must already stream at the release. A
    /// bus without one holds the position it finds (EtherCATController::open_command_epoch).
    bool streams_torque = true;
    bool stopping = false;  ///< EtherCATController::ramp_stop_active()
};

[[nodiscard]] constexpr EnableAction decide_enable(EnableInputs in) noexcept {
    if (!in.started || in.stopped) {
        return EnableAction::NOT_STARTED;
    }
    if (in.approval != models::Approval::APPROVED && !in.bench_unapproved) {
        return EnableAction::NOT_APPROVED;
    }
    if (in.protective_stop) {
        return EnableAction::PROTECTIVE_STOP;
    }
    if (in.stopping) {
        return EnableAction::STOPPING;
    }
    if (in.streams_torque && !in.command_fresh) {
        return EnableAction::STALE_COMMAND;
    }
    if (in.brake_engaged == 0 && in.brake_operation_pending == 0) {
        return EnableAction::ALREADY_ENABLED;
    }
    return EnableAction::RELEASE;
}

/// What DriveHost::stop does, decided before it touches the bus.
enum class StopAction : uint8_t {
    NOTHING,   ///< never started, or stop() already ran: report, touch nothing
    STOP_LOOP,  ///< started but never released: no brake was opened; stop the loop, nothing to confirm
    QUIESCE,   ///< released since start: ramp_stop, disable, confirm every brake engaged, then stop the loop
};

/// `released`: an enable() reached its release since start. Before that the
/// brakes are as the run found them (engaged; the drives were not enabled),
/// and confirming them would wait on statuswords a slave short of OP never sends.
[[nodiscard]] constexpr StopAction decide_stop(bool started, bool stopped, bool released) noexcept {
    if (!started || stopped) {
        return StopAction::NOTHING;
    }
    return released ? StopAction::QUIESCE : StopAction::STOP_LOOP;
}

/// The stop outcome of a run that released nothing (NOTHING / STOP_LOOP):
/// every one of `axes` joints engaged (the state the run found) and
/// never_released, so it is not read as a confirmation.
[[nodiscard]] constexpr fieldbus::ethercat::BrakeQuiesceOutcome unreleased_stop_outcome(size_t axes) noexcept {
    fieldbus::ethercat::BrakeQuiesceOutcome outcome;
    outcome.all_joints_mask = static_cast<uint8_t>((1u << axes) - 1u);
    outcome.engaged_mask = outcome.all_joints_mask;
    outcome.never_released = true;
    return outcome;
}

/// An axis without end stops: the file's `limits.position: null`, held as
/// (-inf, +inf): an endless axis in every mode. The one place that reads it
/// from the limits.
[[nodiscard]] constexpr bool axis_endless(double position_min_rad, double position_max_rad) noexcept {
    return position_min_rad == -std::numeric_limits<double>::infinity() &&
           position_max_rad == std::numeric_limits<double>::infinity();
}

/// An endless axis's hardware count past the drive's int32 0x6064 range, in
/// motor turns (`encoder_resolution` counts each, nonzero per the spec gate);
/// nullopt inside [INT32_MIN, INT32_MAX].
[[nodiscard]] constexpr std::optional<double> endless_turns_past_int32(int64_t count,
                                                                       uint32_t encoder_resolution) noexcept {
    constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
    constexpr int64_t kMin = std::numeric_limits<int32_t>::min();
    const int64_t past = count > kMax ? count - kMax : count < kMin ? kMin - count : 0;
    if (past == 0) {
        return std::nullopt;
    }
    return static_cast<double>(past) / encoder_resolution;
}

/// MotionLimit per axis for the RT safety step (set_drive_limits).
/// effort_limit is informational (MotionLimit::is_valid wants > 0); the
/// drive-side clamp is ServoInfo's; a csp or csv axis has none (+inf).
template <size_t N>
[[nodiscard]] std::array<core::MotionLimit, N> drive_limits(const models::DriveModelSpec<N>& spec) noexcept {
    std::array<core::MotionLimit, N> out{};
    for (size_t i = 0; i < N; ++i) {
        const auto& s = spec.safety[i];
        out[i] = core::MotionLimit{.lower_limit = s.position_min_rad,
                                   .upper_limit = s.position_max_rad,
                                   .velocity_limit = s.velocity_rad_s,
                                   .effort_limit =
                                       spec.axes[i].modes_of_operation == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE
                                           ? std::max(-s.torque_min_nm, s.torque_max_nm)
                                           : std::numeric_limits<double>::infinity()};
    }
    return out;
}

/// Each axis's stop deceleration for EtherCATController::set_drive_stop; 0 on a cst axis.
template <size_t N>
[[nodiscard]] std::array<double, N> drive_stop_decelerations(const models::DriveModelSpec<N>& spec) noexcept {
    std::array<double, N> out{};
    for (size_t i = 0; i < N; ++i) {
        out[i] = spec.safety[i].acceleration_rad_s2;
    }
    return out;
}

/// A cst axis on the bus: it streams torque, so a release needs a fresh
/// command and a stop is the drives' own disable reaction.
template <size_t N>
[[nodiscard]] bool drive_streams_torque(const models::DriveModelSpec<N>& spec) noexcept {
    return std::any_of(spec.axes.begin(), spec.axes.end(), [](const models::AxisSpec& axis) {
        return axis.modes_of_operation == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
    });
}

/// Axis i's csp velocity cap (rad/s), the speed a planner may command: the
/// velocity limit less one position count per cycle, so a target rounded to
/// the count grid never steps past the one-cycle step check (limit × cycle);
/// an axis without end stops keeps a second count for the binary64 error of
/// angles far past int32 counts.
template <size_t N>
[[nodiscard]] double drive_csp_velocity_cap(const models::DriveModelSpec<N>& spec, size_t i) noexcept {
    const auto& s = spec.safety[i];
    const double margin_counts = axis_endless(s.position_min_rad, s.position_max_rad) ? 2.0 : 1.0;
    const double count_rad = fieldbus::ethercat::to_angle(core::units::Pulse(1), spec.axes[i].factors.position).value();
    return s.velocity_rad_s - margin_counts * count_rad / (spec.cycle_time_us * 1e-6);
}

}  // namespace plem::drive::detail
