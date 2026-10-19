// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/ethercat/controller.hpp"

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <fmt/format.h>

#include <plem/core/logging/logger.hpp>
#include <plem/core/monitoring.hpp>
#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/threading/rt_memory.hpp>
#include <plem/core/timing/cycle_conversion.hpp>
#include <plem/core/timing/timespec.hpp>
#include <plem/core/units.hpp>
#include <plem/fieldbus/ethercat/state_machine.hpp>

#include "ethercat/brake_cycle_step.hpp"
#include "ethercat/joint_data_plane.hpp"
#include "ethercat/rt_cycle_monitor.hpp"
#include "ethercat/sequence_step.hpp"
#include "plem/fieldbus/watchdog.hpp"

namespace plem::fieldbus::ethercat {

using plem::core::now_ns;
using plem::core::monitoring::LimitStopReason;

static auto& logger = plem::core::Logger::instance();

namespace {

// Fail fast at construction: the ctor derives cycle counts by dividing by the
// period, which would otherwise hit divide-by-zero UB before PeriodicTimerRT's
// own positive-period check (which only runs at start()).
int64_t validated_cycle_period_ns(int cycle_time_us) {
    if (cycle_time_us <= 0) {
        throw std::invalid_argument(
            fmt::format("EtherCATController: config.cycle_time_us must be > 0, got {}", cycle_time_us));
    }
    return static_cast<int64_t>(cycle_time_us) * 1000;
}

/// The mode-acceptance predicate: the axis has produced a recognized
/// statusword and its 0x6061 display equals the declared mode it is commanded
/// on 0x6060. Checked on the RT-owned PDO image of the current cycle.
bool in_declared_mode(const DeviceServo& servo) noexcept {
    return servo.is_initialized() && servo.status_recognized() &&
           servo.get_modes_of_operation_display() == static_cast<int8_t>(servo.get_modes_of_operation());
}

}  // namespace

template <size_t DOF>
EtherCATController<DOF>::EtherCATController(const Config& config)
    : config_(config),
      cycle_period_ns_(validated_cycle_period_ns(config.cycle_time_us)),
      wkc_threshold_cycles_(plem::core::cycles_at_least(kWkcLossThresholdNs, cycle_period_ns_)) {
    // Sequence silent-stall window, set as a time like the WKC threshold
    // above: an active brake sequence off its step for this long emits one
    // BRAKE_SEQUENCE_STALLED per episode (observability only).
    sequence_stall_threshold_cycles_ = plem::core::cycles_at_least(kBrakeSequenceStallNs, cycle_period_ns_);
}

template <size_t DOF>
EtherCATController<DOF>::~EtherCATController() {
    stop();
}

template <size_t DOF>
core::Result<void> EtherCATController<DOF>::start() {
    if (running_.load(std::memory_order_acquire)) {
        return core::Result<void>::ok();  // Already running
    }

    auto validation = validate_preconditions();
    if (!validation) {
        return validation;
    }

    running_.store(true, std::memory_order_release);
    try {
        thread_ = std::thread(&EtherCATController::run_loop, this);
    } catch (const std::exception& e) {  // system_error, or libstdc++'s allocation for the thread state
        running_.store(false, std::memory_order_release);  // or the next start() would report a loop that never ran
        return core::Result<void>::error(std::string("[") + logger_tag() +
                                         "] cannot create the RT thread: " + e.what());
    }

    pthread_setname_np(thread_.native_handle(), "FieldbusCtrl");

    return core::Result<void>::ok();
}

template <size_t DOF>
void EtherCATController<DOF>::stop() {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
}

template <size_t DOF>
bool EtherCATController<DOF>::is_running() const {
    return running_.load(std::memory_order_acquire);
}

// The policy/priority/affinity primitive is single-sourced in
// plem::core::threading::apply_rt_thread_config. These two methods only adapt the flat Config fields and route the
// result into the bus logger; the scheduling and affinity aspects are split
// across them because run_loop reports each failure independently.
template <size_t DOF>
bool EtherCATController<DOF>::apply_rt_scheduling() {
    const auto result =
        plem::core::threading::apply_rt_thread_config({.policy = config_.policy, .priority = config_.rt_priority});
    if (!result.scheduling_ok) {
        logger.error(
            "[{}] RT scheduling (priority={}) failed: {}; running with the default policy. Grant an rtprio "
            "limit or CAP_SYS_NICE (docs/install.md §3)",
            logger_tag(), config_.rt_priority, std::strerror(result.sched_errno));
    }
    return result.scheduling_ok;
}

template <size_t DOF>
bool EtherCATController<DOF>::apply_cpu_affinity() {
    const auto result = plem::core::threading::apply_rt_thread_config(
        {.policy = plem::core::threading::SchedulingPolicy::OTHER, .affinity = config_.cpu_affinity});
    if (!result.affinity_ok) {
        logger.error("[{}] CPU affinity failed: {}; running without pinning", logger_tag(),
                     std::strerror(result.affinity_errno));
    }
    return result.affinity_ok;
}

template <size_t DOF>
void EtherCATController<DOF>::set_bus(std::shared_ptr<EtherCATBus> bus) {
    this->throw_if_running("set_bus");
    ecat_bus_ = std::move(bus);
}

template <size_t DOF>
void EtherCATController<DOF>::run_watchdog_cycle(bool wkc_ok) noexcept {
    const auto wd_result = plem::fieldbus::evaluate_watchdog(watchdog_state_, wkc_ok, wkc_threshold_cycles_);
    watchdog_state_ = wd_result.next_state;

    // Route WKC degradation to the live safety surface. Level-re-assert semantics: see
    // WatchdogResult::protective_stop_active. The null guard keeps this method total for
    // direct unit calls; run_loop itself is gated by validate_preconditions().
    if (mode_flags_ && wd_result.protective_stop_active) {
        mode_flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    }
}

template <size_t DOF>
void EtherCATController<DOF>::set_drive_limits(std::span<const core::MotionLimit> limits) {
    this->throw_if_running("set_drive_limits");
    if (limits.size() != DOF) {
        throw std::invalid_argument(std::string("[") + logger_tag() +
                                    "] set_drive_limits: " + std::to_string(limits.size()) +
                                    " limits for a controller built for exactly " + std::to_string(DOF) + " axes");
    }
    for (size_t i = 0; i < DOF; ++i) {
        // is_valid() refuses NaN and inverted or non-positive bounds; ±inf positions (an endless
        // axis) and an infinite effort (a csp or csv axis) stay valid. The velocity must be finite:
        // it bounds the csp step and csv checks and the stop's 1 % settle band, so +inf would
        // switch them all off.
        if (!limits[i].is_valid() || !std::isfinite(limits[i].velocity_limit)) {
            throw std::invalid_argument(
                std::string("[") + logger_tag() + "] set_drive_limits: axes[" + std::to_string(i) +
                "] needs lower < upper, a finite velocity limit > 0 and an effort limit > 0 "
                "(got lower " +
                std::to_string(limits[i].lower_limit) + ", upper " + std::to_string(limits[i].upper_limit) +
                ", velocity " + std::to_string(limits[i].velocity_limit) + ", effort " +
                std::to_string(limits[i].effort_limit) + ")");
        }
    }
    std::copy(limits.begin(), limits.end(), drive_limits_.begin());
    drive_limits_declared_ = true;
}

template <size_t DOF>
void EtherCATController<DOF>::set_drive_stop(std::span<const double> decelerations_rad_s2) {
    this->throw_if_running("set_drive_stop");
    if (decelerations_rad_s2.size() != DOF) {
        throw std::invalid_argument(
            std::string("[") + logger_tag() + "] set_drive_stop: " + std::to_string(decelerations_rad_s2.size()) +
            " decelerations for a controller built for exactly " + std::to_string(DOF) + " axes");
    }
    for (size_t i = 0; i < DOF; ++i) {
        if (!std::isfinite(decelerations_rad_s2[i]) || decelerations_rad_s2[i] < 0.0) {
            throw std::invalid_argument(std::string("[") + logger_tag() + "] set_drive_stop: axes[" +
                                        std::to_string(i) + "]'s deceleration must be finite and >= 0 (got " +
                                        std::to_string(decelerations_rad_s2[i]) + ")");
        }
    }
    std::copy(decelerations_rad_s2.begin(), decelerations_rad_s2.end(), stop_deceleration_.begin());
}

template <size_t DOF>
void EtherCATController<DOF>::set_command_stale_timeout_ns(int64_t timeout_ns) {
    this->throw_if_running("set_command_stale_timeout_ns");
    if (timeout_ns <= 0) {
        throw std::invalid_argument(std::string("[") + logger_tag() +
                                    "] set_command_stale_timeout_ns: the budget must be > 0 (got " +
                                    std::to_string(timeout_ns) + ")");
    }
    command_stale_timeout_ns_ = timeout_ns;
}

template <size_t DOF>
void EtherCATController<DOF>::request_ramp_stop() noexcept {
    ramp_stop_requested_.store(true, std::memory_order_release);
}

template <size_t DOF>
void EtherCATController<DOF>::open_command_epoch() noexcept {
    command_epoch_ns_.store(now_ns(), std::memory_order_release);
}

template <size_t DOF>
bool EtherCATController<DOF>::run_ramp_stop_cycle(bool gate_open,
                                                  const std::array<plem::core::JointActual, DOF>& actuals,
                                                  int64_t now_ns) noexcept {
    const auto& servos = ecat_bus_->servo_devices();
    // Running before the request is taken: ramp_stop_active() never reads false
    // between the two (a request landing after the load waits a cycle).
    bool requested = false;
    if (ramp_stop_requested_.load(std::memory_order_acquire)) {
        ramp_stop_running_.store(true, std::memory_order_release);
        requested = ramp_stop_requested_.exchange(false, std::memory_order_acq_rel);
    }
    const bool idle = ramp_stop_state_.phase == RampStopPhase::IDLE;
    bool at_rest = true;
    bool settled = true;
    bool all_closed = true;
    int64_t decelerate_ns = 0;
    for (size_t i = 0; i < DOF; ++i) {
        const auto& servo = *servos[i];
        if (requested && idle) {
            stop_velocity_[i] = applied_velocity_[i];
            if (stop_deceleration_[i] > 0.0) {
                decelerate_ns =
                    std::max(decelerate_ns, ramp_stop_duration_ns(stop_velocity_[i], stop_deceleration_[i]));
            }
        }
        at_rest = at_rest && stop_velocity_[i] == 0.0;
        settled = settled && std::abs(actuals[i].velocity_actual_value) <= 0.01 * drive_limits_[i].velocity_limit;
        all_closed = all_closed && servo.is_initialized() && servo.status_recognized() &&
                     proves_brake_closed(servo.get_servo_state());
    }
    const auto decision = decide_ramp_stop(ramp_stop_state_, RampStopInputs{.requested = requested,
                                                                            .live = gate_open && hold_on_release_,
                                                                            .at_rest = at_rest,
                                                                            .settled = settled,
                                                                            .all_closed = all_closed,
                                                                            .now_ns = now_ns,
                                                                            .decelerate_ns = decelerate_ns});
    if (requested && idle) {
        ramp_stop_started_ns_ = now_ns;
    }
    ramp_stop_state_ = decision.next;
    switch (decision.action) {
        case RampStopAction::NONE:
            break;
        case RampStopAction::QUICK_STOP:
            for (size_t i = 0; i < DOF; ++i) {
                if (servos[i]->get_servo_state() == ServoState::OPERATION_ENABLED) {
                    (void)servos[i]->transition_state(ServoState::QUICK_STOP_ACTIVE);
                }
            }
            {
                auto event = plem::core::monitoring::Event::error(plem::core::monitoring::EventType::RAMP_STOP_TIMEOUT,
                                                                  0, plem::core::monitoring::kJointIdNotApplicable);
                event.value = static_cast<float>(static_cast<double>(now_ns - ramp_stop_started_ns_) * 1e-9);
                PLEM_EMIT(event);
                break;
            }
        case RampStopAction::ENGAGE:
            mode_flags_->motion_enabled.store(false, std::memory_order_release);
            // A drive still in Quick stop active past its time: Disable voltage
            // (transition 12); the engage sequence cannot step out of it.
            for (size_t i = 0; i < DOF; ++i) {
                if (servos[i]->get_servo_state() == ServoState::QUICK_STOP_ACTIVE) {
                    (void)servos[i]->transition_state(ServoState::SWITCH_ON_DISABLED);
                }
            }
            start_engage(all_joints_mask(), /*preserve_active=*/true);
            stop_velocity_ = {};
            break;
    }
    ramp_stop_running_.store(ramp_stop_state_.phase != RampStopPhase::IDLE, std::memory_order_release);
    return !idle || ramp_stop_state_.phase != RampStopPhase::IDLE;
}

template <size_t DOF>
void EtherCATController<DOF>::run_drive_safety_cycle(const std::array<plem::core::JointActual, DOF>& actuals) noexcept {
    // Each axis checked in full (no short circuit): every violating axis is named.
    std::array<LimitStopCause, DOF> causes{};
    bool all_within_limits = true;
    const auto& servos = ecat_bus_->servo_devices();
    for (size_t i = 0; i < DOF; ++i) {
        // An axis whose position does not decode (a bounded axis's 0x6064
        // word outside int32 joint counts, i.e. a saturated value, or an
        // endless axis whose count left its domain and stays invalid) reports one
        // value while the drive moves, so neither the limits nor a csp step can check it:
        // held like a limit violation, latched before any release, reset
        // refused while it stands. Checked from the first real frame on; the
        // zero image before it is no feedback, and release needs that frame anyway.
        const bool undecodable = servos[i]->is_initialized() && !servos[i]->position_word_decodable();
        causes[i] = feedback_stop_cause(actuals[i].position_actual_value, actuals[i].velocity_actual_value, undecodable,
                                        drive_limits_[i]);
        all_within_limits = all_within_limits && causes[i].reason == LimitStopReason::NONE;
    }
    // The only consumer of the operator reset on this assembly. With a
    // higher-layer host, that host's control thread consumes it instead; never
    // both.
    const bool reset_requested = mode_flags_->reset_error.exchange(false, std::memory_order_acq_rel);
    const bool was_internal = mode_flags_->is_internal_protective_stop();
    const auto decision =
        plem::core::evaluate_internal_protective_stop(was_internal, all_within_limits, reset_requested,
                                                      /*persistent_fault=*/false);

    if (decision.set_protective_stop) {
        // Flag only: the PROTECTIVE_STOP event is the brake step's
        // (it sees this rising edge in the same cycle and emits once; the
        // higher-layer host deliberately emits nothing here either).
        mode_flags_->set_protective_stop(plem::core::ProtectiveStopSource::INTERNAL);
        // The cause, where the INTERNAL stop is newly set, once per episode,
        // one per violating axis. A latched stop's persisting violation
        // and a refused reset name nothing new.
        for (size_t i = 0; !was_internal && i < DOF; ++i) {
            if (causes[i].reason != LimitStopReason::NONE) {
                PLEM_EMIT(plem::core::monitoring::Event::limit_stop(static_cast<uint8_t>(i), causes[i].reason,
                                                                    causes[i].value));
            }
        }
    }
    if (decision.clear_protective_stop) {
        // Optimistic clear of both sources (level semantics, the watchdogs'
        // own contract): the freshness watchdog re-asserts EXTERNAL right
        // after (3c), before the brake step and the apply point; the WKC watchdog runs after
        // send(), so under a persisting WKC fault an operator reset opens the
        // torque gate for one cycle: the same ≤1-cycle window a higher-layer host's
        // control thread accepts.
        mode_flags_->clear_protective_stop(plem::core::ProtectiveStopSource::INTERNAL);
        mode_flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
        PLEM_EMIT(plem::core::monitoring::Event::protective_stop_clear());
    } else if (decision.clear_rejected) {
        auto evt = plem::core::monitoring::Event::protective_stop();
        evt.error_code = 1;  // clear refused; a limit violation still stands
        PLEM_EMIT(evt);
    }
}

template <size_t DOF>
void EtherCATController<DOF>::run_command_freshness_cycle(int64_t update_ns, int64_t now_ns) noexcept {
    const auto result = plem::fieldbus::evaluate_command_freshness(command_freshness_state_, update_ns, now_ns,
                                                                   command_stale_timeout_ns_);
    command_freshness_state_ = result.next_state;

    if (result.should_protective_stop) {
        // One diagnostic event carrying the observed staleness age. The live
        // safety response (0x40 PROTECTIVE_STOP + forced brake sequence) is
        // owned by the brake cycle step (decide_control_brake), which runs
        // after this in the same cycle and picks the EXTERNAL latch up there.
        PLEM_EMIT(plem::core::monitoring::Event::command_stale(
            static_cast<float>(static_cast<double>(now_ns - update_ns) * 1e-9)));
    }

    // Level re-assert while stale: an optimistic operator reset only sticks
    // once the writer is stamping again (same contract as the WKC watchdog).
    if (mode_flags_ && result.protective_stop_active) {
        mode_flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    }
}

template <size_t DOF>
void EtherCATController<DOF>::run_drive_command_cycle(const std::array<double, DOF>& command,
                                                      const std::array<double, DOF>& rate, int64_t stamp_ns,
                                                      bool host_stamped, bool gate_open,
                                                      const std::array<plem::core::JointActual, DOF>& actuals,
                                                      int64_t now_ns) noexcept {
    const auto& servos = ecat_bus_->servo_devices();
    const double cycle_s = static_cast<double>(this->config_.cycle_time_us) * 1e-6;
    // The source of this cycle's commands: the stop's own ramp, the host's
    // command, or (released without a command of this release) the hold.
    const bool ramp_stopping = run_ramp_stop_cycle(gate_open, actuals, now_ns);
    const double age_s = static_cast<double>(std::max<int64_t>(0, now_ns - stamp_ns)) * 1e-9;
    const bool hold = hold_on_release_ && !host_stamped;

    // Check every axis first (one snapshot), apply after: a violation on a
    // later axis must not leave an earlier one commanded in this frame.
    std::array<std::optional<CheckedTarget>, DOF> targets{};
    std::array<LimitStopCause, DOF> causes{};
    bool violation = false;
    const bool live = gate_open && (host_stamped || hold || ramp_stopping);
    for (size_t i = 0; live && i < DOF; ++i) {
        const auto& servo = *servos[i];
        switch (servo.get_modes_of_operation()) {
            case OperationMode::CYCLIC_SYNC_POSITION: {
                // Feedback decodability is the safety step's (it latched before this
                // gate could open), so `reference` is a real position here.
                const double reference = csp_applied_rad_[i].value_or(servo.get_actual_position().value());
                if (ramp_stopping) {
                    stop_velocity_[i] = stop_velocity_step(stop_velocity_[i], stop_deceleration_[i], cycle_s);
                }
                // The advance follows the actual wake, the step check the nominal
                // cycle: at high speed a late wake is a step over the limit and a
                // protective stop (CSP_STEP); the drive would get that move in one
                // cycle.
                const double want = ramp_stopping ? reference + stop_velocity_[i] * cycle_s
                                    : hold        ? reference
                                                  : command[i] + rate[i] * age_s;
                targets[i] = servo.encode_position(want);
                causes[i] = targets[i] ? csp_target_cause(targets[i]->si, reference, drive_limits_[i], cycle_s)
                                       : LimitStopCause{LimitStopReason::UNENCODABLE, static_cast<float>(want)};
                break;
            }
            case OperationMode::CYCLIC_SYNC_VELOCITY:
                if (ramp_stopping) {
                    stop_velocity_[i] = stop_velocity_step(stop_velocity_[i], stop_deceleration_[i], cycle_s);
                }
                targets[i] = servo.encode_velocity(ramp_stopping ? stop_velocity_[i] : hold ? 0.0 : command[i]);
                causes[i] = targets[i] ? csv_target_cause(targets[i]->si, drive_limits_[i])
                                       : LimitStopCause{LimitStopReason::UNENCODABLE, static_cast<float>(command[i])};
                break;
            default:  // CST: clamped at set_command (the command-limit protective stop covers CSP/CSV only)
                break;
        }
        violation = violation || causes[i].reason != LimitStopReason::NONE;
    }
    if (violation) {
        // Latch; the brake step picks the level up next cycle,
        // and the one-shot trip survives a reset consumed before it. The gate
        // was open, so this is always a new stop: every violating axis named.
        mode_flags_->set_protective_stop(plem::core::ProtectiveStopSource::INTERNAL);
        in_service_trip_pending_ = true;
        for (size_t i = 0; i < DOF; ++i) {
            if (causes[i].reason != LimitStopReason::NONE) {
                PLEM_EMIT(plem::core::monitoring::Event::limit_stop(static_cast<uint8_t>(i), causes[i].reason,
                                                                    causes[i].value));
            }
        }
    }

    const bool apply = live && !violation;
    for (size_t i = 0; i < DOF; ++i) {
        auto& servo = *servos[i];
        if (!servo.is_initialized()) {
            continue;
        }
        switch (servo.get_modes_of_operation()) {
            case OperationMode::CYCLIC_SYNC_POSITION:
                // Not applied: read() already put the raw mirror on 0x607A.
                if (apply) {
                    servo.set_target_position(*targets[i]);
                    applied_velocity_[i] =
                        csp_applied_rad_[i] ? (targets[i]->si - *csp_applied_rad_[i]) / cycle_s : 0.0;
                    csp_applied_rad_[i] = targets[i]->si;
                    csp_sent_target_[i] = targets[i];
                } else {
                    // The violating frame keeps this axis's previous target:
                    // the brake step ran before the check, so the stop
                    // controlword goes out only next frame, and the raw mirror
                    // under Enable Operation would step the target back by the
                    // following lag (a drive can trip on that: the A6B
                    // example drive raises alarm Err27).
                    if (violation && csp_sent_target_[i]) {
                        servo.set_target_position(*csp_sent_target_[i]);
                    }
                    csp_sent_target_[i].reset();
                    applied_velocity_[i] = 0.0;
                    // An endless axis's hold is its last observed word, which
                    // can lag the feedback across incomplete frames: its next
                    // application is checked from that transmitted word. A
                    // bounded axis's hold is each frame's own word, so its
                    // next application is checked from that frame's actual.
                    if (servo.position_endless()) {
                        csp_applied_rad_[i] = servo.applied_command();
                    } else {
                        csp_applied_rad_[i].reset();
                    }
                }
                break;
            case OperationMode::CYCLIC_SYNC_VELOCITY:
                servo.set_target_velocity_word(apply ? targets[i]->word : 0);
                applied_velocity_[i] = apply ? targets[i]->si : 0.0;
                break;
            default:
                // CST, primed from the brake-release step on (may_hold_brake_open).
                if (may_hold_brake_open(servo.get_servo_state())) {
                    servo.set_command(plem::core::units::NewtonMeter(apply ? command[i] : 0.0));
                }
                break;
        }
    }
}

template <size_t DOF>
core::Result<void> EtherCATController<DOF>::validate_preconditions() const {
    if (!ecat_bus_) {
        return core::Result<void>::error(std::string("[") + logger_tag() + "] bus not configured");
    }
    if (!mode_flags_) {
        return core::Result<void>::error(std::string("[") + logger_tag() +
                                         "] mode flags not configured (they carry the protective-stop and "
                                         "torque-behavior state)");
    }
    // Every existence check first, then consistency. A drive-count mismatch
    // reported to a caller who has not wired the safety surface or the
    // shared data buries the more fundamental defect.
    if (!shared_data_) {
        return core::Result<void>::error(std::string("[") + logger_tag() +
                                         "] no SharedJointData set — the RT loop exchanges joint data with the host "
                                         "through it; call set_shared_data() before start()");
    }

    // DOF is this controller's exact axis count, not an upper bound, and the
    // loop indexes its std::array<…, DOF> buffers with the bus's drive count.
    // Too many drives writes past every one of those buffers; too few can
    // never run: execute_state_transitions() holds all_drives_enabled false
    // whenever the bus is short, so the torque gate stays shut forever and
    // the missing axes' brake sequences never settle. Both are start-time
    // facts, so refuse here rather than run a robot that cannot work.
    // The assembly layer checks the same count on its own path; this gate
    // covers every other way a
    // controller reaches start(), which all route through here before the
    // thread exists.
    //
    // The count is the bus's registered drives, which initialize() materializes
    // from the servo config, so the fix is a matching model, not a late
    // set_servo_config() on an already-initialized bus.
    if (const size_t servo_count = ecat_bus_->servo_count(); servo_count != DOF) {
        return core::Result<void>::error(std::string("[") + logger_tag() + "] bus has " + std::to_string(servo_count) +
                                         " registered drives but this controller is built for exactly " +
                                         std::to_string(DOF) + " axes. Build the bus from a model declaring " +
                                         std::to_string(DOF) +
                                         " axes and initialize() it before starting this controller.");
    }
    // The bus derives its send interval, fault-reset pulse length and IO device
    // period from its own period; a loop at another period runs them all off.
    const uint64_t loop_ns = static_cast<uint64_t>(this->config_.cycle_time_us) * 1000U;
    if (const uint64_t bus_ns = ecat_bus_->cycle_time_ns(); bus_ns != loop_ns) {
        return core::Result<void>::error(std::string("[") + logger_tag() + "] the bus runs every " +
                                         std::to_string(bus_ns) + " ns but this controller's loop every " +
                                         std::to_string(loop_ns) + " ns. Call set_cycle_time_ns(" +
                                         std::to_string(loop_ns) + ") on the bus before initialize().");
    }
    // A stop not confirmed in time commands Quick stop (transition 11) to every
    // enabled drive (run_ramp_stop_cycle); a csp or csv axis must be able to take it.
    const auto& servos = ecat_bus_->servo_devices();
    for (size_t i = 0; i < DOF; ++i) {
        // Without declared limits the loop applies no csp or csv command (step 6 primes
        // cst torque only) and request_ramp_stop() has nothing to ramp: such an axis
        // would start and silently do nothing.
        if (!drive_limits_declared_ && servos[i]->get_modes_of_operation() != OperationMode::CYCLIC_SYNC_TORQUE) {
            return core::Result<void>::error(std::string("[") + logger_tag() + "] axes[" + std::to_string(i) +
                                             "] is a csp or csv axis but no drive limits are declared; call "
                                             "set_drive_limits() (and set_drive_stop()) before start() — without "
                                             "them the loop applies cst torque only");
        }
        // A csp or csv stop ramps at this deceleration; at 0 it never slows and always ends in Quick stop.
        if (drive_limits_declared_ && servos[i]->get_modes_of_operation() != OperationMode::CYCLIC_SYNC_TORQUE &&
            !(stop_deceleration_[i] > 0.0)) {
            return core::Result<void>::error(std::string("[") + logger_tag() + "] axes[" + std::to_string(i) +
                                             "] is a csp or csv axis with drive limits but no stop deceleration; "
                                             "call set_drive_stop() with a value > 0 for it before start()");
        }
        if (servos[i]->get_modes_of_operation() != OperationMode::CYCLIC_SYNC_TORQUE &&
            !fieldbus::detail::declares_cia402_transition(servos[i]->cia402_profile(), ServoState::OPERATION_ENABLED,
                                                          ServoState::QUICK_STOP_ACTIVE)) {
            return core::Result<void>::error(
                std::string("[") + logger_tag() + "] axes[" + std::to_string(i) +
                "]'s CiA 402 profile has no operation_enabled → quick_stop_active row (transition 11, Quick stop) "
                "— a csp or csv axis's stop falls back to it when the ramp is not confirmed in time; declare the "
                "row (controlword 0x0002) or use the built-in table");
        }
    }
    return core::Result<void>::ok();
}

template <size_t DOF>
void EtherCATController<DOF>::set_mode_flags(std::shared_ptr<plem::core::ControlModeFlags> flags) {
    this->throw_if_running("set_mode_flags");
    mode_flags_ = std::move(flags);
}

template <size_t DOF>
const char* EtherCATController<DOF>::logger_tag() const {
    return "EtherCATController";
}

template <size_t DOF>
void EtherCATController<DOF>::run_loop() {
    RtCycleMonitor<DOF> monitor{this->config_.cycle_time_us};
    // Allocate the timer before entering RT scheduling (no malloc at SCHED_FIFO priority)
    this->timer_ = std::make_unique<plem::core::PeriodicTimerRT>(this->config_.cycle_time_us);

    // Every loop start is a fresh bus bring-up, so the WKC watchdog re-enters
    // unarmed (boot-arming gate), even though the deployed lifecycle constructs
    // this controller once per process.
    watchdog_state_ = {};
    command_freshness_state_ = {};  // reset the freshness bring-up gate too
    csp_applied_rad_ = {};          // a new run's first CSP application is checked against the actual
    csp_sent_target_ = {};
    applied_velocity_ = {};
    stop_velocity_ = {};
    ramp_stop_state_ = {};  // a stop belongs to the run it was asked of
    ramp_stop_requested_.store(false, std::memory_order_release);
    ramp_stop_running_.store(false, std::memory_order_release);
    hold_on_release_ =
        std::none_of(ecat_bus_->servo_devices().begin(), ecat_bus_->servo_devices().end(), [](const auto& servo) {
            return servo->get_modes_of_operation() == plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
        });

    // Revoke any stale drive-enablement grant before the first cycle: a host
    // control thread's lifecycle is independent of this loop's:
    // on a stop()/start() cycle it may already be running against a `true`
    // left over from the previous life, and its first cycle runs immediately
    // while this loop still waits out its first timer period. The matching
    // revoke at loop exit (below) covers every termination path (stop(),
    // loop break) for a control thread that keeps running.
    mode_flags_->all_drives_enabled.store(false, std::memory_order_release);

    // Log the communication plane before RT escalation: a logger call may
    // allocate and takes the queue mutex (logger.hpp), so it stays out of the
    // RT context; the cycle reports through PLEM_EMIT.
    logger.info("[EtherCATController] Using SharedJointData for RT joint data");

    // DC hoist: the knob is fixed before initialize()
    // and immutable while the loop runs; hoisting keeps the disabled hot path
    // free of the two per-cycle bus calls entirely.
    const bool dc_active = ecat_bus_->dc_enabled();

    // Apply RT scheduling and CPU affinity from within the thread before entering the control loop
    // Each logs its own failure; the loop runs either way.
    this->apply_rt_scheduling();
    this->apply_cpu_affinity();

    // Prefault stack to prevent page faults during RT operation
    plem::core::threading::prefault_stack();

    // Set per-instance monitoring context for this thread (routes PLEM_EMIT to correct queues)
    if (this->monitoring_context_) {
        plem::core::monitoring::set_thread_context(this->monitoring_context_);
    }

    // Initialize page fault monitoring (baseline captured after prefaulting)
    plem::core::threading::PageFaultMonitor page_fault_monitor;

    // Run-scoped by contract; see JointDataPlane's lifetime note.
    JointDataPlane<DOF> joint_data{this->shared_data_};

    int64_t t_wake = 0;

    // Fault-reset cutoff: the run boundary, taken before the first request
    // can be acquired. A request posted while this controller was stopped
    // describes an axis state minutes old; letting it survive would fire it
    // against this run's first recognized FAULT. Doing it here rather than at
    // loop exit is what makes the contract hold: a request can arrive after
    // any tail runs.
    //
    // The pulse state is deliberately not cleared; it describes the wire, not
    // a request. If the last frame of the previous run was high, the first
    // write of this run still owes the falling edge; it is folded to exactly
    // that edge, so the previous run's hold does not resume.
    for (const auto& servo : ecat_bus_->servo_devices()) {
        servo->discard_fault_reset_request();
        servo->collapse_fault_reset_pulse();
    }

    while (this->running_.load(std::memory_order_acquire)) {
        // 1. Wait for periodic timer (wake edge). The wait sits at the loop top
        // so every wake-adjacent stamp below (the DC application time and the
        // phase-lock epoch) lies on the timer grid from the very first cycle;
        // a consumer that latches the first nonzero epoch would otherwise bake
        // a permanent phase error into its alignment.
        this->timer_->wait_period();

        t_wake = static_cast<int64_t>(now_ns());
        ecat_bus_->set_application_time(static_cast<uint64_t>(t_wake));
        // Phase-lock epoch channel: publish this cycle's wake
        // so the control thread can one-shot align its timer to bus wake + offset.
        // Independent scalar timestamp; relaxed is sufficient (nothing is published
        // through it). Cost: one lock-free relaxed store (no
        // syscall/blocking/allocation): small, not zero; count it in RT budget audits.
        this->shared_data_->last_cycle_ns.store(t_wake, std::memory_order_relaxed);

        // 2. Receive from EtherCAT master
        ecat_bus_->receive();
        // Domain state of the frame just received (ecrt_domain_process ran in
        // receive(); send() does not change it). Queried once, reused below by
        // the watchdog and RT monitoring.
        const auto domain_state = ecat_bus_->get_domain_state();
        process_data_complete_.store(domain_state.wkc_ok, std::memory_order_relaxed);

        // 2b. DC drift monitor: process the datagram queued last cycle (raw;
        // UINT32_MAX = monitor unavailable this cycle; the publisher fold
        // skips sentinels instead of max()ing them).
        uint32_t dc_sync_monitor_ns = 0;
        if (dc_active) {
            dc_sync_monitor_ns = ecat_bus_->dc_sync_monitor_process();
        }

        // IO devices: the frame just received and its working counter,
        // before read(), before write() replaces what it carried.
        for (const auto& io_device : ecat_bus_->io_devices()) {
            io_device->note_received_frame(domain_state.wkc_ok);
        }

        // Read PDO data from all servos (an endless axis observes its
        // position only on a complete frame)
        for (const auto& servo : ecat_bus_->servo_devices()) {
            servo->note_received_frame(domain_state.wkc_ok);
            servo->read(ecat_bus_->domain_pd());
        }

        // Read PDO data from all IO devices.
        for (const auto& io_device : ecat_bus_->io_devices()) {
            io_device->read(ecat_bus_->domain_pd());
        }

        // 3. Write joint actual values (to shared_data)
        const auto& servos = ecat_bus_->servo_devices();
        joint_data.publish_actuals(servos);

        // 3b. Drive-layer safety step (declared drive limits only; see
        // set_drive_limits). Before the brake step: an INTERNAL protective stop raised
        // here is the rising edge the brake step force-engages on this cycle.
        if (drive_limits_declared_) {
            run_drive_safety_cycle(joint_data.actuals());
        }

        // 3c. Read the host commands and check their freshness. Before the
        // brake step: a stale stream's EXTERNAL protective stop is a rising edge the
        // brake step force-engages on this cycle (when no other stop source is
        // already active), so the stop controlword (Disable operation) goes
        // out in the same frame as the closed gate's neutral targets. One frame of
        // Enable Operation with a CSP target stepped back to the measured
        // position is a backward command step a drive can trip on (the A6B
        // example drive raises alarm Err27).
        // After 3b: its optimistic reset clears first, the re-assert below
        // wins while the stream is still stale. A torn seqlock read keeps the
        // previous stamp; sustained read failure ages out the same way a hung
        // writer does. t_wake is this cycle's CLOCK_MONOTONIC wake time (same
        // domain as the writer's stamp).
        const uint8_t command_zoh = joint_data.read_command() ? 1 : 0;
        int64_t watched_stamp = joint_data.last_command_update_ns();
        if (drive_limits_declared_ &&
            command_watch_unarmed(watched_stamp, mode_flags_->motion_enabled.load(std::memory_order_acquire))) {
            command_freshness_state_ = {};  // unarmed: re-arms on this release's first command
            watched_stamp = 0;
        }
        run_command_freshness_cycle(watched_stamp, t_wake);

        // 4. Brake step (includes protective stop handling). One step per
        // cycle: safety-trigger decision -> apply -> request-mask consume ->
        // request decision -> apply. The decisions are pure (brake_logic.hpp);
        // the observable ordering (two stop samples with the exchange between
        // them) is fixed by run_brake_cycle_step.
        // The host's enable as of now, before the brake step consumes its
        // requests: DriveHost::disable posts the engage before it clears this,
        // so a cleared flag seen here has its engage consumed below, in the
        // same frame the gate closes. Read at the gate instead, a clear landing
        // after the brake step would close the gate a frame before the stop.
        const bool motion_enabled = mode_flags_->motion_enabled.load(std::memory_order_acquire);
        step_brake_requests(ecat_bus_->motion_possible());

        // 5. Execute state transitions
        execute_state_transitions();

        // 6. Read target positions for monitoring (the commands read in 3c
        //    are applied by the drive command step below)
        joint_data.read_target_values();

        // Drive-layer torque gate (drive_safety.hpp): live flags of this cycle
        // (all_drives_enabled was just published by execute_state_transitions).
        // Undeclared limits short-circuit to constant open with no flag loads:
        // the host's own control thread gates what it writes.
        if (drive_limits_declared_) {
            const int64_t stamp = joint_data.last_command_update_ns();
            run_drive_command_cycle(
                joint_data.command(), joint_data.command_rate(), stamp,
                stamp != 0 && (!hold_on_release_ || !predates_release(stamp)),
                drive_torque_allowed(motion_enabled, mode_flags_->all_drives_enabled.load(std::memory_order_acquire),
                                     mode_flags_->is_protective_stop_active()),
                joint_data.actuals(), t_wake);
        } else {
            for (size_t i = 0; i < DOF; ++i) {
                if (servos[i]->is_initialized()) {
                    // Torque is primed from the brake-release step onward, so a
                    // brake that opens meets a compensating command
                    // (may_hold_brake_open; brake_logic.hpp owns the step and
                    // its vendor caveat).
                    if (may_hold_brake_open(servos[i]->get_servo_state())) {
                        servos[i]->set_command(plem::core::units::NewtonMeter(joint_data.command()[i]));
                    }
                }
            }
        }

        // 7. Write PDO data and send
        for (const auto& servo : servos) {
            servo->write(ecat_bus_->domain_pd());
        }
        report_fault_reset_publications();

        // Write IO device PDO data.
        for (const auto& io_device : ecat_bus_->io_devices()) {
            io_device->write(ecat_bus_->domain_pd());
        }

        // DC sync triple (ref-clock push + slave drift compensation + monitor
        // queue) rides immediately before send() so its datagrams join this
        // cycle's frame (IgH dc_user pattern). +3 datagrams while DC is active.
        if (dc_active) {
            ecat_bus_->sync_distributed_clocks();
        }

        ecat_bus_->send();

        // Watchdog: WKC check and protective stop
        run_watchdog_cycle(domain_state.wkc_ok);

        // The loop stops the cycle clock itself; the monitor only reports the
        // duration it is handed.
        const int64_t t_end = now_ns();

        // 8. RT monitoring: project this cycle onto the sample + event streams
        // (domain_state reused from the watchdog query above).
        monitor.emit(
            typename RtCycleMonitor<DOF>::CycleObservation{.wake_ns = t_wake,
                                                           .exec_end_ns = t_end,
                                                           .wkc = domain_state.wkc,
                                                           .wkc_ok = domain_state.wkc_ok,
                                                           .wc_state = domain_state.wc_state,
                                                           .wkc_mismatch_count = watchdog_state_.mismatch_count,
                                                           .command_zoh = command_zoh,
                                                           .dc_sync_monitor_ns = dc_sync_monitor_ns},
            servos, joint_data);
    }

    // Loop over: nothing recomputes drive enablement from fresh statuswords
    // after this, so the grant must not outlive its evidence (a host control
    // thread that keeps running after stop() would otherwise
    // integrate against a frozen `true` indefinitely).
    mode_flags_->all_drives_enabled.store(false, std::memory_order_release);

    // Report page faults during RT operation
    auto pf = page_fault_monitor.get_delta();
    if (pf.major > 0) {
        logger.error(
            "[EtherCATController] {} major page faults in this process during the RT loop's run (process-wide "
            "count; it does not identify which thread faulted)",
            pf.major);
    }
    if (pf.minor > 0) {
        logger.debug("[EtherCATController] Page faults during RT operation: minor={}, major={}", pf.minor, pf.major);
    }
}

// State Transition Management

template <size_t DOF>
void EtherCATController<DOF>::step_brake_requests(core::TriState motion_possible) {
    BrakeRequestMaskRefs masks{brake_release_request_mask_, brake_engage_request_mask_, brake_operation_busy_mask_};
    brake_logic_state_ = run_brake_cycle_step(
        BrakeCycleInputs{.motion_possible = motion_possible,
                         .release_admission_closed = release_admission_closed_.load(std::memory_order_acquire),
                         .mode_trip = std::exchange(in_service_trip_pending_, false)},
        *mode_flags_, [this]() noexcept { return sample_committed_mode(); },
        [this](uint8_t consumed_release) noexcept {
            // Nothing consumed: readiness is not read (the decision checks it only
            // for a consumed release), so no mask is built from stale states.
            if (consumed_release == 0) {
                return uint8_t{0};
            }
            // The slave states this request is checked on: sampled for it, after
            // the consume; a post landing mid-step is checked on a fresh sample too.
            ecat_bus_->sample_servo_slave_states(slave_states_);
            return release_ready_mask();
        },
        masks, brake_logic_state_, all_joints_mask(),
        [this](const BrakeEffect& effect) { apply_brake_effect(effect); });
}

template <size_t DOF>
void EtherCATController<DOF>::refresh_statusword_mirror() noexcept {
    const auto& servos = ecat_bus_->servo_devices();
    for (size_t i = 0; i < servos.size() && i < DOF; ++i) {
        statusword_mirror_[i].store(servos[i]->get_statusword(), std::memory_order_relaxed);
    }
}

template <size_t DOF>
void EtherCATController<DOF>::execute_state_transitions() {
    refresh_statusword_mirror();

    const auto& servo_devices = ecat_bus_->servo_devices();

    for (size_t i = 0; i < servo_devices.size() && i < DOF; ++i) {
        auto& seq = servo_sequences_[i];
        if (seq.size == 0) {
            continue;
        }

        auto& servo = servo_devices[i];
        const auto target_state = seq.states[seq.head];

        // ① Sample: this cycle's PDO read as values. Scope notes and
        // counter semantics are in sequence_step.hpp; the eager
        // can_transition probe runs only for joints with an active sequence
        // (transient enable/disable windows, ≤ kMaxTransitions(16) profile
        // rows per probe, not the steady-state hot path).
        SequenceStepInputs in;
        in.initialized = servo->is_initialized();
        in.at_target = servo->get_servo_state() == target_state;
        in.is_faulted = servo->is_faulted();
        in.can_transition = servo->can_transition_to(target_state);
        in.engage_settle_proven = servo->status_recognized() && proves_brake_closed(servo->get_servo_state());
        in.release_target = seq.release_target;
        in.policy = fault_reset_policy_;
        in.state = {sequence_stall_cycles_[i], sequence_stall_reported_[i], fault_reset_cycles_[i],
                    fault_reset_attempts_[i]};

        // ② Decide: pure. The branch priority is table-pinned by
        // test_sequence_step.cpp; the cycle-exact wire behavior stays pinned
        // by the golden traces.
        const SequenceStepDecision d = decide_sequence_step(in, sequence_stall_threshold_cycles_,
                                                            fault_reset_interval_cycles_, fault_reset_max_attempts_);
        SequenceStepState next = d.next;

        // ③ Effects. All diagnostics are PLEM_EMIT, not logger (1 kHz hot
        // loop: wait-free events only; the publisher renders them
        // off-thread). error_code = the axis's statusword.
        //
        // Stall report first: it coexists with any action, and the same
        // cycle's reset diagnostic must land after it (characterization pin
        // StallReportOutrunsFaultResetInTheSameCycle).
        if (d.emit_stall) {
            // Silent-stall observability. One event per episode; it asserts
            // only "no step progress for the window", not a cause (missing
            // profile row, unresponsive drive and fault-reset exhaustion all
            // land here). A never-initialized axis reports statusword 0
            // and is reported here too.
            auto event = plem::core::monitoring::Event::error(plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED,
                                                              servo->get_statusword(), static_cast<uint8_t>(i));
            event.value = static_cast<float>(static_cast<int>(target_state));
            PLEM_EMIT(event);
        }

        switch (d.action) {
            case SequenceStepAction::ADVANCE: {
                // Current step completed for this servo; advance independently.
                ++seq.head;
                --seq.size;
                if (seq.size == 0) {
                    // Group-atomic settlement: the flip helpers own
                    // the shared-brake physics (a completed sequence is a
                    // physical-state confirmation on power-coupled hardware)
                    // and re-check the partners' live state at this effect
                    // point, never the decision's per-joint sample.
                    const uint8_t bit = static_cast<uint8_t>(uint8_t{1} << i);
                    if (seq.release_target) {
                        settle_group_released(bit);
                    } else {
                        settle_group_engaged(bit);
                    }
                }
                break;
            }
            case SequenceStepAction::SETTLE_ENGAGED_EARLY: {
                // Conservative engage fallback (decided before the
                // transition command): a drive already resting in a
                // proven-unpowered step is done; stepping it up toward the
                // disable sequence's first step would power a voltage stage
                // during an engage. The recognized gate on the consume
                // limits this action only: an unrecognized sample
                // skips the early settle and the decision falls through to
                // can_transition, which may still command. A persistent
                // unknown lands in BRAKE_SEQUENCE_STALLED.
                seq.size = 0;
                // Same group-atomic rule as the completion path above.
                const uint8_t bit = static_cast<uint8_t>(uint8_t{1} << i);
                settle_group_engaged(bit);
                break;
            }
            case SequenceStepAction::TRY_FAULT_RESET: {
                // Two-phase fold: the attempt counts only if the device
                // accepted the request (recognized FAULT). Acceptance is not
                // publication; a later write() decides that, and may instead
                // hold the request one frame (a falling edge is owed) or drop
                // it (that frame no longer decodes to FAULT).
                // SERVO_FAULT_RESET_RESULT reports the terminal outcomes only,
                // published or dropped; a held request reports nothing until
                // the frame that resolves it. Neither the wire nor the drive is
                // confirmed anywhere. value = attempt #.
                const bool accepted = servo->reset_fault();
                next = commit_fault_reset(next, accepted);
                if (accepted) {
                    auto event =
                        plem::core::monitoring::Event::error(plem::core::monitoring::EventType::SERVO_FAULT_RESET,
                                                             servo->get_statusword(), static_cast<uint8_t>(i));
                    event.value = static_cast<float>(next.reset_attempts);
                    PLEM_EMIT(event);
                }
                break;
            }
            case SequenceStepAction::GIVE_UP_FAULT_RESET: {
                // Bounded (the model's max_attempts): unbounded resets
                // re-enable the drive into the same fault, so the sequence
                // stops requesting. The axis is left in whatever state it
                // reports, which need not be a settled FAULT (the cap can be
                // crossed on FRA or a held unrecognized word).
                // One-shot (decision sentinel).
                auto event =
                    plem::core::monitoring::Event::error(plem::core::monitoring::EventType::SERVO_FAULT_RESET_EXHAUSTED,
                                                         servo->get_statusword(), static_cast<uint8_t>(i));
                event.value = static_cast<float>(fault_reset_max_attempts_);
                PLEM_EMIT(event);
                break;
            }
            case SequenceStepAction::COMMAND_TRANSITION:
                // transition_state failure is a per-cycle retry signal, not a
                // fault. The actual servo fault (if any) is reported once on
                // rising edge by RtEventEmitter (see rt_event_emitter.hpp).
                // Emitting here as well would produce a 1 kHz SERVO_FAULT
                // burst whenever a joint stays in FAULT/FAULT_REACTION_ACTIVE
                // while a release sequence keeps re-attempting.
                (void)servo->transition_state(target_state);
                break;
            case SequenceStepAction::HOLD:
                break;
        }

        // ④ Counter round-trip: the member arrays stay the single source of
        // truth; `next` was this cycle's working copy.
        sequence_stall_cycles_[i] = next.stall_cycles;
        sequence_stall_reported_[i] = next.stall_reported;
        fault_reset_cycles_[i] = next.reset_cycles;
        fault_reset_attempts_[i] = next.reset_attempts;
    }

    // Settled-mirror: one release store per cycle so non-RT observers
    // (e.g. a caller sequencing per-joint brake operations) can distinguish "these confirmed bits
    // are settled terminals" from "an operation is in flight and the bits may
    // be stale". Covers the active CiA 402 state sequences.
    uint8_t brake_busy = 0;
    for (size_t i = 0; i < DOF; ++i) {
        if (servo_sequences_[i].size != 0) {
            brake_busy |= static_cast<uint8_t>(uint8_t{1} << i);
        }
    }
    brake_operation_busy_mask_.store(brake_busy, std::memory_order_release);

    // Torque-gate publication: a host control thread commands torque only while
    // motion_enabled and all_drives_enabled hold (control_mode_flags.hpp).
    // The condition is drive enablement: every
    // axis initialized and in OPERATION_ENABLED, i.e. torque commands are
    // physically realizable on every joint, not the brake mask: a shared
    // brake settles its whole group released the moment its first axis
    // enables, so the mask reaches all-released while partner drives are
    // still walking (a mask gate would reopen the torque wind-up window).
    // Recomputed from the statuswords every cycle
    // at this single site, so no settle/fallback path can drift it.
    //
    // status_recognized() is load-bearing: get_servo_state()
    // HOLDS the previous state on an unrecognized word, and a held
    // OPERATION_ENABLED is not evidence; an undecoded fault word would hide
    // in exactly that hold. An unrecognized cycle closes the gate
    // (one zero-torque cycle), and a recognized word reopens it.
    //
    // The declared-mode term joins the same check: a drive is only
    // "enabled" for torque when it is also accepted in its declared mode of
    // operation (0x6061 display == declaration). An enabled drive that is not
    // accepted is the in-service mode trip, fault class: the protective_stop flag
    // (EXTERNAL source) is asserted every cycle the condition holds (a level: an optimistic reset of it sticks
    // only once no enabled axis is off its mode; a drive that left
    // OPERATION_ENABLED while still off its mode is no longer a trip, and
    // the flag is not cleared here), the axis is named
    // once per episode (MODES_OF_OPERATION_MISMATCH), and the next brake step
    // force-engages every brake (BrakeCycleInputs::mode_trip), even under an
    // already-active protective stop, where no rising edge is left to fire. Every axis
    // is checked (no early exit): a later axis's trip must not hide behind an
    // earlier axis merely being off its step.
    //
    // A second cause joins the same
    // episode: a released axis leaving OPERATION_ENABLED outside a walking
    // sequence (OPERATION_ENABLED_LOST). It treats such an axis as unable to
    // hold torque while its group is open (PLEM's contract for the brake
    // couplings it supports). The trip raises the stop surfaces and starts the
    // forced engage; whether that engage completes depends on the fault-reset
    // policy and the drive (a FAULT under NONE, or a QUICK_STOP_ACTIVE, HOLDs
    // and stalls), and neither pending resolution nor an optimistic protective stop
    // clear is a closure barrier.
    bool all_enabled = servo_devices.size() >= DOF;  // a missing axis can never count as enabled
    uint8_t onsets = 0;
    const uint8_t brake_mask = brake_state_mask_.load(std::memory_order_acquire);  // bit set = engaged
    for (size_t i = 0; i < servo_devices.size() && i < DOF; ++i) {
        const auto& servo = servo_devices[i];
        const uint8_t bit = static_cast<uint8_t>(uint8_t{1} << i);
        const bool accepted = in_declared_mode(*servo);
        if (accepted) {
            mode_seen_ |= bit;  // first-match record; see release_admissible()
        }
        const bool enabled = servo->is_initialized() && servo->status_recognized() &&
                             servo->get_servo_state() == ServoState::OPERATION_ENABLED;
        all_enabled = all_enabled && enabled && accepted;

        // Two causes, one episode (mutually exclusive on one sample):
        //  - mode trip: an enabled axis off its declared mode;
        //  - enable loss: an axis whose
        //    last sequence was a release and has completed, whose group
        //    still reads released, seen on a recognized word outside
        //    OPERATION_ENABLED. `release_target` outlives the sequence
        //    (set by the release load, cleared by the engage load), so a
        //    normal engage that settled early while its shared-group
        //    partner is still powered is not a loss. A
        //    walking sequence is never checked (its stall window names a
        //    partner that never enables, not this trip), and an
        //    unrecognized word is neither a loss nor a recovery.
        const auto& seq = servo_sequences_[i];
        const bool walking = seq.size != 0;
        const bool engage_walking = walking && !seq.release_target;
        const bool released_for_service = (brake_mask & bit) == 0 && !walking && seq.release_target;
        const bool enable_lost = released_for_service && servo->is_initialized() && servo->status_recognized() &&
                                 servo->get_servo_state() != ServoState::OPERATION_ENABLED;
        const bool mode_bad = enabled && !accepted;
        const bool tripped = mode_bad || enable_lost;
        if (tripped) {
            mode_flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
        }
        // Episode record: re-armed only by an accepted sample on an axis
        // with no engage still walking and no enable loss standing (a
        // display that flaps while the drive ignores its disable must not
        // re-trigger the forced engage every cycle; a lost axis keeps its
        // accepted display, so acceptance alone cannot close the episode);
        // a new onset reports the axis by its cause and arms the trigger.
        if (accepted && !engage_walking && !enable_lost) {
            in_service_trip_reported_ &= static_cast<uint8_t>(~bit);
        } else if (tripped && (in_service_trip_reported_ & bit) == 0) {
            in_service_trip_reported_ |= bit;
            onsets |= bit;
            if (mode_bad) {
                auto event =
                    plem::core::monitoring::Event::error(plem::core::monitoring::EventType::MODES_OF_OPERATION_MISMATCH,
                                                         servo->get_statusword(), static_cast<uint8_t>(i));
                event.value = static_cast<float>(servo->get_modes_of_operation_display());
                PLEM_EMIT(event);
            } else {
                PLEM_EMIT(
                    plem::core::monitoring::Event::error(plem::core::monitoring::EventType::OPERATION_ENABLED_LOST,
                                                         servo->get_statusword(), static_cast<uint8_t>(i)));
            }
        }
    }
    if (onsets != 0) {
        in_service_trip_pending_ = true;
    }
    mode_flags_->all_drives_enabled.store(all_enabled, std::memory_order_release);
}

template <size_t DOF>
void EtherCATController<DOF>::set_brake_release_sequence(uint8_t mask) noexcept {
    mask &= all_joints_mask();
    if (mask == 0x00) {
        return;
    }
    // Whole-group physics: a shared brake serves every joint of its group:
    // a release touching any joint powers the whole group, or the
    // gravity-loaded partner opens unpowered. Identity under singleton
    // groups.
    mask = brake_group_closure(mask);
    // Settled-mirror pre-publish: the consume publishes only the requested
    // bits, not their group closure, and a safety-forced engage arrives with
    // no request at all; every op must read busy from the moment it starts;
    // the cycle-end recompute only takes over once the sequence state
    // reflects it.
    brake_operation_busy_mask_.fetch_or(mask, std::memory_order_release);

    // The CiA 402 enable sequence is the release (native brake coupling).
    load_release_sequence(mask);

    PLEM_EMIT(plem::core::monitoring::Event::brake_release());
}

template <size_t DOF>
void EtherCATController<DOF>::load_release_sequence(uint8_t mask) noexcept {
    // Per-servo startup sequence: READY_TO_SWITCH_ON -> SWITCHED_ON -> OPERATION_ENABLED.
    // The brake follows drive power (brake_logic.hpp's release-step
    // assumption), so this sequence is the release (the same sequence serves
    // the per-joint-brake fallback path).
    for (size_t i = 0; i < DOF; ++i) {
        if ((mask & (uint8_t{1} << i)) == 0) {
            continue;
        }

        auto& seq = servo_sequences_[i];
        seq.head = 0;
        seq.size = 3;
        seq.release_target = true;
        seq.states[0] = ServoState::READY_TO_SWITCH_ON;
        seq.states[1] = ServoState::SWITCHED_ON;
        seq.states[2] = ServoState::OPERATION_ENABLED;
        sequence_stall_cycles_[i] = 0;  // fresh episode for the stall window
        sequence_stall_reported_[i] = false;
    }
}

template <size_t DOF>
void EtherCATController<DOF>::start_engage(uint8_t mask, bool preserve_active) noexcept {
    mask &= all_joints_mask();
    if (mask == 0x00) {
        return;
    }
    // Whole-group physics, engage direction: disabling one axis of a group
    // leaves the brake held open by its partner, so that joint ends unpowered
    // and unbraked. Identity under singleton groups. The closure is computed
    // before any preservation: an axis is only skipped at its own load, so a
    // preserved axis's partner is still loaded and the busy/brake-notice/event
    // contracts below see the whole group.
    mask = brake_group_closure(mask);
    // Settled-mirror pre-publish: same contract as the release entry above.
    brake_operation_busy_mask_.fetch_or(mask, std::memory_order_release);

    // The CiA 402 disable sequence is the engage (power off = brake closed).
    load_engage_sequence(mask, preserve_active);

    PLEM_EMIT(plem::core::monitoring::Event::brake_engage());
}

template <size_t DOF>
void EtherCATController<DOF>::report_fault_reset_publications() {
    const auto& servos = ecat_bus_->servo_devices();
    for (size_t i = 0; i < servos.size(); ++i) {
        const auto publication = servos[i]->last_fault_reset_publication();
        if (publication == FaultResetPublication::NONE) {
            continue;
        }
        // error_code = the statusword that decided it, value = the outcome.
        // Both are raw observations of this cycle; neither claims the drive
        // did anything. Resetting an axis that is not in FAULT is routine (a
        // recovery resets every axis); an undecodable statusword stays an error.
        using plem::core::monitoring::Severity;
        const Severity severity =
            publication == FaultResetPublication::DROPPED_UNRECOGNIZED ? Severity::ERROR : Severity::INFO;
        auto event = plem::core::monitoring::Event::safety(plem::core::monitoring::EventType::SERVO_FAULT_RESET_RESULT,
                                                           severity, static_cast<uint8_t>(i));
        event.error_code = servos[i]->get_statusword();
        event.value = static_cast<float>(publication);
        PLEM_EMIT(event);
    }
}

template <size_t DOF>
void EtherCATController<DOF>::load_engage_sequence(uint8_t mask, bool preserve_active) noexcept {
    // Per-servo shutdown sequence: SWITCHED_ON -> READY_TO_SWITCH_ON ->
    // SWITCH_ON_DISABLED. The terminal step is the CiA 402 standard one:
    // Disable voltage (transitions 7/9/10/12) lands in SWITCH_ON_DISABLED.
    // NOT_READY_TO_SWITCH_ON is reachable only by power-on/application
    // reset, so no drive can be commanded there; a step targeting it would
    // stall forever on a family whose closed-evidence excludes
    // READY_TO_SWITCH_ON.
    for (size_t i = 0; i < DOF; ++i) {
        if ((mask & (uint8_t{1} << i)) == 0) {
            continue;
        }

        auto& seq = servo_sequences_[i];
        if (preserve_active && seq.size != 0 && !seq.release_target) {
            // Safety-forced engage over an engage already walking: keep the
            // head, the remaining steps and the stall history; re-loading
            // would re-consume the first target (a drive already at
            // SWITCHED_ON sees its READY command slip a cycle; one at READY
            // settles closed either way) and, repeated, starve the axis of
            // its next step.
            continue;
        }
        seq.head = 0;
        seq.size = 3;
        seq.release_target = false;
        seq.states[0] = ServoState::SWITCHED_ON;
        seq.states[1] = ServoState::READY_TO_SWITCH_ON;
        seq.states[2] = ServoState::SWITCH_ON_DISABLED;
        sequence_stall_cycles_[i] = 0;  // fresh episode for the stall window
        sequence_stall_reported_[i] = false;
    }
}

template <size_t DOF>
void EtherCATController<DOF>::set_fault_reset_policy(FaultResetPolicy policy,
                                                     const std::optional<FaultResetPacing>& pacing) {
    this->throw_if_running("set_fault_reset_policy");
    if (const core::Result<void> valid = validate_fault_reset(policy, pacing); !valid) {
        throw std::invalid_argument("[EtherCATController] set_fault_reset_policy: " + valid.message());
    }
    uint32_t interval_cycles = 0;
    uint32_t max_attempts = 0;
    if (policy == FaultResetPolicy::BOUNDED_AUTO_RESET) {
        // ceil(interval / period) must fit uint32, asked as
        // floor((interval - 1) / period) <= UINT32_MAX - 1, a division: the
        // product form (UINT32_MAX × period) overflows int64 at a long period
        // (this API takes any positive period; the product hardware range is
        // not a precondition here). Checked before cycles_at_least_ms truncates
        // its int64 result to uint32; interval >= 1 ms (validate_fault_reset
        // refused 0), and uint32 ms → ns fits int64 with room to spare.
        const int64_t interval_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds{pacing->interval_ms})
                .count();
        if ((interval_ns - 1) / cycle_period_ns_ > static_cast<int64_t>(UINT32_MAX) - 1) {
            throw std::invalid_argument(
                "[EtherCATController] set_fault_reset_policy: fault_reset_pacing.interval_ms (" +
                std::to_string(pacing->interval_ms) + ") does not fit a uint32 cycle count at a " +
                std::to_string(cycle_period_ns_) + " ns period");
        }
        interval_cycles = plem::core::cycles_at_least_ms(pacing->interval_ms, cycle_period_ns_);
        max_attempts = pacing->max_attempts;
    }
    // All three members change together, after every check passed.
    fault_reset_policy_ = policy;
    fault_reset_interval_cycles_ = interval_cycles;
    fault_reset_max_attempts_ = max_attempts;
}

template <size_t DOF>
void EtherCATController<DOF>::set_brake_setup(const std::array<uint8_t, DOF>& groups) {
    this->throw_if_running("set_brake_setup");
    // The partition rule, the same as the robot tier's brake-setup gate: an all-zero
    // table declares nothing (every joint its own brake); otherwise every joint's
    // group holds that joint, names declared joints only, and every joint it names
    // declares the same mask. A mask outside it would expand a brake op to a joint
    // whose own group disagrees, or to no joint at all.
    const bool declared = std::any_of(groups.begin(), groups.end(), [](uint8_t g) { return g != 0; });
    for (size_t i = 0; declared && i < DOF; ++i) {
        const uint8_t group = groups[i];
        const auto where = [&] {
            return std::string("[") + logger_tag() + "] set_brake_setup: groups[" + std::to_string(i) + "] (0x" +
                   fmt::format("{:x}", group) + ")";
        };
        if ((group & (1u << i)) == 0) {
            throw std::invalid_argument(where() + " does not include joint " + std::to_string(i) +
                                        " itself — once any group is declared, groups[i] is the mask released "
                                        "together with joint i (self bit included)");
        }
        if ((group & ~all_joints_mask()) != 0) {
            throw std::invalid_argument(where() + " names joints beyond this controller's " + std::to_string(DOF) +
                                        " axes");
        }
        for (size_t j = 0; j < DOF; ++j) {
            if ((group & (1u << j)) != 0 && groups[j] != group) {
                throw std::invalid_argument(where() + " and groups[" + std::to_string(j) + "] (0x" +
                                            fmt::format("{:x}", groups[j]) +
                                            ") disagree — joints sharing one brake declare the same mask");
            }
        }
    }
    // Distinct group masks → slots (deduplicated: partner joints declare the same mask).
    brake_group_count_ = 0;
    for (size_t i = 0; i < DOF; ++i) {
        const uint8_t mask = groups[i];
        if (mask == 0) {
            continue;
        }
        bool known = false;
        for (size_t g = 0; g < brake_group_count_; ++g) {
            if (brake_group_masks_[g] == mask) {
                known = true;
                break;
            }
        }
        if (!known && brake_group_count_ < DOF) {
            brake_group_masks_[brake_group_count_] = mask;
            ++brake_group_count_;
        }
    }
}

template <size_t DOF>
uint8_t EtherCATController<DOF>::brake_group_closure(uint8_t bits) const noexcept {
    // Expand joint bits to their full shared-brake groups. A joint in no
    // group is its own group. The table is written only by
    // set_brake_setup(), before the RT loop starts.
    if (bits == 0x00) {
        return 0x00;
    }
    uint8_t closure = bits;
    for (size_t g = 0; g < brake_group_count_; ++g) {
        if ((brake_group_masks_[g] & bits) != 0) {
            closure |= brake_group_masks_[g];
        }
    }
    return closure;
}

template <size_t DOF>
bool EtherCATController<DOF>::release_admissible(size_t axis) const noexcept {
    const auto& servos = ecat_bus_->servo_devices();
    if (axis >= servos.size() || servos[axis] == nullptr) {
        return false;  // a missing device is never admitted
    }
    // In SAFEOP the inputs read valid while the drive ignores the outputs:
    // without OP, a statusword and display that pass below prove nothing
    // about a release the drive would execute. Gates both branches.
    if (!slave_states_[axis].operational) {
        return false;
    }
    const DeviceServo& servo = *servos[axis];
    if (in_declared_mode(servo)) {
        return true;
    }
    // The POST_ENABLE exception: such a drive may show 0 until its first
    // enable. Admitted only from a recognized
    // resting state, only before the axis ever showed its mode, and never
    // while an in-service trip episode is open; a re-admission under an
    // open episode would suppress the next forced engage.
    // Any nonzero mismatch is a mismatch whatever the declaration says.
    const uint8_t bit = static_cast<uint8_t>(uint8_t{1} << axis);
    if (servo.mode_display_timing() != ModeDisplayTiming::POST_ENABLE ||
        ((mode_seen_ | in_service_trip_reported_) & bit) != 0) {
        return false;
    }
    if (!servo.is_initialized() || !servo.status_recognized() || servo.get_modes_of_operation_display() != 0) {
        return false;
    }
    const ServoState state = servo.get_servo_state();
    return state == ServoState::SWITCH_ON_DISABLED || state == ServoState::READY_TO_SWITCH_ON;
}

template <size_t DOF>
uint8_t EtherCATController<DOF>::release_ready_mask() const noexcept {
    // Per-axis acceptance R from this cycle's PDO image (a missing device is
    // never ready), then the group closure: drop every group that is not
    // entirely inside R, checked against R itself, not the mask being
    // reduced, so `M ⊆ ready ⇔ brake_group_closure(M) ⊆ R` holds for the
    // one-step closure the sequence setters apply.
    // Joints outside every group keep their own bit (identity).
    const auto& servos = ecat_bus_->servo_devices();
    uint8_t accepted = 0;
    for (size_t i = 0; i < servos.size() && i < DOF; ++i) {
        if (release_admissible(i)) {
            accepted |= static_cast<uint8_t>(uint8_t{1} << i);
        }
    }
    uint8_t ready = accepted;
    for (size_t g = 0; g < brake_group_count_; ++g) {
        if ((accepted & brake_group_masks_[g]) != brake_group_masks_[g]) {
            ready &= static_cast<uint8_t>(~brake_group_masks_[g]);
        }
    }
    return ready;
}

template <size_t DOF>
void EtherCATController<DOF>::emit_release_refused(uint8_t requested) noexcept {
    // The decision refused the whole request because some joint of its brake-
    // group closure is not admitted; name each such joint: a slave not
    // operational by its AL state, otherwise with what the axis showed this
    // cycle (statusword, 0x6061 display; raw observations, no cause). Same
    // samples as the decision: nothing reads the bus between the decide and
    // this apply.
    const uint8_t closure = brake_group_closure(requested);
    const auto& servos = ecat_bus_->servo_devices();
    for (size_t i = 0; i < DOF; ++i) {
        if ((closure & (uint8_t{1} << i)) == 0) {
            continue;
        }
        const DeviceServo* servo = i < servos.size() ? servos[i].get() : nullptr;
        if (release_admissible(i)) {
            continue;
        }
        if (servo != nullptr && !slave_states_[i].operational) {
            auto event = plem::core::monitoring::Event::error(plem::core::monitoring::EventType::SLAVE_NOT_OPERATIONAL,
                                                              slave_states_[i].al_state, static_cast<uint8_t>(i));
            event.value = static_cast<float>(servo->position());
            PLEM_EMIT(event);
            continue;
        }
        auto event = plem::core::monitoring::Event::error(
            plem::core::monitoring::EventType::MODES_OF_OPERATION_MISMATCH,
            servo != nullptr ? servo->get_statusword() : 0, static_cast<uint8_t>(i));
        event.value = servo != nullptr ? static_cast<float>(servo->get_modes_of_operation_display()) : 0.0f;
        PLEM_EMIT(event);
    }
}

template <size_t DOF>
BrakeQuiesceOutcome EtherCATController<DOF>::quiesce_brakes() noexcept {
    close_brake_release_admission();

    BrakeQuiesceOutcome outcome;
    outcome.all_joints_mask = all_joints_mask();

    for (int attempt = 0; attempt < kQuiesceEngageAttempts; ++attempt) {
        // Any pending brake op (a posted request, an active sequence)
        // is not a confirmed terminal; the wait below holds until the
        // group-atomic settlement lands instead of fast-pathing on a stale
        // engaged bit.
        const uint8_t remaining = static_cast<uint8_t>(
            all_joints_mask() & (~brake_state_mask_.load(std::memory_order_acquire) | brake_operation_pending()));
        if (remaining == 0) {
            break;
        }
        request_brake_engage(remaining);  // the normal RT mailbox; the live loop consumes it
        const int64_t deadline = now_ns() + quiesce_attempt_timeout_ns_;
        while (now_ns() < deadline) {
            const uint8_t open = static_cast<uint8_t>(
                all_joints_mask() & (~brake_state_mask_.load(std::memory_order_acquire) | brake_operation_pending()));
            if (open == 0) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    outcome.engaged_mask = static_cast<uint8_t>(brake_state_mask_.load(std::memory_order_acquire) &
                                                static_cast<uint8_t>(~brake_operation_pending()));
    outcome.timeout_mask = static_cast<uint8_t>(outcome.all_joints_mask & ~outcome.engaged_mask);

    // Drive-quiet observation (statusword snapshot: measured facts for the
    // operator message, not control inputs). Read the per-cycle atomic
    // mirror, not DeviceServo::get_statusword(); that getter returns the
    // plain RT-owned PDO field and this caller is a non-RT thread with the RT
    // loop still cycling (the mirror lags at most one cycle).
    for (size_t i = 0; i < DOF && i < outcome.statuswords.size(); ++i) {
        outcome.statuswords[i] = statusword_mirror_[i].load(std::memory_order_relaxed);
    }
    return outcome;
}

template <size_t DOF>
void EtherCATController<DOF>::settle_group_released(uint8_t bit) noexcept {
    // One enabled axis opens the shared brake: every joint of the group is
    // unheld from that instant. Report the partner released while its own
    // sequence still climbs (the safe direction: observers must not assume it
    // is held). Sequence completion is a physical confirmation on
    // power-coupled hardware, so the whole group re-verifies too.
    const uint8_t group = brake_group_closure(bit);
    brake_state_mask_.fetch_and(static_cast<uint8_t>(~group), std::memory_order_release);
}

template <size_t DOF>
void EtherCATController<DOF>::settle_group_engaged(uint8_t bit) noexcept {
    // The shared brake is confirmed closed only when every axis of the group
    // sits in a proven-unpowered step (allowlist below). A
    // partner in any other state might hold the brake open; flipping this
    // joint's bit then would report "engaged" on a joint that is neither
    // braked nor powered. The partner's own later completion (or fallback)
    // re-runs this check and flips the whole group; a partner that never
    // quiets leaves the group un-engaged (and the stall window
    // names it). An axis with no statusword (or an unrecognized one, whose
    // held state is a stale reading) cannot prove closure and
    // blocks the flip (fail-safe).
    const uint8_t group = brake_group_closure(bit);
    const auto& servo_devices = ecat_bus_->servo_devices();
    for (size_t j = 0; j < DOF; ++j) {
        if ((group & (uint8_t{1} << j)) == 0) {
            continue;
        }
        if (j >= servo_devices.size() || !servo_devices[j]->is_initialized() ||
            !servo_devices[j]->status_recognized()) {
            return;
        }
        // An allowlist: only a step that proves the brake
        // closed may flip the group (proves_brake_closed; brake_logic.hpp
        // owns the step and its vendor caveat). A state that might hold the
        // brake open blocks the flip, fail-safe: the partner's own disable
        // sequence (or fault handling) walks it into a proven step, whose
        // settlement then flips the group.
        if (!proves_brake_closed(servo_devices[j]->get_servo_state())) {
            return;
        }
    }
    brake_state_mask_.fetch_or(group, std::memory_order_release);
}

template <size_t DOF>
bool EtherCATController<DOF>::request_brake_release() {
    return request_brake_release(all_joints_mask());
}

template <size_t DOF>
bool EtherCATController<DOF>::request_brake_release(uint8_t joint_mask) {
    joint_mask &= all_joints_mask();
    if (joint_mask == 0x00) {
        return true;  // nothing to queue, not an admission refusal
    }
    if (release_admission_closed_.load(std::memory_order_acquire)) {
        return false;  // refused, nothing queued (reporting
                       // surface; the RT consume re-check is the authority)
    }

    // The OR + opposite-AND pair is serialized against other posts
    // (interleaved pairs could clear both masks, the last request lost).
    std::lock_guard<std::mutex> lock(brake_request_post_mutex_);
    // Seqlock: the OR + opposite-AND pair is a two-word write; an
    // unguarded reader can interleave (read release before the OR, engage
    // after the AND) and see a union of 0 while the real union never was.
    // Odd version = pair in progress; brake_operation_pending() retries.
    brake_request_version_.fetch_add(1, std::memory_order_acq_rel);
    brake_release_request_mask_.fetch_or(joint_mask, std::memory_order_release);
    if (brake_release_post_pair_hook_for_test_) {
        brake_release_post_pair_hook_for_test_();  // deterministic interleave pin, see header
    }
    brake_engage_request_mask_.fetch_and(static_cast<uint8_t>(~joint_mask), std::memory_order_release);
    brake_request_version_.fetch_add(1, std::memory_order_acq_rel);
    return true;
}

template <size_t DOF>
void EtherCATController<DOF>::close_brake_release_admission() noexcept {
    release_admission_closed_.store(true, std::memory_order_release);
}

template <size_t DOF>
bool EtherCATController<DOF>::is_brake_release_admission_open() const noexcept {
    return !release_admission_closed_.load(std::memory_order_acquire);
}

template <size_t DOF>
void EtherCATController<DOF>::request_brake_engage() {
    request_brake_engage(all_joints_mask());
}

template <size_t DOF>
void EtherCATController<DOF>::request_brake_engage(uint8_t joint_mask) {
    joint_mask &= all_joints_mask();
    if (joint_mask == 0x00) {
        return;
    }

    std::lock_guard<std::mutex> lock(brake_request_post_mutex_);     // see request_brake_release
    brake_request_version_.fetch_add(1, std::memory_order_acq_rel);  // seqlock, see release side
    brake_engage_request_mask_.fetch_or(joint_mask, std::memory_order_release);
    brake_release_request_mask_.fetch_and(static_cast<uint8_t>(~joint_mask), std::memory_order_release);
    brake_request_version_.fetch_add(1, std::memory_order_acq_rel);
}

template <size_t DOF>
uint8_t EtherCATController<DOF>::brake_engaged() const noexcept {
    return brake_state_mask_.load(std::memory_order_acquire);
}

template <size_t DOF>
void EtherCATController<DOF>::apply_brake_effect(const BrakeEffect& effect) noexcept {
    switch (effect.kind) {
        case BrakeEffectKind::START_ENGAGE:
            start_engage(effect.mask, /*preserve_active=*/false);
            break;
        case BrakeEffectKind::START_FORCED_ENGAGE:
            start_engage(effect.mask, /*preserve_active=*/true);
            break;
        case BrakeEffectKind::START_RELEASE:
            set_brake_release_sequence(effect.mask);
            break;
        case BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY:
            PLEM_EMIT(plem::core::monitoring::Event::brake_engage());
            break;
        case BrakeEffectKind::EMIT_PROTECTIVE_STOP:
            PLEM_EMIT(plem::core::monitoring::Event::protective_stop());
            break;
        case BrakeEffectKind::EMIT_PROTECTIVE_STOP_CLEAR:
            PLEM_EMIT(plem::core::monitoring::Event::protective_stop_clear());
            break;
        case BrakeEffectKind::EMIT_RELEASE_REFUSED:
            emit_release_refused(effect.mask);
            break;
    }
}

// Explicit template instantiations for the model schema's range 1..8 (the
// drive layer's model file chooses N).
template class EtherCATController<1>;
template class EtherCATController<2>;
template class EtherCATController<3>;
template class EtherCATController<4>;
template class EtherCATController<5>;
template class EtherCATController<6>;
template class EtherCATController<7>;
template class EtherCATController<8>;

}  // namespace plem::fieldbus::ethercat
