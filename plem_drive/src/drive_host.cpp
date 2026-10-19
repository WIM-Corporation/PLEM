// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/drive/drive_host.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <plem/core/logging/logger.hpp>
#include <plem/core/threading/rt_memory.hpp>
#include <plem/core/timing/timespec.hpp>
#include <plem/fieldbus/watchdog.hpp>

#include "bus_from_spec.hpp"
#include "drive_host_rules.hpp"
#include "plem/drive/bus_setup.hpp"

namespace plem::drive {

namespace {
auto& logger = plem::core::Logger::instance();
}  // namespace

template <size_t N>
core::Result<std::unique_ptr<DriveHost<N>>> DriveHost<N>::create(const models::DriveModelSpec<N>& spec,
                                                                 std::shared_ptr<fieldbus::ethercat::EtherCATBus> bus,
                                                                 const DriveHostParams& params) {
    using Result = core::Result<std::unique_ptr<DriveHost<N>>>;
    try {
        // Value gate first (no side effect), then the graph gate.
        if (const core::Result<void> prepared = detail::prepare_drive_bus(spec, bus.get()); !prepared) {
            return Result::error(prepared.message());
        }
        if (const core::Result<void> init = bus->initialize(); !init) {
            return Result::error(detail::drive_host_tag(spec.model_name) + " bus initialize failed: " + init.message());
        }
        if (bus->servo_count() != N) {
            return Result::error(detail::drive_host_tag(spec.model_name) + " bus registered " +
                                 std::to_string(bus->servo_count()) + " drives but this host is built for exactly " +
                                 std::to_string(N));
        }

        std::unique_ptr<DriveHost<N>> host(new DriveHost<N>());
        host->approval_ = spec.approval;
        host->bench_unapproved_ = params.bench_unapproved;
        host->model_name_ = spec.model_name;
        host->streams_torque_ = detail::drive_streams_torque(spec);
        host->axis_names_ = spec.axis_names;
        for (size_t i = 0; i < N; ++i) {
            const auto& safety = spec.safety[i];
            if (detail::axis_endless(safety.position_min_rad, safety.position_max_rad)) {
                host->endless_encoder_resolution_[i] = spec.axes[i].factors.position.encoder_resolution;
            }
        }
        for (size_t s = 0; s < spec.slave_count; ++s) {
            host->pdo_mapping_digests_.push_back(
                bus->scanned_pdo_mapping_digest(static_cast<uint32_t>(spec.slaves[s].ring_position)));
        }
        host->bus_ = std::move(bus);
        host->flags_ = std::make_shared<core::ControlModeFlags>();
        host->shared_ = std::make_shared<core::SharedJointData<N>>();
        if (const core::Result<void> monitoring = host->monitoring_.initialize({}); !monitoring) {
            return Result::error(detail::drive_host_tag(spec.model_name) + " monitoring: " + monitoring.message());
        }

        typename fieldbus::ethercat::EtherCATController<N>::Config config;
        config.cycle_time_us = static_cast<int>(spec.cycle_time_us);
        config.policy = core::threading::SchedulingPolicy::FIFO;
        config.rt_priority = params.rt_priority;
        config.cpu_affinity = params.cpu_affinity;
        if (!config.cpu_affinity) {
            if (const auto cpu = core::threading::detect_isolated_cpu()) {
                config.cpu_affinity = core::threading::CpuAffinity{{*cpu}};
            } else {
                logger.warn("{} No isolated CPU detected — the RT loop runs unpinned",
                            detail::drive_host_tag(spec.model_name));
            }
        }
        auto& ctrl = host->controller_;
        ctrl = std::make_unique<fieldbus::ethercat::EtherCATController<N>>(config);
        ctrl->set_bus(host->bus_);
        ctrl->set_mode_flags(host->flags_);  // flags are the committed mode word here (no probe)
        ctrl->set_shared_data(host->shared_);
        ctrl->set_monitoring_context(&host->monitoring_);
        ctrl->set_fault_reset_policy(spec.fault_reset, spec.fault_reset_pacing);
        const auto limits = detail::drive_limits(spec);
        ctrl->set_drive_limits(limits);
        const auto decelerations = detail::drive_stop_decelerations(spec);
        ctrl->set_drive_stop(decelerations);
        ctrl->set_command_stale_timeout_ns(kCommandStaleTimeoutNs);
        logger.info("{} assembled: {} drives on {} slaves, cycle_time_ns {}, sync_mode {}",
                    detail::drive_host_tag(spec.model_name), N, spec.slave_count, uint64_t{spec.cycle_time_us} * 1000,
                    spec.dc_enable ? "dc_sync0" : "drive_default");
        return Result::ok(std::move(host));
    } catch (const std::exception& e) {
        // The gates report through Result above; anything that still throws
        // (allocation, the bus, a controller setter refusing what the value gate
        // let through) is reported verbatim.
        return Result::error(e.what());
    }
}

template <size_t N>
DriveHost<N>::~DriveHost() {
    if (started_ && !stopped_) {
        (void)stop();
    }
}

template <size_t N>
core::Result<void> DriveHost<N>::start() {
    if (stopped_) {
        return core::Result<void>::error(detail::drive_host_tag(model_name_) +
                                         " one-shot lifecycle: stop() has run — create a new host");
    }
    if (started_) {
        return core::Result<void>::ok();
    }
    core::threading::apply_memory_lock();
    if (auto r = controller_->start(); !r) {
        return r;
    }
    started_ = true;
    return core::Result<void>::ok();
}

template <size_t N>
fieldbus::ethercat::BrakeQuiesceOutcome DriveHost<N>::stop() {
    const auto action = detail::decide_stop(started_, stopped_, released_);
    if (action != detail::StopAction::QUIESCE) {
        if (!stopped_) {
            stop_outcome_ = detail::unreleased_stop_outcome(N);
        }
        if (action == detail::StopAction::STOP_LOOP) {
            controller_->stop();
            stopped_ = true;
        }
        return stop_outcome_;
    }
    ramp_stop();
    // Bounded by the loop's own deadlines (deceleration + settle, then Quick
    // stop); the cap only guards a loop that stopped cycling.
    const int64_t give_up = core::now_ns() + 30'000'000'000;
    while (controller_->ramp_stop_active() && core::now_ns() < give_up) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    disable();
    stop_outcome_ = controller_->quiesce_brakes();  // closes release admission itself
    if (!stop_outcome_.fully_engaged()) {
        logger.error("{} forced stop: brakes not confirmed engaged (timeout mask 0x{:02x}) — the loop stops anyway",
                     detail::drive_host_tag(model_name_), stop_outcome_.timeout_mask);
    }
    controller_->stop();
    stopped_ = true;
    warn_endless_counts_past_int32();
    return stop_outcome_;
}

// A multi-turn absolute drive re-derives 0x6064 from its encoder at the next
// start and may raise a counter-overflow alarm when the count no longer fits.
// Warn only: PLEM cannot change that drive behaviour. A heuristic over this
// run's count: it starts from the drive's word (any earlier wrap is unseen) and
// the drive's own offsets are not known here. The loop has stopped,
// so the RT-written counts are quiescent.
template <size_t N>
void DriveHost<N>::warn_endless_counts_past_int32() const {
    const auto& servos = bus_->servo_devices();
    for (size_t i = 0; i < N; ++i) {
        const uint32_t resolution = endless_encoder_resolution_[i];
        if (resolution == 0) {
            continue;
        }
        const auto turns = detail::endless_turns_past_int32(servos[i]->position_count(), resolution);
        if (!turns) {
            continue;
        }
        logger.warn(
            "{} axes[{}] ({}): this run's position count (counted from the drive's value at start) ended {:.1f} "
            "motor turns past the drive's 32-bit range. A drive with a multi-turn absolute encoder may refuse its "
            "next start with a counter-overflow alarm unless its encoder runs in incremental mode or a setting "
            "ignores that overflow while keeping the full 32-bit position wrap. The absence of this warning does not "
            "prove the next start is fine — see the drive model reference, endless axes.",
            detail::drive_host_tag(model_name_), i, axis_names_[i].data(), *turns);
    }
}

template <size_t N>
core::Result<void> DriveHost<N>::enable() {
    const int64_t last = last_write_ns_.load(std::memory_order_acquire);
    const bool fresh = last != 0 && (core::now_ns() - last) <= kCommandStaleTimeoutNs;
    const auto action = detail::decide_enable(detail::EnableInputs{
        .started = started_,
        .stopped = stopped_,
        .approval = approval_,
        .protective_stop = flags_->is_protective_stop_active(),
        .command_fresh = fresh,
        .brake_engaged = controller_->brake_engaged(),
        .brake_operation_pending = controller_->brake_operation_pending(),
        .bench_unapproved = bench_unapproved_,
        .streams_torque = streams_torque_,
        .stopping = controller_->ramp_stop_active(),
    });
    using detail::EnableAction;
    switch (action) {
        case EnableAction::ALREADY_ENABLED:
            return core::Result<void>::ok();
        case EnableAction::NOT_STARTED:
            return core::Result<void>::error(detail::drive_host_tag(model_name_) +
                                             " enable() needs a started, not yet stopped host");
        case EnableAction::NOT_APPROVED:
            return core::Result<void>::error(
                detail::drive_host_tag(model_name_) +
                " motion refused: the model file is unapproved (approval.status) — "
                "bench it (`plem drive run <file>` on a terminal offers the bench, which records the approval in "
                "that file); or "
                "run in bench mode (`plem drive run <file> --unapproved`) with an operator at the drive; "
                "start and stop stay allowed");
        case EnableAction::PROTECTIVE_STOP:
            return core::Result<void>::error(
                detail::drive_host_tag(model_name_) + " enable() refused: protective stop active (0x" +
                std::to_string(flags_->protective_stop.load()) +
                ") — look in recorded rt_events for the event that set it (a broken limit: LIMIT_STOP with the "
                "axis and value); send reset_error (DriveHost::reset) and wait until status reports "
                "protective_stop 0. The stop is not cleared while a measured position or velocity still breaks "
                "a limit or a position does not decode (rt_events PROTECTIVE_STOP, error_code 1), nor while a "
                "silent stream still stands (COMMAND_STALE) — stream again, then reset_error");
        case EnableAction::STALE_COMMAND:
            return core::Result<void>::error(detail::drive_host_tag(model_name_) +
                                             " enable() refused: no fresh command — a bus with a cst axis must "
                                             "stream its torque (DriveHost::write) before releasing, so the "
                                             "release meets a live command");
        case EnableAction::STOPPING:
            return core::Result<void>::error(detail::drive_host_tag(model_name_) +
                                             " enable() refused: a stop is still running and ends by engaging "
                                             "every brake; release after it ends (DriveHost::read() reports "
                                             "stopping false; on the ROS `status` topic (DriveStatus), which has no "
                                             "stopping field: brake_engaged set for every axis and "
                                             "brake_operation_pending 0)");
        case EnableAction::RELEASE:
            break;
    }
    controller_->open_command_epoch();
    released_ = true;
    flags_->motion_enabled.store(true, std::memory_order_release);
    if (!controller_->request_brake_release()) {
        flags_->motion_enabled.store(false, std::memory_order_release);
        return core::Result<void>::error(detail::drive_host_tag(model_name_) +
                                         " enable() refused: the controller is not accepting a brake release now");
    }
    return core::Result<void>::ok();
}

template <size_t N>
void DriveHost<N>::disable() {
    // Engage first: the RT loop reads this flag before its brake step, so a
    // cleared flag it sees always has its engage posted; the gate closes in
    // the frame that carries the stop controlword, not one frame before it.
    controller_->request_brake_engage();
    flags_->motion_enabled.store(false, std::memory_order_release);
}

template <size_t N>
void DriveHost<N>::ramp_stop() {
    if (streams_torque_ || !flags_->motion_enabled.load(std::memory_order_acquire)) {
        disable();
        return;
    }
    controller_->request_ramp_stop();
}

template <size_t N>
bool DriveHost<N>::write(std::span<const double> command, std::span<const double> rate, int64_t sampled_ns) noexcept {
    if (command.size() != N || (!rate.empty() && rate.size() != N)) {
        return false;
    }
    // The stale stop and enable() compute now − stamp: a stamp from another
    // clock (epoch time is far ahead) would hold the stop off for good.
    if (sampled_ns <= 0 || sampled_ns > core::now_ns() + kCommandStaleTimeoutNs) {
        return false;
    }
    core::JointCommand<N> cmd;
    for (size_t i = 0; i < N; ++i) {
        cmd.value[i] = command[i];
        cmd.rate[i] = rate.empty() ? 0.0 : rate[i];
        if (!std::isfinite(cmd.value[i]) || !std::isfinite(cmd.rate[i])) {
            return false;
        }
    }
    cmd.update_ns = sampled_ns;
    shared_->command.write(cmd);
    last_write_ns_.store(cmd.update_ns, std::memory_order_release);
    return true;
}

template <size_t N>
bool DriveHost<N>::read(State& out) const noexcept {
    if (!shared_->joint_actual.try_read(out.actual)) {
        return false;
    }
    // Pending first: the loop settles the brake state, then clears the busy
    // mirror, then publishes enabled; a pending 0 read here sees the brakes
    // it settled, so a finished release never reads "engaged, nothing pending".
    out.brake_operation_pending = controller_->brake_operation_pending();
    out.protective_stop = flags_->protective_stop.load(std::memory_order_acquire);
    out.operation_enabled = flags_->all_drives_enabled.load(std::memory_order_acquire);
    out.brake_engaged = controller_->brake_engaged();
    out.stopping = controller_->ramp_stop_active();
    for (size_t i = 0; i < N; ++i) {
        out.statusword[i] = controller_->statusword(i);
    }
    out.process_data_complete = controller_->process_data_complete();
    return true;
}

template <size_t N>
void DriveHost<N>::reset() {
    (void)bus_->reset_all_faults();
    flags_->reset_error.store(true, std::memory_order_release);
}

// Every N the schema admits (kMaxAxes); the ROS layer erases N behind
// IDriveHost, so a drive model file picks it at run time.
template class DriveHost<1>;
template class DriveHost<2>;
template class DriveHost<3>;
template class DriveHost<4>;
template class DriveHost<5>;
template class DriveHost<6>;
template class DriveHost<7>;
template class DriveHost<8>;

}  // namespace plem::drive
