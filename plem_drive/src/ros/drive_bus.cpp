// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "ros/drive_bus.hpp"

#include <exception>
#include <stdexcept>
#include <utility>
#include <variant>

#include <plem/core/monitoring/event.hpp>
#include <plem/core/monitoring/rt_sample.hpp>
#include <plem/core/monitoring/stream_traits.hpp>
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/fieldbus/ethercat/bus.hpp>

#include "plem/drive/ros/rt_event.hpp"

namespace plem::drive::ros {

// The event stream keeps the descriptor's depth and QoS; the drive layer publishes it
// under the node's private name ("~/rt_events", getting-started.md), not the bare one.
constexpr auto kEventStream = plem::core::monitoring::StreamTraits<plem::core::monitoring::Event>::descriptor;
static_assert(kEventStream.qos == plem::core::monitoring::QoSPreset::ReliableTransient,
              "DriveBus builds a reliable transient_local QoS: the rt_events descriptor changed");

// ~/status publishes the host's ProtectiveStopSource bits as they are; DriveStatus names them.
static_assert(plem_drive_msgs::msg::DriveStatus::PROTECTIVE_STOP_SOURCE_INTERNAL ==
              static_cast<uint8_t>(plem::core::ProtectiveStopSource::INTERNAL));
static_assert(plem_drive_msgs::msg::DriveStatus::PROTECTIVE_STOP_SOURCE_EXTERNAL ==
              static_cast<uint8_t>(plem::core::ProtectiveStopSource::EXTERNAL));

std::unique_ptr<IDriveHost> build_drive_host(const plem::drive::DriveModelSpecAny& spec, const std::string& model_file,
                                             unsigned master_index, const plem::drive::DriveHostParams& params) {
    auto built = std::visit(
        [&](const auto& s) {
            return make_drive_host(s, std::make_shared<plem::fieldbus::ethercat::EtherCATBus>(master_index), params);
        },
        spec);
    if (!built) {
        throw std::runtime_error("Drive model '" + model_file + "' could not be built: " + built.message());
    }
    return std::move(built).value();
}

DriveBus::DriveBus(rclcpp::Node& node, std::unique_ptr<IDriveHost> host, bool unapproved_motion)
    : node_(node), host_(std::move(host)), unapproved_motion_(unapproved_motion) {
    axes_ = host_->axis_names().size();  // 1..kMaxAxes: make_drive_host<N> is the only host
    status_.statusword.resize(axes_);

    // transient_local: an operator who attaches `ros2 topic echo` after the
    // fact still sees the current level rather than waiting for the next tick.
    status_pub_ = node_.create_publisher<plem_drive_msgs::msg::DriveStatus>(
        "~/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    // Events are the timing record (RT monotonic stamps), retained and deep
    // enough that a burst around a protective stop survives a late subscriber.
    rt_event_pub_ = node_.create_publisher<plem_rt_msgs::msg::RtEvent>(
        "~/rt_events", rclcpp::QoS(rclcpp::KeepLast(kEventStream.depth)).reliable().transient_local());
    // Static facts, once: latched so the bench runner (and an operator) that
    // attaches later still reads them.
    info_pub_ = node_.create_publisher<plem_drive_msgs::msg::DriveInfo>(
        "~/info", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    plem_drive_msgs::msg::DriveInfo info;
    info.header.stamp = node_.get_clock()->now();
    const auto digests = host_->pdo_mapping_digests();
    info.pdo_mapping_digest.assign(digests.begin(), digests.end());
    const auto modes = host_->modes_of_operation();
    info.modes_of_operation.assign(modes.begin(), modes.end());
    info_pub_->publish(info);

    rt_raw_ = std::make_unique<RtRawBatcher>(node_);

    command_service_ = std::make_shared<ControlCommandService>(
        node_,
        ControlCommandService::Callbacks{
            // Requested stop: every csp and csv axis decelerates to rest, then the
            // brakes engage (a bus with a cst axis: at once). Asynchronous.
            .protective_stop =
                [this] {
                    host_->ramp_stop();
                    return plem::core::Result<void>::ok();
                },
            .release_brake = [this] { return on_release_brake(); },
            .free_drive =
                [] {
                    return plem::core::Result<void>::error(
                        "free_drive is not a drive-layer mode: this node sends each axis the command of its "
                        "declared mode (cst, csp or csv) and does not compensate gravity, so there is no "
                        "hand-guiding state to enter here");
                },
            .reset_error = [this] { host_->reset(); },
            // The readiness gate (slave in OP, mode display) can refuse the
            // release a cycle later; only ~/rt_events names that refusal.
            .release_brake_accepted = "requested — watch status.operation_enabled; a readiness refusal is "
                                      "reported on rt_events",
            .protective_stop_accepted = "requested — the stop runs on after this answer; watch "
                                        "status.brake_engaged",
        },
        "~/control_command");
}

DriveBus::~DriveBus() {
    stop_draining();
}

plem::core::Result<void> DriveBus::start(std::chrono::milliseconds op_timeout) {
    auto result = host_->start();
    if (result && !drain_thread_.joinable()) {
        draining_.store(true, std::memory_order_release);
        try {
            drain_thread_ = std::thread([this] { drain_samples(); });
        } catch (const std::exception& e) {  // system_error, or bad_alloc for the thread state
            // rt_raw is only a recording: the host keeps running and stop() quiesces it as usual.
            draining_.store(false, std::memory_order_release);
            RCLCPP_ERROR(node_.get_logger(), "rt_raw not started: %s — the drive runs on, unrecorded", e.what());
        }
    }
    if (!result) {
        return result;
    }
    RCLCPP_INFO(node_.get_logger(), "waiting for every slave to reach OP (working counter complete)");
    const auto started = std::chrono::steady_clock::now();
    DriveState state;
    while (!(host_->read(state) && state.process_data_complete)) {
        // The controller manager activates the hardware while it handles the
        // robot description: a Ctrl-C here shuts the context down, and nothing
        // else would stop the bus before the process ends.
        if (!rclcpp::ok(node_.get_node_base_interface()->get_context())) {
            (void)stop();
            return plem::core::Result<void>::error("interrupted before every slave reached OP");
        }
        if (std::chrono::steady_clock::now() - started > op_timeout) {
            (void)stop();
            return plem::core::Result<void>::error(
                "not every slave reached OP within " + std::to_string(op_timeout.count()) +
                " ms: the working counter stayed incomplete — `ethercat slaves` shows each slave's state, and "
                "the kernel log (`sudo dmesg`) why one stayed out of OP");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node_.get_logger(), "every slave in OP (working counter complete) %.1f s after the loop started",
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    return result;
}

plem::fieldbus::ethercat::BrakeQuiesceOutcome DriveBus::stop() {
    host_->ramp_stop();
    stop_draining();
    return host_->stop();
}

void DriveBus::stop_draining() {
    draining_.store(false, std::memory_order_release);
    if (drain_thread_.joinable()) {
        drain_thread_.join();
    }
}

void DriveBus::drain_samples() noexcept {
    auto* samples = host_->monitoring_context()->queue<plem::core::monitoring::RtSample>();
    try {
        while (draining_.load(std::memory_order_acquire)) {
            bool drained = false;
            while (draining_.load(std::memory_order_acquire)) {
                auto* sample = samples->front();
                if (sample == nullptr) {
                    break;
                }
                rt_raw_->add(*sample);
                samples->pop();
                drained = true;
            }
            if (!drained) {
                rt_raw_->idle();
            }
            std::this_thread::sleep_for(kSampleDrainPeriod);
        }
        // A context already shut down cannot publish: the partial batch goes with it.
        if (rclcpp::ok(node_.get_node_base_interface()->get_context())) {
            rt_raw_->flush();
        }
    } catch (const std::exception& e) {
        // Ends the drain only: this thread makes no host call, so the drive and
        // its stop() are untouched.
        RCLCPP_ERROR(node_.get_logger(), "rt_raw stopped: %s — the drive runs on, unrecorded", e.what());
    }
}

plem::core::Result<void> DriveBus::on_release_brake() {
    // A release that finds every brake released and settled is the host's
    // no-op (decide_enable's ALREADY_ENABLED, the same two fields); only a
    // real release is bench evidence. A torn snapshot counts as a release: the
    // conservative side.
    DriveState before{};
    const bool already_enabled =
        host_->read(before) && before.brake_engaged == 0 && before.brake_operation_pending == 0;
    auto result = host_->enable();
    if (result && unapproved_motion_ && !already_enabled) {
        RCLCPP_WARN(node_.get_logger(),
                    "BENCH MODE — brake released on an unapproved model (hardware parameter unapproved=true, which "
                    "plem drive run --unapproved sets); this release is bench evidence, not approved motion");
    }
    return result;
}

std::optional<DriveState> DriveBus::publish_tick() {
    // One bus-clock time per tick, on this tick's events and its status, the
    // events published first.
    const auto now = node_.get_clock()->now();

    auto* events = host_->monitoring_context()->queue<plem::core::monitoring::Event>();
    while (auto* event = events->front()) {
        rt_event_pub_->publish(to_rt_event(*event, now));
        events->pop();
    }

    DriveState state{};
    if (!host_->read(state)) {
        return std::nullopt;  // torn snapshot; the next tick is one period away
    }
    if (++status_ticks_ >= kStatusDecimation) {
        status_ticks_ = 0;
        status_.header.stamp = now;
        status_.protective_stop = state.protective_stop;
        status_.operation_enabled = state.operation_enabled;
        status_.brake_engaged = state.brake_engaged;
        status_.brake_operation_pending = state.brake_operation_pending;
        for (size_t i = 0; i < axes_; ++i) {
            status_.statusword[i] = state.statusword[i];
        }
        status_pub_->publish(status_);
    }
    return state;
}

}  // namespace plem::drive::ros
