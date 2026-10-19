// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_bus.hpp
/// @brief The ROS surface one drive bus keeps whatever writes its commands:
/// `~/status`, `~/rt_events`, `~/info`, `rt_raw` on the node it is given, and
/// the control command service. The drive system interface puts it on a node
/// of its own (named `plem_drive`).

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>

#include <plem_drive_msgs/msg/drive_info.hpp>
#include <plem_drive_msgs/msg/drive_status.hpp>
#include <plem_rt_msgs/msg/rt_event.hpp>
#include <rclcpp/rclcpp.hpp>

#include <plem/core/result.hpp>
#include <plem/drive/drive_host.hpp>
#include <plem/drive/drive_model_file.hpp>
#include <plem/fieldbus/ethercat/controller.hpp>

#include "plem/drive/ros/control_command_service.hpp"
#include "plem/drive/ros/rt_raw_batcher.hpp"
#include "ros/drive_host_adapter.hpp"

namespace plem::drive::ros {

/// The host `spec` describes, on EtherCAT master `master_index`.
/// @throws std::runtime_error with the host's refusal, naming `model_file`.
[[nodiscard]] std::unique_ptr<IDriveHost> build_drive_host(const plem::drive::DriveModelSpecAny& spec,
                                                           const std::string& model_file, unsigned master_index,
                                                           const plem::drive::DriveHostParams& params);

/// @thread_safety The service callbacks and publish_tick() must run on one
/// thread (a single-threaded executor); they share IDriveHost's lifecycle
/// thread with start()/stop(), which their owner calls while that executor is
/// not spinning.
class DriveBus {
  public:
    /// `node` outlives this. `host` is non-null with 1..kMaxAxes axes, as
    /// make_drive_host builds it. `unapproved_motion` = motion opened on an
    /// unapproved model: every real release is logged as bench evidence.
    DriveBus(rclcpp::Node& node, std::unique_ptr<IDriveHost> host, bool unapproved_motion);
    /// Ends and joins the rt_raw drain if stop() did not.
    ~DriveBus();
    DriveBus(const DriveBus&) = delete;
    DriveBus& operator=(const DriveBus&) = delete;

    [[nodiscard]] IDriveHost& host() noexcept {
        return *host_;
    }

    /// Starts the host; once it runs, a thread of this bus drains its RT
    /// samples onto `rt_raw` (the samples' only consumer, no host lifecycle
    /// call). A thread that cannot be created is logged; the host runs on.
    /// Returns once every slave is in OP (the working counter complete); one
    /// that is not within `op_timeout` stops the bus and fails the start.
    [[nodiscard]] plem::core::Result<void> start(std::chrono::milliseconds op_timeout = kOpTimeout);
    /// The stop first (ramp_stop; the command watch stays armed until it is asked
    /// for), then the drain ends, then the host stops (it waits for the stop).
    plem::fieldbus::ethercat::BrakeQuiesceOutcome stop();

    /// Drains the RT events onto `~/rt_events`, takes a snapshot and publishes
    /// `~/status` every tenth call. The snapshot, or nullopt when it was torn.
    std::optional<DriveState> publish_tick();

    /// The control service, served on the bus's node as `~/control_command`
    /// (handle_request is public for direct tests).
    [[nodiscard]] const std::shared_ptr<ControlCommandService>& command_service() const noexcept {
        return command_service_;
    }

    /// `~/status` every tenth publish_tick.
    static constexpr uint8_t kStatusDecimation = 10;
    /// How long start() waits for every slave to reach OP. IgH (1.6.13,
    /// fsm_slave_config.c) configures the slaves one at a time and holds a DC
    /// slave short of OP until its clock is within 10 us of the reference
    /// clock, for up to 5 s: about 5.5 s a slave, 8 slaves at most.
    static constexpr std::chrono::milliseconds kOpTimeout{60'000};

  private:
    [[nodiscard]] plem::core::Result<void> on_release_brake();
    void drain_samples() noexcept;
    void stop_draining();

    /// rt_raw drain wake-up; the RT queue holds seconds of samples even at 5 kHz.
    static constexpr std::chrono::milliseconds kSampleDrainPeriod{10};

    rclcpp::Node& node_;
    std::unique_ptr<IDriveHost> host_;
    size_t axes_ = 0;
    bool unapproved_motion_ = false;
    rclcpp::Publisher<plem_drive_msgs::msg::DriveStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<plem_rt_msgs::msg::RtEvent>::SharedPtr rt_event_pub_;
    rclcpp::Publisher<plem_drive_msgs::msg::DriveInfo>::SharedPtr info_pub_;  ///< kept alive: its one sample is latched
    std::shared_ptr<ControlCommandService> command_service_;
    std::unique_ptr<RtRawBatcher> rt_raw_;
    std::atomic<bool> draining_{false};
    std::thread drain_thread_;
    plem_drive_msgs::msg::DriveStatus status_;  ///< prototype: sizes fixed at construction
    uint8_t status_ticks_ = 0;
};

}  // namespace plem::drive::ros
