// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_system_interface.hpp
/// @brief One drive bus as ros2_control hardware (`plem drive run` names it in
/// the robot description it writes). Each joint is an axis of the drive model
/// file, in its axes[] order, with its mode's command interfaces (csp:
/// position and velocity (the planned speed, by which the bus loop advances a
/// sample it reads late), csv: velocity, cst: effort) and position, velocity
/// and effort states. Hardware parameters: `model_file` (required),
/// `master_index` (default 0), `unapproved` (true = the bench bootstrap: an
/// unapproved file may open motion, every release is logged as bench evidence),
/// `rt_cpu` (the CPU the bus loop is pinned to; unset = the last isolated CPU).
///
/// A cycle writes the bus only when every axis has a command: a controller not
/// yet writing, or a stream that went silent (StreamController), leaves NaN and
/// nothing is stamped; the bus holds, and a released bus's 50 ms command watch
/// sees the silence. Beside the controllers the bus keeps its own ROS surface
/// (DriveBus: status, events, rt_raw, the control command service) on a thread
/// of its own; deactivation stops the axes (DriveHost::ramp_stop) before it lets go.
/// The host is one-shot: once deactivated the hardware does not activate again
/// (restart the controller manager).

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros/drive_bus.hpp"
#include "ros/drive_host_adapter.hpp"

namespace plem::drive::ros {

/// The hardware parameter `rt_cpu`: empty = nullopt (the bus loop's default,
/// the last isolated CPU), else a CPU this process may run on (its affinity
/// mask; whether the CPU is isolated is not checked). Anything else throws
/// std::invalid_argument naming what to pass.
std::optional<plem::core::threading::CpuAffinity> rt_cpu_affinity(const std::string& value);

/// The hardware parameter `master_index`: the EtherCAT master's index, a
/// whole number (decimal digits only, in unsigned range). Anything else throws
/// std::invalid_argument naming what to pass.
unsigned master_index(const std::string& value);

/// The stop line for `outcome` and whether it is an error: never released →
/// an info line that carries neither bench marker (nothing was observed;
/// a bench run that never released fails its stop-quiesce check), whatever
/// the masks say; every joint confirmed → the info line the bench requires
/// (bench.py CONFIRMED_BRAKES), with the engaged mask; else the error line
/// (bench.py UNCONFIRMED_BRAKES) with the masks, each configured joint's
/// statusword and, for a joint that timed out still in Fault, why PLEM could
/// not confirm its brake and what to do.
struct StopBrakesLine {
    bool error;
    std::string text;
};
StopBrakesLine stop_brakes_line(const plem::fieldbus::ethercat::BrakeQuiesceOutcome& outcome);

class DriveSystemInterface : public hardware_interface::SystemInterface {
  public:
    /// The host and whether motion is opened as the bench bootstrap.
    struct Built {
        std::unique_ptr<IDriveHost> host;
        bool unapproved_motion = false;
    };
    using HostFactory = std::function<Built(const hardware_interface::HardwareInfo&)>;

    /// The plugin: builds the host from the hardware parameters.
    DriveSystemInterface();
    ~DriveSystemInterface() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  protected:
    /// Test seam: a host from elsewhere (the bus stays out of the test).
    explicit DriveSystemInterface(HostFactory factory);

  private:
    /// The host started: the bus's ROS surface spins on its own thread.
    void start_spinning();
    /// Ends the spin thread, then stops the host (the stop first), once.
    void stop();

    HostFactory factory_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<DriveBus> bus_;
    rclcpp::TimerBase::SharedPtr tick_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spinner_;
    std::atomic<bool> spinning_{false};
    bool running_ = false;

    std::vector<int8_t> modes_;                                     ///< per axis, 0x6060 (8 csp, 9 csv, 10 cst)
    std::vector<double> position_, velocity_, effort_;              ///< states
    std::vector<double> position_cmd_, velocity_cmd_, effort_cmd_;  ///< commands
    std::vector<double> command_, rate_;                            ///< one write's arrays
    int64_t sampled_ns_ = 0;  ///< this cycle's read (CLOCK_MONOTONIC): the stamp of its write
};

}  // namespace plem::drive::ros
