// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file stream_controller.hpp
/// @brief The stream lane of a drive bus (`plem drive run` starts it for its
/// csv and cst axes): forwards `~/commands` (std_msgs/Float64MultiArray, one
/// value per joint in `joints` order) to each joint's one command interface,
/// and lets go of a stream that stops: 50 ms after the last message every
/// value becomes NaN, the drive system interface then writes nothing, and the
/// bus loop's own 50 ms command watch latches the protective stop (about 100 ms
/// after the last message). Before the first message a velocity joint is
/// commanded 0 and a torque joint NaN (a torque axis must stream before its
/// release). Deactivating the controller writes NaN the same way, so a stream
/// left behind cannot keep the axes moving, unless every command interface is
/// locked at that moment (Jazzy), which fails the deactivation with an error
/// log. On a bus with no torque axis the watch runs only while the bus is
/// released.

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller_interface.hpp>
#include <plem/drive/drive_host.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace plem::drive::ros {

/// One joint's command this cycle: `value` while the stream is fresh, NaN once
/// its newest message is older than plem::drive::kCommandStaleTimeoutNs, and
/// before any message 0 on a velocity joint (`velocity`) or NaN otherwise.
[[nodiscard]] constexpr double stream_command(bool received, int64_t age_ns, double value, bool velocity) noexcept {
    constexpr double kNone = std::numeric_limits<double>::quiet_NaN();
    if (!received) {
        return velocity ? 0.0 : kNone;
    }
    return age_ns > plem::drive::kCommandStaleTimeoutNs ? kNone : value;
}

class StreamController final : public controller_interface::ControllerInterface {
  public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:
    struct Sample {
        std::vector<double> values;
        int64_t received_ns = 0;  ///< steady clock; 0 = none since activation
    };

    std::vector<std::string> joints_;
    std::vector<std::string> interfaces_;
    realtime_tools::RealtimeBuffer<Sample> latest_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

}  // namespace plem::drive::ros
