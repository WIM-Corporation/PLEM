// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "ros/stream_controller.hpp"

#include <chrono>
#include <limits>
#include <type_traits>

namespace plem::drive::ros {
namespace {

int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/// set_value returns false on Jazzy when the handle's lock stayed busy, and nothing on Humble.
template <class Handle>
bool set_command(Handle& handle, double value) {
    if constexpr (std::is_void_v<decltype(handle.set_value(value))>) {
        handle.set_value(value);
        return true;
    } else {
        return handle.set_value(value);
    }
}

}  // namespace

controller_interface::CallbackReturn StreamController::on_init() {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<std::string>>("interface_names", {});
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StreamController::on_configure(const rclcpp_lifecycle::State&) {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    interfaces_ = get_node()->get_parameter("interface_names").as_string_array();
    if (joints_.empty() || joints_.size() != interfaces_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "joints (%zu) and interface_names (%zu) must be the same non-empty length — one command "
                     "interface per joint (velocity for a csv axis, effort for a cst axis)",
                     joints_.size(), interfaces_.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const std_msgs::msg::Float64MultiArray& msg) {
            if (msg.data.size() != joints_.size()) {
                RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                      "~/commands dropped: %zu values for %zu joints (order: the joints parameter)",
                                      msg.data.size(), joints_.size());
                return;
            }
            latest_.writeFromNonRT(Sample{msg.data, steady_ns()});
        });
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StreamController::on_activate(const rclcpp_lifecycle::State&) {
    // A stream belongs to the activation it arrived in.
    latest_.writeFromNonRT(Sample{});
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StreamController::on_deactivate(const rclcpp_lifecycle::State&) {
    // The hardware sends whatever a command interface holds, stamped with the
    // cycle's read, so a command left here would keep the axes going with no
    // stream and the bus's 50 ms command watch would never see a silence. NaN
    // is no command, and a cycle writes the bus only when every axis has one:
    // a single NaN stops the bus writes, and the watch then latches the bus's
    // protective stop. So every handle is tried, and only a deactivation that
    // could clear none of them (on Jazzy each handle's lock was busy) fails.
    bool cleared = false;
    for (auto& handle : command_interfaces_) {
        cleared = set_command(handle, std::numeric_limits<double>::quiet_NaN()) || cleared;
    }
    if (!cleared) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "every command interface was busy at deactivation, so the last stream commands stay in place. "
                     "Stop the bus (Bus.stop() or Ctrl-C in plem drive run).");
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration StreamController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config{controller_interface::interface_configuration_type::INDIVIDUAL,
                                                        {}};
    for (size_t i = 0; i < joints_.size(); ++i) {
        config.names.push_back(joints_[i] + "/" + interfaces_[i]);
    }
    return config;
}

controller_interface::InterfaceConfiguration StreamController::state_interface_configuration() const {
    return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::return_type StreamController::update(const rclcpp::Time&, const rclcpp::Duration&) {
    const Sample& sample = *latest_.readFromRT();
    const bool received = sample.received_ns != 0;
    const int64_t age_ns = steady_ns() - sample.received_ns;
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        // On Jazzy set_value returns false when another thread held the handle's
        // lock through every try: a skipped write, not an error. The joint keeps
        // its previous command for every cycle the handle stays busy; the loaned
        // interface counts the misses and warns with the totals when the
        // controller releases it. The hardware write re-stamps a held value as
        // fresh, so the bus's 50 ms command watch does not see it.
        (void)command_interfaces_[i].set_value(
            stream_command(received, age_ns, received ? sample.values[i] : 0.0, interfaces_[i] == "velocity"));
    }
    return controller_interface::return_type::OK;
}

}  // namespace plem::drive::ros
