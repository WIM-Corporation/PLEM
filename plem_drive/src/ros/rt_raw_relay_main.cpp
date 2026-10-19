// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file rt_raw_relay_main.cpp
/// @brief `ros2 run plem_drive rt_raw_relay --ros-args -r __ns:=/<bus.name>`:
/// `rt_raw` one message per sample on `rt_raw/samples`.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros/rt_raw_relay.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<plem::drive::ros::RtRawRelay>());
    rclcpp::shutdown();
    return 0;
}
