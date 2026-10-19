// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file plugins.cpp
/// @brief The pluginlib registrations of this package, alone in their own
/// library (plem_drive_plugins, the library the plugin descriptions
/// name). class_loader credits a factory to the library being loaded when the
/// factory registers, so a registration inside the linkable plem_drive_ros
/// would be credited to whichever library or program links it first, and
/// pluginlib could then no longer create the plugin from its own library.
/// Linking plem_drive_ros registers nothing.

#include <pluginlib/class_list_macros.hpp>

#include "ros/drive_system_interface.hpp"
#include "ros/stream_controller.hpp"

PLUGINLIB_EXPORT_CLASS(plem::drive::ros::DriveSystemInterface, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(plem::drive::ros::StreamController, controller_interface::ControllerInterface)
