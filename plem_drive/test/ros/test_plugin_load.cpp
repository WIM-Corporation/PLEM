// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_plugin_load.cpp
/// @brief pluginlib creates both plugins of this package from the package's
/// plugin descriptions after another plugin that links plem_drive_ros is
/// loaded: the case of a hardware plugin that links the helpers (or
/// subclasses DriveSystemInterface) and runs beside these plugins in one
/// controller manager. The other plugin is the test-only recording host
/// (test prefix, CMakeLists.txt); this package's descriptions are the
/// installed ones (colcon test sources the install).

#include <gtest/gtest.h>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>

namespace {

TEST(PluginLoad, BothPluginsLoadAfterAPluginThatLinksTheHelpers) {
    pluginlib::ClassLoader<hardware_interface::SystemInterface> hardware("hardware_interface",
                                                                         "hardware_interface::SystemInterface");
    pluginlib::ClassLoader<controller_interface::ControllerInterface> controllers(
        "controller_interface", "controller_interface::ControllerInterface");

    auto linking = hardware.createUniqueInstance("plem_drive/test/RecordingDriveSystemInterface");
    ASSERT_NE(linking, nullptr);

    EXPECT_NE(hardware.createUniqueInstance("plem_drive/DriveSystemInterface"), nullptr);
    EXPECT_NE(controllers.createUniqueInstance("plem_drive/StreamController"), nullptr);
}

}  // namespace
