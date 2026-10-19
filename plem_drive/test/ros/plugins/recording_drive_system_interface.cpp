// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file recording_drive_system_interface.cpp
/// @brief Test-only hardware plugin: DriveSystemInterface over a simulated
/// host (each csp axis is where it was last commanded, each csv axis moves
/// at its last commanded speed while its position stays, a release enables
/// every axis, a stop engages them) that appends each lifecycle call to the
/// file named by the hardware parameter `record_file` (`op_after_ms`: how long
/// after start every slave is in OP). A test sees what the
/// controller manager and the Python client made the bus do, without a bus.
/// Built only with the tests, never installed.

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "mock_drive_host.hpp"
#include "ros/drive_system_interface.hpp"

namespace plem::drive::ros::test {

class RecordingHost : public MockDriveHost {
  public:
    RecordingHost(std::vector<std::string> names, std::string file)
        : MockDriveHost(std::move(names)), file_(std::move(file)) {
        state.brake_engaged = 0xff;
    }
    Result<void> start() override {
        record("start");
        op_at_ = std::chrono::steady_clock::now() + op_after;
        return MockDriveHost::start();
    }
    Result<void> enable() override {
        record("enable");
        std::lock_guard lock(mutex_);
        state.operation_enabled = true;
        state.brake_engaged = 0;
        first_after_release_ = true;
        return Result<void>::ok();
    }
    /// A stop decelerates past the last command: the axes come to rest 0.05 rad
    /// on, 100 ms later, after the trajectory controller took its hold.
    void ramp_stop() override {
        record("ramp_stop");
        std::lock_guard lock(mutex_);
        if (state.operation_enabled) {
            drift_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
        }
        state.operation_enabled = false;
        state.brake_engaged = static_cast<uint8_t>((1u << axis_names().size()) - 1u);
    }
    plem::fieldbus::ethercat::BrakeQuiesceOutcome stop() override {
        record("stop");
        return MockDriveHost::stop();
    }
    [[nodiscard]] bool write(std::span<const double> command, std::span<const double> rate,
                             int64_t /*sampled_ns*/) noexcept override {
        std::lock_guard lock(mutex_);
        if (drift_at_ && std::chrono::steady_clock::now() >= *drift_at_) {
            for (size_t i = 0; i < axis_names().size(); ++i) {
                state.position[i] += 0.05;
            }
            drift_at_.reset();
        }
        for (size_t i = 0; i < command.size(); ++i) {
            // The bus's one-cycle step check, for the first command after a release.
            if (modes[i] == 8 && state.operation_enabled && first_after_release_ &&
                std::abs(command[i] - state.position[i]) > 0.01) {
                record("jump");
            }
            if (modes[i] == 8 && state.operation_enabled) {
                state.velocity[i] = rate.empty() ? 0.0 : rate[i];
                state.position[i] = command[i];
            }
            if (modes[i] == 9 && state.operation_enabled) {
                state.velocity[i] = command[i];
            }
        }
        first_after_release_ = first_after_release_ && !state.operation_enabled;
        return true;
    }
    [[nodiscard]] bool read(DriveState& out) const noexcept override {
        std::lock_guard lock(mutex_);
        out = state;
        out.process_data_complete = std::chrono::steady_clock::now() >= op_at_;
        return true;
    }
    /// How long after start every slave is in OP (hardware parameter op_after_ms).
    std::chrono::milliseconds op_after{0};

  private:
    void record(const char* call) {
        std::ofstream(file_, std::ios::app) << call << '\n';
    }
    std::string file_;
    mutable std::mutex mutex_;
    bool first_after_release_ = false;
    std::optional<std::chrono::steady_clock::time_point> drift_at_;
    std::chrono::steady_clock::time_point op_at_{};
};

class RecordingDriveSystemInterface : public DriveSystemInterface {
  public:
    RecordingDriveSystemInterface()
        : DriveSystemInterface([](const hardware_interface::HardwareInfo& info) {
              std::vector<std::string> names;
              std::vector<int8_t> modes;
              for (const auto& joint : info.joints) {
                  names.push_back(joint.name);
                  const auto& command = joint.command_interfaces.front().name;
                  modes.push_back(joint.command_interfaces.size() == 2 ? 8 : command == "velocity" ? 9 : 10);
              }
              auto host = std::make_unique<RecordingHost>(names, info.hardware_parameters.at("record_file"));
              host->modes = modes;
              if (const auto it = info.hardware_parameters.find("op_after_ms"); it != info.hardware_parameters.end()) {
                  host->op_after = std::chrono::milliseconds(std::stoi(it->second));
              }
              host->digests.assign(1, "");
              return Built{std::move(host), false};
          }) {}
};

}  // namespace plem::drive::ros::test

PLUGINLIB_EXPORT_CLASS(plem::drive::ros::test::RecordingDriveSystemInterface, hardware_interface::SystemInterface)
