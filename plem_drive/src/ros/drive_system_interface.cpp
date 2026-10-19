// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "ros/drive_system_interface.hpp"

#include <sched.h>

#include <bit>
#include <charconv>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fmt/format.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <plem/core/timing/timespec.hpp>
#include <plem/fieldbus/ethercat/state_machine.hpp>

namespace plem::drive::ros {

namespace {

std::string unconfirmed_brakes_message(const plem::fieldbus::ethercat::BrakeQuiesceOutcome& outcome) {
    std::string words;
    unsigned faulted = 0;
    for (size_t i = 0; i < outcome.statuswords.size(); ++i) {
        if ((outcome.all_joints_mask & (1U << i)) == 0) {
            continue;
        }
        words += fmt::format(" j{}=0x{:04x}", i, outcome.statuswords[i]);
        if ((outcome.timeout_mask & (1U << i)) != 0 &&
            plem::fieldbus::ethercat::to_servo_state(outcome.statuswords[i]) ==
                plem::fieldbus::ethercat::ServoState::FAULT) {
            faulted |= 1U << i;
        }
    }
    std::string line = fmt::format(
        "brakes were NOT confirmed engaged on stop (engaged mask 0x{:02x} of 0x{:02x}, timeout mask 0x{:02x}, "
        "statuswords{}) — treat the bus as holding an unknown brake state",
        outcome.engaged_mask, outcome.all_joints_mask, outcome.timeout_mask, words);
    if (faulted != 0) {
        line += fmt::format(
            "; joints 0x{:02x} were still in Fault at the end of the stop: PLEM could not walk them through "
            "their disable sequence, so those brakes were not confirmed. Whether they closed depends on the "
            "drive's own servo-off / alarm behaviour, which PLEM does not observe. Clear the fault "
            "(reset_error, then wait for the status to leave Fault) before stopping",
            faulted);
    }
    return line;
}

std::string confirmed_brakes_message(const plem::fieldbus::ethercat::BrakeQuiesceOutcome& outcome) {
    return fmt::format("brakes confirmed engaged on stop (engaged mask 0x{:02x} of 0x{:02x})", outcome.engaged_mask,
                       outcome.all_joints_mask);
}

std::string never_released_brakes_message(const plem::fieldbus::ethercat::BrakeQuiesceOutcome& outcome) {
    return fmt::format("brakes were never released this run — the stop observed no brake ({} joints, none released)",
                       std::popcount(static_cast<unsigned>(outcome.all_joints_mask)));
}

}  // namespace

StopBrakesLine stop_brakes_line(const plem::fieldbus::ethercat::BrakeQuiesceOutcome& outcome) {
    if (outcome.never_released) {
        return {false, never_released_brakes_message(outcome)};
    }
    if (outcome.fully_engaged()) {
        return {false, confirmed_brakes_message(outcome)};
    }
    return {true, unconfirmed_brakes_message(outcome)};
}

namespace {

using hardware_interface::CallbackReturn;
constexpr int8_t kCsp = 8;
constexpr int8_t kCsv = 9;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

rclcpp::Logger logger() {
    return rclcpp::get_logger("DriveSystemInterface");
}

/// The command interfaces an axis of mode `mode` takes, in order.
std::vector<std::string> mode_commands(int8_t mode) {
    if (mode == kCsp) {
        return {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
    }
    if (mode == kCsv) {
        return {hardware_interface::HW_IF_VELOCITY};
    }
    return {hardware_interface::HW_IF_EFFORT};
}

std::string param(const hardware_interface::HardwareInfo& info, const std::string& key, const std::string& fallback) {
    const auto it = info.hardware_parameters.find(key);
    return it == info.hardware_parameters.end() ? fallback : it->second;
}

DriveSystemInterface::Built build_from_file(const hardware_interface::HardwareInfo& info) {
    const std::string model_file = param(info, "model_file", "");
    if (model_file.empty()) {
        throw std::runtime_error(
            "hardware parameter model_file is not set: it names the drive model file this bus runs (plem drive run "
            "sets it)");
    }
    const unsigned master = master_index(param(info, "master_index", "0"));
    const auto cpu = rt_cpu_affinity(param(info, "rt_cpu", ""));
    const bool unapproved = param(info, "unapproved", "false") == "true";
    // Bench mode (--unapproved) loads any file as unapproved, its approval record unused.
    auto loaded = plem::drive::load_drive_model_file(model_file, unapproved ? plem::drive::ApprovalRecord::IGNORE
                                                                            : plem::drive::ApprovalRecord::USE);
    if (!loaded) {
        throw std::runtime_error(loaded.message());
    }
    const auto spec = std::move(loaded).value();
    if (unapproved) {
        RCLCPP_WARN(logger(),
                    "BENCH MODE — motion on an unapproved model (hardware parameter unapproved=true, which plem drive "
                    "run --unapproved sets); this run's "
                    "evidence is bench evidence only");
    }
    return {build_drive_host(spec, model_file, master,
                             plem::drive::DriveHostParams{.cpu_affinity = cpu, .bench_unapproved = unapproved}),
            unapproved};
}

}  // namespace

unsigned master_index(const std::string& value) {
    unsigned index = 0;
    const char* end = value.data() + value.size();
    if (const auto [ptr, ec] = std::from_chars(value.data(), end, index); ec != std::errc() || ptr != end) {
        throw std::invalid_argument("master_index must be an EtherCAT master index, a whole number (got '" + value +
                                    "') — plem drive run sets it from --master N");
    }
    return index;
}

std::optional<plem::core::threading::CpuAffinity> rt_cpu_affinity(const std::string& value) {
    if (value.empty()) {
        return std::nullopt;
    }
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    size_t used = 0;
    int cpu = -1;
    try {
        cpu = std::stoi(value, &used);
    } catch (const std::exception&) {
        used = 0;
    }
    if (used != value.size() || cpu < 0 || cpu >= CPU_SETSIZE || sched_getaffinity(0, sizeof(allowed), &allowed) != 0 ||
        !CPU_ISSET(cpu, &allowed)) {
        throw std::invalid_argument("rt_cpu must be a CPU this process may run on (got '" + value +
                                    "'), or unset for the last isolated CPU — the isolated CPUs are listed in "
                                    "/sys/devices/system/cpu/isolated");
    }
    return plem::core::threading::CpuAffinity{{cpu}};
}

DriveSystemInterface::DriveSystemInterface() : DriveSystemInterface(build_from_file) {}

DriveSystemInterface::DriveSystemInterface(HostFactory factory) : factory_(std::move(factory)) {}

DriveSystemInterface::~DriveSystemInterface() {
    stop();
}

CallbackReturn DriveSystemInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    Built built;
    try {
        built = factory_(info_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "%s", e.what());
        return CallbackReturn::ERROR;
    }
    const auto names = built.host->axis_names();
    const auto modes = built.host->modes_of_operation();
    if (info_.joints.size() != names.size()) {
        RCLCPP_ERROR(logger(),
                     "the robot description has %zu joints, the drive bus %zu axes — one joint per axis, "
                     "in the file's axes[] order (plem drive run writes it so)",
                     info_.joints.size(), names.size());
        return CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < names.size(); ++i) {
        const auto& joint = info_.joints[i];
        if (joint.name != names[i]) {
            RCLCPP_ERROR(logger(),
                         "joint %zu is '%s', the bus's axis %zu is '%s' — one joint per axis, in the "
                         "file's axes[] order",
                         i, joint.name.c_str(), i, names[i].c_str());
            return CallbackReturn::ERROR;
        }
        std::vector<std::string> commands;
        for (const auto& c : joint.command_interfaces) {
            commands.push_back(c.name);
        }
        if (commands != mode_commands(modes[i])) {
            std::string expected;
            for (const auto& c : mode_commands(modes[i])) {
                expected += (expected.empty() ? "" : ", ") + c;
            }
            RCLCPP_ERROR(logger(),
                         "joint '%s' (mode %d) takes the command interfaces [%s] — its mode's, in that "
                         "order",
                         joint.name.c_str(), modes[i], expected.c_str());
            return CallbackReturn::ERROR;
        }
    }
    modes_.assign(modes.begin(), modes.end());
    const size_t n = names.size();
    position_.assign(n, 0.0);
    velocity_.assign(n, 0.0);
    effort_.assign(n, 0.0);
    // NaN until a controller writes: no stamp, the bus holds.
    position_cmd_.assign(n, kNaN);
    velocity_cmd_.assign(n, kNaN);
    effort_cmd_.assign(n, kNaN);
    command_.assign(n, 0.0);
    rate_.assign(n, 0.0);

    node_ = std::make_shared<rclcpp::Node>("plem_drive");
    bus_ = std::make_unique<DriveBus>(*node_, std::move(built.host), built.unapproved_motion);
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DriveSystemInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> out;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& name = info_.joints[i].name;
        out.emplace_back(name, hardware_interface::HW_IF_POSITION, &position_[i]);
        out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
        out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_[i]);
    }
    return out;
}

std::vector<hardware_interface::CommandInterface> DriveSystemInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> out;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& name = info_.joints[i].name;
        if (modes_[i] == kCsp) {
            out.emplace_back(name, hardware_interface::HW_IF_POSITION, &position_cmd_[i]);
            out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_cmd_[i]);
        } else if (modes_[i] == kCsv) {
            out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_cmd_[i]);
        } else {
            out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_cmd_[i]);
        }
    }
    return out;
}

CallbackReturn DriveSystemInterface::on_activate(const rclcpp_lifecycle::State&) {
    if (running_) {
        return CallbackReturn::SUCCESS;
    }
    if (const auto started = bus_->start(); !started) {
        RCLCPP_ERROR(logger(), "the drive bus did not start: %s", started.message().c_str());
        return CallbackReturn::ERROR;
    }
    try {
        start_spinning();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger(), "the drive bus's service thread did not start: %s", e.what());
        executor_.reset();  // the thread is the last step: a throw leaves none to join
        tick_.reset();
        RCLCPP_ERROR(logger(), "%s", stop_brakes_line(bus_->stop()).text.c_str());
        return CallbackReturn::ERROR;
    }
    running_ = true;
    return CallbackReturn::SUCCESS;
}

void DriveSystemInterface::start_spinning() {
    tick_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this] { (void)bus_->publish_tick(); });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spinning_.store(true, std::memory_order_release);
    // spin_once in a loop, not spin(): a cancel() that lands before spin() starts is lost.
    spinner_ = std::thread([this] {
        while (spinning_.load(std::memory_order_acquire)) {
            executor_->spin_once(std::chrono::milliseconds(10));
        }
    });
}

void DriveSystemInterface::stop() {
    if (!running_) {
        return;
    }
    running_ = false;
    // The service and the tick share the host's lifecycle thread: they end first.
    spinning_.store(false, std::memory_order_release);
    spinner_.join();
    executor_.reset();
    tick_.reset();
    // The outcome is logged both ways: the event drain (tick_) has ended, so
    // this line is the run's only report of the brake state.
    if (const StopBrakesLine line = stop_brakes_line(bus_->stop()); line.error) {
        RCLCPP_ERROR(logger(), "%s", line.text.c_str());
    } else {
        RCLCPP_INFO(logger(), "%s", line.text.c_str());
    }
}

CallbackReturn DriveSystemInterface::on_deactivate(const rclcpp_lifecycle::State&) {
    stop();
    return CallbackReturn::SUCCESS;
}

CallbackReturn DriveSystemInterface::on_shutdown(const rclcpp_lifecycle::State&) {
    stop();
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DriveSystemInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
    sampled_ns_ = plem::core::now_ns();  // the instant this cycle's controllers plan from
    DriveState state{};
    if (bus_->host().read(state)) {  // a torn snapshot keeps the last
        for (size_t i = 0; i < position_.size(); ++i) {
            position_[i] = state.position[i];
            velocity_[i] = state.velocity[i];
            effort_[i] = state.torque[i];
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DriveSystemInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    if (sampled_ns_ == 0) {
        return hardware_interface::return_type::OK;  // no read yet: nothing planned from the bus
    }
    for (size_t i = 0; i < command_.size(); ++i) {
        const bool csp = modes_[i] == kCsp;
        command_[i] = csp ? position_cmd_[i] : modes_[i] == kCsv ? velocity_cmd_[i] : effort_cmd_[i];
        if (std::isnan(command_[i])) {
            return hardware_interface::return_type::OK;  // an axis without a command: nothing this cycle
        }
        rate_[i] = csp && !std::isnan(velocity_cmd_[i]) ? velocity_cmd_[i] : 0.0;
    }
    (void)bus_->host().write(command_, rate_, sampled_ns_);  // a refusal (inf) leaves the stamp to age
    return hardware_interface::return_type::OK;
}

}  // namespace plem::drive::ros
