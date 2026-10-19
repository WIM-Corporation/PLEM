// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_system_interface.cpp
/// @brief The drive bus as ros2_control hardware: which joints and interfaces
/// it accepts and exports, what a cycle's read and write move between the
/// interfaces and the host, and that activation starts the host while
/// deactivation stops it (the stop first). A mock host stands in for the bus.

#include <gtest/gtest.h>

#include <sched.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rcutils/logging.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <plem/core/timing/timespec.hpp>
#include <rclcpp/rclcpp.hpp>

#include "drive_bench_fixture.hpp"
#include "mock_drive_host.hpp"
#include "ros/drive_system_interface.hpp"

namespace plem::drive::ros {
namespace {

using hardware_interface::CallbackReturn;
using test::MockDriveHost;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// One joint as `plem drive run` describes it: its mode's command interfaces, all three states.
hardware_interface::InterfaceInfo named_interface(const std::string& name) {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    return info;
}

hardware_interface::ComponentInfo joint(const std::string& name, std::vector<std::string> commands) {
    hardware_interface::ComponentInfo info;
    info.name = name;
    info.type = "joint";
    for (auto& c : commands) {
        info.command_interfaces.push_back(named_interface(c));
    }
    for (const char* s : {"position", "velocity", "effort"}) {
        info.state_interfaces.push_back(named_interface(s));
    }
    return info;
}

class TestableInterface : public DriveSystemInterface {
  public:
    explicit TestableInterface(MockDriveHost*& seen)
        : DriveSystemInterface([&seen](const hardware_interface::HardwareInfo&) {
              auto host = std::make_unique<MockDriveHost>(std::vector<std::string>{"p", "v", "t"});
              host->modes = {8, 9, 10};  // CSP, CSV, CST
              seen = host.get();
              return Built{std::move(host), false};
          }) {}
};

class DriveSystemInterfaceTest : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    hardware_interface::HardwareInfo info(std::vector<hardware_interface::ComponentInfo> joints) {
        hardware_interface::HardwareInfo i;
        i.name = "drive_bus";
        i.type = "system";
#if PLEM_ROS2_CONTROL_JAZZY
        i.hardware_plugin_name = "plem_drive/DriveSystemInterface";
#else
        i.hardware_class_type = "plem_drive/DriveSystemInterface";
#endif
        i.joints = std::move(joints);
        return i;
    }
    hardware_interface::HardwareInfo good() {
        return info({joint("p", {"position", "velocity"}), joint("v", {"velocity"}), joint("t", {"effort"})});
    }
    /// The interface handle `<joint>/<name>` among `handles`.
    template <typename Handles>
    static auto& find(Handles& handles, const std::string& joint, const std::string& name) {
        for (auto& h : handles) {
            if (h.get_prefix_name() == joint && h.get_interface_name() == name) {
                return h;
            }
        }
        throw std::out_of_range(joint + "/" + name);
    }

    /// Writes a command as a controller would; a handle that refuses it fails the test.
    static void set(hardware_interface::CommandInterface& handle, double value) {
#if PLEM_ROS2_CONTROL_JAZZY
        ASSERT_TRUE(handle.set_value(value)) << handle.get_name();
#else
        handle.set_value(value);
#endif
    }

    MockDriveHost* host_ = nullptr;
    TestableInterface si_{host_};
};

TEST_F(DriveSystemInterfaceTest, EachJointIsAnAxisWithItsModesInterfaces) {
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    const auto commands = si_.export_command_interfaces();
    std::vector<std::string> names;
    for (const auto& c : commands) {
        names.push_back(c.get_name());
    }
    EXPECT_EQ(names, (std::vector<std::string>{"p/position", "p/velocity", "v/velocity", "t/effort"}));
    EXPECT_EQ(si_.export_state_interfaces().size(), 9u);
}

TEST_F(DriveSystemInterfaceTest, AJointThatIsNotTheBusAxisInOrderIsRefused) {
    EXPECT_EQ(
        si_.on_init(info({joint("v", {"velocity"}), joint("p", {"position", "velocity"}), joint("t", {"effort"})})),
        CallbackReturn::ERROR);
    EXPECT_EQ(si_.on_init(info({joint("p", {"position", "velocity"}), joint("v", {"velocity"})})),
              CallbackReturn::ERROR)
        << "an axis left out";
}

TEST_F(DriveSystemInterfaceTest, AnInterfaceOtherThanTheAxisModesIsRefused) {
    EXPECT_EQ(si_.on_init(info({joint("p", {"velocity"}), joint("v", {"velocity"}), joint("t", {"effort"})})),
              CallbackReturn::ERROR)
        << "a csp axis takes position (and its velocity)";
    EXPECT_EQ(
        si_.on_init(info({joint("p", {"position", "velocity"}), joint("v", {"velocity"}), joint("t", {"position"})})),
        CallbackReturn::ERROR);
}

TEST_F(DriveSystemInterfaceTest, AWriteSendsEachAxisItsModesValueAndTheCspRate) {
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    auto commands = si_.export_command_interfaces();
    set(find(commands, "p", "position"), 0.25);
    set(find(commands, "p", "velocity"), 1.5);
    set(find(commands, "v", "velocity"), -0.5);
    set(find(commands, "t", "effort"), 3.0);
    ASSERT_EQ(si_.read(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    ASSERT_EQ(si_.write(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    EXPECT_EQ(host_->last_write, (std::vector<double>{0.25, -0.5, 3.0}));
    EXPECT_EQ(host_->last_rate, (std::vector<double>{1.5, 0.0, 0.0}));
}

TEST_F(DriveSystemInterfaceTest, AWriteIsStampedWithItsCyclesRead) {
    // The controllers plan from the cycle's read (the trajectory sampled at its
    // start); the bus advances a csp value by its rate from that stamp. A stamp
    // taken at write() would carry the update's own jitter into every target.
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    auto commands = si_.export_command_interfaces();
    set(find(commands, "p", "position"), 0.25);
    set(find(commands, "v", "velocity"), 0.0);
    set(find(commands, "t", "effort"), 0.0);
    const int64_t before_read = plem::core::now_ns();
    ASSERT_EQ(si_.read(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    const int64_t after_read = plem::core::now_ns();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    ASSERT_EQ(si_.write(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    EXPECT_GE(host_->last_sampled_ns, before_read);
    EXPECT_LE(host_->last_sampled_ns, after_read);
}

TEST_F(DriveSystemInterfaceTest, AWriteBeforeAnyReadWritesNothing) {
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    auto commands = si_.export_command_interfaces();
    set(find(commands, "p", "position"), 0.25);
    set(find(commands, "v", "velocity"), 0.0);
    set(find(commands, "t", "effort"), 0.0);
    ASSERT_EQ(si_.write(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    EXPECT_EQ(host_->write_calls, 0) << "no read yet: nothing to stamp the command with";
}

TEST_F(DriveSystemInterfaceTest, ACycleWithAnAxisWithoutACommandWritesNothing) {
    // No controller has written yet (NaN), or a stream went silent: no stamp, so
    // the bus holds, and a released axis's 50 ms watch sees the silence.
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    auto commands = si_.export_command_interfaces();
    set(find(commands, "p", "position"), 0.25);
    set(find(commands, "v", "velocity"), 0.0);
    set(find(commands, "t", "effort"), kNaN);
    ASSERT_EQ(si_.read(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    ASSERT_EQ(si_.write(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    EXPECT_EQ(host_->write_calls, 0);
    set(find(commands, "t", "effort"), 1.0);
    ASSERT_EQ(si_.write(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    EXPECT_EQ(host_->write_calls, 1);
    EXPECT_EQ(host_->last_rate[0], 0.0) << "a csp velocity left NaN (no trajectory controller) is no rate";
}

TEST_F(DriveSystemInterfaceTest, AReadExportsTheHostsActuals) {
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    auto states = si_.export_state_interfaces();
    host_->state.position = {0.1, 0.2, 0.3};
    host_->state.velocity = {1.0, 2.0, 3.0};
    host_->state.torque = {-1.0, -2.0, -3.0};
    ASSERT_EQ(si_.read(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)), hardware_interface::return_type::OK);
    EXPECT_EQ(find(states, "v", "position").get_value(), 0.2);
    EXPECT_EQ(find(states, "t", "velocity").get_value(), 3.0);
    EXPECT_EQ(find(states, "p", "effort").get_value(), -1.0);
}

TEST_F(DriveSystemInterfaceTest, ActivationStartsTheHostAndDeactivationStopsItStopFirst) {
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
    ASSERT_EQ(si_.on_activate(active), CallbackReturn::SUCCESS);
    host_->calls.clear();
    ASSERT_EQ(si_.on_deactivate(active), CallbackReturn::SUCCESS);
    EXPECT_EQ(host_->calls, (std::vector<std::string>{"ramp_stop", "stop"}));
    host_->calls.clear();
    EXPECT_EQ(si_.on_shutdown(active), CallbackReturn::SUCCESS);
    EXPECT_TRUE(host_->calls.empty()) << "stopped once";
}

TEST_F(DriveSystemInterfaceTest, AHostThatRefusesToStartFailsTheActivation) {
    ASSERT_EQ(si_.on_init(good()), CallbackReturn::SUCCESS);
    host_->start_result = plem::core::Result<void>::error("bus refused");
    rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    EXPECT_EQ(si_.on_activate(inactive), CallbackReturn::ERROR);
}

// The plugin as pluginlib makes it: the default constructor builds the host
// from the file the hardware parameters name. A master index no machine has
// makes every build fail at the bus, after the file and the approval are checked.

std::vector<std::string>* g_log = nullptr;

void capture_log(const rcutils_log_location_t*, int severity, const char*, rcutils_time_point_value_t,
                 const char* format, va_list* args) {
    char text[4096];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(text, sizeof(text), format, copy);
    va_end(copy);
    const char* level = severity == RCUTILS_LOG_SEVERITY_ERROR  ? "ERROR "
                        : severity == RCUTILS_LOG_SEVERITY_WARN ? "WARN "
                                                                : "";
    g_log->push_back(level + std::string(text));
}

class FileBuiltInterfaceTest : public DriveSystemInterfaceTest {
  protected:
    void SetUp() override {
        g_log = &log_;
        previous_ = rcutils_logging_get_output_handler();
        rcutils_logging_set_output_handler(capture_log);
    }
    void TearDown() override {
        rcutils_logging_set_output_handler(previous_);
        g_log = nullptr;
    }

    /// The parameters plem drive run writes, for the synthetic three-axis file at `model_file`.
    hardware_interface::HardwareInfo params(const std::string& model_file, const std::string& unapproved) {
        auto i = info({});
        if (!model_file.empty()) {
            i.hardware_parameters["model_file"] = model_file;
        }
        i.hardware_parameters["master_index"] = "4000000000";
        if (!unapproved.empty()) {
            i.hardware_parameters["unapproved"] = unapproved;
        }
        return i;
    }

    /// The synthetic file claiming an approval this build cannot honor (its digests are not the file's).
    std::string stale_approved_file() {
        std::ifstream in(DRIVE_EXAMPLE_YAML);
        std::string yaml((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const size_t at = yaml.find(plem::drive::test::kUnapprovedBlock);
        if (at == std::string::npos) {
            ADD_FAILURE() << "the synthetic file no longer ends with the unapproved block";
            return "";
        }
        std::string slaves;
        for (const char* s : {"0", "1", "2"}) {
            slaves += std::string("      - { slave: ") + s +
                      ", serial_number: 7, revision_number: null, hardware_version: null, software_version: null, "
                      "pdo_mapping_digest: null }\n";
        }
        const std::string record =
            "approval:\n  status: approved\n  record:\n    config_digest: \"sha256:0000\"\n"
            "    profile_digest: \"sha256:1111\"\n    implementation_revision: \"0.0.1\"\n"
            "    slaves:\n" +
            slaves + "    bench:\n" +
            plem::drive::test::indented(plem::drive::test::bench_body("sha256:0000", "sha256:1111", "0.0.1", slaves),
                                        6);
        yaml.replace(at, plem::drive::test::kUnapprovedBlock.size(), record);
        std::ofstream(dir_.path + "/drive.yaml") << yaml;
        return dir_.path + "/drive.yaml";
    }

    bool logged(std::string_view says) const {
        return std::any_of(log_.begin(), log_.end(),
                           [&](const std::string& l) { return l.find(says) != std::string::npos; });
    }
    std::string all() const {
        std::string out;
        for (const auto& l : log_) {
            out += "\n  " + l;
        }
        return out;
    }

    std::vector<std::string> log_;
    rcutils_logging_output_handler_t previous_ = nullptr;
    plem::drive::test::ScratchDir dir_{"plem_plugin_entry_"};
};

TEST_F(FileBuiltInterfaceTest, NoModelFileFailsTheInitAndSaysWhoSetsIt) {
    DriveSystemInterface plugin;
    EXPECT_EQ(plugin.on_init(params("", "")), CallbackReturn::ERROR);
    EXPECT_TRUE(
        logged("ERROR hardware parameter model_file is not set: it names the drive model file this bus runs (plem "
               "drive run sets it)"))
        << all();
}

TEST_F(FileBuiltInterfaceTest, ARunUsesTheApprovalRecordAndRefusesOneItCannotHonor) {
    const std::string file = stale_approved_file();
    ASSERT_FALSE(file.empty());
    for (const char* flag : {"", "false"}) {
        SCOPED_TRACE(std::string("unapproved='") + flag + "'");
        log_.clear();
        DriveSystemInterface plugin;
        EXPECT_EQ(plugin.on_init(params(file, flag)), CallbackReturn::ERROR);
        EXPECT_TRUE(logged("ERROR " + file + ": approval.status is approved but this software cannot honor it — "))
            << all();
        EXPECT_FALSE(logged("BENCH MODE")) << all();
    }
}

TEST_F(FileBuiltInterfaceTest, BenchModeIgnoresTheRecordWarnsAndReachesTheBus) {
    const std::string file = stale_approved_file();
    ASSERT_FALSE(file.empty());
    DriveSystemInterface plugin;
    EXPECT_EQ(plugin.on_init(params(file, "true")), CallbackReturn::ERROR);
    EXPECT_TRUE(logged("WARN BENCH MODE — motion on an unapproved model (hardware parameter unapproved=true")) << all();
    EXPECT_FALSE(logged("cannot honor it")) << all();
    EXPECT_TRUE(logged("ERROR Drive model '" + file +
                       "' could not be built: [DriveHost synthetic_3slave_cst] bus initialize failed: Failed to "
                       "request EtherCAT master "
                       "4000000000"))
        << all();
}

TEST(UnconfirmedBrakesMessage, FaultedDrivesAreNamedWithoutClaimingTheBrake) {
    plem::fieldbus::ethercat::BrakeQuiesceOutcome outcome;
    outcome.all_joints_mask = 0x03;
    outcome.timeout_mask = 0x03;
    outcome.statuswords[0] = 0x0238;
    outcome.statuswords[1] = 0x0238;
    const std::string line = stop_brakes_line(outcome).text;
    EXPECT_NE(line.find("statuswords j0=0x0238 j1=0x0238"), std::string::npos) << line;
    EXPECT_NE(line.find("joints 0x03 were still in Fault at the end of the stop"), std::string::npos) << line;
    EXPECT_NE(line.find("unknown brake state"), std::string::npos) << line;
    EXPECT_NE(line.find("could not walk them through their disable sequence"), std::string::npos) << line;
    EXPECT_NE(line.find("reset_error, then wait for the status to leave Fault"), std::string::npos) << line;
    EXPECT_EQ(line.find("held"), std::string::npos) << line;
}

// Only configured, timed-out joints count: a faulted word on a joint that
// settled, or on a slot outside the joint mask, adds no Fault sentence and no
// statusword; the highest slot (joint 7) is addressed like the others.
TEST(UnconfirmedBrakesMessage, OnlyConfiguredTimedOutJointsCount) {
    plem::fieldbus::ethercat::BrakeQuiesceOutcome outcome;
    outcome.all_joints_mask = 0x81;
    outcome.engaged_mask = 0x01;
    outcome.timeout_mask = 0x80;
    outcome.statuswords[0] = 0x0238;  // faulted but not timed out
    outcome.statuswords[1] = 0x0238;  // outside all_joints_mask
    outcome.statuswords[7] = 0x0238;
    const std::string line = stop_brakes_line(outcome).text;
    EXPECT_NE(line.find("statuswords j0=0x0238 j7=0x0238)"), std::string::npos) << line;
    EXPECT_NE(line.find("joints 0x80 were still in Fault"), std::string::npos) << line;

    outcome.timeout_mask = 0x00;
    outcome.engaged_mask = 0x00;
    EXPECT_EQ(stop_brakes_line(outcome).text.find("Fault"), std::string::npos);
}

TEST(UnconfirmedBrakesMessage, NoFaultSentenceWhenNoDriveIsFaulted) {
    plem::fieldbus::ethercat::BrakeQuiesceOutcome outcome;
    outcome.all_joints_mask = 0x03;
    outcome.engaged_mask = 0x01;
    outcome.timeout_mask = 0x02;
    outcome.statuswords[0] = 0x0250;
    outcome.statuswords[1] = 0x0233;
    const std::string line = stop_brakes_line(outcome).text;
    EXPECT_NE(line.find("statuswords j0=0x0250 j1=0x0233"), std::string::npos) << line;
    EXPECT_EQ(line.find("Fault"), std::string::npos) << line;
}

TEST(RtCpuAffinity, UnsetIsTheDefaultAnAllowedCpuPinsAnythingElseIsRefused) {
    EXPECT_FALSE(rt_cpu_affinity("").has_value()) << "unset = the last isolated CPU";
    cpu_set_t allowed;
    CPU_ZERO(&allowed);
    ASSERT_EQ(sched_getaffinity(0, sizeof(allowed), &allowed), 0);
    int cpu = 0;
    while (!CPU_ISSET(cpu, &allowed)) {
        ++cpu;
    }
    const auto pinned = rt_cpu_affinity(std::to_string(cpu));
    ASSERT_TRUE(pinned.has_value());
    EXPECT_EQ(pinned->cpus, std::vector<int>{cpu});
    for (const char* bad : {"-1", "x", "1x", "100000", " "}) {
        try {
            (void)rt_cpu_affinity(bad);
            ADD_FAILURE() << "accepted '" << bad << "'";
        } catch (const std::invalid_argument& e) {
            EXPECT_NE(std::string(e.what()).find("/sys/devices/system/cpu/isolated"), std::string::npos) << e.what();
        }
    }
}

// The hardware parameter master_index: a whole number and nothing else, as
// probe reads --master ("1x" is not master 1, "-1" not a wrapped huge index).
TEST(MasterIndex, AcceptsOnlyAWholeNumber) {
    EXPECT_EQ(master_index("0"), 0u);
    EXPECT_EQ(master_index("1"), 1u);
    for (const char* bad : {"", "-1", "+1", "1x", " 1", "x", "4294967296"}) {
        try {
            (void)master_index(bad);
            ADD_FAILURE() << "accepted '" << bad << "'";
        } catch (const std::invalid_argument& e) {
            EXPECT_NE(std::string(e.what()).find("--master"), std::string::npos) << e.what();
        }
    }
}

// The stop's brake line, word for word: the operator reads it after every
// stop. Never released wins over the masks; a fault at the deadline adds why.
TEST(StopBrakesLine, NamesEachOutcome) {
    fieldbus::ethercat::BrakeQuiesceOutcome outcome;
    outcome.all_joints_mask = 0x03;
    outcome.engaged_mask = 0x03;
    outcome.never_released = true;
    StopBrakesLine line = stop_brakes_line(outcome);
    EXPECT_FALSE(line.error);
    EXPECT_EQ(line.text, "brakes were never released this run — the stop observed no brake (2 joints, none released)");

    outcome.never_released = false;
    line = stop_brakes_line(outcome);
    EXPECT_FALSE(line.error);
    EXPECT_EQ(line.text, "brakes confirmed engaged on stop (engaged mask 0x03 of 0x03)");

    outcome.engaged_mask = 0x01;
    outcome.timeout_mask = 0x02;
    outcome.statuswords = {0x0237, 0x0218};  // j1 in Fault
    line = stop_brakes_line(outcome);
    EXPECT_TRUE(line.error);
    EXPECT_EQ(line.text,
              "brakes were NOT confirmed engaged on stop (engaged mask 0x01 of 0x03, timeout mask 0x02, statuswords "
              "j0=0x0237 j1=0x0218) — treat the bus as holding an unknown brake state; joints 0x02 were still in Fault "
              "at the end of the stop: PLEM could not walk them through their disable sequence, so those brakes were "
              "not confirmed. Whether they closed depends on the drive's own servo-off / alarm behaviour, which PLEM "
              "does not observe. Clear the fault (reset_error, then wait for the status to leave Fault) before "
              "stopping");
}

}  // namespace
}  // namespace plem::drive::ros
