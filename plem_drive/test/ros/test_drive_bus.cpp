// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_bus.cpp
/// @brief DriveBus's wiring: what reaches the host, and what the host's
/// answers look like on the wire. A mock host stands in for the bus.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <rcutils/logging.h>
#include <plem_drive_msgs/msg/drive_info.hpp>
#include <plem_drive_msgs/msg/drive_status.hpp>
#include <plem_rt_msgs/msg/rt_event.hpp>
#include <plem_rt_msgs/msg/rt_raw_batch.hpp>
#include <plem_rt_msgs/srv/set_control_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <plem/core/monitoring/event.hpp>
#include <plem/core/monitoring/rt_sample.hpp>

#include "mock_drive_host.hpp"
#include "plem/drive/ros/rt_event.hpp"
#include "ros/drive_bus.hpp"

namespace plem::drive::ros {
namespace {

using namespace std::chrono_literals;
using plem::core::Result;
using SetControlCommand = plem_rt_msgs::srv::SetControlCommand;
using test::MockDriveHost;

/// A bus over a mock host, on its own node.
struct BusOnNode {
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("plem_drive");
    std::unique_ptr<DriveBus> bus;
    BusOnNode(std::unique_ptr<IDriveHost> host, bool unapproved_motion)
        : bus(std::make_unique<DriveBus>(*node, std::move(host), unapproved_motion)) {}
};

std::pair<bool, std::string> request(DriveBus& bus, bool reset_error, bool protective_stop, bool release_brake,
                                     bool free_drive) {
    auto req = std::make_shared<SetControlCommand::Request>();
    req->reset_error = reset_error;
    req->protective_stop = protective_stop;
    req->release_brake = release_brake;
    req->free_drive = free_drive;
    auto response = std::make_shared<SetControlCommand::Response>();
    bus.command_service()->handle_request(req, response);
    return {response->success, response->message};
}

class DriveBusTest : public ::testing::Test {
  protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);  // the executor needs a live context; build it after this
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        auto owned = std::make_unique<MockDriveHost>(std::vector<std::string>{"j0", "j1"});
        host_ = owned.get();
        bus_ = std::make_unique<BusOnNode>(std::move(owned), false);

        probe_ = std::make_shared<rclcpp::Node>("drive_bus_probe");
        status_sub_ = probe_->create_subscription<plem_drive_msgs::msg::DriveStatus>(
            "/plem_drive/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [this](plem_drive_msgs::msg::DriveStatus::SharedPtr msg) { status_ = *msg; });
        event_sub_ = probe_->create_subscription<plem_rt_msgs::msg::RtEvent>(
            "/plem_drive/rt_events", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local(),
            [this](const plem_rt_msgs::msg::RtEvent& msg) { events_.push_back(msg); });

        executor_->add_node(probe_);
        executor_->add_node(bus_->node);
    }

    void TearDown() override {
        executor_.reset();
        status_sub_.reset();
        event_sub_.reset();
        probe_.reset();
        bus_.reset();
        rclcpp::shutdown();
    }

    /// Spin until `done` or the budget runs out.
    bool pump(const std::function<bool()>& done, std::chrono::milliseconds budget = 2s) {
        const auto deadline = std::chrono::steady_clock::now() + budget;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            executor_->spin_some(10ms);
            std::this_thread::sleep_for(1ms);
        }
        return done();
    }

    std::pair<bool, std::string> command(bool reset_error, bool protective_stop, bool release_brake, bool free_drive) {
        return request(*bus_->bus, reset_error, protective_stop, release_brake, free_drive);
    }

    DriveBus& bus() {
        return *bus_->bus;
    }

    MockDriveHost* host_ = nullptr;
    std::unique_ptr<BusOnNode> bus_;
    std::shared_ptr<rclcpp::Node> probe_;
    rclcpp::Subscription<plem_drive_msgs::msg::DriveStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<plem_rt_msgs::msg::RtEvent>::SharedPtr event_sub_;
    rclcpp::Subscription<plem_drive_msgs::msg::DriveInfo>::SharedPtr info_sub_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::optional<plem_drive_msgs::msg::DriveStatus> status_;
    std::vector<plem_rt_msgs::msg::RtEvent> events_;
};

/// The control service is the drive node's own `~/control_command`: one node
/// per bus, so the path is /<bus>/plem_drive/control_command (here the bus node
/// sits at the root).
TEST_F(DriveBusTest, TheControlServiceIsTheDriveNodesOwn) {
    const auto served = [this] {
        try {
            return bus_->node->get_service_names_and_types_by_node("plem_drive", "/")
                       .count("/plem_drive/control_command") > 0;
        } catch (const std::runtime_error&) {  // not in the graph cache yet
            return false;
        }
    };
    EXPECT_TRUE(pump(served));
    const auto names = bus_->node->get_node_names();
    EXPECT_EQ(std::count(names.begin(), names.end(), "/plem_control_command"), 0) << "the service is on the bus node";
}

// ~/info
// Static facts, published once and latched: a subscriber that attaches after
// the bus built (the bench runner does) still receives them.

TEST_F(DriveBusTest, InfoIsLatchedWithTheHostsDigestsInFileSlaveOrder) {
    std::vector<plem_drive_msgs::msg::DriveInfo> infos;
    info_sub_ = probe_->create_subscription<plem_drive_msgs::msg::DriveInfo>(
        "/plem_drive/info", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [&infos](plem_drive_msgs::msg::DriveInfo::SharedPtr msg) { infos.push_back(*msg); });
    ASSERT_TRUE(pump([&] { return !infos.empty(); }));
    EXPECT_EQ(infos.front().pdo_mapping_digest, host_->digests);
    EXPECT_EQ(infos.front().modes_of_operation, host_->modes)
        << "0x6060 codes, file axis order — the unit of ~/command";
    pump([] { return false; }, 300ms);
    EXPECT_EQ(infos.size(), 1u) << "published once — static, not per tick";
}

// model file
// Rows that reach a bus name master 63, which no host has: a mutation that let
// one past its check fails on the master request instead of driving a bus.

/// What `body` throws ("" = nothing).
std::string refusal_of(const std::function<void()>& body) {
    try {
        body();
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

/// Two axes on one channel of one slave: legal for the loader and the value
/// gate, refused by the bus's Step 0 before any ecrt call.
constexpr std::string_view kSharedChannelFile = R"(schema: cia402_drive/v1
model: { name: shared_channel }
bus: { cycle_time_ns: 1000000, sync_mode: drive_default }
fault_reset: { policy: none, pulse_ms: 0 }
slaves:
  - ring_position: 0
    vendor_id: 0x00c0ffee  # kSyntheticVendorId
    product_code: 0x00001001  # kSyntheticProductCode
    pdo_mapping:
      sync_managers:
        - sm: 2
          direction: output
          watchdog: true
          pdos:
            - index: 0x1600
              entries:
                - { index: 0x6040, subindex: 0, bits: 16 }
                - { index: 0x6060, subindex: 0, bits: 8 }
                - { index: 0x6071, subindex: 0, bits: 16 }
        - sm: 3
          direction: input
          watchdog: false
          pdos:
            - index: 0x1a00
              entries:
                - { index: 0x6041, subindex: 0, bits: 16 }
                - { index: 0x6061, subindex: 0, bits: 8 }
                - { index: 0x6064, subindex: 0, bits: 32 }
                - { index: 0x606c, subindex: 0, bits: 32 }
                - { index: 0x6077, subindex: 0, bits: 16 }
    startup_sdo: []
    axes:
      - name: j0
        channel: 1
        mode: cst
        mode_display_timing: pre_enable
        factors:
          gear_ratio: 100.0
          encoder_resolution: 65536
          torque: { unit: scaled_current, counts_per_amp: 50.0, torque_constant: 0.12 }
          position: { unit: rotary_motor_encoder }
          velocity: { unit: rotary_motor_encoder, velocity_factor: 1.0 }
        flip: false
        zero_offset_count: 0
        error_code_pdo: null
        limits:
          position: { min_rad: -1.0, max_rad: 1.0 }
          velocity_rad_s: 1.0
          torque_nm: { min: -1.0, max: 1.0 }
      - name: j1
        channel: 1
        mode: cst
        mode_display_timing: pre_enable
        factors:
          gear_ratio: 100.0
          encoder_resolution: 65536
          torque: { unit: scaled_current, counts_per_amp: 50.0, torque_constant: 0.12 }
          position: { unit: rotary_motor_encoder }
          velocity: { unit: rotary_motor_encoder, velocity_factor: 1.0 }
        flip: false
        zero_offset_count: 0
        error_code_pdo: null
        limits:
          position: { min_rad: -1.0, max_rad: 1.0 }
          velocity_rad_s: 1.0
          torque_nm: { min: -1.0, max: 1.0 }
approval:
  status: unapproved
  record: null
)";

TEST(DriveModelFile, AFileModelIsBuiltThroughTheHost) {
    // The refusal can only come from DriveHost::create's bus, so the loaded
    // spec went through the visit into make_drive_host.
    const std::string path =
        ::testing::TempDir() + "plem_drive_bus_shared_channel." + std::to_string(::getpid()) + ".yaml";
    std::ofstream(path) << kSharedChannelFile;
    const std::string refusal = refusal_of([&] {
        (void)build_drive_host(plem::drive::load_drive_model_file(path).value(), path, 63,
                               plem::drive::DriveHostParams{});
    });
    std::filesystem::remove(path);
    EXPECT_EQ(refusal.rfind("Drive model '" + path + "' could not be built: ", 0), 0u) << refusal;
    // The cause itself, not an error number: the operator fixes the file from it.
    EXPECT_NE(refusal.find("bus initialize failed: Axes 0 and 1 share slave 0 AND channel offset 0x0000"),
              std::string::npos)
        << refusal;
}

// approval source

std::vector<std::string>* g_warnings = nullptr;

void capture_warnings(const rcutils_log_location_t*, int severity, const char*, rcutils_time_point_value_t,
                      const char* format, va_list* args) {
    if (g_warnings == nullptr || severity != RCUTILS_LOG_SEVERITY_WARN) {
        return;
    }
    va_list copy;
    va_copy(copy, *args);
    char line[512];
    std::vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    g_warnings->emplace_back(line);
}

/// The WARN lines a bus logs while `body` runs; handler swap and context
/// undone even if the bus throws.
std::vector<std::string> warnings_while(const std::function<void()>& body) {
    std::vector<std::string> warnings;
    rclcpp::init(0, nullptr);
    struct Capture {
        rcutils_logging_output_handler_t previous = rcutils_logging_get_output_handler();
        explicit Capture(std::vector<std::string>& sink) {
            g_warnings = &sink;
            rcutils_logging_set_output_handler(capture_warnings);
        }
        ~Capture() {
            rcutils_logging_set_output_handler(previous);
            g_warnings = nullptr;
            rclcpp::shutdown();
        }
    } capture(warnings);
    body();
    return warnings;
}

TEST(DriveBusBench, OutsideBenchModeAReleaseSaysNothing) {
    const auto warnings = warnings_while([] {
        BusOnNode on(std::make_unique<MockDriveHost>(std::vector<std::string>{"j0"}), false);
        (void)request(*on.bus, false, false, true, false);
    });
    for (const auto& line : warnings) {
        EXPECT_EQ(line.find("BENCH MODE"), std::string::npos) << line;
    }
}

TEST(DriveBusBench, EveryRealReleaseIsLoggedAsBenchEvidenceANoOpIsNot) {
    const std::string release =
        "BENCH MODE — brake released on an unapproved model (hardware parameter unapproved=true, which plem drive "
        "run --unapproved sets); this release is bench evidence, not approved motion";
    std::vector<long> seen;  // the count of release lines after each request
    const auto warnings = warnings_while([&] {
        auto owned = std::make_unique<MockDriveHost>(std::vector<std::string>{"j0"});
        MockDriveHost* host = owned.get();
        BusOnNode on(std::move(owned), true);
        const auto release_brake = [&] { (void)request(*on.bus, false, false, true, false); };
        const auto count = [&] { return std::count(g_warnings->begin(), g_warnings->end(), release); };
        host->state.brake_engaged = 0x01;
        release_brake();
        seen.push_back(count());  // a real release
        host->state.brake_engaged = 0;
        host->state.brake_operation_pending = 0;
        release_brake();
        seen.push_back(count());  // every brake released and settled: the host's no-op, no line
        host->state.brake_operation_pending = 0x01;
        host->enable_result = Result<void>::error("torque command is stale");
        release_brake();
        seen.push_back(count());  // a refused release is no evidence
        host->enable_result = Result<void>::ok();
        host->readable = false;
        release_brake();
        seen.push_back(count());  // a torn snapshot counts as a release
    });
    EXPECT_EQ(seen, (std::vector<long>{1, 1, 1, 2}));
    EXPECT_EQ(std::count(warnings.begin(), warnings.end(), release), 2);
}

// command service

TEST_F(DriveBusTest, CommandFlagsReachTheHost) {
    EXPECT_TRUE(command(false, true, false, false).first);
    EXPECT_EQ(host_->ramp_stop_calls, 1) << "protective_stop is the requested stop";

    EXPECT_TRUE(command(false, false, true, false).first);
    EXPECT_EQ(host_->enable_calls, 1);

    EXPECT_TRUE(command(true, false, false, false).first);
    EXPECT_EQ(host_->reset_calls, 1);
}

// A stop with a release: the stop runs alone (host ramp_stop), everything else is
// dropped; reset_error + protective_stop keeps the fixed order (reset, then
// ramp_stop).
TEST_F(DriveBusTest, AStopWithAReleaseRampStopsAndDropsTheRest) {
    struct Row {
        bool reset_error, release_brake, free_drive;
        bool success;
        std::vector<std::string> calls;
    };
    const Row rows[] = {
        {false, false, false, true, {"ramp_stop"}}, {true, false, false, true, {"reset", "ramp_stop"}},
        {false, true, false, false, {"ramp_stop"}}, {true, true, false, false, {"ramp_stop"}},
        {false, false, true, false, {"ramp_stop"}}, {true, false, true, false, {"ramp_stop"}},
        {false, true, true, false, {"ramp_stop"}},  {true, true, true, false, {"ramp_stop"}},
    };
    for (const Row& row : rows) {
        SCOPED_TRACE(::testing::Message()
                     << "R" << row.reset_error << " B" << row.release_brake << " F" << row.free_drive);
        host_->calls.clear();
        const auto [success, message] = command(row.reset_error, true, row.release_brake, row.free_drive);
        EXPECT_EQ(success, row.success) << message;
        EXPECT_EQ(host_->calls, row.calls);
    }
}

TEST_F(DriveBusTest, ReleaseBrakeCarriesTheHostsRefusalVerbatim) {
    host_->enable_result = Result<void>::error("torque command is stale");
    const auto [success, message] = command(false, false, true, false);
    EXPECT_FALSE(success);
    EXPECT_EQ(message, "release_brake: torque command is stale");
}

// A release the host accepts is only requested: the readiness gate can refuse
// it a cycle later, and the response says where that refusal shows.
TEST_F(DriveBusTest, AnAcceptedReleaseSaysItIsOnlyRequested) {
    const auto [success, message] = command(false, false, true, false);
    EXPECT_TRUE(success);
    EXPECT_EQ(
        message,
        "release_brake: requested — watch status.operation_enabled; a readiness refusal is reported on rt_events");
}

TEST_F(DriveBusTest, FreeDriveIsRefusedWithItsReason) {
    const auto [success, message] = command(false, false, false, true);
    EXPECT_FALSE(success);
    // Pinned in full: an operator acts on this sentence. It must not read as a
    // substitute procedure: "publish 0 Nm instead" would have them release the
    // brakes of a gravity-loaded axis under zero torque.
    EXPECT_EQ(
        message,
        "free_drive: free_drive is not a drive-layer mode: this node sends each axis the command of its declared mode "
        "(cst, csp or csv) and does not compensate gravity, so there is no hand-guiding state to enter here");
    EXPECT_EQ(host_->enable_calls, 0);
}

// publishing

TEST_F(DriveBusTest, ATornSnapshotSkipsTheTickInsteadOfPublishingStaleValues) {
    host_->readable = false;
    for (int i = 0; i < DriveBus::kStatusDecimation; ++i) {
        EXPECT_FALSE(bus().publish_tick().has_value());
    }
    pump([this] { return status_.has_value(); }, 200ms);
    EXPECT_FALSE(status_.has_value());
}

TEST_F(DriveBusTest, StatusCarriesEveryDriveTierField) {
    host_->state.protective_stop = 0x04;
    host_->state.operation_enabled = true;
    host_->state.brake_engaged = 0x02;
    host_->state.brake_operation_pending = 0x01;
    host_->state.statusword = {0x0237, 0x0631};

    for (int i = 0; i < DriveBus::kStatusDecimation; ++i) {
        ASSERT_TRUE(bus().publish_tick().has_value());
    }
    ASSERT_TRUE(pump([this] { return status_.has_value(); })) << "no /plem_drive/status arrived";

    EXPECT_EQ(status_->protective_stop, 0x04);
    EXPECT_TRUE(status_->operation_enabled);
    EXPECT_EQ(status_->brake_engaged, 0x02);
    EXPECT_EQ(status_->brake_operation_pending, 0x01);
    EXPECT_EQ(status_->statusword, (std::vector<uint16_t>{0x0237, 0x0631}));
}

TEST_F(DriveBusTest, RtEventsAreDrainedOntoTheTopic) {
    plem::core::monitoring::Event event{};
    event.monotonic_ns = 123456789;
    event.type = plem::core::monitoring::EventType::COMMAND_STALE;
    event.joint_id = 1;
    event.value = 0.1002f;
    ASSERT_TRUE(host_->monitoring_context()->emit(event));

    (void)bus().publish_tick();
    ASSERT_TRUE(pump([this] { return !events_.empty(); })) << "no /plem_drive/rt_events arrived";

    EXPECT_EQ(events_.front().monotonic_ns, 123456789u);
    EXPECT_EQ(events_.front().event_type, static_cast<uint8_t>(plem::core::monitoring::EventType::COMMAND_STALE));
    EXPECT_EQ(events_.front().joint_id, 1);
    EXPECT_FLOAT_EQ(events_.front().value, 0.1002f);
}

// /rt_raw
// From start() until stop() the bus's own thread drains the RT samples onto
// /rt_raw by the shared batching rule (RtRawBatcher), never the executor, which serves
// the command path.

/// Queues `n` RT samples 1 ms apart, from a loop whose period is `period_us`,
/// over the fake host's two axes.
void queue_samples(IDriveHost& host, int n, float period_us) {
    for (int i = 0; i < n; ++i) {
        plem::core::monitoring::RtSample sample{};
        sample.axis_count = 2;
        sample.monotonic_ns = 1'000'000'000ULL + static_cast<uint64_t>(i) * 1'000'000ULL;
        sample.loop_period_us = period_us;
        ASSERT_TRUE(host.monitoring_context()->emit(sample));
    }
}

bool samples_pending(IDriveHost& host) {
    return host.monitoring_context()->queue<plem::core::monitoring::RtSample>()->front() != nullptr;
}

class DriveBusRtRawTest : public DriveBusTest {
  protected:
    void SetUp() override {
        DriveBusTest::SetUp();
        rt_raw_sub_ = probe_->create_subscription<plem_rt_msgs::msg::RtRawBatch>(
            "/rt_raw", rclcpp::QoS(rclcpp::KeepLast(64)).best_effort(),
            [this](const plem_rt_msgs::msg::RtRawBatch& msg) { batches_.push_back(msg); });
        ASSERT_TRUE(pump([this] { return rt_raw_sub_->get_publisher_count() > 0; }));
    }
    void TearDown() override {
        rt_raw_sub_.reset();
        DriveBusTest::TearDown();
    }
    size_t samples_published() const {
        size_t n = 0;
        for (const auto& batch : batches_) {
            n += batch.batch_size;
        }
        return n;
    }

    rclcpp::Subscription<plem_rt_msgs::msg::RtRawBatch>::SharedPtr rt_raw_sub_;
    std::vector<plem_rt_msgs::msg::RtRawBatch> batches_;
};

TEST_F(DriveBusRtRawTest, EverySampleIsBatchedFromStartInTheLoopsOwnPeriod) {
    queue_samples(*host_, 45, 500.0f);
    ASSERT_TRUE(bus().start());
    ASSERT_TRUE(pump([this] { return samples_published() == 45; })) << samples_published() << " of 45";
    EXPECT_EQ(batches_.front().batch_size, 40u);
    EXPECT_EQ(batches_.front().period_ns, 500'000u) << "the RT loop's period as the sample carries it";
    EXPECT_EQ(batches_.front().axis_count, 2u) << "the axes the sample carries, not the RT record's width";
    EXPECT_EQ(batches_.front().actual_position.size(), 40u * 2u);
    (void)bus().stop();
}

TEST_F(DriveBusRtRawTest, NothingIsDrainedWhenStartIsRefused) {
    host_->start_result = Result<void>::error("refused");
    queue_samples(*host_, 3, 1000.0f);
    ASSERT_FALSE(bus().start());
    pump([] { return false; }, 200ms);
    EXPECT_TRUE(samples_pending(*host_));
    EXPECT_TRUE(batches_.empty());
}

// The bus is up when every slave is in OP: start() returns once the working
// counter is complete, and a bus that never gets there is stopped and refused.
TEST_F(DriveBusRtRawTest, StartReturnsOnceEverySlaveIsInOp) {
    host_->incomplete_reads = 5;
    ASSERT_TRUE(bus().start());
    EXPECT_EQ(host_->incomplete_reads.load(), 0) << "start() returned before the working counter was complete";
    (void)bus().stop();
}

TEST_F(DriveBusRtRawTest, AStartThatNeverReachesOpStopsTheBusAndSaysWhy) {
    host_->incomplete_reads = std::numeric_limits<int>::max();
    const auto started = bus().start(50ms);
    ASSERT_FALSE(started);
    EXPECT_NE(started.message().find("not every slave reached OP"), std::string::npos) << started.message();
    EXPECT_EQ(host_->calls, (std::vector<std::string>{"ramp_stop", "stop"}));
}

// The bus's end asks for the stop before anything else: the 50 ms command
// watch is armed while the drain winds down, so a stop asked later could meet
// a stale command and become the immediate disable instead of the ramp.
TEST_F(DriveBusRtRawTest, StopAsksTheHostToStopFirst) {
    ASSERT_TRUE(bus().start());
    host_->calls.clear();
    (void)bus().stop();
    EXPECT_EQ(host_->calls, (std::vector<std::string>{"ramp_stop", "stop"}));
}

TEST_F(DriveBusRtRawTest, StopEndsTheDrain) {
    ASSERT_TRUE(bus().start());
    (void)bus().stop();
    queue_samples(*host_, 3, 1000.0f);
    pump([] { return false; }, 200ms);
    EXPECT_TRUE(samples_pending(*host_));
}

// Destruction without stop() once the drain has published. Whether samples are
// still queued at that moment is not synchronized; this does not pin the
// backlog case, only that the destructor ends and joins a running drain.
TEST_F(DriveBusRtRawTest, DestroyingARunningBusJoinsTheDrain) {
    queue_samples(*host_, 4000, 1000.0f);
    ASSERT_TRUE(bus().start());
    ASSERT_TRUE(pump([this] { return samples_published() > 0; })) << "the drain is running";
    bus_->bus.reset();  // no stop(): the destructor ends and joins the drain
    SUCCEED();
}

}  // namespace
}  // namespace plem::drive::ros
