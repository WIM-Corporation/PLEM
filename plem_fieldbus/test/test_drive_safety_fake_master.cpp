// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_safety_fake_master.cpp
/// @brief Drive-layer RT loop on the fake master: EtherCATController<1> with
/// declared drive limits: the torque gate at the apply point, the limit →
/// INTERNAL P-STOP step, the reset consumption and the freshness interplay.
/// Same seam as test_ethercat_controller_fake_master.cpp (real run_loop
/// against fake_ecrt, SCHED_OTHER).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/monitoring/rt_sample.hpp>
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/core/types/motion_limit.hpp>
#include <plem/core/types/shared_joint_data.hpp>
#include <plem/core/units.hpp>

#include "device_servo_test_access.hpp"
#include "fake_master_fixture.hpp"
#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/controller.hpp"
#include "plem/fieldbus/ethercat/drive_safety.hpp"
#include "plem/fieldbus/ethercat/utils/servo_conversions.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

namespace {

using plem::core::ControlModeFlags;
using plem::core::ProtectiveStopSource;
using plem::core::units::Pulse;
using plem::fieldbus::ethercat::EtherCATBus;
using plem::fieldbus::ethercat::EtherCATController;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::test::FakeEcrt;

// PDO object indices; at() resolves them to the fake domain's offsets.
constexpr unsigned int kTargetTorqueEntry = 0x6071;
constexpr unsigned int kStatuswordEntry = 0x6041;
constexpr unsigned int kModesOfOperationDisplayEntry = 0x6061;
constexpr unsigned int kActualPositionEntry = 0x6064;
constexpr unsigned int kActualVelocityEntry = 0x606C;
constexpr unsigned int kTargetPositionEntry = 0x607A;
constexpr unsigned int kTargetVelocityEntry = 0x60FF;
constexpr unsigned int kControlwordEntry = 0x6040;

uint32_t at(unsigned int axis, unsigned int index) {
    return plem::fieldbus::test::pdo_offset(axis, index);
}

thread_local bool t_on_loop_thread = false;

/// While the loop runs the domain is the loop thread's; test code reaches it
/// through this mailbox. pump(), the fake's send observer, applies the
/// writes staged since the last frame (the next receive reads them),
/// snapshots the frame just sent, then runs the test's send hook. On the loop
/// thread, or with no loop running, write() and read() touch the domain
/// directly; from the test thread under a running loop, write() stages and
/// read() sees the last snapshot. A write staged after the controller's last
/// send is dropped (no current test relies on it).
class DomainMailbox {
  public:
    void attach(uint8_t* pd, std::function<bool()> loop_running) {
        std::lock_guard lock(mutex_);
        pd_ = pd;
        loop_running_ = std::move(loop_running);
        staged_.clear();
        hook_.reset();
        frame_.assign(pd, pd + ecrt_domain_size(nullptr));
        FakeEcrt::set_send_observer([this] { pump(); });
    }

    void write(std::function<void(uint8_t*)> apply) {
        if (direct()) {
            apply(pd_);
            return;
        }
        std::lock_guard lock(mutex_);
        staged_.push_back(std::move(apply));
    }

    template <typename Read>
    auto read(Read&& from) {
        if (direct()) {
            return from(static_cast<const uint8_t*>(pd_));
        }
        std::lock_guard lock(mutex_);
        return from(static_cast<const uint8_t*>(frame_.data()));
    }

    /// Runs `hook` on the loop thread after every frame (nullptr clears it).
    void on_send(std::function<void()> hook) {
        auto shared = hook ? std::make_shared<const std::function<void()>>(std::move(hook)) : nullptr;
        std::lock_guard lock(mutex_);
        hook_ = std::move(shared);
    }

  private:
    bool direct() const {
        return t_on_loop_thread || !loop_running_();
    }

    void pump() {
        t_on_loop_thread = true;
        std::shared_ptr<const std::function<void()>> hook;
        {
            std::lock_guard lock(mutex_);
            for (const auto& apply : staged_) {
                apply(pd_);
            }
            staged_.clear();
            std::copy_n(pd_, frame_.size(), frame_.begin());
            hook = hook_;
        }
        if (hook) {
            (*hook)();
        }
        t_on_loop_thread = false;
    }

    std::mutex mutex_;
    uint8_t* pd_ = nullptr;
    std::function<bool()> loop_running_;
    std::vector<std::function<void(uint8_t*)>> staged_;
    std::vector<uint8_t> frame_;
    std::shared_ptr<const std::function<void()>> hook_;
};

constexpr uint16_t kSwitchedOn = 0x0023;
constexpr uint16_t kOperationEnabled = 0x0027;

using plem::fieldbus::test::make_servo_info;
using plem::fieldbus::test::wait_for;

plem::core::MotionLimit tight_limit() {
    return plem::core::MotionLimit{
        .lower_limit = -0.05, .upper_limit = 0.05, .velocity_limit = 10.0, .effort_limit = 5.0};
}

using plem::core::monitoring::Event;
using plem::core::monitoring::EventType;
using plem::core::monitoring::LimitStopReason;
using plem::core::monitoring::Severity;

/// Drains the event queue into `seen`, keeping the LIMIT_STOPs.
size_t limit_stops(plem::core::monitoring::MonitoringContext& ctx, std::vector<Event>& seen) {
    auto* queue = ctx.queue<Event>();
    while (auto* event = queue->front()) {
        if (event->type == plem::core::monitoring::EventType::LIMIT_STOP) {
            seen.push_back(*event);
        }
        queue->pop();
    }
    return seen.size();
}

class DriveSafetyLoopTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FakeEcrt::reset();
        ASSERT_TRUE(ctx_.initialize().is_ok());
        bus_ = std::make_shared<EtherCATBus>(0);
        bus_->set_servo_config({make_servo_info()});
        bus_->set_cycle_time_ns(1'000'000);
        const auto init = bus_->initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
        pd_ = ecrt_domain_data(nullptr);
        wire_.attach(pd_, [this] { return controller_ && controller_->is_running(); });
        set_status(kSwitchedOn);
        EC_WRITE_U8(pd_ + at(0, kModesOfOperationDisplayEntry), 0x0A);  // CST displayed
        set_actual_position(0, 0);

        flags_ = std::make_shared<ControlModeFlags>();
        shared_ = std::make_shared<plem::core::SharedJointData<1>>();

        auto config = plem::fieldbus::test::test_controller_config<1>();
        controller_ = std::make_unique<EtherCATController<1>>(config);
        controller_->set_bus(bus_);
        controller_->set_mode_flags(flags_);
        controller_->set_shared_data(shared_);
        controller_->set_monitoring_context(&ctx_);
        const std::vector<plem::core::MotionLimit> limits{tight_limit()};
        controller_->set_drive_limits(limits);
    }

    void TearDown() override {
        streaming_.store(false);
        if (streamer_.joinable()) {
            streamer_.join();
        }
        if (controller_) {
            controller_->stop();
        }
        FakeEcrt::reset();
    }

    void set_status(uint16_t word) {
        const uint32_t off = at(0, kStatuswordEntry);
        wire_.write([off, word](uint8_t* pd) { EC_WRITE_U16(pd + off, word); });
    }
    void set_actual_position(unsigned int axis, int32_t word) {
        const uint32_t off = at(axis, kActualPositionEntry);
        wire_.write([off, word](uint8_t* pd) { EC_WRITE_S32(pd + off, word); });
    }
    void set_actual_velocity(unsigned int axis, int32_t word) {
        const uint32_t off = at(axis, kActualVelocityEntry);
        wire_.write([off, word](uint8_t* pd) { EC_WRITE_S32(pd + off, word); });
    }
    int16_t target_torque_on_wire() {
        const uint32_t off = at(0, kTargetTorqueEntry);
        return wire_.read([off](const uint8_t* pd) { return EC_READ_S16(pd + off); });
    }

    /// Host-side torque writer: `nm` with a fresh stamp every 5 ms (well inside
    /// the 100 ms freshness window). The first write lands before this returns,
    /// so a wait_cycles() after it sees a fresh command.
    void stream_torque(double nm) {
        const auto write = [this, nm] {
            plem::core::JointCommand<1> cmd{};
            cmd.value[0] = nm;
            cmd.update_ns = plem::core::now_ns();
            shared_->command.write(cmd);
        };
        write();
        streaming_.store(true);
        streamer_ = std::thread([this, write] {
            while (streaming_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                write();
            }
        });
    }
    void stop_streaming() {
        streaming_.store(false);
        if (streamer_.joinable()) {
            streamer_.join();
        }
    }

    /// Waits `n` cycle starts: evidence the loop has run on since this call (scheduling, not memory ordering).
    void wait_cycles(int n = 2) {
        for (int k = 0; k < n; ++k) {
            const auto t = shared_->last_cycle_ns.load();
            ASSERT_TRUE(wait_for([&] { return shared_->last_cycle_ns.load() != t; }));
        }
    }

    void start() {
        auto r = controller_->start();
        ASSERT_TRUE(r) << r.message();
    }

    plem::core::monitoring::MonitoringContext ctx_;
    std::shared_ptr<EtherCATBus> bus_;
    std::shared_ptr<ControlModeFlags> flags_;
    std::shared_ptr<plem::core::SharedJointData<1>> shared_;
    std::unique_ptr<EtherCATController<1>> controller_;
    uint8_t* pd_{nullptr};
    DomainMailbox wire_;
    std::atomic<bool> streaming_{false};
    std::thread streamer_;
};

TEST_F(DriveSafetyLoopTest, SetterRefusesWrongCountAndARunningLoop) {
    const std::vector<plem::core::MotionLimit> two{tight_limit(), tight_limit()};
    EXPECT_THROW(controller_->set_drive_limits(two), std::invalid_argument);
    start();
    const std::vector<plem::core::MotionLimit> one{tight_limit()};
    EXPECT_THROW(controller_->set_drive_limits(one), std::logic_error);
}

TEST_F(DriveSafetyLoopTest, SetDriveStopRefusesANegativeOrNonFiniteDeceleration) {
    for (const double bad : {-1.0, std::nan(""), std::numeric_limits<double>::infinity()}) {
        const std::vector<double> decelerations{bad};
        EXPECT_THROW(controller_->set_drive_stop(decelerations), std::invalid_argument) << bad;
    }
    const std::vector<double> cst{0.0};  // a cst axis declares no ramp
    EXPECT_NO_THROW(controller_->set_drive_stop(cst));
}

TEST_F(DriveSafetyLoopTest, SetDriveLimitsRefusesNaNInvertedNonPositiveOrInfiniteVelocity) {
    const double inf = std::numeric_limits<double>::infinity();
    auto refused = [&](plem::core::MotionLimit limit) {
        const std::vector<plem::core::MotionLimit> limits{limit};
        EXPECT_THROW(controller_->set_drive_limits(limits), std::invalid_argument);
    };
    refused({.lower_limit = std::nan(""), .upper_limit = 1.0, .velocity_limit = 1.0, .effort_limit = 5.0});
    refused({.lower_limit = 1.0, .upper_limit = -1.0, .velocity_limit = 1.0, .effort_limit = 5.0});
    refused({.lower_limit = -1.0, .upper_limit = 1.0, .velocity_limit = 0.0, .effort_limit = 5.0});
    // +inf would switch off the csp step and csv checks and make the stop's settle band always true.
    refused({.lower_limit = -1.0, .upper_limit = 1.0, .velocity_limit = inf, .effort_limit = 5.0});
    refused({.lower_limit = -1.0, .upper_limit = 1.0, .velocity_limit = 1.0, .effort_limit = std::nan("")});
    // An endless axis's ±inf position pair and a csp or csv axis's infinite effort stay valid.
    const std::vector<plem::core::MotionLimit> endless{
        {.lower_limit = -inf, .upper_limit = inf, .velocity_limit = 1.0, .effort_limit = inf}};
    EXPECT_NO_THROW(controller_->set_drive_limits(endless));
}

TEST_F(DriveSafetyLoopTest, ACstBusStartsWithAZeroStopDeceleration) {
    const std::vector<double> cst{0.0};  // the drive layer's value on a cst axis
    controller_->set_drive_stop(cst);
    start();
}

TEST_F(DriveSafetyLoopTest, StatuswordMirrorFollowsTheWire) {
    start();
    EXPECT_TRUE(wait_for([&] { return controller_->statusword(0) == kSwitchedOn; }));
    set_status(kOperationEnabled);
    EXPECT_TRUE(wait_for([&] { return controller_->statusword(0) == kOperationEnabled; }));
}

TEST_F(DriveSafetyLoopTest, GateHoldsZeroInTheSwitchedOnPrimingWindow) {
    // SWITCHED_ON: may_hold_brake_open primes torque there for a host without drive limits;
    // with declared limits the gate keeps 0 until the drive is OE.
    flags_->motion_enabled.store(true);
    stream_torque(1.0);
    start();
    ASSERT_TRUE(wait_for([&] { return shared_->last_cycle_ns.load() != 0; }));
    wait_cycles(50);
    EXPECT_EQ(target_torque_on_wire(), 0);
    // Now OE: all_drives_enabled → the same command reaches the wire.
    set_status(kOperationEnabled);
    EXPECT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));
    EXPECT_TRUE(flags_->all_drives_enabled.load());
}

TEST_F(DriveSafetyLoopTest, ProtectiveStopZeroesTorqueWhileTheDriveStaysEnabled) {
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream_torque(1.0);
    start();
    ASSERT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));
    flags_->set_protective_stop(ProtectiveStopSource::EXTERNAL);
    // The fake drive never leaves OE (a stalled disable); the gate alone must zero the wire.
    EXPECT_TRUE(wait_for([&] { return target_torque_on_wire() == 0; }));
    wait_cycles(30);
    EXPECT_EQ(target_torque_on_wire(), 0);
}

TEST_F(DriveSafetyLoopTest, DisableZeroesTorqueWhileTheDriveStaysEnabled) {
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream_torque(1.0);
    start();
    ASSERT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));
    flags_->motion_enabled.store(false);
    EXPECT_TRUE(wait_for([&] { return target_torque_on_wire() == 0; }));
}

TEST_F(DriveSafetyLoopTest, LimitViolationRaisesInternalProtectiveStopAndResetClearsIt) {
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream_torque(1.0);
    start();
    ASSERT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));

    set_actual_position(0, 50'000'000);  // far outside ±0.05 rad
    EXPECT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
    EXPECT_TRUE(wait_for([&] { return target_torque_on_wire() == 0; }));

    // Reset while still outside: rejected (flag stays), request consumed.
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->reset_error.load(); }));
    EXPECT_TRUE(flags_->is_internal_protective_stop());

    // Back inside + reset: cleared, torque flows again.
    set_actual_position(0, 0);
    wait_cycles(10);
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->is_protective_stop_active(); }));
    EXPECT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));
}

// The stop names its axis and cause once per INTERNAL episode: not
// every cycle the violation persists, not on a refused reset, and again for a
// new episode after a reset.
TEST_F(DriveSafetyLoopTest, ALimitStopNamesTheAxisAndCauseOncePerEpisode) {
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream_torque(1.0);
    start();
    ASSERT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));

    std::vector<Event> stops;
    set_actual_position(0, 50'000'000);  // far outside ±0.05 rad
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    EXPECT_EQ(stops[0].joint_id, 0);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::POSITION));
    EXPECT_GT(stops[0].value, 0.05f) << "the measured angle, rad";
    EXPECT_EQ(stops[0].severity, static_cast<uint8_t>(Severity::ERROR));
    flags_->reset_error.store(true);  // refused: still outside
    EXPECT_TRUE(wait_for([&] { return !flags_->reset_error.load(); }));
    wait_cycles(20);
    EXPECT_EQ(limit_stops(ctx_, stops), 1u);

    set_actual_position(0, 0);
    wait_cycles();
    flags_->reset_error.store(true);
    ASSERT_TRUE(wait_for([&] { return !flags_->is_protective_stop_active(); }));
    set_actual_position(0, -50'000'000);
    EXPECT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) == 2; }));
}

// An EXTERNAL stop already standing does not hide a limit violation's cause.
TEST_F(DriveSafetyLoopTest, ALimitStopIsNamedUnderAnExternalStopToo) {
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream_torque(1.0);
    start();
    ASSERT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));
    stop_streaming();
    ASSERT_TRUE(wait_for([&] { return flags_->is_external_protective_stop(); }));

    std::vector<Event> stops;
    set_actual_position(0, 50'000'000);
    EXPECT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) == 1; }));
}

// A velocity violation carries the measured velocity (rad/s), not the position.
TEST_F(DriveSafetyLoopTest, AVelocityViolationIsNamedWithTheMeasuredVelocity) {
    start();
    std::vector<Event> stops;
    set_actual_velocity(0, 50'000'000);  // ~40 rad/s past 10, at 0 rad
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    EXPECT_EQ(stops[0].joint_id, 0);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::VELOCITY));
    EXPECT_GT(stops[0].value, 10.0f) << "the measured velocity, rad/s";
}

TEST_F(DriveSafetyLoopTest, StaleHostTripsExternalAndResetClearsOnlyOnceFresh) {
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream_torque(1.0);
    start();
    ASSERT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));

    stop_streaming();  // the host goes silent → freshness trips EXTERNAL
    EXPECT_TRUE(wait_for([&] { return flags_->is_external_protective_stop(); }));
    EXPECT_TRUE(wait_for([&] { return target_torque_on_wire() == 0; }));

    // A reset while still stale is consumed but the level re-asserts.
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->reset_error.load(); }));
    wait_cycles(20);
    EXPECT_TRUE(flags_->is_external_protective_stop());

    // Fresh writes again + reset: the clear sticks.
    stream_torque(1.0);
    wait_cycles(10);
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->is_protective_stop_active(); }));
    EXPECT_TRUE(wait_for([&] { return target_torque_on_wire() != 0; }));
}

}  // namespace

// CSP/CSV command stage
// Two axes: axis 0 CST (tight limits, resting at 0), axis 1 the mode under test
// with flip and a large offset, so a transform applied twice or not at all
// moves every word. A CSP/CSV target reaches the wire only while the torque
// gate is open, the command carries a host stamp and every axis's command
// passes its check; one violation latches INTERNAL P-STOP, arms the in-service
// trip and writes every axis neutral in that same frame (CST 0 Nm, CSP the raw
// position mirror, CSV 0).

namespace {

constexpr int32_t kOffset = 1'000'000;
constexpr int32_t kRestWord = 1'234'567;  // axis 1's raw 0x6064 at rest

plem::core::MotionLimit wide_limit() {
    return plem::core::MotionLimit{.lower_limit = -1.0, .upper_limit = 1.0, .velocity_limit = 1.0, .effort_limit = 5.0};
}

}  // namespace

class DriveCommandLoopTest : public ::testing::Test {
  protected:
    void TearDown() override {
        stop_streaming();
        if (controller_) {
            controller_->stop();
        }
        FakeEcrt::set_send_observer(nullptr);
        FakeEcrt::reset();
    }

    void build(plem::fieldbus::OperationMode mode, int32_t offset = kOffset, int32_t rest_word = kRestWord,
               plem::core::MotionLimit axis1_limit = wide_limit(), bool flip = true) {
        FakeEcrt::reset();
        ASSERT_TRUE(ctx_.initialize().is_ok());
        auto first = make_servo_info();
        first.modes_of_operation = axis0_mode_;
        auto other = make_servo_info();
        other.modes_of_operation = mode;
        other.flip = flip;
        other.joint_offset = Pulse(offset);
        other.position_endless = std::isinf(axis1_limit.lower_limit);  // as the drive layer derives it
        if (axis1_profile_) {
            other.cia402_profile = *axis1_profile_;
        }
        offset_ = offset;
        channels_ = other.factors;
        bus_ = std::make_shared<EtherCATBus>(0);
        bus_->set_servo_config({first, other});
        bus_->set_cycle_time_ns(1'000'000);
        const auto init = bus_->initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
        pd_ = ecrt_domain_data(nullptr);
        wire_.attach(pd_, [this] { return controller_ && controller_->is_running(); });
        set_status(kSwitchedOn);
        EC_WRITE_U8(pd_ + at(0, kModesOfOperationDisplayEntry), static_cast<uint8_t>(axis0_mode_));
        EC_WRITE_U8(pd_ + at(1, kModesOfOperationDisplayEntry), static_cast<uint8_t>(mode));
        set_actual_position(0, 0);
        set_actual_position(1, rest_word);

        flags_ = std::make_shared<ControlModeFlags>();
        shared_ = std::make_shared<plem::core::SharedJointData<2>>();
        auto config = plem::fieldbus::test::test_controller_config<2>();
        controller_ = std::make_unique<EtherCATController<2>>(config);
        controller_->set_bus(bus_);
        controller_->set_mode_flags(flags_);
        controller_->set_shared_data(shared_);
        controller_->set_monitoring_context(&ctx_);
        if (declare_limits_) {
            const std::vector<plem::core::MotionLimit> limits{tight_limit(), axis1_limit};
            controller_->set_drive_limits(limits);
        }
        const auto ramp = [](plem::fieldbus::OperationMode m) {
            return m == plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE ? 0.0 : 2.0;
        };
        const std::vector<double> decelerations{ramp(axis0_mode_), ramp(mode)};
        controller_->set_drive_stop(decelerations);
    }

    void set_status(uint16_t word) {
        const uint32_t off0 = at(0, kStatuswordEntry);
        const uint32_t off1 = at(1, kStatuswordEntry);
        wire_.write([off0, off1, word](uint8_t* pd) {
            EC_WRITE_U16(pd + off0, word);
            EC_WRITE_U16(pd + off1, word);
        });
    }
    void set_actual_position(unsigned int axis, int32_t word) {
        const uint32_t off = at(axis, kActualPositionEntry);
        wire_.write([off, word](uint8_t* pd) { EC_WRITE_S32(pd + off, word); });
    }
    void set_actual_velocity(unsigned int axis, int32_t word) {
        const uint32_t off = at(axis, kActualVelocityEntry);
        wire_.write([off, word](uint8_t* pd) { EC_WRITE_S32(pd + off, word); });
    }
    int32_t position_word(size_t axis) {
        const uint32_t off = at(axis, kTargetPositionEntry);
        return wire_.read([off](const uint8_t* pd) { return EC_READ_S32(pd + off); });
    }
    uint16_t controlword(size_t axis) {
        const uint32_t off = at(axis, kControlwordEntry);
        return wire_.read([off](const uint8_t* pd) { return EC_READ_U16(pd + off); });
    }
    int32_t velocity_word(size_t axis) {
        const uint32_t off = at(axis, kTargetVelocityEntry);
        return wire_.read([off](const uint8_t* pd) { return EC_READ_S32(pd + off); });
    }
    int16_t torque_word(size_t axis) {
        const uint32_t off = at(axis, kTargetTorqueEntry);
        return wire_.read([off](const uint8_t* pd) { return EC_READ_S16(pd + off); });
    }
    /// Axis 1's resting angle under the default build (kOffset, flip), usable before build().
    static double rest_rad() {
        return *plem::fieldbus::ethercat::decode_position_word(kRestWord, make_servo_info().factors.position, kOffset,
                                                               true);
    }
    int32_t position_word_for(double rad) {
        return plem::fieldbus::ethercat::encode_position_checked(rad, channels_.position, offset_, true)->word;
    }

    /// Host writer: the two commands below, a fresh stamp every 2 ms, the
    /// first one before this returns.
    void stream(double axis0, double axis1) {
        retarget(axis0, axis1);
        const auto write = [this] {
            plem::core::JointCommand<2> cmd{};
            cmd.value[0] = command_[0].load();
            cmd.value[1] = command_[1].load();
            cmd.update_ns = plem::core::now_ns();
            shared_->command.write(cmd);
        };
        write();
        streaming_.store(true);
        streamer_ = std::thread([this, write] {
            while (streaming_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                write();
            }
        });
    }
    /// The host's command written from the RT thread (stamped now), for
    /// cycle-exact scripts, where the 2 ms streamer thread cannot be placed.
    void write_command(double axis0, double axis1, double axis1_rate = 0.0) {
        plem::core::JointCommand<2> cmd{};
        cmd.value[0] = axis0;
        cmd.value[1] = axis1;
        cmd.rate[1] = axis1_rate;
        cmd.update_ns = plem::core::now_ns();
        shared_->command.write(cmd);
    }
    /// Runs `step(cycle)` on the RT thread after every send (cycle 0 = the
    /// first). What it writes into the domain is the next cycle's frame.
    void per_cycle(std::function<void(int)> step) {
        auto cycle = std::make_shared<int>(0);
        wire_.on_send([step = std::move(step), cycle] { step((*cycle)++); });
    }
    void retarget(double axis0, double axis1) {
        command_[0].store(axis0);
        command_[1].store(axis1);
    }
    void stop_streaming() {
        streaming_.store(false);
        if (streamer_.joinable()) {
            streamer_.join();
        }
    }

    size_t count(plem::core::monitoring::EventType type) {
        auto* queue = ctx_.queue<plem::core::monitoring::Event>();
        while (auto* event = queue->front()) {
            ++seen_[static_cast<uint8_t>(event->type)];
            queue->pop();
        }
        return seen_[static_cast<uint8_t>(type)];
    }

    /// Waits `n` cycle starts: evidence the loop has run on since this call (scheduling, not memory ordering).
    void wait_cycles(int n = 2) {
        for (int k = 0; k < n; ++k) {
            const auto t = shared_->last_cycle_ns.load();
            ASSERT_TRUE(wait_for([&] { return shared_->last_cycle_ns.load() != t; }));
        }
    }

    void start() {
        auto r = controller_->start();
        ASSERT_TRUE(r) << r.message();
    }

    /// The newest RT sample (drains the queue).
    plem::core::monitoring::RtSample latest_sample() {
        auto* queue = ctx_.queue<plem::core::monitoring::RtSample>();
        plem::core::monitoring::RtSample last{};
        while (auto* sample = queue->front()) {
            last = *sample;
            queue->pop();
        }
        return last;
    }
    /// The torque the CST axis 0's wire word stands for (flip off).
    float wire_torque0() {
        return static_cast<float>(
            plem::fieldbus::ethercat::to_torque(plem::core::units::TorqueCount::from_hardware(torque_word(0), false),
                                                make_servo_info().factors.torque)
                .value());
    }

    /// Enabled and applying `axis1_target` (CSP): the state the violation cases start from.
    void run_applying_csp(double axis1_target) {
        build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
        flags_->motion_enabled.store(true);
        set_status(kOperationEnabled);
        stream(1.0, axis1_target);
        start();
        ASSERT_TRUE(wait_for([&] { return position_word(1) == position_word_for(axis1_target); }));
        ASSERT_FALSE(flags_->is_protective_stop_active());
    }

    plem::fieldbus::OperationMode axis0_mode_ = plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
    bool declare_limits_ = true;  // false: build() skips set_drive_limits()
    std::optional<plem::fieldbus::Cia402TransitionProfile> axis1_profile_;
    plem::core::monitoring::MonitoringContext ctx_;
    std::shared_ptr<EtherCATBus> bus_;
    std::shared_ptr<ControlModeFlags> flags_;
    std::shared_ptr<plem::core::SharedJointData<2>> shared_;
    std::unique_ptr<EtherCATController<2>> controller_;
    plem::fieldbus::ethercat::AxisFactors channels_{};
    int32_t offset_{0};
    uint8_t* pd_{nullptr};
    DomainMailbox wire_;
    std::atomic<double> command_[2]{};
    std::atomic<bool> streaming_{false};
    std::thread streamer_;
    std::array<size_t, 256> seen_{};
};

// Without declared limits the loop applies cst torque only and request_ramp_stop()
// has nothing to ramp: a csp or csv axis would start and silently do nothing.
TEST_F(DriveCommandLoopTest, StartRefusesACspOrCsvAxisWithoutDeclaredLimits) {
    for (const auto mode :
         {plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY}) {
        SCOPED_TRACE(static_cast<int>(mode));
        declare_limits_ = false;
        build(mode);
        const auto r = controller_->start();
        ASSERT_FALSE(r.is_ok());
        EXPECT_NE(r.message().find("axes[1] is a csp or csv axis but no drive limits are declared"), std::string::npos)
            << r.message();
        EXPECT_NE(r.message().find("set_drive_limits()"), std::string::npos) << r.message();
        EXPECT_FALSE(controller_->is_running());
    }
}

// groups[i] is the mask released together with joint i: it holds joint i, names
// declared joints only, and every joint it names declares the same mask.
TEST_F(DriveCommandLoopTest, SetBrakeSetupRefusesAMaskOutsideThePartition) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE);
    EXPECT_THROW(controller_->set_brake_setup({0x02, 0x02}), std::invalid_argument)
        << "joint 0's mask leaves joint 0 out";
    EXPECT_THROW(controller_->set_brake_setup({0x01, 0x03}), std::invalid_argument)
        << "joint 1 names joint 0, which disagrees";
    EXPECT_THROW(controller_->set_brake_setup({0x05, 0x02}), std::invalid_argument) << "bit 2 is beyond the two axes";
    EXPECT_THROW(controller_->set_brake_setup({0x00, 0x02}), std::invalid_argument)
        << "a declared table names every joint";
    EXPECT_NO_THROW(controller_->set_brake_setup({0x03, 0x03}));
    EXPECT_NO_THROW(controller_->set_brake_setup({0x00, 0x00}));  // undeclared: every joint its own brake
}

// A command violation names its axis and cause; the other axis, within
// its checks, is not named.
TEST_F(DriveCommandLoopTest, StartRefusesACspOrCsvAxisWithoutAStopDeceleration) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    const std::vector<double> no_ramp{0.0, 0.0};  // axis 0 is cst; axis 1 csp has no ramp
    controller_->set_drive_stop(no_ramp);
    const auto r = controller_->start();
    ASSERT_FALSE(r);
    EXPECT_NE(r.message().find("axes[1]"), std::string::npos) << r.message();
}

TEST_F(DriveCommandLoopTest, ACommandViolationNamesItsAxisAndCause) {
    run_applying_csp(rest_rad() + 0.0005);
    std::vector<Event> stops;
    retarget(1.0, rest_rad() + 0.0105);  // ten cycles' travel in one
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    wait_cycles(20);
    ASSERT_EQ(limit_stops(ctx_, stops), 1u);
    EXPECT_EQ(stops[0].joint_id, 1);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::CSP_STEP));
    EXPECT_NEAR(stops[0].value, 0.01f, 1e-4f) << "target minus the applied target, rad";
}

// Every violating axis is named in the same cycle (the checks do not stop at
// the first), and an undecodable word is named as such (its position is a
// saturated value, NaN carried).
TEST_F(DriveCommandLoopTest, EveryViolatingAxisIsNamedAndAnUndecodableWordAsSuch) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 2'000'000'000, -2'000'000'000,
          plem::core::MotionLimit{.lower_limit = -1e4, .upper_limit = 1e4, .velocity_limit = 1.0, .effort_limit = 5.0});
    set_actual_position(0, 50'000'000);  // axis 0 far outside ±0.05 rad
    start();
    std::vector<Event> stops;
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 2; }));
    wait_cycles(20);
    ASSERT_EQ(limit_stops(ctx_, stops), 2u);
    EXPECT_EQ(stops[0].joint_id, 0);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::POSITION));
    EXPECT_EQ(stops[1].joint_id, 1);
    EXPECT_EQ(stops[1].error_code, static_cast<uint16_t>(LimitStopReason::UNDECODABLE));
    EXPECT_TRUE(std::isnan(stops[1].value));
}

TEST_F(DriveCommandLoopTest, ACsvCommandPastItsLimitIsNamedWithTheCommand) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 2.0);  // twice axis 1's 1 rad/s
    start();
    std::vector<Event> stops;
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    EXPECT_EQ(stops[0].joint_id, 1);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::CSV_VELOCITY));
    EXPECT_NEAR(stops[0].value, 2.0f, 1e-4f) << "the command, rad/s";
}

TEST_F(DriveCommandLoopTest, ACspTargetWithNoDriveWordIsNamedWithTheCommand) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 1e6);  // ~1e12 counts: no int32 word
    start();
    std::vector<Event> stops;
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    EXPECT_EQ(stops[0].joint_id, 1);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::UNENCODABLE));
    EXPECT_EQ(stops[0].value, 1e6f) << "the command as sent";
}

// applied_command is what the cycle's frame carried, back in each
// axis's declared-mode unit, checked here against the domain bytes, so the
// clamp, the count rounding and the flip are all in it.
TEST_F(DriveCommandLoopTest, AppliedCommandIsTheCspAndCstWordsOnTheWire) {
    run_applying_csp(rest_rad() + 0.0005);
    retarget(50.0, rest_rad() + 0.0005);  // CST past its 5 Nm effort limit: the clamp reaches the wire
    ASSERT_TRUE(wait_for([&] { return wire_torque0() > 4.0f; }));
    wait_cycles();
    const auto sample = latest_sample();
    EXPECT_FLOAT_EQ(sample.applied_command[0], wire_torque0());
    EXPECT_LT(sample.applied_command[0], 50.0f) << "the clamped torque, not the command";
    EXPECT_FLOAT_EQ(sample.applied_command[1], static_cast<float>(*plem::fieldbus::ethercat::decode_position_word(
                                                   position_word(1), channels_.position, offset_, true)));
}

TEST_F(DriveCommandLoopTest, AppliedCommandIsTheCsvWordOnTheWire) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 0.3);
    start();
    ASSERT_TRUE(wait_for([&] { return velocity_word(1) != 0; }));
    wait_cycles();
    const auto sample = latest_sample();
    EXPECT_FLOAT_EQ(sample.applied_command[1],
                    static_cast<float>(plem::fieldbus::ethercat::to_angular_velocity(
                                           plem::core::units::PulsePerSecond::from_hardware(velocity_word(1), true),
                                           channels_.velocity)
                                           .value()));
    EXPECT_NEAR(sample.applied_command[1], 0.3f, 1e-4f);
}

// Not applying: a csp axis's frame carries the raw position mirror: its angle,
// or NaN when that word does not decode.
TEST_F(DriveCommandLoopTest, AppliedCommandOfAnUnappliedCspAxisIsItsMirrorOrNaN) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    start();
    wait_cycles();
    EXPECT_FLOAT_EQ(latest_sample().applied_command[1], static_cast<float>(rest_rad()));
    controller_->stop();

    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 2'000'000'000, -2'000'000'000,
          plem::core::MotionLimit{.lower_limit = -1e4, .upper_limit = 1e4, .velocity_limit = 1.0, .effort_limit = 5.0});
    start();
    wait_cycles();
    EXPECT_TRUE(std::isnan(latest_sample().applied_command[1]));
}

TEST_F(DriveCommandLoopTest, CspHoldsTheRawMirrorUntilEveryDriveIsEnabledThenFollowsTheCommand) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    const double target = rest_rad() + 0.0005;  // within one cycle's travel of the actual
    flags_->motion_enabled.store(true);
    stream(1.0, target);
    start();
    ASSERT_TRUE(wait_for([&] { return shared_->last_cycle_ns.load() != 0; }));
    wait_cycles(30);
    EXPECT_EQ(position_word(1), kRestWord) << "SWITCHED_ON: the gate is closed — the raw mirror, not the command";
    EXPECT_EQ(torque_word(0), 0);

    set_status(kOperationEnabled);
    EXPECT_TRUE(wait_for([&] { return position_word(1) == position_word_for(target); }));
    EXPECT_NE(torque_word(0), 0);
    EXPECT_EQ(velocity_word(1), 0);
    EXPECT_EQ(torque_word(1), 0);
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

TEST_F(DriveCommandLoopTest, CspWithoutAHostStampHoldsTheMirrorWithTheGateOpen) {
    // Stamp 0 is the initial payload (0.0 rad, a real, far-away position),
    // never a host's command; the freshness watchdog is not armed by it either.
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    start();
    ASSERT_TRUE(wait_for([&] { return flags_->all_drives_enabled.load(); }));
    wait_cycles(30);
    EXPECT_EQ(position_word(1), kRestWord);
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

TEST_F(DriveCommandLoopTest, CspJumpBeyondOneCycleTripsAndHoldsEveryAxisInThatFrame) {
    run_applying_csp(rest_rad() + 0.0005);
    struct Frame {  // owned by the hook (outlives the body); read after `captured`
        std::atomic<bool> captured{false};
        int16_t torque0 = 0;
        int32_t position1 = 0;
    };
    const auto frame = std::make_shared<Frame>();
    wire_.on_send([this, frame] {
        if (!frame->captured.load() && flags_->is_internal_protective_stop()) {
            frame->torque0 = torque_word(0);
            frame->position1 = position_word(1);
            frame->captured.store(true);
        }
    });
    retarget(1.0, rest_rad() + 0.0105);  // 0.01 rad past the applied target, ten cycles' travel
    ASSERT_TRUE(wait_for([&] { return frame->captured.load(); }));
    EXPECT_EQ(frame->torque0, 0) << "the earlier CST axis goes neutral in the violating frame";
    EXPECT_EQ(frame->position1, position_word_for(rest_rad() + 0.0005))
        << "the CSP axis keeps its previous target: the stop controlword goes out only next frame, and the raw "
           "mirror under Enable Operation would step the target back";
    EXPECT_TRUE(wait_for([&] { return count(plem::core::monitoring::EventType::PROTECTIVE_STOP) >= 1; }));
}

// A host that goes silent while a CSP axis moves: the frame that first holds
// the raw mirror on 0x607A must already carry the stop controlword.
// Enable Operation with the target stepped back to the measured position for
// even one frame is a backward command step a drive may trip on (A6B Err27 on
// hardware).
TEST_F(DriveCommandLoopTest, AStaleHostStopsInTheSameFrameItsCspAxisGoesNeutral) {
    run_applying_csp(rest_rad() + 0.0005);
    struct Frame {  // owned by the hook (outlives the body); read after `captured`
        std::atomic<bool> captured{false};
        std::atomic<int> first_stopped{-1};  // the first hooked frame's stop state: 0 = still applying
        uint16_t control1 = 0;
    };
    const auto frame = std::make_shared<Frame>();
    wire_.on_send([this, frame] {
        if (frame->first_stopped.load() < 0) {
            frame->first_stopped.store(flags_->is_external_protective_stop() || position_word(1) == kRestWord ? 1 : 0);
        }
        if (!frame->captured.load() && flags_->is_external_protective_stop() && position_word(1) == kRestWord) {
            frame->control1 = controlword(1);
            frame->captured.store(true);
        }
    });
    stop_streaming();  // the host goes silent → freshness trips EXTERNAL
    ASSERT_TRUE(wait_for([&] { return frame->captured.load(); }));
    ASSERT_EQ(frame->first_stopped.load(), 0) << "the stream went stale before the hook saw an applying frame";
    EXPECT_EQ(frame->control1 & 0x8F, 0x07) << "the neutral frame must carry the engage's first step (Disable "
                                               "operation), controlword 0x"
                                            << std::hex << frame->control1;
}

TEST_F(DriveCommandLoopTest, AResetInTheCycleAfterAViolationStillForcesTheEngage) {
    // The safety step consumes the reset before the brake step, so the brake
    // step never sees the INTERNAL level rise; the one-shot trip must carry
    // the engage. Disabling in the same instant keeps the command from
    // re-latching, so the only PROTECTIVE_STOP here is the trip's.
    run_applying_csp(rest_rad() + 0.0005);
    (void)count(plem::core::monitoring::EventType::PROTECTIVE_STOP);
    const auto armed = std::make_shared<std::atomic<bool>>(false);  // owned by the hook
    wire_.on_send([this, armed] {
        if (!armed->load() && flags_->is_internal_protective_stop()) {
            flags_->motion_enabled.store(false);
            flags_->reset_error.store(true);
            armed->store(true);
        }
    });
    retarget(1.0, rest_rad() + 0.0105);
    ASSERT_TRUE(wait_for([&] { return armed->load() && !flags_->reset_error.load(); }));
    EXPECT_TRUE(wait_for([&] { return count(plem::core::monitoring::EventType::PROTECTIVE_STOP) >= 1; }));
    EXPECT_FALSE(flags_->is_protective_stop_active()) << "the reset did clear the latch";
    EXPECT_EQ(count(EventType::LIMIT_STOP), 1u) << "named although the reset consumed the latch next cycle";
}

TEST_F(DriveCommandLoopTest, AResetWithTheSameBadCommandRelatchesAndAFixedOneResumes) {
    run_applying_csp(rest_rad() + 0.0005);
    retarget(1.0, rest_rad() + 0.0105);
    ASSERT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
    ASSERT_TRUE(wait_for([&] { return count(plem::core::monitoring::EventType::PROTECTIVE_STOP) >= 1; }));

    flags_->reset_error.store(true);  // the bad command still streams
    EXPECT_TRUE(wait_for([&] { return count(plem::core::monitoring::EventType::PROTECTIVE_STOP) >= 2; }));
    EXPECT_TRUE(wait_for([&] { return count(EventType::LIMIT_STOP) >= 2; })) << "each relatch is named again";
    EXPECT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
    EXPECT_EQ(position_word(1), kRestWord);

    // The first application after the stop is checked against the actual.
    retarget(1.0, rest_rad() + 0.0005);
    wait_cycles(10);
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->is_protective_stop_active(); }));
    EXPECT_TRUE(wait_for([&] { return position_word(1) == position_word_for(rest_rad() + 0.0005); }));
}

TEST_F(DriveCommandLoopTest, CspTargetPastThePositionLimitTrips) {
    // Rest just inside the upper limit; a target one small step beyond it.
    const double rest = 0.9995;
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, kOffset,
          plem::fieldbus::ethercat::encode_position_checked(rest, make_servo_info().factors.position, kOffset, true)
              ->word);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 1.0004);
    start();
    EXPECT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
    EXPECT_NE(position_word(1), position_word_for(1.0004));
    std::vector<Event> stops;
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::CSP_RANGE));
    EXPECT_NEAR(stops[0].value, 1.0004f, 1e-5f) << "the target, rad";
}

TEST_F(DriveCommandLoopTest, CspAxisWhoseFeedbackDoesNotDecodeIsNeverCommanded) {
    // A 2e9 offset with raw -2e9: the joint-frame decode saturates (to ~1700
    // rad). Limits wide enough that the saturated actual passes the limit
    // step, so only the decode check stands between it and a step checked
    // against a position the axis is not at. The mirror keeps the raw word.
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 2'000'000'000, -2'000'000'000,
          plem::core::MotionLimit{.lower_limit = -1e4, .upper_limit = 1e4, .velocity_limit = 1.0, .effort_limit = 5.0});
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    // Command the saturated angle itself (flip: -(−2e9 − 2e9) clamps to INT32_MAX
    // counts); the step and the limits would pass against it.
    const double saturated = plem::fieldbus::ethercat::to_angle(Pulse(std::numeric_limits<int32_t>::max()),
                                                                make_servo_info().factors.position)
                                 .value();
    stream(1.0, saturated);
    start();
    EXPECT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
    wait_cycles(20);
    EXPECT_EQ(position_word(1), -2'000'000'000);
}

TEST_F(DriveCommandLoopTest, UndecodableCspFeedbackLatchesWhileDisabledAndRefusesReset) {
    // The axis must never be released on feedback it cannot place.
    // The safety step checks it like a limit violation, independent of the
    // command gate (disabled here, and the other axis never reaches OE), and a
    // reset is refused while it stands.
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 2'000'000'000, -2'000'000'000,
          plem::core::MotionLimit{.lower_limit = -1e4, .upper_limit = 1e4, .velocity_limit = 1.0, .effort_limit = 5.0});
    start();
    EXPECT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->reset_error.load(); }));
    wait_cycles(20);
    EXPECT_TRUE(flags_->is_internal_protective_stop()) << "reset refused while the feedback does not decode";
    EXPECT_EQ(position_word(1), -2'000'000'000);

    set_actual_position(1, 1'999'990'000);  // decodes again
    wait_cycles(10);
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->is_protective_stop_active(); }));
}

// A saturated joint-frame position repeats one value while the drive
// moves, so a bounded axis in any mode is held like a limit violation while
// its word does not decode, not only a csp one. Limits wide enough that the
// saturated actual (~1700 rad) passes them.
TEST_F(DriveCommandLoopTest, UndecodableFeedbackLatchesABoundedCstOrCsvAxisToo) {
    for (const auto mode :
         {plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE, plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY}) {
        SCOPED_TRACE(static_cast<int>(mode));
        build(mode, 2'000'000'000, -2'000'000'000,
              plem::core::MotionLimit{
                  .lower_limit = -1e4, .upper_limit = 1e4, .velocity_limit = 1.0, .effort_limit = 5.0});
        start();
        EXPECT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
        flags_->reset_error.store(true);
        EXPECT_TRUE(wait_for([&] { return !flags_->reset_error.load(); }));
        wait_cycles(20);
        EXPECT_TRUE(flags_->is_internal_protective_stop()) << "reset refused while the feedback does not decode";
        controller_->stop();
    }
}

// Before the first real frame the image is zero, and with the
// widest offset (INT32_MIN, no flip) a zero word does not decode; that is no
// feedback yet, not a saturated position, so it must not latch a stop the
// valid feedback that follows would have to reset. The shipped files' zero
// offsets (example: 0; drive_synthetic: 1234, -77, …; the loader's maximal
// file: -5 flipped) start the same way without a stop.
TEST_F(DriveCommandLoopTest, AStartFromTheZeroImageLatchesNothingForValidFeedback) {
    const std::pair<int32_t, bool> cases[] = {
        {std::numeric_limits<int32_t>::min(), false}, {0, false}, {-5, true}, {1234, false}, {-77, false}};
    for (const auto& [offset, flip] : cases) {
        SCOPED_TRACE(offset);
        const int32_t at_zero = offset;  // the raw word at q = 0, flipped or not
        build(plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE, offset, at_zero,
              plem::core::MotionLimit{
                  .lower_limit = -3.0, .upper_limit = 3.0, .velocity_limit = 1.0, .effort_limit = 5.0},
              flip);
        set_status(0);
        set_actual_position(0, 0);
        set_actual_position(1, 0);
        start();
        wait_cycles(20);
        // Position before status: the controller must never see the first
        // nonzero statusword beside the old zero word (a real frame carries both).
        set_actual_position(1, at_zero);
        set_status(kSwitchedOn);
        wait_cycles(20);
        EXPECT_FALSE(flags_->is_protective_stop_active());
        controller_->stop();
    }
}

// An axis without end stops (±inf travel, raw zero, endless) is not tripped by
// where it is: far away, or on the one word a flipped int32 decode saturates
// (INT32_MIN) as the count wraps past it.
TEST_F(DriveCommandLoopTest, AnUnboundedAxisIsNeverStoppedForItsPosition) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    build(
        plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY, 0, 1'000'000'000,
        plem::core::MotionLimit{.lower_limit = -kInf, .upper_limit = kInf, .velocity_limit = 1.0, .effort_limit = 5.0});
    start();
    wait_cycles(30);
    EXPECT_FALSE(flags_->is_protective_stop_active());
    set_actual_position(1, std::numeric_limits<int32_t>::min());
    wait_cycles(30);
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

// Endless axis (limits.position: null) through the RT loop

plem::core::MotionLimit endless_limit() {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    return plem::core::MotionLimit{
        .lower_limit = -kInf, .upper_limit = kInf, .velocity_limit = 1.0, .effort_limit = 5.0};
}

// The same file with bounded travel cannot name a target past int32 counts:
// the command is refused as UNENCODABLE.
TEST_F(DriveCommandLoopTest, ABoundedCspAxisStillRefusesATargetPastInt32Counts) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 0, 0,
          plem::core::MotionLimit{.lower_limit = -1e6, .upper_limit = 1e6, .velocity_limit = 1.0, .effort_limit = 5.0});
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 4000.0);  // ~4.2e9 counts
    start();
    std::vector<Event> stops;
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    EXPECT_EQ(stops[0].joint_id, 1);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::UNENCODABLE));
}

// A cst null axis reports one continuous angle across the 0x6064 wrap.
TEST_F(DriveCommandLoopTest, AnEndlessCstAxisIsContinuousAcrossTheWordWrap) {
    constexpr int32_t kMax = std::numeric_limits<int32_t>::max();
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE, 0, kMax - 10, endless_limit());
    start();
    wait_cycles(3);
    set_actual_position(1, std::numeric_limits<int32_t>::min() + 10);
    const double expected = plem::fieldbus::ethercat::endless_count_to_angle(-(int64_t{kMax} + 11), channels_.position);
    EXPECT_TRUE(wait_for([&] {
        plem::core::JointActualValues<2> actual{};
        return shared_->joint_actual.read(actual) && actual.states[1].position_actual_value == expected;
    })) << "21 counts on, not a 2^32 jump";
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

// An endless axis whose count leaves ±kEndlessCountDomain stops (UNDECODABLE),
// and the stop stays until PLEM restarts: a reset is refused.
TEST_F(DriveCommandLoopTest, AnEndlessAxisPastItsDomainLatchesUndecodableUntilRestart) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE, 0, 1000, endless_limit());
    const auto armed = std::make_shared<std::atomic<bool>>(false);
    wire_.on_send([this, armed] {
        // On the RT thread, between frames, once the count is seeded.
        if (bus_->servo_devices()[1]->is_initialized() && !armed->exchange(true)) {
            plem::fieldbus::test::DeviceServoTestAccess::set_position_count(
                *bus_->servo_devices()[1], plem::fieldbus::ethercat::kEndlessCountDomain - 5);
        }
    });
    start();
    std::vector<Event> stops;
    ASSERT_TRUE(wait_for([&] { return limit_stops(ctx_, stops) >= 1; }));
    EXPECT_EQ(stops[0].joint_id, 1);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::UNDECODABLE));
    flags_->reset_error.store(true);
    EXPECT_TRUE(wait_for([&] { return !flags_->reset_error.load(); }));
    wait_cycles(20);
    EXPECT_TRUE(flags_->is_internal_protective_stop()) << "sticky: reset refused";
}

// A frame whose working counter is incomplete is no observation of an endless
// axis, so its count holds, and so must the word its raw hold writes: the
// hold puts the last observed word on 0x607A, the word the held angle stands
// for, not the incomplete frame's word. Commands that resume during such a
// frame are checked from that same word, so the wire never steps further than
// the step check allowed. Here incomplete frames report a word 5000 counts
// (more than one cycle's travel) from the axis while it is held, and the
// host's first command (the held position) arrives inside that stretch.
TEST_F(DriveCommandLoopTest, AnEndlessCspHoldOnAnIncompleteFrameKeepsTheObservedWordAndResumesWithoutAStep) {
    constexpr int32_t kRest = 1'000'000;
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 0, kRest, endless_limit());
    const auto& ch = channels_.position;
    const double rest = plem::fieldbus::ethercat::endless_count_to_angle(-int64_t{kRest}, ch);  // flipped
    const auto travel_counts = static_cast<int32_t>(1e-3 / plem::fieldbus::ethercat::endless_count_to_angle(1, ch));
    struct Trace {
        std::atomic<int32_t> widest{0};
        std::atomic<bool> held_elsewhere{false};
        int32_t last = kRest;
    };
    const auto trace = std::make_shared<Trace>();
    per_cycle([this, trace, rest](int cycle) {
        const int32_t word = position_word(1);
        const int32_t step = plem::fieldbus::ethercat::wrap_diff(word, trace->last);
        trace->widest.store(std::max(trace->widest.load(), step < 0 ? -step : step));
        trace->last = word;
        if (cycle >= 21 && cycle <= 24 && word != kRest) {
            trace->held_elsewhere.store(true);
        }
        if (cycle == 20) {  // frames 21..23: incomplete, a word far from the axis
            FakeEcrt::set_domain_wc_state(EC_WC_INCOMPLETE);
            set_actual_position(1, kRest + 5000);
        }
        if (cycle == 23) {
            FakeEcrt::set_domain_wc_state(EC_WC_COMPLETE);
            set_actual_position(1, kRest);
        }
        if (cycle >= 21) {  // the host resumes with the held position, inside the incomplete stretch
            write_command(1.0, rest);
        }
    });
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    start();
    wait_cycles(40);
    controller_->stop();
    EXPECT_FALSE(trace->held_elsewhere.load()) << "the hold and the resumed command keep the observed word";
    EXPECT_LE(trace->widest.load(), travel_counts) << "no wire step past one cycle's travel";
    EXPECT_EQ(count(EventType::LIMIT_STOP), 0u);
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

// Frame validity through the RT loop. At startup the first frames come back
// incomplete with an enabled statusword and a word half a period away; the
// axis seeds only on the first complete frame. Later the bus loses three
// frames (incomplete, zero word, under the 5-cycle WKC watchdog): the
// reported position holds, nothing latches, and the applied command stays on
// the wire; the complete frames after it carry on from the same count.
TEST_F(DriveCommandLoopTest, AnEndlessAxisObservesOnlyCompleteFramesAtStartupThroughALossAndAfter) {
    constexpr int32_t kRest = 1'000'000;
    const int32_t junk = static_cast<int32_t>(static_cast<uint32_t>(kRest) + (uint32_t{1} << 31) - 5);
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 0, junk, endless_limit());
    const double rest = plem::fieldbus::ethercat::endless_count_to_angle(-int64_t{kRest}, channels_.position);
    struct Trace {
        std::atomic<bool> moved{false};
        std::atomic<bool> off_wire{false};
        std::atomic<bool> seeded_late{false};
    };
    const auto trace = std::make_shared<Trace>();
    FakeEcrt::set_domain_wc_state(EC_WC_INCOMPLETE);  // frames 0..4
    per_cycle([this, trace, rest](int cycle) {
        const auto& servo = *bus_->servo_devices()[1];
        if (cycle <= 4 && servo.is_initialized()) {
            trace->seeded_late.store(true);  // seeded from an incomplete frame
        }
        if (cycle >= 7) {
            trace->moved.store(trace->moved.load() || servo.get_actual_position().value() != rest);
        }
        if (cycle >= 10 && position_word(1) != kRest) {
            trace->off_wire.store(true);
        }
        if (cycle == 4) {
            FakeEcrt::set_domain_wc_state(EC_WC_COMPLETE);
            set_actual_position(1, kRest);
        }
        if (cycle == 20) {  // frames 21..23 lost
            FakeEcrt::set_domain_wc_state(EC_WC_INCOMPLETE);
            set_actual_position(1, 0);
        }
        if (cycle == 23) {
            FakeEcrt::set_domain_wc_state(EC_WC_COMPLETE);
            set_actual_position(1, kRest);
        }
        if (cycle >= 8) {
            write_command(1.0, rest);
        }
    });
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    start();
    wait_cycles(40);
    controller_->stop();
    EXPECT_FALSE(trace->seeded_late.load());
    EXPECT_FALSE(trace->moved.load()) << "the position is the complete frames' count throughout";
    EXPECT_FALSE(trace->off_wire.load()) << "the applied command stays on 0x607A through the loss";
    EXPECT_EQ(count(EventType::LIMIT_STOP), 0u) << "no false UNDECODABLE";
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

// Per-cycle csp targets walk an endless axis through the word
// wrap (80000000h → 7FFFFFFFh, flipped), each step under one cycle's travel;
// the drive here follows 0x607A exactly.
TEST_F(DriveCommandLoopTest, AnEndlessCspAxisStepsAcrossTheWordWrap) {
    constexpr int32_t kRest = std::numeric_limits<int32_t>::min() + 3000;
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 0, kRest, endless_limit());
    const auto& ch = channels_.position;
    const int64_t start_count = -int64_t{kRest};  // flipped joint count
    struct Trace {
        std::atomic<bool> wrapped{false};
        std::atomic<int32_t> widest{0};
        int32_t last = kRest;
    };
    const auto trace = std::make_shared<Trace>();
    per_cycle([this, trace, start_count, &ch](int cycle) {
        const int32_t word = position_word(1);
        const int32_t step = plem::fieldbus::ethercat::wrap_diff(word, trace->last);
        trace->widest.store(std::max(trace->widest.load(), step < 0 ? -step : step));
        trace->wrapped.store(trace->wrapped.load() || (trace->last < 0 && word > 0 && step < 0));
        trace->last = word;
        set_actual_position(1, word);                // the drive follows
        const int k = std::clamp(cycle - 9, 0, 10);  // 900 counts a cycle, ten steps
        write_command(1.0, plem::fieldbus::ethercat::endless_count_to_angle(start_count + 900 * k, ch));
    });
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    start();
    wait_cycles(40);
    controller_->stop();
    EXPECT_FALSE(flags_->is_protective_stop_active());
    EXPECT_EQ(count(EventType::LIMIT_STOP), 0u);
    EXPECT_TRUE(trace->wrapped.load());
    EXPECT_LE(trace->widest.load(), 900 + 1);
    EXPECT_EQ(bus_->servo_devices()[1]->get_actual_position().value(),
              plem::fieldbus::ethercat::endless_count_to_angle(start_count + 9000, ch));
}

// The hold of an endless axis is the last observed word, which can lag the
// drive across incomplete frames: the drive reports A, frames 21–22 come back
// incomplete, and the first complete frame reports B. The host resumes on that
// same frame with B. The step is checked from the word last transmitted (A),
// not from the newest feedback: B beyond one cycle's travel stops with
// CSP_STEP, B within it resumes. Every frame that is not the stop frame keeps
// its 0x607A step within one cycle's travel. The stop frame writes the raw
// hold, B, like any neutral.
class EndlessRecoveryTest : public DriveCommandLoopTest {
  protected:
    struct Trace {
        std::atomic<int32_t> widest{0};  // over frames without a protective stop
        int32_t last = kRest;
    };

    /// Runs the recovery with the drive reporting kRest + `moved` on frame 23.
    std::shared_ptr<Trace> run_recovery(int32_t moved) {
        build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 0, kRest, endless_limit());
        const auto& ch = channels_.position;
        const double b = plem::fieldbus::ethercat::endless_count_to_angle(-(int64_t{kRest} + moved), ch);  // flipped
        auto trace = std::make_shared<Trace>();
        per_cycle([this, trace, moved, b](int cycle) {
            const int32_t word = position_word(1);
            const int32_t step = plem::fieldbus::ethercat::wrap_diff(word, trace->last);
            if (!flags_->is_protective_stop_active()) {
                trace->widest.store(std::max(trace->widest.load(), step < 0 ? -step : step));
            }
            trace->last = word;
            if (cycle == 20) {  // frames 21..22: incomplete, the drive's word not refreshed
                FakeEcrt::set_domain_wc_state(EC_WC_INCOMPLETE);
            }
            if (cycle == 22) {  // frame 23: complete, the drive is at B
                FakeEcrt::set_domain_wc_state(EC_WC_COMPLETE);
                set_actual_position(1, kRest + moved);
            }
            if (cycle >= 22) {  // the host resumes with B on the recovery frame
                write_command(1.0, b);
            }
        });
        flags_->motion_enabled.store(true);
        set_status(kOperationEnabled);
        start();
        wait_cycles(40);
        controller_->stop();
        return trace;
    }

    int32_t travel_counts() const {
        return static_cast<int32_t>(1e-3 / plem::fieldbus::ethercat::endless_count_to_angle(1, channels_.position));
    }

    static constexpr int32_t kRest = 1'000'000;
};

TEST_F(EndlessRecoveryTest, AResumeFarFromTheTransmittedWordStopsWithCspStep) {
    const auto trace = run_recovery(5000);  // beyond one cycle's travel (1043 counts)
    std::vector<Event> stops;
    ASSERT_EQ(limit_stops(ctx_, stops), 1u);
    EXPECT_EQ(stops[0].joint_id, 1);
    EXPECT_EQ(stops[0].error_code, static_cast<uint16_t>(LimitStopReason::CSP_STEP));
    EXPECT_LE(trace->widest.load(), travel_counts()) << "no accepted frame jumps A→B";
}

TEST_F(EndlessRecoveryTest, AResumeWithinOneCycleOfTheTransmittedWordContinues) {
    const auto trace = run_recovery(500);
    EXPECT_EQ(count(EventType::LIMIT_STOP), 0u);
    EXPECT_FALSE(flags_->is_protective_stop_active());
    EXPECT_LE(trace->widest.load(), travel_counts());
    EXPECT_EQ(position_word(1), kRest + 500);
}

// A protective stop, the drive moved while held (complete frames), a reset:
// the hold followed the drive, so the first application after the reset is
// checked from where the drive is: no false CSP_STEP.
TEST_F(DriveCommandLoopTest, AnEndlessCspAxisResumesAfterAResetWhereTheDriveWasMovedWhileStopped) {
    constexpr int32_t kRest = 1'000'000;
    constexpr int32_t kMoved = kRest + 50'000;
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, 0, kRest, endless_limit());
    const auto& ch = channels_.position;
    const double a = plem::fieldbus::ethercat::endless_count_to_angle(-int64_t{kRest}, ch);
    const double b = plem::fieldbus::ethercat::endless_count_to_angle(-int64_t{kMoved}, ch);
    per_cycle([this, a, b](int cycle) {
        if (cycle == 10) {
            flags_->set_protective_stop(ProtectiveStopSource::EXTERNAL);
        }
        if (cycle == 14) {  // moved by hand while stopped
            set_actual_position(1, kMoved);
        }
        if (cycle == 20) {
            flags_->reset_error.store(true);
        }
        write_command(1.0, cycle >= 15 ? b : a);  // the host follows joint_states
    });
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    start();
    wait_cycles(40);
    controller_->stop();
    EXPECT_FALSE(flags_->is_protective_stop_active()) << "the reset cleared it";
    EXPECT_EQ(count(EventType::LIMIT_STOP), 0u);
    EXPECT_EQ(position_word(1), kMoved);
}

TEST_F(DriveCommandLoopTest, CspSmallStepsAreJudgedAgainstThePreviousTargetNotTheActual) {
    // The fake axis never moves: each step is within one cycle's travel of the
    // last applied target, while the target walks well past one cycle's
    // travel from the actual.
    run_applying_csp(rest_rad() + 0.0005);
    for (int k = 2; k <= 6; ++k) {
        const double target = rest_rad() + 0.0005 * k;
        retarget(1.0, target);
        ASSERT_TRUE(wait_for([&] { return position_word(1) == position_word_for(target); })) << k;
    }
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

TEST_F(DriveCommandLoopTest, CsvFollowsTheCommandWhileEnabledAndWritesZeroOtherwise) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    const int32_t word = plem::fieldbus::ethercat::encode_velocity_checked(0.5, channels_.velocity, true)->word;
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 0.5);
    start();
    EXPECT_TRUE(wait_for([&] { return velocity_word(1) == word; }));
    EXPECT_EQ(position_word(1), kRestWord) << "CSV keeps the position mirror";
    EXPECT_EQ(torque_word(1), 0);
    flags_->motion_enabled.store(false);
    EXPECT_TRUE(wait_for([&] { return velocity_word(1) == 0; }));
}

TEST_F(DriveCommandLoopTest, CsvOverTheVelocityLimitTripsAndNeutralizes) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 0.5);
    start();
    ASSERT_TRUE(wait_for([&] { return velocity_word(1) != 0; }));
    retarget(1.0, 1.5);  // limit 1.0 rad/s
    EXPECT_TRUE(wait_for([&] { return flags_->is_internal_protective_stop(); }));
    EXPECT_TRUE(wait_for([&] { return velocity_word(1) == 0 && torque_word(0) == 0; }));
}

TEST_F(DriveCommandLoopTest, RtSampleCmdTorqueIsNanOnANonCstAxis) {
    // The command slot carries rad on a CSP axis; it is
    // not a torque, so command_torque says "none" instead of a wrong unit. CST
    // axes keep the command they were sent.
    const double target = rest_rad() + 0.0005;
    run_applying_csp(target);
    auto* samples = ctx_.queue<plem::core::monitoring::RtSample>();
    bool seen = false;
    ASSERT_TRUE(wait_for([&] {
        while (auto* s = samples->front()) {
            if (s->command_torque[0] == 1.0f) {
                seen = seen || std::isnan(s->command_torque[1]);
                EXPECT_TRUE(std::isnan(s->command_torque[1])) << s->command_torque[1];
            }
            samples->pop();
        }
        return seen;
    }));
}

// command_position·command_velocity follow the same mode rule as command_torque: a CSP axis's
// command is its commanded position, a CSV axis's its commanded velocity. The
// other slot (and every slot of a CST axis) stays the motion-API target
// conduit (joint_target), which a higher-layer host fills and the drive layer never
// writes.
TEST_F(DriveCommandLoopTest, RtSampleCmdPosIsTheCommandOnACspAxis) {
    const double target = rest_rad() + 0.0005;
    run_applying_csp(target);
    auto* samples = ctx_.queue<plem::core::monitoring::RtSample>();
    bool seen = false;
    ASSERT_TRUE(wait_for([&] {
        while (auto* s = samples->front()) {
            if (s->command_torque[0] == 1.0f && s->command_position[1] != 0.0f) {
                seen = true;
                EXPECT_EQ(s->command_position[1], static_cast<float>(target));
                EXPECT_EQ(s->command_velocity[1], 0.0f) << "a CSP command is not a velocity";
                EXPECT_EQ(s->command_position[0], 0.0f) << "a CST axis keeps the conduit value";
            }
            samples->pop();
        }
        return seen;
    }));
}

TEST_F(DriveCommandLoopTest, RtSampleCmdVelIsTheCommandOnACsvAxis) {
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, 0.5);
    start();
    auto* samples = ctx_.queue<plem::core::monitoring::RtSample>();
    bool seen = false;
    ASSERT_TRUE(wait_for([&] {
        while (auto* s = samples->front()) {
            if (s->command_torque[0] == 1.0f && s->command_velocity[1] != 0.0f) {
                seen = true;
                EXPECT_EQ(s->command_velocity[1], 0.5f);
                EXPECT_EQ(s->command_position[1], 0.0f) << "a CSV command is not a position";
            }
            samples->pop();
        }
        return seen;
    }));
}

// Requested stop and the release hold (a bus without a cst axis)
// Both axes csp (or axis 1 csv): axis 0 rests at 0, axis 1 moves. A stop
// ramps every command to rest at its deceleration, then engages; not settled
// in time it commands Quick stop. Without a cst axis a release holds the
// position it found until the host commands, and the stream watch arms on
// the first command after the release.

class DriveStopLoopTest : public DriveCommandLoopTest {
  protected:
    static constexpr double kDecel = 2.0;  // rad/s²: 0.5 rad/s rests in 250 cycles
    static constexpr double kSpeed = 0.5;  // rad/s: 0.0005 rad a cycle

    void build_stop_bus(plem::fieldbus::OperationMode axis1_mode = plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION,
                        plem::core::MotionLimit axis1_limit = wide_limit()) {
        axis0_mode_ = plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION;
        build(axis1_mode, kOffset, kRestWord, axis1_limit);
        const std::vector<double> decelerations{kDecel, kDecel};
        controller_->set_drive_stop(decelerations);
    }
    double target_rad(int32_t word) {
        return *plem::fieldbus::ethercat::decode_position_word(word, channels_.position, offset_, true);
    }

    /// Released and moving axis 1 at kSpeed (one command a cycle from the RT
    /// thread); `targets` gets axis 1's 0x607A angle and `velocities` its
    /// 0x60FF word each cycle. Returns once the motion is under way.
    struct Trace {
        std::mutex mutex;
        std::vector<double> targets;
        std::vector<int32_t> velocities;
        std::atomic<bool> commanding{true};
    };
    std::shared_ptr<Trace> run_moving(plem::fieldbus::OperationMode axis1_mode) {
        build_stop_bus(axis1_mode);
        flags_->motion_enabled.store(true);
        set_status(kOperationEnabled);
        controller_->open_command_epoch();
        auto trace = std::make_shared<Trace>();
        const bool csv = axis1_mode == plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY;
        const double from = rest_rad();
        per_cycle([this, trace, csv, from](int cycle) {
            if (trace->commanding.load()) {
                write_command(0.0, csv ? kSpeed : from + kSpeed * 1e-3 * (cycle + 1));
            }
            std::lock_guard lock(trace->mutex);
            trace->targets.push_back(target_rad(position_word(1)));
            trace->velocities.push_back(velocity_word(1));
        });
        start();
        wait_cycles(60);
        return trace;
    }
    std::vector<double> targets(Trace& trace) {
        std::lock_guard lock(trace.mutex);
        return trace.targets;
    }
};

TEST_F(DriveStopLoopTest, AStopRampsAMovingCspAxisToRestAtItsDecelerationThenEngages) {
    auto trace = run_moving(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    size_t from = 0;
    {
        std::lock_guard lock(trace->mutex);
        from = trace->targets.size();
    }
    controller_->request_ramp_stop();
    trace->commanding.store(false);  // the host goes quiet: the stop needs nothing from it
    ASSERT_TRUE(wait_for([&] { return !flags_->motion_enabled.load(); }, std::chrono::milliseconds(3000)));
    // The loop clears motion first and the stop's running flag later in the same cycle; wait for it.
    EXPECT_TRUE(wait_for([&] { return !controller_->ramp_stop_active(); })) << "the stop ended";
    EXPECT_TRUE(wait_for([&] { return (controlword(1) & 0x8F) == 0x07; })) << "the engage's first step (Disable "
                                                                              "operation)";
    EXPECT_FALSE(flags_->is_protective_stop_active());
    EXPECT_EQ(count(EventType::LIMIT_STOP), 0u);

    // From the request to rest: the speed falls by at most a·T a cycle (plus
    // the count rounding of two targets), never rises, never turns back.
    const auto t = targets(*trace);
    const double rounding = 2.0 * std::abs(target_rad(kRestWord + 1) - target_rad(kRestWord)) / 1e-3;
    // A few cycles before the request: the loop may take it a cycle or two before this thread looked.
    ASSERT_GT(from, 10u);
    double speed = kSpeed;
    double travelled = 0.0;
    size_t k = from - 10;
    for (; k + 1 < t.size() && speed > 0.0; ++k) {
        const double next = (t[k + 1] - t[k]) / 1e-3;
        EXPECT_LE(next, speed + rounding) << "never speeds up, cycle " << k - from;
        EXPECT_GE(next, speed - kDecel * 1e-3 - rounding) << "at most a·T, cycle " << k - from;
        speed = next < rounding ? 0.0 : next;  // under a count a cycle: at rest (never negative, see above)
        travelled += t[k + 1] - t[k];
    }
    EXPECT_EQ(speed, 0.0) << "came to rest";
    EXPECT_NEAR(static_cast<double>(k - from), kSpeed / kDecel / 1e-3, 12.0) << "in v/a";
    EXPECT_NEAR(travelled, kSpeed * kSpeed / (2 * kDecel) + 10 * kSpeed * 1e-3, 0.006) << "over v²/2a";
}

TEST_F(DriveStopLoopTest, AStopRampsACsvAxisToZeroVelocityThenEngages) {
    auto trace = run_moving(plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY);
    const int32_t cruising = velocity_word(1);
    ASSERT_NE(cruising, 0);
    controller_->request_ramp_stop();
    trace->commanding.store(false);
    ASSERT_TRUE(wait_for([&] { return !flags_->motion_enabled.load(); }, std::chrono::milliseconds(3000)));
    std::vector<int32_t> words;
    {
        std::lock_guard lock(trace->mutex);
        words = trace->velocities;
    }
    const auto last_cruise = std::find(words.rbegin(), words.rend(), cruising);
    ASSERT_NE(last_cruise, words.rend());
    const auto first_lower = last_cruise.base();
    for (auto it = first_lower; it + 1 != words.end(); ++it) {
        EXPECT_LE(std::abs(*(it + 1)), std::abs(*it)) << "the speed only falls";
    }
    EXPECT_GT(std::count_if(first_lower, words.end(), [](int32_t w) { return w != 0; }), 200)
        << "a ramp of about 250 cycles, not a cut";
    EXPECT_EQ(words.back(), 0);
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

TEST_F(DriveStopLoopTest, AStopNotConfirmedInTimeCommandsQuickStopAndEndsOnceTheDrivesAreOff) {
    auto trace = run_moving(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    set_actual_velocity(1, 1'000'000);  // the axis keeps reporting speed: never settles
    controller_->request_ramp_stop();
    trace->commanding.store(false);
    // 250 ms of ramp + kRampStopSettleTimeoutNs, then Quick stop on every enabled axis.
    ASSERT_TRUE(wait_for([&] { return (controlword(1) & 0x0007) == 0x0002; }, std::chrono::milliseconds(2000)));
    EXPECT_EQ(controlword(0) & 0x0007, 0x0002);
    EXPECT_EQ(count(EventType::RAMP_STOP_TIMEOUT), 1u);
    EXPECT_TRUE(controller_->ramp_stop_active()) << "waiting for the drives' own ramp";

    set_status(0x0007);  // Quick stop active
    wait_cycles(5);
    EXPECT_EQ(controlword(1) & 0x0007, 0x0002) << "Quick stop is held while the drive ramps down";
    set_status(0x0040);  // 0x605A = 2: Switch on disabled
    ASSERT_TRUE(wait_for([&] { return !controller_->ramp_stop_active(); }));
    EXPECT_FALSE(flags_->motion_enabled.load());
    EXPECT_TRUE(wait_for([&] { return controller_->brake_engaged() == 0x03; })) << "both engaged";
}

// A drive that stays in Quick stop active (0x605A not 2): after
// kQuickStopTimeoutNs the stop ends with Disable voltage (transition 12), and
// the engage sequence keeps that word: it has no step out of Quick stop active.
TEST_F(DriveStopLoopTest, AQuickStopTheDriveDoesNotEndIsCutWithDisableVoltage) {
    auto trace = run_moving(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    set_actual_velocity(1, 1'000'000);
    controller_->request_ramp_stop();
    trace->commanding.store(false);
    ASSERT_TRUE(wait_for([&] { return (controlword(1) & 0x0007) == 0x0002; }, std::chrono::milliseconds(2000)));
    set_status(0x0007);  // Quick stop active, and it stays there
    const auto quick = std::chrono::steady_clock::now();
    ASSERT_TRUE(wait_for([&] { return !controller_->ramp_stop_active(); }, std::chrono::milliseconds(3000)));
    EXPECT_GE(std::chrono::steady_clock::now() - quick, std::chrono::milliseconds(1900)) << "kQuickStopTimeoutNs";
    wait_cycles(5);
    EXPECT_EQ(controlword(1) & 0x008F, 0x0000) << "Disable voltage, held";
    EXPECT_EQ(controlword(0) & 0x008F, 0x0000);
    EXPECT_FALSE(flags_->motion_enabled.load());
}

// A stop not confirmed in time commands Quick stop (transition 11) to every
// enabled drive, so a csp or csv axis whose profile lacks that row is refused
// before the loop runs.
TEST_F(DriveStopLoopTest, ACspAxisWithoutTheQuickStopRowIsRefusedAtStart) {
    auto profile = plem::fieldbus::kStandardCia402Profile;
    const auto end = std::remove_if(profile.transitions.begin(), profile.transitions.begin() + profile.transition_count,
                                    [](const plem::fieldbus::Cia402Transition& t) {
                                        return t.from == plem::fieldbus::ServoState::OPERATION_ENABLED &&
                                               t.to == plem::fieldbus::ServoState::QUICK_STOP_ACTIVE;
                                    });
    ASSERT_NE(end, profile.transitions.begin() + profile.transition_count) << "the standard table declares the row";
    profile.transition_count = static_cast<size_t>(end - profile.transitions.begin());
    axis1_profile_ = profile;
    build_stop_bus();
    const auto started = controller_->start();
    ASSERT_FALSE(started);
    EXPECT_NE(started.message().find("axes[1]"), std::string::npos) << started.message();
    EXPECT_NE(started.message().find("transition 11"), std::string::npos) << started.message();
    EXPECT_FALSE(controller_->is_running());
}

TEST_F(DriveStopLoopTest, AStopOnABusWithACstAxisEngagesAtOnce) {
    // Axis 0 cst: a torque axis is not ramped by the loop, and the gate is one
    // for the whole bus; the drives' own disable reaction (0x605C) stops it.
    build(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
    const std::vector<double> decelerations{0.0, kDecel};
    controller_->set_drive_stop(decelerations);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    stream(1.0, rest_rad() + 0.0005);
    start();
    ASSERT_TRUE(wait_for([&] { return position_word(1) == position_word_for(rest_rad() + 0.0005); }));
    controller_->request_ramp_stop();
    wait_cycles(3);  // at once: the request's cycle, then the frame that carries the word
    EXPECT_FALSE(flags_->motion_enabled.load());
    EXPECT_EQ(controlword(1) & 0x8F, 0x07);
}

TEST_F(DriveStopLoopTest, AReleaseHoldsThePositionItFoundUntilTheHostCommands) {
    build_stop_bus();
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    controller_->open_command_epoch();
    start();
    ASSERT_TRUE(wait_for([&] { return flags_->all_drives_enabled.load(); }));
    wait_cycles(5);
    EXPECT_EQ(position_word(1), kRestWord);

    set_actual_position(1, kRestWord - 400);  // the load sags
    wait_cycles(10);
    EXPECT_EQ(position_word(1), kRestWord) << "the position held at the release, not the sagging actual";
    EXPECT_FALSE(flags_->is_protective_stop_active()) << "no command yet: nothing to go stale";

    stream(0.0, rest_rad() + 0.0003);
    EXPECT_TRUE(wait_for([&] { return position_word(1) == position_word_for(rest_rad() + 0.0003); }));
    EXPECT_FALSE(flags_->is_protective_stop_active());
}

TEST_F(DriveStopLoopTest, ACommandWrittenBeforeTheReleaseIsNotApplied) {
    build_stop_bus();
    write_command(0.0, rest_rad() + 0.0003);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    controller_->open_command_epoch();
    start();
    ASSERT_TRUE(wait_for([&] { return flags_->all_drives_enabled.load(); }));
    wait_cycles(10);
    EXPECT_EQ(position_word(1), kRestWord) << "an earlier life's command is not this release's";
    write_command(0.0, rest_rad() + 0.0003);
    EXPECT_TRUE(wait_for([&] { return position_word(1) == position_word_for(rest_rad() + 0.0003); }));
}

TEST_F(DriveStopLoopTest, TheStreamWatchArmsOnTheFirstCommandOfAReleaseAndRestsWhileMotionIsOff) {
    build_stop_bus();
    stream(0.0, rest_rad());
    start();
    wait_cycles(5);
    stop_streaming();  // motion off: a silent host is no hazard
    wait_cycles(150);
    EXPECT_FALSE(flags_->is_protective_stop_active());

    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    controller_->open_command_epoch();
    wait_cycles(5);
    stream(0.0, rest_rad());
    wait_cycles(5);
    stop_streaming();
    EXPECT_TRUE(wait_for([&] { return flags_->is_external_protective_stop(); }));
}

// A host on its own clock (a controller manager beside the bus loop): its
// samples come every third cycle, each with the planned speed. Held, a sample
// would make steps of 0, 0 then 3·v·T, over the CSP step check at
// v > limit/3. The loop advances a csp sample by its rate over its age: each
// frame's target is the newest sample + rate × (that cycle's wake − the
// sample's stamp). The test loop is not a real-time thread: under load a wake
// can come late by more than the CSP step allows at 1 rad/s, so the limit is
// raised to keep that protective stop out of a test about the advance.
TEST_F(DriveStopLoopTest, ASampleThatArrivesLateIsAdvancedByItsRate) {
    auto limit = wide_limit();
    limit.velocity_limit = 100.0;
    build_stop_bus(plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION, limit);
    flags_->motion_enabled.store(true);
    set_status(kOperationEnabled);
    controller_->open_command_epoch();
    const double speed = 0.4;  // rad/s
    const double from = rest_rad();
    struct Script {
        std::mutex mutex;
        int64_t t0 = 0;
        double value = 0.0;  // the sample the loop reads next cycle
        int64_t stamp = 0;
        double read_value = 0.0;  // the sample this cycle's frame came from
        int64_t read_stamp = 0;
        std::vector<double> errors;  // frame target − (sample + rate × age), rad
    };
    const auto script = std::make_shared<Script>();
    per_cycle([this, script, from, speed](int cycle) {
        std::lock_guard lock(script->mutex);
        if (script->read_stamp != 0) {
            const double age = static_cast<double>(shared_->last_cycle_ns.load() - script->read_stamp) * 1e-9;
            script->errors.push_back(target_rad(position_word(1)) - (script->read_value + speed * age));
        }
        script->read_value = script->value;
        script->read_stamp = script->stamp;
        if (cycle % 3 != 0) {
            return;  // the host skips this cycle
        }
        const int64_t now = plem::core::now_ns();
        script->t0 = script->t0 != 0 ? script->t0 : now;
        script->value = from + speed * static_cast<double>(now - script->t0) * 1e-9;
        script->stamp = now;
        write_command(0.0, script->value, speed);
    });
    start();
    wait_cycles(200);
    std::lock_guard lock(script->mutex);
    ASSERT_GT(script->errors.size(), 100u);
    const double count = std::abs(target_rad(kRestWord + 1) - target_rad(kRestWord));
    for (size_t k = 0; k < script->errors.size(); ++k) {
        EXPECT_LE(std::abs(script->errors[k]), 2 * count) << "frame " << k;
    }
    EXPECT_FALSE(flags_->is_protective_stop_active()) << "a stop, not the advance, moved the target";
}
