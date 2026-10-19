// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_host_lifecycle.cpp
/// @brief DriveHost's own methods (create → start → enable → write → read →
/// stop, and the destructor) on the shared plem_fieldbus library with the
/// fake master linked into this executable; the executable's ecrt_*
/// definitions take precedence over libethercat's (SetUp checks, before any
/// bus contact, that the process's global symbol scope resolves ecrt_* to the
/// fake, the scope libplem_fieldbus's references bind in). DriveHostLifecycle's
/// drive never answers a controlword, so those pins stop at what the host
/// decides; DriveHostWithAnsweringDrive puts the shared answering drive
/// (Cia402DriveSim) behind the fake master and walks one csp axis through
/// release → move → stale stop → reset → move → ramp stop → stop, and
/// through a working-counter loss. The
/// cycle-level behaviour is pinned on plem_fieldbus's fake-master targets.

#include <gtest/gtest.h>

#include <dlfcn.h>
#include <ecrt.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <thread>

#include <plem/core/monitoring/event.hpp>
#include <plem/core/timing/timespec.hpp>
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/fieldbus/ethercat/bus.hpp>
#include <plem/fieldbus/test/cia402_drive_sim.hpp>
#include <plem/fieldbus/test/fake_ecrt.hpp>

#include "drive_model_spec_fixture.hpp"
#include "plem/drive/drive_host.hpp"

namespace plem::drive {
namespace {

using fieldbus::test::FakeEcrt;

// Five motor turns past the axis's zero, behind the fixture's 100:1 gear:
// a position the zero-initialized snapshot never holds.
constexpr int32_t kSeededTurns = 5;
constexpr double kSeededRad = kSeededTurns * 2.0 * std::numbers::pi / 100.0;

class DriveHostLifecycle : public ::testing::Test {
  protected:
    void SetUp() override {
        FakeEcrt::reset();
        // Interposition: the global scope's ecrt_* is the fake this executable defines.
        ASSERT_EQ(dlsym(RTLD_DEFAULT, "ecrt_request_master"), reinterpret_cast<void*>(&ecrt_request_master))
            << "libethercat's ecrt_request_master would answer, not the fake";
    }

    void TearDown() override {
        stop_streaming();
    }

    static std::unique_ptr<DriveHost<1>> make_host() {
        auto created =
            DriveHost<1>::create(test::make_n_drives<1>(), std::make_shared<fieldbus::ethercat::EtherCATBus>(0),
                                 DriveHostParams{.rt_priority = 1});
        EXPECT_TRUE(created) << created.message();
        return created ? std::move(created).value() : nullptr;
    }

    /// The drive's frame, written into the fake domain before start() (no
    /// loop runs yet, so no race): Switch on disabled (0x6041; a drive
    /// reports its actuals only once it says anything) and the seeded 0x6064.
    static void seed_position() {
        const int status = FakeEcrt::domain_offset_of(0, 0x6041, 0);
        const int position = FakeEcrt::domain_offset_of(0, 0x6064, 0);
        ASSERT_GE(status, 0);
        ASSERT_GE(position, 0);
        EC_WRITE_U16(ecrt_domain_data(nullptr) + status, 0x0250);
        EC_WRITE_S32(ecrt_domain_data(nullptr) + position, test::kAxisZeroOffsets[0] + kSeededTurns * int32_t{65536});
    }

    template <typename Pred>
    static bool wait_for(Pred pred) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            if (pred()) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return pred();
    }

    /// True once the loop has published the seeded position.
    static bool loop_published(DriveHost<1>& host) {
        DriveHost<1>::State state;
        return host.read(state) && std::abs(state.actual.states[0].position_actual_value - kSeededRad) < 1e-6;
    }

    static double last_position(DriveHost<1>& host) {
        DriveHost<1>::State state;
        return host.read(state) ? state.actual.states[0].position_actual_value : -999.0;
    }

    /// A host writer: a fresh command every 5 ms until stop_streaming().
    void stream(DriveHost<1>& host) {
        streaming_.store(true);
        streamer_ = std::thread([this, &host] {
            const std::array<double, 1> cmd{0.0};
            while (streaming_.load()) {
                (void)host.write(cmd, {}, core::now_ns());
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    }
    void stop_streaming() {
        streaming_.store(false);
        if (streamer_.joinable()) {
            streamer_.join();
        }
    }

    std::atomic<bool> streaming_{false};
    std::thread streamer_;
};

TEST_F(DriveHostLifecycle, CreateBringsTheBusUpOnTheLinkedFakeMaster) {
    auto host = make_host();
    ASSERT_NE(host, nullptr);
    const auto& calls = FakeEcrt::calls();
    const auto has = [&](const std::string& prefix) {
        return std::any_of(calls.begin(), calls.end(), [&](const std::string& c) { return c.rfind(prefix, 0) == 0; });
    };
    EXPECT_TRUE(has("request_master"));
    EXPECT_TRUE(has("master_activate"));
}

TEST_F(DriveHostLifecycle, WriteRefusesAMalformedCommand) {
    auto host = make_host();
    ASSERT_NE(host, nullptr);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const int64_t now = core::now_ns();
    const std::array<double, 1> ok{0.5};
    const std::array<double, 2> two{0.5, 0.5};
    const std::array<double, 1> bad{nan};
    EXPECT_FALSE(host->write(two, {}, now)) << "one value per axis";
    EXPECT_FALSE(host->write(ok, two, now)) << "rate is empty or one per axis";
    EXPECT_FALSE(host->write(bad, {}, now));
    EXPECT_FALSE(host->write(ok, bad, now));
    EXPECT_TRUE(host->write(ok, {}, now));
}

// The stale stop and enable()'s freshness compute now − stamp on
// CLOCK_MONOTONIC; a stamp ahead of now or not positive would hold them off.
TEST_F(DriveHostLifecycle, WriteRefusesAFutureOrNonPositiveStamp) {
    auto host = make_host();
    ASSERT_NE(host, nullptr);
    const std::array<double, 1> cmd{0.5};
    EXPECT_FALSE(host->write(cmd, {}, 0));
    EXPECT_FALSE(host->write(cmd, {}, -1));
    EXPECT_FALSE(host->write(cmd, {}, core::now_ns() + 10'000'000'000)) << "10 s ahead";
    timespec rt{};
    ::clock_gettime(CLOCK_REALTIME, &rt);
    EXPECT_FALSE(host->write(cmd, {}, int64_t{rt.tv_sec} * 1'000'000'000 + rt.tv_nsec)) << "epoch time";
    EXPECT_TRUE(host->write(cmd, {}, core::now_ns()));
}

TEST_F(DriveHostLifecycle, EnableNeedsAStartedHostAndALiveTorqueStream) {
    auto host = make_host();
    ASSERT_NE(host, nullptr);
    ASSERT_NO_FATAL_FAILURE(seed_position());
    auto refused = host->enable();
    ASSERT_FALSE(refused);
    EXPECT_NE(refused.message().find("needs a started"), std::string::npos) << refused.message();

    ASSERT_TRUE(host->start());
    ASSERT_TRUE(wait_for([&] { return loop_published(*host); }))
        << "the loop publishes the drive's actuals; last read " << last_position(*host) << " want " << kSeededRad;

    refused = host->enable();
    ASSERT_FALSE(refused);
    EXPECT_NE(refused.message().find("no fresh command"), std::string::npos) << refused.message();

    stream(*host);
    const bool released = wait_for([&] { return static_cast<bool>(host->enable()); });
    stop_streaming();  // before any assertion can return and destroy the host the streamer writes to
    ASSERT_TRUE(released) << "a live stream opens the release";

    const auto outcome = host->stop();
    EXPECT_FALSE(outcome.never_released) << "this run released";
    EXPECT_TRUE(outcome.fully_engaged()) << "the drive stayed in Switch on disabled, which proves its brake closed";
}

TEST_F(DriveHostLifecycle, AStopWithNothingReleasedSaysSoAndIsOneShot) {
    auto host = make_host();
    ASSERT_NE(host, nullptr);
    ASSERT_NO_FATAL_FAILURE(seed_position());
    ASSERT_TRUE(host->start());
    EXPECT_TRUE(host->start()) << "a second start is a no-op";
    ASSERT_TRUE(wait_for([&] { return loop_published(*host); }));
    const auto outcome = host->stop();
    EXPECT_TRUE(outcome.never_released);
    const auto again = host->start();
    ASSERT_FALSE(again);
    EXPECT_NE(again.message().find("one-shot lifecycle"), std::string::npos) << again.message();
}

TEST_F(DriveHostLifecycle, DestroyingARunningHostStopsTheLoop) {
    auto host = make_host();
    ASSERT_NE(host, nullptr);
    ASSERT_NO_FATAL_FAILURE(seed_position());
    ASSERT_TRUE(host->start());
    ASSERT_TRUE(wait_for([&] { return loop_published(*host); })) << "a confirmed cycle before the destructor";
    host.reset();  // joins the RT thread; a leaked thread would terminate the binary
    SUCCEED();
}

/// One csp axis behind the shared answering drive: each frame the drive
/// answers the controlword just sent, displays the commanded mode and, while
/// operation is enabled, follows the target (0x6064 = 0x607A), an ideal csp
/// drive. All on the loop thread (the fake's send observer).
class DriveHostWithAnsweringDrive : public DriveHostLifecycle {
  protected:
    using Sim = fieldbus::test::Cia402DriveSim;
    static constexpr uint16_t kRest = 0x0250;  // Switch on disabled
    static constexpr double kSpeedRadS = 0.2;

    void SetUp() override {
        DriveHostLifecycle::SetUp();
        auto spec = test::make_n_drives<1>();
        test::set_mode(spec, 0, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
        auto created = DriveHost<1>::create(spec, std::make_shared<fieldbus::ethercat::EtherCATBus>(0),
                                            DriveHostParams{.rt_priority = 1});
        ASSERT_TRUE(created) << created.message();
        host_ = std::move(created).value();

        uint8_t* pd = ecrt_domain_data(nullptr);
        const auto offset = [](unsigned int index) { return FakeEcrt::domain_offset_of(0, index, 0); };
        const int controlword = offset(0x6040);
        const int mode = offset(0x6060);
        const int target = offset(0x607A);
        const int status = offset(0x6041);
        const int mode_display = offset(0x6061);
        const int actual = offset(0x6064);
        ASSERT_GE(std::min({controlword, mode, target, status, mode_display, actual}), 0);
        EC_WRITE_U16(pd + status, kRest);
        EC_WRITE_S32(pd + actual, test::kAxisZeroOffsets[0]);  // 0 rad
        auto drive = std::make_shared<fieldbus::test::Cia402DriveSim>(kRest);
        FakeEcrt::set_send_observer([=] {
            drive->consume(EC_READ_U16(pd + controlword));
            EC_WRITE_U16(pd + status, drive->status());
            EC_WRITE_S8(pd + mode_display, EC_READ_S8(pd + mode));
            if (drive->status() == Sim::kOperationEnabled) {
                EC_WRITE_S32(pd + actual, EC_READ_S32(pd + target));
            }
        });
    }

    void TearDown() override {
        stop_moving();
        host_.reset();  // joins the loop before the observer's domain goes away
        FakeEcrt::set_send_observer(nullptr);
        DriveHostLifecycle::TearDown();
    }

    DriveHost<1>::State state() {
        DriveHost<1>::State out;
        EXPECT_TRUE(wait_for([&] { return host_->read(out); }));
        return out;
    }

    /// Released and settled: the drive reports Operation enabled.
    bool released() {
        const auto s = state();
        return s.operation_enabled && s.brake_engaged == 0 && s.brake_operation_pending == 0 &&
               s.statusword[0] == Sim::kOperationEnabled;
    }
    std::string describe() {
        const auto s = state();
        char buf[160];
        std::snprintf(buf, sizeof buf, "oe=%d engaged=0x%x pending=0x%x sw=0x%04x pstop=0x%x stopping=%d",
                      s.operation_enabled, s.brake_engaged, s.brake_operation_pending, s.statusword[0],
                      s.protective_stop, s.stopping);
        return buf;
    }
    /// Engaged and settled. The engage walks the drive down to Ready to switch
    /// on and settles there; it does not command Disable voltage.
    bool engaged() {
        const auto s = state();
        return !s.operation_enabled && s.brake_engaged == 1 && s.brake_operation_pending == 0 &&
               s.statusword[0] == Sim::kReadyToSwitchOn;
    }

    /// A host writer moving the axis at `speed` from where it stands (0 =
    /// holding it there), a fresh stamp every 2 ms, the first one before this
    /// returns, until stop_moving().
    void move(double speed = kSpeedRadS) {
        const double from = state().actual.states[0].position_actual_value;
        const int64_t t0 = core::now_ns();
        const auto write = [this, from, t0, speed] {
            const int64_t now = core::now_ns();
            const std::array<double, 1> cmd{from + speed * static_cast<double>(now - t0) * 1e-9};
            const std::array<double, 1> rate{speed};
            if (host_->write(cmd, rate, now)) {
                last_write_ns_.store(now);
            }
        };
        write();
        moving_.store(true);
        mover_ = std::thread([this, write] {
            while (moving_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                write();
            }
        });
    }
    void stop_moving() {
        moving_.store(false);
        if (mover_.joinable()) {
            mover_.join();
        }
    }

    double position() {
        return state().actual.states[0].position_actual_value;
    }

    /// Drains the host's event queue, counting `type`.
    size_t drain(core::monitoring::EventType type) {
        size_t n = 0;
        auto* queue = host_->monitoring_context()->queue<core::monitoring::Event>();
        while (auto* event = queue->front()) {
            n += event->type == type ? 1 : 0;
            queue->pop();
        }
        return n;
    }

    std::unique_ptr<DriveHost<1>> host_;
    std::atomic<bool> moving_{false};
    std::atomic<int64_t> last_write_ns_{0};
    std::thread mover_;
};

TEST_F(DriveHostWithAnsweringDrive, ReleaseMoveStaleStopResetMoveRampStopStop) {
    ASSERT_TRUE(host_->start());
    ASSERT_TRUE(wait_for([&] { return std::abs(position()) < 1e-9 && state().statusword[0] == kRest; }))
        << "the loop publishes the resting drive";

    // Release: a bus without a cst axis needs no stream before enable().
    ASSERT_TRUE(host_->enable());
    ASSERT_TRUE(wait_for([&] { return released(); })) << "the drive walks to Operation enabled: " << describe();

    // Move: the drive follows the host's csp stream.
    move();
    EXPECT_TRUE(wait_for([&] { return position() > 0.05; })) << "moved to " << position();

    // Stale: the host goes quiet; past the 50 ms budget the loop latches the
    // EXTERNAL protective stop and engages the brake.
    stop_moving();
    const int64_t quiet_from = last_write_ns_.load();
    constexpr auto kExternal = static_cast<uint8_t>(core::ProtectiveStopSource::EXTERNAL);
    ASSERT_TRUE(wait_for([&] { return (state().protective_stop & kExternal) != 0; }));
    const int64_t latched = core::now_ns();
    EXPECT_GE(latched - quiet_from, kCommandStaleTimeoutNs) << "not before the freshness budget";
    EXPECT_TRUE(wait_for([&] { return engaged(); })) << "the stale stop engages the brake: " << describe();
    EXPECT_GE(drain(core::monitoring::EventType::COMMAND_STALE), 1u);
    const double held = position();
    EXPECT_FALSE(host_->enable()) << "a latched protective stop refuses motion";

    // A reset sticks only under a live stream (a stale one re-latches the
    // stop), so the host holds the axis first; then release and move on from
    // where the axis stood.
    move(0.0);
    host_->reset();
    ASSERT_TRUE(wait_for([&] { return state().protective_stop == 0; })) << describe();
    ASSERT_TRUE(host_->enable());
    ASSERT_TRUE(wait_for([&] { return released(); })) << describe();
    EXPECT_NEAR(position(), held, 1e-3) << "no jump at the release (the host holds the axis there too)";
    stop_moving();
    move();
    EXPECT_TRUE(wait_for([&] { return position() > held + 0.02; })) << "moved to " << position() << ", " << describe();

    // Ramp stop with the host gone quiet: the axis travels on about
    // v²/2a = 0.005 rad (a cut would stop it within a cycle or two), then the
    // brake engages; not a protective stop.
    const double at_request = position();
    host_->ramp_stop();
    stop_moving();
    EXPECT_TRUE(wait_for([&] { return engaged() && !state().stopping; })) << "the stop ends engaged: " << describe();
    EXPECT_EQ(state().protective_stop, 0);
    const double travelled = position() - at_request;
    EXPECT_GT(travelled, 0.003) << "decelerated, not cut";
    EXPECT_LT(travelled, 0.02);

    const auto outcome = host_->stop();
    EXPECT_FALSE(outcome.never_released);
    EXPECT_TRUE(outcome.fully_engaged());
    EXPECT_EQ(outcome.statuswords[0], Sim::kReadyToSwitchOn);
}

// The working counter goes incomplete under a live stream: the loop latches
// the EXTERNAL protective stop and engages the brake; with the counter whole
// again, a reset clears it.
TEST_F(DriveHostWithAnsweringDrive, AnIncompleteWorkingCounterStopsTheBusAndEngages) {
    ASSERT_TRUE(host_->start());
    ASSERT_TRUE(wait_for([&] { return state().statusword[0] == kRest; }));
    ASSERT_TRUE(host_->enable());
    ASSERT_TRUE(wait_for([&] { return released(); })) << describe();
    move(0.0);
    ASSERT_TRUE(wait_for([&] { return state().process_data_complete; }));

    FakeEcrt::set_domain_wc_state(EC_WC_INCOMPLETE);
    constexpr auto kExternal = static_cast<uint8_t>(core::ProtectiveStopSource::EXTERNAL);
    ASSERT_TRUE(wait_for([&] { return (state().protective_stop & kExternal) != 0; })) << describe();
    EXPECT_FALSE(state().process_data_complete);
    EXPECT_TRUE(wait_for([&] { return engaged(); })) << "the stop engages the brake: " << describe();

    FakeEcrt::set_domain_wc_state(EC_WC_COMPLETE);
    ASSERT_TRUE(wait_for([&] { return state().process_data_complete; }));
    host_->reset();
    EXPECT_TRUE(wait_for([&] { return state().protective_stop == 0; })) << describe();
}

}  // namespace
}  // namespace plem::drive
