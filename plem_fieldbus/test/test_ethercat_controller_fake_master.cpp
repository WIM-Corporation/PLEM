// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_ethercat_controller_fake_master.cpp
/// @brief RT-loop brake wiring characterization on the fake master.
///
/// Compiles the real EtherCATController run_loop against the fake_ecrt link
/// seam, so the cycle-level brake contract is pinned at device level (ordered
/// PLEM_EMIT trace). These are wiring pins; the exhaustive
/// decision truth table lives in test_brake_logic.cpp; the shell ordering
/// (two flag samples, exchange order, apply-before-exchange) lives in
/// test_brake_cycle_step.cpp.
///
/// Determinism notes (fake-master environment):
/// - The fixture seeds every servo with a recognized NOT_READY statusword and
///   a CST mode display (the fake domain is zeroed at reset and never answers
///   a controlword or echoes 0x6060), so every axis is initialized, no drive
///   ever reaches OPERATION_ENABLED and EtherCATBus::motion_possible() is
///   false every cycle, so the P-STOP engage-pending flag deterministically
///   clears via the !motion_possible path one cycle after the rising edge. A
///   release sequence issues its first command and holds there; an engage
///   settles early (NOT_READY is a proven-closed step), so the brake mask
///   stays all-engaged for the loop's whole lifetime.
/// - The loop runs at the config period on a normal-priority thread; pins are
///   steady-state ("eventually exactly N events"), never cycle-indexed.
/// - The test thread only touches FakeEcrt outside the loop's lifetime; the
///   Event queue is SPSC (RT producer / test consumer) so draining while the
///   loop runs is the intended contract.

#include <gtest/gtest.h>

#include "plem/fieldbus/cia402_profile.hpp"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <plem/core/monitoring/event.hpp>
#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/monitoring/rt_sample.hpp>
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/core/types/shared_joint_data.hpp>
#include <plem/core/units.hpp>

#include <plem/core/test/seqlock_data_test_peer.hpp>
#include "fake_master_fixture.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/controller.hpp"
#include "plem/fieldbus/test/ethercat_test_probes.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

using plem::core::ControlModeFlags;
using plem::core::ProtectiveStopSource;
using plem::core::TriState;
using plem::core::monitoring::EventType;
using plem::core::test::SeqlockDataTestPeer;
using plem::core::units::Pulse;
using plem::fieldbus::ethercat::DeviceServo;
using plem::fieldbus::ethercat::EtherCATBus;
using plem::fieldbus::ethercat::EtherCATController;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::test::EtherCATControllerTestProbe;
using plem::fieldbus::test::FakeEcrt;

namespace {

using plem::fieldbus::test::one_io_info;
using plem::fieldbus::test::pdo_offset;
using plem::fieldbus::test::six_servos;

/// The fixture's IO slave: one registered entry, counting the RT loop's calls.
struct OneEntryIoDevice : plem::fieldbus::ethercat::DeviceBase {
    explicit OneEntryIoDevice(const plem::fieldbus::ethercat::IODeviceContext& context)
        : DeviceBase(context.master, context.slave_position, context.io_info.vendor_id, context.io_info.product_code) {}
    plem::core::Result<void> register_PDOs(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index) override {
        return register_PDO_entries(domain_regs, domain_index, {{0x7000, 0x01, &offset_}});
    }
    void read(const uint8_t* /*domain_pd*/) noexcept override {
        reads.fetch_add(1, std::memory_order_relaxed);
    }
    void write(uint8_t* /*domain_pd*/) noexcept override {
        writes.fetch_add(1, std::memory_order_relaxed);
    }
    uint32_t offset_{0};
    std::atomic<uint32_t> reads{0};
    std::atomic<uint32_t> writes{0};
};

std::shared_ptr<plem::fieldbus::ethercat::DeviceBase> make_one_entry_device(
    const plem::fieldbus::ethercat::IODeviceContext& context) {
    return std::make_shared<OneEntryIoDevice>(context);
}

}  // namespace

class ControllerLoopFakeMasterTest : public ::testing::Test {
  protected:
    /// Hook for a derived fixture to set fake-master knobs the bus reads at initialize().
    virtual void before_bus_init() {}

    void SetUp() override {
        FakeEcrt::reset();
        before_bus_init();
        ASSERT_TRUE(ctx_.initialize().is_ok());  // queue<T>() returns nullptr on an uninitialized context

        bus_ = std::make_shared<EtherCATBus>(0);
        bus_->set_servo_config(six_servos());
        bus_->set_io_config({one_io_info()});
        bus_->set_io_device_factory(make_one_entry_device);
        bus_->set_cycle_time_ns(1'000'000);
        const auto init = bus_->initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
        // Cold state of the fake drives (see the determinism notes): a
        // recognized NOT_READY statusword and the declared mode on the 0x6061
        // display, per axis. The loop only writes RxPDO fields, so the TxPDO
        // region is race-free.
        uint8_t* pd = ecrt_domain_data(nullptr);
        for (uint32_t j = 0; j < 6; ++j) {
            EC_WRITE_U16(pd + pdo_offset(j, 0x6041), 0x0020);  // NOT_READY_TO_SWITCH_ON
            EC_WRITE_U8(pd + pdo_offset(j, 0x6061), 0x0A);     // modes-of-operation display = CST
        }

        flags_ = std::make_shared<ControlModeFlags>();
        // Motion committed for the whole characterization suite
        // (release consumption requires motion_enabled; these traces model a
        // robot in a motion mode; the discard behavior without it is pinned
        // in the synchronous golden suite).
        flags_->motion_enabled.store(true, std::memory_order_release);
        shared_ = std::make_shared<plem::core::SharedJointData<6>>();

        auto config = plem::fieldbus::test::test_controller_config<6>();
        controller_ = std::make_unique<EtherCATController<6>>(config);
        controller_->set_bus(bus_);
        controller_->set_mode_flags(flags_);
        controller_->set_shared_data(shared_);
        controller_->set_monitoring_context(&ctx_);
    }

    void TearDown() override {
        if (controller_) {
            controller_->stop();
        }
    }

    void start_loop() {
        auto result = controller_->start();
        ASSERT_TRUE(result) << result.message();
    }

    /// Drain the Event queue, appending brake-family events to the ordered
    /// trace (other event types, such as WKC and RT diagnostics, are ignored).
    void drain_events() {
        auto* queue = ctx_.queue<plem::core::monitoring::Event>();
        while (auto* event = queue->front()) {
            switch (event->type) {
                case EventType::BRAKE_RELEASE:
                case EventType::BRAKE_ENGAGE:
                case EventType::PROTECTIVE_STOP:
                case EventType::PROTECTIVE_STOP_CLEAR:
                    trace_.push_back(event->type);
                    break;
                default:
                    break;
            }
            queue->pop();
        }
    }

    int count(EventType type) {
        int n = 0;
        for (auto t : trace_) {
            n += (t == type) ? 1 : 0;
        }
        return n;
    }

    /// The shared bounded poll, draining the event queue before each look.
    template <typename Pred>
    bool wait_for(Pred&& pred, std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
        return plem::fieldbus::test::wait_for(
            [&] {
                drain_events();
                return pred();
            },
            timeout);
    }

    /// Negative-assertion settle: give the 1 kHz loop a bounded window in
    /// which the asserted-absent behavior would have shown up (intentional
    /// bounded sleep: there is no positive edge to wait for).
    /// The loop's cycle stamp must move across the window, or the absence
    /// proves nothing.
    void settle(std::chrono::milliseconds window = std::chrono::milliseconds(100)) {
        const int64_t before = shared_->last_cycle_ns.load(std::memory_order_relaxed);
        std::this_thread::sleep_for(window);
        drain_events();
        EXPECT_GT(shared_->last_cycle_ns.load(std::memory_order_relaxed), before)
            << "the loop did not cycle during the settle window";
    }

    plem::core::monitoring::MonitoringContext ctx_;
    std::shared_ptr<EtherCATBus> bus_;
    std::shared_ptr<ControlModeFlags> flags_;
    std::shared_ptr<plem::core::SharedJointData<6>> shared_;
    std::unique_ptr<EtherCATController<6>> controller_;
    std::vector<EventType> trace_;
};

// Wiring-setter guards
// Every setter below binds a dependency the RT loop dereferences per cycle,
// so each one refuses while the loop runs (a higher-layer host's control
// thread does the same for its wiring setters).
TEST_F(ControllerLoopFakeMasterTest, WiringSettersRefuseWhileTheLoopRuns) {
    start_loop();
    ASSERT_TRUE(wait_for([&] { return controller_->is_running(); }));

    EXPECT_THROW(controller_->set_shared_data(shared_), std::logic_error);
    EXPECT_THROW(controller_->set_monitoring_context(&ctx_), std::logic_error);
    EXPECT_THROW(controller_->set_bus(bus_), std::logic_error);
    EXPECT_THROW(controller_->set_mode_flags(flags_), std::logic_error);
    EXPECT_THROW(controller_->set_committed_mode_probe(nullptr, nullptr), std::logic_error);
    EXPECT_THROW(controller_->set_brake_setup({0x01, 0x02, 0x04, 0x08, 0x10, 0x20}), std::logic_error);
    EXPECT_THROW(controller_->set_fault_reset_policy(plem::fieldbus::ethercat::FaultResetPolicy::NONE, std::nullopt),
                 std::logic_error);
}

// The guard is about the running loop, not about having run once: a stopped
// controller is back at wiring time and must rewire without complaint.
TEST_F(ControllerLoopFakeMasterTest, WiringSettersAcceptAfterTheLoopStops) {
    start_loop();
    ASSERT_TRUE(wait_for([&] { return controller_->is_running(); }));
    controller_->stop();

    EXPECT_NO_THROW(controller_->set_shared_data(shared_));
    EXPECT_NO_THROW(controller_->set_monitoring_context(&ctx_));
    EXPECT_NO_THROW(controller_->set_bus(bus_));
    EXPECT_NO_THROW(controller_->set_mode_flags(flags_));
    EXPECT_NO_THROW(controller_->set_committed_mode_probe(nullptr, nullptr));
    EXPECT_NO_THROW(controller_->set_brake_setup({0x01, 0x02, 0x04, 0x08, 0x10, 0x20}));
    EXPECT_NO_THROW(
        controller_->set_fault_reset_policy(plem::fieldbus::ethercat::FaultResetPolicy::NONE, std::nullopt));

    start_loop();  // and start() accepts the rewired controller again
    EXPECT_TRUE(wait_for([&] { return controller_->is_running(); }));
}

// Lifecycle
TEST_F(ControllerLoopFakeMasterTest, DoubleStartIsIdempotent) {
    ASSERT_TRUE(controller_->start());
    EXPECT_TRUE(controller_->is_running());

    // Second start should be a no-op (returns ok)
    EXPECT_TRUE(controller_->start());
    EXPECT_TRUE(controller_->is_running());

    controller_->stop();
    EXPECT_FALSE(controller_->is_running());
}

TEST_F(ControllerLoopFakeMasterTest, DoubleStopIsIdempotent) {
    ASSERT_TRUE(controller_->start());
    controller_->stop();
    EXPECT_FALSE(controller_->is_running());

    // Second stop should be a no-op
    EXPECT_NO_THROW(controller_->stop());
    EXPECT_FALSE(controller_->is_running());
}

// Startup is quiescent
// No request and no stop: a healthy loop emits zero brake-family events no
// matter how many cycles run (also pins that the step does not re-emit per
// cycle). The boot engage is the assembly's post, pinned next.
TEST_F(ControllerLoopFakeMasterTest, QuiescentStartupEmitsNoBrakeEvents) {
    start_loop();
    settle();
    EXPECT_TRUE(trace_.empty()) << "brake-family events on a quiescent loop";
}

// The boot engage
// A host posts one all-joints engage before start(). The loop
// consumes it on its first cycle (exactly one BRAKE_ENGAGE) and stays quiet
// after: a request posted before the loop runs survives to the first cycle.
TEST_F(ControllerLoopFakeMasterTest, EngagePostedBeforeStartIsOneBrakeEngageThenQuiet) {
    controller_->request_brake_engage();
    start_loop();
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_ENGAGE) >= 1; }));
    settle();
    const std::vector<EventType> expected = {EventType::BRAKE_ENGAGE};
    EXPECT_EQ(trace_, expected);
}

// Restart revokes a stale drive-enablement grant
// A higher-layer host's control-thread lifecycle is independent of this
// loop's: after stop()/start() it may already be running against a `true`
// left over from the previous life while this loop still waits out its first
// timer period. In this harness no drive ever reaches OPERATION_ENABLED, so
// nothing can legitimately re-grant it and any surviving `true` is the stale one.
// Scope of this pin: a polling test observes only "revoked once the loop
// runs"; the per-cycle publication alone satisfies that. The dedicated
// entry/exit revokes in run_loop() close the sub-cycle window (controller's
// first cycle runs before the loop's first timer period elapses) and the
// keeps-running-after-stop() case, neither of which a polling assertion can
// deterministically catch; their justification is the window analysis at
// the store sites, this test pins the observable restart contract.
TEST_F(ControllerLoopFakeMasterTest, RestartRevokesStaleDriveEnablementGrant) {
    start_loop();
    ASSERT_TRUE(wait_for([&] { return controller_->is_running(); }));
    controller_->stop();
    flags_->all_drives_enabled.store(true, std::memory_order_release);  // stale grant from the previous life
    start_loop();
    EXPECT_TRUE(wait_for([&] { return !flags_->all_drives_enabled.load(std::memory_order_acquire); }))
        << "a (re)started loop must revoke a stale drive-enablement grant";
}

// Requested release: once
TEST_F(ControllerLoopFakeMasterTest, RequestedReleaseStartsOnce) {
    start_loop();
    settle();
    ASSERT_TRUE(controller_->request_brake_release());
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_RELEASE) >= 1; }));
    settle();
    EXPECT_EQ(count(EventType::BRAKE_RELEASE), 1) << "one request, one release — not one per cycle";
    EXPECT_EQ(count(EventType::PROTECTIVE_STOP), 0);
}

// Requested engage: once
TEST_F(ControllerLoopFakeMasterTest, RequestedEngageStartsOnce) {
    start_loop();
    ASSERT_TRUE(controller_->request_brake_release());
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_RELEASE) >= 1; }));

    controller_->request_brake_engage();
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_ENGAGE) >= 1; }));
    settle();
    EXPECT_EQ(count(EventType::BRAKE_ENGAGE), 1);
}

// P-STOP rising edge: ordered [ENGAGE, PROTECTIVE_STOP, ENGAGE], each once
// Cycle N (rising edge): engage sequence start (BRAKE_ENGAGE from the
// sequence-start emit) then PROTECTIVE_STOP. Cycle N+1: the engage-pending
// flag clears via !motion_possible and emits the completion BRAKE_ENGAGE.
// The pending flag clears through the ENGAGE path, so PROTECTIVE_STOP_CLEAR
// must never appear, and the level (still latched) must not re-fire the edge.
TEST_F(ControllerLoopFakeMasterTest, ProtectiveStopRisingEdgeEngagesAllWithOrderedEventsOnce) {
    start_loop();
    settle();
    flags_->set_protective_stop(ProtectiveStopSource::EXTERNAL);
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_ENGAGE) >= 2; }));
    settle();

    const std::vector<EventType> expected = {EventType::BRAKE_ENGAGE, EventType::PROTECTIVE_STOP,
                                             EventType::BRAKE_ENGAGE};
    EXPECT_EQ(trace_, expected);
}

// Direct release is suppressed during P-STOP and not re-queued
// The queued mask is consumed unconditionally every cycle; a release that was
// suppressed by the protective stop is discarded; clearing the stop later
// must not release brakes from a stale request. (Load-bearing safety pin.)
TEST_F(ControllerLoopFakeMasterTest, DirectReleaseSuppressedDuringProtectiveStopIsConsumedNotRequeued) {
    start_loop();
    flags_->set_protective_stop(ProtectiveStopSource::EXTERNAL);
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_ENGAGE) >= 2; }));

    ASSERT_TRUE(controller_->request_brake_release());  // admission open, so queued (suppression is downstream)
    settle();
    EXPECT_EQ(count(EventType::BRAKE_RELEASE), 0) << "direct release must be suppressed while P-STOP is active";

    flags_->clear_all_protective_stop();
    settle();
    EXPECT_EQ(count(EventType::BRAKE_RELEASE), 0) << "suppressed release must NOT be re-queued after the stop clears";
    EXPECT_EQ(count(EventType::PROTECTIVE_STOP_CLEAR), 0) << "pending already cleared via engage — no CLEAR event";

    // A fresh request after the clear goes through.
    ASSERT_TRUE(controller_->request_brake_release());
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_RELEASE) >= 1; }));
    settle();
    EXPECT_EQ(count(EventType::BRAKE_RELEASE), 1);
}

// Freedrive recovery allows direct release under an active P-STOP
TEST_F(ControllerLoopFakeMasterTest, FreedriveRecoveryAllowsDirectReleaseDuringProtectiveStop) {
    start_loop();
    flags_->set_protective_stop(ProtectiveStopSource::EXTERNAL);
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_ENGAGE) >= 2; }));  // engage-pending cleared

    flags_->freedrive_enabled.store(true, std::memory_order_release);
    ASSERT_TRUE(controller_->request_brake_release());
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_RELEASE) >= 1; }));
    settle();
    EXPECT_EQ(count(EventType::BRAKE_RELEASE), 1);
}

// Live-loop shutdown quiesce

TEST_F(ControllerLoopFakeMasterTest, QuiesceBrakesFastPathOnEngagedStateAndClosesAdmission) {
    // Shutdown quiesce on the live loop, from the cold all-engaged state. Nothing to
    // engage: the outcome reports every group confirmed, and the admission
    // is closed as a side contract. (The released-state pipeline cannot be
    // reached on this harness: the fake never answers a controlword, so the
    // enable sequence cannot complete; the deep engage pipeline is pinned
    // cycle-exactly in the synchronous golden suite instead.)
    start_loop();

    const auto outcome = controller_->quiesce_brakes();
    EXPECT_TRUE(outcome.fully_engaged());
    EXPECT_EQ(outcome.timeout_mask, 0x00);
    EXPECT_FALSE(controller_->is_brake_release_admission_open()) << "quiesce closes admission";
    EXPECT_FALSE(controller_->request_brake_release()) << "releases refused after quiesce";
}

TEST_F(ControllerLoopFakeMasterTest, ClosedAdmissionRefusesReleaseAtPostTime) {
    EXPECT_TRUE(controller_->is_brake_release_admission_open());
    controller_->close_brake_release_admission();
    EXPECT_FALSE(controller_->is_brake_release_admission_open());
    EXPECT_FALSE(controller_->request_brake_release()) << "post-time refusal — nothing may queue after the close";
    EXPECT_FALSE(controller_->request_brake_release(0x01));

    start_loop();
    settle();
    EXPECT_EQ(count(EventType::BRAKE_RELEASE), 0);
}

TEST_F(ControllerLoopFakeMasterTest, ReleaseQueuedBeforeCloseIsDiscardedAtConsumption) {
    // The TOCTOU leg: the request is admitted (open) and queued, the
    // close lands before the RT loop ever consumes it; the consume-time
    // re-check must discard it (deterministic here: the loop starts after).
    ASSERT_TRUE(controller_->request_brake_release());  // admitted + queued
    controller_->close_brake_release_admission();

    start_loop();
    settle();
    EXPECT_EQ(count(EventType::BRAKE_RELEASE), 0) << "a pre-close queued release must NOT fire after the close";
}

// Direct engage is never gated
TEST_F(ControllerLoopFakeMasterTest, DirectEngageIsHonoredInNormalOperation) {
    start_loop();
    settle();
    controller_->request_brake_engage();
    ASSERT_TRUE(wait_for([&] { return count(EventType::BRAKE_ENGAGE) >= 1; }));
    settle();
    EXPECT_EQ(count(EventType::BRAKE_ENGAGE), 1);
    EXPECT_EQ(count(EventType::PROTECTIVE_STOP), 0);
}

namespace {

/// Direct-request mailbox probe (deterministic interleave pin).
/// No bus: posts touch atomics + the post mutex only.
class RequestMailboxProbe : public EtherCATControllerTestProbe<6> {
  public:
    using EtherCATControllerTestProbe<6>::EtherCATControllerTestProbe;
    void set_pair_hook(std::function<void()> hook) {
        set_brake_release_post_pair_hook_for_test(std::move(hook));
    }
};

}  // namespace

TEST_F(ControllerLoopFakeMasterTest, ReleasePostPairHoldsOutConcurrentEngagePost) {
    // The release post's OR + opposite-AND pair must be
    // atomic against a concurrent engage post for the same joint (an
    // interleave can clear both masks: the last request silently lost).
    //
    // Shape: the releaser parks mid-pair (production release-side hook,
    // inside the critical section) while a separate engager thread calls
    // request_brake_engage(); the test observes over a bounded window that
    // the engage call does not complete while the releaser is parked, then
    // unparks and verifies completion + last-effect-wins masks.
    //
    // Claim scope: the engager completing inside the parked window proves
    // the pair is not mutex-guarded, which turns a mutex-removal mutation
    // red. Non-completion is only consistent with the mutex: a scheduler
    // could also starve a mutex-less engager for the whole window.
    auto config = plem::fieldbus::test::test_controller_config<6>();
    RequestMailboxProbe probe(config);

    std::atomic<bool> in_pair{false};
    std::atomic<bool> unpark{false};
    probe.set_pair_hook([&] {
        in_pair.store(true, std::memory_order_release);
        // Parked mid-pair; bounded failsafe so a broken test cannot hang.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!unpark.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    std::atomic<bool> release_accepted{false};
    std::thread releaser([&] { release_accepted.store(probe.request_brake_release(0x01)); });
    // Bounded pair-entry wait: a hook-call regression must turn red,
    // never hang (the 10 s park failsafe only works once the hook ran).
    const auto entry_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!in_pair.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < entry_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!in_pair.load(std::memory_order_acquire)) {
        unpark.store(true, std::memory_order_release);
        releaser.join();
        FAIL() << "the releaser never entered its parked pair (release-side hook not invoked); the pin "
                  "cannot observe anything";
    }
    std::atomic<bool> engager_done{false};
    std::thread engager([&] {
        probe.request_brake_engage(0x01);
        engager_done.store(true, std::memory_order_release);
    });

    // Bounded observation: while the releaser is parked inside its pair,
    // the engage post must not complete (it is blocked on the post mutex).
    bool engager_done_while_parked = false;
    for (int i = 0; i < 200 && !engager_done_while_parked; ++i) {
        engager_done_while_parked = engager_done.load(std::memory_order_acquire);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    unpark.store(true, std::memory_order_release);
    releaser.join();
    engager.join();

    EXPECT_TRUE(release_accepted.load());
    EXPECT_FALSE(engager_done_while_parked)
        << "the engage post completed inside the parked release pair: the pair is not mutex-atomic";
    EXPECT_TRUE(engager_done.load()) << "after unparking, the engage post must complete";
    const auto [release_mask, engage_mask] = probe.brake_request_masks_for_test();
    EXPECT_EQ(engage_mask & 0x01, 0x01) << "last effect (the engager ran after the pair) must win";
    EXPECT_EQ(release_mask & 0x01, 0x00);
}

TEST_F(ControllerLoopFakeMasterTest, PendingGetterNeverFabricatesSettledZeroMidPair) {
    // The direction-swap post writes two words (OR release,
    // AND-clear engage); an unguarded reader interleaving with the pair can
    // compute a union of 0 that never existed. The seqlock (odd version
    // while the pair is in progress) plus the bounded-retry conservative
    // fallback guarantee: a call during the pair returns nonzero (either a
    // stable snapshot or the everything-pending fallback), deterministically
    // (pinned single-threaded with the mid-pair park hook).
    auto config = plem::fieldbus::test::test_controller_config<6>();
    RequestMailboxProbe probe(config);

    probe.request_brake_engage(0x01);  // pre-state: the true union is nonzero from here on

    std::atomic<bool> in_pair{false};
    std::atomic<bool> unpark{false};
    std::atomic<uint8_t> observed_mid_pair{0};
    probe.set_pair_hook([&] {
        in_pair.store(true, std::memory_order_release);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!unpark.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    std::thread releaser([&] { (void)probe.request_brake_release(0x01); });
    const auto entry_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!in_pair.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < entry_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!in_pair.load(std::memory_order_acquire)) {
        unpark.store(true, std::memory_order_release);
        releaser.join();
        FAIL() << "the releaser never entered its parked pair — the pin cannot observe anything";
    }

    // Deterministic mid-pair observation from this thread: version is odd
    // for the whole park, so the getter exhausts its bounded retries and
    // must return the conservative everything-pending mask, never 0.
    observed_mid_pair.store(probe.brake_operation_pending(), std::memory_order_release);
    unpark.store(true, std::memory_order_release);
    releaser.join();

    // Exact-value assertion: the parked pair holds the version odd, so
    // the getter must exhaust its bounded retries and return the DOF-6
    // everything-pending fallback 0x3F. Asserting != 0 alone is mutation-blind:
    // a seqlock-stripped getter would read the raw masks (0x01) and still pass.
    EXPECT_EQ(observed_mid_pair.load(), 0x3F) << "a mid-pair pending read must return the exact conservative fallback"
                                                 " (0 reports a false settle; a raw-mask value means the"
                                                 " seqlock/bounded-retry path is gone)";
    EXPECT_EQ(probe.brake_operation_pending() & 0x01, 0x01)
        << "after the pair completes the union must show the queued release";
}

// Process data exchange: every slave in OP
// The domain's working counter is complete only when every slave answers
// both halves of the process data, i.e. is in OP: a host waits on this
// before it calls the bus up.
TEST_F(ControllerLoopFakeMasterTest, ProcessDataCompleteFollowsTheDomainWorkingCounter) {
    FakeEcrt::set_domain_wc_state(EC_WC_INCOMPLETE);
    start_loop();
    settle();
    EXPECT_FALSE(controller_->process_data_complete());

    FakeEcrt::set_domain_wc_state(EC_WC_COMPLETE);
    settle();
    EXPECT_TRUE(controller_->process_data_complete());
}

// rt_sample.modes_of_operation_display reports the drive's 0x6061 display
// The RT sample's per-joint modes_of_operation_display relays the drive's own
// 0x6061 word (the TxPDO carried in every frame), so it shows what the drive
// reports, including NO_MODE before the mode lands and a rejected mode switch.
TEST_F(ControllerLoopFakeMasterTest, RtSampleModesOfOperationDisplayReportsDriveDisplayedMode) {
    // Distinct per-joint display feedback, written before the loop starts;
    // the loop only writes RxPDO fields, so the TxPDO region is race-free for
    // the loop's whole lifetime.
    uint8_t* pd = ecrt_domain_data(nullptr);
    constexpr uint8_t kDisplays[6] = {0, 8, 9, 10, 8, 9};
    for (uint32_t s = 0; s < 6; ++s) {
        EC_WRITE_U8(pd + pdo_offset(s, 0x6061), kDisplays[s]);
    }

    start_loop();

    // Every cycle emits one RtSample; any sample after start carries the
    // injected display words. Drain until one arrives (bounded poll).
    auto* queue = ctx_.queue<plem::core::monitoring::RtSample>();
    ASSERT_NE(queue, nullptr);
    plem::core::monitoring::RtSample sample{};
    bool got = wait_for([&] {
        bool any = false;
        while (auto* s = queue->front()) {
            sample = *s;
            queue->pop();
            any = true;
        }
        return any;
    });
    ASSERT_TRUE(got) << "the RT loop must emit RtSample";

    for (size_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(sample.modes_of_operation_display[j], static_cast<int8_t>(kDisplays[j]));
    }
}

// The RT observation record carries every axis of an 8-axis bus
// A bus carries up to eight axes (uint8_t joint masks). The per-cycle RtSample
// and the fault onset it derives must cover the last of them: rt_raw and
// ~/rt_events are the only places an operator sees axis 7's values and its
// SERVO_FAULT onset.
TEST(RtObservationWidth, AnEightAxisBusReportsItsLastAxisAndItsFaultOnset) {
    constexpr size_t kAxes = 8;
    FakeEcrt::reset();
    plem::core::monitoring::MonitoringContext ctx;
    ASSERT_TRUE(ctx.initialize().is_ok());
    auto bus = std::make_shared<EtherCATBus>(0);
    bus->set_servo_config(std::vector<ServoInfo>(kAxes, six_servos().front()));
    bus->set_io_config({one_io_info()});
    bus->set_io_device_factory(make_one_entry_device);
    bus->set_cycle_time_ns(1'000'000);
    const auto init = bus->initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    uint8_t* pd = ecrt_domain_data(nullptr);
    for (uint32_t j = 0; j < kAxes; ++j) {
        EC_WRITE_U16(pd + pdo_offset(j, 0x6041), 0x0020);  // NOT_READY_TO_SWITCH_ON
        EC_WRITE_U8(pd + pdo_offset(j, 0x6061), 0x0A);     // CST
    }
    EC_WRITE_S32(pd + pdo_offset(0, 0x6064), 100000);
    EC_WRITE_S32(pd + pdo_offset(7, 0x6064), 100000);
    EC_WRITE_U16(pd + pdo_offset(7, 0x6041), 0x0008);  // axis 7 in FAULT

    auto config = plem::fieldbus::test::test_controller_config<kAxes>();
    EtherCATController<kAxes> controller(config);
    controller.set_bus(bus);
    controller.set_mode_flags(std::make_shared<ControlModeFlags>());
    controller.set_shared_data(std::make_shared<plem::core::SharedJointData<kAxes>>());
    controller.set_monitoring_context(&ctx);
    auto started = controller.start();
    ASSERT_TRUE(started) << started.message();

    auto* samples = ctx.queue<plem::core::monitoring::RtSample>();
    auto* events = ctx.queue<plem::core::monitoring::Event>();
    plem::core::monitoring::RtSample sample{};
    bool got_sample = false;
    bool fault_onset_axis7 = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!(got_sample && fault_onset_axis7) && std::chrono::steady_clock::now() < deadline) {
        while (auto* s = samples->front()) {
            sample = *s;
            samples->pop();
            got_sample = true;
        }
        while (auto* e = events->front()) {
            fault_onset_axis7 |= e->type == EventType::SERVO_FAULT && e->joint_id == 7;
            events->pop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    controller.stop();

    ASSERT_TRUE(got_sample) << "the RT loop must emit RtSample";
    ASSERT_GT(sample.actual_position.size(), 7u) << "the record is narrower than the bus";
    EXPECT_EQ(sample.axis_count, kAxes);
    EXPECT_NE(sample.actual_position[0], 0.0f);
    EXPECT_FLOAT_EQ(sample.actual_position[7], sample.actual_position[0]) << "axis 7's position reaches the record";
    EXPECT_TRUE(fault_onset_axis7) << "axis 7's SERVO_FAULT onset must be reported";
}

// RT loop traverses registered IO devices (both directions)

TEST_F(ControllerLoopFakeMasterTest, RtLoopTraversesRegisteredIoDevicesBothDirections) {
    // The fixture's IO device came through the factory at initialize(); what
    // this pins is the RT loop's cyclic traversal, both directions.
    ASSERT_EQ(bus_->io_devices().size(), 1u);
    const auto io = std::dynamic_pointer_cast<OneEntryIoDevice>(bus_->io_devices().front());
    ASSERT_NE(io, nullptr);

    start_loop();
    ASSERT_TRUE(wait_for([&] { return io->reads.load() > 100 && io->writes.load() > 100; }))
        << "the 1 kHz loop must call the IO device's read() AND write() every cycle "
        << "(reads=" << io->reads.load() << ", writes=" << io->writes.load() << ")";
}

// Run-scoped loop carry resets on every stop()/start()
// The RT loop's per-run carry (the torque/target ZOH buffers, the RtSample
// scratch and its previous-wake stamp) lives as run_loop locals, so a
// restarted loop starts from zero instead of replaying the dead run's last
// values. Run carry must be reset for every loop start. Persisting the
// previous run's command through restart would replay its torque during
// initial ZOH cycles; persisting wake timing would produce bogus first-cycle
// jitter. Any relocation of these values must preserve their per-run lifetime.
//
// Determinism: RtQueue drops the newest on overflow, so after draining at
// stop() the next front() is run 2's first cycle no matter when we poll.
TEST_F(ControllerLoopFakeMasterTest, RunScopedLoopCarryResetsOnRestart) {
    auto* queue = ctx_.queue<plem::core::monitoring::RtSample>();
    ASSERT_NE(queue, nullptr);

    // run 1: publish a nonzero torque/target so the carry is dirty
    plem::core::JointCommand<6> torque{};
    for (size_t j = 0; j < 6; ++j) {
        torque.value[j] = 1.0 + static_cast<double>(j);
    }
    torque.update_ns = plem::core::now_ns();
    shared_->command.write(torque);

    plem::core::JointTargetValues<6> targets{};
    for (size_t j = 0; j < 6; ++j) {
        targets.commands[j].target_position = 10.0 + static_cast<double>(j);
        targets.commands[j].target_velocity = 20.0 + static_cast<double>(j);
    }
    targets.command_update_ns = plem::core::now_ns();
    shared_->joint_target.write(targets);

    start_loop();

    plem::core::monitoring::RtSample seen{};
    ASSERT_TRUE(wait_for([&] {
        while (auto* s = queue->front()) {
            seen = *s;
            queue->pop();
        }
        return seen.command_torque[0] != 0.0f;
    })) << "run 1 must carry the published torque into the sample";
    EXPECT_FLOAT_EQ(seen.command_position[0], 10.0f);

    controller_->stop();

    // Nothing produces after the join, so the drain is total.
    while (queue->front()) {
        queue->pop();
    }

    // run 2: both writers frozen mid-write => try_read fails => ZOH
    SeqlockDataTestPeer::freeze_writer_mid_write(shared_->command);
    SeqlockDataTestPeer::freeze_writer_mid_write(shared_->joint_target);

    start_loop();

    plem::core::monitoring::RtSample first{};
    ASSERT_TRUE(wait_for([&] {
        if (auto* s = queue->front()) {
            first = *s;
            return true;
        }
        return false;
    })) << "run 2 must emit an RtSample";

    EXPECT_EQ(first.command_zoh, 1) << "the frozen writer must force the zero-order-hold path";
    for (size_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_FLOAT_EQ(first.command_torque[j], 0.0f) << "a restart must not replay run 1's torque";
        EXPECT_FLOAT_EQ(first.command_position[j], 0.0f);
        EXPECT_FLOAT_EQ(first.command_velocity[j], 0.0f);
    }
    EXPECT_FLOAT_EQ(first.loop_jitter_us, 0.0f) << "a run's first cycle has no previous wake";

    // Leave the conduits consistent for TearDown.
    SeqlockDataTestPeer::resume_writer_and_complete(shared_->command, torque);
    SeqlockDataTestPeer::resume_writer_and_complete(shared_->joint_target, targets);
}

// The fault-reset cutoff is a run boundary, not a loop tail
// A request posted while the controller was stopped describes an axis state
// that is now arbitrarily old; firing it against the next run's first
// recognized FAULT is not what the operator asked for. The discard therefore
// happens at run entry: a tail could not cover requests that arrive after it,
// nor the early returns that never reach one.
//
// Two things can be pending when a run begins: a request (T) and an owed
// falling edge (L). The cutoff owns only the first. The tests below cover
// the three nonempty combinations: T alone, reached by two different routes; L
// alone; and both at once. Two more sit outside that grid: a run that
// early-returns before the cutoff ever runs, and a request posted from inside
// a run, which the cutoff must not touch at all.
//
// These tests read the wire through a send observer rather than by polling the
// fake domain: the observer runs on the worker thread once per cycle with that
// cycle's bytes in place, and the test inspects the recording after join().
// Polling from the test thread both races the writer and misses a pulse that
// lasts one cycle, including a violation on the very first frame.
class FaultResetCutoffTest : public ControllerLoopFakeMasterTest {
  protected:
    void SetUp() override {
        ControllerLoopFakeMasterTest::SetUp();
        pd_ = ecrt_domain_data(nullptr);
        joint0_status_off_ = pdo_offset(0, 0x6041);
        joint0_control_off_ = pdo_offset(0, 0x6040);
        plem::fieldbus::test::FakeEcrt::set_send_observer([this] {
            // Worker thread. The vector is written only here and read only
            // after join(); the counter is the sole cross-thread channel, so
            // the test can wait on progress without touching the vector.
            wire_.push_back(EC_READ_U16(pd_ + joint0_control_off_));
            if (post_request_on_first_frame_.exchange(false, std::memory_order_acq_rel)) {
                // Posted from the worker itself, after this cycle's write and
                // long after the cutoff: the request lands between two frames,
                // so the next one owns it. Posting from the test thread would
                // race which frame that is.
                bus_->servo_devices()[0]->request_fault_reset();
            }
            frames_.fetch_add(1, std::memory_order_release);
        });
    }

    void TearDown() override {
        // Stop before clearing the observer: a worker still in a cycle would
        // otherwise call an emptied std::function, or race its destruction.
        if (controller_) {
            controller_->stop();
        }
        plem::fieldbus::test::FakeEcrt::set_send_observer(nullptr);
        ControllerLoopFakeMasterTest::TearDown();
    }

    /// Run the loop until it has produced at least `cycles` frames, then stop
    /// and join. Everything the run wrote is in wire_ afterwards.
    ///
    /// Always paired with a stop. start() on an already-running controller
    /// returns immediately without spawning, so a second call is harmless.
    void run_for(size_t cycles) {
        const size_t target = frames_.load(std::memory_order_acquire) + cycles;
        start_loop();
        const bool cycled = wait_for([&] { return frames_.load(std::memory_order_acquire) >= target; });
        // Join first, unconditionally: an assertion that returns early would
        // leave the worker running against storage the caller is about to read.
        controller_->stop();
        ASSERT_TRUE(cycled) << "the loop must actually cycle";
    }

    DeviceServo& servo() {
        return *bus_->servo_devices()[0];
    }

    uint32_t joint0_status_off_ = 0;  // set in SetUp, before any loop runs
    uint32_t joint0_control_off_ = 0;

    uint8_t* pd_ = nullptr;
    std::vector<uint16_t> wire_;     // worker-written, test-read after join
    std::atomic<size_t> frames_{0};  // the only field both threads touch
    /// Set before start() to have the worker post one request from inside its
    /// first cycle. Test-written before the thread exists, worker-cleared.
    std::atomic<bool> post_request_on_first_frame_{false};
};

// The plain case: posted with no run in flight.
TEST_F(FaultResetCutoffTest, RequestPostedWhileStoppedDoesNotFireOnTheNextRun) {
    EC_WRITE_U16(pd_ + joint0_status_off_, 0x0208);  // FAULT: everything a pulse needs
    servo().request_fault_reset();

    run_for(20);
    for (size_t i = 0; i < wire_.size(); ++i) {
        EXPECT_NE(wire_[i], 0x0080) << "the stale request must never publish — frame " << i;
    }
    ASSERT_FALSE(wire_.empty());
}

// A request can survive a stop: accepted while a falling edge is owed, it is
// KEPT (write() stores it back), and stopping before the next cycle leaves it
// pending.
TEST_F(FaultResetCutoffTest, RequestKeptAcrossAStopDoesNotFireOnTheNextRun) {
    // Build the KEPT state on the device, with no loop running, so the frame
    // it lands on is chosen rather than raced for: publish a pulse, then have
    // the next recognized-FAULT frame hold a second request.
    EC_WRITE_U16(pd_ + joint0_status_off_, 0x0208);
    servo().read(pd_);
    ASSERT_TRUE(servo().reset_fault());
    servo().write(pd_);
    ASSERT_EQ(EC_READ_U16(pd_ + joint0_control_off_), 0x0080) << "pulse out; a falling edge is owed";

    ASSERT_TRUE(servo().reset_fault()) << "still a recognized FAULT: accepted";
    servo().read(pd_);
    servo().write(pd_);
    ASSERT_EQ(EC_READ_U16(pd_ + joint0_control_off_), 0x0000) << "owed low — and the request is KEPT, not dropped";

    // That request is pending across the boundary. The cutoff must drop it.
    run_for(20);
    for (size_t i = 0; i < wire_.size(); ++i) {
        EXPECT_NE(wire_[i], 0x0080) << "a KEPT request must not survive into the next run — frame " << i;
    }
    ASSERT_FALSE(wire_.empty());
}

// The pulse state is not a request and the cutoff must not touch it: if the
// previous run's last frame was high, this run's first frame owes the falling
// edge. Clearing it alongside the request would leave bit 7 standing on the
// wire across the restart, the exact defect the pulse exists to prevent. The
// first recorded frame is the whole assertion, which is why it is recorded
// rather than polled.
TEST_F(FaultResetCutoffTest, ARestartAfterAPublishedPulseStillOwesTheFallingEdge) {
    EC_WRITE_U16(pd_ + joint0_status_off_, 0x0208);
    servo().read(pd_);
    servo().request_fault_reset();
    servo().write(pd_);
    ASSERT_EQ(EC_READ_U16(pd_ + joint0_control_off_), 0x0080) << "the wire is left high";

    run_for(20);
    ASSERT_FALSE(wire_.empty());
    EXPECT_EQ(wire_[0], 0x0000) << "the new run's FIRST write must take the wire low";
    for (size_t i = 0; i < wire_.size(); ++i) {
        EXPECT_EQ(wire_[i], 0x0000) << "and it stays low: the cutoff dropped no request — frame " << i;
    }
}

// A wide pulse cut by a stop does not resume: the restarted run's first
// frame is the falling edge, whatever width was left.
TEST_F(FaultResetCutoffTest, ARestartMidWidePulseOwesOnlyTheFallingEdge) {
    servo().set_fault_reset_pulse_frames(5);
    EC_WRITE_U16(pd_ + joint0_status_off_, 0x0208);
    servo().read(pd_);
    servo().request_fault_reset();
    servo().write(pd_);
    ASSERT_EQ(EC_READ_U16(pd_ + joint0_control_off_), 0x0080) << "the wire is left high, four frames still owed";

    run_for(20);
    ASSERT_FALSE(wire_.empty());
    for (size_t i = 0; i < wire_.size(); ++i) {
        EXPECT_EQ(wire_[i], 0x0000) << "frame " << i;
    }
}

// Both at once: the state the other three never reach, and the only one that
// tells the two jobs apart. Guard the cutoff's clear with
// `if (!reset_pulse_published_)` and every other test in this family still
// passes: they enter with one of T/L, never both. This one fails, because the
// run's first frame pays the owed low and clears the latch, and its second
// frame publishes the request the cutoff was supposed to drop.
TEST_F(FaultResetCutoffTest, ACutoffDropsAPendingRequestEvenWhileAFallingEdgeIsOwed) {
    EC_WRITE_U16(pd_ + joint0_status_off_, 0x0208);
    servo().read(pd_);
    servo().request_fault_reset();
    servo().write(pd_);
    ASSERT_EQ(EC_READ_U16(pd_ + joint0_control_off_), 0x0080) << "seed: a falling edge is owed";
    servo().request_fault_reset();  // and a request is pending across the boundary

    run_for(20);
    ASSERT_FALSE(wire_.empty());
    EXPECT_EQ(wire_[0], 0x0000) << "the first frame still pays the owed low";
    for (size_t i = 0; i < wire_.size(); ++i) {
        EXPECT_EQ(wire_[i], 0x0000) << "and no later frame publishes the dropped request — frame " << i;
    }
}

// Everything above posts before the run starts, which leaves the family blind
// to the case the feature exists for: an operator asking for a reset while the
// bus is running. Move the discard from run entry into the loop body and every
// test above still passes, because none of them has a request left to delete,
// while every live request an operator makes dies before it can be published.
TEST_F(FaultResetCutoffTest, ARequestPostedDuringARunReachesTheWire) {
    EC_WRITE_U16(pd_ + joint0_status_off_, 0x0208);
    post_request_on_first_frame_.store(true, std::memory_order_release);

    run_for(20);
    ASSERT_GE(wire_.size(), 3u);
    EXPECT_EQ(wire_[0], 0x0000) << "the cutoff frame: nothing was pending across the boundary";
    EXPECT_EQ(wire_[1], 0x0080) << "the request posted mid-run publishes on the very next frame";
    EXPECT_EQ(wire_[2], 0x0000) << "and its falling edge follows immediately";
    for (size_t i = 3; i < wire_.size(); ++i) {
        EXPECT_EQ(wire_[i], 0x0000) << "one request, one pulse — frame " << i;
    }
}

// Shared plane: a torn read holds the previous cycle's torque
// The ZOH is the whole reason the bus loop uses try_read instead of read
// here: the writer shares this core at a lower RT priority and can
// never finish while we spin. What "hold" means is load-bearing and easy to
// break silently: resending zero instead of the last commanded torque would
// let the axes drop, and resending garbage would be worse.
TEST_F(ControllerLoopFakeMasterTest, SharedPlaneTornReadHoldsPreviousTorque) {
    auto* queue = ctx_.queue<plem::core::monitoring::RtSample>();
    ASSERT_NE(queue, nullptr);

    plem::core::JointCommand<6> torque{};
    for (size_t j = 0; j < 6; ++j) {
        torque.value[j] = 3.0 + static_cast<double>(j);
    }
    torque.update_ns = plem::core::now_ns();
    shared_->command.write(torque);

    start_loop();

    plem::core::monitoring::RtSample seen{};
    ASSERT_TRUE(wait_for([&] {
        while (auto* s = queue->front()) {
            seen = *s;
            queue->pop();
        }
        return seen.command_torque[0] == 3.0f && seen.command_zoh == 0;
    })) << "the loop must first observe the published torque with no ZOH";

    // Writer stops inside its critical section: every later try_read fails.
    SeqlockDataTestPeer::freeze_writer_mid_write(shared_->command);

    plem::core::monitoring::RtSample held{};
    ASSERT_TRUE(wait_for([&] {
        while (auto* s = queue->front()) {
            held = *s;
            queue->pop();
        }
        return held.command_zoh == 1;
    })) << "a frozen writer must take the ZOH path";

    for (size_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_FLOAT_EQ(held.command_torque[j], static_cast<float>(3.0 + static_cast<double>(j)))
            << "the held cycle must resend the last landed torque, not zero";
    }

    SeqlockDataTestPeer::resume_writer_and_complete(shared_->command, torque);
}

// Shared plane: the ZOH flag clears once reads land again
// command_zoh feeds the phase-lock acceptance gate, which reads a growing
// count as misalignment. A flag that latched would make every run after one
// torn read look permanently misaligned.
TEST_F(ControllerLoopFakeMasterTest, SharedPlaneZohClearsWhenReadsResume) {
    auto* queue = ctx_.queue<plem::core::monitoring::RtSample>();
    ASSERT_NE(queue, nullptr);

    plem::core::JointCommand<6> torque{};
    torque.value[0] = 7.0;
    torque.update_ns = plem::core::now_ns();
    shared_->command.write(torque);

    start_loop();
    SeqlockDataTestPeer::freeze_writer_mid_write(shared_->command);

    plem::core::monitoring::RtSample sample{};
    ASSERT_TRUE(wait_for([&] {
        while (auto* s = queue->front()) {
            sample = *s;
            queue->pop();
        }
        return sample.command_zoh == 1;
    })) << "precondition: the frozen writer must raise the ZOH flag";

    plem::core::JointCommand<6> refreshed{};
    refreshed.value[0] = 11.0;
    refreshed.update_ns = plem::core::now_ns();
    SeqlockDataTestPeer::resume_writer_and_complete(shared_->command, refreshed);

    ASSERT_TRUE(wait_for([&] {
        while (auto* s = queue->front()) {
            sample = *s;
            queue->pop();
        }
        return sample.command_zoh == 0;
    })) << "the flag must clear on the first cycle whose read lands";
    EXPECT_FLOAT_EQ(sample.command_torque[0], 11.0f);
}

// Precondition: the bus must carry exactly the controller's axis count
// DOF is this controller's exact axis count. Without the servo_count()==DOF
// start gate, too many drives would index beyond DOF-sized buffers; too few
// would keep all_drives_enabled false, the torque gate closed and missing-axis
// brake sequences unsettled. validate_preconditions() refuses both mismatches
// before the thread starts.
// The assembly layer checks the same count on its own path; this gate covers
// every other way a controller reaches start(), which all route through
// validate_preconditions() before the thread exists.
namespace {

/// Exposes the protected gate so this test can assert the refusal without
/// starting the loop it is proving unsafe.
class PreconditionProbeController : public EtherCATController<6> {
  public:
    using EtherCATController<6>::EtherCATController;
    using EtherCATController<6>::validate_preconditions;
};

}  // namespace

// Shared body: build a bus with `count` drives and return the gate's verdict.
namespace {

plem::core::Result<void> precondition_for_bus_of(size_t count, int controller_cycle_us = 1000) {
    auto bus = std::make_shared<EtherCATBus>(0);
    std::vector<ServoInfo> servos(count, six_servos().front());
    bus->set_servo_config(servos);
    bus->set_io_config({one_io_info()});
    bus->set_io_device_factory(make_one_entry_device);
    bus->set_cycle_time_ns(1'000'000);
    const auto init = bus->initialize();
    EXPECT_TRUE(init.is_ok()) << init.message();

    EtherCATController<6>::Config config;
    config.cycle_time_us = controller_cycle_us;
    PreconditionProbeController controller{config};
    controller.set_bus(bus);
    controller.set_mode_flags(std::make_shared<ControlModeFlags>());
    controller.set_shared_data(std::make_shared<plem::core::SharedJointData<6>>());

    return controller.validate_preconditions();
}

}  // namespace

TEST(EtherCATControllerPreconditions, RefusesBusWithMoreDrivesThanAxes) {
    FakeEcrt::reset();
    const auto result = precondition_for_bus_of(7);
    EXPECT_FALSE(result) << "a bus larger than the controller's axis count must be refused before the loop starts";
    EXPECT_NE(result.message().find('7'), std::string::npos) << result.message();
    EXPECT_NE(result.message().find('6'), std::string::npos) << result.message();
}

TEST(EtherCATControllerPreconditions, RefusesBusWithFewerDrivesThanAxes) {
    FakeEcrt::reset();
    const auto result = precondition_for_bus_of(4);
    EXPECT_FALSE(result) << "a short bus can never open the torque gate — refuse it instead of starting a "
                            "robot that cannot move";
    EXPECT_NE(result.message().find('4'), std::string::npos) << result.message();
}

TEST(EtherCATControllerPreconditions, RefusesBusRunningAtAnotherPeriod) {
    // The bus derives its send interval, fault-reset pulse length and IO
    // device period from its own period; a loop at another period would
    // run all three on the wrong clock.
    FakeEcrt::reset();
    const auto result = precondition_for_bus_of(6, 500);
    EXPECT_FALSE(result) << "a bus at 1 ms under a 500 us loop must be refused before the loop starts";
    EXPECT_NE(result.message().find("1000000"), std::string::npos) << result.message();
    EXPECT_NE(result.message().find("500000"), std::string::npos) << result.message();
}

TEST(EtherCATControllerPreconditions, AcceptsBusMatchingAxisCount) {
    FakeEcrt::reset();
    const auto result = precondition_for_bus_of(6);
    EXPECT_TRUE(result) << result.message();
}

namespace {

/// A controller wired with the shared data plane and nothing else: every
/// assembly's shape.
struct SharedPlaneRig {
    std::shared_ptr<EtherCATBus> bus = std::make_shared<EtherCATBus>(0);
    std::shared_ptr<ControlModeFlags> flags = std::make_shared<ControlModeFlags>();
    std::shared_ptr<plem::core::SharedJointData<6>> shared = std::make_shared<plem::core::SharedJointData<6>>();
    std::unique_ptr<PreconditionProbeController> controller;

    SharedPlaneRig() {
        FakeEcrt::reset();
        bus->set_servo_config(six_servos());
        bus->set_cycle_time_ns(1'000'000);
        const auto init = bus->initialize();
        EXPECT_TRUE(init.is_ok()) << init.message();
        auto config = plem::fieldbus::test::test_controller_config<6>();
        controller = std::make_unique<PreconditionProbeController>(config);
        controller->set_bus(bus);
        controller->set_mode_flags(flags);
        controller->set_shared_data(shared);
    }
};

}  // namespace

TEST(EtherCATControllerSharedPlane, PreconditionsAdmitSharedData) {
    SharedPlaneRig rig;
    const auto result = rig.controller->validate_preconditions();
    EXPECT_TRUE(result) << result.message();
}

TEST(EtherCATControllerSharedPlane, PreconditionsRefuseMissingSharedData) {
    SharedPlaneRig rig;
    rig.controller->set_shared_data(nullptr);
    const auto result = rig.controller->validate_preconditions();
    EXPECT_FALSE(result) << "no SharedJointData must be refused before the loop starts";
    EXPECT_NE(result.message().find("no SharedJointData set"), std::string::npos) << result.message();
}

TEST(EtherCATControllerSharedPlane, LoopCyclesOnTheSharedPlane) {
    // The loop runs on the shared data plane alone. Cycling is observed
    // through the wake stamp the shared plane publishes every cycle.
    SharedPlaneRig rig;
    ASSERT_TRUE(rig.controller->start()) << "start must accept the shared-plane wiring";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    int64_t first_stamp = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        first_stamp = rig.shared->last_cycle_ns.load(std::memory_order_relaxed);
        if (first_stamp != 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_NE(first_stamp, 0) << "the loop never published a wake stamp";
    EXPECT_TRUE(plem::fieldbus::test::wait_for([&] {
        return rig.shared->last_cycle_ns.load(std::memory_order_relaxed) > first_stamp;
    })) << "the loop stopped cycling";
    rig.controller->stop();
}
