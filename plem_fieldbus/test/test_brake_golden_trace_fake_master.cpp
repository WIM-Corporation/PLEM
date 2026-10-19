// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_brake_golden_trace_fake_master.cpp
/// @brief Brake golden traces: cycle-exact mode/controlword sequences,
/// driven one synchronous cycle at a time through the protected stepping
/// seam (run_cycle(), run_loop's per-cycle order; one test walks a release
/// through the real loop in lock step to pin that order). Mode-path release/engage = the native CiA 402 sequence
/// (measured on a bench: the drive couples its brake to drive power).

#include <gtest/gtest.h>

#include <fmt/format.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
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
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/core/types/shared_joint_data.hpp>

#include "device_servo_test_access.hpp"
#include "fake_master_fixture.hpp"
#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/controller.hpp"
#include "plem/fieldbus/test/cia402_drive_sim.hpp"
#include "plem/fieldbus/test/ethercat_test_probes.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

using plem::core::monitoring::EventType;
using plem::core::units::Pulse;
using plem::fieldbus::ethercat::EtherCATBus;
using plem::fieldbus::ethercat::EtherCATController;
using plem::fieldbus::ethercat::FaultResetPacing;
using plem::fieldbus::ethercat::FaultResetPolicy;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::test::DeviceServoTestAccess;
using plem::fieldbus::test::EtherCATControllerTestProbe;
using plem::fieldbus::test::FakeEcrt;

namespace {

// Fixtures

using plem::fieldbus::test::six_servos;

/// Protected-seam probe: the golden traces call the sequencing entry points
/// directly, one per fake-master cycle.
class SequencingProbe : public EtherCATControllerTestProbe<6> {
  public:
    using EtherCATControllerTestProbe<6>::EtherCATControllerTestProbe;
    using EtherCATController<6>::execute_state_transitions;
    using EtherCATController<6>::report_fault_reset_publications;
    using EtherCATController<6>::set_brake_setup;
    using EtherCATController<6>::set_fault_reset_policy;
    using EtherCATController<6>::set_brake_release_sequence;
};

using plem::fieldbus::test::pdo_offset;

// Canonical CiA 402 status words (masked): NOT_READY 0x0020, READY 0x0221,
// SWITCHED_ON 0x0233, OPERATION_ENABLED 0x0237, FAULT 0x0208 (fed with bit 5 set: 0x0228).
constexpr uint16_t kStNotReady = 0x0020;
constexpr uint16_t kStReady = 0x0221;
constexpr uint16_t kStSwitchedOn = 0x0233;
constexpr uint16_t kStOpEnabled = 0x0237;
constexpr uint16_t kStFault = 0x0228;                // a fault word whose bit 5 (quick-stop,
                                                     // active-low) rides along with the fault bit
constexpr uint16_t kStFaultReactionActive = 0x022F;  // masked 0x2F; bit 5 rides along, as with the fault word
constexpr uint16_t kStSwitchOnDisabled = 0x0240;     // masked 0x0040
constexpr uint16_t kStSodRestWord = 0x0060;          // bench resting word: SOD (CiA 402 bit5 don't-care)
constexpr uint16_t kStQuickStopActive = 0x0207;      // masked 0x07: QUICK_STOP_ACTIVE
constexpr uint16_t kStUnrecognized = 0x0064;         // masks to no known CiA 402 pattern

/// The release walk of one joint from NOT_READY: (fed status, controlword on
/// the wire after that cycle).
constexpr std::pair<uint16_t, uint16_t> kReleaseTrace[] = {
    {kStNotReady, 0x0006},    // command READY
    {kStReady, 0x0006},       // step reached: consumed, word holds
    {kStReady, 0x0007},       // command SWITCHED_ON
    {kStSwitchedOn, 0x0007},  // step reached
    {kStSwitchedOn, 0x000F},  // command OPERATION_ENABLED
    {kStOpEnabled, 0x000F},   // final step reached: released
};

// The pacing every bounded-reset trace below was recorded under (200 cycles at
// this fixture's 1 ms period, 25 accepted requests per episode), injected
// explicitly because the model owns it.
// Every expected word and cycle index in this file is an independent literal;
// none is derived from these two numbers.
constexpr FaultResetPacing kGoldenPacing{200, 25};

/// Drain the queue into a copy (one drain serves several counts; a second
/// helper on the same queue would find it empty).
std::vector<plem::core::monitoring::Event> drain_events(plem::core::monitoring::MonitoringContext& ctx) {
    std::vector<plem::core::monitoring::Event> out;
    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    while (auto* event = queue->front()) {
        out.push_back(*event);
        queue->pop();
    }
    return out;
}

/// The events of one type in a drained batch, in order. The diagnostics the
/// traces read: MODES_OF_OPERATION_MISMATCH (the readiness refusal and the
/// in-service trip: joint_id = the axis, error_code = its statusword, value =
/// its 0x6061), OPERATION_ENABLED_LOST (joint_id = the released axis
/// seen off OPERATION_ENABLED, error_code = its statusword, value = 0) and
/// SLAVE_NOT_OPERATIONAL (error_code = the AL state, value = the ring position).
std::vector<plem::core::monitoring::Event> events_of(const std::vector<plem::core::monitoring::Event>& events,
                                                     EventType type) {
    std::vector<plem::core::monitoring::Event> out;
    for (const auto& event : events) {
        if (event.type == type) {
            out.push_back(event);
        }
    }
    return out;
}

int count_of(const std::vector<plem::core::monitoring::Event>& events, plem::core::monitoring::EventType type) {
    int n = 0;
    for (const auto& event : events) {
        n += event.type == type ? 1 : 0;
    }
    return n;
}

class BrakeGoldenTraceTest : public ::testing::Test {
  protected:
    void SetUp() override {
        build(six_servos(), 0x0A);
    }

    /// Bring up the fake bench: `servos` on the bus, then `display_seed` on
    /// every axis's 0x6061 before the first read. The golden traces seed CST
    /// (a drive that accepted its mode); the cold POST_ENABLE bench seeds 0
    /// (a drive that has not reported yet). Statuswords stay per-test;
    /// neither seed says "a drive that answered a controlword".
    void build(std::vector<ServoInfo> servos, uint8_t display_seed) {
        FakeEcrt::reset();
        bus_ = std::make_shared<EtherCATBus>(0);
        bus_->set_servo_config(std::move(servos));
        const auto init = bus_->initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
        pd_ = ecrt_domain_data(nullptr);
        // The fake domain is zeroed at reset and never echoes 0x6060 into
        // 0x6061, so the TxPDO byte is seeded once here.
        for (uint32_t j = 0; j < 6; ++j) {
            feed_display(j, display_seed);
        }

        auto config = plem::fieldbus::test::test_controller_config<6>();
        controller_ = std::make_unique<SequencingProbe>(config);
        controller_->set_bus(bus_);
    }

    /// One synchronous fake-master cycle in run_loop order: read every servo,
    /// one brake step (motion_possible as the bus reports it from this
    /// cycle's PDO image, which is production's input, so a forced engage stays
    /// pending exactly while every drive is still enabled), the sequencing
    /// step, then write every servo.
    void run_cycle() {
        for (const auto& servo : bus_->servo_devices()) {
            servo->read(pd_);
        }
        controller_->step_brake_requests_for_test(bus_->motion_possible());
        controller_->execute_state_transitions();
        for (const auto& servo : bus_->servo_devices()) {
            servo->write(pd_);
        }
        controller_->report_fault_reset_publications();
    }

    /// Per-joint wire access on the fake domain grid.
    uint16_t controlword(uint32_t joint) const {
        return EC_READ_U16(pd_ + pdo_offset(joint, 0x6040));
    }
    void feed_status(uint32_t joint, uint16_t word) {
        EC_WRITE_U16(pd_ + pdo_offset(joint, 0x6041), word);
    }
    void feed_display(uint32_t joint, uint8_t mode) {
        EC_WRITE_U8(pd_ + pdo_offset(joint, 0x6061), mode);
    }

    /// One cycle for joint 0: feed the drive's statusword, cycle, and return
    /// the (controlword, mode) now on the wire.
    std::pair<uint16_t, uint8_t> step(uint16_t statusword) {
        feed_status(0, statusword);
        run_cycle();
        return {controlword(0), EC_READ_U8(pd_ + pdo_offset(0, 0x6060))};
    }

    /// Hold a recognized FAULT until the bounded policy publishes exactly one
    /// pulse. The fault branch issues no command, so `standing` (the word the
    /// walk last wrote) is what the wire carries until the pulse replaces it.
    void hold_fault_until_pulse(uint16_t standing) {
        for (int cycle = 0; cycle < 199; ++cycle) {
            ASSERT_EQ(step(kStFault).first, standing) << "pre-pulse fault cycle " << cycle;
        }
        ASSERT_EQ(step(kStFault).first, 0x0080) << "one pulse at the interval";
    }

    /// Walk the release sequence for joint 0 from NOT_READY to completion,
    /// asserting the exact per-cycle controlword trace (the golden sequence).
    void walk_release_sequence_and_pin() {
        int cycle = 0;
        for (const auto& [fed, expected_cw] : kReleaseTrace) {
            SCOPED_TRACE("sequence cycle " + std::to_string(cycle++));
            const auto [cw, mode] = step(fed);
            EXPECT_EQ(cw, expected_cw);
            EXPECT_EQ(mode, 0x0A);  // CST all the way; no mode is ever held here
        }
    }

    /// One cycle feeding both joint 0 and joint 1 statuswords (a shared-brake
    /// module needs both axes initialized/checked). Returns the two
    /// controlwords now on the wire.
    std::pair<uint16_t, uint16_t> step_pair(uint16_t st0, uint16_t st1) {
        feed_status(0, st0);
        feed_status(1, st1);
        run_cycle();
        return {controlword(0), controlword(1)};
    }

    /// Set ControlModeFlags::motion_enabled, the consume-time
    /// authority: request release consumption requires a committed
    /// motion mode (the harness default flags are motion-disabled, exactly
    /// like a cold BRAKED assembly).
    void arm_motion() {
        flags_ = std::make_shared<plem::core::ControlModeFlags>();
        flags_->motion_enabled.store(true, std::memory_order_release);
        controller_->set_mode_flags(flags_);
    }

    /// One cycle feeding all six joints the same statusword (optionally one
    /// joint a different word); the all-drives torque-gate publication needs
    /// every axis initialized and checked, unlike the joint-0 golden traces.
    void feed_all_and_step(uint16_t statusword, int except_joint = -1, uint16_t except_word = 0) {
        feed_all(statusword);
        if (except_joint >= 0) {
            feed_status(static_cast<uint32_t>(except_joint), except_word);
        }
        run_cycle();
    }

    void feed_all(uint16_t statusword) {
        for (uint32_t j = 0; j < 6; ++j) {
            feed_status(j, statusword);
        }
    }

    /// Committed {P_STOP, FREEDRIVE} with every axis released by the recovery
    /// lane: the aggregate stop's edge is consumed while every drive rests
    /// (the forced engage settles at once), then a full release posted
    /// through the mailbox walks every drive to OPERATION_ENABLED. Leaves
    /// every controlword at 0x000F and the torque gate open.
    void enter_recovery_released() {
        static constexpr plem::fieldbus::ethercat::CommittedModeSample kFreedrive{true, true};
        controller_->set_committed_mode_probe(+[](const void*) noexcept { return kFreedrive; }, nullptr);
        flags_ = std::make_shared<plem::core::ControlModeFlags>();
        flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
        controller_->set_mode_flags(flags_);
        feed_all(kStNotReady);
        run_cycle();  // the rising edge: forced engage, settled at rest
        ASSERT_TRUE(controller_->request_brake_release(0x3F));
        run_cycle();  // pending resolves; the release starts
        for (uint32_t j = 0; j < 6; ++j) {
            ASSERT_EQ(controlword(j), 0x0006) << "joint " << j;
        }
        feed_all(kStReady);
        run_cycle();
        run_cycle();
        feed_all(kStSwitchedOn);
        run_cycle();
        run_cycle();
        feed_all(kStOpEnabled);
        run_cycle();
        for (uint32_t j = 0; j < 6; ++j) {
            ASSERT_EQ(controlword(j), 0x000F) << "joint " << j;
        }
        ASSERT_TRUE(flags_->all_drives_enabled.load(std::memory_order_acquire));
        ASSERT_EQ(controller_->brake_engaged(), 0x00);
    }

    /// Release every joint under an armed motion mode and walk all six to
    /// OPERATION_ENABLED: the in-service starting point (torque gate open,
    /// every group released, every sequence completed as a release).
    void release_all_and_enable() {
        feed_all(kStNotReady);
        run_cycle();
        ASSERT_TRUE(controller_->request_brake_release(0x3F));
        for (int cycle = 0; cycle < 3 && controlword(5) != 0x0006; ++cycle) {
            run_cycle();
        }
        for (uint32_t j = 0; j < 6; ++j) {
            ASSERT_EQ(controlword(j), 0x0006) << "joint " << j;
        }
        feed_all(kStReady);
        run_cycle();
        run_cycle();
        feed_all(kStSwitchedOn);
        run_cycle();
        run_cycle();
        feed_all(kStOpEnabled);
        run_cycle();
        for (uint32_t j = 0; j < 6; ++j) {
            ASSERT_EQ(controlword(j), 0x000F) << "joint " << j;
        }
        ASSERT_TRUE(flags_->all_drives_enabled.load(std::memory_order_acquire));
        ASSERT_EQ(controller_->brake_engaged(), 0x00);
    }

    std::shared_ptr<EtherCATBus> bus_;
    std::unique_ptr<SequencingProbe> controller_;
    std::shared_ptr<plem::core::ControlModeFlags> flags_;
    uint8_t* pd_ = nullptr;
};

// Scenario 1: mode-path release is the native sequence

TEST_F(BrakeGoldenTraceTest, StartStateReleaseWalksSequenceImmediately) {
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01);  // cold: engaged
    controller_->set_brake_release_sequence(0x01);
    walk_release_sequence_and_pin();
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x00);  // sequence completion = released
}

TEST_F(BrakeGoldenTraceTest, MaskedReleaseRequestRunsSequencePoweredOpen) {
    // request_brake_release(mask) = motion semantics (powered open),
    // which needs the drive enabled.
    arm_motion();  // release consumption requires a committed motion mode
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    walk_release_sequence_and_pin();
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x00);
}

// run_cycle() copies run_loop's per-cycle order. The same masked release
// through the real RT loop, parked after every cycle, writes the same trace:
// if run_loop moves the state transitions ahead of the brake step, or the
// servo write ahead of the state transitions, the two part here.
TEST(BrakeGoldenTraceRunLoopTest, TheRealLoopWritesTheHarnessReleaseTrace) {
    FakeEcrt::reset();
    auto bus = std::make_shared<EtherCATBus>(0);
    bus->set_servo_config(six_servos());
    const auto init = bus->initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    uint8_t* pd = bus->domain_pd();
    for (uint32_t j = 0; j < 6; ++j) {
        EC_WRITE_U8(pd + pdo_offset(j, 0x6061), 0x0A);  // CST on the mode display, as the harness seeds it
    }
    plem::core::monitoring::MonitoringContext ctx;
    ASSERT_TRUE(ctx.initialize().is_ok());
    auto flags = std::make_shared<plem::core::ControlModeFlags>();
    flags->motion_enabled.store(true, std::memory_order_release);
    plem::fieldbus::test::Lockstep gate;
    EtherCATController<6> controller(plem::fieldbus::test::test_controller_config<6>());
    controller.set_bus(bus);
    controller.set_mode_flags(flags);
    controller.set_shared_data(std::make_shared<plem::core::SharedJointData<6>>());
    controller.set_monitoring_context(&ctx);
    FakeEcrt::set_send_observer([&gate] { gate.on_send(); });
    struct StopLoop {
        plem::fieldbus::test::Lockstep& gate;
        EtherCATController<6>& controller;
        ~StopLoop() {
            gate.free_run();
            controller.stop();
            FakeEcrt::set_send_observer(nullptr);
        }
    } stop_loop{gate, controller};
    ASSERT_TRUE(controller.start());
    ASSERT_TRUE(gate.wait_first());

    ASSERT_TRUE(controller.request_brake_release(0x01));
    int cycle = 0;
    for (const auto& [fed, expected_cw] : kReleaseTrace) {
        SCOPED_TRACE("loop cycle " + std::to_string(cycle++));
        EC_WRITE_U16(pd + pdo_offset(0, 0x6041), fed);
        ASSERT_TRUE(gate.step()) << "the RT loop stopped cycling";
        EXPECT_EQ(EC_READ_U16(pd + pdo_offset(0, 0x6040)), expected_cw);
    }
    EXPECT_EQ(controller.brake_engaged() & 0x01, 0x00);
}

// The boot engage: a host posts one all-joints engage before the
// loop starts. The first cycle consumes it: one BRAKE_ENGAGE and no wire
// command (cold drives already prove closure, so every axis settles early),
// and nothing follows.
TEST_F(BrakeGoldenTraceTest, EngagePostedBeforeTheFirstCycleEmitsOneBrakeEngageThenNothing) {
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    controller_->request_brake_engage();
    feed_all(kStNotReady);
    run_cycle();
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::BRAKE_ENGAGE), 1)
        << "the posted engage is consumed on the first cycle";
    for (int cycle = 0; cycle < 5; ++cycle) {
        run_cycle();
    }
    EXPECT_TRUE(drain_events(ctx).empty()) << "one post, one engage — nothing after";
    EXPECT_EQ(controller_->brake_operation_pending(), 0x00);
    EXPECT_EQ(controller_->brake_engaged(), 0x3F);
    for (uint32_t j = 0; j < 6; ++j) {
        EXPECT_EQ(controlword(j), 0x0000) << "joint " << j;
    }
}

// Torque-gate publication: a higher-layer host's control thread commands
// torque only while motion_enabled and all_drives_enabled both hold, so the
// sequencing plane publishes the settled-mask picture every cycle. False at
// boot (all engaged); still false after a partial release settles: commanding
// there integrates against the held joints and whichever drive enables next
// eats the pegged command; true only once the last joint settles released;
// false again the cycle an engage settles.
TEST_F(BrakeGoldenTraceTest, PublishesAllDrivesEnabledOnlyWithEveryDriveOperational) {
    arm_motion();
    step(kStNotReady);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire))
        << "boot parks disabled — the torque gate must publish closed";

    // Joint 0 walks its real sequence to OPERATION_ENABLED; five stay parked.
    // The brake mask may already read the whole group released here (a shared
    // brake settles on its first enabled axis), so the gate must key on drive
    // enablement, not the mask.
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    walk_release_sequence_and_pin();
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire))
        << "PARTIAL enablement (1 of 6 drives) must keep the torque gate closed";

    // Every drive reads OPERATION_ENABLED; the publish must open the gate.
    feed_all_and_step(kStOpEnabled);
    EXPECT_TRUE(flags_->all_drives_enabled.load(std::memory_order_acquire))
        << "all drives operational must open the torque gate";

    // Any drive leaving OPERATION_ENABLED closes it on the very next cycle.
    feed_all_and_step(kStOpEnabled, /*except_joint=*/3, kStSwitchedOn);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire))
        << "one drive off its operational step must close the torque gate";

    // An unrecognized word closes it too: the decoder
    // holds the previous state on an unknown pattern, and a held
    // OPERATION_ENABLED is not evidence: a fault word outside the table
    // (e.g. 0x0028) would hide in exactly that hold. A future fault variant outside
    // the decode table must fail closed, not keep torque flowing.
    feed_all_and_step(kStOpEnabled);
    ASSERT_TRUE(flags_->all_drives_enabled.load(std::memory_order_acquire));
    feed_all_and_step(kStOpEnabled, /*except_joint=*/1, kStUnrecognized);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire))
        << "a held state behind an unrecognized word must close the torque gate";
    feed_all_and_step(kStOpEnabled);
    EXPECT_TRUE(flags_->all_drives_enabled.load(std::memory_order_acquire))
        << "a recognized operational word must reopen it";
}

TEST_F(BrakeGoldenTraceTest, RequestedReleaseDiscardedWithoutMotionEnabled) {
    // Consume-time authority, request-mask leg: a release that
    // reaches the RT consume while motion_enabled is false (cold BRAKED, a
    // racing set_mode(BRAKED), a safety-forced BRAKED commit) starts no
    // sequence and is discarded, not parked; a stale release must never fire
    // on the next motion entry.
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000) << "no sequence may start without a motion mode";
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01) << "still engaged";
    step(kStNotReady);
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "discarded — not parked for later";
    ASSERT_EQ(step(kStNotReady).first, 0x0000) << "and it must STAY discarded";
}

TEST_F(BrakeGoldenTraceTest, StaleFlagsCannotResurrectReleaseAgainstCommittedBraked) {
    // The stale-hardware-action interleave, pinned: a suspended
    // set_mode(FREEDRIVE) thread may overwrite the derived flags to true
    // after a safety forcing committed BRAKED. The consume's authority is
    // the committed word (probe), so the stale flags admit nothing.
    static std::atomic<bool> committed_motion{false};
    committed_motion.store(false);
    controller_->set_committed_mode_probe(
        +[](const void* p) noexcept {
            return plem::fieldbus::ethercat::CommittedModeSample{static_cast<const std::atomic<bool>*>(p)->load(),
                                                                 false};
        },
        &committed_motion);
    arm_motion();  // the stale derived flag says motion; the word says BRAKED

    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000) << "the committed word must win over stale flags";
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01);
}

TEST_F(BrakeGoldenTraceTest, CommittedWordAdmitsReleasePostedBeforeFlagStore) {
    // The motion-entry ordering, pinned: the mode CAS commits
    // before the hardware action posts the release, so a consume landing
    // between the post and the (later) derived-flag store must admit the
    // release: the word is already motion. Here: word=motion, flags still
    // false (the mid-window state).
    static std::atomic<bool> committed_motion{true};
    committed_motion.store(true);
    controller_->set_committed_mode_probe(
        +[](const void* p) noexcept {
            return plem::fieldbus::ethercat::CommittedModeSample{static_cast<const std::atomic<bool>*>(p)->load(),
                                                                 false};
        },
        &committed_motion);

    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0006) << "the committed word must admit — no silent discard";
}

TEST_F(BrakeGoldenTraceTest, StaleFreedriveFlagCannotUnlockProtectiveStopReleaseAgainstCommittedTrajectory) {
    // The recovery-lane bit shares the single-snapshot authority:
    // the committed word says TRAJECTORY (motion, not recovery) while a stale
    // ControlModeFlags::freedrive_enabled=true lingers. Under an active
    // P-STOP the recovery lane opens only for a committed FREEDRIVE; the
    // stale flag must not unlock it.
    static constexpr plem::fieldbus::ethercat::CommittedModeSample kTrajectory{true, false};
    controller_->set_committed_mode_probe(+[](const void*) noexcept { return kTrajectory; }, nullptr);
    auto flags = std::make_shared<plem::core::ControlModeFlags>();
    flags->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    flags->freedrive_enabled.store(true, std::memory_order_release);  // stale
    controller_->set_mode_flags(flags);
    step(kStNotReady);  // P-STOP edge consumed (forced engage settles at rest)
    step(kStNotReady);

    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000)
        << "committed TRAJECTORY under P-STOP: no recovery release — the stale freedrive flag admits nothing";
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01);
}

TEST_F(BrakeGoldenTraceTest, CommittedFreedriveOpensProtectiveStopRecoveryDespiteStaleFalseFlag) {
    // The recovery-lane mirror (legitimate BRAKED→FREEDRIVE recovery under P-STOP):
    // the CAS committed the recovery lane ({P_STOP, FREEDRIVE} projected)
    // before the hardware action posted the release, so the consume
    // must admit even while the derived freedrive_enabled flag is still
    // false (the mid-window state).
    static constexpr plem::fieldbus::ethercat::CommittedModeSample kFreedrive{true, true};
    controller_->set_committed_mode_probe(+[](const void*) noexcept { return kFreedrive; }, nullptr);
    auto flags = std::make_shared<plem::core::ControlModeFlags>();
    flags->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);  // freedrive flag still false
    controller_->set_mode_flags(flags);
    step(kStNotReady);  // P-STOP edge consumed
    step(kStNotReady);

    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0006)
        << "committed FREEDRIVE opens the recovery lane — no silent discard against the stale false flag";
}

// Scenario 2: repeated release

TEST_F(BrakeGoldenTraceTest, RepeatedReleaseRequeuesSequence) {
    controller_->set_brake_release_sequence(0x01);
    walk_release_sequence_and_pin();

    // The sequence path is unfiltered: a second release re-queues it, and from
    // OPERATION_ENABLED the first step command is the down-transition word
    // 0x0006 (OP -> READY exists in the table).
    controller_->set_brake_release_sequence(0x01);
    const auto [cw, mode] = step(kStOpEnabled);
    EXPECT_EQ(cw, 0x0006);
    EXPECT_EQ(mode, 0x0A);
}

// Scenario 3: mid-sequence fault

TEST_F(BrakeGoldenTraceTest, MidSequenceFaultWithoutDeclaredGroupsStalls) {
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);

    // Drive faults mid-sequence. Without a declared policy the sequence stalls: no fault reset is
    // ever issued (0x0080 must never appear), the last word simply holds.
    // 250 cycles cross the 200-cycle reset interval of the bounded traces
    // to prove this path did not inherit it.
    for (int cycle = 0; cycle < 250; ++cycle) {
        SCOPED_TRACE("fault cycle " + std::to_string(cycle));
        const auto [cw, mode] = step(kStFault);
        ASSERT_EQ(cw, 0x0007);
        ASSERT_EQ(mode, 0x0A);
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01) << "stalled release: still reported engaged";
}

TEST_F(BrakeGoldenTraceTest, DeclaredGroupsAloneNeverResetAMidSequenceFault) {
    // A brake-group declaration (set_brake_setup) says which joints share a
    // brake, a wiring fact. Whether
    // the sequence resets a drive that faults mid-transition is a separate
    // model-declared policy (FaultResetPolicy). Declaring the first must not
    // switch on the second: with groups only, a faulted joint's sequence
    // stalls exactly as the undeclared default does.
    controller_->set_brake_setup({0x01, 0x02, 0x04, 0x08, 0x10, 0x20});  // singleton closure, no policy
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);

    for (int cycle = 0; cycle < 450; ++cycle) {  // past two reset intervals
        SCOPED_TRACE("fault cycle " + std::to_string(cycle));
        ASSERT_EQ(step(kStFault).first, 0x0007) << "no controlword-bit-7 reset without a declared policy";
    }
}

TEST_F(BrakeGoldenTraceTest, MidSequenceFaultWithBoundedAutoResetPolicyResetsAndResumes) {
    // A model declaring FaultResetPolicy::BOUNDED_AUTO_RESET gets the bounded
    // auto-reset on the native mode sequence (rate-limited, one attempt per
    // interval). The policy alone is enough: no brake groups are declared here.
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);

    // The reset diagnostic is a PLEM_EMIT event (hot loop, no logger):
    // capture it with a thread-local monitoring context.
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    for (int cycle = 0; cycle < 199; ++cycle) {
        SCOPED_TRACE("pre-reset fault cycle " + std::to_string(cycle));
        ASSERT_EQ(step(kStFault).first, 0x0007);
    }
    EXPECT_EQ(step(kStFault).first, 0x0080) << "reset attempt 1 exactly at the 200th fault cycle";

    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    int reset_events = 0;
    while (auto* event = queue->front()) {
        if (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET) {
            EXPECT_EQ(event->joint_id, 0);
            EXPECT_EQ(event->error_code, 0x0228);  // the observed statusword
            EXPECT_FLOAT_EQ(event->value, 1.0f);   // attempt #
            ++reset_events;
        }
        queue->pop();
    }
    EXPECT_EQ(reset_events, 1) << "exactly one reset event per issued reset";
    plem::core::monitoring::set_thread_context(nullptr);

    // When the fault clears, the sequence resumes at the same head, and attempt
    // bookkeeping resets on the first healthy step. The frame after the pulse
    // publishes its owed falling edge, erasing the step command selected that
    // cycle; the following frame reissues the step. The owed low therefore
    // delays the wire re-command by one frame. The head has not moved and the
    // direct row still exists.
    EXPECT_EQ(step(kStReady).first, 0x0000) << "falling edge outranks the re-commanded step";
    EXPECT_EQ(step(kStReady).first, 0x0007) << "and the step is re-issued on the next frame";
    EXPECT_EQ(step(kStSwitchedOn).first, 0x0007);
    EXPECT_EQ(step(kStSwitchedOn).first, 0x000F);
    EXPECT_EQ(step(kStOpEnabled).first, 0x000F);
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x00);
}

// The fault-reset pulse's named residuals
// The frame after a pulse is a forced low, and that low erases whatever the
// decider produced on the same cycle. Whether the walk survives depends on
// where its head was and what the drive answered: six shapes, of which only
// one retries by itself. The rest park the axis in SWITCH_ON_DISABLED under
// Disable voltage (the safe direction) and stop; recovery is a new
// request-mask release edge from above, never an automatic resume. These are
// characterization pins, not approvals, and the rows differ in whether they
// can be observed at all.
//
// Drive answer after the pulse, by head position:
//   Row 1  SWITCH_ON_DISABLED, head READY_TO_SWITCH_ON   -> re-issued after the low
//   Row 2  SWITCH_ON_DISABLED, head past READY           -> holds until READY is offered
//   Row 3  arrives at the head on the low frame          -> permanent hold, reports stalled
//   Row 4  pulse on the terminal step                    -> sequence gone, enable loss reported
//   Row 5  engage settles early on proven-closed         -> correct quiet end
//   Row 6  selected command erased, then SWITCH_ON_DISABLED -> permanent hold, reports stalled

// Row 1, the one supported recovery: the drive is back at SWITCH_ON_DISABLED
// and the head still declares a direct row to it. The low erases the command
// and the very next cycle re-issues it, because nothing about the decision
// changed.
TEST_F(BrakeGoldenTraceTest, Row1SodWithReadyHeadReissuesTheCommandAfterTheLow) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006) << "head = READY_TO_SWITCH_ON";

    hold_fault_until_pulse(0x0006);

    EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "SOD -> READY is selected, then erased by the owed low";
    EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0006) << "and re-issued: same head, same row, same decision";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x01);
}

// Row 2: same drive answer, head one step further on. There is no
// SWITCH_ON_DISABLED -> SWITCHED_ON row, so the walk simply holds. It is not
// stuck forever by construction: a later READY_TO_SWITCH_ON sample commands 7
// for a SWITCHED_ON head (and would command F for an OPERATION_ENABLED one).
TEST_F(BrakeGoldenTraceTest, Row2SodWithSwitchedOnHeadHoldsUntilTheDriveOffersReady) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006) << "arrival: head = SWITCHED_ON";

    hold_fault_until_pulse(0x0006);

    EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "owed low";
    for (int cycle = 0; cycle < 5; ++cycle) {
        EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "no SOD -> SWITCHED_ON row: HOLD";
    }
    EXPECT_EQ(step(kStReady).first, 0x0007) << "a READY sample IS commandable for this head";
}

// Row 2 again with the head one step further still: the same hold, but the
// escape command is F, not 7. The two heads are different rows of the matrix
// and a single test would hide whichever one regressed.
TEST_F(BrakeGoldenTraceTest, Row2SodWithOperationEnabledHeadHoldsUntilTheDriveOffersReady) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);
    ASSERT_EQ(step(kStSwitchedOn).first, 0x0007) << "arrival: head = OPERATION_ENABLED";

    hold_fault_until_pulse(0x0007);

    EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "owed low";
    for (int cycle = 0; cycle < 5; ++cycle) {
        EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "no SOD -> OPERATION_ENABLED row: HOLD";
    }
    EXPECT_EQ(step(kStReady).first, 0x000F) << "a READY sample IS commandable for this head — via RTSO -> OE";
}

// Row 3: the drive arrives at the head's own state on the pulse's low frame.
// Arrival outranks command, so the head advances without issuing anything, and
// the drive then follows Disable voltage down to SWITCH_ON_DISABLED, for
// which the new head declares no row. Permanent hold.
TEST_F(BrakeGoldenTraceTest, Row3ArrivalOnTheLowFrameAdvancesTheHeadIntoAPermanentHold) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006) << "head = READY_TO_SWITCH_ON — NOT yet consumed";

    hold_fault_until_pulse(0x0006);

    EXPECT_EQ(step(kStReady).first, 0x0000) << "ADVANCE issues nothing; the owed low is what reaches the wire";
    for (int cycle = 0; cycle < 5; ++cycle) {
        EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "new head is SWITCHED_ON — no row from SOD";
    }
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x01) << "observable: still busy";
}

// Row 4: the pulse lands on the last step. The terminal ADVANCE empties the
// sequence, so there is nothing left to retry and busy goes to 0. A released
// axis dropping to SWITCH_ON_DISABLED is an enable loss and announces
// itself (see below).
TEST_F(BrakeGoldenTraceTest, Row4PulseOnTheTerminalStepLeavesNothingToRetryAndNoBusyBit) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);
    ASSERT_EQ(step(kStSwitchedOn).first, 0x0007);
    ASSERT_EQ(step(kStSwitchedOn).first, 0x000F) << "head = OPERATION_ENABLED, the last step";

    hold_fault_until_pulse(0x000F);

    EXPECT_EQ(step(kStOpEnabled).first, 0x0000) << "terminal ADVANCE, then the owed low";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "busy is CLEAR — the sequence is gone";

    // The residual is reported: the axis
    // completed a release, its group reads released, and the next recognized
    // word is SWITCH_ON_DISABLED, an enable loss. It is named once, the
    // EXTERNAL P-STOP is asserted and every brake is force-engaged. Joint 0's
    // forced engage settles early on its proven-closed word (no command is
    // ever re-issued; 0x000F stays gone); the five axes this fixture never
    // initialized stall on theirs, which is the silent-stall diagnostic doing
    // its own job, not this residual's.
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    controller_->set_sequence_stall_threshold_for_test(20);
    for (int cycle = 0; cycle < 40; ++cycle) {
        EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "0x000F is never re-issued: the sequence is gone";
    }
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "joint 0's forced engage settled on its SOD";
    const auto events = drain_events(ctx);
    plem::core::monitoring::set_thread_context(nullptr);
    const auto lost = events_of(events, EventType::OPERATION_ENABLED_LOST);
    ASSERT_EQ(lost.size(), 1u) << "the residual is reported exactly once";
    EXPECT_EQ(lost[0].joint_id, 0);
    EXPECT_EQ(lost[0].error_code, kStSwitchOnDisabled);
    EXPECT_EQ(count_of(events, plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01) << "joint 0 settled engaged on its proven-closed word";
    uint8_t stalled_axes = 0;
    for (const auto& event : events) {
        if (event.type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED) {
            stalled_axes |= static_cast<uint8_t>(uint8_t{1} << event.joint_id);
            EXPECT_EQ(event.error_code, 0) << "a never-initialized axis reports statusword 0";
        }
    }
    EXPECT_EQ(stalled_axes, 0x3E) << "exactly the five never-initialized axes stall on the forced engage";
}

// Row 5: an engage walking into the conservative early settle. Also terminal,
// also busy=0, but this one is a correct ending rather than a lost recovery:
// the axis reached proven-closed evidence and had nothing left to command.
TEST_F(BrakeGoldenTraceTest, Row5PulseThenEarlySettleEndsTheEngageWithNoBusyBit) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_state_for_test(0x00);
    controller_->set_brake_engage_sequence(0x01);
    // Walk to the brief's head (READY_TO_SWITCH_ON, the engage's middle step)
    // so this is the matrix row and not a restatement of the first one.
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007) << "command SWITCHED_ON";
    ASSERT_EQ(step(kStSwitchedOn).first, 0x0007) << "arrival: head = READY_TO_SWITCH_ON";

    hold_fault_until_pulse(0x0007);

    EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "owed low; the sample is already proven-closed";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "settled early — the engage is DONE";
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01) << "and recorded as engaged";

    // Silence is measured from after the settle: the stall check runs before
    // the action, so a walk that had been stuck can still report on the cycle
    // it finishes. A terminated sequence reports nothing past a full stall
    // window, so rows 4 and 5 end silently while rows 2/3/6 report themselves.
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    controller_->set_sequence_stall_threshold_for_test(20);
    for (int cycle = 0; cycle < 40; ++cycle) {
        EXPECT_EQ(step(kStSwitchOnDisabled).first, 0x0000);
    }
    int stalled = 0;
    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    while (auto* event = queue->front()) {
        stalled += (event->type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED) ? 1 : 0;
        queue->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);
    EXPECT_EQ(stalled, 0);
}

// Row 6: the head has not moved and a direct row exists, yet the walk still
// stops: the low erased the command that was selected on that very cycle, and
// the drive answered it by dropping to SWITCH_ON_DISABLED, where this head has
// no row. This is how an axis enters row 2's hold. The busy bit is only half
// the observability story: the stalled walk also keeps reporting. Rows 3 and 6
// are therefore discoverable without reading controlwords at all, unlike
// rows 4 and 5, which end quietly.
TEST_F(BrakeGoldenTraceTest, Row6SelectedCommandErasedByTheLowThenSodLeavesAPermanentHoldThatKeepsReportingStalled) {
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    // Threshold placed past the fault hold on purpose: the stall we want to
    // observe is the one the permanent hold produces, not the one the fault
    // itself would. The episode latch reports once, so the two must not be
    // allowed to overlap or the pin would prove nothing about the hold.
    controller_->set_sequence_stall_threshold_for_test(250);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006) << "arrival resets the stall count; it restarts at the fault";
    hold_fault_until_pulse(0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0000)
        << "READY -> SWITCHED_ON (7) is selected and then erased; stall count is 201 here — below the threshold";
    for (int cycle = 0; cycle < 60; ++cycle) {
        ASSERT_EQ(step(kStSwitchOnDisabled).first, 0x0000) << "the drive followed the low down; no row from SOD";
        if (cycle == 4) {
            EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x01) << "observable: still busy";
        }
    }

    int stalled = 0;
    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    while (auto* event = queue->front()) {
        stalled += (event->type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED) ? 1 : 0;
        queue->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);
    EXPECT_EQ(stalled, 1) << "a walk that cannot progress reports it — exactly once per episode";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x01);
}

// An unrecognized statusword makes the state machine hold the previous state.
// Nothing may be spent on a held FAULT: no attempt, no event, no pulse. This
// is what keeps the allowance from burning down before the first real pulse.
TEST_F(BrakeGoldenTraceTest, HeldFaultAcrossUnrecognizedFramesSpendsNoAttemptAndPublishesNothing) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);

    plem::fieldbus::test::ScopedMonitoringContext ctx;

    ASSERT_EQ(step(kStFault).first, 0x0006) << "a recognized FAULT, so the state machine now HOLDS it";
    for (int cycle = 0; cycle < 600; ++cycle) {  // three reset intervals
        SCOPED_TRACE("unrecognized cycle " + std::to_string(cycle));
        ASSERT_EQ(step(kStUnrecognized).first, 0x0006) << "held FAULT is not live evidence — never a pulse";
    }

    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    int reset_events = 0, exhausted_events = 0;
    while (auto* event = queue->front()) {
        reset_events += (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET) ? 1 : 0;
        exhausted_events += (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET_EXHAUSTED) ? 1 : 0;
        queue->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);
    EXPECT_EQ(reset_events, 0);
    EXPECT_EQ(exhausted_events, 0) << "the allowance must still be intact when a readable FAULT arrives";

    // And it is, on an exactly known cycle. The interval counter did keep
    // running and was consumed at each boundary (199, 399, 599 of the
    // blackout), leaving it at 1 after the 600th. So the pulse is due 199
    // returning-FAULT cycles later, and the attempt it spends is the first.
    plem::fieldbus::test::ScopedMonitoringContext ctx2;
    for (int cycle = 0; cycle < 198; ++cycle) {
        SCOPED_TRACE("returning-fault cycle " + std::to_string(cycle));
        ASSERT_EQ(step(kStFault).first, 0x0006);
    }
    EXPECT_EQ(step(kStFault).first, 0x0080) << "the consumed-interval phase is deterministic, not 'within 200'";

    int first_attempt = -1;
    auto* q2 = ctx2.queue<plem::core::monitoring::Event>();
    ASSERT_NE(q2, nullptr);
    while (auto* event = q2->front()) {
        if (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET && first_attempt < 0) {
            first_attempt = static_cast<int>(event->value);
        }
        q2->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);
    EXPECT_EQ(first_attempt, 1) << "the blackout spent NO allowance — this is attempt #1, not #4";
}

// A drop is terminal: the operator who saw "requested" has no other way to
// learn the axis threw the request away and must ask again. So every terminal
// outcome is reported per axis, with the reason: a recognized non-FAULT axis
// is worth re-requesting once it settles, while an undecodable statusword
// says nothing about the axis at all.
TEST_F(BrakeGoldenTraceTest, EveryTerminalFaultResetOutcomeIsReportedPerAxis) {
    using plem::fieldbus::ethercat::FaultResetPublication;
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    auto& servo = *bus_->servo_devices()[0];
    ASSERT_EQ(step(kStFault).first, 0x0000);

    // published → owed low (no outcome) → dropped (reacting) → dropped (unknown)
    servo.request_fault_reset();
    ASSERT_EQ(step(kStFault).first, 0x0080);
    ASSERT_EQ(step(kStFault).first, 0x0000);
    servo.request_fault_reset();
    ASSERT_EQ(step(kStFaultReactionActive).first, 0x0000);
    servo.request_fault_reset();
    ASSERT_EQ(step(kStUnrecognized).first, 0x0000);

    using plem::core::monitoring::Severity;
    std::vector<int> outcomes;
    std::vector<int> severities;
    std::vector<uint16_t> statuswords;
    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    while (auto* event = queue->front()) {
        if (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET_RESULT) {
            EXPECT_EQ(event->joint_id, 0) << "the axis is named — a bus-wide count would not identify it";
            outcomes.push_back(static_cast<int>(event->value));
            severities.push_back(event->severity);
            statuswords.push_back(event->error_code);
        }
        queue->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);

    const std::vector<int> expected = {
        static_cast<int>(FaultResetPublication::PUBLISHED),
        static_cast<int>(FaultResetPublication::DROPPED_NOT_IN_FAULT),
        static_cast<int>(FaultResetPublication::DROPPED_UNRECOGNIZED),
    };
    EXPECT_EQ(outcomes, expected) << "one report per terminal outcome, and silence on the frames that had none";
    // A reset of a healthy axis is routine (a recovery resets every axis), not
    // an error; an undecodable statusword says nothing about the axis and stays one.
    const std::vector<int> expected_severities = {
        static_cast<int>(Severity::INFO),
        static_cast<int>(Severity::INFO),
        static_cast<int>(Severity::ERROR),
    };
    EXPECT_EQ(severities, expected_severities);
    const std::vector<uint16_t> deciding = {kStFault, kStFaultReactionActive, kStUnrecognized};
    EXPECT_EQ(statuswords, deciding) << "error_code = the statusword that decided each outcome";
}

// The pulse is per axis. A partner that is not resetting keeps its own
// standing command; the inactive baseline is never a bus-wide HOLD.
TEST_F(BrakeGoldenTraceTest, APulseOnOneAxisLeavesAnotherAxisStandingCommandAlone) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x03);  // joints 0 and 1
    feed_status(1, kStNotReady);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(controlword(1), 0x0006) << "joint 1 is walking too";

    // Joint 0 faults; joint 1 stays healthy and holds at READY_TO_SWITCH_ON.
    feed_status(1, kStReady);
    for (int cycle = 0; cycle < 199; ++cycle) {
        ASSERT_EQ(step(kStFault).first, 0x0006);
    }
    ASSERT_EQ(step(kStFault).first, 0x0080) << "joint 0 pulses";
    EXPECT_EQ(controlword(1), 0x0007) << "joint 1 kept walking — untouched by joint 0's pulse";

    EXPECT_EQ(step(kStFault).first, 0x0000) << "joint 0's owed falling edge";
    EXPECT_NE(controlword(1), 0x0000) << "and the low is joint 0's alone, not a bus-wide baseline";
}

// A manual request needs no sequence at all: it is published by the same
// write() as everything else, and is followed by the same falling edge.
TEST_F(BrakeGoldenTraceTest, ManualRequestPublishesOnAnAxisWithNoSequence) {
    ASSERT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "no sequence is running";

    ASSERT_EQ(step(kStFault).first, 0x0000);
    bus_->reset_all_faults();
    EXPECT_EQ(step(kStFault).first, 0x0080);
    EXPECT_EQ(step(kStFault).first, 0x0000) << "the falling edge does not depend on a sequence either";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "and no sequence was created by the request";
}

TEST_F(BrakeGoldenTraceTest, FaultResetGivesUpAfterCapAndEmitsExhaustedOnce) {
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);

    plem::fieldbus::test::ScopedMonitoringContext ctx;

    // Permanent fault across 27 reset intervals: exactly 25 accepted requests
    // (one per 200 fault cycles), then the cap stops the asking. EXHAUSTED is
    // emitted exactly once, on the interval after the last one.
    //
    // Record the wire, not just the events: acceptance and publication are
    // different things, and a test that counts only events still passes with
    // the pulse deleted.
    std::vector<uint16_t> words;
    words.reserve(27 * 200);
    for (int interval = 0; interval < 27; ++interval) {
        for (int cycle = 0; cycle < 200; ++cycle) {
            words.push_back(step(kStFault).first);
        }
    }

    // Before the first pulse the walk's standing command stands; the pulse's
    // own falling edge erases it, and the fault branch never re-commands, so
    // from then on the axis carries Disable voltage between pulses, for as long
    // as the fault lasts.
    std::vector<size_t> highs;
    for (size_t i = 0; i < words.size(); ++i) {
        if (words[i] == 0x0080) {
            highs.push_back(i);
            ASSERT_LT(i + 1, words.size());
            EXPECT_EQ(words[i + 1], 0x0000) << "every pulse owes a falling edge — index " << i;
        } else {
            EXPECT_EQ(words[i], i < 199 ? 0x0007 : 0x0000) << "index " << i;
        }
    }
    ASSERT_EQ(highs.size(), 25u) << "one rising edge per accepted request, all the way to the cap";
    for (size_t k = 0; k < highs.size(); ++k) {
        EXPECT_EQ(highs[k], 199 + 200 * k) << "pulses land one per interval, exactly — pulse " << k;
    }
    EXPECT_LT(highs.back(), 25 * 200) << "nothing is published after the allowance is spent";

    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    int reset_events = 0, exhausted_events = 0;
    while (auto* event = queue->front()) {
        reset_events += (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET) ? 1 : 0;
        exhausted_events += (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET_EXHAUSTED) ? 1 : 0;
        queue->pop();
    }
    EXPECT_EQ(reset_events, 25);
    EXPECT_EQ(exhausted_events, 1) << "the give-up must be reported exactly once";
}

// Scenario 4: mode-path engage is the native disable sequence

TEST_F(BrakeGoldenTraceTest, EngageRunsDisableSequenceAndFlipsState) {
    controller_->set_brake_release_sequence(0x01);
    walk_release_sequence_and_pin();
    ASSERT_EQ(controller_->brake_engaged() & 0x01, 0x00);

    // Engage = the disable sequence; power off = brake closed. The vendor
    // mechanism is never consulted, and the sequence's own completion is the
    // authority for "engaged" (the bench-verified power coupling).
    controller_->set_brake_engage_sequence(0x01);
    uint16_t st = kStOpEnabled;
    for (int cycle = 0; cycle < 20 && (controller_->brake_engaged() & 0x01) == 0; ++cycle) {
        const auto [cw, mode] = step(st);
        EXPECT_EQ(mode, 0x0A);
        switch (cw) {  // compliant-drive model: the status follows the command
            case 0x0006:
                st = kStReady;
                break;
            case 0x0007:
                st = kStSwitchedOn;
                break;
            case 0x000F:
                st = kStOpEnabled;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01) << "disable-sequence completion must report engaged";
}

TEST_F(BrakeGoldenTraceTest, SameCycleReleaseThenEngageLastDirectionWins) {
    // Two mode requests in one cycle: the later sequence overwrites the
    // earlier one's sequence: exactly one direction acts (the last), and from
    // a resting drive the conservative engage fallback lands immediately.
    controller_->set_brake_release_sequence(0x01);
    controller_->set_brake_engage_sequence(0x01);
    step(kStNotReady);
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01) << "the engage (last direction) must win";
}

// Scenario 5: whole-group closure on the native path

TEST_F(BrakeGoldenTraceTest, SingleJointReleaseExpandsToWholeBrakeGroup) {
    // One module brake serves two joints: a single-joint release
    // must power the whole group (a partial mask would open the
    // gravity-loaded partner without powering its drive).
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});

    controller_->set_brake_release_sequence(0x01);  // joint 0 only
    EXPECT_EQ(controller_->brake_operation_pending() & 0x03, 0x03) << "the request expands to the full module group";
    const auto [cw0, cw1] = step_pair(kStNotReady, kStNotReady);
    EXPECT_EQ(cw0, 0x0006);
    EXPECT_EQ(cw1, 0x0006) << "the partner joint's sequence must start too";
}

TEST_F(BrakeGoldenTraceTest, SingleJointEngageExpandsToWholeBrakeGroup) {
    // Engage direction of the same physics: disabling one axis leaves the
    // module brake held open by its partner, so that joint ends unpowered and
    // unbraked. The engage must cover the whole group.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});

    controller_->set_brake_engage_sequence(0x01);  // joint 0 only
    EXPECT_EQ(controller_->brake_operation_pending() & 0x03, 0x03) << "the engage expands to the full module group";
    step_pair(kStNotReady, kStNotReady);  // conservative fallback from rest
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x03) << "both joints of the module report engaged";
}

// Scenario 6: the direct vendor-brake op (recovery surface)

TEST_F(BrakeGoldenTraceTest, EngageCompletionDoesNotFlipWhilePartnerStillPowered) {
    // The false-engaged direction: joint 0's disable sequence
    // completes while its module partner is still OPERATION_ENABLED. The
    // shared brake is physically held open by the partner, so joint 0 is
    // neither braked nor powered; its bit must not read engaged until the
    // partner quiets, and then the whole group flips together.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    controller_->set_brake_state_for_test(0x3C);  // module 0 open (both joints)

    controller_->set_brake_engage_sequence(0x03);
    // Joint 0 walks the disable sequence down while the partner stays loud.
    uint16_t st0 = kStOpEnabled;
    for (int cycle = 0; cycle < 20; ++cycle) {
        const auto [cw0, cw1] = step_pair(st0, kStOpEnabled);
        switch (cw0) {
            case 0x0006:
                st0 = kStReady;
                break;
            case 0x0007:
                st0 = kStSwitchedOn;
                break;
            case 0x000F:
                st0 = kStOpEnabled;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00)
        << "partner still powers the shared brake open — NOTHING may read engaged";

    // Partner quiets and walks down; its settlement flips the whole group.
    uint16_t st1 = kStOpEnabled;
    for (int cycle = 0; cycle < 20 && (controller_->brake_engaged() & 0x03) != 0x03; ++cycle) {
        const auto [cw0, cw1] = step_pair(st0, st1);
        switch (cw1) {
            case 0x0006:
                st1 = kStReady;
                break;
            case 0x0007:
                st1 = kStSwitchedOn;
                break;
            case 0x000F:
                st1 = kStOpEnabled;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x03) << "both axes quiet — the group flips together";
}

TEST_F(BrakeGoldenTraceTest, MixedRestingGroupEngageSettlesWithoutPowerUp) {
    // Fallback before the transition command: a READY drive under an
    // engage must not be stepped up toward SWITCHED_ON (the sequence's first
    // step, a voltage power-up during an engage) while the partner's
    // fallback settles the group on the still-READY reading. Both axes rest
    // in proven-unpowered steps, so the engage settles immediately with
    // zero controlwords on the wire.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    controller_->set_brake_state_for_test(0x3C);  // module 0 open

    controller_->set_brake_engage_sequence(0x03);
    // Partner fed the real bench resting word 0x0060, which pins the CiA 402
    // bit5-don't-care SOD decode (a mis-decode would read
    // unrecognized and block the settlement).
    const auto [cw0, cw1] = step_pair(kStReady, kStSodRestWord);
    EXPECT_EQ(cw0, 0x0000) << "a resting READY drive must not be powered toward SWITCHED_ON during an ENGAGE";
    EXPECT_EQ(cw1, 0x0000);
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x03) << "both axes in proven steps — the group settles at once";
}

TEST_F(BrakeGoldenTraceTest, UnrecognizedPartnerStatuswordBlocksEngageSettlement) {
    // The FSM holds its previous state on an unrecognized
    // statusword, so the held reading is stale, not live evidence: the
    // settlement allowlist must refuse it.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    controller_->set_brake_state_for_test(0x3C);

    controller_->set_brake_engage_sequence(0x03);
    // Joint 0 walks the disable sequence down; the partner emits only an
    // unrecognized word (its held state stays at the whitelisted ctor
    // default, exactly the stale reading the flag exists to expose).
    uint16_t st0 = kStOpEnabled;
    for (int cycle = 0; cycle < 10; ++cycle) {
        const auto [cw0, cw1] = step_pair(st0, kStUnrecognized);
        switch (cw0) {
            case 0x0006:
                st0 = kStReady;
                break;
            case 0x0007:
                st0 = kStSwitchedOn;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00)
        << "an unrecognized partner statusword is a stale reading — no settlement on it";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x02, 0x02)
        << "the unrecognized partner's sequence must stay ALIVE (a consumed request with a refused "
           "settlement would be lost with no retry owner)";

    // The partner produces a recognized proven step: its still-alive
    // fallback settles the group; no re-request needed.
    step_pair(st0, kStSwitchOnDisabled);
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x03)
        << "recognized proven step settles the group through the kept sequence";
}

TEST_F(BrakeGoldenTraceTest, SettlementDuringFaultPartnerDoesNotFlipGroup) {
    // Allowlist discriminator: joint 0 completes its disable sequence
    // while the partner sits in FAULT, and FAULT proves nothing about brake
    // power, so the flip must wait for the partner to reach a proven step.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    controller_->set_brake_state_for_test(0x3C);  // module 0 open

    controller_->set_brake_engage_sequence(0x03);
    uint16_t st0 = kStOpEnabled;
    for (int cycle = 0; cycle < 10; ++cycle) {  // j0 walks down; partner FAULT throughout
        const auto [cw0, cw1] = step_pair(st0, kStFault);
        switch (cw0) {
            case 0x0006:
                st0 = kStReady;
                break;
            case 0x0007:
                st0 = kStSwitchedOn;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00)
        << "a FAULT partner cannot prove the shared brake closed — no flip at joint 0's settlement";

    // Partner recovers and walks down; its own settlement flips the group.
    uint16_t st1 = kStReady;
    for (int cycle = 0; cycle < 20 && (controller_->brake_engaged() & 0x03) != 0x03; ++cycle) {
        const auto [cw0, cw1] = step_pair(st0, st1);
        switch (cw1) {
            case 0x0006:
                st1 = kStReady;
                break;
            case 0x0007:
                st1 = kStSwitchedOn;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x03) << "recovered partner settles the group";
}

TEST_F(BrakeGoldenTraceTest, SettlementDuringSwitchedOnPartnerDoesNotFlipGroup) {
    // Same discriminator, SWITCHED_ON variant: per-joint-brake drive
    // families open the brake as early as SWITCHED_ON and no bench measured
    // it; an unproven state cannot settle the group.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    controller_->set_brake_state_for_test(0x3C);

    controller_->set_brake_engage_sequence(0x03);
    uint16_t st0 = kStOpEnabled;
    for (int cycle = 0; cycle < 10; ++cycle) {  // j0 walks down; partner SWITCHED_ON throughout
        const auto [cw0, cw1] = step_pair(st0, kStSwitchedOn);
        switch (cw0) {
            case 0x0006:
                st0 = kStReady;
                break;
            case 0x0007:
                st0 = kStSwitchedOn;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00)
        << "SWITCHED_ON cannot prove the shared brake closed — no flip at joint 0's settlement";

    uint16_t st1 = kStReady;
    for (int cycle = 0; cycle < 20 && (controller_->brake_engaged() & 0x03) != 0x03; ++cycle) {
        const auto [cw0, cw1] = step_pair(st0, st1);
        switch (cw1) {
            case 0x0006:
                st1 = kStReady;
                break;
            case 0x0007:
                st1 = kStSwitchedOn;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x03) << "the quieted partner settles the group";
}

TEST_F(BrakeGoldenTraceTest, ReleaseCompletionFreesWholeGroupEvenWithPartnerLagging) {
    // The release direction of the same physics: one axis reaching
    // OPERATION_ENABLED opens the shared brake: the partner is unheld from
    // that instant even though its own sequence still climbs, and reporting
    // it engaged would claim a hold that does not exist.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});

    controller_->set_brake_release_sequence(0x01);  // closure 0x03
    // Joint 0 walks up; the partner never leaves NOT_READY (lagging drive).
    uint16_t st0 = kStNotReady;
    for (int cycle = 0; cycle < 20 && (controller_->brake_engaged() & 0x01) != 0x00; ++cycle) {
        const auto [cw0, cw1] = step_pair(st0, kStNotReady);
        switch (cw0) {
            case 0x0006:
                st0 = kStReady;
                break;
            case 0x0007:
                st0 = kStSwitchedOn;
                break;
            case 0x000F:
                st0 = kStOpEnabled;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00)
        << "one enabled axis opens the shared brake — the WHOLE group reads released";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x02, 0x02)
        << "the lagging partner's sequence still reads pending (observers must not treat it settled)";
}

// Scenario 7: shutdown quiesce (native engage)

TEST_F(BrakeGoldenTraceTest, QuiesceEngagesOpenGroupsNatively) {
    // The engage loop needs no actuator: quiesce requests the disable sequence
    // through the normal mailbox and confirms on sequence completion (power
    // off = brake closed).
    controller_->set_brake_state_for_test(0x3E);  // joint 0 open
    controller_->set_quiesce_attempt_timeout_for_test(500'000'000);

    plem::fieldbus::ethercat::BrakeQuiesceOutcome outcome;
    std::atomic<bool> done{false};
    std::thread quiesce([&] {
        outcome = controller_->quiesce_brakes();
        done.store(true, std::memory_order_release);
    });
    uint16_t st = kStOpEnabled;
    for (int cycle = 0; cycle < 1000 && !done.load(std::memory_order_acquire); ++cycle) {
        const auto [cw, mode] = step(st);
        switch (cw) {  // compliant drive follows the disable sequence down
            case 0x0006:
                st = kStReady;
                break;
            case 0x0007:
                st = kStSwitchedOn;
                break;
            case 0x000F:
                st = kStOpEnabled;
                break;
            default:
                break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    quiesce.join();
    EXPECT_TRUE(outcome.fully_engaged()) << "engaged=0x" << std::hex << int(outcome.engaged_mask);
}

TEST_F(BrakeGoldenTraceTest, QuiesceClassifiesUnresponsiveJointAsTimeout) {
    // A drive that never reaches its step is TIMEOUT (unconfirmed, not failed),
    // never reported engaged.
    controller_->set_brake_state_for_test(0x3E);
    controller_->set_quiesce_attempt_timeout_for_test(60'000'000);  // 60 ms per attempt

    plem::fieldbus::ethercat::BrakeQuiesceOutcome outcome;
    std::atomic<bool> done{false};
    std::thread quiesce([&] {
        outcome = controller_->quiesce_brakes();
        done.store(true, std::memory_order_release);
    });
    for (int cycle = 0; cycle < 1000 && !done.load(std::memory_order_acquire); ++cycle) {
        step(kStOpEnabled);  // never complies with the disable sequence
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    quiesce.join();

    EXPECT_EQ(outcome.engaged_mask, 0x3E);
    EXPECT_EQ(outcome.timeout_mask, 0x01) << "unconfirmed is TIMEOUT";
    EXPECT_FALSE(outcome.fully_engaged());
    // The outcome statuswords come from the per-cycle atomic mirror.
    EXPECT_EQ(outcome.statuswords[0], kStOpEnabled);
}

// The release-readiness gate on the decision path
// A release request starts the enable sequence only
// when every joint of its brake-group closure is accepted in its declared
// mode this cycle (initialized, recognized statusword, 0x6061 display ==
// declaration). Otherwise the whole request is refused (consumed, never
// held), and each failing axis is named (MODES_OF_OPERATION_MISMATCH with the
// statusword and display it showed). Readiness recovering never releases by
// itself; the upper layer re-requests. The direct sequence setters (the
// executor seam the traces above use) bypass this gate on purpose.

namespace {

/// Drain the queue and fail on a MODES_OF_OPERATION_MISMATCH (refusal or trip) or
/// a PROTECTIVE_STOP; the informative BRAKE_ENGAGE / BRAKE_RELEASE emissions
/// of a resolution or a start are not what these traces check.
void expect_no_mode_rejection_or_stop(plem::core::monitoring::MonitoringContext& ctx) {
    const auto events = drain_events(ctx);
    EXPECT_EQ(count_of(events, plem::core::monitoring::EventType::MODES_OF_OPERATION_MISMATCH), 0);
    EXPECT_EQ(count_of(events, plem::core::monitoring::EventType::PROTECTIVE_STOP), 0);
}

}  // namespace

TEST_F(BrakeGoldenTraceTest, ReleaseRefusedWhileDisplayMismatchesThenAdmittedOnReRequest) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    // Joint 0 displays CSP (8) while CST is declared: the request is refused
    // and names joint 0 with what it showed.
    feed_display(0, 0x08);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000) << "no sequence may start while the mode is not accepted";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "refused = consumed, not parked";
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u);
    EXPECT_EQ(refusals[0].joint_id, 0);
    EXPECT_EQ(refusals[0].error_code, kStNotReady);
    EXPECT_FLOAT_EQ(refusals[0].value, 8.0f);

    // The display recovers: nothing happens by itself (no request is pending).
    feed_display(0, 0x0A);
    ASSERT_EQ(step(kStNotReady).first, 0x0000) << "readiness recovering must not release on its own";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty());

    // The upper layer re-requests: admitted, the sequence walks.
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0006) << "a fresh request after recovery is the release";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty());
}

// 0x6061 is INTEGER8: a vendor mode −1 (0xFF on the wire) is reported as −1,
// the value RtRawBatch's int8[] shows, never 255.
TEST_F(BrakeGoldenTraceTest, ANegativeVendorModeIsReportedSigned) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0xFF);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000);
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u);
    EXPECT_FLOAT_EQ(refusals[0].value, -1.0f);
}

TEST_F(BrakeGoldenTraceTest, ReadinessNeedsInitializationAndARecognizedWordBesidesTheDisplay) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    // Joint 0 initialized (one nonzero word seen), then statusword 0 with
    // the CST display: 0 decodes to NOT_READY (recognized), so the axis is
    // accepted and the release starts. (A never-initialized axis reading the
    // same 0 is refused (pinned by the bench trace); the payload cannot tell
    // the two apart, which is why the label claims no cause.)
    step(kStNotReady);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(0x0000).first, 0x0006) << "initialized + Not-Ready + CST displayed = accepted";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty());

    // Joint 1 initialized, then a word outside the CiA 402 decode table with
    // the CST display: not recognized, so refused, named with the raw word.
    feed_status(1, kStNotReady);
    run_cycle();
    feed_status(1, kStUnrecognized);
    ASSERT_TRUE(controller_->request_brake_release(0x02));
    run_cycle();
    EXPECT_EQ(controlword(1), 0x0000) << "an unrecognized word is not acceptance, whatever the display says";
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u);
    EXPECT_EQ(refusals[0].joint_id, 1);
    EXPECT_EQ(refusals[0].error_code, kStUnrecognized);
    EXPECT_FLOAT_EQ(refusals[0].value, 10.0f);
}

TEST_F(BrakeGoldenTraceTest, SharedGroupReleaseRefusedWhenThePartnerIsNotAccepted) {
    // Group {0,1}: joint 0 is accepted, joint 1 displays NO_MODE. The request
    // for joint 0 alone is refused as a whole (its closure would power the
    // partner) and the diagnostic names the partner only; the accepted joint
    // is never reported as the cause.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(1, 0x00);

    ASSERT_TRUE(controller_->request_brake_release(0x01));
    const auto [cw0, cw1] = step_pair(kStNotReady, kStNotReady);
    EXPECT_EQ(cw0, 0x0000);
    EXPECT_EQ(cw1, 0x0000);
    EXPECT_EQ(controller_->brake_operation_pending() & 0x03, 0x00);
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u) << "exactly the partner that failed the gate";
    EXPECT_EQ(refusals[0].joint_id, 1);
    EXPECT_EQ(refusals[0].error_code, kStNotReady);
    EXPECT_FLOAT_EQ(refusals[0].value, 0.0f);

    // Partner accepted → the same request is admitted and expands to the group.
    feed_display(1, 0x0A);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    const auto [cw0b, cw1b] = step_pair(kStNotReady, kStNotReady);
    EXPECT_EQ(cw0b, 0x0006);
    EXPECT_EQ(cw1b, 0x0006);
}

// The in-service mode trip and the safety-forced engage
// An enabled axis whose 0x6061 display leaves the declared mode is fault
// class: the torque gate closes, the protective_stop flag (EXTERNAL source) is
// asserted every cycle the condition holds (a level), the
// axis is named once per episode, and the next brake step force-engages every
// brake, even under an already-active P-STOP (recovery freedrive), where no
// rising edge is left. The forced engage keeps an engage already walking on
// any axis; a user re-engage restarts. Each joint's controlword, statusword and mode
// display sit at the offsets the fake domain hands out (pdo_offset()).

TEST_F(BrakeGoldenTraceTest, EnabledAxisLeavingItsModeTripsTheStopOnceAndForcesTheEngage) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_all_and_step(kStOpEnabled);
    ASSERT_TRUE(flags_->all_drives_enabled.load(std::memory_order_acquire));

    // Joint 2 displays CSP (8) while enabled.
    feed_display(2, 0x08);
    feed_all_and_step(kStOpEnabled);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire))
        << "an enabled drive off its declared mode closes the torque gate";
    EXPECT_TRUE(flags_->is_external_protective_stop()) << "the trip asserts the EXTERNAL P-STOP source";
    {
        const auto trips = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(trips.size(), 1u) << "named once";
        EXPECT_EQ(trips[0].joint_id, 2);
        EXPECT_EQ(trips[0].error_code, kStOpEnabled);
        EXPECT_FLOAT_EQ(trips[0].value, 8.0f);
    }

    // Next step: the forced engage commands every joint down (OE -> SWITCHED_ON
    // = 0x0007) and PROTECTIVE_STOP is emitted; the trip persists without a
    // second diagnostic.
    feed_all_and_step(kStOpEnabled);
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0007) << "forced engage on every axis";
    }
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
    feed_all_and_step(kStOpEnabled);
    feed_all_and_step(kStOpEnabled);
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty())
        << "one diagnostic per episode while the trip persists";

    // Level semantics: an optimistic clear of the flags source does not stick
    // while the drive stays off its mode.
    flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    feed_all_and_step(kStOpEnabled);
    EXPECT_TRUE(flags_->is_external_protective_stop()) << "re-latched: the condition still holds";

    // The display flaps back to CST while the drive ignores its disable and
    // stays enabled: with an engage still walking on the axis the episode is
    // not re-armed: no second diagnostic, no second trigger.
    feed_display(2, 0x0A);
    feed_all_and_step(kStOpEnabled);
    feed_display(2, 0x08);
    feed_all_and_step(kStOpEnabled);
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty())
        << "a flapping display under a walking engage is one episode";
}

// Cold POST_ENABLE admission: a drive that reports its mode only
//    after its first enable. Every trace below starts from a cold drive whose
//    0x6061 reads 0 on the first read: the admission the strict traces above
//    refuse (PRE_ENABLE), admitted once and only once under POST_ENABLE.
class ColdPostEnableTraceTest : public BrakeGoldenTraceTest {
  protected:
    void SetUp() override {
        build(six_servos(plem::fieldbus::ModeDisplayTiming::POST_ENABLE), 0x00);
    }
};

TEST_F(ColdPostEnableTraceTest, ColdZeroDisplayIsAdmittedOnceAndTheFirstEnabledZeroTrips) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    // Cold bench: every axis recognized READY, display 0. Admitted: the walk
    // starts at the READY step (already reached: ADVANCE, standing word 0x0000
    // on the clean domain), then commands SWITCHED_ON. No refusal is named.
    feed_all_and_step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_all_and_step(kStReady);
    EXPECT_EQ(controlword(0), 0x0000) << "READY already reached: the step advances, the standing word holds";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty())
        << "a cold POST_ENABLE zero is not a refusal";
    feed_all_and_step(kStReady);
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0007) << "command SWITCHED_ON";
    }
    EXPECT_EQ(EC_READ_U8(pd_ + pdo_offset(0, 0x6060)), 0x0A) << "0x6060 keeps the declaration";

    // SWITCHED_ON with the display still 0: the walking release is not
    // cancelled (a new request would be refused here; pinned separately).
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStSwitchedOn);
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x000F) << "command OPERATION_ENABLED";
    }
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty());

    // First enabled sample still displaying 0 = the in-service trip, on that
    // very cycle (W = 0): torque gate closed, EXTERNAL P-STOP, every axis
    // named once with what it showed.
    feed_all_and_step(kStOpEnabled);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire));
    EXPECT_TRUE(flags_->is_external_protective_stop());
    {
        const auto trips = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(trips.size(), 6u);
        for (const auto& trip : trips) {
            EXPECT_EQ(trip.error_code, kStOpEnabled);
            EXPECT_FLOAT_EQ(trip.value, 0.0f);
        }
    }
    // The next brake step force-engages every axis (OE -> SWITCHED_ON).
    feed_all_and_step(kStOpEnabled);
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0007) << "forced engage on every axis";
    }
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
}

TEST_F(ColdPostEnableTraceTest, AnAxisWithAnOpenTripEpisodeIsNotReAdmittedUnderTheZeroException) {
    // After the cold trip the drives follow the forced engage back to
    // READY, still displaying 0, under the standing P-STOP. A recovery release
    // must be refused: the open episode would otherwise suppress the next
    // forced engage and leave the drives enabled off their mode.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_all_and_step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_all_and_step(kStReady);
    feed_all_and_step(kStReady);
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStOpEnabled);  // trip
    feed_all_and_step(kStOpEnabled);  // forced engage
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStReady);
    feed_all_and_step(kStReady);
    ASSERT_EQ(controller_->brake_engaged(), 0x3F) << "every axis is braked again";
    ASSERT_TRUE(flags_->is_external_protective_stop()) << "the stop level stands";
    (void)drain_events(ctx);

    // The recovery lane (committed {P_STOP, FREEDRIVE}) posts a new release.
    static constexpr plem::fieldbus::ethercat::CommittedModeSample kFreedrive{true, true};
    controller_->set_committed_mode_probe(+[](const void*) noexcept { return kFreedrive; }, nullptr);
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_all_and_step(kStReady);
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0006) << "no release starts — the standing engage word holds";
    }
    EXPECT_EQ(controller_->brake_operation_pending(), 0x00) << "refused = consumed, not parked";
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 6u) << "every axis is named: its episode is open and it never showed its mode";
    for (const auto& refusal : refusals) {
        EXPECT_EQ(refusal.error_code, kStReady);
        EXPECT_FLOAT_EQ(refusal.value, 0.0f);
    }
}

TEST_F(ColdPostEnableTraceTest, TheFirstEnabledSampleShowingTheModeOpensTheTorqueGateWithoutATrip) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_all_and_step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_all_and_step(kStReady);
    feed_all_and_step(kStReady);
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStSwitchedOn);
    // The drives report CST on the same sample that shows OPERATION_ENABLED.
    for (uint32_t j = 0; j < 6; ++j) {
        feed_display(j, 0x0A);
    }
    feed_all_and_step(kStOpEnabled);
    EXPECT_TRUE(flags_->all_drives_enabled.load(std::memory_order_acquire)) << "accepted on the first enabled sample";
    EXPECT_FALSE(flags_->is_external_protective_stop());
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty());
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x000F) << "released";
    }

    // One axis late by a sample is that axis's trip, on its first enabled
    // zero, named alone.
}

TEST_F(ColdPostEnableTraceTest, OneAxisStillZeroOnItsFirstEnabledSampleTripsAlone) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_all_and_step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_all_and_step(kStReady);
    feed_all_and_step(kStReady);
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStSwitchedOn);
    for (uint32_t j = 0; j < 6; ++j) {
        feed_display(j, j == 3 ? 0x00 : 0x0A);
    }
    feed_all_and_step(kStOpEnabled);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire));
    const auto trips = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(trips.size(), 1u);
    EXPECT_EQ(trips[0].joint_id, 3);
    EXPECT_EQ(trips[0].error_code, kStOpEnabled);
    EXPECT_FLOAT_EQ(trips[0].value, 0.0f);
}

TEST_F(ColdPostEnableTraceTest, AModeOnceSeenLatchesTheStrictReadingForTheControllerLifetime) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    // Idle READY showing CST, read by a cycle (no request): the axis has now
    // reported its mode once.
    feed_display(0, 0x0A);
    step(kStReady);
    // Back to 0: from here on 0 is a mismatch, not "not reported yet".
    feed_display(0, 0x00);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(kStReady).first, 0x0000) << "refused";
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u);
    EXPECT_EQ(refusals[0].joint_id, 0);
    EXPECT_EQ(refusals[0].error_code, kStReady);
    EXPECT_FLOAT_EQ(refusals[0].value, 0.0f);

    // Neither an engage (settled early: READY is the software's closure
    // evidence, proves_brake_closed; no word is written) nor a re-request
    // revives the exception.
    controller_->set_brake_engage_sequence(0x01);
    step(kStReady);
    step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(kStReady).first, 0x0000) << "still refused: no release word";
    EXPECT_EQ(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).size(), 1u);
}

TEST_F(ColdPostEnableTraceTest, TheZeroExceptionIsConfinedToTheRestingStatesAndARecognizedInitializedAxis) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    // Never initialized (no word seen yet): refused.
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(0x0000).first, 0x0000);
    ASSERT_EQ(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).size(), 1u);

    // Initialized: SWITCHED_ON, FAULT, FAULT_REACTION_ACTIVE, QUICK_STOP_ACTIVE,
    // an unrecognized word: all refused.
    step(kStReady);
    for (const uint16_t word : {kStSwitchedOn, kStFault, kStFaultReactionActive, kStQuickStopActive, kStUnrecognized}) {
        SCOPED_TRACE(fmt::format("word 0x{:04x}", word));
        ASSERT_TRUE(controller_->request_brake_release(0x01));
        EXPECT_EQ(step(word).first, 0x0000) << "refused";
        EXPECT_EQ(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).size(), 1u);
    }

    // A nonzero mismatch is never "not reported yet".
    feed_display(0, 0x08);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(kStReady).first, 0x0000) << "refused";
    {
        const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(refusals.size(), 1u);
        EXPECT_FLOAT_EQ(refusals[0].value, 8.0f);
    }

    // SWITCH_ON_DISABLED start, display 0: admitted, and the walk commands READY.
    feed_display(0, 0x00);
    step(kStSodRestWord);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(kStSodRestWord).first, 0x0006) << "SOD -> READY_TO_SWITCH_ON";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty());
}

TEST_F(ColdPostEnableTraceTest, AStrictPartnerShowingZeroRefusesTheGroupWhileAnAdmittedZeroPartnerDoesNot) {
    // Mixed group {0, 1}: axis 1 declares PRE_ENABLE (strict), axis 0
    // POST_ENABLE (cold zero admitted). Built here because the declaration is
    // immutable on the device.
    auto servos = six_servos(plem::fieldbus::ModeDisplayTiming::POST_ENABLE);
    servos[1].mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE;
    build(std::move(servos), 0x00);
    controller_->set_brake_setup({0x03, 0x03, 0x04, 0x08, 0x10, 0x20});  // {0,1} shared, the rest singleton
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    step_pair(kStReady, kStReady);

    // Axis 1 (strict) shows 0: the whole request is refused, and only axis 1
    // is named; axis 0's zero is admitted, not a cause.
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    auto words = step_pair(kStReady, kStReady);
    EXPECT_EQ(words.first, 0x0000);
    EXPECT_EQ(words.second, 0x0000);
    {
        const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(refusals.size(), 1u) << "the admitted zero axis is not a refusal cause";
        EXPECT_EQ(refusals[0].joint_id, 1);
        EXPECT_EQ(refusals[0].error_code, kStReady);
        EXPECT_FLOAT_EQ(refusals[0].value, 0.0f);
    }

    // Axis 1 shows CST: the group is admitted (accepted strict partner +
    // admitted cold zero partner); both walk from READY: ADVANCE first.
    feed_display(1, 0x0A);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    step_pair(kStReady, kStReady);
    words = step_pair(kStReady, kStReady);
    EXPECT_EQ(words.first, 0x0007);
    EXPECT_EQ(words.second, 0x0007);
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty());
}

TEST_F(ColdPostEnableTraceTest, NotReadyIsNotARestingStateForTheZeroException) {
    // The zero exception's resting states are {SWITCH_ON_DISABLED, READY_TO_SWITCH_ON}: a
    // recognized NOT_READY showing 0 is refused (the exception is not
    // "anything below OPERATION_ENABLED").
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    step(kStNotReady);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(kStNotReady).first, 0x0000) << "refused";
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u);
    EXPECT_EQ(refusals[0].error_code, kStNotReady);
    EXPECT_FLOAT_EQ(refusals[0].value, 0.0f);
}

TEST_F(ColdPostEnableTraceTest, TheModeSeenInAFaultSampleLatchesTooAndSodZeroIsThenRefused) {
    // The first-match record has no state restriction: a FAULT
    // sample showing CST counts. After the fault clears to SOD with the
    // display back at 0, the zero exception is gone.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0x0A);
    step(kStFault);
    feed_display(0, 0x00);
    step(kStSodRestWord);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(kStSodRestWord).first, 0x0000) << "refused: the mode was shown once, 0 is now a mismatch";
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u);
    EXPECT_EQ(refusals[0].error_code, kStSodRestWord);
    EXPECT_FLOAT_EQ(refusals[0].value, 0.0f);
}

TEST_F(ColdPostEnableTraceTest, OneAxisOpenEpisodeDoesNotSpillOntoAnUntrippedSingletonAxis) {
    // The episode term is per axis. Axis 0 alone is walked to its cold
    // trip; axis 1 never left READY/0, so a fresh request for axis 1 alone is
    // still admitted (singleton groups: no partner closure).
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_all_and_step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    step_pair(kStReady, kStReady);
    step_pair(kStReady, kStReady);
    step_pair(kStSwitchedOn, kStReady);
    step_pair(kStSwitchedOn, kStReady);
    step_pair(kStOpEnabled, kStReady);  // axis 0 trips on its first enabled zero
    ASSERT_EQ(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).size(), 1u);
    step_pair(kStOpEnabled, kStReady);  // forced engage on every axis
    step_pair(kStSwitchedOn, kStReady);
    step_pair(kStSwitchedOn, kStReady);
    step_pair(kStReady, kStReady);
    step_pair(kStReady, kStReady);
    ASSERT_EQ(controller_->brake_engaged(), 0x3F);
    (void)drain_events(ctx);

    static constexpr plem::fieldbus::ethercat::CommittedModeSample kFreedrive{true, true};
    controller_->set_committed_mode_probe(+[](const void*) noexcept { return kFreedrive; }, nullptr);
    // Axis 0 (open episode): refused. Axis 1 (never tripped, still 0): admitted.
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    step_pair(kStReady, kStReady);
    ASSERT_EQ(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).size(), 1u) << "axis 0 refused";
    ASSERT_TRUE(controller_->request_brake_release(0x02));
    step_pair(kStReady, kStReady);
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).empty()) << "axis 1 admitted";
    const auto words = step_pair(kStReady, kStReady);
    EXPECT_EQ(words.second, 0x0007) << "axis 1 walks: command SWITCHED_ON";
    EXPECT_EQ(words.first, 0x0006) << "axis 0 keeps its standing engage word";
}

TEST_F(BrakeGoldenTraceTest, AStrictAxisShowingZeroIsRefusedLikeAnyMismatch) {
    // PRE_ENABLE (every golden trace): 0 on the display is a mismatch, not
    // "not reported yet"; the POST_ENABLE exception never applies here.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0x00);
    step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    EXPECT_EQ(step(kStReady).first, 0x0000) << "refused";
    const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
    ASSERT_EQ(refusals.size(), 1u);
    EXPECT_EQ(refusals[0].joint_id, 0);
    EXPECT_FLOAT_EQ(refusals[0].value, 0.0f);
}

TEST_F(BrakeGoldenTraceTest, UserReEngageRestartsAWalkingEngage) {
    // A user engage walks joint 0 from OE: 0x0007, SWITCHED_ON reached (the
    // reached step is consumed, the word holds), then READY is commanded.
    controller_->set_brake_engage_sequence(0x01);
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007);
    ASSERT_EQ(step(kStSwitchedOn).first, 0x0007);
    // A user re-engage restarts from the first step: the reached
    // step is consumed again, so no READY command this cycle. (The forced
    // engage keeping a walking engage is the ForcedEngage… traces below.)
    controller_->set_brake_engage_sequence(0x01);
    EXPECT_EQ(step(kStSwitchedOn).first, 0x0007) << "user re-engage restarts (the first step is re-consumed)";
    EXPECT_EQ(step(kStSwitchedOn).first, 0x0006);
}

TEST_F(BrakeGoldenTraceTest, AlternatingModeTripsOnOtherAxesDoNotStarveAWalkingEngage) {
    // Axes 0 and 1 stay enabled and ignore every disable while
    // their displays (variant 0) or displays and statuswords (variant 1) alternate every
    // cycle; a healthy axis 2 follows its commands. The alternating onsets
    // keep re-triggering the forced engage, but axis 2 must still receive
    // its READY command (0x0006) and settle closed; the forced engage never
    // resets its head. Events are tallied per tick (the fixture queue is
    // small); axis 2's controlword register is cleared at each variant's
    // start so only a fresh command counts.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_all_and_step(kStOpEnabled);

    for (int variant = 0; variant < 2; ++variant) {
        SCOPED_TRACE(variant == 0 ? "variant 0: displays alternate, both stay enabled"
                                  : "variant 1: (enabled, off-mode) <-> (READY, on-mode) alternate");
        // Warm-up: every axis enabled and accepted, axis 2 idle with a clean
        // controlword register (variant 0 left it holding its last command).
        // This cycle's inputs per joint: statusword and 0x6061 display.
        uint16_t status[6] = {kStOpEnabled, kStOpEnabled, kStOpEnabled, kStOpEnabled, kStOpEnabled, kStOpEnabled};
        uint8_t display[6] = {0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A};
        for (int warm = 0; warm < 3; ++warm) {
            for (uint32_t j = 0; j < 6; ++j) {
                feed_display(j, display[j]);
            }
            feed_all_and_step(kStOpEnabled);
        }
        DeviceServoTestAccess::set_controlword(*bus_->servo_devices()[2], 0x0000);
        (void)drain_events(ctx);

        int trips = 0;
        int stops = 0;
        bool c_got_ready_command = false;
        uint16_t c_status = 0x0237;
        for (int tick = 0; tick < 12; ++tick) {
            const bool phase = (tick % 2) == 0;
            // Axis 0 off-mode on even ticks, axis 1 on odd ticks; variant 1
            // additionally reports READY/on-mode on the accepted phase.
            display[0] = phase ? 0x08 : 0x0A;
            display[1] = phase ? 0x0A : 0x08;
            status[0] = (variant == 1 && !phase) ? kStReady : kStOpEnabled;
            status[1] = (variant == 1 && phase) ? kStReady : kStOpEnabled;
            status[2] = c_status;
            for (uint32_t j = 0; j < 6; ++j) {
                feed_status(j, status[j]);
                feed_display(j, display[j]);
            }
            run_cycle();
            switch (controlword(2)) {  // C follows its command
                case 0x0007:
                    c_status = kStSwitchedOn;
                    break;
                case 0x0006:
                    c_status = kStReady;
                    c_got_ready_command = true;
                    break;
                default:
                    break;
            }
            for (const auto& event : drain_events(ctx)) {
                trips += event.type == plem::core::monitoring::EventType::MODES_OF_OPERATION_MISMATCH ? 1 : 0;
                stops += event.type == plem::core::monitoring::EventType::PROTECTIVE_STOP ? 1 : 0;
            }
        }
        EXPECT_TRUE(c_got_ready_command) << "axis 2 must receive its READY command while the trips keep coming";
        EXPECT_EQ(c_status, kStReady) << "axis 2 walked its engage down";
        EXPECT_EQ(controller_->brake_operation_pending() & 0x04, 0x00)
            << "axis 2's engage settled (READY proves closure)";
        // The trigger cadence this trace relies on: variant 0's fixed-enabled axes
        // trip once each (their engage walks, never eligible again), variant 1's alternating
        // axes become eligible again on every accepted READY sample and trip again on every
        // enabled sample, so the forced engage keeps being re-applied while
        // axis 2 walks; that is exactly what the preservation is for.
        if (variant == 0) {
            EXPECT_EQ(trips, 2) << "variant 0: one onset per axis, not eligible again while their engage walks";
            EXPECT_EQ(stops, 2);
        } else {
            EXPECT_GE(trips, 8) << "variant 1: the alternating axes must keep tripping";
            EXPECT_GE(stops, 8) << "variant 1: every new onset re-applies the forced engage";
        }
        // Leave the variant with every axis accepted.
        for (uint32_t j = 0; j < 6; ++j) {
            feed_display(j, 0x0A);
        }
    }
}

// Recovery / pending / re-eligibility boundaries of the in-service trip
// Every trace below runs under a committed {P_STOP, FREEDRIVE} whose edge is
// long consumed and whose recovery lane released every axis (every drive
// OPERATION_ENABLED); the bus's motion_possible is YES there, so a forced
// engage stays pending for as long as every drive ignores its disable.

TEST_F(BrakeGoldenTraceTest, RecoveryReleasedArmReEngagesOnAModeExitAndStaysEngagedWithoutANewRequest) {
    enter_recovery_released();
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    // Joint 0 leaves its mode while enabled: the axis is named, the torque
    // gate closes, and the next brake step force-engages every joint.
    feed_display(0, 0x08);
    feed_all_and_step(kStOpEnabled);
    ASSERT_EQ(events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH).size(), 1u);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire));
    feed_all_and_step(kStOpEnabled);
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0007) << "the forced engage commands every joint down from OE";
    }
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);

    // Every drive follows: SWITCHED_ON reached (consumed), READY commanded,
    // READY reached: the engage settles (READY proves the brake closed) and
    // the busy mirror clears.
    feed_all_and_step(kStSwitchedOn);
    feed_all_and_step(kStSwitchedOn);
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0006);
    }
    feed_all_and_step(kStReady);
    feed_all_and_step(kStReady);
    EXPECT_EQ(controller_->brake_operation_pending(), 0x00) << "every engage settled";
    EXPECT_EQ(controller_->brake_engaged(), 0x3F) << "every axis is braked";

    // No new request: nothing releases the axes again, not
    // the still-committed recovery lane, not joint 0's display recovering.
    feed_display(0, 0x0A);
    for (int i = 0; i < 4; ++i) {
        feed_all_and_step(kStReady);
    }
    EXPECT_EQ(controlword(0), 0x0006) << "no release without a new request";
    EXPECT_EQ(controller_->brake_engaged(), 0x3F);
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::BRAKE_RELEASE), 0);
}

TEST_F(BrakeGoldenTraceTest, PendingForcedEngageSuppressesARecoveryReleaseWhileEveryDriveStaysEnabled) {
    enter_recovery_released();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0x08);
    feed_all_and_step(kStOpEnabled);  // onset
    feed_all_and_step(kStOpEnabled);  // forced engage; every drive ignores it
    (void)drain_events(ctx);

    // Joint 0's display recovers while every drive stays enabled: the forced
    // engage is still pending (motion_possible YES), so a recovery release
    // posted now is consumed and dropped: no refusal, no sequence swap, and
    // joint 0's episode stays open (its engage is still walking).
    feed_display(0, 0x0A);
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_all_and_step(kStOpEnabled);
    EXPECT_EQ(controller_->brake_request_masks_for_test().first, 0x00) << "consumed";
    EXPECT_TRUE(drain_events(ctx).empty()) << "dropped by the pending engage: no refusal, no trip, no stop";
    for (uint32_t j = 0; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0007) << "the engage keeps walking — a release would command 0x0006 from OE";
    }

    // The display flaps off-mode again: same episode, no second diagnostic.
    feed_display(0, 0x08);
    feed_all_and_step(kStOpEnabled);
    EXPECT_TRUE(drain_events(ctx).empty());
}

TEST_F(BrakeGoldenTraceTest, ReleaseReplacingAStalledEngageReArmsTheEpisodeInTheSameCycle) {
    enter_recovery_released();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0x08);
    feed_all_and_step(kStOpEnabled);  // onset on joint 0
    feed_all_and_step(kStOpEnabled);  // forced engage on every joint
    (void)drain_events(ctx);

    // Joint 1 obeys (SWITCHED_ON) so the pending engage resolves; joint 0
    // ignores its disable, its display back in mode: accepted, but its
    // engage is walking, so the episode stays open.
    feed_display(0, 0x0A);
    feed_all_and_step(kStOpEnabled, /*except_joint=*/1, kStSwitchedOn);
    EXPECT_EQ(controlword(0), 0x0007) << "joint 0's engage keeps re-commanding SWITCHED_ON";

    // The upper layer re-requests joint 0's release: admitted (recovery lane,
    // nothing pending) and ready (OE, recognized, CST), so the release replaces
    // the walking engage, and the same sequencing step re-arms the episode
    // (a release walking on the axis is not an engage walking).
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    feed_all_and_step(kStOpEnabled, /*except_joint=*/1, kStSwitchedOn);
    EXPECT_EQ(controlword(0), 0x0006) << "the release's first target (READY) is commanded from OE";
    expect_no_mode_rejection_or_stop(ctx);  // an admitted, ready release neither refuses nor trips

    // Joint 0 leaves its mode again while still enabled: a new onset, named
    // again, and the next step force-engages (the release is replaced).
    feed_display(0, 0x08);
    feed_all_and_step(kStOpEnabled, /*except_joint=*/1, kStReady);
    {
        const auto trips = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(trips.size(), 1u) << "re-armed: the second exit is a new episode";
        EXPECT_EQ(trips[0].joint_id, 0);
        EXPECT_FLOAT_EQ(trips[0].value, 8.0f);
    }
    feed_all_and_step(kStOpEnabled, /*except_joint=*/1, kStReady);
    EXPECT_EQ(controlword(0), 0x0007) << "the forced engage replaces the walking release";
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
}

TEST_F(BrakeGoldenTraceTest, ReleasePostedAtThePendingResolutionIsRefusedWhileAJointIsOffModeAndNeverRevives) {
    // Joint 0 trips and every drive ignores the forced engage. A release is
    // posted just before joint 1 finally obeys: the resolution clears the
    // pending engage in the same cycle the request is consumed, so the
    // request is admitted, and refused as a whole while joint 0 is still off
    // its mode. Readiness recovering afterwards revives nothing (no 0x000F
    // ever again without a fresh request).
    enter_recovery_released();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0x08);
    feed_all(kStOpEnabled);
    run_cycle();  // onset
    run_cycle();  // forced engage
    run_cycle();  // still pending: nothing new
    (void)drain_events(ctx);
    EXPECT_EQ(controlword(0), 0x0007);

    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_status(1, kStSwitchedOn);
    run_cycle();  // pending resolves → the request is admitted → refused
    {
        const auto refusals = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(refusals.size(), 1u);
        EXPECT_EQ(refusals[0].joint_id, 0);
        EXPECT_FLOAT_EQ(refusals[0].value, 8.0f);
    }
    EXPECT_EQ(controlword(0), 0x0007) << "joint 0's engage keeps walking after the refusal";

    // Joint 0's display recovers and the drive finally walks down: nothing
    // releases it; the engage settles at READY.
    feed_display(0, 0x0A);
    run_cycle();
    EXPECT_TRUE(drain_events(ctx).empty()) << "a refused request is consumed — nothing is left to revive";
    feed_status(0, kStSwitchedOn);
    run_cycle();
    run_cycle();
    EXPECT_EQ(controlword(0), 0x0006) << "the engage commands READY";
    feed_status(0, kStReady);
    run_cycle();
    run_cycle();
    EXPECT_EQ(controlword(0), 0x0006) << "never 0x000F — nothing released joint 0 again";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "joint 0's engage settled";
}

TEST_F(BrakeGoldenTraceTest, ReleasePostedAtThePendingResolutionReplacesTheWalkingEngageWhenEveryJointIsReady) {
    // Same shape, but joint 0 is back in its mode before the resolution
    // cycle: the request is admitted and ready, so the release starts on
    // every joint, replacing joint 0's walking engage (the release's READY
    // target is commanded from OE, then the drive walks up to 0x000F, which
    // an engage would never command).
    enter_recovery_released();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0x08);
    feed_all(kStOpEnabled);
    run_cycle();  // onset
    run_cycle();  // forced engage
    (void)drain_events(ctx);

    feed_display(0, 0x0A);
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    feed_status(1, kStSwitchedOn);
    run_cycle();                            // pending resolves → the request is admitted, every joint ready
    expect_no_mode_rejection_or_stop(ctx);  // admitted and ready: no refusal
    EXPECT_EQ(controlword(0), 0x0006) << "the release's first target from OE, not the engage's 0x0007";
    feed_status(0, kStReady);
    run_cycle();
    run_cycle();
    EXPECT_EQ(controlword(0), 0x0007);
    feed_status(0, kStSwitchedOn);
    run_cycle();
    run_cycle();
    EXPECT_EQ(controlword(0), 0x000F) << "the release walked joint 0 back up — an engage never commands this";
}

TEST_F(BrakeGoldenTraceTest, AReleasedAxisTripsItsOwnEpisodeWhileAnotherAxisEpisodeStaysOpen) {
    enter_recovery_released();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    feed_display(0, 0x08);
    feed_all_and_step(kStOpEnabled);  // onset on joint 0
    feed_all_and_step(kStOpEnabled);  // forced engage
    (void)drain_events(ctx);

    // Joint 0 stays enabled off its mode (its episode open, the level held);
    // the other five obey and settle at READY.
    feed_all_and_step(kStSwitchedOn, /*except_joint=*/0, kStOpEnabled);
    feed_all_and_step(kStSwitchedOn, /*except_joint=*/0, kStOpEnabled);
    feed_all_and_step(kStReady, /*except_joint=*/0, kStOpEnabled);
    feed_all_and_step(kStReady, /*except_joint=*/0, kStOpEnabled);
    EXPECT_EQ(controller_->brake_operation_pending() & 0x3E, 0x00);

    // Joint 1 alone is re-released (ready: READY, recognized, CST) and walks
    // back up to OPERATION_ENABLED; joint 0's open episode is not re-reported.
    ASSERT_TRUE(controller_->request_brake_release(0x02));
    feed_all_and_step(kStReady, /*except_joint=*/0, kStOpEnabled);
    feed_all_and_step(kStReady, /*except_joint=*/0, kStOpEnabled);
    EXPECT_EQ(controlword(1), 0x0007);
    feed_status(1, kStSwitchedOn);
    run_cycle();
    run_cycle();
    EXPECT_EQ(controlword(1), 0x000F);
    feed_status(1, kStOpEnabled);
    run_cycle();
    expect_no_mode_rejection_or_stop(ctx);  // no trip, no stop while joint 1 walks up

    // Joint 1 now leaves its mode while enabled: its own onset, named, and
    // the next step force-engages it for real.
    feed_display(1, 0x08);
    run_cycle();
    {
        const auto trips = events_of(drain_events(ctx), EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(trips.size(), 1u);
        EXPECT_EQ(trips[0].joint_id, 1);
        EXPECT_EQ(trips[0].error_code, kStOpEnabled);
        EXPECT_FLOAT_EQ(trips[0].value, 8.0f);
    }
    run_cycle();
    EXPECT_EQ(controlword(1), 0x0007) << "joint 1's onset starts its engage";
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
}

// What the forced engage preserves besides the head

TEST_F(BrakeGoldenTraceTest, ForcedEngageKeepsTheStallHistoryOfAWalkingEngage) {
    // A user engage on joint 0 whose drive ignores it: two cycles without
    // progress, then the P-STOP rising edge force-engages. The preserved
    // stall window reports on the third cycle (the forced step itself); a
    // reset of the stall counters would push the report out by a window.
    controller_->set_sequence_stall_threshold_for_test(3);
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    controller_->set_brake_engage_sequence(0x01);
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007);
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007);
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED), 0);

    flags_ = std::make_shared<plem::core::ControlModeFlags>();
    flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    controller_->set_mode_flags(flags_);
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007);
    int stalled_joint0 = 0;
    for (const auto& event : drain_events(ctx)) {
        stalled_joint0 +=
            (event.type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED && event.joint_id == 0) ? 1 : 0;
    }
    EXPECT_EQ(stalled_joint0, 1) << "the stall window carried over the forced engage";
}

TEST_F(BrakeGoldenTraceTest, ForcedEngageReplacesAWalkingReleaseAndKeepsAWalkingEngage) {
    // Singleton groups (a shared group never holds both directions; every
    // loader applies the closure). Joint 0 walks an engage (SWITCHED_ON
    // reached), joint 1 walks a release (READY reached); the forced engage
    // keeps joint 0 (READY commanded next) and replaces joint 1's release
    // with an engage, which settles at once (READY proves the brake closed);
    // its release's next step (0x0007) is never commanded. One
    // BRAKE_ENGAGE for the whole closure.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    controller_->set_brake_engage_sequence(0x01);
    ASSERT_TRUE(controller_->request_brake_release(0x02));
    {
        const auto [cw0, cw1] = step_pair(kStOpEnabled, kStNotReady);
        ASSERT_EQ(cw0, 0x0007);
        ASSERT_EQ(cw1, 0x0006);
    }
    {
        const auto [cw0, cw1] = step_pair(kStSwitchedOn, kStReady);  // both steps reached (consumed)
        ASSERT_EQ(cw0, 0x0007);
        ASSERT_EQ(cw1, 0x0006);
    }
    (void)drain_events(ctx);

    flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    const auto [cw0, cw1] = step_pair(kStSwitchedOn, kStReady);
    EXPECT_EQ(cw0, 0x0006) << "joint 0's engage continues with its next step";
    EXPECT_EQ(cw1, 0x0006) << "joint 1's release is gone — no 0x0007; the engage settled at READY";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x02, 0x00) << "joint 1 settled engaged";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x01) << "joint 0 still walking";
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::BRAKE_ENGAGE), 1);
}

TEST_F(BrakeGoldenTraceTest, ForcedEngageThatPreservesEveryAxisStillPublishesBusyAndTheEngageEvent) {
    // Every axis walks an engage (SWITCHED_ON reached, READY next) when the
    // P-STOP rising edge fires: the forced engage loads nothing new (every
    // joint is preserved) and must still publish the closure: every engage
    // continues, the busy mirror covers every joint, and BRAKE_ENGAGE is
    // emitted once (no "nothing to load → return" shortcut).
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    controller_->set_brake_engage_sequence(0x3F);
    feed_all_and_step(kStOpEnabled);
    for (uint32_t j = 0; j < 6; ++j) {
        ASSERT_EQ(controlword(j), 0x0007) << "joint " << j;
    }
    feed_all_and_step(kStSwitchedOn);  // every step reached (consumed)
    (void)drain_events(ctx);

    flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    feed_all_and_step(kStSwitchedOn);
    for (uint32_t j = 0; j < 6; ++j) {
        EXPECT_EQ(controlword(j), 0x0006) << "joint " << j << ": the preserved engage continues with READY";
    }
    EXPECT_EQ(controller_->brake_operation_pending(), 0x3F) << "every joint busy";
    const auto events = drain_events(ctx);
    EXPECT_EQ(count_of(events, plem::core::monitoring::EventType::BRAKE_ENGAGE), 1)
        << "the closure's engage event is emitted even though nothing was loaded";
    EXPECT_EQ(count_of(events, plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
}

TEST_F(BrakeGoldenTraceTest, AQueuedEngageStillRestartsAnAxisTheForcedEngageJustPreserved) {
    // The mailbox engage (user request; the quiesce retry posts through the
    // same mailbox) keeps its restart semantics in the very cycle a forced
    // engage preserves the axis: joint 0 at SWITCHED_ON with that step
    // consumed; joint 3's trip arms the forced engage for the next step, and
    // an engage for joint 0 is posted. That step: forced engage preserves
    // (head at READY), then the request re-loads (head at SWITCHED_ON,
    // re-consumed: the word holds 0x0007); READY is commanded one cycle
    // later than the preservation alone would have.
    arm_motion();
    controller_->set_brake_engage_sequence(0x01);
    feed_status(0, kStOpEnabled);
    feed_status(3, kStOpEnabled);
    run_cycle();
    ASSERT_EQ(controlword(0), 0x0007);
    feed_status(0, kStSwitchedOn);
    feed_display(3, 0x08);  // joint 3's onset lands in this step
    run_cycle();
    ASSERT_EQ(controlword(0), 0x0007);

    controller_->request_brake_engage(0x01);
    run_cycle();  // forced engage (preserve) then the queued engage (restart)
    EXPECT_EQ(controlword(0), 0x0007) << "restarted: the first step is re-consumed, no READY command yet";
    run_cycle();
    EXPECT_EQ(controlword(0), 0x0006) << "READY commanded one cycle later";
}

TEST_F(BrakeGoldenTraceTest, ForcedEngageReCommandsThePreservedTargetWhenTheDriveReEntersOe) {
    // A drive that re-enters OPERATION_ENABLED mid-engage, then a P-STOP
    // rising edge: the preserved sequence re-commands its current target:
    // SWITCHED_ON (0x0007, the same word a restart would send), READY
    // (0x0006, where a restart would send 0x0007), and the terminal
    // SWITCH_ON_DISABLED, commanded from OE with Disable voltage (0x0000,
    // CiA 402 transition 9).
    arm_motion();
    auto rising_edge_from_oe = [this]() {
        flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
        (void)step(kStOpEnabled);  // the falling edge settles; the next set is a
                                   // fresh rising edge
        flags_->set_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
        return step(kStOpEnabled).first;
    };

    controller_->set_brake_engage_sequence(0x01);
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007);  // target SWITCHED_ON
    EXPECT_EQ(rising_edge_from_oe(), 0x0007) << "SWITCHED_ON target re-commanded";

    controller_->set_brake_engage_sequence(0x01);
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007);
    ASSERT_EQ(step(kStSwitchedOn).first,
              0x0007);  // SWITCHED_ON reached: target READY
    EXPECT_EQ(rising_edge_from_oe(), 0x0006) << "READY target re-commanded from OE — not the restart's 0x0007";

    controller_->set_brake_engage_sequence(0x01);
    ASSERT_EQ(step(kStOpEnabled).first, 0x0007);
    ASSERT_EQ(step(kStSwitchedOn).first, 0x0007);
    ASSERT_EQ(step(kStSwitchedOn).first, 0x0006);
    ASSERT_EQ(step(kStReady).first,
              0x0006);  // READY reached: target SWITCH_ON_DISABLED
    EXPECT_EQ(rising_edge_from_oe(), 0x0000) << "SWITCH_ON_DISABLED target re-commanded: Disable voltage";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x01)
        << "still walking: the harness holds the drive at OE, so the step never arrives";
}
// Full-mask release from mixed rest states must command every axis
//
// Rest statuswords observed on hardware: 0x0040 and 0x0060 (both
// SWITCH_ON_DISABLED). A full-mask release from them must walk every axis to
// OPERATION_ENABLED: an axis left at controlword 0x0000 while the mode
// transition reports success is the failure these traces pin, replayed
// against a responding drive sim. Native-brake sequencing only, with the
// actuator absent; actuator behavior is not validated.

using plem::fieldbus::test::Cia402DriveSim;

class BrakeMixedRestStateTraceTest : public BrakeGoldenTraceTest {
  protected:
    void SetUp() override {
        BrakeGoldenTraceTest::SetUp();
        // These traces replay TRAJECTORY-entry releases: a committed
        // motion mode (release consumption requires motion_enabled).
        arm_motion();
    }

    /// Observed rest statuswords in joint order: SWITCH_ON_DISABLED with bit 5
    /// clear (0x0040) or set (0x0060).
    static constexpr uint16_t kMixedRest[6] = {0x0060, 0x0040, 0x0040, 0x0060, 0x0040, 0x0060};

    void SetUpDrives() {
        drives_.clear();
        for (uint16_t rest : kMixedRest) {
            drives_.emplace_back(rest);
        }
    }

    /// One synchronous fake-master cycle over all six axes: feed each drive's
    /// statusword, read, consume direct requests, run sequencing, write, then
    /// let each drive respond to the controlword now on the wire.
    void step_all() {
        for (size_t i = 0; i < drives_.size(); ++i) {
            feed_status(static_cast<uint32_t>(i), drives_[i].status());
        }
        run_cycle();
        for (size_t i = 0; i < drives_.size(); ++i) {
            drives_[i].consume(controlword(static_cast<uint32_t>(i)));
        }
    }

    void expect_all_enabled(const char* phase) {
        for (size_t i = 0; i < drives_.size(); ++i) {
            SCOPED_TRACE(std::string(phase) + " joint " + std::to_string(i));
            EXPECT_EQ(controlword(static_cast<uint32_t>(i)), 0x000F)
                << "axis never commanded to OPERATION_ENABLED (silent sequence stall)";
            EXPECT_EQ(drives_[i].status(), 0x0237);
        }
    }

    std::vector<Cia402DriveSim> drives_;
};

TEST_F(BrakeMixedRestStateTraceTest, FullMaskReleaseFromMixedRestStatesCommandsEveryAxis) {
    SetUpDrives();
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    for (int cycle = 0; cycle < 40; ++cycle) {
        step_all();
    }
    expect_all_enabled("single release");
}

TEST_F(BrakeMixedRestStateTraceTest, StalledReleaseEmitsOneSequenceStalledEventPerJointPerEpisode) {
    // Whatever stalls a sequence, it must name itself. Frozen drives
    // (statuses never leave the rest values) stall all six release sequences. Pins: the exact window
    // boundary (nothing at N-1 cycles, one event at N), the payload
    // (error_code = the axis's statusword, value = the unreached target
    // ServoState: READY, the release sequence's first step), the per-episode
    // freeze (no re-emission while the stall persists), and a fresh episode
    // on re-load.
    SetUpDrives();
    controller_->set_sequence_stall_threshold_for_test(50);
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    auto step_frozen = [this]() {
        for (size_t i = 0; i < drives_.size(); ++i) {
            feed_status(static_cast<uint32_t>(i), drives_[i].rest_status());
        }
        run_cycle();
    };
    struct Stall {
        uint16_t error_code;
        float value;
    };
    auto drain_stalls = [&ctx]() {
        std::array<std::vector<Stall>, 6> per_joint{};
        auto* queue = ctx.queue<plem::core::monitoring::Event>();
        while (auto* event = queue->front()) {
            if (event->type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED) {
                EXPECT_LT(event->joint_id, 6);
                per_joint[event->joint_id % 6].push_back({event->error_code, event->value});
            }
            queue->pop();
        }
        return per_joint;
    };

    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    for (int cycle = 0; cycle < 49; ++cycle) {
        step_frozen();
    }
    {
        auto stalls = drain_stalls();
        for (size_t i = 0; i < 6; ++i) {
            SCOPED_TRACE("window boundary joint " + std::to_string(i));
            EXPECT_TRUE(stalls[i].empty()) << "no event before the stall window elapses";
        }
    }
    for (int cycle = 0; cycle < 101; ++cycle) {
        step_frozen();  // crosses the window at cycle 50, then runs 2x past it
    }
    {
        auto stalls = drain_stalls();
        for (size_t i = 0; i < 6; ++i) {
            SCOPED_TRACE("episode 1 joint " + std::to_string(i));
            ASSERT_EQ(stalls[i].size(), 1u) << "one event per joint per episode (frozen once reported)";
            EXPECT_EQ(stalls[i][0].error_code, kMixedRest[i]) << "payload carries the axis's statusword";
            // value = the unreached target ServoState, whose numeric is the
            // canonical masked statusword pattern (READY_TO_SWITCH_ON=0x21).
            EXPECT_FLOAT_EQ(stalls[i][0].value, static_cast<float>(0x21));
        }
    }

    // A new sequence load arms a new episode: the same frozen drives must
    // report again after another window.
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    for (int cycle = 0; cycle < 60; ++cycle) {
        step_frozen();
    }
    {
        auto stalls = drain_stalls();
        for (size_t i = 0; i < 6; ++i) {
            SCOPED_TRACE("episode 2 joint " + std::to_string(i));
            ASSERT_EQ(stalls[i].size(), 1u);
        }
    }
}

TEST_F(BrakeMixedRestStateTraceTest, ReleaseWithNeverInitializedAxisIsRefusedAndNamesIt) {
    // An axis that never produced a statusword is not accepted in its
    // declared mode, so the full-mask release is refused as a whole at the
    // readiness gate and consumed (nothing pending, nothing loaded on any axis,
    // no controlword leaves), and the refusal names the axis with what it
    // showed: statusword 0 and the CST display the fixture seeds. No stall
    // accounting can start, because no sequence did; the executor-seam pin
    // below keeps the "active sequence above the initialization gate" class.
    SetUpDrives();
    drives_[0] = Cia402DriveSim(0x0000);
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    for (int cycle = 0; cycle < 3; ++cycle) {
        for (size_t i = 0; i < drives_.size(); ++i) {
            feed_status(static_cast<uint32_t>(i), drives_[i].rest_status());
        }
        run_cycle();
    }
    EXPECT_EQ(controller_->brake_operation_pending(), 0x00) << "refused = consumed and discarded — nothing pending";
    for (size_t i = 0; i < drives_.size(); ++i) {
        SCOPED_TRACE("joint " + std::to_string(i));
        EXPECT_EQ(controlword(static_cast<uint32_t>(i)), 0x0000) << "no axis may be commanded by a refused release";
    }

    int refusals = 0;
    int stalls = 0;
    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    while (auto* event = queue->front()) {
        if (event->type == plem::core::monitoring::EventType::MODES_OF_OPERATION_MISMATCH) {
            EXPECT_EQ(event->joint_id, 0) << "only the axis that failed the gate is named";
            EXPECT_EQ(event->error_code, 0) << "error_code = the statusword it showed (none)";
            EXPECT_FLOAT_EQ(event->value, 10.0f) << "value = the 0x6061 display it showed (the seeded CST)";
            ++refusals;
        }
        stalls += event->type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED ? 1 : 0;
        queue->pop();
    }
    EXPECT_EQ(refusals, 1) << "one refusal diagnostic per request";
    EXPECT_EQ(stalls, 0) << "no sequence started, so nothing can stall";
}

TEST_F(BrakeMixedRestStateTraceTest, StalledSequenceStillCountsAnUninitializedAxis) {
    // An engage posted to an axis that never produced a statusword: the axis
    // stays !is_initialized() and the sequence loop HOLDs it: exactly the
    // silent class BRAKE_SEQUENCE_STALLED exists for. The
    // stall accounting runs above the initialization gate; the payload
    // statusword is 0 (nothing was ever observed). Engage is not
    // readiness-gated, so the public request reaches the loader on a cold
    // axis; a cold release is refused before it could stall (the
    // ReleaseWithNeverInitializedAxis… trace).
    controller_->set_sequence_stall_threshold_for_test(3);
    plem::fieldbus::test::ScopedMonitoringContext ctx;

    controller_->request_brake_engage(0x01);
    for (int cycle = 0; cycle < 5; ++cycle) {
        run_cycle();  // joint 0's statusword stays 0x0000, never observed
    }

    int joint0_stalls = 0;
    for (const auto& event : drain_events(ctx)) {
        if (event.type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED && event.joint_id == 0) {
            EXPECT_EQ(event.error_code, 0) << "never-initialized axis reports statusword 0";
            ++joint0_stalls;
        }
    }
    EXPECT_EQ(joint0_stalls, 1);
}

TEST_F(BrakeMixedRestStateTraceTest, ToggleStormThenReleaseCommandsEveryAxis) {
    // Observed on hardware: BRAKED/TRAJECTORY toggled ~6 times within seconds, sequences overwriting each other
    // mid-step, before the final release was left to settle.
    SetUpDrives();
    controller_->request_brake_engage(0x3F);
    for (int cycle = 0; cycle < 3; ++cycle) {
        step_all();
    }
    for (int toggle = 0; toggle < 3; ++toggle) {
        ASSERT_TRUE(controller_->request_brake_release(0x3F));
        for (int cycle = 0; cycle < 3; ++cycle) {
            step_all();
        }
        controller_->request_brake_engage(0x3F);
        for (int cycle = 0; cycle < 3; ++cycle) {
            step_all();
        }
    }
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    for (int cycle = 0; cycle < 60; ++cycle) {
        step_all();
    }
    expect_all_enabled("post-storm release");
}

// Sequence-step ordering and counter pins

TEST_F(BrakeGoldenTraceTest, StallReportOutrunsFaultResetInTheSameCycle) {
    // The stall window and the fault-reset interval can elapse on the same
    // cycle. The stall check sits above the action branches, so the episode
    // report lands in the queue before that cycle's reset diagnostic; the
    // executor must keep this order.
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_sequence_stall_threshold_for_test(200);  // == kGoldenPacing interval, in cycles
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);  // READY reached: stall and interval counters rearm together

    plem::fieldbus::test::ScopedMonitoringContext ctx;
    for (int cycle = 0; cycle < 199; ++cycle) {
        SCOPED_TRACE("fault cycle " + std::to_string(cycle));
        ASSERT_EQ(step(kStFault).first, 0x0006);  // held wire word; neither window elapsed yet
    }
    EXPECT_EQ(step(kStFault).first, 0x0080) << "both windows elapse on this cycle";

    std::vector<plem::core::monitoring::EventType> order;
    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    while (auto* event = queue->front()) {
        if (event->type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED ||
            event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET) {
            EXPECT_EQ(event->joint_id, 0);
            order.push_back(event->type);
        }
        queue->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);
    ASSERT_EQ(order.size(), 2u) << "exactly one stall report and one reset diagnostic";
    EXPECT_EQ(order[0], plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED) << "stall-first";
    EXPECT_EQ(order[1], plem::core::monitoring::EventType::SERVO_FAULT_RESET);
}

TEST_F(BrakeGoldenTraceTest, UnrecognizedSelfWordKeepsEngageFallbackAliveUntilRecognized) {
    // Self leg (the partner leg is pinned above): a drive already
    // resting in a proven-unpowered step settles an engage without walking,
    // but only on a recognized word. An unrecognized sample holds the stale
    // previous state, so the fallback refuses to consume; the sequence stays
    // alive and settles the moment a recognized proven word returns.
    step(kStSodRestWord);                         // initialize joint 0 resting in SWITCH_ON_DISABLED
    controller_->set_brake_state_for_test(0x3E);  // joint 0 released, rest engaged
    controller_->set_brake_engage_sequence(0x01);
    for (int cycle = 0; cycle < 5; ++cycle) {
        SCOPED_TRACE("unrecognized cycle " + std::to_string(cycle));
        step(kStUnrecognized);
        ASSERT_EQ(controller_->brake_engaged() & 0x01, 0x00) << "no settlement on a held (stale) reading";
        ASSERT_EQ(controller_->brake_operation_pending() & 0x01, 0x01) << "the engage must stay alive, not get lost";
    }
    step(kStSodRestWord);
    EXPECT_EQ(controller_->brake_engaged() & 0x01, 0x01) << "recognized proven word settles the kept sequence";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00);
}

TEST_F(BrakeGoldenTraceTest, RecoveryPreResetsAttemptsButCarriesTheElapsedInterval) {
    // The healthy-transition path zeroes the attempt allowance before
    // commanding (a later, genuinely separate fault gets the full budget),
    // but deliberately does not touch the interval counter: a recovery
    // shorter than one interval leaves the elapsed cycles in place, so the
    // next episode's first reset arrives early. Both halves are measured
    // current behavior.
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);

    plem::fieldbus::test::ScopedMonitoringContext ctx;

    for (int cycle = 0; cycle < 100; ++cycle) {
        ASSERT_EQ(step(kStFault).first, 0x0006);  // half the 200-cycle interval elapses
    }
    ASSERT_EQ(step(kStReady).first, 0x0007) << "recovery commands the next step (attempts pre-reset here)";
    for (int cycle = 0; cycle < 99; ++cycle) {
        SCOPED_TRACE("second-episode fault cycle " + std::to_string(cycle));
        ASSERT_EQ(step(kStFault).first, 0x0007);
    }
    EXPECT_EQ(step(kStFault).first, 0x0080) << "carried interval: the second episode resets after 100 cycles";

    ASSERT_EQ(step(kStReady).first, 0x0000) << "the frame after a pulse is the falling edge";
    ASSERT_EQ(step(kStReady).first, 0x0007) << "recover again — attempts 1 -> 0 before the command";
    for (int cycle = 0; cycle < 199; ++cycle) {
        SCOPED_TRACE("third-episode fault cycle " + std::to_string(cycle));
        ASSERT_EQ(step(kStFault).first, 0x0007);
    }
    EXPECT_EQ(step(kStFault).first, 0x0080) << "full interval again — the reset itself cleared the carry";

    int resets = 0;
    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    while (auto* event = queue->front()) {
        if (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET) {
            EXPECT_FLOAT_EQ(event->value, 1.0f) << "attempt # restarts per episode — the pre-reset pin";
            ++resets;
        }
        queue->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);
    EXPECT_EQ(resets, 2);
}

TEST_F(BrakeGoldenTraceTest, UnrecognizedWordStillDrivesTheTransitionCommand) {
    // Recognition is not a common gate (it is fallback-only): after a
    // recognized arrival consumed the READY step, an unrecognized sample
    // holds that state, and the walk keeps commanding the next step on it.
    // Promoting recognition to a common gate would stall every walk whose
    // drive interleaves unknown words; this pin makes that mutant red.
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);  // recognized arrival consumes READY
    EXPECT_EQ(step(kStUnrecognized).first, 0x0007)
        << "the transition command must ride the HELD state, not wait for recognition";
}

// The pacing is the model's, declared in time

TEST_F(BrakeGoldenTraceTest, DeclaredPacingSetsTheRequestCadenceAndTheCap) {
    // {50, 3}: a pacing nothing in the controller could have synthesized: at
    // the 1 ms fixture period one accepted request per 50 fault cycles, three
    // per episode. Every index below is a literal, not derived from the pair.
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, FaultResetPacing{50, 3});
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);

    plem::fieldbus::test::ScopedMonitoringContext ctx;

    std::vector<uint16_t> words;
    for (int cycle = 0; cycle < 5 * 50; ++cycle) {
        words.push_back(step(kStFault).first);
    }
    std::vector<size_t> highs;
    for (size_t i = 0; i < words.size(); ++i) {
        if (words[i] == 0x0080) {
            highs.push_back(i);
            ASSERT_LT(i + 1, words.size());
            EXPECT_EQ(words[i + 1], 0x0000) << "every pulse owes a falling edge — index " << i;
        } else {
            EXPECT_EQ(words[i], i < 49 ? 0x0007 : 0x0000) << "index " << i;
        }
    }
    ASSERT_EQ(highs.size(), 3u) << "one rising edge per accepted request, up to the DECLARED cap";
    for (size_t k = 0; k < highs.size(); ++k) {
        EXPECT_EQ(highs[k], 49 + 50 * k) << "pulses land one per DECLARED interval — pulse " << k;
    }

    auto* queue = ctx.queue<plem::core::monitoring::Event>();
    ASSERT_NE(queue, nullptr);
    int reset_events = 0, exhausted_events = 0;
    float exhausted_value = -1.0F;
    while (auto* event = queue->front()) {
        reset_events += (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET) ? 1 : 0;
        if (event->type == plem::core::monitoring::EventType::SERVO_FAULT_RESET_EXHAUSTED) {
            ++exhausted_events;
            exhausted_value = event->value;
        }
        queue->pop();
    }
    plem::core::monitoring::set_thread_context(nullptr);
    EXPECT_EQ(reset_events, 3);
    EXPECT_EQ(exhausted_events, 1);
    EXPECT_FLOAT_EQ(exhausted_value, 3.0F) << "the exhausted event carries the declared cap, not a constant";
}

TEST_F(BrakeGoldenTraceTest, FaultResetSetterRefusesAnInvalidPolicyPacingPair) {
    // The shared rule, in the setter's context (the spec gate applies the
    // same rule before assembly; a hand-wired controller meets it here).
    EXPECT_THROW(controller_->set_fault_reset_policy(FaultResetPolicy::NONE, FaultResetPacing{200, 25}),
                 std::invalid_argument);
    EXPECT_THROW(controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, std::nullopt),
                 std::invalid_argument);
    EXPECT_THROW(
        controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, FaultResetPacing{200, 10'001}),
        std::invalid_argument);
}

TEST_F(BrakeGoldenTraceTest, FaultResetSetterRefusesAnIntervalThatOverflowsTheCycleCount) {
    // ceil(interval / period) must fit uint32. At the 1 ms fixture period no
    // uint32 interval can overflow (UINT32_MAX ms is exactly UINT32_MAX
    // cycles), so the bound is only reachable below 1 ms: at 200 µs the last
    // representable interval is 858'993'459 ms. The check runs before
    // cycles_at_least_ms would silently truncate the count.
    EXPECT_NO_THROW(
        controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, FaultResetPacing{UINT32_MAX, 1}));

    auto fast = plem::fieldbus::test::test_controller_config<6>(200);
    SequencingProbe at_5khz(fast);
    EXPECT_NO_THROW(
        at_5khz.set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, FaultResetPacing{858'993'459, 1}));
    EXPECT_THROW(at_5khz.set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, FaultResetPacing{858'993'460, 1}),
                 std::invalid_argument);

    // The predicate is a division, not UINT32_MAX × period: at a 3 s period
    // that product would overflow int64 while the shipped {200, 25} is a
    // perfectly valid single cycle (this API takes any positive period;
    // the product hardware range is not its precondition).
    auto slow = plem::fieldbus::test::test_controller_config<6>(3'000'000);
    SequencingProbe at_3s(slow);
    EXPECT_NO_THROW(at_3s.set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, FaultResetPacing{200, 25}));
}

TEST_F(BrakeGoldenTraceTest, DeclaredIntervalIsConvertedAtThisControllersPeriod) {
    // The declared 200 ms reaches the decider as cycles of this period, rounded
    // up: at 300 µs that is 667, not 200 (a regression that stored the ms
    // value as cycles) and not 666 (floor). Same bus, a controller at 300 µs
    // swapped in for the fixture's 1 ms one so step() drives it.
    auto cfg = plem::fieldbus::test::test_controller_config<6>(300);
    auto at_300us = std::make_unique<SequencingProbe>(cfg);
    at_300us->set_bus(bus_);
    controller_.swap(at_300us);

    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, FaultResetPacing{200, 3});
    controller_->set_brake_release_sequence(0x01);
    ASSERT_EQ(step(kStNotReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0006);
    ASSERT_EQ(step(kStReady).first, 0x0007);

    // Two windows of the wire only; the cap and the exhausted event are the
    // {50, 3} test's observation; what this one adds is the 667.
    std::vector<size_t> highs;
    for (size_t i = 0; i < 2 * 667; ++i) {
        if (step(kStFault).first == 0x0080) {
            highs.push_back(i);
        }
    }
    ASSERT_EQ(highs.size(), 2u);
    EXPECT_EQ(highs[0], 666u) << "200 ms at 300 µs = 667 cycles, rounded up (199 = ms stored as cycles, 665 = floor)";
    EXPECT_EQ(highs[1], 1333u);
}

// Enable loss: a released axis leaving OPERATION_ENABLED
//    outside a walking sequence is an in-service trip, with the same stop policy as
//    the mode trip (EXTERNAL P-STOP level + one forced engage per episode).
//    The predicate is per axis: "its last sequence was a release and completed,
//    its group still reads released, and its recognized word is not OE".

TEST_F(BrakeGoldenTraceTest, ReleasedAxisLeavingOperationEnabledTripsTheStopOnceAndForcesTheEngage) {
    arm_motion();
    release_all_and_enable();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);

    // Axis 1 drops to FAULT one sample after the hold began (the bench word).
    feed_all_and_step(kStOpEnabled, 1, kStFault);
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire));
    EXPECT_TRUE(flags_->is_external_protective_stop()) << "the loss asserts the EXTERNAL P-STOP source";
    {
        const auto events = drain_events(ctx);
        const auto lost = events_of(events, EventType::OPERATION_ENABLED_LOST);
        ASSERT_EQ(lost.size(), 1u) << "named once";
        EXPECT_EQ(lost[0].joint_id, 1);
        EXPECT_EQ(lost[0].error_code, kStFault);
        EXPECT_FLOAT_EQ(lost[0].value, 0.0f) << "no derived payload — the statusword is the observation";
        EXPECT_TRUE(events_of(events, EventType::MODES_OF_OPERATION_MISMATCH).empty())
            << "the display is still accepted: not a mode trip";
    }

    // Next brake step: every healthy axis is commanded down; the faulted axis
    // has its engage loaded (a FAULT cannot be commanded to SWITCHED_ON, so
    // its standing word holds; no immediate 0x0007 is claimed for it).
    feed_all_and_step(kStOpEnabled, 1, kStFault);
    for (uint32_t j = 0; j < 6; ++j) {
        if (j == 1) {
            continue;
        }
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0007) << "forced engage on every healthy axis";
    }
    EXPECT_EQ(controller_->brake_operation_pending() & 0x02, 0x02) << "the faulted axis's engage is walking";
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
    feed_all_and_step(kStOpEnabled, 1, kStFault);
    feed_all_and_step(kStOpEnabled, 1, kStFault);
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).empty())
        << "one diagnostic per episode while the loss persists";

    // Eligible again: the drive recovers to SWITCH_ON_DISABLED on its own, which is proven
    // closed, so the walking engage settles early and the axis's last
    // sequence is now an engage; a fresh release and a second loss open a
    // second episode (second diagnostic).
    feed_all(kStSwitchOnDisabled);
    run_cycle();
    run_cycle();
    EXPECT_EQ(controller_->brake_engaged(), 0x3F) << "every group re-engaged";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).empty());
    flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    release_all_and_enable();
    drain_events(ctx);
    feed_all_and_step(kStOpEnabled, 1, kStFault);
    EXPECT_EQ(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).size(), 1u)
        << "a new episode after a completed re-release";
}

TEST_F(BrakeGoldenTraceTest, AnAxisThatCompletedItsReleaseIsJudgedBeforeThePartnersReachOperationEnabled) {
    // The bench order: two axes complete their release a sample before the
    // rest; one of them faults on the very sample the rest arrive. The torque
    // gate never opened (no all-OE sample); the check is per axis, not
    // gated on a past all_drives_enabled.
    arm_motion();
    feed_all(kStNotReady);
    run_cycle();
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    for (int cycle = 0; cycle < 3 && controlword(5) != 0x0006; ++cycle) {
        run_cycle();
    }
    feed_all(kStReady);
    run_cycle();
    run_cycle();
    feed_all(kStSwitchedOn);
    run_cycle();
    run_cycle();
    for (uint32_t j = 0; j < 6; ++j) {
        ASSERT_EQ(controlword(j), 0x000F) << "joint " << j;
    }
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);

    feed_all(kStSwitchedOn);
    feed_status(0, kStOpEnabled);
    feed_status(1, kStOpEnabled);
    run_cycle();  // axes 0 and 1 complete their release; the rest still walk
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire));
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).empty());

    feed_all(kStOpEnabled);
    feed_status(1, kStFault);
    run_cycle();  // the rest arrive; axis 1 has already dropped
    EXPECT_FALSE(flags_->all_drives_enabled.load(std::memory_order_acquire)) << "no all-OE sample ever existed";
    const auto lost = events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST);
    ASSERT_EQ(lost.size(), 1u);
    EXPECT_EQ(lost[0].joint_id, 1);
    EXPECT_TRUE(flags_->is_external_protective_stop());
}

TEST_F(BrakeGoldenTraceTest, ANormalSharedGroupEngageSettlingEarlyIsNotAnEnableLoss) {
    // A normal engage on a shared group: joint 0 settles early
    // (RTSO proves closed) while its partner is still powered, so the group
    // bit stays released. Its last sequence is an engage, not a release: no
    // loss, no stop, no forced engage on the other groups.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    arm_motion();
    release_all_and_enable();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);

    controller_->set_brake_engage_sequence(0x03);
    uint16_t st0 = kStOpEnabled;
    for (int cycle = 0; cycle < 20; ++cycle) {
        feed_all(kStOpEnabled);
        feed_status(0, st0);
        run_cycle();
        switch (controlword(0)) {
            case 0x0006:
                st0 = kStReady;
                break;
            case 0x0007:
                st0 = kStSwitchedOn;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00) << "partner still powers the shared brake open";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "joint 0's engage settled early";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).empty())
        << "a completed engage is not a lost release";
    EXPECT_FALSE(flags_->is_external_protective_stop());
    for (uint32_t j = 2; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x000F) << "other groups untouched";
    }

    // The partner walks down and the group flips engaged; still nothing.
    uint16_t st1 = kStOpEnabled;
    for (int cycle = 0; cycle < 20 && (controller_->brake_engaged() & 0x03) != 0x03; ++cycle) {
        feed_all(kStOpEnabled);
        feed_status(0, st0);
        feed_status(1, st1);
        run_cycle();
        switch (controlword(1)) {
            case 0x0006:
                st1 = kStReady;
                break;
            case 0x0007:
                st1 = kStSwitchedOn;
                break;
            default:
                break;
        }
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x03);
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).empty());
    EXPECT_FALSE(flags_->is_external_protective_stop());
}

TEST_F(BrakeGoldenTraceTest, EveryRecognizedWordOffOperationEnabledOnAReleasedAxisIsAnEnableLoss) {
    struct Row {
        uint16_t word;
        bool lost;
    };
    const Row rows[] = {
        {kStFaultReactionActive, true}, {kStQuickStopActive, true}, {kStSwitchedOn, true}, {kStReady, true},
        {kStSwitchOnDisabled, true},    {kStSodRestWord, true},     {kStNotReady, true},   {kStUnrecognized, false},
    };
    for (const auto& row : rows) {
        SCOPED_TRACE(fmt::format("word 0x{:04x}", row.word));
        build(six_servos(), 0x0A);
        arm_motion();
        release_all_and_enable();
        plem::fieldbus::test::ScopedMonitoringContext ctx;
        drain_events(ctx);
        feed_all_and_step(kStOpEnabled, 2, row.word);
        const auto lost = events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST);
        EXPECT_EQ(lost.size(), row.lost ? 1u : 0u);
        if (row.lost && !lost.empty()) {
            EXPECT_EQ(lost[0].joint_id, 2);
            EXPECT_EQ(lost[0].error_code, row.word);
        }
        EXPECT_EQ(flags_->is_external_protective_stop(), row.lost) << "an unrecognized word neither trips nor clears";
        plem::core::monitoring::set_thread_context(nullptr);
    }
}

TEST_F(BrakeGoldenTraceTest, ATerminalReleaseDroppingToSwitchOnDisabledIsReportedWhileColdAxesAreNot) {
    // The terminal-ADVANCE residual (sequence gone) is reported;
    // the five never-released axes resting at NOT_READY are not checked.
    arm_motion();
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    walk_release_sequence_and_pin();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);
    feed_all(kStNotReady);
    feed_status(0, kStSwitchOnDisabled);
    run_cycle();
    const auto lost = events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST);
    ASSERT_EQ(lost.size(), 1u);
    EXPECT_EQ(lost[0].joint_id, 0);
    EXPECT_EQ(lost[0].error_code, kStSwitchOnDisabled);
    EXPECT_TRUE(flags_->is_external_protective_stop());
}

TEST_F(BrakeGoldenTraceTest, AnEnableLossUnderAnActiveRecoveryStopStillForcesTheEngage) {
    // Under a recovery release (P-STOP level already active, no rising edge
    // left) the onset itself is the trigger, as for the mode trip.
    enter_recovery_released();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);
    feed_all_and_step(kStOpEnabled, 3, kStFault);
    ASSERT_EQ(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).size(), 1u);
    feed_all_and_step(kStOpEnabled, 3, kStFault);
    for (uint32_t j = 0; j < 6; ++j) {
        if (j == 3) {
            continue;
        }
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x0007) << "forced engage under the active level";
    }
    EXPECT_EQ(controller_->brake_operation_pending() & 0x08, 0x08);
}

TEST_F(BrakeGoldenTraceTest, AnEnableLossUnderFaultResetPolicyNoneHoldsTheForcedEngageWithoutAPulse) {
    // FaultResetPolicy::NONE: the forced engage is loaded but a FAULT
    // has no row to SWITCHED_ON and no auto-reset; the axis HOLDs on its
    // standing word (the operator's manual reset is the exit; this test pins
    // that no pulse and no disable command are invented).
    arm_motion();
    release_all_and_enable();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);
    feed_all_and_step(kStOpEnabled, 1, kStFault);
    ASSERT_EQ(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).size(), 1u);
    for (int cycle = 0; cycle < 30; ++cycle) {
        feed_all_and_step(kStOpEnabled, 1, kStFault);
        ASSERT_EQ(controlword(1), 0x000F) << "cycle " << cycle << ": standing word holds — no pulse, no disable";
    }
    EXPECT_EQ(controller_->brake_operation_pending() & 0x02, 0x02) << "the engage stays loaded (HOLD)";
    EXPECT_EQ(controlword(0), 0x0007) << "healthy axes were commanded down";
    EXPECT_TRUE(count_of(drain_events(ctx), plem::core::monitoring::EventType::SERVO_FAULT_RESET) == 0)
        << "NONE never requests a reset by itself";

    // The exit is the operator's manual reset: one pulse is published on the
    // faulted axis, and only the drive's recognized SOD answer lets the
    // engage settle and the group close.
    bus_->reset_all_faults();
    int pulses = 0;
    for (int cycle = 0; cycle < 4; ++cycle) {
        feed_all_and_step(kStOpEnabled, 1, kStFault);
        pulses += controlword(1) == 0x0080 ? 1 : 0;
    }
    EXPECT_EQ(pulses, 1) << "the manual request publishes exactly one rising edge";
    {
        const auto events = drain_events(ctx);
        const auto results = [&] {
            std::vector<plem::core::monitoring::Event> out;
            for (const auto& event : events) {
                if (event.type == plem::core::monitoring::EventType::SERVO_FAULT_RESET_RESULT) {
                    out.push_back(event);
                }
            }
            return out;
        }();
        // reset_all_faults() posts on every axis: the faulted one publishes
        // (value 1), the five healthy ones are dropped as not-in-FAULT (2).
        ASSERT_EQ(results.size(), 6u);
        std::vector<std::pair<int, int>> seen;
        for (const auto& result : results) {
            seen.emplace_back(result.joint_id, static_cast<int>(result.value));
        }
        std::sort(seen.begin(), seen.end());
        const std::vector<std::pair<int, int>> expected = {{0, 2}, {1, 1}, {2, 2}, {3, 2}, {4, 2}, {5, 2}};
        EXPECT_EQ(seen, expected) << "one result per axis: joint 1 PUBLISHED, the rest DROPPED_NOT_IN_FAULT";
    }
    EXPECT_EQ(controller_->brake_engaged() & 0x02, 0x00) << "the pulse alone closes nothing";
    feed_all(kStSwitchOnDisabled);
    run_cycle();
    run_cycle();
    EXPECT_EQ(controller_->brake_engaged(), 0x3F) << "the drive's SOD answer settles every group";
}

TEST_F(BrakeGoldenTraceTest, AReleaseCompletedAxisIsJudgedWhileItsSharedPartnerIsStillWalking) {
    // The shared-group direction of the axis-wise check.
    // Joint 0 completes its release (the group reads released from that
    // instant) while its partner is still walking; joint 0 then drops. It is
    // checked on its own; the partner's busy bit is not part of the predicate.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    arm_motion();
    feed_all(kStNotReady);
    run_cycle();
    ASSERT_TRUE(controller_->request_brake_release(0x3F));
    for (int cycle = 0; cycle < 3 && controlword(5) != 0x0006; ++cycle) {
        run_cycle();
    }
    feed_all(kStReady);
    run_cycle();
    run_cycle();
    feed_all(kStSwitchedOn);
    run_cycle();
    run_cycle();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);

    feed_all(kStSwitchedOn);
    feed_status(0, kStOpEnabled);
    run_cycle();  // joint 0 completes; joint 1 (its partner) and the rest still walk
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00) << "one enabled axis opens the shared brake";
    EXPECT_EQ(controller_->brake_operation_pending() & 0x02, 0x02) << "the partner is still walking";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).empty());

    feed_all(kStSwitchedOn);
    feed_status(0, kStFault);
    run_cycle();  // the completed axis drops while everything else still walks
    {
        const auto lost = events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST);
        ASSERT_EQ(lost.size(), 1u) << "the completed axis is checked alone";
        EXPECT_EQ(lost[0].joint_id, 0);
        EXPECT_EQ(lost[0].error_code, kStFault);
    }
    EXPECT_TRUE(flags_->is_external_protective_stop());
    feed_all(kStSwitchedOn);
    feed_status(0, kStFault);
    run_cycle();  // next brake step: the forced engage is loaded on every axis
    EXPECT_EQ(controller_->brake_operation_pending(), 0x3F) << "every axis walks an engage now";
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1);
}

TEST_F(BrakeGoldenTraceTest, ANormalSharedGroupEngageCompletingOnItsTerminalStepIsNotAnEnableLoss) {
    // The terminal-ADVANCE ending of a normal engage (the
    // drive walks SO → RTSO → SOD) while the partner stays powered. Same
    // verdict as the early settle: the axis's last sequence is an engage.
    controller_->set_brake_setup({0x03, 0x03, 0x0C, 0x0C, 0x30, 0x30});
    arm_motion();
    release_all_and_enable();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);

    controller_->set_brake_engage_sequence(0x03);
    const auto cycle_with = [&](uint16_t word) {
        feed_all(kStOpEnabled);
        feed_status(0, word);
        run_cycle();
        return controlword(0);
    };
    EXPECT_EQ(cycle_with(kStOpEnabled), 0x0007) << "command SWITCHED_ON";
    cycle_with(kStSwitchedOn);        // arrived: consumed
    cycle_with(kStSwitchedOn);        // command READY_TO_SWITCH_ON
    cycle_with(kStReady);             // arrived, consumed (target: SWITCH_ON_DISABLED)
    cycle_with(kStSwitchOnDisabled);  // arrived on the terminal step: terminal ADVANCE
    EXPECT_EQ(controller_->brake_operation_pending() & 0x01, 0x00) << "joint 0's engage ended on its terminal step";
    EXPECT_EQ(controller_->brake_engaged() & 0x03, 0x00) << "partner still powers the shared brake open";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).empty())
        << "a completed engage is not a lost release";
    EXPECT_FALSE(flags_->is_external_protective_stop());
    for (uint32_t j = 2; j < 6; ++j) {
        SCOPED_TRACE("joint " + std::to_string(j));
        EXPECT_EQ(controlword(j), 0x000F) << "other groups untouched";
    }
}

TEST_F(BrakeGoldenTraceTest, AnEnableLossUnderBoundedAutoResetPulsesAndSettlesOnlyAfterTheDriveAnswers) {
    // The bounded auto-reset policy. The loss loads the forced engage;
    // the faulted axis HOLDs until the pacing interval, then the request is
    // accepted and published (0x58/0x60: host domain, not the wire), and the
    // group closes only once the drive answers with a proven-closed word.
    controller_->set_fault_reset_policy(FaultResetPolicy::BOUNDED_AUTO_RESET, kGoldenPacing);
    arm_motion();
    release_all_and_enable();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);

    feed_all_and_step(kStOpEnabled, 1, kStFault);  // onset
    feed_all_and_step(kStOpEnabled, 1, kStFault);  // forced engage loaded
    drain_events(ctx);
    std::vector<int> highs;
    for (int cycle = 0; cycle < 450; ++cycle) {
        feed_all(kStSwitchOnDisabled);  // healthy axes answer proven-closed at once
        feed_status(1, kStFault);       // the faulted axis stays in FAULT
        run_cycle();
        const uint16_t cw = controlword(1);
        if (cw == 0x0080) {
            highs.push_back(cycle);
        } else if (!highs.empty() && highs.back() == cycle - 1) {
            EXPECT_EQ(cw, 0x0000) << "cycle " << cycle << ": the owed low follows every high";
        }
        EXPECT_EQ(controller_->brake_engaged() & 0x02, 0x00) << "cycle " << cycle << ": FAULT closes nothing";
    }
    ASSERT_EQ(highs.size(), 2u) << "one accepted request per 200 ms interval while the FAULT persists";
    EXPECT_GE(highs[0], 150);
    EXPECT_LE(highs[0], 250);
    EXPECT_EQ(highs[1] - highs[0], 200) << "the pacing interval between rising edges";
    EXPECT_EQ(controller_->brake_engaged() & 0x3D, 0x3D) << "the healthy singleton groups settled on their SOD";
    {
        const auto events = drain_events(ctx);
        std::vector<float> attempts;
        std::vector<float> outcomes;
        for (const auto& event : events) {
            if (event.type == plem::core::monitoring::EventType::SERVO_FAULT_RESET) {
                EXPECT_EQ(event.joint_id, 1);
                attempts.push_back(event.value);
            } else if (event.type == plem::core::monitoring::EventType::SERVO_FAULT_RESET_RESULT) {
                EXPECT_EQ(event.joint_id, 1);
                outcomes.push_back(event.value);
            }
        }
        EXPECT_EQ(attempts, (std::vector<float>{1.0f, 2.0f})) << "attempt ordinals within the episode";
        EXPECT_EQ(outcomes, (std::vector<float>{1.0f, 1.0f})) << "both PUBLISHED";
        EXPECT_EQ(count_of(events, plem::core::monitoring::EventType::SERVO_FAULT_RESET_EXHAUSTED), 0);
        EXPECT_TRUE(events_of(events, EventType::OPERATION_ENABLED_LOST).empty())
            << "no second diagnostic while the engage walks";
    }
    feed_all(kStSwitchOnDisabled);  // the drive answers
    run_cycle();
    run_cycle();
    EXPECT_EQ(controller_->brake_engaged(), 0x3F) << "settled only after the recognized SOD answer";
}

TEST_F(BrakeGoldenTraceTest, AQuickStopActiveLossHoldsTheForcedEngageAndStalls) {
    // QUICK_STOP_ACTIVE is a loss, but the forced engage
    // it starts has no row from QSA and no proven-closed evidence, so it HOLDs
    // and the stall window names it. No recovery is invented.
    arm_motion();
    release_all_and_enable();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);
    controller_->set_sequence_stall_threshold_for_test(20);
    feed_all_and_step(kStOpEnabled, 2, kStQuickStopActive);
    ASSERT_EQ(events_of(drain_events(ctx), EventType::OPERATION_ENABLED_LOST).size(), 1u);
    for (int cycle = 0; cycle < 30; ++cycle) {
        feed_all_and_step(kStOpEnabled, 2, kStQuickStopActive);
        ASSERT_EQ(controlword(2), 0x000F) << "cycle " << cycle << ": standing word holds — nothing is commanded";
    }
    EXPECT_EQ(controller_->brake_operation_pending() & 0x04, 0x04) << "the engage stays loaded (HOLD)";
    const auto events = drain_events(ctx);
    int stalled_on_axis_2 = 0;
    for (const auto& event : events) {
        if (event.type == plem::core::monitoring::EventType::BRAKE_SEQUENCE_STALLED && event.joint_id == 2) {
            ++stalled_on_axis_2;
        }
    }
    EXPECT_EQ(stalled_on_axis_2, 1) << "the stall window names the held axis once";
}

TEST_F(BrakeGoldenTraceTest, MixedCausesOnOneCycleNameEachAxisOnceAndShareTheEpisode) {
    // The two causes are one episode. On one cycle axis 1
    // drops to FAULT (display still accepted) and axis 2 stays enabled but
    // shows CSP: one diagnostic each, by cause, one trigger. Then, while
    // axis 1's forced engage walks, its word flaps to unrecognized and to an
    // enabled-but-mismatching sample: the open episode absorbs both.
    arm_motion();
    release_all_and_enable();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    drain_events(ctx);

    feed_display(2, 0x08);
    feed_all(kStOpEnabled);
    feed_status(1, kStFault);
    run_cycle();
    {
        const auto events = drain_events(ctx);
        const auto lost = events_of(events, EventType::OPERATION_ENABLED_LOST);
        const auto modes = events_of(events, EventType::MODES_OF_OPERATION_MISMATCH);
        ASSERT_EQ(lost.size(), 1u);
        EXPECT_EQ(lost[0].joint_id, 1);
        EXPECT_FLOAT_EQ(lost[0].value, 0.0f);
        ASSERT_EQ(modes.size(), 1u);
        EXPECT_EQ(modes[0].joint_id, 2);
        EXPECT_FLOAT_EQ(modes[0].value, 8.0f);
    }
    feed_all(kStOpEnabled);
    feed_status(1, kStFault);
    run_cycle();  // one forced engage for both onsets
    EXPECT_EQ(count_of(drain_events(ctx), plem::core::monitoring::EventType::PROTECTIVE_STOP), 1) << "one trigger";
    EXPECT_EQ(controller_->brake_operation_pending(), 0x3F);

    feed_all(kStOpEnabled);
    feed_status(1, kStUnrecognized);
    run_cycle();
    feed_all(kStOpEnabled);
    feed_status(1, kStOpEnabled);
    feed_display(1, 0x08);
    run_cycle();
    {
        const auto events = drain_events(ctx);
        EXPECT_TRUE(events_of(events, EventType::OPERATION_ENABLED_LOST).empty())
            << "no second loss diagnostic under the open episode";
        EXPECT_TRUE(events_of(events, EventType::MODES_OF_OPERATION_MISMATCH).empty())
            << "a cause change under the open episode is not a new onset";
    }
    EXPECT_TRUE(flags_->is_external_protective_stop()) << "the level follows the standing condition";
}

// The slave must be operational (in OP with this configuration)
// In SAFEOP a slave's inputs read valid while the drive ignores the outputs, so
// a statusword and display that pass the gate prove nothing about the drive
// executing the release. The gate reads the master's slave-configuration
// state (sampled once per slave for every consumed release request); a
// not-operational axis is refused in both admission branches and named with
// SLAVE_NOT_OPERATIONAL (error_code = the AL state, 0 = offline or the query failed;
// value = the slave's ring position) instead of a mode refusal.

namespace {

constexpr unsigned int kAlSafeOp = 4;
constexpr unsigned int kAlOp = 8;

}  // namespace

TEST_F(BrakeGoldenTraceTest, ASafeOpSlaveIsRefusedAndNamedThenAdmittedInOp) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    FakeEcrt::set_slave_config_state(0, kAlSafeOp);

    // Statusword and display pass the gate; the slave state alone refuses.
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000) << "no sequence while the slave is not operational";
    const auto events = drain_events(ctx);
    EXPECT_TRUE(events_of(events, EventType::MODES_OF_OPERATION_MISMATCH).empty())
        << "the cause is the slave state, not the mode";
    const auto named = events_of(events, EventType::SLAVE_NOT_OPERATIONAL);
    ASSERT_EQ(named.size(), 1u);
    EXPECT_EQ(named[0].joint_id, 0);
    EXPECT_EQ(named[0].error_code, kAlSafeOp);
    EXPECT_FLOAT_EQ(named[0].value, 0.0f) << "ring position 0";

    FakeEcrt::set_slave_config_state(0, kAlOp);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0006) << "operational: the release starts";
    EXPECT_TRUE(events_of(drain_events(ctx), EventType::SLAVE_NOT_OPERATIONAL).empty());
}

TEST_F(BrakeGoldenTraceTest, APendingReconfigurationInOpIsNotOperational) {
    // AL state OP with force_config pending: the master has not brought the
    // slave into OP with this configuration yet.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    FakeEcrt::set_slave_config_state(0, kAlOp, /*force_config=*/true);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000);
    const auto named = events_of(drain_events(ctx), EventType::SLAVE_NOT_OPERATIONAL);
    ASSERT_EQ(named.size(), 1u);
    EXPECT_EQ(named[0].error_code, kAlOp);
}

TEST_F(BrakeGoldenTraceTest, AFailedStateQueryAfterAnOpSampleRefuses) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    ASSERT_TRUE(controller_->request_brake_release(0x02));
    feed_status(1, kStNotReady);
    run_cycle();  // sampled every slave: operational
    ASSERT_EQ(controlword(1), 0x0006);
    FakeEcrt::set_slave_config_state_failure(0);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000) << "an unanswered query is not operational";
    const auto named = events_of(drain_events(ctx), EventType::SLAVE_NOT_OPERATIONAL);
    ASSERT_EQ(named.size(), 1u);
    EXPECT_EQ(named[0].joint_id, 0);
    EXPECT_EQ(named[0].error_code, 0) << "0 = the state could not be read";
}

TEST_F(BrakeGoldenTraceTest, AnOfflineSlaveIsRefusedWithStateZero) {
    // A slave that is not on the bus: the master answers the query with
    // online 0 and AL state 0: not operational, named with 0.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    FakeEcrt::set_slave_config_state(0, 0);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0000);
    const auto named = events_of(drain_events(ctx), EventType::SLAVE_NOT_OPERATIONAL);
    ASSERT_EQ(named.size(), 1u);
    EXPECT_EQ(named[0].error_code, 0) << "0 = offline, or the state could not be read";
}

TEST_F(BrakeGoldenTraceTest, TheSlaveStateIsSampledForAConsumedReleaseOnly) {
    for (int cycle = 0; cycle < 250; ++cycle) {
        run_cycle();
    }
    const auto samples = [] {
        int n = 0;
        for (const auto& call : FakeEcrt::calls()) {
            n += call == "slave_config_state(pos=0)" ? 1 : 0;
        }
        return n;
    };
    EXPECT_EQ(samples(), 0) << "no release, no query";
    arm_motion();
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    run_cycle();
    EXPECT_EQ(samples(), 1) << "one query per slave for the consumed request";
    run_cycle();
    EXPECT_EQ(samples(), 1);
}

TEST_F(BrakeGoldenTraceTest, AReleasePostedAfterTheStepBeganIsJudgedOnAFreshSample) {
    // The request lands between the start of the brake step and the mask
    // consume: it must be checked on a sample taken for it, not on the one
    // taken for an earlier request.
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    ASSERT_EQ(step(kStNotReady).first, 0x0006) << "sampled: every slave operational";
    for (int cycle = 0; cycle < 10; ++cycle) {
        run_cycle();
    }
    drain_events(ctx);
    FakeEcrt::set_slave_config_state(1, kAlSafeOp);
    bool posted = false;
    controller_->set_brake_step_entry_hook_for_test([&] {
        if (!posted) {
            posted = true;
            EXPECT_TRUE(controller_->request_brake_release(0x02));
        }
    });
    feed_status(1, kStNotReady);
    run_cycle();
    ASSERT_TRUE(posted);
    EXPECT_EQ(controlword(1), 0x0000) << "checked on the sample taken for this request";
    const auto named = events_of(drain_events(ctx), EventType::SLAVE_NOT_OPERATIONAL);
    ASSERT_EQ(named.size(), 1u);
    EXPECT_EQ(named[0].joint_id, 1);
    EXPECT_EQ(named[0].error_code, kAlSafeOp);
}

TEST_F(ColdPostEnableTraceTest, ASafeOpSlaveIsRefusedUnderTheZeroExceptionToo) {
    arm_motion();
    plem::fieldbus::test::ScopedMonitoringContext ctx;
    FakeEcrt::set_slave_config_state(0, kAlSafeOp);
    feed_all_and_step(kStReady);
    ASSERT_TRUE(controller_->request_brake_release(0x01));
    feed_all_and_step(kStReady);
    feed_all_and_step(kStReady);
    EXPECT_EQ(controlword(0), 0x0000) << "the cold-zero admission also needs an operational slave";
    const auto named = events_of(drain_events(ctx), EventType::SLAVE_NOT_OPERATIONAL);
    ASSERT_EQ(named.size(), 1u);
    EXPECT_EQ(named[0].joint_id, 0);
    EXPECT_EQ(named[0].error_code, kAlSafeOp);
}
}  // namespace
