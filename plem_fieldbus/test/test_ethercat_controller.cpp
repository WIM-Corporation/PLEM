// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include <plem/fieldbus/ethercat/controller.hpp>

using namespace plem::fieldbus;
using namespace plem::fieldbus::ethercat;

class EtherCATControllerTest : public ::testing::Test {
  protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(EtherCATControllerTest, StartWithoutBusReturnsError) {
    EtherCATController<6>::Config config;
    EtherCATController<6> controller(config);

    // Should return error when starting without bus configured
    auto result = controller.start();
    EXPECT_FALSE(result);
    EXPECT_NE(result.message().find("bus not configured"), std::string::npos);
}

TEST_F(EtherCATControllerTest, StopWithoutStartIsNoOp) {
    EtherCATController<6>::Config config;
    EtherCATController<6> controller(config);

    EXPECT_NO_THROW(controller.stop());
    EXPECT_FALSE(controller.is_running());
}

// WKC Watchdog → protective-stop wiring
//
// Requirement: WKC degradation beyond the watchdog threshold must reach the
// live safety surface (ControlModeFlags EXTERNAL protective stop). The EXTERNAL latch must survive bus recovery until
// an operator reset, and an optimistic reset while the bus is still degraded must be re-asserted on the next cycle.
//
// run_watchdog_cycle() is the exact member run_loop() calls once per cycle.

#include <plem/core/types/control_mode_flags.hpp>

namespace {

class WatchdogBridgeController : public EtherCATController<6> {
  public:
    using EtherCATController<6>::EtherCATController;
    using EtherCATController<6>::run_watchdog_cycle;
};

}  // namespace

class WatchdogBridgeTest : public ::testing::Test {
  protected:
    void SetUp() override {
        EtherCATController<6>::Config config;  // watchdog defaults: 5 ms (= 5 cycles @ 1 kHz), P_STOP
        controller_ = std::make_unique<WatchdogBridgeController>(config);
        flags_ = std::make_shared<plem::core::ControlModeFlags>();
        controller_->set_mode_flags(flags_);
    }

    void run_cycles(size_t n, bool wkc_ok) {
        for (size_t i = 0; i < n; ++i) {
            controller_->run_watchdog_cycle(wkc_ok);
        }
    }

    /// The watchdog only counts mismatches after the first healthy cycle
    /// (boot-arming: bring-up WKC noise must not latch). Degradation
    /// scenarios start from an armed, in-operation bus.
    void arm_watchdog() {
        run_cycles(1, /*wkc_ok=*/true);
    }

    static constexpr size_t kThreshold = 5;  // Config default 5 ms -> 5 cycles at the 1 kHz default period
    std::unique_ptr<WatchdogBridgeController> controller_;
    std::shared_ptr<plem::core::ControlModeFlags> flags_;
};

TEST_F(WatchdogBridgeTest, BootMismatchesBeforeFirstHealthyCycleDoNotLatch) {
    // WKC mismatches while the slaves transition to OP must not reach the
    // safety surface; degradation after the first healthy cycle must.
    run_cycles(kThreshold * 4, /*wkc_ok=*/false);
    EXPECT_FALSE(flags_->is_external_protective_stop());

    arm_watchdog();
    run_cycles(kThreshold, /*wkc_ok=*/false);
    EXPECT_TRUE(flags_->is_external_protective_stop());
}

TEST_F(WatchdogBridgeTest, LatchesExternalProtectiveStopWhenWkcDegradedBeyondThreshold) {
    arm_watchdog();
    run_cycles(kThreshold - 1, /*wkc_ok=*/false);
    EXPECT_FALSE(flags_->is_external_protective_stop());

    run_cycles(1, /*wkc_ok=*/false);
    EXPECT_TRUE(flags_->is_external_protective_stop());
    EXPECT_FALSE(flags_->is_internal_protective_stop());  // fieldbus uses the EXTERNAL source
}

TEST_F(WatchdogBridgeTest, ReassertsExternalProtectiveStopAfterOptimisticResetWhileStillDegraded) {
    arm_watchdog();
    run_cycles(kThreshold, /*wkc_ok=*/false);
    ASSERT_TRUE(flags_->is_external_protective_stop());

    // Operator reset clears optimistically…
    flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    // …but the bus is still degraded: the next cycle must re-latch.
    run_cycles(1, /*wkc_ok=*/false);
    EXPECT_TRUE(flags_->is_external_protective_stop());
}

TEST_F(WatchdogBridgeTest, KeepsExternalProtectiveStopLatchedAfterBusRecoveryUntilReset) {
    arm_watchdog();
    run_cycles(kThreshold, /*wkc_ok=*/false);
    run_cycles(3, /*wkc_ok=*/true);                      // bus recovered
    EXPECT_TRUE(flags_->is_external_protective_stop());  // latched until operator reset

    flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    run_cycles(3, /*wkc_ok=*/true);
    EXPECT_FALSE(flags_->is_external_protective_stop());  // reset sticks once healthy
}

TEST(EtherCATControllerConfig, RejectsNonPositiveCyclePeriodAtConstruction) {
    // The ctor derives cycle counts from the period, so a
    // non-positive period must fail loudly here, not as divide-by-zero UB
    // (PeriodicTimerRT's own check only runs later, at start()).
    EtherCATController<6>::Config config;
    config.cycle_time_us = 0;
    EXPECT_THROW(EtherCATController<6>{config}, std::invalid_argument);
    config.cycle_time_us = -1000;
    EXPECT_THROW(EtherCATController<6>{config}, std::invalid_argument);
}

TEST_F(WatchdogBridgeTest, WkcThresholdIsTimeBasedAcrossBusPeriods) {
    // The WKC debounce tolerates a sustained-loss duration
    // (stale-torque exposure scales with time, not frame count). At a 200 µs
    // bus period the same 5 ms threshold must take 25 consecutive mismatches;
    // a raw count of 5 would silently tighten the debounce to 1 ms.
    EtherCATController<6>::Config config;
    config.cycle_time_us = 200;  // 5 kHz
    WatchdogBridgeController controller(config);
    auto flags = std::make_shared<plem::core::ControlModeFlags>();
    controller.set_mode_flags(flags);

    controller.run_watchdog_cycle(/*wkc_ok=*/true);  // first healthy WKC: the watchdog starts watching
    for (int i = 0; i < 24; ++i) {
        controller.run_watchdog_cycle(/*wkc_ok=*/false);
    }
    EXPECT_FALSE(flags->is_external_protective_stop());  // 24 x 200 µs = 4.8 ms < 5 ms

    controller.run_watchdog_cycle(/*wkc_ok=*/false);
    EXPECT_TRUE(flags->is_external_protective_stop());  // 25 x 200 µs = 5 ms
}

TEST_F(WatchdogBridgeTest, StaysTotalWhenCalledDirectlyBeforeModeFlagsWiring) {
    // start() fail-fasts without mode flags (validate_preconditions), so run_loop
    // never sees a null. Direct unit calls on an unwired instance must still be
    // total: no UB on the safety path.
    EtherCATController<6>::Config config;
    WatchdogBridgeController bare(config);
    bare.run_watchdog_cycle(/*wkc_ok=*/true);  // starts watching (boot-arming gate)
    for (size_t i = 0; i < kThreshold + 1; ++i) {
        bare.run_watchdog_cycle(/*wkc_ok=*/false);  // must not crash
    }
    SUCCEED();
}

TEST_F(WatchdogBridgeTest, StartFailsFastWithoutModeFlags) {
    // ControlModeFlags is the live safety surface: a controller
    // started without it would run the watchdog with nowhere to latch EXTERNAL.
    EtherCATController<6>::Config config;
    EtherCATController<6> controller(config);
    controller.set_bus(std::make_shared<EtherCATBus>());  // ctor only stores the master index

    auto result = controller.start();
    ASSERT_FALSE(result);
    EXPECT_NE(result.message().find("mode flags"), std::string::npos);
}

// Command-freshness → EXTERNAL protective-stop bridge
//
// Requirement: a hung/starved control thread (stale torque stamp) must reach
// the live safety surface exactly like WKC degradation: EXTERNAL latch,
// re-assert across an optimistic reset while still stale, clear sticks only
// once the writer is stamping again. run_command_freshness_cycle() is the exact
// member run_loop() calls once per cycle on the shared_data plane.

namespace {

class CommandFreshnessBridgeController : public EtherCATController<6> {
  public:
    using EtherCATController<6>::EtherCATController;
    using EtherCATController<6>::run_command_freshness_cycle;
};

}  // namespace

class CommandFreshnessBridgeTest : public ::testing::Test {
  protected:
    void SetUp() override {
        EtherCATController<6>::Config config;
        controller_ = std::make_unique<CommandFreshnessBridgeController>(config);
        flags_ = std::make_shared<plem::core::ControlModeFlags>();
        controller_->set_mode_flags(flags_);
    }

    static constexpr int64_t kBudget = kDefaultCommandStaleTimeoutNs;
    std::unique_ptr<CommandFreshnessBridgeController> controller_;
    std::shared_ptr<plem::core::ControlModeFlags> flags_;
};

TEST_F(CommandFreshnessBridgeTest, UnstampedBootDoesNotLatch) {
    // The writer has never stamped (update_ns 0): bring-up must not latch no
    // matter how long the loop has been running (mirrors the WKC arming gate).
    for (int i = 0; i < 50; ++i) {
        controller_->run_command_freshness_cycle(/*update_ns=*/0, /*now_ns=*/(i + 1) * kBudget);
    }
    EXPECT_FALSE(flags_->is_external_protective_stop());
}

TEST_F(CommandFreshnessBridgeTest, LatchesExternalProtectiveStopWhenStampGoesStale) {
    controller_->run_command_freshness_cycle(/*update_ns=*/1, /*now_ns=*/2);  // armed + fresh
    EXPECT_FALSE(flags_->is_external_protective_stop());

    controller_->run_command_freshness_cycle(/*update_ns=*/1, /*now_ns=*/2 + kBudget);  // stale
    EXPECT_TRUE(flags_->is_external_protective_stop());
    EXPECT_FALSE(flags_->is_internal_protective_stop());  // fieldbus uses the EXTERNAL source
}

TEST_F(CommandFreshnessBridgeTest, ReassertsExternalProtectiveStopAfterOptimisticResetWhileStillStale) {
    controller_->run_command_freshness_cycle(1, 2);
    controller_->run_command_freshness_cycle(1, 2 + kBudget);
    ASSERT_TRUE(flags_->is_external_protective_stop());

    // Operator reset clears optimistically…
    flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    // …but the writer is still hung: the next cycle must re-latch.
    controller_->run_command_freshness_cycle(1, 3 + 2 * kBudget);
    EXPECT_TRUE(flags_->is_external_protective_stop());
}

TEST_F(CommandFreshnessBridgeTest, ResetSticksOnceWriterResumesStamping) {
    controller_->run_command_freshness_cycle(1, 2);
    controller_->run_command_freshness_cycle(1, 2 + kBudget);
    ASSERT_TRUE(flags_->is_external_protective_stop());

    // Writer resumed (fresh stamp): the latch persists until reset…
    controller_->run_command_freshness_cycle(3 + kBudget, 4 + kBudget);
    EXPECT_TRUE(flags_->is_external_protective_stop());

    // …and a reset now sticks.
    flags_->clear_protective_stop(plem::core::ProtectiveStopSource::EXTERNAL);
    controller_->run_command_freshness_cycle(5 + kBudget, 6 + kBudget);
    EXPECT_FALSE(flags_->is_external_protective_stop());
}

TEST_F(CommandFreshnessBridgeTest, AHostSetBudgetReplacesTheDefault) {
    // The drive layer's 50 ms (plem::drive): a stamp 50 ms old is fresh, 1 ns more is stale.
    controller_->set_command_stale_timeout_ns(50'000'000);
    controller_->run_command_freshness_cycle(1, 1 + 50'000'000);
    EXPECT_FALSE(flags_->is_external_protective_stop());
    controller_->run_command_freshness_cycle(1, 2 + 50'000'000);
    EXPECT_TRUE(flags_->is_external_protective_stop());
}

TEST_F(CommandFreshnessBridgeTest, StaysTotalWhenCalledDirectlyBeforeModeFlagsWiring) {
    // Unwired instance: must not crash (same rule as the WKC case above).
    EtherCATController<6>::Config config;
    CommandFreshnessBridgeController bare(config);
    bare.run_command_freshness_cycle(1, 2 + kBudget);
    SUCCEED();
}
