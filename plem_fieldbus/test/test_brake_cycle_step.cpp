// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_brake_cycle_step.cpp
/// @brief Shell-step ordering pins for run_brake_cycle_step.
///
/// The fake-master harness cannot inject a flag toggle between the two
/// safety samples of one cycle deterministically; this suite can: scripted
/// fakes log every interaction (flag samples, mask exchanges, effect
/// applications) into one ordered log, so any reordering mutation (single-
/// sample collapse, exchange before decide/apply, engage-before-release
/// consume, re-queue introduction) breaks an exact-sequence assertion.

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <utility>
#include <vector>

#include "ethercat/brake_cycle_step.hpp"

using plem::core::TriState;
using plem::fieldbus::ethercat::BrakeCycleInputs;
using plem::fieldbus::ethercat::BrakeEffect;
using plem::fieldbus::ethercat::BrakeEffectKind;
using plem::fieldbus::ethercat::BrakeLogicState;
using plem::fieldbus::ethercat::BrakeRequestMaskRefs;
using plem::fieldbus::ethercat::CommittedModeSample;
using plem::fieldbus::ethercat::run_brake_cycle_step;

namespace {

constexpr uint8_t kAll = 0x3F;

std::string effect_name(const BrakeEffect& effect) {
    switch (effect.kind) {
        case BrakeEffectKind::START_ENGAGE:
            return "apply:START_ENGAGE(" + std::to_string(effect.mask) + ")";
        case BrakeEffectKind::START_RELEASE:
            return "apply:START_RELEASE(" + std::to_string(effect.mask) + ")";
        case BrakeEffectKind::EMIT_BRAKE_ENGAGE_ONLY:
            return "apply:EMIT_BRAKE_ENGAGE_ONLY";
        case BrakeEffectKind::EMIT_PROTECTIVE_STOP:
            return "apply:EMIT_PROTECTIVE_STOP";
        case BrakeEffectKind::EMIT_PROTECTIVE_STOP_CLEAR:
            return "apply:EMIT_PROTECTIVE_STOP_CLEAR";
        case BrakeEffectKind::EMIT_RELEASE_REFUSED:
            return "apply:EMIT_RELEASE_REFUSED(" + std::to_string(effect.mask) + ")";
        case BrakeEffectKind::START_FORCED_ENGAGE:
            return "apply:START_FORCED_ENGAGE(" + std::to_string(effect.mask) + ")";
    }
    return "apply:?";
}

/// FlagsLike fake: scripted per-call values so a toggle lands exactly between
/// the two samples of one step; every sample is appended to the shared log.
struct ScriptedFlags {
    std::vector<std::string>* log{nullptr};
    bool protective_stop_first{false};
    bool protective_stop_second{false};
    size_t protective_stop_calls{0};

    bool is_protective_stop_active() {
        ++protective_stop_calls;
        log->push_back("protective_stop.sample#" + std::to_string(protective_stop_calls));
        return (protective_stop_calls <= 1) ? protective_stop_first : protective_stop_second;
    }
};

/// ModeLike fake: the committed-mode authority sample as a
/// scripted callable: one call yields both bits (mirroring the production
/// single-snapshot probe). Motion true / recovery false by default. Every
/// call is logged, so an extra sample, or one taken before the exchange,
/// breaks the exact-sequence pins.
struct ScriptedMode {
    std::vector<std::string>* log{nullptr};
    bool motion{true};
    bool protective_stop_recovery{false};

    CommittedModeSample operator()() {
        log->push_back("mode.sample");
        return {motion, protective_stop_recovery};
    }
};

/// MasksLike fake: logs exchange (and any re-queue write) with its slot name.
struct ScriptedMaskSlot {
    std::vector<std::string>* log{nullptr};
    const char* name{""};
    uint8_t value{0};

    uint8_t load(std::memory_order) {
        log->push_back(std::string(name) + ".load");
        return value;
    }

    uint8_t fetch_and(uint8_t bits, std::memory_order) {
        log->push_back(std::string(name) + ".fetch_and");
        const uint8_t old = value;
        value &= bits;
        return old;
    }

    // Re-queue detector: the production mutation "push the suppressed release
    // back" would have to write through the mask, and any such write is logged.
    uint8_t fetch_or(uint8_t bits, std::memory_order) {
        log->push_back(std::string(name) + ".fetch_or");
        const uint8_t old = value;
        value |= bits;
        return old;
    }
};

struct ScriptedMasks {
    ScriptedMaskSlot release;
    ScriptedMaskSlot engage;
    ScriptedMaskSlot busy;
};

// RT contract pin: the cycle-step shell is unconditionally noexcept:
// a throwing ApplyLike terminates instead of unwinding the RT loop.
static_assert(noexcept(run_brake_cycle_step(std::declval<const BrakeCycleInputs&>(), std::declval<ScriptedFlags&>(),
                                            std::declval<CommittedModeSample (*)() noexcept>(),
                                            std::declval<uint8_t (*)(uint8_t) noexcept>(),
                                            std::declval<ScriptedMasks&>(), std::declval<const BrakeLogicState&>(),
                                            uint8_t{0x3F}, std::declval<void (*)(const BrakeEffect&)>())));

class BrakeCycleStepTest : public ::testing::Test {
  protected:
    void SetUp() override {
        flags_.log = &log_;
        motion_.log = &log_;
        masks_.release = {&log_, "release", 0x00};
        masks_.engage = {&log_, "engage", 0x00};
        masks_.busy = {&log_, "busy", 0x00};
    }

    BrakeLogicState step(const BrakeCycleInputs& inputs, const BrakeLogicState& state, uint8_t ready = kAll) {
        return run_brake_cycle_step(
            inputs, flags_, motion_, [ready](uint8_t) noexcept { return ready; }, masks_, state, kAll,
            [&](const BrakeEffect& effect) { log_.push_back(effect_name(effect)); });
    }

    std::vector<std::string> log_;
    ScriptedFlags flags_;
    ScriptedMode motion_;
    ScriptedMasks masks_;
};

}  // namespace

TEST_F(BrakeCycleStepTest, QuiescentStepPinsSampleExchangeSampleOrder) {
    // No effects on either leg: the log is the bare interaction skeleton:
    // two samples with the release-then-engage exchange strictly between.
    (void)step({.motion_possible = TriState::YES}, {});

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample"};
    EXPECT_EQ(log_, expected);
}

TEST_F(BrakeCycleStepTest, ControlEffectsApplyBeforeMaskExchange) {
    // Rising P-STOP edge: the safety-forced effects must land before the
    // direct requests are consumed (a same-cycle direct engage overwrites the
    // forced sequence, never the reverse).
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = true;

    const auto next = step({.motion_possible = TriState::YES}, {});

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "apply:START_FORCED_ENGAGE(" + std::to_string(kAll) + ")",
                                               "apply:EMIT_PROTECTIVE_STOP",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample"};
    EXPECT_EQ(log_, expected);
    EXPECT_TRUE(next.engage_pending);
}

TEST_F(BrakeCycleStepTest, StopRisingBetweenSamplesSuppressesDirectRelease) {
    // First sample clean, second sample stopped: the direct leg must use the
    // second sample; a single-snapshot collapse would release brakes into a
    // protective stop that arrived mid-cycle.
    flags_.protective_stop_first = false;
    flags_.protective_stop_second = true;
    masks_.release.value = kAll;

    (void)step({.motion_possible = TriState::YES}, {});

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample"};
    EXPECT_EQ(log_, expected) << "no START_RELEASE may appear — the mid-cycle stop suppresses it";
    EXPECT_EQ(masks_.release.value, 0x00) << "the suppressed request must still be consumed";
}

TEST_F(BrakeCycleStepTest, StopClearingBetweenSamplesAllowsDirectRelease) {
    // The mirror toggle: first sample stopped (control leg suppressed),
    // second sample clear: the direct release goes through. Both toggle
    // directions together pin the two-sample semantic exactly.
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = false;
    masks_.release.value = 0x01;
    // last_protective_stop_active=true so the first sample is a level, not an edge.
    const BrakeLogicState state{true, false};

    (void)step({.motion_possible = TriState::YES}, state);

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:START_RELEASE(1)"};
    EXPECT_EQ(log_, expected);
}

TEST_F(BrakeCycleStepTest, SuppressedReleaseIsConsumedNeverRequeued) {
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = true;
    masks_.release.value = kAll;
    const BrakeLogicState state{true, false};

    (void)step({.motion_possible = TriState::YES}, state);

    EXPECT_EQ(masks_.release.value, 0x00) << "consume must clear the mask unconditionally";
    for (const auto& entry : log_) {
        EXPECT_TRUE(entry.find("release.fetch_or") == std::string::npos &&
                    entry.find("engage.fetch_or") == std::string::npos)
            << "a suppressed release was written back into a mask: " << entry;
    }
}

TEST_F(BrakeCycleStepTest, DirectLegUsesSameCyclePendingFromControlDecision) {
    // The rising edge this cycle sets engage_pending in the control
    // decision's next_state; the direct leg must consume that same-cycle
    // value (not the stale pre-step state) and suppress the release even
    // though the committed recovery lane would otherwise allow it.
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = true;
    motion_.protective_stop_recovery = true;
    masks_.release.value = kAll;

    const auto next = step({.motion_possible = TriState::YES}, {});

    EXPECT_TRUE(next.engage_pending);
    for (const auto& entry : log_) {
        EXPECT_EQ(entry.find("START_RELEASE"), std::string::npos)
            << "direct release ran against a same-cycle P-STOP engage: " << entry;
        EXPECT_TRUE(entry.find("release.fetch_or") == std::string::npos &&
                    entry.find("engage.fetch_or") == std::string::npos)
            << "the suppressed same-cycle release was re-queued: " << entry;
    }
    EXPECT_EQ(masks_.release.value, 0x00) << "the suppressed request must still be consumed";
}

TEST_F(BrakeCycleStepTest, DirectLegUsesSameCyclePendingResolutionFromControlDecision) {
    // The mirror of the same-cycle rule: the forced engage resolves this
    // cycle (motion no longer possible), so the control decision's
    // next_state clears engage_pending, and the request leg, reading that
    // same-cycle value, admits the recovery release queued for this cycle.
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = true;
    motion_.protective_stop_recovery = true;
    masks_.release.value = kAll;
    const BrakeLogicState state{true, true};

    const auto next = step({.motion_possible = TriState::NO}, state);

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "apply:EMIT_BRAKE_ENGAGE_ONLY",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:START_RELEASE(" + std::to_string(kAll) + ")"};
    EXPECT_EQ(log_, expected);
    EXPECT_FALSE(next.engage_pending);
}

TEST_F(BrakeCycleStepTest, RecoveryLaneAtTheRequestSampleAllowsRequestedRelease) {
    // P-STOP level held on both samples; the committed recovery lane, read
    // by the single mode sample after the exchange, admits the release.
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = true;
    motion_.protective_stop_recovery = true;
    masks_.release.value = 0x01;
    const BrakeLogicState state{true, false};

    (void)step({.motion_possible = TriState::YES}, state);

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:START_RELEASE(1)"};
    EXPECT_EQ(log_, expected);
}

TEST_F(BrakeCycleStepTest, MotionDisabledAtTheRequestSampleSuppressesRequestedRelease) {
    // The safety-forced BRAKED interleave: a BRAKED commit landing any time
    // before the exchange is seen by the mode sample taken after it: the
    // request leg suppresses the release (enabling a zero-torque drive would
    // open its brake unheld).
    motion_.motion = false;
    masks_.release.value = kAll;

    (void)step({.motion_possible = TriState::YES}, {});

    for (const auto& entry : log_) {
        EXPECT_EQ(entry.find("apply:START_RELEASE"), std::string::npos)
            << "no release may start after a mid-cycle BRAKED commit: " << entry;
    }
    EXPECT_EQ(masks_.release.value, 0x00) << "the suppressed request must still be consumed (discarded)";
}

TEST_F(BrakeCycleStepTest, RisingEdgeWithQueuedDirectEngagePreservesFullTrace) {
    // Both legs fire in one cycle: the forced engage lands before the
    // consume, the direct engage after the second sample; duplicate engage
    // sequences are the contract (ordered effects, never OR-merged).
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = true;
    masks_.engage.value = 0x05;

    (void)step({.motion_possible = TriState::YES}, {});

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "apply:START_FORCED_ENGAGE(" + std::to_string(kAll) + ")",
                                               "apply:EMIT_PROTECTIVE_STOP",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:START_ENGAGE(5)"};
    EXPECT_EQ(log_, expected);
}

TEST_F(BrakeCycleStepTest, DirectEngageAppliesAfterSecondSample) {
    masks_.engage.value = 0x05;

    (void)step({.motion_possible = TriState::YES}, {});

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:START_ENGAGE(5)"};
    EXPECT_EQ(log_, expected);
}

TEST_F(BrakeCycleStepTest, ProductionMaskRefsConsumeRealAtomics) {
    // BrakeRequestMaskRefs is the production MasksLike instantiation: verify
    // the exchange semantics against real std::atomic<uint8_t> members.
    std::atomic<uint8_t> release{0x03};
    std::atomic<uint8_t> engage{0x00};
    std::atomic<uint8_t> busy{0x00};
    BrakeRequestMaskRefs masks{release, engage, busy};
    ScriptedFlags flags;
    ScriptedMode motion;
    std::vector<std::string> log;
    flags.log = &log;
    motion.log = &log;

    std::vector<BrakeEffect> applied;
    const auto next = run_brake_cycle_step(
        BrakeCycleInputs{.motion_possible = TriState::YES}, flags, motion, [](uint8_t) noexcept { return kAll; }, masks,
        BrakeLogicState{}, kAll, [&](const BrakeEffect& effect) { applied.push_back(effect); });

    ASSERT_EQ(applied.size(), 1u);
    EXPECT_EQ(applied[0], (BrakeEffect{BrakeEffectKind::START_RELEASE, 0x03}));
    EXPECT_EQ(release.load(), 0x00);
    EXPECT_EQ(engage.load(), 0x00);
    EXPECT_EQ(busy.load(), 0x03) << "consumed bits must be published to the busy mirror";
    EXPECT_FALSE(next.engage_pending);
}

// The readiness mask reaches the request leg unchanged

TEST_F(BrakeCycleStepTest, ReadinessMaskGatesTheRequestInItsOwnSlot) {
    // A queued direct release, joint 0 not ready: refused in the request slot
    // after the second sample, and the request is still consumed.
    masks_.release.value = 0x01;

    (void)step({.motion_possible = TriState::YES}, {}, 0x3E);

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:EMIT_RELEASE_REFUSED(1)"};
    EXPECT_EQ(log_, expected);
    EXPECT_EQ(masks_.release.value, 0x00) << "the refused request must still be consumed";
}

TEST_F(BrakeCycleStepTest, ReadinessIsAskedAfterTheExchangeWithTheConsumedReleaseBits) {
    // Readiness is checked for exactly the request this step consumed: a post
    // landing after the step began is in the consumed bits it is asked about.
    masks_.release.value = 0x05;

    (void)run_brake_cycle_step(
        BrakeCycleInputs{.motion_possible = TriState::YES}, flags_, motion_,
        [&](uint8_t consumed) noexcept {
            log_.push_back("ready(" + std::to_string(consumed) + ")");
            return kAll;
        },
        masks_, BrakeLogicState{}, kAll, [&](const BrakeEffect& effect) { log_.push_back(effect_name(effect)); });

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "ready(5)",
                                               "apply:START_RELEASE(5)"};
    EXPECT_EQ(log_, expected);
}

TEST_F(BrakeCycleStepTest, ReadinessGatesTheRequestAgainstItsOwnJoints) {
    // Joint 0 not ready; the queued direct release asks for joint 1 only:
    // started. A shell that collapsed readiness to all-or-nothing would
    // refuse it.
    masks_.release.value = 0x02;

    (void)step({.motion_possible = TriState::YES}, {}, 0x3E);

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:START_RELEASE(2)"};
    EXPECT_EQ(log_, expected);
}

TEST_F(BrakeCycleStepTest, AZeroReadinessIsFailClosedInTheShell) {
    // Readiness is a required argument (no default to forget); 0 = nothing
    // ready: a direct release every other permit admits is refused, never started.
    masks_.release.value = 0x01;

    (void)step({.motion_possible = TriState::YES}, {}, 0x00);

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample",
                                               "apply:EMIT_RELEASE_REFUSED(1)"};
    EXPECT_EQ(log_, expected);
}

// The mode trip reaches the control leg as a trigger

TEST_F(BrakeCycleStepTest, ModeTripPendingSuppressesAQueuedRecoveryReleaseWithoutARefusal) {
    // Both samples already stopped (the edge was consumed long ago) with the
    // committed recovery lane open: a mode trip on the cycle inputs still
    // fires the forced-engage/stop pair in the control slot, before the
    // exchange. A release queued in the same cycle finds that forced engage
    // pending in the same-cycle state the request leg reads, so it is
    // consumed and dropped: no START_RELEASE, and no refusal either (the
    // pending engage owns the discard; readiness is never consulted).
    flags_.protective_stop_first = true;
    flags_.protective_stop_second = true;
    motion_.protective_stop_recovery = true;
    masks_.release.value = kAll;
    const BrakeLogicState state{true, false};

    const auto next = step({.motion_possible = TriState::YES, .mode_trip = true}, state);

    const std::vector<std::string> expected = {"protective_stop.sample#1",
                                               "apply:START_FORCED_ENGAGE(" + std::to_string(kAll) + ")",
                                               "apply:EMIT_PROTECTIVE_STOP",
                                               "release.load",
                                               "engage.load",
                                               "busy.fetch_or",
                                               "release.fetch_and",
                                               "engage.fetch_and",
                                               "protective_stop.sample#2",
                                               "mode.sample"};
    EXPECT_EQ(log_, expected) << "no release effect of any kind after the second sample";
    EXPECT_EQ(masks_.release.value, 0x00) << "consumed, not parked";
    EXPECT_TRUE(next.engage_pending);
}
