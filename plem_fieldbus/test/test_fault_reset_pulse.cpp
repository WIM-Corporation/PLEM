// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_fault_reset_pulse.cpp
/// @brief Truth table for the CiA 402 fault-reset pulse decision.
/// Pins the falling-edge guarantee, the two-frame request lifetime, and the
/// property easiest to get wrong: an ineligible request is
/// discarded whether or not a pulse is in flight.

#include <gtest/gtest.h>

#include <vector>

#include "ethercat/servo/fault_reset_pulse.hpp"

using plem::fieldbus::kCia402FaultResetControlword;
using plem::fieldbus::ethercat::FaultResetRequest;
using plem::fieldbus::ethercat::kCia402DisableVoltageControlword;
using plem::fieldbus::ethercat::next_fault_reset_frame;

namespace {

/// A command the transition path could plausibly be holding: Shutdown
/// (transition 2/6). Anything nonzero and not the pulse word works.
constexpr uint16_t kStanding = 0x0006;

TEST(FaultResetPulse, EligibleRequestPublishesTheRisingEdge) {
    const auto f = next_fault_reset_frame(0, true, true, kStanding);
    EXPECT_EQ(f.controlword, kCia402FaultResetControlword);
    EXPECT_EQ(f.request, FaultResetRequest::CONSUMED);
}

TEST(FaultResetPulse, PublishedPulseIsAlwaysFollowedByALowFrame) {
    // The whole point: transition 15 fires on the rising edge, so a second
    // request can only work if the wire went low in between. No input
    // combination may keep the word high two frames running.
    for (const bool request : {false, true}) {
        for (const bool eligible : {false, true}) {
            const auto f = next_fault_reset_frame(1, request, eligible, kStanding);
            // The word is the inactive baseline, not the standing command:
            // the low erases a command the decider produced on this same
            // cycle, so it cannot fire once the sequence that wanted it is
            // gone. (That the erasure persists is a property of the caller's
            // store, pinned in test_ethercat_bus_fake_master.cpp.)
            EXPECT_EQ(f.controlword, kCia402DisableVoltageControlword)
                << "request=" << request << " eligible=" << eligible;
            EXPECT_NE(f.request, FaultResetRequest::CONSUMED);
        }
    }
}

TEST(FaultResetPulse, IneligibleRequestIsDiscardedWhetherOrNotAPulseIsInFlight) {
    // The sequence high → recognized SWITCH_ON_DISABLED → FAULT must
    // not republish the pre-recovery request against the new fault. That holds
    // only because disposition is independent of `owed`.
    EXPECT_EQ(next_fault_reset_frame(0, true, false, kStanding).request, FaultResetRequest::DISCARDED);
    EXPECT_EQ(next_fault_reset_frame(1, true, false, kStanding).request, FaultResetRequest::DISCARDED);
}

TEST(FaultResetPulse, DiscardedRequestLeavesTheStandingCommandAlone) {
    const auto f = next_fault_reset_frame(0, true, false, kStanding);
    EXPECT_EQ(f.controlword, kStanding);
    EXPECT_EQ(f.request, FaultResetRequest::DISCARDED);
}

TEST(FaultResetPulse, EligibleRequestOnAForcedLowFrameIsHeldExactlyOneFrame) {
    const auto low = next_fault_reset_frame(1, true, true, kStanding);
    EXPECT_EQ(low.request, FaultResetRequest::KEPT);
    EXPECT_EQ(low.controlword, kCia402DisableVoltageControlword);

    // Re-offered on the next frame, it publishes, so a request survives at
    // most two frames and needs no expiry counter, epoch or cancel store.
    const auto next = next_fault_reset_frame(low.owed, true, true, kStanding);
    EXPECT_EQ(next.request, FaultResetRequest::CONSUMED);
    EXPECT_EQ(next.controlword, kCia402FaultResetControlword);
}

TEST(FaultResetPulse, NoRequestPassesTheStandingCommandThrough) {
    const auto f = next_fault_reset_frame(0, false, true, kStanding);
    EXPECT_EQ(f.controlword, kStanding);
    EXPECT_EQ(f.request, FaultResetRequest::NONE);
}

TEST(FaultResetPulse, ConsumedAndDiscardedAreMutuallyExclusiveOverTheWholeTable) {
    // The structural claim behind the enum: an acquired request ends in
    // exactly one of published / dropped / held, published and dropped are
    // mutually exclusive, and held is reachable only on the one
    // forced-low-while-eligible frame.
    for (const uint32_t owed : {0u, 1u}) {
        for (const bool eligible : {false, true}) {
            const auto with = next_fault_reset_frame(owed, true, eligible, kStanding);
            EXPECT_NE(with.request, FaultResetRequest::NONE) << "owed=" << owed << " eligible=" << eligible;
            EXPECT_EQ(with.request == FaultResetRequest::KEPT, owed == 1 && eligible);
            EXPECT_EQ(with.request == FaultResetRequest::CONSUMED, owed == 0 && eligible);

            EXPECT_EQ(next_fault_reset_frame(owed, false, eligible, kStanding).request, FaultResetRequest::NONE);
        }
    }
}

TEST(FaultResetPulse, PulseWordIsTheSpecFixedBit7Command) {
    // The pulse never comes from a profile lookup: the validator rejects
    // bit 7 in any row, so this constant is the only source.
    EXPECT_EQ(kCia402FaultResetControlword, 0x0080);
    EXPECT_EQ(kCia402DisableVoltageControlword, 0x0000);
}

// A declared width (the tests above run at the one-frame default)

TEST(FaultResetPulse, AWidePulseHoldsItsWidthThenOwesOneLowFrame) {
    constexpr uint32_t kWidth = 4;
    auto f = next_fault_reset_frame(0, true, true, kStanding, kWidth);
    ASSERT_EQ(f.request, FaultResetRequest::CONSUMED);
    std::vector<uint16_t> wire{f.controlword};
    while (f.owed > 0) {
        f = next_fault_reset_frame(f.owed, false, true, kStanding, kWidth);
        wire.push_back(f.controlword);
    }
    EXPECT_EQ(wire, (std::vector<uint16_t>{0x0080, 0x0080, 0x0080, 0x0080, 0x0000}));
}

TEST(FaultResetPulse, LeavingFaultEndsAWidePulseOnThatFrame) {
    auto f = next_fault_reset_frame(0, true, true, kStanding, 10);
    f = next_fault_reset_frame(f.owed, false, true, kStanding, 10);
    ASSERT_EQ(f.controlword, kCia402FaultResetControlword) << "still FAULT: the hold goes on";
    f = next_fault_reset_frame(f.owed, false, false, kStanding, 10);
    EXPECT_EQ(f.controlword, kCia402DisableVoltageControlword) << "the axis left FAULT: this frame is the edge";
    EXPECT_EQ(f.owed, 0u) << "however much of the width was left";
}

TEST(FaultResetPulse, AnEligibleRequestDuringTheHoldWaitsForTheFallingEdge) {
    auto f = next_fault_reset_frame(0, true, true, kStanding, 3);
    for (const uint16_t word :
         {kCia402FaultResetControlword, kCia402FaultResetControlword, kCia402DisableVoltageControlword}) {
        f = next_fault_reset_frame(f.owed, true, true, kStanding, 3);
        EXPECT_EQ(f.controlword, word);
        EXPECT_EQ(f.request, FaultResetRequest::KEPT);
    }
    f = next_fault_reset_frame(f.owed, true, true, kStanding, 3);
    EXPECT_EQ(f.request, FaultResetRequest::CONSUMED) << "a NEW rising edge, after the low";
}

TEST(FaultResetPulse, AZeroWidthIsTheOneFramePulse) {
    EXPECT_EQ(next_fault_reset_frame(0, true, true, kStanding, 0).owed, 1u);
}

}  // namespace
