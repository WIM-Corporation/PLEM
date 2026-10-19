// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/ethercat/state_machine.hpp"
#include "plem/fieldbus/test/cia402_drive_sim.hpp"

using namespace plem::fieldbus;
using namespace plem::fieldbus::ethercat;

class ServoStateMachineTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Explicit synthetic-profile injection (the transition table is
        // model-declared data; no profile-less constructor).
        machine_ = std::make_unique<ServoStateMachine>(plem::fieldbus::kNotReadyShutdownCia402Profile);
    }

    std::unique_ptr<ServoStateMachine> machine_;
};

TEST_F(ServoStateMachineTest, InitialState) {
    // Machine should start in NOT_READY_TO_SWITCH_ON state
    EXPECT_EQ(machine_->get_current_state(), ServoState::NOT_READY_TO_SWITCH_ON);
    EXPECT_NE(machine_->get_current_state(), ServoState::OPERATION_ENABLED);
    EXPECT_FALSE(machine_->is_faulted());
}

TEST_F(ServoStateMachineTest, UpdateStateFromStatusword) {
    machine_->update_state(0x0221);  // Status word for READY_TO_SWITCH_ON
    EXPECT_EQ(machine_->get_current_state(), ServoState::READY_TO_SWITCH_ON);

    machine_->update_state(0x0223);  // Status word for SWITCHED_ON
    EXPECT_EQ(machine_->get_current_state(), ServoState::SWITCHED_ON);

    machine_->update_state(0x0237);  // Status word for OPERATION_ENABLED
    EXPECT_EQ(machine_->get_current_state(), ServoState::OPERATION_ENABLED);

    machine_->update_state(0x0217);  // Status word for QUICK_STOP_ACTIVE
    EXPECT_EQ(machine_->get_current_state(), ServoState::QUICK_STOP_ACTIVE);
    EXPECT_NE(machine_->get_current_state(), ServoState::OPERATION_ENABLED);
}

TEST_F(ServoStateMachineTest, UpdateStateWithMasking) {
    // Test that status word is properly masked (only bits 0x006F are considered)
    machine_->update_state(0xFF27);  // Should be recognized as OPERATION_ENABLED
    EXPECT_EQ(machine_->get_current_state(), ServoState::OPERATION_ENABLED);

    machine_->update_state(0x1023);  // Should be recognized as SWITCHED_ON
    EXPECT_EQ(machine_->get_current_state(), ServoState::SWITCHED_ON);
}

TEST_F(ServoStateMachineTest, TransitionFromNotReady) {
    machine_->update_state(0x0220);  // NOT_READY_TO_SWITCH_ON

    uint16_t controlword = machine_->get_controlword_for_transition(ServoState::READY_TO_SWITCH_ON);
    EXPECT_EQ(controlword, 0x0006);
    EXPECT_TRUE(machine_->can_transition_to(ServoState::READY_TO_SWITCH_ON));

    controlword = machine_->get_controlword_for_transition(ServoState::OPERATION_ENABLED);
    EXPECT_EQ(controlword, 0);  // Should return 0 for invalid transition
    EXPECT_FALSE(machine_->can_transition_to(ServoState::OPERATION_ENABLED));
}

TEST_F(ServoStateMachineTest, TransitionFromReadyToSwitch) {
    machine_->update_state(0x0221);  // READY_TO_SWITCH_ON

    uint16_t controlword = machine_->get_controlword_for_transition(ServoState::SWITCHED_ON);
    EXPECT_EQ(controlword, 0x0007);

    controlword = machine_->get_controlword_for_transition(ServoState::NOT_READY_TO_SWITCH_ON);
    EXPECT_EQ(controlword, 0x0000);
}

TEST_F(ServoStateMachineTest, TransitionFromSwitchedOn) {
    machine_->update_state(0x0223);  // SWITCHED_ON

    uint16_t controlword = machine_->get_controlword_for_transition(ServoState::OPERATION_ENABLED);
    EXPECT_EQ(controlword, 0x000F);

    controlword = machine_->get_controlword_for_transition(ServoState::READY_TO_SWITCH_ON);
    EXPECT_EQ(controlword, 0x0006);
}

TEST_F(ServoStateMachineTest, TransitionFromOperationEnabled) {
    machine_->update_state(0x0237);  // OPERATION_ENABLED

    uint16_t controlword = machine_->get_controlword_for_transition(ServoState::SWITCHED_ON);
    EXPECT_EQ(controlword, 0x0007);

    controlword = machine_->get_controlword_for_transition(ServoState::QUICK_STOP_ACTIVE);
    EXPECT_EQ(controlword, 0x0002);
}

TEST_F(ServoStateMachineTest, TransitionFromQuickStop) {
    machine_->update_state(0x0217);  // QUICK_STOP_ACTIVE

    uint16_t controlword = machine_->get_controlword_for_transition(ServoState::SWITCHED_ON);
    EXPECT_EQ(controlword, 0x0000);

    controlword = machine_->get_controlword_for_transition(ServoState::NOT_READY_TO_SWITCH_ON);
    EXPECT_EQ(controlword, 0x0000);
}

TEST_F(ServoStateMachineTest, FaultStateHandling) {
    machine_->update_state(0x0008);  // FAULT
    EXPECT_EQ(machine_->get_current_state(), ServoState::FAULT);
    EXPECT_TRUE(machine_->is_faulted());
    EXPECT_NE(machine_->get_current_state(), ServoState::OPERATION_ENABLED);

    // The state machine has no path to the reset word: fault reset is an
    // edge (transition 15 fires on the rising edge of bit 7), so it is owned
    // by the device's publication point and the validator rejects bit 7 in
    // any profile row. Leaving FAULT is therefore undeclared here.
    EXPECT_FALSE(machine_->can_transition_to(ServoState::NOT_READY_TO_SWITCH_ON));
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::NOT_READY_TO_SWITCH_ON), 0x0000);
    EXPECT_FALSE(machine_->can_transition_to(ServoState::SWITCH_ON_DISABLED));
}

TEST_F(ServoStateMachineTest, FaultReactionActiveHandling) {
    machine_->update_state(0x000F);  // FAULT_REACTION_ACTIVE
    EXPECT_EQ(machine_->get_current_state(), ServoState::FAULT_REACTION_ACTIVE);
    EXPECT_TRUE(machine_->is_faulted());
    EXPECT_NE(machine_->get_current_state(), ServoState::OPERATION_ENABLED);

    // Same as FAULT: no profile row can carry the reset bit (see above), and
    // FAULT_REACTION_ACTIVE additionally reaches FAULT on its own (transition
    // 14) rather than by any command from here.
    EXPECT_FALSE(machine_->can_transition_to(ServoState::NOT_READY_TO_SWITCH_ON));
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::NOT_READY_TO_SWITCH_ON), 0x0000);
}

TEST_F(ServoStateMachineTest, SameStateControlwords) {
    machine_->update_state(0x0221);
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::READY_TO_SWITCH_ON), 0x0006);

    machine_->update_state(0x0223);
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::SWITCHED_ON), 0x0007);

    machine_->update_state(0x0237);
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::OPERATION_ENABLED), 0x000F);
    EXPECT_TRUE(machine_->can_transition_to(ServoState::OPERATION_ENABLED)) << "staying is always a valid transition";
}

TEST(ToServoStateDecoder, SingleDecoderBitFiveDontCarePairs) {
    // to_servo_state() is the decoder (the FSM delegates to it):
    // pin the bit-5 don't-care pairs and the unrecognized fallback here so
    // the header table cannot regress independently of FSM behavior.
    EXPECT_EQ(to_servo_state(0x0020), ServoState::NOT_READY_TO_SWITCH_ON);
    EXPECT_EQ(to_servo_state(0x0060), ServoState::SWITCH_ON_DISABLED);
    EXPECT_EQ(to_servo_state(0x0240), ServoState::SWITCH_ON_DISABLED);
    EXPECT_EQ(to_servo_state(0x0041), ServoState::UNKNOWN);
}

TEST(ToServoStateDecoder, FaultBitFiveDontCarePairs) {
    // CiA 402 masks FAULT (x0xx1000) and FAULT_REACTION_ACTIVE (x0xx1111)
    // with 0x4F: bit 5 (quick-stop, active-low) is don't-care, same as the
    // NOT_READY/SOD pairs above. A drive may raise bit 5 together with the
    // fault bit (0x0028). Decoding that UNKNOWN would hold the previous
    // state, so is_faulted() would never fire, and the bounded fault auto-reset
    // and the SERVO_FAULT rising edge would both be unreachable on such a
    // drive.
    EXPECT_EQ(to_servo_state(0x0028), ServoState::FAULT);
    EXPECT_EQ(to_servo_state(0x0008), ServoState::FAULT);
    EXPECT_EQ(to_servo_state(0x002F), ServoState::FAULT_REACTION_ACTIVE);
    EXPECT_EQ(to_servo_state(0x000F), ServoState::FAULT_REACTION_ACTIVE);
}

TEST_F(ServoStateMachineTest, RecognizesHardwareFaultWordWithBitFive) {
    machine_->update_state(0x0028);  // fault word with bit 5 set
    EXPECT_EQ(machine_->get_current_state(), ServoState::FAULT);
    EXPECT_TRUE(machine_->is_faulted());
    EXPECT_TRUE(machine_->status_recognized());
}

TEST_F(ServoStateMachineTest, FsmDelegatesToTheSingleDecoder) {
    machine_->update_state(0x0060);  // a resting word: SOD
    EXPECT_EQ(machine_->get_current_state(), ServoState::SWITCH_ON_DISABLED);
    EXPECT_TRUE(machine_->status_recognized());
    machine_->update_state(0x0041);  // unrecognized: HOLD, retract recognition
    EXPECT_EQ(machine_->get_current_state(), ServoState::SWITCH_ON_DISABLED);
    EXPECT_FALSE(machine_->status_recognized());
}

TEST_F(ServoStateMachineTest, DeclaredZeroControlwordRowIsCommandableUndeclaredIsNot) {
    machine_->update_state(0x0221);  // READY_TO_SWITCH_ON
    // The synthetic profile declares READY -> SWITCH_ON_DISABLED with 0x0000:
    // CiA 402 transition 7, commanded with Disable voltage (bit7=0, bit1=0).
    // 0x0000 is a command, not an "exists but not commandable" marker.
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::SWITCH_ON_DISABLED), 0x0000);
    EXPECT_TRUE(machine_->can_transition_to(ServoState::SWITCH_ON_DISABLED));
    // Undeclared is the absence of a row. The getter returns the same 0 here,
    // which is why callers ask can_transition_to(); that is the marker.
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::NOT_READY_TO_SWITCH_ON), 0x0000);
    EXPECT_FALSE(machine_->can_transition_to(ServoState::NOT_READY_TO_SWITCH_ON));
    EXPECT_EQ(machine_->get_controlword_for_transition(ServoState::QUICK_STOP_ACTIVE), 0x0000);
    EXPECT_FALSE(machine_->can_transition_to(ServoState::QUICK_STOP_ACTIVE));
}

// The shared answering drive (test/include): its table is written from the
// CiA 402 state machine, and its words decode to the states it claims.
namespace {

using plem::fieldbus::test::Cia402DriveSim;

ServoState decoded(const Cia402DriveSim& drive) {
    return to_servo_state(drive.status());
}

}  // namespace

TEST(Cia402DriveSim, FromSwitchOnDisabledOnlyShutdownLeaves) {
    Cia402DriveSim drive(0x0250);
    drive.consume(0x0007);
    drive.consume(0x000F);
    EXPECT_EQ(drive.status(), 0x0250);
    drive.consume(0x0006);
    EXPECT_EQ(decoded(drive), ServoState::READY_TO_SWITCH_ON);
    drive.consume(0x0007);
    EXPECT_EQ(decoded(drive), ServoState::SWITCHED_ON);
    drive.consume(0x000F);
    EXPECT_EQ(decoded(drive), ServoState::OPERATION_ENABLED);
    drive.consume(0x0000);
    EXPECT_EQ(drive.status(), 0x0250);
}

TEST(Cia402DriveSim, QuickStopFromOperationEnabledEndsSwitchOnDisabled) {
    Cia402DriveSim drive(0x0250);
    drive.consume(0x0006);
    drive.consume(0x000F);
    ASSERT_EQ(decoded(drive), ServoState::OPERATION_ENABLED);
    drive.consume(0x0002);
    EXPECT_EQ(decoded(drive), ServoState::QUICK_STOP_ACTIVE);
    drive.consume(0x0002);
    EXPECT_EQ(decoded(drive), ServoState::SWITCH_ON_DISABLED);
}

TEST(Cia402DriveSim, AFaultLeavesOnlyOnAResetRisingEdge) {
    Cia402DriveSim drive(0x0250);
    drive.consume(0x0006);
    drive.inject_fault();
    EXPECT_EQ(decoded(drive), ServoState::FAULT);
    drive.consume(0x0006);
    drive.consume(0x000F);
    EXPECT_EQ(decoded(drive), ServoState::FAULT);
    drive.consume(0x0080);
    EXPECT_EQ(decoded(drive), ServoState::SWITCH_ON_DISABLED);

    drive.inject_fault();
    drive.consume(0x0080);  // bit 7 held high since the last cycle: no edge
    EXPECT_EQ(decoded(drive), ServoState::FAULT);
    drive.consume(0x0000);
    drive.consume(0x0080);
    EXPECT_EQ(decoded(drive), ServoState::SWITCH_ON_DISABLED);
}

TEST(Cia402DriveSim, ADelayedDriveReportsEachChangeThatManyCyclesLate) {
    Cia402DriveSim drive(0x0250, 2);
    drive.consume(0x0006);
    EXPECT_EQ(drive.status(), 0x0250);
    drive.consume(0x0006);
    EXPECT_EQ(drive.status(), 0x0250);
    drive.consume(0x0006);
    EXPECT_EQ(decoded(drive), ServoState::READY_TO_SWITCH_ON);
}
