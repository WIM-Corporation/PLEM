// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/ethercat/state_machine.hpp"

namespace plem::fieldbus::ethercat {

ServoStateMachine::ServoStateMachine(const Cia402TransitionProfile& profile)
    : profile_(profile), current_state_(ServoState::NOT_READY_TO_SWITCH_ON), last_status_recognized_(false) {}

void ServoStateMachine::update_state(uint16_t statusword) {
    // Single decoder: delegate to the public constexpr
    // to_servo_state() so this FSM and the free helper cannot drift; the
    // CiA 402 case table (bit-5 don't-care pairs included) lives in one
    // switch, in the header.
    const ServoState decoded = to_servo_state(statusword);
    if (decoded == ServoState::UNKNOWN) {
        // Unknown pattern: HOLD the current state, but mark the reading
        // unrecognized; holders of get_current_state() that need live
        // physical evidence (brake settlement) must check
        // status_recognized().
        last_status_recognized_ = false;
        return;
    }
    last_status_recognized_ = true;
    current_state_ = decoded;
}

ServoStateMachine::TransitionLookup ServoStateMachine::find_transition(ServoState target_state) const noexcept {
    // Bounded linear scan over the injected model profile (fixed max + count,
    // owned by value, RT-compatible; the model injects it, see
    // cia402_profile.hpp).
    for (size_t i = 0; i < profile_.transition_count; ++i) {
        const auto& t = profile_.transitions[i];
        if (t.from == current_state_ && t.to == target_state) {
            return {true, t.controlword};
        }
    }
    return {false, 0};
}

uint16_t ServoStateMachine::get_controlword_for_transition(ServoState target_state) const {
    if (const auto lookup = find_transition(target_state); lookup.found) {
        // A declared 0x0000 row (Disable voltage) also returns 0, the same as
        // the no-row return below, so callers check can_transition_to() first
        // (DeviceServo::transition_state does).
        return lookup.controlword;
    }

    if (current_state_ == target_state) {
        switch (current_state_) {
            case ServoState::NOT_READY_TO_SWITCH_ON:
                return 0x0000;
            case ServoState::READY_TO_SWITCH_ON:
                return 0x0006;
            case ServoState::SWITCHED_ON:
                return 0x0007;
            case ServoState::OPERATION_ENABLED:
                return 0x000F;
            case ServoState::QUICK_STOP_ACTIVE:
                return 0x0002;
            default:
                return 0x0000;
        }
    }

    return 0;
}

bool ServoStateMachine::can_transition_to(ServoState target_state) const {
    if (current_state_ == target_state) {
        return true;
    }
    // "Undeclared" is the absence of a row, not a 0x0000 word in one. Disable
    // voltage (bit7=0, bit1=0) is a legal CiA 402 command whose representative
    // encoding is 0x0000, and it is how transitions 7/9/10/12 are commanded;
    // a `!= 0` check would make every one of them unreachable.
    return find_transition(target_state).found;
}

}  // namespace plem::fieldbus::ethercat
