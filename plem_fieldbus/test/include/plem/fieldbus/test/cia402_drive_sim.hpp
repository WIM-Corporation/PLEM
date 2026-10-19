// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file cia402_drive_sim.hpp
/// @brief The answering CiA 402 drive the fake-master tests share: each cycle
/// it consumes the controlword on the wire and reports a statusword.
/// Test-support header (installed only with BUILD_TESTING, never part of the
/// release surface).
///
/// The transition table is written here from the CiA 402 state machine
/// (IEC 61800-7-201), not taken from the production walk, so a test that
/// drives the core against it compares two independent readings:
///
/// | state                | leaves on                                                              |
/// |----------------------|------------------------------------------------------------------------|
/// | Switch on disabled   | Shutdown → Ready to switch on (Switch on / Enable operation ignored)  |
/// | Ready to switch on   | Switch on → Switched on; Enable operation → Operation enabled (3+4);  |
/// |                      | Disable voltage / Quick stop → Switch on disabled                      |
/// | Switched on          | Enable operation → Operation enabled; Shutdown → Ready to switch on;   |
/// |                      | Disable voltage / Quick stop → Switch on disabled                      |
/// | Operation enabled    | Switch on → Switched on; Shutdown → Ready to switch on;                |
/// |                      | Disable voltage → Switch on disabled; Quick stop → Quick stop active   |
/// | Quick stop active    | Enable operation → Operation enabled; otherwise the stop completes:   |
/// |                      | Switch on disabled the next cycle (quick stop option code 2)          |
/// | Fault                | a rising edge of controlword bit 7 → Switch on disabled                |
///
/// Commands are decoded with the standard masks (Shutdown xxxx x110, Switch
/// on 0xxx 0111, Enable operation 0xxx 1111, Disable voltage xxxx xx0x,
/// Quick stop xxxx x01x). Statuswords carry bit 9 (remote) and, above Switch
/// on disabled, bit 4 (voltage enabled), as real drives do; Switch on
/// disabled reports the rest word the test gives. `delay_cycles` makes every
/// state change show up that many cycles late (a slow drive); inject_fault()
/// drops the drive into Fault. It models no motion: a test that needs a
/// following axis copies the target into the actual itself.

#pragma once

#include <cstdint>
#include <deque>

namespace plem::fieldbus::test {

class Cia402DriveSim {
  public:
    static constexpr uint16_t kReadyToSwitchOn = 0x0231;
    static constexpr uint16_t kSwitchedOn = 0x0233;
    static constexpr uint16_t kOperationEnabled = 0x0237;
    static constexpr uint16_t kQuickStopActive = 0x0217;
    static constexpr uint16_t kFault = 0x0218;

    explicit Cia402DriveSim(uint16_t rest, int delay_cycles = 0) : rest_(rest), reported_(delay_cycles + 1, rest) {}

    /// The statusword the drive reports this cycle.
    [[nodiscard]] uint16_t status() const {
        return reported_.front();
    }
    /// Switch on disabled, as this drive reports it.
    [[nodiscard]] uint16_t rest_status() const {
        return rest_;
    }

    /// One cycle: answer `controlword`, the word now on the wire.
    void consume(uint16_t controlword) {
        state_ = next(controlword);
        previous_controlword_ = controlword;
        reported_.pop_front();
        reported_.push_back(word());
    }

    /// The drive faults (reported after the delay, like any change).
    void inject_fault() {
        state_ = State::FAULT;
        reported_.back() = word();
    }

  private:
    enum class State {
        SWITCH_ON_DISABLED,
        READY_TO_SWITCH_ON,
        SWITCHED_ON,
        OPERATION_ENABLED,
        QUICK_STOP_ACTIVE,
        FAULT
    };

    State next(uint16_t cw) const {
        const bool shutdown = (cw & 0x87) == 0x06;
        const bool switch_on = (cw & 0x8F) == 0x07;
        const bool enable = (cw & 0x8F) == 0x0F;
        const bool disable_voltage = (cw & 0x02) == 0x00;
        const bool quick_stop = (cw & 0x06) == 0x02;
        switch (state_) {
            case State::SWITCH_ON_DISABLED:
                return shutdown ? State::READY_TO_SWITCH_ON : state_;
            case State::READY_TO_SWITCH_ON:
                if (disable_voltage || quick_stop) {
                    return State::SWITCH_ON_DISABLED;
                }
                return switch_on ? State::SWITCHED_ON : enable ? State::OPERATION_ENABLED : state_;
            case State::SWITCHED_ON:
                if (disable_voltage || quick_stop) {
                    return State::SWITCH_ON_DISABLED;
                }
                return enable ? State::OPERATION_ENABLED : shutdown ? State::READY_TO_SWITCH_ON : state_;
            case State::OPERATION_ENABLED:
                if (disable_voltage) {
                    return State::SWITCH_ON_DISABLED;
                }
                if (quick_stop) {
                    return State::QUICK_STOP_ACTIVE;
                }
                return switch_on ? State::SWITCHED_ON : shutdown ? State::READY_TO_SWITCH_ON : state_;
            case State::QUICK_STOP_ACTIVE:
                return enable ? State::OPERATION_ENABLED : State::SWITCH_ON_DISABLED;
            case State::FAULT: {
                const bool reset_edge = (cw & 0x80) != 0 && (previous_controlword_ & 0x80) == 0;
                return reset_edge ? State::SWITCH_ON_DISABLED : state_;
            }
        }
        return state_;
    }

    uint16_t word() const {
        switch (state_) {
            case State::SWITCH_ON_DISABLED:
                return rest_;
            case State::READY_TO_SWITCH_ON:
                return kReadyToSwitchOn;
            case State::SWITCHED_ON:
                return kSwitchedOn;
            case State::OPERATION_ENABLED:
                return kOperationEnabled;
            case State::QUICK_STOP_ACTIVE:
                return kQuickStopActive;
            case State::FAULT:
                return kFault;
        }
        return rest_;
    }

    uint16_t rest_;
    State state_{State::SWITCH_ON_DISABLED};
    uint16_t previous_controlword_{0};
    std::deque<uint16_t> reported_;  ///< front = this cycle's word, back = the newest
};

}  // namespace plem::fieldbus::test
