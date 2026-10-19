// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/cia402_types.hpp"

namespace plem::fieldbus::ethercat {

// Re-export ServoState from cia402_types.hpp for convenience
using plem::fieldbus::ServoState;

/**
 * @brief Convert a statusword to ServoState: the CiA 402 decoder.
 *
 * Single decode point: ServoStateMachine::update_state()
 * delegates here, so this helper and the FSM cannot drift. UNKNOWN means
 * the pattern proves nothing; callers needing live physical evidence must
 * treat it as unrecognized, not as a state.
 *
 * @param statusword Raw statusword from the drive
 * @return Corresponding ServoState (UNKNOWN for unrecognized patterns)
 */
inline constexpr ServoState to_servo_state(uint16_t statusword) {
    constexpr uint16_t status_mask = 0x6F;
    uint16_t masked_status = statusword & status_mask;

    switch (masked_status) {
        case static_cast<uint16_t>(ServoState::NOT_READY_TO_SWITCH_ON):
        case 0x0020:  // NOT_READY's CiA 402 mask is 0x004F; bit 5 (quick-stop,
                      // active-low) is don't-care (pairs with SOD below)
            return ServoState::NOT_READY_TO_SWITCH_ON;
        case static_cast<uint16_t>(ServoState::SWITCH_ON_DISABLED):
        case 0x0060:  // SOD mask is 0x004F too; 0x0060 is a resting word
                      // real drives report
            return ServoState::SWITCH_ON_DISABLED;
        case static_cast<uint16_t>(ServoState::READY_TO_SWITCH_ON):
            return ServoState::READY_TO_SWITCH_ON;
        case static_cast<uint16_t>(ServoState::SWITCHED_ON):
            return ServoState::SWITCHED_ON;
        case static_cast<uint16_t>(ServoState::OPERATION_ENABLED):
            return ServoState::OPERATION_ENABLED;
        case static_cast<uint16_t>(ServoState::QUICK_STOP_ACTIVE):
            return ServoState::QUICK_STOP_ACTIVE;
        case static_cast<uint16_t>(ServoState::FAULT_REACTION_ACTIVE):
        case 0x002F:  // FAULT_REACTION's CiA 402 mask is 0x004F; bit 5 don't-care
            return ServoState::FAULT_REACTION_ACTIVE;
        case static_cast<uint16_t>(ServoState::FAULT):
        case 0x0028:  // FAULT's CiA 402 mask is 0x004F too; drives may raise
                      // bit 5 with the fault bit. Decoding it UNKNOWN would
                      // hold the previous state, so is_faulted(), the bounded
                      // fault auto-reset and the SERVO_FAULT rising edge would
                      // never fire for that drive.
            return ServoState::FAULT;
        default:
            return ServoState::UNKNOWN;
    }
}

/**
 * @brief State machine for drive control
 *
 * The requested-transition mapping is not hardcoded here: it is the injected
 * Cia402TransitionProfile (model-declared, owned by value; see
 * cia402_profile.hpp for the scope contract). Statusword decoding and the
 * same-state hold words remain core-owned.
 */
class ServoStateMachine {
  public:
    /**
     * @brief Construct with the model-declared transition profile (mandatory)
     * @param profile Requested-transition mapping, owned by value from here on
     *
     * There is no profile-less constructor on purpose: a default table would
     * be a silent vendor-bias fallback. Validation (duplicates, invalid
     * states, absence) happens fail-fast at the assembly/bus entry,
     * validate_cia402_profile(); this class stores what it is given.
     */
    explicit ServoStateMachine(const Cia402TransitionProfile& profile);

    void update_state(uint16_t statusword);

    /**
     * @brief Get the controlword needed for transitioning to target state
     * @param target_state Desired state to transition to
     * @return uint16_t Controlword to send, or 0 if transition not possible
     *
     * A declared 0x0000 row returns 0x0000, the Disable voltage command. The
     * return value alone cannot distinguish that from the 0 returned when no
     * row exists; ask can_transition_to() first, which answers on row presence.
     */
    uint16_t get_controlword_for_transition(ServoState target_state) const;

    bool can_transition_to(ServoState target_state) const;

    /// FAULT or FAULT_REACTION_ACTIVE.
    bool is_faulted() const {
        return current_state_ == ServoState::FAULT || current_state_ == ServoState::FAULT_REACTION_ACTIVE;
    }

    ServoState get_current_state() const {
        return current_state_;
    }

    /**
     * @brief Was the last processed statusword a recognized CiA 402 pattern?
     *
     * update_state() HOLDS the previous state on an unrecognized pattern;
     * get_current_state() is then a stale reading, not live evidence. A
     * consumer that treats the state as physical proof (the brake group
     * settlement allowlist) must require this flag: an unrecognized
     * word proves nothing about where the drive actually is.
     */
    bool status_recognized() const {
        return last_status_recognized_;
    }

    /// The model-declared transition profile this machine walks.
    const Cia402TransitionProfile& profile() const {
        return profile_;
    }

  private:
    /// {found, controlword} lookup result. `found` is the declared/undeclared
    /// answer that can_transition_to() returns; the word may legitimately be
    /// 0x0000 (Disable voltage).
    struct TransitionLookup {
        bool found;
        uint16_t controlword;
    };

    TransitionLookup find_transition(ServoState target_state) const noexcept;

    Cia402TransitionProfile profile_;
    ServoState current_state_;
    /// Last update_state() word matched a known CiA 402 pattern (see getter).
    bool last_status_recognized_;
};

}  // namespace plem::fieldbus::ethercat
