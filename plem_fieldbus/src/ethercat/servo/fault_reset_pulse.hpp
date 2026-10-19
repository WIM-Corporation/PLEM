// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include "ethercat/internal_visibility.hpp"
#include "plem/fieldbus/cia402_profile.hpp"

namespace plem::fieldbus::ethercat {

// Package-private fault-reset pulse decision (deliberately not an installed
// header): the one place that owns what the controlword carries when a
// CiA 402 fault reset is in flight. CiA 402 transition 15 (FAULT →
// SWITCH_ON_DISABLED) is edge-triggered (the drive acts on the rising edge
// of controlword bit 7), so a host that leaves bit 7 standing high offers no
// further edge and then holds a word the drive ignores.
//
// Claim scope: what this guarantees is a property of the words the host
// writes into domain memory, on consecutive writes of the same object: a
// published pulse holds bit 7 for at most its declared width and is followed
// by a low frame, so the next eligible request writes a new rising edge. It does not claim the drive received either
// frame (a lost frame is invisible here), that a reset occurred, or that the fault cause cleared; and a newly
// constructed device knows nothing about the last word a drive was actually receiving. The claim also covers only the
// words this decision produces (the test-only DeviceServoTestAccess::set_controlword bypasses it).
//
// Pure constexpr decision with POD structs and an enum; no virtuals or
// allocation. Table-tested in test_fault_reset_pulse.cpp; fake-master traces
// pin cycle-exact publication behavior.

/// @brief The inactive controlword this pulse writes between pulses.
///
/// Claim scope: 0x0000 is the CiA 402 command "Disable voltage" (bit 7 = 0 ∧
/// bit 1 = 0), which is what makes it a safe pulse baseline; it is not a
/// physical-closure guarantee. The stored torque setpoint keeps being written
/// (ServoPDOHandler::write), brake_logic does not accept FAULT/FRA/QSA as
/// closure evidence, and a shared-brake partner may still be holding open.
inline constexpr uint16_t kCia402DisableVoltageControlword = 0x0000;

/// @brief What this frame did with the request the caller handed in.
///
/// An acquired request ends in exactly one of three outcomes: published
/// (`CONSUMED`), dropped (`DISCARDED`), or held while a pulse is still on the
/// wire (`KEPT`). `CONSUMED` and `DISCARDED` are mutually exclusive by
/// construction, and `KEPT` is reachable only while a pulse owes frames,
/// which bounds a request's life to the pulse width plus two frames. `NONE`
/// means the caller acquired nothing this frame.
enum class FaultResetRequest : uint8_t {
    NONE,       ///< no request was in flight
    CONSUMED,   ///< published as this frame's rising edge
    DISCARDED,  ///< dropped: the axis was not eligible at publication time
    KEPT,       ///< held: eligible, but a pulse still owes this frame (its hold or its falling edge)
};

/// @brief This frame's controlword, the request's disposition, and the
/// frames the pulse still owes: the caller's `owed` for the next frame.
struct FaultResetFrame {
    uint16_t controlword{kCia402DisableVoltageControlword};  ///< the frame's word = the new standing word
    FaultResetRequest request{FaultResetRequest::NONE};
    uint32_t owed{0};  ///< frames still owed after this one; the last of them is the falling edge
};

/// @brief Decide one frame of the fault-reset pulse.
///
/// @param owed            frames the pulse in flight still owes (the previous
///                        frame's `owed`; 0 = no pulse on the wire). 1 = this
///                        frame owes the falling edge, whatever else is true;
///                        more = it owes a further high frame first.
/// @param request         a request was acquired for this frame.
/// @param reset_eligible  composed at the call site: `status_recognized() ∧
///                        state == FAULT`. Composed rather than split because
///                        the decision never distinguishes the two halves —
///                        an unrecognized frame and a recognized non-FAULT
///                        frame both mean "do not publish". The caller's
///                        characterization tests keep them apart.
/// @param standing_controlword  the word the transition path last wrote.
/// @param pulse_frames    the pulse width in frames (the device's, from the
///                        model's `fault_reset_pulse_ms` at the bus period); 0
///                        is read as 1. 1 = the one-frame pulse.
///
/// | prio | condition                                   | word     | request                   | next owed |
/// |-----:|---------------------------------------------|----------|---------------------------|-----------|
/// |    1 | `owed > 1 ∧ eligible`                       | 0x0080   | held (see below)          | owed − 1  |
/// |    2 | `owed ≥ 1` (the rest)                       | 0x0000   | held (see below)          | 0         |
/// |    3 | `owed = 0 ∧ request ∧ eligible`             | 0x0080   | CONSUMED                  | width     |
/// |    4 | `owed = 0 ∧ request` (incl. unrecognized)   | standing | DISCARDED                 | 0         |
/// |    5 | no request                                  | standing | NONE                      | 0         |
///
/// "held" = no request → NONE, else KEPT if eligible, else DISCARDED.
///
/// Three properties this table is built to hold. (a) An ineligible request is
/// discarded regardless of `owed`, even on a forced-low frame, so
/// `high → recognized SWITCH_ON_DISABLED → FAULT` cannot republish the
/// pre-recovery request against the new fault. (An eligible request does
/// depend on it: CONSUMED when nothing is owed, KEPT while a pulse is.) (b)
/// Request lifetime is at most the pulse width plus two frames, so no expiry
/// counter, epoch, queue or cancel store exists; an unrecognized frame drops
/// the request and the operator re-requests. (c) Bit 7 stays high only while
/// the axis reads a recognized FAULT: the frame that no longer does is the
/// falling edge, however much of the width is left; a drive that has left
/// FAULT gets no further Fault reset command.
///
/// Priority 2 overwrites the standing word with 0x0000 rather than only the
/// output (and priority 1 with 0x0080): a command generated by the decider
/// during that same pulse cycle must not survive to fire a cycle later, after
/// a forced engage or an early settle has already ended the sequence that
/// wanted it.
[[nodiscard]] PLEM_FIELDBUS_INTERNAL constexpr FaultResetFrame next_fault_reset_frame(
    uint32_t owed, bool request, bool reset_eligible, uint16_t standing_controlword,
    uint32_t pulse_frames = 1) noexcept {
    if (owed > 0) {
        const FaultResetRequest held = request
                                           ? (reset_eligible ? FaultResetRequest::KEPT : FaultResetRequest::DISCARDED)
                                           : FaultResetRequest::NONE;
        if (owed > 1 && reset_eligible) {
            return {kCia402FaultResetControlword, held, owed - 1};
        }
        return {kCia402DisableVoltageControlword, held, 0};
    }
    if (request && reset_eligible) {
        return {kCia402FaultResetControlword, FaultResetRequest::CONSUMED, pulse_frames > 1 ? pulse_frames : 1};
    }
    return {standing_controlword, request ? FaultResetRequest::DISCARDED : FaultResetRequest::NONE, 0};
}

}  // namespace plem::fieldbus::ethercat
