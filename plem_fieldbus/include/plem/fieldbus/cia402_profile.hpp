// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include <plem/core/result.hpp>

#include "plem/fieldbus/cia402_types.hpp"

namespace plem::fieldbus {

/// @brief CiA 402 spec-fixed fault-reset command (controlword bit 7).
///
/// Fault reset is deliberately outside the injected transition profile, and
/// the validator below enforces that: no profile row may carry bit 7. The
/// reset is an edge, not a level (transition 15 fires on the rising edge of
/// bit 7), so it needs a writer that owns both edges: DeviceServo::write()
/// via next_fault_reset_frame. A row carrying bit 7 would reach the wire
/// through the ordinary transition path, which only ever writes levels, and
/// would then repeat the word every cycle without ever producing a second
/// edge. A vendor that varies even this escalates by widening the profile
/// scope with hardware evidence — never by a fallback here.
inline constexpr uint16_t kCia402FaultResetControlword = 0x0080;

/// @brief One requested-transition mapping: (from, to) → controlword.
struct Cia402Transition {
    ServoState from = ServoState::UNKNOWN;
    ServoState to = ServoState::UNKNOWN;
    uint16_t controlword = 0;
    constexpr bool operator==(const Cia402Transition&) const = default;
};

/// @brief Vendor/model-declared CiA 402 requested-transition profile.
///
/// Scope: requested-transition mapping only. A transition table is a vendor
/// profile; the declaring model states its provenance per row group (some rows
/// are drive-verified, others are declared from the standard). Statusword
/// decoding, same-state hold words, state sequences and the fault-reset
/// command stay core-owned and hardcoded.
///
/// Within that mapping the declarable rows are the walk this repository
/// executes: a subset of CiA 402 plus the one compatibility row
/// (NOT_READY_TO_SWITCH_ON + Shutdown → READY_TO_SWITCH_ON; see
/// standard_transition_target). validate_cia402_profile enforces both directions, and they are
/// not the same set: the ten required pairs must be declared
/// (kCia402RequiredTransitions, not every walk row; the compatibility row,
/// the one-word 3+4 row and transitions 11 and 16 are optional), and every declared row must be a pair
/// this walk maps.
/// Documented vendor shortcuts outside this walk, such as Mitsubishi MR-JET-G
/// reaching OPERATION_ENABLED from SWITCH_ON_DISABLED with one Enable
/// operation, are out of scope. An undeclared pair requested by the release
/// head holds. Widening the walk requires reviewing the validator's allowed
/// rows, runtime recovery and admission, the profile's provenance and
/// declaring model, an independent controlword trace, and a bench record
/// together.
///
/// Mandatory at the C++ assembly layer (AxisSpec::cia402_profile /
/// ServoInfo::cia402_profile): no default exists there (a fallback would
/// silently apply one vendor's table to every other vendor's drive); an
/// undeclared profile fails fast at assembly entry. The drive model file
/// loader uses kStandardCia402Profile when the file has no cia402_profile key
/// and logs that at debug level; the approval record's profile digest then pins the
/// table the drive was approved with.
///
/// Representation: fixed max + count POD, owned by value down the injection
/// chain (model spec → ServoInfo → DeviceServo → ServoStateMachine): bounded
/// linear scan, no map, no virtual dispatch (RT-compatible).
struct Cia402TransitionProfile {
    /// Fixed transition capacity (fixed max + count idiom).
    /// kNotReadyShutdownCia402Profile declares 13 rows. Fixed capacity; raise
    /// it when a model needs more rows.
    static constexpr size_t kMaxTransitions = 16;

    std::array<Cia402Transition, kMaxTransitions> transitions{};
    size_t transition_count = 0;
    constexpr bool operator==(const Cia402TransitionProfile&) const = default;
};

/// @brief The CiA 402 subset this walk executes: transitions 2-12 plus the
/// one-word READY_TO_SWITCH_ON → OPERATION_ENABLED row (3+4); transition 16
/// is walkable but not declared here. No row lies outside the standard (no
/// compatibility row; see kNotReadyShutdownCia402Profile). Reference this
/// value; do not copy it.
///
/// At the C++ assembly layer this is a declaration an axis opts into by name,
/// not a default: C++ code must still set each axis's cia402_profile. The
/// drive model file is the one place omission reaches it (see above).
///
/// A family that deviates declares its own table and sets its axes'
/// cia402_profile to that instead; the rows below are not a
/// ceiling on what a vendor may declare; kNotReadyShutdownCia402Profile
/// below is one such table.
///
/// Provenance per row group:
///
/// - The sequence words (0x0006 / 0x0007 / 0x000F / 0x0002) are verified on
///   one drive family and showed no measured difference on a second.
/// - Transition 2 (SWITCH_ON_DISABLED → READY_TO_SWITCH_ON, "Shutdown") is
///   the entry into the powered sequence for a drive resting in 0x0040/0x0060,
///   and the only commanded move out of that state; without the row such a
///   drive holds there until the release times out, and on an axis whose brake
///   is independent of drive power the joint can sit reported braked with its
///   brake open. Like the Disable voltage
///   rows below, it is declared from the standard. Its absence is known to
///   stall; its acceptance is not hardware-confirmed. Confirmation needs a
///   release bench starting in SWITCH_ON_DISABLED.
/// - The Disable voltage rows carry 0x0000, the representative encoding of
///   bit 7 = 0 and bit 1 = 0. Their destinations (CiA 402 transitions
///   7/9/10/12, all SWITCH_ON_DISABLED) are declared from the standard and are
///   not hardware-confirmed; confirmation needs a bench. They are commandable;
///   absence of a row makes the transition undeclared.
/// - There are no FAULT / FAULT_REACTION_ACTIVE rows: fault reset is an edge
///   (see kCia402FaultResetControlword).
inline constexpr Cia402TransitionProfile kStandardCia402Profile{
    .transitions = {{
        // From SWITCH_ON_DISABLED: transition 2 ("Shutdown").
        {ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON, 0x0006},
        // From READY_TO_SWITCH_ON: transition 7 (Disable voltage / Quick stop)
        {ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED, 0x0000},
        {ServoState::READY_TO_SWITCH_ON, ServoState::SWITCHED_ON, 0x0007},
        {ServoState::READY_TO_SWITCH_ON, ServoState::OPERATION_ENABLED, 0x000F},
        // From SWITCHED_ON: transition 10 (Disable voltage / Quick stop)
        {ServoState::SWITCHED_ON, ServoState::SWITCH_ON_DISABLED, 0x0000},
        {ServoState::SWITCHED_ON, ServoState::READY_TO_SWITCH_ON, 0x0006},
        {ServoState::SWITCHED_ON, ServoState::OPERATION_ENABLED, 0x000F},
        // From OPERATION_ENABLED: transition 9 (Disable voltage)
        {ServoState::OPERATION_ENABLED, ServoState::SWITCH_ON_DISABLED, 0x0000},
        {ServoState::OPERATION_ENABLED, ServoState::QUICK_STOP_ACTIVE, 0x0002},
        {ServoState::OPERATION_ENABLED, ServoState::READY_TO_SWITCH_ON, 0x0006},
        {ServoState::OPERATION_ENABLED, ServoState::SWITCHED_ON, 0x0007},
        // From QUICK_STOP_ACTIVE: transition 12 (Disable voltage)
        {ServoState::QUICK_STOP_ACTIVE, ServoState::SWITCH_ON_DISABLED, 0x0000},
    }},
    .transition_count = 12,
};

/// @brief For a drive family that reports 0x0020 in NOT_READY and accepts
/// Shutdown there: kStandardCia402Profile plus the one compatibility row
/// NOT_READY_TO_SWITCH_ON + Shutdown → READY_TO_SWITCH_ON, first.
///
/// The row is outside CiA 402: the standard leaves NOT_READY_TO_SWITCH_ON
/// only by automatic transition 1 (see standard_transition_target). It exists
/// because this FSM starts in NOT_READY and such a drive reports 0x0020
/// there, then accepts Shutdown. A vendor observation, so it lives in this
/// named table, not in the standard one: a drive model file gets it only by
/// declaring the row. Built from kStandardCia402Profile, so no row is copied.
inline constexpr Cia402TransitionProfile kNotReadyShutdownCia402Profile = [] {
    static_assert(kStandardCia402Profile.transition_count < Cia402TransitionProfile::kMaxTransitions);
    Cia402TransitionProfile out{};
    out.transitions[0] = {ServoState::NOT_READY_TO_SWITCH_ON, ServoState::READY_TO_SWITCH_ON, 0x0006};
    for (size_t i = 0; i < kStandardCia402Profile.transition_count; ++i) {
        out.transitions[i + 1] = kStandardCia402Profile.transitions[i];
    }
    out.transition_count = kStandardCia402Profile.transition_count + 1;
    return out;
}();

/// The eight decodable CiA 402 states: the states a profile row may
/// reference (never UNKNOWN; UNKNOWN is the decoder's "hold current state"
/// sentinel, not a state a transition can leave from or land on).
inline constexpr std::array<ServoState, 8> kCia402ProfileStates{
    ServoState::NOT_READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED,
    ServoState::READY_TO_SWITCH_ON,     ServoState::SWITCHED_ON,
    ServoState::OPERATION_ENABLED,      ServoState::QUICK_STOP_ACTIVE,
    ServoState::FAULT_REACTION_ACTIVE,  ServoState::FAULT,
};

/// The name of a profile state (diagnostics only).
[[nodiscard]] inline constexpr const char* cia402_state_name(ServoState state) noexcept {
    switch (state) {
        case ServoState::NOT_READY_TO_SWITCH_ON:
            return "NOT_READY_TO_SWITCH_ON";
        case ServoState::SWITCH_ON_DISABLED:
            return "SWITCH_ON_DISABLED";
        case ServoState::READY_TO_SWITCH_ON:
            return "READY_TO_SWITCH_ON";
        case ServoState::SWITCHED_ON:
            return "SWITCHED_ON";
        case ServoState::OPERATION_ENABLED:
            return "OPERATION_ENABLED";
        case ServoState::QUICK_STOP_ACTIVE:
            return "QUICK_STOP_ACTIVE";
        case ServoState::FAULT:
            return "FAULT";
        case ServoState::FAULT_REACTION_ACTIVE:
            return "FAULT_REACTION_ACTIVE";
        default:
            return "UNKNOWN";
    }
}

namespace detail {

/// Controlword bit 10, reserved by IEC 61800-7-201, so a profile row must
/// leave it clear. Reserved is not "free for a vendor": a drive is entitled
/// to act on it the moment the standard assigns it a meaning.
inline constexpr uint16_t kCia402ReservedControlwordBit = 0x0400;

/// A state a profile row may reference: one of kCia402ProfileStates.
[[nodiscard]] inline constexpr bool is_valid_profile_state(ServoState state) noexcept {
    for (const ServoState listed : kCia402ProfileStates) {
        if (listed == state) {
            return true;
        }
    }
    return false;
}

/// @brief The command a CiA 402 controlword issues.
///
/// SWITCH_ON is also how "Disable operation" (transition 5) is encoded; the
/// two are one word, which is why the decode names both. FAULT_RESET is here
/// because the decode below is total and must be able to name what it found,
/// not because a profile row may carry it.
enum class Cia402Command : uint8_t {
    FAULT_RESET,
    DISABLE_VOLTAGE,
    QUICK_STOP,
    SHUTDOWN,
    SWITCH_ON,
    ENABLE_OPERATION,
};

/// @brief Decode a controlword into the command it issues.
///
/// The standard's don't-care order (IEC 61800-7-201 table "Command coding"):
/// bit 7 outranks everything, then bit 1 == 0, then bit 2 == 0, then
/// bit 0 == 0, then bit 3 == 0. The decode is total over the low nibble:
/// all sixteen values name a command (Disable voltage 0,1,4,5,8,9,C,D ·
/// Quick stop 2,3,A,B · Shutdown 6,E · Switch on 7 · Enable operation F),
/// which is why no "set of permitted controlwords" check can exist, and why
/// the bit rules and the per-row (from, command) → to match carry that job
/// instead.
///
/// This is a static classification of a word. It says nothing about whether
/// the drive was in a state that accepts the command, or acted on it.
[[nodiscard]] inline constexpr Cia402Command cia402_command(uint16_t controlword) noexcept {
    if ((controlword & kCia402FaultResetControlword) != 0) {
        return Cia402Command::FAULT_RESET;
    }
    if ((controlword & 0x0002) == 0) {
        return Cia402Command::DISABLE_VOLTAGE;
    }
    if ((controlword & 0x0004) == 0) {
        return Cia402Command::QUICK_STOP;
    }
    if ((controlword & 0x0001) == 0) {
        return Cia402Command::SHUTDOWN;
    }
    if ((controlword & 0x0008) == 0) {
        return Cia402Command::SWITCH_ON;
    }
    return Cia402Command::ENABLE_OPERATION;
}

/// Every command, so a caller can ask which of them reach a given state.
inline constexpr std::array<Cia402Command, 6> kCia402Commands{
    Cia402Command::FAULT_RESET, Cia402Command::DISABLE_VOLTAGE, Cia402Command::QUICK_STOP,
    Cia402Command::SHUTDOWN,    Cia402Command::SWITCH_ON,       Cia402Command::ENABLE_OPERATION,
};

/// One controlword that issues `command`, the encoding this repository's
/// profiles use. It is the inverse of cia402_command() on that one word, and
/// the suite pins the round trip; it is an example for a diagnostic, not the
/// only legal encoding (the decode is total, so every command has several).
[[nodiscard]] inline constexpr uint16_t cia402_representative_word(Cia402Command command) noexcept {
    switch (command) {
        case Cia402Command::FAULT_RESET:
            return kCia402FaultResetControlword;
        case Cia402Command::DISABLE_VOLTAGE:
            return 0x0000;
        case Cia402Command::QUICK_STOP:
            return 0x0002;
        case Cia402Command::SHUTDOWN:
            return 0x0006;
        case Cia402Command::SWITCH_ON:
            return 0x0007;
        case Cia402Command::ENABLE_OPERATION:
            return 0x000F;
    }
    return 0;
}

/// The standard's name for a decoded command (diagnostics only).
[[nodiscard]] inline constexpr const char* cia402_command_name(Cia402Command command) noexcept {
    switch (command) {
        case Cia402Command::FAULT_RESET:
            return "Fault reset";
        case Cia402Command::DISABLE_VOLTAGE:
            return "Disable voltage";
        case Cia402Command::QUICK_STOP:
            return "Quick stop";
        case Cia402Command::SHUTDOWN:
            return "Shutdown";
        case Cia402Command::SWITCH_ON:
            return "Switch on / Disable operation";
        case Cia402Command::ENABLE_OPERATION:
            return "Enable operation";
    }
    return "unrecognized command";
}

/// @brief Where the standard walk lands when `command` is issued in `from`,
/// or UNKNOWN when that pair is not a move this walk declares.
///
/// `state_machine.cpp` holds no table of its own; it looks the injected
/// profile up directly. This is the subset of CiA 402 those lookups walk:
/// transitions 2-12 and 16, plus the one compatibility row (see below). It is
/// deliberately smaller than CiA 402 itself; the scope note on
/// Cia402TransitionProfile says what widening it would take.
///
/// FAULT and FAULT_REACTION_ACTIVE have no entry at all: the only commanded
/// exit from FAULT is transition 15, which is an edge this profile may not
/// carry (kCia402FaultResetControlword), and FAULT_REACTION_ACTIVE leaves on
/// the drive's own schedule.
[[nodiscard]] inline constexpr ServoState standard_transition_target(ServoState from, Cia402Command command) noexcept {
    switch (from) {
        case ServoState::NOT_READY_TO_SWITCH_ON:
            // Compatibility row, not a standard transition. CiA 402 leaves
            // NOT_READY_TO_SWITCH_ON by transition 1, which is automatic and
            // lands in SWITCH_ON_DISABLED; no command reaches it. This row
            // exists because this FSM starts in NOT_READY and some drive
            // families report 0x0020 there and accept Shutdown (see
            // kNotReadyShutdownCia402Profile). It is the single exception,
            // and only for Shutdown: every other command out of NOT_READY is
            // rejected.
            return command == Cia402Command::SHUTDOWN ? ServoState::READY_TO_SWITCH_ON : ServoState::UNKNOWN;
        case ServoState::SWITCH_ON_DISABLED:
            // Transition 2: the only commanded move out of this state.
            return command == Cia402Command::SHUTDOWN ? ServoState::READY_TO_SWITCH_ON : ServoState::UNKNOWN;
        case ServoState::READY_TO_SWITCH_ON:
            switch (command) {
                case Cia402Command::SWITCH_ON:
                    return ServoState::SWITCHED_ON;  // transition 3
                case Cia402Command::ENABLE_OPERATION:
                    return ServoState::OPERATION_ENABLED;  // transitions 3+4 in one word
                case Cia402Command::DISABLE_VOLTAGE:
                case Cia402Command::QUICK_STOP:
                    return ServoState::SWITCH_ON_DISABLED;  // transition 7
                default:
                    return ServoState::UNKNOWN;
            }
        case ServoState::SWITCHED_ON:
            switch (command) {
                case Cia402Command::ENABLE_OPERATION:
                    return ServoState::OPERATION_ENABLED;  // transition 4
                case Cia402Command::SHUTDOWN:
                    return ServoState::READY_TO_SWITCH_ON;  // transition 6
                case Cia402Command::DISABLE_VOLTAGE:
                case Cia402Command::QUICK_STOP:
                    return ServoState::SWITCH_ON_DISABLED;  // transition 10
                default:
                    return ServoState::UNKNOWN;
            }
        case ServoState::OPERATION_ENABLED:
            switch (command) {
                case Cia402Command::SWITCH_ON:
                    return ServoState::SWITCHED_ON;  // transition 5 (Disable operation)
                case Cia402Command::SHUTDOWN:
                    return ServoState::READY_TO_SWITCH_ON;  // transition 8
                case Cia402Command::DISABLE_VOLTAGE:
                    return ServoState::SWITCH_ON_DISABLED;  // transition 9
                case Cia402Command::QUICK_STOP:
                    return ServoState::QUICK_STOP_ACTIVE;  // transition 11
                default:
                    return ServoState::UNKNOWN;
            }
        case ServoState::QUICK_STOP_ACTIVE:
            switch (command) {
                case Cia402Command::DISABLE_VOLTAGE:
                    return ServoState::SWITCH_ON_DISABLED;  // transition 12
                case Cia402Command::ENABLE_OPERATION:
                    return ServoState::OPERATION_ENABLED;  // transition 16 (option-code dependent)
                default:
                    return ServoState::UNKNOWN;
            }
        default:
            return ServoState::UNKNOWN;
    }
}

/// One (from, to) pair the sequencer looks up directly, so a profile that
/// omits it holds instead of walking. The command that makes the pair legal
/// is not stored here; it is derived from standard_transition_target, which
/// is the one authority on the walk.
struct Cia402RequiredTransition {
    uint8_t number;  ///< CiA 402 transition number
    ServoState from;
    ServoState to;
};

/// @brief The rows every profile must declare.
///
/// The executor does no path search: it looks up (current state, current
/// goal) and holds when the row is absent (`state_machine.cpp`,
/// `controller.cpp`). So "the walk is declarable" is exactly "these ten pairs
/// are present".
///
/// Transition 12 is unconditional. QUICK_STOP_ACTIVE is reachable without any
/// command of ours (a local or external quick stop puts the drive there), so
/// the exit cannot be required only when OPERATION_ENABLED → QUICK_STOP_ACTIVE
/// is declared.
///
/// NOT_READY_TO_SWITCH_ON is deliberately not required: that row is a
/// compatibility row, not a standard transition, and a profile that omits it
/// is absorbed by automatic transition 1 followed by SWITCH_ON_DISABLED →
/// READY_TO_SWITCH_ON.
///
/// This is declaration completeness. It does not promise the physical walk
/// closes, that runtime makes progress, or that the profile is a general
/// CiA 402 implementation. It is coupled to the walk on purpose: change the
/// walk or the unknown-state policy and this table changes with it.
inline constexpr std::array<Cia402RequiredTransition, 10> kCia402RequiredTransitions{{
    {2, ServoState::SWITCH_ON_DISABLED, ServoState::READY_TO_SWITCH_ON},
    {3, ServoState::READY_TO_SWITCH_ON, ServoState::SWITCHED_ON},
    {4, ServoState::SWITCHED_ON, ServoState::OPERATION_ENABLED},
    {5, ServoState::OPERATION_ENABLED, ServoState::SWITCHED_ON},
    {6, ServoState::SWITCHED_ON, ServoState::READY_TO_SWITCH_ON},
    {7, ServoState::READY_TO_SWITCH_ON, ServoState::SWITCH_ON_DISABLED},
    {8, ServoState::OPERATION_ENABLED, ServoState::READY_TO_SWITCH_ON},
    {9, ServoState::OPERATION_ENABLED, ServoState::SWITCH_ON_DISABLED},
    {10, ServoState::SWITCHED_ON, ServoState::SWITCH_ON_DISABLED},
    {12, ServoState::QUICK_STOP_ACTIVE, ServoState::SWITCH_ON_DISABLED},
}};

/// Does the profile carry a row for this pair? (Its command word is checked
/// per row, by the standard-pairing rule.)
[[nodiscard]] inline constexpr bool declares_cia402_transition(const Cia402TransitionProfile& profile, ServoState from,
                                                               ServoState to) noexcept {
    for (size_t i = 0; i < profile.transition_count; ++i) {
        if (profile.transitions[i].from == from && profile.transitions[i].to == to) {
            return true;
        }
    }
    return false;
}

/// The commands that reach `to` from `from`, named with one example word
/// each, "Disable voltage (0x0000) or Quick stop (0x0002)". Empty when no
/// command reaches it; the suite checks this never happens for a required
/// pair.
[[nodiscard]] inline std::string cia402_commands_reaching(ServoState from, ServoState to);

/// "0x000F": a controlword is read as hex everywhere else in this contract.
[[nodiscard]] inline std::string cia402_hex_word(uint16_t value) {
    constexpr char kDigits[] = "0123456789ABCDEF";
    std::string out = "0x";
    for (int shift = 12; shift >= 0; shift -= 4) {
        out += kDigits[(value >> shift) & 0xF];
    }
    return out;
}

inline std::string cia402_commands_reaching(ServoState from, ServoState to) {
    std::string out;
    if (to == ServoState::UNKNOWN) {
        // standard_transition_target returns UNKNOWN for "no such move", so
        // asking which commands reach UNKNOWN would answer "all of them".
        return out;
    }
    for (const Cia402Command command : kCia402Commands) {
        if (command == Cia402Command::FAULT_RESET || standard_transition_target(from, command) != to) {
            continue;
        }
        if (!out.empty()) {
            out += " or ";
        }
        out += std::string(cia402_command_name(command)) + " (" + cia402_hex_word(cia402_representative_word(command)) +
               ")";
    }
    return out;
}

}  // namespace detail

/// @brief Validate a declared profile. ok when valid, else an error whose message
/// answers what/why/how (init-path use only; allocates).
///
/// Structure: non-empty (absent = undeclared, the mandatory-declaration
/// contract), count within capacity, then per row:
///
/// - both endpoints are decodable states, never UNKNOWN;
/// - the row is not a self-transition (`from == to`);
/// - the controlword carries neither bit 7 (fault reset is edge-owned
///   elsewhere; see kCia402FaultResetControlword) nor bit 10 (reserved);
/// - no other row repeats its (from, to) pair, which would make the effective
///   controlword order-dependent;
/// - its (from, command) pair is one the standard walk declares, and lands
///   where the row says it lands;
///
/// and finally that the ten required pairs are present
/// (detail::kCia402RequiredTransitions). Not every pair the sequencer may look
/// up: the optional rows a profile is allowed but not obliged to declare
/// (READY_TO_SWITCH_ON → OPERATION_ENABLED, transition 11, transition 16) are
/// looked up too, and their absence holds, which is a declaration the vendor
/// is entitled to make, not an error.
///
/// What the bit rules do not check: bits 4-6 and 9 (mode-specific), 8 (halt)
/// and 11-15 (manufacturer-specific) are carried through untouched. Unchecked
/// is not the same as safe or meaningless; those bits reach the drive.
///
/// Order: rows are visited in declaration order and the first failure is
/// returned, so a profile with several defects surfaces them one at a time.
/// The promise is per row: within a row the checks run in the order listed
/// above, which is what the suite pins. There is no cross-row promise: an
/// early row's pairing error speaks before a late row's duplicate.
///
/// A controlword of 0x0000 is a valid row and a real command: it is CiA 402
/// "Disable voltage" (bit 7 = 0 ∧ bit 1 = 0). "Undeclared" is the absence of
/// a row, which is how ServoStateMachine::can_transition_to reads it.
[[nodiscard]] inline core::Result<void> validate_cia402_profile(const Cia402TransitionProfile& profile) {
    if (profile.transition_count == 0) {
        return core::Result<void>::error(
            "CiA 402 transition profile is absent (transition_count == 0) — the profile is a mandatory "
            "model declaration with no fallback; set the axis's cia402_profile to kStandardCia402Profile, "
            "or declare the drive's own transition table and set it to that");
    }
    if (profile.transition_count > Cia402TransitionProfile::kMaxTransitions) {
        return core::Result<void>::error(
            "CiA 402 transition profile declares " + std::to_string(profile.transition_count) +
            " transitions — exceeds the fixed capacity (" + std::to_string(Cia402TransitionProfile::kMaxTransitions) +
            "); raise kMaxTransitions if the model needs more rows");
    }
    for (size_t i = 0; i < profile.transition_count; ++i) {
        const auto& t = profile.transitions[i];
        const std::string row = "CiA 402 transition profile row " + std::to_string(i);
        if (!detail::is_valid_profile_state(t.from) || !detail::is_valid_profile_state(t.to)) {
            return core::Result<void>::error(
                row + " references an invalid state (from=" + std::to_string(static_cast<uint16_t>(t.from)) +
                ", to=" + std::to_string(static_cast<uint16_t>(t.to)) +
                ") — rows must use the eight decodable CiA 402 states, never UNKNOWN");
        }
        if (t.from == t.to) {
            return core::Result<void>::error(
                row + " declares " + cia402_state_name(t.from) +
                " → itself; a self row would replace the core's hold word for that state. Delete it (the core " +
                "already holds) or fix the wrong endpoint");
        }
        if ((t.controlword & kCia402FaultResetControlword) != 0) {
            return core::Result<void>::error(
                row + " carries controlword bit 7 (fault reset); a profile row would send it as a repeated " +
                "level, never a second edge. If the row only resets (bare 0x0080), delete it and use the core's "
                "fault-reset request; otherwise clear bit 7 and check the remaining word is the command that "
                "transition needs");
        }
        if ((t.controlword & detail::kCia402ReservedControlwordBit) != 0) {
            return core::Result<void>::error(
                row + " sets controlword bit 10 in " + detail::cia402_hex_word(t.controlword) +
                " — IEC 61800-7-201 reserves that bit and requires it clear; a drive is entitled to act on it " +
                "once the standard assigns it a meaning. Clear the bit and check the remaining word is still " +
                "the command this transition needs");
        }
        for (size_t j = i + 1; j < profile.transition_count; ++j) {
            if (profile.transitions[j].from == t.from && profile.transitions[j].to == t.to) {
                return core::Result<void>::error(
                    "CiA 402 transition profile rows " + std::to_string(i) + " and " + std::to_string(j) +
                    " duplicate the same (from, to) pair — the effective controlword would be "
                    "order-dependent; keep exactly one row per requested transition");
            }
        }
        const detail::Cia402Command command = detail::cia402_command(t.controlword);
        const ServoState standard_to = detail::standard_transition_target(t.from, command);
        const auto declared = [&] {
            return row + " declares " + cia402_state_name(t.from) + " → " + cia402_state_name(t.to) +
                   " with controlword " + detail::cia402_hex_word(t.controlword) + " (" +
                   detail::cia402_command_name(command) + ")";
        };
        if (standard_to == ServoState::UNKNOWN) {
            return core::Result<void>::error(
                declared() + ", but that command is not one the standard walk issues from " +
                cia402_state_name(t.from) +
                "; correct the command word or drop the row (shortcuts outside this walk are not supported)");
        }
        if (standard_to != t.to) {
            return core::Result<void>::error(
                declared() + ", but this walk maps that command to " + cia402_state_name(standard_to) + ". Name " +
                cia402_state_name(standard_to) + " as the destination, or use the command word that reaches " +
                cia402_state_name(t.to) +
                ". (This walk maps a subset of CiA 402 plus one compatibility row, NOT_READY_TO_SWITCH_ON + " +
                "Shutdown → READY_TO_SWITCH_ON.)");
        }
    }
    for (const auto& required : detail::kCia402RequiredTransitions) {
        if (!detail::declares_cia402_transition(profile, required.from, required.to)) {
            return core::Result<void>::error(
                "CiA 402 transition profile does not declare CiA 402 transition " + std::to_string(required.number) +
                " (" + cia402_state_name(required.from) + " → " + cia402_state_name(required.to) +
                ") — the sequencer looks that pair up directly and has no command to send when the row is " +
                "absent, so the walk cannot be commanded through that step. Add the row with a " +
                detail::cia402_commands_reaching(required.from, required.to) + " command word");
        }
    }
    return core::Result<void>::ok();
}

}  // namespace plem::fieldbus
