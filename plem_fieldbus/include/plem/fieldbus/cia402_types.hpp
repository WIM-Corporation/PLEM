// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

namespace plem::fieldbus {

/**
 * @brief Drive states according to CiA 402 standard
 */
enum class ServoState : uint16_t {
    NOT_READY_TO_SWITCH_ON = 0x0000,
    SWITCH_ON_DISABLED = 0x0040,
    READY_TO_SWITCH_ON = 0x0021,
    SWITCHED_ON = 0x0023,
    OPERATION_ENABLED = 0x0027,
    QUICK_STOP_ACTIVE = 0x0007,
    FAULT = 0x0008,
    FAULT_REACTION_ACTIVE = 0x000F,
    UNKNOWN = 0xFFFF,
};

/**
 * @brief Modes of operation according to CiA 402. 0x6060 Modes of operation and
 * 0x6061 Modes of operation display are INTEGER8 (negative values are vendor
 * modes), so the type is signed.
 */
enum class OperationMode : int8_t {
    NO_MODE = 0,
    CYCLIC_SYNC_POSITION = 8,
    CYCLIC_SYNC_VELOCITY = 9,
    CYCLIC_SYNC_TORQUE = 10
};

/// @brief When a drive first reports the declared mode on 0x6061, a
/// per-drive-family fact the model declares (mandatory; UNDECLARED is the
/// absence sentinel the spec gates refuse).
///
/// PRE_ENABLE: the display follows the 0x6060 command in the idle state,
/// with no enable needed: the strict reading, and the conservative choice
/// when the echo's order against the first enable is not known. POST_ENABLE:
/// the display may read 0 in the idle state before the first enable (0x6061 = 0
/// throughout an idle READY_TO_SWITCH_ON while 0x6060 already holds the mode)
/// and first shows the mode at some later transition.
enum class ModeDisplayTiming : uint8_t {
    UNDECLARED = 0,  ///< absence sentinel, never a valid declaration
    PRE_ENABLE,
    POST_ENABLE,
};

/// @brief Can the bus command this mode? One rule, read by every layer that
/// checks what the bus is handed (ServoInfo::is_valid, the PDO role table, the
/// drive-layer value gate): the three cyclic synchronous modes, each with its
/// own target and role table. A mode without a command path is rejected. A host may be narrower (e.g. CST only, at its
/// own gate), so this predicate alone does not admit a host's axes.
[[nodiscard]] inline constexpr bool operation_mode_supported(OperationMode mode) noexcept {
    return mode == OperationMode::CYCLIC_SYNC_TORQUE || mode == OperationMode::CYCLIC_SYNC_POSITION ||
           mode == OperationMode::CYCLIC_SYNC_VELOCITY;
}

/// @brief Is the reporting timing declared? (Both known values; the
/// UNDECLARED sentinel and any other number are not.) Same single-rule
/// contract as operation_mode_supported.
[[nodiscard]] inline constexpr bool mode_display_timing_declared(ModeDisplayTiming display) noexcept {
    return display == ModeDisplayTiming::PRE_ENABLE || display == ModeDisplayTiming::POST_ENABLE;
}

}  // namespace plem::fieldbus
