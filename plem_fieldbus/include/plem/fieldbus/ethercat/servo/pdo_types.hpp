// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include <plem/core/units.hpp>

#include "plem/fieldbus/cia402_types.hpp"

namespace plem::fieldbus::ethercat {

using plem::core::units::Pulse;
using plem::core::units::PulsePerSecond;
using plem::core::units::TorqueCount;
using plem::fieldbus::OperationMode;  // re-exported for code that uses this namespace

/**
 * @brief PDO data structure for servo communication
 *
 * This structure represents the Process Data Objects (PDO) exchanged
 * between the EtherCAT master and servo drives according to CiA 402.
 */
struct ServoPDOData {
    // RxPDO (Master to Slave)
    uint16_t controlword{0};           ///< Controlword (0x6040)
    int8_t modes_of_operation{0};      ///< 0x6060 Modes of operation (INTEGER8)
    Pulse target_position{};           ///< Target position in encoder pulses
    PulsePerSecond target_velocity{};  ///< Target velocity in velocity counts (AxisFactors velocity_factor)
    TorqueCount target_torque{};       ///< Target torque in counts (scaled)

    // TxPDO (Slave to Master)
    uint16_t statusword{0};                  ///< Statusword (0x6041)
    int8_t modes_of_operation_display{0};    ///< 0x6061 Modes of operation display (INTEGER8)
    Pulse position_actual_value{};           ///< Actual position in encoder pulses
    PulsePerSecond velocity_actual_value{};  ///< Actual velocity in velocity counts (AxisFactors velocity_factor)
    TorqueCount torque_actual_value{};       ///< Actual torque in counts (scaled)

    /// @brief Error code (0x603F). Zero when the drive is healthy.
    ///
    /// A later SDO read can miss a transient 0x603F value, for example after
    /// a successful fault reset. NONE does not reset automatically;
    /// BOUNDED_AUTO_RESET uses declared pacing and an attempt cap and does not
    /// guarantee a successful reset. The same-frame PDO value is retained here.
    /// Populated only when the model declares the ErrorCode role (ServoInfo);
    /// undeclared models never register it and this field stays 0.
    uint16_t error_code{0};

    /// @brief Raw 0x6064 word as received (before offset and flip); the CSP
    /// mirror copies it verbatim; decoding into `position_actual_value` saturates.
    int32_t position_actual_value_word{0};
    /// @brief Raw 0x607A word a CSP axis sends (ServoPDOHandler writes it
    /// verbatim instead of encoding `target_position`).
    int32_t target_position_word{0};
};

/// @brief Offset value meaning "this object is not mapped by the slave".
///
/// Not every drive maps every CiA 402 object (a vendor may map no target
/// velocity 0x60FF at all). A zero offset cannot mean "absent" because zero is
/// a legitimate offset, and writing to it would land on top of the
/// controlword every cycle. Every read/write of a PDO field checks against
/// this first (ServoPDOHandler).
inline constexpr uint32_t kUnmappedPdoOffset = 0xFFFFFFFFu;

/**
 * @brief PDO offset structure for domain memory access
 *
 * This structure stores the byte offsets of PDO entries within the
 * EtherCAT domain memory. These offsets are calculated during PDO
 * registration and used for efficient data access.
 *
 * Every offset starts UNMAPPED. Registration overwrites the ones the slave
 * actually maps; anything the slave omits stays unmapped and is skipped by
 * read()/write() rather than aliasing offset 0; a default-constructed offsets
 * object means "nothing mapped", not "everything at offset 0".
 */
struct ServoPDOOffsets {
    // RxPDO offsets
    uint32_t controlword{kUnmappedPdoOffset};         ///< Offset of the controlword
    uint32_t modes_of_operation{kUnmappedPdoOffset};  ///< Offset of Modes of operation
    uint32_t target_position{kUnmappedPdoOffset};     ///< Offset for target position
    uint32_t target_velocity{kUnmappedPdoOffset};     ///< Offset for target velocity
    uint32_t target_torque{kUnmappedPdoOffset};       ///< Offset for target torque

    // TxPDO offsets
    uint32_t statusword{kUnmappedPdoOffset};                  ///< Offset of the statusword
    uint32_t modes_of_operation_display{kUnmappedPdoOffset};  ///< Offset of Modes of operation display
    uint32_t position_actual_value{kUnmappedPdoOffset};       ///< Offset for actual position
    uint32_t velocity_actual_value{kUnmappedPdoOffset};       ///< Offset for actual velocity
    uint32_t torque_actual_value{kUnmappedPdoOffset};         ///< Offset for actual torque
    uint32_t error_code{kUnmappedPdoOffset};                  ///< Offset for CiA 402 error code (0x603F, optional role)
};

}  // namespace plem::fieldbus::ethercat
