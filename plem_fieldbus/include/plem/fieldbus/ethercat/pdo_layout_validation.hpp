// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <span>
#include <string>

#include <plem/core/result.hpp>

#include "plem/fieldbus/cia402_types.hpp"
#include "plem/fieldbus/ethercat/pdo_layout.hpp"

namespace plem::fieldbus::ethercat {

/**
 * @brief What one axis requires from the PDO layout of the slave hosting it.
 *
 * Filled by the bus from ServoInfo: the channel offset, the declared mode of operation and
 * the ErrorCode role all ride that one carry. Standard CiA 402 roles resolve
 * at `base + od_offset`; the ErrorCode role resolves at
 * `error_code_pdo.index + od_offset` (the declared index is the channel-1
 * base; the offset is added exactly once, like every other standard entry;
 * see ServoInfo::error_code_pdo).
 */
struct AxisPdoRequirements {
    /// CiA 402 channel base (0, 0x800, …). The caller has already checked the
    /// 0x800 grid (ServoInfo::is_valid); address arithmetic here is 32-bit
    /// and rejects > 0xFFFF independently.
    uint16_t od_offset = 0;

    /// Selects the required target role (CST 0x6071, CSP 0x607A, CSV 0x60FF).
    /// A mode without a command path is rejected.
    OperationMode mode = OperationMode::CYCLIC_SYNC_TORQUE;

    /// ErrorCode role: index 0 = undeclared. When declared, the layout must
    /// map the resolved index at exactly this subindex and bit_length.
    PdoEntryMapping error_code_pdo{0, 0, 0};
};

/**
 * @brief Semantic validation of a declared PDO layout against the axes that
 * share it. Returns ok, or the refusal naming the defect: structural defects by SM/PDO/entry position, role
 * defects by role name, resolved `index:subindex` and axis position within
 * `axes` (the caller adds the physical slave and servo ids). Same convention
 * as validate_cia402_profile.
 *
 * Pure and ecrt-free; non-RT, called by a caller's assembly step
 * (structure only) and by the bus's initialize() Step 0 (full call), see
 * Boundaries below. Checks:
 * - structure: count>0 with a null list; sm_index range and uniqueness; PDO
 *   index 0, duplicated, or outside its sync manager's mapping range (RxPDO
 *   0x1600–0x17FF in output SMs, TxPDO 0x1A00–0x1BFF in input SMs);
 *   entry-less PDOs (the master reads `n_entries == 0` as "use the slave's
 *   default mapping"; a declaration must list what it writes, and a scan
 *   that shows none holds nothing the core could register); zero-width
 *   entries (gaps included);
 * - per axis: every role the mode requires is mapped; every mapped role
 *   (optional ones too: a mapped target is written) has the subindex, the
 *   fixed read/write width and the direction the core uses, is mapped once,
 *   and starts on a byte boundary (registration passes no bit position, so
 *   the master would refuse a misaligned entry only after hardware contact);
 * - across the slave: no two mapped (axis, role) requirements resolve to the
 *   same (index, subindex); one entry cannot feed two consumers.
 *
 * Division of labour with the caller: the 0x800 channel grid, the validity
 * of each axis's own fields and the ErrorCode decode width (16:
 * ServoPDOHandler reads it with EC_READ_U16) are checked before this call;
 * the layout itself and the resolved-address overflow are checked here. A
 * declared-vs-mapped width agreement says nothing about RT support for other
 * widths.
 *
 * Boundaries: an empty `axes` runs the structural checks only and may be
 * accepted (acceptance is not motion admission); that is how an assembly
 * layer checks each slave's materialized
 * graph before any bus contact; the bus's initialize()
 * Step 0 then runs the full call with the slave's axes for a declared
 * layout; a scanned slave's layout (pdo_scan.hpp) arrives here at Step 3
 * with the same axes. A layout with sync_count 0 and a non-empty `axes` is
 * rejected for missing roles (the scan assembler refuses a slave the master
 * reports with no sync manager before it gets here). Null lists are
 * rejected before any dereference; storage length and dangling pointers are
 * not provable here.
 */
[[nodiscard]] core::Result<void> validate_pdo_mapping(const SlavePdoMapping& layout,
                                                      std::span<const AxisPdoRequirements> axes);

}  // namespace plem::fieldbus::ethercat
