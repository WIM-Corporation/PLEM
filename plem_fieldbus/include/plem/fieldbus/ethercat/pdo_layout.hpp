// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

namespace plem::fieldbus::ethercat {

/**
 * @file pdo_layout.hpp
 * @brief Description of a slave's PDO mapping, declared by a model, or
 * assembled from the master's scan (pdo_scan.hpp)
 *
 * Two per-slave policies, told apart by the same bit at every layer
 * (SlaveSpec::pdo_mapping.sync_count / ServoInfo::pdo_mapping pointer: non-null or > 0 = declared, `= nullptr`,
 * 0, null = scanned; a drive-model file writes `pdo_mapping: scanned`):
 * - Declared: the bus writes the layout at PREOP (0x1c12/0x1c13 are writable
 *   only before master activation). For a slave whose power-on assignment is
 *   incomplete this is the only way past an opaque ENOENT at registration.
 *   A drive that does not accept PDO reconfiguration takes the write as an
 *   assertion of its own image; if the two differ the drive refuses the
 *   image after activation (AL 0x001E); nothing before activation reports
 *   that.
 * - Scanned: the bus reads the slave's current mapping from the master at
 *   initialize (before any device is created), assembles it into this same
 *   shape and writes nothing. The drive's own image is taken as it is.
 * Both are checked by one validator (validate_pdo_mapping) against every axis
 * of the slave, before activation; a mapping that lacks a required role is
 * refused by name either way. The policy is always explicit: a drive model
 * file without the key does not load.
 *
 * Gaps are load-bearing. An entry with index 0x0000 is padding: not
 * registerable, but it occupies image space. Dropping a gap shifts every
 * following entry and silently moves axis 2 off its byte offset. Mirror the
 * device exactly, as dumped by `ethercat -m 0 pdos`.
 *
 * Plain data on purpose (no ecrt.h), so a C++ caller can declare a layout
 * without dragging master headers into its package. Lifetime: a declared
 * layout must outlive the bus (model declarations are static constexpr).
 */

/// @brief Number of sync managers a slave can expose (IgH EC_MAX_SYNC_MANAGERS,
/// mirrored here so declarations and their validation stay ecrt-free);
/// valid sm_index values are 0..kSyncManagerCount-1. This is the master's
/// index range, not a declaration's storage capacity (a model's value spec
/// may hold fewer SyncManagerAssignments than that).
inline constexpr uint8_t kSyncManagerCount = 16;

/// @brief One entry within a PDO. index 0x0000 means padding (a gap).
struct PdoEntryMapping {
    uint16_t index;      ///< Object index, or 0x0000 for a gap
    uint8_t subindex;    ///< Object subindex
    uint8_t bit_length;  ///< Width in bits (gaps still occupy their width)
    constexpr bool operator==(const PdoEntryMapping&) const = default;
};

/// @brief One PDO and the entries mapped into it
struct PdoMapping {
    uint16_t index;                  ///< PDO index, e.g. 0x1600 / 0x1a00
    const PdoEntryMapping* entries;  ///< Entry list, in device order
    uint8_t entry_count;
};

/// @brief One sync manager and the PDOs assigned to it
///
/// `watchdog` is the core's explicit sync-manager watchdog setting for a
/// declared layout, stated per SM and written at application time
/// (EC_WD_DEFAULT / EC_WD_DISABLE). A scanned layout carries what the master
/// reports for the slave (its decode of the SM control register) and the core
/// writes nothing: the drive keeps its own setting.
struct SyncManagerAssignment {
    uint8_t sm_index;        ///< Sync manager number (0..kSyncManagerCount-1)
    bool output;             ///< true = master->slave (Rx), false = slave->master (Tx)
    const PdoMapping* pdos;  ///< Assigned PDOs (may be null for mailbox SMs)
    uint8_t pdo_count;
    bool watchdog;  ///< Enable the sync-manager watchdog for this SM
};

/// @brief A slave's complete sync-manager / PDO configuration
///
/// Declared by the caller (ServoInfo::pdo_mapping) and applied by EtherCATBus once per slave, not
/// once per axis. A multi-axis slave has several DeviceServo instances sharing
/// one ec_slave_config_t, so applying it per device would have each one
/// overwrite the last. Every axis of a slave must therefore point at the same
/// layout object; a slave whose axes disagree is rejected fail-fast at
/// initialize() (never silently skipped).
struct SlavePdoMapping {
    const SyncManagerAssignment* syncs;
    uint8_t sync_count;
};

}  // namespace plem::fieldbus::ethercat
