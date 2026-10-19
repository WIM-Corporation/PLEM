// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <plem/core/result.hpp>

#include "plem/fieldbus/ethercat/pdo_layout.hpp"

// ecrt.h only forward-declares the master handle; the same forward declaration
// keeps this header ecrt-free for its consumers (the bus includes ecrt.h itself).
struct ec_master;
typedef struct ec_master ec_master_t;

namespace plem::fieldbus::ethercat {

/**
 * @brief A slave's PDO mapping as the master scanned it, assembled into the
 * SlavePdoMapping shape a model declares, so the one validator
 * (validate_pdo_mapping) and the one registration filter
 * (DeviceServo::pdo_mapping_carries) check a scanned slave exactly as a declared one.
 *
 * Owns its storage. The vectors are reserved before the pointer graph is built
 * over them, so `view` and the pointers inside it stay valid for the object's
 * life, and across a move (the heap buffers travel with the vectors). A copy
 * would carry pointers into the source's buffers, so copying is deleted.
 */
struct ScannedPdoMapping {
    std::vector<PdoEntryMapping> entries;
    std::vector<PdoMapping> pdos;
    std::vector<SyncManagerAssignment> syncs;
    SlavePdoMapping view{nullptr, 0};
    /// The identity the master reports for the slave (SII), the one source
    /// for everything a log line or a refusal quotes about the firmware it saw.
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    uint32_t revision_number = 0;

    ScannedPdoMapping() = default;
    ScannedPdoMapping(ScannedPdoMapping&&) noexcept = default;
    ScannedPdoMapping& operator=(ScannedPdoMapping&&) noexcept = default;
    ScannedPdoMapping(const ScannedPdoMapping&) = delete;
    ScannedPdoMapping& operator=(const ScannedPdoMapping&) = delete;
};

/**
 * @brief Read the slave's current sync-manager / PDO assignment from the
 * master (ecrt_master_get_slave → get_sync_manager → get_pdo → get_pdo_entry)
 * into a ScannedPdoMapping.
 *
 * Blocking master queries; initialization context only, after the master is
 * requested and before it is activated. A query that fails (ret != 0) fails
 * the scan with a message naming the slave and the SM/PDO/entry position; a
 * slave the master reports with no sync manager fails with its own message
 * naming the causes this call cannot tell apart (the master's scan of it is
 * not complete, a different device (one without sync managers) sits at the
 * position because the wiring changed, or the master needs a rescan), plus
 * the vendor/product the master reports so the operator can identify the
 * device.
 *
 * Direction and watchdog are taken as the master reports them (its decode of
 * the SM control register): EC_DIR_OUTPUT → `output`, EC_WD_ENABLE →
 * `watchdog`. The result states what the master currently holds for the
 * slave; whether the drive accepts that mapping at activation is not
 * observable here.
 */
[[nodiscard]] core::Result<ScannedPdoMapping> scan_pdo_mapping(ec_master_t* master, uint16_t position);

/**
 * @brief The layout as drive-model-file text: the `pdo_mapping` of
 * cia402-drive-model.md §4.1, from `sync_managers:` down, one
 * line per mailbox SM and per entry. What a bring-up log prints for a scanned
 * slave and what a rejection quotes as the observed mapping. Takes a
 * structurally sound layout (a count never paired with a null list, what
 * scan_pdo_mapping assembles and what the validator's structure check
 * admits); it does not defend against one that is not.
 */
[[nodiscard]] std::string format_pdo_mapping(const SlavePdoMapping& layout);

/**
 * @brief `sha256:` + core::crypto::sha256_hex of format_pdo_mapping(layout):
 * the one digest of a mapping: what the INFO summary line prints and what an
 * approval record's `pdo_mapping_digest` is compared with (configuration.hpp
 * SlaveIdentity). Its input is the format text alone, so the digest changes
 * exactly when that text does. Bytes pinned by test_pdo_scan_fake_master.cpp.
 */
[[nodiscard]] std::string pdo_mapping_digest(const SlavePdoMapping& layout);

}  // namespace plem::fieldbus::ethercat
