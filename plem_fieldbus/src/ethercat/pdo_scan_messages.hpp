// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "plem/fieldbus/ethercat/pdo_scan.hpp"

// Package-private (deliberately not an installed header): the texts the bus
// logs and refuses with around a scanned PDO mapping (pdo_scan.cpp).

namespace plem::fieldbus::ethercat {

/**
 * @brief The one-line summary of a scanned layout the bus logs at INFO for
 * each scanned slave: `<S> sync managers, <P> PDOs, <E> entries, revision
 * 0x<R>, <pdo_mapping_digest>`. Both texts are a contract an operator reads
 * back (the digest pairs the log line with the mapping text); their bytes
 * are pinned by test_pdo_scan_fake_master.cpp.
 */
[[nodiscard]] std::string describe_scanned_mapping(const ScannedPdoMapping& layout);

/**
 * @brief The refusal the bus logs when a scanned mapping does not serve the
 * slave's axes (validate_pdo_mapping returned `validator_error`): the slave,
 * its axes (`servos`, the "0,3" list), the identity the master scanned
 * (vendor/product/revision, one source), the validator's finding, the
 * observed mapping (format_pdo_mapping) and what to do. Both remedies,
 * configuring the drive to map the object, or declaring the mapping so the
 * core writes it in PREOP, need a drive that accepts PDO reconfiguration; a
 * drive with a fixed mapping that lacks the role cannot be driven in this
 * mode, and the message says so. Bytes pinned by
 * test_pdo_scan_fake_master.cpp.
 */
[[nodiscard]] std::string describe_scan_rejection(uint16_t slave, const std::string& servos,
                                                  const ScannedPdoMapping& layout, const std::string& validator_error);

/**
 * @brief The refusal the bus logs when the approval record's
 * `pdo_mapping_digest` differs from the digest of the mapping it scanned for
 * that slave: both digests, the causes this comparison cannot tell apart
 * (drive or firmware swapped · a unit of another profile · the record
 * mistyped · an assignment another tool wrote · a CoE read failure during
 * the master's scan leaving SII values · a stale master scan) and what to do
 * (nameplate, re-bench, re-approve). Bytes pinned by
 * test_pdo_scan_fake_master.cpp.
 */
[[nodiscard]] std::string describe_digest_mismatch(uint32_t slave, std::string_view scanned, std::string_view declared);

/**
 * @brief The refusal the bus logs when the record declares a
 * `pdo_mapping_digest` for a slave whose mapping is declared (not scanned):
 * there is no scanned mapping to compare with; the loader refuses such a
 * record, a spec assembled another way can still carry one, and the bus
 * treats it as a mismatch, never a skip. Bytes pinned by
 * test_pdo_scan_fake_master.cpp.
 */
[[nodiscard]] std::string describe_digest_without_scan(uint32_t slave, std::string_view declared);

}  // namespace plem::fieldbus::ethercat
