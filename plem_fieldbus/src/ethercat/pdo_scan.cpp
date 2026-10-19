// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "ethercat/pdo_scan_messages.hpp"

#include <limits>

#include <ecrt.h>
#include <fmt/format.h>

#include <plem/core/crypto/sha256.hpp>

namespace plem::fieldbus::ethercat {

core::Result<ScannedPdoMapping> scan_pdo_mapping(ec_master_t* master, uint16_t position) {
    using R = core::Result<ScannedPdoMapping>;

    ec_slave_info_t info{};
    if (const int ret = ecrt_master_get_slave(master, position, &info); ret != 0) {
        return R::error(fmt::format(
            "slave {}: ecrt_master_get_slave failed ({}) — the position may hold no slave, or the master's scan of "
            "the ring is not complete: check `ethercat slaves`, then retry (or `ethercat rescan`)",
            position, ret));
    }
    if (info.sync_count == 0) {
        return R::error(fmt::format(
            "slave {}: the master reports no sync manager for it (vendor 0x{:x} product 0x{:x}, al_state 0x{:x}) — "
            "the master's scan of it is not complete, or a different device sits at this position (one without "
            "sync managers — the wiring changed; compare the vendor/product with the model's), or the master needs "
            "a rescan: check `ethercat slaves`, retry once the scan has finished, or `ethercat rescan`",
            position, info.vendor_id, info.product_code, info.al_state));
    }

    ScannedPdoMapping out;
    out.vendor_id = info.vendor_id;
    out.product_code = info.product_code;
    out.revision_number = info.revision_number;

    // Pass 1: query everything, entries into their final vector (nothing points
    // into it yet, so it may reallocate), SM/PDO shapes into scratch.
    struct PdoShape {
        uint16_t index;
        uint8_t entry_count;
    };
    std::vector<ec_sync_info_t> syncs_raw;
    std::vector<PdoShape> pdos_raw;
    syncs_raw.reserve(info.sync_count);
    for (uint8_t s = 0; s < info.sync_count; ++s) {
        ec_sync_info_t sync{};
        if (const int ret = ecrt_master_get_sync_manager(master, position, s, &sync); ret != 0) {
            return R::error(fmt::format("slave {}: ecrt_master_get_sync_manager(sm {}) failed ({})", position, s, ret));
        }
        if (sync.n_pdos > std::numeric_limits<uint8_t>::max()) {
            return R::error(fmt::format("slave {}: sync manager {} assigns {} PDOs — more than a layout holds ({})",
                                        position, s, sync.n_pdos, std::numeric_limits<uint8_t>::max()));
        }
        for (uint16_t p = 0; p < sync.n_pdos; ++p) {
            ec_pdo_info_t pdo{};
            if (const int ret = ecrt_master_get_pdo(master, position, s, p, &pdo); ret != 0) {
                return R::error(
                    fmt::format("slave {}: ecrt_master_get_pdo(sm {}, pdo {}) failed ({})", position, s, p, ret));
            }
            if (pdo.n_entries > std::numeric_limits<uint8_t>::max()) {
                return R::error(fmt::format("slave {}: PDO 0x{:04x} maps {} entries — more than a layout holds ({})",
                                            position, pdo.index, pdo.n_entries, std::numeric_limits<uint8_t>::max()));
            }
            for (uint16_t e = 0; e < pdo.n_entries; ++e) {
                ec_pdo_entry_info_t entry{};
                if (const int ret = ecrt_master_get_pdo_entry(master, position, s, p, e, &entry); ret != 0) {
                    return R::error(
                        fmt::format("slave {}: ecrt_master_get_pdo_entry(sm {}, pdo {}, entry {}) failed ({})",
                                    position, s, p, e, ret));
                }
                out.entries.push_back(PdoEntryMapping{entry.index, entry.subindex, entry.bit_length});
            }
            pdos_raw.push_back(PdoShape{pdo.index, static_cast<uint8_t>(pdo.n_entries)});
        }
        syncs_raw.push_back(sync);
    }

    // Pass 2: the pointer graph over storage that no longer moves.
    out.pdos.reserve(pdos_raw.size());
    out.syncs.reserve(syncs_raw.size());
    size_t entry_at = 0;
    size_t pdo_at = 0;
    for (uint8_t s = 0; s < info.sync_count; ++s) {
        const ec_sync_info_t& sync = syncs_raw[s];
        const size_t pdo_begin = out.pdos.size();
        for (unsigned int p = 0; p < sync.n_pdos; ++p) {
            const PdoShape& shape = pdos_raw[pdo_at++];
            out.pdos.push_back(
                PdoMapping{shape.index, shape.entry_count ? &out.entries[entry_at] : nullptr, shape.entry_count});
            entry_at += shape.entry_count;
        }
        out.syncs.push_back(
            SyncManagerAssignment{s, sync.dir == EC_DIR_OUTPUT, sync.n_pdos ? &out.pdos[pdo_begin] : nullptr,
                                  static_cast<uint8_t>(sync.n_pdos), sync.watchdog_mode == EC_WD_ENABLE});
    }
    out.view = SlavePdoMapping{out.syncs.data(), info.sync_count};
    return R::ok(std::move(out));
}

std::string pdo_mapping_digest(const SlavePdoMapping& layout) {
    return "sha256:" + plem::core::crypto::sha256_hex(format_pdo_mapping(layout));
}

std::string describe_scanned_mapping(const ScannedPdoMapping& layout) {
    return fmt::format("{} sync managers, {} PDOs, {} entries, revision 0x{:x}, {}", layout.view.sync_count,
                       layout.pdos.size(), layout.entries.size(), layout.revision_number,
                       pdo_mapping_digest(layout.view));
}

std::string describe_scan_rejection(uint16_t slave, const std::string& servos, const ScannedPdoMapping& layout,
                                    const std::string& validator_error) {
    return fmt::format(
        "slave {} (axes {}, vendor 0x{:x} product 0x{:x} revision 0x{:x} — as the master reports it): the PDO "
        "mapping the master scanned does not serve the axes — {}. Observed mapping:\n{}If the drive accepts PDO "
        "reconfiguration, either configure it to map the object, or declare the mapping (`pdo_mapping`, or "
        "ServoInfo::pdo_mapping from C++) so the core writes it in PREOP. A drive with a fixed mapping that does not "
        "carry this role "
        "cannot be driven in this mode.",
        slave, servos, layout.vendor_id, layout.product_code, layout.revision_number, validator_error,
        format_pdo_mapping(layout.view));
}

std::string describe_digest_mismatch(uint32_t slave, std::string_view scanned, std::string_view declared) {
    return fmt::format(
        "slave {}: the scanned PDO mapping digests to {} but the approval record declares "
        "{} — the mapping the master holds for this slave is not the one the bench observed. Causes this "
        "check cannot tell apart: the drive or its firmware was swapped; this unit has a different device "
        "profile or variant; the record's digest was mistyped; another application or tool wrote a PDO assignment "
        "to the drive; a CoE read failed during the master's scan and SII values remained; the master's "
        "scan is stale (`ethercat rescan`, then retry). Check the drive's nameplate against the record, "
        "then re-bench this drive (`plem drive run <file>` on a terminal offers the bench).",
        slave, scanned, declared);
}

std::string describe_digest_without_scan(uint32_t slave, std::string_view declared) {
    return fmt::format(
        "slave {}: the approval record declares a PDO mapping digest ({}) but this slave's "
        "mapping is DECLARED, not scanned — there is no scanned mapping to compare it with. Make the "
        "slave scanned (`pdo_mapping: scanned`) and re-bench, or declare the digest null.",
        slave, declared);
}

std::string format_pdo_mapping(const SlavePdoMapping& layout) {
    std::string text = "sync_managers:\n";
    for (uint8_t s = 0; s < layout.sync_count; ++s) {
        const SyncManagerAssignment& sm = layout.syncs[s];
        const char* direction = sm.output ? "output" : "input";
        const char* watchdog = sm.watchdog ? "true" : "false";
        if (sm.pdo_count == 0) {
            text += fmt::format("  - {{ sm: {}, direction: {}, watchdog: {}, pdos: [] }}\n", sm.sm_index, direction,
                                watchdog);
            continue;
        }
        text += fmt::format("  - sm: {}\n    direction: {}\n    watchdog: {}\n    pdos:\n", sm.sm_index, direction,
                            watchdog);
        for (uint8_t p = 0; p < sm.pdo_count; ++p) {
            const PdoMapping& pdo = sm.pdos[p];
            text += fmt::format("      - index: 0x{:04x}\n        entries:", pdo.index);
            if (pdo.entry_count == 0) {
                text += " []\n";
                continue;
            }
            text += "\n";
            for (uint8_t e = 0; e < pdo.entry_count; ++e) {
                const PdoEntryMapping& entry = pdo.entries[e];
                text += fmt::format("          - {{ index: 0x{:04x}, subindex: {}, bits: {} }}\n", entry.index,
                                    entry.subindex, entry.bit_length);
            }
        }
    }
    return text;
}

}  // namespace plem::fieldbus::ethercat
