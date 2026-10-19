// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file approval.cpp
/// @brief Judgments 2-8 of schema §6 and the bench items (approval.hpp).

#include "loader/approval.hpp"

#include <algorithm>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace plem::drive::detail {
namespace {

std::string quote(std::string_view text) {
    return "'" + std::string(text) + "'";
}

/// The same set of (slave, identity), order-free (the bench reads slaves
/// as it meets them). A record that repeats a slave already has judgment 5's
/// reason; a bench that does cannot equal a record that does not.
bool same_identities(std::vector<SlaveIdentityRecord> a, std::vector<SlaveIdentityRecord> b) {
    const auto by_slave = [](const SlaveIdentityRecord& x, const SlaveIdentityRecord& y) { return x.slave < y.slave; };
    std::sort(a.begin(), a.end(), by_slave);
    std::sort(b.begin(), b.end(), by_slave);
    return a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(), [](const SlaveIdentityRecord& x, const SlaveIdentityRecord& y) {
               return x.slave == y.slave && x.identity == y.identity;
           });
}

}  // namespace

std::vector<std::string_view> required_bench_items(const DriveModelSpecAny& spec) {
    const auto [ramped, streamed] = std::visit(
        [](const auto& s) {
            bool moving = false;
            bool torque = false;
            bool streamed = false;
            for (const auto& axis : s.axes) {
                moving |= axis.modes_of_operation != fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
                torque |= axis.modes_of_operation == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
                streamed |= axis.modes_of_operation != fieldbus::OperationMode::CYCLIC_SYNC_POSITION;
            }
            // A cst axis makes the bus's stop an immediate disable (no ramp to measure).
            return std::pair{moving && !torque, streamed};
        },
        spec);
    std::vector<std::string_view> out;
    for (const std::string_view id : kRequiredBenchItems) {
        if ((id == "stale-engage" && !streamed) || (id == "stop-decel" && !ramped)) {
            continue;
        }
        out.push_back(id);
    }
    return out;
}

core::Result<void> judge_bench_items(const BenchFile& bench, const DriveModelSpecAny& spec) {
    for (const std::string_view required : required_bench_items(spec)) {
        const auto row =
            std::find_if(bench.items.begin(), bench.items.end(), [&](const BenchItem& i) { return i.id == required; });
        if (row == bench.items.end()) {
            return core::Result<void>::error(
                "the bench has no '" + std::string(required) +
                "' item — every required v1 item must be measured and PASS (schema §6.1 lists them)");
        }
        if (row->result != "PASS") {
            return core::Result<void>::error(
                "the bench reports '" + std::string(required) + "' as " + row->result +
                " — every required v1 item must PASS before a record can be approved (evidence: " + row->evidence +
                ")");
        }
    }
    for (const BenchItem& item : bench.items) {
        if (item.result == "FAIL") {
            return core::Result<void>::error(
                "the bench reports '" + item.id +
                "' as FAIL — a measured failure is not optional; an item the drive does not offer is NA "
                "(evidence: " +
                item.evidence + ")");
        }
    }
    return core::Result<void>::ok();
}

void judge_record(const ApprovalClaims& record, const DriveModelDigests& digests, const DriveModelSpecAny& spec,
                  size_t slave_count, std::vector<std::string>& reasons) {
    constexpr std::string_view kReapprove =
        " — re-bench the file on this software (`plem drive run <file>` on a terminal offers the bench; it replaces "
        "the record)";
    if (record.config_digest != digests.config) {
        reasons.push_back("approval.record.config_digest is " + quote(record.config_digest) +
                          " but this declaration digests to " + quote(digests.config) +
                          ": the declaration changed after approval, or the record was written for another file" +
                          std::string(kReapprove));
    }
    if (record.profile_digest != digests.profile) {
        reasons.push_back("approval.record.profile_digest is " + quote(record.profile_digest) +
                          " but the resolved cia402_profile digests to " + quote(digests.profile) +
                          ": the transition table (the file's, or the core's built-in one) changed after approval" +
                          std::string(kReapprove));
    }
    std::vector<size_t> listed(slave_count, 0);
    for (size_t i = 0; i < record.slaves.size(); ++i) {
        const std::string entry = "approval.record.slaves[" + std::to_string(i) + "]";
        const SlaveIdentityRecord& r = record.slaves[i];
        if (r.slave >= slave_count) {
            reasons.push_back(entry + ".slave is " + std::to_string(r.slave) + " but the file declares slaves 0.." +
                              std::to_string(slave_count - 1) + " — one entry per declared slave, by slaves[] index");
            continue;
        }
        ++listed[r.slave];
        // Serial 0 is the SII "not recorded" convention, so it counts as empty;
        // a declared revision number is a read value, 0 included.
        const bool serial_empty = !r.identity.serial_number || *r.identity.serial_number == 0;
        if (serial_empty && !r.identity.revision_number && !r.identity.hardware_version &&
            !r.identity.software_version && !r.identity.pdo_mapping_digest) {
            reasons.push_back(entry +
                              " observes nothing (serial_number 0 or null, revision_number null, hardware_version "
                              "null, software_version null, pdo_mapping_digest null) — a record binds to at "
                              "least one value the bench read from the drive: write its SII revision number, "
                              "its 0x1009 / 0x100A strings, a non-zero serial_number, or the scanned mapping's "
                              "digest from the bus INFO line");
        }
        const bool declares_layout =
            std::visit([&](const auto& s) { return s.slaves[r.slave].pdo_mapping.sync_count != 0; }, spec);
        if (r.identity.pdo_mapping_digest && declares_layout) {
            reasons.push_back(entry + ".pdo_mapping_digest is set but slaves[" + std::to_string(r.slave) +
                              "] declares its pdo_mapping — a digest binds a `pdo_mapping: scanned` slave (the bus "
                              "compares it with the mapping it scans); a declared mapping is bound by config_digest "
                              "and has no scan to compare. Write null here, or make the slave scanned and re-bench");
        }
    }
    for (size_t k = 0; k < slave_count; ++k) {
        if (listed[k] != 1) {
            reasons.push_back("approval.record.slaves lists slave " + std::to_string(k) + " " +
                              std::to_string(listed[k]) +
                              " times — exactly one entry per declared slave (the bench "
                              "writes them from what it observed)");
        }
    }
}

void judge_bench(const RecordedApproval& record, const DriveModelSpecAny& spec, std::vector<std::string>& reasons) {
    const BenchFile& b = record.bench;
    const auto claim = [&](std::string_view what, const std::string& claimed, const std::string& recorded) {
        if (claimed != recorded) {
            reasons.push_back(
                "approval.record.bench.claims." + std::string(what) + " is " + quote(claimed) +
                " but the record says " + quote(recorded) +
                ": the bench was run on another declaration or software — a bench binds only to "
                "what it measured; re-bench the file (`plem drive run <file>` on a terminal offers the bench)");
        }
    };
    claim("config_digest", b.claims.config_digest, record.claims.config_digest);
    claim("profile_digest", b.claims.profile_digest, record.claims.profile_digest);
    claim("implementation_revision", b.claims.implementation_revision, record.claims.implementation_revision);
    if (!same_identities(b.claims.slaves, record.claims.slaves)) {
        reasons.push_back(
            "approval.record.bench.claims.slaves differ from approval.record.slaves — the bench observed "
            "other drives (or other values) than the record binds to; the bench writes both from what "
            "it observed, so edit neither by hand");
    }
    if (const core::Result<void> failed = judge_bench_items(b, spec); !failed) {
        reasons.push_back(failed.message());
    }
}

IdentityScope scope_of(const RecordedApproval& record) {
    if (record.claims.slaves.empty()) {
        return IdentityScope::NOT_APPLICABLE;  // judgment 5 refuses it; the scope says so too
    }
    for (const SlaveIdentityRecord& r : record.claims.slaves) {
        if (!r.identity.serial_number || *r.identity.serial_number == 0) {
            return IdentityScope::MODEL;
        }
    }
    return IdentityScope::UNIT;
}

}  // namespace plem::drive::detail
