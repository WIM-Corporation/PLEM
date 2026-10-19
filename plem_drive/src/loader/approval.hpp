// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file approval.hpp
/// @brief The approval record and its check (not installed): what an
/// approval binds to and the bench evidence it carries (schema §6, §6.1),
/// the items a bench must PASS, and judgments 2-8, which the loader runs on
/// a file claiming approved and `drive_model_tool approve` runs on a bench.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <plem/core/result.hpp>

#include "loader/digest.hpp"
#include "plem/drive/drive_model_file.hpp"

namespace plem::drive::detail {

/// One entry of approval.record.slaves (schema §6): the file's slaves[] index
/// and what the bench observed on it.
struct SlaveIdentityRecord {
    size_t slave = 0;
    fieldbus::ethercat::SlaveIdentity identity{};
};

/// What an approval binds to: the record's head and the bench's `claims`
/// block are this one shape (schema §6/§6.1): a bench run on one declaration,
/// software and set of drives cannot be attached to another (judgment 7).
struct ApprovalClaims {
    std::string config_digest;
    std::string profile_digest;
    std::string implementation_revision;
    std::vector<SlaveIdentityRecord> slaves;
};

struct BenchItem {
    std::string id;
    std::string result;  ///< PASS | FAIL | NA
    std::string evidence;
};

/// Where and by whom a bench ran, carried whole into the approval (free
/// text, never checked).
struct BenchConditions {
    std::string host;
    std::string master;
    std::string date;
    std::string measured_by;
    std::string notes;
};

/// A bench's evidence (schema §6.1): what it claims it measured, where, and
/// the items. The standalone `cia402_bench/v1` file (approve's input) and the
/// block embedded in approval.record.bench are this one shape; the file
/// adds only its `schema` line.
struct BenchFile {
    ApprovalClaims claims;
    BenchConditions conditions;
    std::vector<BenchItem> items;
};

/// approval.record as written: the file layer's projection of a file that
/// claims approved: what the approval binds to, and the bench evidence it was
/// made from. Checked (not trusted) by inspect_drive_model.
struct RecordedApproval {
    ApprovalClaims claims;
    BenchFile bench;
};

/// The v1 items a bench must report PASS on, where they apply (required_bench_items).
/// Any other id is optional: NA with its evidence, or absent, never FAIL.
/// An approval outlives releases; this list is what retires it (schema §6.1):
/// a change that makes an item's pass contract stricter (runner threshold,
/// C++ budget or spec, wherever it lives) renames that id here (e.g.
/// `stale-engage` → `stale-engage-2`) and says "drive re-bench required" in
/// the release notes. Old benches lack the new id, so judgment 8 refuses them.
inline constexpr std::string_view kRequiredBenchItems[] = {
    "assembly-match", "idle-non-oe", "start-rtso",     "release-oe",
    "stale-engage",   "stop-decel",  "disable-engage", "stop-quiesce",
};

/// @brief The items of kRequiredBenchItems a bench of `spec` must report PASS
/// on: all but `stale-engage` on a bus with no stream axis (csv or cst; the
/// csp lane's trajectory controller writes every cycle; a stalled controller
/// manager's stale stop is not measured by the bench) and `stop-decel` on a
/// bus with no moving axis (csp or csv) or with a cst axis (that bus's stop is
/// an immediate disable, not the ramp the item measures). An item left out is
/// optional.
[[nodiscard]] std::vector<std::string_view> required_bench_items(const DriveModelSpecAny& spec);

/// @brief Judgment 8, made by the loader's inspection (which `drive_model_tool
/// approve` runs too): every item required for `spec` (required_bench_items) present with
/// PASS, and no item (required or not) reporting FAIL; the opt-out
/// vocabulary is NA. ok = holds; else an error with the reason.
[[nodiscard]] core::Result<void> judge_bench_items(const BenchFile& bench, const DriveModelSpecAny& spec);

/// What an approved record's slave identities bind the approval to (schema
/// §6). MODEL: any declared serial_number is 0 or null; the approval holds for
/// every unit of the same VID/PID, hardware and software version (some drives
/// report serial 0). UNIT: every slave declares a non-zero
/// serial_number. NOT_APPLICABLE: no record, or a record observing no slave
/// (judgment 5 refuses that one).
enum class IdentityScope : uint8_t {
    NOT_APPLICABLE,
    MODEL,
    UNIT,
};

/// Judgments 2-5 of schema §6: the record's values against this build and
/// this declaration (4: no release comparison; implementation_revision is
/// where the bench ran, provenance only: an approval outlives releases). Every
/// failure is a reason; none is an error.
void judge_record(const ApprovalClaims& record, const DriveModelDigests& digests, const DriveModelSpecAny& spec,
                  size_t slave_count, std::vector<std::string>& reasons);

/// Judgments 7-8 of schema §6 (6: no separate bench-file hash): the embedded
/// evidence claims what the record says (7) and reports every required item
/// PASS (8, the same function `drive_model_tool approve` runs). What 7 catches is a one-sided edit: an operator who
/// edits a limit, sees the digest refusal and pastes the new digest into the record passes 2 and 3; the evidence's own
/// claim still names the old digest. It does not prove a bench ran, and an edit that changes both copies (or the
/// evidence's conditions and items) passes it; the record is change
/// detection against mistakes, not authentication.
void judge_bench(const RecordedApproval& record, const DriveModelSpecAny& spec, std::vector<std::string>& reasons);

/// @brief The identity scope of an approved record (IdentityScope).
[[nodiscard]] IdentityScope scope_of(const RecordedApproval& record);

}  // namespace plem::drive::detail
