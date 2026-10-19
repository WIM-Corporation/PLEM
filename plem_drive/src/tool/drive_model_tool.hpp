// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_model_tool.hpp
/// @brief The verbs of `drive_model_tool`, the executable behind `plem drive`
/// (not installed): `check` (check.cpp) reports a drive model file's
/// inspection on stdout (a summary for people, or with --json a
/// machine-readable JSON contract); `approve` (approve.cpp) writes the
/// approval record, the bench evidence embedded, into the file itself;
/// `probe` (probe.cpp) runs the bus's start-up check of the drives on a
/// master. `main` (main.cpp) only dispatches; gtest drives these with
/// fixtures.
///
/// Exit codes: 0 = done (check: effective == claimed), 1 = refused or a
/// claim not honored, 2 = usage, 3 = probe: only identity values differ
/// (re-bench).

#pragma once

#include <ostream>
#include <string>
#include <string_view>
#include <vector>

#include <plem/core/result.hpp>

#include "loader/approval.hpp"

namespace plem::drive::tool {

inline constexpr int kToolOk = 0;
inline constexpr int kToolRefused = 1;
inline constexpr int kToolUsage = 2;
/// probe: the drives differ from the record only in values a bench replaces.
inline constexpr int kToolRebench = 3;

/// The `format` of `plem drive check`'s JSON; a consumer refuses any other.
/// Adding a key does not bump it (consumers ignore unknown keys); removing or
/// renaming a key, or changing its type or meaning, bumps the trailing number.
inline constexpr std::string_view kCheckFormat = "plem-drive-check/1";

/// @brief `plem drive check [--json] <model.yaml>`. With --json, `out` = one
/// JSON document (format kCheckFormat): the loader's check (digests, profile source,
/// release, approval claimed/effective, identity scope, reasons, warnings,
/// approve_blocker: null or why approve would refuse), what a bench runner
/// addresses the bus with (cycle and command-stale times, fault-reset
/// policy, required bench items, axes, slaves; it never reads the YAML
/// itself) and `not_judged`, what this tool leaves to the bus at startup.
/// Without it, `out` = a summary derived from that document: the bus, its
/// axes, the approval and each reason it does not hold, the warnings and the
/// next step (no digests). The exit code is the same either way.
/// A file the loader or the bus's startup check refuses prints nothing on
/// `out` and the refusal on `err`. The loader also logs (its mapping line, at
/// debug level); the executable's main keeps that off stdout; these functions only
/// own the two streams they are given. Exit 0 when effective == claimed.
[[nodiscard]] int tool_check(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

/// @brief `drive_model_tool approve <model.yaml> <bench.yaml>`, internal: the
/// bench (plem_drive bench.py, offered by `plem drive run`) calls it with
/// its evidence in a private temporary file; it is not a user command, and
/// tool_usage leaves it out. The file may claim
/// anything (a re-bench replaces an approved block); the bench's claimed
/// digests must equal the ones this software computes for the file as it is now and
/// every required item must PASS. The bench's claims.slaves and
/// implementation_revision are copied verbatim (the revision is not checked).
/// The file's `approval:` block (its line at the start of
/// a line and the indented lines under it) is replaced by the approved block
/// (the bench evidence whole: claims, conditions, items), or the block is
/// appended when the file has none; every other byte kept, the digests
/// unchanged (approval is not in them). Written atomically (a temporary file
/// in the same directory, its permission bits, a rename; a symlink is
/// written through); refused when the result would exceed the loader's file
/// size cap. Drives with a shared brake are outside this v1 procedure:
/// approve cannot check physical brake sharing.
[[nodiscard]] int tool_approve(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

/// @brief `drive_model_tool probe <model.yaml> --master N`, internal: `plem
/// drive run` calls it before it starts an approved file. The bus's own
/// start-up check of the drives on master N (their model, presence and the
/// approval record's identities), then the master is let go: nothing is
/// configured, written or activated. Exit kToolOk = the drives match,
/// kToolRebench = only identity values differ (a bench at these drives
/// replaces the record), kToolRefused = anything else (another model, a
/// missing slave, drives in another order, a version read failing without an
/// SDO abort, a file the loader refuses, a master that does not open),
/// kToolUsage = a malformed command. The reason
/// is the bus's own wording, on `err`; `out` stays empty.
[[nodiscard]] int tool_probe(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

/// @brief The usage text a user sees (`--help`, usage errors): `check` only.
[[nodiscard]] std::string tool_usage();

/// @brief Why approve refuses this file before it looks at a bench; empty =
/// it would not. `check` prints it as approve_blocker, approve as its refusal.
[[nodiscard]] std::string approve_blocker(const std::string& text, std::string_view origin);

/// @brief Whether `args` asks for help (`--help` or `-h`).
[[nodiscard]] bool wants_help(const std::vector<std::string>& args);

/// @brief The JSON and summary word for an identity scope.
[[nodiscard]] std::string_view name_of(detail::IdentityScope scope);

}  // namespace plem::drive::tool
