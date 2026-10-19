// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_model_file.hpp
/// @brief The drive model file: one CiA 402 drive bus
/// declared in YAML (schema cia402-drive-model.md, first line
/// `schema: cia402_drive/v1`), loaded into the DriveModelSpec<N> DriveHost
/// runs. N (1..kMaxAxes) is the file's axis count, so the result is a
/// variant over every N; std::visit hands it to a builder templated on N.
///
/// What a successful load means: the file passed the file layer (its YAML
/// subset, every key and type, the rules only a file can break) and the drive
/// value gate validate_drive_model_spec. Everything else is checked where it is
/// checked for a hand-assembled spec, and not earlier: the bus's Step 0
/// (per-slave PDO roles, (ring_position, channel) pairs, object-dictionary
/// overflow) inside DriveHost::create before any ecrt call, the fault-reset
/// pacing against the period after the bus initializes, and the DC sync0 rules
/// once the master is acquired.
///
/// The file's `approval` block (schema §6) decides motion: `status:
/// unapproved` loads with UNAPPROVED (assembles and starts, enable() refuses
/// motion); `status: approved` carries the record the bench's approval step wrote,
/// the bench evidence included, and loads with APPROVED only when every
/// check holds: the digests of this declaration, one observed identity per
/// slave, the embedded evidence claiming the same, and every required bench
/// item PASS. A claim this software cannot honor is refused (fail-closed),
/// never demoted; `plem drive check` shows the reasons.
///
/// The value gate, the bus and the runtime speak in flat axis indices; the
/// loader names a value-gate refusal by the file's path instead
/// (slaves[s].axes[k]…), and logs the mapping axes[i] = slaves[s].axes[k] at
/// debug level.

#pragma once

#include <cstdint>
#include <string>
#include <variant>

#include <plem/core/result.hpp>

#include "plem/drive/models/drive_model_spec.hpp"

namespace plem::drive {

/// One drive model of any supported axis count.
using DriveModelSpecAny = std::variant<models::DriveModelSpec<1>, models::DriveModelSpec<2>, models::DriveModelSpec<3>,
                                       models::DriveModelSpec<4>, models::DriveModelSpec<5>, models::DriveModelSpec<6>,
                                       models::DriveModelSpec<7>, models::DriveModelSpec<8>>;
static_assert(std::variant_size_v<DriveModelSpecAny> == kMaxAxes, "one alternative per supported axis count");

/// What the loader does with the file's approval record.
enum class ApprovalRecord : std::uint8_t {
    USE,     ///< The record decides APPROVED or UNAPPROVED; a claim it cannot honor is refused.
    IGNORE,  ///< A bench run (an operator at the drives): the file loads as unapproved with no
              ///< identity to compare, so it can be re-benched on drives other than its record's.
};

/// @brief Load the drive model file at `path`. A relative path resolves
/// against the process's working directory and `~` is not expanded. Refuses a
/// path that is not a readable regular file and a file over 1 MiB. Error text
/// names the file, then where: a YAML syntax refusal its line (an encoding
/// refusal its byte), a schema refusal its key path, a value-gate refusal the
/// same file path (slaves[s].axes[k].limits…; for a drive of a ring_position
/// list, which one leads the message; schema §3.1).
/// An approval claim this software cannot honor is refused. With
/// ApprovalRecord::IGNORE the record is not used at all.
[[nodiscard]] core::Result<DriveModelSpecAny> load_drive_model_file(const std::string& path,
                                                                    ApprovalRecord record = ApprovalRecord::USE);

}  // namespace plem::drive
