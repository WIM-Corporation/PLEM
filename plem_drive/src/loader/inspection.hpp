// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file inspection.hpp
/// @brief The loader's last stage (not installed): inspect_drive_model runs
/// text → tree (yaml_tree.hpp) → project (reader.hpp: the file layer, record
/// included) → the axis-mapping line → the value gate → the approval check
/// (approval.hpp). load = inspect + fail-closed. Tests drive `project` with edited
/// trees (no YAML re-emitter) and `parse_drive_model` / `inspect_drive_model`
/// with inline YAML; drive_model_tool reads the inspection.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <plem/core/result.hpp>

#include "loader/approval.hpp"
#include "loader/digest.hpp"
#include "plem/drive/drive_model_file.hpp"

namespace plem::drive::detail {

/// The largest file load_drive_model_file reads: several times a file at
/// full schema capacity (eight slaves with every PDO and startup SDO list full
/// come to about 140 KiB in the test example's layout).
inline constexpr size_t kMaxDriveModelFileBytes = size_t{1} << 20;

/// The vendor_id of the synthetic example (examples/example_drive.yaml), whose
/// values fit no real drive: check warns of it and sends to replace them.
inline constexpr uint32_t kExampleVendorId = 0x00c0ffee;

/// @brief The file wrapper shared by the model and bench files (path,
/// readability, size cap), text out. `what` names the file kind in errors.
[[nodiscard]] core::Result<std::string> read_text_file(const std::string& path, std::string_view what);

/// @brief Everything inspect_drive_model establishes about a file. `spec`
/// carries the effective approval; expected_identity is filled only when the
/// record was honored. `reasons` lists every way the claim failed (empty
/// when effective == claimed), each "<what> — <how>".
struct DriveModelInspection {
    DriveModelSpecAny spec;
    DriveModelDigests digests;
    ProfileSource profile_source = ProfileSource::BUILT_IN;
    std::optional<std::string> bus_name;  ///< FileProjection::bus_name
    models::Approval claimed = models::Approval::UNAPPROVED;
    models::Approval effective = models::Approval::UNAPPROVED;
    IdentityScope identity_scope = IdentityScope::NOT_APPLICABLE;
    std::vector<std::string> reasons;
    /// What the file leaves to the drive's stored values or sizes as 0: a
    /// csp/csv axis's stop objects (0x605A/0x605C/0x6084/0x6085), a DC slave's
    /// SYNC0 shift of 0, position limits past what the int32 0x6064 decodes to,
    /// the synthetic example's vendor_id. `plem drive check` shows them; they
    /// are never a refusal. Each "<where>: <what> — <how>".
    std::vector<std::string> warnings;
};

/// @brief parse_tree → project → describe_mapping (logged) → the value gate
/// → the approval check. An error is the file layer's or the value
/// gate's; a claim the build cannot honor is not an error here; it is
/// `effective != claimed` with its reasons (the read-only check tool shows
/// them; the loader refuses).
[[nodiscard]] core::Result<DriveModelInspection> inspect_drive_model(std::string_view yaml, std::string_view origin);

/// @brief inspect_drive_model, fail-closed: `effective != claimed` is an error
/// naming the first reason, unless `bench_unapproved`, which loads any file
/// as unapproved with no expected identity (load_drive_model_file).
[[nodiscard]] core::Result<DriveModelSpecAny> parse_drive_model(std::string_view yaml, std::string_view origin,
                                                                bool bench_unapproved = false);

}  // namespace plem::drive::detail
