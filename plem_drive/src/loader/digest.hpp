// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file digest.hpp
/// @brief The two digests a drive model file's approval record binds to
/// (not installed; schema cia402-drive-model.md §6, normalization §7) and
/// the implementation revision the running code carries. One implementation: the loader's inspection and
/// drive_model_tool both call it, so a file's digest is what the runtime recomputes.
///
/// The digest input is the validated spec value, serialized as fixed-order
/// `key=value` lines (norm/1): every field but `approval`, `expected_identity`
/// and `stored_sdo` (digest.cpp says why), sequences in file
/// order, integers in decimal, reals in their shortest round-trip form with
/// -0.0 folded to 0.0, plus one line naming where the transition table came
/// from. YAML text is never re-emitted, so comments, indentation and key order
/// are not in the digest; the names (`model.name`, `axes[].name`) are.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "plem/drive/drive_model_file.hpp"

namespace plem::drive::detail {

/// Where the axes' transition table came from. Part of config_digest's input:
/// a file that copies the built-in rows inline declares something different
/// from one that names the built-in table (a core table change invalidates
/// the latter's approval, not the former's), so they are different approvals.
enum class ProfileSource : uint8_t {
    BUILT_IN,
    INLINE,
};

/// @brief `built_in` | `inline`: the token in the digest input and in the
/// check tool's stdout contract (one spelling, both places).
[[nodiscard]] std::string_view name_of(ProfileSource source) noexcept;

struct DriveModelDigests {
    std::string config;   ///< "sha256:<64 hex>" over the spec fields above
    std::string profile;  ///< "sha256:<64 hex>" over axes[i].cia402_profile, every axis
    bool operator==(const DriveModelDigests&) const = default;
};

/// @brief The digests of a spec that passed the loader (norm/1). Pure.
[[nodiscard]] DriveModelDigests drive_model_digests(const DriveModelSpecAny& spec, ProfileSource source);

/// @brief The platform version, baked in at build time from this package's
/// package.xml <version>, which the release process keeps equal to the
/// release version (no suffix). `check` prints it; the bench copies it into
/// its claims as `implementation_revision` (schema §6), for provenance; it is
/// not checked. It names the release, not the binary: a dev tree and a release
/// package built from the same version carry the same string.
[[nodiscard]] std::string_view platform_version() noexcept;

}  // namespace plem::drive::detail
