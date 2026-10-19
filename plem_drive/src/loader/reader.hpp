// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file reader.hpp
/// @brief The drive model file's file layer (not installed): the YAML tree
/// (yaml_tree.hpp) projected to the flat spec, every key, type and file-only
/// rule checked, the approval record read but not checked (approval.hpp); and
/// the standalone bench file's reader.

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <plem/core/result.hpp>

#include "loader/approval.hpp"
#include "loader/digest.hpp"
#include "loader/yaml_tree.hpp"
#include "plem/drive/drive_model_file.hpp"

namespace plem::drive::detail {

/// The one `schema` value this loader reads (schema §10).
inline constexpr std::string_view kDriveModelSchema = "cia402_drive/v1";

/// What the file layer projects: the spec (approval UNAPPROVED; the record
/// is checked later), where the transition table came from, and the record
/// (nullopt = `approval.status: unapproved`).
struct FileProjection {
    DriveModelSpecAny spec;
    ProfileSource profile_source = ProfileSource::BUILT_IN;
    std::optional<RecordedApproval> record;
    /// bus.name: the bus's ROS name (plem drive run starts it as /<name>).
    /// Optional here (the robot tier's files carry none), required by
    /// `plem drive check`. Not in the spec, so not in config_digest:
    /// renaming a bus keeps its approval.
    std::optional<std::string> bus_name;
    /// Where a flat axis / slave index sits in the file: its path
    /// ("slaves[e].axes[a]", "slaves[e]") and, for a drive of a ring_position
    /// list, which one ("slaves[e].ring_position[c]", else empty): how
    /// inspect_drive_model names a value-gate refusal in the file's terms.
    struct Place {
        std::string path;
        std::string drive;
    };
    std::vector<Place> axis_places;
    std::vector<Place> slave_places;
};

/// @brief The file layer: every key, type and file-only rule, then the flat
/// projection (N = the file's axis count). Does not run the value gate. Error
/// text: "<origin>: <key path> <what is wrong> — <how to fix it>".
[[nodiscard]] core::Result<FileProjection> project(const YamlNode& root, std::string_view origin);

/// @brief The debug line the loader logs once the file layer passes: origin,
/// model name, where the transition profile came from, and each axis index's
/// place in the file. Pure.
[[nodiscard]] std::string describe_mapping(std::string_view origin, const DriveModelSpecAny& spec,
                                           ProfileSource source);

/// The standalone bench file `drive_model_tool approve` reads (schema §6.1,
/// `cia402_bench/v1`): the same YAML subset.
inline constexpr std::string_view kBenchSchema = "cia402_bench/v1";

/// @brief The standalone bench file's file layer (schema line, then the one
/// evidence shape: ids unique, results named).
[[nodiscard]] core::Result<BenchFile> parse_bench(std::string_view yaml, std::string_view origin);

}  // namespace plem::drive::detail
