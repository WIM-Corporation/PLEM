// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file yaml_tree.hpp
/// @brief The drive model file's strict-YAML reader (not installed).
/// libyaml's event stream becomes a small value tree:
/// mapping / sequence / scalar, nothing else. Everything the file format
/// refuses at the syntax level (schema cia402-drive-model.md §2) is
/// refused here, before any value is interpreted: directives, a second
/// document, anchors and aliases, every tag, block scalars, a non-scalar or
/// repeated key, nesting past kMaxYamlDepth and input that is not UTF-8. What
/// a scalar means (integer, number, name) is the schema layer's job.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <plem/core/result.hpp>

namespace plem::drive::detail {

/// Deepest nesting the reader accepts (the drive model schema nests ten
/// levels: slaves → slave → pdo_mapping → sync_managers → SM → pdos → PDO →
/// entries → entry). libyaml has no depth limit of its own.
inline constexpr size_t kMaxYamlDepth = 16;

struct YamlNode {
    enum class Kind : uint8_t {
        SCALAR,
        SEQUENCE,
        MAPPING
    };

    Kind kind = Kind::SCALAR;
    std::string scalar;           ///< SCALAR: the decoded text (may hold a NUL; the schema layer refuses it)
    bool plain = false;           ///< SCALAR: unquoted; only a plain scalar can be a number, a bool or null
    std::vector<YamlNode> items;  ///< SEQUENCE, in file order
    std::vector<std::pair<std::string, YamlNode>> members;  ///< MAPPING, in file order, keys unique

    /// The member named `key`, or nullptr (MAPPING only).
    [[nodiscard]] const YamlNode* find(std::string_view key) const;
};

/// @brief Parse `text` (one YAML document) into a tree. A refusal leads with
/// where libyaml marks it: "line N, column M: " (both 1-based), or "byte N: "
/// (a 0-based offset) for an encoding error; a file that holds no document
/// has no place.
[[nodiscard]] core::Result<YamlNode> parse_tree(std::string_view text);

}  // namespace plem::drive::detail
