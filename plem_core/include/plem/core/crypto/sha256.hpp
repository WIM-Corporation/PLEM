// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <string_view>

#include <plem/core/visibility_control.hpp>

namespace plem::core::crypto {

/**
 * @brief SHA-256 digest of the given bytes as a lowercase 64-char hex string.
 *
 * In this tree: the scanned PDO-mapping digest (plem_fieldbus pdo_scan) and the
 * drive model config digest (plem_drive loader).
 * Implementation delegates to libsodium (linked privately by plem_core).
 *
 * @thread_safety Thread-safe (no shared state).
 * @realtime_safe No: allocates the result string. Initialization paths only.
 */
PLEM_CORE_PUBLIC std::string sha256_hex(std::string_view data);

}  // namespace plem::core::crypto
