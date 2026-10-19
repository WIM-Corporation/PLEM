// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

namespace plem::core {

/**
 * @brief Three-state boolean: YES, NO or UNDEFINED (not yet known)
 *
 * Used to represent states that can be YES, NO, or UNDEFINED (not TRUE/FALSE:
 * glib and <rpc/types.h> define those as macros).
 *
 * @note Trivially copyable: usable inside seqlock payloads.
 * @realtime_safe No dynamic allocation
 */
enum class TriState : int8_t {
    NO = 0,         ///< Explicitly false/inactive
    YES = 1,        ///< Explicitly true/active
    UNDEFINED = -1  ///< State not yet determined or not applicable
};

}  // namespace plem::core
