// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>

namespace plem::core {

/// Most axes one fieldbus carries: 8, the width of the uint8_t joint masks.
inline constexpr std::size_t kMaxBusAxes = 8;

}  // namespace plem::core
