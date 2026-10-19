// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include "plem/core/units/unit_base.hpp"

namespace plem::core::units {

struct RadianTag {};

/// An angle in radians.
class PLEM_CORE_PUBLIC Radian : public Unit<double, RadianTag> {
  public:
    using Base = Unit<double, RadianTag>;

    constexpr Radian() noexcept : Base() {}
    explicit constexpr Radian(double val) noexcept : Base(val) {}
};

}  // namespace plem::core::units
