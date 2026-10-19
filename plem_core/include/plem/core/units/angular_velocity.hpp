// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include "plem/core/units/angle.hpp"
#include "plem/core/units/unit_base.hpp"

namespace plem::core::units {

struct RadianPerSecondTag {};

/// An angular velocity in radians per second.
class PLEM_CORE_PUBLIC RadianPerSecond : public Unit<double, RadianPerSecondTag> {
  public:
    using Base = Unit<double, RadianPerSecondTag>;

    constexpr RadianPerSecond() noexcept : Base() {}
    explicit constexpr RadianPerSecond(double val) noexcept : Base(val) {}
};

}  // namespace plem::core::units
