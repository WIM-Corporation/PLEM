// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include "plem/core/units/unit_base.hpp"

namespace plem::core::units {

struct NewtonMeterTag {};

/// A torque in newton-metres (N⋅m).
class PLEM_CORE_PUBLIC NewtonMeter : public Unit<double, NewtonMeterTag> {
  public:
    using Base = Unit<double, NewtonMeterTag>;

    constexpr NewtonMeter() noexcept : Base() {}
    explicit constexpr NewtonMeter(double val) noexcept : Base(val) {}
};

}  // namespace plem::core::units
