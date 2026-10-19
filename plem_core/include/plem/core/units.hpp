// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file units.hpp
 * @brief Main header for the plem_core units system
 *
 * This file provides a convenient way to include all unit types.
 * The units system provides type-safe representations of physical quantities
 * with compile-time error detection for unit mismatches.
 *
 * @example
 * ```cpp
 * #include <plem/core/units.hpp>
 *
 * using namespace plem::core::units;
 *
 * auto angle = Radian(1.5708);              // Type-safe angle (rad)
 * auto velocity = RadianPerSecond(3.14);    // Type-safe angular velocity (rad/s)
 * auto torque = NewtonMeter(5.0);           // Type-safe torque (N·m)
 *
 * double raw = angle.value();               // Extract the raw value at the boundary
 *
 * // Distinct tag types cannot be assigned to each other:
 * // Radian r = velocity;                   // Error: different unit types
 * ```
 *
 * To add a unit, define a distinct tag, derive from Unit<T, Tag>, and add its
 * conversions. The Unit base operations are constexpr/inline and allocate no
 * memory.
 */

// Specific unit types
#include "plem/core/units/angle.hpp"
#include "plem/core/units/angular_velocity.hpp"
#include "plem/core/units/pulse.hpp"
#include "plem/core/units/pulse_velocity.hpp"
#include "plem/core/units/torque.hpp"
#include "plem/core/units/torque_count.hpp"
