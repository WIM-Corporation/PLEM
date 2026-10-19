// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <cmath>
#include <cstdint>

#include "plem/core/units/pulse.hpp"
#include "plem/core/units/unit_base.hpp"

namespace plem::core::units {

struct PulsePerSecondTag {};

/**
 * @brief Drive velocity PDO counts.
 *
 * Used for CSV target velocity and actual-velocity feedback. The axis's
 * velocity_factor defines their scale; counts need not equal encoder position
 * pulses per second.
 */
class PLEM_CORE_PUBLIC PulsePerSecond : public Unit<int32_t, PulsePerSecondTag> {
  public:
    constexpr PulsePerSecond() noexcept : Unit<int32_t, PulsePerSecondTag>() {}
    explicit constexpr PulsePerSecond(int32_t val) noexcept : Unit<int32_t, PulsePerSecondTag>(val) {}

    /**
     * @brief Construct from double value (rounded, range-checked)
     * @param val Velocity as double
     *
     * Rounds to nearest and saturates to the int32 range; non-finite input
     * (NaN/inf) maps to 0: same boundary policy as Pulse(double), see
     * saturate_round_to_int32.
     */
    explicit PulsePerSecond(double val) noexcept : Unit<int32_t, PulsePerSecondTag>(saturate_round_to_int32(val)) {}

    /// The wire's sign convention, unsaturated: flip negates. It is its own
    /// inverse, so it maps joint → hardware and hardware → joint alike.
    static constexpr int64_t flip_count(int64_t count, bool flip) noexcept {
        return flip ? -count : count;
    }

    // Hardware interface conversion methods

    /**
     * @brief Convert to hardware value with optional flip transformation
     * @param flip Whether to negate the velocity
     * @return Transformed value for hardware communication
     */
    constexpr int32_t to_hardware(bool flip) const noexcept {
        // In int64 the flip negation of INT32_MIN cannot overflow; saturate at the
        // register boundary, the same policy as Pulse.
        return saturate_to_int32(flip_count(value_, flip));
    }

    /**
     * @brief Create PulsePerSecond from hardware value with inverse transformation
     * @param hw_value Raw hardware value
     * @param flip Whether the hardware value was flipped
     * @return PulsePerSecond in original coordinate system
     */
    static constexpr PulsePerSecond from_hardware(int32_t hw_value, bool flip) noexcept {
        return PulsePerSecond(saturate_to_int32(flip_count(hw_value, flip)));
    }
};

}  // namespace plem::core::units
