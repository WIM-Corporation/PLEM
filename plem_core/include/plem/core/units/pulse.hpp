// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <cmath>
#include <cstdint>

#include "plem/core/units/unit_base.hpp"

namespace plem::core::units {

struct PulseTag {};

/**
 * @brief A position in drive counts, as carried in the CiA 402 position words.
 */
class PLEM_CORE_PUBLIC Pulse : public Unit<int32_t, PulseTag> {
  public:
    constexpr Pulse() noexcept : Unit<int32_t, PulseTag>() {}
    explicit constexpr Pulse(int32_t val) noexcept : Unit<int32_t, PulseTag>(val) {}

    /**
     * @brief Construct from double value (rounded, range-checked)
     * @param val Pulse count value as double
     *
     * Rounds to nearest and saturates to the int32 range; non-finite input
     * (NaN/inf) maps to 0. A bare static_cast of an out-of-range/non-finite
     * double would be undefined behavior; see saturate_round_to_int32.
     */
    explicit Pulse(double val) noexcept : Unit<int32_t, PulseTag>(saturate_round_to_int32(val)) {}

    /// The wire's sign convention, unsaturated: joint count → hardware count.
    /// With flip the hardware count is offset − joint, otherwise joint + offset;
    /// joint_count() is its exact inverse. Both are exact in int64 for int32
    /// inputs and for int64 counts within ±2^62 (the endless axis keeps its
    /// counts within ±2^48).
    static constexpr int64_t hardware_count(int64_t joint, int32_t offset, bool flip) noexcept {
        return flip ? (offset - joint) : (joint + offset);
    }

    /// Hardware count → joint count, the inverse of hardware_count().
    static constexpr int64_t joint_count(int64_t hardware, int32_t offset, bool flip) noexcept {
        const int64_t adjusted = hardware - offset;
        return flip ? -adjusted : adjusted;
    }

    // Hardware interface conversion methods

    /**
     * @brief Convert to hardware value with offset and flip transformations
     * @param offset Calibration offset in hardware counts
     * @param flip Whether the joint direction is inverted on the wire
     * @return Transformed value for hardware communication
     */
    constexpr int32_t to_hardware(int32_t offset, bool flip) const noexcept {
        // Must be the exact inverse of from_hardware(): its flip decode
        // -(hw - offset) is the hardware-validated read convention, so the
        // flip encode is offset - value (any other encode displaces every
        // commanded position by a constant on the wire). Saturate at the
        // register boundary.
        return saturate_to_int32(hardware_count(value_, offset, flip));
    }

    /**
     * @brief Create Pulse from hardware value with inverse transformations
     * @param hw_value Raw hardware value
     * @param offset Offset to remove from the hardware value
     * @param flip Whether the hardware value was flipped
     * @return Pulse in original coordinate system
     */
    static constexpr Pulse from_hardware(int32_t hw_value, int32_t offset, bool flip) noexcept {
        // Saturate at the register boundary (mirror of to_hardware).
        return Pulse(saturate_to_int32(joint_count(hw_value, offset, flip)));
    }
};

}  // namespace plem::core::units
