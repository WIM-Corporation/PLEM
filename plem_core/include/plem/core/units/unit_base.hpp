// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>
#include <compare>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace plem::core::units {

/**
 * @brief Saturating narrow of a widened integer to int32.
 *
 * The hardware sinks for pulse/velocity values are 32-bit registers
 * (EC_WRITE_S32), so a value that overflows int32 has no faithful
 * representation. Saturation is the defined boundary behavior, never a
 * two's-complement wrap, which would sign-flip a servo command (INT32_MAX+1
 * wrapping to INT32_MIN is a full-scale reversal). Callers compute in int64_t
 * so the offset/negation itself cannot overflow, then narrow here.
 */
constexpr int32_t saturate_to_int32(int64_t value) noexcept {
    if (value > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
        return std::numeric_limits<int32_t>::max();
    }
    if (value < static_cast<int64_t>(std::numeric_limits<int32_t>::min())) {
        return std::numeric_limits<int32_t>::min();
    }
    return static_cast<int32_t>(value);
}

/**
 * @brief Saturating narrow of a widened integer to int16.
 *
 * Torque counts ride a 16-bit register (EC_WRITE_S16). Same rationale as
 * saturate_to_int32; in particular the flip negation of INT16_MIN would wrap
 * back to INT16_MIN through the int16 conversion (a full-scale wrong-sign
 * torque on a flipped joint). Callers compute in int32_t, then narrow here.
 */
constexpr int16_t saturate_to_int16(int32_t value) noexcept {
    if (value > static_cast<int32_t>(std::numeric_limits<int16_t>::max())) {
        return std::numeric_limits<int16_t>::max();
    }
    if (value < static_cast<int32_t>(std::numeric_limits<int16_t>::min())) {
        return std::numeric_limits<int16_t>::min();
    }
    return static_cast<int16_t>(value);
}

/**
 * @brief Round-to-nearest then saturate a double into int32, proving the range
 *        before the cast.
 *
 * A bare static_cast<int32_t> of an out-of-range or non-finite double is
 * undefined behavior (platform-dependent: AArch64 saturates, x86 wraps to INT_MIN).
 * Non-finite input (NaN/inf) maps to 0: a garbage magnitude has no meaningful
 * integer value and 0 is the safe neutral; a finite out-of-range value saturates
 * to the nearest bound.
 */
inline int32_t saturate_round_to_int32(double value) noexcept {
    if (!std::isfinite(value)) {
        return 0;
    }
    const double rounded = std::round(value);
    if (rounded > static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return std::numeric_limits<int32_t>::max();
    }
    if (rounded < static_cast<double>(std::numeric_limits<int32_t>::min())) {
        return std::numeric_limits<int32_t>::min();
    }
    return static_cast<int32_t>(rounded);
}

/**
 * @brief Round-to-nearest then saturate a double into int16 (torque-count
 *        register width). Same policy as saturate_round_to_int32: non-finite
 *        maps to 0 (safe neutral), a finite out-of-range value saturates.
 */
inline int16_t saturate_round_to_int16(double value) noexcept {
    if (!std::isfinite(value)) {
        return 0;
    }
    const double rounded = std::round(value);
    if (rounded > static_cast<double>(std::numeric_limits<int16_t>::max())) {
        return std::numeric_limits<int16_t>::max();
    }
    if (rounded < static_cast<double>(std::numeric_limits<int16_t>::min())) {
        return std::numeric_limits<int16_t>::min();
    }
    return static_cast<int16_t>(rounded);
}

/**
 * @brief Strong typing base template for physical units
 *
 * This template provides a type-safe wrapper for numeric values with specific
 * units. It prevents accidental mixing of different unit types at compile time.
 * The type is a narrow boundary value: construction, value() extraction, and
 * comparison. It intentionally exposes no arithmetic; callers operate on the
 * raw value() at the hardware boundary.
 *
 * @tparam T The underlying numeric type (usually double)
 * @tparam Tag A unique tag type to distinguish different units
 */
template <typename T, typename Tag>
class Unit {
    static_assert(std::is_arithmetic_v<T>, "Unit type T must be arithmetic");

  protected:
    T value_;

  public:
    constexpr Unit() noexcept : value_(T{0}) {}
    explicit constexpr Unit(T val) noexcept : value_(val) {}

    constexpr T value() const noexcept {
        return value_;
    }

    constexpr auto operator<=>(const Unit&) const noexcept = default;
};

}  // namespace plem::core::units
