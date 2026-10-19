// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <plem/core/units.hpp>

#include "plem/fieldbus/ethercat/config/configuration.hpp"

namespace plem::fieldbus::ethercat {

using plem::core::units::NewtonMeter;
using plem::core::units::Pulse;
using plem::core::units::PulsePerSecond;
using plem::core::units::Radian;
using plem::core::units::RadianPerSecond;
using plem::core::units::TorqueCount;

// Unit conversions between PLEM's SI joint quantities and a drive's PDO counts,
// per axis channel. Each channel has one formula (torque: SCALED_CURRENT, which
// RATED_PER_MILLE reuses with scale 1000 and the rated torque as Kt;
// position/velocity: ROTARY_MOTOR_ENCODER), so nothing here dispatches on the
// unit. validate_axis_factors (at the spec and bus boundaries) admits only
// those units with finite, positive parameters, and a debug assert on the
// channel's is_valid() refuses a direct caller with a zero parameter in debug
// builds. Adding a unit means adding its conversion and its validation
// together.
//
// Every expression keeps one fixed evaluation order (operand order,
// parentheses, local factor variables, integer boundaries): reordering
// `/Kt/gear` into `/(Kt*gear)` was measured to change integer outputs at
// half-count boundaries, so a reordering is a behavior change, not a refactor.
// The two encode factors each live in one place (detail::rad_to_count,
// detail::rad_s_to_count): the saturating encode and its checked twin read
// the same factor, and a synthetic half-count input in test_servo_conversions
// pins the order.
//
// These functions are host-side arithmetic. Whether a drive's counts mean what
// the declaration says is shown by the model's evidence and hardware
// validation. Offsets and sign are applied at the PDO boundary
// (ServoInfo::joint_offset, Pulse::to_hardware), never here.

namespace detail {
/// Counts per output-shaft radian (ROTARY_MOTOR_ENCODER). Order is contract.
[[nodiscard]] inline double rad_to_count(const PositionChannel& ch) noexcept {
    return ch.encoder_resolution * ch.gear_ratio / (2.0 * M_PI);
}

/// Velocity counts per output-shaft rad/s (ROTARY_MOTOR_ENCODER), the exact
/// inverse of to_angular_velocity's factor. Order is contract.
[[nodiscard]] inline double rad_s_to_count(const VelocityChannel& ch) noexcept {
    return ch.encoder_resolution * ch.gear_ratio / (2.0 * M_PI * ch.velocity_factor);
}
}  // namespace detail

/**
 * @brief Output-shaft angle → position count (ROTARY_MOTOR_ENCODER).
 * @param angle Angle in radians (output shaft)
 * @param ch    The axis's position channel
 */
inline Pulse angle_to_pulse(const Radian& angle, const PositionChannel& ch) {
    // Pre-validated invariant: the channel was accepted by validate_axis_factors()
    // at the config boundary (EtherCATBus::initialize()), so encoder_resolution/
    // gear_ratio are non-zero and finite here. Debug-assert the invariant instead
    // of silently fabricating a 0 command; a no-op in release (NDEBUG): the
    // assert adds no branch of its own there (the saturating boundary
    // constructors keep theirs), and nothing here throws.
    assert(ch.is_valid());
    // A non-finite command is an upstream contract violation, not a valid 0;
    // debug-assert it (the RT command path uses encode_position_checked
    // instead).
    assert(std::isfinite(angle.value()));
    // Route the int32 narrowing through the saturating Pulse(double) boundary
    // constructor: an out-of-range product is a UB cast if done bare; a
    // non-finite product (finite input whose factor overflows) becomes 0.
    return Pulse(angle.value() * detail::rad_to_count(ch));
}

/**
 * @brief Position count → output-shaft angle (ROTARY_MOTOR_ENCODER).
 */
inline Radian to_angle(const Pulse& pulse, const PositionChannel& ch) {
    assert(ch.is_valid());
    double pulse_to_rad = (2.0 * M_PI) / (ch.encoder_resolution * ch.gear_ratio);
    return Radian(pulse.value() * pulse_to_rad);
}

/**
 * @brief Output-shaft angular velocity → velocity count (ROTARY_MOTOR_ENCODER).
 */
inline PulsePerSecond to_pulse_velocity(const RadianPerSecond& velocity, const VelocityChannel& ch) {
    assert(ch.is_valid());
    // Non-finite command guard; see angle_to_pulse.
    assert(std::isfinite(velocity.value()));
    // Exact inverse of to_angular_velocity; the two must move together or a
    // commanded velocity lands off by the same factor in the other direction.
    // Saturating boundary constructor; see angle_to_pulse.
    return PulsePerSecond(velocity.value() * detail::rad_s_to_count(ch));
}

/**
 * @brief Velocity count → output-shaft angular velocity (ROTARY_MOTOR_ENCODER).
 */
inline RadianPerSecond to_angular_velocity(const PulsePerSecond& pulse_velocity, const VelocityChannel& ch) {
    assert(ch.is_valid());
    // velocity_factor is what makes this different from the position
    // conversion: drives are free to report velocity in their own unit.
    // 1.0 keeps velocity counts equal to position pulses.
    double pulse_to_rad_per_sec = (2.0 * M_PI) * ch.velocity_factor / (ch.encoder_resolution * ch.gear_ratio);
    return RadianPerSecond(pulse_velocity.value() * pulse_to_rad_per_sec);
}

/**
 * @brief Output-shaft torque → torque count (SCALED_CURRENT, RATED_PER_MILLE).
 *
 *   count = τ · counts_per_unit / torque_constant / gear_ratio
 *
 * τ in Nm at the output shaft; counts_per_unit = drive counts per amp;
 * torque_constant = motor Nm/A; gear_ratio = motor turns per output turn.
 * RATED_PER_MILLE: counts_per_unit = 1000, torque_constant = motor rated torque.
 * Runs in the RT write path (DeviceServo::write).
 */
inline TorqueCount to_torque_count(const NewtonMeter& torque, const TorqueChannel& ch) {
    // Pre-validated invariant (see angle_to_pulse): channel accepted at the
    // boundary. NDEBUG-compiled-out, never a per-cycle branch or throw.
    assert(ch.is_valid());
    double torque_count = torque.value() * ch.counts_per_unit / ch.torque_constant / ch.gear_ratio;
    // This is an installed public header, so the double→int16 conversion must
    // be total on its own: non-finite → 0 (safe neutral; a finite input whose
    // intermediate overflows lands here too, as 0, not as saturation),
    // out-of-range saturates, the same deterministic policy as the
    // Pulse/PPS double boundaries. The production ingress
    // (DeviceServo::set_command) already rejects non-finite torque fail-fast;
    // this is the release-mode backstop for direct callers of the header (a
    // bare double→int16 cast is UB).
    return TorqueCount(plem::core::units::saturate_round_to_int16(torque_count));
}

/**
 * @brief Torque count → output-shaft torque (SCALED_CURRENT, RATED_PER_MILLE).
 *
 *   τ = count · torque_constant · gear_ratio / counts_per_unit
 *
 * The count is the drive's torque feedback (scaled current, or per mille of
 * rated torque); the result is Nm at the output shaft after the reducer: the
 * drive's estimate, not a torque sensor reading.
 */
inline NewtonMeter to_torque(const TorqueCount& torque_count, const TorqueChannel& ch) {
    assert(ch.is_valid());
    double torque_nm =
        static_cast<double>(torque_count.value()) * ch.torque_constant * ch.gear_ratio / ch.counts_per_unit;
    return NewtonMeter(torque_nm);
}

// Checked conversions (the CSP/CSV command path)
// The conversions above saturate. The decodes and the torque encode run on the
// RT path; no RT command path calls angle_to_pulse or to_pulse_velocity. A
// position or velocity target must not saturate: a target the wire cannot
// carry is a violation the caller acts on (protective stop), never a nearby
// word the drive would follow. These apply offset and flip exactly once
// (Pulse::hardware_count / PulsePerSecond::flip_count, the convention the
// saturating to_hardware uses too) and report the value the chosen count
// decodes back to, which is what limit and step checks must check (the count
// is what the drive receives, not the double).

/// A target the drive receives exactly: the 0x607A/0x60FF word, and the
/// joint-frame SI value (rad or rad/s) that word decodes back to.
struct CheckedTarget {
    int32_t word;
    double si;
};

namespace detail {
/// round-half-away-from-zero like saturate_round_to_int32, but refusing
/// non-finite and out-of-range values instead of saturating them.
inline std::optional<int64_t> round_to_int32_checked(double value) noexcept {
    if (!std::isfinite(value)) {
        return std::nullopt;
    }
    const double rounded = std::round(value);
    if (rounded < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
        rounded > static_cast<double>(std::numeric_limits<int32_t>::max())) {
        return std::nullopt;
    }
    return static_cast<int64_t>(rounded);
}

inline bool fits_int32(int64_t value) noexcept {
    return value >= std::numeric_limits<int32_t>::min() && value <= std::numeric_limits<int32_t>::max();
}
}  // namespace detail

/**
 * @brief Output-shaft angle → 0x607A word, or nothing when the wire cannot
 *        carry it (non-finite, joint count or offset-applied word outside int32).
 */
inline std::optional<CheckedTarget> encode_position_checked(double rad, const PositionChannel& ch, int32_t offset,
                                                            bool flip) noexcept {
    assert(ch.is_valid());
    const auto count = detail::round_to_int32_checked(rad * detail::rad_to_count(ch));
    if (!count) {
        return std::nullopt;
    }
    const int64_t word = Pulse::hardware_count(*count, offset, flip);
    if (!detail::fits_int32(word)) {
        return std::nullopt;
    }
    return CheckedTarget{static_cast<int32_t>(word), to_angle(Pulse(static_cast<int32_t>(*count)), ch).value()};
}

/**
 * @brief 0x6064/0x607A word → output-shaft angle, or nothing when the word does
 *        not decode into int32 joint counts (Pulse::from_hardware would saturate).
 */
inline std::optional<double> decode_position_word(int32_t word, const PositionChannel& ch, int32_t offset,
                                                  bool flip) noexcept {
    assert(ch.is_valid());
    const int64_t count = Pulse::joint_count(word, offset, flip);
    if (!detail::fits_int32(count)) {
        return std::nullopt;
    }
    return to_angle(Pulse(static_cast<int32_t>(count)), ch).value();
}

// Endless position axis (no position limits)
// The host keeps an int64 count that follows the drive's int32 word across its
// 2^32 wrap. Supported |count| ≤ kEndlessCountDomain: inside it a double rad
// still resolves a fraction of a count, and negation cannot overflow.

inline constexpr int64_t kEndlessCountDomain = int64_t{1} << 48;

/// Short-way difference a − b of two int32 words, modulo 2^32. Exactly half a
/// period (2^31) is ambiguous and reads as −2^31.
[[nodiscard]] constexpr int32_t wrap_diff(int32_t a, int32_t b) noexcept {
    return static_cast<int32_t>(static_cast<uint32_t>(a) - static_cast<uint32_t>(b));
}

/// Joint count (int64) → output-shaft angle; the factor order of to_angle.
[[nodiscard]] inline double endless_count_to_angle(int64_t count, const PositionChannel& ch) noexcept {
    assert(ch.is_valid());
    const double pulse_to_rad = (2.0 * M_PI) / (ch.encoder_resolution * ch.gear_ratio);
    return static_cast<double>(count) * pulse_to_rad;
}

/**
 * @brief Output-shaft angle → wrapped 0x607A word for an endless axis, or
 *        nothing when the count is non-finite or outside ±kEndlessCountDomain
 *        (checked before rounding). The word is the hardware count modulo 2^32;
 *        si is the joint count's angle.
 */
[[nodiscard]] inline std::optional<CheckedTarget> encode_endless_position_checked(double rad, const PositionChannel& ch,
                                                                                  bool flip) noexcept {
    assert(ch.is_valid());
    const double x = rad * detail::rad_to_count(ch);
    if (!std::isfinite(x) || std::fabs(x) > static_cast<double>(kEndlessCountDomain)) {
        return std::nullopt;
    }
    const int64_t count = std::llround(x);
    const int64_t hw = Pulse::hardware_count(count, 0, flip);
    return CheckedTarget{static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint64_t>(hw))),
                         endless_count_to_angle(count, ch)};
}

/**
 * @brief Output-shaft angular velocity → 0x60FF word, or nothing when the wire
 *        cannot carry it (non-finite, or outside int32 before or after the flip).
 */
inline std::optional<CheckedTarget> encode_velocity_checked(double rad_s, const VelocityChannel& ch,
                                                            bool flip) noexcept {
    assert(ch.is_valid());
    const auto count = detail::round_to_int32_checked(rad_s * detail::rad_s_to_count(ch));
    if (!count) {
        return std::nullopt;
    }
    const int64_t word = PulsePerSecond::flip_count(*count, flip);
    if (!detail::fits_int32(word)) {
        return std::nullopt;
    }
    return CheckedTarget{static_cast<int32_t>(word),
                         to_angular_velocity(PulsePerSecond(static_cast<int32_t>(*count)), ch).value()};
}

}  // namespace plem::fieldbus::ethercat
