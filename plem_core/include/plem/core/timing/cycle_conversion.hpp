// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

namespace plem::core {

/**
 * @brief Derive an RT-loop cycle count that covers at least @p duration_ns
 *
 * RT-loop timeouts and intervals are specified as time; the cycle count is derived
 * from the loop period at init so the wall-clock meaning survives a cycle-rate
 * change (a raw cycle-count constant silently tightens 5x at 5 kHz). The RT
 * hot loop keeps comparing plain integer counters; call this at init (non-RT)
 * only.
 *
 * Rounds up (never below the requested duration) and never returns 0: a
 * derived 0 would mean "expire immediately" for a timeout or "fire every
 * cycle" for an interval. Distinct from sizing a diagnostics aggregation
 * window, which rounds down (~duration); this bounds a wait (>= duration).
 *
 * @param duration_ns Requested duration (nanoseconds, >= 0)
 * @param period_ns RT loop period (nanoseconds, must be > 0; callers
 *                  validate at the boundary, e.g. validated_cycle_period_ns()
 *                  or a compile-time constant)
 * @return ceil(duration_ns / period_ns), minimum 1. The div+mod ceil form has
 *         no overflowing intermediate; in-domain inputs (<= 65 s durations,
 *         >= 1 µs periods) stay far below the uint32 range.
 */
constexpr uint32_t cycles_at_least(int64_t duration_ns, int64_t period_ns) noexcept {
    const int64_t cycles = duration_ns / period_ns + (duration_ns % period_ns != 0 ? 1 : 0);
    return static_cast<uint32_t>(cycles < 1 ? 1 : cycles);
}

/// @brief Millisecond convenience over cycles_at_least(), for durations
/// declared in milliseconds (the fault-reset pulse width and pacing interval).
constexpr uint32_t cycles_at_least_ms(int64_t duration_ms, int64_t period_ns) noexcept {
    return cycles_at_least(duration_ms * 1'000'000, period_ns);
}

}  // namespace plem::core
