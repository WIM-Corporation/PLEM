// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <stdint.h>
#include <time.h>

namespace plem::core {

/**
 * @brief Real-time periodic timer using clock_nanosleep with absolute time
 *
 * Sleeps with clock_nanosleep(TIMER_ABSTIME) to absolute deadlines, so the
 * period does not drift.
 *
 * Thread Safety: Each thread should use its own instance
 * Ownership: non-copyable and non-movable (owns its clock state)
 *
 * Example usage:
 * @code
 *   PeriodicTimerRT timer(1000);  // 1ms period
 *   while (running) {
 *       timer.wait_period();
 *       // Perform periodic work
 *   }
 * @endcode
 */
class PLEM_CORE_PUBLIC PeriodicTimerRT {
  public:
    /**
     * @param period_us Period in microseconds (must be positive)
     * @throws std::invalid_argument if period is zero
     */
    explicit PeriodicTimerRT(unsigned period_us);

    PeriodicTimerRT(const PeriodicTimerRT&) = delete;
    PeriodicTimerRT& operator=(const PeriodicTimerRT&) = delete;
    PeriodicTimerRT(PeriodicTimerRT&&) = delete;
    PeriodicTimerRT& operator=(PeriodicTimerRT&&) = delete;

    /**
     * @brief Start the timer from current time
     */
    void start();

    /**
     * @brief Start the timer so the next wait_period() wakes at an absolute time
     * @param first_wake_ns Requested first wake (CLOCK_MONOTONIC, nanoseconds)
     *
     * One-shot phase-alignment seam: a caller that knows another loop's wake
     * epoch can align this timer to epoch + offset. A past first_wake_ns is
     * normalized forward by wait_period()'s overrun rebase (whole periods, phase
     * preserving), so callers need no min-future guard. Any int64 value is
     * accepted: a negative one (no CLOCK_MONOTONIC reading is negative) is
     * taken at its phase, as the time in (0, period] on its period grid.
     * RT-safe: no syscalls.
     */
    void start_at(int64_t first_wake_ns);

    /**
     * @brief Wait until next period
     *
     * If the timer is uninitialized (neither start() nor start_at() called
     * yet), initializes via start() first; after start_at() the seeded
     * deadline is honored, not reset.
     * Uses TIMER_ABSTIME to prevent drift accumulation.
     */
    void wait_period();

  private:
    timespec next_period_{};
    int64_t period_ns_;
    bool initialized_{false};
};

}  // namespace plem::core
