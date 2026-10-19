// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/core/timing/periodic_timer_rt.hpp"

#include <errno.h>
#include <time.h>
#include <stdexcept>

#include "plem/core/timing/timespec.hpp"

namespace plem::core {

PeriodicTimerRT::PeriodicTimerRT(unsigned period_us) : period_ns_(static_cast<int64_t>(period_us) * 1000L) {
    if (period_us == 0) {
        throw std::invalid_argument("Period must be positive");
    }
}

void PeriodicTimerRT::start() {
    if (clock_gettime(kMonotonicClockId, &next_period_) != 0) {
        throw std::runtime_error("clock_gettime failed");
    }
    initialized_ = true;
}

void PeriodicTimerRT::start_at(int64_t first_wake_ns) {
    // No CLOCK_MONOTONIC reading is negative; a negative request is taken at its
    // phase, the time in (0, period] on its period grid (so even INT64_MIN
    // seeds within one period of 0 and no subtraction below can overflow).
    if (first_wake_ns < 0) {
        first_wake_ns = first_wake_ns % period_ns_ + period_ns_;
    }
    // wait_period() pre-increments the deadline by one period before sleeping,
    // so seed it one period behind the requested first wake. A negative seed is
    // fine: wait_period() reconstructs the deadline as tv_sec*1e9 + tv_nsec and
    // its overrun rebase normalizes any past deadline forward phase-preservingly.
    const int64_t seed_ns = first_wake_ns - period_ns_;
    next_period_.tv_sec = seed_ns / kNsecPerSec;
    next_period_.tv_nsec = seed_ns % kNsecPerSec;
    initialized_ = true;
}

void PeriodicTimerRT::wait_period() {
    if (!initialized_) {
        start();
    }

    next_period_.tv_nsec += period_ns_;

    while (next_period_.tv_nsec >= kNsecPerSec) {
        next_period_.tv_sec++;
        next_period_.tv_nsec -= kNsecPerSec;
    }

    struct timespec now;
    if (clock_gettime(kMonotonicClockId, &now) != 0) {
        throw std::runtime_error("clock_gettime failed");
    }

    const int64_t now_ns = timespec_to_ns(now);
    int64_t next_ns = timespec_to_ns(next_period_);

    // If the loop overran, the next absolute deadline can already be in the
    // past. Sleeping to a past timestamp returns immediately, causing one or
    // more back-to-back catch-up cycles. For EtherCAT this can queue the next
    // cyclic datagram before the previous response has been received, which
    // IgH reports as SKIPPED/UNMATCHED. Drop missed periods and rebase to the
    // next future deadline instead.
    if (now_ns >= next_ns) {
        const int64_t missed_periods = ((now_ns - next_ns) / period_ns_) + 1;
        next_ns += missed_periods * period_ns_;
        next_period_.tv_sec = next_ns / kNsecPerSec;
        next_period_.tv_nsec = next_ns % kNsecPerSec;
    }

    int ret;
    do {
        ret = clock_nanosleep(kMonotonicClockId, TIMER_ABSTIME, &next_period_, nullptr);
    } while (ret == EINTR);
}

}  // namespace plem::core
