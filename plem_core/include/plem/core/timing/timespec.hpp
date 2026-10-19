// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <stdint.h>
#include <time.h>

namespace plem::core {

inline constexpr int64_t kNsecPerSec = 1'000'000'000;

/// Clock behind every monotonic-ns helper below (and the RT periodic timers).
inline constexpr clockid_t kMonotonicClockId = CLOCK_MONOTONIC;

[[nodiscard]] constexpr int64_t timespec_to_ns(struct timespec ts) noexcept {
    return ts.tv_sec * kNsecPerSec + ts.tv_nsec;
}

/// Current CLOCK_MONOTONIC time in nanoseconds, as int64. Unsigned consumers
/// (wire / storage fields) cast at their boundary.
/// @realtime_safe Yes: one clock_gettime syscall (vDSO), no allocation.
[[nodiscard]] inline int64_t now_ns() noexcept {
    struct timespec ts {};
    clock_gettime(kMonotonicClockId, &ts);
    return timespec_to_ns(ts);
}

}  // namespace plem::core
