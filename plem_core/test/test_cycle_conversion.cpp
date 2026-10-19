// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/timing/cycle_conversion.hpp>

namespace {

using plem::core::cycles_at_least;
using plem::core::cycles_at_least_ms;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t k1kHzPeriodNs = 1'000'000;  // 1 ms
constexpr int64_t k5kHzPeriodNs = 200'000;    // 200 µs

// At a 1 kHz bus period every derived count equals the duration in ms.
TEST(CycleConversion, OneKilohertzDerivationEqualsMilliseconds) {
    EXPECT_EQ(cycles_at_least(100 * kMs, k1kHzPeriodNs), 100u);
    EXPECT_EQ(cycles_at_least(50 * kMs, k1kHzPeriodNs), 50u);
    EXPECT_EQ(cycles_at_least(5 * kMs, k1kHzPeriodNs), 5u);        // kWkcLossThresholdNs
    EXPECT_EQ(cycles_at_least(1000 * kMs, k1kHzPeriodNs), 1000u);  // a 1 s interval
    EXPECT_EQ(cycles_at_least(100 * kMs, k1kHzPeriodNs) * k1kHzPeriodNs, 100 * kMs);
}

// The point of the time source of truth: at 5 kHz the same durations derive 5x the count,
// keeping the wall-clock meaning instead of silently tightening it 5x.
TEST(CycleConversion, FiveKilohertzDerivationPreservesWallClockMeaning) {
    EXPECT_EQ(cycles_at_least(100 * kMs, k5kHzPeriodNs), 500u);
    EXPECT_EQ(cycles_at_least(50 * kMs, k5kHzPeriodNs), 250u);
    EXPECT_EQ(cycles_at_least(5 * kMs, k5kHzPeriodNs), 25u);
    EXPECT_EQ(cycles_at_least(1000 * kMs, k5kHzPeriodNs), 5000u);
}

// Ceiling contract: the derived count must cover at least the requested
// duration; a non-dividing period rounds up, never down.
TEST(CycleConversion, NonDividingPeriodRoundsUp) {
    EXPECT_EQ(cycles_at_least(1 * kMs, 300'000), 4u);  // 3.33 -> 4 (1.2 ms >= 1 ms)
    EXPECT_GE(cycles_at_least(1 * kMs, 300'000) * 300'000, 1 * kMs);
    EXPECT_EQ(cycles_at_least(999'999, k1kHzPeriodNs), 1u);
    EXPECT_EQ(cycles_at_least(1'000'001, k1kHzPeriodNs), 2u);
}

// A zero/sub-period duration still yields one cycle; a derived 0 would mean
// "fire every comparison" or "never wait", neither of which any caller wants.
TEST(CycleConversion, MinimumOneCycle) {
    EXPECT_EQ(cycles_at_least(0, k1kHzPeriodNs), 1u);
    EXPECT_EQ(cycles_at_least(1, k1kHzPeriodNs), 1u);
}

// The millisecond convenience must agree with the ns derivation; it is the
// form every millisecond-interval consumer uses.
TEST(CycleConversion, MillisecondOverloadMatchesNanosecondDerivation) {
    EXPECT_EQ(cycles_at_least_ms(1000, k1kHzPeriodNs), cycles_at_least(1000 * kMs, k1kHzPeriodNs));
    EXPECT_EQ(cycles_at_least_ms(1000, k1kHzPeriodNs), 1000u);
    EXPECT_EQ(cycles_at_least_ms(100, k1kHzPeriodNs), 100u);
    EXPECT_EQ(cycles_at_least_ms(1000, k5kHzPeriodNs), 5000u);
    EXPECT_EQ(cycles_at_least_ms(100, k5kHzPeriodNs), 500u);
    EXPECT_EQ(cycles_at_least_ms(0, k1kHzPeriodNs), 1u);  // min-1 contract inherited
}

// Init-time derivation must be usable in constant expressions.
static_assert(cycles_at_least(100 * kMs, k1kHzPeriodNs) == 100u);
static_assert(cycles_at_least(100 * kMs, k5kHzPeriodNs) == 500u);
static_assert(cycles_at_least_ms(1000, k1kHzPeriodNs) == 1000u);

}  // namespace
