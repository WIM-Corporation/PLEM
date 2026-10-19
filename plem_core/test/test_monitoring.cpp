// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <plem/core/monitoring.hpp>
#include <plem/core/monitoring/event.hpp>
#include <plem/core/monitoring/rt_sample.hpp>

namespace plem::core::monitoring {
namespace {

class MonitoringTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Ensure clean state before each test
        plem::core::monitoring::shutdown();
    }

    void TearDown() override {
        plem::core::monitoring::shutdown();
    }
};

/// The sequence numbers of what `T`'s queue holds, oldest first (drains it).
template <typename T>
std::vector<uint64_t> drained_sequences() {
    std::vector<uint64_t> sequences;
    auto* q = plem::core::monitoring::queue<T>();
    for (; q != nullptr && q->front() != nullptr; q->pop()) {
        sequences.push_back(q->front()->sequence);
    }
    return sequences;
}

TEST_F(MonitoringTest, EachTypeNumbersItsOwnEmits) {
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(plem::core::monitoring::emit(RtSample{}));
    }
    for (int i = 0; i < 2; ++i) {
        ASSERT_TRUE(plem::core::monitoring::emit(Event{}));
    }
    EXPECT_EQ(drained_sequences<RtSample>(), (std::vector<uint64_t>{0, 1, 2}));
    EXPECT_EQ(drained_sequences<Event>(), (std::vector<uint64_t>{0, 1})) << "a separate counter";
}

TEST_F(MonitoringTest, ShutdownResetsSequences) {
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());
    ASSERT_TRUE(plem::core::monitoring::emit(RtSample{}));
    ASSERT_TRUE(plem::core::monitoring::emit(RtSample{}));

    plem::core::monitoring::shutdown();
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());
    ASSERT_TRUE(plem::core::monitoring::emit(RtSample{}));

    EXPECT_EQ(drained_sequences<RtSample>(), (std::vector<uint64_t>{0}));
}

TEST_F(MonitoringTest, PublicApiInitializeAndEmit) {
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());

    RtSample sample{};
    sample.loop_exec_us = 500.0f;

    EXPECT_TRUE(plem::core::monitoring::emit(sample));

    auto* q = plem::core::monitoring::queue<RtSample>();
    ASSERT_NE(q, nullptr);
    ASSERT_NE(q->front(), nullptr);
    EXPECT_EQ(q->front()->loop_exec_us, 500.0f);
    EXPECT_NE(q->front()->monotonic_ns, 0u);  // Auto-filled
    EXPECT_EQ(q->front()->sequence, 0u);      // First sequence

    plem::core::monitoring::shutdown();
}

TEST_F(MonitoringTest, EmitBeforeInitializeReturnsFalse) {
    RtSample sample{};
    EXPECT_FALSE(plem::core::monitoring::emit(sample));
}

TEST_F(MonitoringTest, DoubleInitializeIsIdempotent) {
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());

    // Emit first sample
    RtSample sample1{};
    sample1.loop_exec_us = 100.0f;
    EXPECT_TRUE(plem::core::monitoring::emit(sample1));

    // Double initialize should be no-op
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());

    // Emit second sample - sequence should continue
    RtSample sample2{};
    sample2.loop_exec_us = 200.0f;
    EXPECT_TRUE(plem::core::monitoring::emit(sample2));

    // Verify both samples are in queue with correct sequences
    auto* q = plem::core::monitoring::queue<RtSample>();
    ASSERT_NE(q, nullptr);

    // First sample
    ASSERT_NE(q->front(), nullptr);
    EXPECT_EQ(q->front()->loop_exec_us, 100.0f);
    EXPECT_EQ(q->front()->sequence, 0u);
    q->pop();

    // Second sample
    ASSERT_NE(q->front(), nullptr);
    EXPECT_EQ(q->front()->loop_exec_us, 200.0f);
    EXPECT_EQ(q->front()->sequence, 1u);

    plem::core::monitoring::shutdown();
}

TEST_F(MonitoringTest, QueueOverflowReturnsFalseAndTracksCount) {
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());

    auto* q = plem::core::monitoring::queue<RtSample>();
    ASSERT_NE(q, nullptr);

    // Get queue capacity from stats
    auto initial_stats = q->stats();
    const size_t capacity = initial_stats.capacity;
    ASSERT_GT(capacity, 0u);

    // Fill the queue to capacity
    size_t pushed_count = 0;
    for (size_t i = 0; i < capacity + 100; ++i) {
        RtSample sample{};
        sample.loop_exec_us = static_cast<float>(i);
        if (plem::core::monitoring::emit(sample)) {
            ++pushed_count;
        }
    }

    // Should have pushed exactly capacity items
    EXPECT_EQ(pushed_count, capacity);

    // Check overflow stats
    auto stats = q->stats();
    EXPECT_EQ(stats.overflow_total, 100u);
    EXPECT_EQ(stats.overflow_delta, 100u);
    EXPECT_EQ(stats.current_size, capacity);

    // Reset delta and verify
    q->reset_overflow_delta();
    auto stats_after_reset = q->stats();
    EXPECT_EQ(stats_after_reset.overflow_total, 100u);  // Total unchanged
    EXPECT_EQ(stats_after_reset.overflow_delta, 0u);    // Delta reset

    plem::core::monitoring::shutdown();
}

TEST_F(MonitoringTest, EmitAfterShutdownReturnsFalse) {
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());

    plem::core::monitoring::shutdown();
    EXPECT_EQ(plem::core::monitoring::queue<RtSample>(), nullptr);

    RtSample sample{};
    EXPECT_FALSE(plem::core::monitoring::emit(sample));
}

TEST_F(MonitoringTest, PublicApiInitializeWithCustomConfig) {
    Config custom_config{};
    custom_config.rt_queue_size = 2048;
    custom_config.event_queue_size = 128;

    ASSERT_TRUE(plem::core::monitoring::initialize(custom_config).is_ok());

    auto* rt_queue = plem::core::monitoring::queue<RtSample>();
    auto* event_queue = plem::core::monitoring::queue<Event>();
    ASSERT_NE(rt_queue, nullptr);
    ASSERT_NE(event_queue, nullptr);

    EXPECT_EQ(rt_queue->stats().capacity, 2048u);
    EXPECT_EQ(event_queue->stats().capacity, 128u);

    plem::core::monitoring::shutdown();
}

}  // namespace
}  // namespace plem::core::monitoring
