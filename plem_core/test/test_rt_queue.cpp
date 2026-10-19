// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/monitoring/rt_queue.hpp>

#include <limits>
#include <stdexcept>
#include <thread>

using namespace plem::core::monitoring;

// QueueStats Tests

class QueueStatsTest : public ::testing::Test {};

TEST_F(QueueStatsTest, FillPercentZeroWhenCapacityZero) {
    // Edge case: avoid division by zero
    QueueStats stats{.current_size = 0, .capacity = 0, .overflow_total = 0, .overflow_delta = 0};

    EXPECT_FLOAT_EQ(stats.fill_percent(), 0.0f);
}

// RtSampleQueue Basic Tests

class RtSampleQueueTest : public ::testing::Test {
  protected:
    static constexpr size_t kCapacity = 16;
};

TEST_F(RtSampleQueueTest, ConstructionWithCapacity) {
    RtSampleQueue queue(kCapacity);

    auto stats = queue.stats();
    EXPECT_EQ(stats.capacity, kCapacity);
    EXPECT_EQ(stats.current_size, 0u);
    EXPECT_EQ(stats.overflow_total, 0u);
    EXPECT_EQ(stats.overflow_delta, 0u);
}

// The vendor ring adds a slack element to the capacity: SIZE_MAX would wrap it to
// 0 and every push would write past the allocation. 0 is refused too (the ring
// would silently hold one element while stats() reported a capacity of 0).
TEST_F(RtSampleQueueTest, RefusesACapacityOutsideOneToMax) {
    EXPECT_THROW(RtSampleQueue{0}, std::invalid_argument);
    EXPECT_THROW(RtSampleQueue{RtSampleQueue::kMaxCapacity + 1}, std::invalid_argument);
    EXPECT_THROW(RtSampleQueue{std::numeric_limits<size_t>::max()}, std::invalid_argument);
    EXPECT_NO_THROW(RtQueue<uint8_t>{RtQueue<uint8_t>::kMaxCapacity});
}

// The smallest queue holds exactly one element: a second push is an overflow, and
// the one element comes back.
TEST_F(RtSampleQueueTest, ACapacityOfOneHoldsExactlyOneElement) {
    RtQueue<uint8_t> queue(1);
    EXPECT_TRUE(queue.try_push(7));
    EXPECT_FALSE(queue.try_push(8));
    EXPECT_EQ(queue.stats().capacity, 1u);
    EXPECT_EQ(queue.stats().current_size, 1u);
    EXPECT_EQ(queue.stats().overflow_total, 1u);
    ASSERT_NE(queue.front(), nullptr);
    EXPECT_EQ(*queue.front(), 7);
    queue.pop();
    EXPECT_EQ(queue.front(), nullptr);
}

TEST_F(RtSampleQueueTest, PushAndPopSample) {
    RtSampleQueue queue(kCapacity);

    RtSample sample{};
    sample.monotonic_ns = 1234567890;
    sample.sequence = 42;
    sample.loop_exec_us = 850.0f;

    EXPECT_TRUE(queue.try_push(sample));
    EXPECT_EQ(queue.stats().current_size, 1u);

    RtSample* front = queue.front();
    ASSERT_NE(front, nullptr);
    EXPECT_EQ(front->monotonic_ns, 1234567890u);
    EXPECT_EQ(front->sequence, 42u);
    EXPECT_FLOAT_EQ(front->loop_exec_us, 850.0f);

    queue.pop();
    EXPECT_EQ(queue.stats().current_size, 0u);
    EXPECT_EQ(queue.front(), nullptr);
}

TEST_F(RtSampleQueueTest, MultiplePushPop) {
    RtSampleQueue queue(kCapacity);

    // Push multiple samples
    for (uint64_t i = 0; i < 5; ++i) {
        RtSample sample{};
        sample.sequence = i;
        EXPECT_TRUE(queue.try_push(sample));
    }

    EXPECT_EQ(queue.stats().current_size, 5u);

    // Pop and verify order (FIFO)
    for (uint64_t i = 0; i < 5; ++i) {
        RtSample* front = queue.front();
        ASSERT_NE(front, nullptr);
        EXPECT_EQ(front->sequence, i);
        queue.pop();
    }

    EXPECT_EQ(queue.front(), nullptr);
}

// RtSampleQueue Overflow Tests

class RtSampleQueueOverflowTest : public ::testing::Test {
  protected:
    static constexpr size_t kSmallCapacity = 4;
};

TEST_F(RtSampleQueueOverflowTest, ResetOverflowDelta) {
    RtSampleQueue queue(kSmallCapacity);

    // Fill the queue and cause overflows
    RtSample sample{};
    for (size_t i = 0; i < kSmallCapacity; ++i) {
        EXPECT_TRUE(queue.try_push(sample));
    }
    for (int i = 0; i < 3; ++i) {
        EXPECT_FALSE(queue.try_push(sample));
    }

    auto stats_before = queue.stats();
    EXPECT_EQ(stats_before.overflow_total, 3u);
    EXPECT_EQ(stats_before.overflow_delta, 3u);

    // Reset delta
    queue.reset_overflow_delta();

    auto stats_after = queue.stats();
    EXPECT_EQ(stats_after.overflow_total, 3u);  // Total unchanged
    EXPECT_EQ(stats_after.overflow_delta, 0u);  // Delta reset

    // Cause more overflows
    for (int i = 0; i < 2; ++i) {
        EXPECT_FALSE(queue.try_push(sample));
    }

    auto stats_final = queue.stats();
    EXPECT_EQ(stats_final.overflow_total, 5u);  // Total incremented
    EXPECT_EQ(stats_final.overflow_delta, 2u);  // Delta since reset
}

// RtSampleQueue Fill Percentage Tests

TEST_F(RtSampleQueueTest, FillPercentage) {
    RtSampleQueue queue(kCapacity);

    EXPECT_FLOAT_EQ(queue.stats().fill_percent(), 0.0f);

    // Push half the capacity
    RtSample sample{};
    for (size_t i = 0; i < kCapacity / 2; ++i) {
        EXPECT_TRUE(queue.try_push(sample));
    }

    EXPECT_FLOAT_EQ(queue.stats().fill_percent(), 50.0f);

    // Fill the rest
    for (size_t i = 0; i < kCapacity / 2; ++i) {
        EXPECT_TRUE(queue.try_push(sample));
    }

    EXPECT_FLOAT_EQ(queue.stats().fill_percent(), 100.0f);
}

// Concurrent Tests

class RtSampleQueueConcurrentTest : public ::testing::Test {
  protected:
    static constexpr size_t kCapacity = 1024;
    static constexpr int kNumItems = 500;
};

TEST_F(RtSampleQueueConcurrentTest, SingleProducerSingleConsumer) {
    RtSampleQueue queue(kCapacity);
    std::atomic<int> consumed_count{0};

    // Producer thread
    std::thread producer([&queue]() {
        for (int i = 0; i < kNumItems; ++i) {
            RtSample sample{};
            sample.sequence = static_cast<uint64_t>(i);
            while (!queue.try_push(sample)) {
                // Spin until push succeeds
            }
        }
    });

    // Consumer thread
    std::thread consumer([&queue, &consumed_count]() {
        uint64_t expected = 0;
        while (expected < kNumItems) {
            RtSample* front = queue.front();
            if (front != nullptr) {
                EXPECT_EQ(front->sequence, expected);
                queue.pop();
                ++expected;
                consumed_count.fetch_add(1);
            }
        }
    });

    producer.join();
    consumer.join();

    EXPECT_EQ(consumed_count.load(), kNumItems);
    EXPECT_EQ(queue.stats().overflow_total, 0u);
}
