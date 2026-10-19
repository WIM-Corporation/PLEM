// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <plem/core/third_party/rigtorp/SPSCQueue.h>

#include "plem/core/monitoring/event.hpp"
#include "plem/core/monitoring/rt_sample.hpp"

namespace plem::core::monitoring {

/**
 * @brief Queue statistics for monitoring
 *
 * Contains current queue state and overflow counts for monitoring
 * queue health and detecting data loss.
 */
struct PLEM_CORE_PUBLIC QueueStats {
    size_t current_size;      ///< Current number of items in queue
    size_t capacity;          ///< Maximum queue capacity
    uint64_t overflow_total;  ///< Total overflow count since creation
    uint64_t overflow_delta;  ///< Overflow count since last reset

    /**
     * @brief Calculate fill percentage
     *
     * @return Fill percentage (0.0 - 100.0), 0.0 if capacity is zero
     */
    [[nodiscard]] float fill_percent() const noexcept {
        if (capacity == 0) {
            return 0.0f;
        }
        return static_cast<float>(current_size) / static_cast<float>(capacity) * 100.0f;
    }
};

/**
 * @brief Generic RT-safe SPSC queue with overflow tracking
 *
 * Wraps rigtorp::SPSCQueue for single producer (RT thread) single consumer
 * (publishing thread) pattern. Tracks overflow counts when the queue is full.
 *
 * @tparam T Trivially copyable type (enforced at compile time)
 *
 * @note Producer side (try_push) is RT-safe: no allocation, no locks, wait-free.
 * @note Consumer side (front, pop) is wait-free and allocates nothing too; it
 *       belongs to the single consumer (publishing) thread.
 */
template <typename T>
    requires std::is_trivially_copyable_v<T>
class RtQueue {
  public:
    /// Largest capacity: the ring adds a slack element and padding slots, so a
    /// capacity near SIZE_MAX would wrap; 2^24 entries is far past any RT stream.
    static constexpr size_t kMaxCapacity = size_t{1} << 24;

    /// @throws std::invalid_argument when capacity is 0 or above kMaxCapacity
    explicit RtQueue(size_t capacity) : queue_(checked(capacity)), capacity_(capacity) {}

    // Non-copyable, non-movable
    RtQueue(const RtQueue&) = delete;
    RtQueue& operator=(const RtQueue&) = delete;
    RtQueue(RtQueue&&) = delete;
    RtQueue& operator=(RtQueue&&) = delete;

    /**
     * @brief RT-safe push (wait-free)
     * @return true if push succeeded, false if queue was full
     */
    [[nodiscard]] bool try_push(const T& item) noexcept {
        if (queue_.try_push(item)) {
            return true;
        }
        overflow_count_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    /// @brief Get front element (consumer thread, wait-free)
    [[nodiscard]] T* front() noexcept {
        return queue_.front();
    }

    /// @brief Pop front element (consumer thread, wait-free)
    void pop() noexcept {
        queue_.pop();
    }

    /// @brief Get queue statistics (thread-safe, const)
    [[nodiscard]] QueueStats stats() const noexcept {
        // Snapshot first (acquire pairs with the release in reset_overflow_delta): the
        // total read after it is at least the one the snapshot came from, so a reset on
        // another thread between the two loads cannot make the delta wrap.
        const uint64_t snapshot = last_overflow_snapshot_.load(std::memory_order_acquire);
        const uint64_t total = overflow_count_.load(std::memory_order_relaxed);
        return QueueStats{
            .current_size = queue_.size(),
            .capacity = capacity_,
            .overflow_total = total,
            .overflow_delta = total - snapshot,
        };
    }

    /// @brief Reset overflow delta (call from consumer periodically)
    void reset_overflow_delta() noexcept {
        last_overflow_snapshot_.store(overflow_count_.load(std::memory_order_relaxed), std::memory_order_release);
    }

  private:
    static size_t checked(size_t capacity) {
        if (capacity == 0 || capacity > kMaxCapacity) {
            throw std::invalid_argument("RtQueue capacity must be 1.." + std::to_string(kMaxCapacity) + " (got " +
                                        std::to_string(capacity) + ")");
        }
        return capacity;
    }

    rigtorp::SPSCQueue<T> queue_;
    size_t capacity_;
    std::atomic<uint64_t> overflow_count_{0};
    std::atomic<uint64_t> last_overflow_snapshot_{0};  // written by reset_overflow_delta(), read by stats()
};

using EventQueue = RtQueue<Event>;
using RtSampleQueue = RtQueue<RtSample>;

}  // namespace plem::core::monitoring
