// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file monitoring.hpp
 * @brief Public API for RT monitoring system
 *
 * Usage:
 * @code
 * // At startup (before RT loop)
 * if (const auto ready = plem::core::monitoring::initialize(); !ready) {
 *     // ready.message() says why
 * }
 *
 * // In domain code (anywhere)
 * PLEM_EMIT(sample);
 *
 * // At shutdown
 * plem::core::monitoring::shutdown();
 * @endcode
 */

#include <cstdint>

#include <plem/core/monitoring/config.hpp>
#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/monitoring/rt_queue.hpp>

namespace plem::core::monitoring {

namespace detail {

/// @brief The global default context: emit() and queue() use it on a thread
///        that has no context of its own (set_thread_context).
inline MonitoringContext g_default_context;

}  // namespace detail

/**
 * @brief Initialize the monitoring system
 *
 * Creates the global default context's queue for each type in AllMetricTypes
 * (a type whose Config queue size is 0 gets none). Until then, on a thread
 * without its own context, emit() returns false and queue() returns nullptr.
 *
 * @param config Configuration (optional, uses defaults)
 * @return an error, and no change, when already initialized with a different
 *         config (call shutdown() first) or a queue size is above
 *         RtQueue::kMaxCapacity; the same config again is a no-op that succeeds.
 * @note Not synchronized — call from one thread before producers and consumers
 *       start.
 */
inline Result<void> initialize(const Config& config = {}) {
    return detail::g_default_context.initialize(config);
}

/**
 * @brief Shut down the monitoring system
 *
 * Releases all queues and resets state.
 * @note Not synchronized — call from one thread after every producer and
 *       consumer has stopped (the queues they hold are freed).
 */
inline void shutdown() {
    detail::g_default_context.shutdown();
}

/**
 * @brief Emit a metric to its queue (RT-safe)
 *
 * Automatically fills monotonic_ns (if zero) and always overwrites sequence.
 *
 * @tparam T Metric type (must be in AllMetricTypes)
 * @param data Metric data
 * @return true if pushed, false if not initialized, the stream is not
 *         allocated (its Config queue size is 0) or the queue is full
 *
 * @note RT-safe: wait-free, no allocation
 */
template <typename T>
[[nodiscard]] inline bool emit(T data) noexcept {
    auto* ctx = get_thread_context();
    if (ctx) {
        return ctx->emit(std::move(data));
    }
    return detail::g_default_context.emit(std::move(data));
}

/**
 * @brief Get queue for a metric type (consumer access)
 *
 * @tparam T Metric type
 * @return Pointer to queue, nullptr if not initialized or the stream is not
 *         allocated (its Config queue size is 0)
 */
template <typename T>
[[nodiscard]] inline RtQueue<T>* queue() noexcept {
    auto* ctx = get_thread_context();
    if (ctx) {
        return ctx->queue<T>();
    }
    return detail::g_default_context.queue<T>();
}

}  // namespace plem::core::monitoring

/// Emits a metric and drops the result. A full queue counts the drop in its own
/// statistics; an uninitialized context or an unallocated stream counts it
/// nowhere. Call emit() to see the result.
#define PLEM_EMIT(data) static_cast<void>(::plem::core::monitoring::emit(data))
