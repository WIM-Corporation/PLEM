// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

/**
 * @file monitoring_context.hpp
 * @brief Metric queues one producer set owns. Each RT thread emits into its
 *        own context (set_thread_context); a thread with none emits into the
 *        global default context (plem/core/monitoring.hpp).
 */

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>

#include "plem/core/monitoring/config.hpp"
#include "plem/core/monitoring/event.hpp"
#include "plem/core/monitoring/rt_diagnostics.hpp"
#include "plem/core/monitoring/rt_queue.hpp"
#include "plem/core/monitoring/rt_sample.hpp"
#include "plem/core/monitoring/rt_signal.hpp"
#include "plem/core/result.hpp"
#include "plem/core/timing/timespec.hpp"

namespace plem::core::monitoring {

/// Every metric type a context carries, one queue each.
using AllMetricTypes = std::tuple<RtSample, Event, ControllerSignal, RtDiagnostics>;

/// @brief Per-instance monitoring context: one queue per metric type.
/// @thread_safety initialize()/shutdown() are not synchronized: call them from
///                one thread, initialize() before any producer or consumer
///                starts and shutdown() after all of them have stopped
///                (shutdown() frees the queues queue() handed out). queue()
///                is a plain read, not synchronized with either.
///                initialize() again with the same config, and shutdown()
///                when not initialized, are no-ops; initialize() with a
///                different config returns an error and changes nothing.
///                Every stream's queue is single-producer/single-consumer:
///                emit() of one metric type on one context must come from one
///                thread at a time (a new producer only after the previous one
///                has been joined), and queue() reads from one consumer
///                thread. Two producer threads emitting the same type
///                (typically Event) need a context each; nothing here detects
///                a second producer; it corrupts or loses entries.
///                PLEM_EMIT emits into the calling thread's context
///                (set_thread_context), or the global default context when
///                the thread has none.
class PLEM_CORE_PUBLIC MonitoringContext {
  public:
    /// @brief Initialize the monitoring context, creating all metric queues.
    /// @param config Configuration (optional, uses defaults)
    /// @return an error, and no change, when the context is already initialized
    ///         with a different config (call shutdown() first) or a queue size is
    ///         above RtQueue::kMaxCapacity; the same config again is a no-op that
    ///         succeeds.
    /// @note Not synchronized: call from one thread before producers and
    ///       consumers start.
    Result<void> initialize(const Config& config = {}) {
        if (initialized_) {
            if (config == config_) {
                return Result<void>::ok();
            }
            return Result<void>::error(
                "initialize() called again with a different config; the first config stays active until shutdown()");
        }
        for (const size_t size : {config.rt_queue_size, config.event_queue_size, config.controller_signal_queue_size,
                                  config.diagnostics_queue_size}) {
            if (size > RtQueue<Event>::kMaxCapacity) {
                return Result<void>::error("a queue size of " + std::to_string(size) +
                                           " exceeds RtQueue::kMaxCapacity (" +
                                           std::to_string(RtQueue<Event>::kMaxCapacity) + ")");
            }
        }
        config_ = config;

        // A size of 0 leaves that stream unallocated (queue<T>() then returns nullptr).
        auto queue_size = [&config]<typename T>() -> size_t {
            if constexpr (std::is_same_v<T, RtSample>) {
                return config.rt_queue_size;
            } else if constexpr (std::is_same_v<T, Event>) {
                return config.event_queue_size;
            } else if constexpr (std::is_same_v<T, ControllerSignal>) {
                return config.controller_signal_queue_size;
            } else {
                static_assert(std::is_same_v<T, RtDiagnostics>, "every metric type takes its queue size from Config");
                return config.diagnostics_queue_size;
            }
        };
        auto make_queue = [&queue_size]<typename T>(std::unique_ptr<RtQueue<T>>& queue) {
            const size_t size = queue_size.template operator()<T>();
            queue = size > 0 ? std::make_unique<RtQueue<T>>(size) : nullptr;
        };
        // Build every queue first: an allocation that throws leaves the context untouched.
        decltype(streams_.queues) queues;
        std::apply([&make_queue](auto&... q) { (make_queue(q), ...); }, queues);
        streams_.queues = std::move(queues);
        initialized_ = true;
        return Result<void>::ok();
    }

    /// @brief Shut down the monitoring context, releasing all queues.
    /// @note Not synchronized: call from one thread after every producer and
    ///       consumer has stopped (the queues they hold are freed).
    void shutdown() {
        if (!initialized_) {
            return;
        }
        initialized_ = false;
        streams_.queues = {};
        std::apply([](auto&... seqs) { (seqs.value.store(0, std::memory_order_relaxed), ...); }, streams_.sequences);
    }

    /// @brief Emit a metric to its queue (RT-safe)
    ///
    /// Automatically fills monotonic_ns (if zero) and always overwrites sequence.
    ///
    /// @tparam T Metric type (must be in AllMetricTypes)
    /// @param data Metric data
    /// @return true if pushed, false if not initialized, the stream is not
    ///         allocated (queue size 0) or the queue is full
    /// @note RT-safe: wait-free, no allocation
    /// @note Single producer per type per context (see the class note).
    template <typename T>
    [[nodiscard]] bool emit(T data) noexcept {
        auto* q = queue<T>();
        if (!q) [[unlikely]] {
            return false;
        }
        if (data.monotonic_ns == 0) {
            data.monotonic_ns = static_cast<uint64_t>(core::now_ns());
        }
        data.sequence = std::get<Sequence<T>>(streams_.sequences).value.fetch_add(1, std::memory_order_relaxed);
        return q->try_push(data);
    }

    /// @brief Get queue for a metric type (consumer access)
    /// @tparam T Metric type
    /// @return Pointer to queue, nullptr if not initialized or the stream was
    ///         configured with a queue size of 0
    template <typename T>
    [[nodiscard]] RtQueue<T>* queue() noexcept {
        return std::get<std::unique_ptr<RtQueue<T>>>(streams_.queues).get();
    }

  private:
    // One distinct type per metric type, so std::get<Sequence<T>> picks T's counter.
    template <typename T>
    struct Sequence {
        std::atomic<uint64_t> value{0};
    };

    template <typename Types>
    struct Streams;
    template <typename... Ts>
    struct Streams<std::tuple<Ts...>> {
        std::tuple<std::unique_ptr<RtQueue<Ts>>...> queues;
        std::tuple<Sequence<Ts>...> sequences{};
    };

    Streams<AllMetricTypes> streams_;
    bool initialized_{false};
    Config config_{};
};

// Thread-local context routing (per-thread PLEM_EMIT routing)

/// @brief Set the monitoring context for the current thread
/// @param ctx Context pointer (nullptr to reset to default)
/// @note Call this in each RT thread's initialization before PLEM_EMIT
/// @note RT-safe: single pointer store, no allocation, noexcept
PLEM_CORE_PUBLIC void set_thread_context(MonitoringContext* ctx) noexcept;

/// @brief Get the monitoring context for the current thread
/// @return Thread-local context, or nullptr if none was set (emit() then
///         falls back to the global default)
/// @note RT-safe: single TLS pointer read
[[nodiscard]] PLEM_CORE_PUBLIC MonitoringContext* get_thread_context() noexcept;

}  // namespace plem::core::monitoring
