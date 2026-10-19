// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file rt_raw_batcher.hpp
/// @brief `/rt_raw`: every RT sample, batched onto plem_rt_msgs/RtRawBatch.
///
/// One batching rule for every publisher of RT samples: the drive bus, or
/// another host's monitoring publisher, hands its samples here. It knows samples only: where
/// they come from and what else a caller tracks about them (queue health,
/// sequence-gap counts) stay with the caller.

#pragma once

#include <cstdint>

#include <plem_rt_msgs/msg/rt_raw_batch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <plem/core/monitoring/rt_sample.hpp>

namespace plem::drive::ros {

/// @thread_safety One thread calls add/idle/flush (the queue's one consumer).
class RtRawBatcher {
  public:
    /// K samples per full batch.
    static constexpr uint16_t kBatchSamples = 40;
    /// Latency bound: sparse but monotonic input cannot sit in the batch longer.
    static constexpr uint64_t kMaxBatchAgeNs = 40'000'000ULL;
    /// Empty drains in a row after which a partial batch goes out (emission stopped mid-batch).
    static constexpr uint8_t kIdleFlushTicks = 2;

    /// Creates the publisher on `node`; topic and QoS from StreamTraits<RtSample>,
    /// relative to the node's namespace.
    explicit RtRawBatcher(rclcpp::Node& node);

    /// Appends one sample: its first axis_count joint slots, the batch's
    /// axis_count. A sample claiming more than kMaxBusAxes axes is corrupt and
    /// dropped (its joint arrays cannot be read). Splits before it on a different axis_count, or on corrupt
    /// input (a timestamp that goes backward or a non-increasing sequence); forward
    /// sequence and timestamp gaps stay inside the batch, since that loss and
    /// jitter is what consumers measure. Flushes at kBatchSamples, or once the
    /// batch spans kMaxBatchAgeNs. A batch's period_ns is its first sample's
    /// loop_period_us, the RT loop's own configured period.
    void add(const plem::core::monitoring::RtSample& sample);

    /// One drain that found nothing.
    void idle();

    /// Publishes the (possibly partial) batch now; nothing when empty.
    void flush();

  private:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<plem_rt_msgs::msg::RtRawBatch>::SharedPtr publisher_;
    /// Arrays reserved to kBatchSamples once; clear() keeps the heap, so appends never reallocate.
    plem_rt_msgs::msg::RtRawBatch batch_;
    uint8_t idle_ticks_ = 0;
};

}  // namespace plem::drive::ros
