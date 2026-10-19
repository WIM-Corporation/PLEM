// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file rt_raw_relay.hpp
/// @brief `rt_raw_relay`: `rt_raw` batches in, one RtRawBatch per sample out on
/// `rt_raw/samples`, so a plotter that draws one point per message (PlotJuggler)
/// shows every RT cycle.
///
/// An on-demand observation tool for either tier: it reads the axis count from
/// each batch. It runs as an ordinary process (normal scheduling, outside the
/// RT-isolated cores) and never touches the producer's RT path.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <plem/core/result.hpp>

#include <plem_rt_msgs/msg/rt_raw_batch.hpp>
#include <rclcpp/rclcpp.hpp>

namespace plem::drive::ros {

/// Epoch time [ns] of the sample at CLOCK_MONOTONIC `sample_monotonic_ns`, by a
/// batch's adjacent (header.stamp, anchor_monotonic_ns) pair:
/// stamp − (anchor − sample). A sample after the anchor lands after the stamp.
[[nodiscard]] constexpr int64_t sample_stamp_ns(int64_t stamp_ns, uint64_t anchor_monotonic_ns,
                                                uint64_t sample_monotonic_ns) noexcept {
    return stamp_ns - (static_cast<int64_t>(anchor_monotonic_ns) - static_cast<int64_t>(sample_monotonic_ns));
}

/// Whether `batch` can be relayed: ok, or an error saying why not. Refused: an array whose length
/// disagrees with batch_size (per-sample) or batch_size × axis_count (per-joint).
[[nodiscard]] plem::core::Result<void> relay_refusal(const plem_rt_msgs::msg::RtRawBatch& batch);

/// Sample `i` of `batch` as a batch of one, same type and field names:
/// header.stamp = the sample's RT time (sample_stamp_ns), anchor_monotonic_ns =
/// its monotonic_ns, every array that sample's values, the rest copied.
/// Precondition: relay_refusal(batch).is_ok() and i < batch_size.
[[nodiscard]] plem_rt_msgs::msg::RtRawBatch sample_of(const plem_rt_msgs::msg::RtRawBatch& batch, size_t i);

/// The node: subscribes `rt_raw` and publishes `rt_raw/samples`, both relative to
/// its namespace (`-r __ns:=/<bus.name>` for a drive bus),
/// both best_effort like the source. A refused batch is dropped whole, with a
/// throttled warning.
/// @thread_safety Callbacks run on the executor that spins the node.
class RtRawRelay : public rclcpp::Node {
  public:
    explicit RtRawRelay(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  private:
    void on_batch(const plem_rt_msgs::msg::RtRawBatch& batch);

    rclcpp::Subscription<plem_rt_msgs::msg::RtRawBatch>::SharedPtr sub_;
    rclcpp::Publisher<plem_rt_msgs::msg::RtRawBatch>::SharedPtr pub_;
};

}  // namespace plem::drive::ros
