// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/drive/ros/rt_raw_batcher.hpp"

#include <cmath>

#include <plem/core/monitoring/stream_traits.hpp>
#include <plem/core/timing/timespec.hpp>

#include "ros/rt_raw_arrays.hpp"

namespace plem::drive::ros {

namespace {

using plem::core::monitoring::RtSample;

constexpr auto kRtRawStream = plem::core::monitoring::StreamTraits<RtSample>::descriptor;
static_assert(kRtRawStream.qos == plem::core::monitoring::QoSPreset::BestEffort,
              "RtRawBatcher builds a best_effort QoS: the /rt_raw descriptor changed");

}  // namespace

RtRawBatcher::RtRawBatcher(rclcpp::Node& node)
    : clock_(node.get_clock()),
      // Depth counts batches (StreamTraits descriptor).
      publisher_(node.create_publisher<plem_rt_msgs::msg::RtRawBatch>(kRtRawStream.topic,
                                                                      rclcpp::QoS(kRtRawStream.depth).best_effort())) {
    const size_t k = kBatchSamples;
    const size_t kj = size_t{kBatchSamples} * plem::core::kMaxBusAxes;
    for_each_rt_raw_array([&](const char*, auto array, auto) { (batch_.*array).reserve(k); },
                          [&](const char*, auto array, auto) { (batch_.*array).reserve(kj); });
}

void RtRawBatcher::add(const RtSample& sample) {
    if (sample.axis_count > plem::core::kMaxBusAxes) {
        return;  // a corrupt record: its joint arrays hold kMaxBusAxes slots, so they cannot be read
    }
    idle_ticks_ = 0;
    if (batch_.batch_size > 0 &&
        (sample.axis_count != batch_.axis_count || sample.monotonic_ns < batch_.monotonic_ns.back() ||
         sample.sequence <= batch_.sequence.back())) {
        flush();
    }

    if (batch_.batch_size == 0) {
        batch_.period_ns = static_cast<uint32_t>(std::lround(sample.loop_period_us * 1000.0f));
        batch_.axis_count = sample.axis_count;
    }
    for_each_rt_raw_array([&](const char*, auto array, auto field) { (batch_.*array).push_back(sample.*field); },
                          [&](const char*, auto array, auto field) {
                              for (size_t j = 0; j < sample.axis_count; ++j) {
                                  (batch_.*array).push_back((sample.*field)[j]);
                              }
                          });
    ++batch_.batch_size;

    if (batch_.batch_size >= kBatchSamples || sample.monotonic_ns - batch_.monotonic_ns.front() >= kMaxBatchAgeNs) {
        flush();
    }
}

void RtRawBatcher::idle() {
    if (batch_.batch_size > 0 && ++idle_ticks_ >= kIdleFlushTicks) {
        flush();
    }
}

void RtRawBatcher::flush() {
    if (batch_.batch_size == 0) {
        return;
    }
    // epoch<->mono correlation anchor: two adjacent clock reads at flush
    // (µs-scale skew; publish-time correlation, not a literal same-instant pair).
    batch_.header.stamp = clock_->now();
    batch_.anchor_monotonic_ns = static_cast<uint64_t>(plem::core::now_ns());
    publisher_->publish(batch_);

    const auto clear = [&](const char*, auto array, auto) { (batch_.*array).clear(); };
    for_each_rt_raw_array(clear, clear);
    batch_.batch_size = 0;
    idle_ticks_ = 0;
}

}  // namespace plem::drive::ros
