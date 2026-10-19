// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "ros/rt_raw_relay.hpp"

#include <string>
#include <type_traits>

#include <plem/core/monitoring/rt_sample.hpp>
#include <plem/core/monitoring/stream_traits.hpp>
#include <plem/core/timing/timespec.hpp>
#include <rclcpp/time.hpp>

#include "plem/drive/ros/rt_raw_batcher.hpp"
#include "ros/rt_raw_arrays.hpp"

namespace plem::drive::ros {

namespace {

using plem_rt_msgs::msg::RtRawBatch;

constexpr auto kRtRawStream = plem::core::monitoring::StreamTraits<plem::core::monitoring::RtSample>::descriptor;

/// "" when `values` holds `expected` entries, else the refusal naming the field.
template <typename Values>
std::string length_refusal(const char* field, const Values& values, size_t expected) {
    if (values.size() == expected) {
        return {};
    }
    return std::string(field) + " has " + std::to_string(values.size()) + " values, expected " +
           std::to_string(expected);
}

}  // namespace

plem::core::Result<void> relay_refusal(const RtRawBatch& b) {
    const size_t n = b.batch_size;
    const size_t nj = n * b.axis_count;
    std::string why;
    const auto length = [&](size_t expected) {
        return [&b, &why, expected](const char* name, auto array, auto) {
            if (why.empty()) {
                why = length_refusal(name, b.*array, expected);
            }
        };
    };
    for_each_rt_raw_array(length(n), length(nj));
    if (!why.empty()) {
        return plem::core::Result<void>::error(why + " (batch_size " + std::to_string(n) + ", axis_count " +
                                               std::to_string(b.axis_count) + ")");
    }
    return plem::core::Result<void>::ok();
}

RtRawBatch sample_of(const RtRawBatch& b, size_t i) {
    const size_t a = b.axis_count;
    const auto joints = [&](const auto& v) {
        return std::decay_t<decltype(v)>(v.begin() + i * a, v.begin() + (i + 1) * a);
    };
    RtRawBatch out;
    out.header.frame_id = b.header.frame_id;
    out.header.stamp =
        rclcpp::Time(sample_stamp_ns(int64_t{b.header.stamp.sec} * plem::core::kNsecPerSec + b.header.stamp.nanosec,
                                     b.anchor_monotonic_ns, b.monotonic_ns[i]));
    out.anchor_monotonic_ns = b.monotonic_ns[i];
    out.period_ns = b.period_ns;
    out.batch_size = 1;
    out.axis_count = b.axis_count;
    for_each_rt_raw_array([&](const char*, auto array, auto) { out.*array = {(b.*array)[i]}; },
                          [&](const char*, auto array, auto) { out.*array = joints(b.*array); });
    return out;
}

RtRawRelay::RtRawRelay(const rclcpp::NodeOptions& options) : rclcpp::Node("rt_raw_relay", options) {
    // Same QoS as the batches; the output keeps as many samples in flight as
    // the input's depth holds batches of.
    pub_ = create_publisher<RtRawBatch>(
        std::string(kRtRawStream.topic) + "/samples",
        rclcpp::QoS(size_t{kRtRawStream.depth} * RtRawBatcher::kBatchSamples).best_effort());
    sub_ = create_subscription<RtRawBatch>(kRtRawStream.topic, rclcpp::QoS(kRtRawStream.depth).best_effort(),
                                           [this](const RtRawBatch& batch) { on_batch(batch); });
    RCLCPP_INFO(get_logger(), "relaying %s -> %s, one message per sample", sub_->get_topic_name(),
                pub_->get_topic_name());
}

void RtRawRelay::on_batch(const RtRawBatch& batch) {
    if (const auto relayable = relay_refusal(batch); !relayable) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "batch dropped: %s", relayable.message().c_str());
        return;
    }
    for (size_t i = 0; i < batch.batch_size; ++i) {
        pub_->publish(sample_of(batch, i));
    }
}

}  // namespace plem::drive::ros
