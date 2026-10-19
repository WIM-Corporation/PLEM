// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_rt_raw_batcher.cpp
/// @brief RtRawBatcher's axis count: a batch carries exactly the axes its
/// samples say they carry, stride = axis_count, and nothing past them.

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

#include <plem_rt_msgs/msg/rt_raw_batch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <plem/core/monitoring/rt_sample.hpp>

#include "plem/drive/ros/rt_raw_batcher.hpp"

namespace plem::drive::ros {
namespace {

using namespace std::chrono_literals;
using plem::core::kMaxBusAxes;
using plem::core::monitoring::RtSample;

/// Every RtSample joint slot filled, the ones past a small bus included: a
/// batcher that copies them shows up as values the producer never owned.
RtSample sample(uint64_t seq, uint8_t axes) {
    RtSample s{};
    s.axis_count = axes;
    s.loop_period_us = 1000.0f;
    s.sequence = seq;
    s.monotonic_ns = 1'000'000'000ULL + seq * 1'000'000ULL;
    s.dc_sync_monitor_ns = seq == 2 ? std::numeric_limits<uint32_t>::max() : static_cast<uint32_t>(100 + seq);
    for (size_t j = 0; j < kMaxBusAxes; ++j) {
        const auto v = static_cast<float>(seq * 10 + j);
        s.actual_position[j] = v;
        s.actual_velocity[j] = v + 0.1f;
        s.actual_torque[j] = v + 0.2f;
        s.command_position[j] = v + 0.3f;
        s.command_velocity[j] = v + 0.4f;
        s.command_torque[j] = v + 0.5f;
        s.applied_command[j] = v + 0.6f;
        s.statusword[j] = static_cast<uint16_t>(seq * 10 + j);
        s.modes_of_operation_display[j] = static_cast<int8_t>(j);
    }
    return s;
}

class RtRawBatcherTest : public ::testing::Test {
  protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("rt_raw_batcher_test");
        sub_ = node_->create_subscription<plem_rt_msgs::msg::RtRawBatch>(
            "rt_raw", rclcpp::QoS(rclcpp::KeepLast(64)).best_effort(),
            [this](const plem_rt_msgs::msg::RtRawBatch& msg) { batches_.push_back(msg); });
    }
    void TearDown() override {
        sub_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    /// Every batch `samples` flush into, in order; `expected` samples are waited for
    /// (all of them unless some are dropped).
    std::vector<plem_rt_msgs::msg::RtRawBatch> batches_of(const std::vector<RtSample>& samples,
                                                          std::optional<size_t> expected = std::nullopt) {
        RtRawBatcher batcher(*node_);
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node_);
        const auto matched = std::chrono::steady_clock::now() + 2s;
        while (sub_->get_publisher_count() == 0 && std::chrono::steady_clock::now() < matched) {
            exec.spin_some(10ms);
        }
        for (const auto& s : samples) {
            batcher.add(s);
        }
        batcher.flush();
        size_t published = 0;
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        const size_t wanted = expected.value_or(samples.size());
        while (published < wanted && std::chrono::steady_clock::now() < deadline) {
            exec.spin_some(10ms);
            std::this_thread::sleep_for(1ms);
            published = 0;
            for (const auto& b : batches_) {
                published += b.batch_size;
            }
        }
        exec.remove_node(node_);
        return batches_;
    }

    /// One flushed batch of `n` samples of `axes` axes.
    plem_rt_msgs::msg::RtRawBatch batch_of(uint8_t axes, uint64_t n) {
        std::vector<RtSample> samples;
        for (uint64_t i = 0; i < n; ++i) {
            samples.push_back(sample(i, axes));
        }
        const auto batches = batches_of(samples);
        EXPECT_EQ(batches.size(), 1u);
        return batches.empty() ? plem_rt_msgs::msg::RtRawBatch{} : batches.front();
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<plem_rt_msgs::msg::RtRawBatch>::SharedPtr sub_;
    std::vector<plem_rt_msgs::msg::RtRawBatch> batches_;
};

// A sample claiming more axes than its record holds is corrupt: dropped, the
// samples around it batched as if it never came.
TEST_F(RtRawBatcherTest, ASampleWiderThanItsRecordIsDropped) {
    const auto batches = batches_of({sample(0, 2), sample(1, kMaxBusAxes + 1), sample(2, 2)}, 2);
    ASSERT_EQ(batches.size(), 1u);
    const auto& b = batches.front();
    ASSERT_EQ(b.batch_size, 2u);
    EXPECT_EQ(b.axis_count, 2u);
    ASSERT_EQ(b.sequence.size(), 2u);
    EXPECT_EQ(b.sequence[0], 0u);
    EXPECT_EQ(b.sequence[1], 2u);
    ASSERT_EQ(b.actual_position.size(), 2u * 2u);
    EXPECT_FLOAT_EQ(b.actual_position[2], 20.0f) << "the sample after the corrupt one, at stride 2";
}

TEST_F(RtRawBatcherTest, ATwoAxisBatchCarriesTwoAxesPerSampleAndNoFillers) {
    const auto b = batch_of(2, 3);
    EXPECT_EQ(b.axis_count, 2u);
    ASSERT_EQ(b.batch_size, 3u);
    for (const auto* joint : {&b.actual_position, &b.actual_velocity, &b.actual_torque, &b.command_position,
                              &b.command_velocity, &b.command_torque, &b.applied_command}) {
        ASSERT_EQ(joint->size(), 3u * 2u);
    }
    ASSERT_EQ(b.statusword.size(), 3u * 2u);
    ASSERT_EQ(b.modes_of_operation_display.size(), 3u * 2u);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            const auto v = static_cast<float>(i * 10 + j);
            EXPECT_FLOAT_EQ(b.actual_position[i * 2 + j], v) << "sample-major, stride = axis_count";
            EXPECT_FLOAT_EQ(b.applied_command[i * 2 + j], v + 0.6f);
            EXPECT_EQ(b.statusword[i * 2 + j], i * 10 + j);
            EXPECT_EQ(b.modes_of_operation_display[i * 2 + j], static_cast<int8_t>(j));
        }
    }
    EXPECT_EQ(std::vector<uint32_t>(b.dc_sync_monitor_ns.begin(), b.dc_sync_monitor_ns.end()),
              (std::vector<uint32_t>{100, 101, std::numeric_limits<uint32_t>::max()}))
        << "one raw DC value per sample, the unavailable sentinel included";
}

// A full bus: every RtSample slot, the last axis included.
TEST_F(RtRawBatcherTest, AnEightAxisBatchCarriesEveryRtSampleSlot) {
    const auto b = batch_of(static_cast<uint8_t>(kMaxBusAxes), 2);
    EXPECT_EQ(b.axis_count, kMaxBusAxes);
    ASSERT_EQ(b.actual_position.size(), 2u * kMaxBusAxes);
    ASSERT_EQ(b.command_torque.size(), 2u * kMaxBusAxes);
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < kMaxBusAxes; ++j) {
            const auto v = static_cast<float>(i * 10 + j);
            EXPECT_FLOAT_EQ(b.actual_position[i * kMaxBusAxes + j], v);
            EXPECT_FLOAT_EQ(b.command_torque[i * kMaxBusAxes + j], v + 0.5f);
        }
    }
    EXPECT_EQ(std::vector<uint32_t>(b.dc_sync_monitor_ns.begin(), b.dc_sync_monitor_ns.end()),
              (std::vector<uint32_t>{100, 101}))
        << "a full-width batch carries the same per-sample DC value";
}

// One axis_count per batch: a sample with another count starts the next one.
TEST_F(RtRawBatcherTest, ADifferentAxisCountStartsANewBatch) {
    const auto batches = batches_of({sample(0, 2), sample(1, 2), sample(2, 3)});
    ASSERT_EQ(batches.size(), 2u);
    EXPECT_EQ(batches[0].axis_count, 2u);
    EXPECT_EQ(batches[0].actual_position.size(), 2u * 2u);
    EXPECT_EQ(batches[1].axis_count, 3u);
    EXPECT_EQ(batches[1].actual_position.size(), 3u);
}

}  // namespace
}  // namespace plem::drive::ros
