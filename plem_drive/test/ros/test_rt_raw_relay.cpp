// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_rt_raw_relay.cpp
/// @brief rt_raw_relay: one RtRawBatch per sample, stamped with that sample's
/// RT time, for any axis count.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <plem_rt_msgs/msg/rt_raw_batch.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros/rt_raw_relay.hpp"

namespace plem::drive::ros {
namespace {

using namespace std::chrono_literals;
using plem_rt_msgs::msg::RtRawBatch;

/// A message array as a std::vector, to compare with a literal.
template <typename Values>
auto vec(const Values& values) {
    return std::vector<std::decay_t<decltype(*values.begin())>>(values.begin(), values.end());
}

int64_t stamp_ns(const builtin_interfaces::msg::Time& t) {
    return int64_t{t.sec} * 1'000'000'000 + t.nanosec;
}

/// `n` samples of `axes` axes, 1 ms apart, flushed 0.5 ms after the last one.
/// Every value names its sample and axis.
RtRawBatch batch(uint16_t n, uint8_t axes) {
    RtRawBatch b;
    b.header.frame_id = "frame";
    b.header.stamp.sec = 1'700'000'000;
    b.header.stamp.nanosec = 250'000'000;
    b.period_ns = 1'000'000;
    b.batch_size = n;
    b.axis_count = axes;
    const uint64_t first = 5'000'000'000ULL;
    b.anchor_monotonic_ns = first + (n - 1) * 1'000'000ULL + 500'000ULL;
    for (uint16_t i = 0; i < n; ++i) {
        b.monotonic_ns.push_back(first + i * 1'000'000ULL);
        b.sequence.push_back(100 + i);
        b.loop_exec_us.push_back(10.0f + i);
        b.loop_jitter_us.push_back(0.5f + i);
        b.deadline_miss.push_back(i % 2);
        b.wkc.push_back(static_cast<uint16_t>(3 + i));
        b.wkc_mismatch.push_back(0);
        b.dc_sync_monitor_ns.push_back(200 + i);
        for (uint8_t j = 0; j < axes; ++j) {
            const auto v = static_cast<float>(i * 10 + j);
            b.actual_position.push_back(v);
            b.actual_velocity.push_back(v + 0.1f);
            b.actual_torque.push_back(v + 0.2f);
            b.command_position.push_back(v + 0.3f);
            b.command_velocity.push_back(v + 0.4f);
            b.command_torque.push_back(v + 0.5f);
            b.applied_command.push_back(v + 0.6f);
            b.statusword.push_back(static_cast<uint16_t>(i * 10 + j));
            b.modes_of_operation_display.push_back(static_cast<int8_t>(8 + j));
        }
    }
    return b;
}

// the stamp rule

TEST(RtRawRelayStamp, ASampleAtTheAnchorGetsTheHeaderStamp) {
    EXPECT_EQ(sample_stamp_ns(1'000'000'000, 7'000, 7'000), 1'000'000'000);
}

TEST(RtRawRelayStamp, ASampleBeforeTheAnchorIsEarlierByTheMonotonicGap) {
    EXPECT_EQ(sample_stamp_ns(1'000'000'000, 7'000, 2'000), 1'000'000'000 - 5'000);
}

TEST(RtRawRelayStamp, ASampleAfterTheAnchorIsLaterByTheMonotonicGap) {
    EXPECT_EQ(sample_stamp_ns(1'000'000'000, 7'000, 9'500), 1'000'000'000 + 2'500);
}

// one sample out

TEST(RtRawRelaySample, CarriesThatSamplesValuesAsABatchOfOne) {
    const auto in = batch(3, 2);
    const auto out = sample_of(in, 1);
    // Sample 1's two axes: entries 2 and 3 of a joint array.
    const auto second = [](const auto& values) { return std::vector(values.begin() + 2, values.begin() + 4); };
    EXPECT_EQ(out.batch_size, 1u);
    EXPECT_EQ(out.axis_count, 2u);
    EXPECT_EQ(out.period_ns, in.period_ns);
    EXPECT_EQ(out.header.frame_id, "frame");
    EXPECT_EQ(vec(out.monotonic_ns), std::vector<uint64_t>{in.monotonic_ns[1]});
    EXPECT_EQ(vec(out.sequence), std::vector<uint64_t>{101});
    EXPECT_EQ(vec(out.loop_exec_us), std::vector<float>{11.0f});
    EXPECT_EQ(vec(out.loop_jitter_us), std::vector<float>{1.5f});
    EXPECT_EQ(vec(out.deadline_miss), std::vector<uint8_t>{1});
    EXPECT_EQ(vec(out.wkc), std::vector<uint16_t>{4});
    EXPECT_EQ(vec(out.wkc_mismatch), std::vector<uint8_t>{0});
    EXPECT_EQ(vec(out.dc_sync_monitor_ns), std::vector<uint32_t>{201});
    EXPECT_EQ(vec(out.actual_position), (std::vector<float>{10.0f, 11.0f}));
    EXPECT_EQ(vec(out.actual_velocity), second(in.actual_velocity));
    EXPECT_EQ(vec(out.actual_torque), second(in.actual_torque));
    EXPECT_EQ(vec(out.command_position), second(in.command_position));
    EXPECT_EQ(vec(out.command_velocity), second(in.command_velocity));
    EXPECT_EQ(vec(out.command_torque), second(in.command_torque));
    EXPECT_EQ(vec(out.applied_command), second(in.applied_command));
    EXPECT_EQ(vec(out.statusword), (std::vector<uint16_t>{10, 11}));
    EXPECT_EQ(vec(out.modes_of_operation_display), (std::vector<int8_t>{8, 9}));
}

TEST(RtRawRelaySample, KeepsEveryAxisTheBatchCarries) {
    const auto out = sample_of(batch(2, 6), 1);
    EXPECT_EQ(out.axis_count, 6u);
    EXPECT_EQ(vec(out.actual_position), (std::vector<float>{10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f}));
}

TEST(RtRawRelaySample, IsStampedWithTheSamplesRtTimeAndAnchoredOnIt) {
    const auto in = batch(3, 1);
    for (size_t i = 0; i < 3; ++i) {
        const auto out = sample_of(in, i);
        EXPECT_EQ(stamp_ns(out.header.stamp),
                  sample_stamp_ns(stamp_ns(in.header.stamp), in.anchor_monotonic_ns, in.monotonic_ns[i]));
        EXPECT_EQ(out.anchor_monotonic_ns, in.monotonic_ns[i]);
    }
    // 2.5 ms, 1.5 ms, 0.5 ms before the flush.
    EXPECT_EQ(stamp_ns(sample_of(in, 0).header.stamp), stamp_ns(in.header.stamp) - 2'500'000);
}

// A consumer that reads the output by the same stamp/anchor rule gets the
// sample's original time, and relaying the output again changes nothing.
TEST(RtRawRelaySample, RoundTripsThroughTheSameStampRule) {
    const auto in = batch(4, 2);
    for (size_t i = 0; i < 4; ++i) {
        const auto out = sample_of(in, i);
        EXPECT_EQ(sample_stamp_ns(stamp_ns(out.header.stamp), out.anchor_monotonic_ns, out.monotonic_ns[0]),
                  sample_stamp_ns(stamp_ns(in.header.stamp), in.anchor_monotonic_ns, in.monotonic_ns[i]));
        EXPECT_EQ(sample_of(out, 0), out);
    }
}

TEST(RtRawRelaySample, BorrowsASecondWhenTheSampleIsBeforeTheStampsSecond) {
    auto in = batch(1, 1);
    in.header.stamp.sec = 10;
    in.header.stamp.nanosec = 100;
    in.anchor_monotonic_ns = in.monotonic_ns[0] + 200;  // the sample is 200 ns before the flush
    const auto out = sample_of(in, 0);
    EXPECT_EQ(out.header.stamp.sec, 9);
    EXPECT_EQ(out.header.stamp.nanosec, 999'999'900u);
}

// what is refused

TEST(RtRawRelayRefusal, AWellFormedBatchIsRelayed) {
    const auto valid = relay_refusal(batch(3, 2));
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    const auto valid2 = relay_refusal(batch(0, 2));
    EXPECT_TRUE(valid2.is_ok()) << valid2.message() << "an empty batch relays nothing, which is fine";
}

TEST(RtRawRelayRefusal, ArraysThatDisagreeWithTheCountsAreRefused) {
    auto short_scalar = batch(3, 2);
    short_scalar.sequence.pop_back();
    EXPECT_NE(relay_refusal(short_scalar).message().find("sequence"), std::string::npos)
        << relay_refusal(short_scalar).message();

    auto wrong_stride = batch(3, 2);
    wrong_stride.axis_count = 3;  // arrays still two wide
    EXPECT_NE(relay_refusal(wrong_stride).message().find("actual_position"), std::string::npos)
        << relay_refusal(wrong_stride).message();

    auto short_dc = batch(3, 2);
    short_dc.dc_sync_monitor_ns.pop_back();
    EXPECT_NE(relay_refusal(short_dc).message().find("dc_sync_monitor_ns"), std::string::npos)
        << relay_refusal(short_dc).message();
}

// the node

class RtRawRelayNodeTest : public ::testing::Test {
  protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);  // the executor needs a live context; build it after this
        exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        relay_ = std::make_shared<RtRawRelay>(rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/bus0"}));
        probe_ = std::make_shared<rclcpp::Node>("rt_raw_relay_probe");
        pub_ = probe_->create_publisher<RtRawBatch>("/bus0/rt_raw", rclcpp::QoS(8).best_effort());
        sub_ = probe_->create_subscription<RtRawBatch>("/bus0/rt_raw/samples", rclcpp::QoS(1000).best_effort(),
                                                       [this](RtRawBatch::SharedPtr msg) { out_.push_back(*msg); });
        exec_->add_node(relay_);
        exec_->add_node(probe_);
        ASSERT_TRUE(pump([this] { return pub_->get_subscription_count() > 0 && sub_->get_publisher_count() > 0; }));
    }
    void TearDown() override {
        exec_->remove_node(probe_);
        exec_->remove_node(relay_);
        sub_.reset();
        pub_.reset();
        probe_.reset();
        relay_.reset();
        exec_.reset();
        rclcpp::shutdown();
    }
    bool pump(const std::function<bool()>& done, std::chrono::milliseconds budget = 2s) {
        const auto deadline = std::chrono::steady_clock::now() + budget;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            exec_->spin_some(10ms);
            std::this_thread::sleep_for(1ms);
        }
        return done();
    }

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
    std::shared_ptr<RtRawRelay> relay_;
    std::shared_ptr<rclcpp::Node> probe_;
    rclcpp::Publisher<RtRawBatch>::SharedPtr pub_;
    rclcpp::Subscription<RtRawBatch>::SharedPtr sub_;
    std::vector<RtRawBatch> out_;
};

TEST_F(RtRawRelayNodeTest, RelaysEachSampleOfABatchInOrderUnderTheNamespace) {
    const auto in = batch(3, 2);
    pub_->publish(in);
    ASSERT_TRUE(pump([this] { return out_.size() == 3; })) << out_.size() << " of 3";
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_EQ(out_[i], sample_of(in, i)) << "sample " << i;
    }
}

TEST_F(RtRawRelayNodeTest, DropsARefusedBatchWhole) {
    auto bad = batch(3, 2);
    bad.actual_position.pop_back();
    pub_->publish(bad);
    pub_->publish(batch(1, 2));
    ASSERT_TRUE(pump([this] { return !out_.empty(); }));
    pump([] { return false; }, 100ms);
    ASSERT_EQ(out_.size(), 1u) << "nothing of the refused batch";
    EXPECT_EQ(vec(out_[0].sequence), std::vector<uint64_t>{100});
}

}  // namespace
}  // namespace plem::drive::ros
