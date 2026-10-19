// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_stream_controller.cpp
/// @brief The stream lane: what each joint's command interface holds before a
/// message, while the stream is fresh, and once it stops.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <shared_mutex>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ros/stream_controller.hpp"

namespace plem::drive::ros {
namespace {

constexpr int64_t kStale = plem::drive::kCommandStaleTimeoutNs;
static_assert(stream_command(true, kStale, 2.5, false) == 2.5, "the budget itself is fresh");
static_assert(stream_command(false, 0, 2.5, true) == 0.0, "no message yet: a velocity joint is told 0");

TEST(StreamCommand, NaNOnceStaleOrBeforeTheFirstTorque) {
    EXPECT_TRUE(std::isnan(stream_command(true, kStale + 1, 2.5, true)));
    EXPECT_TRUE(std::isnan(stream_command(false, 0, 2.5, false))) << "a torque joint must stream first";
}

class StreamControllerTest : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        const auto options = rclcpp::NodeOptions().parameter_overrides(
            {{"joints", std::vector<std::string>{"v", "t"}},
             {"interface_names", std::vector<std::string>{"velocity", "effort"}}});
#if PLEM_ROS2_CONTROL_JAZZY
        ASSERT_EQ(controller_.init("stream_controller", "", 1000u, "", options), controller_interface::return_type::OK);
#else
        ASSERT_EQ(controller_.init("stream_controller", "", options), controller_interface::return_type::OK);
#endif
        ASSERT_EQ(controller_.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        std::vector<hardware_interface::LoanedCommandInterface> loaned;
        loaned.emplace_back(velocity_);
        loaned.emplace_back(effort_);
        controller_.assign_interfaces(std::move(loaned), {});
        ASSERT_EQ(controller_.get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
        publisher_node_ = std::make_shared<rclcpp::Node>("streamer");
        publisher_ = publisher_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/stream_controller/commands", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        executor_.add_node(controller_.get_node()->get_node_base_interface());
    }

    void update() {
        ASSERT_EQ(controller_.update(rclcpp::Time(0), rclcpp::Duration(0, 1'000'000)),
                  controller_interface::return_type::OK);
    }
    /// Publishes and spins until the controller has it (its interfaces show the values).
    void send(std::vector<double> values) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = values;
        for (int k = 0; k < 200; ++k) {
            publisher_->publish(msg);
            executor_.spin_some(std::chrono::milliseconds(5));
            update();
            if (velocity_.get_value() == values[0]) {
                return;
            }
        }
        FAIL() << "the controller never took the message";
    }

    double v_ = 7.0, t_ = 7.0;
    hardware_interface::CommandInterface velocity_{"v", "velocity", &v_};
    hardware_interface::CommandInterface effort_{"t", "effort", &t_};
    StreamController controller_;
    rclcpp::Node::SharedPtr publisher_node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(StreamControllerTest, BeforeAMessageVelocityHoldsZeroAndTorqueNothing) {
    update();
    EXPECT_EQ(v_, 0.0);
    EXPECT_TRUE(std::isnan(t_));
}

TEST_F(StreamControllerTest, AFreshStreamIsForwardedAndAStoppedOneLetGo) {
    send({0.5, -1.25});
    EXPECT_EQ(v_, 0.5);
    EXPECT_EQ(t_, -1.25);
    std::this_thread::sleep_for(std::chrono::nanoseconds(kStale) + std::chrono::milliseconds(10));
    update();
    EXPECT_TRUE(std::isnan(v_));
    EXPECT_TRUE(std::isnan(t_));
}

TEST_F(StreamControllerTest, AMessageOfTheWrongLengthIsDropped) {
    // Matched first, so the messages below reach the controller: a dropped
    // message and an undelivered one would otherwise look the same.
    for (int k = 0; k < 400 && publisher_->get_subscription_count() == 0; ++k) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GE(publisher_->get_subscription_count(), 1u) << "the controller never subscribed";
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {1.0};
    for (int k = 0; k < 20; ++k) {
        publisher_->publish(msg);
        executor_.spin_some(std::chrono::milliseconds(5));
    }
    update();
    EXPECT_EQ(v_, 0.0) << "still no stream";
}

TEST_F(StreamControllerTest, DeactivatingLeavesNoCommandBehind) {
    // The hardware sends any value it finds, stamped with the cycle's read: a
    // command left in the buffer would keep the axis going with no stream and
    // without the bus's 50 ms watch ever seeing a silence.
    send({0.5, -1.25});
    ASSERT_EQ(controller_.get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_TRUE(std::isnan(v_));
    EXPECT_TRUE(std::isnan(t_));
}

#if PLEM_ROS2_CONTROL_JAZZY
TEST_F(StreamControllerTest, AJointWhoseHandleIsBusyKeepsItsLastCommand) {
    send({0.5, -1.25});
    std::promise<void> held, release;
    std::thread reader([&] {
        std::shared_lock<std::shared_mutex> lock(effort_.get_mutex());
        held.set_value();
        release.get_future().wait();
    });
    held.get_future().wait();
    send({0.75, 2.0});  // set_value on the effort handle fails every try while the reader holds it
    release.set_value();
    reader.join();
    EXPECT_EQ(v_, 0.75);
    EXPECT_EQ(t_, -1.25) << "the busy joint keeps its last command; update() still returned OK";
    send({1.0, 3.0});  // a new message: the held one may have gone stale while the reader let go
    EXPECT_EQ(t_, 3.0) << "and takes the stream again once the handle is free";
}

/// Holds the given handles' locks from another thread for the duration of `body`
/// (a lock the calling thread owned itself would make its own try_lock undefined).
template <class Body>
void with_busy(std::vector<hardware_interface::CommandInterface*> busy, Body body) {
    std::promise<void> held, release;
    std::thread reader([&] {
        std::vector<std::shared_lock<std::shared_mutex>> locks;
        for (auto* handle : busy) {
            locks.emplace_back(handle->get_mutex());
        }
        held.set_value();
        release.get_future().wait();
    });
    held.get_future().wait();
    body();
    release.set_value();
    reader.join();
}

TEST_F(StreamControllerTest, DeactivatingWithOneBusyHandleStillStopsTheBus) {
    send({0.5, -1.25});
    with_busy({&effort_}, [&] {
        EXPECT_EQ(controller_.get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    });
    EXPECT_TRUE(std::isnan(v_)) << "the free handle takes NaN: one NaN axis stops the bus writes";
    EXPECT_EQ(t_, -1.25);
}

TEST_F(StreamControllerTest, DeactivatingWithEveryHandleBusyFails) {
    send({0.5, -1.25});
    with_busy({&velocity_, &effort_}, [&] {
        EXPECT_NE(controller_.get_node()->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    });
    EXPECT_EQ(v_, 0.5);
    EXPECT_EQ(t_, -1.25);
}
#endif

}  // namespace
}  // namespace plem::drive::ros
