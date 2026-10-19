// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_control_command_service.cpp
/// @brief Unit tests for ControlCommandService
///
/// Validates that plem_rt_msgs/srv/SetControlCommand request flags are routed to
/// the configured callbacks and that the response aggregates per-flag outcomes
/// correctly.
///
/// The service callback (handle_request) is exposed publicly on the class so
/// tests can invoke it directly without spinning a ROS 2 executor; this keeps
/// the test focused on the routing/aggregation logic rather than rclcpp
/// transport.
///
/// Every `EXPECT_EQ(message, ...)` below is a characterization pin: the
/// response text is the user-facing contract and must survive this layer's
/// refactors byte for byte. The callback failure text "fake set_mode failure"
/// is part of the pinned messages; change both together.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <unistd.h>

#include <plem_rt_msgs/srv/set_control_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <plem/core/result.hpp>

#include "plem/drive/ros/control_command_service.hpp"

namespace plem::drive::ros {
namespace {

using SetControlCommand = plem_rt_msgs::srv::SetControlCommand;

/// @brief Test double for the command callbacks: records the calls in order
///        and can be told to fail or to throw.
class CallRecorder {
  public:
    ControlCommandService::CommandCallback bind(std::string name) {
        return [this, name = std::move(name)]() -> plem::core::Result<void> {
            calls_.push_back(name);
            if (throws_) {
                throw std::runtime_error("driver blew up");
            }
            if (fails_) {
                return plem::core::Result<void>::error("fake set_mode failure");
            }
            return plem::core::Result<void>::ok();
        };
    }

    void set_fails(bool fails) {
        fails_ = fails;
    }
    void set_throws(bool throws) {
        throws_ = throws;
    }
    [[nodiscard]] const std::vector<std::string>& calls() const {
        return calls_;
    }

  private:
    std::vector<std::string> calls_;
    bool fails_{false};
    bool throws_{false};
};

class ControlCommandServiceTest : public ::testing::Test {
  protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        reset_error_called_ = false;
    }
    void TearDown() override {
        service_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    /// The service on a host node of its own (`name` resolved against it).
    std::shared_ptr<ControlCommandService> serve(ControlCommandService::Callbacks callbacks,
                                                 const std::string& ns = "testbot",
                                                 const std::string& name = "~/control_command") {
        node_ = std::make_shared<rclcpp::Node>(host_name(), ns);
        return std::make_shared<ControlCommandService>(*node_, std::move(callbacks), name);
    }

    /// A node name no other process on the domain uses; the by-node graph
    /// query below matches (node name, namespace).
    static std::string host_name() {
        static int count = 0;
        return "ccs_host_" + std::to_string(::getpid()) + "_" + std::to_string(count++);
    }

    ControlCommandService::Callbacks make_callbacks() {
        ControlCommandService::Callbacks callbacks;
        callbacks.protective_stop = recorder_.bind("protective_stop");
        callbacks.release_brake = recorder_.bind("release_brake");
        callbacks.free_drive = recorder_.bind("free_drive");
        callbacks.reset_error = [this]() { reset_error_called_ = true; };
        return callbacks;
    }

    void make_service() {
        service_ = serve(make_callbacks());
    }

    /// @brief Build a request and invoke the service callback directly.
    std::pair<bool, std::string> invoke(bool reset_error, bool protective_stop, bool release_brake, bool free_drive) {
        auto request = std::make_shared<SetControlCommand::Request>();
        request->reset_error = reset_error;
        request->protective_stop = protective_stop;
        request->release_brake = release_brake;
        request->free_drive = free_drive;
        auto response = std::make_shared<SetControlCommand::Response>();
        service_->handle_request(request, response);
        return {response->success, response->message};
    }

    CallRecorder recorder_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<ControlCommandService> service_;
    bool reset_error_called_{false};
};

// Construction Tests

// Every callback is required, reset_error included: both hosts wire it, and a
// request that sets it must reach a recovery attempt, not an "unavailable" answer.
TEST_F(ControlCommandServiceTest, ConstructWithAMissingCallbackThrows) {
    for (const char* missing : {"protective_stop", "release_brake", "free_drive", "reset_error"}) {
        SCOPED_TRACE(missing);
        auto callbacks = make_callbacks();
        const std::string name = missing;
        if (name == "protective_stop") {
            callbacks.protective_stop = nullptr;
        } else if (name == "release_brake") {
            callbacks.release_brake = nullptr;
        } else if (name == "free_drive") {
            callbacks.free_drive = nullptr;
        } else {
            callbacks.reset_error = nullptr;
        }
        rclcpp::Node host("ccs_throw_host");
        try {
            ControlCommandService(host, std::move(callbacks), "~/control_command");
            ADD_FAILURE() << "constructed without " << name;
        } catch (const std::invalid_argument& e) {
            EXPECT_STREQ(e.what(), ("ControlCommandService: callback '" + name + "' is required").c_str());
        }
    }
}

// Flag Routing Tests

TEST_F(ControlCommandServiceTest, NoFlagsIsNoOpSuccess) {
    make_service();
    auto [success, message] = invoke(false, false, false, false);
    EXPECT_TRUE(success);
    EXPECT_EQ(message, "No control command flags set (no-op)");
    EXPECT_FALSE(reset_error_called_);
    EXPECT_TRUE(recorder_.calls().empty());
}

TEST_F(ControlCommandServiceTest, ResetErrorInvokesCallback) {
    make_service();
    auto [success, message] = invoke(/*reset_error=*/true, false, false, false);
    EXPECT_TRUE(success);
    EXPECT_TRUE(reset_error_called_);
    EXPECT_EQ(message, "reset_error: requested");
    EXPECT_TRUE(recorder_.calls().empty());
}

TEST_F(ControlCommandServiceTest, ProtectiveStopInvokesCallback) {
    make_service();
    auto [success, message] = invoke(false, /*protective_stop=*/true, false, false);
    EXPECT_TRUE(success);
    EXPECT_EQ(recorder_.calls(), std::vector<std::string>{"protective_stop"});
    EXPECT_EQ(message, "protective_stop: ok");
}

TEST_F(ControlCommandServiceTest, ReleaseBrakeInvokesCallback) {
    make_service();
    auto [success, message] = invoke(false, false, /*release_brake=*/true, false);
    EXPECT_TRUE(success);
    EXPECT_EQ(recorder_.calls(), std::vector<std::string>{"release_brake"});
    EXPECT_EQ(message, "release_brake: ok");
}

// A host whose release is only requested says so; the default stays "ok".
TEST_F(ControlCommandServiceTest, ReleaseBrakeAnswersTheHostsAcceptedText) {
    auto callbacks = make_callbacks();
    callbacks.release_brake_accepted = "requested";
    service_ = serve(std::move(callbacks));
    auto [success, message] = invoke(false, false, /*release_brake=*/true, false);
    EXPECT_TRUE(success);
    EXPECT_EQ(message, "release_brake: requested");
}

// A host whose stop runs on after the answer says so, alone or with dropped flags.
TEST_F(ControlCommandServiceTest, ProtectiveStopAnswersTheHostsAcceptedText) {
    auto callbacks = make_callbacks();
    callbacks.protective_stop_accepted = "requested";
    service_ = serve(std::move(callbacks));
    EXPECT_EQ(invoke(false, /*protective_stop=*/true, false, false).second, "protective_stop: requested");
    EXPECT_EQ(invoke(false, true, /*release_brake=*/true, false).second,
              "protective_stop: requested; release_brake ignored (the stop runs alone) — after the stop, send the "
              "command you want in its own request");
}

TEST_F(ControlCommandServiceTest, FreeDriveInvokesCallback) {
    make_service();
    auto [success, message] = invoke(false, false, false, /*free_drive=*/true);
    EXPECT_TRUE(success);
    EXPECT_EQ(recorder_.calls(), std::vector<std::string>{"free_drive"});
    EXPECT_EQ(message, "free_drive: ok");
}

TEST_F(ControlCommandServiceTest, NonContradictoryFlagsAggregateInFixedOrder) {
    make_service();
    auto [success, message] = invoke(true, false, true, true);
    EXPECT_TRUE(success);
    EXPECT_TRUE(reset_error_called_);
    EXPECT_EQ(recorder_.calls(), (std::vector<std::string>{"release_brake", "free_drive"}));
    EXPECT_EQ(message, "reset_error: requested; release_brake: ok; free_drive: ok");
}

// Stop first: protective_stop with a release runs the stop only and drops the
// releases and reset_error, success=false. Every stop row, with and without
// reset_error; reset_error + protective_stop is not a contradiction and keeps
// the fixed order.
TEST_F(ControlCommandServiceTest, AStopWithAReleaseRunsTheStopAndDropsTheRest) {
    struct Row {
        bool reset_error, release_brake, free_drive;
        bool success;
        bool reset_runs;
        const char* message;
    };
    const Row rows[] = {
        {false, false, false, true, false, "protective_stop: ok"},
        {true, false, false, true, true, "reset_error: requested; protective_stop: ok"},
        {false, true, false, false, false,
         "protective_stop: ok; release_brake ignored (the stop runs alone) — after the stop, send the "
         "command you want in its own request"},
        {true, true, false, false, false,
         "protective_stop: ok; release_brake, reset_error ignored (the stop runs alone) — after the stop, "
         "send the command you want in its own request"},
        {false, false, true, false, false,
         "protective_stop: ok; free_drive ignored (the stop runs alone) — after the stop, send the command "
         "you want in its own request"},
        {true, false, true, false, false,
         "protective_stop: ok; free_drive, reset_error ignored (the stop runs alone) — after the stop, send "
         "the command you want in its own request"},
        {false, true, true, false, false,
         "protective_stop: ok; release_brake, free_drive ignored (the stop runs alone) — after the stop, "
         "send the command you want in its own request"},
        {true, true, true, false, false,
         "protective_stop: ok; release_brake, free_drive, reset_error ignored (the stop runs alone) — after "
         "the stop, send the command you want in its own request"},
    };
    for (const Row& row : rows) {
        SCOPED_TRACE(row.message);
        CallRecorder recorder;
        reset_error_called_ = false;
        auto callbacks = make_callbacks();
        callbacks.protective_stop = recorder.bind("protective_stop");
        callbacks.release_brake = recorder.bind("release_brake");
        callbacks.free_drive = recorder.bind("free_drive");
        service_ = serve(std::move(callbacks));
        auto [success, message] = invoke(row.reset_error, /*protective_stop=*/true, row.release_brake, row.free_drive);
        EXPECT_EQ(success, row.success);
        EXPECT_EQ(reset_error_called_, row.reset_runs);
        EXPECT_EQ(recorder.calls(), std::vector<std::string>{"protective_stop"});
        EXPECT_EQ(message, row.message);
    }
}

TEST_F(ControlCommandServiceTest, AThrowingStopWithAReleaseStillNamesWhatWasDropped) {
    make_service();
    recorder_.set_throws(true);
    auto [success, message] = invoke(/*reset_error=*/true, /*protective_stop=*/true, false, /*free_drive=*/true);
    EXPECT_FALSE(success);
    EXPECT_FALSE(reset_error_called_);
    EXPECT_EQ(message,
              "protective_stop threw: driver blew up; free_drive, reset_error ignored (the stop runs alone) — after "
              "the stop, send the command you want in its own request");
}

TEST_F(ControlCommandServiceTest, AFailedStopWithAReleaseSaysTheStopFailed) {
    make_service();
    recorder_.set_fails(true);
    auto [success, message] = invoke(false, /*protective_stop=*/true, /*release_brake=*/true, false);
    EXPECT_FALSE(success);
    EXPECT_EQ(recorder_.calls(), std::vector<std::string>{"protective_stop"});
    EXPECT_EQ(message,
              "protective_stop failed: fake set_mode failure; release_brake ignored (the stop runs alone) — "
              "after the stop, send the command you want in its own request");
}

TEST_F(ControlCommandServiceTest, CallbackFailurePropagatesToResponse) {
    make_service();
    recorder_.set_fails(true);
    auto [success, message] = invoke(false, false, /*release_brake=*/true, false);
    EXPECT_FALSE(success);
    EXPECT_EQ(message, "release_brake: fake set_mode failure");
}

TEST_F(ControlCommandServiceTest, ResetErrorPlusCommandAggregates) {
    make_service();
    auto [success, message] = invoke(/*reset_error=*/true, false, /*release_brake=*/true, false);
    EXPECT_TRUE(success);
    EXPECT_TRUE(reset_error_called_);
    EXPECT_EQ(recorder_.calls(), std::vector<std::string>{"release_brake"});
    EXPECT_EQ(message, "reset_error: requested; release_brake: ok");
}

TEST_F(ControlCommandServiceTest, ResetErrorRunsEvenWhenLaterFlagFails) {
    make_service();
    recorder_.set_fails(true);
    auto [success, message] = invoke(/*reset_error=*/true, false, /*release_brake=*/true, false);
    EXPECT_FALSE(success);             // overall failure because release_brake failed
    EXPECT_TRUE(reset_error_called_);  // but reset_error still ran
    EXPECT_EQ(message, "reset_error: requested; release_brake: fake set_mode failure");
}

// Exception Safety

TEST_F(ControlCommandServiceTest, ExceptionFromCallbackIsCaughtAndReported) {
    make_service();
    recorder_.set_throws(true);
    auto [success, message] = invoke(false, false, /*release_brake=*/true, false);
    EXPECT_FALSE(success);
    EXPECT_EQ(message, "Exception while handling control command: driver blew up");
}

// Service Path: characterization of the user-facing contract

/// @brief Wait for @p path to appear as a service of @p node.
///
/// Scoped to the node on purpose: the graph-wide query goes green on any other
/// process in the domain publishing the same name. Each host node here has a
/// name of its own (host_name), so the by-node query sees only this test's
/// service. The by-node query throws while the node is not in the graph cache
/// yet, so catching it makes a slow discovery backend a quiet retry. The
/// deadline (3 s) is a discovery allowance, not a latency bound.
bool wait_for_service_path(const rclcpp::Node& node, const std::string& path) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        try {
            if (node.get_service_names_and_types_by_node(node.get_name(), node.get_namespace()).count(path) > 0) {
                return true;
            }
        } catch (const std::runtime_error&) {  // node not in the graph cache yet
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

/// The drive bus's form: a private name on its host node.
TEST_F(ControlCommandServiceTest, APrivateNameIsServedUnderTheHostNode) {
    service_ = serve(make_callbacks(), "bus7", "~/control_command");
    EXPECT_TRUE(wait_for_service_path(*node_, std::string("/bus7/") + node_->get_name() + "/control_command"));
}

/// A higher layer's form: a relative name resolves in the host's namespace.
TEST_F(ControlCommandServiceTest, ARelativeNameResolvesInTheHostNamespace) {
    service_ = serve(make_callbacks(), "arm1", "plem/control_command");
    EXPECT_TRUE(wait_for_service_path(*node_, "/arm1/plem/control_command"));
}

/// A host in the global namespace: "" and "/" both resolve to the root.
TEST_F(ControlCommandServiceTest, ARelativeNameAtTheRootHasOneSlash) {
    for (const std::string ns : {"", "/"}) {
        SCOPED_TRACE("namespace '" + ns + "'");
        service_ = serve(make_callbacks(), ns, "plem/control_command");
        EXPECT_TRUE(wait_for_service_path(*node_, "/plem/control_command"));
    }
}

}  // namespace
}  // namespace plem::drive::ros
