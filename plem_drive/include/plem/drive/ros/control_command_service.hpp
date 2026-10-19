// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <functional>
#include <memory>
#include <string>

#include <plem_rt_msgs/srv/set_control_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <plem/core/result.hpp>

namespace plem::drive::ros {

/**
 * @brief ROS 2 service server for plem_rt_msgs::srv::SetControlCommand
 *
 * Binds the four boolean request flags to caller-supplied callbacks, so the
 * host decides what each flag does. The drive bus maps them onto its
 * host's enable/ramp_stop/reset (plem::drive::ros::DriveBus); another host
 * may bind them differently.
 *
 * Flags in a single request are processed independently in a fixed order:
 * reset_error, protective_stop, release_brake, free_drive. The response
 * aggregates per-flag outcomes into a single human-readable message.
 * response.success is true iff every requested flag succeeded.
 *
 * protective_stop with release_brake or free_drive runs the stop alone (see
 * SetControlCommand.srv).
 *
 * @thread_safety This class owns no mutable state: a request reads only the
 *                callbacks fixed at construction. The callbacks need
 *                serialising; both hosts spin the node on a
 *                SingleThreadedExecutor, which runs them one at a time. A host
 *                that uses a reentrant callback group must serialise its
 *                callbacks itself.
 *
 * The service lives on the host's node, under the name the host gives it: the
 * drive bus serves `~/control_command` (/<bus>/plem_drive/control_command); a
 * higher layer may serve a relative name on a node of its own.
 */
class ControlCommandService {
  public:
    using SetControlCommand = plem_rt_msgs::srv::SetControlCommand;
    /// @brief Requests a recovery attempt. Confirms nothing.
    using ResetErrorCallback = std::function<void()>;
    /// @brief Carries out one command flag; its error message reaches the caller verbatim.
    using CommandCallback = std::function<plem::core::Result<void>()>;

    /// @brief One callback per request flag, each required (the constructor
    ///        throws std::invalid_argument naming an empty one).
    ///        Fill it with designated initializers: the three command fields
    ///        share a type, so positional initialisation miswires them silently.
    struct Callbacks {
        CommandCallback protective_stop;
        CommandCallback release_brake;
        CommandCallback free_drive;
        ResetErrorCallback reset_error;
        /// What a successful release_brake answers after "release_brake: ".
        /// The drive bus says it is only requested; its readiness gate can
        /// still refuse a cycle later.
        std::string release_brake_accepted = "ok";
        /// The same for protective_stop: the drive bus's stop runs on after
        /// the answer (it decelerates, then engages).
        std::string protective_stop_accepted = "ok";
    };

    /**
     * @brief Serve SetControlCommand on @p node
     *
     * @param node The host node; it outlives this and its executor serves the
     *             requests.
     * @param callbacks Per-flag actions. Throws std::invalid_argument if any
     *                  of the four callbacks (reset_error included) is empty.
     * @param service_name Resolved by rclcpp against @p node: "~/x" is private
     *                     to the node, "a/x" relative to its namespace.
     */
    ControlCommandService(rclcpp::Node& node, Callbacks callbacks, const std::string& service_name);

    ~ControlCommandService() = default;

    ControlCommandService(const ControlCommandService&) = delete;
    ControlCommandService& operator=(const ControlCommandService&) = delete;
    ControlCommandService(ControlCommandService&&) = delete;
    ControlCommandService& operator=(ControlCommandService&&) = delete;

    /**
     * @brief Service callback, also exposed publicly for direct unit tests.
     *
     * Routes request flags to the configured callbacks. Catches any
     * std::exception raised by a callback and converts it into a failed
     * response with a descriptive message.
     */
    void handle_request(const std::shared_ptr<SetControlCommand::Request> request,
                        std::shared_ptr<SetControlCommand::Response> response);

  private:
    rclcpp::Logger logger_;
    const Callbacks callbacks_;
    rclcpp::Service<SetControlCommand>::SharedPtr service_;
};

}  // namespace plem::drive::ros
