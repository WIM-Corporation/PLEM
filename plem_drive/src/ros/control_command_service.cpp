// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/drive/ros/control_command_service.hpp"

#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace plem::drive::ros {
namespace {

template <typename Callback>
void require(const Callback& cb, const char* name) {
    if (!cb) {
        throw std::invalid_argument(std::string("ControlCommandService: callback '") + name + "' is required");
    }
}

}  // namespace

ControlCommandService::ControlCommandService(rclcpp::Node& node, Callbacks callbacks, const std::string& service_name)
    : logger_(node.get_logger()), callbacks_(std::move(callbacks)) {
    using namespace std::placeholders;

    require(callbacks_.protective_stop, "protective_stop");
    require(callbacks_.release_brake, "release_brake");
    require(callbacks_.free_drive, "free_drive");
    require(callbacks_.reset_error, "reset_error");

    service_ = node.create_service<SetControlCommand>(service_name,
                                                      std::bind(&ControlCommandService::handle_request, this, _1, _2));

    RCLCPP_INFO(logger_, "ControlCommandService initialized (service: %s)", service_->get_service_name());
}

void ControlCommandService::handle_request(const std::shared_ptr<SetControlCommand::Request> request,
                                           std::shared_ptr<SetControlCommand::Response> response) {
    // Fast path: no flags set -> idempotent no-op.
    if (!request->reset_error && !request->protective_stop && !request->release_brake && !request->free_drive) {
        response->success = true;
        response->message = "No control command flags set (no-op)";
        return;
    }

    bool overall_success = true;
    std::ostringstream msg;
    bool first = true;

    auto append = [&](const std::string& part) {
        if (!first) {
            msg << "; ";
        }
        msg << part;
        first = false;
    };

    auto run = [&](bool requested, const char* name, const CommandCallback& callback, const char* accepted = "ok") {
        if (!requested) {
            return;
        }
        const auto result = callback();
        if (result) {
            append(std::string(name) + ": " + accepted);
            RCLCPP_INFO(logger_, "%s accepted", name);
        } else {
            overall_success = false;
            append(std::string(name) + ": " + result.message());
            RCLCPP_ERROR(logger_, "%s failed: %s", name, result.message().c_str());
        }
    };

    try {
        // In the fixed order below the release would undo the stop, so the
        // stop runs alone.
        if (request->protective_stop && (request->release_brake || request->free_drive)) {
            std::string ignored;
            for (const auto& [set, name] : {std::pair<bool, const char*>{request->release_brake, "release_brake"},
                                            std::pair<bool, const char*>{request->free_drive, "free_drive"},
                                            std::pair<bool, const char*>{request->reset_error, "reset_error"}}) {
                if (set) {
                    ignored += (ignored.empty() ? "" : ", ") + std::string(name);
                }
            }
            std::string stop_outcome;
            try {
                const auto stop = callbacks_.protective_stop();
                stop_outcome = stop ? "protective_stop: " + callbacks_.protective_stop_accepted
                                    : "protective_stop failed: " + stop.message();
            } catch (const std::exception& e) {
                stop_outcome = std::string("protective_stop threw: ") + e.what();
            }
            response->success = false;
            response->message = stop_outcome + "; " + ignored +
                                " ignored (the stop runs alone) — after the stop, send the command you want in its "
                                "own request";
            RCLCPP_WARN(logger_, "%s", response->message.c_str());
            return;
        }

        // reset_error runs first so that if the drives do clear, a command
        // flag in this same request can succeed; the callback returns
        // nothing and confirms nothing, so ordering is all this buys.
        if (request->reset_error) {
            callbacks_.reset_error();
            append("reset_error: requested");
            RCLCPP_INFO(logger_, "reset_error invoked");
        }

        run(request->protective_stop, "protective_stop", callbacks_.protective_stop,
            callbacks_.protective_stop_accepted.c_str());
        run(request->release_brake, "release_brake", callbacks_.release_brake,
            callbacks_.release_brake_accepted.c_str());
        run(request->free_drive, "free_drive", callbacks_.free_drive);
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Exception while handling control command: ") + e.what();
        RCLCPP_ERROR(logger_, "ControlCommandService exception: %s", e.what());
        return;
    } catch (...) {
        response->success = false;
        response->message = "Unknown exception while handling control command";
        RCLCPP_ERROR(logger_, "ControlCommandService unknown exception");
        return;
    }

    response->success = overall_success;
    response->message = msg.str();
}

}  // namespace plem::drive::ros
