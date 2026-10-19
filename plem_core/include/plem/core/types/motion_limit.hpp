// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <algorithm>

namespace plem::core {

/**
 * @brief Motion limits for a single joint
 *
 * Defines position and velocity constraints for joint safety checking.
 *
 * @note This type is trivially copyable.
 */
struct PLEM_CORE_PUBLIC MotionLimit {
    double lower_limit;     ///< Lower position limit (radians)
    double upper_limit;     ///< Upper position limit (radians)
    double velocity_limit;  ///< Maximum velocity (rad/s)
    double effort_limit;    ///< Maximum joint torque magnitude (Nm): URDF <limit effort>, per-joint hardware spec

    /**
     * @brief Check if joint state is within limits
     * @param joint_angle Current joint angle (radians)
     * @param joint_velocity Current joint velocity (rad/s)
     * @return true if within limits
     */
    [[nodiscard]] bool check_limit(double joint_angle, double joint_velocity) const {
        return joint_angle >= lower_limit && joint_angle <= upper_limit && joint_velocity <= velocity_limit &&
               joint_velocity >= -velocity_limit;
    }

    /// @brief Clamp a target position into [lower_limit, upper_limit].
    [[nodiscard]] constexpr double clamp_position(double position) const noexcept {
        return std::clamp(position, lower_limit, upper_limit);
    }

    /// @brief Clamp a target velocity into the symmetric band [-velocity_limit, +velocity_limit].
    [[nodiscard]] constexpr double clamp_velocity(double velocity) const noexcept {
        return std::clamp(velocity, -velocity_limit, velocity_limit);
    }

    /// @brief Clamp a torque into the symmetric band [-effort_limit, +effort_limit].
    [[nodiscard]] constexpr double clamp_torque(double torque) const noexcept {
        return std::clamp(torque, -effort_limit, effort_limit);
    }

    /// @brief Validate motion limit parameters
    /// @return true if limits are physically valid (lower < upper, velocity > 0, effort > 0)
    [[nodiscard]] bool is_valid() const noexcept {
        return lower_limit < upper_limit && velocity_limit > 0.0 && effort_limit > 0.0;
    }
};

}  // namespace plem::core
