// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <cstddef>
#include <cstdint>

namespace plem::core {

/**
 * @brief One joint's target values (host → bus loop)
 *
 * Target position, velocity, and acceleration for one joint, in joint
 * coordinates (rad, rad/s, rad/s^2), not a PDO image: the bus loop encodes
 * the drive's target words from it.
 * Carried in-process in SharedJointData.
 *
 * @note Trivially copyable (carried in SharedJointData's seqlock slots).
 * @realtime_safe
 */
struct PLEM_CORE_PUBLIC JointTarget {
    double target_position;      ///< Target position in radians
    double target_velocity;      ///< Target velocity in rad/s
    double target_acceleration;  ///< Target acceleration in rad/s^2

    JointTarget() : target_position(0.0), target_velocity(0.0), target_acceleration(0.0) {}
};

/**
 * @brief Target values from the host motion API.
 *
 * Carried in SharedJointData::joint_target; a higher-layer host's control
 * thread consumes them and the bus loop reads them for monitoring.
 *
 * @tparam DOF Degrees of freedom (number of joints)
 * @note Trivially copyable (carried in SharedJointData's seqlock slots).
 * @realtime_safe No dynamic allocation
 */
template <std::size_t DOF>
struct JointTargetValues {
    JointTarget commands[DOF];

    /// Command freshness timestamp (CLOCK_MONOTONIC nanoseconds)
    /// Set by the writer on every write(). The drive layer does not read it.
    int64_t command_update_ns{0};
};

/**
 * @brief One joint's actual values (bus loop → host)
 *
 * Actual position, velocity, and torque for one joint, in joint coordinates
 * (rad, rad/s, Nm), not a PDO image: the bus loop decodes them from the
 * drive's actual-value words.
 * Carried in-process in SharedJointData (bus loop → host).
 *
 * @note Trivially copyable (carried in SharedJointData's seqlock slots).
 * @realtime_safe
 */
struct PLEM_CORE_PUBLIC JointActual {
    double position_actual_value;  ///< Actual position in radians
    double velocity_actual_value;  ///< Actual velocity in rad/s
    double torque_actual_value;    ///< Actual torque in Nm

    JointActual() : position_actual_value(0.0), velocity_actual_value(0.0), torque_actual_value(0.0) {}
};

/**
 * @brief Actual values from all joints (bus loop → host)
 *
 * Carried in-process in SharedJointData::joint_actual.
 *
 * @tparam DOF Degrees of freedom (number of joints)
 * @note Trivially copyable (carried in SharedJointData's seqlock slots).
 * @realtime_safe No dynamic allocation
 */
template <std::size_t DOF>
struct JointActualValues {
    JointActual states[DOF];
};

/**
 * @brief The per-joint command for all joints: the mode-general command slot
 *
 * `value[i]` is in the unit of joint i's mode of operation, fixed by its
 * configuration: Nm for CST. One stamp covers
 * the whole array.
 *
 * @tparam DOF Degrees of freedom (number of joints)
 * @note Trivially copyable (carried in SharedJointData's seqlock slots).
 * @realtime_safe No dynamic allocation
 */
template <std::size_t DOF>
struct JointCommand {
    double value[DOF];
    /// How fast each value changes, in its unit per second (0 = holds). A
    /// drive host's position axis (CSP) carries its planned speed here: the
    /// bus loop advances a sample it reads late by rate × its age.
    double rate[DOF];

    /// CLOCK_MONOTONIC stamp of the writer's cycle; 0 = never stamped (bring-up).
    /// A higher-layer host's control thread stamps every torque branch once per
    /// cycle; DriveHost copies the caller's sampled_ns into every accepted
    /// write. The EtherCAT loop compares its age with the configured
    /// command-staleness budget (kDefaultCommandStaleTimeoutNs by default). A
    /// stale stamp means no recent command reached the slot: the writer hung,
    /// starved or exited, or DriveHost rejected every write.
    int64_t update_ns{0};

    JointCommand() {
        for (std::size_t i = 0; i < DOF; ++i) {
            value[i] = 0.0;
            rate[i] = 0.0;
        }
    }
};

}  // namespace plem::core
