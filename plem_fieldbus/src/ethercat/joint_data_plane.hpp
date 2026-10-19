// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <plem/core/types/joint_state.hpp>
#include <plem/core/types/shared_joint_data.hpp>

#include "ethercat/internal_visibility.hpp"
#include "plem/fieldbus/ethercat/servo/device.hpp"

namespace plem::fieldbus::ethercat {

// Package-private joint data plane (deliberately not an installed header):
// the per-cycle joint data the RT loop exchanges with the host over the
// SharedJointData seqlock plane: actuals out, command and targets in.
//
// Scope is joint data only. The control command and the phase-lock epoch also
// ride shared memory but are not joint data, and stay with the loop that acts
// on them.
//
// Lifetime is run-scoped: the loop constructs one per run_loop() entry, so a
// stop()/start() restarts from zero buffers instead of replaying the dead
// run's last command. RunScopedLoopCarryResetsOnRestart pins the torque and
// target buffers; the remaining members are run-scoped by the same
// construction.
//
// The shared-data handle is a reference to the controller's own, not a copy,
// so every consumer in a cycle (these buffers and the phase-lock epoch)
// dereferences the same handle. The handle is non-null for the whole run (validate_preconditions
// refuses to start without it) and cannot change under a run:
// set_shared_data() refuses once start() has returned
// (EtherCATController::throw_if_running).
//
// Every buffer below is DOF-sized: EtherCATController::validate_preconditions()
// refuses to start unless the bus's servo_count() == DOF.
template <size_t DOF>
class PLEM_FIELDBUS_INTERNAL JointDataPlane {
  public:
    /// @param shared        the controller's shared-data handle (non-null);
    ///                       must outlive this plane (it is a member of the
    ///                       object running the loop)
    explicit JointDataPlane(const std::shared_ptr<plem::core::SharedJointData<DOF>>& shared) : shared_(shared) {}

    /// Snapshot the drives' actual position/velocity/torque, then publish the
    /// snapshot on the shared-data plane. A drive that is not yet
    /// initialized holds its previous sample rather than publishing a zero
    /// readers cannot tell from a real measurement.
    void publish_actuals(const std::vector<std::shared_ptr<DeviceServo>>& servos) {
        for (size_t i = 0; i < DOF; ++i) {
            if (servos[i]->is_initialized()) {
                actuals_[i].position_actual_value = servos[i]->get_actual_position().value();
                actuals_[i].velocity_actual_value = servos[i]->get_actual_velocity().value();
                actuals_[i].torque_actual_value = servos[i]->get_actual_torque().value();
            }
        }

        plem::core::JointActualValues<DOF> actual_values;
        for (size_t i = 0; i < DOF; ++i) {
            actual_values.states[i] = actuals_[i];
        }
        shared_->joint_actual.write(actual_values);
    }

    /// Latch this cycle's command (each joint in the unit of its mode of operation:
    /// torque for CST).
    /// @return true when the seqlock read did not land and the previous
    ///         cycle's command is held instead (zero-order hold).
    [[nodiscard]] bool read_command() {
        // try_read, not read: when the writer is a higher-layer host's control
        // thread on this core at a lower FIFO priority, the bus loop strictly
        // preempts it. If this cycle lands inside its write critical section the
        // writer cannot resume until we sleep, so spinning kMaxRetries (measured ~4.6 µs)
        // can never succeed; keep the previous command immediately (the same
        // ZOH an exhausted spin would give, without spending the spin).
        plem::core::JointCommand<DOF> latest;
        if (!shared_->command.try_read(latest)) {
            // Writer mid-write: previous command re-sent this cycle (zero-order
            // hold). Observability for the phase-lock acceptance gate:
            // after alignment this count must not grow.
            return true;
        }
        for (size_t i = 0; i < DOF; ++i) {
            command_[i] = latest.value[i];
            command_rate_[i] = latest.rate[i];
        }
        last_command_update_ns_ = latest.update_ns;
        return false;
    }

    /// Latch this cycle's commanded position/velocity. Monitoring-only:
    /// nothing downstream acts on these, which is why a failed read is
    /// silent rather than observable the way the torque ZOH is.
    void read_target_values() {
        // try_read, not read: the writers (the host's motion API)
        // are non-RT and can sit descheduled mid-write for milliseconds, so
        // the retry budget (kMaxRetries, measured ~4.6 µs) burns without buying a
        // result. Monitoring data: the previous cycle's copy is fine.
        plem::core::JointTargetValues<DOF> target_data;
        if (shared_->joint_target.try_read(target_data)) {
            for (size_t i = 0; i < DOF; ++i) {
                target_values_[i] = target_data.commands[i];
            }
        }
    }

    /// This cycle's actual-value snapshot (monitoring reads it after publish).
    [[nodiscard]] const std::array<plem::core::JointActual, DOF>& actuals() const noexcept {
        return actuals_;
    }

    /// This cycle's command (held from an earlier cycle on ZOH).
    [[nodiscard]] const std::array<double, DOF>& command() const noexcept {
        return command_;
    }

    /// This cycle's command rates (JointCommand::rate), held with the command.
    [[nodiscard]] const std::array<double, DOF>& command_rate() const noexcept {
        return command_rate_;
    }

    /// This cycle's commanded position/velocity (monitoring).
    [[nodiscard]] const std::array<plem::core::JointTarget, DOF>& target_values() const noexcept {
        return target_values_;
    }

    /// Writer stamp of the last command read that landed (any mode), 0 until
    /// the first one does.
    [[nodiscard]] int64_t last_command_update_ns() const noexcept {
        return last_command_update_ns_;
    }

  private:
    const std::shared_ptr<plem::core::SharedJointData<DOF>>& shared_;

    std::array<plem::core::JointActual, DOF> actuals_{};
    std::array<double, DOF> command_{};
    std::array<double, DOF> command_rate_{};
    std::array<plem::core::JointTarget, DOF> target_values_{};
    int64_t last_command_update_ns_ = 0;
};

}  // namespace plem::fieldbus::ethercat
