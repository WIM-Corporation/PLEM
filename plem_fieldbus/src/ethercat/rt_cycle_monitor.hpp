// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <plem/core/monitoring.hpp>
#include <plem/core/monitoring/rt_sample.hpp>

#include "ethercat/internal_visibility.hpp"
#include "ethercat/joint_data_plane.hpp"
#include "ethercat/rt_event_emitter.hpp"
#include "plem/fieldbus/ethercat/servo/device.hpp"

namespace plem::fieldbus::ethercat {

// Package-private RT cycle monitor (deliberately not an installed header):
// the projection from one bus cycle onto the two monitoring streams: the
// per-cycle RtSample and the derived event stream. Keeping the projection
// here is what lets the RT loop read as a sequence of cycle steps instead of
// spelling out the sample's wire layout inline.
//
// Lifetime is run-scoped: the sample carry must restart at zero on a
// stop()/start(): the previous-wake stamp seeds the jitter figure,
// so a carried-over stamp reports the whole downtime as one cycle's jitter.
// RunScopedLoopCarryResetsOnRestart pins that jitter behavior; the emitter's
// own edge state follows the same rule by construction.
template <size_t DOF>
class PLEM_FIELDBUS_INTERNAL RtCycleMonitor {
    static_assert(DOF <= core::kMaxBusAxes, "the RT observation record holds kMaxBusAxes axes");

  public:
    /// The cycle facts that are not joint data: what the fieldbus reported,
    /// and the two per-cycle observability counters the loop computes.
    struct CycleObservation {
        int64_t wake_ns{0};  ///< this cycle's timer wake (CLOCK_MONOTONIC)
        /// Where the loop stopped the cycle clock. Excludes this projection
        /// itself, so loop_exec_us never covers monitoring cost.
        int64_t exec_end_ns{0};
        uint16_t wkc{0};                 ///< working counter reported by the domain
        bool wkc_ok{false};              ///< working counter matched expectation
        uint8_t wc_state{0};             ///< IgH wc_state (0=ZERO, 1=INCOMPLETE, 2=COMPLETE)
        uint32_t wkc_mismatch_count{0};  ///< watchdog's running consecutive-mismatch count
        uint8_t command_zoh{0};          ///< 1 when the command read did not land this cycle
        uint32_t dc_sync_monitor_ns{0};  ///< DC drift sample, 0 while DC is disabled
    };

    /// @param cycle_time_us  the loop's configured period
    explicit RtCycleMonitor(int cycle_time_us) : cycle_period_us_(static_cast<float>(cycle_time_us)) {
        sample_.axis_count = static_cast<uint8_t>(DOF);
    }

    /// Project this cycle onto both monitoring streams. Call once per cycle,
    /// after send(). The exec figure comes from the loop's own stop of the
    /// cycle clock (CycleObservation::exec_end_ns), not from this call.
    void emit(const CycleObservation& cycle, const std::vector<std::shared_ptr<DeviceServo>>& servos,
              const JointDataPlane<DOF>& joints) {
        constexpr size_t joint_count = DOF;
        const auto& actuals = joints.actuals();
        const auto& command = joints.command();
        const auto& target_values = joints.target_values();

        // Timing (sequence auto-filled by emit())
        sample_.monotonic_ns = static_cast<uint64_t>(cycle.wake_ns);
        sample_.loop_exec_us = static_cast<float>(cycle.exec_end_ns - cycle.wake_ns) / 1000.0f;
        sample_.loop_period_us = cycle_period_us_;
        if (prev_wake_ns_ > 0) {
            sample_.loop_jitter_us = static_cast<float>(cycle.wake_ns - prev_wake_ns_) / 1000.0f - cycle_period_us_;
        } else {
            sample_.loop_jitter_us = 0.0f;
        }
        sample_.deadline_miss = (sample_.loop_exec_us > cycle_period_us_) ? 1 : 0;

        // Joint data
        for (size_t i = 0; i < joint_count; ++i) {
            sample_.actual_position[i] = actuals[i].position_actual_value;
            sample_.actual_velocity[i] = actuals[i].velocity_actual_value;
            sample_.actual_torque[i] = static_cast<float>(actuals[i].torque_actual_value);
            // The command slot is in the axis's mode unit: a CSP axis's value
            // is its commanded position, a CSV axis's its commanded velocity.
            // Every other slot keeps the motion-API target conduit
            // (joint_target, a higher-layer host's; the drive layer never writes it).
            const OperationMode mode = servos[i]->get_modes_of_operation();
            sample_.command_position[i] = mode == OperationMode::CYCLIC_SYNC_POSITION
                                              ? static_cast<float>(command[i])
                                              : static_cast<float>(target_values[i].target_position);
            sample_.command_velocity[i] = mode == OperationMode::CYCLIC_SYNC_VELOCITY
                                              ? static_cast<float>(command[i])
                                              : static_cast<float>(target_values[i].target_velocity);
            // Only a CST axis's value is a torque. NaN = "not a torque" (a CSP
            // rad would read as Nm).
            sample_.command_torque[i] = mode == OperationMode::CYCLIC_SYNC_TORQUE
                                            ? static_cast<float>(command[i])
                                            : std::numeric_limits<float>::quiet_NaN();
            sample_.applied_command[i] = static_cast<float>(servos[i]->applied_command());
            sample_.statusword[i] = servos[i]->get_statusword();
            // 0x6061 display: what the drive reports it is in, not what plem
            // commands (get_modes_of_operation() is the declared mode, which
            // would hide NO_MODE and rejected-switch states).
            sample_.modes_of_operation_display[i] = servos[i]->get_modes_of_operation_display();
        }

        // Fieldbus status
        sample_.wkc = cycle.wkc;
        sample_.wkc_mismatch = cycle.wkc_ok ? 0 : 1;
        sample_.wc_state = cycle.wc_state;
        sample_.command_zoh = cycle.command_zoh;                // unconditional: the sample struct is reused
        sample_.dc_sync_monitor_ns = cycle.dc_sync_monitor_ns;  // unconditional; 0 while DC disabled

        // Push to queue (lock-free, RT-safe)
        PLEM_EMIT(sample_);

        // Event derivation reads the same sample object, after the push;
        // the order is the contract, not the queue handoff (PLEM_EMIT copies).
        std::array<ServoStatus, core::kMaxBusAxes> servo_status{};
        for (size_t i = 0; i < joint_count; ++i) {
            servo_status[i] = {
                .faulted = servos[i]->is_faulted(),
                .statusword = servos[i]->get_statusword(),
                .error_code = servos[i]->error_code(),
            };
        }
        emitter_.process(sample_, servo_status, static_cast<uint64_t>(cycle.wake_ns), cycle.wkc_mismatch_count);

        prev_wake_ns_ = cycle.wake_ns;
    }

  private:
    RtEventEmitter emitter_;
    float cycle_period_us_;

    // Reused across cycles by design: slots past joint_count keep the zeros
    // this was constructed with rather than being rewritten every cycle.
    plem::core::monitoring::RtSample sample_{};
    int64_t prev_wake_ns_ = 0;
};

}  // namespace plem::fieldbus::ethercat
