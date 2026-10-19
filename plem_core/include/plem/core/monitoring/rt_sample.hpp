// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <array>
#include <cstdint>
#include <type_traits>

#include "plem/core/constants.hpp"

namespace plem::core::monitoring {

/**
 * @brief RT monitoring sample data structure
 *
 * This struct contains all data captured in a single RT loop iteration.
 * It is designed for use in a lock-free SPSC queue from the RT loop
 * to non-RT publishing threads.
 *
 * @note This type must be trivially copyable for RT-safe queue operations.
 * @note Arrays are fixed-size std::array, so the struct stays trivially copyable.
 * @realtime_safe No dynamic allocation, trivially copyable
 */
struct PLEM_CORE_PUBLIC RtSample {
    // Timing Information
    uint64_t monotonic_ns;  ///< Sample timestamp [ns] (CLOCK_MONOTONIC) at loop wake-up
                            ///< (emit() auto-fills if zero)
    uint64_t sequence;      ///< Sequence number for continuity check (starts at 0, +1 per sample;
                            ///< always set by emit())

    // Loop Performance (per EtherCAT cycle)
    //
    // Timing diagram:
    //   t_wake         t_end                    t_wake (next)
    //     |──────────────|                         |
    //     │ loop_exec_us │                         │
    //     │ (processing) │←── sleep waiting ──────→│
    //     │←────────────── actual_period ─────────→│
    //
    //   loop_jitter_us = actual_period - target_period (loop_period_us)
    //
    float loop_exec_us;     ///< Total processing time per RT cycle [us]
                            ///< (one bus-loop iteration: PDO receive → command hand-off → PDO send)
    float loop_period_us;   ///< Target cycle period [us] from config (constant, e.g. 1 kHz → 1000 us)
    float loop_jitter_us;   ///< Period deviation [us]: actual period - target period
                            ///< negative = woke early, positive = woke late, ideal = 0
    uint8_t deadline_miss;  ///< Deadline-miss flag
                            ///< 1: processing time > period, 0: normal
    uint8_t axis_count;     ///< Axes this sample carries (1..kMaxBusAxes): the joint arrays'
                            ///< first axis_count slots; the rest stay 0

    // Joint Data (first axis_count slots)
    // Actual values (Servo → Master): Read from EtherCAT TxPDO
    std::array<float, kMaxBusAxes> actual_position;  ///< Actual joint position [rad]
    std::array<float, kMaxBusAxes> actual_velocity;  ///< Actual joint velocity [rad/s]
    std::array<float, kMaxBusAxes> actual_torque;    ///< Actual joint torque [Nm]

    // Command values (Master → Servo): the host's command, read from the shared command slot
    std::array<float, kMaxBusAxes>
        command_position;  ///< Commanded joint position [rad]: a CSP axis's host command (drive layer:
                           ///< as received; the frame's word is applied_command);
                           ///< other axes: the target conduit's target_position (a higher-layer host;
                           ///< 0 on the drive layer, which never writes the conduit)
    std::array<float, kMaxBusAxes>
        command_velocity;  ///< Commanded joint velocity [rad/s]: a CSV axis's host command (drive
                           ///< layer); other axes: the target conduit's target_velocity (0 when
                           ///< upstream sends no velocity, and on the drive layer)
    std::array<float, kMaxBusAxes> command_torque;  ///< Commanded joint torque [Nm]: CST axes only. CSP·CSV axes
                                                    ///< (drive layer) are NaN: their command is not a torque. The value
                                                    ///< before the DeviceServo clamp (on a higher-layer host: after the
                                                    ///< host's effort clamp)
    std::array<float, kMaxBusAxes>
        applied_command;  ///< The target word actually carried in this cycle's frame, converted
                          ///< back to the axis's declared mode unit: CST Nm (0x6071) · CSP rad
                          ///< (0x607A) · CSV rad/s (0x60FF). Includes clamping, count rounding,
                          ///< flip and saturation. NaN when a CSP word does not resolve to joint
                          ///< coordinates (raw position mirror)

    // CiA 402 Drive Status
    // CiA 402: the CANopen standard protocol for servo drives
    std::array<uint16_t, kMaxBusAxes> statusword;                ///< Statusword (servo TxPDO)
                                                                 ///< state decoded with mask 0x006F
                                                                 ///< 0x0027=OPERATION_ENABLED, 0x0008=FAULT
    std::array<int8_t, kMaxBusAxes> modes_of_operation_display;  ///< Mode of operation: the drive-reported
                                                                 ///< 0x6061 display (not the 0x6060 command)
                                                                 ///< 0=NO_MODE, 8=CSP, 9=CSV, 10=CST

    // EtherCAT Fieldbus Status
    uint16_t wkc;          ///< Working Counter (slaves that exchanged PDOs successfully)
    uint8_t wkc_mismatch;  ///< WKC mismatch (1: expected != actual)
    /// IgH wc_state value: 0=EC_WC_ZERO (no slave responded),
    /// 1=EC_WC_INCOMPLETE (some responded), 2=EC_WC_COMPLETE (all responded).
    uint8_t wc_state;
    /// Command hand-off ZOH flag: 1 = this cycle's single command read found
    /// the writer mid-write, so the previous command was re-sent (zero-order
    /// hold); 0 = fresh command. Assigned every cycle.
    uint8_t command_zoh;
    /// DC drift-monitor raw per-cycle value [ns]:
    /// upper bound of the slave clock offset observed by the DC sync-monitor
    /// datagram queued last cycle. 0 while DC is disabled; UINT32_MAX = monitor
    /// unavailable this cycle (IgH error sentinel; window folds must skip
    /// sentinels rather than max() them, publishing UINT32_MAX only when the
    /// whole window is sentinel). Assigned every cycle.
    uint32_t dc_sync_monitor_ns;
};

// Compile-time verification that RtSample is suitable for RT-safe queue operations
static_assert(std::is_trivially_copyable_v<RtSample>,
              "RtSample must be trivially copyable for RT-safe queue operations");

// Verify size is reasonable (296 bytes with std::array and alignment)
static_assert(sizeof(RtSample) == 296, "RtSample size changed: recompute the RT queue memory");

}  // namespace plem::core::monitoring
