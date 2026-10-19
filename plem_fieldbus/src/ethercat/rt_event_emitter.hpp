// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>

#include <plem/core/constants.hpp>
#include <plem/core/monitoring.hpp>
#include <plem/core/monitoring/event.hpp>
#include <plem/core/monitoring/rt_sample.hpp>

#include "ethercat/internal_visibility.hpp"

namespace plem::fieldbus::ethercat {

// Package-private RT event emitter (deliberately not an installed header): the
// edge detection RtCycleMonitor runs on each cycle's RtSample to derive events.

/**
 * @brief Servo status snapshot for event detection
 *
 * Lightweight struct decoupling event monitoring from fieldbus details.
 */
struct PLEM_FIELDBUS_INTERNAL ServoStatus {
    bool faulted;            ///< Servo is in fault state
    uint16_t statusword;     ///< CiA 402 status word for diagnostics
    uint16_t error_code{0};  ///< CiA 402 error code (0x603F) captured in the same
                             ///< frame as the fault edge; nonzero only when the
                             ///< model declares the ErrorCode optional-PDO role
                             ///< (read as a PDO in the fault frame itself: a later
                             ///< SDO read can miss a transient value, for example
                             ///< after a successful fault reset). Carried in SERVO_FAULT's
                             ///< `value` field (error_code keeps the status word).
};

/**
 * @brief RT-safe event emitter with edge-triggered detection
 *
 * Detects state changes in RT samples and emits events to a lock-free queue.
 * Uses rising-edge detection to avoid event flooding.
 *
 * Detected conditions:
 * - Servo faults (per joint, rising edge)
 * - Deadline misses (rising edge with cooldown)
 * - WKC mismatches (rising edge with cooldown)
 *
 * @note RT-safe: emits with core::monitoring::emit (wait-free) into the calling
 *       thread's context, or the global default context when it has none.
 * @note Single-producer only - one emitter per RT thread.
 */
class PLEM_FIELDBUS_INTERNAL RtEventEmitter {
  public:
    RtEventEmitter() = default;

    /**
     * @brief Process RT sample and emit events for state changes
     *
     * Call this once per RT loop iteration after collecting sample data.
     *
     * @param sample Current RT loop sample
     * @param servos Current servo status array (size = core::kMaxBusAxes; slots past the bus's axes stay unfaulted)
     * @param now_ns Current monotonic time in nanoseconds
     * @param wkc_mismatch_count Watchdog's running consecutive WKC-mismatch count.
     *                           Carried in WKC_MISMATCH event's `value` field so a consumer
     *                           can show "(n / threshold)" from the event alone.
     *
     * @note RT-safe: no allocation, no blocking, bounded execution
     */
    void process(const core::monitoring::RtSample& sample, const std::array<ServoStatus, core::kMaxBusAxes>& servos,
                 uint64_t now_ns, uint32_t wkc_mismatch_count = 0) {
        // 1. Servo faults (rising edge per joint). error_code = statusword;
        //    value = the drive's CiA 402 error code from the same frame (0 unless
        //    the ErrorCode PDO role is declared). No other event carries that
        //    drive code.
        for (size_t i = 0; i < core::kMaxBusAxes; ++i) {
            const bool faulted = servos[i].faulted;
            if (faulted && !prev_faulted_[i]) {
                emit_event(core::monitoring::EventType::SERVO_FAULT, static_cast<uint8_t>(i),
                           core::monitoring::Severity::ERROR, servos[i].statusword,
                           static_cast<float>(servos[i].error_code), now_ns, sample.sequence);
            }
            prev_faulted_[i] = faulted;
        }

        // 2. Deadline miss (rising edge with cooldown)
        const bool miss = sample.deadline_miss > 0;
        if (miss && !prev_deadline_miss_) {
            if (now_ns - last_deadline_event_ns_ >= kDeadlineCooldownNs) {
                emit_event(core::monitoring::EventType::DEADLINE_MISS, core::monitoring::kJointIdNotApplicable,
                           core::monitoring::Severity::WARN, 0, sample.loop_exec_us, now_ns, sample.sequence);
                last_deadline_event_ns_ = now_ns;
            }
        }
        prev_deadline_miss_ = miss;

        // 3. WKC mismatch (rising edge with cooldown).
        //    Cooldown caps flapping-induced flooding (~10 events/s ceiling). The
        //    `wkc_event_emitted_once_` guard guarantees the very first WKC event
        //    is never swallowed even when now_ns < kWkcCooldownNs.
        //    error_code carries IgH wc_state (0=ZERO/1=INCOMPLETE/2=COMPLETE);
        //    value carries the watchdog's consecutive mismatch count (1..threshold).
        const bool wkc_err = sample.wkc_mismatch > 0;
        if (wkc_err && !prev_wkc_mismatch_) {
            const bool cooldown_passed = !wkc_event_emitted_once_ || (now_ns - last_wkc_event_ns_ >= kWkcCooldownNs);
            if (cooldown_passed) {
                emit_event(core::monitoring::EventType::WKC_MISMATCH, core::monitoring::kJointIdNotApplicable,
                           core::monitoring::Severity::ERROR, static_cast<uint16_t>(sample.wc_state),
                           static_cast<float>(wkc_mismatch_count), now_ns, sample.sequence);
                last_wkc_event_ns_ = now_ns;
                wkc_event_emitted_once_ = true;
            }
        }
        prev_wkc_mismatch_ = wkc_err;
    }

  private:
    static constexpr uint64_t kDeadlineCooldownNs = 100'000'000;  // 100ms
    static constexpr uint64_t kWkcCooldownNs = 100'000'000;       // 100ms

    void emit_event(core::monitoring::EventType type, uint8_t joint, core::monitoring::Severity sev,
                    uint16_t error_code, float value, uint64_t timestamp_ns, uint64_t ref_seq) {
        core::monitoring::Event e{};
        e.type = type;
        e.joint_id = joint;
        e.severity = static_cast<uint8_t>(sev);
        e.source_id = static_cast<uint8_t>(core::monitoring::SourceId::RT_LOOP);
        e.monotonic_ns = timestamp_ns;
        e.ref_sample_sequence = ref_seq;
        e.error_code = error_code;
        e.value = value;

        (void)core::monitoring::emit(e);
    }

    std::array<bool, core::kMaxBusAxes> prev_faulted_{};
    bool prev_deadline_miss_ = false;
    bool prev_wkc_mismatch_ = false;
    bool wkc_event_emitted_once_ = false;
    uint64_t last_deadline_event_ns_ = 0;
    uint64_t last_wkc_event_ns_ = 0;
};

}  // namespace plem::fieldbus::ethercat
