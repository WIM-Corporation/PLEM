// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <cstdint>
#include <type_traits>

namespace plem::core::monitoring {

/// Joint ID value indicating the event is not joint-specific
constexpr uint8_t kJointIdNotApplicable = 0xFF;

/**
 * @brief Event type classification
 *
 * Events are grouped by category:
 * - 0x40-0x4F: Safety events
 * - 0x50-0x6F: Error events (the band continues past 0x5F)
 *
 * A number, once assigned, is never reused. No core event type takes these numbers.
 * Never reused: 0x20, 0x21, 0x30, 0x44, 0x48, 0x49, 0x51, 0x5A, 0x5B, 0x5D,
 * 0x5E, 0xF0; left to event types defined above the core: 0x45-0x47,
 * 0x4A-0x4D, 0x55-0x57.
 */
enum class EventType : uint8_t {
    // Safety events (0x40-0x4F)
    /// Protective stop triggered (error_code 0), or a requested clear refused
    /// because a cause still stands (error_code 1).
    PROTECTIVE_STOP = 0x40,
    PROTECTIVE_STOP_CLEAR = 0x41,  ///< Protective stop cleared
    BRAKE_ENGAGE = 0x42,           ///< Brake engaged
    BRAKE_RELEASE = 0x43,          ///< Brake released
    /// A drive-layer axis set the INTERNAL protective stop: joint = the axis,
    /// error_code = LimitStopReason, value = the offending value in the
    /// reason's unit (NaN when there is none). One per violating axis, where
    /// the stop is newly set; not paired with the brake step's PROTECTIVE_STOP
    /// (a command violation precedes it by one cycle).
    LIMIT_STOP = 0x4E,
    /// A drive-layer stop (EtherCATController::request_ramp_stop) was not confirmed
    /// by its planned deceleration plus the settle margin: Quick stop is
    /// commanded to every enabled drive. error_code = 0, value = seconds since
    /// the request.
    RAMP_STOP_TIMEOUT = 0x4F,

    // Error events (0x50-0x6F)
    /// Servo drive fault detected (rising edge, joint = the axis):
    /// error_code = the axis's statusword; value = the drive's error code
    /// from the same frame, read through the configured error-code PDO role
    /// (normally CiA 402 0x603F, at the axis's channel address); 0 when no
    /// error-code PDO is mapped.
    SERVO_FAULT = 0x50,
    /// Working counter mismatch: error_code = IgH wc_state (0 ZERO,
    /// 1 INCOMPLETE, 2 COMPLETE), value = consecutive mismatch count.
    WKC_MISMATCH = 0x52,
    DEADLINE_MISS = 0x53,  ///< RT loop deadline exceeded (error_code 0, value = loop exec time in µs)
    /// Command stream stale beyond budget (robot: torque leg; drive: any
    /// mode): error_code 0, value = the stamp's age in seconds.
    COMMAND_STALE = 0x54,
    /// Mid-brake-sequence fault-reset request accepted (models declaring an
    /// automatic reset policy; value = accepted-request ordinal within the
    /// episode, error_code = the axis's statusword). Acceptance is not
    /// publication: the request can still be held a cycle and then discarded.
    /// SERVO_FAULT_RESET_RESULT reports what reached domain memory.
    SERVO_FAULT_RESET = 0x58,
    /// Automatic fault-reset allowance spent: the sequence stops asking
    /// (value = the model's declared cap, FaultResetPacing::max_attempts;
    /// error_code = the axis's statusword). Claims only that: the axis is
    /// left in whatever state it reports, which need not be a settled FAULT
    /// (the cap can be crossed while the axis reads FAULT_REACTION_ACTIVE or
    /// an unrecognized word). Neither event is evidence of a drive reset.
    SERVO_FAULT_RESET_EXHAUSTED = 0x59,
    /// Brake power sequence made no step progress for the stall window
    /// (error_code = statusword, value = unreached target ServoState).
    /// Claims only the absence of progress, not its cause.
    BRAKE_SEQUENCE_STALLED = 0x5C,
    /// An axis is not accepted in its declared CiA 402 mode of operation
    /// (error_code = the axis's statusword at the observation, value = its
    /// 0x6061 modes-of-operation display, joint = the axis). Emitted per
    /// axis when a release request is refused at the readiness gate (the
    /// request's brake-group closure is checked; every failing axis reports;
    /// no frame yet, an unrecognized statusword or a display mismatch all
    /// count) and once per episode when an enabled axis leaves the declared
    /// mode (the in-service trip that requests the protective stop). Both fields are
    /// raw observations of that cycle; they encode no cause.
    MODES_OF_OPERATION_MISMATCH = 0x5F,

    /// What a fault-reset request did at its publication point, per axis
    /// (error_code = the axis's statusword that frame, value = the
    /// FaultResetPublication outcome). Emitted once per terminal outcome:
    /// bit 7 written, or dropped because the axis was recognized but not in
    /// FAULT, or dropped because the statusword was not decodable. A drop is
    /// final; the operator re-requests. Claims nothing about the drive.
    ///
    /// Delivery is best effort, like every RT event: the queue may reject an
    /// emission under pressure, and the device's stored outcome is replaced by
    /// the next write. A missing report is not evidence that nothing happened.
    SERVO_FAULT_RESET_RESULT = 0x60,

    /// A released axis (one whose last brake sequence was a release and has
    /// completed) is seen, on a recognized statusword, outside
    /// OPERATION_ENABLED while its brake group is still reported released
    /// (error_code = the axis's statusword at the observation, value = 0,
    /// joint = the axis). Emitted once per in-service trip episode, on the
    /// cycle the loss is detected; the same episode asserts the EXTERNAL
    /// protective stop and requests the forced engage of every brake on the next brake
    /// step. A detection-and-stop-request diagnostic: it claims nothing about the
    /// cause, the drive's 0x603F, whether the engage completes, the physical
    /// brake or the recovery.
    OPERATION_ENABLED_LOST = 0x61,

    /// A release request was refused because the axis's slave is not
    /// operational: not in OP with the configuration the master gave it, per
    /// the master's slave-configuration state sampled for that request
    /// (error_code = that AL state: 0 = offline, or the state could not be
    /// read; 1 INIT, 2 PREOP, 4 SAFEOP, 8 OP with a reconfiguration pending;
    /// value = the slave's ring position; joint = the axis). Emitted per
    /// refused axis in place of MODES_OF_OPERATION_MISMATCH. The causes it does
    /// not tell apart: right after start the slave may still be on its way to
    /// OP (wait and request again); otherwise the kernel log
    /// (`sudo dmesg | grep -i ethercat`) holds the AL status code the slave
    /// reported, or a startup SDO it refused (startup_sdo, or
    /// dc.write_cycle_time's 0x1C32:02).
    SLAVE_NOT_OPERATIONAL = 0x62,
};

/**
 * @brief LIMIT_STOP error_code: why a drive-layer axis was stopped.
 *
 * A wire and public-header contract: a number, once shipped, is never reused.
 */
enum class LimitStopReason : uint16_t {
    NONE = 0,
    POSITION = 1,      ///< measured position outside limits.position (value: rad)
    VELOCITY = 2,      ///< measured |velocity| over limits.velocity_rad_s (value: rad/s)
    UNDECODABLE = 3,   ///< the 0x6064 word does not decode into joint counts (value: NaN)
    CSP_RANGE = 4,     ///< csp target outside limits.position (value: target rad)
    CSP_STEP = 5,      ///< csp target over one cycle's travel (value: target − reference, rad)
    CSV_VELOCITY = 6,  ///< csv |target| over limits.velocity_rad_s (value: rad/s)
    UNENCODABLE = 7,   ///< the command does not encode into the drive's count (value: the command)
};

/**
 * @brief Event severity levels
 */
enum class Severity : uint8_t {
    INFO = 0,      ///< Informational event
    WARN = 1,      ///< Warning condition
    ERROR = 2,     ///< Error condition
    CRITICAL = 3,  ///< Critical error requiring immediate attention
};

/**
 * @brief Event source identifiers
 *
 * Which component generated the event.
 */
enum class SourceId : uint8_t {
    RT_LOOP = 1,  ///< Real-time control loop
    // 2, 3: reserved, never reuse.
};

/**
 * @brief RT monitoring event data structure
 *
 * One discrete event from the RT loop, passed to non-RT logging/monitoring
 * threads over a lock-free SPSC queue. Exactly one 64-byte cache line
 * (alignas(64)).
 *
 * Memory layout (64 bytes total):
 *   Offset  0: type, source_id, severity, joint_id (4 bytes)
 *   Offset  4: padding_ (4 bytes, aligns uint64_t)
 *   Offset  8: monotonic_ns (8 bytes)
 *   Offset 16: sequence (8 bytes)
 *   Offset 24: ref_sample_sequence (8 bytes)
 *   Offset 32: error_code (2 bytes)
 *   Offset 36: value (4 bytes)
 *   Offset 40: tail padding up to 64 bytes (alignas(64))
 *
 * @note This type must be trivially copyable for RT-safe queue operations.
 * @realtime_safe No dynamic allocation, trivially copyable
 */
struct PLEM_CORE_PUBLIC alignas(64) Event {
    // Byte 0-3: Classification fields
    EventType type;
    uint8_t source_id;  ///< Source identifier (see SourceId enum)
    uint8_t severity;   ///< Severity level (see Severity enum)
    uint8_t joint_id;   ///< Axis index (0-7, below kMaxBusAxes), or kJointIdNotApplicable

    // Byte 4-7: Explicit padding for uint64_t alignment
    uint8_t padding_[4];  ///< Reserved for alignment (do not use)

    // Byte 8-31: Timing fields
    uint64_t monotonic_ns;         ///< Event timestamp (CLOCK_MONOTONIC nanoseconds)
    uint64_t sequence;             ///< Per-context emit order (set by emit(), +1 per event)
    uint64_t ref_sample_sequence;  ///< Associated RT sample sequence (0 = N/A)

    // Byte 32-33: Data field
    /// Per event type (see EventType): a statusword, an AL state, a wc_state,
    /// a LimitStopReason, 1 for a refused protective stop clear, else 0. Not the
    /// drive's error code; SERVO_FAULT carries that in `value`.
    uint16_t error_code;

    // Byte 36-39: Numeric value
    float value;  ///< Meaning per event type (see EventType)

    // Factory Functions (RT-safe, constexpr, noexcept)

    /// Create a safety-category event (protective stop, brake, fault reset)
    [[nodiscard]] static constexpr Event safety(EventType event_type, Severity sev = Severity::WARN,
                                                uint8_t joint = kJointIdNotApplicable) noexcept {
        Event e{};
        e.type = event_type;
        e.source_id = static_cast<uint8_t>(SourceId::RT_LOOP);
        e.severity = static_cast<uint8_t>(sev);
        e.joint_id = joint;
        return e;
    }

    /// Create an error event with error code
    [[nodiscard]] static constexpr Event error(EventType event_type, uint16_t err_code,
                                               uint8_t joint = kJointIdNotApplicable) noexcept {
        Event e{};
        e.type = event_type;
        e.source_id = static_cast<uint8_t>(SourceId::RT_LOOP);
        e.severity = static_cast<uint8_t>(Severity::ERROR);
        e.joint_id = joint;
        e.error_code = err_code;
        return e;
    }

    /// Create a safety event with numeric value
    [[nodiscard]] static constexpr Event safety_with_value(EventType event_type, float val,
                                                           Severity sev = Severity::WARN) noexcept {
        Event e = safety(event_type, sev);
        e.value = val;
        return e;
    }

    // Domain-specific Convenience Factories

    [[nodiscard]] static constexpr Event protective_stop(Severity sev = Severity::WARN) noexcept {
        return safety(EventType::PROTECTIVE_STOP, sev);
    }

    [[nodiscard]] static constexpr Event protective_stop_clear() noexcept {
        return safety(EventType::PROTECTIVE_STOP_CLEAR, Severity::INFO);
    }

    [[nodiscard]] static constexpr Event brake_engage() noexcept {
        return safety(EventType::BRAKE_ENGAGE, Severity::INFO);
    }

    [[nodiscard]] static constexpr Event brake_release() noexcept {
        return safety(EventType::BRAKE_RELEASE, Severity::INFO);
    }

    /// A drive-layer axis's protective-stop cause (LIMIT_STOP).
    [[nodiscard]] static constexpr Event limit_stop(uint8_t axis, LimitStopReason reason, float value) noexcept {
        Event e = error(EventType::LIMIT_STOP, static_cast<uint16_t>(reason), axis);
        e.value = value;
        return e;
    }

    /// COMMAND_STALE: no fresh command within the budget. On a higher-layer host
    /// this is the control thread's torque command to the bus loop (a hung or
    /// starved writer: the last torque is reused and limit monitoring is lost);
    /// on the drive layer it is the host's command in whatever mode each axis
    /// runs. `value` is the stamp's age in seconds at the trip.
    [[nodiscard]] static constexpr Event command_stale(float age_seconds) noexcept {
        return safety_with_value(EventType::COMMAND_STALE, age_seconds, Severity::ERROR);
    }
};

// Compile-time verification that Event is suitable for RT-safe queue operations
static_assert(std::is_trivially_copyable_v<Event>, "Event must be trivially copyable for RT-safe queue operations");

// Verify Event is exactly cache-line sized
static_assert(sizeof(Event) == 64, "Event must be cache-line sized (64 bytes)");

}  // namespace plem::core::monitoring
