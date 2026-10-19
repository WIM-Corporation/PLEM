// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>

#include <plem_rt_msgs/msg/rt_event.hpp>
#include <rclcpp/time.hpp>

#include <plem/core/monitoring/event.hpp>

namespace plem::drive::ros {

namespace rt_event_pin {

using Msg = plem_rt_msgs::msg::RtEvent;
using plem::core::monitoring::EventType;
using plem::core::monitoring::LimitStopReason;

// RtEvent.msg names every core code. These switches pin each enumerator to the
// same-named message constant: a value that differs fails the static_asserts
// below, and an enumerator with no case here fails -Wswitch. A value outside
// the enum (one a higher layer defines on a reserved slot) is not the
// message's to name.
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wswitch"
constexpr bool matches(EventType t) {
    const auto v = static_cast<uint8_t>(t);
    switch (t) {
        case EventType::PROTECTIVE_STOP:
            return v == Msg::PROTECTIVE_STOP;
        case EventType::PROTECTIVE_STOP_CLEAR:
            return v == Msg::PROTECTIVE_STOP_CLEAR;
        case EventType::BRAKE_ENGAGE:
            return v == Msg::BRAKE_ENGAGE;
        case EventType::BRAKE_RELEASE:
            return v == Msg::BRAKE_RELEASE;
        case EventType::LIMIT_STOP:
            return v == Msg::LIMIT_STOP;
        case EventType::RAMP_STOP_TIMEOUT:
            return v == Msg::RAMP_STOP_TIMEOUT;
        case EventType::SERVO_FAULT:
            return v == Msg::SERVO_FAULT;
        case EventType::WKC_MISMATCH:
            return v == Msg::WKC_MISMATCH;
        case EventType::DEADLINE_MISS:
            return v == Msg::DEADLINE_MISS;
        case EventType::COMMAND_STALE:
            return v == Msg::COMMAND_STALE;
        case EventType::SERVO_FAULT_RESET:
            return v == Msg::SERVO_FAULT_RESET;
        case EventType::SERVO_FAULT_RESET_EXHAUSTED:
            return v == Msg::SERVO_FAULT_RESET_EXHAUSTED;
        case EventType::BRAKE_SEQUENCE_STALLED:
            return v == Msg::BRAKE_SEQUENCE_STALLED;
        case EventType::MODES_OF_OPERATION_MISMATCH:
            return v == Msg::MODES_OF_OPERATION_MISMATCH;
        case EventType::SERVO_FAULT_RESET_RESULT:
            return v == Msg::SERVO_FAULT_RESET_RESULT;
        case EventType::OPERATION_ENABLED_LOST:
            return v == Msg::OPERATION_ENABLED_LOST;
        case EventType::SLAVE_NOT_OPERATIONAL:
            return v == Msg::SLAVE_NOT_OPERATIONAL;
    }
    return true;
}

constexpr bool matches(LimitStopReason r) {
    const auto v = static_cast<uint16_t>(r);
    switch (r) {
        case LimitStopReason::NONE:
            return v == Msg::LIMIT_STOP_REASON_NONE;
        case LimitStopReason::POSITION:
            return v == Msg::LIMIT_STOP_REASON_POSITION;
        case LimitStopReason::VELOCITY:
            return v == Msg::LIMIT_STOP_REASON_VELOCITY;
        case LimitStopReason::UNDECODABLE:
            return v == Msg::LIMIT_STOP_REASON_UNDECODABLE;
        case LimitStopReason::CSP_RANGE:
            return v == Msg::LIMIT_STOP_REASON_CSP_RANGE;
        case LimitStopReason::CSP_STEP:
            return v == Msg::LIMIT_STOP_REASON_CSP_STEP;
        case LimitStopReason::CSV_VELOCITY:
            return v == Msg::LIMIT_STOP_REASON_CSV_VELOCITY;
        case LimitStopReason::UNENCODABLE:
            return v == Msg::LIMIT_STOP_REASON_UNENCODABLE;
    }
    return true;
}
#pragma GCC diagnostic pop

template <typename E>
constexpr bool all_match() {
    for (int i = 0; i < 256; ++i) {
        if (!matches(static_cast<E>(i))) {
            return false;
        }
    }
    return true;
}

static_assert(all_match<EventType>(), "an EventType value differs from its RtEvent.msg constant");
// The loop visits 0-255; reasons are numbered in order, so the last one bounds them.
static_assert(static_cast<uint16_t>(LimitStopReason::UNENCODABLE) < 256,
              "a LimitStopReason past 255: widen all_match and move this check to the new last reason");
static_assert(all_match<LimitStopReason>(), "a LimitStopReason value differs from its RtEvent.msg constant");

using plem::core::monitoring::Severity;
using plem::core::monitoring::SourceId;
static_assert(static_cast<uint8_t>(Severity::INFO) == Msg::SEVERITY_INFO &&
              static_cast<uint8_t>(Severity::WARN) == Msg::SEVERITY_WARN &&
              static_cast<uint8_t>(Severity::ERROR) == Msg::SEVERITY_ERROR &&
              static_cast<uint8_t>(Severity::CRITICAL) == Msg::SEVERITY_CRITICAL);
static_assert(static_cast<uint8_t>(SourceId::RT_LOOP) == Msg::SOURCE_RT_LOOP);
static_assert(plem::core::monitoring::kJointIdNotApplicable == Msg::JOINT_ID_NOT_APPLICABLE);

}  // namespace rt_event_pin

/// Event → RtEvent, verbatim field carry. Lives here, not with a higher-layer host's
/// publisher binding, because the drive bus publishes the same events
/// and cannot depend on a higher-layer host's node package (that package depends on this one).
inline plem_rt_msgs::msg::RtEvent to_rt_event(const plem::core::monitoring::Event& event, const rclcpp::Time& stamp) {
    plem_rt_msgs::msg::RtEvent msg;

    // Header
    msg.header.stamp = stamp;

    // Timing
    msg.monotonic_ns = event.monotonic_ns;
    msg.event_sequence = event.sequence;
    msg.ref_sample_sequence = event.ref_sample_sequence;

    // Classification
    msg.event_type = static_cast<uint8_t>(event.type);
    msg.source_id = event.source_id;
    msg.severity = event.severity;
    msg.joint_id = event.joint_id;

    // Event details
    msg.error_code = event.error_code;
    msg.value = event.value;

    return msg;
}

}  // namespace plem::drive::ros
