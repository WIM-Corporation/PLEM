// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/monitoring/event.hpp>

#include <type_traits>

using namespace plem::core::monitoring;

// Layout the RT queues rely on (size and trivial copyability are
// static_asserts in event.hpp)

TEST(EventLayoutTest, IsCacheLineAlignedAndStandardLayout) {
    EXPECT_EQ(alignof(Event), 64u);
    EXPECT_TRUE(std::is_standard_layout_v<Event>);
}

// The factories are constexpr: pin what each one sets at compile time.
namespace {
constexpr auto kRt = static_cast<uint8_t>(SourceId::RT_LOOP);
constexpr auto sev(Severity s) {
    return static_cast<uint8_t>(s);
}

constexpr Event kSafety = Event::safety(EventType::PROTECTIVE_STOP, Severity::WARN, 3);
static_assert(kSafety.type == EventType::PROTECTIVE_STOP && kSafety.source_id == kRt &&
              kSafety.severity == sev(Severity::WARN) && kSafety.joint_id == 3);
constexpr Event kSafetyDefaults = Event::safety(EventType::BRAKE_ENGAGE);
static_assert(kSafetyDefaults.severity == sev(Severity::WARN) && kSafetyDefaults.joint_id == kJointIdNotApplicable);
constexpr Event kError = Event::error(EventType::SERVO_FAULT, 0x1234, 5);
static_assert(kError.type == EventType::SERVO_FAULT && kError.source_id == kRt &&
              kError.severity == sev(Severity::ERROR) && kError.joint_id == 5 && kError.error_code == 0x1234);
static_assert(Event::safety_with_value(EventType::PROTECTIVE_STOP, 0.5f).value == 0.5f);
static_assert(Event::protective_stop().type == EventType::PROTECTIVE_STOP &&
              Event::protective_stop().severity == sev(Severity::WARN));
static_assert(Event::protective_stop_clear().type == EventType::PROTECTIVE_STOP_CLEAR &&
              Event::protective_stop_clear().severity == sev(Severity::INFO));
static_assert(Event::brake_engage().type == EventType::BRAKE_ENGAGE &&
              Event::brake_engage().severity == sev(Severity::INFO));
static_assert(Event::brake_release().type == EventType::BRAKE_RELEASE &&
              Event::brake_release().severity == sev(Severity::INFO));
// value = the observed staleness age (s) at the trip
static_assert(Event::command_stale(0.0123f).type == EventType::COMMAND_STALE &&
              static_cast<uint8_t>(EventType::COMMAND_STALE) == 0x54 &&
              Event::command_stale(0.0123f).severity == sev(Severity::ERROR) &&
              Event::command_stale(0.0123f).value == 0.0123f);
}  // namespace
