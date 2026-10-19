// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/monitoring.hpp>

#include "ethercat/rt_event_emitter.hpp"

using namespace plem::core::monitoring;
using plem::core::kMaxBusAxes;
using plem::fieldbus::ethercat::RtEventEmitter;
using plem::fieldbus::ethercat::ServoStatus;

class RtEventEmitterTest : public ::testing::Test {
  protected:
    void SetUp() override {
        ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());
        emitter_ = std::make_unique<RtEventEmitter>();
    }

    void TearDown() override {
        emitter_.reset();
        plem::core::monitoring::shutdown();
    }

    EventQueue* queue() {
        return plem::core::monitoring::queue<Event>();
    }

    std::unique_ptr<RtEventEmitter> emitter_;
};

TEST_F(RtEventEmitterTest, ConstructionWithQueue) {
    EXPECT_NE(emitter_.get(), nullptr);
}

TEST_F(RtEventEmitterTest, NoEventsOnInitialProcess) {
    RtSample sample{};
    sample.deadline_miss = 0;
    sample.wkc_mismatch = 0;

    std::array<ServoStatus, kMaxBusAxes> servos{};
    for (auto& s : servos) {
        s.faulted = false;
        s.statusword = 0x0000;
    }

    emitter_->process(sample, servos, 1000000);

    EXPECT_EQ(queue()->front(), nullptr);
}

TEST_F(RtEventEmitterTest, ServoFaultRisingEdgeEmitsEvent) {
    RtSample sample{};
    sample.sequence = 100;
    sample.deadline_miss = 0;
    sample.wkc_mismatch = 0;

    std::array<ServoStatus, kMaxBusAxes> servos{};
    for (auto& s : servos) {
        s.faulted = false;
        s.statusword = 0x0000;
    }

    // First call - no fault, no event
    emitter_->process(sample, servos, 1000000);
    EXPECT_EQ(queue()->front(), nullptr);

    // Second call - joint 2 faults (rising edge)
    servos[2].faulted = true;
    servos[2].statusword = 0x1234;
    sample.sequence = 101;

    emitter_->process(sample, servos, 2000000);

    auto* event = queue()->front();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->type, EventType::SERVO_FAULT);
    EXPECT_EQ(event->joint_id, 2u);
    EXPECT_EQ(event->error_code, 0x1234u);
    EXPECT_EQ(event->ref_sample_sequence, 101u);
}

TEST_F(RtEventEmitterTest, ServoFaultNoEventOnSteadyState) {
    RtSample sample{};
    sample.sequence = 100;
    sample.deadline_miss = 0;
    sample.wkc_mismatch = 0;

    std::array<ServoStatus, kMaxBusAxes> servos{};
    for (auto& s : servos) {
        s.faulted = false;
        s.statusword = 0x0000;
    }
    servos[0].faulted = true;  // Start in fault state
    servos[0].statusword = 0x5678;

    // First call - already faulted, emits event (first observation)
    emitter_->process(sample, servos, 1000000);
    ASSERT_NE(queue()->front(), nullptr);
    queue()->pop();  // Clear first event

    // Second call - still faulted, NO new event
    sample.sequence = 101;
    emitter_->process(sample, servos, 2000000);

    EXPECT_EQ(queue()->front(), nullptr);
}

TEST_F(RtEventEmitterTest, DeadlineMissRisingEdgeEmitsEvent) {
    RtSample sample{};
    sample.sequence = 200;
    sample.deadline_miss = 0;
    sample.loop_exec_us = 900.0f;
    sample.wkc_mismatch = 0;

    std::array<ServoStatus, kMaxBusAxes> servos{};

    // Use timestamps > 100ms to satisfy initial cooldown from 0
    constexpr uint64_t kBaseTime = 200'000'000;  // 200ms

    // First call - no miss
    emitter_->process(sample, servos, kBaseTime);
    EXPECT_EQ(queue()->front(), nullptr);

    // Second call - deadline miss (rising edge)
    sample.deadline_miss = 1;
    sample.loop_exec_us = 1200.0f;
    sample.sequence = 201;

    emitter_->process(sample, servos, kBaseTime + 1'000'000);

    auto* event = queue()->front();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->type, EventType::DEADLINE_MISS);
    EXPECT_EQ(event->joint_id, kJointIdNotApplicable);
    EXPECT_FLOAT_EQ(event->value, 1200.0f);
}

TEST_F(RtEventEmitterTest, DeadlineMissCooldownPreventsFlooding) {
    RtSample sample{};
    sample.sequence = 300;
    sample.deadline_miss = 0;
    sample.wkc_mismatch = 0;

    std::array<ServoStatus, kMaxBusAxes> servos{};

    // Start at time > 100ms to satisfy initial cooldown from 0
    uint64_t time_ns = 200'000'000;  // 200ms

    // First miss - emits event
    sample.deadline_miss = 1;
    emitter_->process(sample, servos, time_ns);
    ASSERT_NE(queue()->front(), nullptr);
    queue()->pop();

    // Clear miss then re-trigger within cooldown
    sample.deadline_miss = 0;
    sample.sequence = 301;
    time_ns += 10'000'000;  // +10ms
    emitter_->process(sample, servos, time_ns);

    sample.deadline_miss = 1;
    sample.sequence = 302;
    time_ns += 10'000'000;  // +20ms total from first miss
    emitter_->process(sample, servos, time_ns);

    // Should not emit due to cooldown (100ms)
    EXPECT_EQ(queue()->front(), nullptr);

    // After cooldown expires
    sample.deadline_miss = 0;
    sample.sequence = 303;
    time_ns += 100'000'000;  // +120ms total from first miss
    emitter_->process(sample, servos, time_ns);

    sample.deadline_miss = 1;
    sample.sequence = 304;
    time_ns += 1'000'000;  // +121ms total from first miss
    emitter_->process(sample, servos, time_ns);

    // Now should emit
    ASSERT_NE(queue()->front(), nullptr);
    EXPECT_EQ(queue()->front()->type, EventType::DEADLINE_MISS);
}

TEST_F(RtEventEmitterTest, WkcMismatchRisingEdgeEmitsEvent) {
    RtSample sample{};
    sample.sequence = 400;
    sample.deadline_miss = 0;
    sample.wkc_mismatch = 0;
    sample.wkc = 12;
    sample.wc_state = 2;  // EC_WC_COMPLETE

    std::array<ServoStatus, kMaxBusAxes> servos{};

    // First call - no mismatch
    emitter_->process(sample, servos, 1000000);
    EXPECT_EQ(queue()->front(), nullptr);

    // Second call - WKC mismatch (rising edge), partial slave response
    sample.wkc_mismatch = 1;
    sample.wkc = 10;
    sample.wc_state = 1;  // EC_WC_INCOMPLETE
    sample.sequence = 401;

    constexpr uint32_t kMismatchCount = 3;
    emitter_->process(sample, servos, 2000000, kMismatchCount);

    auto* event = queue()->front();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->type, EventType::WKC_MISMATCH);
    EXPECT_EQ(event->error_code, 1u);     // wc_state = EC_WC_INCOMPLETE
    EXPECT_FLOAT_EQ(event->value, 3.0f);  // watchdog mismatch_count
}

TEST_F(RtEventEmitterTest, WkcMismatchFirstEventEmittedBelowCooldown) {
    // First WKC event must pass even when now_ns < cooldown, so
    // operators see the very first communication blip after RT loop start.
    RtSample sample{};
    sample.sequence = 500;
    sample.wkc_mismatch = 1;
    sample.wc_state = 0;  // EC_WC_ZERO

    std::array<ServoStatus, kMaxBusAxes> servos{};

    // 5ms (< 100ms cooldown) - should still emit due to first-event guard
    emitter_->process(sample, servos, 5'000'000, 2);

    auto* event = queue()->front();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->type, EventType::WKC_MISMATCH);
    EXPECT_EQ(event->error_code, 0u);  // EC_WC_ZERO
    EXPECT_FLOAT_EQ(event->value, 2.0f);
}

TEST_F(RtEventEmitterTest, WkcMismatchCooldownPreventsFlooding) {
    RtSample sample{};
    sample.sequence = 600;
    sample.deadline_miss = 0;
    sample.wkc_mismatch = 0;
    sample.wc_state = 2;

    std::array<ServoStatus, kMaxBusAxes> servos{};

    // Use timestamps > 100ms so cooldown comparisons are well-defined.
    uint64_t time_ns = 200'000'000;  // 200ms

    // First mismatch - emits (first event always passes)
    sample.wkc_mismatch = 1;
    sample.wc_state = 1;
    emitter_->process(sample, servos, time_ns, 1);
    ASSERT_NE(queue()->front(), nullptr);
    queue()->pop();

    // Clear, then re-trigger inside cooldown window
    sample.wkc_mismatch = 0;
    sample.wc_state = 2;
    sample.sequence = 601;
    time_ns += 10'000'000;  // +10ms
    emitter_->process(sample, servos, time_ns, 0);

    sample.wkc_mismatch = 1;
    sample.wc_state = 1;
    sample.sequence = 602;
    time_ns += 10'000'000;  // +20ms from first event
    emitter_->process(sample, servos, time_ns, 2);

    // Cooldown (100ms) suppresses the rising edge
    EXPECT_EQ(queue()->front(), nullptr);

    // After cooldown expires
    sample.wkc_mismatch = 0;
    sample.wc_state = 2;
    sample.sequence = 603;
    time_ns += 100'000'000;  // ~120ms from first event
    emitter_->process(sample, servos, time_ns, 0);

    sample.wkc_mismatch = 1;
    sample.wc_state = 1;
    sample.sequence = 604;
    time_ns += 1'000'000;  // ~121ms from first event
    emitter_->process(sample, servos, time_ns, 3);

    auto* event = queue()->front();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->type, EventType::WKC_MISMATCH);
    EXPECT_FLOAT_EQ(event->value, 3.0f);
}

// SERVO_FAULT carries the same-frame CiA 402 error code (0x603F) in `value`;
// error_code keeps the status word. The fieldbus device only stores the code,
// and this event is where it is reported.
TEST_F(RtEventEmitterTest, ServoFaultCarriesDriveErrorCodeInValue) {
    RtSample sample{};
    sample.sequence = 42;

    std::array<ServoStatus, kMaxBusAxes> servos{};
    servos[2] = {.faulted = true, .statusword = 0x0208, .error_code = 0x7500};

    emitter_->process(sample, servos, 1000000);

    auto* event = queue()->front();
    ASSERT_NE(event, nullptr);
    EXPECT_EQ(event->type, EventType::SERVO_FAULT);
    EXPECT_EQ(event->joint_id, 2u);
    EXPECT_EQ(event->error_code, 0x0208u);
    EXPECT_FLOAT_EQ(event->value, static_cast<float>(0x7500));
}
