// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/monitoring.hpp>
#include <plem/core/monitoring/monitoring_context.hpp>

namespace plem::core::monitoring {
namespace {

class MonitoringContextTest : public ::testing::Test {
  protected:
    void TearDown() override {
        // Clean up thread-local state
        set_thread_context(nullptr);
        // Shutdown any contexts we created
        ctx_a_.shutdown();
        ctx_b_.shutdown();
        plem::core::monitoring::shutdown();
    }

    MonitoringContext ctx_a_;
    MonitoringContext ctx_b_;
};

TEST_F(MonitoringContextTest, IndependentInitShutdown) {
    // Two contexts can be initialized and shut down independently
    ASSERT_TRUE(ctx_a_.initialize().is_ok());
    EXPECT_NE(ctx_a_.queue<RtSample>(), nullptr);
    EXPECT_EQ(ctx_b_.queue<RtSample>(), nullptr);

    ASSERT_TRUE(ctx_b_.initialize().is_ok());
    EXPECT_NE(ctx_a_.queue<RtSample>(), nullptr);
    EXPECT_NE(ctx_b_.queue<RtSample>(), nullptr);

    ctx_a_.shutdown();
    EXPECT_EQ(ctx_a_.queue<RtSample>(), nullptr);
    EXPECT_NE(ctx_b_.queue<RtSample>(), nullptr);
}

TEST_F(MonitoringContextTest, EmitRoutesToOwnQueue) {
    ASSERT_TRUE(ctx_a_.initialize().is_ok());
    ASSERT_TRUE(ctx_b_.initialize().is_ok());

    // Emit to context A
    RtSample sample{};
    sample.loop_jitter_us = 42.0f;
    EXPECT_TRUE(ctx_a_.emit(sample));

    // Context A has data, context B does not
    auto* qa = ctx_a_.queue<RtSample>();
    auto* qb = ctx_b_.queue<RtSample>();
    ASSERT_NE(qa, nullptr);
    ASSERT_NE(qb, nullptr);

    ASSERT_NE(qa->front(), nullptr);
    RtSample out = *qa->front();
    qa->pop();
    EXPECT_FLOAT_EQ(out.loop_jitter_us, 42.0f);

    // B's queue should be empty
    EXPECT_EQ(qb->front(), nullptr);
}

TEST_F(MonitoringContextTest, DefaultConfigAllocatesOnlyTheSampleAndEventStreams) {
    ASSERT_TRUE(ctx_a_.initialize().is_ok());
    EXPECT_NE(ctx_a_.queue<RtSample>(), nullptr);
    EXPECT_NE(ctx_a_.queue<Event>(), nullptr);
    EXPECT_EQ(ctx_a_.queue<ControllerSignal>(), nullptr);
    EXPECT_EQ(ctx_a_.queue<RtDiagnostics>(), nullptr);
    EXPECT_FALSE(ctx_a_.emit(ControllerSignal{})) << "emit into an unallocated stream drops";
    EXPECT_FALSE(ctx_a_.emit(RtDiagnostics{}));
    EXPECT_TRUE(ctx_a_.emit(Event{}));
}

TEST_F(MonitoringContextTest, AQueueSizeAboveTheRingsBoundIsRefusedNotThrown) {
    constexpr size_t too_big = RtQueue<Event>::kMaxCapacity + 1;
    for (const Config& config :
         {Config{.rt_queue_size = too_big}, Config{.event_queue_size = too_big},
          Config{.controller_signal_queue_size = too_big}, Config{.diagnostics_queue_size = too_big}}) {
        MonitoringContext ctx;
        const auto refused = ctx.initialize(config);
        ASSERT_FALSE(refused.is_ok());
        EXPECT_NE(refused.message().find("exceeds RtQueue::kMaxCapacity"), std::string::npos) << refused.message();
        EXPECT_EQ(ctx.queue<Event>(), nullptr) << "nothing was allocated";
    }
}

TEST_F(MonitoringContextTest, RepeatedInitializeKeepsTheFirstConfig) {
    EXPECT_TRUE(ctx_a_.initialize().is_ok());
    EXPECT_TRUE(ctx_a_.initialize().is_ok()) << "the same config again is a plain no-op";
    const auto different = ctx_a_.initialize({.controller_signal_queue_size = 8});
    ASSERT_FALSE(different.is_ok()) << "a different config is refused";
    EXPECT_EQ(different.message(),
              "initialize() called again with a different config; the first config stays active until shutdown()");
    EXPECT_EQ(ctx_a_.queue<ControllerSignal>(), nullptr) << "the first config stays active";
    ctx_a_.shutdown();
    EXPECT_TRUE(ctx_a_.initialize({.controller_signal_queue_size = 8}).is_ok())
        << "after shutdown a new config applies";
    EXPECT_NE(ctx_a_.queue<ControllerSignal>(), nullptr);
}

TEST_F(MonitoringContextTest, NonZeroQueueSizeAllocatesTheOptionalStreams) {
    ASSERT_TRUE(ctx_a_.initialize({.controller_signal_queue_size = 8, .diagnostics_queue_size = 8}).is_ok());
    EXPECT_NE(ctx_a_.queue<ControllerSignal>(), nullptr);
    EXPECT_NE(ctx_a_.queue<RtDiagnostics>(), nullptr);
}

TEST_F(MonitoringContextTest, ThreadContextRoutesEmit) {
    ASSERT_TRUE(ctx_a_.initialize().is_ok());
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());

    // Set thread context to ctx_a_
    set_thread_context(&ctx_a_);

    // plem::core::monitoring::emit should route to ctx_a_
    RtSample sample{};
    sample.loop_jitter_us = 77.0f;
    EXPECT_TRUE(plem::core::monitoring::emit(sample));

    // Verify data went to ctx_a_, not global
    auto* qa = ctx_a_.queue<RtSample>();
    ASSERT_NE(qa, nullptr);
    ASSERT_NE(qa->front(), nullptr);
    RtSample out = *qa->front();
    qa->pop();
    EXPECT_FLOAT_EQ(out.loop_jitter_us, 77.0f);

    // Global queue should be empty
    auto* qg = plem::core::monitoring::queue<RtSample>();
    ASSERT_NE(qg, nullptr);
    EXPECT_EQ(qg->front(), nullptr);
}

TEST_F(MonitoringContextTest, ThreadContextRoutesQueueLookup) {
    ASSERT_TRUE(ctx_a_.initialize().is_ok());
    ASSERT_TRUE(plem::core::monitoring::initialize().is_ok());

    set_thread_context(&ctx_a_);

    // plem::core::monitoring::queue<T>() should return ctx_a_'s queue
    auto* q_via_api = plem::core::monitoring::queue<RtSample>();
    auto* q_direct = ctx_a_.queue<RtSample>();
    EXPECT_EQ(q_via_api, q_direct);
}

}  // namespace
}  // namespace plem::core::monitoring
