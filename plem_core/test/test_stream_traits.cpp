// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <plem/core/monitoring/stream_traits.hpp>

using namespace plem::core::monitoring;

class StreamTraitsTest : public ::testing::Test {};

TEST_F(StreamTraitsTest, RtSampleHasCorrectTopic) {
    constexpr auto desc = StreamTraits<RtSample>::descriptor;
    EXPECT_STREQ(desc.topic, "rt_raw");
}

TEST_F(StreamTraitsTest, RtSampleHasBestEffortQoS) {
    constexpr auto desc = StreamTraits<RtSample>::descriptor;
    EXPECT_EQ(desc.qos, QoSPreset::BestEffort);
}

TEST_F(StreamTraitsTest, EventHasCorrectTopic) {
    constexpr auto desc = StreamTraits<Event>::descriptor;
    EXPECT_STREQ(desc.topic, "rt_events");
}

TEST_F(StreamTraitsTest, EventHasReliableTransientQoS) {
    constexpr auto desc = StreamTraits<Event>::descriptor;
    EXPECT_EQ(desc.qos, QoSPreset::ReliableTransient);
}
