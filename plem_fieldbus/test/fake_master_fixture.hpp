// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file fake_master_fixture.hpp
/// @brief Shared helpers for the fake-master test TUs (both fake-master gtest
/// targets compile the real bus/device sources against the fake-master kit):
/// a valid ServoInfo, six of them, one IO slave, the ecrt-call-log search, the
/// bounded poll, the lock-step gate, the controller config and monitoring context
/// every loop test uses, and the bus reset fixture. Test-local (not installed).

#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <plem/core/monitoring/monitoring_context.hpp>

#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/controller.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

namespace plem::fieldbus::test {

using plem::core::units::Pulse;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::test::FakeEcrt;

inline ServoInfo make_servo_info(
    plem::fieldbus::ModeDisplayTiming mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE) {
    ServoInfo info{};
    info.vendor_id = plem::fieldbus::test::kSyntheticVendorId;
    info.product_code = plem::fieldbus::test::kSyntheticProductCode;
    info.factors = plem::fieldbus::ethercat::make_axis_factors(plem::fieldbus::test::kSyntheticServoSpec);
    info.flip = false;
    info.joint_offset = Pulse(0);
    info.cia402_profile = plem::fieldbus::kNotReadyShutdownCia402Profile;
    info.mode_display_timing = mode_display_timing;
    info.pdo_mapping = &plem::fieldbus::test::kSingleAxisCstLayout;
    return info;
}

inline std::vector<ServoInfo> six_servos(
    plem::fieldbus::ModeDisplayTiming mode_display_timing = plem::fieldbus::ModeDisplayTiming::PRE_ENABLE) {
    return std::vector<ServoInfo>(6, make_servo_info(mode_display_timing));
}

inline plem::fieldbus::ethercat::IOInfo one_io_info() {
    plem::fieldbus::ethercat::IOInfo info{};
    info.vendor_id = plem::fieldbus::test::kSyntheticVendorId;
    info.product_code = 0x40000000;
    return info;
}

/// An int32 as the little-endian bytes an SDO upload answers with.
inline std::vector<uint8_t> le32(int32_t value) {
    const auto u = static_cast<uint32_t>(value);
    return {static_cast<uint8_t>(u), static_cast<uint8_t>(u >> 8), static_cast<uint8_t>(u >> 16),
            static_cast<uint8_t>(u >> 24)};
}

inline bool log_contains(const std::string& needle) {
    const auto& calls = FakeEcrt::calls();
    return std::any_of(calls.begin(), calls.end(),
                       [&](const std::string& call) { return call.find(needle) != std::string::npos; });
}

/// Byte offset the fake domain handed out for (slave position, object index,
/// subindex 0), resolved from the bus's own registration, so a test does not
/// depend on registration order. Throws if the entry was never registered.
inline uint32_t pdo_offset(unsigned int position, unsigned int index) {
    const int offset = FakeEcrt::domain_offset_of(position, index, 0);
    if (offset < 0) {
        throw std::logic_error("pdo_offset: entry not registered in the fake domain");
    }
    return static_cast<uint32_t>(offset);
}

inline int count_calls(const std::string& needle) {
    const auto& calls = FakeEcrt::calls();
    return static_cast<int>(std::count_if(
        calls.begin(), calls.end(), [&](const std::string& call) { return call.find(needle) != std::string::npos; }));
}

inline int index_of(const std::string& needle) {
    const auto& calls = FakeEcrt::calls();
    for (size_t i = 0; i < calls.size(); ++i) {
        if (calls[i].find(needle) != std::string::npos) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

/// Bounded 5 ms poll: true once `pred` holds, false at `timeout`.
template <typename Pred>
bool wait_for(Pred&& pred, std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

/// Parks the RT thread after every send until the test allows the next cycle:
/// install on_send() as the FakeEcrt send observer, then step() one cycle at a time.
class Lockstep {
  public:
    void on_send() {
        std::unique_lock lock(mutex_);
        ++sent_;
        cv_.notify_all();
        cv_.wait(lock, [&] { return free_run_ || sent_ < allowed_; });
    }
    bool wait_first() {
        std::unique_lock lock(mutex_);
        return cv_.wait_for(lock, std::chrono::seconds(10), [&] { return sent_ >= 1; });
    }
    bool step() {
        std::unique_lock lock(mutex_);
        const uint64_t target = sent_ + 1;
        allowed_ = target;
        cv_.notify_all();
        return cv_.wait_for(lock, std::chrono::seconds(10), [&] { return sent_ >= target; });
    }
    void free_run() {
        std::lock_guard lock(mutex_);
        free_run_ = true;
        cv_.notify_all();
    }

  private:
    std::mutex mutex_;
    std::condition_variable cv_;
    uint64_t sent_{0};
    uint64_t allowed_{0};
    bool free_run_{false};
};

/// A loop test's controller: `cycle_us`, SCHED_OTHER (no RT privileges needed).
template <size_t N>
typename plem::fieldbus::ethercat::EtherCATController<N>::Config test_controller_config(int cycle_us = 1000) {
    typename plem::fieldbus::ethercat::EtherCATController<N>::Config config;
    config.cycle_time_us = cycle_us;
    config.policy = plem::core::threading::SchedulingPolicy::OTHER;
    config.rt_priority = 0;
    return config;
}

/// An initialized monitoring context, this thread's for its lifetime; a
/// failed ASSERT that leaves the scope clears it too.
class ScopedMonitoringContext : public plem::core::monitoring::MonitoringContext {
  public:
    ScopedMonitoringContext() {
        EXPECT_TRUE(initialize().is_ok());
        plem::core::monitoring::set_thread_context(this);
    }
    ~ScopedMonitoringContext() {
        plem::core::monitoring::set_thread_context(nullptr);
    }
    ScopedMonitoringContext(const ScopedMonitoringContext&) = delete;
    ScopedMonitoringContext& operator=(const ScopedMonitoringContext&) = delete;
};

}  // namespace plem::fieldbus::test

class EtherCATBusFakeMasterTest : public ::testing::Test {
  protected:
    void SetUp() override {
        plem::fieldbus::test::FakeEcrt::reset();
    }
};
