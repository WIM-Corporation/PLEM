// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file mock_drive_host.hpp
/// @brief A drive host for the node and system-interface tests: records what
/// it is asked and answers what the test sets. Test-local (not installed).

#pragma once

#include <atomic>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/result.hpp>

#include "ros/drive_host_adapter.hpp"

namespace plem::drive::ros::test {

using plem::core::Result;

/// Records what the node asked of the host and hands back what the test wants.
class MockDriveHost : public IDriveHost {
  public:
    explicit MockDriveHost(std::vector<std::string> names) : names_(std::move(names)) {
        EXPECT_TRUE(monitoring_.initialize().is_ok());
        state.process_data_complete = true;  // every slave in OP unless a test says otherwise
    }
    ~MockDriveHost() override {
        monitoring_.shutdown();
    }

    [[nodiscard]] std::span<const std::string> axis_names() const noexcept override {
        return names_;
    }
    Result<void> start() override {
        return start_result;
    }
    plem::fieldbus::ethercat::BrakeQuiesceOutcome stop() override {
        calls.emplace_back("stop");
        return {};
    }
    Result<void> enable() override {
        ++enable_calls;
        calls.emplace_back("enable");
        return enable_result;
    }
    void ramp_stop() override {
        ++ramp_stop_calls;
        calls.emplace_back("ramp_stop");
    }
    // Records and answers as configured: the length/finiteness rules belong to
    // the real host (IDriveHost::write), so the node is not pinned against a
    // copy of them.
    [[nodiscard]] bool write(std::span<const double> command, std::span<const double> rate,
                             int64_t sampled_ns) noexcept override {
        ++write_calls;
        last_write.assign(command.begin(), command.end());
        last_rate.assign(rate.begin(), rate.end());
        last_sampled_ns = sampled_ns;
        return write_result;
    }
    [[nodiscard]] bool read(DriveState& out) const noexcept override {
        if (!readable) {
            return false;
        }
        out = state;
        if (incomplete_reads.load() > 0) {
            --incomplete_reads;
            out.process_data_complete = false;
        }
        return true;
    }
    void reset() override {
        ++reset_calls;
        calls.emplace_back("reset");
    }
    [[nodiscard]] plem::core::monitoring::MonitoringContext* monitoring_context() noexcept override {
        return &monitoring_;
    }
    [[nodiscard]] std::span<const std::string> pdo_mapping_digests() const noexcept override {
        return digests;
    }
    [[nodiscard]] std::span<const int8_t> modes_of_operation() const noexcept override {
        return modes;
    }

    std::vector<std::string> digests{"sha256:" + std::string(64, 'a'), ""};  // a scanned and a declared slave
    std::vector<int8_t> modes{8, 10};                                        // CSP, CST
    std::vector<double> last_write;
    std::vector<double> last_rate;
    int64_t last_sampled_ns = 0;
    int write_calls = 0;
    bool write_result = true;
    int enable_calls = 0;
    int ramp_stop_calls = 0;
    int reset_calls = 0;
    std::vector<std::string> calls;  ///< enable/ramp_stop/reset, in call order
    bool readable = true;
    Result<void> enable_result = Result<void>::ok();
    Result<void> start_result = Result<void>::ok();
    DriveState state{};
    /// The next reads report the working counter incomplete (a slave not yet in OP).
    mutable std::atomic<int> incomplete_reads{0};

  private:
    std::vector<std::string> names_;
    plem::core::monitoring::MonitoringContext monitoring_;
};

}  // namespace plem::drive::ros::test
