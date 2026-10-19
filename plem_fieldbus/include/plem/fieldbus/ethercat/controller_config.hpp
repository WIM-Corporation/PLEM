// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/// @file controller_config.hpp
/// @brief EtherCATController's configuration, without the master headers, so a
/// model header can name its defaults without pulling in ecrt.h.

#include <cstdint>
#include <optional>

#include <plem/core/threading/rt_thread_config.hpp>

namespace plem::fieldbus::ethercat {

/// Configuration of the RT loop (EtherCATController::Config).
struct ControllerConfig {
    int cycle_time_us = 1000;  ///< Cycle period in microseconds (default: 1kHz)
    plem::core::threading::SchedulingPolicy policy =
        plem::core::threading::SchedulingPolicy::FIFO;  ///< RT scheduling policy
    int rt_priority = 98;                               ///< RT priority (1-99, higher = more priority)
    std::optional<plem::core::threading::CpuAffinity> cpu_affinity = std::nullopt;  ///< Optional CPU pinning
};

}  // namespace plem::fieldbus::ethercat
