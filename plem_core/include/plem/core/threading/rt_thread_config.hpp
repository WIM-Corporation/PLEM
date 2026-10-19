// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace plem::core::threading {

/**
 * @brief Parse the last CPU index from kernel isolated-CPU list syntax.
 *
 * Accepts the /sys/devices/system/cpu/isolated formats: "7", "7,11", "5-7",
 * "0-3,5", "5-7,9-11" (last list token; a range token contributes its upper
 * bound). Only the last token is validated: strict full-consumption integer
 * parse, trailing whitespace/newline tolerated; a malformed last token (junk,
 * trailing text, negative, overflow) is std::nullopt, never a throw. Earlier
 * tokens are not inspected because the input is kernel-generated sysfs content.
 *
 * The token is resolved comma-first, then dash within that token.
 */
[[nodiscard]] PLEM_CORE_PUBLIC std::optional<int> parse_last_isolated_cpu(std::string_view line) noexcept;

/**
 * @brief Detect isolated CPUs from kernel configuration.
 *
 * Reads /sys/devices/system/cpu/isolated to find CPUs isolated via
 * kernel boot parameter (isolcpus=). Returns the last isolated CPU,
 * which is typically reserved for RT workloads.
 *
 * @return Last isolated CPU index, or std::nullopt if none isolated
 *         (or the file content is malformed; see parse_last_isolated_cpu)
 *
 * Example:
 * @code
 *   RtThreadConfig config;
 *   if (auto rt_cpu = detect_isolated_cpu()) {
 *       config.affinity = CpuAffinity{{*rt_cpu}};
 *   }
 * @endcode
 */
PLEM_CORE_PUBLIC std::optional<int> detect_isolated_cpu();

/**
 * @brief CPU affinity configuration for thread pinning
 *
 * Used to pin threads to specific CPU cores for deterministic performance.
 * Empty cpus vector means no affinity (thread can run on any core).
 *
 * Example:
 * @code
 *   CpuAffinity affinity{{11}};  // Pin to CPU 11
 *   CpuAffinity multi{{0, 1, 2}};  // Pin to CPUs 0, 1, 2
 * @endcode
 */
struct PLEM_CORE_PUBLIC CpuAffinity {
    std::vector<int> cpus;  ///< CPU cores to pin thread to
};

/**
 * @brief Real-time scheduling policy for threads
 *
 * Maps directly to Linux SCHED_* constants:
 * - OTHER (0): leave the thread's policy and priority as they are
 *   (apply_rt_thread_config makes no scheduling call)
 * - FIFO (1): Real-time FIFO (requires root/CAP_SYS_NICE)
 * - RR (2): Real-time Round Robin (requires root/CAP_SYS_NICE)
 *
 * @note RT policies (FIFO, RR) require root privileges or CAP_SYS_NICE capability
 */
enum class SchedulingPolicy : std::uint8_t {
    OTHER = 0,  ///< No scheduling call: the thread keeps its current policy and priority
    FIFO = 1,   ///< SCHED_FIFO - Real-time FIFO (highest determinism)
    RR = 2      ///< SCHED_RR - Real-time Round Robin
};

/**
 * @brief Real-time thread configuration parameters
 *
 * Encapsulates all RT thread settings that can be applied after thread creation.
 *
 * Example:
 * @code
 *   RtThreadConfig config;
 *   config.policy = SchedulingPolicy::FIFO;
 *   config.priority = 98;  // High priority (1-99)
 *   config.affinity = CpuAffinity{{11}};  // Pin to CPU 11
 * @endcode
 *
 * @note Priority range for FIFO/RR is 1-99 (higher = more priority)
 */
struct PLEM_CORE_PUBLIC RtThreadConfig {
    SchedulingPolicy policy = SchedulingPolicy::FIFO;    ///< Scheduling policy
    int priority = 80;                                   ///< RT priority (1-99, higher = more priority)
    std::optional<CpuAffinity> affinity = std::nullopt;  ///< Optional CPU pinning
};

/**
 * @brief Outcome of applying an RtThreadConfig to the calling thread.
 *
 * Each aspect is reported independently so the caller can log a precise warning
 * and continue best-effort (an RT-promotion failure must never abort the thread).
 */
struct PLEM_CORE_PUBLIC RtApplyResult {
    bool scheduling_ok{true};  ///< false if the policy/priority could not be set
    bool affinity_ok{true};    ///< false if CPU pinning could not be set
    int sched_errno{0};        ///< errno from the scheduling call (0 = ok / no-op)
    int affinity_errno{0};     ///< errno from the affinity call (0 = ok / no-op)

    [[nodiscard]] bool ok() const noexcept {
        return scheduling_ok && affinity_ok;
    }
};

/**
 * @brief Apply scheduling policy/priority + CPU affinity to the calling thread.
 *
 * The RT threads of these packages (the fieldbus loop among them) apply their
 * scheduling and pinning through this. Call from inside the target thread after
 * all heap pre-allocation is done and before the hot loop; applying SCHED_FIFO
 * before allocation would risk a malloc at RT priority.
 *
 * `SchedulingPolicy::OTHER` makes no scheduling call (the thread keeps its
 * policy and priority). A CPU
 * index outside [0, CPU_SETSIZE) refuses the pinning (affinity_errno EINVAL)
 * and leaves the thread's mask as it was. A failure
 * (e.g. missing CAP_SYS_NICE) is reported in the result, never thrown; the
 * caller logs and keeps running best-effort.
 *
 * @thread_safety Operates only on the calling thread (pthread_self); safe to
 *                invoke concurrently from independent threads with distinct configs.
 * @realtime_safe No: issues pthread_* syscalls; invoke once at thread start.
 */
[[nodiscard]] PLEM_CORE_PUBLIC RtApplyResult apply_rt_thread_config(const RtThreadConfig& cfg);

}  // namespace plem::core::threading
