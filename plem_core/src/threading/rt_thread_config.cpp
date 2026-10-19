// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <plem/core/threading/rt_thread_config.hpp>

#include <pthread.h>
#include <sched.h>

#include <cerrno>
#include <charconv>
#include <fstream>
#include <string>

namespace plem::core::threading {

std::optional<int> parse_last_isolated_cpu(std::string_view line) noexcept {
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) {
        line.remove_suffix(1);
    }
    if (line.empty()) {
        return std::nullopt;
    }
    // Last list token (e.g. "0-3,5" → "5"), then its range upper bound
    // (e.g. "9-11" → "11"). A leading '-' is not a range separator.
    if (size_t comma_pos = line.rfind(','); comma_pos != std::string_view::npos) {
        line = line.substr(comma_pos + 1);
    }
    if (size_t dash_pos = line.rfind('-'); dash_pos != std::string_view::npos && dash_pos > 0) {
        line = line.substr(dash_pos + 1);
    }

    int value = -1;
    const char* last = line.data() + line.size();
    const auto [ptr, ec] = std::from_chars(line.data(), last, value);
    if (ec != std::errc{} || ptr != last || value < 0) {
        return std::nullopt;
    }
    return value;
}

std::optional<int> detect_isolated_cpu() {
    std::ifstream file("/sys/devices/system/cpu/isolated");
    if (!file.is_open()) {
        return std::nullopt;
    }

    std::string line;
    if (!std::getline(file, line)) {
        return std::nullopt;
    }
    return parse_last_isolated_cpu(line);
}

RtApplyResult apply_rt_thread_config(const RtThreadConfig& cfg) {
    RtApplyResult result;
    if (cfg.policy != SchedulingPolicy::OTHER) {
        const int policy = static_cast<int>(cfg.policy);
        const int min_priority = sched_get_priority_min(policy);
        const int max_priority = sched_get_priority_max(policy);
        if (min_priority == -1 || max_priority == -1) {
            result.scheduling_ok = false;
            result.sched_errno = errno;
        } else if (cfg.priority < min_priority || cfg.priority > max_priority) {
            result.scheduling_ok = false;
            result.sched_errno = EINVAL;
        } else {
            sched_param param{};
            param.sched_priority = cfg.priority;
            const int rc = pthread_setschedparam(pthread_self(), policy, &param);
            if (rc != 0) {
                result.scheduling_ok = false;
                result.sched_errno = rc;
            }
        }
    }

    if (cfg.affinity && !cfg.affinity->cpus.empty()) {
        cpu_set_t cpu_set;
        CPU_ZERO(&cpu_set);
        for (int cpu : cfg.affinity->cpus) {
            if (cpu < 0 || cpu >= CPU_SETSIZE) {  // not a CPU this mask can name: pin nothing
                result.affinity_ok = false;
                result.affinity_errno = EINVAL;
                return result;
            }
            CPU_SET(cpu, &cpu_set);
        }
        const int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);
        if (rc != 0) {
            result.affinity_ok = false;
            result.affinity_errno = rc;
        }
    }
    return result;
}

}  // namespace plem::core::threading
