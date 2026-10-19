// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file rt_memory.hpp
/// @brief RT memory hardening: process-wide (apply_memory_lock, once before any
/// RT thread starts), per RT thread (prefault_stack), and page-fault counts to
/// check the result outside the loop.

#pragma once

#include <plem/core/visibility_control.hpp>

#include <sys/resource.h>
#include <unistd.h>

#include <cstddef>

namespace plem::core::threading {

/// Apply RT memory hardening: 1) mallopt(M_TRIM_THRESHOLD=-1, M_MMAP_MAX=0),
/// 2) mlockall(MCL_CURRENT | MCL_FUTURE), 3) prefault the heap. Every failure
/// is logged as a warning and tolerated (development boxes may lack
/// CAP_IPC_LOCK); without a successful mlockall the prefault is skipped.
PLEM_CORE_PUBLIC void apply_memory_lock();

// Stack Prefaulting

/**
 * @brief Prefault thread stack to prevent page faults during RT operation.
 *
 * Should be called at the beginning of each RT thread's entry function,
 * before entering the main loop. This ensures the thread stack is backed by
 * physical memory before any RT-critical operations.
 *
 * Note: Must be called from within the target thread (stack is thread-local).
 *       The prefault size should be smaller than the thread's stack limit.
 *
 * @tparam kStackSize Size in bytes to prefault (default: 64KB)
 */
template <size_t kStackSize = 64 * 1024>
inline void prefault_stack() {
    volatile char stack_buf[kStackSize];
    const auto page_size = static_cast<size_t>(sysconf(_SC_PAGESIZE));

    // Touch each page to force physical memory mapping
    for (size_t i = 0; i < kStackSize; i += page_size) {
        stack_buf[i] = 0;
    }

    // Prevent compiler from optimizing out the buffer
    (void)stack_buf[0];
}

// Page Fault Monitoring

/**
 * @brief Page fault counters from getrusage().
 */
struct PLEM_CORE_PUBLIC PageFaults {
    long minor;  ///< Minor faults: RAM remapping (1-10µs latency)
    long major;  ///< Major faults: disk I/O required (1-100ms latency) - CRITICAL!
};

/**
 * @brief Monitor page faults during RT operation for quality verification.
 *
 * Usage:
 * - Create at RT thread start
 * - Call get_delta() at shutdown to report faults during operation
 *
 * WARNING: Do not call from within RT loop (getrusage is a syscall).
 *          Use only for initialization/shutdown reporting.
 *
 * Example:
 * @code
 *   PageFaultMonitor pf_monitor;
 *   // ... RT operation ...
 *   auto pf = pf_monitor.get_delta();
 *   if (pf.major > 0) {
 *       logger.error("Major page faults detected: {}", pf.major);
 *   }
 * @endcode
 */
class PLEM_CORE_PUBLIC PageFaultMonitor {
  public:
    PageFaultMonitor() : baseline_(get_current()) {}

    /**
     * @brief Get page faults since construction or last get_delta() call.
     * @return Delta page fault counts
     */
    PageFaults get_delta() {
        PageFaults current = get_current();
        PageFaults delta{current.minor - baseline_.minor, current.major - baseline_.major};
        baseline_ = current;
        return delta;
    }

    /**
     * @brief Get current process page fault counts.
     * @return Current page fault counts from getrusage()
     */
    static PageFaults get_current() {
        struct rusage usage {};
        getrusage(RUSAGE_SELF, &usage);
        return {usage.ru_minflt, usage.ru_majflt};
    }

  private:
    PageFaults baseline_;
};

}  // namespace plem::core::threading
