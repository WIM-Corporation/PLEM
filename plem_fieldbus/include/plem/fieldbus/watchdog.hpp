// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <cstdint>

namespace plem::fieldbus {

/// @brief Watchdog state (RT-safe: trivially copyable, no allocation)
struct WatchdogState {
    uint32_t mismatch_count{0};
    bool triggered{false};
    bool armed{false};  ///< first healthy WKC observed; mismatch counting starts only after
};

/// @brief Result of a single watchdog evaluation
struct WatchdogResult {
    bool protective_stop_active{false};  ///< Level: asserted every cycle while degraded beyond threshold
    WatchdogState next_state;
};

/// @brief Pure function: evaluate watchdog logic for one RT cycle
///
/// @param current   Current watchdog state
/// @param wkc_ok    True if WKC check passed this cycle
/// @param threshold Consecutive mismatch count before the protective stop
/// @return WatchdogResult with next state and action flags
///
/// @note RT-safe: constexpr noexcept, no dynamic allocation, no exceptions
[[nodiscard]] constexpr WatchdogResult evaluate_watchdog(WatchdogState current, bool wkc_ok,
                                                         uint32_t threshold) noexcept {
    WatchdogResult result;
    result.next_state = current;

    if (wkc_ok) {
        result.next_state.armed = true;
        result.next_state.mismatch_count = 0;
        result.next_state.triggered = false;
        return result;
    }

    // Bring-up gate: WKC mismatches before the bus has ever been healthy are
    // slave OP-transition transients, not degradation; counting them would
    // latch an EXTERNAL protective stop on every start. The watchdog detects
    // healthy -> unhealthy only. Not fail-open: a
    // bus that never comes up never enables the servos (no torque path, the
    // brakes hold).
    if (!current.armed) {
        return result;
    }

    if (!current.triggered) {
        ++result.next_state.mismatch_count;
    }

    if (result.next_state.mismatch_count >= threshold && !current.triggered) {
        result.next_state.triggered = true;
    }

    // Level view: stays asserted while the bus remains degraded beyond the
    // threshold. Consumers use it to re-assert the protective stop after an
    // optimistic clear (operator reset), so a clear only sticks once healthy.
    result.protective_stop_active = result.next_state.triggered;

    return result;
}

// Host command writer → EtherCAT command freshness

/// @brief Default freshness budget for the host command writer → EtherCAT
/// loop. EtherCATController starts with it; a
/// host may set its own (set_command_stale_timeout_ns; the drive layer uses
/// plem::drive::kCommandStaleTimeoutNs).
///
/// The command writer stamps one write per 1 kHz cycle, but it may be a plain
/// SCHED_OTHER std::thread (a host running user control code there must not
/// let that code starve the system under FIFO). CFS gives no latency bound,
/// so the budget must absorb the scheduler tail: a 10 ms budget can false-trip
/// on scheduler jitter alone on an idle system. 50 ms would also cover that
/// jitter; 100 ms lets a host use one staleness model across its command legs,
/// accepting about 0.16 rad more worst-case hang travel than 50 ms while
/// bounding exposure from a real hang to 0.1 s.
/// Scope of that bound: it arms on the first stamped write (update_ns == 0 is
/// bring-up, not stale), and the 0.1 s ends at the protective stop latch; what happens
/// to the frozen value after the latch is the command owner's: with declared
/// drive limits EtherCATController's torque gate writes 0 Nm from that cycle
/// on (drive_safety.hpp); otherwise the host relies on the forced engage and
/// its control thread's own zero-torque branches.
///
/// Limitation: a wall-clock budget cannot distinguish a hung writer from a
/// legal control step/preemption exceeding 100 ms: both trip. Control code
/// whose step can legitimately exceed this needs a per-step budget, not a
/// larger freshness window.
inline constexpr int64_t kDefaultCommandStaleTimeoutNs = 100'000'000;  // 100 ms wall-clock (writer paces at 1 kHz)
// Floor pin for the doc block above, header-side so a retune trips in every consumer TU.
static_assert(kDefaultCommandStaleTimeoutNs >= 100'000'000,
              "budget must absorb the SCHED_OTHER writer's scheduler tail, or the stop trips falsely at boot");

/// @brief Command freshness state (RT-safe: trivially copyable, no allocation)
struct CommandFreshnessState {
    bool triggered{false};
    bool armed{false};  ///< first stamped write observed; staleness counts only after
};

/// @brief Result of a single command-freshness evaluation
struct CommandFreshnessResult {
    bool should_protective_stop{false};  ///< Edge: fires exactly once when the budget is crossed
    bool protective_stop_active{false};  ///< Level: asserted every cycle while still stale
    CommandFreshnessState next_state;
};

/// @brief Pure function: evaluate command freshness for one RT cycle
///
/// @param current    Current freshness state
/// @param update_ns  CLOCK_MONOTONIC stamp of the latest observed command write
///                   (0 = the writer has never stamped)
/// @param now_ns     CLOCK_MONOTONIC now (same clock domain as the writer)
/// @param timeout_ns Staleness budget (stale when age > timeout_ns, strictly greater-than)
///
/// @note RT-safe: constexpr noexcept, no dynamic allocation, no exceptions
[[nodiscard]] constexpr CommandFreshnessResult evaluate_command_freshness(CommandFreshnessState current,
                                                                          int64_t update_ns, int64_t now_ns,
                                                                          int64_t timeout_ns) noexcept {
    CommandFreshnessResult result;
    result.next_state = current;

    // Bring-up gate (mirrors WatchdogState::armed):
    // before the control thread has ever stamped a write, staleness is
    // meaningless. Not fail-open: a thread that never started never produced
    // torque either — the brakes hold. An already-old first stamp DOES count
    // (writer wrote once and hung before this loop started observing).
    if (!current.armed) {
        if (update_ns == 0) {
            return result;
        }
        result.next_state.armed = true;
    }

    const bool stale = (now_ns - update_ns) > timeout_ns;
    if (!stale) {
        result.next_state.triggered = false;  // writer resumed; recovery mirrors wkc_ok
        return result;
    }

    if (!current.triggered) {
        result.should_protective_stop = true;  // edge: exactly once per hang episode
    }
    result.next_state.triggered = true;
    result.protective_stop_active = true;
    return result;
}

}  // namespace plem::fieldbus
