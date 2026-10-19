// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <atomic>
#include <cstdint>

namespace plem::core {

/**
 * @brief Protective stop source flags (bitfield)
 *
 * Multiple sources can be active simultaneously.
 * All must be cleared before motion can resume.
 */
enum class ProtectiveStopSource : uint8_t {
    NONE = 0,
    /// Asserted by the RT command owner's own limit checks (drive layer: EtherCATController,
    /// LIMIT_STOP; a higher-layer host: its control thread);
    /// cleared by an armed reset_error.
    INTERNAL = 1 << 0,
    /// Asserted from outside the controller thread: fieldbus-plane health (WKC loss,
    /// torque-command staleness, an enabled drive leaving its declared CiA 402 mode of
    /// operation). Optimistically cleared on reset; re-latches while the cause persists. No
    /// software E-stop path asserts this (hardware E-stop acts at the drive/STO level).
    EXTERNAL = 1 << 1,
};

/**
 * @brief Atomic flags for controller mode state
 *
 * Provides lock-free, RT-safe mode state sharing between
 * the host's mode state machine, its control loop and EtherCATController.
 *
 * @thread_safety All members are atomic for concurrent access.
 * @realtime_safe No allocations, no syscalls.
 */
struct PLEM_CORE_PUBLIC ControlModeFlags {
    /// Motion is enabled (brake released, ready for trajectory following)
    /// When false: no motion torque flows (see all_drives_enabled below) and a
    /// brake release request is discarded; engaging is the host's request.
    std::atomic<bool> motion_enabled{false};

    /// Hand-guiding mode, set by the host (the drive layer never sets it); the
    /// fieldbus reads it only in the brake logic's protective-stop recovery path.
    /// Robot tier: internal, not part of the public API; may change without notice.
    std::atomic<bool> freedrive_enabled{false};

    /// Every drive is confirmed in OPERATION_ENABLED in its declared
    /// mode of operation (0x6061 == the axis declaration), so commands are
    /// physically realizable on every joint. Published each sequencing cycle by
    /// the fieldbus controller from each drive's last statusword and mode
    /// display: false while any axis is not yet initialized, shows an
    /// unrecognized statusword pattern (a held state is not evidence), is not
    /// in OPERATION_ENABLED (so false at boot), or displays another mode
    /// (which is also the in-service trip). The brake sequences' own progress
    /// is not consulted. Revoked whenever the fieldbus loop is not running
    /// (at loop entry and exit).
    ///
    /// Hosts gate motion commands on motion_enabled and this flag.
    /// motion_enabled alone is the mode intent, set while the enable sequence
    /// is still running; commanding in that window integrates against drives
    /// that do not yet act. The gate is drive enablement rather than the brake
    /// mask because a shared-brake group reports released as soon as its first
    /// axis enables, while the other drives of the group may still be enabling.
    std::atomic<bool> all_drives_enabled{false};

    /// Active ProtectiveStopSource bits (see the enum for who sets and clears
    /// each). Every source must be cleared before motion resumes. A stop
    /// initiates forced brake engage; the bitfield does not confirm brake
    /// closure.
    std::atomic<uint8_t> protective_stop{0};

    /// Reset error command (clear protective stop)
    /// Armed by the operator reset, consumed via exchange() by exactly one RT
    /// command owner per assembly (the drive layer's EtherCATController, or a
    /// higher-layer host's control thread)
    std::atomic<bool> reset_error{false};

    // Protective Stop Helpers (RT-safe)

    /// Check if any protective stop is active
    [[nodiscard]] bool is_protective_stop_active() const noexcept {
        return protective_stop.load(std::memory_order_acquire) != 0;
    }

    /// Check if the INTERNAL protective stop is active (see ProtectiveStopSource::INTERNAL)
    [[nodiscard]] bool is_internal_protective_stop() const noexcept {
        return (protective_stop.load(std::memory_order_acquire) &
                static_cast<uint8_t>(ProtectiveStopSource::INTERNAL)) != 0;
    }

    /// Check if the EXTERNAL protective stop is active (fieldbus-plane health;
    /// see ProtectiveStopSource::EXTERNAL; not a hardware E-stop)
    [[nodiscard]] bool is_external_protective_stop() const noexcept {
        return (protective_stop.load(std::memory_order_acquire) &
                static_cast<uint8_t>(ProtectiveStopSource::EXTERNAL)) != 0;
    }

    /// Set a protective stop source (atomic OR)
    void set_protective_stop(ProtectiveStopSource source) noexcept {
        protective_stop.fetch_or(static_cast<uint8_t>(source), std::memory_order_acq_rel);
    }

    /// Clear a protective stop source (atomic AND with complement)
    void clear_protective_stop(ProtectiveStopSource source) noexcept {
        protective_stop.fetch_and(~static_cast<uint8_t>(source), std::memory_order_acq_rel);
    }

    /// Clear all protective stop sources
    void clear_all_protective_stop() noexcept {
        protective_stop.store(0, std::memory_order_release);
    }
};

/// INTERNAL protective stop set/clear decision, shared by every RT command owner
/// (decision part; the per-joint bus flag writes stay with the calling RT thread).
struct InternalProtectiveStopDecision {
    bool set_protective_stop{false};    ///< call mode_flags->set_protective_stop(INTERNAL); re-fires every
                                        ///< cycle on a new violation or while latched
    bool clear_protective_stop{false};  ///< reset requested + all within limits + no persistent
                                        ///< fault: clear bus flags + mode clear + emit
                                        ///< protective_stop_clear
    bool clear_rejected{false};         ///< reset requested + a violation or persistent internal
                                        ///< fault remains: emit protective_stop with error_code=1
};

/// INTERNAL protective stop decision.
/// `set_protective_stop` fires when a violation is present (latched or fresh); callers apply set
/// before clear, so the clearing cycle also fires set first (set and clear are
/// independent flags, both may be true in one cycle).
/// `!all_within_limits` stands for the per-joint/workspace violation accumulation
/// (all_within_limits is false iff at least one violation was detected).
/// `persistent_fault` = an INTERNAL source that keeps asserting independent of pose.
/// Clearing while it holds would publish a phantom PROTECTIVE_STOP_CLEAR (the source re-latches
/// within the same cycle), so the reset is rejected instead of cleared.
[[nodiscard]] constexpr InternalProtectiveStopDecision evaluate_internal_protective_stop(
    bool initial_internal_protective_stop, bool all_within_limits, bool reset_requested,
    bool persistent_fault) noexcept {
    InternalProtectiveStopDecision d;
    d.set_protective_stop = initial_internal_protective_stop || !all_within_limits || persistent_fault;
    d.clear_protective_stop = reset_requested && all_within_limits && !persistent_fault;
    d.clear_rejected = reset_requested && (!all_within_limits || persistent_fault);
    return d;
}

}  // namespace plem::core
