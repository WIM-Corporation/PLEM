// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace plem::core::monitoring {

/// Max flat signal slots per ControllerSignal sample (= sum of manifest signal.count).
/// A 6-axis host publishing 14 signals per axis needs 14 * 6 = 84; 96 = modest headroom. ~792 B/sample.
/// A host that allocates this stream sizes it with Config::controller_signal_queue_size (0, off, by default);
/// depth*10 = 640 samples is ~507 KB locked into RAM.
inline constexpr std::size_t kMaxSignalSlots = 96;
static_assert(kMaxSignalSlots >= 84, "a 6-axis host emits 14 signals per axis");

/// @brief One RT cycle's controller telemetry snapshot (RT -> non-RT over a bounded SPSC queue;
/// a full queue drops the sample and counts the overflow).
/// Produced by a higher-layer host's control thread; the drive packages allocate no queue for it.
/// POD: trivially copyable. monotonic_ns/sequence auto-filled by MonitoringContext::emit().
/// Robot tier: internal, not part of the public API; may change without notice.
/// @realtime_safe No dynamic allocation, trivially copyable.
struct PLEM_CORE_PUBLIC ControllerSignal {
    uint64_t monotonic_ns{0};
    uint64_t sequence{0};
    uint16_t slot_count{0};                        ///< number of valid leading values (= manifest signal_slots)
    uint16_t reserved0{0};                         ///< explicit padding
    uint32_t reserved1{0};                         ///< explicit padding
    std::array<double, kMaxSignalSlots> values{};  ///< [slot_count] valid; rest unspecified
};
static_assert(std::is_trivially_copyable_v<ControllerSignal>);

}  // namespace plem::core::monitoring
