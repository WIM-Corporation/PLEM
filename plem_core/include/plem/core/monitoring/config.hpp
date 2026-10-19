// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <cstddef>
#include <cstdint>

namespace plem::core::monitoring {

/**
 * @brief Configuration for the RT monitoring pipeline: the per-stream queue sizes.
 *
 * Any *_queue_size of 0 leaves that stream unallocated: queue<T>() returns
 * nullptr and emit() returns false (drops). RtSample and Event are on by
 * default; the two optional control streams are off.
 *
 * @note All values have sensible defaults for a 1kHz RT loop.
 */
struct PLEM_CORE_PUBLIC Config {
    /// Size of the RT sample queue (number of RtSample entries).
    /// At 1kHz, 8192 samples = ~8 seconds of data.
    size_t rt_queue_size = 8192;

    /// Size of the event queue (number of Event entries).
    /// Used for low-frequency events like state changes and faults.
    size_t event_queue_size = 512;

    /// Size of the ControllerSignal queue, an optional per-cycle
    /// controller-telemetry stream. Off (0) by default; a host that emits it
    /// sets a non-zero size.
    size_t controller_signal_queue_size = 0;

    /// Size of the RtDiagnostics queue, an optional control-thread
    /// diagnostics stream. Off (0) by default; a host that emits it sets a
    /// non-zero size.
    size_t diagnostics_queue_size = 0;

    bool operator==(const Config&) const = default;
};

}  // namespace plem::core::monitoring
