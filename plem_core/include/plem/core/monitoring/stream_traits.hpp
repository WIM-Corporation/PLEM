// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include <cstddef>
#include <cstdint>

#include "plem/core/monitoring/event.hpp"
#include "plem/core/monitoring/rt_diagnostics.hpp"
#include "plem/core/monitoring/rt_sample.hpp"
#include "plem/core/monitoring/rt_signal.hpp"

namespace plem::core::monitoring {

/// QoS preset (ROS-independent representation)
enum class QoSPreset : uint8_t {
    BestEffort,         ///< High-speed, loss allowed (sensor data)
    Reliable,           ///< Reliability guaranteed (commands, status)
    ReliableTransient,  ///< Reliable + late-joiner (events)
};

/// Stream metadata (topic publishing characteristics)
struct PLEM_CORE_PUBLIC StreamDescriptor {
    const char* topic;  ///< Topic name
    QoSPreset qos;      ///< QoS preset
    size_t depth;       ///< Queue depth
};

/// Stream traits primary template (no definition = compile error)
template <typename T>
struct StreamTraits;

// Stream Definitions: Data type + publishing characteristics

template <>
struct StreamTraits<RtSample> {
    // Wire type is RtRawBatch: every cycle, batched up to 40 samples per
    // message, so depth counts batches: 8 batches ≈ 320 samples in flight.
    static constexpr StreamDescriptor descriptor = {
        .topic = "rt_raw",
        .qos = QoSPreset::BestEffort,
        .depth = 8,
    };
};

template <>
struct StreamTraits<Event> {
    static constexpr StreamDescriptor descriptor = {
        .topic = "rt_events",
        .qos = QoSPreset::ReliableTransient,
        .depth = 50,
    };
};

template <>
struct StreamTraits<ControllerSignal> {
    static constexpr StreamDescriptor descriptor = {
        .topic = "controller_signals_raw",  // relative; node ns auto-prefixes (the host's publisher batches it)
        .qos = QoSPreset::BestEffort,
        .depth = 64,  // consumer drains-all each spin; Config's default is 0 (off): a host that emits it sizes the
                      // queue (e.g. depth*10)
    };
};

template <>
struct StreamTraits<RtDiagnostics> {
    static constexpr StreamDescriptor descriptor = {
        .topic = "rt_diagnostics",   // volatile diagnostics channel, separate from retained rt_events
        .qos = QoSPreset::Reliable,  // reliable + volatile durability (no late-joiner replay of stale windows)
        .depth = 10,
    };
};

}  // namespace plem::core::monitoring
