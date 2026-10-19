// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/core/monitoring/monitoring_context.hpp"

namespace plem::core::monitoring {

// Thread-local context

namespace {
thread_local MonitoringContext* t_context = nullptr;
}  // namespace

void set_thread_context(MonitoringContext* ctx) noexcept {
    t_context = ctx;
}

MonitoringContext* get_thread_context() noexcept {
    return t_context;  // nullptr means caller should use default
}

}  // namespace plem::core::monitoring
