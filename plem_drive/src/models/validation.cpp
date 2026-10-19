// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file validation.cpp
/// @brief Slave and axis spec checks; see models/validation.hpp.

#include "plem/drive/models/validation.hpp"

#include <algorithm>

#include <fmt/format.h>

#include <plem/fieldbus/cia402_profile.hpp>
#include <plem/fieldbus/cia402_types.hpp>
#include <plem/fieldbus/ethercat/config/configuration.hpp>

#include "models/pdo_capacity.hpp"

namespace plem::drive::models {

core::Result<void> check_slaves(std::span<const models::SlaveSpec> slaves, std::span<const models::AxisSpec> axes,
                                std::string_view tag) {
    // Per slave: ring position, identity, no duplicate or dead declaration,
    // and the PDO-layout value representation's presence and fixed-array
    // capacities (structure is the graph gate's job).
    for (size_t s = 0; s < slaves.size(); ++s) {
        const auto& slave = slaves[s];
        const std::string where = "[" + std::string(tag) + "] model spec slaves[" + std::to_string(s) + "]";
        if (slave.ring_position < 0) {
            return core::Result<void>::error((where + ".ring_position (" + std::to_string(slave.ring_position) +
                                              ") is negative — a SlaveSpec holds the resolved ring position"));
        }
        if (slave.vendor_id == 0 || slave.product_code == 0) {
            return core::Result<void>::error(
                (where + " identity is incomplete (vendor_id/product_code must be non-zero) — take both "
                         "from the drive's ESI or `ethercat slaves -v`"));
        }
        for (size_t prev = 0; prev < s; ++prev) {
            if (slaves[prev].ring_position == slave.ring_position) {
                return core::Result<void>::error(
                    (where + " and slaves[" + std::to_string(prev) + "] both declare ring position " +
                     std::to_string(slave.ring_position) + " — each physical slave is declared once"));
            }
        }
        bool hosts_an_axis = false;
        for (size_t i = 0; i < axes.size(); ++i) {
            hosts_an_axis = hosts_an_axis || axes[i].slave_index == s;
        }
        if (!hosts_an_axis) {
            return core::Result<void>::error((where + " (ring position " + std::to_string(slave.ring_position) +
                                              ") hosts no axis — a drive slave without a joint is a dead declaration"));
        }
        // PDO-layout value representation: only the fixed-array capacities
        // are checked here (they exist on the spec value alone). SM/PDO
        // structure is checked once, by the fieldbus validator, on the graph
        // create() materializes from this value (one rule set).
        // sync_count 0 is the scanned policy for the slave (no graph is
        // materialized; the bus reads the mapping from the master at
        // initialize and checks it with that same validator).
        if (const auto over = first_capacity_excess(slave.pdo_mapping)) {
            // The established wording: `.sync_count` on the layout, `pdo_mapping syncs[…]` below it.
            return core::Result<void>::error(where + ".pdo_mapping" + (over->field == "sync_count" ? "." : " ") +
                                             over->field + " (" + std::to_string(over->count) +
                                             ") exceeds the fixed capacity (" + std::to_string(over->capacity) + ")");
        }
    }
    return core::Result<void>::ok();
}

core::Result<void> check_axes(std::span<const models::AxisSpec> axes, std::string_view tag) {
    // Per axis: servo spec, channel grid, the mandatory CiA 402 profile and
    // the ErrorCode role width. The scalar rules (channel validity, the 0x800
    // grid, the served mode, the declared display) are the fieldbus
    // predicates, applied here to the hand-assembled spec before any bus
    // contact; ServoInfo::is_valid applies them to what the bus is handed.
    // Cross-axis (slave, channel) pair uniqueness is left to the bus.
    for (size_t i = 0; i < axes.size(); ++i) {
        const auto& axis = axes[i];
        const std::string where = "[" + std::string(tag) + "] model spec axes[" + std::to_string(i) + "]";
        // Channel map: every channel must name an implemented unit with
        // finite, positive parameters; the sentinel/unknown units and a zero
        // parameter are refused with the channel and value named.
        if (const core::Result<void> channel_error = fieldbus::ethercat::validate_axis_factors(axis.factors);
            !channel_error) {
            return core::Result<void>::error((where + ".factors rejected — " + channel_error.message()));
        }
        if (!fieldbus::ethercat::od_offset_on_channel_grid(axis.od_offset)) {
            return core::Result<void>::error(
                (where + ".od_offset (" + std::to_string(axis.od_offset) +
                 ") is off the CiA 402 0x800 channel grid — channel N (1..8) sits at (N-1)*0x800"));
        }
        // Declared mode of operation: the shared rule
        // (fieldbus::operation_mode_supported), front-loaded here so a wrong
        // declaration fails before any bus contact instead of at bus init. A
        // caller may run a narrower check of its own first.
        if (!fieldbus::operation_mode_supported(axis.modes_of_operation)) {
            return core::Result<void>::error(
                (where + ".modes_of_operation (" + std::to_string(static_cast<int>(axis.modes_of_operation)) +
                 ") is not a cyclic synchronous mode — the bus serves CST (10), CSP (8) and CSV (9)"));
        }
        // Mode-display timing (release admission): mandatory, two known
        // values. The UNDECLARED sentinel or any other number would admit a
        // release under a reporting timing the drive never declared.
        if (!fieldbus::mode_display_timing_declared(axis.mode_display_timing)) {
            return core::Result<void>::error(
                (where + ".mode_display_timing (" + std::to_string(static_cast<int>(axis.mode_display_timing)) +
                 ") is neither PRE_ENABLE nor POST_ENABLE — declare it (no default exists)"));
        }
        // Mandatory declaration, no fallback: an absent profile means the axis
        // never declared its cia402_profile; an invalid one (duplicate/invalid
        // rows) would drive the state sequence blind.
        if (const core::Result<void> profile_error = fieldbus::validate_cia402_profile(axis.cia402_profile);
            !profile_error) {
            return core::Result<void>::error((where + ".cia402_profile rejected — " + profile_error.message()));
        }
        // ErrorCode optional-PDO role (declared when index != 0): a zero-width
        // declared role registers nothing meaningful.
        if (axis.error_code_pdo.index != 0 && axis.error_code_pdo.bit_length == 0) {
            return core::Result<void>::error((where + ".error_code_pdo declares index 0x" +
                                              fmt::format("{:x}", axis.error_code_pdo.index) +
                                              " with bit_length 0 — declare the object's real width (0x603F is 16)"));
        }
    }
    return core::Result<void>::ok();
}

core::Result<void> check_model_name(std::span<const char> model_name, std::string_view tag) {
    // model_name contract: a NUL must exist inside the fixed array (readers
    // stop at it; a full 32-byte fill would leave it unterminated), and the
    // terminated string must contain at least one non-whitespace character
    // (a blank identity is as useless in logs and diagnostics as an empty one).
    const auto nul = std::find(model_name.begin(), model_name.end(), '\0');
    if (nul == model_name.end()) {
        return core::Result<void>::error(("[" + std::string(tag) +
                                          "] model spec model_name is not NUL-terminated within its fixed array — "
                                          "terminate the name within the array"));
    }
    const std::string_view name(model_name.data(), static_cast<size_t>(nul - model_name.begin()));
    if (name.find_first_not_of(" \t\r\n") == std::string_view::npos) {
        return core::Result<void>::error(("[" + std::string(tag) +
                                          "] model spec model_name is empty or blank — logs and diagnostics "
                                          "identify the model by it; give the model a non-blank name"));
    }
    return core::Result<void>::ok();
}

}  // namespace plem::drive::models
