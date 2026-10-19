// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

// validate_bus_configuration: pure (no ecrt call), so a file tool can run the
// bus's check offline.

#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/pdo_layout_validation.hpp"

#include "ethercat/slave_axes.hpp"

namespace plem::fieldbus::ethercat {

core::Result<void> validate_bus_configuration(const std::vector<ServoInfo>& servos,
                                              const std::vector<SlaveSdos>& startup,
                                              const std::vector<SlaveSdos>& stored,
                                              const std::vector<SlaveExpectedIdentity>& expected) {
    for (size_t i = 0; i < servos.size(); ++i) {
        const ServoInfo& info = servos[i];
        if (!info.is_valid()) {
            const core::Result<void> channels = validate_axis_factors(info.factors);
            return core::Result<void>::error(fmt::format(
                "Axis {} has an invalid configuration — refusing to initialize: {}. Channels: {}. "
                "Required: every channel declares an implemented unit with finite parameters > 0 (encoder_resolution > "
                "0), vendor_id/product_code != 0, finite min_torque_nm < max_torque_nm with min_torque_nm <= 0 <= "
                "max_torque_nm, od_offset on the 0x800 grid of "
                "channels 1..{}, slave_position -1..65535 (the master addresses a slave by a 16-bit ring position), "
                "modes_of_operation CYCLIC_SYNC_TORQUE, CYCLIC_SYNC_POSITION or CYCLIC_SYNC_VELOCITY, "
                "mode_display_timing declared PRE_ENABLE or POST_ENABLE (no default exists), "
                "fault_reset_pulse_ms <= {}, joint_offset 0 on an endless axis (position_endless).",
                i, info.dump(), channels ? std::string("ok") : channels.message(), kMaxCia402Channels,
                kMaxFaultResetPulseMs));
        }
        // CiA 402 transition profile: mandatory model declaration, no fallback
        // (an absent/duplicated/invalid table would drive the state sequence
        // blind; see cia402_profile.hpp). Same pre-hardware fail-fast window
        // as the channel check above: zero ecrt calls on rejection.
        if (const core::Result<void> profile_error = validate_cia402_profile(info.cia402_profile); !profile_error) {
            return core::Result<void>::error(
                fmt::format("Axis {} CiA 402 transition profile rejected — refusing to initialize: {}", i,
                            profile_error.message()));
        }
        // Declared ErrorCode role must be consistent: index overflow, and
        // actually present in the declared layout (a
        // declared-but-unmapped role would fail the whole registration list
        // with an opaque ENOENT on hardware; catch it here with a name).
        if (info.error_code_pdo.index != 0) {
            const uint32_t role_index = info.error_code_pdo.index + static_cast<uint32_t>(info.od_offset);
            if (role_index > 0xFFFF) {
                return core::Result<void>::error(
                    fmt::format("Axis {}: Error code (0x603F) role index 0x{:04x} + od_offset 0x{:04x} overflows the "
                                "object dictionary — refusing to initialize.",
                                i, info.error_code_pdo.index, info.od_offset));
            }
            // The read path decodes the role with a fixed 16-bit read
            // (EC_READ_U16, ServoPDOHandler). Any other declared width would
            // silently mis-read on hardware, so it is rejected here, before
            // any hardware contact.
            if (info.error_code_pdo.bit_length != 16) {
                return core::Result<void>::error(
                    fmt::format("Axis {}: Error code (0x603F) role declares bit_length {} — the decode is fixed at "
                                "16 bits (EC_READ_U16); declare 16 or drop the role. Refusing to initialize.",
                                i, info.error_code_pdo.bit_length));
            }
        }
    }

    // Cross-servo consistency (multi-axis wiring is per-model data: a wiring
    // typo must fail here with a name, not on hardware as an opaque ENOENT or
    // a silently misconfigured slave):
    // - (slave_position, od_offset) pairs must be unique (two axes at the
    //   same channel of the same slave would fight over one register file);
    // - all axes of one slave must agree on identity (one ec_slave_config_t
    //   per slave; a VID/PID mismatch means the model wired two different
    //   device types to one position) and on distributed clocks (configured
    //   once per slave);
    // - all axes of one slave must point at the same layout object (applied
    //   or scanned once per slave; null = scanned counts as an object, so a
    //   slave half declared, half scanned is refused too; a disagreement is
    //   never silently skipped).
    for (size_t i = 0; i < servos.size(); ++i) {
        const ServoInfo& a = servos[i];
        const int32_t slave_a = slave_of(servos, i);
        for (size_t j = i + 1; j < servos.size(); ++j) {
            const ServoInfo& b = servos[j];
            const int32_t slave_b = slave_of(servos, j);
            if (slave_a != slave_b) {
                continue;
            }
            if (a.od_offset == b.od_offset) {
                return core::Result<void>::error(
                    fmt::format("Axes {} and {} share slave {} AND channel offset 0x{:04x} — duplicate "
                                "(slave_position, od_offset) pair; give each axis its own channel.",
                                i, j, slave_a, a.od_offset));
            }
            if (a.vendor_id != b.vendor_id || b.product_code != a.product_code) {
                return core::Result<void>::error(
                    fmt::format("Axes {} and {} share slave {} but disagree on identity "
                                "(0x{:x}/0x{:x} vs 0x{:x}/0x{:x}) — one slave has one identity; fix the model wiring.",
                                i, j, slave_a, a.vendor_id, a.product_code, b.vendor_id, b.product_code));
            }
            if (a.dc_assign_activate != b.dc_assign_activate || a.dc_sync0_cycle_ns != b.dc_sync0_cycle_ns ||
                a.dc_sync0_shift_ns != b.dc_sync0_shift_ns || a.dc_write_cycle_time != b.dc_write_cycle_time) {
                return core::Result<void>::error(fmt::format(
                    "Axes {} and {} share slave {} but disagree on distributed clocks "
                    "(assign_activate 0x{:x}/0x{:x}, sync0 cycle {}/{} ns, shift {}/{} ns, write_cycle_time {}/{}) "
                    "— DC is configured once per slave; declare the same DC values on every axis of it.",
                    i, j, slave_a, a.dc_assign_activate, b.dc_assign_activate, a.dc_sync0_cycle_ns, b.dc_sync0_cycle_ns,
                    a.dc_sync0_shift_ns, b.dc_sync0_shift_ns, a.dc_write_cycle_time, b.dc_write_cycle_time));
            }
            if (a.pdo_mapping != b.pdo_mapping) {
                return core::Result<void>::error(fmt::format(
                    "Axes {} and {} share slave {} but declare different PDO layouts; the layout is applied "
                    "once per slave, so every axis must point at the same layout object (null = scanned counts as "
                    "one: declare every axis of a slave, or scan every axis).",
                    i, j, slave_a));
            }
        }
    }

    // Declared PDO layouts: semantic validation (pdo_layout_validation.hpp),
    // once per physical slave with every axis sharing it: roles resolve at
    // each axis's own channel, so a shared layout's other-axis entries are not
    // duplicates. A scanned slave (null layout) has nothing to check yet; its
    // mapping is read and checked with the same validator at Step 3.
    for (size_t i = 0; i < servos.size(); ++i) {
        if (first_axis_index_of_slave(servos, i) != i) {
            continue;  // validated under the slave's first axis
        }
        const ServoInfo& first = servos[i];
        if (first.pdo_mapping == nullptr) {
            continue;  // scanned at Step 3 (every axis of this slave is null, checked above)
        }
        const int32_t slave = slave_of(servos, i);
        const SlaveAxes group = axes_of_slave(servos, slave);
        if (const core::Result<void> error = validate_pdo_mapping(*first.pdo_mapping, group.axes); !error) {
            return core::Result<void>::error(
                fmt::format("slave {} (axes {}): declared PDO layout rejected — refusing to initialize: {}", slave,
                            group.members, error.message()));
        }
    }
    // Startup SDO lists (drive-model schema §4.7): each names a slave hosting
    // an injected servo, and every write is one the master can reserve; same
    // zero-ecrt-call window as the checks above (configure_SDOs relies on it).
    const auto hosts_servo = [&](uint32_t position) {
        for (size_t i = 0; i < servos.size(); ++i) {
            if (slave_of(servos, i) == static_cast<int32_t>(position)) {
                return true;
            }
        }
        return false;
    };
    // Startup writes and stored values (drive-model schema §4.9) share the two
    // refusals: each list names a servo slave, each SDO a width it can carry.
    for (const auto& [lists, kind] : {std::pair{&startup, "startup"}, std::pair{&stored, "stored"}}) {
        for (const SlaveSdos& list : *lists) {
            if (!hosts_servo(list.slave_position)) {
                return core::Result<void>::error(
                    fmt::format("{} SDOs name slave {}, which hosts no configured axis — give the ring position of a "
                                "slave that hosts a configured axis, or drop the list. Refusing to initialize.",
                                kind, list.slave_position));
            }
            for (size_t i = 0; i < list.sdos.size(); ++i) {
                const StartupSdo& sdo = list.sdos[i];
                if (!sdo.is_valid()) {
                    return core::Result<void>::error(fmt::format(
                        "slave {} {} SDO {} (0x{:04x}:{:02x}) has size {} and value 0x{:x} — the size must be 1, 2 "
                        "or 4 bytes and the value must fit in it. Refusing to initialize.",
                        list.slave_position, kind, i, sdo.index, sdo.subindex, sdo.size, sdo.value));
                }
            }
        }
    }
    // Expected identities (drive-model schema §6): each names a slave hosting
    // an injected servo; same zero-ecrt-call window.
    for (const SlaveExpectedIdentity& identity : expected) {
        if (!hosts_servo(identity.slave_position)) {
            return core::Result<void>::error(
                fmt::format("an expected identity names slave {}, which hosts no configured axis — the approval "
                            "record's slaves must be the declared drive slaves. Refusing to initialize.",
                            identity.slave_position));
        }
    }
    return core::Result<void>::ok();
}

}  // namespace plem::fieldbus::ethercat
