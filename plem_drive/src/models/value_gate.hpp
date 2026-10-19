// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file value_gate.hpp
/// @brief The value gate for DriveModelSpec<N> (not installed). Runs first in
/// DriveHost::create(), before any bus contact. Reuses the shared span
/// validators (check_slaves / check_axes / check_model_name) under the
/// "DriveHost" tag and adds the drive-layer rules: the 16-bit ring position,
/// the safety declaration, the output-SM watchdog (a dead master must still stop
/// the drive), the DC knob, the period envelope, the fault-reset pair, the
/// startup SDO lists and the approval enum. DC: `dc_enable` (the file's
/// `bus.sync_mode: dc_sync0`) is the model's deployment knob (robot-tier
/// idiom): when set, every slave must declare DC. The drive rules speak the
/// file's keys under the flat axis index (axes[i].limits, axes[i].factors …);
/// the loader rewrites that index to the file's path. Returns ok or the refusal.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>

#include <fmt/format.h>

#include <plem/core/result.hpp>

#include <plem/fieldbus/ethercat/brake_logic.hpp>
#include <plem/fieldbus/ethercat/drive_safety.hpp>
#include <plem/fieldbus/ethercat/utils/servo_conversions.hpp>

#include "drive_host_rules.hpp"
#include "plem/drive/models/drive_model_spec.hpp"
#include "plem/drive/models/validation.hpp"

namespace plem::drive::detail {

/// Core-owned objects (schema §4.7): a startup SDO on them would be a second
/// source. Slave-wide: PDO mapping and assignment, which ecrt.h reserves to
/// the master (it configures the declared ones from pdo_mapping; 0x1C10-0x1C2F
/// is ecrt.h's assignment range), and the sync-manager parameters, which the
/// dc_* fields declare (with dc_enable set, the core writes 0x1C32:02 only
/// for a slave declaring dc_write_cycle_time; it never reads it). Per
/// axis, at base + od_offset: controlword, mode and the mapped command targets,
/// which the core writes, and the limit objects and the home offset, which
/// compete with safety[]; the core does not write those. An address check:
/// it checks no vendor object's physical effect.
inline constexpr std::array<std::array<uint16_t, 2>, 4> kCoreOwnedSlaveObjects{
    {{0x1600, 0x17FF}, {0x1A00, 0x1BFF}, {0x1C10, 0x1C2F}, {0x1C32, 0x1C33}}};
inline constexpr std::array<uint16_t, 9> kCoreOwnedAxisObjects{0x6040, 0x6060, 0x6071, 0x607A, 0x60FF,
                                                               0x6072, 0x607C, 0x607D, 0x6080};

/// The drive file's key (under `axes[i].factors`) for a conversion factor
/// defect, or nullptr for a defect no file key can carry: a unit defect (the
/// file's unit is a closed choice) and a RATED_PER_MILLE scale other than 1000
/// (the file has no scale key under that unit). `torque` is the axis's torque
/// unit; it picks the torque keys.
[[nodiscard]] constexpr const char* drive_factor_key(fieldbus::ethercat::ChannelDefect::Kind kind,
                                                     fieldbus::ethercat::TorqueUnit torque) noexcept {
    using Kind = fieldbus::ethercat::ChannelDefect::Kind;
    switch (kind) {
        case Kind::GEAR_RATIO:
            return "gear_ratio";
        case Kind::ENCODER_RESOLUTION:
            return "encoder_resolution";
        case Kind::COUNTS_PER_UNIT:
            return "torque.counts_per_amp";
        case Kind::TORQUE_CONSTANT:
            return torque == fieldbus::ethercat::TorqueUnit::RATED_PER_MILLE ? "torque.motor_rated_torque_nm"
                                                                             : "torque.torque_constant";
        case Kind::VELOCITY_FACTOR:
            return "velocity.velocity_factor";
        case Kind::NONE:
        case Kind::UNIT_UNDECLARED:
        case Kind::UNIT_OUT_OF_RANGE:
        case Kind::PER_MILLE_SCALE:  // the loader writes 1000; a C++ spec gets the shared check's wording
            break;
    }
    return nullptr;
}

/// What a per-slave SDO list holds, in the words its refusals use.
struct SdoListWords {
    const char* listed;   ///< how the list holds its entries
    const char* purpose;  ///< what nothing would do with them past slave_count
    const char* entries;  ///< what one entry is
};
inline constexpr SdoListWords kStartupSdoWords{"declares writes", "reserve", "writes"};
inline constexpr SdoListWords kStoredSdoWords{"lists values", "read", "values"};

/// The shape both per-slave SDO lists share: empty past slave_count, the
/// count within capacity. `where` names the list, `slave` its index.
[[nodiscard]] inline core::Result<void> check_sdo_list_shape(const models::StartupSdoList& list,
                                                             const std::string& where, size_t slave, size_t slave_count,
                                                             const SdoListWords& words) {
    if (slave >= slave_count) {
        if (list.count != 0) {
            return core::Result<void>::error(where + " " + words.listed + " for a slave past slave_count (" +
                                             std::to_string(slave_count) + ") — nothing would " + words.purpose +
                                             " them");
        }
        return core::Result<void>::ok();
    }
    if (list.count > models::StartupSdoList::kMaxWrites) {
        return core::Result<void>::error(where + ".count (" + std::to_string(list.count) +
                                         ") exceeds the capacity of " +
                                         std::to_string(models::StartupSdoList::kMaxWrites) + " " + words.entries);
    }
    return core::Result<void>::ok();
}

/// One list entry as a refusal names it: `<list>[i] (0xIIII:SS)`.
[[nodiscard]] inline std::string sdo_item(const std::string& where, size_t i,
                                          const fieldbus::ethercat::StartupSdo& sdo) {
    return fmt::format("{}[{}] (0x{:04x}:{:02x})", where, i, sdo.index, sdo.subindex);
}

/// The entry's own shape: a size of 1, 2 or 4 bytes and a value that fits it.
[[nodiscard]] inline core::Result<void> check_sdo_entry(const std::string& item,
                                                        const fieldbus::ethercat::StartupSdo& sdo) {
    if (!sdo.is_valid()) {
        return core::Result<void>::error(fmt::format(
            "{} has size {} and value 0x{:x} — the size must be 1, 2 or 4 bytes and the value must fit in it", item,
            sdo.size, sdo.value));
    }
    return core::Result<void>::ok();
}

/// Each axis names a declared slave.
template <size_t N>
[[nodiscard]] core::Result<void> check_axis_slaves(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    for (size_t i = 0; i < N; ++i) {
        const std::string where = tag + "axes[" + std::to_string(i) + "]";
        if (spec.axes[i].slave_index >= spec.slave_count) {
            return core::Result<void>::error(where + ".slave_index (" + std::to_string(spec.axes[i].slave_index) +
                                             ") is outside the declared slaves (slave_count " +
                                             std::to_string(spec.slave_count) + ")");
        }
    }
    return core::Result<void>::ok();
}

/// The factor group in the file's terms (schema §4.2), ahead of the shared
/// channel check whose wording uses the higher-layer hosts' terms ("torque channel (SCALED_CURRENT) …").
template <size_t N>
[[nodiscard]] core::Result<void> check_factors_in_file_terms(const models::DriveModelSpec<N>& spec,
                                                             const std::string& tag) {
    for (size_t i = 0; i < N; ++i) {
        const auto& ch = spec.axes[i].factors;
        const std::string where = tag + "axes[" + std::to_string(i) + "].factors";
        // Each conversion's own defect() is the rule; only the wording is the
        // file's. A unit defect cannot come from a file (closed choice in the
        // loader) and is left to the shared check below.
        for (const auto defect : {ch.torque.defect(), ch.position.defect(), ch.velocity.defect()}) {
            if (const char* key = drive_factor_key(defect.kind, ch.torque.unit)) {
                if (defect.kind == fieldbus::ethercat::ChannelDefect::Kind::ENCODER_RESOLUTION) {
                    return core::Result<void>::error(
                        where +
                        ".encoder_resolution is 0 — it counts encoder increments per motor revolution, "
                        "so it is > 0");
                }
                return core::Result<void>::error(
                    fmt::format("{}.{} ({}) must be finite and > 0", where, key, defect.value));
            }
        }
    }
    return core::Result<void>::ok();
}

/// The safety declaration (`axes[i].limits`) against the axis's mode and factors.
template <size_t N>
[[nodiscard]] core::Result<void> check_limits(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    for (size_t i = 0; i < N; ++i) {
        const auto& s = spec.safety[i];
        const std::string where = tag + "axes[" + std::to_string(i) + "].limits";
        const bool unbounded = axis_endless(s.position_min_rad, s.position_max_rad);
        // The whole count range must land on a finite, non-zero angle: an
        // overflowing encoder_resolution × gear_ratio decodes every count to 0
        // (motion unseen), an underflowing one decodes a count to ±inf. The
        // range is the drive's int32 count, or the endless count domain on an
        // axis without end stops; its negative end is the larger, so it bounds both.
        const auto& position_channel = spec.axes[i].factors.position;
        const double count_end_rad =
            unbounded
                ? fieldbus::ethercat::endless_count_to_angle(-fieldbus::ethercat::kEndlessCountDomain, position_channel)
                : fieldbus::ethercat::to_angle(core::units::Pulse(std::numeric_limits<int32_t>::min()),
                                               position_channel)
                      .value();
        if (!std::isfinite(count_end_rad) || !(count_end_rad < 0.0)) {
            return core::Result<void>::error(
                tag + "axes[" + std::to_string(i) + "].factors maps the drive's " +
                (unbounded ? "endless count range (2^48 counts)" : "int32 count") + " to " +
                fmt::format("{}", count_end_rad) +
                " rad — factors.encoder_resolution × factors.gear_ratio must keep that conversion finite and "
                "non-zero");
        }
        if (unbounded && s.zero_offset_count != 0) {
            return core::Result<void>::error(
                where + ".position is null but zero_offset_count is " + std::to_string(s.zero_offset_count) +
                " — an axis without end stops counts from the drive's raw zero, so a non-zero offset would create "
                "a range where the position saturates (a saturated position hides motion from the stop checks); "
                "set zero_offset_count: 0");
        }
        if (!unbounded && (!std::isfinite(s.position_min_rad) || !std::isfinite(s.position_max_rad) ||
                           !(s.position_min_rad < s.position_max_rad))) {
            return core::Result<void>::error(
                where + ".position must be finite with min < max (got " + fmt::format("{}", s.position_min_rad) + ", " +
                fmt::format("{}", s.position_max_rad) +
                ") — the joint's travel in rad about zero_offset_count, or null for an axis without end stops");
        }
        if (!std::isfinite(s.velocity_rad_s) || !(s.velocity_rad_s > 0.0)) {
            return core::Result<void>::error(where + ".velocity_rad_s must be finite and > 0 (got " +
                                             fmt::format("{}", s.velocity_rad_s) + ")");
        }
        if (spec.axes[i].modes_of_operation != fieldbus::OperationMode::CYCLIC_SYNC_TORQUE) {
            if (s.torque_min_nm != 0.0 || s.torque_max_nm != 0.0) {
                return core::Result<void>::error(
                    where +
                    ".torque_nm is declared on an axis whose mode is not cst — it clamps cst torque commands "
                    "only; leave it 0 (in a file: drop the key)");
            }
        } else if (!std::isfinite(s.torque_min_nm) || !std::isfinite(s.torque_max_nm) || !(s.torque_min_nm < 0.0) ||
                   !(s.torque_max_nm > 0.0)) {
            return core::Result<void>::error(where + ".torque_nm must be finite with min < 0 < max (got " +
                                             fmt::format("{}", s.torque_min_nm) + ", " +
                                             fmt::format("{}", s.torque_max_nm) +
                                             ") — a clamp that cannot carry 0 Nm would distort every stop command");
        }
        if (spec.axes[i].modes_of_operation == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE) {
            if (s.acceleration_rad_s2 != 0.0) {
                return core::Result<void>::error(
                    where +
                    ".acceleration_rad_s2 is declared on a cst axis — it is a csp or csv axis's stop deceleration "
                    "(a cst axis stops by its drive's own disable reaction); leave it 0 (in a file: drop the key)");
            }
        } else if (!std::isfinite(s.acceleration_rad_s2) || !(s.acceleration_rad_s2 > 0.0)) {
            return core::Result<void>::error(where + ".acceleration_rad_s2 must be finite and > 0 (got " +
                                             fmt::format("{}", s.acceleration_rad_s2) +
                                             ") — the deceleration a stop ramps this csp or csv axis to rest at");
        } else if (const auto& profile = spec.axes[i].cia402_profile;
                   std::none_of(profile.transitions.begin(), profile.transitions.begin() + profile.transition_count,
                                [](const fieldbus::Cia402Transition& t) {
                                    return t.from == fieldbus::ServoState::OPERATION_ENABLED &&
                                           t.to == fieldbus::ServoState::QUICK_STOP_ACTIVE;
                                })) {
            return core::Result<void>::error(
                tag + "axes[" + std::to_string(i) +
                "].cia402_profile has no operation_enabled → quick_stop_active row (CiA 402 transition 11, "
                "Quick stop) — a csp or csv axis's stop falls back to it when the ramp is not confirmed in time; "
                "declare the row (controlword 0x0002) or use the built-in table");
        }
    }
    return core::Result<void>::ok();
}

/// Axis identity (schema §4.5). The drive layer has no URDF, so a label here
/// is the only name `~/joint_states` can carry; an empty or repeated one
/// makes the stream unreadable, and the lower-case C-identifier subset is
/// safe wherever a joint name lands.
template <size_t N>
[[nodiscard]] core::Result<void> check_axis_names(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    std::array<std::string_view, N> names{};
    for (size_t i = 0; i < N; ++i) {
        const std::string where = tag + "axes[" + std::to_string(i) + "].name";
        const auto& raw = spec.axis_names[i];
        const auto* nul = std::find(raw.begin(), raw.end(), '\0');
        if (nul == raw.end()) {
            return core::Result<void>::error(where + " is not NUL-terminated within its " + std::to_string(raw.size()) +
                                             "-byte array — a label that fills the array has no end to read to");
        }
        names[i] = std::string_view(raw.data(), static_cast<size_t>(nul - raw.begin()));
        if (names[i].empty()) {
            return core::Result<void>::error(
                where + " is empty — the drive layer has no URDF, so this is the joint's only label");
        }
        for (size_t c = 0; c < names[i].size(); ++c) {
            const char ch = names[i][c];
            if (!((ch >= 'a' && ch <= 'z') || ch == '_' || (c > 0 && ch >= '0' && ch <= '9'))) {
                return core::Result<void>::error(
                    where + " (\"" + std::string(names[i]) +
                    "\") must match [a-z_][a-z0-9_]* (lower-case ASCII — the drive-model schema's "
                    "axis-name rule)");
            }
        }
        for (size_t j = 0; j < i; ++j) {
            if (names[j] == names[i]) {
                return core::Result<void>::error(where + " (\"" + std::string(names[i]) + "\") repeats axes[" +
                                                 std::to_string(j) +
                                                 "].name — joint labels address the axes, so they must be distinct");
            }
        }
    }
    return core::Result<void>::ok();
}

/// Per slave: the 16-bit ring position, the output-SM watchdog and the DC knob.
template <size_t N>
[[nodiscard]] core::Result<void> check_slave_settings(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    for (size_t s = 0; s < spec.slave_count; ++s) {
        const auto& slave = spec.slaves[s];
        const std::string where = tag + "slaves[" + std::to_string(s) + "]";
        // ecrt addresses a slave by a uint16 ring position; a wider value
        // would be narrowed silently on the way to the master.
        if (slave.ring_position > 0xFFFF) {
            return core::Result<void>::error(
                where + ".ring_position (" + std::to_string(slave.ring_position) +
                ") exceeds 65535 — the master addresses a slave by a 16-bit ring position");
        }
        // A scanned slave (sync_count 0) declares no sync manager to check:
        // the core writes nothing, so the drive runs on its own SM watchdog
        // setting, which this check cannot see.
        bool output_watchdog = slave.pdo_mapping.sync_count == 0;
        for (size_t m = 0; m < slave.pdo_mapping.sync_count; ++m) {
            const auto& sync = slave.pdo_mapping.syncs[m];
            output_watchdog = output_watchdog || (sync.output && sync.watchdog);
        }
        if (!output_watchdog) {
            return core::Result<void>::error(
                where +
                ".pdo_mapping declares no output sync manager with the watchdog enabled — losing the master "
                "must stop the drive, and once the master is gone the core relies on the drive's SM watchdog for "
                "that; enable the watchdog on an output sync manager");
        }
        if (spec.dc_enable && slave.dc_assign_activate == 0) {
            return core::Result<void>::error(
                where +
                " declares no DC (dc.assign_activate 0) while bus.sync_mode is dc_sync0 — every slave must "
                "declare DC to run the ring on distributed clocks");
        }
        if (!spec.dc_enable && slave.dc_write_cycle_time) {
            return core::Result<void>::error(
                where +
                " declares dc.write_cycle_time while bus.sync_mode is drive_default — the cycle time is written "
                "only with DC "
                "on; drop it, or set bus.sync_mode: dc_sync0");
        }
    }
    return core::Result<void>::ok();
}

/// Startup SDO lists (schema §4.7): well-formed, and never on a core-owned object.
template <size_t N>
[[nodiscard]] core::Result<void> check_startup_sdo(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    for (size_t s = 0; s < N; ++s) {
        const auto& list = spec.startup_sdo[s];
        const std::string where = tag + "slaves[" + std::to_string(s) + "].startup_sdo";
        if (core::Result<void> e = check_sdo_list_shape(list, where, s, spec.slave_count, kStartupSdoWords); !e) {
            return e;
        }
        if (s >= spec.slave_count) {
            continue;
        }
        for (size_t w = 0; w < list.count; ++w) {
            const auto& sdo = list.writes[w];
            const std::string item = sdo_item(where, w, sdo);
            if (core::Result<void> e = check_sdo_entry(item, sdo); !e) {
                return e;
            }
            for (const auto& [first, last] : kCoreOwnedSlaveObjects) {
                if (sdo.index >= first && sdo.index <= last) {
                    return core::Result<void>::error(
                        item +
                        " is a PDO assignment/mapping or sync-manager object — core-owned: ecrt.h reserves PDO "
                        "mapping and assignment to the master's own configuration, and the dc_* fields declare the "
                        "synchronisation");
                }
            }
            for (size_t i = 0; i < N; ++i) {
                if (spec.axes[i].slave_index != s) {
                    continue;
                }
                for (const uint16_t base : kCoreOwnedAxisObjects) {
                    if (sdo.index == base + static_cast<uint32_t>(spec.axes[i].od_offset)) {
                        return core::Result<void>::error(fmt::format(
                            "{} is axes[{}]'s 0x{:04x} (+ od_offset 0x{:04x}) — core-owned: the core writes "
                            "controlword, mode and the mapped command targets, and the limit and home objects "
                            "compete with axes[{}].limits (the core does not write those to the drive), so a startup "
                            "SDO "
                            "would be a second source",
                            item, i, base, spec.axes[i].od_offset, i));
                    }
                }
            }
        }
    }
    return core::Result<void>::ok();
}

/// Stored-value lists (schema §4.9): refused only when malformed; what a
/// value means, and whether the bus can read it, is not checked here.
template <size_t N>
[[nodiscard]] core::Result<void> check_stored_sdo(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    for (size_t s = 0; s < N; ++s) {
        const auto& list = spec.stored_sdo[s];
        const std::string where = tag + "slaves[" + std::to_string(s) + "].stored_sdo";
        if (core::Result<void> e = check_sdo_list_shape(list, where, s, spec.slave_count, kStoredSdoWords); !e) {
            return e;
        }
        if (s >= spec.slave_count) {
            continue;
        }
        const auto& startup = spec.startup_sdo[s];
        for (size_t v = 0; v < list.count; ++v) {
            const auto& sdo = list.writes[v];
            const std::string item = sdo_item(where, v, sdo);
            if (core::Result<void> e = check_sdo_entry(item, sdo); !e) {
                return e;
            }
            const auto same_object = [&](const fieldbus::ethercat::StartupSdo& other) {
                return other.index == sdo.index && other.subindex == sdo.subindex;
            };
            if (std::any_of(list.writes.begin(), list.writes.begin() + static_cast<std::ptrdiff_t>(v), same_object)) {
                return core::Result<void>::error(item + " is listed twice — one object has one expected value");
            }
            if (std::any_of(startup.writes.begin(), startup.writes.begin() + static_cast<std::ptrdiff_t>(startup.count),
                            same_object)) {
                return core::Result<void>::error(
                    item +
                    " is also written by this slave's startup_sdo — the drive holds that write until the next "
                    "power cycle, so reading it proves nothing; drop it from stored_sdo");
            }
        }
    }
    return core::Result<void>::ok();
}

/// The bus period within the envelope this release accepts.
template <size_t N>
[[nodiscard]] core::Result<void> check_cycle_time(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    if (spec.cycle_time_us < static_cast<uint32_t>(models::kMinCycleTimeUs) ||
        spec.cycle_time_us > static_cast<uint32_t>(models::kMaxCycleTimeUs)) {
        return core::Result<void>::error(
            tag + "bus.cycle_time_ns (" + std::to_string(uint64_t{spec.cycle_time_us} * 1000) +
            ") is outside the range this release accepts " + std::to_string(models::kMinCycleTimeUs * 1000) + ".." +
            std::to_string(models::kMaxCycleTimeUs * 1000) + " ns");
    }
    return core::Result<void>::ok();
}

/// A csp axis's per-cycle step: the speed a planner may command stays positive, and
/// an endless axis's largest step stays under 2^31 counts. Checked after the period
/// check it divides by.
template <size_t N>
[[nodiscard]] core::Result<void> check_csp_step(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    for (size_t i = 0; i < N; ++i) {
        if (spec.axes[i].modes_of_operation != fieldbus::OperationMode::CYCLIC_SYNC_POSITION) {
            continue;
        }
        const auto& s = spec.safety[i];
        const double cycle_s = spec.cycle_time_us * 1e-6;
        const double count_rad =
            fieldbus::ethercat::to_angle(core::units::Pulse(1), spec.axes[i].factors.position).value();
        // The cap is the limit less the count-rounding margin per cycle (drive_csp_velocity_cap):
        // at or under that margin no speed is left to command, and `plem drive check` would print
        // a velocity_cap_rad_s of 0 or less.
        if (const double cap = drive_csp_velocity_cap(spec, i); !(cap > 0.0)) {
            return core::Result<void>::error(fmt::format(
                "{}axes[{}].limits.velocity_rad_s ({}) at bus.cycle_time_ns {} leaves a planner no speed to command "
                "(velocity_cap_rad_s would be {}: the limit less one position count per cycle, two on an axis without "
                "end stops); raise limits.velocity_rad_s, lengthen bus.cycle_time_ns or use a finer position channel",
                tag, i, s.velocity_rad_s, uint64_t{spec.cycle_time_us} * 1000, cap));
        }
        // An endless csp axis: the drive takes successive 0x607A words as
        // differences modulo 2^32, so one cycle's largest command increment
        // (at the velocity limit, plus rounding) stays under half that period.
        if (axis_endless(s.position_min_rad, s.position_max_rad) &&
            !(s.velocity_rad_s * cycle_s / count_rad + 2.0 < 2147483648.0)) {
            return core::Result<void>::error(fmt::format(
                "{}axes[{}].limits.velocity_rad_s × bus.cycle_time_ns is {} position counts per cycle — an axis "
                "without end stops writes 0x607A modulo 2^32, so one cycle's step must stay under 2^31 counts; "
                "lower limits.velocity_rad_s",
                tag, i, s.velocity_rad_s * cycle_s / count_rad));
        }
    }
    return core::Result<void>::ok();
}

/// The approval enum: a drive model is approved or unapproved, never undeclared.
template <size_t N>
[[nodiscard]] core::Result<void> check_approval(const models::DriveModelSpec<N>& spec, const std::string& tag) {
    switch (spec.approval) {
        case models::Approval::APPROVED:
        case models::Approval::UNAPPROVED:
            break;
        case models::Approval::UNDECLARED:
            return core::Result<void>::error(
                tag +
                "approval is undeclared — a drive model states APPROVED or UNAPPROVED; the drive layer has no "
                "ungated mode");
        default:
            return core::Result<void>::error(tag + "approval holds an out-of-range value (" +
                                             std::to_string(static_cast<int>(spec.approval)) + ")");
    }
    return core::Result<void>::ok();
}

template <size_t N>
[[nodiscard]] core::Result<void> validate_drive_model_spec(const models::DriveModelSpec<N>& spec) {
    const std::string tag = "[" + std::string(kDriveHostTag) + "] model spec ";
    if (spec.slave_count == 0 || spec.slave_count > N) {
        return core::Result<void>::error(tag + "slave_count (" + std::to_string(spec.slave_count) + ") must be 1.." +
                                         std::to_string(N) +
                                         " — one SlaveSpec per physical drive slave, at most one per axis");
    }
    if (core::Result<void> e = check_axis_slaves(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e =
            models::check_slaves({spec.slaves.data(), spec.slave_count}, {spec.axes.data(), N}, kDriveHostTag);
        !e) {
        return e;
    }
    if (core::Result<void> e = check_factors_in_file_terms(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = models::check_axes({spec.axes.data(), N}, kDriveHostTag); !e) {
        return e;
    }
    if (core::Result<void> e = check_limits(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = check_axis_names(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = check_slave_settings(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = check_startup_sdo(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = check_stored_sdo(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = check_cycle_time(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = check_csp_step(spec, tag); !e) {
        return e;
    }
    if (core::Result<void> e = fieldbus::ethercat::validate_fault_reset(spec.fault_reset, spec.fault_reset_pacing);
        !e) {
        return core::Result<void>::error(tag + e.message());
    }
    if (core::Result<void> e = check_approval(spec, tag); !e) {
        return e;
    }
    return models::check_model_name(spec.model_name, kDriveHostTag);
}

}  // namespace plem::drive::detail
