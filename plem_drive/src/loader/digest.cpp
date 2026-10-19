// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file digest.cpp
/// @brief norm/1 serialization of DriveModelSpec<N> and its SHA-256; see
/// digest.hpp for the contract.

#include "loader/digest.hpp"

#include <charconv>
#include <concepts>
#include <cstddef>
#include <cstring>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

#include <plem/core/crypto/sha256.hpp>

namespace plem::drive::detail {
namespace {

// Every DriveModelSpec field is serialized below except three: the two the
// approval is about, `approval` (the record itself) and `expected_identity`
// (what the record claims; it comes from the record, not the declaration),
// and `stored_sdo`, which nothing executes (a warn-only expectation, schema §4.9:
// editing it must not void an approval). A field added to the spec usually
// changes this size: add it to normalize() (or
// state here why it is not part of the approval), then update the pin. It is
// a tripwire, not a proof: a bool/uint8/uint16 that lands in existing
// padding leaves the size alone, so a spec header change is reviewed with
// normalize() open. A field added to SlaveIdentity (expected_identity's
// element) moves the size too and is not a digest input; the pin then only
// asks that this comment be re-read. The number is the LP64 / libstdc++
// layout (aarch64 and x86_64 agree), so the pin is checked on LP64 only.
#if defined(__LP64__)
static_assert(sizeof(models::DriveModelSpec<1>) == 1928,
              "DriveModelSpec changed — serialize the new field in digest.cpp, then update this pin");
#endif

constexpr std::string_view kConfigNorm = "cia402_drive/v1 norm/1";
constexpr std::string_view kProfileNorm = "cia402_drive/v1 profile/1";

/// `key=value` lines. Values: decimal integers, true/false, shortest
/// round-trip reals (-0.0 → 0.0), enums by their underlying number, text
/// with `\` and line breaks escaped (the loader refuses control characters;
/// the escape keeps the line grammar injective for any value). Keys are
/// frozen with the norm version (norm/1) and may differ from the file's key
/// names and the C++ fields (`.control_word`, `.mode_of_operation`,
/// `.mode_display` are norm/1 spellings). Never rename one to match the file
/// or the code: a renamed key here changes every digest and voids every approval.
class Lines {
  public:
    void put(std::string_view key, std::string_view text) {
        out_ += key;
        out_ += '=';
        for (const char c : text) {
            if (c == '\\' || c == '\n') {
                out_ += '\\';
            }
            out_ += c == '\n' ? 'n' : c;
        }
        out_ += '\n';
    }
    void put(std::string_view key, bool value) {
        put(key, std::string_view(value ? "true" : "false"));
    }
    void put(std::string_view key, double value) {
        if (value == 0.0) {
            value = 0.0;  // -0.0 declares the same physical value
        }
        char buffer[32];
        const auto end = std::to_chars(buffer, buffer + sizeof(buffer), value).ptr;
        put(key, std::string_view(buffer, static_cast<size_t>(end - buffer)));
    }
    template <std::integral T>
        requires(!std::is_same_v<T, bool>)
    void put(std::string_view key, T value) {
        put(key, std::string_view(std::to_string(value)));
    }
    template <typename E>
        requires std::is_enum_v<E>
    void put(std::string_view key, E value) {
        put(key, static_cast<std::underlying_type_t<E>>(value));
    }
    void put_name(std::string_view key, const char* nul_terminated, size_t capacity) {
        put(key, std::string_view(nul_terminated, strnlen(nul_terminated, capacity)));
    }

    [[nodiscard]] std::string take() && {
        return std::move(out_);
    }

  private:
    std::string out_;
};

std::string at(std::string_view prefix, size_t index) {
    return std::string(prefix) + "[" + std::to_string(index) + "]";
}

void put_entry(Lines& lines, const std::string& key, const fieldbus::ethercat::PdoEntryMapping& entry) {
    lines.put(key + ".index", entry.index);
    lines.put(key + ".subindex", entry.subindex);
    lines.put(key + ".bit_length", entry.bit_length);
}

void put_profile(Lines& lines, const std::string& key, const fieldbus::Cia402TransitionProfile& profile) {
    lines.put(key + ".transition_count", profile.transition_count);
    for (size_t k = 0; k < profile.transition_count; ++k) {
        const std::string row = at(key + ".transitions", k);
        lines.put(row + ".from", profile.transitions[k].from);
        lines.put(row + ".to", profile.transitions[k].to);
        lines.put(row + ".control_word", profile.transitions[k].controlword);
    }
}

void put_slave(Lines& lines, const std::string& key, const models::SlaveSpec& slave) {
    lines.put(key + ".ring_position", slave.ring_position);
    lines.put(key + ".vendor_id", slave.vendor_id);
    lines.put(key + ".product_code", slave.product_code);
    const std::string layout = key + ".pdo_mapping";
    lines.put(layout + ".sync_count", slave.pdo_mapping.sync_count);
    for (size_t s = 0; s < slave.pdo_mapping.sync_count; ++s) {
        const auto& sync = slave.pdo_mapping.syncs[s];
        const std::string sm = at(layout + ".syncs", s);
        lines.put(sm + ".sm_index", sync.sm_index);
        lines.put(sm + ".output", sync.output);
        lines.put(sm + ".watchdog", sync.watchdog);
        lines.put(sm + ".pdo_count", sync.pdo_count);
        for (size_t p = 0; p < sync.pdo_count; ++p) {
            const auto& pdo = sync.pdos[p];
            const std::string pd = at(sm + ".pdos", p);
            lines.put(pd + ".index", pdo.index);
            lines.put(pd + ".entry_count", pdo.entry_count);
            for (size_t e = 0; e < pdo.entry_count; ++e) {
                put_entry(lines, at(pd + ".entries", e), pdo.entries[e]);
            }
        }
    }
    lines.put(key + ".dc_assign_activate", slave.dc_assign_activate);
    lines.put(key + ".dc_sync0_shift_ns", slave.dc_sync0_shift_ns);
    lines.put(key + ".dc_sync0_cycle_ns", slave.dc_sync0_cycle_ns);
    // Written only when set: a file without the key keeps its digest.
    if (slave.dc_write_cycle_time) {
        lines.put(key + ".dc_write_cycle_time", true);
    }
}

void put_startup(Lines& lines, const std::string& key, const models::StartupSdoList& list) {
    lines.put(key + ".count", list.count);
    for (size_t w = 0; w < list.count; ++w) {
        const std::string write = at(key + ".writes", w);
        lines.put(write + ".index", list.writes[w].index);
        lines.put(write + ".subindex", list.writes[w].subindex);
        lines.put(write + ".size", list.writes[w].size);
        lines.put(write + ".value", list.writes[w].value);
    }
}

void put_axis(Lines& lines, const std::string& key, const models::AxisSpec& axis) {
    lines.put(key + ".slave_index", axis.slave_index);
    // Retired field, kept as its old constant so approvals made before its removal stay valid.
    lines.put(key + ".slave_position_derived", false);
    lines.put(key + ".od_offset", axis.od_offset);
    // One gear ratio and one encoder resolution per axis: the loader writes
    // the file's one value into every internal copy, so the torque copy stands
    // for them all.
    const auto& ch = axis.factors;
    lines.put(key + ".factors.gear_ratio", ch.torque.gear_ratio);
    lines.put(key + ".factors.encoder_resolution", ch.position.encoder_resolution);
    lines.put(key + ".factors.torque.unit", ch.torque.unit);
    if (ch.torque.unit == fieldbus::ethercat::TorqueUnit::RATED_PER_MILLE) {
        lines.put(key + ".factors.torque.motor_rated_torque_nm", ch.torque.torque_constant);  // the scale is the unit's
    } else {
        lines.put(key + ".factors.torque.counts_per_amp", ch.torque.counts_per_unit);
        lines.put(key + ".factors.torque.torque_constant", ch.torque.torque_constant);
    }
    lines.put(key + ".factors.position.unit", ch.position.unit);
    lines.put(key + ".factors.velocity.unit", ch.velocity.unit);
    lines.put(key + ".factors.velocity.velocity_factor", ch.velocity.velocity_factor);
    lines.put(key + ".flip", axis.flip);
    put_profile(lines, key + ".cia402_profile", axis.cia402_profile);
    lines.put(key + ".mode_of_operation", axis.modes_of_operation);
    lines.put(key + ".mode_display", axis.mode_display_timing);
    put_entry(lines, key + ".error_code_pdo", axis.error_code_pdo);
    // Written only when declared: an axis without it keeps its digest.
    if (axis.position_before_enable_index != 0) {
        lines.put(key + ".position_before_enable_index", axis.position_before_enable_index);
    }
}

/// `key` is the axis's: zero_offset_count sits on the axis, the rest under its limits.
void put_safety(Lines& lines, const std::string& key, const models::AxisSafety& safety) {
    lines.put(key + ".zero_offset_count", safety.zero_offset_count);
    lines.put(key + ".limits.position.min_rad", safety.position_min_rad);
    lines.put(key + ".limits.position.max_rad", safety.position_max_rad);
    lines.put(key + ".limits.velocity_rad_s", safety.velocity_rad_s);
    // A csp or csv axis only (a cst axis holds 0), so cst files keep their digest.
    if (safety.acceleration_rad_s2 != 0.0) {
        lines.put(key + ".limits.acceleration_rad_s2", safety.acceleration_rad_s2);
    }
    // A cst axis only (the value gate holds every other at 0/0), so cst files keep their digest.
    if (safety.torque_min_nm != 0.0 || safety.torque_max_nm != 0.0) {
        lines.put(key + ".limits.torque_nm.min", safety.torque_min_nm);
        lines.put(key + ".limits.torque_nm.max", safety.torque_max_nm);
    }
}

template <size_t N>
std::string normalize(const models::DriveModelSpec<N>& spec, ProfileSource source) {
    Lines lines;
    lines.put("schema", kConfigNorm);
    lines.put_name("model_name", spec.model_name.data(), spec.model_name.size());
    lines.put("bus.cycle_time_ns", uint64_t{spec.cycle_time_us} * 1000);
    // The norm/1 value `sm` is the file's `drive_default` (a frozen norm/1 spelling).
    lines.put("bus.sync_mode", std::string_view(spec.dc_enable ? "dc_sync0" : "sm"));
    lines.put("fault_reset", spec.fault_reset);
    if (spec.fault_reset_pacing) {
        lines.put("fault_reset_pacing.interval_ms", spec.fault_reset_pacing->interval_ms);
        lines.put("fault_reset_pacing.max_attempts", spec.fault_reset_pacing->max_attempts);
    } else {
        lines.put("fault_reset_pacing", std::string_view("null"));
    }
    lines.put("fault_reset_pulse_ms", spec.fault_reset_pulse_ms);
    lines.put("cia402_profile.source", name_of(source));
    lines.put("slave_count", spec.slave_count);
    for (size_t s = 0; s < spec.slave_count; ++s) {
        put_slave(lines, at("slaves", s), spec.slaves[s]);
        put_startup(lines, at("slaves", s) + ".startup_sdo", spec.startup_sdo[s]);
    }
    for (size_t i = 0; i < N; ++i) {
        put_axis(lines, at("axes", i), spec.axes[i]);
        lines.put_name(at("axes", i) + ".name", spec.axis_names[i].data(), spec.axis_names[i].size());
        put_safety(lines, at("axes", i), spec.safety[i]);
    }
    return std::move(lines).take();
}

template <size_t N>
std::string normalize_profiles(const models::DriveModelSpec<N>& spec) {
    Lines lines;
    lines.put("schema", kProfileNorm);
    for (size_t i = 0; i < N; ++i) {
        put_profile(lines, at("axes", i) + ".cia402_profile", spec.axes[i].cia402_profile);
    }
    return std::move(lines).take();
}

std::string digest(std::string_view text) {
    return "sha256:" + core::crypto::sha256_hex(text);
}

}  // namespace

std::string_view name_of(ProfileSource source) noexcept {
    return source == ProfileSource::INLINE ? "inline" : "built_in";
}

DriveModelDigests drive_model_digests(const DriveModelSpecAny& spec, ProfileSource source) {
    return std::visit(
        [&](const auto& s) {
            return DriveModelDigests{.config = digest(normalize(s, source)), .profile = digest(normalize_profiles(s))};
        },
        spec);
}

std::string_view platform_version() noexcept {
    return PLEM_PLATFORM_VERSION;
}

}  // namespace plem::drive::detail
