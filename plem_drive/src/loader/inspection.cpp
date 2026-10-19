// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file inspection.cpp
/// @brief inspect_drive_model and load_drive_model_file: the file layer
/// (reader.hpp), the value gate, the approval check (approval.hpp) and the
/// warnings, then the file wrapper.

#include "plem/drive/drive_model_file.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include <plem/core/logging/logger.hpp>
#include <plem/fieldbus/ethercat/utils/servo_conversions.hpp>

#include "loader/approval.hpp"
#include "loader/digest.hpp"
#include "loader/inspection.hpp"
#include "loader/reader.hpp"
#include "models/value_gate.hpp"

namespace plem::drive {
namespace detail {
namespace {

using models::Approval;

auto& logger = core::Logger::instance();

/// One object a warning names: its channel-1 index and CiA 402 name.
struct WarnedObject {
    uint16_t base;
    std::string_view name;
};

/// How a moving axis stops (schema §4.4): the option codes and the
/// decelerations they select.
constexpr WarnedObject kStopObjects[] = {{0x605a, "Quick stop option code"},
                                         {0x605c, "Disable operation option code"},
                                         {0x6084, "Profile deceleration"},
                                         {0x6085, "Quick stop deceleration"}};

/// "0x605a (Quick stop option code), …" for the objects of `group` the
/// axis's slave does not write in startup_sdo (at the axis's channel,
/// subindex 0); empty = all declared.
template <size_t N>
std::string undeclared(const models::DriveModelSpec<N>& spec, size_t i, std::span<const WarnedObject> group) {
    const auto& axis = spec.axes[i];
    const auto& list = spec.startup_sdo[axis.slave_index];
    std::string names;
    for (const WarnedObject& object : group) {
        const auto index = static_cast<uint16_t>(object.base + axis.od_offset);
        if (std::none_of(list.writes.begin(), list.writes.begin() + list.count,
                         [&](const auto& w) { return w.index == index && w.subindex == 0; })) {
            names += fmt::format("{}0x{:04x} ({})", names.empty() ? "" : ", ", index, object.name);
        }
    }
    return names;
}

/// A csp/csv axis whose slave leaves its stop objects to the drive's stored
/// values. A declared write proves nothing about the value's fitness either:
/// the warning names only what the file leaves undeclared.
template <size_t N>
void warn_undeclared_objects(const models::DriveModelSpec<N>& spec, std::vector<std::string>& warnings) {
    for (size_t i = 0; i < N; ++i) {
        const auto mode = spec.axes[i].modes_of_operation;
        const bool csp = mode == fieldbus::OperationMode::CYCLIC_SYNC_POSITION;
        if (!csp && mode != fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY) {
            continue;
        }
        const std::string head = "axes[" + std::to_string(i) + "] (" + spec.axis_names[i].data() + ", " +
                                 (csp ? "csp" : "csv") + "): slaves[" + std::to_string(spec.axes[i].slave_index) +
                                 "].startup_sdo sets no ";
        if (const std::string names = undeclared(spec, i, kStopObjects); !names.empty()) {
            warnings.push_back(
                head + names +
                " — the drive keeps the values it has stored: 0x605c with its deceleration (usually "
                "0x6084) decides how the axis stops when PLEM disables it, and 0x605a with the deceleration it "
                "selects (0x6084 or 0x6085) how a quick stop ends (0x605e, the fault reaction, is not checked); "
                "declare them in the drive model file's startup_sdo");
        }
    }
}

/// A distributed-clock slave whose SYNC0 shift is 0 (a negative shift is a
/// phase like any other: -x fires where period - x does).
template <size_t N>
void warn_unsized_sync0_shift(const models::DriveModelSpec<N>& spec, std::vector<std::string>& warnings) {
    for (size_t s = 0; s < spec.slave_count; ++s) {
        const auto& slave = spec.slaves[s];
        if (slave.dc_assign_activate != 0 && slave.dc_sync0_shift_ns == 0) {
            warnings.push_back(
                "slaves[" + std::to_string(s) + "] (ring position " + std::to_string(slave.ring_position) +
                "): dc.sync0_shift_ns is 0 — SYNC0 then fires when the cycle starts, before that cycle's frame "
                "reaches the drive, so the drive may skip or repeat a cycle's target (torque spikes, "
                "following-error jumps); size the shift as the drive model reference says (docs/cia402-drive-model.md, "
                "SYNC0 shift)");
        }
    }
}

/// An axis with end stops whose limits.position reaches past the joint
/// positions the drive's int32 0x6064 decodes to with its zero_offset_count
/// and flip (Pulse::from_hardware saturates past them; decode_position_word
/// refuses them).
template <size_t N>
void warn_unreachable_limits(const models::DriveModelSpec<N>& spec, std::vector<std::string>& warnings) {
    constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
    constexpr int64_t kMin = std::numeric_limits<int32_t>::min();
    for (size_t i = 0; i < N; ++i) {
        const auto& s = spec.safety[i];
        if (axis_endless(s.position_min_rad, s.position_max_rad)) {
            continue;
        }
        // joint count = hw - z, or z - hw when flipped, for every int32 hw; itself an int32.
        const int64_t z = s.zero_offset_count;
        const bool flip = spec.axes[i].flip;
        const int64_t lo = std::max(kMin, flip ? z - kMax : kMin - z);
        const int64_t hi = std::min(kMax, flip ? z - kMin : kMax - z);
        const auto angle = [&](int64_t count) {
            return fieldbus::ethercat::to_angle(core::units::Pulse(static_cast<int32_t>(count)),
                                                spec.axes[i].factors.position)
                .value();
        };
        const double q_lo = angle(lo);
        const double q_hi = angle(hi);
        if (s.position_min_rad >= q_lo && s.position_max_rad <= q_hi) {
            continue;
        }
        // Rounded inward to 4 decimals, so a limit copied from the message is inside; + 0.0 turns -0 into 0.
        const auto inward = [](double q, bool up) {
            double r = (up ? std::ceil(q * 1e4) : std::floor(q * 1e4)) / 1e4;
            if (up ? r < q : r > q) {  // q * 1e4 rounded across an integer
                r += up ? 1e-4 : -1e-4;
            }
            return r + 0.0;
        };
        warnings.push_back(fmt::format(
            "axes[{}] ({}): limits.position [{}, {}] rad reaches past what the drive's int32 position (0x6064) "
            "decodes to with zero_offset_count {} and flip {}, [{:.4f}, {:.4f}] rad (rounded inward) — past it the "
            "measured position is not the joint's (the joint count saturates, or the drive's count wraps to the other "
            "end), so the limit does not hold there; narrow limits.position to within it "
            "(docs/cia402-drive-model.md, Saturation of the position)",
            i, spec.axis_names[i].data(), s.position_min_rad, s.position_max_rad, z, flip, inward(q_lo, true),
            inward(q_hi, false)));
    }
}

/// A slave identified as the synthetic example.
template <size_t N>
void warn_example_identity(const models::DriveModelSpec<N>& spec, std::vector<std::string>& warnings) {
    for (size_t s = 0; s < spec.slave_count; ++s) {
        if (spec.slaves[s].vendor_id == kExampleVendorId) {
            warnings.push_back(fmt::format(
                "slaves[{}] (ring position {}): vendor_id 0x{:08x} is the synthetic example's, whose values fit no "
                "real drive — replace the example's values with your drive's (`ethercat slaves -v`, "
                "docs/getting-started.md)",
                s, spec.slaves[s].ring_position, kExampleVendorId));
        }
    }
}

}  // namespace

// The value gate speaks the C++ spec ("[DriveHost] model spec axes[3]… repeats
// axes[0]…", "fault_reset_pacing.interval_ms"); a file author reads the same
// refusal in the file's terms ("slaves[1].axes[0]… repeats slaves[0].axes[0]…",
// "fault_reset.pacing.interval_ms"), led by the listed drives it names.
static std::string in_file_terms(const std::string& message, const FileProjection& file) {
    const std::string tag = "[" + std::string(kDriveHostTag) + "] model spec ";
    if (message.rfind(tag, 0) != 0) {
        return message;
    }
    const std::string_view text = std::string_view(message).substr(tag.size());
    std::string out;
    std::vector<std::string> drives;
    for (size_t i = 0; i < text.size();) {
        // A flat reference starts a word: not inside a path the message already spells out.
        const bool word_start = i == 0 || !(std::isalnum(static_cast<unsigned char>(text[i - 1])) ||
                                            text[i - 1] == '_' || text[i - 1] == '.' || text[i - 1] == ']');
        bool replaced = false;
        for (const auto& [flat, places] : {std::pair{std::string_view("axes["), &file.axis_places},
                                           std::pair{std::string_view("slaves["), &file.slave_places}}) {
            if (!word_start || text.substr(i, flat.size()) != flat) {
                continue;
            }
            const size_t close = text.find(']', i + flat.size());
            size_t index = 0;
            const auto digits = close == std::string_view::npos ? std::string_view()
                                                                : text.substr(i + flat.size(), close - i - flat.size());
            if (!digits.empty() &&
                std::from_chars(digits.data(), digits.data() + digits.size(), index).ptr ==
                    digits.data() + digits.size() &&
                index < places->size()) {
                const auto& place = (*places)[index];
                out += place.path;
                if (!place.drive.empty() && std::find(drives.begin(), drives.end(), place.drive) == drives.end()) {
                    drives.push_back(place.drive);
                }
                i = close + 1;
                replaced = true;
            }
            break;
        }
        if (!replaced) {
            out += text[i++];
        }
    }
    for (const auto& [cpp, key] :
         {std::pair{std::string_view("fault_reset_pacing."), std::string_view("fault_reset.pacing.")},
          std::pair{std::string_view("fault_reset_policy"), std::string_view("fault_reset.policy")}}) {
        for (size_t at = out.find(cpp); at != std::string::npos; at = out.find(cpp, at + key.size())) {
            out.replace(at, cpp.size(), key);
        }
    }
    if (drives.empty()) {
        return out;
    }
    std::string lead = drives.size() == 1 ? "for the drive at " : "for the drives at ";
    for (size_t d = 0; d < drives.size(); ++d) {
        lead += (d == 0 ? "" : d + 1 == drives.size() ? " and " : ", ") + drives[d];
    }
    return lead + ": " + out;
}

core::Result<DriveModelInspection> inspect_drive_model(std::string_view yaml, std::string_view origin) {
    using Result = core::Result<DriveModelInspection>;
    const auto tree = parse_tree(yaml);
    if (!tree) {
        return Result::error(std::string(origin) + ": " + tree.message());
    }
    auto projected = project(tree.value(), origin);
    if (!projected) {
        return Result::error(projected.message());
    }
    FileProjection file = std::move(projected).value();
    DriveModelInspection out;
    out.spec = file.spec;  // trivially copyable: a move is a copy
    out.profile_source = file.profile_source;
    out.bus_name = std::move(file.bus_name);
    logger.debug("{}", describe_mapping(origin, out.spec, out.profile_source));
    if (const core::Result<void> error =
            std::visit([](const auto& s) { return validate_drive_model_spec(s); }, out.spec);
        !error) {
        return Result::error(std::string(origin) + ": " + in_file_terms(error.message(), file));
    }
    out.digests = drive_model_digests(out.spec, out.profile_source);
    std::visit(
        [&](const auto& s) {
            warn_undeclared_objects(s, out.warnings);
            warn_unsized_sync0_shift(s, out.warnings);
            warn_unreachable_limits(s, out.warnings);
            warn_example_identity(s, out.warnings);
        },
        out.spec);
    const std::optional<RecordedApproval>& record = file.record;
    out.claimed = record ? Approval::APPROVED : Approval::UNAPPROVED;
    out.effective = Approval::UNAPPROVED;
    if (record) {
        out.identity_scope = scope_of(*record);
        const size_t slave_count = std::visit([](const auto& s) { return s.slave_count; }, out.spec);
        judge_record(record->claims, out.digests, out.spec, slave_count, out.reasons);
        judge_bench(*record, out.spec, out.reasons);
        if (out.reasons.empty()) {
            out.effective = Approval::APPROVED;
            std::visit(
                [&](auto& s) {
                    s.approval = Approval::APPROVED;
                    for (const SlaveIdentityRecord& r : record->claims.slaves) {
                        s.expected_identity[r.slave] = r.identity;
                    }
                },
                out.spec);
        }
    }
    return Result::ok(std::move(out));
}

core::Result<DriveModelSpecAny> parse_drive_model(std::string_view yaml, std::string_view origin,
                                                  bool bench_unapproved) {
    using Result = core::Result<DriveModelSpecAny>;
    auto inspected = inspect_drive_model(yaml, origin);
    if (!inspected) {
        return Result::error(inspected.message());
    }
    DriveModelInspection in = std::move(inspected).value();
    if (in.effective != in.claimed && !bench_unapproved) {
        std::string message = std::string(origin) +
                              ": approval.status is approved but this software cannot honor "
                              "it — " +
                              in.reasons.front();
        if (in.reasons.size() > 1) {
            message += " (+" + std::to_string(in.reasons.size() - 1) + " more — `plem drive check` lists them all)";
        }
        return Result::error(std::move(message));
    }
    if (bench_unapproved) {
        // A bench run uses no record: it measures the drives in front of it,
        // which may be other drives than the record's.
        std::visit(
            [](auto& spec) {
                spec.approval = models::Approval::UNAPPROVED;
                spec.expected_identity = {};
            },
            in.spec);
    }
    return Result::ok(in.spec);
}

core::Result<std::string> read_text_file(const std::string& path, std::string_view what) {
    using Result = core::Result<std::string>;
    const std::string kind(what);
    const std::string how = " — pass the " + kind +
                            "'s absolute path (a relative path resolves against the working directory; ~ is not "
                            "expanded)";
    // A permission problem is fixed with permissions, not with another path.
    const std::string readable =
        " — make the file readable and its directories searchable (x) by the user this process runs as";
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
        return Result::error("cannot load " + kind + " '" + path +
                             "': " + (ec ? ec.message() : std::string("not a regular file")) +
                             (ec == std::errc::permission_denied ? readable : how));
    }
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        const std::error_code cause(errno, std::generic_category());  // before anything else can touch errno
        return Result::error("cannot open " + kind + " '" + path + "': " + cause.message() +
                             (cause == std::errc::permission_denied ? readable : std::string()));
    }
    std::string text(kMaxDriveModelFileBytes + 1, '\0');
    in.read(text.data(), static_cast<std::streamsize>(text.size()));
    if (in.bad()) {
        return Result::error("cannot read " + kind + " '" + path + "': an I/O error stopped the read");
    }
    text.resize(static_cast<size_t>(in.gcount()));
    if (text.size() > kMaxDriveModelFileBytes) {
        return Result::error(kind + " '" + path + "' is larger than " + std::to_string(kMaxDriveModelFileBytes) +
                             " bytes: check that the path names the " + kind);
    }
    return Result::ok(std::move(text));
}

}  // namespace detail

core::Result<DriveModelSpecAny> load_drive_model_file(const std::string& path, ApprovalRecord record) {
    const auto text = detail::read_text_file(path, "drive model file");
    if (!text) {
        return core::Result<DriveModelSpecAny>::error(text.message());
    }
    return detail::parse_drive_model(text.value(), path, record == ApprovalRecord::IGNORE);
}

}  // namespace plem::drive
