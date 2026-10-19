// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file check.cpp
/// @brief `check` (`plem drive check`): a drive model file's inspection as a summary for people or,
/// with --json, the `plem-drive-check/1` document; see drive_model_tool.hpp.

#include "tool/drive_model_tool.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include "plem/drive/drive_host.hpp"

#include "bus_from_spec.hpp"
#include "drive_host_rules.hpp"
#include "loader/digest.hpp"
#include "loader/inspection.hpp"
#include "loader/reader.hpp"

namespace plem::drive::tool {

using detail::drive_bus_injection;
using detail::drive_csp_velocity_cap;
using detail::DriveBusInjection;
using detail::DriveModelInspection;
using detail::IdentityScope;
using detail::inspect_drive_model;
using detail::kDriveModelSchema;
using detail::kExampleVendorId;
using detail::platform_version;
using detail::read_text_file;
using detail::required_bench_items;

std::string_view name_of(IdentityScope scope) {
    switch (scope) {
        case IdentityScope::MODEL:
            return "model";
        case IdentityScope::UNIT:
            return "unit";
        case IdentityScope::NOT_APPLICABLE:
            break;
    }
    return "n/a";
}

bool wants_help(const std::vector<std::string>& args) {
    return std::any_of(args.begin(), args.end(), [](const std::string& a) { return a == "--help" || a == "-h"; });
}

namespace {

using models::Approval;

std::string_view name_of(Approval v) {
    return v == Approval::APPROVED ? "approved" : "unapproved";
}

/// The file's word for a mode the value gate admitted (the loader's vocabulary).
std::string_view mode_word(fieldbus::OperationMode mode) {
    switch (mode) {
        case fieldbus::OperationMode::CYCLIC_SYNC_POSITION:
            return "csp";
        case fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY:
            return "csv";
        default:
            return "cst";
    }
}

/// What the bus's startup check (initialize() Step 0) answers for this
/// file, run on the injection the bus would be handed; ok =
/// accepted. A scanned slave's mapping is checked only after the scan.
template <size_t N>
core::Result<void> bus_startup_refusal(const models::DriveModelSpec<N>& spec) {
    const auto injected = drive_bus_injection(spec);
    if (!injected) {
        return core::Result<void>::error(injected.message());
    }
    const DriveBusInjection& injection = injected.value();
    return fieldbus::ethercat::validate_bus_configuration(injection.servos, injection.startup, injection.stored,
                                                          injection.expected);
}

/// The per-axis and per-slave facts a bench runner addresses the bus with:
/// file slave index, ring position, 1-based channel and its object offset,
/// name, safety limits (null = no end stop), mode, encoder counts per motor
/// turn, flip, and a rated_per_mille axis's rated torque in 0x6076's mNm
/// (six significant digits, never rounded to whole mNm: a sub-mNm value must
/// not print as 0); ring position, mapping provenance and the [index,
/// subindex, value] writes of startup_sdo per slave, so it
/// never reads the YAML itself.
template <size_t N>
void put_layout(const models::DriveModelSpec<N>& spec, nlohmann::ordered_json& out) {
    const auto finite_or_null = [](double v) {
        return std::isfinite(v) ? nlohmann::ordered_json(v) : nlohmann::ordered_json(nullptr);
    };
    auto& axes = out["axes"] = nlohmann::ordered_json::array();
    for (size_t i = 0; i < N; ++i) {
        const auto& axis = spec.axes[i];
        const auto& safety = spec.safety[i];
        nlohmann::ordered_json rated = nullptr;
        if (axis.factors.torque.unit == fieldbus::ethercat::TorqueUnit::RATED_PER_MILLE) {
            // The ×1000 carries no binary noise (1.2704 Nm → 1270.4, not 1270.3999999999999).
            rated = std::strtod(fmt::format("{:.6g}", axis.factors.torque.torque_constant * 1000.0).c_str(), nullptr);
        }
        nlohmann::ordered_json& a = axes.emplace_back();
        a["name"] = spec.axis_names[i].data();
        a["slave"] = axis.slave_index;
        a["ring_position"] = spec.slaves[axis.slave_index].ring_position;
        a["channel"] = 1 + axis.od_offset / 0x800;
        a["od_offset"] = axis.od_offset;
        a["mode"] = std::string(mode_word(axis.modes_of_operation));
        a["position_min_rad"] = finite_or_null(safety.position_min_rad);
        a["position_max_rad"] = finite_or_null(safety.position_max_rad);
        a["velocity_limit_rad_s"] = safety.velocity_rad_s;
        // What a planner may command: on a csp axis the limit less a cycle's count rounding (the step check).
        a["velocity_cap_rad_s"] = axis.modes_of_operation == fieldbus::OperationMode::CYCLIC_SYNC_POSITION
                                      ? drive_csp_velocity_cap(spec, i)
                                      : safety.velocity_rad_s;
        // A csp or csv axis's stop deceleration; null on a cst axis.
        a["acceleration_limit_rad_s2"] = safety.acceleration_rad_s2 != 0.0
                                             ? nlohmann::ordered_json(safety.acceleration_rad_s2)
                                             : nlohmann::ordered_json(nullptr);
        // A cst axis's torque clamp; null on a csp or csv axis.
        const bool cst = axis.modes_of_operation == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
        a["torque_min_nm"] = cst ? nlohmann::ordered_json(safety.torque_min_nm) : nlohmann::ordered_json(nullptr);
        a["torque_max_nm"] = cst ? nlohmann::ordered_json(safety.torque_max_nm) : nlohmann::ordered_json(nullptr);
        a["encoder_resolution"] = axis.factors.position.encoder_resolution;
        a["flip"] = axis.flip;
        a["rated_torque_mnm"] = rated;
    }
    auto& slaves = out["slaves"] = nlohmann::ordered_json::array();
    for (size_t s = 0; s < spec.slave_count; ++s) {
        nlohmann::ordered_json& slave = slaves.emplace_back();
        slave["ring_position"] = spec.slaves[s].ring_position;
        slave["pdo_mapping"] = spec.slaves[s].pdo_mapping.sync_count == 0 ? "scanned" : "declared";
        // What the file writes at PREOP->SAFEOP (and the drive keeps in RAM):
        // a bench reading those objects before the node sees the drive's
        // stored value, not the file's. [index, subindex, value]
        auto& writes = slave["startup_sdo"] = nlohmann::ordered_json::array();
        const auto& list = spec.startup_sdo[s];
        for (size_t w = 0; w < list.count; ++w) {
            writes.push_back({list.writes[w].index, list.writes[w].subindex, list.writes[w].value});
        }
    }
}

/// `path` as a shell word: as is when it has no character a shell treats
/// specially, else single-quoted (a ' inside written '\\''); a leading '-'
/// gets ./ so a command does not read the path as an option.
std::string shell_word(const std::string& file) {
    std::string path = file.starts_with('-') ? "./" + file : file;
    const bool plain = !path.empty() && std::all_of(path.begin(), path.end(), [](unsigned char c) {
        return std::isalnum(c) != 0 ||
               std::string_view("@%+=:,./-_").find(static_cast<char>(c)) != std::string_view::npos;
    });
    if (plain) {
        return path;
    }
    std::string quoted = "'";
    for (const char c : path) {
        quoted += c == '\'' ? std::string("'\\''") : std::string(1, c);
    }
    return quoted + "'";
}

/// "1 axis", "2 axes".
std::string count(size_t n, std::string_view one, std::string_view many) {
    return std::to_string(n) + " " + std::string(n == 1 ? one : many);
}

/// What a person reads from check's JSON: the bus, its axes, the approval
/// (with each reason it does not hold), the warnings and the one next step;
/// `example` = a slave carries the synthetic example's identity.
std::string summary(const nlohmann::ordered_json& j, const std::string& file, bool example) {
    const auto text = [](const nlohmann::ordered_json& v) { return v.get<std::string>(); };
    const std::string path = shell_word(file);
    std::string out = "bus " + text(j.at("bus_name")) + ": " + count(j.at("axes").size(), "axis", "axes") + " on " +
                      count(j.at("slaves").size(), "drive", "drives") + "\n";
    size_t width = 0;
    for (const auto& axis : j.at("axes")) {
        width = std::max(width, text(axis.at("name")).size());
    }
    for (const auto& axis : j.at("axes")) {
        const std::string name = text(axis.at("name"));
        out += "  " + name + std::string(width - name.size(), ' ') + "  " + text(axis.at("mode")) + "  ring " +
               std::to_string(axis.at("ring_position").get<int>()) + " channel " +
               std::to_string(axis.at("channel").get<int>()) + "\n";
    }
    const bool approved = j.at("approval_effective") == "approved";
    if (approved && j.at("identity_scope") == "unit") {
        out += "approval: approved for this unit — the record holds its serial number\n";
    } else if (approved) {  // schema §6 Scope
        out +=
            "approval: approved for the model — a serial number in the record is 0 or null, so any drive that "
            "matches the identity values the record holds opens motion too\n";
    } else if (j.at("reasons").empty()) {
        out += "approval: not approved\n";
    } else {
        out += "approval: not approved — the approval the file claims does not hold:\n";
        for (const auto& reason : j.at("reasons")) {
            out += "  " + text(reason) + "\n";
        }
    }
    for (const auto& warning : j.at("warnings")) {
        out += "warning: " + text(warning) + "\n";
    }
    if (approved) {
        out += "next: plem drive run " + path + "\n";
    } else if (!j.at("approve_blocker").is_null()) {
        out += "next: fix this first, or the bench cannot record its approval: " + text(j.at("approve_blocker")) + "\n";
    } else if (example) {
        out +=
            "next: replace the example's values with your drive's (`ethercat slaves -v`, docs/getting-started.md), "
            "then check the file again\n";
    } else {
        out += "next: plem drive run " + path +
               " — on a terminal it offers the bench at the drives, which records the approval\n";
    }
    return out + "details: plem drive check --json " + path + "\n";
}

}  // namespace

std::string tool_usage() {
    return "usage: plem drive check [--json] <model.yaml>\n"
           "\n"
           "Reads a drive model file the way the bus does, refuses what the bus would refuse at startup (PDO\n"
           "layout, channels, per-slave agreement) and a file without bus.name, and prints a summary: the bus,\n"
           "its axes, the approval (each reason a claimed one does not hold), the warnings and the next step.\n"
           "\n"
           "--json prints one JSON document instead, for scripts\n"
           "(\"format\": \"plem-drive-check/1\"): the bus name, digests, release, approval claimed/effective,\n"
           "every reason an approved claim is not honored, every warning (objects a csp/csv axis leaves to the\n"
           "drive's stored values, a zero SYNC0 shift, position limits past what 0x6064 decodes to, the synthetic\n"
           "example's vendor_id — never a refusal), approve_blocker (null, or why the bench's approval step would\n"
           "refuse), what a bench runner addresses the bus with (cycle and command-stale times, fault-reset\n"
           "policy, required bench items, axes, slaves) and not_judged.\n"
           "Exit 0 when effective == claimed, 1 otherwise or on a refusal (printed on stderr).\n";
}

int tool_check(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    if (wants_help(args)) {
        out << tool_usage();
        return kToolOk;
    }
    const bool as_json = std::find(args.begin(), args.end(), "--json") != args.end();
    std::vector<std::string> files;
    std::copy_if(args.begin(), args.end(), std::back_inserter(files),
                 [](const std::string& a) { return a != "--json"; });
    if (files.size() != 1) {
        err << tool_usage();
        return kToolUsage;
    }
    const std::string& path = files[0];
    const auto text = read_text_file(path, "drive model file");
    if (!text) {
        err << text.message() << "\n";
        return kToolRefused;
    }
    const auto inspected = inspect_drive_model(text.value(), path);
    if (!inspected) {
        err << inspected.message() << "\n";
        return kToolRefused;
    }
    const DriveModelInspection& in = inspected.value();
    if (!in.bus_name) {  // optional to the loader; a bus plem drive run starts is named by it
        err << path
            << ": bus.name is missing — name the bus: bus: { name: <bus>, cycle_time_ns: …, sync_mode: … } "
               "(letters, digits and _, not starting with a digit); plem drive run starts it as /<bus>\n";
        return kToolRefused;
    }
    if (const core::Result<void> refusal = std::visit([](const auto& s) { return bus_startup_refusal(s); }, in.spec);
        !refusal) {
        err << path << ": the bus would refuse this file at startup — " << refusal.message()
            << " (slave n = the slave at ring position n)\n";
        for (const std::string& reason : in.reasons) {
            err << "and the approval it claims does not hold: " << reason << "\n";
        }
        return kToolRefused;
    }
    nlohmann::ordered_json json;
    json["format"] = std::string(kCheckFormat);
    json["schema"] = std::string(kDriveModelSchema);
    json["bus_name"] = *in.bus_name;
    json["config_digest"] = in.digests.config;
    json["profile_digest"] = in.digests.profile;
    json["profile_source"] = std::string(name_of(in.profile_source));
    json["implementation_revision"] = std::string(platform_version());
    json["approval_claimed"] = std::string(name_of(in.claimed));
    json["approval_effective"] = std::string(name_of(in.effective));
    json["identity_scope"] = in.identity_scope == IdentityScope::NOT_APPLICABLE
                                 ? nlohmann::ordered_json(nullptr)
                                 : nlohmann::ordered_json(std::string(name_of(in.identity_scope)));
    json["reasons"] = in.reasons;
    json["warnings"] = in.warnings;
    const std::string blocker = approve_blocker(text.value(), path);
    json["approve_blocker"] = blocker.empty() ? nlohmann::ordered_json(nullptr) : nlohmann::ordered_json(blocker);
    std::visit(
        [&](const auto& spec) {
            json["cycle_time_ns"] = uint64_t{spec.cycle_time_us} * 1000;
            json["command_stale_timeout_ns"] = kCommandStaleTimeoutNs;
            json["fault_reset_policy"] =
                spec.fault_reset == fieldbus::ethercat::FaultResetPolicy::NONE ? "none" : "bounded_auto_reset";
            const auto required = required_bench_items(in.spec);
            json["required_bench_items"] = std::vector<std::string>(required.begin(), required.end());
            put_layout(spec, json);
        },
        in.spec);
    json["not_judged"] =
        "a scanned slave's PDO mapping is read from the drive when the bus starts and judged then, with the same "
        "rules as a declared one; the live slave identity (SII serial, 0x1009, 0x100A, scanned PDO mapping digest), "
        "fault-reset pacing against the period and the DC sync0 rules are judged at startup too; the drive accepts "
        "or refuses its startup SDOs (startup_sdo, and the 0x1C32:02 written for dc.write_cycle_time) at "
        "PREOP->SAFEOP; a SYNC0 cycle it cannot run is refused at PREOP->SAFEOP, or later as a sync error that drops "
        "the slave out of OP after it reached OP (documented drive behaviour, not a guarantee for every drive); both "
        "show in the kernel log";
    const bool example = std::visit(
        [](const auto& spec) {
            return std::any_of(spec.slaves.begin(), spec.slaves.begin() + spec.slave_count,
                               [](const models::SlaveSpec& slave) { return slave.vendor_id == kExampleVendorId; });
        },
        in.spec);
    out << (as_json ? json.dump(2) + "\n" : summary(json, path, example));
    return in.effective == in.claimed ? kToolOk : kToolRefused;
}

}  // namespace plem::drive::tool
