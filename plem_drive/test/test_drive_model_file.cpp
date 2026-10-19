// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_model_file.cpp
/// @brief The drive model file: the example golden, the table-driven
/// walk over two roots (every key deleted, an unknown key in every mapping),
/// the conditional keys, the scalar grammar and widths, the file-only rules
/// and the file wrapper. Every refusal goes through one helper, which also
/// proves the message speaks the file's language, never a C++ spelling a
/// file author cannot act on.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include <unistd.h>

#include <fmt/format.h>

#include <plem/core/crypto/sha256.hpp>
#include <plem/fieldbus/ethercat/pdo_scan.hpp>
#include <plem/fieldbus/ethercat/utils/servo_conversions.hpp>
#include <plem/fieldbus/test/cia402_connection_fixture.hpp>
#include <plem/fieldbus/test/ethercat_test_probes.hpp>
#include <plem/fieldbus/test/single_axis_cst_layout.hpp>

#include "bus_from_spec.hpp"
#include "drive_bench_fixture.hpp"
#include "drive_model_spec_fixture.hpp"
#include "loader/digest.hpp"
#include "loader/inspection.hpp"
#include "loader/reader.hpp"
#include "models/value_gate.hpp"
#include "plem/drive/drive_host.hpp"
#include "plem/drive/drive_model_file.hpp"

namespace plem::drive {
namespace {

using detail::ProfileSource;
using detail::YamlNode;
using models::Approval;
using models::DriveModelSpec;
using test::bench_body;
using test::bench_yaml;
using test::indented;
using test::kAllPass;
using test::kSlave0;
using test::kUnapprovedBlock;
using test::ScratchDir;
using Loaded = core::Result<DriveModelSpecAny>;

/// C++ spellings a file author cannot act on; no refusal may carry one.
constexpr std::string_view kCppSymbols[] = {
    "kVendorId",     "kPdoLayout",         "kCia402Profile",          "kServoSpecs", "kModeDisplayTiming",
    "DeviceServo::", "ICiA402Bus::",       "Cia402TransitionProfile", "SlaveSpec",   "[DriveHost]",
    "NONE",          "BOUNDED_AUTO_RESET", "fault_reset_pacing",
};

/// Every loader refusal here: refused; for a file-layer refusal, the key path
/// leads the message; and no C++ spelling in it. An empty `key_path` marks a
/// value-gate refusal (it names the file path too: slaves[e].axes[a]).
template <typename R>
void expect_file_rejection(const R& result, std::string_view key_path) {
    ASSERT_FALSE(result) << "accepted — expected a refusal at '" << key_path << "'";
    const std::string& message = result.message();
    if (!key_path.empty()) {
        EXPECT_NE(message.find(": " + std::string(key_path) + " "), std::string::npos) << message;
    }
    for (const std::string_view symbol : kCppSymbols) {
        EXPECT_EQ(message.find(symbol), std::string::npos) << symbol << " in: " << message;
    }
}

Loaded parse(std::string_view yaml) {
    return detail::parse_drive_model(yaml, "test.yaml");
}

YamlNode tree(std::string_view yaml) {
    auto parsed = detail::parse_tree(yaml);
    EXPECT_TRUE(parsed) << parsed.message();
    return parsed ? std::move(parsed).value() : YamlNode{};
}

/// Every occurrence of `from` (at least one) becomes `to`.
std::string edited(std::string text, std::string_view from, std::string_view to) {
    size_t count = 0;
    for (size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
        text.replace(at, from.size(), to);
        ++count;
    }
    EXPECT_GT(count, 0u) << "nothing to edit: " << from;
    return text;
}

/// `yaml` with every cst axis turned csp: the mode word, and the torque
/// limit (a cst axis's key only) traded for the stop deceleration (a csp or
/// csv axis's).
std::string as_csp(const std::string& yaml) {
    return edited(edited(yaml, "mode: cst", "mode: csp"), ", torque_nm: { min: -30.0, max: 40.0 } }",
                  ", acceleration_rad_s2: 4.0 }");
}

std::string child(const std::string& path, std::string_view key) {
    return path.empty() ? std::string(key) : path + "." + std::string(key);
}

/// Visits every mapping of the tree in file order with its key path.
void each_mapping(YamlNode& node, const std::string& path,
                  const std::function<void(YamlNode&, const std::string&)>& visit) {
    if (node.kind == YamlNode::Kind::MAPPING) {
        visit(node, path);
        for (auto& [key, value] : node.members) {
            each_mapping(value, child(path, key), visit);
        }
    } else if (node.kind == YamlNode::Kind::SEQUENCE) {
        for (size_t i = 0; i < node.items.size(); ++i) {
            each_mapping(node.items[i], path + "[" + std::to_string(i) + "]", visit);
        }
    }
}

/// `edit` applied to the mapping at `path` of a copy of `root`, projected.
core::Result<detail::FileProjection> project_edited(const YamlNode& root, const std::string& path,
                                                    const std::function<void(YamlNode&)>& edit) {
    YamlNode copy = root;
    size_t hits = 0;
    each_mapping(copy, "", [&](YamlNode& mapping, const std::string& at) {
        if (at == path) {
            edit(mapping);
            ++hits;
        }
    });
    EXPECT_EQ(hits, 1u) << "no mapping at " << path;
    return detail::project(copy, "test.yaml");
}

void erase_key(YamlNode& mapping, std::string_view key) {
    std::erase_if(mapping.members, [&](const auto& member) { return member.first == key; });
}

// The files the rows start from

std::string example_text() {
    std::ifstream in(DRIVE_EXAMPLE_YAML);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// One single-channel CST slave body (kSingleAxisCstLayout), axes key last.
constexpr std::string_view kOneChannelSlave = R"(    vendor_id: 0x00c0ffee
    product_code: 0x00001001
    pdo_mapping:
      sync_managers:
        - { sm: 0, direction: output, watchdog: false, pdos: [] }
        - { sm: 1, direction: input, watchdog: false, pdos: [] }
        - sm: 2
          direction: output
          watchdog: true
          pdos:
            - index: 0x1600
              entries:
                - { index: 0x6040, subindex: 0, bits: 16 }
                - { index: 0x607a, subindex: 0, bits: 32 }
                - { index: 0x60ff, subindex: 0, bits: 32 }
                - { index: 0x6071, subindex: 0, bits: 16 }
                - { index: 0x6060, subindex: 0, bits: 8 }
        - sm: 3
          direction: input
          watchdog: false
          pdos:
            - index: 0x1a00
              entries:
                - { index: 0x6041, subindex: 0, bits: 16 }
                - { index: 0x6064, subindex: 0, bits: 32 }
                - { index: 0x606c, subindex: 0, bits: 32 }
                - { index: 0x6077, subindex: 0, bits: 16 }
                - { index: 0x6061, subindex: 0, bits: 8 }
    startup_sdo: []
    axes:
)";

/// The per-axis values make_n_drives() declares (name, channel and offset are
/// written per axis).
constexpr std::string_view kAxisBody = R"(        mode: cst
        mode_display_timing: pre_enable
        factors:
          gear_ratio: 100.0
          encoder_resolution: 65536
          torque: { unit: scaled_current, counts_per_amp: 50.0, torque_constant: 0.12 }
          position: { unit: rotary_motor_encoder }
          velocity: { unit: rotary_motor_encoder, velocity_factor: 1.0 }
        flip: false
        error_code_pdo: null
        limits: { position: { min_rad: -1.0, max_rad: 1.5 }, velocity_rad_s: 2.0, torque_nm: { min: -30.0, max: 40.0 } }
)";

/// Slaves at ring positions 0, 1, … carrying `axes_per_slave` axes each
/// (channels 1, 2, …); joint i is `j<i>`. With one axis per slave it is the
/// file make_n_drives<N>() describes.
std::string drives_yaml(const std::vector<size_t>& axes_per_slave) {
    std::string yaml =
        "schema: cia402_drive/v1\nmodel: { name: bench_drive }\nbus: { cycle_time_ns: 1000000, sync_mode: "
        "drive_default }\n"
        "fault_reset: { policy: none, pulse_ms: 0 }\nslaves:";
    yaml += axes_per_slave.empty() ? " []\n" : "\n";
    size_t joint = 0;
    for (size_t s = 0; s < axes_per_slave.size(); ++s) {
        yaml += "  - ring_position: " + std::to_string(s) + "\n" + std::string(kOneChannelSlave);
        if (axes_per_slave[s] == 0) {
            yaml.replace(yaml.size() - 1, 1, " []\n");
        }
        for (size_t a = 0; a < axes_per_slave[s]; ++a, ++joint) {
            yaml += "      - name: j" + std::to_string(joint) + "\n        channel: " + std::to_string(a + 1) +
                    "\n        zero_offset_count: " + std::to_string(test::kAxisZeroOffsets[joint % 8]) + "\n" +
                    std::string(kAxisBody);
        }
    }
    return yaml + std::string(kUnapprovedBlock);
}

std::string one_drive_yaml() {
    return drives_yaml({1});
}

/// one_drive_yaml() with the slave's layout replaced by the scanned policy.
std::string scanned_one_drive_yaml() {
    std::string yaml = one_drive_yaml();
    const size_t from = yaml.find("    pdo_mapping:\n");
    const size_t to = yaml.find("    startup_sdo: []");
    EXPECT_NE(from, std::string::npos);
    EXPECT_NE(to, std::string::npos);
    return yaml.replace(from, to - from, "    pdo_mapping: scanned\n");
}

/// Every value-carrying key form the example leaves out: a non-empty startup
/// SDO list, a stored-value list, DC switched on with every slave's dc block, bounded auto reset
/// with its pacing, an ErrorCode role declared as a mapping, and the drive's
/// own transition table (the 12 rows of cia402_connection_fixture.hpp).
constexpr std::string_view kMaximal = R"(schema: cia402_drive/v1
model: { name: maximal }
bus: { cycle_time_ns: 500000, sync_mode: dc_sync0 }
fault_reset:
  policy: bounded_auto_reset
  pacing: { interval_ms: 500, max_attempts: 3 }
  pulse_ms: 20
cia402_profile:
  - { from: not_ready_to_switch_on, to: ready_to_switch_on, controlword: 0x0006 }
  - { from: switch_on_disabled, to: ready_to_switch_on, controlword: 0x0006 }
  - { from: ready_to_switch_on, to: switched_on, controlword: 0x0007 }
  - { from: ready_to_switch_on, to: switch_on_disabled, controlword: 0x0000 }
  - { from: switched_on, to: operation_enabled, controlword: 0x000f }
  - { from: switched_on, to: ready_to_switch_on, controlword: 0x0006 }
  - { from: switched_on, to: switch_on_disabled, controlword: 0x0000 }
  - { from: operation_enabled, to: switched_on, controlword: 0x0007 }
  - { from: operation_enabled, to: ready_to_switch_on, controlword: 0x0006 }
  - { from: operation_enabled, to: switch_on_disabled, controlword: 0x0000 }
  - { from: operation_enabled, to: quick_stop_active, controlword: 0x0002 }
  - { from: quick_stop_active, to: switch_on_disabled, controlword: 0x0000 }
slaves:
  - ring_position: 7
    vendor_id: 0x00c0ffee
    product_code: 0x00001001
    pdo_mapping:
      sync_managers:
        - { sm: 0, direction: output, watchdog: false, pdos: [] }
        - sm: 2
          direction: output
          watchdog: true
          pdos:
            - index: 0x1600
              entries:
                - { index: 0x6040, subindex: 0, bits: 16 }
                - { index: 0x6060, subindex: 0, bits: 8 }
                - { index: 0x6071, subindex: 0, bits: 16 }
        - sm: 3
          direction: input
          watchdog: false
          pdos:
            - index: 0x1a00
              entries:
                - { index: 0x6041, subindex: 0, bits: 16 }
                - { index: 0x6061, subindex: 0, bits: 8 }
                - { index: 0x6064, subindex: 0, bits: 32 }
                - { index: 0x606c, subindex: 0, bits: 32 }
                - { index: 0x6077, subindex: 0, bits: 16 }
                - { index: 0x603f, subindex: 0, bits: 16 }
    dc: { assign_activate: 0x0300, sync0_cycle_ns: 500000, sync0_shift_ns: -1000 }
    startup_sdo:
      - { index: 0x2000, subindex: 1, size: 1, value: 0xaa }
      - { index: 0x2001, subindex: 0, size: 4, value: 0xffffffff }
    stored_sdo:
      - { index: 0x3015, subindex: 0, size: 2, value: 2 }
    axes:
      - name: joint_a
        channel: 1
        mode: cst
        mode_display_timing: post_enable
        factors:
          gear_ratio: 100.0
          encoder_resolution: 65536
          torque: { unit: scaled_current, counts_per_amp: 50.0, torque_constant: 0.12 }
          position: { unit: rotary_motor_encoder }
          velocity: { unit: rotary_motor_encoder, velocity_factor: 1.0 }
        flip: true
        zero_offset_count: -5
        error_code_pdo: { index: 0x603f, subindex: 0, bits: 16 }
        limits: { position: { min_rad: -1.0, max_rad: 1.5 }, velocity_rad_s: 2.0, torque_nm: { min: -30.0, max: 40.0 } }
approval:
  status: unapproved
  record: null
)";
constexpr std::string_view kMaximalDc =
    "    dc: { assign_activate: 0x0300, sync0_cycle_ns: 500000, sync0_shift_ns: -1000 }\n";
constexpr std::string_view kMaximalPacing = "  pacing: { interval_ms: 500, max_attempts: 3 }\n";

template <size_t N>
const DriveModelSpec<N>& as(const Loaded& loaded) {
    const auto* spec = std::get_if<DriveModelSpec<N>>(&loaded.value());
    EXPECT_NE(spec, nullptr) << "loaded with a different axis count";
    static const DriveModelSpec<N> none{};
    return spec != nullptr ? *spec : none;
}

/// Whole-value comparison, reported field by field so a failure names it.
template <size_t N>
void expect_same(const DriveModelSpec<N>& actual, const DriveModelSpec<N>& expected) {
    for (size_t k = 0; k < N; ++k) {
        EXPECT_TRUE(actual.slaves[k] == expected.slaves[k]) << "slaves[" << k << "]";
        EXPECT_TRUE(actual.startup_sdo[k] == expected.startup_sdo[k]) << "startup_sdo[" << k << "]";
        EXPECT_TRUE(actual.stored_sdo[k] == expected.stored_sdo[k]) << "stored_sdo[" << k << "]";
        EXPECT_TRUE(actual.axes[k] == expected.axes[k]) << "axes[" << k << "]";
        EXPECT_TRUE(actual.safety[k] == expected.safety[k]) << "safety[" << k << "]";
        EXPECT_EQ(actual.axis_names[k], expected.axis_names[k]) << "axis_names[" << k << "]";
    }
    EXPECT_TRUE(actual == expected) << "a model-wide field differs";
}

/// What drive_synthetic.yaml declares, built from the spec the value-gate
/// pins start from: three slaves at {5, 0, 2}, joints 0/1 on slave 5 with
/// their channels reversed, ErrorCode mapped on the two-channel slaves.
DriveModelSpec<5> expected_example() {
    namespace connection = fieldbus::test::connection;
    auto spec = test::make_n_drives<5>();
    const models::SlaveSpec two_channel{.ring_position = 5,
                                        .vendor_id = fieldbus::test::kSyntheticVendorId,
                                        .product_code = fieldbus::test::kSyntheticProductCode,
                                        .pdo_mapping = models::to_pdo_mapping_spec(connection::kLayout)};
    spec.slave_count = 3;
    spec.slaves = {};
    spec.slaves[0] = two_channel;
    spec.slaves[1] = spec.slaves[0];
    spec.slaves[1].ring_position = 0;
    spec.slaves[1].pdo_mapping = models::to_pdo_mapping_spec(fieldbus::test::kSingleAxisCstLayout);
    spec.slaves[2] = two_channel;
    spec.slaves[2].ring_position = 2;
    constexpr size_t kSlaveOf[] = {0, 0, 1, 2, 2};
    constexpr uint16_t kOdOffset[] = {0x800, 0, 0, 0, 0x800};
    for (size_t i = 0; i < 5; ++i) {
        spec.axes[i].slave_index = kSlaveOf[i];
        spec.axes[i].od_offset = kOdOffset[i];
        spec.axes[i].error_code_pdo =
            kSlaveOf[i] == 1 ? fieldbus::ethercat::PdoEntryMapping{0, 0, 0} : connection::kErrorCodePdo;
    }
    spec.approval = Approval::UNAPPROVED;
    spec.model_name = {};
    const char name[] = "synthetic_3slave_cst";
    std::memcpy(spec.model_name.data(), name, sizeof(name));
    return spec;
}

// Golden

TEST(DriveModelFile, TheExampleLoadsToItsGolden) {
    const auto loaded = load_drive_model_file(DRIVE_EXAMPLE_YAML);
    ASSERT_TRUE(loaded) << loaded.message();
    expect_same(as<5>(loaded), expected_example());
}

// An axis runs in the mode its file declares; the three
// cyclic synchronous modes are the vocabulary, nothing else is guessed.
TEST(DriveModelFile, AnAxisModeIsCstCspOrCsv) {
    const std::pair<std::string_view, fieldbus::OperationMode> modes[] = {
        {"cst", fieldbus::OperationMode::CYCLIC_SYNC_TORQUE},
        {"csp", fieldbus::OperationMode::CYCLIC_SYNC_POSITION},
        {"csv", fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY},
    };
    for (const auto& [word, mode] : modes) {
        SCOPED_TRACE(word);
        const std::string file = edited(one_drive_yaml(), "mode: cst", "mode: " + std::string(word));
        const auto loaded =
            parse(mode == fieldbus::OperationMode::CYCLIC_SYNC_TORQUE
                      ? file
                      : edited(file, ", torque_nm: { min: -30.0, max: 40.0 } }", ", acceleration_rad_s2: 4.0 }"));
        ASSERT_TRUE(loaded) << loaded.message();
        EXPECT_EQ(as<1>(loaded).axes[0].modes_of_operation, mode);
    }
    expect_file_rejection(parse(edited(one_drive_yaml(), "mode: cst", "mode: pp")), "slaves[0].axes[0].mode");
}

TEST(DriveModelFile, MapsEveryAxisIndexToItsPlaceInTheFile) {
    EXPECT_EQ(detail::describe_mapping("drive_synthetic.yaml", expected_example(), ProfileSource::BUILT_IN),
              "drive model file drive_synthetic.yaml: model 'synthetic_3slave_cst', cia402_profile built-in (12 "
              "rows); axes[0] = slaves[0].axes[0] (j0), axes[1] = slaves[0].axes[1] (j1), axes[2] = "
              "slaves[1].axes[0] (j2), axes[3] = slaves[2].axes[0] (j3), axes[4] = slaves[2].axes[1] (j4)");
}

TEST(DriveModelFile, TheApprovalKeyIsOptionalAndShapedByItsStatus) {
    const std::string base = one_drive_yaml();
    const auto loaded = parse(base);
    ASSERT_TRUE(loaded) << loaded.message();
    EXPECT_EQ(as<1>(loaded).approval, Approval::UNAPPROVED);
    // No key = unapproved, the same as `status: unapproved` / `record: null`.
    const auto absent = parse(edited(base, kUnapprovedBlock, ""));
    ASSERT_TRUE(absent) << absent.message();
    EXPECT_EQ(as<1>(absent).approval, Approval::UNAPPROVED);
    expect_file_rejection(parse(edited(base, kUnapprovedBlock, "approval: null\n")), "approval");
    // approved needs the record; unapproved carries the plain word null and nothing else.
    const auto approved_null = parse(edited(base, "status: unapproved", "status: approved"));
    expect_file_rejection(approved_null, "approval.record");
    EXPECT_NE(approved_null.message().find("is null while approval.status is approved"), std::string::npos)
        << approved_null.message();
    const auto unapproved_record = parse(edited(base, "record: null", "record: { config_digest: \"x\" }"));
    expect_file_rejection(unapproved_record, "approval.record");
    EXPECT_NE(unapproved_record.message().find("is a mapping while approval.status is unapproved"), std::string::npos)
        << unapproved_record.message();
    expect_file_rejection(parse(edited(base, "record: null", "record: ~")), "approval.record");
    expect_file_rejection(parse(edited(base, "status: unapproved", "status: pending")), "approval.status");
    // Other status words, e.g. verified and unverified, are refused with the valid list (no alias).
    for (const std::string_view old : {"verified", "unverified"}) {
        const auto refused = parse(edited(base, "status: unapproved", "status: " + std::string(old)));
        expect_file_rejection(refused, "approval.status");
        EXPECT_NE(refused.message().find("('" + std::string(old) + "') is not one of: unapproved, approved"),
                  std::string::npos)
            << refused.message();
    }
}

// bus.name: the bus's ROS name (plem drive run starts it as /<name>).
// Optional to the loader (the robot tier's files carry none; plem drive check
// and run require it), a ROS name when present, and outside the digest:
// renaming a bus keeps its approval.
TEST(DriveModelFile, BusNameIsAnOptionalRosNameOutsideTheDigest) {
    const std::string base = one_drive_yaml();
    const std::string bus = "bus: { cycle_time_ns: 1000000, sync_mode: drive_default }";
    const auto named = [&](std::string_view name) {
        return edited(base, bus,
                      "bus: { name: " + std::string(name) + ", cycle_time_ns: 1000000, sync_mode: drive_default }");
    };
    const auto without = detail::inspect_drive_model(base, "test.yaml");
    ASSERT_TRUE(without) << without.message();
    EXPECT_FALSE(without.value().bus_name.has_value());
    for (const std::string_view name : {"pair_a", "_pair_b2"}) {
        SCOPED_TRACE(name);
        const auto with = detail::inspect_drive_model(named(name), "test.yaml");
        ASSERT_TRUE(with) << with.message();
        EXPECT_EQ(with.value().bus_name, std::string(name));
        EXPECT_EQ(with.value().digests, without.value().digests) << "the name is not part of the approval";
    }
    for (const std::string_view bad : {"2pair", "a-b", "\"a/b\"", "\"\"", "\"a b\""}) {
        SCOPED_TRACE(bad);
        const auto refused = parse(named(bad));
        expect_file_rejection(refused, "bus.name");
        EXPECT_NE(refused.message().find("letters, digits and _, not starting with a digit"), std::string::npos)
            << refused.message();
    }
}

TEST(DriveModelFile, LoadsEveryAxisCountAndRefusesTheRest) {
    auto one = test::make_n_drives<1>();
    one.approval = Approval::UNAPPROVED;
    const auto loaded_one = parse(drives_yaml({1}));
    ASSERT_TRUE(loaded_one) << loaded_one.message();
    expect_same(as<1>(loaded_one), one);

    auto eight = test::make_n_drives<8>();
    eight.approval = Approval::UNAPPROVED;
    const auto loaded_eight = parse(drives_yaml(std::vector<size_t>(8, 1)));
    ASSERT_TRUE(loaded_eight) << loaded_eight.message();
    expect_same(as<8>(loaded_eight), eight);

    expect_file_rejection(parse(drives_yaml({5, 4})), "slaves");
    expect_file_rejection(parse(drives_yaml({})), "slaves");
    // An empty axes list is refused before the axes are counted, so it wins
    // over the count: one slave with none, and a later empty slave.
    expect_file_rejection(parse(drives_yaml({0})), "slaves[0].axes");
    expect_file_rejection(parse(drives_yaml({1, 0})), "slaves[1].axes");
    expect_file_rejection(parse(drives_yaml({2, 2, 0})), "slaves[2].axes");
}

// Table-driven walk over two roots

/// Keys whose absence means something (schema §1); their own rows cover
/// them: dc and pacing below, a missing cia402_profile in the golden,
/// stored_sdo in its own section, approval in TheApprovalKeyIsOptional…,
/// bus.name in BusNameIsAnOptionalRosName…, the position and velocity unit in
/// ThePositionAndVelocityUnitMayBeLeftOut, error_code_pdo in
/// AMissingErrorCodePdoIsTheMappedErrorCode and mode_display_timing in
/// AMissingModeDisplayTimingIsPreEnable.
bool has_its_own_rows(const std::string& path, std::string_view key) {
    const bool motion_unit = key == "unit" && (path.ends_with(".position") || path.ends_with(".velocity"));
    return key == "dc" || key == "pacing" || key == "cia402_profile" || key == "stored_sdo" || key == "approval" ||
           (path == "bus" && key == "name") || key == "error_code_pdo" || key == "mode_display_timing" ||
           (path.ends_with(".factors") && key == "position") || motion_unit;
}

/// `root` loads as is; then every key deleted one at a time, and an unknown
/// key added to every mapping, is refused with that key's path.
void walk(const YamlNode& root) {
    ASSERT_TRUE(detail::project(root, "test.yaml")) << "the root itself must load";
    std::vector<std::pair<std::string, std::vector<std::string>>> mappings;
    YamlNode scan = root;
    each_mapping(scan, "", [&](YamlNode& mapping, const std::string& path) {
        std::vector<std::string> keys;
        for (const auto& member : mapping.members) {
            keys.push_back(member.first);
        }
        mappings.emplace_back(path, std::move(keys));
    });
    size_t rows = 0;
    for (const auto& [path, keys] : mappings) {
        for (const auto& key : keys) {
            if (has_its_own_rows(path, key)) {
                continue;
            }
            SCOPED_TRACE("without " + child(path, key));
            expect_file_rejection(project_edited(root, path, [&](YamlNode& m) { erase_key(m, key); }),
                                  child(path, key));
            ++rows;
        }
        SCOPED_TRACE("zz_unknown in " + (path.empty() ? std::string("the file") : path));
        expect_file_rejection(project_edited(root, path,
                                             [](YamlNode& m) {
                                                 YamlNode zero;
                                                 zero.scalar = "0";
                                                 zero.plain = true;
                                                 m.members.emplace_back("zz_unknown", std::move(zero));
                                             }),
                              child(path, "zz_unknown"));
        ++rows;
    }
    EXPECT_GT(rows, 2 * mappings.size()) << "the walk barely visited anything";
}

TEST(DriveModelFileWalk, TheExampleRefusesEveryMissingOrUnknownKey) {
    walk(tree(example_text()));
}

TEST(DriveModelFileWalk, TheMaximalFileRefusesEveryMissingOrUnknownKey) {
    const auto loaded = parse(kMaximal);
    ASSERT_TRUE(loaded) << loaded.message();
    walk(tree(kMaximal));
}

TEST(DriveModelFileWalk, AMissingKeySaysToWriteIt) {
    const auto loaded = project_edited(tree(example_text()), "model", [](YamlNode& m) { erase_key(m, "name"); });
    EXPECT_EQ(loaded.message(),
              "test.yaml: model.name is missing — write it (schema §1 lists the keys that may be left out, "
              "and the few whose absence is written as null or [])");
}

// Conditional blocks (schema §1: a switched-off feature takes none)

void expect_value_gate_refusal(const Loaded& result, std::string_view says) {
    expect_file_rejection(result, "");
    EXPECT_NE(result.message().find(says), std::string::npos) << result.message();
}

TEST(DriveModelFile, ConditionalBlocksFollowTheirSwitch) {
    const std::string maximal(kMaximal);
    // Switch on, block missing: refused by the file, in the file's keys.
    const auto no_dc = parse(edited(maximal, kMaximalDc, ""));
    expect_file_rejection(no_dc, "slaves[0].dc");
    EXPECT_NE(no_dc.message().find("is missing — bus.sync_mode: dc_sync0"), std::string::npos) << no_dc.message();
    const auto no_pacing = parse(edited(maximal, kMaximalPacing, ""));
    expect_file_rejection(no_pacing, "fault_reset.pacing");
    EXPECT_NE(no_pacing.message().find("is missing — fault_reset.policy: bounded_auto_reset"), std::string::npos)
        << no_pacing.message();
    // A block its switch does not use.
    expect_file_rejection(parse(edited(maximal, "sync_mode: dc_sync0", "sync_mode: drive_default")), "slaves[0].dc");
    // The pacing's own values are checked by the value gate, in the file's keys.
    expect_value_gate_refusal(parse(edited(maximal, "interval_ms: 500", "interval_ms: 0")),
                              "fault_reset.pacing.interval_ms is 0");
    expect_value_gate_refusal(parse(edited(maximal, "max_attempts: 3", "max_attempts: 0")),
                              "fault_reset.pacing.max_attempts (0) is outside");
    const auto stray_pacing = parse(edited(maximal, "policy: bounded_auto_reset", "policy: none"));
    expect_file_rejection(stray_pacing, "fault_reset.pacing");
    EXPECT_NE(stray_pacing.message().find("is declared while fault_reset.policy is none"), std::string::npos)
        << stray_pacing.message();
    // null is not a way to leave a block out; the refusal says what is.
    for (const auto& [block, as_null, path] : {std::tuple{kMaximalDc, "    dc: null\n", "slaves[0].dc"},
                                               std::tuple{kMaximalPacing, "  pacing: null\n", "fault_reset.pacing"}}) {
        const auto loaded = parse(edited(maximal, block, as_null));
        expect_file_rejection(loaded, path);
        EXPECT_NE(loaded.message().find("null neither declares nor leaves out a block"), std::string::npos)
            << loaded.message();
        EXPECT_NE(loaded.message().find("write its values while its switch is on (dc: bus.sync_mode dc_sync0; pacing: "
                                        "fault_reset.policy bounded_auto_reset), drop the key while it is off"),
                  std::string::npos)
            << loaded.message();
    }
}

TEST(DriveModelFile, TheMaximalFileCarriesEveryValueForm) {
    const auto loaded = parse(kMaximal);
    ASSERT_TRUE(loaded) << loaded.message();
    const auto& spec = as<1>(loaded);
    EXPECT_TRUE(spec.dc_enable);
    EXPECT_EQ(spec.slaves[0].dc_assign_activate, 0x0300);
    EXPECT_EQ(spec.slaves[0].dc_sync0_cycle_ns, 500000u);
    EXPECT_EQ(spec.slaves[0].dc_sync0_shift_ns, -1000);
    EXPECT_EQ(spec.fault_reset, fieldbus::ethercat::FaultResetPolicy::BOUNDED_AUTO_RESET);
    EXPECT_TRUE(spec.fault_reset_pacing ==
                (fieldbus::ethercat::FaultResetPacing{.interval_ms = 500, .max_attempts = 3}));
    EXPECT_EQ(spec.fault_reset_pulse_ms, 20u);
    ASSERT_EQ(spec.startup_sdo[0].count, 2u);
    EXPECT_TRUE(spec.startup_sdo[0].writes[0] ==
                (fieldbus::ethercat::StartupSdo{.index = 0x2000, .subindex = 1, .size = 1, .value = 0xAA}));
    EXPECT_TRUE(spec.startup_sdo[0].writes[1] ==
                (fieldbus::ethercat::StartupSdo{.index = 0x2001, .subindex = 0, .size = 4, .value = 0xFFFFFFFF}));
    ASSERT_EQ(spec.stored_sdo[0].count, 1u);
    EXPECT_TRUE(spec.stored_sdo[0].writes[0] ==
                (fieldbus::ethercat::StartupSdo{.index = 0x3015, .subindex = 0, .size = 2, .value = 2}));
    EXPECT_TRUE(spec.axes[0].error_code_pdo ==
                (fieldbus::ethercat::PdoEntryMapping{.index = 0x603F, .subindex = 0, .bit_length = 16}));
    EXPECT_TRUE(spec.axes[0].flip);
    EXPECT_EQ(spec.axes[0].mode_display_timing, fieldbus::ModeDisplayTiming::POST_ENABLE);
    EXPECT_EQ(spec.safety[0].zero_offset_count, -5);
    EXPECT_EQ(spec.slaves[0].ring_position, 7);
    EXPECT_EQ(spec.cycle_time_us, 500u);
    EXPECT_TRUE(spec.axes[0].cia402_profile == fieldbus::test::connection::kCia402Profile);
    EXPECT_FALSE(spec.slaves[0].dc_write_cycle_time) << "absent = the core writes no 0x1C32";
}

// ── dc.write_cycle_time (schema §5): the drive takes 0x1C32:02 from the master ─

/// kMaximal's dc block with `write_cycle_time` set to `value` (a YAML scalar).
std::string maximal_writing_cycle_time(std::string_view value) {
    return edited(std::string(kMaximal), "sync0_shift_ns: -1000 }",
                  "sync0_shift_ns: -1000, write_cycle_time: " + std::string(value) + " }");
}

TEST(DriveModelFile, WriteCycleTimeTrueReachesTheSlave) {
    const auto loaded = parse(maximal_writing_cycle_time("true"));
    ASSERT_TRUE(loaded) << loaded.message();
    EXPECT_TRUE(as<1>(loaded).slaves[0].dc_write_cycle_time);
}

TEST(DriveModelFile, WriteCycleTimeTakesOnlyTrue) {
    // One meaning, one spelling: leaving the key out is "do not write".
    const auto refused = parse(maximal_writing_cycle_time("false"));
    expect_file_rejection(refused, "slaves[0].dc.write_cycle_time");
    EXPECT_NE(refused.message().find("leave the key out"), std::string::npos) << refused.message();
    for (const std::string_view word : {"1", "yes", "\"true\"", "null"}) {
        SCOPED_TRACE(word);
        expect_file_rejection(parse(maximal_writing_cycle_time(word)), "slaves[0].dc.write_cycle_time");
    }
    // The dc block, and so the key, exists only under dc_sync0.
    expect_file_rejection(
        parse(edited(maximal_writing_cycle_time("true"), "sync_mode: dc_sync0", "sync_mode: drive_default")),
        "slaves[0].dc");
}

/// Keeps what the assembly injected, then stops create() at initialize()
/// (this target has no master).
class InjectionProbeBus : public fieldbus::test::EtherCATBusTestProbe {
  public:
    using fieldbus::test::EtherCATBusTestProbe::EtherCATBusTestProbe;
    core::Result<void> initialize() override {
        servos = injected_servo_config();
        return core::Result<void>::error("probe: no master");
    }
    std::vector<fieldbus::ethercat::ServoInfo> servos;
};

TEST(DriveModelFile, WriteCycleTimeTravelsFromTheFileToTheBus) {
    // Every hop (file, slave spec, the axis's ServoInfo, the bus) keeps the flag.
    for (const bool declared : {true, false}) {
        SCOPED_TRACE(declared ? "declared" : "absent");
        const auto loaded = parse(declared ? maximal_writing_cycle_time("true") : std::string(kMaximal));
        ASSERT_TRUE(loaded) << loaded.message();
        auto bus = std::make_shared<InjectionProbeBus>(0u);
        ASSERT_FALSE(DriveHost<1>::create(as<1>(loaded), bus, DriveHostParams{})) << "the probe refuses initialize()";
        ASSERT_EQ(bus->servos.size(), 1u);
        EXPECT_EQ(bus->servos[0].dc_write_cycle_time, declared);
    }
}

// The drive's own transition table

TEST(DriveModelFileProfile, SaysTheTableCameFromTheFile) {
    // The debug mapping line and the inspection report the same source.
    const auto inspected = detail::inspect_drive_model(kMaximal, "m.yaml");
    ASSERT_TRUE(inspected) << inspected.message();
    EXPECT_EQ(inspected.value().profile_source, ProfileSource::INLINE);
    EXPECT_EQ(detail::inspect_drive_model(one_drive_yaml(), "d.yaml").value().profile_source, ProfileSource::BUILT_IN);
    EXPECT_EQ(detail::describe_mapping("m.yaml", inspected.value().spec, inspected.value().profile_source),
              "drive model file m.yaml: model 'maximal', cia402_profile inline (12 rows); axes[0] = "
              "slaves[0].axes[0] (joint_a)");
}

TEST(DriveModelFileProfile, RefusesWhatATableCannotBe) {
    const std::string maximal(kMaximal);
    constexpr std::string_view kFirst =
        "  - { from: not_ready_to_switch_on, to: ready_to_switch_on, controlword: 0x0006 }\n";
    constexpr std::string_view kLast = "  - { from: quick_stop_active, to: switch_on_disabled, controlword: 0x0000 }\n";
    expect_file_rejection(parse(edited(maximal, "from: not_ready_to_switch_on", "from: ready")),
                          "cia402_profile[0].from");
    expect_file_rejection(parse(edited(maximal, "to: quick_stop_active", "to: QUICK_STOP_ACTIVE")),
                          "cia402_profile[10].to");
    expect_value_gate_refusal(parse(edited(maximal, kFirst, std::string(kFirst) + std::string(kFirst))),
                              "duplicate the same (from, to) pair");
    // Bit 7 is the value gate's refusal, worded for whoever wrote the row.
    expect_value_gate_refusal(parse(edited(maximal, "to: quick_stop_active, controlword: 0x0002",
                                           "to: quick_stop_active, controlword: 0x0082")),
                              "carries controlword bit 7");
    // A command the walk issues but maps elsewhere, and one it does not issue at all.
    expect_value_gate_refusal(parse(edited(maximal, "from: switched_on, to: switch_on_disabled, controlword: 0x0000",
                                           "from: switched_on, to: switch_on_disabled, controlword: 0x0006")),
                              "this walk maps that command to");
    expect_value_gate_refusal(
        parse(edited(maximal, "from: switch_on_disabled, to: ready_to_switch_on, controlword: 0x0006",
                     "from: switch_on_disabled, to: ready_to_switch_on, controlword: 0x000f")),
        "not one the standard walk issues");
    // Seventeen rows: refused on the count, before any row is read.
    std::string seventeen(kLast);
    for (int i = 0; i < 5; ++i) {
        seventeen += kLast;
    }
    expect_file_rejection(parse(edited(maximal, kLast, seventeen)), "cia402_profile");
    expect_file_rejection(parse(one_drive_yaml() + "cia402_profile: []\n"), "cia402_profile");
    const auto null_table = project_edited(tree(kMaximal), "", [](YamlNode& root) {
        for (auto& [key, value] : root.members) {
            if (key == "cia402_profile") {
                value = YamlNode{};
                value.scalar = "null";
                value.plain = true;
            }
        }
    });
    expect_file_rejection(null_table, "cia402_profile");
    EXPECT_NE(null_table.message().find("null does not select the built-in table"), std::string::npos);
    // A misspelt key cannot pass for an omitted one (and fall back to the built-in table).
    expect_file_rejection(parse(edited(maximal, "cia402_profile:\n", "cia402_profle:\n")), "cia402_profle");
}

// Scalars (grammar and widths)

struct Edit {
    std::string_view from;
    std::string to;
    std::string_view path;
};

void expect_edits_refused(const std::string& base, std::initializer_list<Edit> rows) {
    for (const auto& row : rows) {
        SCOPED_TRACE(row.to);
        expect_file_rejection(parse(edited(base, row.from, row.to)), row.path);
    }
}

std::string repeated(std::string_view item, size_t count) {
    std::string out;
    for (size_t i = 0; i < count; ++i) {
        out += (i == 0 ? "" : ", ") + std::string(item);
    }
    return out;
}

TEST(DriveModelFileScalars, IntegersAreDecimalOrHexAndNeverWrap) {
    constexpr std::string_view kVendor = "vendor_id: 0x00c0ffee";
    constexpr std::string_view kVendorPath = "slaves[0].vendor_id";
    constexpr std::string_view kOffset = "zero_offset_count: 1234";
    constexpr std::string_view kOffsetPath = "slaves[0].axes[0].zero_offset_count";
    expect_edits_refused(one_drive_yaml(), {
                                               {kVendor, "vendor_id: +137", kVendorPath},
                                               {kVendor, "vendor_id: 0XC0FFEE", kVendorPath},
                                               {kVendor, "vendor_id: 1_000", kVendorPath},
                                               {kVendor, "vendor_id: 0137", kVendorPath},  // YAML 1.1 octal
                                               {kVendor, "vendor_id: -1", kVendorPath},    // an unsigned field
                                               // no sign, even on zero (0 is a valid position)
                                               {"ring_position: 0\n", "ring_position: -0\n", "slaves[0].ring_position"},
                                               {kVendor, "vendor_id: 137.0", kVendorPath},
                                               {kVendor, "vendor_id: \"137\"", kVendorPath},  // quoted = text
                                               {kVendor, "vendor_id: 0x100000000", kVendorPath},
                                               {kVendor, "vendor_id: 4294967296", kVendorPath},
                                               // past 64 bits: refused, never wrapped to 0 (0 is a legal offset)
                                               {kOffset, "zero_offset_count: 0x10000000000000000", kOffsetPath},
                                               {kOffset, "zero_offset_count: 18446744073709551616", kOffsetPath},
                                               {kOffset, "zero_offset_count: -0x10", kOffsetPath},
                                               {kOffset, "zero_offset_count: -2147483649", kOffsetPath},
                                               {kOffset, "zero_offset_count: 2147483648", kOffsetPath},
                                               // inside 64 bits but past int64: refused, never cast to -1 or 1
                                               {kOffset, "zero_offset_count: 0xffffffffffffffff", kOffsetPath},
                                               {kOffset, "zero_offset_count: -18446744073709551615", kOffsetPath},
                                               // magnitude 2^63: refused before conversion to int64_t or negation.
                                               // Negating INT64_MIN would have undefined behavior
                                               // (detectable by UBSan).
                                               {kOffset, "zero_offset_count: -9223372036854775808", kOffsetPath},
                                           });
}

TEST(DriveModelFileScalars, EachWidthTakesItsWholeRange) {
    const std::string base = one_drive_yaml();
    const auto with_sdo = [&](std::string_view write) {
        return edited(base, "startup_sdo: []", "startup_sdo: [" + std::string(write) + "]");
    };
    const auto widest = parse(edited(edited(with_sdo("{ index: 0xffff, subindex: 255, size: 4, value: 0xffffffff }"),
                                            "vendor_id: 0x00c0ffee", "vendor_id: 4294967295"),
                                     "zero_offset_count: 1234", "zero_offset_count: -2147483648"));
    ASSERT_TRUE(widest) << widest.message();
    EXPECT_TRUE(as<1>(widest).startup_sdo[0].writes[0] ==
                (fieldbus::ethercat::StartupSdo{.index = 0xFFFF, .subindex = 255, .size = 4, .value = 0xFFFFFFFF}));
    EXPECT_EQ(as<1>(widest).slaves[0].vendor_id, 0xFFFFFFFFu);
    EXPECT_EQ(as<1>(widest).safety[0].zero_offset_count, std::numeric_limits<int32_t>::min());
    const auto top = parse(edited(base, "zero_offset_count: 1234", "zero_offset_count: 2147483647"));
    ASSERT_TRUE(top) << top.message();
    EXPECT_EQ(as<1>(top).safety[0].zero_offset_count, std::numeric_limits<int32_t>::max());

    expect_edits_refused(with_sdo("{ index: 0x2000, subindex: 1, size: 1, value: 0 }"),
                         {
                             {"subindex: 1, size", "subindex: 256, size", "slaves[0].startup_sdo[0].subindex"},
                             {"subindex: 1, size", "subindex: -1, size", "slaves[0].startup_sdo[0].subindex"},
                             {"index: 0x2000", "index: 0x10000", "slaves[0].startup_sdo[0].index"},
                             {"value: 0 }", "value: 0x100000000 }", "slaves[0].startup_sdo[0].value"},
                         });
}

TEST(DriveModelFileScalars, RealsAreJsonNumbersAndFinite) {
    const std::string base = one_drive_yaml();
    constexpr std::string_view kVelocity = "velocity_rad_s: 2.0";
    constexpr std::string_view kPath = "slaves[0].axes[0].limits.velocity_rad_s";
    expect_edits_refused(base, {
                                   {kVelocity, "velocity_rad_s: 010", kPath},
                                   {kVelocity, "velocity_rad_s: +1.0", kPath},
                                   {kVelocity, "velocity_rad_s: .5", kPath},
                                   {kVelocity, "velocity_rad_s: 1.", kPath},
                                   {kVelocity, "velocity_rad_s: 0x2", kPath},
                                   {kVelocity, "velocity_rad_s: 1e999", kPath},
                                   {kVelocity, "velocity_rad_s: .inf", kPath},
                                   {kVelocity, "velocity_rad_s: nan", kPath},
                                   {kVelocity, "velocity_rad_s: \"2.0\"", kPath},
                               });
    EXPECT_NE(parse(edited(base, kVelocity, "velocity_rad_s: 1e999")).message().find("within double range"),
              std::string::npos);
    for (const std::string_view accepted :
         {"velocity_rad_s: 2", "velocity_rad_s: 2e0", "velocity_rad_s: 20E-1", "velocity_rad_s: 0.2e+1"}) {
        const auto loaded = parse(edited(base, kVelocity, accepted));
        ASSERT_TRUE(loaded) << accepted << ": " << loaded.message();
        EXPECT_DOUBLE_EQ(as<1>(loaded).safety[0].velocity_rad_s, 2.0) << accepted;
    }
}

TEST(DriveModelFileScalars, BooleansAreTrueOrFalse) {
    constexpr std::string_view kFlip = "flip: false";
    constexpr std::string_view kPath = "slaves[0].axes[0].flip";
    expect_edits_refused(one_drive_yaml(), {
                                               {kFlip, "flip: yes", kPath},
                                               {kFlip, "flip: False", kPath},
                                               {kFlip, "flip: 0", kPath},
                                               {kFlip, "flip: \"false\"", kPath},
                                           });
}

TEST(DriveModelFileScalars, TextIsQuotedWhenItWouldReadAsAnythingElse) {
    const std::string base = one_drive_yaml();
    constexpr std::string_view kName = "name: j0";
    constexpr std::string_view kPath = "slaves[0].axes[0].name";
    expect_edits_refused(base, {
                                   {kName, "name: 12", kPath},
                                   {kName, "name: -1.5", kPath},
                                   {kName, "name: 0x1f", kPath},
                                   {kName, "name: true", kPath},
                                   {kName, "name: null", kPath},
                                   {kName, "name: ~", kPath},
                                   {kName, "name:", kPath},
                                   {kName, "name: \"j\\0x\"", kPath},  // a NUL inside
                               });
    const auto quoted = parse(edited(base, kName, "name: \"j0\""));
    EXPECT_TRUE(quoted) << quoted.message();
}

TEST(DriveModelFileScalars, NamesHoldThirtyOneBytesAndAreNeverTruncated) {
    const std::string base = one_drive_yaml();
    const std::string fits(31, 'a');
    const std::string over(32, 'a');
    const auto axis = parse(edited(base, "name: j0", "name: " + fits));
    EXPECT_TRUE(axis) << axis.message();
    const auto model = parse(edited(base, "name: bench_drive", "name: " + fits));
    EXPECT_TRUE(model) << model.message();
    expect_file_rejection(parse(edited(base, "name: j0", "name: " + over)), "slaves[0].axes[0].name");
    expect_file_rejection(parse(edited(base, "name: bench_drive", "name: " + over)), "model.name");
}

// Rules only a file can break

TEST(DriveModelFile, ChannelsCountFromOneAndAreRangeCheckedBeforeTheMultiply) {
    const std::string base = one_drive_yaml();
    const auto second = parse(edited(base, "channel: 1", "channel: 2"));
    ASSERT_TRUE(second) << second.message();
    EXPECT_EQ(as<1>(second).axes[0].od_offset, 0x800);
    const auto last = parse(edited(base, "channel: 1", "channel: 8"));
    ASSERT_TRUE(last) << last.message();
    EXPECT_EQ(as<1>(last).axes[0].od_offset, 0x3800);
    constexpr std::string_view kPath = "slaves[0].axes[0].channel";
    expect_edits_refused(base, {
                                   {"channel: 1", "channel: 0", kPath},
                                   {"channel: 1", "channel: 9", kPath},
                                   // (channel - 1) * 0x800 would wrap to 0 in 64 bits
                                   {"channel: 1", "channel: 0x20000000000001", kPath},
                               });
}

TEST(DriveModelFile, RefusesWhatOnlyAFileCanGetWrong) {
    const std::string base = one_drive_yaml();
    expect_edits_refused(base,
                         {
                             {"vendor_id: 0x00c0ffee", "vendor_id: 0", "slaves[0].vendor_id"},
                             {"product_code: 0x00001001", "product_code: 0", "slaves[0].product_code"},
                             {"name: bench_drive", "name: \"   \"", "model.name"},
                             // an ASCII control character (here a line break)
                             {"name: bench_drive", "name: \"bench\\ndrive\"", "model.name"},
                             {"error_code_pdo: null", "error_code_pdo: { index: 0, subindex: 0, bits: 16 }",
                              "slaves[0].axes[0].error_code_pdo.index"},
                             // null is the plain word null, not ~, an empty value or a quoted "null"
                             {"error_code_pdo: null", "error_code_pdo: ~", "slaves[0].axes[0].error_code_pdo"},
                             {"error_code_pdo: null", "error_code_pdo:", "slaves[0].axes[0].error_code_pdo"},
                             {"error_code_pdo: null", "error_code_pdo: \"null\"", "slaves[0].axes[0].error_code_pdo"},
                             {"schema: cia402_drive/v1", "schema: cia402_drive/v2", "schema"},
                             {"schema: cia402_drive/v1", "schema: plem.cia402.drive/0-draft", "schema"},
                             {"sync_mode: drive_default }", "sync_mode: drive_default, <<: {} }", "bus.<<"},
                         });
    EXPECT_NE(parse(edited(base, "schema: cia402_drive/v1", "schema: cia402_drive/v2"))
                  .message()
                  .find("this loader reads cia402_drive/v1"),
              std::string::npos);
    expect_file_rejection(project_edited(tree(base), "slaves[0].pdo_mapping",
                                         [](YamlNode& m) { m.members.front().second.items.clear(); }),
                          "slaves[0].pdo_mapping.sync_managers");
    const std::string syntax = parse("a: [").message();
    EXPECT_EQ(syntax.rfind("test.yaml: line ", 0), 0u) << syntax;
    EXPECT_NE(syntax.find("YAML syntax error"), std::string::npos) << syntax;
}

// bus.cycle_time_ns is the ETG unit (0x1C32:02, ns) on the core's 1 us grid:
// a value off the grid is refused, never rounded, and the envelope speaks ns.
TEST(DriveModelFile, TheCycleTimeIsNanosecondsOnTheMicrosecondGrid) {
    const std::string base = one_drive_yaml();
    constexpr std::string_view kCycle = "cycle_time_ns: 1000000";
    const auto quarter = parse(edited(base, kCycle, "cycle_time_ns: 250000"));
    ASSERT_TRUE(quarter) << quarter.message();
    EXPECT_EQ(as<1>(quarter).cycle_time_us, 250u);
    const auto off_grid = parse(edited(base, kCycle, "cycle_time_ns: 1000500"));
    expect_file_rejection(off_grid, "bus.cycle_time_ns");
    EXPECT_NE(off_grid.message().find("(1000500) is not a whole number of microseconds"), std::string::npos)
        << off_grid.message();
    expect_edits_refused(base, {
                                   {kCycle, "cycle_time_ns: -1000", "bus.cycle_time_ns"},
                                   {kCycle, "cycle_time_ns: 4294967296000", "bus.cycle_time_ns"},
                               });
    expect_value_gate_refusal(
        parse(edited(base, kCycle, "cycle_time_ns: 100000")),
        "bus.cycle_time_ns (100000) is outside the range this release accepts 200000..1000000 ns");
}

// bus.sync_mode: drive_default (PLEM turns no DC on; the drive's own
// synchronization setting applies) or dc_sync0; free run and anything else
// are refused.
TEST(DriveModelFile, TheSyncModeIsDriveDefaultOrDcSync0) {
    const std::string base = one_drive_yaml();
    const auto drive_default = parse(base);
    ASSERT_TRUE(drive_default) << drive_default.message();
    EXPECT_FALSE(as<1>(drive_default).dc_enable);
    for (const std::string_view word : {"free_run", "dc", "sync0", "true"}) {
        SCOPED_TRACE(word);
        const auto refused = parse(edited(base, "sync_mode: drive_default", "sync_mode: " + std::string(word)));
        expect_file_rejection(refused, "bus.sync_mode");
        EXPECT_NE(refused.message().find("is not one of: drive_default, dc_sync0"), std::string::npos)
            << refused.message();
    }
}

// A factor the value gate refuses is named by its file key, never by the
// robot tier's channel wording (" channel (", counts_per_unit …).
TEST(DriveModelFile, AFactorRefusalSpeaksTheFilesKeys) {
    const std::tuple<std::string_view, std::string_view, std::string_view> rows[] = {
        {"gear_ratio: 100.0", "gear_ratio: 0", "axes[0].factors.gear_ratio (0) must be finite and > 0"},
        {"counts_per_amp: 50.0", "counts_per_amp: -1", "axes[0].factors.torque.counts_per_amp (-1) must be"},
        {"torque_constant: 0.12", "torque_constant: 0", "axes[0].factors.torque.torque_constant (0) must be"},
        {"velocity_factor: 1.0", "velocity_factor: 0", "axes[0].factors.velocity.velocity_factor (0) must be"},
        {"encoder_resolution: 65536", "encoder_resolution: 0", "axes[0].factors.encoder_resolution is 0"},
    };
    for (const auto& [from, to, says] : rows) {
        SCOPED_TRACE(to);
        const auto refused = parse(edited(one_drive_yaml(), from, to));
        expect_value_gate_refusal(refused, says);
        for (const std::string_view old : {" channel (", "counts_per_unit"}) {
            EXPECT_EQ(refused.message().find(old), std::string::npos) << old << " in: " << refused.message();
        }
    }
}

// torque.unit: rated_per_mille (the CiA 402 standard unit)

/// one_drive_yaml() with its torque declared in per mille of a 1.27 Nm motor rated torque.
std::string per_mille_one_drive_yaml() {
    return edited(one_drive_yaml(), "{ unit: scaled_current, counts_per_amp: 50.0, torque_constant: 0.12 }",
                  "{ unit: rated_per_mille, motor_rated_torque_nm: 1.27 }");
}

TEST(DriveModelFile, APerMilleTorqueLoadsAsScale1000AndTheRatedTorque) {
    const auto loaded = parse(per_mille_one_drive_yaml());
    ASSERT_TRUE(loaded) << loaded.message();
    const fieldbus::ethercat::TorqueChannel expected{fieldbus::ethercat::TorqueUnit::RATED_PER_MILLE, 1000.0, 1.27,
                                                     fieldbus::test::kSyntheticServoSpec.gear_ratio};
    EXPECT_TRUE(as<1>(loaded).axes[0].factors.torque == expected);
    // Every key the unit picks is mandatory, and nothing else is a key.
    walk(tree(per_mille_one_drive_yaml()));
}

TEST(DriveModelFile, APerMilleTorqueTakesOnlyItsOwnKeys) {
    const std::string base = per_mille_one_drive_yaml();
    const auto with_counts =
        parse(edited(base, "motor_rated_torque_nm: 1.27", "motor_rated_torque_nm: 1.27, counts_per_amp: 50.0"));
    expect_file_rejection(with_counts, "slaves[0].axes[0].factors.torque.counts_per_amp");
    EXPECT_NE(with_counts.message().find("is not a key of this schema"), std::string::npos) << with_counts.message();
    expect_file_rejection(parse(edited(base, ", motor_rated_torque_nm: 1.27", "")),
                          "slaves[0].axes[0].factors.torque.motor_rated_torque_nm");
    for (const std::string_view value : {"0", "-1.27"}) {
        SCOPED_TRACE(value);
        const auto refused =
            parse(edited(base, "motor_rated_torque_nm: 1.27", "motor_rated_torque_nm: " + std::string(value)));
        expect_value_gate_refusal(refused, "axes[0].factors.torque.motor_rated_torque_nm (" + std::string(value) +
                                               ") must be finite and > 0");
        for (const std::string_view old : {"torque_constant", "counts_per_unit", "counts_per_amp"}) {
            EXPECT_EQ(refused.message().find(old), std::string::npos) << old << " in: " << refused.message();
        }
    }
}

TEST(DriveModelFile, RefusesWhatTheSpecCannotHold) {
    const std::string base = one_drive_yaml();
    constexpr std::string_view kMailbox = "{ sm: 0, direction: output, watchdog: false, pdos: [] }";
    const std::string sm0 = "{ sm: 0, direction: output, watchdog: false, pdos: [";
    const std::string gap = "{ index: 0, subindex: 0, bits: 8 }";
    expect_edits_refused(
        base, {
                  {kMailbox, std::string(kMailbox) + "\n        - " + std::string(kMailbox),
                   "slaves[0].pdo_mapping.sync_managers"},
                  {kMailbox, sm0 + repeated("{ index: 0x1601, entries: [" + gap + "] }", 5) + "] }",
                   "slaves[0].pdo_mapping.sync_managers[0].pdos"},
                  {kMailbox, sm0 + "{ index: 0x1601, entries: [" + repeated(gap, 17) + "] }] }",
                   "slaves[0].pdo_mapping.sync_managers[0].pdos[0].entries"},
                  {"startup_sdo: []",
                   "startup_sdo: [" + repeated("{ index: 0x2000, subindex: 0, size: 1, value: 0 }", 17) + "]",
                   "slaves[0].startup_sdo"},
                  {"startup_sdo: []",
                   "startup_sdo: []\n    stored_sdo: [" +
                       repeated("{ index: 0x2000, subindex: 0, size: 1, value: 0 }", 17) + "]",
                   "slaves[0].stored_sdo"},
              });
}

// stored_sdo (values the drive must have stored, schema §4.9)

/// one_drive_yaml() with `items` as the slave's stored_sdo list.
std::string stored_one_drive_yaml(std::string_view items) {
    return edited(one_drive_yaml(), "    startup_sdo: []\n",
                  "    startup_sdo: []\n    stored_sdo: [" + std::string(items) + "]\n");
}

TEST(DriveModelFile, StoredSdoIsOptionalAndReadsAsTheStartupElement) {
    EXPECT_EQ(as<1>(parse(one_drive_yaml())).stored_sdo[0].count, 0u) << "left out = nothing is read";
    const auto loaded = parse(stored_one_drive_yaml(
        "{ index: 0x3401, subindex: 0, size: 4, value: 0 }, { index: 0x3015, subindex: 0, size: 2, value: 2 }"));
    ASSERT_TRUE(loaded) << loaded.message();
    const auto& list = as<1>(loaded).stored_sdo[0];
    ASSERT_EQ(list.count, 2u);
    EXPECT_TRUE(list.writes[0] ==
                (fieldbus::ethercat::StartupSdo{.index = 0x3401, .subindex = 0, .size = 4, .value = 0}));
    EXPECT_TRUE(list.writes[1] ==
                (fieldbus::ethercat::StartupSdo{.index = 0x3015, .subindex = 0, .size = 2, .value = 2}));
    EXPECT_EQ(as<1>(loaded).startup_sdo[0].count, 0u) << "the two lists stay apart";
}

TEST(DriveModelFile, StoredSdoRefusesWhatIsMalformed) {
    const auto empty = parse(stored_one_drive_yaml(""));
    expect_file_rejection(empty, "slaves[0].stored_sdo");
    EXPECT_NE(empty.message().find("is empty — leave the key out to compare nothing"), std::string::npos)
        << empty.message();
    expect_file_rejection(
        parse(edited(one_drive_yaml(), "    startup_sdo: []\n", "    startup_sdo: []\n    stored_sdo: null\n")),
        "slaves[0].stored_sdo");
    expect_file_rejection(parse(stored_one_drive_yaml("{ index: 0x3015, subindex: 0, size: 2, value: 2, width: 2 }")),
                          "slaves[0].stored_sdo[0].width");
    expect_file_rejection(parse(stored_one_drive_yaml("{ index: 0x3015, subindex: 0, size: 2, value: 0x100000000 }")),
                          "slaves[0].stored_sdo[0].value");
    expect_value_gate_refusal(parse(stored_one_drive_yaml("{ index: 0x3015, subindex: 0, size: 1, value: 0x100 }")),
                              "slaves[0].stored_sdo[0] (0x3015:00) has size 1 and value 0x100");
    expect_value_gate_refusal(
        parse(stored_one_drive_yaml(
            "{ index: 0x3015, subindex: 0, size: 2, value: 2 }, { index: 0x3015, subindex: 0, size: 2, value: 3 }")),
        "slaves[0].stored_sdo[1] (0x3015:00) is listed twice");
    expect_value_gate_refusal(
        parse(edited(stored_one_drive_yaml("{ index: 0x2000, subindex: 1, size: 1, value: 3 }"), "startup_sdo: []",
                     "startup_sdo: [{ index: 0x2000, subindex: 1, size: 1, value: 3 }]")),
        "slaves[0].stored_sdo[0] (0x2000:01) is also written by this slave's startup_sdo");
}

TEST(DriveModelFile, StoredSdoHasNoStaticObjectRefusalAndStartupSdoKeepsItsOwn) {
    // What the bus exchanges every cycle is checked by the bus (a warning), not
    // the loader; startup_sdo's core-owned refusals are unchanged.
    for (const std::string_view index : {"0x6040", "0x60b8", "0x6072", "0x1600"}) {
        SCOPED_TRACE(index);
        const std::string item = "{ index: " + std::string(index) + ", subindex: 0, size: 2, value: 0 }";
        const auto stored = parse(stored_one_drive_yaml(item));
        EXPECT_TRUE(stored) << stored.message();
        const auto startup = parse(edited(one_drive_yaml(), "startup_sdo: []", "startup_sdo: [" + item + "]"));
        if (index == "0x60b8") {
            EXPECT_TRUE(startup) << "0x60B8 is no core-owned object: " << startup.message();
        } else {
            expect_file_rejection(startup, "");
        }
    }
}

// limits.torque_nm (a cst axis's command clamp)

TEST(DriveModelFile, TheTorqueLimitIsACstAxisKeyOnly) {
    expect_file_rejection(parse(edited(one_drive_yaml(), ", torque_nm: { min: -30.0, max: 40.0 } }", " }")),
                          "slaves[0].axes[0].limits.torque_nm");
    for (const std::string_view mode : {"csp", "csv"}) {
        SCOPED_TRACE(mode);
        const std::string declared = edited(one_drive_yaml(), "mode: cst", "mode: " + std::string(mode));
        const auto refused = parse(declared);
        expect_file_rejection(refused, "slaves[0].axes[0].limits.torque_nm");
        EXPECT_NE(refused.message().find("clamps cst torque commands only"), std::string::npos) << refused.message();
        const auto loaded =
            parse(edited(declared, ", torque_nm: { min: -30.0, max: 40.0 } }", ", acceleration_rad_s2: 4.0 }"));
        ASSERT_TRUE(loaded) << loaded.message();
        EXPECT_EQ(as<1>(loaded).safety[0].torque_min_nm, 0.0);
        EXPECT_EQ(as<1>(loaded).safety[0].torque_max_nm, 0.0);
    }
}

// limits.acceleration_rad_s2 (a csp or csv axis's stop deceleration)

TEST(DriveModelFile, ACspOrCsvAxisDeclaresTheDecelerationItStopsAt) {
    for (const std::string_view mode : {"csp", "csv"}) {
        SCOPED_TRACE(mode);
        const std::string declared = edited(as_csp(one_drive_yaml()), "mode: csp", "mode: " + std::string(mode));
        const auto loaded = parse(declared);
        ASSERT_TRUE(loaded) << loaded.message();
        EXPECT_EQ(as<1>(loaded).safety[0].acceleration_rad_s2, 4.0);

        const auto missing = parse(edited(declared, ", acceleration_rad_s2: 4.0", ""));
        expect_file_rejection(missing, "slaves[0].axes[0].limits.acceleration_rad_s2");
        EXPECT_NE(missing.message().find("stop"), std::string::npos) << missing.message();
        for (const std::string_view value : {"0", "-4.0"}) {
            const auto refused =
                parse(edited(declared, "acceleration_rad_s2: 4.0", "acceleration_rad_s2: " + std::string(value)));
            expect_file_rejection(refused, "slaves[0].axes[0].limits.acceleration_rad_s2");
            EXPECT_NE(refused.message().find("must be > 0"), std::string::npos) << refused.message();
        }
    }
    // A cst axis stops by its drive's own disable reaction (0x605C): the key is not its.
    const auto cst = parse(edited(one_drive_yaml(), "max: 40.0 } }", "max: 40.0 }, acceleration_rad_s2: 4.0 }"));
    expect_file_rejection(cst, "slaves[0].axes[0].limits.acceleration_rad_s2");
    EXPECT_NE(cst.message().find("csp or csv"), std::string::npos) << cst.message();
}

TEST(DriveModelFile, TheValueGateHoldsTheStopDecelerationToItsMode) {
    auto spec = expected_example();  // cst axes
    spec.safety[0].acceleration_rad_s2 = 4.0;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("acceleration_rad_s2"), std::string::npos);
    spec.safety[0].acceleration_rad_s2 = 0.0;
    spec.axes[0].modes_of_operation = fieldbus::OperationMode::CYCLIC_SYNC_POSITION;
    spec.safety[0].torque_min_nm = spec.safety[0].torque_max_nm = 0.0;
    EXPECT_NE(detail::validate_drive_model_spec(spec).message().find("acceleration_rad_s2 must be finite and > 0"),
              std::string::npos);
}

// ramp (retired)

/// one_drive_yaml() as a csp axis with the retired ramp block.
std::string ramp_one_drive_yaml() {
    return edited(as_csp(one_drive_yaml()), "acceleration_rad_s2: 4.0 }\n",
                  "acceleration_rad_s2: 4.0 }\n        ramp: { acceleration_rad_s2: 5.0 }\n");
}

TEST(DriveModelFile, TheRetiredRampIsRefusedWithWhatToDo) {
    const auto refused = parse(ramp_one_drive_yaml());
    expect_file_rejection(refused, "slaves[0].axes[0].ramp");
    for (const std::string_view says :
         {"retired", "delete the block", "limits.acceleration_rad_s2", "the client plans the motion", "plem.drive",
          "move_to", "a FollowJointTrajectory goal to the bus that plem drive run starts"}) {
        EXPECT_NE(refused.message().find(says), std::string::npos) << says << " not in: " << refused.message();
    }
}

// pdo_mapping: scanned (the scanned policy of a slave)

TEST(DriveModelFile, ReadsTheScannedPolicyAsAnEmptyLayout) {
    const auto loaded = parse(scanned_one_drive_yaml());
    ASSERT_TRUE(loaded) << loaded.message();
    const auto& spec = std::get<DriveModelSpec<1>>(loaded.value());
    EXPECT_EQ(spec.slaves[0].pdo_mapping.sync_count, 0u) << "sync_count 0 = scanned (no graph, the bus scans)";
    EXPECT_EQ(spec.slaves[0].pdo_mapping, models::SlavePdoMappingSpec{});
    // Everything else about the slave reads as in the declared file.
    const auto declared = parse(one_drive_yaml());
    ASSERT_TRUE(declared) << declared.message();
    const auto& declared_spec = std::get<DriveModelSpec<1>>(declared.value());
    EXPECT_EQ(spec.slaves[0].ring_position, declared_spec.slaves[0].ring_position);
    EXPECT_EQ(spec.slaves[0].vendor_id, declared_spec.slaves[0].vendor_id);
    EXPECT_EQ(spec.slaves[0].product_code, declared_spec.slaves[0].product_code);
    EXPECT_EQ(spec.axes[0], declared_spec.axes[0]);
    // The value gate passes a scanned slave (it declares no sync manager to
    // check; the watchdog rule is a declaration's).
    const auto valid = detail::validate_drive_model_spec(spec);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST(DriveModelFile, TheScannedMappingTextLoadsAsTheDeclarationItCameFrom) {
    // What a bring-up log prints for a scanned slave (format_pdo_mapping, the
    // DEBUG text) pasted under `pdo_mapping:` loads to the same
    // SlavePdoMappingSpec as the hand-written declaration of that layout: the
    // round trip an author relies on when copying the log into a file.
    const auto& layout = fieldbus::test::kSingleAxisCstLayout;
    const std::string text = fieldbus::ethercat::format_pdo_mapping(layout);
    std::string block = "    pdo_mapping:\n";
    for (size_t at = 0; at < text.size();) {
        const size_t end = text.find('\n', at);
        ASSERT_NE(end, std::string::npos) << "every line ends in a newline";
        block += "      " + text.substr(at, end - at) + "\n";
        at = end + 1;
    }
    std::string yaml = one_drive_yaml();
    const size_t from = yaml.find("    pdo_mapping:\n");
    const size_t to = yaml.find("    startup_sdo: []");
    ASSERT_NE(from, std::string::npos);
    ASSERT_NE(to, std::string::npos);
    yaml.replace(from, to - from, block);
    const auto loaded = parse(yaml);
    ASSERT_TRUE(loaded) << loaded.message();
    const auto& spec = std::get<DriveModelSpec<1>>(loaded.value());
    EXPECT_EQ(spec.slaves[0].pdo_mapping, models::to_pdo_mapping_spec(layout));
    const auto declared = parse(one_drive_yaml());
    ASSERT_TRUE(declared) << declared.message();
    EXPECT_EQ(spec.slaves[0].pdo_mapping, std::get<DriveModelSpec<1>>(declared.value()).slaves[0].pdo_mapping);
}

TEST(DriveModelFile, RefusesEveryOtherPdoLayoutForm) {
    const std::string base = scanned_one_drive_yaml();
    expect_edits_refused(
        base, {
                  {"pdo_mapping: scanned", "pdo_mapping: \"scanned\"", "slaves[0].pdo_mapping"},
                  {"pdo_mapping: scanned", "pdo_mapping: null", "slaves[0].pdo_mapping"},
                  {"pdo_mapping: scanned", "pdo_mapping:", "slaves[0].pdo_mapping"},
                  {"pdo_mapping: scanned", "pdo_mapping: declared", "slaves[0].pdo_mapping"},
                  {"pdo_mapping: scanned", "pdo_mapping: [ ]", "slaves[0].pdo_mapping"},
                  {"pdo_mapping: scanned", "pdo_mapping: { sync_managers: [] }", "slaves[0].pdo_mapping.sync_managers"},
              });
    // The refusal names the three accepted forms.
    const std::string reason = parse(edited(base, "pdo_mapping: scanned", "pdo_mapping: \"scanned\"")).message();
    EXPECT_NE(reason.find("plain (unquoted) word generated"), std::string::npos) << reason;
    EXPECT_NE(reason.find("plain word scanned"), std::string::npos) << reason;
    EXPECT_NE(reason.find("mapping with sync_managers"), std::string::npos) << reason;
}

// The file wrapper

TEST(DriveModelFileWrapper, SaysWhichPathFailedAndWhatToPassInstead) {
    EXPECT_EQ(load_drive_model_file("/nonexistent/drive.yaml").message(),
              "cannot load drive model file '/nonexistent/drive.yaml': No such file or directory — pass the drive "
              "model file's absolute path (a relative path resolves against the working directory; ~ is not "
              "expanded)");
    const auto directory = load_drive_model_file(::testing::TempDir());
    EXPECT_NE(directory.message().find("': not a regular file — "), std::string::npos) << directory.message();
}

TEST(DriveModelFileWrapper, SaysWhyAFileCannotBeOpened) {
    if (::geteuid() == 0) {
        GTEST_SKIP() << "root opens a file whatever its mode";
    }
    constexpr std::string_view kFix =
        "Permission denied — make the file readable and its directories searchable (x) by the user this "
        "process runs as";
    const std::string path =
        ::testing::TempDir() + "plem_drive_model_unreadable." + std::to_string(::getpid()) + ".yaml";
    std::ofstream(path) << "schema: cia402_drive/v1\n";
    std::filesystem::permissions(path, std::filesystem::perms::none);
    const auto unreadable = load_drive_model_file(path);
    std::filesystem::remove(path);
    EXPECT_EQ(unreadable.message(), "cannot open drive model file '" + path + "': " + std::string(kFix));
    // A directory the process cannot search: stat fails first; same cause, same fix, not "pass another path".
    const std::string dir = ::testing::TempDir() + "plem_drive_model_locked." + std::to_string(::getpid());
    std::filesystem::create_directory(dir);
    std::ofstream(dir + "/drive.yaml") << "schema: cia402_drive/v1\n";
    std::filesystem::permissions(dir, std::filesystem::perms::none);
    const auto locked = load_drive_model_file(dir + "/drive.yaml");
    std::filesystem::permissions(dir, std::filesystem::perms::owner_all);
    std::filesystem::remove_all(dir);
    EXPECT_EQ(locked.message(), "cannot load drive model file '" + dir + "/drive.yaml': " + std::string(kFix));
}

TEST(DriveModelFileWrapper, RefusesAFileOverTheSizeCap) {
    const std::string path = ::testing::TempDir() + "plem_drive_model_over_cap." + std::to_string(::getpid()) + ".yaml";
    {
        std::ofstream out(path, std::ios::binary);
        out << std::string(detail::kMaxDriveModelFileBytes + 1, '#');
    }
    const auto loaded = load_drive_model_file(path);
    std::filesystem::remove(path);
    EXPECT_EQ(loaded.message(), "drive model file '" + path +
                                    "' is larger than 1048576 bytes: check that the path names the drive model "
                                    "file");
}

// Digests

detail::DriveModelDigests digests_of(std::string_view yaml, ProfileSource source = ProfileSource::BUILT_IN) {
    const auto loaded = parse(yaml);
    EXPECT_TRUE(loaded) << loaded.message();
    return loaded ? detail::drive_model_digests(loaded.value(), source) : detail::DriveModelDigests{};
}

/// one_drive_yaml() with no end stops: travel null, counted from the raw zero.
std::string unbounded_one_drive_yaml() {
    return edited(edited(one_drive_yaml(), "position: { min_rad: -1.0, max_rad: 1.5 }", "position: null"),
                  "zero_offset_count: 1234", "zero_offset_count: 0");
}

TEST(DriveModelFile, AnAxisWithoutEndStopsWritesItsTravelAsNull) {
    const auto loaded = parse(unbounded_one_drive_yaml());
    ASSERT_TRUE(loaded) << loaded.message();
    EXPECT_EQ(as<1>(loaded).safety[0].position_min_rad, -std::numeric_limits<double>::infinity());
    EXPECT_EQ(as<1>(loaded).safety[0].position_max_rad, std::numeric_limits<double>::infinity());
    // Omitting the key is still a refusal (the walk tests); null is how a file says it.
    expect_value_gate_refusal(parse(edited(unbounded_one_drive_yaml(), "zero_offset_count: 0", "zero_offset_count: 7")),
                              "set zero_offset_count: 0");
    expect_file_rejection(parse(edited(one_drive_yaml(), "min_rad: -1.0", "min_rad: -.inf")), "");
    // Endless in every mode: a csp axis without end stops loads too.
    const auto csp = parse(as_csp(unbounded_one_drive_yaml()));
    ASSERT_TRUE(csp) << csp.message();
    EXPECT_EQ(as<1>(csp).axes[0].modes_of_operation, fieldbus::OperationMode::CYCLIC_SYNC_POSITION);
}

TEST(DriveModelDigest, ANullTravelIsItsOwnDeclaration) {
    const auto bounded = digests_of(edited(one_drive_yaml(), "zero_offset_count: 1234", "zero_offset_count: 0"));
    const auto unbounded = digests_of(unbounded_one_drive_yaml());
    EXPECT_NE(unbounded.config, bounded.config);  // an approved travel turned null is a new approval
    EXPECT_EQ(digests_of(unbounded_one_drive_yaml()).config, unbounded.config);
}

TEST(DriveModelDigest, TheExampleHasItsGoldenDigests) {
    const auto digests = digests_of(example_text());
    EXPECT_EQ(digests.config, "sha256:11ae9ba4bfe777d03f2d7876116e480721b3d85f0b36e63afe5a3fcb85fbb41d");
    EXPECT_EQ(digests.profile, "sha256:43d76b4dabb0679b373e78edfb8b798e40c66fa0394e18b0ab29a66941930bce");
}

TEST(DriveModelDigest, TheShippedTableKeepsThePreSplitBuiltInDigest) {
    // Replacing the example's axis profiles with kNotReadyShutdownCia402Profile
    // preserves this pinned profile digest when normalized with
    // ProfileSource::BUILT_IN. Renaming or moving the profile declaration must
    // preserve its normalized contents.
    const auto loaded = parse(example_text());
    ASSERT_TRUE(loaded) << loaded.message();
    auto spec = loaded.value();
    std::visit(
        [](auto& s) {
            for (auto& axis : s.axes) {
                axis.cia402_profile = fieldbus::kNotReadyShutdownCia402Profile;
            }
        },
        spec);
    EXPECT_EQ(detail::drive_model_digests(spec, ProfileSource::BUILT_IN).profile,
              "sha256:d38fb470416cb728aa29dc544c3e3886206f4a1da0b32eec1cd5f82366a90390");
}

TEST(DriveModelDigest, TheScannedPolicyHasItsOwnDigestAndAMappingKeepsItsOwn) {
    // `pdo_mapping.sync_count=0` is the one normalized line a scanned slave
    // contributes, so a scanned file and its declared twin digest differently
    // (goldens below, machine-filled).
    const auto scanned = digests_of(scanned_one_drive_yaml());
    const auto declared = digests_of(one_drive_yaml());
    EXPECT_NE(scanned.config, declared.config);
    EXPECT_EQ(scanned.profile, declared.profile);
    EXPECT_EQ(scanned.config, "sha256:134f00505d31088f4082878fe70bcfa6d3f58d996fe33573ca51912b5d37eb98");
    EXPECT_EQ(declared.config, "sha256:da1948e9551636e4110554bdf61268bbb5c204d3d13c85b08dfa9108820a95ab");
}

TEST(DriveModelDigest, APerMilleTorqueHasItsGoldenDigest) {
    // A rated_per_mille axis writes its unit and motor_rated_torque_nm; the
    // scaled_current lines (the goldens above) are unchanged. Machine-filled.
    const auto per_mille = digests_of(per_mille_one_drive_yaml());
    EXPECT_EQ(per_mille.config, "sha256:1b843f50265e3aacca38b137261557025476d787b042f0a3f7d5e30efd47081c");
    EXPECT_EQ(per_mille.profile, digests_of(one_drive_yaml()).profile);
    EXPECT_NE(digests_of(edited(per_mille_one_drive_yaml(), "1.27", "1.28")).config, per_mille.config);
    // The same channel values declared as scaled_current are another declaration.
    EXPECT_NE(digests_of(edited(one_drive_yaml(), "counts_per_amp: 50.0, torque_constant: 0.12",
                                "counts_per_amp: 1000.0, torque_constant: 1.27"))
                  .config,
              per_mille.config);
}

TEST(DriveModelDigest, TheSameDeclarationInAnotherSpellingHasTheSameDigest) {
    const auto base = digests_of(example_text());
    // Integers by value, not by spelling.
    EXPECT_EQ(digests_of(edited(example_text(), "vendor_id: 0x00c0ffee", "vendor_id: 12648430")), base);
    // Comments are not data.
    EXPECT_EQ(digests_of(edited(example_text(), "# rev 2026-09-13.1", "# rev 2026-09-14.1")), base);
    // Mapping key order is not data.
    EXPECT_EQ(
        digests_of(edited(example_text(), "bus: { name: synthetic, cycle_time_ns: 1000000, sync_mode: drive_default }",
                          "bus: { sync_mode: drive_default, cycle_time_ns: 1000000, name: synthetic }")),
        base);
}

TEST(DriveModelDigest, AChangedValueChangesTheConfigDigest) {
    const auto base = digests_of(example_text());
    // One ULP of a torque constant.
    const auto kt = digests_of(edited(example_text(), "torque_constant: 0.12", "torque_constant: 0.12000000000000001"));
    EXPECT_NE(kt.config, base.config);
    EXPECT_EQ(kt.profile, base.profile);
    // An axis name is data (schema §7).
    EXPECT_NE(digests_of(edited(example_text(), "name: j0", "name: j9")).config, base.config);
    // Sequence order is execution meaning: two startup SDO writes swapped.
    const auto maximal = digests_of(kMaximal, ProfileSource::INLINE);
    const auto swapped = digests_of(edited(std::string(kMaximal),
                                           "      - { index: 0x2000, subindex: 1, size: 1, value: 0xaa }\n"
                                           "      - { index: 0x2001, subindex: 0, size: 4, value: 0xffffffff }\n",
                                           "      - { index: 0x2001, subindex: 0, size: 4, value: 0xffffffff }\n"
                                           "      - { index: 0x2000, subindex: 1, size: 1, value: 0xaa }\n"),
                                    ProfileSource::INLINE);
    EXPECT_NE(swapped.config, maximal.config);
    EXPECT_EQ(swapped.profile, maximal.profile);
    // The drive's own table is a different profile from the built-in one.
    EXPECT_NE(maximal.profile, base.profile);
}

TEST(DriveModelDigest, AStoredSdoListIsNotInEitherDigest) {
    // Warn-only and unexecuted (schema §4.9): editing it keeps an approval.
    const auto base = digests_of(one_drive_yaml());
    EXPECT_EQ(digests_of(stored_one_drive_yaml("{ index: 0x3015, subindex: 0, size: 2, value: 2 }")), base);
    EXPECT_EQ(digests_of(stored_one_drive_yaml("{ index: 0x3015, subindex: 0, size: 2, value: 3 }")), base);
}

TEST(DriveModelDigest, WriteCycleTimeEntersTheConfigDigestOnlyWhenTrue) {
    // Absent keeps every existing digest (the goldens above); true is a bus
    // that writes one more object, so a record made without it no longer matches.
    const auto without = digests_of(kMaximal, ProfileSource::INLINE);
    const auto with = digests_of(maximal_writing_cycle_time("true"), ProfileSource::INLINE);
    EXPECT_NE(with.config, without.config);
    EXPECT_EQ(with.profile, without.profile);
}

TEST(DriveModelFile, ThePulseWidthIsMandatoryAndCapped) {
    const std::string base = one_drive_yaml();
    constexpr std::string_view kPath = "fault_reset.pulse_ms";
    expect_edits_refused(base, {
                                   {"pulse_ms: 0", "pulse_ms: 1001", kPath},
                                   {"pulse_ms: 0", "pulse_ms: -1", kPath},
                               });
    expect_file_rejection(parse(edited(base, ", pulse_ms: 0", "")), kPath);
    const auto widest = parse(edited(base, "pulse_ms: 0", "pulse_ms: 1000"));
    ASSERT_TRUE(widest) << widest.message();
    EXPECT_EQ(as<1>(widest).fault_reset_pulse_ms, 1000u);
}

TEST(DriveModelDigest, ThePulseWidthIsInTheConfigDigestOnly) {
    const auto base = digests_of(example_text());
    const auto wide = digests_of(edited(example_text(), "pulse_ms: 0", "pulse_ms: 20"));
    EXPECT_NE(wide.config, base.config);
    EXPECT_EQ(wide.profile, base.profile);
}

TEST(DriveModelDigest, NegativeZeroIsZero) {
    const auto zero = digests_of(edited(example_text(), "min_rad: -1.0", "min_rad: 0.0"));
    EXPECT_EQ(digests_of(edited(example_text(), "min_rad: -1.0", "min_rad: -0.0")), zero);
    EXPECT_NE(zero.config, digests_of(example_text()).config);
}

TEST(DriveModelDigest, TheProfileSourceIsInTheConfigDigestOnly) {
    const auto built_in = digests_of(example_text(), ProfileSource::BUILT_IN);
    const auto inline_copy = digests_of(example_text(), ProfileSource::INLINE);
    EXPECT_NE(built_in.config, inline_copy.config);
    EXPECT_EQ(built_in.profile, inline_copy.profile);
}

TEST(DriveModelDigest, ThePlatformVersionIsTheReleaseTrainString) {
    const std::string_view version = detail::platform_version();
    EXPECT_FALSE(version.empty());
    EXPECT_EQ(version.find_first_not_of("0123456789."), std::string_view::npos) << version;
    EXPECT_EQ(std::count(version.begin(), version.end(), '.'), 2) << version;
}

// Shorter spellings of the same declaration (schema §4.1, §4.3)

/// What the bus's startup check answers for a loaded file; empty =
/// accepted.
template <size_t N>
std::string bus_refusal(const DriveModelSpec<N>& spec) {
    const auto injection = detail::drive_bus_injection(spec).value();
    return fieldbus::ethercat::validate_bus_configuration(injection.servos, injection.startup, injection.stored,
                                                          injection.expected)
        .message();
}

/// drives_yaml({1, 1}) written as one slave entry for both drives: the ring
/// positions as a list, and one name, flip and zero offset per drive.
std::string two_identical_drives_yaml() {
    std::string yaml = drives_yaml({1});
    yaml = edited(yaml, "  - ring_position: 0\n", "  - ring_position: [0, 1]\n");
    yaml = edited(yaml, "      - name: j0\n", "      - name: [j0, j1]\n");
    yaml = edited(yaml, "zero_offset_count: " + std::to_string(test::kAxisZeroOffsets[0]) + "\n",
                  "zero_offset_count: [" + std::to_string(test::kAxisZeroOffsets[0]) + ", " +
                      std::to_string(test::kAxisZeroOffsets[1]) + "]\n");
    return edited(yaml, "        flip: false\n", "        flip: [false, false]\n");
}

TEST(DriveModelFile, ARingPositionListDeclaresIdenticalDrivesOnce) {
    const std::string spelled_out = drives_yaml({1, 1});
    const std::string listed = two_identical_drives_yaml();
    const auto expected = parse(spelled_out);
    const auto loaded = parse(listed);
    ASSERT_TRUE(expected) << expected.message();
    ASSERT_TRUE(loaded) << loaded.message();
    expect_same(as<2>(loaded), as<2>(expected));
    EXPECT_EQ(digests_of(listed), digests_of(spelled_out)) << "the same declaration keeps its approval";

    // One value per drive for the per-drive keys, never one shared by all.
    const auto shared_name = parse(edited(listed, "name: [j0, j1]", "name: j0"));
    expect_file_rejection(shared_name, "slaves[0].axes[0].name");
    EXPECT_NE(shared_name.message().find("must list one value per drive — this "
                                         "slave's ring_position lists 2 drives"),
              std::string::npos)
        << shared_name.message();
    expect_file_rejection(parse(edited(listed, "flip: [false, false]", "flip: false")), "slaves[0].axes[0].flip");
    expect_file_rejection(parse(edited(listed, "zero_offset_count: [", "zero_offset_count: [0, ")),
                          "slaves[0].axes[0].zero_offset_count");
    expect_file_rejection(parse(edited(listed, "name: [j0, j1]", "name: [j0, 5]")), "slaves[0].axes[0].name[1]");
    // A list is two or more drives; a single drive keeps single values.
    expect_file_rejection(parse(edited(listed, "ring_position: [0, 1]", "ring_position: [0]")),
                          "slaves[0].ring_position");
    expect_file_rejection(parse(edited(listed, "ring_position: [0, 1]", "ring_position: [0, x]")),
                          "slaves[0].ring_position[1]");
    expect_file_rejection(parse(edited(drives_yaml({1}), "name: j0", "name: [j0, j1]")), "slaves[0].axes[0].name");
    // Two drives at one position are the value gate's refusal, as when spelled
    // out.
    expect_value_gate_refusal(parse(edited(listed, "ring_position: [0, 1]", "ring_position: [0, 0]")),
                              "both declare ring position 0");
    // A value-gate refusal names the file's path (not the flat axis index),
    // and for a listed drive which ring position it is.
    const auto upper = parse(edited(listed, "name: [j0, j1]", "name: [j0, J1]"));
    expect_value_gate_refusal(upper, "for the drive at slaves[0].ring_position[1]: slaves[0].axes[0].name (\"J1\")");
    EXPECT_EQ(upper.message().find("axes[1]"), std::string::npos) << upper.message();
    const auto second = parse(edited(spelled_out, "name: j1", "name: J1"));
    expect_value_gate_refusal(second, "test.yaml: slaves[1].axes[0].name (\"J1\")");
    // Every index in the message is the file's, not only the first.
    const auto repeated = parse(edited(spelled_out, "name: j1", "name: j0"));
    expect_value_gate_refusal(repeated, "test.yaml: slaves[1].axes[0].name (\"j0\") repeats slaves[0].axes[0].name — ");
    const auto repeated_listed = parse(edited(listed, "name: [j0, j1]", "name: [j0, j0]"));
    expect_value_gate_refusal(repeated_listed,
                              "for the drives at slaves[0].ring_position[1] and slaves[0].ring_position[0]: "
                              "slaves[0].axes[0].name (\"j0\") repeats slaves[0].axes[0].name — ");
}

TEST(DriveModelFile, ThePositionAndVelocityUnitMayBeLeftOut) {
    const std::string full = one_drive_yaml();
    const std::string short_form =
        edited(edited(full, "          position: { unit: rotary_motor_encoder }\n", ""),
               "velocity: { unit: rotary_motor_encoder, velocity_factor: 1.0 }", "velocity: { velocity_factor: 1.0 }");
    const auto expected = parse(full);
    const auto loaded = parse(short_form);
    ASSERT_TRUE(expected) << expected.message();
    ASSERT_TRUE(loaded) << loaded.message();
    expect_same(as<1>(loaded), as<1>(expected));
    EXPECT_EQ(digests_of(short_form), digests_of(full));
    // Written, the unit is still only the one value; velocity_factor stays
    // required.
    expect_file_rejection(parse(edited(full, "position: { unit: rotary_motor_encoder }", "position: { unit: linear }")),
                          "slaves[0].axes[0].factors.position.unit");
    expect_file_rejection(parse(edited(short_form, "velocity: { velocity_factor: 1.0 }", "velocity: {}")),
                          "slaves[0].axes[0].factors.velocity.velocity_factor");
}

TEST(DriveModelFile, AMissingModeDisplayTimingIsPreEnable) {
    // Left out = pre_enable, the conservative reading: the same spec and the
    // same digests as the file that writes it, so no approval changes.
    const std::string written = one_drive_yaml();
    const std::string left_out = edited(written, "        mode_display_timing: pre_enable\n", "");
    const auto expected = parse(written);
    const auto loaded = parse(left_out);
    ASSERT_TRUE(expected) << expected.message();
    ASSERT_TRUE(loaded) << loaded.message();
    EXPECT_EQ(as<1>(loaded).axes[0].mode_display_timing, fieldbus::ModeDisplayTiming::PRE_ENABLE);
    expect_same(as<1>(loaded), as<1>(expected));
    EXPECT_EQ(digests_of(left_out), digests_of(written));
    // Written, it is still one of the two values.
    expect_file_rejection(parse(edited(written, "mode_display_timing: pre_enable", "mode_display_timing: maybe")),
                          "slaves[0].axes[0].mode_display_timing");
}

TEST(DriveModelFile, AMissingErrorCodePdoIsTheMappedErrorCode) {
    // Declared mappings carrying 0x603F (channel 1, and channel 2 at +0x800): the
    // same file.
    const std::string example = example_text();
    std::string without = example;
    for (size_t at = without.find("        error_code_pdo: {"); at != std::string::npos;
         at = without.find("        error_code_pdo: {", at)) {
        without.erase(at, without.find('\n', at) + 1 - at);
    }
    ASSERT_NE(without, example);
    const auto loaded = parse(without);
    ASSERT_TRUE(loaded) << loaded.message();
    expect_same(as<5>(loaded), expected_example());

    // A mapping without 0x603F, or one read only at start: the key is written.
    const auto unmapped = parse(edited(one_drive_yaml(), "        error_code_pdo: null\n", ""));
    expect_file_rejection(unmapped, "slaves[0].axes[0].error_code_pdo");
    EXPECT_NE(unmapped.message().find("the declared mapping has no Error code (0x603F)"), std::string::npos)
        << unmapped.message();
    const auto scanned = parse(edited(scanned_one_drive_yaml(), "        error_code_pdo: null\n", ""));
    expect_file_rejection(scanned, "slaves[0].axes[0].error_code_pdo");
    EXPECT_NE(scanned.message().find("a scanned mapping is read only when the bus starts"), std::string::npos)
        << scanned.message();
}

/// `yaml` (a drives_yaml form) with every slave's mapping `generated`, each
/// axis in `mode` and its error code left to the mapping.
std::string generated(std::string yaml, std::string_view mode = "csp") {
    for (size_t from = yaml.find("    pdo_mapping:\n"); from != std::string::npos;
         from = yaml.find("    pdo_mapping:\n", from + 1)) {
        yaml.replace(from, yaml.find("    startup_sdo: []", from) - from, "    pdo_mapping: generated\n");
    }
    yaml = edited(yaml, "        error_code_pdo: null\n", "");
    yaml = edited(yaml, "mode: cst", "mode: " + std::string(mode));
    return mode == "cst" ? yaml
                         : edited(yaml, ", torque_nm: { min: -30.0, max: 40.0 } }", ", acceleration_rad_s2: 4.0 }");
}

std::string generated_yaml(const std::vector<size_t>& axes, std::string_view mode = "csp") {
    return generated(drives_yaml(axes), mode);
}

TEST(DriveModelFile, TheGeneratedMappingIsTheCia402ObjectsTheAxesUse) {
    using fieldbus::ethercat::PdoEntryMapping;
    const auto loaded = parse(generated_yaml({1}));
    ASSERT_TRUE(loaded) << loaded.message();
    const auto& spec = as<1>(loaded);
    models::SlavePdoMappingSpec expected{};
    expected.sync_count = 2;
    expected.syncs[0] = {.sm_index = 2, .output = true, .pdo_count = 1, .watchdog = true};
    expected.syncs[0].pdos[0] = {.index = 0x1600, .entry_count = 3};
    expected.syncs[0].pdos[0].entries = {PdoEntryMapping{0x6040, 0, 16}, {0x6060, 0, 8}, {0x607a, 0, 32}};
    expected.syncs[1] = {.sm_index = 3, .output = false, .pdo_count = 1};
    expected.syncs[1].pdos[0] = {.index = 0x1a00, .entry_count = 6};
    expected.syncs[1].pdos[0].entries = {PdoEntryMapping{0x6041, 0, 16},
                                         {0x6061, 0, 8},
                                         {0x6064, 0, 32},
                                         {0x606c, 0, 32},
                                         {0x6077, 0, 16},
                                         {0x603f, 0, 16}};
    EXPECT_TRUE(spec.slaves[0].pdo_mapping == expected);
    EXPECT_EQ(spec.axes[0].error_code_pdo, (PdoEntryMapping{0x603f, 0, 16}));
    EXPECT_EQ(bus_refusal(spec), "");
    // The target follows the mode.
    for (const auto& [mode, target] :
         {std::pair{"csv", PdoEntryMapping{0x60ff, 0, 32}}, std::pair{"cst", PdoEntryMapping{0x6071, 0, 16}}}) {
        SCOPED_TRACE(mode);
        const auto other = parse(generated_yaml({1}, mode));
        ASSERT_TRUE(other) << other.message();
        EXPECT_EQ(as<1>(other).slaves[0].pdo_mapping.syncs[0].pdos[0].entries[2], target);
        EXPECT_EQ(bus_refusal(as<1>(other)), "");
    }

    // A two-channel drive: one PDO per axis, the second at +1 and its objects at
    // +0x800.
    const auto two = parse(generated_yaml({2}));
    ASSERT_TRUE(two) << two.message();
    const auto& layout = as<2>(two).slaves[0].pdo_mapping;
    ASSERT_EQ(layout.syncs[0].pdo_count, 2);
    EXPECT_EQ(layout.syncs[0].pdos[1].index, 0x1601);
    EXPECT_EQ(layout.syncs[0].pdos[1].entries[0], (PdoEntryMapping{0x6840, 0, 16}));
    EXPECT_EQ(layout.syncs[1].pdos[1].index, 0x1a01);
    EXPECT_EQ(layout.syncs[1].pdos[1].entries[5], (PdoEntryMapping{0x683f, 0, 16}));
    EXPECT_EQ(bus_refusal(as<2>(two)), "");

    // Two generated slaves: each layout holds only its own axis, so the first
    // is generated before the second slave's axis is read.
    const auto pair = parse(generated_yaml({1, 1}));
    ASSERT_TRUE(pair) << pair.message();
    for (size_t s = 0; s < 2; ++s) {
        SCOPED_TRACE(s);
        EXPECT_TRUE(as<2>(pair).slaves[s].pdo_mapping == expected);
    }
    EXPECT_EQ(bus_refusal(as<2>(pair)), "");
    // The same drive listed at two ring positions: one layout per copy.
    const auto listed = parse(generated(two_identical_drives_yaml()));
    ASSERT_TRUE(listed) << listed.message();
    for (size_t s = 0; s < 2; ++s) {
        SCOPED_TRACE(s);
        EXPECT_TRUE(as<2>(listed).slaves[s].pdo_mapping == expected);
    }
    EXPECT_EQ(bus_refusal(as<2>(listed)), "");

    // More axes than one PDO each can carry: declare that drive's mapping.
    const auto five = parse(generated_yaml({5}));
    expect_file_rejection(five, "slaves[0].pdo_mapping");
    EXPECT_NE(five.message().find("declare this drive's mapping instead"), std::string::npos) << five.message();
}

// Approval judgment (schema §6)

/// An approved block: the record's head (`slaves` in kSlave0 form, or empty
/// for `[]`) and the bench evidence it embeds (`bench`, a bench_body()).
std::string approved_block(std::string_view config, std::string_view profile, std::string_view revision,
                           std::string_view slaves, std::string_view bench) {
    return "approval:\n  status: approved\n  record:\n    config_digest: \"" + std::string(config) +
           "\"\n    profile_digest: \"" + std::string(profile) + "\"\n    implementation_revision: \"" +
           std::string(revision) + "\"\n    slaves:" + (slaves.empty() ? " []\n" : "\n" + std::string(slaves)) +
           "    bench:\n" + indented(bench, 6);
}

/// The evidence of an honest bench on one_drive_yaml(): every required item PASS.
std::string honest_bench(std::string_view slaves = kSlave0, std::string_view items = kAllPass) {
    const auto digests = digests_of(one_drive_yaml());
    return bench_body(digests.config, digests.profile, detail::platform_version(), slaves, items);
}

/// one_drive_yaml() claiming approved with this build's digests and revision:
/// the record's head is `slaves`, the embedded bench is `bench` (by default
/// one that claims the same).
std::string approved_file(std::string_view slaves = kSlave0, std::string_view bench = {}) {
    const auto digests = digests_of(one_drive_yaml());
    return edited(one_drive_yaml(), kUnapprovedBlock,
                  approved_block(digests.config, digests.profile, detail::platform_version(), slaves,
                                 bench.empty() ? honest_bench(slaves) : std::string(bench)));
}

detail::DriveModelInspection inspected(std::string_view yaml) {
    auto result = detail::inspect_drive_model(yaml, "test.yaml");
    EXPECT_TRUE(result) << result.message();
    return result ? std::move(result).value() : detail::DriveModelInspection{};
}

std::string joined(const std::vector<std::string>& reasons) {
    std::string out;
    for (const auto& reason : reasons) {
        out += "\n  " + reason;
    }
    return out;
}

void expect_reason(const detail::DriveModelInspection& in, std::string_view says) {
    const bool found = std::any_of(in.reasons.begin(), in.reasons.end(),
                                   [&](const std::string& r) { return r.find(says) != std::string::npos; });
    EXPECT_TRUE(found) << "no reason says '" << says << "' among:" << joined(in.reasons);
    EXPECT_EQ(in.effective, Approval::UNAPPROVED);
    EXPECT_EQ(in.claimed, Approval::APPROVED);
    for (const auto& reason : in.reasons) {
        for (const std::string_view symbol : kCppSymbols) {
            EXPECT_EQ(reason.find(symbol), std::string::npos) << symbol << " in: " << reason;
        }
    }
}

void expect_honored(const detail::DriveModelInspection& in) {
    EXPECT_TRUE(in.reasons.empty()) << joined(in.reasons);
    EXPECT_EQ(in.claimed, Approval::APPROVED);
    EXPECT_EQ(in.effective, Approval::APPROVED);
}

TEST(DriveModelApproval, AnUnapprovedFileIsWhatItSays) {
    const auto in = inspected(one_drive_yaml());
    EXPECT_EQ(in.claimed, Approval::UNAPPROVED);
    EXPECT_EQ(in.effective, Approval::UNAPPROVED);
    EXPECT_EQ(in.identity_scope, detail::IdentityScope::NOT_APPLICABLE);
    EXPECT_TRUE(in.reasons.empty()) << joined(in.reasons);
    EXPECT_EQ(in.digests, digests_of(one_drive_yaml()));
    EXPECT_EQ(std::get<DriveModelSpec<1>>(in.spec).expected_identity[0], fieldbus::ethercat::SlaveIdentity{});
}

TEST(DriveModelApproval, AnHonestRecordIsHonoredAndNeedsNoOtherFile) {
    const auto in = inspected(approved_file());
    expect_honored(in);
    EXPECT_EQ(in.identity_scope, detail::IdentityScope::MODEL);
    const auto& spec = std::get<DriveModelSpec<1>>(in.spec);
    EXPECT_EQ(spec.approval, Approval::APPROVED);
    fieldbus::ethercat::SlaveIdentity identity{};
    identity.serial_number = 0;
    identity.hardware_version = std::array<char, 32>{'1', '.', '3'};
    identity.software_version = std::array<char, 32>{'2', '.', '0', '7'};
    EXPECT_EQ(spec.expected_identity[0], identity);
    // The evidence travels inside the file: moved to another directory, alone, it loads the same.
    const ScratchDir first("plem_approval_a_");
    const ScratchDir second("plem_approval_b_");
    std::ofstream(first.path + "/drive.yaml") << approved_file();
    std::filesystem::rename(first.path + "/drive.yaml", second.path + "/moved.yaml");
    const auto loaded = load_drive_model_file(second.path + "/moved.yaml");
    ASSERT_TRUE(loaded) << loaded.message();
    EXPECT_EQ(as<1>(loaded).approval, Approval::APPROVED);
    EXPECT_EQ(as<1>(loaded).expected_identity[0], identity);
}

// A bench run (an operator at the drives) uses no record: an approved file
// loads as unapproved with no identity to compare, so the bench can re-bench
// it on drives other than the record's.
TEST(DriveModelApproval, ABenchRunIgnoresTheApprovalRecord) {
    const ScratchDir dir("plem_approval_bench_");
    std::ofstream(dir.path + "/drive.yaml") << approved_file();
    const auto loaded = load_drive_model_file(dir.path + "/drive.yaml", ApprovalRecord::IGNORE);
    ASSERT_TRUE(loaded) << loaded.message();
    EXPECT_EQ(as<1>(loaded).approval, Approval::UNAPPROVED);
    EXPECT_EQ(as<1>(loaded).expected_identity[0], fieldbus::ethercat::SlaveIdentity{});
}

TEST(DriveModelApproval, EveryDigestMismatchIsAReason) {
    const auto digests = digests_of(one_drive_yaml());
    // The record's head only (the bench claims keep the true digests): judgment 2/3, and 7 for the head's edit.
    const auto head = [&](std::string_view from, std::string_view to) {
        std::string file = approved_file();
        const size_t at = file.find(from);  // the head comes first
        return file.replace(at, from.size(), to);
    };
    const auto config = inspected(head(digests.config, "sha256:0000"));
    expect_reason(config, "approval.record.config_digest is 'sha256:0000' but this declaration digests to '" +
                              digests.config + "'");
    expect_reason(config, "re-bench the file on this software (`plem drive run <file>` on a terminal offers the bench");
    const auto profile = inspected(head(digests.profile, "sha256:1111"));
    expect_reason(profile,
                  "approval.record.profile_digest is 'sha256:1111' but the resolved cia402_profile digests "
                  "to '" +
                      digests.profile + "'");
    // A file that edits a value after approval: the digest moves, the record does not.
    expect_reason(inspected(edited(approved_file(), "torque_constant: 0.12", "torque_constant: 0.09")),
                  "approval.record.config_digest is '" + digests.config + "' but this declaration digests to");
    // The release a record was benched on is provenance, not a check: an
    // approval benched on another release (both copies say so) is honored.
    expect_honored(inspected(edited(approved_file(), std::string(detail::platform_version()), "0.0.1")));
    // Every reason is collected, not the first; the loader names the first and counts the rest.
    const auto all =
        inspected(edited(edited(approved_file(), digests.config, "sha256:0000"), digests.profile, "sha256:1111"));
    EXPECT_EQ(all.reasons.size(), 2u) << joined(all.reasons);
    const auto refused =
        parse(edited(edited(approved_file(), digests.config, "sha256:0000"), digests.profile, "sha256:1111"));
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.message().rfind("test.yaml: approval.status is approved but this software cannot honor it — ", 0),
              0u)
        << refused.message();
    EXPECT_NE(refused.message().find("(+1 more — `plem drive check` lists them all)"), std::string::npos)
        << refused.message();
    // Bench mode re-benches such a file: the claim it cannot honor loads as unapproved.
    const auto bench = detail::parse_drive_model(
        edited(edited(approved_file(), digests.config, "sha256:0000"), digests.profile, "sha256:1111"), "test.yaml",
        true);
    ASSERT_TRUE(bench) << bench.message();
    EXPECT_EQ(as<1>(bench).approval, Approval::UNAPPROVED);
}

TEST(DriveModelApproval, TheRecordListsEveryDeclaredSlaveOnceWithSomethingObserved) {
    const auto none = inspected(approved_file(""));
    expect_reason(none, "approval.record.slaves lists slave 0 0 times");
    EXPECT_EQ(none.identity_scope, detail::IdentityScope::NOT_APPLICABLE) << "no observation binds to nothing";
    expect_reason(inspected(approved_file(std::string(kSlave0) + std::string(kSlave0))),
                  "approval.record.slaves lists slave 0 2 times");
    expect_reason(
        inspected(approved_file("      - { slave: 1, serial_number: 0, revision_number: null, hardware_version: "
                                "\"1.3\", software_version: \"2.07\", pdo_mapping_digest: null }\n")),
        "approval.record.slaves[0].slave is 1 but the file declares slaves 0..0");
    expect_reason(
        inspected(approved_file("      - { slave: 0, serial_number: 0, revision_number: null, hardware_version: "
                                "null, software_version: null, pdo_mapping_digest: null }\n")),
        "approval.record.slaves[0] observes nothing (serial_number 0 or null, revision_number null");
    expect_reason(inspected(approved_file(
                      "      - { slave: 0, serial_number: null, revision_number: null, hardware_version: null, "
                      "software_version: null, pdo_mapping_digest: null }\n")),
                  "approval.record.slaves[0] observes nothing");
    // One observed value is enough; a null field is "not claimed", not "empty".
    expect_honored(inspected(
        approved_file("      - { slave: 0, serial_number: null, revision_number: null, hardware_version: null, "
                      "software_version: \"2.07\", pdo_mapping_digest: null }\n")));
    // Shape is the file layer's: an unknown key, a quoted serial, a 32-byte version.
    expect_file_rejection(
        parse(approved_file("      - { slave: 0, serial_number: 0, revision_number: null, hardware_version: \"1.3\", "
                            "software_version: \"2.07\", pdo_mapping_digest: null, "
                            "firmware: \"x\" }\n",
                            honest_bench())),
        "approval.record.slaves[0].firmware");
    expect_file_rejection(
        parse(approved_file(
            "      - { slave: 0, serial_number: \"0x0\", revision_number: null, hardware_version: \"1.3\", "
            "software_version: \"2.07\", pdo_mapping_digest: null }\n",
            honest_bench())),
        "approval.record.slaves[0].serial_number");
    expect_file_rejection(
        parse(approved_file("      - { slave: 0, serial_number: 0, revision_number: null, hardware_version: \"" +
                                std::string(32, 'v') + "\", software_version: \"2.07\", pdo_mapping_digest: null }\n",
                            honest_bench())),
        "approval.record.slaves[0].hardware_version");
}

/// scanned_one_drive_yaml() claiming approved with this build's digests and revision.
std::string approved_scanned(std::string_view slaves) {
    const auto digests = digests_of(scanned_one_drive_yaml());
    return edited(scanned_one_drive_yaml(), kUnapprovedBlock,
                  approved_block(digests.config, digests.profile, detail::platform_version(), slaves,
                                 bench_body(digests.config, digests.profile, detail::platform_version(), slaves)));
}

TEST(DriveModelApproval, ThePdoMappingDigestBindsAScannedSlaveOnly) {
    constexpr std::string_view kDigest = "sha256:63f608fc09430a5f804e8c5506e06212013d43b91bdd252031105b90c28e5d8f";
    const std::string digest_only =
        "      - { slave: 0, serial_number: null, revision_number: null, hardware_version: null, software_version: "
        "null, "
        "pdo_mapping_digest: \"" +
        std::string(kDigest) + "\" }\n";
    // A scanned slave: the digest is an observation on its own (rule 4) and travels as declared.
    expect_honored(inspected(approved_scanned(digest_only)));
    // A declared slave: there is no scan to compare with; the record is not honored (rule 2).
    expect_reason(inspected(approved_file(digest_only)),
                  "approval.record.slaves[0].pdo_mapping_digest is set but slaves[0] declares its pdo_mapping");
    // Shape is the file layer's: the key is mandatory (null for a declared slave), the value is `sha256:` + 64 hex.
    expect_file_rejection(parse(approved_file("      - { slave: 0, serial_number: 0, revision_number: null, "
                                              "hardware_version: \"1.3\", software_version: \"2.07\" }\n",
                                              honest_bench())),
                          "approval.record.slaves[0].pdo_mapping_digest");
    expect_file_rejection(parse(approved_scanned("      - { slave: 0, serial_number: null, revision_number: null, "
                                                 "hardware_version: null, software_version: null, "
                                                 "pdo_mapping_digest: \"sha256:abc\" }\n")),
                          "approval.record.slaves[0].pdo_mapping_digest");
    expect_file_rejection(parse(approved_scanned(
                              "      - { slave: 0, serial_number: null, revision_number: null, hardware_version: null, "
                              "software_version: null, pdo_mapping_digest: \"" +
                              std::string(kDigest).replace(7, 1, "G") + "\" }\n")),
                          "approval.record.slaves[0].pdo_mapping_digest");
}

TEST(DriveModelApproval, TheRevisionNumberIsARequiredObservationZeroIncluded) {
    // A declared revision number is a read value, 0 included; unlike serial
    // 0, no SII convention makes it "not recorded". It names a model, never a unit.
    const auto revision_only =
        inspected(approved_file("      - { slave: 0, serial_number: null, revision_number: 0, hardware_version: null, "
                                "software_version: null, pdo_mapping_digest: null }\n"));
    expect_honored(revision_only);
    EXPECT_EQ(revision_only.identity_scope, detail::IdentityScope::MODEL);
    // Shape is the file layer's: the key is mandatory (null when not claimed), the value an integer.
    expect_file_rejection(parse(approved_file("      - { slave: 0, serial_number: 0, hardware_version: \"1.3\", "
                                              "software_version: \"2.07\", pdo_mapping_digest: null }\n",
                                              honest_bench())),
                          "approval.record.slaves[0].revision_number");
    expect_file_rejection(parse(approved_file("      - { slave: 0, serial_number: 0, revision_number: \"0x2\", "
                                              "hardware_version: \"1.3\", software_version: \"2.07\", "
                                              "pdo_mapping_digest: null }\n",
                                              honest_bench())),
                          "approval.record.slaves[0].revision_number");
}

TEST(DriveModelApproval, TheIdentityScopeFollowsTheSerial) {
    EXPECT_EQ(inspected(approved_file()).identity_scope, detail::IdentityScope::MODEL);
    EXPECT_EQ(
        inspected(approved_file("      - { slave: 0, serial_number: null, revision_number: null, hardware_version: "
                                "\"1.3\", software_version: null, pdo_mapping_digest: null }\n"))
            .identity_scope,
        detail::IdentityScope::MODEL);
    EXPECT_EQ(
        inspected(approved_file("      - { slave: 0, serial_number: 0x2a, revision_number: null, hardware_version: "
                                "\"1.3\", software_version: \"2.07\", pdo_mapping_digest: null }\n"))
            .identity_scope,
        detail::IdentityScope::UNIT);
}

TEST(DriveModelApproval, AnEmptyRevisionIsAFileError) {
    // Provenance is not checked, but it is never blank: the bench copies the
    // version `plem drive check` printed.
    const auto result =
        detail::inspect_drive_model(edited(approved_file(), std::string(detail::platform_version()), ""), "test.yaml");
    ASSERT_FALSE(result);
    EXPECT_NE(result.message().find("approval.record.implementation_revision is empty"), std::string::npos)
        << result.message();
}

// The embedded bench evidence (schema §6.1): judgments 7-8

TEST(DriveModelBench, TheEvidenceMustClaimWhatTheRecordSays) {
    // Judgment 7 catches a one-sided edit: the evidence still names what the bench measured.
    const auto digests = digests_of(one_drive_yaml());
    const auto bench_says = [&](std::string_view from, std::string_view to) {
        return inspected(approved_file(kSlave0, edited(honest_bench(), from, to)));
    };
    expect_reason(bench_says(digests.config, "sha256:9999"),
                  "approval.record.bench.claims.config_digest is 'sha256:9999' but the record says '" + digests.config +
                      "'");
    expect_reason(bench_says(digests.profile, "sha256:8888"),
                  "approval.record.bench.claims.profile_digest is 'sha256:8888' but the record says '" +
                      digests.profile + "'");
    expect_reason(bench_says(detail::platform_version(), "0.0.1"),
                  "approval.record.bench.claims.implementation_revision is '0.0.1' but the record says '" +
                      std::string(detail::platform_version()) + "'");
    // The record says slave 0 serial 0; the bench observed serial 3 (or another drive's software).
    expect_reason(
        inspected(approved_file(kSlave0, honest_bench("      - { slave: 0, serial_number: 3, revision_number: "
                                                      "null, hardware_version: \"1.3\", software_version: "
                                                      "\"2.07\", pdo_mapping_digest: null }\n"))),
        "approval.record.bench.claims.slaves differ from approval.record.slaves");
    // A bench that lists one slave twice cannot stand in for the missing one (the set is exact).
    expect_reason(inspected(approved_file(kSlave0, honest_bench(std::string(kSlave0) + std::string(kSlave0)))),
                  "approval.record.bench.claims.slaves differ from approval.record.slaves");
    // It is not authentication: an edit that changes both copies passes it
    // (schema §6: the record detects mistakes, it does not prove a bench ran).
    expect_honored(inspected(edited(approved_file(), "hardware_version: \"1.3\"", "hardware_version: \"1.4\"")));
}

TEST(DriveModelBench, EveryRequiredItemMustPass) {
    const std::string all(kAllPass);
    const auto with_items = [&](const std::string& items) {
        return inspected(approved_file(kSlave0, honest_bench(kSlave0, items)));
    };
    const std::string_view quiesce =
        "  - { id: stop-quiesce, result: PASS, evidence: \"service stop + SIGINT: 0 "
        "left, engaged\" }\n";
    expect_reason(with_items(edited(all, quiesce, "")),
                  "the bench has no 'stop-quiesce' item — every required v1 item must be measured and PASS");
    expect_reason(with_items(edited(all, "id: release-oe, result: PASS", "id: release-oe, result: FAIL")),
                  "the bench reports 'release-oe' as FAIL — every required v1 item must PASS before a record can be "
                  "approved (evidence: OE at 0 Nm, second call no-op)");
    expect_reason(with_items(edited(all, "id: stale-engage, result: PASS", "id: stale-engage, result: NA")),
                  "the bench reports 'stale-engage' as NA");
    // A measured failure is never optional; the opt-out vocabulary is NA.
    expect_reason(with_items(all + "  - { id: fault-reset, result: FAIL, evidence: \"reset never left FAULT\" }\n"),
                  "the bench reports 'fault-reset' as FAIL — a measured failure is not optional");
    // Optional items: NA with its evidence, or any other id, changes nothing.
    expect_honored(with_items(all +
                              "  - { id: fault-reset, result: NA, evidence: \"policy none\" }\n  - { id: my-extra, "
                              "result: PASS, evidence: \"\" }\n"));
    // Judgment 8 is one function; `drive_model_tool approve` runs the same one.
    const auto bench = detail::parse_bench("schema: cia402_bench/v1\n" + honest_bench(), "b.yaml");
    ASSERT_TRUE(bench) << bench.message();
    const auto valid = detail::judge_bench_items(bench.value(), parse(one_drive_yaml()).value());
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

/// `yaml` claiming approved by a bench of it that reports `items`.
std::string approved_as(const std::string& yaml, std::string_view items) {
    const auto d = digests_of(yaml);
    const std::string revision(detail::platform_version());
    return edited(yaml, kUnapprovedBlock,
                  approved_block(d.config, d.profile, revision, kSlave0,
                                 bench_body(d.config, d.profile, revision, kSlave0, items)));
}

TEST(DriveModelBench, TheRequiredItemsFollowTheAxisModes) {
    const std::string cst = one_drive_yaml();
    const std::string csp = as_csp(cst);
    const std::string csv = edited(csp, "mode: csp", "mode: csv");
    const std::string all(kAllPass);
    const std::string decel = "  - { id: stop-decel, result: PASS, evidence: \"0.4 rad/s -> rest at 3.9 rad/s2\" }\n";
    const std::string decel_na = "  - { id: stop-decel, result: NA, evidence: \"no moving axis\" }\n";
    // A moving axis (csp, csv): its stop is measured, and a bench without that item does not hold.
    for (const std::string& moving : {csp, csv}) {
        expect_reason(inspected(approved_as(moving, all)), "the bench has no 'stop-decel' item");
        expect_reason(inspected(approved_as(moving, all + decel_na)), "the bench reports 'stop-decel' as NA");
        expect_honored(inspected(approved_as(moving, all + decel)));
    }
    // For approval, stale-engage may be NA only when the bus has no CSV or CST axis. This does
    // not exempt CSP commands from the runtime freshness watchdog.
    const std::string stale_na = edited(all, "id: stale-engage, result: PASS", "id: stale-engage, result: NA");
    expect_honored(inspected(approved_as(csp, stale_na + decel)));
    expect_reason(inspected(approved_as(csv, stale_na + decel)), "the bench reports 'stale-engage' as NA");
    // A torque-only bus is not moved by the bench: stop-decel is NA, or absent (benches before it).
    expect_honored(inspected(approved_as(cst, all + decel_na)));
    expect_honored(inspected(approved_as(cst, all)));
    EXPECT_EQ(detail::required_bench_items(parse(cst).value()),
              (std::vector<std::string_view>{"assembly-match", "idle-non-oe", "start-rtso", "release-oe",
                                             "stale-engage", "disable-engage", "stop-quiesce"}));
}

TEST(DriveModelBench, AMixedBusWithATorqueAxisNeedsNoStopDecel) {
    // A cst axis makes the bus's stop an immediate disable (no ramp), so the ramp
    // deceleration is not the stop to measure there.
    std::string mixed = drives_yaml({1, 1});
    mixed.replace(mixed.find("mode: cst"), 9, "mode: csp");
    const std::string torque = ", torque_nm: { min: -30.0, max: 40.0 } }";
    mixed.replace(mixed.find(torque), torque.size(), ", acceleration_rad_s2: 4.0 }");
    const auto spec = parse(mixed);
    ASSERT_TRUE(spec) << spec.message();
    EXPECT_EQ(detail::required_bench_items(spec.value()),
              (std::vector<std::string_view>{"assembly-match", "idle-non-oe", "start-rtso", "release-oe",
                                             "stale-engage", "disable-engage", "stop-quiesce"}));
    // A record made before (it carries a stop-decel PASS) and one made now (NA or absent) both hold.
    const std::string all(kAllPass);
    for (const std::string& items :
         {all,
          all + "  - { id: stop-decel, result: NA, evidence: \"bus with a cst axis: stop is an immediate "
                "disable\" }\n",
          all + "  - { id: stop-decel, result: PASS, evidence: \"0.4 rad/s -> rest at 3.9 rad/s2\" }\n"}) {
        const auto bench = detail::parse_bench("schema: cia402_bench/v1\n" + honest_bench(kSlave0, items), "b.yaml");
        ASSERT_TRUE(bench) << bench.message();
        const auto judged = detail::judge_bench_items(bench.value(), spec.value());
        EXPECT_TRUE(judged.is_ok()) << judged.message();
    }
}

TEST(DriveModelBench, TheEvidenceIsShapedByTheFileLayer) {
    // Embedded, the evidence is part of the file: a malformed one is a file error at its key path.
    const std::string honest = honest_bench();
    expect_file_rejection(parse(approved_file(kSlave0, honest + std::string(kAllPass))),
                          "approval.record.bench.items[7].id");
    expect_file_rejection(
        parse(approved_file(kSlave0, edited(honest, "result: PASS, evidence: \"host assembly == file\"",
                                            "result: pass, evidence: \"host assembly == file\""))),
        "approval.record.bench.items[0].result");
    expect_file_rejection(parse(approved_file(kSlave0, edited(honest, "notes: \"\"", "notes: \"\", rig: 3"))),
                          "approval.record.bench.conditions.rig");
    // The schema line belongs to the standalone file only.
    expect_file_rejection(parse(approved_file(kSlave0, "schema: cia402_bench/v1\n" + honest)),
                          "approval.record.bench.schema");
}

TEST(DriveModelBench, TheStandaloneFileIsTheSchemaLineThenTheSameEvidence) {
    const std::string honest = honest_bench();
    const auto read = detail::parse_bench("schema: cia402_bench/v1\n" + honest, "b.yaml");
    ASSERT_TRUE(read) << read.message();
    // Every condition is kept; the approval carries them whole.
    EXPECT_EQ(read.value().conditions.host, "jetson");
    EXPECT_EQ(read.value().conditions.master, "IgH 1.6");
    EXPECT_EQ(read.value().conditions.date, "2026-09-16");
    EXPECT_EQ(read.value().conditions.measured_by, "bench");
    EXPECT_EQ(read.value().conditions.notes, "");
    EXPECT_EQ(read.value().items.size(), 7u);
    const auto v2 = detail::parse_bench("schema: cia402_bench/v2\n" + honest, "b.yaml");
    ASSERT_FALSE(v2);
    EXPECT_NE(v2.message().find("b.yaml: schema is 'cia402_bench/v2' — a bench file says cia402_bench/v1"),
              std::string::npos)
        << v2.message();
    const auto twice = detail::parse_bench("schema: cia402_bench/v1\n" + honest + std::string(kAllPass), "b.yaml");
    ASSERT_FALSE(twice);
    EXPECT_NE(twice.message().find("items[7].id ('assembly-match') is listed twice — one row per item"),
              std::string::npos)
        << twice.message();
}

// Stop declarations (schema §4.4): warnings, never refusals

/// Every object a csp axis on channel 1 is warned about, declared (any value;
/// the warning checks the declaration, not the value).
constexpr std::string_view kStopDeclared =
    "startup_sdo: [{ index: 0x605a, subindex: 0, size: 2, value: 1 }, { index: 0x605c, subindex: 0, size: 2, value: 1 "
    "}, { index: 0x6084, subindex: 0, size: 4, value: 1 }, { index: 0x6085, subindex: 0, size: 4, value: 1 }]";

/// `yaml` identifying its slaves as a drive, not as the synthetic example
/// (the fixtures' vendor_id, which is warned of).
std::string as_a_drive(const std::string& yaml) {
    return edited(yaml, "vendor_id: 0x00c0ffee", "vendor_id: 0x0000abcd");
}

TEST(DriveModelWarnings, ACspAxisIsWarnedOfTheStopObjectsItLeavesToTheDrive) {
    const std::string file = as_a_drive(as_csp(one_drive_yaml()));
    const auto in = inspected(file);
    // The following-error objects (0x6065/0x6066) are not warned: PLEM does not act on
    // statusword bit 13, and on drives like the A6B the window only sets that bit.
    ASSERT_EQ(in.warnings.size(), 1u) << joined(in.warnings);
    EXPECT_EQ(in.warnings[0],
              "axes[0] (j0, csp): slaves[0].startup_sdo sets no 0x605a (Quick stop option code), 0x605c (Disable "
              "operation option code), 0x6084 (Profile deceleration), 0x6085 (Quick stop deceleration) — the drive "
              "keeps the values it has stored: 0x605c with its deceleration (usually 0x6084) decides how the axis "
              "stops when PLEM disables it, and 0x605a with the deceleration it selects (0x6084 or 0x6085) how a "
              "quick stop ends (0x605e, the fault "
              "reaction, is not checked); declare them in the drive model file's startup_sdo");
    EXPECT_TRUE(in.reasons.empty()) << "a warning is never a reason";
    EXPECT_TRUE(parse(file)) << "and never a refusal";
    EXPECT_TRUE(inspected(edited(file, "startup_sdo: []", kStopDeclared)).warnings.empty());
    // Only what is left undeclared is named.
    const auto partly = inspected(edited(file, "startup_sdo: []",
                                         "startup_sdo: [{ index: 0x6084, subindex: 0, size: 4, value: 1 }, { index: "
                                         "0x6085, subindex: 0, size: 4, value: 1 }]"));
    ASSERT_EQ(partly.warnings.size(), 1u) << joined(partly.warnings);
    EXPECT_NE(partly.warnings[0].find("sets no 0x605a (Quick stop option code), 0x605c (Disable operation option "
                                      "code) — "),
              std::string::npos)
        << partly.warnings[0];
}

TEST(DriveModelWarnings, ACsvAxisIsWarnedOfItsStopObjectsOnlyAndACstAxisOfNone) {
    EXPECT_TRUE(inspected(as_a_drive(one_drive_yaml())).warnings.empty());
    const auto in = inspected(edited(as_a_drive(as_csp(one_drive_yaml())), "mode: csp", "mode: csv"));
    ASSERT_EQ(in.warnings.size(), 1u) << joined(in.warnings);
    EXPECT_NE(in.warnings[0].find("axes[0] (j0, csv): slaves[0].startup_sdo sets no 0x605a"), std::string::npos)
        << in.warnings[0];
}

TEST(DriveModelWarnings, TheSecondAxisOfASlaveIsJudgedAtItsOwnChannel) {
    const auto in = inspected(edited(as_a_drive(as_csp(drives_yaml({2}))), "startup_sdo: []", kStopDeclared));
    ASSERT_EQ(in.warnings.size(), 1u) << joined(in.warnings);
    EXPECT_NE(in.warnings[0].find("axes[1] (j1, csp): slaves[0].startup_sdo sets no 0x685a (Quick stop option "
                                  "code), 0x685c"),
              std::string::npos)
        << in.warnings[0];
}

TEST(DriveModelWarnings, ADistributedClockSlaveWithAZeroSync0ShiftIsWarned) {
    // A negative shift is a phase like a positive one (-x fires where period - x does): not warned.
    const std::string maximal = as_a_drive(std::string(kMaximal));
    EXPECT_TRUE(inspected(maximal).warnings.empty()) << joined(inspected(maximal).warnings);
    const auto in = inspected(edited(maximal, "sync0_shift_ns: -1000", "sync0_shift_ns: 0"));
    ASSERT_EQ(in.warnings.size(), 1u) << joined(in.warnings);
    EXPECT_EQ(
        in.warnings[0],
        "slaves[0] (ring position 7): dc.sync0_shift_ns is 0 — SYNC0 then fires when the cycle starts, before "
        "that cycle's frame reaches the drive, so the drive may skip or repeat a cycle's target (torque spikes, "
        "following-error jumps); size the shift as the drive model reference says (docs/cia402-drive-model.md, SYNC0 "
        "shift)");
}

TEST(DriveModelWarnings, EachSlaveWithTheSyntheticExamplesVendorIdIsWarned) {
    const auto in = inspected(drives_yaml({1, 1}));
    ASSERT_EQ(in.warnings.size(), 2u) << joined(in.warnings);
    EXPECT_EQ(in.warnings[0],
              "slaves[0] (ring position 0): vendor_id 0x00c0ffee is the synthetic example's, whose values fit no "
              "real drive — replace the example's values with your drive's (`ethercat slaves -v`, "
              "docs/getting-started.md)");
    EXPECT_EQ(in.warnings[1].rfind("slaves[1] (ring position 1): vendor_id 0x00c0ffee is the synthetic", 0), 0u)
        << in.warnings[1];
    EXPECT_TRUE(in.reasons.empty()) << "a warning is never a reason";
    EXPECT_TRUE(parse(drives_yaml({1, 1}))) << "and never a refusal";
}

// The position limits against what 0x6064 can report (schema §4.4, saturation)

/// j0's position channel in one_drive_yaml(): 65536 counts per motor turn, 100:1.
const fieldbus::ethercat::PositionChannel kJ0Position{fieldbus::ethercat::MotionUnit::ROTARY_MOTOR_ENCODER, 65536,
                                                      100.0};

/// j0's angle at joint count `count`, as the loop decodes it.
double j0_angle(int64_t count) {
    return fieldbus::ethercat::to_angle(core::units::Pulse(static_cast<int32_t>(count)), kJ0Position).value();
}

/// one_drive_yaml() as a drive's, j0 zeroed at `zero` counts, `flip`, travel [min_rad, max_rad] (written
/// shortest-exact, so the file holds these very doubles).
std::string reach_yaml(int32_t zero, bool flip, double min_rad, double max_rad) {
    std::string yaml =
        edited(as_a_drive(one_drive_yaml()), "zero_offset_count: 1234", "zero_offset_count: " + std::to_string(zero));
    yaml = edited(yaml, "flip: false", flip ? "flip: true" : "flip: false");
    return edited(yaml, "position: { min_rad: -1.0, max_rad: 1.5 }",
                  fmt::format("position: {{ min_rad: {}, max_rad: {} }}", min_rad, max_rad));
}

constexpr int64_t kInt32Max = std::numeric_limits<int32_t>::max();
constexpr int64_t kInt32Min = std::numeric_limits<int32_t>::min();

TEST(DriveModelWarnings, ALimitPastWhatTheZeroOffsetLeavesOfTheInt32PositionIsWarned) {
    // 2e9 counts above the drive's 0 leave 147483647 joint counts (141.398 rad) up to INT32_MAX.
    const auto in = inspected(reach_yaml(2'000'000'000, false, -1.0, 150.0));
    ASSERT_EQ(in.warnings.size(), 1u) << joined(in.warnings);
    // The printed range is rounded inward, so a limit copied from it is inside.
    EXPECT_EQ(in.warnings[0],
              "axes[0] (j0): limits.position [-1, 150] rad reaches past what the drive's int32 position (0x6064) "
              "decodes to with zero_offset_count 2000000000 and flip false, [-2058.8741, 141.3981] rad (rounded "
              "inward) — past it the measured position is not the joint's (the joint count saturates, or the drive's "
              "count wraps to the other end), so the limit does not hold there; narrow limits.position to within it "
              "(docs/cia402-drive-model.md, Saturation of the position)");
    EXPECT_TRUE(in.reasons.empty()) << "a warning is never a reason";
    EXPECT_TRUE(parse(reach_yaml(2'000'000'000, false, -1.0, 150.0))) << "and never a refusal";
}

TEST(DriveModelWarnings, AnEndOneCountBelowZeroPrintsAsZeroNotMinusZero) {
    // Flipped with z one below the int32 maximum, the lowest joint count is -1: about 1 µrad below 0.
    const auto in = inspected(reach_yaml(2'147'483'646, true, -1.0, 1.0));
    ASSERT_EQ(in.warnings.size(), 1u) << joined(in.warnings);
    EXPECT_NE(in.warnings[0].find(", [0.0000, "), std::string::npos) << in.warnings[0];
    EXPECT_EQ(in.warnings[0].find("-0.0000"), std::string::npos) << in.warnings[0];
}

TEST(DriveModelWarnings, TheReachableRangeFollowsTheZeroOffsetsSignAndFlipToTheExactCount) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    constexpr int32_t kZero = 2'000'000'000;
    struct Edge {
        int32_t zero;
        bool flip;
        bool upper;     // which end the offset narrows
        int64_t count;  // the last joint count 0x6064 decodes to on that end
    };
    for (const Edge& e : {Edge{kZero, false, true, kInt32Max - kZero}, Edge{-kZero, false, false, kInt32Min + kZero},
                          Edge{kZero, true, false, kZero - kInt32Max}, Edge{-kZero, true, true, -kZero - kInt32Min}}) {
        const std::string where = fmt::format("zero {} flip {}", e.zero, e.flip);
        const double edge = j0_angle(e.count);
        const double past = std::nextafter(edge, e.upper ? kInf : -kInf);
        const auto at =
            inspected(e.upper ? reach_yaml(e.zero, e.flip, -1.0, edge) : reach_yaml(e.zero, e.flip, edge, 1.0));
        EXPECT_TRUE(at.warnings.empty()) << where << joined(at.warnings);
        const auto beyond =
            inspected(e.upper ? reach_yaml(e.zero, e.flip, -1.0, past) : reach_yaml(e.zero, e.flip, past, 1.0));
        ASSERT_EQ(beyond.warnings.size(), 1u) << where << joined(beyond.warnings);
        EXPECT_NE(beyond.warnings[0].find(fmt::format("zero_offset_count {} and flip {}", e.zero, e.flip)),
                  std::string::npos)
            << beyond.warnings[0];
    }
}

TEST(DriveModelWarnings, AnAxisWithoutEndStopsHasNoReachToWarnOf) {
    const std::string endless =
        edited(edited(as_a_drive(one_drive_yaml()), "zero_offset_count: 1234", "zero_offset_count: 0"),
               "position: { min_rad: -1.0, max_rad: 1.5 }", "position: null");
    EXPECT_TRUE(inspected(endless).warnings.empty()) << joined(inspected(endless).warnings);
}

}  // namespace
}  // namespace plem::drive
