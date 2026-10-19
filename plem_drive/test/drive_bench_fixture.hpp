// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_bench_fixture.hpp
/// @brief Test-local (not installed) text fixtures of the approval workflow
/// (schema §6/§6.1): the canonical unapproved block, a bench with every
/// required item PASS (as its own file, or as the evidence an approval
/// embeds), and the shipped drive's identity entry. Shared by the
/// loader's approval-check pins and drive_model_tool's fixture runs, so the bench
/// shape is written once.

#pragma once

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <string_view>

#include <unistd.h>

namespace plem::drive::test {

/// A fresh directory under gtest's TempDir(), removed on destruction, where
/// a model file and the bench it names sit side by side.
struct ScratchDir {
    explicit ScratchDir(std::string_view prefix) {
        static int counter = 0;
        path =
            ::testing::TempDir() + std::string(prefix) + std::to_string(::getpid()) + "_" + std::to_string(counter++);
        std::filesystem::create_directories(path);
    }
    ~ScratchDir() {
        std::filesystem::remove_all(path);
    }
    ScratchDir(const ScratchDir&) = delete;
    ScratchDir& operator=(const ScratchDir&) = delete;
    std::string path;
};

/// The approval block of a file not yet approved: the canonical three lines
/// `drive_model_tool approve` replaces (schema §6).
constexpr std::string_view kUnapprovedBlock = "approval:\n  status: unapproved\n  record: null\n";

constexpr std::string_view kSlave0 =
    "      - { slave: 0, serial_number: 0, revision_number: null, hardware_version: \"1.3\", software_version: "
    "\"2.07\", "
    "pdo_mapping_digest: null }\n";

constexpr std::string_view kAllPass =
    "  - { id: assembly-match, result: PASS, evidence: \"host assembly == file\" }\n"
    "  - { id: idle-non-oe, result: PASS, evidence: \"0x6041 non-OE, position within limits\" }\n"
    "  - { id: start-rtso, result: PASS, evidence: \"RTSO, brakes engaged, pending 0, protective stop 0\" }\n"
    "  - { id: release-oe, result: PASS, evidence: \"OE at 0 Nm, second call no-op\" }\n"
    "  - { id: stale-engage, result: PASS, evidence: \"STALE -> EXTERNAL -> engaged\" }\n"
    "  - { id: disable-engage, result: PASS, evidence: \"0 torque, engaged\" }\n"
    "  - { id: stop-quiesce, result: PASS, evidence: \"service stop + SIGINT: 0 left, engaged\" }\n";

/// The bench evidence (claims, conditions, items) at column 0.
inline std::string bench_body(std::string_view config, std::string_view profile, std::string_view revision,
                              std::string_view slaves, std::string_view items = kAllPass) {
    return "claims:\n  config_digest: \"" + std::string(config) + "\"\n  profile_digest: \"" + std::string(profile) +
           "\"\n  implementation_revision: \"" + std::string(revision) +
           "\"\n  slaves:" + (slaves.empty() ? " []\n" : "\n" + std::string(slaves)) +
           "conditions: { host: jetson, master: \"IgH 1.6\", date: \"2026-09-16\", measured_by: bench, notes: \"\" "
           "}\nitems:\n" +
           std::string(items);
}

/// The standalone bench file `drive_model_tool approve` reads: the schema line, then the evidence.
inline std::string bench_yaml(std::string_view config, std::string_view profile, std::string_view revision,
                              std::string_view slaves, std::string_view items = kAllPass) {
    return "schema: cia402_bench/v1\n" + bench_body(config, profile, revision, slaves, items);
}

/// `text` with every line indented by `columns` spaces.
inline std::string indented(std::string_view text, size_t columns) {
    std::string out;
    size_t from = 0;
    while (from < text.size()) {
        const size_t to = text.find('\n', from);
        const size_t end = to == std::string_view::npos ? text.size() : to + 1;
        out += std::string(columns, ' ') + std::string(text.substr(from, end - from));
        from = end;
    }
    return out;
}

}  // namespace plem::drive::test
