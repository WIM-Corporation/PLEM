// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_model_tool.cpp
/// @brief drive_model_tool's `check` / `approve` over fixtures: the stdout contract,
/// every refusal with how to fix it, and the in-place replacement that leaves the
/// rest of the file byte-identical.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <unistd.h>

#include <nlohmann/json.hpp>

#include "drive_bench_fixture.hpp"
#include "loader/digest.hpp"
#include "loader/inspection.hpp"
#include "loader/reader.hpp"
#include "tool/drive_model_tool.hpp"

namespace plem::drive {
namespace {

using tool::kToolOk;
using tool::kToolRefused;
using tool::kToolUsage;
using tool::tool_approve;
using tool::tool_check;

struct ToolRun {
    int rc = -1;
    std::string out;
    std::string err;
    [[nodiscard]] bool out_has(std::string_view line) const {
        return out.find(line) != std::string::npos;
    }
    [[nodiscard]] bool err_has(std::string_view text) const {
        return err.find(text) != std::string::npos;
    }
    /// stdout as the JSON it must be (a parse failure fails the test).
    [[nodiscard]] nlohmann::json json() const {
        return nlohmann::json::parse(out);
    }
};

/// `check --json <args>`: the machine contract.
ToolRun check(std::vector<std::string> args) {
    args.insert(args.begin(), "--json");
    std::ostringstream out;
    std::ostringstream err;
    const int rc = tool_check(args, out, err);
    return {rc, out.str(), err.str()};
}

/// `check <args>`: what a person reads.
ToolRun summary(const std::vector<std::string>& args) {
    std::ostringstream out;
    std::ostringstream err;
    const int rc = tool_check(args, out, err);
    return {rc, out.str(), err.str()};
}

ToolRun approve(const std::vector<std::string>& args) {
    std::ostringstream out;
    std::ostringstream err;
    const int rc = tool_approve(args, out, err);
    return {rc, out.str(), err.str()};
}

/// What a shell command prints (its stdout, or its stderr with `2>&1 1>/dev/null`).
std::string shell(const std::string& command) {
    std::string out;
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) {
        return out;
    }
    char buffer[4096];
    while (const size_t n = fread(buffer, 1, sizeof buffer, pipe)) {
        out.append(buffer, n);
    }
    pclose(pipe);
    return out;
}

std::string slurp(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// The synthetic example's three slaves, as a bench observes them.
constexpr std::string_view kThreeSlaves =
    "      - { slave: 0, serial_number: 0, revision_number: 2, hardware_version: \"1.3\", software_version: \"2.07\", "
    "pdo_mapping_digest: null }\n"
    "      - { slave: 1, serial_number: 0, revision_number: null, hardware_version: \"1.3\", software_version: "
    "\"2.07\", "
    "pdo_mapping_digest: null }\n"
    "      - { slave: 2, serial_number: 0, revision_number: null, hardware_version: \"1.3\", software_version: "
    "\"2.07\", "
    "pdo_mapping_digest: null }\n";

/// A fresh directory holding a copy of drive_synthetic.yaml (the unapproved
/// original, comments and all) and a bench that claims exactly what this
/// software computes for it. Removed on destruction.
struct Workshop {
    Workshop() : dir("drive_model_tool_") {
        model = dir.path + "/drive.yaml";
        bench = dir.path + "/bench.yaml";
        // The example ends with the block; a comment after it pins that the
        // tail survives the replacement too.
        original = slurp(DRIVE_EXAMPLE_YAML) + "# trailing note kept by approve\n";
        std::ofstream(model) << original;
        const auto inspected =
            detail::inspect_drive_model(detail::read_text_file(model, "drive model file").value(), model);
        EXPECT_TRUE(inspected) << inspected.message();
        digests = inspected.value().digests;
        write_bench(test::bench_yaml(digests.config, digests.profile, detail::platform_version(), kThreeSlaves));
    }
    void write_bench(const std::string& text) {
        std::ofstream(bench) << text;
    }
    void write_model(const std::string& text) {
        std::ofstream(model) << text;
    }
    test::ScratchDir dir;
    std::string model, bench, original;
    detail::DriveModelDigests digests;
};

TEST(PlemDriveCheck, PrintsTheJsonContractTheGoldenFilePins) {
    // drive_synthetic.yaml: slaves at ring 5, 0, 2; j0 on channel 2 of slave 0.
    // Only the release varies between builds.
    const ToolRun run = check({DRIVE_EXAMPLE_YAML});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    nlohmann::json printed = run.json();
    EXPECT_EQ(printed.at("implementation_revision"), detail::platform_version());
    printed["implementation_revision"] = "<this build>";
    EXPECT_EQ(printed, nlohmann::json::parse(slurp(CHECK_GOLDEN_JSON))) << printed.dump(2);
    EXPECT_TRUE(run.err.empty()) << run.err;
    // A torque-only bus: every required item but stop-decel (the bench moves no torque axis).
    std::vector<std::string> required(std::begin(detail::kRequiredBenchItems), std::end(detail::kRequiredBenchItems));
    std::erase(required, "stop-decel");
    EXPECT_EQ(printed.at("required_bench_items"), required) << "the golden file is this software's list";
}

/// `text` with every `from` replaced by `to`.
std::string replaced(std::string text, std::string_view from, std::string_view to) {
    for (size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
        text.replace(at, from.size(), to);
    }
    return text;
}

/// `text` identifying its slaves as drives, not as the synthetic example (the
/// fixture's vendor_id, which check warns of and sends to replace its values).
std::string as_a_drive(const std::string& text) {
    return replaced(text, "vendor_id: 0x00c0ffee", "vendor_id: 0x0000abcd");
}

TEST(PlemDriveCheck, PrintsTheSummaryTheGoldenFilePins) {
    // Without --json: the bus, its axes, the approval and the one next step;
    // no digests. The file path varies between runs.
    const ToolRun run = summary({DRIVE_EXAMPLE_YAML});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    // A checkout path a shell would split is printed quoted: either form is <file>.
    const std::string printed =
        replaced(replaced(run.out, "'" DRIVE_EXAMPLE_YAML "'", "<file>"), DRIVE_EXAMPLE_YAML, "<file>");
    EXPECT_EQ(printed, slurp(CHECK_SUMMARY_GOLDEN)) << run.out;
    EXPECT_EQ(run.out.find("sha256:"), std::string::npos) << run.out;
    EXPECT_TRUE(run.err.empty()) << run.err;
}

TEST(PlemDriveCheck, TheSummaryNamesEachReasonAndWarningAndTheExitCodeIsTheJsons) {
    Workshop w;
    ASSERT_EQ(approve({w.model, w.bench}).rc, kToolOk);
    const ToolRun approved = summary({w.model});
    EXPECT_EQ(approved.rc, kToolOk) << approved.err;
    // Every bench serial number is 0: the approval binds the model, and the summary says what that opens.
    EXPECT_TRUE(
        approved.out_has("\napproval: approved for the model — a serial number in the record is 0 or null, "
                         "so any drive that matches the identity values the record holds opens motion too\n"))
        << approved.out;
    EXPECT_TRUE(approved.out_has("\nnext: plem drive run " + w.model + "\n")) << approved.out;
    // A record an edit made stale: not approved, each reason on its own line, exit 1 like --json.
    std::string text = as_a_drive(slurp(w.model));
    text.replace(text.find("zero_offset_count: 1234"), 23, "zero_offset_count: 1235");
    text.replace(text.find("mode: cst"), 9, "mode: csp");
    const std::string torque = ", torque_nm: { min: -30.0, max: 40.0 } }\n";
    text.replace(text.find(torque), torque.size(), ", acceleration_rad_s2: 4.0 }\n");
    w.write_model(text);
    const ToolRun stale = summary({w.model});
    const ToolRun as_json = check({w.model});
    EXPECT_EQ(stale.rc, kToolRefused);
    EXPECT_EQ(stale.rc, as_json.rc);
    EXPECT_TRUE(stale.out_has("\napproval: not approved — the approval the file claims does not hold:\n")) << stale.out;
    for (const auto& reason : as_json.json().at("reasons")) {
        EXPECT_TRUE(stale.out_has("\n  " + reason.get<std::string>() + "\n")) << stale.out;
    }
    ASSERT_EQ(as_json.json().at("warnings").size(), 1u);
    EXPECT_TRUE(stale.out_has("\nwarning: " + as_json.json().at("warnings").at(0).get<std::string>() + "\n"))
        << stale.out;
    EXPECT_TRUE(stale.out_has("\nnext: plem drive run " + w.model +
                              " — on a terminal it offers the bench at the drives, which records the approval\n"))
        << stale.out;
}

TEST(PlemDriveCheck, TheSyntheticExampleIsSentToReplaceItsValuesNotToRun) {
    // drive_synthetic.yaml keeps the example's vendor_id on its three slaves.
    const ToolRun run = summary({DRIVE_EXAMPLE_YAML});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    const nlohmann::json warnings = check({DRIVE_EXAMPLE_YAML}).json().at("warnings");
    ASSERT_EQ(warnings.size(), 3u) << warnings;
    for (const auto& warning : warnings) {
        EXPECT_NE(warning.get<std::string>().find("vendor_id 0x00c0ffee is the synthetic example's"), std::string::npos)
            << warning;
        EXPECT_TRUE(run.out_has("\nwarning: " + warning.get<std::string>() + "\n")) << run.out;
    }
    EXPECT_FALSE(run.out_has("\nnext: plem drive run")) << run.out;
    EXPECT_TRUE(
        run.out_has("\nnext: replace the example's values with your drive's (`ethercat slaves -v`, "
                    "docs/getting-started.md), then check the file again\n"))
        << run.out;
    // A drive's identity is sent to run (and its bench).
    Workshop w;
    w.write_model(as_a_drive(w.original));
    EXPECT_TRUE(summary({w.model}).out_has("\nnext: plem drive run " + w.model + " — on a terminal"))
        << summary({w.model}).out;
}

TEST(PlemDriveCheck, TheSummaryCountsOneAxisAndQuotesAPathAShellWouldSplit) {
    // drive_synthetic.yaml cut to its ring-0 slave (one axis, j2), approved
    // with a serial number: bound to that unit.
    Workshop w;
    const std::string& full = w.original;
    const std::string one =
        as_a_drive(full.substr(0, full.find("  - ring_position: 5")) +
                   full.substr(full.find("  - ring_position: 0"),
                               full.find("  - ring_position: 2") - full.find("  - ring_position: 0")) +
                   full.substr(full.find(test::kUnapprovedBlock)));
    const std::string path = w.dir.path + "/it's mine.yaml";
    std::ofstream(path) << one;
    const ToolRun unapproved = summary({path});
    ASSERT_EQ(unapproved.rc, kToolOk) << unapproved.err;
    EXPECT_EQ(unapproved.out.rfind("bus synthetic: 1 axis on 1 drive\n", 0), 0u) << unapproved.out;
    const std::string quoted = "'" + w.dir.path + "/it'\\''s mine.yaml'";
    EXPECT_TRUE(unapproved.out_has("\nnext: plem drive run " + quoted + " — on a terminal")) << unapproved.out;
    EXPECT_TRUE(unapproved.out_has("\ndetails: plem drive check --json " + quoted + "\n")) << unapproved.out;
    const auto inspected = detail::inspect_drive_model(one, path);
    ASSERT_TRUE(inspected) << inspected.message();
    const std::string serial =
        "      - { slave: 0, serial_number: 4711, revision_number: 2, hardware_version: \"1.3\", software_version: "
        "\"2.07\", pdo_mapping_digest: null }\n";
    std::ofstream(w.bench) << test::bench_yaml(inspected.value().digests.config, inspected.value().digests.profile,
                                               detail::platform_version(), serial);
    ASSERT_EQ(approve({path, w.bench}).rc, kToolOk);
    const ToolRun approved = summary({path});
    EXPECT_TRUE(approved.out_has("\napproval: approved for this unit — the record holds its serial number\n"))
        << approved.out;
}

TEST(PlemDriveCheck, TheSummaryWritesARelativePathStartingWithADashAsAPath) {
    // `plem drive run -x.yaml` would read the file as an option.
    const std::string dash = "-drive_model_tool_summary.yaml";
    std::ofstream(dash) << as_a_drive(slurp(DRIVE_EXAMPLE_YAML));
    const ToolRun run = summary({dash});
    std::filesystem::remove(dash);
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_TRUE(run.out_has("\nnext: plem drive run ./" + dash + " — on a terminal")) << run.out;
    EXPECT_TRUE(run.out_has("\ndetails: plem drive check --json ./" + dash + "\n")) << run.out;
}

TEST(PlemDriveCheck, RefusesAFileWithoutABusNameSayingWhatToWrite) {
    // The loader takes a file without one (the robot tier's); a bus that
    // plem drive run starts is named by it.
    Workshop w;
    std::string text = slurp(DRIVE_EXAMPLE_YAML);
    const std::string_view name = "name: synthetic, ";
    ASSERT_NE(text.find(name), std::string::npos);
    w.write_model(text.replace(text.find(name), name.size(), ""));
    const ToolRun run = check({w.model});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_TRUE(run.out.empty()) << run.out;
    EXPECT_NE(run.err.find(w.model + ": bus.name is missing — name the bus: bus: { name: <bus>, "), std::string::npos)
        << run.err;
    EXPECT_NE(run.err.find("plem drive run starts it as /<bus>"), std::string::npos) << run.err;
}

TEST(PlemDriveCheck, ANullTravelPrintsAsNullLimits) {
    Workshop w;
    std::string text = slurp(DRIVE_EXAMPLE_YAML);  // j0 is the file's first axis
    text.replace(text.find("zero_offset_count: 1234"), 23, "zero_offset_count: 0");
    const std::string travel = "position: { min_rad: -1.0, max_rad: 1.5 }";
    text.replace(text.find(travel), travel.size(), "position: null");
    w.write_model(text);
    const ToolRun run = check({w.model});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    const nlohmann::json j0 = run.json().at("axes").at(0);
    EXPECT_TRUE(j0.at("position_min_rad").is_null()) << j0;
    EXPECT_TRUE(j0.at("position_max_rad").is_null()) << j0;
}

TEST(PlemDriveCheck, PrintsACspAxisWithItsWarnings) {
    Workshop w;
    std::string text = as_a_drive(slurp(DRIVE_EXAMPLE_YAML));  // j0 is the file's first axis
    text.replace(text.find("mode: cst"), 9, "mode: csp");
    const std::string torque = ", torque_nm: { min: -30.0, max: 40.0 } }\n";  // a cst axis's key only
    text.replace(text.find(torque), torque.size(), ", acceleration_rad_s2: 4.0 }\n");
    w.write_model(text);
    const ToolRun run = check({w.model});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    const nlohmann::json out = run.json();
    EXPECT_EQ(out.at("axes").at(0).at("mode"), "csp");
    EXPECT_EQ(out.at("axes").at(0).at("acceleration_limit_rad_s2"), 4.0);
    // A planner's speed: the limit less a cycle's count rounding on a csp axis (the loop's step check).
    EXPECT_LT(out.at("axes").at(0).at("velocity_cap_rad_s").get<double>(), 2.0);
    EXPECT_GT(out.at("axes").at(0).at("velocity_cap_rad_s").get<double>(), 1.9);
    EXPECT_EQ(out.at("axes").at(1).at("velocity_cap_rad_s"), 2.0) << "not a csp axis: the limit";
    EXPECT_TRUE(out.at("axes").at(1).at("acceleration_limit_rad_s2").is_null()) << "a cst axis";
    // The torque clamp: a cst axis's only (a stream client checks its commands against it).
    EXPECT_TRUE(out.at("axes").at(0).at("torque_min_nm").is_null()) << "a csp axis";
    EXPECT_TRUE(out.at("axes").at(0).at("torque_max_nm").is_null()) << "a csp axis";
    EXPECT_EQ(out.at("axes").at(1).at("torque_min_nm"), -30.0);
    EXPECT_EQ(out.at("axes").at(1).at("torque_max_nm"), 40.0);
    // j0 is channel 2 of its slave: the objects it leaves to the drive are
    // named at 0x68xx: a warning, not a refusal (exit 0, no approve blocker).
    ASSERT_EQ(out.at("warnings").size(), 1u) << out.at("warnings");
    EXPECT_EQ(out.at("warnings")
                  .at(0)
                  .get<std::string>()
                  .rfind("axes[0] (j0, csp): slaves[0].startup_sdo sets no 0x685a (Quick stop option code)", 0),
              0u);
    EXPECT_TRUE(out.at("approve_blocker").is_null());
}

TEST(PlemDriveCheck, PrintsARatedPerMilleAxissRatedTorqueInMilliNewtonMetres) {
    // 0x6076's unit, six significant digits; the bench compares it with the
    // drive. Other axes carry null.
    Workshop w;
    std::string text = slurp(DRIVE_EXAMPLE_YAML);  // j0 is the file's first axis
    const std::string scaled = "{ unit: scaled_current, counts_per_amp: 50.0, torque_constant: 0.12 }";
    text.replace(text.find(scaled), scaled.size(), "{ unit: rated_per_mille, motor_rated_torque_nm: 1.2704 }");
    w.write_model(text);
    const ToolRun run = check({w.model});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_NE(run.out.find("\"rated_torque_mnm\": 1270.4"), std::string::npos) << "no binary noise: " << run.out;
    EXPECT_TRUE(run.json().at("axes").at(1).at("rated_torque_mnm").is_null());
    // A value under 0.5 mNm (the 1000×-too-small entry) must not print as 0.
    text.replace(text.find("motor_rated_torque_nm: 1.2704"), 29, "motor_rated_torque_nm: 0.00016");
    w.write_model(text);
    EXPECT_EQ(check({w.model}).json().at("axes").at(0).at("rated_torque_mnm"), 0.16);
}

TEST(PlemDriveCheck, PrintsTheObjectsEachSlavesStartupSdoWrites) {
    Workshop w;
    std::string text = slurp(DRIVE_EXAMPLE_YAML);  // slave 0's list is the file's first
    text.replace(text.find("startup_sdo: []"), 15,
                 "startup_sdo: [{ index: 0x607e, subindex: 0, size: 1, value: 0x80 }, { index: 0x6091, subindex: 2, "
                 "size: 4, value: 1 }]");
    w.write_model(text);
    const ToolRun run = check({w.model});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_EQ(run.json().at("slaves").at(0).at("startup_sdo"),
              nlohmann::json::parse("[[24702, 0, 128], [24721, 2, 1]]"));
    EXPECT_EQ(run.json().at("slaves").at(1).at("startup_sdo"), nlohmann::json::array());
}

TEST(PlemDriveCheck, PrintsEachAxissFlip) {
    // The bench compares the drive's own polarity (0x607E) with it.
    Workshop w;
    std::string text = slurp(DRIVE_EXAMPLE_YAML);  // j0 is the file's first axis
    text.replace(text.find("flip: false"), 11, "flip: true");
    w.write_model(text);
    const ToolRun run = check({w.model});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_EQ(run.json().at("axes").at(0).at("flip"), true);
    EXPECT_EQ(run.json().at("axes").at(1).at("flip"), false);
}

TEST(PlemDriveCheck, NamesAScannedSlaveAndTakesAOneLineApprovalBlock) {
    Workshop w;
    std::string yaml = w.original;
    const size_t from = yaml.find("    pdo_mapping:");  // slave 0's (it carries a trailing comment)
    const size_t to = yaml.find("    startup_sdo:", from);
    ASSERT_NE(from, std::string::npos);
    ASSERT_NE(to, std::string::npos);
    w.write_model(yaml.replace(from, to - from, "    pdo_mapping: scanned\n"));
    const ToolRun scanned = check({w.model});
    ASSERT_EQ(scanned.rc, kToolOk) << scanned.err;
    EXPECT_EQ(scanned.json().at("slaves").at(0).at("pdo_mapping"), "scanned");
    // A one-line block is a block approve can rewrite.
    w.write_model(w.original.substr(0, w.original.find(test::kUnapprovedBlock)) +
                  "approval: { status: unapproved, record: null }\n");
    const ToolRun flow = check({w.model});
    ASSERT_EQ(flow.rc, kToolOk) << flow.err;
    EXPECT_TRUE(flow.json().at("approve_blocker").is_null()) << flow.out;
    ASSERT_EQ(approve({w.model, w.bench}).rc, kToolOk);
    EXPECT_EQ(check({w.model}).json().at("approval_effective"), "approved");
}

TEST(PlemDriveCheck, RenamingAnApprovedBusKeepsItsApproval) {
    // bus.name is not in config_digest: the same declaration under another
    // name is the same approval.
    Workshop w;
    ASSERT_EQ(approve({w.model, w.bench}).rc, kToolOk);
    const nlohmann::json before = check({w.model}).json();
    std::string text = slurp(w.model);
    const std::string_view name = "name: synthetic,";
    ASSERT_NE(text.find(name), std::string::npos);
    w.write_model(text.replace(text.find(name), name.size(), "name: left_pair,"));
    const ToolRun renamed = check({w.model});
    ASSERT_EQ(renamed.rc, kToolOk) << renamed.err;
    const nlohmann::json after = renamed.json();
    EXPECT_EQ(before.at("approval_effective"), "approved");
    EXPECT_EQ(after.at("approval_effective"), "approved") << after.at("reasons");
    EXPECT_EQ(after.at("bus_name"), "left_pair");
    EXPECT_EQ(after.at("config_digest"), before.at("config_digest"));
}

TEST(PlemDriveCheck, RefusesWhatTheBusWouldRefuseAtStartup) {
    // A declared layout without j0's target torque (channel 2 → 0x6871): the
    // loader reads it, the bus would refuse it before any master contact;
    // check says so without the drives.
    Workshop w;
    std::string text = w.original;
    const std::string entry = "                - { index: 0x6871, subindex: 0, bits: 16 }\n";
    text.erase(text.find(entry), entry.size());
    w.write_model(text);
    const ToolRun run = check({w.model});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_TRUE(run.err_has("slave 5 (axes 0,1): declared PDO layout rejected")) << run.err;
    EXPECT_TRUE(run.err_has("0x6871")) << run.err;
    EXPECT_TRUE(run.err_has("(slave n = the slave at ring position n)")) << run.err;
    EXPECT_TRUE(run.out.empty()) << run.out;
}

TEST(PlemDriveCheck, UsageAndRefusalsGoToStderrWithTheirCodes) {
    EXPECT_EQ(check({}).rc, kToolUsage);
    const ToolRun help = check({"--help"});
    EXPECT_EQ(help.rc, kToolOk);
    EXPECT_TRUE(help.out_has("usage: plem drive check")) << help.out;
    // approve is the bench's internal step: a user's help and usage name check only.
    EXPECT_FALSE(help.out_has("approve <model.yaml>")) << help.out;
    EXPECT_EQ(check({"a", "b"}).rc, kToolUsage);
    EXPECT_FALSE(check({"a", "b"}).err_has("approve <model.yaml>"));
    // Its own usage names the executable it is (no `plem drive approve` command exists).
    const ToolRun approve_help = approve({"-h"});
    EXPECT_EQ(approve_help.rc, kToolOk);
    EXPECT_TRUE(approve_help.out_has("usage: drive_model_tool approve <model.yaml> <bench.yaml>")) << approve_help.out;
    EXPECT_TRUE(approve_help.out_has("(`plem drive run` offers the bench)")) << approve_help.out;
    EXPECT_FALSE(approve_help.out_has("plem-drive")) << approve_help.out;
    const ToolRun missing = check({"/nonexistent/drive.yaml"});
    EXPECT_EQ(missing.rc, kToolRefused);
    EXPECT_TRUE(missing.err_has("cannot load drive model file '/nonexistent/drive.yaml'")) << missing.err;
    EXPECT_TRUE(missing.out.empty());
}

TEST(PlemDriveApprove, WritesTheRecordIntoTheModelInPlace) {
    const Workshop w;
    const ToolRun run = approve({w.model, w.bench});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_TRUE(run.out_has("approved: " + w.model + "\n")) << run.out;
    EXPECT_TRUE(run.out_has("identity_scope: model\n")) << run.out;
    EXPECT_EQ(std::distance(std::filesystem::directory_iterator(w.dir.path), {}), 2) << "no temporary file is left";

    // Everything but the approval block is byte-identical, the comment right
    // before the block and the note after it included.
    const std::string result = slurp(w.model);
    const size_t at = w.original.find(test::kUnapprovedBlock);
    ASSERT_NE(at, std::string::npos);
    EXPECT_EQ(result.substr(0, at), w.original.substr(0, at));
    const std::string tail = w.original.substr(at + test::kUnapprovedBlock.size());
    EXPECT_EQ(tail, "# trailing note kept by approve\n");
    EXPECT_EQ(result.substr(result.size() - tail.size()), tail);
    EXPECT_NE(result.find("# Not yet approved: the bench's approval step writes the approval record here."),
              std::string::npos);
    EXPECT_NE(result.find("approval:\n  status: approved\n  record:\n    config_digest: \"" + w.digests.config + "\""),
              std::string::npos)
        << result;
    // The bench evidence, whole: its claims, every condition, every item.
    EXPECT_NE(result.find("    bench:\n      claims:\n        config_digest: \"" + w.digests.config + "\"\n"),
              std::string::npos)
        << result;
    EXPECT_NE(result.find("      conditions: { host: \"jetson\", master: \"IgH 1.6\", date: \"2026-09-16\", "
                          "measured_by: \"bench\", notes: \"\" }\n      items:\n        - { id: \"assembly-match\", "
                          "result: PASS, evidence: \"host assembly == file\" }\n"),
              std::string::npos)
        << result;
    // Each slave line carries the five claimed values as the bench wrote them: the revision number in
    // the master's 0x%08x spelling, the digest null here (the example declares every mapping).
    EXPECT_NE(
        result.find("    slaves:\n      - { slave: 0, serial_number: 0, revision_number: 0x00000002, hardware_version: "
                    "\"1.3\", software_version: "
                    "\"2.07\", pdo_mapping_digest: null }\n      - { slave: 1, "),
        std::string::npos)
        << result;

    const ToolRun approved = check({w.model});
    EXPECT_EQ(approved.rc, kToolOk) << approved.out << approved.err;
    const nlohmann::json judged = approved.json();
    EXPECT_EQ(judged.at("approval_claimed"), "approved") << approved.out;
    EXPECT_EQ(judged.at("approval_effective"), "approved") << approved.out;
    EXPECT_EQ(judged.at("identity_scope"), "model") << approved.out;
    EXPECT_EQ(judged.at("config_digest"), w.digests.config) << "approval is not in the digest";

    // One file to keep: without the bench file, moved elsewhere, it still reads back approved.
    std::filesystem::remove(w.bench);
    const std::string moved = w.dir.path + "/sub/moved.yaml";
    std::filesystem::create_directories(w.dir.path + "/sub");
    std::filesystem::rename(w.model, moved);
    EXPECT_EQ(check({moved}).json().at("approval_effective"), "approved");
}

TEST(PlemDriveApprove, AppendsTheBlockToAFileWithoutOne) {
    Workshop w;
    std::string bare = w.original;
    bare.erase(bare.find(test::kUnapprovedBlock), test::kUnapprovedBlock.size());
    for (const std::string& text : {bare, bare.substr(0, bare.size() - 1)}) {  // with and without a last line ending
        w.write_model(text);
        const ToolRun before = check({w.model});
        ASSERT_EQ(before.rc, kToolOk) << before.err;
        EXPECT_EQ(before.json().at("approval_claimed"), "unapproved");
        EXPECT_TRUE(before.json().at("approve_blocker").is_null()) << before.out;
        const ToolRun run = approve({w.model, w.bench});
        ASSERT_EQ(run.rc, kToolOk) << run.err;
        const std::string result = slurp(w.model);
        EXPECT_EQ(result.rfind(bare + "approval:\n  status: approved\n  record:\n", 0), 0u) << result;
        EXPECT_EQ(result.back(), '\n');
        EXPECT_EQ(check({w.model}).json().at("approval_effective"), "approved");
    }
}

TEST(PlemDriveApprove, ReplacesAnApprovedBlockOnARebench) {
    // A record made stale by an edit, or a fresh bench of an approved file:
    // the new record replaces the old one, the rest of the file stays.
    Workshop w;
    ASSERT_EQ(approve({w.model, w.bench}).rc, kToolOk);
    std::string edited = slurp(w.model);
    edited.replace(edited.find("zero_offset_count: 1234"), 23, "zero_offset_count: 1235");
    w.write_model(edited);
    const ToolRun stale = check({w.model});
    EXPECT_EQ(stale.rc, kToolRefused);
    EXPECT_EQ(stale.json().at("approval_claimed"), "approved");
    EXPECT_EQ(stale.json().at("approval_effective"), "unapproved");
    EXPECT_TRUE(stale.json().at("approve_blocker").is_null()) << stale.out;
    const std::string config = stale.json().at("config_digest");
    std::string bench = test::bench_yaml(config, w.digests.profile, detail::platform_version(), kThreeSlaves);
    bench.replace(bench.find("notes: \"\""), 9, "notes: \"second\"");
    w.write_bench(bench);
    const ToolRun run = approve({w.model, w.bench});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    const std::string result = slurp(w.model);
    const size_t at = w.original.find(test::kUnapprovedBlock);
    EXPECT_EQ(result.substr(0, at), edited.substr(0, at));
    EXPECT_EQ(result.substr(result.size() - 32), "# trailing note kept by approve\n");
    EXPECT_NE(result.find("notes: \"second\""), std::string::npos);
    EXPECT_EQ(result.find("status: approved"), result.rfind("status: approved")) << "one block: " << result;
    EXPECT_EQ(check({w.model}).json().at("approval_effective"), "approved");
}

TEST(PlemDriveApprove, ReplacesABlockWithABlankLineInsideIt) {
    // YAML allows a blank line between a block's keys: the whole block is
    // replaced, and a blank line after it stays outside it.
    Workshop w;
    std::string spaced = w.original;
    const std::string block(test::kUnapprovedBlock);
    spaced.replace(spaced.find(block), block.size(), "approval:\n  status: unapproved\n\n  record: null\n\n");
    w.write_model(spaced);
    const ToolRun before = check({w.model});
    ASSERT_EQ(before.rc, kToolOk) << before.err;
    EXPECT_TRUE(before.json().at("approve_blocker").is_null()) << before.out;
    const ToolRun run = approve({w.model, w.bench});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    const std::string result = slurp(w.model);
    EXPECT_EQ(result.find("record: null"), std::string::npos) << result;
    EXPECT_NE(result.find("\n\n# trailing note kept by approve\n"), std::string::npos) << result;
    EXPECT_EQ(check({w.model}).json().at("approval_effective"), "approved");
}

TEST(PlemDriveApprove, AWhitespaceOnlyLineCountsAsBlank) {
    // A line of spaces is a blank line: it sets no indentation, and
    // one after the block's last key stays outside it.
    Workshop w;
    std::string spaced = w.original;
    const std::string block(test::kUnapprovedBlock);
    spaced.replace(spaced.find(block), block.size(),
                   "approval:\n      \n  status: unapproved\n   \n  record: null\n  \n");
    w.write_model(spaced);
    ASSERT_EQ(check({w.model}).rc, kToolOk);
    const ToolRun run = approve({w.model, w.bench});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    const std::string result = slurp(w.model);
    EXPECT_NE(result.find("\napproval:\n  status: approved\n"), std::string::npos) << result;
    EXPECT_EQ(result.find("record: null"), std::string::npos) << result;
    EXPECT_NE(result.find("\n  \n# trailing note kept by approve\n"), std::string::npos) << result;
    EXPECT_EQ(check({w.model}).json().at("approval_effective"), "approved");
}

TEST(PlemDriveApprove, KeepsTheFilesPermissionsAndWritesThroughASymlink) {
    Workshop w;
    namespace fs = std::filesystem;
    fs::permissions(w.model, fs::perms::owner_read | fs::perms::owner_write | fs::perms::group_read);
    const std::string link = w.dir.path + "/link.yaml";
    fs::create_symlink(w.model, link);
    const ToolRun run = approve({link, w.bench});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_TRUE(fs::is_symlink(link)) << "the link stays a link";
    EXPECT_EQ(check({w.model}).json().at("approval_effective"), "approved");
    EXPECT_EQ(fs::status(w.model).permissions(),
              fs::perms::owner_read | fs::perms::owner_write | fs::perms::group_read);
}

TEST(PlemDriveApprove, TheEvidenceReadsBackAsTheBenchWroteIt) {
    // Free text a double-quoted scalar cannot hold raw (quotes, backslashes, C1 characters (U+0085 is a line
    // break), the line/paragraph separators, the noncharacters U+FFFE/U+FFFF) survives the round trip, and so does
    // non-ASCII text.
    Workshop w;
    const std::string awkward =
        "\"quoted\" \\ back \xc2\x85 nel \xc2\x9f c1 \xe2\x80\xa8 ls \xe2\x80\xa9 ps \xef\xbf\xbe fffe \xef\xbf\xbf "
        "ffff Übergröße";
    const std::string escaped = R"("\"quoted\" \\ back \N nel \x9f c1 \L ls \P ps \uFFFE fffe \uFFFF ffff Übergröße")";
    std::string yaml = test::bench_yaml(w.digests.config, w.digests.profile, detail::platform_version(), kThreeSlaves);
    yaml.replace(yaml.find("notes: \"\""), 9, "notes: " + escaped);
    yaml.replace(yaml.find("evidence: \"host assembly == file\""), 33, "evidence: " + escaped);
    w.write_bench(yaml);
    const auto input = detail::parse_bench(yaml, "bench.yaml");
    ASSERT_TRUE(input) << input.message();
    ASSERT_EQ(input.value().conditions.notes, awkward);
    const ToolRun run = approve({w.model, w.bench});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    const auto tree = detail::parse_tree(slurp(w.model));
    ASSERT_TRUE(tree) << tree.message();
    const auto projected = detail::project(tree.value(), w.model);
    ASSERT_TRUE(projected) << projected.message();
    ASSERT_TRUE(projected.value().record.has_value());
    const detail::BenchFile& embedded = projected.value().record->bench;
    EXPECT_EQ(embedded.conditions.notes, awkward);
    EXPECT_EQ(embedded.items[0].evidence, awkward);
    EXPECT_EQ(embedded.conditions.host, "jetson");
    EXPECT_EQ(embedded.items.size(), input.value().items.size());
    EXPECT_EQ(embedded.claims.slaves.size(), 3u);
}

TEST(PlemDriveApprove, RefusesAnApprovalTheLoaderCouldNotRead) {
    // Each input is under the loader's size cap; the approval embedding both is not.
    Workshop w;
    std::string model = w.original;
    model.insert(0, "# " + std::string(detail::kMaxDriveModelFileBytes - 64 * 1024, 'x') + "\n");
    w.write_model(model);
    std::string items;
    for (int i = 0; i < 3; ++i) {
        items +=
            "  - { id: pad-" + std::to_string(i) + ", result: NA, evidence: \"" + std::string(60000, 'e') + "\" }\n";
    }
    w.write_bench(test::bench_yaml(w.digests.config, w.digests.profile, detail::platform_version(), kThreeSlaves,
                                   std::string(test::kAllPass) + items));
    ASSERT_LT(slurp(w.model).size(), detail::kMaxDriveModelFileBytes);
    ASSERT_LT(slurp(w.bench).size(), detail::kMaxDriveModelFileBytes);
    const ToolRun run = approve({w.model, w.bench});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_TRUE(
        run.err_has("over the " + std::to_string(detail::kMaxDriveModelFileBytes) + " a drive model file may hold"))
        << run.err;
    EXPECT_EQ(run.err.rfind("drive_model_tool approve: the approved file would be ", 0), 0u) << run.err;
    EXPECT_EQ(slurp(w.model), model);
}

TEST(PlemDriveApprove, ABenchFromAnotherReleaseIsApprovedWithItsOwnRevision) {
    // The revision is where the bench ran: copied, not checked (schema §6).
    Workshop w;
    w.write_bench(test::bench_yaml(w.digests.config, w.digests.profile, "0.0.1", kThreeSlaves));
    const ToolRun run = approve({w.model, w.bench});
    ASSERT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_TRUE(run.out_has("implementation_revision: 0.0.1\n")) << run.out;
    const std::string approved = slurp(w.model);
    EXPECT_NE(approved.find("    implementation_revision: \"0.0.1\"\n"), std::string::npos);
    const ToolRun back = check({w.model});
    EXPECT_EQ(back.rc, kToolOk) << back.out << back.err;
    EXPECT_EQ(back.json().at("approval_effective"), "approved") << back.out;
    // Copied, never blank.
    w.write_bench(test::bench_yaml(w.digests.config, w.digests.profile, "", kThreeSlaves));
    const ToolRun blank = approve({w.model, w.bench});
    EXPECT_EQ(blank.rc, kToolRefused);
    EXPECT_TRUE(blank.err_has("claims.implementation_revision is empty")) << blank.err;
    EXPECT_EQ(slurp(w.model), approved);
}

TEST(PlemDriveApprove, RefusesWhatItCannotVouchFor) {
    // An approval block approve cannot find to rewrite: check says so before a
    // bench releases any brake, approve refuses and leaves the file.
    {
        Workshop w;
        const std::string quoted = w.original.substr(0, w.original.find(test::kUnapprovedBlock)) +
                                   "\"approval\": { status: unapproved, record: null }\n";
        w.write_model(quoted);
        const ToolRun checked = check({w.model});
        ASSERT_EQ(checked.rc, kToolOk) << checked.err;
        EXPECT_TRUE(
            summary({w.model}).out_has("\nnext: fix this first, or the bench cannot record its approval: approve "
                                       "cannot find its approval block to rewrite"))
            << summary({w.model}).out;
        EXPECT_EQ(checked.json()
                      .at("approve_blocker")
                      .get<std::string>()
                      .rfind("approve cannot find its approval block to rewrite", 0),
                  0u)
            << checked.out;
        const ToolRun run = approve({w.model, w.bench});
        EXPECT_EQ(run.rc, kToolRefused);
        EXPECT_TRUE(run.err_has("drive_model_tool approve: " + w.model + ": approve cannot find its approval block"))
            << run.err;
        EXPECT_EQ(slurp(w.model), quoted);
    }
    // A file edited after its bench (the bench claims the old digests) is refused.
    {
        Workshop w;
        std::string edited = w.original;
        edited.replace(edited.find("zero_offset_count: 1234"), 23, "zero_offset_count: 1235");
        w.write_model(edited);
        const ToolRun run = approve({w.model, w.bench});
        EXPECT_EQ(run.rc, kToolRefused);
        EXPECT_TRUE(run.err_has("is not honored for " + w.model + " — approval.record.config_digest is '" +
                                w.digests.config + "' but this declaration digests to"))
            << run.err;
        EXPECT_EQ(slurp(w.model), edited);
    }
    // The bench must claim this file's digests; the loader's own judgment 2 says so.
    {
        Workshop w;
        w.write_bench(test::bench_yaml("sha256:0000", w.digests.profile, detail::platform_version(), kThreeSlaves));
        const ToolRun run = approve({w.model, w.bench});
        EXPECT_EQ(run.rc, kToolRefused);
        EXPECT_TRUE(run.err_has("is not honored for " + w.model +
                                " — approval.record.config_digest is 'sha256:0000' but this declaration digests to '" +
                                w.digests.config + "'"))
            << run.err;
        EXPECT_EQ(slurp(w.model), w.original);
    }
    // Every required item must PASS; that is the loader's own check.
    {
        Workshop w;
        std::string items(test::kAllPass);
        items.replace(items.find("id: release-oe, result: PASS"), std::string("id: release-oe, result: PASS").size(),
                      "id: release-oe, result: FAIL");
        w.write_bench(
            test::bench_yaml(w.digests.config, w.digests.profile, detail::platform_version(), kThreeSlaves, items));
        const ToolRun run = approve({w.model, w.bench});
        EXPECT_EQ(run.rc, kToolRefused);
        EXPECT_TRUE(run.err_has("the bench reports 'release-oe' as FAIL")) << run.err;
        EXPECT_EQ(slurp(w.model), w.original);
    }
    // A bench whose slaves do not cover the file: the record would not read
    // back; the file is left as it was.
    {
        Workshop w;
        w.write_bench(test::bench_yaml(w.digests.config, w.digests.profile, detail::platform_version(), test::kSlave0));
        const ToolRun run = approve({w.model, w.bench});
        EXPECT_EQ(run.rc, kToolRefused);
        EXPECT_TRUE(run.err_has("is not honored for " + w.model + " — approval.record.slaves lists slave 1 0 times"))
            << run.err;
        EXPECT_EQ(slurp(w.model), w.original);
    }
    // A bench listing one slave twice: the record copies it, judgment 5 refuses it.
    {
        Workshop w;
        w.write_bench(test::bench_yaml(w.digests.config, w.digests.profile, detail::platform_version(),
                                       std::string(kThreeSlaves) + std::string(test::kSlave0)));
        const ToolRun run = approve({w.model, w.bench});
        EXPECT_EQ(run.rc, kToolRefused);
        EXPECT_TRUE(run.err_has("approval.record.slaves lists slave 0 2 times")) << run.err;
        EXPECT_EQ(slurp(w.model), w.original);
    }
    // CRLF: the loader reads it, approve keeps the file's own line ending.
    {
        Workshop w;
        std::string crlf;
        for (const char c : w.original) {
            crlf += c == '\n' ? "\r\n" : std::string(1, c);
        }
        w.write_model(crlf);
        const ToolRun run = approve({w.model, w.bench});
        ASSERT_EQ(run.rc, kToolOk) << run.err;
        const std::string result = slurp(w.model);
        EXPECT_NE(result.find("approval:\r\n  status: approved\r\n  record:\r\n"), std::string::npos);
        for (size_t at = result.find('\n'); at != std::string::npos; at = result.find('\n', at + 1)) {
            EXPECT_TRUE(at > 0 && result[at - 1] == '\r') << "bare LF at " << at << " — every line ends CRLF";
        }
        EXPECT_EQ(check({w.model}).rc, kToolOk);
    }
    EXPECT_EQ(approve({"only"}).rc, kToolUsage);
    EXPECT_EQ(approve({"a", "b", "--out", "c"}).rc, kToolUsage) << "the record goes into the model file";
}

TEST(PlemDriveApprove, AFailedWriteLeavesTheFileAsItWas) {
    if (::geteuid() == 0) {
        GTEST_SKIP() << "root writes anywhere";
    }
    const Workshop w;
    // The record is honored in memory; only the write fails (no temporary file in a read-only directory).
    std::filesystem::permissions(w.dir.path, std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec);
    const ToolRun run = approve({w.model, w.bench});
    std::filesystem::permissions(w.dir.path, std::filesystem::perms::owner_all);
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_TRUE(run.err_has("cannot write " + w.model)) << run.err;
    EXPECT_EQ(slurp(w.model), w.original);
    EXPECT_EQ(std::distance(std::filesystem::directory_iterator(w.dir.path), {}), 2) << "nothing is left behind";
}

TEST(PlemDriveApprove, RefusesAFileItCannotWriteAndKeepsItReadOnly) {
    // Replacing a read-only file would quietly lift its mark (and a file of
    // another user's would become the caller's): approve refuses instead.
    if (::geteuid() == 0) {
        GTEST_SKIP() << "root writes anywhere";
    }
    namespace fs = std::filesystem;
    const Workshop w;
    fs::permissions(w.model, fs::perms::owner_read | fs::perms::group_read);
    const ToolRun run = approve({w.model, w.bench});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_TRUE(run.err_has("not writable by you")) << run.err;
    EXPECT_EQ(slurp(w.model), w.original);
    EXPECT_EQ(fs::status(w.model).permissions(), fs::perms::owner_read | fs::perms::group_read);
    EXPECT_EQ(std::distance(fs::directory_iterator(w.dir.path), {}), 2) << "nothing is left behind";
}

// the executable

TEST(PlemDriveExecutable, StdoutIsTheContractAloneTheLoggerGoesToStderr) {
    // The functions above write to a stream the test owns; the process must
    // also keep the core logger (whose sink is stdout) off fd 1, or a runner
    // parsing the JSON meets a log line first. At debug the loader logs its
    // axis mapping, the line this pin looks for on stderr.
    const Workshop w;
    const std::string exe = "PLEM_LOG_LEVEL=debug '" PLEM_DRIVE_EXE "' check --json '" + w.model + "'";
    const std::string out = shell(exe + " 2>/dev/null");
    ASSERT_FALSE(out.empty());
    const nlohmann::json doc = nlohmann::json::parse(out, nullptr, false);
    ASSERT_TRUE(doc.is_object()) << "stdout is one JSON document: " << out;
    EXPECT_EQ(doc.at("format"), "plem-drive-check/1");
    const std::string err = shell(exe + " 2>&1 1>/dev/null");
    EXPECT_NE(err.find("drive model file"), std::string::npos) << "the mapping line belongs on stderr: " << err;
}

TEST(PlemDriveExecutable, RefusesBeforeTheVerbWhenStdoutCannotBeKept) {
    // stdout closed: the contract fd cannot be duplicated, so the tool refuses
    // up front instead of running the verb with nowhere to write the contract.
    // Covers the dup() failure path only; a dup2()-only failure is not
    // reproducible from here.
    const Workshop w;
    const std::string exe = "'" PLEM_DRIVE_EXE "' check --json '" + w.model + "'";
    const std::string err = shell("(" + exe + " 2>&1 >&-; echo rc=$?)");
    EXPECT_EQ(err.rfind("plem drive: cannot keep the logger off stdout", 0), 0u) << err;
    EXPECT_NE(err.find("rc=1\n"), std::string::npos) << err;
}

TEST(PlemDriveApprove, ABenchDigestOnADeclaredSlaveYieldsARecordTheLoaderDoesNotHonor) {
    // approve copies claims.slaves verbatim and reads the result back: a digest
    // on a slave whose mapping the file declares is rule 2's reason there.
    Workshop w;
    const std::string with_digest =
        std::string(kThreeSlaves)
            .replace(std::string(kThreeSlaves).find("pdo_mapping_digest: null"), 24,
                     "pdo_mapping_digest: "
                     "\"sha256:63f608fc09430a5f804e8c5506e06212013d43b91bdd252031105b90c28e5d8f\"");
    w.write_bench(test::bench_yaml(w.digests.config, w.digests.profile, detail::platform_version(), with_digest));
    const ToolRun run = approve({w.model, w.bench});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_TRUE(run.err_has("pdo_mapping_digest is set but slaves[0] declares its pdo_mapping")) << run.err;
    EXPECT_EQ(slurp(w.model), w.original) << "the file is left as it was";
}

}  // namespace

}  // namespace plem::drive
