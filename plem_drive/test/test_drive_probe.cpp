// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_probe.cpp
/// @brief `drive_model_tool probe <file> --master N` on the shared plem_drive
/// with the fake master linked into this executable: the bus's own start-up
/// check of the drives an approved file names, graded for `plem drive run`
/// as 0 match, 3 identity values only (a bench at these drives replaces the
/// record), 1 refused (another model, a missing slave, drives in another
/// order, a master that does not open), 2 usage.

#include <gtest/gtest.h>

#include <dlfcn.h>
#include <ecrt.h>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <plem/fieldbus/test/fake_ecrt.hpp>

#include "drive_bench_fixture.hpp"
#include "loader/inspection.hpp"
#include "tool/drive_model_tool.hpp"

namespace plem::drive {
namespace {

using fieldbus::test::FakeEcrt;
using tool::kToolOk;
using tool::kToolRebench;
using tool::kToolRefused;
using tool::kToolUsage;

struct ToolRun {
    int rc;
    std::string out;
    std::string err;
};

ToolRun probe(const std::vector<std::string>& args) {
    std::ostringstream out;
    std::ostringstream err;
    const int rc = tool::tool_probe(args, out, err);
    return {rc, out.str(), err.str()};
}

std::string slurp(const char* path) {
    std::ifstream in(path);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// The synthetic file (slaves at ring 5, 0, 2), approved by a bench on drives
/// with SII serials 11, 12, 13.
constexpr std::string_view kBenchSlaves =
    "      - { slave: 0, serial_number: 11, revision_number: null, hardware_version: \"1.3\", software_version: "
    "\"2.07\", pdo_mapping_digest: null }\n"
    "      - { slave: 1, serial_number: 12, revision_number: null, hardware_version: \"1.3\", software_version: "
    "\"2.07\", pdo_mapping_digest: null }\n"
    "      - { slave: 2, serial_number: 13, revision_number: null, hardware_version: \"1.3\", software_version: "
    "\"2.07\", pdo_mapping_digest: null }\n";

class DriveProbe : public ::testing::Test {
  protected:
    void SetUp() override {
        ASSERT_EQ(dlsym(RTLD_DEFAULT, "ecrt_request_master"), reinterpret_cast<void*>(&ecrt_request_master))
            << "libethercat's ecrt_request_master would answer, not the fake";
        FakeEcrt::reset();
        model = dir.path + "/drive.yaml";
        const std::string bench = dir.path + "/bench.yaml";
        std::ofstream(model) << slurp(DRIVE_EXAMPLE_YAML);
        const auto inspected =
            detail::inspect_drive_model(detail::read_text_file(model, "drive model file").value(), model);
        ASSERT_TRUE(inspected) << inspected.message();
        const auto& digests = inspected.value().digests;
        std::ofstream(bench) << test::bench_yaml(digests.config, digests.profile, detail::platform_version(),
                                                 kBenchSlaves);
        std::ostringstream out;
        std::ostringstream err;
        ASSERT_EQ(tool::tool_approve({model, bench}, out, err), kToolOk) << err.str();
        // The drives the record was made on.
        FakeEcrt::reset();
        FakeEcrt::set_sdo_upload_string(0x1009, "1.3");
        FakeEcrt::set_sdo_upload_string(0x100A, "2.07");
        FakeEcrt::set_slave_serial(5, 11);
        FakeEcrt::set_slave_serial(0, 12);
        FakeEcrt::set_slave_serial(2, 13);
    }

    bool called(const std::string& prefix) const {
        const auto& calls = FakeEcrt::calls();
        return std::any_of(calls.begin(), calls.end(), [&](const std::string& c) { return c.rfind(prefix, 0) == 0; });
    }

    test::ScratchDir dir{"plem_drive_probe_"};
    std::string model;
};

TEST_F(DriveProbe, TheRecordsDrivesMatchAndNothingStarts) {
    const ToolRun run = probe({model, "--master", "0"});
    EXPECT_EQ(run.rc, kToolOk) << run.err;
    EXPECT_TRUE(called("request_master"));
    EXPECT_TRUE(called("release_master"));
    EXPECT_FALSE(called("master_activate")) << "a probe configures and starts nothing";
}

TEST_F(DriveProbe, AnotherUnitOfTheSameModelIsRebenchable) {
    FakeEcrt::set_slave_serial(0, 99);
    const ToolRun run = probe({model, "--master", "0"});
    EXPECT_EQ(run.rc, kToolRebench);
    EXPECT_NE(run.err.find("slave 0: SII serial number is 99 but the approval record declares 12"), std::string::npos)
        << run.err;
    EXPECT_FALSE(called("master_activate"));
}

TEST_F(DriveProbe, DrivesInAnotherOrderAreRefused) {
    FakeEcrt::set_slave_serial(5, 12);
    FakeEcrt::set_slave_serial(0, 11);
    const ToolRun run = probe({model, "--master", "0"});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_NE(run.err.find("another order than at the bench"), std::string::npos) << run.err;
}

TEST_F(DriveProbe, AnotherDriveModelIsRefused) {
    FakeEcrt::set_slave_identity(2, 0x1234, 0x5678);
    const ToolRun run = probe({model, "--master", "0"});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_NE(run.err.find("vendor 0x1234 product 0x5678"), std::string::npos) << run.err;
}

TEST_F(DriveProbe, AMasterThatDoesNotOpenIsRefused) {
    FakeEcrt::set_request_master_failure();
    const ToolRun run = probe({model, "--master", "0"});
    EXPECT_EQ(run.rc, kToolRefused);
    EXPECT_NE(run.err.find("Failed to request EtherCAT master 0"), std::string::npos) << run.err;
}

TEST_F(DriveProbe, AMalformedCommandIsUsage) {
    EXPECT_EQ(probe({}).rc, kToolUsage);
    EXPECT_EQ(probe({model}).rc, kToolUsage) << "the master is named";
    EXPECT_EQ(probe({model, "--master", "x"}).rc, kToolUsage);
    EXPECT_EQ(probe({model, "--master"}).rc, kToolUsage);
    EXPECT_EQ(probe({model, "--master", "99999999999999999999"}).rc, kToolUsage) << "out of range";
    EXPECT_EQ(probe({model, "--master", "1x"}).rc, kToolUsage);
    EXPECT_EQ(probe({model, "--master", "-1"}).rc, kToolUsage);
    EXPECT_FALSE(called("request_master")) << "usage opens nothing";
}

}  // namespace
}  // namespace plem::drive
