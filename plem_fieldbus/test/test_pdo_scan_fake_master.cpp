// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_pdo_scan_fake_master.cpp
/// @brief scan_pdo_mapping / format_pdo_mapping over the fake master, and the
/// fake's own scan contract (set_slave_scan: effective mapping, -ENOENT
/// outside it, config_pdos superseding it). Same gtest target as
/// test_ethercat_bus_fake_master.cpp, fixture in fake_master_fixture.hpp.

#include <gtest/gtest.h>

#include <cerrno>
#include <string>
#include <utility>
#include <vector>

#include <ecrt.h>

#include "fake_master_fixture.hpp"
#include "plem/fieldbus/ethercat/pdo_layout_validation.hpp"
#include "plem/fieldbus/ethercat/pdo_scan.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

#include "ethercat/pdo_scan_messages.hpp"

using plem::fieldbus::ethercat::AxisPdoRequirements;
using plem::fieldbus::ethercat::describe_digest_mismatch;
using plem::fieldbus::ethercat::describe_digest_without_scan;
using plem::fieldbus::ethercat::describe_scan_rejection;
using plem::fieldbus::ethercat::describe_scanned_mapping;
using plem::fieldbus::ethercat::format_pdo_mapping;
using plem::fieldbus::ethercat::pdo_mapping_digest;
using plem::fieldbus::ethercat::PdoEntryMapping;
using plem::fieldbus::ethercat::PdoMapping;
using plem::fieldbus::ethercat::scan_pdo_mapping;
using plem::fieldbus::ethercat::ScannedPdoMapping;
using plem::fieldbus::ethercat::SlavePdoMapping;
using plem::fieldbus::ethercat::SyncManagerAssignment;
using plem::fieldbus::ethercat::validate_pdo_mapping;
using plem::fieldbus::test::count_calls;
using plem::fieldbus::test::FakeEcrt;
using plem::fieldbus::test::kSingleAxisCstLayout;
using plem::fieldbus::test::kSyntheticProductCode;
using plem::fieldbus::test::kSyntheticVendorId;
using plem::fieldbus::test::log_contains;

namespace {

constexpr const char* kSingleAxisCstText =
    "sync_managers:\n"
    "  - { sm: 0, direction: output, watchdog: false, pdos: [] }\n"
    "  - { sm: 1, direction: input, watchdog: false, pdos: [] }\n"
    "  - sm: 2\n"
    "    direction: output\n"
    "    watchdog: true\n"
    "    pdos:\n"
    "      - index: 0x1600\n"
    "        entries:\n"
    "          - { index: 0x6040, subindex: 0, bits: 16 }\n"
    "          - { index: 0x607a, subindex: 0, bits: 32 }\n"
    "          - { index: 0x60ff, subindex: 0, bits: 32 }\n"
    "          - { index: 0x6071, subindex: 0, bits: 16 }\n"
    "          - { index: 0x6060, subindex: 0, bits: 8 }\n"
    "  - sm: 3\n"
    "    direction: input\n"
    "    watchdog: false\n"
    "    pdos:\n"
    "      - index: 0x1a00\n"
    "        entries:\n"
    "          - { index: 0x6041, subindex: 0, bits: 16 }\n"
    "          - { index: 0x6064, subindex: 0, bits: 32 }\n"
    "          - { index: 0x606c, subindex: 0, bits: 32 }\n"
    "          - { index: 0x6077, subindex: 0, bits: 16 }\n"
    "          - { index: 0x6061, subindex: 0, bits: 8 }\n";

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, ScanAssemblesTheMasterViewIntoTheDeclaredShape) {
    FakeEcrt::set_slave_scan(3, 0x2, kSingleAxisCstLayout);
    auto scanned = scan_pdo_mapping(FakeEcrt::master(), 3);
    ASSERT_TRUE(scanned.is_ok()) << scanned.message();
    const ScannedPdoMapping& layout = scanned.value();
    EXPECT_EQ(layout.revision_number, 0x2u);
    ASSERT_EQ(layout.view.sync_count, 4);
    EXPECT_EQ(layout.view.syncs, layout.syncs.data());
    EXPECT_EQ(layout.syncs.size(), 4u);
    EXPECT_EQ(layout.pdos.size(), 2u);
    EXPECT_EQ(layout.entries.size(), 10u);
    // SM shape and direction/watchdog as the fake reports them.
    EXPECT_EQ(layout.view.syncs[2].sm_index, 2);
    EXPECT_TRUE(layout.view.syncs[2].output);
    EXPECT_TRUE(layout.view.syncs[2].watchdog);
    EXPECT_FALSE(layout.view.syncs[3].output);
    EXPECT_FALSE(layout.view.syncs[3].watchdog);
    EXPECT_EQ(layout.view.syncs[0].pdo_count, 0);
    EXPECT_EQ(layout.view.syncs[0].pdos, nullptr);
    ASSERT_EQ(layout.view.syncs[2].pdo_count, 1);
    EXPECT_EQ(layout.view.syncs[2].pdos[0].index, 0x1600);
    ASSERT_EQ(layout.view.syncs[3].pdo_count, 1);
    EXPECT_EQ(layout.view.syncs[3].pdos[0].index, 0x1a00);
    ASSERT_EQ(layout.view.syncs[3].pdos[0].entry_count, 5);
    EXPECT_EQ(layout.view.syncs[3].pdos[0].entries[4], (PdoEntryMapping{0x6061, 0, 8}));
    // The one validator checks the scanned shape exactly as the declared one.
    const std::vector<AxisPdoRequirements> axes = {{.od_offset = 0}};
    const auto valid = validate_pdo_mapping(layout.view, axes);
    EXPECT_TRUE(valid.is_ok()) << valid.message();
    const auto valid2 = validate_pdo_mapping(kSingleAxisCstLayout, axes);
    EXPECT_TRUE(valid2.is_ok()) << valid2.message();
    // Query trace: one get_sync_manager per SM, one get_pdo per PDO, one entry query per entry.
    EXPECT_EQ(count_calls("get_sync_manager(pos=3,"), 4);
    EXPECT_EQ(count_calls("get_pdo(pos=3,"), 2);
    EXPECT_EQ(count_calls("get_pdo_entry(pos=3,"), 10);
}

TEST_F(EtherCATBusFakeMasterTest, ScannedLayoutSurvivesAMove) {
    FakeEcrt::set_slave_scan(0, 0x1, kSingleAxisCstLayout);
    auto scanned = scan_pdo_mapping(FakeEcrt::master(), 0);
    ASSERT_TRUE(scanned.is_ok()) << scanned.message();
    ScannedPdoMapping moved = std::move(scanned).value();
    EXPECT_EQ(moved.view.syncs, moved.syncs.data());
    EXPECT_EQ(moved.view.syncs[2].pdos, moved.pdos.data());
    EXPECT_EQ(moved.view.syncs[2].pdos[0].entries, moved.entries.data());
    const auto valid = validate_pdo_mapping(moved.view, std::vector<AxisPdoRequirements>{{.od_offset = 0}});
    EXPECT_TRUE(valid.is_ok()) << valid.message();
}

TEST_F(EtherCATBusFakeMasterTest, ScanOfASlaveWithoutSyncManagersNamesTheIncompleteScan) {
    // No knob: the fake's get_slave reports sync_count 0, which means the master
    // has not scanned the slave (or nothing sits at the position); no query is made.
    FakeEcrt::set_slave_identity(4, 0, 0);  // and it reports no identity
    auto scanned = scan_pdo_mapping(FakeEcrt::master(), 4);
    ASSERT_FALSE(scanned.is_ok());
    EXPECT_NE(scanned.message().find("slave 4: the master reports no sync manager for it"), std::string::npos)
        << scanned.message();
    EXPECT_NE(scanned.message().find("ethercat rescan"), std::string::npos) << scanned.message();
    // Three causes it cannot tell apart, and the identity the master reports
    // (none here: 0x0/0x0) so the operator can.
    EXPECT_NE(scanned.message().find("the wiring changed"), std::string::npos) << scanned.message();
    EXPECT_NE(scanned.message().find("vendor 0x0 product 0x0"), std::string::npos) << scanned.message();
    EXPECT_EQ(count_calls("get_sync_manager"), 0);
}

TEST_F(EtherCATBusFakeMasterTest, ScanQueryFailureIsNamedBySlaveAndPosition) {
    FakeEcrt::set_slave_scan(1, 0x1, kSingleAxisCstLayout);
    FakeEcrt::set_slave_scan_query_failure(1);
    auto scanned = scan_pdo_mapping(FakeEcrt::master(), 1);
    ASSERT_FALSE(scanned.is_ok());
    EXPECT_EQ(scanned.message(), "slave 1: ecrt_master_get_sync_manager(sm 0) failed (-5)");
}

TEST_F(EtherCATBusFakeMasterTest, FormatPdoLayoutIsTheDriveModelFileMapping) {
    // Characterization golden: the bytes a bench reads back. Input:
    // kSingleAxisCstLayout (the standard single-axis CST entry set).
    EXPECT_EQ(format_pdo_mapping(kSingleAxisCstLayout), kSingleAxisCstText);
    // A scanned layout formats to the same text as the declaration it was scanned from.
    FakeEcrt::set_slave_scan(0, 0x1, kSingleAxisCstLayout);
    auto scanned = scan_pdo_mapping(FakeEcrt::master(), 0);
    ASSERT_TRUE(scanned.is_ok()) << scanned.message();
    EXPECT_EQ(format_pdo_mapping(scanned.value().view), kSingleAxisCstText);
}

TEST_F(EtherCATBusFakeMasterTest, ScannedLayoutSummaryLineIsPinnedWithItsDigest) {
    // Characterization golden of the INFO summary (same input, rev 0x1): the
    // digest is sha256_hex of the format_pdo_mapping text above, `sha256:`
    // prefixed; it is what pairs a bring-up log line with a mapping text.
    FakeEcrt::set_slave_scan(0, 0x1, kSingleAxisCstLayout);
    auto scanned = scan_pdo_mapping(FakeEcrt::master(), 0);
    ASSERT_TRUE(scanned.is_ok()) << scanned.message();
    EXPECT_EQ(describe_scanned_mapping(scanned.value()),
              "4 sync managers, 2 PDOs, 10 entries, revision 0x1, "
              "sha256:63f608fc09430a5f804e8c5506e06212013d43b91bdd252031105b90c28e5d8f");
    // The record's `pdo_mapping_digest` is this same value, from the same helper.
    EXPECT_EQ(pdo_mapping_digest(kSingleAxisCstLayout),
              "sha256:63f608fc09430a5f804e8c5506e06212013d43b91bdd252031105b90c28e5d8f");
}

TEST_F(EtherCATBusFakeMasterTest, ScanRejectionSentenceIsPinned) {
    // Characterization golden of the refusal the bus logs for a scanned
    // mapping the validator turns down: slave, axes, the identity the master
    // scanned, the validator's finding, the observed mapping (the format text
    // pinned above) and the remedies, both conditioned on a drive that
    // accepts PDO reconfiguration, with the fixed-mapping case named as not
    // drivable in this mode. Input: kSingleAxisCstLayout under the fixture
    // servo's identity, reported as rev 0x2.
    ScannedPdoMapping layout;
    layout.view = kSingleAxisCstLayout;
    layout.vendor_id = kSyntheticVendorId;
    layout.product_code = kSyntheticProductCode;
    layout.revision_number = 0x2;
    EXPECT_EQ(describe_scan_rejection(3, "3,4", layout, "no 0x6077 (torque actual) for od_offset 0x0000"),
              std::string("slave 3 (axes 3,4, vendor 0xc0ffee product 0x1001 revision 0x2 — as the master "
                          "reports it): the PDO mapping the master scanned does not serve the axes — no 0x6077 "
                          "(torque actual) for od_offset 0x0000. Observed mapping:\n") +
                  kSingleAxisCstText +
                  "If the drive accepts PDO reconfiguration, either configure it to map the object, or declare "
                  "the mapping (`pdo_mapping`, or ServoInfo::pdo_mapping from C++) so the core writes it in PREOP. "
                  "A drive with "
                  "a fixed mapping that does not carry this role cannot be driven in this mode.");
}

// The fake's scan contract (what the bus pins above rely on)

TEST_F(EtherCATBusFakeMasterTest, FakeRegistrationRefusesAnEntryOutsideTheScannedMapping) {
    FakeEcrt::set_slave_scan(0, 0x1, kSingleAxisCstLayout);
    uint32_t offset = 0;
    ec_pdo_entry_reg_t regs[] = {
        {0, 0, 0, 0, 0x6040, 0, &offset, nullptr},
        {0, 0, 0, 0, 0x603f, 0, &offset, nullptr},  // not in the single-axis CST layout
        {},
    };
    EXPECT_EQ(ecrt_domain_reg_pdo_entry_list(nullptr, regs), -ENOENT);
    EXPECT_TRUE(log_contains("reg_pdo_entry_list(ENOENT pos=0,idx=0x603f,sub=0x0)"));
    // A slave without the knob accepts every registration.
    ec_pdo_entry_reg_t other[] = {{0, 5, 0, 0, 0x603f, 0, &offset, nullptr}, {}};
    EXPECT_EQ(ecrt_domain_reg_pdo_entry_list(nullptr, other), 0);
}

TEST_F(EtherCATBusFakeMasterTest, FakeConfigPdosSupersedesTheScannedMapping) {
    FakeEcrt::set_slave_scan(0, 0x1, kSingleAxisCstLayout);
    // Configure a mapping that adds 0x603f and drops 0x60ff, as apply_pdo_mapping would.
    const ec_pdo_entry_info_t tx[] = {{0x6041, 0, 16}, {0x603f, 0, 16}};
    const ec_pdo_info_t pdos[] = {{0x1a00, 2, tx}};
    const ec_sync_info_t syncs[] = {{3, EC_DIR_INPUT, 1, pdos, EC_WD_DISABLE},
                                    {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}};
    ec_slave_config_t* sc =
        ecrt_master_slave_config(FakeEcrt::master(), 0, 0, kSyntheticVendorId, kSyntheticProductCode);
    ASSERT_EQ(ecrt_slave_config_pdos(sc, EC_END, syncs), 0);
    uint32_t offset = 0;
    ec_pdo_entry_reg_t now_mapped[] = {{0, 0, 0, 0, 0x603f, 0, &offset, nullptr}, {}};
    EXPECT_EQ(ecrt_domain_reg_pdo_entry_list(nullptr, now_mapped), 0);
    ec_pdo_entry_reg_t no_longer_mapped[] = {{0, 0, 0, 0, 0x60ff, 0, &offset, nullptr}, {}};
    EXPECT_EQ(ecrt_domain_reg_pdo_entry_list(nullptr, no_longer_mapped), -ENOENT);
}

TEST_F(EtherCATBusFakeMasterTest, DigestRefusalSentencesArePinned) {
    // Characterization goldens of the two Step 3.5 digest refusals the bus
    // logs verbatim after "[EtherCATBus] ": the mismatch names both digests,
    // six causes it cannot tell apart and the remedy; the declared-slave case
    // names why there is nothing to compare.
    EXPECT_EQ(
        describe_digest_mismatch(2, "sha256:aaaa", "sha256:bbbb"),
        "slave 2: the scanned PDO mapping digests to sha256:aaaa but the approval record declares sha256:bbbb — the "
        "mapping the master holds for this slave is not the one the bench observed. Causes this check cannot tell "
        "apart: the drive or its firmware was swapped; this unit has a different device profile or variant; the "
        "record's digest was mistyped; another application or tool wrote a PDO assignment to the drive; a CoE read "
        "failed during the master's scan and SII values remained; the master's scan is stale (`ethercat rescan`, then retry). Check the "
        "drive's nameplate against the record, then re-bench this drive (`plem drive run <file>` on a terminal offers "
        "the bench).");
    EXPECT_EQ(describe_digest_without_scan(2, "sha256:bbbb"),
              "slave 2: the approval record declares a PDO mapping digest (sha256:bbbb) but this slave's mapping is "
              "DECLARED, not scanned — there is no scanned mapping to compare it with. Make the slave scanned "
              "(`pdo_mapping: scanned`) and re-bench, or declare the digest null.");
}
