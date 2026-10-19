// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_ethercat_bus_scanned_fake_master.cpp
/// @brief EtherCATBus over the fake master: scanned slaves (ServoInfo with a
/// null pdo_mapping): the bus reads the slave's mapping from the master at
/// Step 3, checks it with the same validator as a declaration, writes nothing
/// (no ecrt_slave_config_pdos) and registers the roles the scan maps. Same
/// gtest target as test_ethercat_bus_fake_master.cpp, fixture in
/// fake_master_fixture.hpp.
///
/// What these pins prove: registration follows the scanned list and the
/// write is absent. What they cannot prove: that a drive comes up on that
/// mapping; the SM sizes and the AL state live in the kernel master, so
/// whether a drive boots on the extended layout is provable only on hardware.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <vector>

#include <fmt/format.h>

#include "fake_master_fixture.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/pdo_layout.hpp"
#include "plem/fieldbus/ethercat/pdo_scan.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

using plem::fieldbus::ethercat::EtherCATBus;
using plem::fieldbus::ethercat::PdoEntryMapping;
using plem::fieldbus::ethercat::PdoMapping;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::ethercat::SlaveIdentity;
using plem::fieldbus::ethercat::SlavePdoMapping;
using plem::fieldbus::ethercat::SyncManagerAssignment;
using plem::fieldbus::test::count_calls;
using plem::fieldbus::test::FakeEcrt;
using plem::fieldbus::test::kSingleAxisCstLayout;
using plem::fieldbus::test::kSingleAxisCstRxPdos;
using plem::fieldbus::test::log_contains;
using plem::fieldbus::test::six_servos;

namespace {

// Base layout, kSingleAxisCstLayout: SM2 RxPDO 0x1600 / SM3 TxPDO 0x1a00, the
// five standard entries each.
//
// Extended layout: a TxPDO 0x1a00 that adds 0x6078 (16-bit) and 0x60e4 (32-bit) after
// the five standard entries (7 entries, 19 bytes); the SM assignment and the
// watchdog bits are the base layout's. The pin is order-independent: roles resolve by
// index.
constexpr PdoEntryMapping kExtendedTx[] = {
    {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x606c, 0x00, 32}, {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},  {0x6078, 0x00, 16}, {0x60e4, 0x00, 32},
};
constexpr PdoMapping kExtendedTxPdos[] = {{0x1a00, kExtendedTx, 7}};
constexpr SyncManagerAssignment kExtendedSyncs[] = {
    {0, true, nullptr, 0, false},
    {1, false, nullptr, 0, false},
    {2, true, kSingleAxisCstRxPdos, 1, true},
    {3, false, kExtendedTxPdos, 1, false},
};
constexpr SlavePdoMapping kExtendedLayout{kExtendedSyncs, 4};

// A two-channel slave the way a scan reports it: one PDO per channel
// (0x1600/0x1610 out, 0x1a00/0x1a10 in), channel 2 objects at +0x800.
constexpr PdoEntryMapping kCh2Rx[] = {
    {0x6840, 0x00, 16}, {0x687a, 0x00, 32}, {0x68ff, 0x00, 32}, {0x6871, 0x00, 16}, {0x6860, 0x00, 8},
};
constexpr PdoEntryMapping kCh2Tx[] = {
    {0x6841, 0x00, 16}, {0x6864, 0x00, 32}, {0x686c, 0x00, 32}, {0x6877, 0x00, 16}, {0x6861, 0x00, 8},
};
constexpr PdoMapping kTwoChannelRxPdos[] = {
    {0x1600, plem::fieldbus::test::kSingleAxisCstRx, 5},
    {0x1610, kCh2Rx, 5},
};
constexpr PdoMapping kTwoChannelTxPdos[] = {
    {0x1a00, plem::fieldbus::test::kSingleAxisCstTx, 5},
    {0x1a10, kCh2Tx, 5},
};
constexpr SyncManagerAssignment kTwoChannelSyncs[] = {
    {2, true, kTwoChannelRxPdos, 2, true},
    {3, false, kTwoChannelTxPdos, 2, false},
};
constexpr SlavePdoMapping kTwoChannelLayout{kTwoChannelSyncs, 2};

// A scan without the torque feedback the CST contract requires.
constexpr PdoEntryMapping kNoTorqueTx[] = {
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6061, 0x00, 8},
};
constexpr PdoMapping kNoTorqueTxPdos[] = {{0x1a00, kNoTorqueTx, 4}};
constexpr SyncManagerAssignment kNoTorqueSyncs[] = {
    {2, true, kSingleAxisCstRxPdos, 1, true},
    {3, false, kNoTorqueTxPdos, 1, false},
};
constexpr SlavePdoMapping kNoTorqueLayout{kNoTorqueSyncs, 2};

/// Six one-axis slaves, none declaring a layout; the bus scans each.
std::vector<ServoInfo> six_scanned_servos() {
    auto infos = six_servos();
    for (auto& info : infos) {
        info.pdo_mapping = nullptr;
    }
    return infos;
}

void scan_every_slave(const SlavePdoMapping& layout, unsigned int revision, size_t slaves = 6) {
    for (unsigned int pos = 0; pos < slaves; ++pos) {
        FakeEcrt::set_slave_scan(pos, revision, layout);
    }
}

/// The (position, index) registrations of one slave, in call order.
std::vector<std::string> registered_on(unsigned int position) {
    std::vector<std::string> out;
    const std::string prefix = fmt::format("reg_entry(pos={},", position);
    for (const auto& call : FakeEcrt::calls()) {
        if (call.rfind(prefix, 0) == 0) {
            out.push_back(call);
        }
    }
    return out;
}

const std::vector<std::string>& standard_roles_on_slave_zero() {
    static const std::vector<std::string> expected = [] {
        std::vector<std::string> out;
        for (const uint16_t index : {0x6040, 0x6060, 0x607a, 0x60ff, 0x6071, 0x6041, 0x6061, 0x6064, 0x606c, 0x6077}) {
            out.push_back(fmt::format("reg_entry(pos=0,idx=0x{:x})", index));
        }
        return out;
    }();
    return expected;
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, ScannedFiveEntryMappingRegistersTheStandardRoles) {
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    EtherCATBus bus(0);
    bus.set_servo_config(six_scanned_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    // No write: a scanned slave keeps the mapping the master found.
    EXPECT_EQ(count_calls("config_pdos"), 0);
    // Registration follows the scanned list: the ten standard roles, in the
    // core's fixed order, on every slave.
    EXPECT_EQ(registered_on(0), standard_roles_on_slave_zero());
    EXPECT_EQ(count_calls("reg_entry("), 60);
    EXPECT_TRUE(log_contains("reg_pdo_entry_list(entries=60)"));
    EXPECT_TRUE(log_contains("master_activate"));
    // One scan per slave (4 SMs each), made after the master was requested.
    EXPECT_EQ(count_calls("get_sync_manager("), 24);
    EXPECT_LT(plem::fieldbus::test::index_of("request_master"),
              plem::fieldbus::test::index_of("get_sync_manager(pos=0,"));
}

TEST_F(EtherCATBusFakeMasterTest, AScannedSlavesDigestIsReadableByRingPosition) {
    // What the drive bus publishes on ~/info: the digest Step 3 cached, not
    // a recomputation; the value an approval record's pdo_mapping_digest names.
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    EtherCATBus bus(0);
    bus.set_servo_config(six_scanned_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    const std::string expected = plem::fieldbus::ethercat::pdo_mapping_digest(kSingleAxisCstLayout);
    EXPECT_EQ(bus.scanned_pdo_mapping_digest(0), expected);
    EXPECT_EQ(bus.scanned_pdo_mapping_digest(5), expected);
    EXPECT_EQ(bus.scanned_pdo_mapping_digest(6), "");  // no slave there
}

TEST_F(EtherCATBusFakeMasterTest, ADeclaredSlaveHasNoScannedDigest) {
    EtherCATBus bus(0);
    EXPECT_EQ(bus.scanned_pdo_mapping_digest(0), "");  // before initialize
    bus.set_servo_config(six_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(bus.scanned_pdo_mapping_digest(0), "");
}

TEST_F(EtherCATBusFakeMasterTest, ScannedSevenEntrySupersetRegistersTheSameRoles) {
    scan_every_slave(kExtendedLayout, 0x2);
    EtherCATBus bus(0);
    bus.set_servo_config(six_scanned_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("config_pdos"), 0);
    EXPECT_EQ(registered_on(0), standard_roles_on_slave_zero());
    EXPECT_EQ(count_calls("reg_entry("), 60);
    // The two vendor entries are not roles: never registered, never fatal.
    EXPECT_EQ(count_calls("idx=0x6078"), 0);
    EXPECT_EQ(count_calls("idx=0x60e4"), 0);
    EXPECT_TRUE(log_contains("master_activate"));
}

TEST_F(EtherCATBusFakeMasterTest, ABaseDeclarationOnAnExtendedSlaveStillWritesTheDeclaration) {
    // Contrast pin for the declared path: with the extended mapping on the
    // slave, a base-layout declaration is still written (six ecrt_slave_config_pdos)
    // and registration resolves against the written mapping; the fake, like
    // the master, lets the configured assignment supersede the scanned one.
    // Whether the drive accepts that write is not observable here (a drive that
    // does not refuses its image with AL 0x001E after activation).
    scan_every_slave(kExtendedLayout, 0x2);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());  // declared: kSingleAxisCstLayout
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("config_pdos"), 6);
    EXPECT_EQ(count_calls("get_sync_manager("), 0);  // declared slaves are not scanned
    EXPECT_EQ(registered_on(0), standard_roles_on_slave_zero());
}

TEST_F(EtherCATBusFakeMasterTest, DeclaredPathMakesNoScanQuery) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("get_sync_manager("), 0);
    EXPECT_EQ(count_calls("get_pdo("), 0);
    EXPECT_EQ(count_calls("get_pdo_entry("), 0);
    EXPECT_EQ(count_calls("config_pdos"), 6);
}

TEST_F(EtherCATBusFakeMasterTest, ScannedTwoChannelSlaveRegistersEachAxisAtItsOwnChannel) {
    // Six axes on three scanned two-channel slaves (0,0,1,1,2,2 / channel
    // offsets 0, 0x800): the first axis scans, the second shares the result,
    // and od_offset resolution runs on the scanned graph exactly as on a
    // declared one.
    scan_every_slave(kTwoChannelLayout, 0x1, 3);
    auto infos = six_scanned_servos();
    for (size_t i = 0; i < 6; ++i) {
        infos[i].slave_position = static_cast<int32_t>(i / 2);
        infos[i].od_offset = (i % 2) ? 0x800 : 0;
    }
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("config_pdos"), 0);
    EXPECT_EQ(count_calls("get_sync_manager(pos=0,"), 2);  // scanned once per slave, not per axis
    EXPECT_TRUE(log_contains("reg_entry(pos=0,idx=0x6040)"));
    EXPECT_TRUE(log_contains("reg_entry(pos=0,idx=0x6840)"));
    EXPECT_TRUE(log_contains("reg_entry(pos=2,idx=0x6877)"));
    EXPECT_TRUE(log_contains("reg_entry(pos=2,idx=0x68ff)"));
    EXPECT_EQ(count_calls("reg_entry("), 60);
}

TEST_F(EtherCATBusFakeMasterTest, ScannedMappingLackingARequiredRoleIsRefusedBeforeAnyWriteOrActivation) {
    // Slave 2 maps no torque feedback: the same validator that checks a
    // declaration refuses the scan after the master is requested (the scan
    // needs it) and before any device registration, SDO or activation; the master is
    // released like every post-acquisition refusal.
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    FakeEcrt::set_slave_scan(2, 0x1, kNoTorqueLayout);
    EtherCATBus bus(0);
    bus.set_servo_config(six_scanned_servos());
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_EQ(count_calls("config_pdos"), 0);
    EXPECT_EQ(count_calls("reg_entry("), 0);
    EXPECT_FALSE(log_contains("master_activate"));
    EXPECT_EQ(FakeEcrt::calls().back(), "release_master");
}

TEST_F(EtherCATBusFakeMasterTest, AnUnscannedSlaveRefusesToInitialize) {
    // Slave 4 has no knob: the master reports sync_count 0; the bus does not
    // guess a mapping, and does not go on to the next slave.
    scan_every_slave(kSingleAxisCstLayout, 0x1, 4);
    EtherCATBus bus(0);
    bus.set_servo_config(six_scanned_servos());
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_EQ(count_calls("get_sync_manager(pos=5,"), 0);
    EXPECT_FALSE(log_contains("master_activate"));
    EXPECT_EQ(FakeEcrt::calls().back(), "release_master");
}

// A scan that cannot be read and a scan the validator refuses are both
// registration refusals, with different causes in their messages.
TEST_F(EtherCATBusFakeMasterTest, AnUnreadableScanAndARefusedScanSayDifferentThings) {
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    FakeEcrt::set_slave_scan_query_failure(1);
    EtherCATBus unreadable(0);
    unreadable.set_servo_config(six_scanned_servos());
    const auto read = unreadable.initialize();
    ASSERT_FALSE(read);
    EXPECT_NE(read.message().find("cannot read the slave's PDO mapping from the master"), std::string::npos)
        << read.message();

    FakeEcrt::reset();
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    FakeEcrt::set_slave_scan(2, 0x1, kNoTorqueLayout);
    EtherCATBus refused(0);
    refused.set_servo_config(six_scanned_servos());
    const auto judged = refused.initialize();
    ASSERT_FALSE(judged);
    EXPECT_EQ(judged.message().find("cannot read the slave's PDO mapping"), std::string::npos) << judged.message();
    EXPECT_NE(judged.message().find("slave 2"), std::string::npos) << judged.message();
}

TEST_F(EtherCATBusFakeMasterTest, AScanQueryTheMasterCannotAnswerRefusesToInitialize) {
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    FakeEcrt::set_slave_scan_query_failure(1);
    EtherCATBus bus(0);
    bus.set_servo_config(six_scanned_servos());
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_FALSE(log_contains("master_activate"));
    EXPECT_EQ(FakeEcrt::calls().back(), "release_master");
}

TEST_F(EtherCATBusFakeMasterTest, MixedDeclaredAndScannedAxesOnOneSlaveAreRefusedAtStepZero) {
    // One slave, one axis declaring and the other not: the existing per-slave
    // "same layout object" rule already refuses this before any ecrt call
    // (null and non-null are different objects).
    scan_every_slave(kTwoChannelLayout, 0x1, 3);
    auto infos = six_scanned_servos();
    for (size_t i = 0; i < 6; ++i) {
        infos[i].slave_position = static_cast<int32_t>(i / 2);
        infos[i].od_offset = (i % 2) ? 0x800 : 0;
    }
    infos[1].pdo_mapping = &kTwoChannelLayout;
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, ANullLayoutIsAValidServoInfo) {
    auto info = plem::fieldbus::test::make_servo_info();
    info.pdo_mapping = nullptr;
    EXPECT_TRUE(info.is_valid()) << "null = scanned, not undeclared";
    EXPECT_NE(info.dump().find("pdo_mapping=scanned"), std::string::npos) << info.dump();
    info.pdo_mapping = &kSingleAxisCstLayout;
    EXPECT_NE(info.dump().find("pdo_mapping=declared"), std::string::npos) << info.dump();
}

// The approval record's digest of the scanned mapping (schema §6)

namespace {

/// The rev 0x1 single-axis CST mapping's digest: the INFO-line golden of
/// test_pdo_scan_fake_master.cpp, as a bench copies it into its claims.
constexpr std::string_view kBaseDigest = "sha256:63f608fc09430a5f804e8c5506e06212013d43b91bdd252031105b90c28e5d8f";

SlaveIdentity digest_only(std::string_view digest) {
    SlaveIdentity id;
    std::array<char, SlaveIdentity::kDigestLength> text{};
    std::copy(digest.begin(), digest.end(), text.begin());
    id.pdo_mapping_digest = text;
    return id;
}

plem::core::Result<void> initialize_scanned_with(const SlaveIdentity& identity, uint32_t position = 2) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_scanned_servos());
    bus.set_expected_slave_identity({{.slave_position = position, .identity = identity}});
    return bus.initialize();
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, TheRecordsDigestIsComparedWithTheScannedMappingWithoutAMasterRead) {
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    const auto init = initialize_scanned_with(digest_only(kBaseDigest));
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("master_sdo_upload"), 0) << "the digest is Step 3's scan — nothing is read for it";
    EXPECT_TRUE(log_contains("master_activate"));
}

TEST_F(EtherCATBusFakeMasterTest, AScannedMappingUnlikeTheRecordsDigestRefusesToInitialize) {
    scan_every_slave(kSingleAxisCstLayout, 0x1);
    FakeEcrt::set_slave_scan(2, 0x2, kExtendedLayout);  // the record was made on the base mapping
    EXPECT_NE(
        initialize_scanned_with(digest_only(kBaseDigest)).message().find("slave 2: the scanned PDO mapping digests to"),
        std::string::npos);
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_FALSE(log_contains("master_activate"));
    EXPECT_FALSE(log_contains("config_pdos")) << "refused before any write";
}

TEST_F(EtherCATBusFakeMasterTest, ADigestForADeclaredSlaveIsAMismatchNotASkip) {
    // The loader refuses such a record; a spec assembled another way still
    // must not pass a value the bus has nothing to compare with.
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());  // every slave declares its mapping, no scan, no digest
    bus.set_expected_slave_identity({{.slave_position = 2, .identity = digest_only(kBaseDigest)}});
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_FALSE(log_contains("master_activate"));
}
