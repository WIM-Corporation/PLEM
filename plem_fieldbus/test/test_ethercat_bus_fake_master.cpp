// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_ethercat_bus_fake_master.cpp
/// @brief EtherCATBus over the fake master: cycle time, distributed clocks,
/// master lifetime, servo-spec/profile rejection and the target-seeding frame.
/// Configuration (declared PDO mappings, multi-axis slaves, IO factory,
/// retained layout storage, the connection-fixture consumer half)
/// lives in test_ethercat_bus_configuration_fake_master.cpp; same gtest
/// target, fixture in fake_master_fixture.hpp.

#include <gtest/gtest.h>

#include <cerrno>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "device_servo_test_access.hpp"
#include "fake_master_fixture.hpp"
#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/servo/device.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"

using plem::core::units::Pulse;
using plem::fieldbus::OperationMode;
using plem::fieldbus::ethercat::DeviceServo;
using plem::fieldbus::ethercat::endless_count_to_angle;
using plem::fieldbus::ethercat::EtherCATBus;
using plem::fieldbus::ethercat::kEndlessCountDomain;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::ethercat::SlaveSdos;
using plem::fieldbus::ethercat::wrap_diff;
using plem::fieldbus::test::count_calls;
using plem::fieldbus::test::DeviceServoTestAccess;
using plem::fieldbus::test::FakeEcrt;
using plem::fieldbus::test::index_of;
using plem::fieldbus::test::le32;
using plem::fieldbus::test::log_contains;
using plem::fieldbus::test::make_servo_info;
using plem::fieldbus::test::pdo_offset;
using plem::fieldbus::test::six_servos;

// Cycle-time injection
// Master send interval = the cycle time injected via set_cycle_time_ns(),
// else the 1 ms default; these tests pin both paths.

TEST_F(EtherCATBusFakeMasterTest, InitializeUsesInjectedCycleTimeForSendInterval) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_cycle_time_ns(200'000);  // 5 kHz

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    EXPECT_TRUE(log_contains("set_send_interval(us=200)")) << "injected cycle time must drive the master send interval";
}

TEST_F(EtherCATBusFakeMasterTest, InitializeKeepsDefaultCycleTimeWhenNotInjected) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    EXPECT_TRUE(log_contains("set_send_interval(us=1000)"))
        << "without injection the 1 ms default must be preserved byte-for-byte";
}

// Distributed clocks
// The DC contract is pinned at the bus level: (a) config_dc lands per servo
// before master activation with the drive's registered sync cycle, a firmware
// property (CoE 0x1C32:02), never the bus period (programming the bus period
// tripped the drive's sync supervision: AL 0x002C Fatal Sync → SAFEOP loop),
// (b) the drive's 0x1C32 is never read; the cycle is written to its 0x1C32:02
// only for a drive that takes it from the master, (c) DC disabled adds zero
// ecrt calls (DC off byte-for-byte). The per-cycle "sync pair rides right before send()" leg is a
// 3-line call site in the RT loop (controller.cpp), run by the RT-loop
// harness in test_ethercat_controller_fake_master.cpp.

namespace {

constexpr uint32_t kDriveSync0CycleNs = 2'000'000;  // a drive-fixed sync cycle (0x1C32:02, read-only)

std::vector<ServoInfo> six_servos_with_dc() {
    auto infos = six_servos();
    for (auto& info : infos) {
        info.dc_assign_activate = 0x0300;  // ESI DC-Synchron AssignActivate
        info.dc_sync0_shift_ns = 0;
        info.dc_sync0_cycle_ns = kDriveSync0CycleNs;
    }
    return infos;
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, DcConfiguresEachServoWithDriveCycleBeforeActivateAndSelectsServo0RefClock) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with_dc());
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    const int activate_idx = index_of("master_activate");
    ASSERT_GE(activate_idx, 0);
    for (int pos = 0; pos < 6; ++pos) {
        const int dc_idx = index_of("slave_config_dc(pos=" + std::to_string(pos) + ",aa=0x300,cycle=2000000,shift=0");
        ASSERT_GE(dc_idx, 0)
            << "servo " << pos
            << " must be DC-configured with the DRIVE's registered sync cycle (2 ms), "
               "never the 1 ms bus period: the bus period trips the drive's sync supervision (AL 0x002C)";
        EXPECT_LT(dc_idx, activate_idx) << "IgH rejects DC config after activation — ordering is the contract";
    }
    const int ref_idx = index_of("select_reference_clock(pos=0)");
    ASSERT_GE(ref_idx, 0) << "the lowest-ring-position servo (servo 0 here) must be the explicit DC reference clock "
                             "(never topology-first implicit)";
    EXPECT_LT(ref_idx, activate_idx);
}

// The reference clock is the first DC slave on the ring, whatever order the
// file lists the slaves in: IgH compensates propagation delay only downstream
// of it, and slaves the frame reaches first would be written before the
// reference time is read.
TEST_F(EtherCATBusFakeMasterTest, DcReferenceClockIsTheLowestRingPositionNotTheFirstInTheFile) {
    auto infos = six_servos_with_dc();
    for (size_t i = 0; i < infos.size(); ++i) {
        infos[i].slave_position = static_cast<int32_t>(infos.size() - 1 - i);  // file lists the ring backwards
    }
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_GE(index_of("select_reference_clock(pos=0)"), 0);
    EXPECT_EQ(index_of("select_reference_clock(pos=5)"), -1);
}

TEST_F(EtherCATBusFakeMasterTest, DcSync0CycleStaysDriveConstantAtFasterBusPeriod) {
    // 5 kHz bus (200 µs): SYNC0 stays the drive's 2 ms (an exact 10× multiple).
    // The latch grid is a drive property; it must not follow the bus period.
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with_dc());
    bus.set_cycle_time_ns(200'000);
    bus.set_dc_enable(true);

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_TRUE(log_contains("slave_config_dc(pos=0,aa=0x300,cycle=2000000,shift=0"));
}

TEST_F(EtherCATBusFakeMasterTest, DcEnableRejectsServoWithoutSync0Cycle) {
    auto infos = six_servos_with_dc();
    for (auto& info : infos) {
        info.dc_sync0_cycle_ns = 0;  // assign_activate registered, latch cycle not
    }
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);

    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok()) << "unregistered sync cycle + dc_enable must fail fast, never guess a latch grid";
    EXPECT_NE(refused.message().find("DC enabled but axis 0 has no dc_sync0_cycle_ns registered"), std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("master_activate"), -1);
}

TEST_F(EtherCATBusFakeMasterTest, DcEnableRejectsSync0CycleNotMultipleOfBusPeriod) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with_dc());
    bus.set_cycle_time_ns(300'000);  // 2 ms % 300 µs != 0, so latches would drift across frame boundaries
    bus.set_dc_enable(true);

    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok()) << "SYNC0 must be an integer multiple of the bus period (N frames : 1 latch)";
    EXPECT_NE(refused.message().find("is not an integer multiple of the bus period 300000 ns"), std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("slave_config_dc"), -1) << "must reject before configuring any slave";
    EXPECT_EQ(index_of("master_activate"), -1);
}

// No read of the drive's 0x1C32
// The bus never reads the drive's cycle time: before DC is programmed a drive
// that mirrors its SYNC0 register reads 0 or a stale value, so the read could
// only mislead. A cycle the drive cannot run is the drive's to refuse.

TEST_F(EtherCATBusFakeMasterTest, DcReadsNothingFromTheDriveSyncParameters) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with_dc());
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(index_of("master_sdo_upload"), -1) << "no 0x1C32 (or any) upload during DC configuration";
    EXPECT_TRUE(log_contains("slave_config_dc(pos=0,aa=0x300,cycle=2000000,shift=0"));
}

TEST_F(EtherCATBusFakeMasterTest, DcDisabledAddsNoEcrtCalls) {
    auto infos = six_servos_with_dc();
    for (auto& info : infos) {
        info.dc_write_cycle_time = true;  // inert without DC
    }
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);
    // dc_enable defaults to off: the DC-off behavior byte-for-byte.

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(index_of("slave_config_dc"), -1);
    EXPECT_EQ(index_of("select_reference_clock"), -1);
    EXPECT_EQ(index_of("idx=0x1c3"), -1) << "no 0x1C32/0x1C33 write or read without DC";

    FakeEcrt::reset();
    bus.sync_distributed_clocks();
    EXPECT_EQ(bus.dc_sync_monitor_process(), 0u);
    EXPECT_TRUE(FakeEcrt::calls().empty()) << "disabled DC must issue zero ecrt calls in the hot loop";
}

TEST_F(EtherCATBusFakeMasterTest, SyncDistributedClocksIssuesRefSlavesThenMonitorQueueInOrder) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with_dc());
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    FakeEcrt::reset();
    bus.sync_distributed_clocks();

    ASSERT_EQ(FakeEcrt::calls().size(), 3u);
    EXPECT_EQ(FakeEcrt::calls()[0], "sync_reference_clock");
    EXPECT_EQ(FakeEcrt::calls()[1], "sync_slave_clocks");
    EXPECT_EQ(FakeEcrt::calls()[2], "sync_monitor_queue");
}

TEST_F(EtherCATBusFakeMasterTest, DcSyncMonitorProcessReturnsRawDriftIncludingSentinel) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with_dc());
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    FakeEcrt::set_sync_monitor_drift_ns(1234);
    EXPECT_EQ(bus.dc_sync_monitor_process(), 1234u);

    FakeEcrt::set_sync_monitor_drift_ns(0xFFFFFFFFu);
    EXPECT_EQ(bus.dc_sync_monitor_process(), 0xFFFFFFFFu)
        << "the IgH error sentinel must pass through raw — the publisher fold layer decides (skip, not max)";
}

TEST_F(EtherCATBusFakeMasterTest, DcEnableRejectsServoWithoutAssignActivate) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());  // dc_assign_activate defaults to 0 = unregistered
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);

    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok()) << "unregistered assign_activate + dc_enable must fail fast, never guess 0x300";
    EXPECT_NE(refused.message().find("DC enabled but axis 0 has no dc_assign_activate registered"), std::string::npos)
        << refused.message();
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_EQ(index_of("master_activate"), -1) << "must fail before activation (no silent DC-off fallback)";
}

TEST_F(EtherCATBusFakeMasterTest, DcEnableRejectsInvalidCycleTime) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with_dc());
    // No set_cycle_time_ns(): DC requires the explicitly injected active period;
    // deriving SYNC0 from the 1 ms default would program a period the bus
    // does not run.
    bus.set_dc_enable(true);

    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("DC enabled but no cycle time was set"), std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master"));
}

// Master release (IgH keeps the /dev/EtherCAT reservation until ecrt_release_master)

TEST_F(EtherCATBusFakeMasterTest, InitializeFailureReleasesRequestedMaster) {
    EtherCATBus bus(0);
    // No servo config injected → initialize() refuses after the master was requested.

    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(log_contains("release_master"))
        << "a failed initialize() must release the requested master — dropping the handle keeps the "
           "/dev/EtherCAT reservation for the process lifetime and blocks every later bring-up attempt";
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRefusesASecondCallOnceCompleted) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_cycle_time_ns(1'000'000);
    const auto first = bus.initialize();
    ASSERT_TRUE(first.is_ok()) << first.message();

    const auto again = bus.initialize();
    ASSERT_FALSE(again.is_ok());
    EXPECT_NE(again.message().find("already holds its master"), std::string::npos) << again.message();
    EXPECT_EQ(count_calls("request_master"), 1)
        << "a second request would overwrite the handle and leak the first /dev/EtherCAT reservation";
}

TEST_F(EtherCATBusFakeMasterTest, DcConfigureFailureReleasesRequestedMaster) {
    // The master refuses to reserve the cycle-time write → configure_DC fails after request_master.
    FakeEcrt::set_config_sdo_failure(0x1C32, 0x02, -12);
    auto infos = six_servos_with_dc();
    infos[0].dc_write_cycle_time = true;
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);

    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("slave 0: reserving the cycle time 0x1C32:02 = 2000000 ns failed (-12)."),
              std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master"))
        << "a DC failure must release the master like every sibling failure path — otherwise a retry on a "
           "fresh bus fails at request_master and points at the wrong cause";
}

// Every refusal step's own cause reaches initialize()'s message; the bus does
// not log it, so a generic summary would leave the operator with no cause.
TEST_F(EtherCATBusFakeMasterTest, AStartupSdoTheMasterRefusesIsNamedInTheRefusal) {
    FakeEcrt::set_config_sdo_failure(0x2000, 0x01, -5);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_startup_sdos({SlaveSdos{0, {{0x2000, 0x01, 2, 0x1234}}}});
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("slave 0: reserving startup SDO 0x2000:01 failed (-5)."), std::string::npos)
        << refused.message();
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, AFailedActivationIsNamedInTheRefusal) {
    FakeEcrt::set_activate_failure(-5);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("Failed to activate EtherCAT master (ecrt_master_activate() returned -5)"),
              std::string::npos)
        << refused.message();
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, ADomainTheMasterCannotCreateIsNamedInTheRefusal) {
    FakeEcrt::set_create_domain_failure();
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("Failed to create the EtherCAT domain"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("`sudo systemctl restart ethercat`"), std::string::npos) << refused.message();
    EXPECT_EQ(index_of("slave_config("), -1) << "no slave is configured without a domain";
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, ADomainWithoutProcessDataIsRefusedAfterTheActivation) {
    FakeEcrt::set_domain_data_failure();
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("Failed to get domain process data (ecrt_domain_data() returned null"),
              std::string::npos)
        << refused.message();
    EXPECT_EQ(bus.domain_pd(), nullptr) << "no loop may run on a missing process image";
    EXPECT_TRUE(log_contains("release_master"));
}

// A configured position with no slave on the ring: IgH hands out the slave
// config anyway (it applies once a slave appears) and refuses the slave read.
TEST_F(EtherCATBusFakeMasterTest, AnAbsentDeclaredSlaveIsRefusedByItsPosition) {
    FakeEcrt::set_absent_slave(2);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("axis 2 (slave 2"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("Failed to get slave info for position 2"), std::string::npos)
        << refused.message();
    EXPECT_NE(refused.message().find("check that a slave sits at that position on the ring"), std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_TRUE(bus.servo_devices().empty());
}

TEST_F(EtherCATBusFakeMasterTest, AnAbsentScannedSlaveSaysThePositionMayHoldNoSlave) {
    FakeEcrt::set_absent_slave(2);
    auto servos = six_servos();
    servos[2].pdo_mapping = nullptr;  // read from the slave
    EtherCATBus bus(0);
    bus.set_servo_config(servos);
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("slave 2 (axes 2): cannot read the slave's PDO mapping from the master"),
              std::string::npos)
        << refused.message();
    EXPECT_NE(refused.message().find("the position may hold no slave"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("check `ethercat slaves`, then retry (or `ethercat rescan`)"), std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, AMasterThatCannotBeRequestedNamesThePossibleCauses) {
    FakeEcrt::set_request_master_failure();
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("Failed to request EtherCAT master 0"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("another process"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("not running"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("cannot open /dev/EtherCAT0"), std::string::npos) << refused.message();
}

TEST_F(EtherCATBusFakeMasterTest, AnEntryListTheDomainRefusesIsNamedInTheRefusal) {
    FakeEcrt::set_unmapped_entry(0, 0x6040, 0);  // in the declared layout, refused by the master
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find(
                  "Failed to register the PDO entry list with the domain (ecrt_domain_reg_pdo_entry_list() returned "),
              std::string::npos)
        << refused.message();
    // A declared slave's configured layout supersedes what the master read, so
    // no entry is named against the scan: the generic causes stay.
    EXPECT_NE(refused.message().find("possible causes"), std::string::npos) << refused.message();
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, ADeclaredLayoutTheMasterRefusesToRecordIsNamedInTheRefusal) {
    FakeEcrt::set_config_pdos_failure(2);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find(
                  "slave 2: the master refused to record the declared PDO layout (ecrt_slave_config_pdos() failed)"),
              std::string::npos)
        << refused.message();
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, DestructorReleasesMasterAfterSuccessfulInitialize) {
    {
        EtherCATBus bus(0);
        bus.set_servo_config(six_servos());
        const auto init = bus.initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
    }

    EXPECT_TRUE(log_contains("release_master"));
}

// A device the master cannot configure makes its constructor throw. That must
// stay inside initialize(): a refusal, the master released, and no device
// kept: the ones registered before it borrow slave configs the released master
// owned, and a retry on the same bus would register them a second time.
TEST_F(EtherCATBusFakeMasterTest, InitializeReturnsErrorWhenServoSlaveConfigFails) {
    FakeEcrt::set_slave_config_failure(2);  // servos 0 and 1 are already registered
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());

    EXPECT_NO_THROW(EXPECT_FALSE(bus.initialize().is_ok()));
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master")) << "a failed device must not leak the requested master";
    EXPECT_TRUE(bus.servo_devices().empty()) << "devices must not outlive the master that owns their slave configs";
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRetryAfterServoFailureRegistersEachServoOnce) {
    FakeEcrt::set_slave_config_failure(1);  // the 2nd servo fails after the 1st registered
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    ASSERT_FALSE(bus.initialize().is_ok());

    FakeEcrt::reset();  // the slave answers again
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(bus.servo_devices().size(), six_servos().size()) << "a retry must not keep the failed run's devices";
}

// A file whose vendor_id/product_code differ from the slave at that ring
// position is refused by name. Without the check the master reported only
// "Failed to create slave config: No such file or directory" and a refusal
// with no cause.
TEST_F(EtherCATBusFakeMasterTest, InitializeRefusesASlaveWhoseIdentityDiffersFromTheConfiguration) {
    FakeEcrt::set_slave_identity(2, 0xdead, 0xbeef);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());

    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_TRUE(bus.servo_devices().empty());
}

// A refusal says its cause: two different failures of the same registration
// step carry different, actionable messages.
TEST_F(EtherCATBusFakeMasterTest, EachRegistrationRefusalSaysItsOwnCause) {
    FakeEcrt::set_slave_identity(2, 0xdead, 0xbeef);
    EtherCATBus wrong_drive(0);
    wrong_drive.set_servo_config(six_servos());
    const auto identity = wrong_drive.initialize();
    ASSERT_FALSE(identity);
    EXPECT_NE(identity.message().find("but the slave at that ring position is vendor 0xdead product 0xbeef"),
              std::string::npos)
        << identity.message();

    FakeEcrt::reset();
    FakeEcrt::set_slave_config_failure(1);
    EtherCATBus no_slave(0);
    no_slave.set_servo_config(six_servos());
    const auto missing = no_slave.initialize();
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.message().find("check that a slave sits at that position on the ring"), std::string::npos)
        << missing.message();
    EXPECT_NE(identity.message(), missing.message());
}

// Servo-spec validation (Fail Fast at the config boundary)
// An invalid servo declaration must be rejected before hardware contact.
// Initialization validates channel units and finite positive factors before
// ecrt_request_master, so a bad configuration issues zero ecrt calls.
// Conversion helpers require valid channels and assert that precondition in
// debug builds.

// A servo that never declared when its drive reports the mode
// (or carries an unknown number) is refused at the same boundary (before the
// master is requested), so the release admission never runs under a timing
// the drive did not declare.
TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsUndeclaredModeDisplayBeforeTouchingHardware) {
    for (const auto bad :
         {plem::fieldbus::ModeDisplayTiming::UNDECLARED, static_cast<plem::fieldbus::ModeDisplayTiming>(3)}) {
        SCOPED_TRACE("mode_display_timing " + std::to_string(static_cast<int>(bad)));
        FakeEcrt::reset();
        auto infos = six_servos();
        infos[4].mode_display_timing = bad;
        EtherCATBus bus(0);
        bus.set_servo_config(infos);
        bus.set_cycle_time_ns(1'000'000);
        EXPECT_FALSE(bus.initialize().is_ok());
        EXPECT_TRUE(FakeEcrt::calls().empty()) << "rejected before any ecrt call";
    }
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsInvalidServoSpecBeforeTouchingHardware) {
    auto infos = six_servos();
    infos[3].factors.torque.gear_ratio = 0.0;  // invalid: would divide-by-zero / silent-zero downstream

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);

    EXPECT_FALSE(bus.initialize().is_ok())
        << "an invalid ServoSpec must fail initialize() before any device exists — never construct a "
           "device that silently zeroes commands";
    EXPECT_TRUE(FakeEcrt::calls().empty())
        << "validation must fail fast BEFORE requesting the master (no half-acquired hardware on a bad config)";
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsAModeWithoutACommandPathBeforeTouchingHardware) {
    // The declared mode is commanded on 0x6060 from the first frame; a mode
    // no command path serves is refused at the same pre-hardware window as an
    // invalid ServoSpec, whether the layout is declared or scanned.
    auto infos = six_servos();
    infos[2].modes_of_operation = plem::fieldbus::OperationMode::NO_MODE;

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);

    EXPECT_FALSE(bus.initialize().is_ok()) << "an unserved mode must fail initialize() at the configuration check";
    EXPECT_TRUE(FakeEcrt::calls().empty()) << "validation must fail fast BEFORE requesting the master";
}

TEST_F(EtherCATBusFakeMasterTest, InitializeAcceptsMixedCyclicSyncModesOnOneBus) {
    auto infos = six_servos();
    infos[1].modes_of_operation = plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION;
    infos[2].modes_of_operation = plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY;

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(bus.servo_devices()[1]->get_modes_of_operation(), OperationMode::CYCLIC_SYNC_POSITION);
    EXPECT_EQ(bus.servo_devices()[2]->get_modes_of_operation(), OperationMode::CYCLIC_SYNC_VELOCITY);
}

TEST_F(EtherCATBusFakeMasterTest, ProductionServoConstructorCarriesTheDeclarationToTheWire) {
    // The carry through the production constructor (the bus's path), pinned
    // on a device built directly with the fake master: the declaration is
    // what get_modes_of_operation() reports and the 0x6060 byte written to the
    // domain from the first frame on. The carry only; which modes the bus
    // admits is InitializeAcceptsMixedCyclicSyncModesOnOneBus (the robot
    // assembly's CST-only gate is its own, until the assembly admits CSP).
    auto info = make_servo_info();
    info.modes_of_operation = OperationMode::CYCLIC_SYNC_POSITION;
    DeviceServo servo(FakeEcrt::master(), /*slave_position=*/0, info);
    EXPECT_EQ(servo.get_modes_of_operation(), OperationMode::CYCLIC_SYNC_POSITION);

    ec_pdo_entry_reg_t regs[32] = {};
    uint32_t index = 0;
    const auto registered = servo.register_PDOs(regs, index);
    ASSERT_TRUE(registered.is_ok()) << registered.message();
    ASSERT_EQ(ecrt_domain_reg_pdo_entry_list(ecrt_master_create_domain(FakeEcrt::master()), regs), 0);
    uint8_t* pd = ecrt_domain_data(nullptr);
    servo.write(pd);
    EXPECT_EQ(EC_READ_U8(pd + pdo_offset(0, 0x6060)), 0x08) << "0x6060 carries the declared mode";
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsNonFiniteGearRate) {
    auto infos = six_servos();
    infos[0].factors.position.gear_ratio =
        std::numeric_limits<double>::infinity();  // > 0 but produces garbage conversions

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);

    EXPECT_FALSE(bus.initialize().is_ok()) << "a non-finite gear_ratio must be rejected at init, never reach a device";
    EXPECT_EQ(index_of("request_master"), -1);
}

// Target-position seeding (actual → target mirror in DeviceServo::read)
// DeviceServo itself has no control-path writer of the target (the
// controller's csp step writes 0x607A after read()), so
// read() seeds target from actual unconditionally every cycle; the vendor
// reference writes actual into target before touching the controlword, and
// combined with the to_hardware/from_hardware inverse pair the wire
// invariant is: target word == last actual word, bit-exactly.

namespace {

// PDO object indices; servo_entry_offset() resolves them to the fake domain's offsets.
constexpr unsigned int kControlwordEntry = 0x6040;
constexpr unsigned int kTargetPositionEntry = 0x607A;
constexpr unsigned int kStatuswordEntry = 0x6041;
constexpr unsigned int kActualPositionEntry = 0x6064;

uint32_t servo_entry_offset(size_t servo, unsigned int index) {
    return pdo_offset(static_cast<unsigned int>(servo), index);
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, TargetSeedingMirrorsActualUnconditionallyThroughEnableFrame) {
    auto infos = six_servos();
    infos[0].flip = false;
    infos[0].joint_offset = Pulse(1000);
    infos[1].flip = true;
    infos[1].joint_offset = Pulse(-500);

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    uint8_t* pd = ecrt_domain_data(nullptr);
    const auto& servos = bus.servo_devices();

    // No-flip echo: raw actual wire word comes back as the target word.
    EC_WRITE_S32(pd + servo_entry_offset(0, kActualPositionEntry), 987654);
    servos[0]->read(pd);
    servos[0]->write(pd);
    EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(0, kTargetPositionEntry)), 987654);

    // Flip servo, driven through the enable sequence with a stale
    // target word injected up front: the mirror must overwrite it
    // (unconditional: while no control path commands position, a stale
    // manual target must not survive into any frame), and the enable frame
    // must carry the seeded word. Cycle-by-cycle (controlword, target) trace:
    //
    //   cycle  feedback(actual)  controlword   target wire
    //   0      0 (cold domain)   0x0000        0
    //   1      -1734             0x0000        -1734
    //   2      -1734             0x0006        -1734
    //   3      -1734             0x000F        -1734  ← enable frame, seeded
    servos[1]->set_target_position({999'999, 0.0});  // stale: must never reach the wire
    auto& servo = *servos[1];
    const uint32_t cw_off = servo_entry_offset(1, kControlwordEntry);
    const uint32_t tgt_off = servo_entry_offset(1, kTargetPositionEntry);
    const uint32_t act_off = servo_entry_offset(1, kActualPositionEntry);

    const int32_t resting_wire = -1734;
    const uint16_t controlwords[4] = {0x0000, 0x0000, 0x0006, 0x000F};
    const int32_t expected_target[4] = {0, resting_wire, resting_wire, resting_wire};

    for (int cycle = 0; cycle < 4; ++cycle) {
        SCOPED_TRACE("cycle " + std::to_string(cycle));
        if (cycle >= 1) {
            EC_WRITE_S32(pd + act_off, resting_wire);
        }
        servo.read(pd);
        DeviceServoTestAccess::set_controlword(servo, controlwords[cycle]);
        servo.write(pd);
        EXPECT_EQ(EC_READ_U16(pd + cw_off), controlwords[cycle]);
        EXPECT_EQ(EC_READ_S32(pd + tgt_off), expected_target[cycle]);
    }
}

// Position before the first enable (ServoInfo::position_before_enable_index)
// A drive that reports 0x6064 = 0 until its first enable gets its position
// from a declared object, read once before activation: the axis reports it
// and mirrors it into 0x607A until the drive's own word arrives, so the
// enable frame carries the real position instead of 0.

TEST_F(EtherCATBusFakeMasterTest, PositionBeforeEnableStandsInUntilTheDriveReportsItsOwn) {
    auto infos = six_servos();
    infos[0].position_before_enable_index = 0x2300;
    infos[0].flip = true;
    infos[0].joint_offset = Pulse(880000);
    infos[1].position_before_enable_index = 0x2300;
    FakeEcrt::set_sdo_upload_entry(0, 0x6064, 0, 0, le32(0));
    FakeEcrt::set_sdo_upload_entry(0, 0x2300, 0, 0, le32(870000));
    FakeEcrt::set_sdo_upload_entry(1, 0x6064, 0, 0, le32(5000));  // already reports its position

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(index_of("master_sdo_upload(pos=1,idx=0x2300,sub=0x0)"), -1)
        << "a drive that reports 0x6064 needs no stand-in";

    uint8_t* pd = ecrt_domain_data(nullptr);
    auto& servo = *bus.servo_devices()[0];
    const uint32_t act_off = servo_entry_offset(0, kActualPositionEntry);
    const uint32_t tgt_off = servo_entry_offset(0, kTargetPositionEntry);

    EC_WRITE_S32(pd + act_off, 0);  // before the first enable
    servo.read(pd);
    const double before_enable = servo.get_actual_position().value();
    servo.write(pd);
    EXPECT_EQ(EC_READ_S32(pd + tgt_off), 870000) << "the enable frame carries the stand-in, not 0";

    EC_WRITE_S32(pd + act_off, 870000);  // the drive reports its own position
    servo.read(pd);
    EXPECT_DOUBLE_EQ(before_enable, servo.get_actual_position().value()) << "same counts, same angle";

    EC_WRITE_S32(pd + act_off, 0);  // once the drive has reported, its word is the position
    servo.read(pd);
    servo.write(pd);
    EXPECT_EQ(EC_READ_S32(pd + tgt_off), 0);
}

TEST_F(EtherCATBusFakeMasterTest, PositionBeforeEnableReachesTheRawWordACspAxisMirrors) {
    auto infos = six_servos();
    infos[0].modes_of_operation = OperationMode::CYCLIC_SYNC_POSITION;
    infos[0].position_before_enable_index = 0x2300;
    FakeEcrt::set_sdo_upload_entry(0, 0x6064, 0, 0, le32(0));
    FakeEcrt::set_sdo_upload_entry(0, 0x2300, 0, 0, le32(-160000));

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    uint8_t* pd = ecrt_domain_data(nullptr);
    auto& servo = *bus.servo_devices()[0];
    EC_WRITE_S32(pd + servo_entry_offset(0, kActualPositionEntry), 0);
    servo.read(pd);
    servo.write(pd);
    EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(0, kTargetPositionEntry)), -160000);
}

TEST_F(EtherCATBusFakeMasterTest, PositionBeforeEnableFailingWithoutAnAbortIsReadAgain) {
    auto infos = six_servos();
    infos[0].position_before_enable_index = 0x2300;
    FakeEcrt::set_sdo_upload_entry(0, 0x6064, 0, 0, le32(0));
    FakeEcrt::set_sdo_upload_transient_failures(0, 0x2300, 0, 1);
    FakeEcrt::set_sdo_upload_entry(0, 0x2300, 0, 0, le32(870000));

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    uint8_t* pd = ecrt_domain_data(nullptr);
    auto& servo = *bus.servo_devices()[0];
    EC_WRITE_S32(pd + servo_entry_offset(0, kActualPositionEntry), 0);
    servo.read(pd);
    servo.write(pd);
    EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(0, kTargetPositionEntry)), 870000);
}

TEST_F(EtherCATBusFakeMasterTest, PositionBeforeEnableThatCannotBeReadLeavesTheAxisAsItWas) {
    auto infos = six_servos();
    infos[0].position_before_enable_index = 0x2300;
    FakeEcrt::set_sdo_upload_entry(0, 0x6064, 0, 0, le32(0));
    FakeEcrt::set_sdo_upload_entry(0, 0x2300, 0, -EIO, {}, 0x06020000);

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message() << "a failed read never refuses to start";
    EXPECT_TRUE(log_contains("master_sdo_upload(pos=0,idx=0x2300,sub=0x0)"));

    uint8_t* pd = ecrt_domain_data(nullptr);
    auto& servo = *bus.servo_devices()[0];
    EC_WRITE_S32(pd + servo_entry_offset(0, kActualPositionEntry), 0);
    servo.read(pd);
    servo.write(pd);
    EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(0, kTargetPositionEntry)), 0);
}

// Characterization: the CST mirror decodes 0x6064 into
// joint counts (saturating at int32) and re-encodes it, so an offset large
// enough to saturate the decode puts a word on 0x607A that is not the actual
// one. CST drives ignore 0x607A; this pins the robot/CST bytes so the CSP
// raw-word mirror cannot leak into them.
TEST_F(EtherCATBusFakeMasterTest, CstMirrorKeepsItsDecodeReencodeWordAtASaturatingOffset) {
    auto infos = six_servos();
    infos[0].flip = false;
    infos[0].joint_offset = Pulse(2'000'000'000);
    infos[1].flip = true;
    infos[1].joint_offset = Pulse(2'000'000'000);

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    uint8_t* pd = ecrt_domain_data(nullptr);
    const auto& servos = bus.servo_devices();

    for (size_t s = 0; s < 2; ++s) {
        EC_WRITE_S32(pd + servo_entry_offset(s, kActualPositionEntry), -2'000'000'000);
        servos[s]->read(pd);
        servos[s]->write(pd);
    }
    EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(0, kTargetPositionEntry)), -147'483'648);
    EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(1, kTargetPositionEntry)), -147'483'647);
}

// CSP: 0x607A mirrors the raw 0x6064 word, with no decode and
// re-encode, so a saturating offset cannot put a word other than the actual
// one on the wire of a drive that follows 0x607A. An axis whose feedback does
// not decode into int32 joint counts reports it (the controller refuses to
// command it). A command word written after read() reaches the wire; the next
// read() mirrors again.
TEST_F(EtherCATBusFakeMasterTest, CspMirrorCopiesTheRawActualWordEvenWhereTheDecodeSaturates) {
    auto infos = six_servos();
    for (size_t s = 0; s < 2; ++s) {
        infos[s].modes_of_operation = OperationMode::CYCLIC_SYNC_POSITION;
        infos[s].joint_offset = Pulse(2'000'000'000);
    }
    infos[1].flip = true;

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    uint8_t* pd = ecrt_domain_data(nullptr);
    const auto& servos = bus.servo_devices();

    for (size_t s = 0; s < 2; ++s) {
        SCOPED_TRACE("servo " + std::to_string(s));
        EC_WRITE_S32(pd + servo_entry_offset(s, kActualPositionEntry), -2'000'000'000);
        servos[s]->read(pd);
        EXPECT_FALSE(servos[s]->position_word_decodable());
        servos[s]->write(pd);
        EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(s, kTargetPositionEntry)), -2'000'000'000);

        EC_WRITE_S32(pd + servo_entry_offset(s, kActualPositionEntry), 1'999'990'000);
        servos[s]->read(pd);
        EXPECT_TRUE(servos[s]->position_word_decodable());
        servos[s]->set_target_position({1'999'990'123, 0.0});
        servos[s]->write(pd);
        EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(s, kTargetPositionEntry)), 1'999'990'123);
        servos[s]->read(pd);
        servos[s]->write(pd);
        EXPECT_EQ(EC_READ_S32(pd + servo_entry_offset(s, kTargetPositionEntry)), 1'999'990'000);
    }
}

// Endless position axis (limits.position: null)
// The host counts in int64 from the drive's int32 0x6064 words. Only a valid
// frame (WKC complete, statusword non-zero) is an observation; it advances
// the count by the short-way word difference, so acc ≡ word (mod 2^32) on
// every valid frame. The angle is that count through flip and scale.
class EndlessPositionTest : public EtherCATBusFakeMasterTest {
  protected:
    void build(OperationMode mode, bool flip) {
        auto infos = six_servos();
        infos[0].modes_of_operation = mode;
        infos[0].flip = flip;
        infos[0].position_endless = true;
        flip_ = flip;
        bus_ = std::make_unique<EtherCATBus>(0);
        bus_->set_servo_config(infos);
        const auto init = bus_->initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
        pd_ = ecrt_domain_data(nullptr);
    }

    /// One frame: the drive reports `word` with `status`, the master reports
    /// `wkc_complete`; then read().
    void frame(int32_t word, bool wkc_complete = true, uint16_t status = kStatus) {
        EC_WRITE_S32(pd_ + servo_entry_offset(0, kActualPositionEntry), word);
        EC_WRITE_U16(pd_ + servo_entry_offset(0, kStatuswordEntry), status);
        servo().note_received_frame(wkc_complete);
        servo().read(pd_);
    }

    /// The angle of a host count (hardware sign): the same formula the
    /// device decodes with.
    double angle_of(int64_t count) const {
        return endless_count_to_angle(flip_ ? -count : count, info_position());
    }

    const plem::fieldbus::ethercat::PositionChannel& info_position() const {
        static const auto factors = make_servo_info().factors;
        return factors.position;
    }

    int64_t acc() const {
        return bus_->servo_devices()[0]->position_count();
    }

    DeviceServo& servo() {
        return *bus_->servo_devices()[0];
    }

    uint32_t word_mod() const {
        return static_cast<uint32_t>(EC_READ_S32(pd_ + servo_entry_offset(0, kActualPositionEntry)));
    }

    static constexpr uint16_t kStatus = 0x0237;
    static constexpr int32_t kMin = std::numeric_limits<int32_t>::min();
    static constexpr int32_t kMax = std::numeric_limits<int32_t>::max();
    std::unique_ptr<EtherCATBus> bus_;
    uint8_t* pd_ = nullptr;
    bool flip_ = false;
};

TEST_F(EndlessPositionTest, CountsAcrossTheWrapInBothDirectionsWithAndWithoutFlip) {
    for (const bool flip : {false, true}) {
        SCOPED_TRACE(flip ? "flip" : "no flip");
        build(OperationMode::CYCLIC_SYNC_POSITION, flip);
        const int32_t forward[] = {kMax - 300,         kMax - 100, kMax,          kMin,     kMin + 200,
                                   0x7000'0000 + kMin, -5,         1'000'000'000, kMax - 7, kMin + 3};
        int64_t expected = kMax - 300;
        frame(forward[0]);
        ASSERT_TRUE(servo().is_initialized());
        for (const int32_t word : forward) {
            if (word != forward[0]) {
                expected +=
                    wrap_diff(word, static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint64_t>(expected))));
                frame(word);
            }
            EXPECT_EQ(acc(), expected);
            EXPECT_EQ(static_cast<uint32_t>(static_cast<uint64_t>(acc())), word_mod()) << "acc ≡ word (mod 2^32)";
            EXPECT_TRUE(servo().position_word_decodable());
            EXPECT_EQ(servo().get_actual_position().value(), angle_of(expected));
        }
        EXPECT_GT(acc(), int64_t{1} << 32) << "two forward wraps";
        // And back down through INT32_MIN/INT32_MAX twice.
        for (const int32_t word : {kMin + 3, kMax, kMin, kMax - 2, 0, kMin + 1, kMax}) {
            expected += wrap_diff(word, static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint64_t>(expected))));
            frame(word);
            EXPECT_EQ(acc(), expected);
            EXPECT_EQ(servo().get_actual_position().value(), angle_of(expected));
        }
    }
}

TEST_F(EndlessPositionTest, AnInvalidFrameBetweenWrapWordsMovesNothing) {
    build(OperationMode::CYCLIC_SYNC_POSITION, true);
    frame(kMin);
    const int64_t seeded = acc();
    const double angle = servo().get_actual_position().value();
    frame(0, /*wkc_complete=*/false);  // the zero image of a lost frame
    EXPECT_EQ(acc(), seeded);
    EXPECT_EQ(servo().get_actual_position().value(), angle);
    servo().write(pd_);
    EXPECT_EQ(EC_READ_S32(pd_ + servo_entry_offset(0, kTargetPositionEntry)), kMin)
        << "the hold keeps the observed word, not the lost frame's";
    EXPECT_EQ(servo().applied_command(), angle);
    frame(0, /*wkc_complete=*/true, /*status=*/0);  // WKC fine, but no statusword
    EXPECT_EQ(acc(), seeded);
    frame(kMin);
    EXPECT_EQ(acc(), seeded) << "the zero words never became the previous word";
    EXPECT_EQ(servo().get_actual_position().value(), angle);
}

TEST_F(EndlessPositionTest, SeedsOnlyOnTheFirstValidFrame) {
    build(OperationMode::CYCLIC_SYNC_TORQUE, false);
    frame(kMax - 1, /*wkc_complete=*/false);
    EXPECT_FALSE(servo().is_initialized()) << "a statusword on an incomplete frame is no observation";
    EXPECT_FALSE(servo().position_word_decodable());
    frame(123, /*wkc_complete=*/true, /*status=*/0);
    EXPECT_FALSE(servo().is_initialized());
    frame(kMax);
    EXPECT_TRUE(servo().is_initialized());
    EXPECT_EQ(acc(), kMax);
    frame(kMin);
    EXPECT_EQ(acc(), int64_t{kMax} + 1) << "seeded from the valid word, not an earlier one";
}

TEST_F(EndlessPositionTest, AGapIsIntegratedBelowHalfAPeriodAndAliasesAbove) {
    for (const bool flip : {false, true}) {
        SCOPED_TRACE(flip ? "flip" : "no flip");
        build(OperationMode::CYCLIC_SYNC_POSITION, flip);
        frame(-1000);
        for (int i = 0; i < 50; ++i) {
            frame(0, /*wkc_complete=*/false);
        }
        frame(-1000 + kMax);  // +2^31−1 after the gap
        EXPECT_EQ(acc(), -1000 + int64_t{kMax});

        for (int i = 0; i < 50; ++i) {
            frame(0, /*wkc_complete=*/false);
        }
        const int64_t before = acc();
        const int32_t jumped = static_cast<int32_t>(static_cast<uint32_t>(before) + (uint32_t{1} << 31) + 1);
        frame(jumped);  // +2^31+1: the documented ceiling; it reads as −(2^31−1)
        EXPECT_EQ(acc(), before - kMax);
        EXPECT_EQ(static_cast<uint32_t>(static_cast<uint64_t>(acc())), static_cast<uint32_t>(jumped))
            << "still congruent with the drive's word";
    }
}

TEST_F(EndlessPositionTest, LeavingTheDomainIsStickyAndTheLastAngleIsKept) {
    build(OperationMode::CYCLIC_SYNC_POSITION, false);
    frame(0);
    DeviceServoTestAccess::set_position_count(servo(), kEndlessCountDomain - 10);
    frame(-5);  // +5: still inside
    EXPECT_EQ(acc(), kEndlessCountDomain - 5);
    EXPECT_TRUE(servo().position_word_decodable());
    const double last = servo().get_actual_position().value();
    EXPECT_EQ(last, angle_of(kEndlessCountDomain - 5));

    frame(10);  // candidate D+10
    EXPECT_FALSE(servo().position_word_decodable());
    EXPECT_EQ(acc(), kEndlessCountDomain - 5) << "the candidate is discarded";
    EXPECT_EQ(servo().get_actual_position().value(), last);

    frame(-6);  // back inside relative to the retained count
    EXPECT_FALSE(servo().position_word_decodable()) << "sticky until restart";
    frame(-5);
    EXPECT_FALSE(servo().position_word_decodable());
}

TEST_F(EndlessPositionTest, TheNegativeDomainEdgeDecodesWithFlipWithoutOverflow) {
    build(OperationMode::CYCLIC_SYNC_POSITION, true);
    frame(0);
    DeviceServoTestAccess::set_position_count(servo(), -kEndlessCountDomain + 10);
    frame(0);  // −10: exactly −D
    EXPECT_EQ(acc(), -kEndlessCountDomain);
    EXPECT_TRUE(servo().position_word_decodable());
    EXPECT_EQ(servo().get_actual_position().value(), endless_count_to_angle(kEndlessCountDomain, info_position()));
    EXPECT_TRUE(std::isfinite(servo().get_actual_position().value()));
    frame(-1);
    EXPECT_FALSE(servo().position_word_decodable());
}

TEST_F(EndlessPositionTest, EncodeUsesTheWholeDomainAndRefusesBeyondIt) {
    build(OperationMode::CYCLIC_SYNC_POSITION, true);
    const auto& ch = info_position();
    const double per_count = endless_count_to_angle(1, ch);
    const auto inside = servo().encode_position(static_cast<double>(int64_t{3} << 33) * per_count);
    ASSERT_TRUE(inside) << "far beyond int32 counts";
    EXPECT_EQ(inside->word, 0) << "−3·2^33 folds to 0";
    EXPECT_FALSE(servo().encode_position(static_cast<double>(kEndlessCountDomain) * per_count * 1.000001));
    EXPECT_FALSE(servo().encode_position(std::numeric_limits<double>::quiet_NaN()));
}

TEST_F(EndlessPositionTest, FlippedRawHoldAndCommandsReportTheJointFrameAcrossTheWrap) {
    build(OperationMode::CYCLIC_SYNC_POSITION, true);
    frame(kMax - 10);
    frame(kMin + 10);  // +21 counts, through the wrap
    const int64_t count = int64_t{kMax} + 11;
    ASSERT_EQ(acc(), count);
    servo().write(pd_);
    EXPECT_EQ(EC_READ_S32(pd_ + servo_entry_offset(0, kTargetPositionEntry)), kMin + 10) << "raw hold";
    EXPECT_EQ(servo().applied_command(), angle_of(count)) << "the hold commands the unwrapped actual";
    EXPECT_EQ(servo().applied_command(), servo().get_actual_position().value());

    const auto target = servo().encode_position(angle_of(count + 40));
    ASSERT_TRUE(target);
    servo().set_target_position(*target);
    servo().write(pd_);
    EXPECT_EQ(EC_READ_S32(pd_ + servo_entry_offset(0, kTargetPositionEntry)), kMin + 50);
    EXPECT_EQ(servo().applied_command(), target->si) << "the accepted si, not a decode of the word";
    EXPECT_EQ(target->si, angle_of(count + 40));
}

// CiA 402 transition profile: mandatory declaration

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsServoWithoutCia402ProfileBeforeTouchingHardware) {
    auto infos = six_servos();
    infos[2].cia402_profile = plem::fieldbus::Cia402TransitionProfile{};  // undeclared = empty

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    bus.set_cycle_time_ns(1'000'000);

    EXPECT_FALSE(bus.initialize().is_ok())
        << "an absent CiA 402 transition profile must fail initialize() at the configuration check — "
           "the profile is a mandatory model declaration with no fallback table";
    EXPECT_TRUE(FakeEcrt::calls().empty())
        << "profile validation must fail fast BEFORE requesting the master (no half-acquired hardware)";
}

namespace {

/// The standard profile minus its NOT_READY_TO_SWITCH_ON row. Still a valid
/// declaration: that row is a compatibility row, never one of the required
/// pairs (plem/fieldbus/cia402_profile.hpp).
plem::fieldbus::Cia402TransitionProfile profile_without_not_ready_row() {
    auto profile = plem::fieldbus::kNotReadyShutdownCia402Profile;
    for (size_t i = 1; i < profile.transition_count; ++i) {
        profile.transitions[i - 1] = profile.transitions[i];
    }
    profile.transitions[--profile.transition_count] = {};
    return profile;
}

std::vector<ServoInfo> six_servos_with(const plem::fieldbus::Cia402TransitionProfile& profile) {
    auto infos = six_servos();
    for (auto& info : infos) {
        info.cia402_profile = profile;
    }
    return infos;
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, ColdBootReachesOperationEnabledWithNoNotReadyToSwitchOnRow) {
    // What omitting the compatibility row costs a cold boot, once the drive
    // has reported a state the profile can leave: nothing. Not "anything
    // decodable": 0x0020 decodes too, and decodes to the very state whose
    // exit row is missing (the case below). The FSM starts in
    // NOT_READY_TO_SWITCH_ON, the first statusword moves it to whatever the
    // drive reports (here SWITCH_ON_DISABLED, where these drives rest), and
    // transition 2 carries it from there. The claim is bounded to that: this
    // pins the walk after SWITCH_ON_DISABLED is observed, and says nothing
    // about a drive that never reports one (the case below).
    const auto profile = profile_without_not_ready_row();
    ASSERT_EQ(profile.transitions[0].from, plem::fieldbus::ServoState::SWITCH_ON_DISABLED) << "the row is gone";
    const auto valid = plem::fieldbus::validate_cia402_profile(profile);
    ASSERT_TRUE(valid.is_ok()) << valid.message() << "and the profile is still a valid declaration";

    EtherCATBus bus(0);
    bus.set_servo_config(six_servos_with(profile));
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    uint8_t* pd = ecrt_domain_data(nullptr);
    DeviceServo& servo = *bus.servo_devices()[0];
    const uint32_t cw_off = servo_entry_offset(0, kControlwordEntry);
    const uint32_t sw_off = servo_entry_offset(0, kStatuswordEntry);
    ASSERT_EQ(servo.get_servo_state(), plem::fieldbus::ServoState::NOT_READY_TO_SWITCH_ON) << "cold: nothing read yet";

    // (statusword the drive reports, state we then ask for, controlword owed)
    struct Step {
        uint16_t status;
        plem::fieldbus::ServoState target;
        uint16_t controlword;
    };
    const Step kTrace[] = {
        {0x0240, plem::fieldbus::ServoState::READY_TO_SWITCH_ON, 0x0006},  // transition 2, straight out of cold
        {0x0221, plem::fieldbus::ServoState::SWITCHED_ON, 0x0007},         // transition 3
        {0x0233, plem::fieldbus::ServoState::OPERATION_ENABLED, 0x000F},   // transition 4
    };
    int step = 0;
    for (const Step& s : kTrace) {
        SCOPED_TRACE("step " + std::to_string(step++));
        EC_WRITE_U16(pd + sw_off, s.status);
        servo.read(pd);
        ASSERT_TRUE(servo.transition_state(s.target)) << "the walk must not stall on the missing row";
        servo.write(pd);
        EXPECT_EQ(EC_READ_U16(pd + cw_off), s.controlword);
    }
    EC_WRITE_U16(pd + sw_off, 0x0237);
    servo.read(pd);
    EXPECT_EQ(servo.get_servo_state(), plem::fieldbus::ServoState::OPERATION_ENABLED);
}

TEST_F(EtherCATBusFakeMasterTest, WhileTheFsmSitsInNotReadyOnlyTheRowDecides) {
    // transition_state() consults the profile and nothing else; it never asks
    // status_recognized(), so for as long as the FSM is in
    // NOT_READY_TO_SWITCH_ON the row alone decides, whether or not the
    // statusword decoded. Three cases separate the two axes; the third shows the
    // row decides even when nothing decodes.
    //
    // An accepted request means a controlword was produced, not that a drive
    // moved: nothing here says a cold drive reporting nothing recovers.
    struct Case {
        const char* label;
        uint16_t status;
        bool with_not_ready_row;
        bool recognized;
        bool accepted;
    };
    const Case kCases[] = {
        {"nothing decodes, no row", 0x0064, false, false, false},
        {"NOT_READY decodes, still no row", 0x0020, false, true, false},
        {"nothing decodes, but the row is there", 0x0064, true, false, true},
    };

    for (const Case& c : kCases) {
        SCOPED_TRACE(c.label);
        FakeEcrt::reset();
        const auto profile =
            c.with_not_ready_row ? plem::fieldbus::kNotReadyShutdownCia402Profile : profile_without_not_ready_row();
        EtherCATBus bus(0);
        bus.set_servo_config(six_servos_with(profile));
        const auto init = bus.initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
        uint8_t* pd = ecrt_domain_data(nullptr);
        DeviceServo& servo = *bus.servo_devices()[0];

        EC_WRITE_U16(pd + servo_entry_offset(0, kStatuswordEntry), c.status);
        servo.read(pd);
        EXPECT_EQ(servo.status_recognized(), c.recognized);
        EXPECT_EQ(servo.get_servo_state(), plem::fieldbus::ServoState::NOT_READY_TO_SWITCH_ON)
            << "every case is still in NOT_READY — that is what makes them comparable";
        EXPECT_EQ(servo.transition_state(plem::fieldbus::ServoState::READY_TO_SWITCH_ON), c.accepted);
        if (c.accepted) {
            servo.write(pd);
            EXPECT_EQ(EC_READ_U16(pd + servo_entry_offset(0, kControlwordEntry)), 0x0006);
        }
    }
}

// Fault-reset pulse: device wiring on real domain memory
// The truth table of the decision itself lives in test_fault_reset_pulse.cpp.
// What these pin is the wiring: which state composes eligibility, that the
// request is acquired at the publication point, and how the pulse interacts
// with the standing command word. Every assertion reads the controlword out
// of domain memory at write time: the same bytes pdo_handler puts there.
class FaultResetPulseTest : public EtherCATBusFakeMasterTest {
  protected:
    void SetUp() override {
        EtherCATBusFakeMasterTest::SetUp();
        bus_ = std::make_unique<EtherCATBus>(0);
        bus_->set_servo_config(six_servos());
        const auto init = bus_->initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
        pd_ = ecrt_domain_data(nullptr);
        cw_off_ = servo_entry_offset(0, kControlwordEntry);
        sw_off_ = servo_entry_offset(0, kStatuswordEntry);
    }

    /// One cycle: present `status` to the drive, then publish. Returns the
    /// controlword that reached domain memory.
    uint16_t cycle(uint16_t status) {
        EC_WRITE_U16(pd_ + sw_off_, status);
        servo().read(pd_);
        servo().write(pd_);
        return EC_READ_U16(pd_ + cw_off_);
    }

    DeviceServo& servo() {
        return *bus_->servo_devices()[0];
    }

    /// A fresh bus whose servos declare `pulse_ms`, at `period_ns`.
    void rebuild(uint32_t pulse_ms, uint64_t period_ns) {
        auto infos = six_servos();
        for (auto& info : infos) {
            info.fault_reset_pulse_ms = pulse_ms;
        }
        bus_ = std::make_unique<EtherCATBus>(0);
        bus_->set_servo_config(infos);
        bus_->set_cycle_time_ns(period_ns);
        const auto init2 = bus_->initialize();
        ASSERT_TRUE(init2.is_ok()) << init2.message();
        pd_ = ecrt_domain_data(nullptr);
    }

    static constexpr uint16_t kFault = 0x0008;
    static constexpr uint16_t kFaultReactionActive = 0x000F;
    static constexpr uint16_t kSwitchOnDisabled = 0x0040;
    static constexpr uint16_t kUnrecognized = 0x0004;  // no case in to_servo_state

    std::unique_ptr<EtherCATBus> bus_;
    uint8_t* pd_ = nullptr;
    uint32_t cw_off_ = 0;
    uint32_t sw_off_ = 0;
};

// Transition 15 fires on the rising edge of bit 7, so back-to-back requests
// are worth nothing unless the wire goes low in between. A request posted
// after a publication (i.e. after that frame acquired one) is held exactly one
// frame (the frame that owes the falling edge) and goes out on the next.
// The declared width reaches the wire as whole frames at the bus period,
// rounded up, and 0 ms is one frame at any period (checked at 200 us and 1 ms
// below).
TEST_F(FaultResetPulseTest, TheDeclaredWidthIsHeldForItsFramesAtTheBusPeriod) {
    struct Case {
        uint32_t pulse_ms;
        uint64_t period_ns;
        size_t high;
    };
    for (const Case c : {Case{0, 200'000, 1}, Case{0, 1'000'000, 1}, Case{1, 1'000'000, 1}, Case{2, 200'000, 10},
                         Case{3, 2'000'000, 2}}) {
        SCOPED_TRACE(std::to_string(c.pulse_ms) + " ms at " + std::to_string(c.period_ns) + " ns");
        rebuild(c.pulse_ms, c.period_ns);
        servo().request_fault_reset();
        std::vector<uint16_t> wire;
        for (size_t i = 0; i < c.high + 2; ++i) {
            wire.push_back(cycle(kFault));
        }
        std::vector<uint16_t> expected(c.high, 0x0080);
        expected.insert(expected.end(), {0x0000, 0x0000});
        EXPECT_EQ(wire, expected);
    }
}

TEST_F(FaultResetPulseTest, AWidePulseEndsOnTheFrameTheAxisLeavesFault) {
    rebuild(20, 1'000'000);
    servo().request_fault_reset();
    EXPECT_EQ(cycle(kFault), 0x0080);
    EXPECT_EQ(cycle(kFault), 0x0080);
    EXPECT_EQ(cycle(kSwitchOnDisabled), 0x0000) << "no Fault reset command to a drive that left FAULT";
    EXPECT_EQ(cycle(kSwitchOnDisabled), 0x0000);
}

// The automatic entry spends an attempt only on a new pulse: while a pulse
// still holds bit 7 the request is refused, so pacing shorter than the width
// cannot burn the allowance inside one pulse. The frame owing only the falling
// edge accepts and holds the request over it: the one-frame behavior.
TEST_F(FaultResetPulseTest, AutomaticEntryRefusesWhileAPulseHoldsBitSeven) {
    rebuild(3, 1'000'000);
    EC_WRITE_U16(pd_ + sw_off_, kFault);
    servo().read(pd_);
    ASSERT_TRUE(servo().reset_fault());
    EXPECT_EQ(cycle(kFault), 0x0080);
    EXPECT_FALSE(servo().reset_fault()) << "two more high frames owed";
    EXPECT_EQ(cycle(kFault), 0x0080);
    EXPECT_FALSE(servo().reset_fault()) << "one more high frame owed";
    EXPECT_EQ(cycle(kFault), 0x0080);
    EXPECT_TRUE(servo().reset_fault()) << "only the falling edge is owed";
    EXPECT_EQ(cycle(kFault), 0x0000) << "the edge is paid first";
    EXPECT_EQ(cycle(kFault), 0x0080) << "then the accepted attempt is a new rising edge";
}

TEST_F(FaultResetPulseTest, AdjacentRequestsPulseHighLowHigh) {
    EXPECT_EQ(cycle(kFault), 0x0000) << "no request yet: the inactive baseline";

    EXPECT_TRUE(servo().reset_fault());
    EXPECT_EQ(cycle(kFault), 0x0080) << "rising edge";

    servo().request_fault_reset();  // posted after the previous frame published
    EXPECT_EQ(cycle(kFault), 0x0000) << "the falling edge is owed and always wins";
    EXPECT_EQ(cycle(kFault), 0x0080) << "the held request produces the SECOND rising edge";
    EXPECT_EQ(cycle(kFault), 0x0000) << "and is followed by its own falling edge";
}

// The automatic entry reads state, so it refuses what it must not spend an
// attempt on: FAULT_REACTION_ACTIVE reaches FAULT by itself (transition 14),
// and an unrecognized word leaves the state machine holding the old state.
TEST_F(FaultResetPulseTest, AutomaticEntryRefusesFaultReactionActiveAndUnrecognizedFrames) {
    EC_WRITE_U16(pd_ + sw_off_, kFaultReactionActive);
    servo().read(pd_);
    EXPECT_TRUE(servo().is_faulted()) << "is_faulted() is the WIDER predicate — it covers FRA";
    EXPECT_FALSE(servo().reset_fault()) << "eligibility is narrower: recognized FAULT only";

    EC_WRITE_U16(pd_ + sw_off_, kFault);
    servo().read(pd_);
    EC_WRITE_U16(pd_ + sw_off_, kUnrecognized);
    servo().read(pd_);
    EXPECT_TRUE(servo().is_faulted()) << "the FAULT is HELD across the unrecognized word";
    EXPECT_FALSE(servo().status_recognized());
    EXPECT_FALSE(servo().reset_fault()) << "held state is not live evidence — refusing here is what keeps "
                                           "the attempt allowance from burning down before the first pulse";
}

// The manual entry cannot read state (it runs off the cyclic thread), so it
// posts unconditionally and the publication point decides. An ineligible frame
// discards it; a request never waits for a better frame, which is what bounds
// its lifetime to two frames and removes any need for expiry bookkeeping.
TEST_F(FaultResetPulseTest, ManualRequestOnAnIneligibleFrameIsDiscardedNotQueued) {
    servo().request_fault_reset();
    EXPECT_EQ(cycle(kFaultReactionActive), 0x0000) << "FRA is recognized-but-not-FAULT: discarded";
    EXPECT_EQ(cycle(kFault), 0x0000) << "the discarded request must not resurface on the next FAULT";

    servo().request_fault_reset();
    EXPECT_EQ(cycle(kUnrecognized), 0x0000) << "unrecognized: discarded too";
    EXPECT_EQ(cycle(kFault), 0x0000);
}

// A request acquired before a recovery must not fire against the next
// fault. It holds because disposition is decided independently of whether a
// pulse is in flight: the recovery frame is a forced-low frame and an
// ineligible one, and ineligible wins for the request.
TEST_F(FaultResetPulseTest, RecoveredThenRefaultedAxisDoesNotRepublishThePreRecoveryRequest) {
    ASSERT_EQ(cycle(kFault), 0x0000) << "seed a recognized FAULT — eligibility is read, never assumed";
    EXPECT_TRUE(servo().reset_fault());
    EXPECT_EQ(cycle(kFault), 0x0080);

    servo().request_fault_reset();
    EXPECT_EQ(cycle(kSwitchOnDisabled), 0x0000) << "the drive recovered; this frame owes the falling edge";
    EXPECT_EQ(cycle(kFault), 0x0000) << "a NEW fault must need a NEW request";
}

// The shape above, with the drive reacting rather than recovered. FRA is
// recognized-but-not-FAULT, so it discards exactly like the SOD case; the
// two must not diverge, because a drive that reacts and re-faults is the more
// likely sequence in the field.
TEST_F(FaultResetPulseTest, ReactingThenRefaultedAxisDoesNotRepublishThePreReactionRequest) {
    ASSERT_EQ(cycle(kFault), 0x0000);
    EXPECT_TRUE(servo().reset_fault());
    EXPECT_EQ(cycle(kFault), 0x0080);

    servo().request_fault_reset();
    EXPECT_EQ(cycle(kFaultReactionActive), 0x0000) << "reacting; this frame owes the falling edge";
    EXPECT_EQ(cycle(kFault), 0x0000) << "the settled FAULT must need a NEW request";
}

// An accepted request is published against the next frame, not the one that
// accepted it. Accepted while a falling edge is owed it is held for that one
// frame, and if the frame after that is unrecognized it dies there, spending
// the attempt without ever reaching the wire.
TEST_F(FaultResetPulseTest, RequestHeldOverTheOwedLowIsDiscardedByTheNextUnrecognizedFrame) {
    ASSERT_EQ(cycle(kFault), 0x0000);
    ASSERT_TRUE(servo().reset_fault());
    ASSERT_EQ(cycle(kFault), 0x0080) << "first pulse; the next frame owes a low";

    EXPECT_TRUE(servo().reset_fault()) << "still a recognized FAULT, so the request is accepted";
    EXPECT_EQ(cycle(kFault), 0x0000) << "owed low on a RECOGNIZED FAULT — this is the KEPT frame";
    EXPECT_EQ(cycle(kUnrecognized), 0x0000) << "and the held request dies on the undecodable frame";
    EXPECT_EQ(cycle(kFault), 0x0000) << "no late high: the accepted request is gone";
}

// The forced-low frame overwrites the standing word, not just the output. A
// command generated during that cycle must not survive to fire a cycle later,
// after a forced engage or an early settle has ended the sequence that wanted
// it (the cancelled-Shutdown counterexample).
TEST_F(FaultResetPulseTest, ForcedLowFrameErasesACommandGeneratedInThatCycle) {
    ASSERT_EQ(cycle(kFault), 0x0000);
    EXPECT_TRUE(servo().reset_fault());
    EXPECT_EQ(cycle(kFault), 0x0080);

    EC_WRITE_U16(pd_ + sw_off_, kSwitchOnDisabled);
    servo().read(pd_);
    DeviceServoTestAccess::set_controlword(servo(), 0x0006);  // stand-in for the decider picking Shutdown this cycle
    servo().write(pd_);
    EXPECT_EQ(EC_READ_U16(pd_ + cw_off_), 0x0000) << "the owed falling edge outranks the new command";

    EXPECT_EQ(cycle(kSwitchOnDisabled), 0x0000) << "and the command is GONE, not deferred";
}

// The run boundary's primitive: a request posted while the controller was
// stopped must not fire on the next run's first recognized FAULT.
TEST_F(FaultResetPulseTest, DiscardDropsAPendingRequest) {
    servo().request_fault_reset();
    servo().discard_fault_reset_request();
    EXPECT_EQ(cycle(kFault), 0x0000);

    servo().request_fault_reset();
    EXPECT_EQ(cycle(kFault), 0x0080) << "and a request posted after the cutoff still works";
}

// The first frame after an unrecognized-only history publishes nothing: the
// automatic entry never accepted a request, so there is nothing to fire.
TEST_F(FaultResetPulseTest, AutomaticEntryRefusesBeforeAnyStatusHasBeenRead) {
    EXPECT_FALSE(servo().reset_fault()) << "a freshly constructed device has read no statusword";
    EXPECT_EQ(cycle(kFault), 0x0000);
}

// A publication that lands inside the acquisition window belongs to the next
// frame, never to this one: the acquisition is the boundary.
//
// What this proves and what it does not. It kills any implementation that
// clears the request after deciding the frame, the shape that silently eats a
// concurrent publication. It cannot kill an `exchange` replaced by an adjacent
// `load(); store(false);`, because nothing can be injected between two adjacent
// statements. Atomic acquisition is what `exchange` itself provides; this seam
// only detects a later destructive clear.
TEST_F(FaultResetPulseTest, PublicationInsideTheAcquisitionWindowSurvivesEveryDisposition) {
    struct Case {
        const char* name;
        uint16_t status;    // what the drive shows on the deciding frame
        bool seed_request;  // was a request already pending when the frame began?
        uint16_t expect;    // controlword this frame
    };
    // The three dispositions an acquired request can end in, plus the frame
    // that acquires nothing; a publication in the window must survive all.
    const Case cases[] = {
        {"CONSUMED", kFault, true, 0x0080},
        {"DISCARDED", kFaultReactionActive, true, 0x0000},
        {"NONE", kFault, false, 0x0000},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(c.name);
        SetUp();  // a fresh bus per case; the pulse state is per device
        DeviceServoTestAccess::set_post_acquire_hook(servo(), [this] { servo().request_fault_reset(); });
        if (c.seed_request) {
            servo().request_fault_reset();
        }
        EXPECT_EQ(cycle(c.status), c.expect) << "the in-window publication must not reach THIS frame";

        DeviceServoTestAccess::set_post_acquire_hook(servo(), nullptr);
        // It reaches a later one. After a published pulse the next frame owes
        // the falling edge, so the survivor lands one frame further out.
        if (c.expect == 0x0080) {
            EXPECT_EQ(cycle(kFault), 0x0000) << "owed falling edge first";
        }
        EXPECT_EQ(cycle(kFault), 0x0080) << "the survivor publishes";
    }
}

// KEPT is the fourth disposition: it is the one that stores `true` back, and
// storing it must merge with an in-window publication rather than replace it.
TEST_F(FaultResetPulseTest, PublicationInsideTheWindowOfAKeptFrameIsNotLost) {
    ASSERT_EQ(cycle(kFault), 0x0000);
    ASSERT_TRUE(servo().reset_fault());
    ASSERT_EQ(cycle(kFault), 0x0080);

    servo().request_fault_reset();  // will be KEPT by the owed-low frame
    DeviceServoTestAccess::set_post_acquire_hook(servo(), [this] { servo().request_fault_reset(); });
    EXPECT_EQ(cycle(kFault), 0x0000) << "the owed falling edge";
    DeviceServoTestAccess::set_post_acquire_hook(servo(), nullptr);
    EXPECT_EQ(cycle(kFault), 0x0080);
}

// Requests that arrive before the same acquisition are one request. The flag
// is a bool by design: N requests in a frame are N-to-1, and nothing promises
// N pulses.
TEST_F(FaultResetPulseTest, RequestsBeforeTheSameAcquisitionCoalesceIntoOnePulse) {
    ASSERT_EQ(cycle(kFault), 0x0000);
    servo().request_fault_reset();
    servo().request_fault_reset();
    servo().request_fault_reset();

    EXPECT_EQ(cycle(kFault), 0x0080);
    EXPECT_EQ(cycle(kFault), 0x0000) << "one pulse, not three";
    EXPECT_EQ(cycle(kFault), 0x0000);
}

// And a publication after the domain copy cannot retroactively change the
// bytes already written for this cycle.
TEST_F(FaultResetPulseTest, PublicationAfterTheDomainCopyLeavesThisFrameAlone) {
    ASSERT_EQ(cycle(kFault), 0x0000);
    EC_WRITE_U16(pd_ + sw_off_, kFault);
    servo().read(pd_);
    servo().write(pd_);
    ASSERT_EQ(EC_READ_U16(pd_ + cw_off_), 0x0000);

    servo().request_fault_reset();
    EXPECT_EQ(EC_READ_U16(pd_ + cw_off_), 0x0000) << "domain memory is not touched by a request";
    EXPECT_EQ(cycle(kFault), 0x0080) << "it takes the next write";
}

// The pulse word can never come from a profile row
// A row carrying bit 7 would reach the wire
// through the ordinary transition path, which writes levels and would repeat
// the word every cycle without ever producing a second edge.
TEST_F(EtherCATBusFakeMasterTest, ProfileRowCarryingFaultResetBitIsRejectedAtInitialize) {
    auto infos = six_servos();
    // Two shapes, because the check is on the bit, not on a word: a dedicated
    // reset row, and bit 7 smuggled into an ordinary transition's command
    // (0x0007 | 0x0080). A check written as "== 0x0080" or "only FAULT rows"
    // would let the second one through, and that one reaches the wire on
    // every ordinary walk.
    const std::pair<const char*, plem::fieldbus::Cia402Transition> rows[] = {
        {"dedicated reset row",
         {plem::fieldbus::ServoState::FAULT, plem::fieldbus::ServoState::SWITCH_ON_DISABLED, 0x0080}},
        {"bit 7 inside an ordinary command",
         {plem::fieldbus::ServoState::READY_TO_SWITCH_ON, plem::fieldbus::ServoState::SWITCHED_ON, 0x0087}},
    };
    for (const auto& [name, row] : rows) {
        SCOPED_TRACE(name);
        FakeEcrt::reset();
        auto profile = plem::fieldbus::kNotReadyShutdownCia402Profile;
        if (row.from == plem::fieldbus::ServoState::READY_TO_SWITCH_ON) {
            // Replace the existing row rather than duplicating (from, to);
            // the duplicate check would otherwise reject it first.
            for (size_t i = 0; i < profile.transition_count; ++i) {
                if (profile.transitions[i].from == row.from && profile.transitions[i].to == row.to) {
                    profile.transitions[i] = row;
                }
            }
        } else {
            profile.transitions[profile.transition_count++] = row;
        }
        infos[0].cia402_profile = profile;

        EtherCATBus bus(0);
        bus.set_servo_config(infos);
        EXPECT_FALSE(bus.initialize().is_ok()) << "a bit-7 row must fail initialize() at the configuration check";
        EXPECT_TRUE(FakeEcrt::calls().empty()) << "and fail fast BEFORE requesting the master";
    }
}
