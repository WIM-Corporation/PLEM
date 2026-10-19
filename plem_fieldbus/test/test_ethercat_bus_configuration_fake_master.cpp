// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_ethercat_bus_configuration_fake_master.cpp
/// @brief EtherCATBus bus configuration over the fake master: declared PDO
/// layouts (per-slave application, role registration, role-semantic validation at Step
/// 0), multi-axis slaves, the IO-device factory, retained layout storage,
/// servo-slave startup SDOs and the CiA 402 connection-fixture consumer half. Same
/// gtest target as test_ethercat_bus_fake_master.cpp, fixture in
/// fake_master_fixture.hpp.

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "fake_master_fixture.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

using plem::fieldbus::ethercat::EtherCATBus;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::test::count_calls;
using plem::fieldbus::test::FakeEcrt;
using plem::fieldbus::test::index_of;
using plem::fieldbus::test::le32;
using plem::fieldbus::test::log_contains;
using plem::fieldbus::test::one_io_info;
using plem::fieldbus::test::pdo_offset;
using plem::fieldbus::test::six_servos;

// PDO layout · multi-axis · optional PDO

#include <plem/core/monitoring/event.hpp>
#include <plem/core/monitoring/monitoring_context.hpp>

#include "plem/fieldbus/ethercat/pdo_layout.hpp"

namespace {

using plem::fieldbus::ethercat::PdoEntryMapping;
using plem::fieldbus::ethercat::PdoMapping;
using plem::fieldbus::ethercat::SlavePdoMapping;
using plem::fieldbus::ethercat::SyncManagerAssignment;

// A two-axis slave layout in the vendor shape: channel 2 objects at +0x800,
// no target velocity on either channel (0x60FF/0x68FF absent, the real
// vendor gap), one load-bearing padding entry (index 0x0000) in the TxPDO,
// and the ErrorCode objects mapped on both channels. Watchdog: outputs SM on,
// inputs SM off.
constexpr PdoEntryMapping kRxEntries[] = {
    {0x6040, 0, 16}, {0x6060, 0, 8}, {0x607A, 0, 32}, {0x6071, 0, 16},
    {0x6840, 0, 16}, {0x6860, 0, 8}, {0x687A, 0, 32}, {0x6871, 0, 16},
};
constexpr PdoMapping kRxPdos[] = {{0x1600, kRxEntries, 8}};
constexpr PdoEntryMapping kTxEntries[] = {
    {0x6041, 0, 16}, {0x6061, 0, 8}, {0x6064, 0, 32}, {0x606C, 0, 32}, {0x6077, 0, 16}, {0x603F, 0, 16},
    {0x0000, 0, 16},  // gap: load-bearing padding, occupies 16 bits of image
    {0x6841, 0, 16}, {0x6861, 0, 8}, {0x6864, 0, 32}, {0x686C, 0, 32}, {0x6877, 0, 16}, {0x683F, 0, 16},
};
constexpr PdoMapping kTxPdos[] = {{0x1A00, kTxEntries, 13}};
constexpr SyncManagerAssignment kTwoAxisSyncs[] = {
    {2, true, kRxPdos, 1, true},
    {3, false, kTxPdos, 1, false},
};
constexpr SlavePdoMapping kTwoAxisLayout{kTwoAxisSyncs, 2};

/// Six axes on three two-axis slaves (0,0,1,1,2,2 with channel offsets
/// 0/0x800), every slave declaring the same layout.
std::vector<ServoInfo> six_axes_on_three_slaves() {
    auto infos = six_servos();
    for (size_t i = 0; i < 6; ++i) {
        infos[i].slave_position = static_cast<int32_t>(i / 2);
        infos[i].od_offset = (i % 2) ? 0x800 : 0;
        infos[i].pdo_mapping = &kTwoAxisLayout;
    }
    return infos;
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, PositionBeforeEnableGatesOnTheAxisChannelPositionWord) {
    // Slave 0: channel 1 already reports its position, channel 2 (0x6864)
    // does not; only channel 2 reads its declared object (ServoInfo::
    // position_before_enable_index) and stands it in on 0x687A.
    auto infos = six_axes_on_three_slaves();
    infos[0].position_before_enable_index = 0x2300;
    infos[1].position_before_enable_index = 0x2301;
    FakeEcrt::set_sdo_upload_entry(0, 0x6064, 0, 0, le32(5000));
    FakeEcrt::set_sdo_upload_entry(0, 0x6864, 0, 0, le32(0));
    FakeEcrt::set_sdo_upload_entry(0, 0x2301, 0, 0, le32(690000));

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(index_of("master_sdo_upload(pos=0,idx=0x2300,sub=0x0)"), -1);

    uint8_t* pd = ecrt_domain_data(nullptr);
    auto& servo = *bus.servo_devices()[1];
    EC_WRITE_S32(pd + pdo_offset(0, 0x6864), 0);
    servo.read(pd);
    servo.write(pd);
    EXPECT_EQ(EC_READ_S32(pd + pdo_offset(0, 0x687A)), 690000);
}

TEST_F(EtherCATBusFakeMasterTest, AppliesDeclaredLayoutOncePerSlaveWithWatchdogShape) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_axes_on_three_slaves());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    // Once per slave: two axes share each slave, so exactly 3 applications
    // (per-device application would double them and each axis would overwrite
    // the other's mapping).
    EXPECT_EQ(count_calls("config_pdos(pos=0)"), 1);
    EXPECT_EQ(count_calls("config_pdos(pos=1)"), 1);
    EXPECT_EQ(count_calls("config_pdos(pos=2)"), 1);
    // Shape: outputs SM with watchdog on, inputs SM with watchdog off.
    EXPECT_TRUE(log_contains("config_pdos(pos=0)[sm2:out,wd_on,0x1600(8)][sm3:in,wd_off,0x1a00(13)]"));
}

// The declared six-servo path in IgH's order: every slave config and its PDO
// layout, then the entry registration (ten entries per servo), the send
// interval, and only then the activation (all of them "before
// ecrt_master_activate()", ecrt.h).
TEST_F(EtherCATBusFakeMasterTest, DeclaredSixServoPathConfiguresRegistersThenActivates) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto& calls = FakeEcrt::calls();
    int last_layout = -1;
    for (size_t i = 0; i < calls.size(); ++i) {
        if (calls[i].rfind("config_pdos(", 0) == 0) {
            last_layout = static_cast<int>(i);
        }
    }
    EXPECT_EQ(count_calls("slave_config(alias=0,"), 6);
    EXPECT_EQ(count_calls("config_pdos(pos="), 6);
    ASSERT_EQ(index_of("request_master(index=0)"), 0) << "the master first: the domain is created on it";
    ASSERT_GT(index_of("create_domain"), 0);
    EXPECT_LT(index_of("create_domain"), index_of("slave_config("));
    EXPECT_LT(last_layout, index_of("reg_entry(")) << "every layout is recorded before the first entry registers";
    EXPECT_TRUE(log_contains("reg_pdo_entry_list(entries=60)"));
    EXPECT_LT(index_of("reg_pdo_entry_list("), index_of("set_send_interval(us=1000)"));
    EXPECT_LT(index_of("set_send_interval(us=1000)"), index_of("master_activate"));
    EXPECT_EQ(calls.back(), "master_activate");
}

TEST_F(EtherCATBusFakeMasterTest, DeclaredSingleAxisLayoutRegistersTheStandardEntrySetInOrder) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());  // the single-axis CST layout on every slave
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    // One assignment call per slave; the declared mapping is written at PREOP.
    EXPECT_EQ(count_calls("config_pdos"), 6);
    // The layout maps exactly the standard entry set, so registration keeps
    // the ten standard object indices in the core's fixed order, pinned for
    // slave 0 (the fake records (position, index) only).
    const std::vector<uint16_t> expected = {0x6040, 0x6060, 0x607a, 0x60ff, 0x6071,
                                            0x6041, 0x6061, 0x6064, 0x606c, 0x6077};
    std::vector<std::string> registered;
    for (const auto& call : FakeEcrt::calls()) {
        if (call.rfind("reg_entry(pos=0,", 0) == 0) {
            registered.push_back(call);
        }
    }
    ASSERT_EQ(registered.size(), expected.size());
    for (size_t i = 0; i < expected.size(); ++i) {
        EXPECT_EQ(registered[i], fmt::format("reg_entry(pos=0,idx=0x{:x})", expected[i])) << "entry " << i;
    }
}

TEST_F(EtherCATBusFakeMasterTest, LayoutFilterSkipsUnmappedEntriesAndRegistersChannel2) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_axes_on_three_slaves());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    // The declared layout maps no target velocity on either channel, so the
    // entry is dropped before registration (registering it would fail the
    // whole list with an opaque ENOENT on hardware).
    EXPECT_EQ(count_calls("idx=0x60ff"), 0);
    EXPECT_EQ(count_calls("idx=0x68ff"), 0);
    // Channel 2 registers at base + 0x800 on the same slave.
    EXPECT_TRUE(log_contains("reg_entry(pos=0,idx=0x6040)"));
    EXPECT_TRUE(log_contains("reg_entry(pos=0,idx=0x6840)"));
    EXPECT_TRUE(log_contains("reg_entry(pos=2,idx=0x6877)"));
}

// The mapped execution path equals the function path
//
// A DeviceServo whose PDOs are really registered on the fake domain
// (six_servos(), ServoInfo default limits ±200 Nm): the clamp edge, the
// non-finite rejection with the last good command RE-EMITTED on the wire, and
// the read→getter decode as double bits. The expected
// values are the public conversion functions over the same channels; the
// mapped path must add nothing to them.
namespace mapped {
inline const plem::fieldbus::ethercat::AxisFactors& channels() {
    static const auto c = six_servos()[0].factors;
    return c;
}
inline int16_t torque_wire(double nm) {
    return plem::fieldbus::ethercat::to_torque_count(plem::core::units::NewtonMeter(nm), channels().torque)
        .to_hardware(false);
}
inline uint64_t torque_bits(int32_t count) {
    return std::bit_cast<uint64_t>(plem::fieldbus::ethercat::to_torque(
                                       plem::core::units::TorqueCount(static_cast<int16_t>(count)), channels().torque)
                                       .value());
}
inline uint64_t position_bits(int32_t count) {
    return std::bit_cast<uint64_t>(
        plem::fieldbus::ethercat::to_angle(plem::core::units::Pulse(count), channels().position).value());
}
inline uint64_t velocity_bits(int32_t count) {
    return std::bit_cast<uint64_t>(
        plem::fieldbus::ethercat::to_angular_velocity(plem::core::units::PulsePerSecond(count), channels().velocity)
            .value());
}
}  // namespace mapped

// Servo 0's PDO object indices.
constexpr unsigned int kTargetTorque = 0x6071;
constexpr unsigned int kActualPosition = 0x6064;
constexpr unsigned int kActualVelocity = 0x606C;
constexpr unsigned int kActualTorque = 0x6077;

TEST_F(EtherCATBusFakeMasterTest, ClampEdgeAndNonFiniteRejectionOnTheMappedWire) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    uint8_t* pd = ecrt_domain_data(nullptr);
    auto& servo = *bus.servo_devices()[0];
    const double kInf = std::numeric_limits<double>::infinity();

    // Exactly at the ±200 Nm limit: accepted, wire = the encode of ±200.
    ASSERT_TRUE(servo.set_command(plem::core::units::NewtonMeter(200.0)));
    servo.write(pd);
    EXPECT_EQ(EC_READ_S16(pd + pdo_offset(0, kTargetTorque)), mapped::torque_wire(200.0));
    ASSERT_TRUE(servo.set_command(plem::core::units::NewtonMeter(-200.0)));
    servo.write(pd);
    EXPECT_EQ(EC_READ_S16(pd + pdo_offset(0, kTargetTorque)), mapped::torque_wire(-200.0));

    // One ulp outside either limit: reported clamped, and the wire carries the
    // LIMIT's encode. One ulp inside: accepted, and its own encode.
    for (const double limit : {200.0, -200.0}) {
        const double outside = std::nextafter(limit, limit > 0 ? kInf : -kInf);
        const double inside = std::nextafter(limit, limit > 0 ? -kInf : kInf);
        EXPECT_FALSE(servo.set_command(plem::core::units::NewtonMeter(outside))) << limit;
        servo.write(pd);
        EXPECT_EQ(EC_READ_S16(pd + pdo_offset(0, kTargetTorque)), mapped::torque_wire(limit));
        EXPECT_TRUE(servo.set_command(plem::core::units::NewtonMeter(inside))) << limit;
        servo.write(pd);
        EXPECT_EQ(EC_READ_S16(pd + pdo_offset(0, kTargetTorque)), mapped::torque_wire(inside));
    }

    // Non-finite: rejected and the last good command is what reaches the wire.
    ASSERT_TRUE(servo.set_command(plem::core::units::NewtonMeter(10.0)));
    servo.write(pd);
    const int16_t good = EC_READ_S16(pd + pdo_offset(0, kTargetTorque));
    EXPECT_EQ(good, mapped::torque_wire(10.0));
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(), kInf, -kInf}) {
        EXPECT_FALSE(servo.set_command(plem::core::units::NewtonMeter(bad)));
        EC_WRITE_S16(pd + pdo_offset(0, kTargetTorque), 0);
        servo.write(pd);
        EXPECT_EQ(EC_READ_S16(pd + pdo_offset(0, kTargetTorque)), good)
            << "a rejected non-finite command must not reach the wire";
    }
}

TEST_F(EtherCATBusFakeMasterTest, MappedReadDecodesToTheFunctionDoubleBits) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    uint8_t* pd = ecrt_domain_data(nullptr);
    auto& servo = *bus.servo_devices()[0];  // flip false, offset 0 (six_servos)

    for (const int32_t count : {0, 1, -1, 1234, -1234, 32767, -32768}) {
        SCOPED_TRACE("torque count " + std::to_string(count));
        EC_WRITE_S16(pd + pdo_offset(0, kActualTorque), static_cast<int16_t>(count));
        servo.read(pd);
        EXPECT_EQ(std::bit_cast<uint64_t>(servo.get_actual_torque().value()), mapped::torque_bits(count));
    }
    for (const int32_t count : {0, 1, -1, 1000, -1000, 123456789, -123456789, std::numeric_limits<int32_t>::max(),
                                std::numeric_limits<int32_t>::min()}) {
        SCOPED_TRACE("position/velocity count " + std::to_string(count));
        EC_WRITE_S32(pd + pdo_offset(0, kActualPosition), count);
        EC_WRITE_S32(pd + pdo_offset(0, kActualVelocity), count);
        servo.read(pd);
        EXPECT_EQ(std::bit_cast<uint64_t>(servo.get_actual_position().value()), mapped::position_bits(count));
        EXPECT_EQ(std::bit_cast<uint64_t>(servo.get_actual_velocity().value()), mapped::velocity_bits(count));
    }
}

TEST_F(EtherCATBusFakeMasterTest, DeclaredErrorCodeRoleRegistersAndStoresSameFrameCode) {
    // The single-axis fixture layout plus the ErrorCode object: every standard
    // entry stays mapped, so the frame offsets below (10 standard entries +
    // the role appended last) hold.
    using plem::fieldbus::test::kSingleAxisCstRxPdos;
    static constexpr PdoEntryMapping kTxWithErr[] = {{0x6041, 0, 16}, {0x6064, 0, 32}, {0x606C, 0, 32},
                                                     {0x6077, 0, 16}, {0x6061, 0, 8},  {0x603F, 0, 16}};
    static constexpr PdoMapping kTxWithErrPdos[] = {{0x1A00, kTxWithErr, 6}};
    static constexpr SyncManagerAssignment kWithErrSyncs[] = {{2, true, kSingleAxisCstRxPdos, 1, true},
                                                              {3, false, kTxWithErrPdos, 1, false}};
    static constexpr SlavePdoMapping kWithErrLayout{kWithErrSyncs, 2};

    auto infos = six_servos();
    infos[0].pdo_mapping = &kWithErrLayout;
    infos[0].error_code_pdo = {0x603F, 0, 16};

    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_TRUE(log_contains("reg_entry(pos=0,idx=0x603f)"));

    uint8_t* pd = ecrt_domain_data(nullptr);
    EC_WRITE_U16(pd + pdo_offset(0, 0x6041), 0x0208);  // FAULT
    EC_WRITE_U16(pd + pdo_offset(0, 0x603F), 0x7500);  // the code, present only in this frame
    bus.servo_devices()[0]->read(pd);

    // Storage only: the device emits nothing (RtEventEmitter on the
    // controller fault path is the single SERVO_FAULT emitter and carries this
    // value with the joint id; a device-level emitter would report it twice).
    EXPECT_EQ(bus.servo_devices()[0]->error_code(), 0x7500);

    // A servo without the role keeps the field at 0 (its offset stays
    // unmapped, so nothing is read).
    EC_WRITE_U16(pd + pdo_offset(1, 0x6041), 0x0208);  // servo 1 status
    bus.servo_devices()[1]->read(pd);
    EXPECT_TRUE(bus.servo_devices()[1]->is_faulted()) << "servo 1 decoded its FAULT frame";
    EXPECT_EQ(bus.servo_devices()[1]->error_code(), 0x0000);
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsDuplicateSlaveChannelPair) {
    auto infos = six_axes_on_three_slaves();
    infos[1].od_offset = 0;  // now servos 0 and 1 both claim (slave 0, channel 1)
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsSharedSlaveIdentityMismatch) {
    auto infos = six_axes_on_three_slaves();
    infos[1].product_code = 0x99999999;  // same slave 0, different device type
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsSharedSlaveLayoutDisagreement) {
    static constexpr SlavePdoMapping kOtherLayout{kTwoAxisSyncs, 1};
    auto infos = six_axes_on_three_slaves();
    infos[1].pdo_mapping = &kOtherLayout;  // axis 2 of slave 0 disagrees
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok())
        << "a first-wins silent skip is exactly the failure mode this check exists to prevent";
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsSharedSlaveDcDisagreement) {
    // One slave, one ec_slave_config_t: its DC activation, SYNC0 cycle and
    // shift and the cycle-time write are configured once, so axes of one slave
    // disagreeing on any of them is refused, never resolved first-wins.
    const auto agreeing = [] {
        auto infos = six_axes_on_three_slaves();
        for (auto& info : infos) {
            info.dc_assign_activate = 0x0300;
            info.dc_sync0_cycle_ns = 1'000'000;
        }
        return infos;
    };
    const std::vector<std::pair<const char*, std::function<void(ServoInfo&)>>> edits = {
        {"dc_assign_activate", [](ServoInfo& i) { i.dc_assign_activate = 0x0700; }},
        {"dc_sync0_cycle_ns", [](ServoInfo& i) { i.dc_sync0_cycle_ns = 2'000'000; }},
        {"dc_sync0_shift_ns", [](ServoInfo& i) { i.dc_sync0_shift_ns = 1000; }},
        {"dc_write_cycle_time", [](ServoInfo& i) { i.dc_write_cycle_time = true; }},
    };
    for (const auto& [field, edit] : edits) {
        SCOPED_TRACE(field);
        FakeEcrt::reset();
        auto infos = agreeing();
        edit(infos[1]);  // axis 2 of slave 0
        EtherCATBus bus(0);
        bus.set_servo_config(infos);
        bus.set_cycle_time_ns(1'000'000);
        bus.set_dc_enable(true);
        EXPECT_FALSE(bus.initialize().is_ok());
        EXPECT_TRUE(FakeEcrt::calls().empty());
    }
    FakeEcrt::reset();
    EtherCATBus bus(0);
    bus.set_servo_config(agreeing());
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);
    const auto init = bus.initialize();
    EXPECT_TRUE(init.is_ok()) << init.message() << "axes that agree pass";
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsOffGridChannelOffset) {
    auto infos = six_servos();
    infos[0].od_offset = 0x400;  // not on the CiA 402 0x800 grid
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsAChannelPastTheEighth) {
    auto infos = six_servos();
    infos[0].od_offset = 0x4000;  // grid-aligned channel 9; the logical-device area holds 8
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsErrorCodeRoleMissingFromDeclaredLayout) {
    // A CST-complete layout (every role the RT loop reads/writes) that maps
    // no ErrorCode object, so the rejection below is about the declared
    // role alone, not about some other missing role.
    static constexpr PdoEntryMapping kNoErrTx[] = {
        {0x6041, 0, 16}, {0x6061, 0, 8}, {0x6064, 0, 32}, {0x606C, 0, 32}, {0x6077, 0, 16}};
    static constexpr PdoMapping kNoErrTxPdos[] = {{0x1A00, kNoErrTx, 5}};
    static constexpr PdoEntryMapping kNoErrRx[] = {{0x6040, 0, 16}, {0x6060, 0, 8}, {0x607A, 0, 32}, {0x6071, 0, 16}};
    static constexpr PdoMapping kNoErrRxPdos[] = {{0x1600, kNoErrRx, 4}};
    static constexpr SyncManagerAssignment kNoErrSyncs[] = {{2, true, kNoErrRxPdos, 1, true},
                                                            {3, false, kNoErrTxPdos, 1, false}};
    static constexpr SlavePdoMapping kNoErrLayout{kNoErrSyncs, 2};

    auto infos = six_servos();
    infos[0].pdo_mapping = &kNoErrLayout;
    infos[0].error_code_pdo = {0x603F, 0, 16};  // declared, but the layout does not map it
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok())
        << "a declared-but-unmapped role would ENOENT the whole registration list on hardware";
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsErrorCodeRoleSubindexMismatchWithLayout) {
    // The layout maps 0x603F:00/16, so a role declaring subindex 1 has no exact
    // descriptor match and must be rejected before hardware (an index-only
    // check would pass it and the mismatch would only surface as a misread).
    auto infos = six_servos();
    infos[0].pdo_mapping = &kTwoAxisLayout;
    infos[0].error_code_pdo = {0x603F, 1, 16};
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsErrorCodeRoleWithUnsupportedBitLength) {
    // The decode is fixed at 16 bits (EC_READ_U16); any other declared width
    // is rejected at validation, before the layout gate looks at the role.
    auto infos = six_servos();
    infos[0].error_code_pdo = {0x603F, 0, 32};
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

// Semantic validation of a declared layout (pdo_layout_validation)
// The rules themselves are pinned in test_pdo_layout_validation.cpp; they
// are not repeated here. Only the wiring, observed through its effects: a
// defective declared layout is refused before any ecrt call (one
// representative), and the axes of one slave are presented together and each at
// its own channel.

namespace {

// kTwoAxisLayout minus the velocity feedback (0x606C/0x686C) on both channels.
constexpr PdoEntryMapping kNoVelTxEntries[] = {
    {0x6041, 0, 16}, {0x6061, 0, 8}, {0x6064, 0, 32}, {0x6077, 0, 16}, {0x603F, 0, 16}, {0x0000, 0, 16},
    {0x6841, 0, 16}, {0x6861, 0, 8}, {0x6864, 0, 32}, {0x6877, 0, 16}, {0x683F, 0, 16},
};
constexpr PdoMapping kNoVelTxPdos[] = {{0x1A00, kNoVelTxEntries, 11}};
constexpr SyncManagerAssignment kNoVelSyncs[] = {{2, true, kRxPdos, 1, true}, {3, false, kNoVelTxPdos, 1, false}};
constexpr SlavePdoMapping kNoVelLayout{kNoVelSyncs, 2};

// One complete CST channel (channel 1 only), ErrorCode mapped.
constexpr PdoEntryMapping kOneChRx[] = {{0x6040, 0, 16}, {0x6060, 0, 8}, {0x607A, 0, 32}, {0x6071, 0, 16}};
constexpr PdoMapping kOneChRxPdos[] = {{0x1600, kOneChRx, 4}};
constexpr PdoEntryMapping kOneChTx[] = {{0x6041, 0, 16}, {0x6061, 0, 8},  {0x6064, 0, 32},
                                        {0x606C, 0, 32}, {0x6077, 0, 16}, {0x603F, 0, 16}};
constexpr PdoMapping kOneChTxPdos[] = {{0x1A00, kOneChTx, 6}};
constexpr SyncManagerAssignment kOneChSyncs[] = {{2, true, kOneChRxPdos, 1, true}, {3, false, kOneChTxPdos, 1, false}};
constexpr SlavePdoMapping kOneChannelLayout{kOneChSyncs, 2};

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, DeclaredCompleteLayoutInitializesAndRegistersItsRoles) {
    // Positive control for the wiring: a complete single-channel declaration
    // passes Step 0 and the bus goes on to apply and register it.
    auto infos = six_servos();
    for (auto& info : infos) {
        info.pdo_mapping = &kOneChannelLayout;
        info.error_code_pdo = {0x603F, 0, 16};
    }
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("config_pdos"), 6);
    EXPECT_TRUE(log_contains("reg_entry(pos=3,idx=0x603f)"));
    EXPECT_EQ(count_calls("idx=0x60ff"), 0);  // optional, unmapped → filtered, not fatal
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRejectsDeclaredLayoutMissingRequiredFeedback) {
    // An unmapped velocity feedback would register nothing and leave the field
    // frozen at its previous value, so the limit checks would go blind.
    auto infos = six_axes_on_three_slaves();
    for (auto& info : infos) {
        info.pdo_mapping = &kNoVelLayout;
    }
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, SharedSlaveAxesArePresentedToOneValidationTogether) {
    // Each axis of slave 0 is valid on its own: axis 1 (offset 0) declares
    // ErrorCode base 0x2801, axis 2 (offset 0x800) declares base 0x2001, and
    // the layout maps 0x2801:00/16 exactly once. Only when the bus hands both
    // axes' requirements to one validation do they resolve to the same entry
    // (a per-axis wiring would accept this).
    static constexpr PdoEntryMapping kSharedEcTx[] = {
        {0x6041, 0, 16}, {0x6061, 0, 8}, {0x6064, 0, 32}, {0x606C, 0, 32}, {0x6077, 0, 16}, {0x2801, 0, 16},
        {0x6841, 0, 16}, {0x6861, 0, 8}, {0x6864, 0, 32}, {0x686C, 0, 32}, {0x6877, 0, 16},
    };
    static constexpr PdoMapping kSharedEcTxPdos[] = {{0x1A00, kSharedEcTx, 11}};
    static constexpr SyncManagerAssignment kSharedEcSyncs[] = {{2, true, kRxPdos, 1, true},
                                                               {3, false, kSharedEcTxPdos, 1, false}};
    static constexpr SlavePdoMapping kSharedEcLayout{kSharedEcSyncs, 2};
    auto infos = six_axes_on_three_slaves();
    for (auto& info : infos) {
        info.pdo_mapping = &kSharedEcLayout;
    }
    infos[0].error_code_pdo = {0x2801, 0, 16};
    infos[1].error_code_pdo = {0x2001, 0, 16};
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, SharedSlaveAxesAreValidatedAtTheirOwnChannels) {
    // Both axes of a slave present their requirements to one validation, each
    // at its own od_offset: a layout that maps channel 1 only cannot host a
    // second axis at 0x800 (its 0x68xx roles are missing), even though the
    // first axis alone would pass.
    auto infos = six_servos();
    infos[0].slave_position = 0;
    infos[1].slave_position = 0;
    infos[1].od_offset = 0x800;
    for (auto& info : infos) {
        info.pdo_mapping = &kOneChannelLayout;
    }
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

// IO devices: the injected factory is the only creation path

namespace {

/// Minimal IO device: registers one PDO entry, counts cyclic read/write
/// participation (atomics: the RT-loop pin reads them cross-thread).
struct FakeIoDevice : plem::fieldbus::ethercat::DeviceBase {
    FakeIoDevice(ec_master_t* master, uint32_t slave_position, const plem::fieldbus::ethercat::IOInfo& info)
        : DeviceBase(master, slave_position, info.vendor_id, info.product_code) {}

    plem::core::Result<void> register_PDOs(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index) override {
        return register_PDO_entries(domain_regs, domain_index, {{0x7000, 0, &pdo_offset_}});
    }
    void read(const uint8_t* /*domain_pd*/) noexcept override {
        reads.fetch_add(1, std::memory_order_relaxed);
    }
    void write(uint8_t* /*domain_pd*/) noexcept override {
        writes.fetch_add(1, std::memory_order_relaxed);
    }

    uint32_t pdo_offset_ = 0;
    std::atomic<uint32_t> reads{0};
    std::atomic<uint32_t> writes{0};
};

std::shared_ptr<plem::fieldbus::ethercat::DeviceBase> make_fake_io(
    const plem::fieldbus::ethercat::IODeviceContext& ctx) {
    return std::make_shared<FakeIoDevice>(ctx.master, ctx.slave_position, ctx.io_info);
}

plem::fieldbus::ethercat::IOInfo second_io_info() {
    auto info = one_io_info();
    info.product_code = 0x50000000;
    return info;
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, IoSlavesFollowTheLastServoSlaveNotTheServoCount) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_axes_on_three_slaves());  // slaves 0..2 (6 axes)
    bus.set_io_config({one_io_info()});
    bus.set_io_device_factory(make_fake_io);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    // max servo slave is 2, so the IO sits at 3; an index-based base
    // would put it at 6 (a hole of ghost slaves).
    EXPECT_TRUE(log_contains("slave_config(alias=0,pos=3,vendor=0xc0ffee,product=0x40000000)"));
}

TEST_F(EtherCATBusFakeMasterTest, InitializeDispatchesIoFactoryOncePerIo) {
    // The factory receives exactly the creation contract (the bus's live
    // master handle + the IO's ring position + its IOInfo) once per IO,
    // and the devices land in io_devices() in IOInfo order.
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_io_config({one_io_info(), second_io_info()});

    int calls = 0;
    ec_master_t* seen_master = nullptr;
    uint32_t seen_slave = 0;
    uint32_t seen_product = 0;
    bus.set_io_device_factory([&](const plem::fieldbus::ethercat::IODeviceContext& ctx) {
        ++calls;
        seen_master = ctx.master;
        seen_slave = ctx.slave_position;
        seen_product = ctx.io_info.product_code;
        return make_fake_io(ctx);
    });

    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(seen_master, FakeEcrt::master());
    EXPECT_EQ(seen_slave, 7u);  // io_base 6 (after six one-axis servos) + index 1
    EXPECT_EQ(seen_product, 0x50000000u);
    ASSERT_EQ(bus.io_devices().size(), 2u);
    EXPECT_EQ(bus.io_devices()[0]->position(), 6u);
    EXPECT_EQ(bus.io_devices()[1]->position(), 7u);
    EXPECT_TRUE(log_contains("slave_config(alias=0,pos=7,vendor=0xc0ffee,product=0x50000000)"));
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRefusesIoWithoutFactory) {
    // No silent default device: an IO slave without a factory fails
    // initialization and releases the master.
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_io_config({one_io_info()});
    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("1 IO slave(s) declared but no IO device factory is injected"), std::string::npos)
        << refused.message();
    EXPECT_TRUE(bus.io_devices().empty());
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, InitializeRefusesIoFactoryWithoutIo) {
    // Mirror direction: a factory that can never fire means the IO
    // declaration was dropped, refused instead of an inert no-op.
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    int calls = 0;
    bus.set_io_device_factory([&](const plem::fieldbus::ethercat::IODeviceContext& ctx) {
        ++calls;
        return make_fake_io(ctx);
    });
    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("an IO device factory is injected but no IO slave is declared"), std::string::npos)
        << refused.message();
    EXPECT_EQ(calls, 0);
    EXPECT_TRUE(bus.io_devices().empty());
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, ThrowingIoFactoryFailsInitializeAndReleasesMaster) {
    // The factory is user code: a throw there must fail initialize
    // like every other failure and release the requested master; the
    // IgH reservation otherwise blocks the interface until process exit.
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_io_config({one_io_info()});
    bus.set_io_device_factory(
        [](const plem::fieldbus::ethercat::IODeviceContext&) -> std::shared_ptr<plem::fieldbus::ethercat::DeviceBase> {
            throw std::runtime_error("vendor factory exploded");
        });

    const auto refused = bus.initialize();
    ASSERT_FALSE(refused);
    EXPECT_NE(refused.message().find("vendor factory exploded"), std::string::npos)
        << "the factory's reason reaches the operator: " << refused.message();
    EXPECT_TRUE(bus.io_devices().empty());
    EXPECT_TRUE(log_contains("release_master")) << "a throwing IO factory must not leak the requested master";
}

TEST_F(EtherCATBusFakeMasterTest, NonStandardThrowingIoFactoryAlsoFailsAndReleasesMaster) {
    // The factory type carries no std::exception-only contract: a
    // non-standard throw (throw 42) must refuse and release the
    // master exactly like the standard one.
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_io_config({one_io_info()});
    bus.set_io_device_factory(
        [](const plem::fieldbus::ethercat::IODeviceContext&) -> std::shared_ptr<plem::fieldbus::ethercat::DeviceBase> {
            throw 42;  // deliberately not std::exception-derived
        });

    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("IO device factory threw a NON-STANDARD exception for IO 0 (slave 6"),
              std::string::npos)
        << refused.message();
    EXPECT_TRUE(bus.io_devices().empty());
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, InitializeFailsFastWhenIoFactoryReturnsNull) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_io_config({one_io_info()});
    bus.set_io_device_factory([](const plem::fieldbus::ethercat::IODeviceContext&) {
        return std::shared_ptr<plem::fieldbus::ethercat::DeviceBase>{};
    });
    const auto refused = bus.initialize();
    EXPECT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("IO device factory returned null for IO 0 (slave 6"), std::string::npos)
        << refused.message();
    EXPECT_TRUE(bus.io_devices().empty());
    EXPECT_TRUE(log_contains("release_master"));
}

TEST_F(EtherCATBusFakeMasterTest, InitializeReturnsErrorWhenIoSlaveConfigFails) {
    // An IO the master cannot configure: its constructor throws inside the
    // factory after the servos and two IOs registered. initialize() refuses,
    // releases the master and keeps no device.
    FakeEcrt::set_slave_config_failure(8);  // the last IO (io_base 6 + index 2)
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_io_config({one_io_info(), second_io_info(), one_io_info()});
    bus.set_io_device_factory(make_fake_io);

    plem::core::Result<void> refused = plem::core::Result<void>::ok();
    EXPECT_NO_THROW(refused = bus.initialize());
    EXPECT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("IO device factory threw for IO 2 (slave 8"), std::string::npos)
        << refused.message();
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_TRUE(bus.servo_devices().empty());
    EXPECT_TRUE(bus.io_devices().empty());
}

TEST_F(EtherCATBusFakeMasterTest, ConfigurePdosTraversesIoDevices) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());  // slaves 0..5, so the IO sits at 6
    bus.set_io_config({second_io_info()});
    bus.set_io_device_factory(make_fake_io);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    // The device's PDO entry joined the same domain registration list
    // through the DeviceBase virtual (no separate hook, no second domain).
    EXPECT_TRUE(log_contains("reg_entry(pos=6,idx=0x7000)"));
}

TEST_F(EtherCATBusFakeMasterTest, RetainedLayoutStorageLivesExactlyAsLongAsTheBus) {
    // The declared-PDO-mapping backing storage is retained by the bus
    // (callers hold shared_ptr<ICiA402Bus> beyond the robot), released only
    // at bus destruction, after the devices.
    auto storage = std::make_shared<const int>(42);
    std::weak_ptr<const int> observer = storage;
    {
        EtherCATBus bus(0);
        bus.retain_pdo_mapping_storage(storage);
        storage.reset();  // the caller's own reference is gone
        EXPECT_FALSE(observer.expired()) << "the bus must keep the storage alive";
    }
    EXPECT_TRUE(observer.expired()) << "bus destruction releases the retained storage";
}

// Connection contract: consumer half
// The producer half (whatever assembly layer builds the bus from model
// constants → bus injection) pins that the assembly hands the bus exactly the connection fixture's
// ServoInfo/IOInfo (torque limits excepted: URDF-owned, compared there).
// This half feeds that same fixture into initialize() and pins the ecrt
// effects: registration addresses and order, one layout application per
// physical slave, the IO slave after the highest servo slave, the factory
// call, and the per-axis DC sequence with the reference clock on joint 0's
// slave. plem/fieldbus/test/cia402_connection_fixture.hpp is the single
// value crossing the two tests.

#include <plem/fieldbus/test/cia402_connection_fixture.hpp>

namespace {

namespace connection = plem::fieldbus::test::connection;

std::vector<std::string> calls_starting_with(const std::string& prefix) {
    std::vector<std::string> out;
    for (const auto& call : FakeEcrt::calls()) {
        if (call.rfind(prefix, 0) == 0) {
            out.push_back(call);
        }
    }
    return out;
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, ConnectionFixtureAppliesEachSlaveOnceAndRegistersEveryRoleAtItsChannel) {
    EtherCATBus bus(0);
    bus.set_servo_config(connection::servo_infos());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    // One layout application per physical slave, in first-appearance order
    // (joint 0 → slave 5, joint 1 → slave 0, joint 2 → slave 2), each with the
    // full four-SM shape (mailbox pair + outputs with watchdog + inputs).
    const std::string shape =
        "[sm0:out,wd_off][sm1:in,wd_off][sm2:out,wd_on,0x1600(5),0x1601(5)][sm3:in,wd_off,0x1a00(7),0x1a01(7)]";
    EXPECT_EQ(calls_starting_with("config_pdos"),
              (std::vector<std::string>{"config_pdos(pos=5)" + shape, "config_pdos(pos=0)" + shape,
                                        "config_pdos(pos=2)" + shape}));

    // Slave 5 hosts joint 0 (channel 1) and joint 3 (channel 2): joint 0's
    // roles register first, then joint 3's at base + 0x800; target velocity
    // (optional, unmapped) is filtered; the ErrorCode role closes each list.
    std::vector<std::string> expected;
    for (const uint16_t channel : {0x0000, 0x0800}) {
        for (const uint16_t base : {0x6040, 0x6060, 0x607A, 0x6071, 0x6041, 0x6061, 0x6064, 0x606C, 0x6077, 0x603F}) {
            expected.push_back(fmt::format("reg_entry(pos=5,idx=0x{:x})", base + channel));
        }
    }
    EXPECT_EQ(calls_starting_with("reg_entry(pos=5,"), expected);
    EXPECT_EQ(count_calls("idx=0x60ff"), 0);
    EXPECT_EQ(count_calls("idx=0x68ff"), 0);
}

TEST_F(EtherCATBusFakeMasterTest, ConnectionFixturePlacesIoAfterTheHighestSlaveAndClocksEveryAxisInJointOrder) {
    EtherCATBus bus(0);
    bus.set_servo_config(connection::servo_infos());
    bus.set_io_config({connection::io_info()});
    int factory_calls = 0;
    uint32_t seen_slave = 0;
    bus.set_io_device_factory([&](const plem::fieldbus::ethercat::IODeviceContext& ctx) {
        ++factory_calls;
        seen_slave = ctx.slave_position;
        return std::make_shared<FakeIoDevice>(ctx.master, ctx.slave_position, ctx.io_info);
    });
    bus.set_cycle_time_ns(1'000'000);
    bus.set_dc_enable(true);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();

    // The IO slave follows the highest servo slave (5 → 6), served by the
    // factory exactly once.
    EXPECT_EQ(factory_calls, 1);
    EXPECT_EQ(seen_slave, connection::kIoSlavePosition);
    EXPECT_TRUE(
        log_contains(fmt::format("slave_config(alias=0,pos={},vendor=0x{:x},product=0x{:x})",
                                 connection::kIoSlavePosition, connection::kIoVendorId, connection::kIoProductCode)));

    // DC: configuration per axis in joint order, nothing read from the drive. A shared
    // slave is clocked once per axis it hosts (the once-per-slave form is
    // the pacing model's), and the reference clock is the lowest ring position, 0, not joint 0's slave 5.
    std::vector<std::string> expected;
    for (const int32_t pos : connection::kSlavePositions) {
        expected.push_back(
            fmt::format("slave_config_dc(pos={},aa=0x{:x},cycle={},shift={},sync1_cycle=0,sync1_shift=0)", pos,
                        connection::kDcAssignActivate, connection::kDcSync0CycleNs, connection::kDcSync0ShiftNs));
    }
    expected.push_back(fmt::format("select_reference_clock(pos={})", connection::kDcReferenceSlave));
    std::vector<std::string> dc_calls;
    for (const auto& call : FakeEcrt::calls()) {
        if (call.rfind("master_sdo_upload", 0) == 0 || call.rfind("slave_config_dc", 0) == 0 ||
            call.rfind("select_reference_clock", 0) == 0) {
            dc_calls.push_back(call);
        }
    }
    EXPECT_EQ(dc_calls, expected);
    EXPECT_LT(index_of("select_reference_clock"), index_of("master_activate"));
}

TEST_F(EtherCATBusFakeMasterTest, EachSlaveIsConfiguredWithItsOwnDeclaredLayout) {
    // Two different declared layouts on one bus: kTwoAxisLayout on slaves 0
    // and 1, the same entries with the TxPDO split in two on slave 2: each
    // slave is applied with its shape (a first-wins global layout would
    // stamp slave 0's shape on slave 2).
    static constexpr PdoMapping kSplitTxPdos[] = {{0x1A00, kTxEntries, 7}, {0x1A01, kTxEntries + 7, 6}};
    static constexpr SyncManagerAssignment kSplitSyncs[] = {{2, true, kRxPdos, 1, true},
                                                            {3, false, kSplitTxPdos, 2, false}};
    static constexpr SlavePdoMapping kSplitLayout{kSplitSyncs, 2};
    auto infos = six_axes_on_three_slaves();
    infos[4].pdo_mapping = &kSplitLayout;
    infos[5].pdo_mapping = &kSplitLayout;
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(
        calls_starting_with("config_pdos"),
        (std::vector<std::string>{"config_pdos(pos=0)[sm2:out,wd_on,0x1600(8)][sm3:in,wd_off,0x1a00(13)]",
                                  "config_pdos(pos=1)[sm2:out,wd_on,0x1600(8)][sm3:in,wd_off,0x1a00(13)]",
                                  "config_pdos(pos=2)[sm2:out,wd_on,0x1600(8)][sm3:in,wd_off,0x1a00(7),0x1a01(6)]"}));
}

TEST_F(EtherCATBusFakeMasterTest, DerivedAndExplicitSlavePositionsResolveToOneSlave) {
    // Joint 0 declares -1 (derive from its index: 0), joint 1 declares 0
    // explicitly, so both name one physical slave: the layout is applied once and both
    // channels register at position 0.
    auto infos = six_axes_on_three_slaves();
    infos[0].slave_position = -1;
    EtherCATBus bus(0);
    bus.set_servo_config(infos);
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("config_pdos(pos=0)"), 1);
    EXPECT_EQ(count_calls("config_pdos"), 3);
    EXPECT_TRUE(log_contains("reg_entry(pos=0,idx=0x6040)"));
    EXPECT_TRUE(log_contains("reg_entry(pos=0,idx=0x6840)"));
}

// Servo-slave startup SDO writes (drive-model schema §4.7)

TEST(StartupSdo, AcceptsExactlyTheWidthsTheMasterReserves) {
    using plem::fieldbus::ethercat::StartupSdo;
    EXPECT_TRUE((StartupSdo{0x2000, 0, 1, 0xFF}.is_valid()));
    EXPECT_TRUE((StartupSdo{0x2000, 0, 2, 0xFFFF}.is_valid()));
    EXPECT_TRUE((StartupSdo{0x2000, 0, 4, 0xFFFFFFFF}.is_valid()));
    EXPECT_FALSE((StartupSdo{0x2000, 0, 1, 0x100}.is_valid())) << "never truncated";
    EXPECT_FALSE((StartupSdo{0x2000, 0, 2, 0x10000}.is_valid()));
    EXPECT_FALSE((StartupSdo{0x2000, 0, 3, 0}.is_valid()));
    EXPECT_FALSE((StartupSdo{0x2000, 0, 0, 0}.is_valid()));
}

TEST_F(EtherCATBusFakeMasterTest, ReservesStartupSdosInListOrderBeforeActivation) {
    using plem::fieldbus::ethercat::SlaveSdos;
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    // The same address twice: order and repetition are data (IO board precedent).
    bus.set_startup_sdos({SlaveSdos{.slave_position = 2,
                                    .sdos = {{0x2000, 0x01, 1, 0xAA},
                                             {0x2001, 0x00, 2, 0xBEEF},
                                             {0x2002, 0x03, 4, 0xDEADBEEF},
                                             {0x2000, 0x01, 1, 0x03}}}});
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    const int first = index_of("slave_config_sdo8(pos=2,idx=0x2000,sub=0x1,val=0xaa)");
    const int second = index_of("slave_config_sdo16(pos=2,idx=0x2001,sub=0x0,val=0xbeef)");
    const int third = index_of("slave_config_sdo32(pos=2,idx=0x2002,sub=0x3,val=0xdeadbeef)");
    const int fourth = index_of("slave_config_sdo8(pos=2,idx=0x2000,sub=0x1,val=0x3)");
    ASSERT_GE(first, 0);
    EXPECT_LT(first, second);
    EXPECT_LT(second, third);
    EXPECT_LT(third, fourth);
    EXPECT_LT(fourth, index_of("master_activate")) << "reserved before activation";
    EXPECT_EQ(count_calls("slave_config_sdo"), 4) << "the declared slave's writes and nothing else";
}

TEST_F(EtherCATBusFakeMasterTest, RefusesStartupSdosForASlaveWithoutAServoBeforeAnyEcrtCall) {
    using plem::fieldbus::ethercat::SlaveSdos;
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());  // slaves 0..5
    bus.set_startup_sdos({SlaveSdos{.slave_position = 6, .sdos = {{0x2000, 0x01, 1, 0xAA}}}});
    const auto init = bus.initialize();
    ASSERT_FALSE(init.is_ok());
    EXPECT_NE(init.message().find("slave 6"), std::string::npos) << init.message();
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

TEST_F(EtherCATBusFakeMasterTest, RefusesAStartupSdoTheMasterCannotReserveBeforeAnyEcrtCall) {
    using plem::fieldbus::ethercat::SlaveSdos;
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_startup_sdos({SlaveSdos{.slave_position = 0, .sdos = {{0x2000, 0x00, 1, 0x100}}}});
    EXPECT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

// Expected slave identity (drive-model schema §6)

namespace {

using plem::fieldbus::ethercat::SlaveExpectedIdentity;
using plem::fieldbus::ethercat::SlaveIdentity;

/// A synthetic drive's identity: serial 0, 0x1009 "1.3", 0x100A "2.07".
SlaveIdentity synthetic_identity() {
    SlaveIdentity id;
    id.serial_number = 0;
    id.hardware_version = std::array<char, 32>{'1', '.', '3'};
    id.software_version = std::array<char, 32>{'2', '.', '0', '7'};
    return id;
}

void fake_synthetic_drive() {
    FakeEcrt::set_sdo_upload_string(0x1009, "1.3");
    FakeEcrt::set_sdo_upload_string(0x100A, "2.07");
}

plem::core::Result<void> initialize_with(const SlaveIdentity& identity, uint32_t position = 2) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_expected_slave_identity({{.slave_position = position, .identity = identity}});
    return bus.initialize();
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, VerifiesEveryDeclaredIdentityValueAfterTheMasterAndBeforeActivation) {
    fake_synthetic_drive();
    const auto init = initialize_with(synthetic_identity());
    ASSERT_TRUE(init.is_ok()) << init.message();
    const int hw = index_of("master_sdo_upload(pos=2,idx=0x1009,sub=0x0)");
    const int sw = index_of("master_sdo_upload(pos=2,idx=0x100a,sub=0x0)");
    ASSERT_GE(hw, 0);
    ASSERT_GE(sw, 0);
    EXPECT_GT(hw, index_of("request_master"));
    EXPECT_LT(sw, index_of("master_activate")) << "read before activation";
    EXPECT_EQ(count_calls("idx=0x1009"), 1) << "the declared slave only";
    EXPECT_EQ(count_calls("idx=0x100a"), 1);
    EXPECT_FALSE(log_contains("slave_config_dc")) << "six_servos() declare no DC — the check is not DC's";
}

TEST_F(EtherCATBusFakeMasterTest, AnUndeclaredIdentityValueIsNeverRead) {
    fake_synthetic_drive();
    const auto init = initialize_with(SlaveIdentity{});
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("master_sdo_upload"), 0);
    FakeEcrt::reset();
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto init2 = bus.initialize();
    ASSERT_TRUE(init2.is_ok()) << init2.message();
    EXPECT_EQ(count_calls("master_sdo_upload"), 0) << "no setter call, no read";
}

// Probe: the start-up check of the drives, without the start
// `plem drive run` probes an approved file's drives before it starts the bus:
// the same Step 0–3.5 check, then the master is let go; nothing is
// configured, written or activated, so nothing can move.

namespace {

SlaveIdentity serial_only(uint32_t serial) {
    SlaveIdentity id;
    id.serial_number = serial;
    return id;
}

plem::core::Result<void> probe_with(EtherCATBus& bus, std::vector<SlaveExpectedIdentity> expected) {
    bus.set_servo_config(six_servos());
    bus.set_expected_slave_identity(std::move(expected));
    bus.set_probe(true);
    return bus.initialize();
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, AProbeStopsAfterTheIdentityCheckAndReleasesTheMaster) {
    FakeEcrt::set_slave_serial(2, 7);
    EtherCATBus bus(0);
    const auto probed = probe_with(bus, {{.slave_position = 2, .identity = serial_only(7)}});
    ASSERT_TRUE(probed.is_ok()) << probed.message();
    EXPECT_TRUE(log_contains("release_master"));
    for (const char* never : {"reg_pdo_entry", "slave_config_sdo", "slave_config_dc", "master_activate"}) {
        EXPECT_FALSE(log_contains(never)) << never;
    }
    EXPECT_FALSE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, AnIdentityValueThatDiffersIsRebenchable) {
    FakeEcrt::set_slave_serial(2, 8);
    EtherCATBus bus(0);
    const auto probed = probe_with(bus, {{.slave_position = 2, .identity = serial_only(7)}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find("slave 2: SII serial number is 8 but the approval record declares 7"),
              std::string::npos)
        << probed.message();
    EXPECT_TRUE(bus.identity_rebenchable()) << "a bench at these drives replaces the record";
    EXPECT_FALSE(log_contains("master_activate"));
}

TEST_F(EtherCATBusFakeMasterTest, DrivesInAnotherOrderAreNotRebenchable) {
    FakeEcrt::set_slave_serial(1, 12);
    FakeEcrt::set_slave_serial(2, 11);
    EtherCATBus bus(0);
    const auto probed = probe_with(
        bus, {{.slave_position = 1, .identity = serial_only(11)}, {.slave_position = 2, .identity = serial_only(12)}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find("slave 1 holds the drive the approval recorded for slave 2 (serial 12) — the "
                                    "drives are in another order than at the bench: restore the cabling"),
              std::string::npos)
        << probed.message();
    EXPECT_FALSE(bus.identity_rebenchable()) << "a bench would bind the axis names to other joints";
}

// The order is checked over every slot before the first refusal: a swap behind
// a slot that differs only by an identity value is still a swap.
constexpr char kAnotherOrder[] = "the drives are in another order than at the bench: restore the cabling";

TEST_F(EtherCATBusFakeMasterTest, ASwapBehindAnotherUnitIsAnotherOrder) {
    FakeEcrt::set_slave_serial(1, 99);
    FakeEcrt::set_slave_serial(2, 11);
    EtherCATBus bus(0);
    const auto probed = probe_with(
        bus, {{.slave_position = 1, .identity = serial_only(11)}, {.slave_position = 2, .identity = serial_only(12)}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find("slave 2 holds the drive the approval recorded for slave 1 (serial 11)"),
              std::string::npos)
        << probed.message();
    EXPECT_FALSE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, ASwapOfTheLastTwoOfThreeIsAnotherOrder) {
    FakeEcrt::set_slave_serial(0, 99);
    FakeEcrt::set_slave_serial(1, 13);
    FakeEcrt::set_slave_serial(2, 12);
    EtherCATBus bus(0);
    const auto probed = probe_with(bus, {{.slave_position = 0, .identity = serial_only(11)},
                                         {.slave_position = 1, .identity = serial_only(12)},
                                         {.slave_position = 2, .identity = serial_only(13)}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find(kAnotherOrder), std::string::npos) << probed.message();
    EXPECT_FALSE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, ASwapBehindARevisionMismatchIsAnotherOrder) {
    FakeEcrt::set_slave_serial(1, 11);
    FakeEcrt::set_slave_serial(2, 13);
    FakeEcrt::set_slave_serial(3, 12);
    SlaveIdentity first = serial_only(11);
    first.revision_number = 5;  // the fake reports revision 0
    EtherCATBus bus(0);
    const auto probed = probe_with(bus, {{.slave_position = 1, .identity = first},
                                         {.slave_position = 2, .identity = serial_only(12)},
                                         {.slave_position = 3, .identity = serial_only(13)}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find(kAnotherOrder), std::string::npos) << probed.message();
    EXPECT_FALSE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, AnotherSlotsSerialIsAnotherOrderWhereNoSerialIsRecorded) {
    FakeEcrt::set_slave_serial(1, 99);
    FakeEcrt::set_slave_serial(2, 11);
    SlaveIdentity model_scope;  // no serial recorded at slot 2
    model_scope.revision_number = 0;
    EtherCATBus bus(0);
    const auto probed = probe_with(
        bus, {{.slave_position = 1, .identity = serial_only(11)}, {.slave_position = 2, .identity = model_scope}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find(kAnotherOrder), std::string::npos) << probed.message();
    EXPECT_FALSE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, ASerialStillAtItsRecordedSlotIsNoEvidenceOfOrder) {
    // A replacement at slot 2 that reports the same serial as slot 1's drive,
    // which is still in place: a duplicate, not a move.
    FakeEcrt::set_slave_serial(1, 11);
    FakeEcrt::set_slave_serial(2, 11);
    SlaveIdentity model_scope;
    model_scope.revision_number = 5;  // the fake reports revision 0
    EtherCATBus bus(0);
    const auto probed = probe_with(
        bus, {{.slave_position = 1, .identity = serial_only(11)}, {.slave_position = 2, .identity = model_scope}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_EQ(probed.message().find(kAnotherOrder), std::string::npos) << probed.message();
    EXPECT_NE(probed.message().find("slave 2: SII revision number is 0x0"), std::string::npos) << probed.message();
    EXPECT_TRUE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, AnotherModelsRecordedSerialIsNoEvidenceOfOrder) {
    // Slot 1 is model A, slot 2 model B (Step 3 holds each to its model): a B
    // unit whose serial happens to equal A's recorded one did not come from slot 1.
    constexpr uint32_t kVendorB = 0x00ABCDEF;
    constexpr uint32_t kProductB = 0x00001234;
    auto servos = six_servos();
    servos[2].vendor_id = kVendorB;
    servos[2].product_code = kProductB;
    FakeEcrt::set_slave_identity(2, kVendorB, kProductB);
    FakeEcrt::set_slave_serial(1, 55);
    FakeEcrt::set_slave_serial(2, 1203);
    EtherCATBus bus(0);
    bus.set_servo_config(servos);
    bus.set_expected_slave_identity(
        {{.slave_position = 1, .identity = serial_only(1203)}, {.slave_position = 2, .identity = serial_only(77)}});
    bus.set_probe(true);
    const auto probed = bus.initialize();
    ASSERT_FALSE(probed.is_ok());
    EXPECT_EQ(probed.message().find(kAnotherOrder), std::string::npos) << probed.message();
    EXPECT_NE(probed.message().find("slave 1: SII serial number is 55 but the approval record declares 1203"),
              std::string::npos)
        << probed.message();
    EXPECT_TRUE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, SerialZeroIsNoEvidenceOfOrder) {
    // A drive that reports serial 0 and a record of serial 0 (model scope) say
    // nothing about which drive is where.
    EtherCATBus bus(0);
    const auto probed = probe_with(
        bus, {{.slave_position = 1, .identity = serial_only(11)}, {.slave_position = 2, .identity = serial_only(0)}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find("slave 1: SII serial number is 0 but the approval record declares 11"),
              std::string::npos)
        << probed.message();
    EXPECT_TRUE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, AnotherDriveModelIsNotRebenchable) {
    FakeEcrt::set_slave_identity(2, 0x1234, 0x5678);
    EtherCATBus bus(0);
    const auto probed = probe_with(bus, {{.slave_position = 2, .identity = serial_only(0)}});
    ASSERT_FALSE(probed.is_ok());
    EXPECT_NE(probed.message().find("vendor 0x1234 product 0x5678"), std::string::npos) << probed.message();
    EXPECT_FALSE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, AStartRefusedForAnIdentityValueSaysItIsRebenchableToo) {
    FakeEcrt::set_slave_serial(2, 8);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_expected_slave_identity({{.slave_position = 2, .identity = serial_only(7)}});
    ASSERT_FALSE(bus.initialize().is_ok());
    EXPECT_TRUE(bus.identity_rebenchable());
}

TEST_F(EtherCATBusFakeMasterTest, ASerialMismatchRefusesToInitializeAndReleasesTheMaster) {
    fake_synthetic_drive();
    FakeEcrt::set_slave_serial(2, 7);
    const auto refused = initialize_with(synthetic_identity());
    EXPECT_NE(refused.message().find("slave 2: SII serial number is 7 but the approval record declares"),
              std::string::npos)
        << refused.message();
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_FALSE(log_contains("master_activate"));
}

TEST_F(EtherCATBusFakeMasterTest, TheSerialIsComparedAsDeclaredZeroIncluded) {
    fake_synthetic_drive();
    SlaveIdentity id = synthetic_identity();
    id.serial_number = 1;
    EXPECT_NE(initialize_with(id).message().find("SII serial number is 0 but the approval record declares 1"),
              std::string::npos)
        << "declared 1, drive reports 0";
    FakeEcrt::reset();
    fake_synthetic_drive();
    FakeEcrt::set_slave_serial(2, 1);
    const auto init = initialize_with(id);
    EXPECT_TRUE(init.is_ok()) << init.message();
}

TEST_F(EtherCATBusFakeMasterTest, TheRevisionNumberIsComparedFromTheSiiAsDeclared) {
    fake_synthetic_drive();
    SlaveIdentity id = synthetic_identity();
    id.revision_number = 2;
    EXPECT_NE(initialize_with(id).message().find("SII revision number is 0x0 but the approval record declares 0x2"),
              std::string::npos)
        << "declared 2, the SII reports 0";
    EXPECT_TRUE(log_contains("release_master"));
    FakeEcrt::reset();
    fake_synthetic_drive();
    id.revision_number = 0;
    const auto init = initialize_with(id);
    EXPECT_TRUE(init.is_ok()) << init.message() << "0 is a value, compared as declared";
    EXPECT_EQ(count_calls("idx=0x1018"), 0) << "the master's SII copy — no SDO read";
}

TEST_F(EtherCATBusFakeMasterTest, AVersionMismatchRefusesToInitialize) {
    FakeEcrt::set_sdo_upload_string(0x1009, "1.3");
    FakeEcrt::set_sdo_upload_string(0x100A, "2.08");
    EXPECT_NE(initialize_with(synthetic_identity()).message().find("software version (0x100a) is '2.08'"),
              std::string::npos);
    EXPECT_TRUE(log_contains("release_master"));
    FakeEcrt::reset();
    FakeEcrt::set_sdo_upload_string(0x1009, std::string("1.3\0", 4));  // a NUL is a byte too
    FakeEcrt::set_sdo_upload_string(0x100A, "2.07");
    EXPECT_NE(initialize_with(synthetic_identity()).message().find("hardware version (0x1009) is"), std::string::npos);
}

TEST_F(EtherCATBusFakeMasterTest, ADeclaredValueTheDriveCannotAnswerIsAMismatch) {
    FakeEcrt::set_sdo_upload_string(0x1009, "1.3");  // 0x100A stays unreadable
    EXPECT_NE(initialize_with(synthetic_identity()).message().find("could not read the software version (0x100a:00"),
              std::string::npos);
    EXPECT_TRUE(log_contains("release_master"));
    FakeEcrt::reset();
    FakeEcrt::set_sdo_upload_string(0x1009, "1.3");
    SlaveIdentity id = synthetic_identity();
    id.software_version.reset();  // not declared, not read
    const auto init = initialize_with(id);
    EXPECT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("idx=0x100a"), 0);
}

TEST_F(EtherCATBusFakeMasterTest, AVersionReadFailingWithoutAnAbortIsReadAgain) {
    fake_synthetic_drive();
    FakeEcrt::set_sdo_upload_transient_failures(2, 0x100A, 0, 3);
    const auto init = initialize_with(synthetic_identity());
    EXPECT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("master_sdo_upload(pos=2,idx=0x100a,sub=0x0)"), 4);
    FakeEcrt::reset();
    fake_synthetic_drive();
    FakeEcrt::set_sdo_upload_transient_failures(2, 0x100A, 0, 4);
    EtherCATBus bus(0);
    const std::string refusal = probe_with(bus, {{.slave_position = 2, .identity = synthetic_identity()}}).message();
    EXPECT_NE(refusal.find("could not read the software version (0x100a:00, ret=-5, abort=0x0, attempts: 4)"),
              std::string::npos);
    EXPECT_NE(refusal.find("start again once the bus has been idle"), std::string::npos) << refusal;
    EXPECT_EQ(refusal.find("re-bench"), std::string::npos) << "a mailbox clash is not the record's fault: " << refusal;
    EXPECT_FALSE(bus.identity_rebenchable());
    EXPECT_EQ(count_calls("master_sdo_upload(pos=2,idx=0x100a,sub=0x0)"), 4);
}

TEST_F(EtherCATBusFakeMasterTest, AVersionReadAbortedByTheDriveStaysRebenchableAndIsNotReadAgain) {
    fake_synthetic_drive();
    FakeEcrt::set_sdo_upload_entry(2, 0x100A, 0, -EIO, {}, 0x06020000);  // object does not exist
    EtherCATBus bus(0);
    const std::string refusal = probe_with(bus, {{.slave_position = 2, .identity = synthetic_identity()}}).message();
    EXPECT_NE(refusal.find("re-bench"), std::string::npos) << refusal;
    EXPECT_TRUE(bus.identity_rebenchable());
    EXPECT_EQ(count_calls("master_sdo_upload(pos=2,idx=0x100a,sub=0x0)"), 1);
}

TEST_F(EtherCATBusFakeMasterTest, AnIdentityForASlaveWithoutAServoIsRefusedBeforeAnyEcrtCall) {
    fake_synthetic_drive();
    EXPECT_FALSE(initialize_with(synthetic_identity(), 6).is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

// Stored drive values (drive-model schema §4.9)

#include <cerrno>
#include <chrono>
#include <mutex>
#include <thread>
#include <utility>

#include <spdlog/sinks/sink.h>
#include <spdlog/spdlog.h>

#include <plem/core/logging/logger.hpp>

namespace {

using plem::fieldbus::ethercat::SlaveSdos;
using plem::fieldbus::ethercat::StartupSdo;

/// Every line the core logger writes, as {level, text}: a sink on its async
/// "plem" logger.
class LineSink final : public spdlog::sinks::sink {
  public:
    void log(const spdlog::details::log_msg& msg) override {
        const std::lock_guard lock(mutex_);
        lines_.emplace_back(msg.level, std::string(msg.payload.data(), msg.payload.size()));
    }
    void flush() override {}
    void set_pattern(const std::string& /*pattern*/) override {}
    void set_formatter(std::unique_ptr<spdlog::formatter> /*formatter*/) override {}

    /// The lines since the last take(). A fence line is logged and awaited:
    /// the logger drains through one worker in order, so every earlier line
    /// has passed once the fence has.
    std::vector<std::pair<spdlog::level::level_enum, std::string>> take() {
        const std::string fence = "line sink fence " + std::to_string(++fences_);
        spdlog::get("plem")->warn(fence);
        for (int waited_ms = 0; waited_ms < 5000; ++waited_ms) {
            {
                const std::lock_guard lock(mutex_);
                const auto at =
                    std::find_if(lines_.begin(), lines_.end(), [&](const auto& line) { return line.second == fence; });
                if (at != lines_.end()) {
                    std::vector<std::pair<spdlog::level::level_enum, std::string>> taken(lines_.begin(), at);
                    lines_.clear();
                    return taken;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        ADD_FAILURE() << "the logger did not drain within 5 s";
        return {};
    }

  private:
    std::mutex mutex_;
    std::vector<std::pair<spdlog::level::level_enum, std::string>> lines_;
    int fences_ = 0;
};

/// Installed during static initialization, before any test logs: the
/// logger's sink list is not safe to change while its worker runs.
LineSink& line_sink() {
    static const auto sink = [] {
        plem::core::Logger::instance();
        auto installed = std::make_shared<LineSink>();
        spdlog::get("plem")->sinks().push_back(installed);
        return installed;
    }();
    return *sink;
}
[[maybe_unused]] LineSink& kLineSinkInstalled = line_sink();

/// The lines this feature wrote since the last take; each names stored_sdo.
struct StoredSdoLog {
    std::vector<std::string> warn;
    std::vector<std::string> info;
};

StoredSdoLog stored_sdo_log() {
    StoredSdoLog log;
    for (auto& [level, text] : line_sink().take()) {
        if (text.find("stored_sdo") == std::string::npos) {
            continue;
        }
        (level == spdlog::level::info ? log.info : log.warn).push_back(std::move(text));
    }
    return log;
}

bool has_line(const std::vector<std::string>& lines, const std::string& needle) {
    return std::any_of(lines.begin(), lines.end(),
                       [&](const std::string& line) { return line.find(needle) != std::string::npos; });
}

plem::core::Result<void> initialize_with_stored(std::vector<SlaveSdos> lists,
                                                std::vector<ServoInfo> servos = six_servos()) {
    EtherCATBus bus(0);
    bus.set_servo_config(std::move(servos));
    bus.set_stored_sdos(std::move(lists));
    return bus.initialize();
}

// Process data on SM4/SM5 (a CoE drive may place it there): the single-axis
// CST entries under PDOs 0x1602 / 0x1a03.
constexpr PdoMapping kSm4Rx[] = {{0x1602, plem::fieldbus::test::kSingleAxisCstRx, 5}};
constexpr PdoMapping kSm5Tx[] = {{0x1a03, plem::fieldbus::test::kSingleAxisCstTx, 5}};
constexpr SyncManagerAssignment kSm45Syncs[] = {
    {0, true, nullptr, 0, false},
    {1, false, nullptr, 0, false},
    {4, true, kSm4Rx, 1, true},
    {5, false, kSm5Tx, 1, false},
};
constexpr SlavePdoMapping kSm45Layout{kSm45Syncs, 4};

class StoredSdoTest : public EtherCATBusFakeMasterTest {
  protected:
    void SetUp() override {
        EtherCATBusFakeMasterTest::SetUp();
        line_sink().take();  // earlier tests' lines
    }
};

/// Output PDO entries and PDO configuration objects are skipped with a WARN
/// each and never uploaded; an input PDO entry is read.
void expect_skips_against(std::vector<ServoInfo> servos) {
    FakeEcrt::set_sdo_upload_entry(2, 0x6064, 0, 0, {0, 0, 0, 0});
    const auto init = initialize_with_stored(
        {{.slave_position = 2,
          .sdos = {{0x60FF, 0x00, 4, 0}, {0x1C14, 0x00, 1, 1}, {0x1602, 0x01, 4, 0x60400010}, {0x6064, 0x00, 4, 0}}}},
        std::move(servos));
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    ASSERT_EQ(log.warn.size(), 3u);
    EXPECT_EQ(log.warn[0],
              "[EtherCATBus] slave 2: 0x60ff:00 — this object is in the slave's output process data, which the master "
              "sends every cycle; a stored value cannot be judged here — drop it from stored_sdo (schema §4.9)");
    EXPECT_EQ(log.warn[1],
              "[EtherCATBus] slave 2: 0x1c14:00 — a PDO mapping or assignment object is PDO configuration, not a "
              "stored drive setting to compare here — drop it from stored_sdo (schema §4.9)");
    EXPECT_TRUE(has_line({log.warn[2]}, "slave 2: 0x1602:01 — a PDO mapping or assignment object"));
    EXPECT_TRUE(log.info.empty()) << "not every value was compared";
    EXPECT_EQ(count_calls("master_sdo_upload"), 1) << "the input entry only";
    EXPECT_EQ(count_calls("master_sdo_upload(pos=2,idx=0x6064,sub=0x0)"), 1);
}

}  // namespace

TEST_F(StoredSdoTest, AllMatchingIsOneInfoLineReadBeforeAnyConfiguration) {
    FakeEcrt::set_sdo_upload_entry(1, 0x3002, 0, 0, {1, 0});
    FakeEcrt::set_sdo_upload_entry(1, 0x3401, 0, 0, {0, 0, 0, 0});
    const auto init =
        initialize_with_stored({{.slave_position = 1, .sdos = {{0x3002, 0x00, 2, 1}, {0x3401, 0x00, 4, 0}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    EXPECT_TRUE(log.warn.empty());
    ASSERT_EQ(log.info.size(), 1u);
    EXPECT_EQ(log.info[0], "[EtherCATBus] slave 1: stored_sdo 2/2 match the drive's current values");
    const int first = index_of("master_sdo_upload(pos=1,idx=0x3002,sub=0x0)");
    const int second = index_of("master_sdo_upload(pos=1,idx=0x3401,sub=0x0)");
    ASSERT_GE(first, 0);
    EXPECT_GT(first, index_of("request_master"));
    EXPECT_LT(second, index_of("config_pdos")) << "Step 3.6 precedes the PDO configuration";
    EXPECT_LT(second, index_of("master_activate"));
}

TEST_F(StoredSdoTest, ReadsAfterTheIdentityCheck) {
    FakeEcrt::set_sdo_upload_string(0x100A, "2.07");
    FakeEcrt::set_sdo_upload_entry(1, 0x3015, 0, 0, {2, 0});
    plem::fieldbus::ethercat::SlaveIdentity id;
    id.software_version = std::array<char, 32>{'2', '.', '0', '7'};
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    bus.set_expected_slave_identity({{.slave_position = 1, .identity = id}});
    bus.set_stored_sdos({{.slave_position = 1, .sdos = {{0x3015, 0x00, 2, 2}}}});
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    const int identity = index_of("master_sdo_upload(pos=1,idx=0x100a,sub=0x0)");
    ASSERT_GE(identity, 0);
    EXPECT_GT(index_of("master_sdo_upload(pos=1,idx=0x3015,sub=0x0)"), identity) << "Step 3.6 follows Step 3.5";
}

TEST_F(StoredSdoTest, ComparesEveryByteLittleEndianAsTheUnsignedPattern) {
    // Mixed bytes catch a read of the low byte only; 0xfffe is a signed
    // object's raw pattern (-2), compared unsigned at its width.
    FakeEcrt::set_sdo_upload_entry(1, 0x2001, 0, 0, {0x34, 0x12});
    FakeEcrt::set_sdo_upload_entry(1, 0x2002, 0, 0, {0x78, 0x56, 0x34, 0x12});
    FakeEcrt::set_sdo_upload_entry(1, 0x2003, 0, 0, {0xfe, 0xff});
    const auto init = initialize_with_stored(
        {{.slave_position = 1,
          .sdos = {{0x2001, 0x00, 2, 0x1234}, {0x2002, 0x00, 4, 0x12345678}, {0x2003, 0x00, 2, 0xfffe}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    EXPECT_TRUE(log.warn.empty()) << (log.warn.empty() ? "" : log.warn[0]);
    ASSERT_EQ(log.info.size(), 1u);
    EXPECT_EQ(log.info[0], "[EtherCATBus] slave 1: stored_sdo 3/3 match the drive's current values");
}

TEST_F(StoredSdoTest, ASignedPatternMismatchHintsTheUnsignedDownload) {
    FakeEcrt::set_sdo_upload_entry(1, 0x2004, 0, 0, {0, 0, 0, 0});
    const auto init = initialize_with_stored({{.slave_position = 1, .sdos = {{0x2004, 0x00, 4, 0xfffffffe}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    ASSERT_EQ(log.warn.size(), 1u);
    EXPECT_TRUE(has_line(log.warn,
                         "slave 1: 0x2004:00 reads 0 (0x0), the model file's stored_sdo expects 4294967294 "
                         "(0xfffffffe)"));
    EXPECT_TRUE(has_line(log.warn, "ethercat -m 0 -p 1 download -t uint32 0x2004 0 0xfffffffe."));
}

TEST_F(StoredSdoTest, AMismatchIsOneWarnLineThatWritesNothing) {
    FakeEcrt::set_sdo_upload_entry(1, 0x3222, 0, 0, {20, 0});
    const auto init = initialize_with_stored({{.slave_position = 1, .sdos = {{0x3222, 0x00, 2, 139}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    ASSERT_EQ(log.warn.size(), 1u);
    EXPECT_EQ(log.warn[0],
              "[EtherCATBus] slave 1: 0x3222:00 reads 20 (0x14), the model file's stored_sdo expects 139 (0x8b) "
              "(schema §4.9); this comparison performs no writes. If the file is right and the drive manual lists "
              "the object as writable and storable, power-cycle first, then with the node stopped run ethercat -m "
              "0 -p 1 download -t uint16 0x3222 0 0x8b. Then run ethercat -m 0 -p 1 download -t uint32 0x1010 1 "
              "0x65766173 (CiA 301 Store parameters, if the drive implements it; see its manual). A save usually "
              "stores every storable value, including unsaved changes since power-up. Power-cycle again if the "
              "manual says the object takes effect at power-up. If the drive is right, change the file's "
              "stored_sdo.");
    EXPECT_TRUE(log.info.empty());
    EXPECT_EQ(count_calls("slave_config_sdo"), 0) << "nothing written or reserved";
}

TEST_F(StoredSdoTest, SkipsTheDeclaredLayoutsOutputEntriesAndPdoObjects) {
    auto servos = six_servos();
    for (auto& servo : servos) {
        servo.pdo_mapping = &kSm45Layout;
    }
    expect_skips_against(std::move(servos));
}

TEST_F(StoredSdoTest, SkipsTheScannedLayoutsOutputEntriesAndPdoObjects) {
    auto servos = six_servos();
    for (unsigned int pos = 0; pos < servos.size(); ++pos) {
        servos[pos].pdo_mapping = nullptr;
        FakeEcrt::set_slave_scan(pos, 0, kSm45Layout);
    }
    expect_skips_against(std::move(servos));
}

TEST_F(StoredSdoTest, AWidthMismatchAnOversizeAnswerAndAnAbortMoveToTheNextValue) {
    FakeEcrt::set_sdo_upload_entry(0, 0x3002, 0, 0, {1, 0, 0, 0});
    FakeEcrt::set_sdo_upload_entry(0, 0x3003, 0, -ENOBUFS, {});
    FakeEcrt::set_sdo_upload_entry(0, 0x3004, 0, -EIO, {}, 0x06020000);
    FakeEcrt::set_sdo_upload_entry(0, 0x3015, 0, 0, {2, 0});
    const auto init = initialize_with_stored(
        {{.slave_position = 0,
          .sdos = {{0x3002, 0x00, 2, 1}, {0x3003, 0x00, 2, 11}, {0x3004, 0x00, 2, 0}, {0x3015, 0x00, 2, 2}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    ASSERT_EQ(log.warn.size(), 3u);
    EXPECT_TRUE(has_line(log.warn, "slave 0: 0x3002:00 answered 4 bytes but the model file's stored_sdo says size 2"));
    EXPECT_TRUE(has_line(log.warn, "slave 0: 0x3003:00 answered more than 4 bytes"));
    EXPECT_TRUE(has_line(log.warn, "slave 0: could not read 0x3004:00 for stored_sdo (SDO abort 0x06020000)"));
    EXPECT_TRUE(log.info.empty());
    EXPECT_EQ(count_calls("master_sdo_upload(pos=0,idx=0x3015,sub=0x0)"), 1) << "read after the three";
    EXPECT_EQ(count_calls("master_sdo_upload(pos=0,idx=0x3004,sub=0x0)"), 1) << "an abort is not read again";
}

TEST_F(StoredSdoTest, ANoAnswerSkipsTheRestOfThatSlaveOnly) {
    // After an abort, a failure that sets no abort code must not inherit the
    // previous one (the outputs are reset per call).
    FakeEcrt::set_sdo_upload_entry(0, 0x3004, 0, -EIO, {}, 0x06020000);
    FakeEcrt::set_sdo_upload_entry(0, 0x3005, 0, -ETIMEDOUT, {});
    FakeEcrt::set_sdo_upload_entry(0, 0x3015, 0, 0, {2, 0});
    FakeEcrt::set_sdo_upload_entry(1, 0x3015, 0, 0, {2, 0});
    const auto init = initialize_with_stored(
        {{.slave_position = 0, .sdos = {{0x3004, 0x00, 2, 0}, {0x3005, 0x00, 2, 0}, {0x3015, 0x00, 2, 2}}},
         {.slave_position = 1, .sdos = {{0x3015, 0x00, 2, 2}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    ASSERT_EQ(log.warn.size(), 2u);
    EXPECT_TRUE(has_line(log.warn,
                         "slave 0: could not read 0x3005:00 for stored_sdo — upload failed without an SDO abort "
                         "(ret=-110, attempts: 1) — no answer, a slave without CoE, or a mailbox error; its "
                         "remaining"));
    EXPECT_TRUE(has_line(log.warn, "its remaining 1 stored_sdo values are not read"));
    EXPECT_EQ(count_calls("master_sdo_upload(pos=0,idx=0x3005,sub=0x0)"), 1) << "only -EIO is read again";
    EXPECT_EQ(count_calls("master_sdo_upload(pos=0,idx=0x3015,sub=0x0)"), 0);
    EXPECT_EQ(count_calls("master_sdo_upload(pos=1,idx=0x3015,sub=0x0)"), 1) << "the next slave is read";
    ASSERT_EQ(log.info.size(), 1u);
    EXPECT_EQ(log.info[0], "[EtherCATBus] slave 1: stored_sdo 1/1 match the drive's current values");
}

TEST_F(StoredSdoTest, AReadFailingWithoutAnAbortIsReadAgain) {
    FakeEcrt::set_sdo_upload_transient_failures(0, 0x3015, 0, 2);
    FakeEcrt::set_sdo_upload_entry(0, 0x3015, 0, 0, {2, 0});
    const auto init = initialize_with_stored({{.slave_position = 0, .sdos = {{0x3015, 0x00, 2, 2}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    EXPECT_TRUE(log.warn.empty());
    ASSERT_EQ(log.info.size(), 1u);
    EXPECT_EQ(log.info[0], "[EtherCATBus] slave 0: stored_sdo 1/1 match the drive's current values");
    EXPECT_EQ(count_calls("master_sdo_upload(pos=0,idx=0x3015,sub=0x0)"), 3);
}

TEST_F(StoredSdoTest, AReadStillFailingAfterThreeRetriesEndsThatSlavesReads) {
    FakeEcrt::set_sdo_upload_transient_failures(0, 0x3015, 0, 4);
    FakeEcrt::set_sdo_upload_entry(0, 0x3015, 0, 0, {2, 0});
    const auto init =
        initialize_with_stored({{.slave_position = 0, .sdos = {{0x3015, 0x00, 2, 2}, {0x3002, 0x00, 2, 1}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    ASSERT_EQ(log.warn.size(), 1u);
    EXPECT_TRUE(has_line(log.warn,
                         "slave 0: could not read 0x3015:00 for stored_sdo — upload failed without an SDO abort "
                         "(ret=-5, attempts: 4) — no answer, a slave without CoE, or a mailbox error such as the "
                         "master's own SDO dictionary read after a bus scan (to compare, start again once the bus has "
                         "been idle for a while);"));
    EXPECT_EQ(count_calls("master_sdo_upload(pos=0,idx=0x3015,sub=0x0)"), 4);
    EXPECT_EQ(count_calls("master_sdo_upload(pos=0,idx=0x3002,sub=0x0)"), 0);
}

TEST_F(StoredSdoTest, AnInterruptedReadEndsTheWholeCheck) {
    FakeEcrt::set_sdo_upload_entry(0, 0x3015, 0, -EINTR, {});
    FakeEcrt::set_sdo_upload_entry(1, 0x3015, 0, 0, {2, 0});
    const auto init =
        initialize_with_stored({{.slave_position = 0, .sdos = {{0x3015, 0x00, 2, 2}, {0x3002, 0x00, 2, 1}}},
                                {.slave_position = 1, .sdos = {{0x3015, 0x00, 2, 2}}}});
    ASSERT_TRUE(init.is_ok()) << init.message();
    const auto log = stored_sdo_log();
    ASSERT_EQ(log.warn.size(), 1u);
    EXPECT_TRUE(
        has_line(log.warn, "slave 0: reading 0x3015:00 was interrupted by a signal — the stored_sdo check stops"));
    EXPECT_TRUE(log.info.empty());
    EXPECT_EQ(count_calls("master_sdo_upload"), 1);
    EXPECT_TRUE(log_contains("master_activate")) << "initialize carries on";
}

TEST_F(StoredSdoTest, NoStoredValueMeansNoUploadAndTheSameCallSequence) {
    const auto sequence = [] {
        std::string out;
        for (const auto& call : FakeEcrt::calls()) {
            out += call + "\n";
        }
        return out;
    };
    {
        EtherCATBus plain(0);
        plain.set_servo_config(six_servos());
        const auto init = plain.initialize();
        ASSERT_TRUE(init.is_ok()) << init.message();
    }  // destroyed like initialize_with_stored's bus: both logs end with release_master
    const std::string without = sequence();
    FakeEcrt::reset();
    std::vector<SlaveSdos> empty;
    for (uint32_t pos = 0; pos < 6; ++pos) {
        empty.push_back({.slave_position = pos, .sdos = {}});
    }
    const auto init2 = initialize_with_stored(empty);
    ASSERT_TRUE(init2.is_ok()) << init2.message();
    EXPECT_EQ(sequence(), without);
    EXPECT_EQ(count_calls("master_sdo_upload"), 0);
    const auto log = stored_sdo_log();
    EXPECT_TRUE(log.warn.empty());
    EXPECT_TRUE(log.info.empty());
}

TEST_F(StoredSdoTest, AListForASlaveWithoutAServoOrAnInvalidWidthIsRefusedBeforeAnyEcrtCall) {
    EXPECT_FALSE(initialize_with_stored({{.slave_position = 6, .sdos = {{0x3015, 0x00, 2, 2}}}}).is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
    EXPECT_FALSE(initialize_with_stored({{.slave_position = 0, .sdos = {{0x3015, 0x00, 3, 2}}}}).is_ok());
    EXPECT_TRUE(FakeEcrt::calls().empty());
}

// DC cycle time: written for a drive that takes it from the master
// (drive-model schema §5, dc.write_cycle_time)

namespace {

constexpr uint32_t kSync0Ns = 1'000'000;  // SYNC0 1 ms on a 500 µs bus: the written value is SYNC0's

/// Six axes on three two-axis slaves, DC declared on every axis.
std::vector<ServoInfo> dc_axes_on_three_slaves(bool write_cycle_time) {
    auto infos = six_axes_on_three_slaves();
    for (auto& info : infos) {
        info.dc_assign_activate = 0x0300;
        info.dc_sync0_cycle_ns = kSync0Ns;
        info.dc_write_cycle_time = write_cycle_time;
    }
    return infos;
}

plem::core::Result<void> initialize_dc(std::vector<ServoInfo> servos) {
    EtherCATBus bus(0);
    bus.set_servo_config(std::move(servos));
    bus.set_cycle_time_ns(500'000);
    bus.set_dc_enable(true);
    return bus.initialize();
}

}  // namespace

TEST_F(EtherCATBusFakeMasterTest, WriteCycleTimeRegistersOneU32SyncCyclePerSlave) {
    const auto init = initialize_dc(dc_axes_on_three_slaves(true));
    ASSERT_TRUE(init.is_ok()) << init.message();
    const int activate = index_of("master_activate");
    ASSERT_GE(activate, 0);
    for (int pos = 0; pos < 3; ++pos) {
        SCOPED_TRACE("slave " + std::to_string(pos));
        const std::string write = "slave_config_sdo32(pos=" + std::to_string(pos) + ",idx=0x1c32,sub=0x2,val=0xf4240)";
        EXPECT_EQ(count_calls(write), 1) << "one write per slave, at its ring position, whatever axes it hosts";
        EXPECT_LT(index_of(write), activate);
    }
    EXPECT_EQ(count_calls("slave_config_sdo"), 3) << "only 0x1C32:02 as U32 — no :01, no 0x1C33, no channel offset";
    EXPECT_EQ(count_calls("master_sdo_upload"), 0) << "the drive's 0x1C32 is never read";
}

TEST_F(EtherCATBusFakeMasterTest, WriteCycleTimeIsDeclaredPerSlave) {
    auto infos = dc_axes_on_three_slaves(false);
    infos[2].dc_write_cycle_time = true;  // slave 1's two axes
    infos[3].dc_write_cycle_time = true;
    const auto init = initialize_dc(infos);
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(count_calls("slave_config_sdo32"), 1);
    EXPECT_EQ(count_calls("slave_config_sdo32(pos=1,idx=0x1c32,sub=0x2,val=0xf4240)"), 1);
    EXPECT_EQ(count_calls("master_sdo_upload"), 0) << "the slaves without the key are not read either";
}

// A DC step the master refuses stops the bring-up before activation: a bus
// running without its declared sync would drift unnoticed.
TEST_F(EtherCATBusFakeMasterTest, ADcConfigurationTheMasterRefusesIsNamedAndNothingActivates) {
    FakeEcrt::set_dc_failure(1, -22);  // slave 1 hosts servos 2 and 3
    EtherCATBus bus(0);                // alive through the asserts: its destructor would release the master too
    bus.set_servo_config(dc_axes_on_three_slaves(false));
    bus.set_cycle_time_ns(500'000);
    bus.set_dc_enable(true);
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("Failed to configure DC on axis 2 (ecrt_slave_config_dc() returned -22)"),
              std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("select_reference_clock"), -1);
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_TRUE(bus.servo_devices().empty());
}

TEST_F(EtherCATBusFakeMasterTest, AReferenceClockTheMasterRefusesIsNamedAndNothingActivates) {
    FakeEcrt::set_reference_clock_failure(-22);
    EtherCATBus bus(0);
    bus.set_servo_config(dc_axes_on_three_slaves(false));
    bus.set_cycle_time_ns(500'000);
    bus.set_dc_enable(true);
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find(
                  "Failed to select slave 0 as DC reference clock (ecrt_master_select_reference_clock() returned -22)"),
              std::string::npos)
        << refused.message();
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(log_contains("release_master"));
    EXPECT_TRUE(bus.servo_devices().empty());
}

// Slave configuration state (the release gate's input)

TEST_F(EtherCATBusFakeMasterTest, SlaveStatesAreQueriedOncePerSlaveAndSharedByItsAxes) {
    EtherCATBus bus(0);
    bus.set_servo_config(six_axes_on_three_slaves());
    const auto init = bus.initialize();
    ASSERT_TRUE(init.is_ok()) << init.message();
    FakeEcrt::set_slave_config_state(1, 4);  // SAFEOP
    FakeEcrt::set_slave_config_state_failure(2);
    FakeEcrt::calls().clear();
    std::array<EtherCATBus::SlaveState, 6> states{};
    bus.sample_servo_slave_states(states);
    EXPECT_EQ(FakeEcrt::calls(), (std::vector<std::string>{"slave_config_state(pos=0)", "slave_config_state(pos=1)",
                                                           "slave_config_state(pos=2)"}));
    for (size_t axis = 0; axis < 6; ++axis) {
        SCOPED_TRACE("axis " + std::to_string(axis));
        const auto& state = states[axis];
        switch (axis / 2) {
            case 0:
                EXPECT_TRUE(state.operational);
                EXPECT_EQ(state.al_state, 8);
                break;
            case 1:
                EXPECT_FALSE(state.operational);
                EXPECT_EQ(state.al_state, 4);
                break;
            default:
                EXPECT_FALSE(state.operational) << "a failed query is not operational";
                EXPECT_EQ(state.al_state, 0);
                break;
        }
    }
}

// A slave that is not the declared drive
// The refusal names both identity pairs, on a declared mapping, and on a
// scanned one before the scan is checked: another drive's mapping failing the
// validator would otherwise send the operator to reconfigure PDOs.

namespace {

constexpr PdoEntryMapping kNoTorqueTx[] = {
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6061, 0x00, 8},
};
constexpr PdoMapping kNoTorqueTxPdos[] = {{0x1a00, kNoTorqueTx, 4}};
constexpr SyncManagerAssignment kNoTorqueSyncs[] = {
    {2, true, plem::fieldbus::test::kSingleAxisCstRxPdos, 1, true},
    {3, false, kNoTorqueTxPdos, 1, false},
};
constexpr SlavePdoMapping kNoTorqueLayout{kNoTorqueSyncs, 2};

std::vector<std::string> error_lines() {
    std::vector<std::string> errors;
    for (auto& [level, text] : line_sink().take()) {
        if (level == spdlog::level::err) {
            errors.push_back(std::move(text));
        }
    }
    return errors;
}

class SlaveIdentityTest : public EtherCATBusFakeMasterTest {
  protected:
    void SetUp() override {
        EtherCATBusFakeMasterTest::SetUp();
        line_sink().take();  // earlier tests' lines
    }
};

constexpr const char* kIdentityRefusal =
    "axis 2 (slave 2): the configuration declares vendor 0xc0ffee product 0x1001, but the slave at that ring "
    "position is vendor 0xdead product 0xbeef";

}  // namespace

TEST_F(SlaveIdentityTest, ADeclaredMappingSlaveOfAnotherDriveIsRefusedByBothIdentities) {
    FakeEcrt::set_slave_identity(2, 0xdead, 0xbeef);
    EtherCATBus bus(0);
    bus.set_servo_config(six_servos());
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find(kIdentityRefusal), std::string::npos) << refused.message();
}

TEST_F(SlaveIdentityTest, AScannedSlaveOfAnotherDriveIsRefusedByIdentityBeforeItsMappingIsJudged) {
    auto servos = six_servos();
    for (unsigned int pos = 0; pos < servos.size(); ++pos) {
        servos[pos].pdo_mapping = nullptr;
        FakeEcrt::set_slave_scan(pos, 0x1, plem::fieldbus::test::kSingleAxisCstLayout);
    }
    FakeEcrt::set_slave_scan(2, 0x1, kNoTorqueLayout);  // the other drive cannot serve the axis either
    FakeEcrt::set_slave_identity(2, 0xdead, 0xbeef);
    EtherCATBus bus(0);
    bus.set_servo_config(servos);
    const auto refused = bus.initialize();
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find(kIdentityRefusal), std::string::npos) << refused.message();
    EXPECT_EQ(refused.message().find("does not serve the axes"), std::string::npos) << refused.message();
    EXPECT_FALSE(has_line(error_lines(), "does not serve the axes"));
}
