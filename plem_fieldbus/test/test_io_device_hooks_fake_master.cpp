// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_io_device_hooks_fake_master.cpp
/// @brief The DeviceBase hooks as the bus and controller call them, seen by a
/// recording IO device injected through the IO device factory:
/// configure_after_domain (once, after the entry list, before the servo
/// startup SDOs; < 0 makes initialize() refuse), the factory context's bus
/// period, and the per-cycle order note_received_frame → read → write. The RT
/// loop runs in lock step (the send observer parks it after every cycle) so
/// the order is cycle-exact.

#include <gtest/gtest.h>

#include <fmt/format.h>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/core/types/shared_joint_data.hpp>

#include "fake_master_fixture.hpp"
#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/controller.hpp"
#include "plem/fieldbus/test/fake_ecrt.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

using plem::fieldbus::ethercat::DeviceBase;
using plem::fieldbus::ethercat::EtherCATBus;
using plem::fieldbus::ethercat::EtherCATController;
using plem::fieldbus::ethercat::IODeviceContext;
using plem::fieldbus::ethercat::IOInfo;
using plem::fieldbus::ethercat::ServoInfo;
using plem::fieldbus::ethercat::SlaveSdos;
using plem::fieldbus::test::FakeEcrt;
using plem::fieldbus::test::index_of;
namespace layout = plem::fieldbus::test;

namespace {

constexpr uint32_t kServos = 6;
constexpr uint16_t kStSwitchOnDisabled = 0x0240;

/// Records every hook call. configure_after_domain also marks the fake's ecrt
/// call log, so its place among the ecrt calls is visible.
class RecordingIoDevice : public DeviceBase {
  public:
    /// A non-empty refusal makes that hook refuse with exactly that text.
    RecordingIoDevice(const IODeviceContext& context, std::string configure_refusal, std::string register_refusal)
        : DeviceBase(context.master, context.slave_position, context.io_info.vendor_id, context.io_info.product_code),
          configure_refusal_(std::move(configure_refusal)),
          register_refusal_(std::move(register_refusal)) {}

    plem::core::Result<void> register_PDOs(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index) override {
        if (!register_refusal_.empty()) {
            return plem::core::Result<void>::error(register_refusal_);
        }
        return register_PDO_entries(domain_regs, domain_index, {{0x7000, 0x01, &offset_}});
    }
    plem::core::Result<void> configure_after_domain(ec_domain_t* /*domain*/) override {
        FakeEcrt::calls().push_back("configure_after_domain(pos=" + std::to_string(position()) + ")");
        if (!configure_refusal_.empty()) {
            return plem::core::Result<void>::error(configure_refusal_);
        }
        return plem::core::Result<void>::ok();
    }
    void note_received_frame(bool /*wkc_complete*/) noexcept override {
        events.emplace_back("frame");
    }
    void read(const uint8_t* /*domain_pd*/) noexcept override {
        events.emplace_back("read");
    }
    void write(uint8_t* /*domain_pd*/) noexcept override {
        events.emplace_back("write");
    }

    /// Written by the RT thread while it runs and by the test thread only
    /// while the loop is parked (Lockstep) or stopped.
    std::vector<std::string> events;

  private:
    std::string configure_refusal_;
    std::string register_refusal_;
    uint32_t offset_{0};
};

IOInfo one_io() {
    IOInfo info{};
    info.vendor_id = layout::kSyntheticVendorId;
    info.product_code = 0x50000000;
    return info;
}

class IoDeviceHooksTest : public ::testing::Test {
  protected:
    void SetUp() override {
        FakeEcrt::reset();
    }

    void TearDown() override {
        gate_.free_run();
        if (controller_) {
            controller_->stop();
        }
        FakeEcrt::set_send_observer(nullptr);
    }

    /// A bus of six servos and one recording IO device (slave 6).
    plem::core::Result<void> init_bus(int64_t period_ns, std::string configure_refusal = {},
                                      std::string register_refusal = {}) {
        bus_ = std::make_shared<EtherCATBus>(0);
        bus_->set_servo_config(std::vector<ServoInfo>(kServos, plem::fieldbus::test::make_servo_info()));
        bus_->set_io_config({one_io()});
        bus_->set_startup_sdos({SlaveSdos{0, {{0x2000, 0x01, 2, 0x1234}}}});
        bus_->set_cycle_time_ns(static_cast<uint64_t>(period_ns));
        bus_->set_io_device_factory([this, configure_refusal, register_refusal](const IODeviceContext& context) {
            seen_period_ns_ = context.cycle_period_ns;
            device_ = std::make_shared<RecordingIoDevice>(context, configure_refusal, register_refusal);
            return device_;
        });
        return bus_->initialize();
    }

    void start_loop() {
        uint8_t* pd = bus_->domain_pd();
        for (uint32_t j = 0; j < kServos; ++j) {
            EC_WRITE_U8(pd + FakeEcrt::domain_offset_of(j, 0x6061, 0), 0x0A);  // CST on the mode display
        }
        set_status_all(kStSwitchOnDisabled);
        ASSERT_TRUE(ctx_.initialize().is_ok());
        flags_ = std::make_shared<plem::core::ControlModeFlags>();
        flags_->motion_enabled.store(true, std::memory_order_release);
        shared_ = std::make_shared<plem::core::SharedJointData<6>>();
        auto config = plem::fieldbus::test::test_controller_config<6>();
        controller_ = std::make_unique<EtherCATController<6>>(config);
        controller_->set_bus(bus_);
        controller_->set_mode_flags(flags_);
        controller_->set_shared_data(shared_);
        controller_->set_monitoring_context(&ctx_);
        FakeEcrt::set_send_observer([this] { gate_.on_send(); });
        ASSERT_TRUE(controller_->start());
        ASSERT_TRUE(gate_.wait_first());
    }

    void run(int cycles) {
        for (int i = 0; i < cycles; ++i) {
            ASSERT_TRUE(gate_.step()) << "the RT loop stopped cycling";
        }
    }

    void set_status_all(uint16_t word) {
        for (uint32_t j = 0; j < kServos; ++j) {
            set_status(j, word);
        }
    }
    void set_status(uint32_t joint, uint16_t word) {
        EC_WRITE_U16(bus_->domain_pd() + FakeEcrt::domain_offset_of(joint, 0x6041, 0), word);
    }

    plem::fieldbus::test::Lockstep gate_;
    plem::core::monitoring::MonitoringContext ctx_;
    std::shared_ptr<EtherCATBus> bus_;
    std::shared_ptr<RecordingIoDevice> device_;
    int64_t seen_period_ns_{0};
    std::shared_ptr<plem::core::ControlModeFlags> flags_;
    std::shared_ptr<plem::core::SharedJointData<6>> shared_;
    std::unique_ptr<EtherCATController<6>> controller_;
};

}  // namespace

TEST_F(IoDeviceHooksTest, ConfigureAfterDomainRunsOnceAfterTheListBeforeTheServoStartupSdos) {
    const auto init = init_bus(1'000'000);
    ASSERT_TRUE(init.is_ok()) << init.message();
    int count = 0;
    for (const auto& call : FakeEcrt::calls()) {
        count += call.rfind("configure_after_domain(", 0) == 0 ? 1 : 0;
    }
    EXPECT_EQ(count, 1);
    const int list = index_of("reg_pdo_entry_list(entries=");
    const int hook = index_of("configure_after_domain(pos=6)");
    const int servo_sdo = index_of("slave_config_sdo16(pos=0,idx=0x2000");
    const int activate = index_of("master_activate");
    ASSERT_GE(list, 0);
    EXPECT_LT(list, hook);
    EXPECT_LT(hook, servo_sdo);
    EXPECT_LT(servo_sdo, activate);
}

TEST_F(IoDeviceHooksTest, ConfigureAfterDomainRefusalSaysItsCauseAndReleasesTheMaster) {
    const auto refused = init_bus(1'000'000, "the board rejected its baud rate");
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("IO device (slave 6) failed to configure after the domain: the board rejected its "
                                     "baud rate"),
              std::string::npos)
        << refused.message();
    EXPECT_GE(index_of("release_master"), 0);
    EXPECT_LT(index_of("configure_after_domain(pos=6)"), index_of("release_master"));
    EXPECT_EQ(index_of("slave_config_sdo16(pos=0,idx=0x2000"), -1) << "no servo startup SDO after the failure";
    EXPECT_EQ(index_of("master_activate"), -1);
    EXPECT_TRUE(bus_->io_devices().empty());
    EXPECT_TRUE(bus_->servo_devices().empty());
}

// The device knows why it cannot register its PDOs; that text is what the
// operator reads in initialize()'s refusal, with the slave it came from.
TEST_F(IoDeviceHooksTest, ARegisterPdosRefusalReachesInitializeWithItsOwnText) {
    const auto refused = init_bus(1'000'000, {}, "object 0x7000:01 is not in this board's mapping");
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("IO device (slave 6)"), std::string::npos) << refused.message();
    EXPECT_NE(refused.message().find("object 0x7000:01 is not in this board's mapping"), std::string::npos)
        << refused.message();
    EXPECT_GE(index_of("release_master"), 0);
}

// The slave's firmware maps a different layout than the device registers
// (field case: a tool board on older firmware): the refusal names the slave,
// its identity and every entry its mapping lacks, so the operator can tell a
// firmware mismatch from a wiring or master fault.
constexpr plem::fieldbus::ethercat::PdoEntryMapping kOtherBoardRx[] = {{0x7000, 0x02, 8}};
constexpr plem::fieldbus::ethercat::PdoMapping kOtherBoardRxPdos[] = {{0x1600, kOtherBoardRx, 1}};
constexpr plem::fieldbus::ethercat::SyncManagerAssignment kOtherBoardSyncs[] = {{2, true, kOtherBoardRxPdos, 1, false}};
constexpr plem::fieldbus::ethercat::SlavePdoMapping kOtherBoardLayout{kOtherBoardSyncs, 1};

TEST_F(IoDeviceHooksTest, AnEntryTheSlaveDoesNotMapIsNamedWithTheSlaveIdentity) {
    FakeEcrt::set_slave_scan(6, 0x3, kOtherBoardLayout);
    const auto refused = init_bus(1'000'000);
    ASSERT_FALSE(refused.is_ok());
    EXPECT_NE(refused.message().find("slave 6 (vendor 0x" + fmt::format("{:x}", layout::kSyntheticVendorId) +
                                     " product 0x50000000 revision 0x3) does not map 1 of the 1 PDO entries "
                                     "registered for it: 0x7000:01"),
              std::string::npos)
        << refused.message();
    EXPECT_NE(refused.message().find("ethercat -m 0 upload -p 6 0x100A 0 --type string"), std::string::npos)
        << refused.message();
    EXPECT_GE(index_of("release_master"), 0);
}

TEST_F(IoDeviceHooksTest, FactoryContextCarriesTheBusPeriod) {
    const auto init = init_bus(500'000);
    ASSERT_TRUE(init.is_ok()) << init.message();
    EXPECT_EQ(seen_period_ns_, 500'000);
}

TEST_F(IoDeviceHooksTest, EveryCycleNotesTheFrameThenReadsThenWrites) {
    const auto init = init_bus(1'000'000);
    ASSERT_TRUE(init.is_ok()) << init.message();
    start_loop();
    run(9);
    const std::vector<std::string>& cyclic = device_->events;
    ASSERT_EQ(cyclic.size(), 30u) << "10 cycles";
    for (size_t i = 0; i < cyclic.size(); i += 3) {
        EXPECT_EQ((std::vector<std::string>{cyclic[i], cyclic[i + 1], cyclic[i + 2]}),
                  (std::vector<std::string>{"frame", "read", "write"}))
            << "cycle " << i / 3 + 1;
    }
}
