// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <cstring>
#include "plem/fieldbus/ethercat/servo/pdo_handler.hpp"
#include "plem/fieldbus/ethercat/servo/pdo_types.hpp"

using namespace plem::fieldbus::ethercat;
using namespace plem::core::units;

class ServoPDOHandlerTest : public ::testing::Test {
  protected:
    void SetUp() override {
        std::memset(domain_buffer_, 0, sizeof(domain_buffer_));

        handler_no_flip_ = std::make_unique<ServoPDOHandler>(Pulse(1000), false);
        handler_with_flip_ = std::make_unique<ServoPDOHandler>(Pulse(-500), true);

        offsets_.controlword = 0;
        offsets_.modes_of_operation = 2;
        offsets_.target_position = 4;
        offsets_.target_velocity = 8;
        offsets_.target_torque = 12;
        offsets_.statusword = 14;
        offsets_.modes_of_operation_display = 16;
        offsets_.position_actual_value = 18;
        offsets_.velocity_actual_value = 22;
        offsets_.torque_actual_value = 26;
    }

    uint8_t domain_buffer_[256];  // Simulated EtherCAT domain
    ServoPDOOffsets offsets_;
    std::unique_ptr<ServoPDOHandler> handler_no_flip_;
    std::unique_ptr<ServoPDOHandler> handler_with_flip_;
};

TEST_F(ServoPDOHandlerTest, WritePDOAppliesOffsetOnlyWithoutFlip) {
    ServoPDOData data;
    data.controlword = 0x000F;
    data.modes_of_operation = 0x0A;  // Cyclic Sync Torque mode
    data.target_position = Pulse(5000);
    data.target_velocity = PulsePerSecond(100);
    data.target_torque = TorqueCount(50);

    auto& handler_offsets = handler_no_flip_->get_offsets();
    handler_offsets = offsets_;

    handler_no_flip_->write(data, domain_buffer_);

    uint16_t written_control = EC_READ_U16(domain_buffer_ + offsets_.controlword);
    EXPECT_EQ(written_control, 0x000F);

    int8_t written_mode = EC_READ_S8(domain_buffer_ + offsets_.modes_of_operation);
    EXPECT_EQ(written_mode, 0x0A);

    // Position should have offset applied: 5000 + 1000 = 6000
    int32_t written_position = EC_READ_S32(domain_buffer_ + offsets_.target_position);
    EXPECT_EQ(written_position, 6000);

    // Velocity and torque should not be flipped
    int32_t written_velocity = EC_READ_S32(domain_buffer_ + offsets_.target_velocity);
    EXPECT_EQ(written_velocity, 100);

    int16_t written_torque = EC_READ_S16(domain_buffer_ + offsets_.target_torque);
    EXPECT_EQ(written_torque, 50);
}

TEST_F(ServoPDOHandlerTest, WritePDOEncodesFlippedPositionAsOffsetMinusValue) {
    ServoPDOData data;
    data.target_position = Pulse(2000);
    data.target_velocity = PulsePerSecond(100);
    data.target_torque = TorqueCount(50);

    auto& handler_offsets = handler_with_flip_->get_offsets();
    handler_offsets = offsets_;

    handler_with_flip_->write(data, domain_buffer_);

    // Flipped position encode is offset - value: -500 - 2000 = -2500.
    int32_t written_position = EC_READ_S32(domain_buffer_ + offsets_.target_position);
    EXPECT_EQ(written_position, -2500);

    // Velocity should be flipped
    int32_t written_velocity = EC_READ_S32(domain_buffer_ + offsets_.target_velocity);
    EXPECT_EQ(written_velocity, -100);

    // Torque should be flipped
    int16_t written_torque = EC_READ_S16(domain_buffer_ + offsets_.target_torque);
    EXPECT_EQ(written_torque, -50);
}

TEST_F(ServoPDOHandlerTest, ReadPDORemovesOffsetOnlyWithoutFlip) {
    EC_WRITE_U16(domain_buffer_ + offsets_.statusword, 0x0237);
    EC_WRITE_U8(domain_buffer_ + offsets_.modes_of_operation_display, 0x0A);
    EC_WRITE_S32(domain_buffer_ + offsets_.position_actual_value, 3000);  // Raw position with offset
    EC_WRITE_S32(domain_buffer_ + offsets_.velocity_actual_value, 200);
    EC_WRITE_S16(domain_buffer_ + offsets_.torque_actual_value, 75);

    auto& handler_offsets = handler_no_flip_->get_offsets();
    handler_offsets = offsets_;

    ServoPDOData data;
    handler_no_flip_->read(data, domain_buffer_);

    EXPECT_EQ(data.statusword, 0x0237);
    EXPECT_EQ(data.modes_of_operation_display, 0x0A);

    // Position should have offset removed: 3000 - 1000 = 2000
    EXPECT_EQ(data.position_actual_value, Pulse(2000));

    // Velocity and torque should not be flipped
    EXPECT_EQ(data.velocity_actual_value, PulsePerSecond(200));
    EXPECT_EQ(data.torque_actual_value, TorqueCount(75));
}

// 0x6061 Modes of operation display is INTEGER8: 0xFF reads as −1.
TEST_F(ServoPDOHandlerTest, ReadsTheModeDisplaySigned) {
    EC_WRITE_U8(domain_buffer_ + offsets_.modes_of_operation_display, 0xFF);
    handler_no_flip_->get_offsets() = offsets_;
    ServoPDOData data;
    handler_no_flip_->read(data, domain_buffer_);
    EXPECT_EQ(static_cast<int>(data.modes_of_operation_display), -1);
}

TEST_F(ServoPDOHandlerTest, ReadPDOUnflipsAndRemovesOffsetWithFlip) {
    EC_WRITE_S32(domain_buffer_ + offsets_.position_actual_value, -2000);  // Raw flipped position
    EC_WRITE_S32(domain_buffer_ + offsets_.velocity_actual_value, -150);
    EC_WRITE_S16(domain_buffer_ + offsets_.torque_actual_value, -60);

    auto& handler_offsets = handler_with_flip_->get_offsets();
    handler_offsets = offsets_;

    ServoPDOData data;
    handler_with_flip_->read(data, domain_buffer_);

    // Position should be unflipped and have offset removed: -(-2000 - (-500)) = 1500
    EXPECT_EQ(data.position_actual_value, Pulse(1500));

    // Velocity should be unflipped
    EXPECT_EQ(data.velocity_actual_value, PulsePerSecond(150));

    // Torque should be unflipped
    EXPECT_EQ(data.torque_actual_value, TorqueCount(60));
}

TEST_F(ServoPDOHandlerTest, PDOReadWriteUsesCorrectOffsetsForSeparateTargetAndActual) {
    // In EtherCAT, write() sets target values and read() gets actual values
    // They use different PDO mappings, so we need to test them separately

    // Master-to-servo commands
    ServoPDOData write_data;
    write_data.controlword = 0x001F;
    write_data.modes_of_operation = 0x0A;  // CST mode
    write_data.target_position = Pulse(12345);
    write_data.target_velocity = PulsePerSecond(678);
    write_data.target_torque = TorqueCount(90);

    // Use a handler with no flip/offset for simple test
    ServoPDOHandler handler(Pulse(0), false);
    auto& handler_offsets = handler.get_offsets();
    handler_offsets = offsets_;

    handler.write(write_data, domain_buffer_);

    EXPECT_EQ(EC_READ_U16(domain_buffer_ + offsets_.controlword), 0x001F);
    EXPECT_EQ(EC_READ_U8(domain_buffer_ + offsets_.modes_of_operation), 0x0A);
    EXPECT_EQ(EC_READ_S32(domain_buffer_ + offsets_.target_position), 12345);
    EXPECT_EQ(EC_READ_S32(domain_buffer_ + offsets_.target_velocity), 678);
    EXPECT_EQ(EC_READ_S16(domain_buffer_ + offsets_.target_torque), 90);

    // Simulated servo response at actual_* offsets
    EC_WRITE_U16(domain_buffer_ + offsets_.statusword, 0x0637);
    EC_WRITE_U8(domain_buffer_ + offsets_.modes_of_operation_display, 0x0A);
    EC_WRITE_S32(domain_buffer_ + offsets_.position_actual_value, 12340);
    EC_WRITE_S32(domain_buffer_ + offsets_.velocity_actual_value, 675);
    EC_WRITE_S16(domain_buffer_ + offsets_.torque_actual_value, 88);

    ServoPDOData read_data;
    handler.read(read_data, domain_buffer_);

    EXPECT_EQ(read_data.statusword, 0x0637);
    EXPECT_EQ(read_data.modes_of_operation_display, 0x0A);
    EXPECT_EQ(read_data.position_actual_value, Pulse(12340));
    EXPECT_EQ(read_data.velocity_actual_value, PulsePerSecond(675));
    EXPECT_EQ(read_data.torque_actual_value, TorqueCount(88));
}

// Pulse::to_hardware/from_hardware widen to int64 and saturate at the int32
// register boundary. Signed overflow in INT32_MAX + joint_offset would be
// undefined behavior and can sign-flip a servo command full-scale.
TEST_F(ServoPDOHandlerTest, WritePDOSaturatesAtInt32BoundaryInsteadOfWrapping) {
    ServoPDOData data;

    // int32 position boundaries
    data.target_position = Pulse(INT32_MAX);
    data.position_actual_value = Pulse(INT32_MIN);

    // int16 torque boundaries
    data.target_torque = TorqueCount(INT16_MAX);
    data.torque_actual_value = TorqueCount(INT16_MIN);

    auto& handler_offsets = handler_no_flip_->get_offsets();
    handler_offsets = offsets_;

    handler_no_flip_->write(data, domain_buffer_);

    // INT32_MAX + offset(1000) saturates to INT32_MAX, never wraps to negative.
    int32_t written_pos = EC_READ_S32(domain_buffer_ + offsets_.target_position);
    EXPECT_EQ(written_pos, INT32_MAX);

    int16_t written_torque = EC_READ_S16(domain_buffer_ + offsets_.target_torque);
    EXPECT_EQ(written_torque, INT16_MAX);

    // Simulated servo feedback at actual_* offsets
    EC_WRITE_S32(domain_buffer_ + offsets_.position_actual_value, INT32_MIN);
    EC_WRITE_S16(domain_buffer_ + offsets_.torque_actual_value, INT16_MIN);

    ServoPDOData read_data;
    handler_no_flip_->read(read_data, domain_buffer_);

    // INT32_MIN - offset(1000) saturates to INT32_MIN, never wraps to positive.
    EXPECT_EQ(read_data.position_actual_value, Pulse(INT32_MIN));
    EXPECT_EQ(read_data.torque_actual_value, TorqueCount(INT16_MIN));
}

TEST_F(ServoPDOHandlerTest, PDOHandlerPassthroughValuesWithZeroOffsetNoFlip) {
    ServoPDOHandler zero_handler(Pulse(0), false);
    auto& handler_offsets = zero_handler.get_offsets();
    handler_offsets = offsets_;

    ServoPDOData data;
    data.target_position = Pulse(5000);
    data.target_velocity = PulsePerSecond(100);
    data.target_torque = TorqueCount(50);

    zero_handler.write(data, domain_buffer_);

    // Position should be unchanged (no offset, no flip)
    int32_t written_position = EC_READ_S32(domain_buffer_ + offsets_.target_position);
    EXPECT_EQ(written_position, 5000);

    EC_WRITE_S32(domain_buffer_ + offsets_.position_actual_value, 4999);
    EC_WRITE_S32(domain_buffer_ + offsets_.velocity_actual_value, 98);
    EC_WRITE_S16(domain_buffer_ + offsets_.torque_actual_value, 49);

    ServoPDOData read_data;
    zero_handler.read(read_data, domain_buffer_);

    // Actual values should be unchanged (no offset, no flip)
    EXPECT_EQ(read_data.position_actual_value, Pulse(4999));
    EXPECT_EQ(read_data.velocity_actual_value, PulsePerSecond(98));
    EXPECT_EQ(read_data.torque_actual_value, TorqueCount(49));
}

// Unmapped-offset guards

// An unmapped field must be skipped, never aliased to offset 0: a drive that
// maps no target velocity (a real vendor shape) must not have its controlword
// region clobbered by the velocity write.
TEST_F(ServoPDOHandlerTest, WriteSkipsUnmappedFieldsInsteadOfAliasingOffsetZero) {
    ServoPDOOffsets offsets = offsets_;
    offsets.target_velocity = kUnmappedPdoOffset;  // slave maps no 0x60FF
    handler_no_flip_->get_offsets() = offsets;

    ServoPDOData data;
    data.controlword = 0x000F;
    data.target_velocity = PulsePerSecond(123456);
    std::memset(domain_buffer_, 0xAA, sizeof(domain_buffer_));
    handler_no_flip_->write(data, domain_buffer_);

    // The mapped controlword landed; the unmapped velocity region (offset 8)
    // still carries the sentinel pattern: nothing wrote through it.
    EXPECT_EQ(EC_READ_U16(domain_buffer_ + offsets.controlword), 0x000F);
    EXPECT_EQ(EC_READ_U32(domain_buffer_ + offsets_.target_velocity), 0xAAAAAAAAu);
}

TEST_F(ServoPDOHandlerTest, ReadLeavesUnmappedFieldsAtTheirPriorValues) {
    ServoPDOOffsets offsets = offsets_;
    offsets.velocity_actual_value = kUnmappedPdoOffset;
    handler_no_flip_->get_offsets() = offsets;

    ServoPDOData data;
    data.velocity_actual_value = PulsePerSecond(777);  // prior value must survive
    std::memset(domain_buffer_, 0, sizeof(domain_buffer_));
    handler_no_flip_->read(data, domain_buffer_);
    EXPECT_EQ(data.velocity_actual_value, PulsePerSecond(777));
}
