// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <type_traits>
#include "plem/fieldbus/ethercat/servo/pdo_types.hpp"

using namespace plem::fieldbus::ethercat;

TEST(ServoPDOTypes, ServoPDOOffsetsDefaultConstructorInitializesToUnmapped) {
    // A default-constructed offsets object is UNMAPPED ("nothing mapped"): zero
    // is a legitimate offset and cannot double as the absent marker (writing
    // through it would alias the controlword).
    ServoPDOOffsets offsets;

    // RxPDO offsets
    EXPECT_EQ(offsets.controlword, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.modes_of_operation, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.target_position, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.target_velocity, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.target_torque, kUnmappedPdoOffset);

    // TxPDO offsets
    EXPECT_EQ(offsets.statusword, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.modes_of_operation_display, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.position_actual_value, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.velocity_actual_value, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.torque_actual_value, kUnmappedPdoOffset);
    EXPECT_EQ(offsets.error_code, kUnmappedPdoOffset);
}

// The RT loop copies the offsets as a block of 11 words.
TEST(ServoPDOTypes, OffsetsAreTriviallyCopyableWords) {
    EXPECT_TRUE(std::is_trivially_copyable<ServoPDOOffsets>::value);
    EXPECT_EQ(sizeof(ServoPDOOffsets), 11 * sizeof(uint32_t));
}

TEST(ServoPDOTypes, OperationModeEnumValuesAreCorrect) {
    EXPECT_EQ(static_cast<int8_t>(OperationMode::NO_MODE), 0);
    EXPECT_EQ(static_cast<int8_t>(OperationMode::CYCLIC_SYNC_POSITION), 8);
    EXPECT_EQ(static_cast<int8_t>(OperationMode::CYCLIC_SYNC_VELOCITY), 9);
    EXPECT_EQ(static_cast<int8_t>(OperationMode::CYCLIC_SYNC_TORQUE), 10);
}
