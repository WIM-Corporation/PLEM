// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file single_axis_cst_layout.hpp
/// @brief Test-only PDO layout for a one-axis CST slave at channel 1: SM0/SM1
/// mailbox, SM2 RxPDO 0x1600 and SM3 TxPDO 0x1A00 each carrying exactly the
/// standard entry set the core registers, no gaps, no vendor objects, no
/// ErrorCode (a test declaring `error_code_pdo` points its axis at a layout
/// that maps it). The synthetic model/ServoInfo fixtures declare this, so the
/// declared path is what they exercise; the scanned path has its own pins
/// (test_ethercat_bus_scanned_fake_master.cpp: the fake serves this same
/// layout as a scan).

#pragma once

#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/pdo_layout.hpp"

namespace plem::fieldbus::test {

using ethercat::PdoEntryMapping;
using ethercat::PdoMapping;
using ethercat::ServoSpec;
using ethercat::SlavePdoMapping;
using ethercat::SyncManagerAssignment;

inline constexpr PdoEntryMapping kSingleAxisCstRx[] = {
    {0x6040, 0x00, 16}, {0x607a, 0x00, 32}, {0x60ff, 0x00, 32}, {0x6071, 0x00, 16}, {0x6060, 0x00, 8},
};
inline constexpr PdoEntryMapping kSingleAxisCstTx[] = {
    {0x6041, 0x00, 16}, {0x6064, 0x00, 32}, {0x606c, 0x00, 32}, {0x6077, 0x00, 16}, {0x6061, 0x00, 8},
};
inline constexpr PdoMapping kSingleAxisCstRxPdos[] = {{0x1600, kSingleAxisCstRx, 5}};
inline constexpr PdoMapping kSingleAxisCstTxPdos[] = {{0x1a00, kSingleAxisCstTx, 5}};
inline constexpr SyncManagerAssignment kSingleAxisCstSyncs[] = {
    {0, true, nullptr, 0, false},
    {1, false, nullptr, 0, false},
    {2, true, kSingleAxisCstRxPdos, 1, true},
    {3, false, kSingleAxisCstTxPdos, 1, false},
};

/// One-axis CST slave layout (channel 1). Static storage: outlives any bus.
inline constexpr SlavePdoMapping kSingleAxisCstLayout{kSingleAxisCstSyncs, 4};

/// The synthetic drive the fixtures declare with that layout: an identity and
/// motor constants made up for tests. They belong to no vendor and no shipped
/// drive (that the id is unassigned by the ETG has not been checked). Test
/// files that cannot include this header (YAML, Python) repeat the literals
/// and name these constants.
inline constexpr uint32_t kSyntheticVendorId = 0x00c0ffee;
inline constexpr uint32_t kSyntheticProductCode = 0x00001001;
/// {gear_ratio, torque_constant [Nm/A], encoder_resolution, counts_per_amp [counts/A]}.
inline constexpr ServoSpec kSyntheticServoSpec{100.0, 0.12, 65536, 50.0};

}  // namespace plem::fieldbus::test
