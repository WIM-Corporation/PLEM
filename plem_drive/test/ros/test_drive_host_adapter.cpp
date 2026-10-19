// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_drive_host_adapter.cpp
/// @brief `make_drive_host<N>` for every axis count the schema admits.
///
/// Two separate things live here.
///
/// The refusal test is a wiring pin, not a behaviour verification: a
/// default-constructed spec fails its own value gate, so nothing reaches a bus,
/// a master or a drive. It proves the adapter exists and links for every N the
/// schema admits, and that the host's refusal
/// arrives unedited. The loop itself is proven on the fieldbus fake-master
/// target and on the bench.
///
/// The name test is the only pin on the link between a user's spec and the
/// joint labels that reach `~/joint_states`. It builds the adapter with no host
/// at all: axis_names() is the one call that does not forward.

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <plem/drive/drive_model_file.hpp>
#include <plem/drive/models/drive_model_spec.hpp>
#include <plem/fieldbus/ethercat/bus.hpp>

#include "ros/drive_host_adapter.hpp"

namespace plem::drive::ros {
namespace {

using plem::drive::DriveHostParams;
using plem::drive::models::DriveModelSpec;
using plem::fieldbus::ethercat::EtherCATBus;

/// Instantiates `make_drive_host<N>` for one N and pins that the value gate's
/// refusal arrives unedited.
template <size_t N>
void expect_value_gate_refusal() {
    auto result = make_drive_host<N>(DriveModelSpec<N>{}, std::make_shared<EtherCATBus>(0), DriveHostParams{});
    ASSERT_FALSE(result) << "N = " << N;
    EXPECT_NE(result.message().find("[DriveHost] model spec"), std::string::npos)
        << "N = " << N << ": " << result.message();
}

TEST(DriveHostAdapter, LinksAndCarriesTheRefusalForEveryAxisCountTheSchemaAdmits) {
    // Every N, not just the ends: a missing explicit instantiation in the
    // middle of the range would otherwise surface only at a user's link.
    expect_value_gate_refusal<1>();
    expect_value_gate_refusal<2>();
    expect_value_gate_refusal<3>();
    expect_value_gate_refusal<4>();
    expect_value_gate_refusal<5>();
    expect_value_gate_refusal<6>();
    expect_value_gate_refusal<7>();
    expect_value_gate_refusal<8>();
}

TEST(DriveHostAdapter, AxisLabelsComeFromTheSpecInOrder) {
    DriveModelSpec<2> spec{};
    const char first[] = "axis_a";
    const char second[] = "axis_b_2";
    std::memcpy(spec.axis_names[0].data(), first, sizeof(first));
    std::memcpy(spec.axis_names[1].data(), second, sizeof(second));

    const detail::DriveHostAdapter<2> adapter(nullptr, spec);
    const auto names = adapter.axis_names();
    EXPECT_EQ(std::vector<std::string>(names.begin(), names.end()), (std::vector<std::string>{"axis_a", "axis_b_2"}));
}

TEST(DriveHostAdapter, AxisModesAreTheDeclaredModeBytesInOrder) {
    DriveModelSpec<3> spec{};
    spec.axes[0].modes_of_operation = plem::fieldbus::OperationMode::CYCLIC_SYNC_TORQUE;
    spec.axes[1].modes_of_operation = plem::fieldbus::OperationMode::CYCLIC_SYNC_POSITION;
    spec.axes[2].modes_of_operation = plem::fieldbus::OperationMode::CYCLIC_SYNC_VELOCITY;
    const detail::DriveHostAdapter<3> adapter(nullptr, spec);
    const auto modes = adapter.modes_of_operation();
    EXPECT_EQ(std::vector<int8_t>(modes.begin(), modes.end()), (std::vector<int8_t>{10, 8, 9}));
}

TEST(DriveHostAdapter, TheInstalledExampleLoadsAsAScannedDrive) {
    // The user's starting file (installed at share/plem_drive/
    // examples/) loads, and its `pdo_mapping: scanned` reads as sync_count 0.
    const auto loaded = plem::drive::load_drive_model_file(PLEM_DRIVE_EXAMPLE_YAML);
    ASSERT_TRUE(loaded) << loaded.message();
    const auto& spec = std::get<DriveModelSpec<1>>(loaded.value());
    EXPECT_EQ(spec.slaves[0].pdo_mapping.sync_count, 0u);
}

}  // namespace
}  // namespace plem::drive::ros
