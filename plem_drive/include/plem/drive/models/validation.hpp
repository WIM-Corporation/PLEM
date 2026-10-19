// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file validation.hpp
/// @brief Checks over the slave and axis spec values, for any program that
/// builds a bus from them (DriveHost runs them through its own value gate).
/// Run them before bus_setup.hpp's functions. Each returns ok or an error
/// carrying the complete text; `tag` is the caller's name ("[<tag>] model
/// spec …").

#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include <plem/core/result.hpp>

#include "plem/drive/models/ethercat_slave_spec.hpp"

namespace plem::drive::models {

/// @brief Per-slave rules over the declared value range only: ring position
/// (>= 0 only, no upper bound), identity, duplicate/dead declarations,
/// PDO-layout fixed-array capacities (SM/PDO structure is validate_pdo_mapping's).
/// Pure: no container, no DOF; `axes` is consulted only for "hosts an axis".
/// Bus-owned and not checked here: axes sharing one slave agreeing on identity
/// and layout object (bus.cpp initialize). Returns ok, or an error with
/// the complete text. `tag` is the caller's name ("[<tag>] model
/// spec …"). Message bytes are a contract: tests pin them verbatim.
core::Result<void> check_slaves(std::span<const models::SlaveSpec> slaves, std::span<const models::AxisSpec> axes,
                                std::string_view tag);

/// @brief Per-axis rules: channel map, od_offset 0x800 grid (no upper bound),
/// served mode, declared mode display, mandatory CiA 402 profile, a declared
/// ErrorCode role (index != 0) with width != 0. Pure, same contract as check_slaves. Bus-owned and not checked
/// here: cross-axis (slave, channel) uniqueness, od_offset / ErrorCode-index
/// OD overflow, the fixed 16-bit ErrorCode width (bus.cpp initialize, schema §8).
core::Result<void> check_axes(std::span<const models::AxisSpec> axes, std::string_view tag);

/// @brief Model-name rules over the fixed char array: NUL-terminated inside
/// the array and not blank. Pure, same contract as check_slaves.
core::Result<void> check_model_name(std::span<const char> model_name, std::string_view tag);

}  // namespace plem::drive::models
