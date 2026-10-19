// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file io_device_factory.hpp
/// @brief IO device factory contract: the only way an IO slave gets a device.
///
/// EtherCATBus::initialize() calls the injected factory once per IOInfo, after
/// the master is acquired, and registers the result (register_io). No ecrt.h
/// on purpose (forward-declared master handle), so a caller names these types
/// without master headers.

#pragma once

#include <cstdint>
#include <functional>
#include <memory>

// Matches ecrt.h's own `typedef struct ec_master ec_master_t;`; a duplicate
// typedef of the same type is well-formed when both are visible.
typedef struct ec_master ec_master_t;  // NOLINT(modernize-use-using): mirrors the C header

namespace plem::fieldbus::ethercat {

class DeviceBase;
struct IOInfo;

/// @brief Creation inputs for one IO device: what DeviceBase
/// construction requires (the bus's live master handle + the device's bus
/// slave position + its declared IOInfo) plus the bus period it will run at.
///
/// Lifetime contract (non-owning view; the type system does not enforce it): valid
/// only for the duration of the factory call.
/// The factory must not store the context, the master pointer, or the IOInfo
/// reference; the returned device owns only state it took from the
/// constructor arguments.
struct IODeviceContext {
    ec_master_t* master;
    uint32_t slave_position;
    const IOInfo& io_info;
    int64_t cycle_period_ns;  ///< the bus period the device will be cycled at
};

/// @brief IO device factory. Called once per IO during
/// EtherCATBus::initialize(); returning nullptr or throwing makes
/// initialize() refuse (the master released, the reason in its message).
using IODeviceFactory = std::function<std::shared_ptr<DeviceBase>(const IODeviceContext&)>;

}  // namespace plem::fieldbus::ethercat
