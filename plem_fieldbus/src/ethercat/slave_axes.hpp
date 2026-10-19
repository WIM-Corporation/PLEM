// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ethercat/internal_visibility.hpp"
#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/pdo_layout_validation.hpp"

// Package-private (deliberately not an installed header): how a servo list maps
// onto bus slaves, shared by the bus (bring-up) and the pure configuration
// validator.

namespace plem::fieldbus::ethercat {

/// Bus position of the slave hosting servo `i` (ServoInfo::slave_position, or
/// the servo's own index when the model derives it: one axis per slave).
PLEM_FIELDBUS_INTERNAL inline int32_t slave_of(const std::vector<ServoInfo>& infos, size_t i) noexcept {
    return infos[i].slave_position >= 0 ? infos[i].slave_position : static_cast<int32_t>(i);
}

/// Index of the first servo in the list hosted by servo `i`'s slave (`i`
/// itself when it is the first); that servo speaks for the slave in every
/// per-slave step; the later axes of the same slave are grouped under it
/// (axes_of_slave). Allocation-free: also used on the RT path.
PLEM_FIELDBUS_INTERNAL inline size_t first_axis_index_of_slave(const std::vector<ServoInfo>& infos, size_t i) noexcept {
    const int32_t slave = slave_of(infos, i);
    size_t j = 0;
    while (slave_of(infos, j) != slave) {
        ++j;
    }
    return j;
}

/// Every axis hosted by `slave`, in servo-list (joint) order, not contiguous
/// in general (a multi-axis model may wire joint 0 to channel 2 of slave 0 and
/// joint 3 to channel 1), as the layout validator's requirements, plus the
/// servo indices ("0,3") for messages.
struct PLEM_FIELDBUS_INTERNAL SlaveAxes {
    std::vector<AxisPdoRequirements> axes;
    std::string members;
};

PLEM_FIELDBUS_INTERNAL inline SlaveAxes axes_of_slave(const std::vector<ServoInfo>& infos, int32_t slave) {
    SlaveAxes group;
    for (size_t j = 0; j < infos.size(); ++j) {
        if (slave_of(infos, j) != slave) {
            continue;
        }
        const ServoInfo& axis = infos[j];
        group.axes.push_back(AxisPdoRequirements{
            .od_offset = axis.od_offset, .mode = axis.modes_of_operation, .error_code_pdo = axis.error_code_pdo});
        group.members += (group.members.empty() ? "" : ",") + std::to_string(j);
    }
    return group;
}

}  // namespace plem::fieldbus::ethercat
