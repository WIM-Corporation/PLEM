// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file pdo_capacity.hpp
/// @brief The fixed-array capacity walk over a PDO layout value (not installed).

#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "plem/drive/models/ethercat_slave_spec.hpp"

namespace plem::drive::models {

/// A count in a layout value that its fixed array cannot hold.
struct CapacityExcess {
    std::string field;  ///< under the layout: sync_count, syncs[s].pdo_count or syncs[s].pdos[p].entry_count
    size_t count;
    size_t capacity;
};

/// The first count `value` declares beyond its fixed array, in walk order
/// (sync_count, then per sync its pdo_count and each PDO's entry_count), or
/// nothing when every count fits. The counts index the arrays, so one beyond
/// them would be read past the end.
[[nodiscard]] inline std::optional<CapacityExcess> first_capacity_excess(const SlavePdoMappingSpec& value) {
    if (value.sync_count > SlavePdoMappingSpec::kMaxSyncManagers) {
        return CapacityExcess{"sync_count", value.sync_count, SlavePdoMappingSpec::kMaxSyncManagers};
    }
    for (size_t s = 0; s < value.sync_count; ++s) {
        const auto& sync = value.syncs[s];
        const std::string at = "syncs[" + std::to_string(s) + "]";
        if (sync.pdo_count > SyncManagerAssignmentSpec::kMaxPdos) {
            return CapacityExcess{at + ".pdo_count", sync.pdo_count, SyncManagerAssignmentSpec::kMaxPdos};
        }
        for (size_t p = 0; p < sync.pdo_count; ++p) {
            if (sync.pdos[p].entry_count > PdoMappingSpec::kMaxEntries) {
                return CapacityExcess{at + ".pdos[" + std::to_string(p) + "].entry_count", sync.pdos[p].entry_count,
                                      PdoMappingSpec::kMaxEntries};
            }
        }
    }
    return std::nullopt;
}

}  // namespace plem::drive::models
