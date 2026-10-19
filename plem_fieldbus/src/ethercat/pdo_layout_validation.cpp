// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/ethercat/pdo_layout_validation.hpp"

#include <vector>

#include <fmt/format.h>

namespace plem::fieldbus::ethercat {

namespace {

/// One object the core registers per axis. `bit_length` is the fixed width
/// ServoPDOHandler reads/writes it with; the two tables must stay in
/// lockstep (kept by hand).
///
/// Standard VAR objects are found by index and held to subindex 0
/// (`key_has_subindex == false`: a wrong subindex is a typo worth naming).
/// The ErrorCode role is addressed by its declared (index, subindex):
/// 0x2000:1 and 0x2000:2 are different objects, so only entries at that
/// subindex count as its mapping (`key_has_subindex == true`).
struct RoleSpec {
    const char* name;
    uint16_t base_index;
    uint8_t subindex;
    bool key_has_subindex;
    uint8_t bit_length;
    bool output;    ///< true = RxPDO (master → slave)
    bool required;  ///< must be mapped in every mode (the mode's own target is added per axis)
};

// Required rationale (the current PLEM execution/observation contract, not a
// universal CiA 402 minimum): CW/SW drive the sequencer and the enable gate,
// MoO is written every cycle, AP/AV feed the controller and the limit checks
// (joint_data_plane → the host control thread), AT/MoOD are published without a
// validity flag (rt_cycle_monitor) and MoOD is the mode-acceptance gate input.
// The target the declared mode writes every cycle is required (CST 0x6071,
// CSP 0x607A, CSV 0x60FF); the other two are optional, written when mapped
// (0 torque/velocity, the position mirror), skipped when not.
constexpr RoleSpec kStandardRoles[] = {
    {"Controlword", 0x6040, 0, false, 16, true, true},
    {"Modes of operation", 0x6060, 0, false, 8, true, true},
    {"Target position", 0x607A, 0, false, 32, true, false},
    {"Target velocity", 0x60FF, 0, false, 32, true, false},
    {"Target torque", 0x6071, 0, false, 16, true, false},
    {"Statusword", 0x6041, 0, false, 16, false, true},
    {"Modes of operation display", 0x6061, 0, false, 8, false, true},
    {"Position actual value", 0x6064, 0, false, 32, false, true},
    {"Velocity actual value", 0x606C, 0, false, 32, false, true},
    {"Torque actual value", 0x6077, 0, false, 16, false, true},
};

/// The target object the declared mode writes every cycle (0 for a mode
/// without a command path; check_axis refuses those first).
constexpr uint16_t mode_target(OperationMode mode) noexcept {
    switch (mode) {
        case OperationMode::CYCLIC_SYNC_TORQUE:
            return 0x6071;
        case OperationMode::CYCLIC_SYNC_POSITION:
            return 0x607A;
        case OperationMode::CYCLIC_SYNC_VELOCITY:
            return 0x60FF;
        case OperationMode::NO_MODE:
            break;
    }
    return 0;
}

/// An entry located in the layout: its sync manager and its start bit within
/// that SM's process image.
struct Located {
    const SyncManagerAssignment* sm;
    uint32_t start_bit;
    const PdoEntryMapping* entry;
};

/// Visit every entry with its SM and start bit. Bits accumulate across the
/// PDOs of one sync manager in declaration order (IgH packs them
/// contiguously, so a PDO boundary does not realign) and restart at 0 for
/// each SM. Gaps and vendor entries count. Requires check_structure() to have
/// passed (no null list is dereferenced otherwise).
template <typename Visit>
void for_each_entry(const SlavePdoMapping& layout, Visit&& visit) {
    for (uint8_t s = 0; s < layout.sync_count; ++s) {
        const SyncManagerAssignment& sm = layout.syncs[s];
        uint32_t bit = 0;
        for (uint8_t p = 0; p < sm.pdo_count; ++p) {
            const PdoMapping& pdo = sm.pdos[p];
            for (uint8_t e = 0; e < pdo.entry_count; ++e) {
                visit(Located{&sm, bit, &pdo.entries[e]});
                bit += pdo.entries[e].bit_length;
            }
        }
    }
}

std::string check_structure(const SlavePdoMapping& layout) {
    if (layout.sync_count > 0 && layout.syncs == nullptr) {
        return fmt::format("sync_count {} with null syncs", layout.sync_count);
    }
    for (uint8_t s = 0; s < layout.sync_count; ++s) {
        const SyncManagerAssignment& sm = layout.syncs[s];
        if (sm.sm_index >= kSyncManagerCount) {
            return fmt::format("syncs[{}]: sm_index {} exceeds the sync-manager index range (0..{})", s, sm.sm_index,
                               kSyncManagerCount - 1);
        }
        for (uint8_t prev = 0; prev < s; ++prev) {
            if (layout.syncs[prev].sm_index == sm.sm_index) {
                return fmt::format("sync manager {} declared twice", sm.sm_index);
            }
        }
        if (sm.pdo_count > 0 && sm.pdos == nullptr) {
            return fmt::format("sync manager {}: pdo_count {} with null pdos", sm.sm_index, sm.pdo_count);
        }
        for (uint8_t p = 0; p < sm.pdo_count; ++p) {
            const PdoMapping& pdo = sm.pdos[p];
            if (pdo.index == 0) {
                return fmt::format(
                    "sync manager {}: PDO index 0 at pdos[{}] — gaps are entry-level; a PDO needs its "
                    "mapping index (e.g. 0x1600/0x1a00)",
                    sm.sm_index, p);
            }
            const bool in_rx_range = pdo.index >= 0x1600 && pdo.index <= 0x17FF;
            const bool in_tx_range = pdo.index >= 0x1A00 && pdo.index <= 0x1BFF;
            if (sm.output ? !in_rx_range : !in_tx_range) {
                return fmt::format("PDO 0x{:04x} in {} sync manager {} is outside the {} mapping range ({})", pdo.index,
                                   sm.output ? "output" : "input", sm.sm_index, sm.output ? "RxPDO" : "TxPDO",
                                   sm.output ? "0x1600–0x17ff" : "0x1a00–0x1bff");
            }
            if (pdo.entry_count == 0) {
                return fmt::format(
                    "PDO 0x{:04x}: no entries — the master reads entry_count 0 as \"use the slave's default "
                    "mapping\"; a declaration must list the PDO's entries, and a scan that shows none holds nothing "
                    "the core could register",
                    pdo.index);
            }
            if (pdo.entries == nullptr) {
                return fmt::format("PDO 0x{:04x}: entry_count {} with null entries", pdo.index, pdo.entry_count);
            }
            for (uint8_t e = 0; e < pdo.entry_count; ++e) {
                if (pdo.entries[e].bit_length == 0) {
                    return fmt::format("PDO 0x{:04x} entry {}: bit_length 0", pdo.index, e);
                }
            }
        }
    }
    // Duplicate PDO index across the whole layout (a PDO is assigned to
    // exactly one sync manager). Lists are non-null from here on.
    for (uint8_t s = 0; s < layout.sync_count; ++s) {
        for (uint8_t p = 0; p < layout.syncs[s].pdo_count; ++p) {
            const uint16_t index = layout.syncs[s].pdos[p].index;
            for (uint8_t t = 0; t <= s; ++t) {
                const uint8_t limit = (t == s) ? p : layout.syncs[t].pdo_count;
                for (uint8_t q = 0; q < limit; ++q) {
                    if (layout.syncs[t].pdos[q].index == index) {
                        return fmt::format(
                            "PDO 0x{:04x} more than once — a PDO is assigned to exactly one sync manager", index);
                    }
                }
            }
        }
    }
    return {};
}

/// One (axis, role) requirement resolved to the address the core registers:
/// the key for the cross-axis collision check. Only mapped roles are recorded.
struct Resolved {
    size_t axis;
    const char* role;
    uint32_t index;
    uint8_t subindex;
};

/// Locate and check one role for one axis; on success records its address.
std::string check_role(const SlavePdoMapping& layout, size_t axis_no, const AxisPdoRequirements& axis,
                       const RoleSpec& role, std::vector<Resolved>& resolved) {
    const uint32_t index = static_cast<uint32_t>(role.base_index) + axis.od_offset;
    const auto where = [&] {
        return fmt::format("axis {} (od_offset 0x{:04x}): {}", axis_no, axis.od_offset, role.name);
    };
    if (index > 0xFFFF) {
        return fmt::format("{} 0x{:04x} + offset overflows the object dictionary", where(), role.base_index);
    }
    const auto address = fmt::format("0x{:04x}:{:02x}", index, role.subindex);

    Located first{};
    size_t match_count = 0;
    for_each_entry(layout, [&](const Located& at) {
        if (at.entry->index == index && (!role.key_has_subindex || at.entry->subindex == role.subindex)) {
            if (match_count++ == 0) {
                first = at;
            }
        }
    });
    if (match_count == 0) {
        return role.required ? fmt::format("{} {} is a required role and is not mapped by the layout", where(), address)
                             : std::string{};
    }
    if (match_count > 1) {
        return fmt::format("{} {} is mapped more than once ({} entries) — registration binds the first only", where(),
                           address, match_count);
    }
    if (first.entry->subindex != role.subindex) {
        return fmt::format("{} 0x{:04x} is mapped at subindex {} but the core registers subindex {}", where(), index,
                           first.entry->subindex, role.subindex);
    }
    if (first.entry->bit_length != role.bit_length) {
        return fmt::format("{} {} is mapped {} bits wide but the core reads/writes it as {} bits", where(), address,
                           first.entry->bit_length, role.bit_length);
    }
    if (first.sm->output != role.output) {
        return fmt::format("{} {} sits in {} sync manager {} but is a {} object — direction mismatch", where(), address,
                           first.sm->output ? "output" : "input", first.sm->sm_index,
                           role.output ? "master→slave" : "slave→master");
    }
    if (first.start_bit % 8 != 0) {
        return fmt::format("{} {} starts at bit {} of sync manager {} — not on a byte boundary", where(), address,
                           first.start_bit, first.sm->sm_index);
    }
    resolved.push_back(Resolved{axis_no, role.name, index, role.subindex});
    return {};
}

std::string check_axis(const SlavePdoMapping& layout, size_t axis_no, const AxisPdoRequirements& axis,
                       std::vector<Resolved>& resolved) {
    if (!operation_mode_supported(axis.mode)) {
        return fmt::format(
            "axis {} (od_offset 0x{:04x}): unsupported mode of operation {} — only CST (10), CSP (8) and CSV (9) "
            "are supported",
            axis_no, axis.od_offset, static_cast<int>(axis.mode));
    }
    for (RoleSpec role : kStandardRoles) {
        role.required = role.required || role.base_index == mode_target(axis.mode);
        if (std::string error = check_role(layout, axis_no, axis, role, resolved); !error.empty()) {
            return error;
        }
    }
    if (axis.error_code_pdo.index != 0) {
        const RoleSpec error_code{"Error code", axis.error_code_pdo.index,      axis.error_code_pdo.subindex,
                                  true,         axis.error_code_pdo.bit_length, false,
                                  true};
        if (std::string error = check_role(layout, axis_no, axis, error_code, resolved); !error.empty()) {
            return error;
        }
    }
    return {};
}

}  // namespace

core::Result<void> validate_pdo_mapping(const SlavePdoMapping& layout, std::span<const AxisPdoRequirements> axes) {
    if (std::string error = check_structure(layout); !error.empty()) {
        return core::Result<void>::error(std::move(error));
    }
    std::vector<Resolved> resolved;
    for (size_t i = 0; i < axes.size(); ++i) {
        if (std::string error = check_axis(layout, i, axes[i], resolved); !error.empty()) {
            return core::Result<void>::error(std::move(error));
        }
    }
    // Two requirements resolving to one entry would hand the same bytes to
    // two consumers (an ErrorCode declared at the Statusword address feeds
    // the statusword into the fault diagnostic).
    for (size_t a = 0; a < resolved.size(); ++a) {
        for (size_t b = a + 1; b < resolved.size(); ++b) {
            if (resolved[a].index == resolved[b].index && resolved[a].subindex == resolved[b].subindex) {
                return core::Result<void>::error(
                    fmt::format("axis {} role {} and axis {} role {} both resolve to 0x{:04x}:{:02x} — one entry "
                                "cannot feed two consumers",
                                resolved[a].axis, resolved[a].role, resolved[b].axis, resolved[b].role,
                                resolved[a].index, resolved[a].subindex));
            }
        }
    }
    return core::Result<void>::ok();
}

}  // namespace plem::fieldbus::ethercat
