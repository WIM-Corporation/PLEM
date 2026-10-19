// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/ethercat/device_base.hpp"

#include <stdexcept>

#include <fmt/format.h>

#include <plem/core/logging/logger.hpp>

namespace plem::fieldbus::ethercat {

static auto& logger = plem::core::Logger::instance();

/// @throws std::runtime_error if the master refuses the slave config or the
///         slave info read fails.
DeviceBase::DeviceBase(ec_master_t* master, uint32_t slave_position, uint32_t vendor_id, uint32_t product_code)
    : device_config_(nullptr), slave_info_{}, slave_position_(slave_position) {
    device_config_ = ecrt_master_slave_config(master, 0, slave_position, vendor_id, product_code);
    if (!device_config_) {
        throw std::runtime_error("Failed to create EtherCAT slave configuration for position " +
                                 std::to_string(slave_position));
    }

    if (ecrt_master_get_slave(master, slave_position, &slave_info_) < 0) {
        throw std::runtime_error("Failed to get slave info for position " + std::to_string(slave_position));
    }
}

DeviceBase::~DeviceBase() = default;

// PDO Functions

core::Result<void> DeviceBase::register_PDO_entries(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index,
                                                    const std::vector<PDOEntryDefinition>& entries) {
    if (entries.empty()) {
        return core::Result<void>::error("PDO entries are not defined for this device.");
    }

    // `ecrt_domain_reg_pdo_entry_list` walks a zero-terminated list, so the
    // caller's last slot belongs to the terminator. Refuse before writing
    // anything: a partial registration would leave the shared array in a
    // state the next device would append to.
    if (domain_index + entries.size() >= kMaxDomainRegs) {
        return core::Result<void>::error(fmt::format(
            "the device declares {} PDO entries but only {} of the domain's {} registration slots are free (the last "
            "slot is the zero terminator). Map fewer PDO entries on this bus, or raise kMaxDomainRegs in "
            "device_base.hpp.",
            entries.size(), domain_index < kMaxDomainRegs ? kMaxDomainRegs - 1 - domain_index : 0, kMaxDomainRegs));
    }

    const ec_slave_info_t& slave_info = this->get_slave_info();
    uint16_t alias = 0;

    for (const auto& entry : entries) {
        domain_regs[domain_index++] = {
            alias,                    // Slave alias
            slave_info.position,      // Slave position
            slave_info.vendor_id,     // Slave vendor ID
            slave_info.product_code,  // Slave product code
            entry.index,              // PDO entry index
            entry.subindex,           // PDO entry subindex
            entry.offset,             // Pointer to the offset variable
            nullptr                   // Bit position (optional, can be nullptr)
        };
    }

    return core::Result<void>::ok();
}

// Cyclic Process-Data Exchange (default no-op)

/// Default no-op; derived devices override.
void DeviceBase::write(uint8_t* domain_pd) noexcept {
    (void)domain_pd;
}

/// Default no-op; derived devices override.
void DeviceBase::read(const uint8_t* domain_pd) noexcept {
    (void)domain_pd;
}

}  // namespace plem::fieldbus::ethercat
