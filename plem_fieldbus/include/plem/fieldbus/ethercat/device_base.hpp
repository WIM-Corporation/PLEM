// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <vector>

#include <ecrt.h>

#include <plem/core/result.hpp>

namespace plem::fieldbus::ethercat {

/// Capacity of the one domain-registration array the bus hands to every
/// device (`EtherCATBus::configure_PDOs`). `ecrt_domain_reg_pdo_entry_list`
/// reads a zero-terminated list, so the last slot belongs to the terminator:
/// a filler must stop at `kMaxDomainRegs - 1` entries.
inline constexpr uint32_t kMaxDomainRegs = 128;

/** Definition for PDO entry registration.
 *
 * This structure is used to define the PDO entries that will be registered
 * in the EtherCAT master. The elements should be defined in the derived classes
 * to specify the index, subindex, and offset for each PDO entry.
 */
struct PDOEntryDefinition {
    uint16_t index;
    uint8_t subindex;
    uint32_t* offset;
};

/// One EtherCAT slave on the bus. This is the IO-device extension point: the
/// bus and controller call an IO device only through this base, its virtuals
/// and position() (built by an IODeviceFactory). DeviceServo derives from it
/// for the slave-config and PDO-registration plumbing; the bus holds servos as
/// DeviceServo, never dispatches them through this base.
class DeviceBase {
  public:
    virtual ~DeviceBase();
    DeviceBase(const DeviceBase&) = delete;
    DeviceBase& operator=(const DeviceBase&) = delete;
    DeviceBase(DeviceBase&&) = delete;
    DeviceBase& operator=(DeviceBase&&) = delete;

    // Cyclic process-data exchange (domain PDO image). RT, every cycle: no
    // allocation, no blocking call, no exception (as note_received_frame).
    virtual void read(const uint8_t* domain_pd) noexcept;
    virtual void write(uint8_t* domain_pd) noexcept;
    uint32_t position() const {
        return slave_position_;
    }

    // PDO Functions (EtherCAT-specific)
    /// Adds this device's entries to the domain registration list. A refusal
    /// names its cause; initialize() refuses with it, naming the slave.
    virtual core::Result<void> register_PDOs(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index) = 0;

    // Bus hooks. Each one is a no-op unless a device overrides it.

    /// Once per device, after the domain's entry list is registered and
    /// before the servo startup SDOs and activation: entries registered one
    /// at a time, the device's own startup SDOs. A refusal names its cause;
    /// initialize() refuses with it, naming the slave.
    virtual core::Result<void> configure_after_domain(ec_domain_t* /*domain*/) {
        return core::Result<void>::ok();
    }
    /// RT, every cycle right after receive and before read(): whether the
    /// frame just received came back with a complete working counter.
    virtual void note_received_frame(bool /*wkc_complete*/) noexcept {}

    // Getter Functions
    ec_slave_config_t* get_config() const {
        return device_config_;
    }
    const ec_slave_info_t& get_slave_info() const {
        return slave_info_;
    }

  protected:
    // Constructor is protected: only derived devices construct a DeviceBase.
    DeviceBase(ec_master_t* master, uint32_t slave_position, uint32_t vendor_id, uint32_t product_code);

    // Helper method for derived classes to register PDO entries
    core::Result<void> register_PDO_entries(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index,
                                            const std::vector<PDOEntryDefinition>& entries);

  private:
    ec_slave_config_t* device_config_;
    ec_slave_info_t slave_info_;
    uint32_t slave_position_{0};
};

}  // namespace plem::fieldbus::ethercat
