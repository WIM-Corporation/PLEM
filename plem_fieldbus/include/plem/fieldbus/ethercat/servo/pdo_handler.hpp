// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <ecrt.h>

#include "plem/fieldbus/ethercat/servo/pdo_types.hpp"

namespace plem::fieldbus::ethercat {

/**
 * @brief Handler for servo PDO data exchange
 *
 * Reads and writes servo PDO data in domain memory, converting between
 * logical values and the hardware representation (joint offset and direction
 * flip).
 */
class ServoPDOHandler {
  public:
    /**
     * @brief Construct a new Servo PDO Handler with required parameters
     * @param joint_offset Offset to apply to joint position (in pulses)
     * @param flip Whether to flip joint direction
     * @param position_word_target true for a CSP axis: 0x607A carries
     *        ServoPDOData::target_position_word verbatim (no encode)
     */
    explicit ServoPDOHandler(const Pulse& joint_offset, bool flip, bool position_word_target = false);

    /**
     * @brief Default constructor - deleted to enforce proper initialization
     */
    ServoPDOHandler() = delete;

    /**
     * @brief Write PDO data to EtherCAT domain memory
     * @param data PDO data to write
     * @param domain_pd Pointer to domain process data (non-const, writes to domain)
     */
    void write(const ServoPDOData& data, uint8_t* domain_pd) const;

    /**
     * @brief Read PDO data from EtherCAT domain memory
     * @param data PDO data to read into
     * @param domain_pd Pointer to domain process data
     */
    void read(ServoPDOData& data, const uint8_t* domain_pd) const;

    /// @brief Set the 0x6064 word and the joint position it decodes to.
    void set_position_actual_value(ServoPDOData& data, int32_t word) const noexcept;

    /**
     * @brief Get reference to PDO offsets for registration
     * @return ServoPDOOffsets& Reference to offsets structure
     */
    ServoPDOOffsets& get_offsets();

    [[nodiscard]] int32_t joint_offset() const noexcept {
        return joint_offset_;
    }
    [[nodiscard]] bool flip() const noexcept {
        return flip_;
    }

  private:
    ServoPDOOffsets offsets_;    ///< PDO offset for domain memory
    int32_t joint_offset_;       ///< Joint offset in encoder counts
    bool flip_;                  ///< Whether to flip joint direction
    bool position_word_target_;  ///< CSP: 0x607A = target_position_word, verbatim
};

}  // namespace plem::fieldbus::ethercat
