// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <plem/core/visibility_control.hpp>

#include "plem/core/units/unit_base.hpp"

namespace plem::core::units {

struct TorqueCountTag {};

/**
 * @brief Represents torque in drive counts
 *
 * Similar to Pulse for position, TorqueCount represents the scaled torque value
 * used in EtherCAT PDO communication.
 *
 * What one count means is the axis's declared torque channel, not this
 * type: for the SCALED_CURRENT unit the count is
 * drive current:
 *   I  = count / counts_per_unit            (A; counts_per_unit = counts per amp)
 *   τ  = I · torque_constant · gear_ratio  (Nm at the output shaft)
 * See plem/fieldbus/ethercat/utils/servo_conversions.hpp and the model
 * headers' provenance for the numbers. This type only owns the int16
 * register width and the flip/saturation at the hardware boundary.
 */
class PLEM_CORE_PUBLIC TorqueCount : public Unit<int16_t, TorqueCountTag> {
  public:
    using Base = Unit<int16_t, TorqueCountTag>;

    constexpr TorqueCount() noexcept : Base() {}
    explicit constexpr TorqueCount(int16_t val) noexcept : Base(val) {}

    /**
     * @brief Convert to hardware representation with flip handling
     * @param flip Whether to flip the torque direction
     * @return Hardware value to write to EtherCAT domain
     */
    constexpr int16_t to_hardware(bool flip) const noexcept {
        // Widen so the flip negation of INT16_MIN cannot wrap back to INT16_MIN
        // (full-scale wrong-sign torque), then saturate at the register boundary.
        return saturate_to_int16(flip ? -static_cast<int32_t>(value_) : static_cast<int32_t>(value_));
    }

    /**
     * @brief Create from hardware representation with flip handling
     * @param hw_value Hardware value from EtherCAT domain
     * @param flip Whether to flip the torque direction
     * @return TorqueCount instance
     */
    static constexpr TorqueCount from_hardware(int16_t hw_value, bool flip) noexcept {
        return TorqueCount(saturate_to_int16(flip ? -static_cast<int32_t>(hw_value) : static_cast<int32_t>(hw_value)));
    }
};

}  // namespace plem::core::units
