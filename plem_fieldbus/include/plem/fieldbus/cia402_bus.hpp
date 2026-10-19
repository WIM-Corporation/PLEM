// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>

#include <plem/core/result.hpp>

namespace plem::fieldbus {

/**
 * @brief Cyclic master interface over a CiA 402 servo + IO device set
 *
 * The contract is CiA 402 (IEC 61800-7-201) servo-drive semantics:
 * reset_all_faults() requests the CiA 402 fault reset.
 * CiA 402 is transport-independent: EtherCAT (CoE)
 * implements it today (ethercat::EtherCATBus), and a CANopen master could
 * implement the same contract. This is deliberately not a protocol-neutral
 * fieldbus abstraction: transports without CiA 402 drives use their own
 * paths.
 *
 * Implementation: ethercat::EtherCATBus. The RT loop does not run on this
 * interface; ethercat::EtherCATController takes the concrete EtherCATBus
 * (cyclic receive/send and the aggregated status live there); this interface
 * serves bus assembly and fault reset.
 */
class ICiA402Bus {
  public:
    virtual ~ICiA402Bus() = default;

    /**
     * @brief Initialize the bus master and all connected devices
     * @return ok, or an error whose message names the cause and the fix
     *         (the bus does not log it; the caller reports it once)
     */
    [[nodiscard]] virtual core::Result<void> initialize() = 0;

    /**
     * @brief Number of servo devices
     */
    virtual size_t servo_count() const = 0;

    /**
     * @brief Post a CiA 402 fault-reset request on every servo device
     *
     * Posts a request; it does not read servo state and does not touch the
     * wire. Whether a request becomes a controlword bit-7 pulse is decided
     * per axis in the cycle that publishes it (a recognized FAULT is
     * required), and whether the drive then leaves FAULT is decided by the
     * drive over subsequent cycles. Nothing here confirms either.
     *
     * @return Number of servos a request was posted on, not the number of
     *         faulted servos, resets performed, or drives recovered. A
     *         healthy N-axis bus returns N.
     * @note Non-blocking, and callable off the cyclic thread.
     */
    virtual size_t reset_all_faults() = 0;
};

}  // namespace plem::fieldbus
