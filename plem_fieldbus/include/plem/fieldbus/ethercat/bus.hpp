// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <span>
#include <vector>

#include <plem/core/types/tri_state.hpp>

#include <ecrt.h>

#include "plem/fieldbus/cia402_bus.hpp"
#include "plem/fieldbus/ethercat/device_base.hpp"
#include "plem/fieldbus/ethercat/io_device_factory.hpp"
#include "plem/fieldbus/ethercat/servo/device.hpp"

namespace plem::fieldbus::test {
class EtherCATBusTestProbe;  // test-only observation (plem/fieldbus/test/, not a release header)
}  // namespace plem::fieldbus::test

namespace plem::fieldbus::ethercat {

/**
 * @brief EtherCAT implementation of ICiA402Bus. Owns the master and domain,
 * initializes and activates the master, registers and configures servo and IO
 * devices, and runs the cyclic PDO exchange.
 */
class EtherCATBus : public ICiA402Bus {
  public:
    explicit EtherCATBus(unsigned int master_index = 0);
    ~EtherCATBus() override;

    // Owns the master reservation: a copy would release it twice.
    EtherCATBus(const EtherCATBus&) = delete;
    EtherCATBus& operator=(const EtherCATBus&) = delete;
    EtherCATBus(EtherCATBus&&) = delete;
    EtherCATBus& operator=(EtherCATBus&&) = delete;

    // ICiA402Bus interface implementation
    [[nodiscard]] core::Result<void> initialize() override;

    /**
     * @brief Set servo configuration for config injection pattern
     *
     * Call this before initialize() to inject servo config.
     * If not called, initialize() fails fast (refuses); there is no file fallback.
     *
     * @param servo_infos Servo configurations for all joints
     */
    void set_servo_config(std::vector<ServoInfo> servo_infos);

    /**
     * @brief Set IO configuration for config injection pattern
     *
     * Call this before initialize() to inject IO config (optional; absent means no IO devices).
     * Every IO slave needs set_io_device_factory().
     *
     * @param io_infos IO device configurations
     */
    void set_io_config(std::vector<IOInfo> io_infos);

    /**
     * @brief Inject servo-slave startup SDO writes before initialize()
     * (optional; absent means none).
     *
     * initialize() refuses, before any master contact, a list for a position
     * that hosts no configured axis and a write StartupSdo::is_valid() rejects;
     * it then reserves every write in list order (Step 4, before activation)
     * and logs each one.
     *
     * @param slaves One ordered list per servo slave
     */
    void set_startup_sdos(std::vector<SlaveSdos> slaves);

    /**
     * @brief Inject the values each servo slave's drive must have stored
     * (drive-model schema §4.9) before initialize() (optional; absent means
     * nothing is read).
     *
     * initialize() refuses, before any master contact, a list for a position
     * that hosts no configured axis and a value StartupSdo::is_valid() rejects.
     * After the identity check it reads each value by blocking upload (init
     * context only) and warns on a difference or a failed read; it never
     * refuses to initialize over one and never writes. An object in the
     * slave's output process data or a PDO mapping/assignment object is not
     * read (one WARN each). PLEM sets no timeout of its own: a read still
     * queued in the master ends on a signal (which ends the whole check;
     * initialize() carries on); once the transfer runs, only IgH's CoE
     * handling bounds it (a per-step response timeout while the slave's
     * mailbox stays empty), so a slave that keeps answering with other
     * mailbox traffic can hold the start.
     *
     * @param slaves One list per servo slave
     */
    void set_stored_sdos(std::vector<SlaveSdos> slaves);

    /**
     * @brief Inject what the approval record claims each servo slave is
     * (drive-model schema §6) before initialize() (optional; absent means
     * no identity is checked).
     *
     * initialize() refuses, before any master contact, an entry for a
     * position that hosts no configured axis. Once the master is acquired (an
     * independent step before any configuration, DC on or off), it reads
     * every declared value (SII serial from the slave info; CoE 0x1009 and
     * 0x100A by blocking upload, init context only) and refuses to
     * initialize on a mismatch or on a declared value it cannot read: a
     * verified record binds motion to the drives it was made on.
     *
     * @param slaves One entry per servo slave the record binds to
     */
    void set_expected_slave_identity(std::vector<SlaveExpectedIdentity> slaves);

    /**
     * @brief Probe: initialize() checks the drives as a start does: the
     * configuration (Step 0), the slaves on the ring and their model (Step 3)
     * and the expected identities (Step 3.5), then releases the master and
     * returns. Nothing is written to a drive or activated, so
     * nothing can move. Set before initialize().
     */
    void set_probe(bool probe) noexcept;

    /**
     * @brief The last initialize() was refused only for identity values a
     * bench at these drives would replace (Step 3.5: serial, revision, 0x1009,
     * 0x100A, scanned PDO mapping digest, or one of them unreadable), not for
     * another model, a missing slave, drives in a different order from the
     * record's (a recorded serial of another slave at this position), or a
     * version read failing with -EIO and no SDO abort (the bus, not the record).
     */
    [[nodiscard]] bool identity_rebenchable() const noexcept;

    /**
     * @brief The pdo_mapping_digest initialize() cached for the scanned slave
     * at ring `position`: the value Step 3.5 compares a record with and the
     * INFO line prints. "" for a declared slave, a position without a servo,
     * or before initialize(). Written only by initialize() (non-RT, before
     * activation); read it after initialize() returned.
     */
    [[nodiscard]] std::string scanned_pdo_mapping_digest(uint32_t position) const;

    /**
     * @brief Inject the IO device factory before initialize()
     *
     * The creation path for IO devices: during initialize(), every IOInfo is served by one factory
     * call with IODeviceContext{live master, ring position, IOInfo, bus
     * period}; the result joins io_devices() (configure_PDOs,
     * configure_after_domain and the RT hooks via the DeviceBase virtuals). A
     * null return, a throw, or an IO with no factory injected fails
     * initialize() (refused, master released). Not needed without IO.
     */
    void set_io_device_factory(IODeviceFactory factory);

    /**
     * @brief Inject the active control period before initialize()
     *
     * The bus derives its master send interval from this value, and DC uses it
     * as the frame-alignment divisor (SYNC0 must be an integer multiple of it;
     * the SYNC0 cycle itself is the drive's registered constant, never this
     * period). 0 (default) = not injected; initialize() keeps the 1 ms
     * default.
     *
     * @param cycle_time_ns Active control period in nanoseconds
     */
    void set_cycle_time_ns(uint64_t cycle_time_ns) noexcept;

    /// @brief The active period initialize() resolved: the injected one, else 1 ms.
    [[nodiscard]] uint64_t cycle_time_ns() const noexcept {
        return cycle_time_ns_;
    }

    /**
     * @brief Enable EtherCAT distributed clocks (DC) before initialize()
     *
     * When enabled, initialize() runs Step 4.5 before master activation:
     * ecrt_slave_config_dc() on every servo (SYNC0 cycle =
     * ServoInfo::dc_sync0_cycle_ns, the drive's sync cycle; shift =
     * ServoInfo::dc_sync0_shift_ns) and the servo with the lowest ring position
     * is selected as the explicit DC reference clock. A slave declaring dc_write_cycle_time gets that cycle
     * as a U32 startup SDO on its 0x1C32:02 (once per slave); the drive's
     * 0x1C32 is never read. A cycle the drive cannot run is the drive's to
     * refuse, at PREOP→SAFEOP or as a sync error that drops the slave out of
     * OP (documented for some drives and the reference slave stack, not a
     * guarantee for every drive); the AL status code is in the kernel log.
     * initialize() fails fast when a servo's dc_assign_activate or
     * dc_sync0_cycle_ns is unregistered (0), when the cycle is not an integer
     * multiple of the injected bus period, when the master refuses to reserve
     * the cycle-time write, or when no active period was injected. Default off: PLEM configures no synchronization; the
     * drive runs its own setting.
     */
    void set_dc_enable(bool enable) noexcept;

    /// @brief Whether DC was enabled via set_dc_enable() (RT loop hoists this)
    [[nodiscard]] bool dc_enabled() const noexcept;

    /**
     * @brief One DC synchronization step (RT hot-loop, call right before send())
     *
     * Issues the IgH DC triple: ecrt_master_sync_reference_clock (push the
     * application time to the reference clock) + ecrt_master_sync_slave_clocks
     * (drift-compensate the chain) + ecrt_master_sync_monitor_queue (queue the
     * drift-monitor datagram processed next cycle by dc_sync_monitor_process()).
     * +3 datagrams on this cycle's frame. No-op while DC is disabled.
     *
     * @realtime_safe Yes: no allocation/blocking; three ecrt datagram queues.
     */
    void sync_distributed_clocks() noexcept;

    /**
     * @brief Process last cycle's DC drift-monitor datagram (RT hot-loop, after receive())
     *
     * @return Drift upper bound [ns]; 0 while DC is disabled; UINT32_MAX =
     *         monitor unavailable this cycle (IgH error sentinel; fold layers
     *         must skip sentinels, not max() them).
     * @realtime_safe Yes: single ecrt read, no allocation/blocking.
     */
    [[nodiscard]] uint32_t dc_sync_monitor_process() noexcept;

    /// @brief Receive data from all devices (cyclic communication)
    void receive();
    /// @brief Send data to all devices (cyclic communication)
    void send();
    void set_application_time(uint64_t app_time_ns) noexcept;
    size_t servo_count() const override;
    size_t reset_all_faults() override;

    /**
     * @brief Whether every servo can move: YES only if all are OPERATION_ENABLED
     *
     * UNDEFINED with no servos registered. Protective stop is not reported
     * here; the live safety path is the controller-layer ControlModeFlags.
     *
     * @note Cyclic thread only: it reads the state machine that thread
     *       writes. Off that thread, read the controller's mirrors
     *       (EtherCATController::statusword(axis), process_data_complete()).
     */
    [[nodiscard]] core::TriState motion_possible() const;

    // EtherCAT-specific methods (not part of ICiA402Bus)
    uint8_t* domain_pd();

    // Device accessors
    const std::vector<std::shared_ptr<DeviceServo>>& servo_devices() const {
        return servo_devices_;
    }

    /// @brief The IO devices, in IOInfo order. The RT loop drives them
    /// through the DeviceBase virtuals.
    const std::vector<std::shared_ptr<DeviceBase>>& io_devices() const {
        return io_devices_;
    }

    /// @brief Domain state snapshot (RT-safe: no allocation)
    struct DomainState {
        uint16_t wkc{0};
        bool wkc_ok{false};
        /// IgH wc_state value: 0=EC_WC_ZERO (no slave responded),
        /// 1=EC_WC_INCOMPLETE (some slaves responded), 2=EC_WC_COMPLETE (all responded).
        uint8_t wc_state{0};
    };

    /// @brief Query current domain state (WKC)
    /// @note RT-safe: reads EtherCAT domain state, no allocation, noexcept
    [[nodiscard]] DomainState get_domain_state() const noexcept;

    /// @brief A slave configuration's state as the master reports it
    /// (ecrt_slave_config_state).
    struct SlaveState {
        /// 1 INIT, 2 PREOP, 4 SAFEOP, 8 OP; 0 = offline (the master answers
        /// 0 for a slave that is not on the bus), or the query failed
        uint8_t al_state{0};
        /// In OP with this configuration, no reconfiguration pending
        bool operational{false};
    };

    /// @brief Sample the state of every servo's slave: one
    /// ecrt_slave_config_state() per physical slave, copied to every axis it
    /// hosts; out[i] is servo i's slave. A failed query reads as al_state 0,
    /// not operational, like an offline slave. Servos past out.size() are not
    /// sampled.
    /// @realtime_safe Yes: one non-blocking query per slave (the master's
    /// cached state, no master lock), no allocation. The cached state lags the
    /// wire by the master's state machine.
    void sample_servo_slave_states(std::span<SlaveState> out) noexcept;

    /**
     * @brief Keep caller-provided backing storage alive for this bus's whole
     * lifetime (type-erased).
     *
     * The declared-PDO-mapping contract requires the mapping objects the
     * ServoInfos point into to outlive the bus, and the bus is handed
     * around as a shared_ptr, so the code that built it alone cannot
     * guarantee that (a caller may hold the bus beyond it). Storage retained here is
     * released only when the bus itself is destroyed, after every device
     * (member order below: retained storage precedes the device lists, so
     * destruction runs devices-first).
     */
    void retain_pdo_mapping_storage(std::shared_ptr<const void> storage);

  private:
    /// initialize() only, after validate_bus_configuration() passed.
    void register_servo(std::shared_ptr<DeviceServo> device_servo);
    /// initialize() only: each IO device factory result (built with this
    /// bus's live master and the device's slave position). The stored device
    /// joins configure_PDOs(), configure_after_domain() and the RT traversal
    /// (note_received_frame, read/write) through the DeviceBase virtuals.
    void register_io(std::shared_ptr<DeviceBase> device);

    friend class ::plem::fieldbus::test::EtherCATBusTestProbe;  // test-only observation (plem/fieldbus/test/, not
                                                                // a release header)

    unsigned int master_index_;
    ec_master_t* master_;
    ec_domain_t* domain_;
    uint8_t* domain_pd_;
    static constexpr uint64_t kDefaultCycleTimeNs = 1'000'000;  ///< period when none is injected
    uint64_t cycle_time_ns_{kDefaultCycleTimeNs};               ///< resolved active period; initialize() sets it to the
                                                                ///< injected value when present, else the default
    uint64_t injected_cycle_time_ns_{0};                        ///< 0 = not injected (the 1 ms default applies)
    bool dc_enable_{false};  ///< DC deployment knob, default off (the drive runs its own setting)

    /// Retained caller storage (PDO-layout backing graphs). Declared before
    /// the device lists: destroyed after them, so a DeviceServo's raw
    /// pdo_mapping pointer stays valid for the device's whole life.
    std::vector<std::shared_ptr<const void>> retained_pdo_mapping_storage_;

    std::vector<std::shared_ptr<DeviceServo>> servo_devices_;
    std::vector<std::shared_ptr<DeviceBase>> io_devices_;

    // Injected configuration (servo config required; initialize() fails fast when absent)
    std::vector<ServoInfo> injected_servo_config_;
    std::vector<IOInfo> injected_io_config_;
    std::vector<SlaveSdos> injected_startup_sdos_;
    std::vector<SlaveSdos> injected_stored_sdos_;
    std::vector<SlaveExpectedIdentity> injected_expected_identity_;
    bool probe_{false};
    bool identity_rebenchable_{false};
    /// Step 3's pdo_mapping_digest of every scanned slave (position → digest),
    /// what Step 3.5 compares a record's `pdo_mapping_digest` with. A declared
    /// slave has no entry.
    std::vector<std::pair<uint32_t, std::string>> scanned_pdo_mapping_digests_;
    IODeviceFactory io_device_factory_;  ///< IO creation path (required when there is IO)

    core::Result<void> configure_PDOs();
    core::Result<void> configure_SDOs();
    core::Result<void> configure_DC();
    core::Result<void> verify_slave_identity();
    void check_stored_sdos(const std::vector<std::pair<uint32_t, const SlavePdoMapping*>>& layouts);
    void seed_position_before_enable(DeviceServo& servo, uint32_t position, const ServoInfo& info);
    core::Result<void> slave_string_matches(uint32_t position, uint16_t index,
                                            const std::array<char, SlaveIdentity::kVersionLength>& declared,
                                            const char* what);
    core::Result<void> activate_master();
    void release_master() noexcept;
};

}  // namespace plem::fieldbus::ethercat
