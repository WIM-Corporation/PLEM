// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>

#include <plem/core/units.hpp>

#include "plem/fieldbus/cia402_types.hpp"
#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/device_base.hpp"
#include "plem/fieldbus/ethercat/servo/pdo_handler.hpp"
#include "plem/fieldbus/ethercat/servo/pdo_types.hpp"
#include "plem/fieldbus/ethercat/state_machine.hpp"
#include "plem/fieldbus/ethercat/utils/servo_conversions.hpp"

namespace plem::fieldbus::test {
struct DeviceServoTestAccess;  // test/device_servo_test_access.hpp, not installed
}  // namespace plem::fieldbus::test

namespace plem::fieldbus::ethercat {

using plem::core::units::NewtonMeter;
using plem::core::units::Pulse;
using plem::core::units::PulsePerSecond;
using plem::core::units::Radian;
using plem::core::units::RadianPerSecond;
using plem::core::units::TorqueCount;

/// @brief What a fault-reset request did at the last publication point.
///
/// Only terminal outcomes: a request held for the frame that owes a falling
/// edge reports NONE and reports its real outcome on the next frame. The two
/// drop reasons are kept apart because they ask different things of the
/// operator: a recognized non-FAULT axis is worth re-requesting once it
/// settles into FAULT, while an undecodable statusword says nothing at all
/// about the axis.
enum class FaultResetPublication : uint8_t {
    NONE = 0,              ///< nothing terminal happened this frame
    PUBLISHED,             ///< controlword bit 7 reached domain memory
    DROPPED_NOT_IN_FAULT,  ///< recognized, but not FAULT (healthy, or still reacting)
    DROPPED_UNRECOGNIZED,  ///< the statusword was not a decodable CiA 402 pattern
};

/**
 * @brief Servo device for cyclic synchronous torque, position or velocity
 * control with type-safe units from plem_core and configurable torque
 * protection limits. ServoStateMachine tracks the CiA 402 state;
 * ServoPDOHandler transfers PDO data.
 */
class DeviceServo final : public DeviceBase {
  public:
    /**
     * @brief Production constructor
     * @param master EtherCAT master instance
     * @param slave_position Position of the slave on the bus
     * @param servo_info Servo configuration information
     * @throws std::runtime_error if EtherCAT configuration fails
     */
    DeviceServo(ec_master_t* master, uint32_t slave_position, const ServoInfo& servo_info);

    ~DeviceServo();

    // Cyclic PDO Exchange (DeviceBase)
    void read(const uint8_t* domain_pd) noexcept override;
    void write(uint8_t* domain_pd) noexcept override;

    /// Whether the frame the next read() consumes came back with a complete
    /// working counter. Called every cycle before read(). On an endless axis
    /// only a complete frame with a non-zero statusword is a position
    /// observation; other axes ignore it.
    void note_received_frame(bool wkc_complete) noexcept override {
        frame_complete_ = wkc_complete;
    }

    // Servo Command / Feedback
    // CSP/CSV command path. Words are what the wire carries
    // (offset and flip applied); the encoders refuse what the wire cannot
    // carry instead of saturating. A CSP axis's 0x607A mirrors the raw 0x6064
    // word in read(); set_target_position() after read() replaces it for this
    // frame only. Checking a command (limits, step) is the caller's.
    // An endless axis (ServoInfo::position_endless) encodes into the wrapped
    // word over ±kEndlessCountDomain counts instead of int32, and its mirror
    // is the last observed 0x6064 word (note_received_frame), so the held word
    // and get_actual_position() stand for the same count.
    [[nodiscard]] std::optional<CheckedTarget> encode_position(double rad) const noexcept;
    [[nodiscard]] std::optional<CheckedTarget> encode_velocity(double rad_s) const noexcept;
    /// The accepted CSP target: its word goes on 0x607A and its joint-frame
    /// angle is what the frame commands.
    void set_target_position(const CheckedTarget& target) noexcept;
    void set_target_velocity_word(int32_t word) noexcept;
    /// False when the last 0x6064 word does not decode into int32 joint counts
    /// (get_actual_position() is then a saturated value, not the axis position).
    /// An endless axis: true once seeded and while its count has never left
    /// ±kEndlessCountDomain (leaving it is sticky until restart).
    [[nodiscard]] bool position_word_decodable() const noexcept;
    /// The current target word, back in the declared mode's unit (CST Nm, CSP
    /// rad, CSV rad/s) through the same flip and saturation the handler applies;
    /// NaN for a CSP word that does not decode. An endless CSP axis returns the
    /// accepted target's si (the raw hold: the actual). It is what the frame carried
    /// only when read on the RT thread after write() and before the next target
    /// change (read() or a set_target_*).
    [[nodiscard]] double applied_command() const noexcept;

    // State management
    bool is_faulted() const;

    // CiA 402 fault reset
    // Three entries, one publication point. Through these entries the bit-7
    // pulse is emitted only by write() (next_fault_reset_frame owns both
    // edges); they post, drop, or refuse a request; none touches the wire,
    // and none means the drive accepted anything.

    /// @brief RT-path request: post a fault-reset request iff this axis is
    /// eligible right now and no pulse still holds bit 7. Reads the state
    /// machine and the pulse state, so it is callable only from the RT cycle
    /// that produced the sample (the sequence executor). A refusal spends no
    /// automatic attempt; a pulse being held is not a new try. The frame that
    /// owes only the falling edge accepts, and write() holds the request over
    /// it, exactly as with a one-frame pulse.
    /// @return true iff the request was accepted (not "the drive reset").
    bool reset_fault();

    /// @brief Off-RT request: post a fault-reset request without reading any
    /// state. Eligibility is re-decided at publication time inside write(),
    /// so an axis that is not in a recognized FAULT by then simply drops it,
    /// which is why this entry can be called from any thread. (Reading the
    /// state machine here would race the RT writer.)
    void request_fault_reset();

    /// @brief Drop any request that has not been published yet.
    ///
    /// The run boundary owns this: a request posted while the controller was
    /// stopped must not fire on the next run's first recognized FAULT. The
    /// contract is "not consumed by the next run", not "the flag reads false
    /// immediately after stop()".
    void discard_fault_reset_request();

    /// @brief Pulse width in frames (write() holds bit 7 this many frames at
    /// most). The bus sets it once, after construction, from the model's
    /// `fault_reset_pulse_ms` at its period; 1 until then. Not RT-safe to call
    /// while the RT loop runs.
    void set_fault_reset_pulse_frames(uint32_t frames) noexcept {
        reset_pulse_frames_ = frames;
    }

    /// @brief Run boundary, RT thread only: fold a pulse still on the wire
    /// into its falling edge. The first write of the new run then takes bit 7
    /// low instead of resuming the previous run's hold; a restart owes the
    /// edge, never the rest of the width.
    void collapse_fault_reset_pulse() noexcept {
        if (reset_pulse_owed_ > 1) {
            reset_pulse_owed_ = 1;
        }
    }

    /// @brief What the last write() did with a fault-reset request. Reset to
    /// NONE by every write, so a caller reads it once per cycle, after the
    /// write loop. RT-only (plain member, single writer).
    [[nodiscard]] FaultResetPublication last_fault_reset_publication() const noexcept {
        return last_fault_reset_publication_;
    }

    // CiA 402 state machine
    ServoState get_servo_state() const;
    /// Was the last PDO statusword a recognized CiA 402 pattern? (see
    /// ServoStateMachine::status_recognized: get_servo_state() holds the
    /// previous state on an unrecognized word.)
    bool status_recognized() const;
    /// The declared mode of operation (ServoInfo::modes_of_operation): the 0x6060
    /// command byte this device sends every cycle. What the drive reports it
    /// is in is get_modes_of_operation_display() (0x6061).
    OperationMode get_modes_of_operation() const;
    /// When this drive first reports the declared mode on 0x6061
    /// (ServoInfo::mode_display_timing, a model declaration, immutable).
    ModeDisplayTiming mode_display_timing() const;
    bool transition_state(ServoState target_state);

    // EtherCAT-Specific Methods
    core::Result<void> register_PDOs(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index) override;

    /// @brief Write the declared sync-manager/PDO mapping to this slave
    /// (PREOP, before master activation). Called by
    /// EtherCATBus::configure_PDOs() once per slave (the axes of a multi-axis
    /// slave share one ec_slave_config_t) and only for declared slaves; a
    /// scanned slave keeps the mapping the master found (never called).
    core::Result<void> apply_pdo_mapping();

    /**
     * @brief Set torque command with hardware protection limits
     * @param torque Target torque in N·m (clamped to configured limits)
     * @return true if torque was within limits, false if clamping occurred
     */
    bool set_command(NewtonMeter torque);

    // Getters in physical units
    Radian get_actual_position() const;
    RadianPerSecond get_actual_velocity() const;
    NewtonMeter get_actual_torque() const;

    // Raw PDO values
    uint16_t get_statusword() const;
    int8_t get_modes_of_operation_display() const;

    /// @brief CiA 402 error code (0x603F) from the current frame, storage
    /// only. Nonzero only while the drive reports a code and the model
    /// declared the ErrorCode optional-PDO role; stays 0 otherwise. The single
    /// consumer is the controller fault path (RtEventEmitter carries it in
    /// SERVO_FAULT.value with the joint id); this device emits nothing.
    uint16_t error_code() const;

    // Additional state helpers
    bool can_transition_to(ServoState target_state) const {
        return state_machine_.can_transition_to(target_state);
    }

    /// The declared CiA 402 transition profile (ServoInfo::cia402_profile).
    const Cia402TransitionProfile& cia402_profile() const {
        return state_machine_.profile();
    }

    bool is_initialized() const {
        return is_initialized_;
    }

    /// @brief Stand in `word` for 0x6064 until the drive reports a nonzero
    /// position of its own (ServoInfo::position_before_enable_index). Init
    /// context only, before the bus is activated.
    void seed_position_before_enable(int32_t word) noexcept {
        position_seed_ = word;
    }

    /// ServoInfo::position_endless: the declaration, immutable.
    [[nodiscard]] bool position_endless() const noexcept {
        return position_endless_;
    }

    /// Endless axis: the unwrapped 64-bit hardware count (0 on a bounded
    /// axis). RT-written without synchronization; read it only while the RT
    /// loop is stopped.
    [[nodiscard]] int64_t position_count() const noexcept {
        return position_count_;
    }

  private:
    friend struct ::plem::fieldbus::test::DeviceServoTestAccess;  // test-only seams (not installed)

    /// @brief Is this axis eligible for a fault-reset pulse right now?
    /// (recognized statusword ∧ FAULT, narrower than is_faulted(), see the
    /// definition). RT-only: it reads the state machine.
    [[nodiscard]] bool fault_reset_eligible() const;

    /// @brief True when the PDO mapping carries `index`.
    [[nodiscard]] bool pdo_mapping_carries(uint16_t index) const noexcept;

    /// @brief Endless axis, valid frame: seed or advance the count from the
    /// raw 0x6064 word and refresh the cached angle.
    void observe_endless_position() noexcept;

    ServoStateMachine state_machine_;  // State management
    ServoPDOHandler pdo_handler_;      // PDO communication handling
    ServoPDOData pdo_data_;            // Buffer that will be written to the PDO

    // Multi-axis / layout / optional-PDO raw material (from ServoInfo).
    const SlavePdoMapping* pdo_layout_;  // non-null: declared, or the bus's Step 3 scan; borrowed, outlives the bus
    uint16_t od_offset_;                 // CiA 402 channel base (0 = first/only axis)
    PdoEntryMapping error_code_pdo_;     // index 0 = role undeclared

    // Torque command and its clamp limits
    NewtonMeter target_torque_{0.0};
    NewtonMeter max_torque_;
    NewtonMeter min_torque_;

    // The axis's channel map: what the unit conversions read
    AxisFactors channels_;

    ModeDisplayTiming mode_display_timing_;  // declared 0x6061 reporting timing (ServoInfo::mode_display_timing)
    bool is_initialized_;                    // Device initialization status (PDO communication established)
    std::optional<int32_t> position_seed_;   // 0x6064 stand-in until the drive reports its own (init-set, RT-cleared)

    // Endless axis (ServoInfo::position_endless). The count is the hardware
    // count (acc ≡ last valid 0x6064 word mod 2^32); the angles are joint frame.
    bool position_endless_;
    bool frame_complete_{false};
    bool position_out_of_domain_{false};  // sticky: only a restart clears it
    int32_t prev_position_word_{0};
    int64_t position_count_{0};
    double actual_position_rad_{0.0};   // decoded once per valid read()
    double applied_position_rad_{0.0};  // si of the CSP word on the wire

    // Fault-reset pulse state. The request is the only cross-thread field:
    // any thread may publish `true`, and the RT writer is the only consumer.
    // `reset_pulse_owed_` is RT-only (write() writes and reads it), so it is
    // a plain integer; making it atomic would advertise a sharing that does
    // not exist. `reset_pulse_frames_` is configuration, set before the loop.
    static_assert(std::atomic<bool>::is_always_lock_free);
    std::atomic<bool> fault_reset_request_{false};
    uint32_t reset_pulse_owed_{0};
    uint32_t reset_pulse_frames_{1};
    FaultResetPublication last_fault_reset_publication_{FaultResetPublication::NONE};
    /// Test seam (DeviceServoTestAccess): run inside write() after the request
    /// is acquired and before the frame is decided. Production never sets it.
    std::function<void()> post_acquire_hook_for_test_;
};

}  // namespace plem::fieldbus::ethercat
