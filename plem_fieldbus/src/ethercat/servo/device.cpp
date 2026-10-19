// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/ethercat/servo/device.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <plem/core/logging/logger.hpp>

#include "ethercat/servo/fault_reset_pulse.hpp"
#include "plem/fieldbus/ethercat/utils/servo_conversions.hpp"

namespace plem::fieldbus::ethercat {

static auto& logger = plem::core::Logger::instance();

DeviceServo::DeviceServo(ec_master_t* master, uint32_t slave_position, const ServoInfo& servo_info)
    : DeviceBase(master, slave_position, servo_info.vendor_id, servo_info.product_code),
      state_machine_(servo_info.cia402_profile),
      pdo_handler_(servo_info.joint_offset, servo_info.flip,
                   servo_info.modes_of_operation == OperationMode::CYCLIC_SYNC_POSITION),
      pdo_data_{},
      pdo_layout_(servo_info.pdo_mapping),
      od_offset_(servo_info.od_offset),
      error_code_pdo_(servo_info.error_code_pdo),
      target_torque_(0.0),
      max_torque_(servo_info.max_torque_nm),
      min_torque_(servo_info.min_torque_nm),
      channels_(servo_info.factors),
      mode_display_timing_(servo_info.mode_display_timing),
      is_initialized_(false),
      position_endless_(servo_info.position_endless) {
    // The declared mode is the 0x6060 command byte (sent every cycle from the
    // first frame on); get_modes_of_operation() reads it back. read() only ever
    // refreshes the 0x6061 display, so this byte stays the declaration.
    pdo_data_.modes_of_operation = static_cast<int8_t>(servo_info.modes_of_operation);
}

DeviceServo::~DeviceServo() = default;

core::Result<void> DeviceServo::register_PDOs(ec_pdo_entry_reg_t* domain_regs, uint32_t& domain_index) {
    auto& offsets = pdo_handler_.get_offsets();

    // Channel base for multi-axis slaves: 0 for the first axis, 0x800 for the
    // second (CiA 402 multi-axis convention). Single-axis drives leave it 0, so
    // these are the standard indices unchanged.
    const uint16_t ch = od_offset_;

    // Define PDO entries for servo motor control
    std::vector<PDOEntryDefinition> pdo_entries = {
        // RxPDO (Master to Slave)
        {static_cast<uint16_t>(0x6040 + ch), 0, &offsets.controlword},         // Controlword
        {static_cast<uint16_t>(0x6060 + ch), 0, &offsets.modes_of_operation},  // Mode of operation
        {static_cast<uint16_t>(0x607A + ch), 0, &offsets.target_position},     // Target position
        {static_cast<uint16_t>(0x60FF + ch), 0, &offsets.target_velocity},     // Target velocity
        {static_cast<uint16_t>(0x6071 + ch), 0, &offsets.target_torque},       // Target torque
        // TxPDO (Slave to Master)
        {static_cast<uint16_t>(0x6041 + ch), 0, &offsets.statusword},                  // Statusword
        {static_cast<uint16_t>(0x6061 + ch), 0, &offsets.modes_of_operation_display},  // Mode of operation display
        {static_cast<uint16_t>(0x6064 + ch), 0, &offsets.position_actual_value},       // Actual position
        {static_cast<uint16_t>(0x606C + ch), 0, &offsets.velocity_actual_value},       // Actual velocity
        {static_cast<uint16_t>(0x6077 + ch), 0, &offsets.torque_actual_value},         // Actual torque
    };

    // The declared layout is authoritative about what the slave actually maps.
    // Registering an object the slave does not map fails the whole list with an
    // opaque ENOENT, so drop those entries here; their offsets stay
    // kUnmappedPdoOffset and read()/write() skip them.
    std::vector<PDOEntryDefinition> mapped;
    mapped.reserve(pdo_entries.size());
    for (const auto& entry : pdo_entries) {
        if (pdo_mapping_carries(entry.index)) {
            mapped.push_back(entry);
        } else {
            logger.info("[DeviceServo] slave {}: object 0x{:04x} is not in the declared PDO mapping — skipping",
                        position(), entry.index);
        }
    }
    pdo_entries.swap(mapped);

    // Optional-PDO role: ErrorCode only. The explicit declaration is the
    // positive evidence that the slave maps the object (bus initialize()
    // validated it against any declared layout); undeclared means it is not
    // registered.
    if (error_code_pdo_.index != 0) {
        pdo_entries.push_back(
            {static_cast<uint16_t>(error_code_pdo_.index + ch), error_code_pdo_.subindex, &offsets.error_code});
    }

    return register_PDO_entries(domain_regs, domain_index, pdo_entries);
}

bool DeviceServo::pdo_mapping_carries(uint16_t index) const noexcept {
    // Index existence only; the declared layout passed semantic validation
    // at bus initialize() (non-null, null lists rejected); this filter just
    // drops the optional roles the slave does not map (a gap's index 0 never
    // matches a role).
    for (uint8_t i = 0; i < pdo_layout_->sync_count; ++i) {
        const SyncManagerAssignment& sm = pdo_layout_->syncs[i];
        for (uint8_t j = 0; j < sm.pdo_count; ++j) {
            const PdoMapping& pdo = sm.pdos[j];
            for (uint8_t k = 0; k < pdo.entry_count; ++k) {
                if (pdo.entries[k].index == index) {
                    return true;
                }
            }
        }
    }
    return false;
}

core::Result<void> DeviceServo::apply_pdo_mapping() {
    ec_slave_config_t* sc = get_config();
    if (sc == nullptr) {
        return core::Result<void>::error(
            fmt::format("slave {}: no slave config — cannot apply PDO layout", position()));
    }

    // IgH takes flat arrays terminated by sentinels, and reads them only during
    // the call, so building them locally is safe. They must stay alive across
    // the ecrt_slave_config_pdos() call, hence the reserve(): a reallocation
    // would dangle the pointers already written into the parent structures.
    std::vector<ec_pdo_entry_info_t> entries;
    std::vector<ec_pdo_info_t> pdos;
    std::vector<ec_sync_info_t> syncs;

    size_t total_entries = 0;
    size_t total_pdos = 0;
    for (uint8_t i = 0; i < pdo_layout_->sync_count; ++i) {
        const SyncManagerAssignment& sm = pdo_layout_->syncs[i];
        total_pdos += sm.pdo_count;
        for (uint8_t j = 0; j < sm.pdo_count; ++j) {
            total_entries += sm.pdos[j].entry_count;
        }
    }
    entries.reserve(total_entries);
    pdos.reserve(total_pdos);
    syncs.reserve(static_cast<size_t>(pdo_layout_->sync_count) + 1);

    for (uint8_t i = 0; i < pdo_layout_->sync_count; ++i) {
        const SyncManagerAssignment& sm = pdo_layout_->syncs[i];
        const size_t pdo_begin = pdos.size();

        for (uint8_t j = 0; j < sm.pdo_count; ++j) {
            const PdoMapping& pdo = sm.pdos[j];
            const size_t entry_begin = entries.size();
            for (uint8_t k = 0; k < pdo.entry_count; ++k) {
                const PdoEntryMapping& e = pdo.entries[k];
                // Gaps (index 0x0000) are passed through verbatim: they are not
                // registerable but they occupy image space, and dropping them
                // shifts every following entry's byte offset.
                entries.push_back(ec_pdo_entry_info_t{e.index, e.subindex, e.bit_length});
            }
            pdos.push_back(
                ec_pdo_info_t{pdo.index, pdo.entry_count, pdo.entry_count ? &entries[entry_begin] : nullptr});
        }

        // Declared layouts only: the core sets the sync-manager watchdog here.
        // A scanned slave never reaches this function and keeps the drive's
        // own (EEPROM) setting; see pdo_layout.hpp.
        syncs.push_back(ec_sync_info_t{sm.sm_index, sm.output ? EC_DIR_OUTPUT : EC_DIR_INPUT, sm.pdo_count,
                                       sm.pdo_count ? &pdos[pdo_begin] : nullptr,
                                       sm.watchdog ? EC_WD_DEFAULT : EC_WD_DISABLE});
    }
    syncs.push_back(ec_sync_info_t{0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT});

    if (ecrt_slave_config_pdos(sc, EC_END, syncs.data()) != 0) {
        return core::Result<void>::error(
            fmt::format("slave {}: the master refused to record the declared PDO layout (ecrt_slave_config_pdos() "
                        "failed) — check the declared sync managers and PDO indices in the drive model file",
                        position()));
    }

    logger.info("[DeviceServo] slave {}: enforced declared PDO mapping ({} sync managers, {} PDOs, {} entries)",
                position(), pdo_layout_->sync_count, total_pdos, total_entries);
    return core::Result<void>::ok();
}

// Cyclic PDO Exchange (DeviceBase)

namespace {
/// The decision returns the request's disposition; the reportable outcome
/// adds the reason a drop happened, which the decision deliberately does not
/// carry (it takes eligibility already composed).
FaultResetPublication to_publication(FaultResetRequest disposition, bool status_recognized) noexcept {
    switch (disposition) {
        case FaultResetRequest::CONSUMED:
            return FaultResetPublication::PUBLISHED;
        case FaultResetRequest::DISCARDED:
            return status_recognized ? FaultResetPublication::DROPPED_NOT_IN_FAULT
                                     : FaultResetPublication::DROPPED_UNRECOGNIZED;
        case FaultResetRequest::NONE:
        case FaultResetRequest::KEPT:
            break;
    }
    return FaultResetPublication::NONE;
}
}  // namespace

void DeviceServo::write(uint8_t* domain_pd) noexcept {
    pdo_data_.target_torque = to_torque_count(target_torque_, channels_.torque);

    // The only publication point of the CiA 402 fault-reset pulse. Acquiring
    // the request here, the last moment before the word reaches domain
    // memory, is what lets the manual entry be state-free: eligibility is
    // re-checked against this cycle's sample, not the sample the requester
    // happened to see. (The RT entry still reads state, to decide whether an
    // automatic attempt is spent at all.) `pdo_data_.controlword` is both the standing word and
    // the pulse output, so no second command store exists to drift.
    const bool recognized = state_machine_.status_recognized();
    const bool taken = fault_reset_request_.exchange(false, std::memory_order_acq_rel);
    if (post_acquire_hook_for_test_) {
        post_acquire_hook_for_test_();
    }
    const auto frame = next_fault_reset_frame(reset_pulse_owed_, taken, fault_reset_eligible(), pdo_data_.controlword,
                                              reset_pulse_frames_);
    pdo_data_.controlword = frame.controlword;
    reset_pulse_owed_ = frame.owed;
    last_fault_reset_publication_ = to_publication(frame.request, recognized);
    if (frame.request == FaultResetRequest::KEPT) {
        // Merge the held request back with anything published since the
        // exchange, never store(false), which would erase a cross-thread
        // publication. A request that arrived after the exchange belongs to
        // the next cycle by design, not to this frame.
        fault_reset_request_.store(true, std::memory_order_release);
    }

    pdo_handler_.write(pdo_data_, domain_pd);
}

void DeviceServo::read(const uint8_t* domain_pd) noexcept {
    pdo_handler_.read(pdo_data_, domain_pd);

    // Until the drive reports a position of its own, the stand-in read before
    // activation is the position: reported, and mirrored into 0x607A below.
    if (position_seed_) {
        if (pdo_data_.position_actual_value_word != 0) {
            position_seed_.reset();
        } else {
            pdo_handler_.set_position_actual_value(pdo_data_, *position_seed_);
        }
    }

    // An endless axis observes only a frame that came back whole from a drive
    // that answers: a lost frame's zero image must move nothing.
    const bool observed = pdo_data_.statusword != 0 && (!position_endless_ || frame_complete_);
    if (position_endless_ && observed) {
        observe_endless_position();
    }

    // Target position tracks actual, in every mode, every frame. Pulse(0) is
    // not zero on the wire; it is the encode of joint angle 0 rad, far from a
    // resting axis. A vendor reference driver seeds actual into target
    // before touching the controlword; mirroring unconditionally reproduces
    // that (pinned by TargetSeedingMirrorsActualUnconditionallyThroughEnableFrame).
    // CSP copies the raw word: a CSP drive follows 0x607A, and the decode into
    // joint counts saturates at a large offset; re-encoding that would put a
    // word other than the actual one on the wire. A CSP command replaces the
    // mirror after read() (set_target_position), for that frame only.
    // CST keeps its decode/re-encode bytes (CST drives ignore 0x607A).
    // An endless axis holds its last observed word instead: the word its
    // held angle stands for (an incomplete frame's word is no observation);
    // before the first observation there is none, so the raw word.
    if (get_modes_of_operation() == OperationMode::CYCLIC_SYNC_POSITION) {
        const bool seeded = observed || is_initialized_;
        pdo_data_.target_position_word =
            position_endless_ && seeded ? prev_position_word_ : pdo_data_.position_actual_value_word;
        applied_position_rad_ = actual_position_rad_;
    } else {
        pdo_data_.target_position = pdo_data_.position_actual_value;
    }

    state_machine_.update_state(pdo_data_.statusword);

    // Update initialization status (an endless axis: its first observation,
    // which seeded the count above)
    if (!is_initialized_ && observed) {
        is_initialized_ = true;
    }
}

void DeviceServo::observe_endless_position() noexcept {
    const int32_t word = pdo_data_.position_actual_value_word;
    if (!is_initialized_) {
        position_count_ = word;
    } else if (!position_out_of_domain_) {
        const int64_t candidate = position_count_ + wrap_diff(word, prev_position_word_);
        if (candidate > kEndlessCountDomain || candidate < -kEndlessCountDomain) {
            position_out_of_domain_ = true;
        } else {
            position_count_ = candidate;
        }
    }
    prev_position_word_ = word;
    actual_position_rad_ =
        endless_count_to_angle(Pulse::joint_count(position_count_, 0, pdo_handler_.flip()), channels_.position);
}

// Servo Command / Feedback Implementation

std::optional<CheckedTarget> DeviceServo::encode_position(double rad) const noexcept {
    if (position_endless_) {
        return encode_endless_position_checked(rad, channels_.position, pdo_handler_.flip());
    }
    return encode_position_checked(rad, channels_.position, pdo_handler_.joint_offset(), pdo_handler_.flip());
}

std::optional<CheckedTarget> DeviceServo::encode_velocity(double rad_s) const noexcept {
    return encode_velocity_checked(rad_s, channels_.velocity, pdo_handler_.flip());
}

void DeviceServo::set_target_position(const CheckedTarget& target) noexcept {
    pdo_data_.target_position_word = target.word;
    applied_position_rad_ = target.si;
}

void DeviceServo::set_target_velocity_word(int32_t word) noexcept {
    // The handler encodes target_velocity with the flip; store the joint-frame
    // count whose encode is `word` (a checked word always round-trips).
    pdo_data_.target_velocity = PulsePerSecond::from_hardware(word, pdo_handler_.flip());
}

bool DeviceServo::position_word_decodable() const noexcept {
    if (position_endless_) {
        return is_initialized_ && !position_out_of_domain_;
    }
    return decode_position_word(pdo_data_.position_actual_value_word, channels_.position, pdo_handler_.joint_offset(),
                                pdo_handler_.flip())
        .has_value();
}

double DeviceServo::applied_command() const noexcept {
    const bool flip = pdo_handler_.flip();
    switch (get_modes_of_operation()) {
        case OperationMode::CYCLIC_SYNC_POSITION:
            if (position_endless_) {
                return applied_position_rad_;
            }
            return decode_position_word(pdo_data_.target_position_word, channels_.position, pdo_handler_.joint_offset(),
                                        flip)
                .value_or(std::numeric_limits<double>::quiet_NaN());
        // Velocity and torque decode the word the handler writes (encode, then
        // back): not the stored count, which differs where the flip saturates.
        case OperationMode::CYCLIC_SYNC_VELOCITY:
            return to_angular_velocity(PulsePerSecond::from_hardware(pdo_data_.target_velocity.to_hardware(flip), flip),
                                       channels_.velocity)
                .value();
        default:
            return to_torque(TorqueCount::from_hardware(pdo_data_.target_torque.to_hardware(flip), flip),
                             channels_.torque)
                .value();
    }
}

OperationMode DeviceServo::get_modes_of_operation() const {
    return static_cast<OperationMode>(pdo_data_.modes_of_operation);
}

bool DeviceServo::set_command(NewtonMeter torque) {
    // Reject a non-finite command before the clamp. std::clamp passes NaN straight
    // through (both NaN < lo and hi < NaN are false), so clamping alone would store NaN,
    // and the RT write() would then cast NaN→int16 (UB) into the drive's
    // torque count. A spurious NaN/inf must not perturb the setpoint: leave the last
    // good command in place and report rejection. (Cheap finite check, no throw, RT-safe.)
    if (!std::isfinite(torque.value())) {
        return false;
    }
    target_torque_ = std::clamp(torque, min_torque_, max_torque_);
    return (torque >= min_torque_ && torque <= max_torque_);
}

Radian DeviceServo::get_actual_position() const {
    if (position_endless_) {
        return Radian(actual_position_rad_);
    }
    return to_angle(pdo_data_.position_actual_value, channels_.position);
}

RadianPerSecond DeviceServo::get_actual_velocity() const {
    return to_angular_velocity(pdo_data_.velocity_actual_value, channels_.velocity);
}

NewtonMeter DeviceServo::get_actual_torque() const {
    return to_torque(pdo_data_.torque_actual_value, channels_.torque);
}

uint16_t DeviceServo::get_statusword() const {
    return pdo_data_.statusword;
}

int8_t DeviceServo::get_modes_of_operation_display() const {
    return pdo_data_.modes_of_operation_display;
}

ModeDisplayTiming DeviceServo::mode_display_timing() const {
    return mode_display_timing_;
}

uint16_t DeviceServo::error_code() const {
    // Storage only: populated by the read path when the ErrorCode PDO role
    // is declared+mapped, 0 otherwise. Emission is the controller fault
    // path's job (RtEventEmitter has the joint id; emitting here too would
    // double-report every fault edge).
    return pdo_data_.error_code;
}

ServoState DeviceServo::get_servo_state() const {
    return state_machine_.get_current_state();
}

bool DeviceServo::status_recognized() const {
    return state_machine_.status_recognized();
}

bool DeviceServo::transition_state(ServoState target_state) {
    // Runs on the cyclic thread: a refusal is the return value, never a log line.
    if (!state_machine_.can_transition_to(target_state)) {
        return false;
    }

    uint16_t controlword = state_machine_.get_controlword_for_transition(target_state);
    pdo_data_.controlword = controlword;
    return true;
}

bool DeviceServo::is_faulted() const {
    return state_machine_.is_faulted();
}

bool DeviceServo::fault_reset_eligible() const {
    // Narrower than is_faulted(), on purpose, and in two ways.
    //
    // FAULT only: FAULT_REACTION_ACTIVE is the drive still reacting, and CiA
    // 402 transition 14 carries it into FAULT by itself; publishing into FRA
    // would spend an attempt on a state that is about to change anyway.
    //
    // Recognized only: an unrecognized statusword makes the state machine HOLD
    // the previous state, so a FAULT held across a run of unknown words would
    // keep accepting automatic attempts, burn the whole allowance and emit
    // SERVO_FAULT_RESET_EXHAUSTED before the first pulse ever went out.
    return state_machine_.status_recognized() && state_machine_.get_current_state() == ServoState::FAULT;
}

bool DeviceServo::reset_fault() {
    if (!fault_reset_eligible() || reset_pulse_owed_ > 1) {
        return false;
    }
    request_fault_reset();
    return true;
}

void DeviceServo::request_fault_reset() {
    fault_reset_request_.store(true, std::memory_order_release);
}

void DeviceServo::discard_fault_reset_request() {
    fault_reset_request_.store(false, std::memory_order_release);
}

}  // namespace plem::fieldbus::ethercat
