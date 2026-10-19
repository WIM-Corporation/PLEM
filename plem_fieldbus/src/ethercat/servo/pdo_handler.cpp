// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/fieldbus/ethercat/servo/pdo_handler.hpp"

namespace plem::fieldbus::ethercat {

ServoPDOHandler::ServoPDOHandler(const Pulse& joint_offset, bool flip, bool position_word_target)
    : offsets_{}, joint_offset_(joint_offset.value()), flip_(flip), position_word_target_(position_word_target) {}

void ServoPDOHandler::write(const ServoPDOData& data, uint8_t* domain_pd) const {
    // Every field guarded: a slave need not map every object, and an unmapped
    // offset would otherwise alias offset 0, writing a target on top of
    // whatever object registered there.
    if (offsets_.controlword != kUnmappedPdoOffset) {
        EC_WRITE_U16(domain_pd + offsets_.controlword, data.controlword);
    }
    if (offsets_.modes_of_operation != kUnmappedPdoOffset) {
        EC_WRITE_S8(domain_pd + offsets_.modes_of_operation, data.modes_of_operation);
    }
    if (offsets_.target_position != kUnmappedPdoOffset) {
        EC_WRITE_S32(domain_pd + offsets_.target_position,
                     position_word_target_ ? data.target_position_word
                                           : data.target_position.to_hardware(joint_offset_, flip_));
    }
    if (offsets_.target_velocity != kUnmappedPdoOffset) {
        EC_WRITE_S32(domain_pd + offsets_.target_velocity, data.target_velocity.to_hardware(flip_));
    }
    if (offsets_.target_torque != kUnmappedPdoOffset) {
        EC_WRITE_S16(domain_pd + offsets_.target_torque, data.target_torque.to_hardware(flip_));
    }
}

void ServoPDOHandler::read(ServoPDOData& data, const uint8_t* domain_pd) const {
    // Each guarded like write(): an unmapped offset would read the controlword
    // as a position. A guarded-out field keeps its previous value.
    if (offsets_.statusword != kUnmappedPdoOffset) {
        data.statusword = EC_READ_U16(domain_pd + offsets_.statusword);
    }
    if (offsets_.modes_of_operation_display != kUnmappedPdoOffset) {
        data.modes_of_operation_display = EC_READ_S8(domain_pd + offsets_.modes_of_operation_display);
    }
    if (offsets_.position_actual_value != kUnmappedPdoOffset) {
        set_position_actual_value(data, EC_READ_S32(domain_pd + offsets_.position_actual_value));
    }
    if (offsets_.velocity_actual_value != kUnmappedPdoOffset) {
        data.velocity_actual_value =
            PulsePerSecond::from_hardware(EC_READ_S32(domain_pd + offsets_.velocity_actual_value), flip_);
    }
    if (offsets_.torque_actual_value != kUnmappedPdoOffset) {
        data.torque_actual_value =
            TorqueCount::from_hardware(EC_READ_S16(domain_pd + offsets_.torque_actual_value), flip_);
    }
    // Not sign-flipped: this is a fault code, not a signed physical quantity.
    if (offsets_.error_code != kUnmappedPdoOffset) {
        data.error_code = EC_READ_U16(domain_pd + offsets_.error_code);
    }
}

void ServoPDOHandler::set_position_actual_value(ServoPDOData& data, int32_t word) const noexcept {
    data.position_actual_value_word = word;
    data.position_actual_value = Pulse::from_hardware(word, joint_offset_, flip_);
}

ServoPDOOffsets& ServoPDOHandler::get_offsets() {
    return offsets_;
}

}  // namespace plem::fieldbus::ethercat
