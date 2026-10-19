// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/// @file device_servo_test_access.hpp
/// @brief Test-only access to DeviceServo internals (friended by the class).
/// Lives outside the installed headers: the seams below are not API.

#include <cstdint>
#include <functional>
#include <utility>

#include "plem/fieldbus/ethercat/servo/device.hpp"

namespace plem::fieldbus::test {

using ethercat::DeviceServo;

struct DeviceServoTestAccess {
    /// Overwrite the standing controlword, bypassing the state machine and the
    /// fault-reset gate (a drive register left holding an arbitrary command).
    static void set_controlword(DeviceServo& servo, uint16_t controlword) {
        servo.pdo_data_.controlword = controlword;
    }

    /// Run inside write(), immediately after the request is acquired and
    /// before the frame is decided: the only instant where a concurrent
    /// publication's fate is interesting, and no thread hammering can place
    /// one there reliably.
    static void set_post_acquire_hook(DeviceServo& servo, std::function<void()> hook) {
        servo.post_acquire_hook_for_test_ = std::move(hook);
    }

    /// Place a seeded endless axis at `count`, with the previous word
    /// congruent to it (acc ≡ word mod 2^32); reaching the domain edge by
    /// streaming would take 2^17 wraps.
    static void set_position_count(DeviceServo& servo, int64_t count) {
        servo.position_count_ = count;
        servo.prev_position_word_ = static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint64_t>(count)));
    }
};

}  // namespace plem::fieldbus::test
