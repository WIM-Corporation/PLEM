// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/// @file ethercat_test_probes.hpp
/// @brief Test-only probe subclasses of EtherCATController and EtherCATBus.
///
/// Header-only test support: installed only when
/// BUILD_TESTING is ON, so release builds carry none of these accessors. The
/// release headers still name the probes as friends and keep the two test
/// data members (the release post hook and the quiesce timeout); a friend name
/// is not an access barrier.
/// Each probe is a friend of the class it derives from; a test derives from
/// the probe (or uses it directly) instead of from the production class.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/controller.hpp"

namespace plem::fieldbus::test {

template <size_t DOF>
class EtherCATControllerTestProbe : public ethercat::EtherCATController<DOF> {
  public:
    using ethercat::EtherCATController<DOF>::EtherCATController;

    /// Start the native CiA 402 disable sequence on the selected joints
    /// (SWITCHED_ON -> READY_TO_SWITCH_ON -> SWITCH_ON_DISABLED: drive power
    /// off = brake closed on a power-coupled drive), full brake groups, as a
    /// fresh engage. Production reaches it through the brake request step.
    void set_brake_engage_sequence(uint8_t mask) noexcept {
        this->start_engage(mask, /*preserve_active=*/false);
    }

    /// Snapshot of the pending request masks (race pin).
    [[nodiscard]] std::pair<uint8_t, uint8_t> brake_request_masks_for_test() const noexcept {
        return {this->brake_release_request_mask_.load(std::memory_order_acquire),
                this->brake_engage_request_mask_.load(std::memory_order_acquire)};
    }

    /// Admission observation (the quiesce linearization pin).
    [[nodiscard]] bool release_admission_closed_for_test() const noexcept {
        return this->release_admission_closed_.load(std::memory_order_acquire);
    }

    /// Invoked between the OR and the opposite-AND of a release post, inside
    /// the critical section (interleave pin).
    void set_brake_release_post_pair_hook_for_test(std::function<void()> hook) {
        this->brake_release_post_pair_hook_for_test_ = std::move(hook);
    }

    /// Invoked at the start of every step_brake_requests_for_test(), before the
    /// request masks are consumed (a post landing mid-step: interleave pin).
    void set_brake_step_entry_hook_for_test(std::function<void()> hook) {
        brake_step_entry_hook_ = std::move(hook);
    }

    /// Forced brake-state override (lets a harness without a live loop enter
    /// the not-all-engaged quiesce path).
    void set_brake_state_for_test(uint8_t mask) noexcept {
        this->brake_state_mask_.store(static_cast<uint8_t>(mask & this->all_joints_mask()), std::memory_order_release);
    }

    /// Shortens the quiesce attempt timeout (the 3×3 s wall clock otherwise).
    void set_quiesce_attempt_timeout_for_test(int64_t timeout_ns) noexcept {
        this->quiesce_attempt_timeout_ns_ = timeout_ns;
    }

    /// Shortens the stall window (2 s of bus cycles in production).
    void set_sequence_stall_threshold_for_test(uint32_t cycles) noexcept {
        this->sequence_stall_threshold_cycles_ = cycles;
    }

    /// One production brake-cycle step: a synchronous harness consumes the
    /// posted request masks itself (run_loop's step is unreachable there).
    /// `motion_possible` is this cycle's bus status value as the harness reads it.
    void step_brake_requests_for_test(core::TriState motion_possible = core::TriState::UNDEFINED) {
        if (!this->mode_flags_) {
            // run_loop refuses to start without flags; the harness never calls run_loop.
            this->mode_flags_ = std::make_shared<plem::core::ControlModeFlags>();
        }
        if (brake_step_entry_hook_) {
            brake_step_entry_hook_();
        }
        this->step_brake_requests(motion_possible);
    }

  private:
    std::function<void()> brake_step_entry_hook_;
};

class EtherCATBusTestProbe : public ethercat::EtherCATBus {
  public:
    using ethercat::EtherCATBus::EtherCATBus;

    /// What set_servo_config() / set_io_config() / set_startup_sdos() /
    /// set_stored_sdos() / retain_pdo_mapping_storage() received, and whether an IO device factory
    /// was injected: the pre-initialize injection surface.
    [[nodiscard]] const std::vector<ethercat::ServoInfo>& injected_servo_config() const noexcept {
        return injected_servo_config_;
    }
    [[nodiscard]] const std::vector<ethercat::IOInfo>& injected_io_config() const noexcept {
        return injected_io_config_;
    }
    [[nodiscard]] const std::vector<ethercat::SlaveSdos>& injected_startup_sdos() const noexcept {
        return injected_startup_sdos_;
    }
    [[nodiscard]] const std::vector<ethercat::SlaveSdos>& injected_stored_sdos() const noexcept {
        return injected_stored_sdos_;
    }
    [[nodiscard]] const std::vector<ethercat::SlaveExpectedIdentity>& injected_expected_identity() const noexcept {
        return injected_expected_identity_;
    }
    [[nodiscard]] const std::vector<std::shared_ptr<const void>>& retained_pdo_mapping_storage() const noexcept {
        return retained_pdo_mapping_storage_;
    }
    [[nodiscard]] bool has_io_device_factory() const noexcept {
        return static_cast<bool>(io_device_factory_);
    }
};

}  // namespace plem::fieldbus::test
