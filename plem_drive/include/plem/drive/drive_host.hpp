// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_host.hpp
/// @brief DriveHost<N>, the drive-layer executor: N
/// CiA 402 drives on one EtherCAT bus, commands in (cst Nm, csp rad, csv
/// rad/s) / actuals out, no URDF, no torque stack. The RT loop (EtherCATController<N> with declared
/// drive limits) owns the safety surface: limit → INTERNAL protective stop, WKC /
/// command-freshness → EXTERNAL protective stop, forced brake engage, and the torque
/// gate (motion ∧ all drives enabled ∧ ¬protective stop, else 0 Nm). The host owns
/// the command path (write → seqlock, the ONE writer), the lifecycle and
/// the mode intent (enable/ramp_stop/disable).
///
/// Thread contract: enable/ramp_stop/disable/stop/reset are serialized on one
/// non-RT thread. write() is the ONE seqlock writer — one caller thread at
/// a time (RT-callable: bool, noexcept, no allocation); two concurrent
/// write() callers would tear the command. read() may run on any thread.
/// One-shot lifecycle: create → start → (enable/write/read/disable/reset)*
/// → stop; no restart (build a new host). If create() fails after the value
/// gate, the bus handed to it is consumed (servo config, retained graphs)
/// and must not be reused.
/// The ROS layer (src/ros/drive_host_adapter.hpp) wraps it behind IDriveHost for the
/// ros2_control hardware plugin.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/result.hpp>
#include <plem/core/threading/rt_thread_config.hpp>
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/core/types/joint_state.hpp>
#include <plem/core/types/shared_joint_data.hpp>
#include <plem/fieldbus/ethercat/bus.hpp>
#include <plem/fieldbus/ethercat/controller.hpp>

#include "plem/drive/models/drive_model_spec.hpp"

namespace plem::drive {

/// The drive layer's command-freshness budget: a released axis whose host stops
/// writing for longer latches the EXTERNAL protective stop. Below the fieldbus
/// default of 100 ms (fieldbus::kDefaultCommandStaleTimeoutNs), from measurement:
/// an rclpy publisher on the test platform under load (10 min, load 8–9) peaked
/// at 41 ms between messages at 200 Hz and 35 ms at 1 kHz, never over 50 ms.
inline constexpr int64_t kCommandStaleTimeoutNs = 50'000'000;

/// Host deployment knobs. Declared at namespace scope so the type does not
/// depend on N: a caller that erases N (the ROS layer's IDriveHost) names a
/// single DriveHostParams.
struct DriveHostParams {
    int rt_priority = fieldbus::ethercat::ControllerConfig{}.rt_priority;  ///< the bus loop's default
    /// Default (nullopt) = the isolated CPU the kernel reports, if any.
    std::optional<core::threading::CpuAffinity> cpu_affinity{};
    /// The bench bootstrap (drive-model schema §6): let an unapproved model
    /// open motion so the bench can measure what its approval record will
    /// bind to. Nothing else changes: no torque or time cap; the run's
    /// evidence is bench evidence only, and the caller says so in its log.
    /// Default false: an unapproved model cannot enable().
    bool bench_unapproved = false;
};

template <size_t N>
class DriveHost {
  public:
    /// Race-free snapshot for non-RT readers: seqlock actuals, atomic flags,
    /// the brake masks and the per-cycle statusword (0x6041) mirror.
    struct State {
        core::JointActualValues<N> actual{};
        uint8_t protective_stop = 0;  ///< ProtectiveStopSource bits
        bool operation_enabled = false;
        uint8_t brake_engaged = 0;            ///< bit i = axis i engaged
        uint8_t brake_operation_pending = 0;  ///< bit i = axis i release/engage unsettled
        bool stopping = false;                ///< a ramp_stop() is running
        std::array<uint16_t, N> statusword{};
        /// The last cycle's working counter was complete: every slave in OP
        /// (EtherCATController::process_data_complete).
        bool process_data_complete = false;
    };

    /// Value gate → graph gate → ServoInfo carry → bus injection + initialize
    /// (servo_count must equal N) → RT controller wiring. No thread starts.
    /// `bus` is the caller's (injected, never created here).
    [[nodiscard]] static core::Result<std::unique_ptr<DriveHost>> create(
        const models::DriveModelSpec<N>& spec, std::shared_ptr<fieldbus::ethercat::EtherCATBus> bus,
        const DriveHostParams& params);

    ~DriveHost();
    DriveHost(const DriveHost&) = delete;
    DriveHost& operator=(const DriveHost&) = delete;

    /// RT memory hardening + the RT loop. Allowed with approval UNAPPROVED
    /// ("assembly possible, motion refused"); enable() is the motion gate.
    [[nodiscard]] core::Result<void> start();

    /// ramp_stop() and wait for it to end → disable → quiesce (engage confirmed
    /// through the still-cycling loop, timeout-bounded) → loop stop. Returns the quiesce outcome so the
    /// caller can report an unconfirmed brake (no silent drop); a second
    /// call returns the stored outcome. A run that released nothing (before
    /// start(), or no enable() reached its release) quiesces nothing: its
    /// outcome has never_released set; check that before reading
    /// fully_engaged() as a confirmation.
    fieldbus::ethercat::BrakeQuiesceOutcome stop();

    /// Motion intent. Refused (Result error) unless: started, approval ==
    /// APPROVED (or DriveHostParams::bench_unapproved), no protective stop latched (reset() and wait for
    /// protective_stop == 0; the clear is asynchronous), no stop running (ramp_stop), and (on a bus
    /// with a cst axis) a host write() landed inside kCommandStaleTimeoutNs (torque
    /// must meet a live command). A bus without a cst axis holds the position
    /// it finds (a csv axis 0) until the first write() after this call, and
    /// commands written before it are not applied. Idempotent
    /// on the physical state (every brake released and settled → ok, no
    /// re-post: a release re-posted to an enabled drive cycles it through
    /// READY_TO_SWITCH_ON). After a protective stop's forced engage the brakes read
    /// engaged again, so the next enable() re-posts the release.
    [[nodiscard]] core::Result<void> enable();

    /// Drop motion intent (the gate writes 0 Nm from this cycle on) and
    /// post the brake engage immediately, even while axes are moving.
    void disable();

    /// Requested stop: every csp and csv axis decelerates to rest at its
    /// file's limits.acceleration_rad_s2, then motion is switched off and the
    /// brakes engage; if the stop is not confirmed in time, every enabled drive gets Quick stop (they
    /// ramp down at their 0x6085 and end in 0x605A's state: 2 = Switch on
    /// disabled). Asynchronous: poll read() (stopping). On a bus with a cst axis,
    /// or when nothing is released, it does disable(). EtherCATController::request_ramp_stop has
    /// the timing.
    void ramp_stop();

    /// Host command, one value per axis in that axis's declared mode's unit
    /// (CST Nm, CSP rad, CSV rad/s; the RT step checks CSP/CSV against the
    /// limits and stops the bus on a violation). The ONE seqlock writer; one
    /// stamp per accepted array, which is also the freshness clock. false =
    /// rejected (size != N, a non-finite value, or a future or non-positive
    /// `sampled_ns`: <= 0 or more than kCommandStaleTimeoutNs ahead of
    /// CLOCK_MONOTONIC now): nothing written, nothing
    /// stamped: a host that keeps sending garbage goes stale and trips the
    /// EXTERNAL protective stop. `rate` (empty, or N finite values): how fast each value
    /// changes, per second (a csp axis's planned speed). `sampled_ns`
    /// (CLOCK_MONOTONIC, core::now_ns): the instant the values describe (when
    /// the host read the bus and planned from it, not when it writes). The loop
    /// advances a csp value by rate x (its wake - sampled_ns), so a stamp
    /// taken later than the plan's instant puts that delay x speed into the
    /// target (at speed, a CSP_STEP stop); it is also the freshness clock.
    [[nodiscard]] bool write(std::span<const double> command, std::span<const double> rate,
                             int64_t sampled_ns) noexcept;

    /// false = the actuals seqlock read was torn (retry); the other fields
    /// are not filled either.
    [[nodiscard]] bool read(State& out) const noexcept;

    /// Operator reset: drive fault reset (non-RT bus call) + set the flag the
    /// RT safety step consumes (INTERNAL and EXTERNAL cleared optimistically;
    /// a persisting condition re-asserts). Asynchronous: poll read().
    void reset();

    /// Per slave, in the spec's slaves[] order: the digest of the mapping
    /// this boot scanned ("sha256:…"), "" for a declared slave (the bus's
    /// cached Step 3 value, copied once at create()). Static for the host's life.
    [[nodiscard]] const std::vector<std::string>& pdo_mapping_digests() const noexcept {
        return pdo_mapping_digests_;
    }

    /// Per-host monitoring context (RT events/samples): the bench runner
    /// and the ROS layer drain it; nobody else does. The bus thread is its
    /// only producer: emitting into it from another thread breaks the
    /// single-producer queues (MonitoringContext).
    [[nodiscard]] core::monitoring::MonitoringContext* monitoring_context() noexcept {
        return &monitoring_;
    }

  private:
    DriveHost() = default;
    /// stop() epilogue: one warning per endless axis whose count left int32.
    void warn_endless_counts_past_int32() const;

    models::Approval approval_{models::Approval::UNDECLARED};
    bool bench_unapproved_{false};
    std::array<char, models::kMaxModelNameLength> model_name_{};
    std::array<std::array<char, models::kMaxAxisNameLength>, N> axis_names_{};
    std::array<uint32_t, N> endless_encoder_resolution_{};  ///< 0 = bounded axis
    bool streams_torque_{true};                             ///< a cst axis on the bus (detail::drive_streams_torque)
    std::vector<std::string> pdo_mapping_digests_;
    std::shared_ptr<fieldbus::ethercat::EtherCATBus> bus_;
    std::shared_ptr<core::ControlModeFlags> flags_;
    std::shared_ptr<core::SharedJointData<N>> shared_;
    core::monitoring::MonitoringContext monitoring_;
    std::unique_ptr<fieldbus::ethercat::EtherCATController<N>> controller_;
    std::atomic<int64_t> last_write_ns_{0};
    bool started_{false};
    bool stopped_{false};
    bool released_{false};  ///< an enable() reached its release since start
    fieldbus::ethercat::BrakeQuiesceOutcome stop_outcome_{};
};

extern template class DriveHost<1>;
extern template class DriveHost<2>;
extern template class DriveHost<3>;
extern template class DriveHost<4>;
extern template class DriveHost<5>;
extern template class DriveHost<6>;
extern template class DriveHost<7>;
extern template class DriveHost<8>;

}  // namespace plem::drive
