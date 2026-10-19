// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <plem/core/constants.hpp>
#include <plem/core/result.hpp>
#include <plem/core/threading/rt_thread_config.hpp>
#include <plem/core/timing/periodic_timer_rt.hpp>
#include <plem/core/types/control_mode_flags.hpp>
#include <plem/core/types/motion_limit.hpp>
#include <plem/core/types/shared_joint_data.hpp>
#include <plem/fieldbus/ethercat/drive_safety.hpp>
#include <plem/fieldbus/watchdog.hpp>

#include "plem/fieldbus/ethercat/brake_logic.hpp"
#include "plem/fieldbus/ethercat/bus.hpp"
#include "plem/fieldbus/ethercat/controller_config.hpp"
#include "plem/fieldbus/ethercat/state_machine.hpp"
#include "plem/fieldbus/ethercat/utils/servo_conversions.hpp"

namespace plem::core::monitoring {
class MonitoringContext;
}  // namespace plem::core::monitoring

namespace plem::fieldbus::test {
template <size_t DOF>
class EtherCATControllerTestProbe;  // test-only seams (plem/fieldbus/test/, not a release header)
}  // namespace plem::fieldbus::test

namespace plem::fieldbus::ethercat {

/// @brief Explicit quiesce result. After a release the masks are measured
/// facts: a bit absent from every mask was never confirmed either way, and the
/// message layer must say "unconfirmed", never guess a physical state. With
/// never_released set nothing was measured; check it before reading
/// fully_engaged() as a confirmation.
struct BrakeQuiesceOutcome {
    uint8_t engaged_mask{0};  ///< groups confirmed engaged (every axis settled)
    uint8_t timeout_mask{0};  ///< joints not confirmed engaged at the deadline
    /// Per-joint statusword snapshot at outcome time (drive-quiet observation;
    /// the masks alone cannot even prove "drive disabled").
    std::array<uint16_t, core::kMaxBusAxes> statuswords{};
    uint8_t all_joints_mask{0};
    /// No brake was released this run, so nothing was observed: the masks are
    /// the startup state (every joint engaged), not a measurement, and
    /// fully_engaged() is then no confirmation. Set by the host that knows no
    /// release happened (the drive layer's stop path); quiesce_brakes() itself
    /// leaves it false.
    bool never_released{false};

    [[nodiscard]] bool fully_engaged() const noexcept {
        return engaged_mask == all_joints_mask;
    }
};

/**
 * @brief Real-time EtherCAT controller (application layer)
 *
 * Manages the cyclic PDO loop (period Config::cycle_time_us) and the CiA 402
 * state transitions:
 * - Receives PDO data from servos
 * - Publishes joint states through SharedJointData
 * - Executes CiA 402 state machine sequences
 * - Applies the host's commands (cst torque, csp position, csv velocity);
 *   a csp or csv axis needs set_drive_limits() first, or start() refuses
 * - Sends PDO data to servos
 *
 * This class implements the application layer of the industrial EtherCAT stack:
 * - Application Layer (this class): State sequences, command processing, timing
 * - EtherCAT Master (EtherCATBus): PDO/SDO communication only
 * - Network Driver: Packet send/receive
 *
 * @thread_safety This class manages its own thread.
 */
template <size_t DOF = 6>
class EtherCATController {
    // RT loop resources. Declared first so they are constructed first and
    // destroyed last: the destructor joins the thread before any member
    // the loop reads goes away.
    ControllerConfig config_;
    std::shared_ptr<plem::core::SharedJointData<DOF>> shared_data_;
    plem::core::monitoring::MonitoringContext* monitoring_context_ = nullptr;
    std::atomic<bool> running_{false};
    std::unique_ptr<plem::core::PeriodicTimerRT> timer_;
    std::thread thread_;

  public:
    using Config = ControllerConfig;

    explicit EtherCATController(const Config& config);

    /**
     * @brief Destructor - stops the thread if it is still running
     */
    virtual ~EtherCATController();

    EtherCATController(const EtherCATController&) = delete;
    EtherCATController& operator=(const EtherCATController&) = delete;
    EtherCATController(EtherCATController&&) = delete;
    EtherCATController& operator=(EtherCATController&&) = delete;

    /**
     * @brief Set the shared data the RT loop exchanges joint data through (required before start())
     * @param data Shared pointer to SharedJointData<DOF>
     * @throws std::logic_error if the RT loop is running (see throw_if_running)
     */
    void set_shared_data(std::shared_ptr<plem::core::SharedJointData<DOF>> data) {
        throw_if_running("set_shared_data");
        shared_data_ = std::move(data);
    }

    /// @brief Set per-instance monitoring context for thread-local routing
    /// @param ctx Monitoring context (nullptr = use global default)
    /// @throws std::logic_error if the RT loop is running (see throw_if_running)
    void set_monitoring_context(plem::core::monitoring::MonitoringContext* ctx) {
        throw_if_running("set_monitoring_context");
        monitoring_context_ = ctx;
    }

    /**
     * @brief Start the control loop thread
     * @return Result<void> error if preconditions not met
     */
    [[nodiscard]] core::Result<void> start();

    /**
     * @brief Stop the control loop thread
     *
     * Blocks until thread exits.
     */
    void stop();

    [[nodiscard]] bool is_running() const;

    /**
     * @brief Set the EtherCAT bus to use
     *
     * Stores the typed EtherCATBus pointer used by the RT loop.
     */
    void set_bus(std::shared_ptr<EtherCATBus> bus);

    /**
     * @brief Set the ControlModeFlags shared with the host
     */
    void set_mode_flags(std::shared_ptr<plem::core::ControlModeFlags> flags);

    /// Type-erased probe of the committed mode word: one atomic load of the
    /// host's committed mode state, projected to the brake-relevant bits.
    using ModeProbe = CommittedModeSample (*)(const void*) noexcept;

    /**
     * @brief Wire the committed-mode authority probe.
     *
     * The probe must read the same lock-free word the host's mode/safety
     * commits write and project both brake-relevant bits from that one load:
     * a flag derived and stored after a commit can be overwritten by a stale
     * writer, a release posted before that store could be discarded against
     * a mode that is committed, and the recovery-lane bit must fold the
     * safety state in (the RT protective_stop flag can rise before the host commits
     * its protective stop safety state, so the mode bit alone would read a stale
     * FREEDRIVE commit as recovery). Reading the commit word itself at
     * consume time has none of these holes. `ctx` must outlive the RT loop.
     * Non-RT: call before the loop starts; throw_if_running() refuses it
     * once start() has returned. Without a probe the brake consume falls
     * back to the ControlModeFlags pair, correct for a host whose flags are
     * the committed mode word (no separate mode state machine) and for test
     * harnesses.
     */
    void set_committed_mode_probe(ModeProbe probe, const void* ctx) {
        this->throw_if_running("set_committed_mode_probe");
        mode_probe_ = probe;
        mode_probe_ctx_ = ctx;
    }

    // Brake Control (atomic flag-based mode-path API)

    /**
     * @brief Request brake release for all joints (enable motion)
     *
     * Convenience overload that releases all configured DOF joints.
     *
     * @return false = release admission is closed (shutdown quiesce); the
     *         request was not queued (no silent drop).
     */
    [[nodiscard]] bool request_brake_release();

    /**
     * @brief Request brake release for selected joints (enable motion)
     *
     * Sets an atomic bitmask that the RT loop will consume. This method is
     * thread-safe and can be called from any thread. Bits above DOF are ignored.
     *
     * Admission: refused (false, nothing queued) once
     * close_brake_release_admission() ran. The post-time check is the
     * reporting surface; the authoritative gate is the RT consume-time
     * re-check; a request that raced past this check is still discarded at
     * consumption after the close.
     *
     * @param joint_mask Bitmask of joints to release (bit 0 = joint 0)
     * @return false = admission closed, request not queued
     */
    [[nodiscard]] bool request_brake_release(uint8_t joint_mask);

    /**
     * @brief One-way close of brake-release admission (the shutdown-quiesce
     * entry; never reopened for this controller's lifetime). Idempotent.
     *
     * After this: release requests are refused (and any already-queued
     * release mask is discarded at the next RT consumption). Engage is never
     * gated; the safe direction stays available. quiesce_brakes() calls it
     * first; a host calls it alone inside its own mode-commit critical
     * section to linearize the close against its mode commits.
     */
    void close_brake_release_admission() noexcept;

    /// @brief Is brake-release admission still open?
    [[nodiscard]] bool is_brake_release_admission_open() const noexcept;

    /**
     * @brief Request brake engage for all joints (disable motion)
     *
     * Convenience overload that engages all configured DOF joints.
     *
     */
    void request_brake_engage();

    /**
     * @brief Request brake engage for selected joints (disable motion)
     *
     * Sets an atomic bitmask that the RT loop will consume. This method is
     * thread-safe and can be called from any thread. Bits above DOF are ignored.
     *
     * @param joint_mask Bitmask of joints to engage (bit 0 = joint 0)
     */
    void request_brake_engage(uint8_t joint_mask);

    /**
     * @brief Current brake state as bitmask
     *
     * Each bit represents a joint: 1 = brake engaged, 0 = brake released.
     */
    [[nodiscard]] uint8_t brake_engaged() const noexcept;

    /**
     * @brief Joints with an unsettled brake operation: a posted-but-unconsumed
     * request or an active CiA 402 state sequence.
     * While a joint's bit is set here, its brake_engaged() bit may still be the
     * last confirmed value, not a fresh terminal; observers checking
     * engagement (e.g. a caller sequencing per-joint brake operations) must require this mask to
     * be clear before trusting brake_engaged(). One release store per RT cycle
     * (the group/sequence mirror); the request masks are read live.
     */
    [[nodiscard]] uint8_t brake_operation_pending() const noexcept {
        // Two correctness devices compose here:
        // 1. Read order: requests first, busy last. On the RT
        //    consume path a bit leaves a request mask only via fetch_and,
        //    sequenced after its busy.fetch_or; an observer that reads
        //    request==0 (acquire of the fetch_and's result) has that
        //    fetch_or happen-before its later busy load.
        // 2. Post seqlock: a direction-swap post writes two words
        //    (OR own mask, AND-clear the opposite); read order alone
        //    cannot snapshot two atomics consistently (read release before
        //    the OR, engage after the AND ⇒ union 0 that never existed).
        //    Posts bump brake_request_version_ around the pair (odd = in
        //    progress, under the post mutex); this getter retries until it
        //    reads a stable even version. Posts are rare and the critical
        //    section is a few instructions, so the retry is bounded in
        //    practice; this getter is a non-RT observer.
        for (int attempt = 0; attempt < 64; ++attempt) {
            const uint32_t version = brake_request_version_.load(std::memory_order_acquire);
            if ((version & 1u) != 0) {
                continue;
            }
            const uint8_t requested_release = brake_release_request_mask_.load(std::memory_order_acquire);
            const uint8_t requested_engage = brake_engage_request_mask_.load(std::memory_order_acquire);
            const uint8_t busy = brake_operation_busy_mask_.load(std::memory_order_acquire);
            if (brake_request_version_.load(std::memory_order_acquire) == version) {
                return static_cast<uint8_t>(requested_release | requested_engage | busy);
            }
        }
        // Bounded-retry fallback: a writer descheduled mid-pair must
        // not spin this observer forever. An unreadable snapshot degrades to
        // the conservative answer: everything pending. Fail-closed: settled
        // gates simply keep polling; a fabricated settled 0 is impossible.
        return all_joints_mask();
    }

    /**
     * @brief Declare per-axis drive limits (drive layer: a host without a
     * control thread of its own). Arms the RT loop's drive safety step and torque gate
     * (drive_safety.hpp): every cycle the SI actuals are checked by
     * feedback_stop_cause → INTERNAL protective stop; an operator reset
     * (ControlModeFlags::reset_error, consumed here by exchange) clears
     * INTERNAL and EXTERNAL optimistically (a persisting WKC/stale condition
     * re-asserts EXTERNAL next cycle); and torque reaches a drive only while
     * motion_enabled ∧ all_drives_enabled ∧ ¬protective stop (0 Nm otherwise).
     * `effort_limit` is informational here; the drive-side torque clamp is
     * ServoInfo's. The gate reads live flags at the apply point; after an
     * optimistic clear under a persisting WKC fault it is open for one
     * cycle (the WKC watchdog re-asserts after send()).
     *
     * Never wire together with a host control thread on the same flags: both
     * would consume reset_error and check limits. A host whose control thread
     * owns limits and reset does not call this: the step then does not run and
     * the gate stays open (cst axes only: start() refuses a csp or csv axis
     * without declared limits). `limits.size()` must equal DOF and each limit needs
     * lower < upper, a finite velocity limit > 0 (it bounds every speed check
     * and the stop's settle band) and an effort limit > 0 (std::invalid_argument).
     * Non-RT, before start() (throw_if_running).
     *
     * Coupling: an axis given ±inf position limits must be built with
     * ServoInfo::position_endless: its position is the endless count, and
     * the only position stop it keeps is that count leaving its domain
     * (UNDECODABLE). The drive layer derives both from one declaration
     * (detail::axis_endless).
     */
    void set_drive_limits(std::span<const core::MotionLimit> limits);

    /**
     * @brief Drive-layer requested stop: each csp or csv axis's deceleration
     * (rad/s²) for request_ramp_stop(); 0 on a cst axis. `size()` must equal DOF
     * (std::invalid_argument). Non-RT, before start() (throw_if_running).
     */
    void set_drive_stop(std::span<const double> decelerations_rad_s2);

    /// Command-freshness budget (run_command_freshness_cycle); the default is
    /// kDefaultCommandStaleTimeoutNs. Non-RT, before start() (throw_if_running).
    void set_command_stale_timeout_ns(int64_t timeout_ns);

    /**
     * @brief Ask the loop for a requested stop (declared drive limits). On a
     * bus without a cst axis, with motion live, every csp and csv command
     * ramps from its current speed to rest at its set_drive_stop deceleration
     * (the host's commands are not applied meanwhile); once the commands rest
     * and every measured speed is within 1 % of its velocity limit, motion is
     * switched off and every brake engaged. If the stop is not confirmed within the
     * planned deceleration plus kRampStopSettleTimeoutNs, every enabled drive is
     * commanded Quick stop (RAMP_STOP_TIMEOUT); the drive then leaves Operation enabled,
     * which latches the EXTERNAL protective stop, and the stop ends once every
     * axis reads a brake-closed state or after kQuickStopTimeoutNs. With
     * nothing moving, or a cst axis on the bus, motion goes off and the brakes
     * engage at once. Thread-safe, any thread.
     */
    void request_ramp_stop() noexcept;

    /// true from request_ramp_stop() until that stop has ended.
    [[nodiscard]] bool ramp_stop_active() const noexcept {
        return ramp_stop_requested_.load(std::memory_order_acquire) ||
               ramp_stop_running_.load(std::memory_order_acquire);
    }

    /**
     * @brief Mark a release (call before posting it). On a bus without a cst
     * axis a command stamped before this mark is not applied: from the release
     * on, a csp axis holds the target it found and a csv axis 0 until the host
     * writes again, and the command-freshness watch arms on that write. It
     * rests while motion is off or a stop runs. A bus with a cst axis keeps
     * the stream contract (fresh torque before the release). Thread-safe.
     */
    void open_command_epoch() noexcept;

    /**
     * @brief Last statusword seen on axis `axis` (race-free mirror, refreshed
     * once per cycle; lags at most one cycle). The off-thread diagnostic
     * plane; DeviceServo::get_statusword() is the RT-owned PDO field.
     * Precondition: axis < DOF (the caller indexes its own axis count).
     */
    [[nodiscard]] uint16_t statusword(size_t axis) const noexcept {
        return statusword_mirror_[axis].load(std::memory_order_relaxed);
    }

    /// The last cycle's domain working counter was complete (EC_WC_COMPLETE):
    /// every slave answered its process data. A slave with output PDOs (every
    /// servo) answers them only in OP; an input-only slave already in SAFEOP.
    /// false before the first cycle. Any thread.
    [[nodiscard]] bool process_data_complete() const noexcept {
        return process_data_complete_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Install the joint→brake-group map.
     *
     * groups[i] = the joint mask sharing joint i's physical brake, including
     * joint i (the host wires its validated brake setup here). Every
     * brake op is expanded to whole groups at the sequencing edge: a shared
     * brake acts on every joint it serves, so a "single joint" op cannot
     * exist. A partial release would open the
     * gravity-loaded partner without powering its drive; a partial engage
     * would disable one axis while the partner holds the brake open,
     * unpowered and unbraked either way. An all-zero table declares nothing;
     * otherwise every groups[i] must include joint i, name only joints below DOF
     * and equal groups[j] for every joint j it names (std::invalid_argument
     * otherwise). Uninstalled default: an empty group table, so every joint is
     * its own group (one joint per brake). Non-RT:
     * call before the RT loop starts; throw_if_running() refuses it once
     * start() has returned.
     */
    void set_brake_setup(const std::array<uint8_t, DOF>& groups);

    /**
     * @brief Declare what the sequence does with a drive that faults
     * mid-transition (see FaultResetPolicy) and, for BOUNDED_AUTO_RESET,
     * the model's pacing. One rule decides the pair (validate_fault_reset:
     * NONE ↔ no pacing, BOUNDED ↔ a valid pacing); this setter adds the
     * single check only it can make: the interval must convert to a cycle
     * count that fits uint32 at this controller's period. A refused
     * policy/pacing/conversion is std::invalid_argument (a running loop is
     * the base's std::logic_error), before any member changes. Undeclared =
     * NONE. Non-RT: call before the RT loop starts (throw_if_running).
     */
    void set_fault_reset_policy(FaultResetPolicy policy, const std::optional<FaultResetPacing>& pacing);

    /**
     * @brief Shutdown brake quiesce: call this first in the stop order,
     * with the RT loop still alive (completion is confirmed
     * through it, timeout-bounded).
     *
     * Engages every not-yet-engaged group through the normal RT mailbox,
     * with release admission closed. Engage runs the native disable sequence
     * (drive power off = brake closed on a drive whose brake is power-coupled,
     * the coupling the brake setup must have been validated for);
     * confirmation is the sequence's group-atomic settlement. Retries a
     * bounded number of times (kQuiesceEngageAttempts); the caller owns the
     * hold/forced-progression decision on an incomplete outcome.
     *
     * Lifecycle scope: whether to quiesce is the host's stop-order choice;
     * a host with power-coupled shared brakes quiesces before stopping.
     *
     * @thread_safety Non-RT caller only (blocks/polls); the RT loop keeps
     *                cycling throughout.
     */
    [[nodiscard]] BrakeQuiesceOutcome quiesce_brakes() noexcept;

  protected:
    [[nodiscard]] const char* logger_tag() const;

    /// The main loop executed in the RT thread, until running_ drops. Applies
    /// the RT setup (scheduling, affinity, stack prefault) before its first cycle.
    void run_loop();

    /// @brief Validate preconditions before start() spawns the RT thread
    /// (bus, mode flags, shared data, drive count)
    [[nodiscard]] core::Result<void> validate_preconditions() const;

    /**
     * @brief Apply real-time scheduling parameters to thread
     * @return true on success, false on failure (continues with default)
     */
    bool apply_rt_scheduling();

    /**
     * @brief Apply CPU affinity to thread
     * @return true on success, false on failure (continues without pinning)
     */
    bool apply_cpu_affinity();

    /**
     * @brief Refuse a wiring setter that arrives after start().
     *
     * Every setter that guards itself with this binds something the RT loop
     * dereferences per cycle. Swapping one mid-run changes what a cycle
     * already in flight is reading, and the loop has no way to notice: a
     * rebind is a wiring defect, so it fails at the call rather than
     * corrupting a cycle. stop() reopens wiring; the guard is about the
     * loop being live, not about the controller having run before.
     *
     * This is a sequencing guard, not a synchronization primitive. Setters,
     * start() and stop() belong to the host's one wiring thread, so no
     * setter can interleave inside start() or stop();
     * what it catches is the ordering mistake: a setter called after start()
     * returned and before stop() returned. Two threads wiring one controller
     * is not a supported configuration and is not detected.
     *
     * @param setter Name of the calling setter (appears in the message)
     * @throws std::logic_error if the RT loop is running
     */
    void throw_if_running(const char* setter) const {
        if (running_.load(std::memory_order_acquire)) {
            throw std::logic_error(std::string(logger_tag()) + ": " + setter +
                                   " called while the RT loop is running — the loop binds its dependencies at "
                                   "start(); stop() first, rewire, then start() again");
        }
    }

    /**
     * @brief One watchdog evaluation + protective-stop latching step
     *
     * Called by run_loop() once per cycle. Separate so the WKC → protective-stop
     * wiring is unit-testable without a live EtherCAT master.
     */
    void run_watchdog_cycle(bool wkc_ok) noexcept;

    /**
     * @brief One command-freshness evaluation + protective-stop latching step
     *
     * A hung/starved host control thread freezes the torque
     * command and silences the software limit monitoring that lives on that
     * thread; this is the independent observer. Called by run_loop() once per
     * cycle.
     * Separate like run_watchdog_cycle so the wiring is unit-testable.
     *
     * @param update_ns CLOCK_MONOTONIC stamp of the latest observed torque write
     * @param now_ns    CLOCK_MONOTONIC now
     */
    void run_command_freshness_cycle(int64_t update_ns, int64_t now_ns) noexcept;

    /**
     * @brief Execute one step of the current per-servo state sequences
     *
     * Each servo progresses through its own sequence, so joints can be
     * released or engaged individually; the all-joint API is a mask wrapper.
     *
     * Protected (with the two sequence setters below) as the deterministic
     * cycle-stepping seam: the golden-trace suite
     * drives one call per fake-master cycle to pin the exact per-cycle
     * mode/controlword sequences, impossible against the free-running RT
     * thread.
     */
    void execute_state_transitions();

    /// @brief Report what each axis's fault-reset request did this cycle.
    /// Runs immediately after the servo write loop, because the publication
    /// point is the only place that knows, and a drop is terminal, so an
    /// operator who was told "requested" has no other way to learn the axis
    /// threw it away and must ask again.
    void report_fault_reset_publications();

    /**
     * @brief Set brake release sequence for selected joints (startup)
     *
     * The native CiA 402 enable sequence
     * (READY_TO_SWITCH_ON -> SWITCHED_ON -> OPERATION_ENABLED): the brake
     * follows drive power on a power-coupled drive (brake_logic.hpp's
     * release-step assumption), so the sequence is the release.
     * The mask expands to full brake groups.
     */
    void set_brake_release_sequence(uint8_t mask) noexcept;

  private:
    /**
     * @brief Drive-layer safety step (runs only when set_drive_limits() armed
     * it; before the brake step so a fresh INTERNAL protective stop force-engages in
     * the same cycle). Limits over the SI actuals, and a CSP axis whose
     * position word does not decode, → INTERNAL; reset_error
     * consumption → optimistic INTERNAL+EXTERNAL clear (core
     * evaluate_internal_protective_stop). Effects:
     * ControlModeFlags, one event per onset /
     * clear / rejected clear.
     */
    void run_drive_safety_cycle(const std::array<plem::core::JointActual, DOF>& actuals) noexcept;

    /**
     * @brief Drive-layer command step (set_drive_limits() armed):
     * each axis in its declared mode: CST torque (clamped), CSP
     * 0x607A, CSV 0x60FF. One snapshot, every axis checked before any is
     * applied: a CSP target inside the position limits and within one cycle's
     * travel of the previous applied target (the actual on a first
     * application), a CSV target within the velocity limit, both encodable
     * without saturation (an undecodable CSP feedback is the safety step's;
     * it latches like a limit violation). Applied only when
     * the gate is open, the command carries a host stamp and every axis
     * passed; otherwise every axis is neutral (0 Nm, the raw 0x607A mirror, 0
     * velocity). A violation latches INTERNAL protective stop and arms the in-service
     * trip, so an operator reset consumed before the next brake step cannot
     * skip the forced engage. A csp host command is advanced by its rate ×
     * its age (now_ns − stamp_ns): a sample the loop reads late, or twice,
     * still steps one cycle's travel (a host on its own clock beside this
     * loop).
     */
    void run_drive_command_cycle(const std::array<double, DOF>& command, const std::array<double, DOF>& rate,
                                 int64_t stamp_ns, bool host_stamped, bool gate_open,
                                 const std::array<plem::core::JointActual, DOF>& actuals, int64_t now_ns) noexcept;

    /// The stop (request_ramp_stop) for this cycle: decide_ramp_stop on this cycle's
    /// facts, then its effect. Before the command step builds any target.
    /// true = the stop owns this cycle's commands (it ran or runs on; the
    /// cycle it ends in still carries its last, resting target).
    [[nodiscard]] bool run_ramp_stop_cycle(bool gate_open, const std::array<plem::core::JointActual, DOF>& actuals,
                                           int64_t now_ns) noexcept;

    /// Whether the freshness watch is unarmed this cycle: on a bus without a
    /// cst axis, while motion is off, a stop runs or the newest command
    /// predates the release. The caller resets the watch and checks stamp 0.
    [[nodiscard]] bool command_watch_unarmed(int64_t update_ns, bool motion_enabled) const noexcept {
        return hold_on_release_ && (!motion_enabled || ramp_stop_active() || predates_release(update_ns));
    }

    /// A command stamped before the current release (open_command_epoch). One
    /// load per call: the epoch can move between two calls in one cycle.
    [[nodiscard]] bool predates_release(int64_t stamp) const noexcept {
        return stamp < command_epoch_ns_.load(std::memory_order_acquire);
    }

    /// @brief Group-atomic release settlement (native shared-brake coupling): one
    /// axis reaching OPERATION_ENABLED opens the shared brake: every joint
    /// of the group is unheld from that instant, so the whole group flips
    /// released (and re-verifies) together.
    void settle_group_released(uint8_t bit) noexcept;

    /// @brief Group-atomic engage settlement: the shared brake is confirmed
    /// closed only when every axis of the group sits in a proven-unpowered
    /// step (allowlist: NOT_READY / SWITCH_ON_DISABLED / READY).
    /// Flips the whole group when that holds (idempotent for the partner's
    /// own later completion) and nothing otherwise; a single-joint flip
    /// while the partner might hold the brake open would report "engaged"
    /// on a joint that is neither braked nor powered. An axis with
    /// no statusword (or an unrecognized one, stale held state)
    /// cannot prove closure and blocks the flip (fail-safe: the last
    /// confirmed state stands).
    void settle_group_engaged(uint8_t bit) noexcept;

    /// Expand joint bits to their full shared-brake groups (a module
    /// brake acts/fails/verifies as a unit).
    [[nodiscard]] uint8_t brake_group_closure(uint8_t bits) const noexcept;

    /// @brief This cycle's release-readiness mask (mode-acceptance gate,
    /// BrakeRequestInputs::release_ready): the joints admitted for release
    /// (release_admissible: strict acceptance, or the POST_ENABLE first-zero
    /// exception), closed over the brake groups; a group counts only when
    /// every member is admitted. Computed inside the brake step after the
    /// request masks are consumed, from the RT-owned PDO image of this
    /// cycle's read and the slave states sampled for the consumed request;
    /// this function keeps no state of its own (the exception reads
    /// mode_seen_ / in_service_trip_reported_).
    [[nodiscard]] uint8_t release_ready_mask() const noexcept;
    /// Per-axis release admission: the axis's slave is operational
    /// (slave_states_: in OP with this configuration), and either the
    /// accepted predicate (in_declared_mode) or the one exception a
    /// POST_ENABLE drive gets: its 0x6061 still 0 in a recognized resting
    /// state (SWITCH_ON_DISABLED / READY_TO_SWITCH_ON) while the axis has
    /// neither shown its mode yet (mode_seen_) nor an open in-service trip
    /// episode (in_service_trip_reported_). Shared by release_ready_mask() and
    /// emit_release_refused() so an admitted zero is never named as a refusal
    /// cause.
    [[nodiscard]] bool release_admissible(size_t axis) const noexcept;

    /// @brief Diagnostic for a refused release (EMIT_RELEASE_REFUSED): one
    /// event per axis of the request's brake-group closure that is not
    /// admitted: SLAVE_NOT_OPERATIONAL when its slave is not operational,
    /// otherwise MODES_OF_OPERATION_MISMATCH, checked on the same samples the
    /// decision used (no read happens between decide and apply). Reports
    /// only. The decision has already refused, and nothing here re-decides.
    void emit_release_refused(uint8_t requested) noexcept;

    /// Refresh statusword_mirror_ from the RT-owned PDO fields, called once
    /// per cycle at the top of execute_state_transitions() (covers the RT
    /// loop and the synchronous sequencing harness through the same seam).
    void refresh_statusword_mirror() noexcept;

    /// @brief Load the CiA 402 enable sequence (the native release: brake
    /// opens as the drive powers up).
    void load_release_sequence(uint8_t mask) noexcept;

    /// @brief Engage entry shared by the user/request path (START_ENGAGE,
    /// `preserve_active` false: an axis mid-engage restarts from the first
    /// step) and the safety-forced path
    /// (START_FORCED_ENGAGE, true: an axis already walking an engage keeps
    /// its head, remaining steps and stall history). Group closure, busy
    /// publication and BRAKE_ENGAGE are identical.
    void start_engage(uint8_t mask, bool preserve_active) noexcept;

    /// @brief Load the conservative CiA 402 disable sequence (the native
    /// engage: brake closes as drive power drops) on every axis of `mask`,
    /// except, with `preserve_active`, an axis whose engage is already
    /// walking (see start_engage).
    void load_engage_sequence(uint8_t mask, bool preserve_active) noexcept;

    /**
     * @brief Apply one ordered brake effect from the cycle-step decisions
     *
     * The per-cycle brake policy itself is pure (brake_logic.hpp: safety-
     * trigger leg + request leg) and sequenced by the package-private cycle step
     * (src/ethercat/brake_cycle_step.hpp); this member is its production
     * apply: sequence starts and safety event emissions only.
     */
    void apply_brake_effect(const BrakeEffect& effect) noexcept;

    // EtherCAT-specific bus (typed access for servo devices, domain_pd, etc.)
    std::shared_ptr<EtherCATBus> ecat_bus_;

    // Mode flags (shared with the host)
    std::shared_ptr<plem::core::ControlModeFlags> mode_flags_;

    // The joint masks here are uint8_t (one bit per joint): a 9th joint would
    // be silently unaddressable (mask saturates at 0xFF), so reject the
    // instantiation outright. Widen the mask types before allowing DOF > 8.
    static_assert(DOF <= core::kMaxBusAxes, "uint8_t joint masks address at most 8 joints");

    static constexpr uint8_t all_joints_mask() noexcept {
        if constexpr (DOF >= 8) {
            return 0xFF;
        } else {
            return static_cast<uint8_t>((uint16_t{1} << DOF) - uint16_t{1});
        }
    }

    // Per-servo state sequence management (RT-safe: no dynamic allocation)
    static constexpr size_t kMaxPendingSequenceLength = 4;  // Max observed: 3 states
    struct PerServoSequence {
        std::array<ServoState, kMaxPendingSequenceLength> states{};
        uint8_t head{0};             // Index of next state to execute
        uint8_t size{0};             // Number of remaining states
        bool release_target{false};  // true => sequence completion means brake released
    };
    std::array<PerServoSequence, DOF> servo_sequences_{};

    /// Mid-sequence fault auto-reset pacing (FaultResetPolicy::
    /// BOUNDED_AUTO_RESET models only; under NONE a faulted sequence
    /// stalls), converted from the model's
    /// FaultResetPacing in set_fault_reset_policy(): the interval in cycles
    /// (cycles_at_least; time is the source of truth, so a faster period keeps the
    /// declared pace instead of shrinking it) and the accepted-request cap
    /// per fault episode. The accepted-request cap is the only retry bound;
    /// there is no outer sequence timeout (see FaultResetPacing and
    /// kMaxSupportedFaultResetAttempts in brake_logic.hpp). Sequence
    /// replacement is event-driven: protective stop forced engage or an operator
    /// command. Giving up stops requests and does not prove settled FAULT.
    /// Both counters stay 0 under NONE, whose policy gate skips the reset
    /// branch.
    uint32_t fault_reset_interval_cycles_{0};
    uint32_t fault_reset_max_attempts_{0};

    std::array<uint32_t, DOF> fault_reset_cycles_{};
    std::array<uint32_t, DOF> fault_reset_attempts_{};

    /// Sequence silent-stall observability window: an active sequence that
    /// stops advancing is reported, even after the mode transition has
    /// already reported success. Time is the source of truth, converted once at
    /// construction (cycles_at_least). The window is deliberately far above
    /// every legitimate sequence pace (steps advanced within a few hundred ms
    /// on the drives measured).
    static constexpr int64_t kBrakeSequenceStallNs = 2'000'000'000;  // 2 s
    /// Sustained WKC mismatch before a protective stop. A time, not a cycle
    /// count, because stale-torque exposure scales with time; the constructor
    /// converts it with cycles_at_least() (5 ms = 5 cycles at 1 kHz).
    static constexpr int64_t kWkcLossThresholdNs = 5'000'000;
    uint32_t sequence_stall_threshold_cycles_{0};
    /// Per-joint cycles an active sequence has sat off its step, and whether
    /// this episode's event already fired (one event per episode; the
    /// counter freezes once reported, so no wraparound re-emission).
    std::array<uint32_t, DOF> sequence_stall_cycles_{};
    std::array<bool, DOF> sequence_stall_reported_{};

    /// In-service trip episodes (sequencing-thread only): bit i set = axis
    /// i's in-service trip has been reported and has triggered the forced
    /// engage for this episode. The trip is either the mode trip (enabled, not
    /// accepted in its declared mode) or the enable loss (its completed release
    /// left OPERATION_ENABLED while its group still reads released).
    /// Re-armed only when the axis is seen accepted, has no engage still
    /// walking and no enable loss standing; an accepted sample while its
    /// engage walks keeps the episode (no repeat diagnostics, no repeat
    /// trigger from that axis). Never touched by a reset. The protective stop level
    /// is not this record: it is re-asserted from the trip condition itself
    /// every cycle it holds.
    uint8_t in_service_trip_reported_{0};
    /// A new in-service trip onset (any axis) since the last brake step,
    /// handed to the next step as BrakeCycleInputs::mode_trip and cleared there.
    bool in_service_trip_pending_{false};
    /// First-match record (sequencing-thread only, controller lifetime, never
    /// cleared): bit i set = axis i has been seen in its declared mode at
    /// least once (any state, any cycle). Read by release_admissible(): a
    /// POST_ENABLE drive's 0 on 0x6061 counts as "not reported yet" only
    /// while this bit is clear; once the mode was shown, 0 is a mismatch.
    uint8_t mode_seen_{0};

    /// Release gate input (brake-step thread only): each axis's slave state,
    /// sampled once per physical slave in every brake step that consumes a
    /// release request, after the consume, so the request is checked on a
    /// sample taken for it. Read only by that step's decision and refusal
    /// report; nothing samples it otherwise (one non-blocking ioctl per slave
    /// per release). The WKC watchdog, not this sample, sees a slave drop out
    /// in service.
    std::array<EtherCATBus::SlaveState, DOF> slave_states_{};

    /// Per-joint statusword snapshot, refreshed once per cycle by the cycle
    /// thread (top of execute_state_transitions). The race-free diagnostic
    /// plane for non-RT readers (quiesce outcome): DeviceServo's
    /// get_statusword() returns the plain RT-owned PDO field, so reading it
    /// off-thread is a data race. Values lag at most one cycle.
    std::array<std::atomic<uint16_t>, core::kMaxBusAxes> statusword_mirror_{};

    /// process_data_complete(): stored once per cycle by the cycle thread.
    std::atomic<bool> process_data_complete_{false};

    /// Bounded quiesce engage attempts + per-attempt confirmation deadline
    /// (bounded retry; no outer timeout exists by design; the caller
    /// holds shutdown on exhaustion). Engineering defaults.
    static constexpr int kQuiesceEngageAttempts = 3;
    static constexpr int64_t kQuiesceAttemptTimeoutNs = 3'000'000'000;  // 3 s per attempt
    /// Per-attempt wait bound, member so the deterministic harness can
    /// shorten it (production never changes it; EtherCATControllerTestProbe).
    int64_t quiesce_attempt_timeout_ns_{kQuiesceAttemptTimeoutNs};

    /// Joint→shared-brake group table (whole-group closure input). Empty
    /// until set_brake_setup(), which runs before the RT loop starts; a joint
    /// in no group is its own group. Immutable once threads exist.
    std::array<uint8_t, DOF> brake_group_masks_{};
    size_t brake_group_count_{0};
    /// Model-declared mid-transition fault handling (set_fault_reset_policy).
    /// A policy the model chooses, independent of the group table: declaring
    /// shared brakes does not switch on auto-reset.
    FaultResetPolicy fault_reset_policy_{FaultResetPolicy::NONE};

    /// Committed-mode authority probe (see set_committed_mode_probe).
    ModeProbe mode_probe_{nullptr};
    const void* mode_probe_ctx_{nullptr};

    /// One committed-mode sample (RT: one indirect call + one atomic load).
    /// A wired probe projects the host's committed {safety, mode} word; the
    /// flags fallback approximates recovery as protective_stop && freedrive.
    [[nodiscard]] CommittedModeSample sample_committed_mode() const noexcept {
        if (mode_probe_ != nullptr) {
            return mode_probe_(mode_probe_ctx_);
        }
        return {mode_flags_->motion_enabled.load(std::memory_order_acquire),
                mode_flags_->is_protective_stop_active() &&
                    mode_flags_->freedrive_enabled.load(std::memory_order_acquire)};
    }

    // Brake decision state (brake_logic.hpp): protective stop edge tracking and the
    // forced-engage pending flag. Owned by the RT loop; the mode-path API only
    // feeds the request masks below.
    BrakeLogicState brake_logic_state_{};

    // Atomic brake request masks (the request API), consumed by the RT
    // loop's brake step.
    std::atomic<uint8_t> brake_release_request_mask_{0x00};
    std::atomic<uint8_t> brake_engage_request_mask_{0x00};

    /// Per-cycle mirror of the active state sequences (RT writes at the
    /// end of execute_state_transitions(); brake_operation_pending() ORs it with
    /// the live request masks).
    std::atomic<uint8_t> brake_operation_busy_mask_{0};

    /// Each post is an OR on its own mask + an AND-clear on the
    /// opposite mask: two words, so two unserialized posts can interleave
    /// into both bits cleared (the last request silently lost; the cycle
    /// serializer cannot recover a mask that never survived). Posts are
    /// non-RT, so a mutex serializes the pair; the RT consume
    /// (load, busy publish, fetch_and per word) stays lock-free and never
    /// takes it.
    std::mutex brake_request_post_mutex_;

    /// Seqlock word for the two-mask post pair (odd = pair in
    /// progress; writers already serialize on the mutex above; the version
    /// exists for the lock-free reader brake_operation_pending()).
    std::atomic<uint32_t> brake_request_version_{0};

    // Test-only seams (plem/fieldbus/test/, not a release header).
    template <size_t>
    friend class ::plem::fieldbus::test::EtherCATControllerTestProbe;

    /// Test seam: invoked between the OR and the opposite-AND of a release
    /// post, inside the critical section. Never set in production.
    std::function<void()> brake_release_post_pair_hook_for_test_;

    /// One brake-cycle step over the posted request masks (the pure logic +
    /// production apply run_loop uses). `motion_possible` is this cycle's bus
    /// status value.
    void step_brake_requests(core::TriState motion_possible);

    /// Release-admission flag: one-way close (quiesce). With a
    /// one-way close, this flag sampled at the RT consume point is the
    /// generation check: every queued/posted release predates the close and
    /// is discarded; nothing is admitted after it. (Per-request generation
    /// stamps would add distinguishable behavior only if admission could
    /// REOPEN — no such consumer exists.)
    std::atomic<bool> release_admission_closed_{false};

    // Current brake state: each bit is 1 = engaged, 0 = released.
    // Conservative startup assumption is all brakes engaged.
    std::atomic<uint8_t> brake_state_mask_{all_joints_mask()};

    // Drive-layer limits (set_drive_limits): declared once before start,
    // read-only in the loop. Undeclared = the step and the gate do not exist.
    std::array<core::MotionLimit, DOF> drive_limits_{};
    bool drive_limits_declared_{false};
    // CSP step reference: the angle this axis's applied target decodes to,
    // held only while the axis applied on every cycle since; any cycle that
    // applied nothing clears it, and the next application is checked against
    // the actual. An endless axis instead keeps the angle of the hold word it
    // transmitted (its hold can lag the feedback across incomplete frames).
    // Loop-thread only; cleared at every run start.
    std::array<std::optional<double>, DOF> csp_applied_rad_{};
    // The CSP target this axis applied last cycle, word and angle as sent,
    // replayed unchanged in a command-violating cycle (no reconversion, which
    // can round off an accepted endpoint). Cleared with csp_applied_rad_ on
    // any cycle that applied nothing. Loop-thread only.
    std::array<std::optional<CheckedTarget>, DOF> csp_sent_target_{};
    // Requested stop (set_drive_stop / request_ramp_stop). Decelerations are
    // declared once before start; the rest is loop-thread state but for the
    // two request/progress atomics.
    std::array<double, DOF> stop_deceleration_{};
    std::atomic<bool> ramp_stop_requested_{false};
    std::atomic<bool> ramp_stop_running_{false};
    RampStopState ramp_stop_state_{};
    int64_t ramp_stop_started_ns_{0};
    // A stopping csp or csv command's speed, rad/s (loop-thread).
    std::array<double, DOF> stop_velocity_{};
    // The speed each axis was last commanded at: a csp axis's applied target
    // step over the cycle, a csv axis's applied target; 0 after a cycle that
    // applied nothing (loop-thread). A stop starts from it.
    std::array<double, DOF> applied_velocity_{};
    // The release mark (open_command_epoch) and whether this run honours it
    // (no cst axis on the bus, fixed at run start).
    std::atomic<int64_t> command_epoch_ns_{0};
    bool hold_on_release_{false};
    int64_t command_stale_timeout_ns_{plem::fieldbus::kDefaultCommandStaleTimeoutNs};

    // Watchdog state (RT-safe: trivially copyable, no allocation)
    plem::fieldbus::WatchdogState watchdog_state_{};

    // Command freshness state (RT-safe, loop-thread only)
    plem::fieldbus::CommandFreshnessState command_freshness_state_{};

    // Init-time cycle-count derivations from the configured bus period: the
    // time constants are the source of truth; see cycles_at_least().
    // Written once in the constructor, read-only in the RT loop (declaration
    // order == constructor init order).
    int64_t cycle_period_ns_{};
    uint32_t wkc_threshold_cycles_{};
};

}  // namespace plem::fieldbus::ethercat
