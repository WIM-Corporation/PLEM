// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file drive_host_adapter.hpp
/// @brief The drive-layer host with its axis count erased.
///
/// `plem::drive::DriveHost<N>` is a template because N sizes its RT storage.
/// The node is not, because the drive model file
/// chooses N at run time. `IDriveHost` is the one seam where N disappears and
/// `make_drive_host<N>` the one adapter that erases it; everything above this
/// header is written once, for every bus width.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <plem/core/monitoring/monitoring_context.hpp>
#include <plem/core/result.hpp>
#include <plem/drive/drive_host.hpp>
#include <plem/drive/models/drive_model_spec.hpp>
#include <plem/fieldbus/ethercat/bus.hpp>
#include <plem/fieldbus/ethercat/controller.hpp>

namespace plem::drive::ros {

/// One race-free snapshot of a drive bus, N erased. The axis count is
/// `IDriveHost::axis_names().size()`; the snapshot does not repeat it. The leading
/// entries of each array are filled, the rest stay zero. The arrays are
/// plem::drive::kMaxAxes wide (the widest bus the spec schema admits), so a
/// snapshot stays a value the node can hold and copy.
struct DriveState {
    std::array<double, plem::drive::kMaxAxes> position{};      ///< rad
    std::array<double, plem::drive::kMaxAxes> velocity{};      ///< rad/s
    std::array<double, plem::drive::kMaxAxes> torque{};        ///< Nm
    std::array<uint16_t, plem::drive::kMaxAxes> statusword{};  ///< CiA 402 0x6041 Statusword
    uint8_t protective_stop = 0;                               ///< ProtectiveStopSource bits
    bool operation_enabled = false;       ///< every drive in Operation enabled, in its declared mode
    uint8_t brake_engaged = 0;            ///< bit i = axis i engaged
    uint8_t brake_operation_pending = 0;  ///< bit i = axis i unsettled (engaged is then stale)
    bool process_data_complete = false;   ///< the working counter is complete: every slave in OP
};

/// @thread_safety Inherited from DriveHost: start/stop/enable/ramp_stop/reset are
/// serialized on one non-RT thread and write() is the only seqlock writer. The
/// node satisfies both by living on a single-threaded executor; a host that
/// spins it otherwise owes that serialisation itself.
class IDriveHost {
  public:
    virtual ~IDriveHost() = default;

    /// Joint labels in axes[] order. The drive layer has no URDF, so these are
    /// the only names the node can publish; their count is the bus's axis
    /// count.
    [[nodiscard]] virtual std::span<const std::string> axis_names() const noexcept = 0;

    [[nodiscard]] virtual plem::core::Result<void> start() = 0;
    virtual plem::fieldbus::ethercat::BrakeQuiesceOutcome stop() = 0;
    [[nodiscard]] virtual plem::core::Result<void> enable() = 0;
    /// Requested stop (DriveHost::ramp_stop), asynchronous; the brakes read
    /// engaged when it has ended.
    virtual void ramp_stop() = 0;
    /// One value per axis, each in the unit of that axis's declared mode
    /// (modes_of_operation(): Nm, rad or rad/s). false = rejected (wrong length or a
    /// non-finite value); nothing written, nothing stamped; the command then
    /// goes stale and trips the protective stop.
    /// `rate` and `sampled_ns` (DriveHost::write): each value's planned change
    /// per second (or empty), and the CLOCK_MONOTONIC instant the values describe.
    [[nodiscard]] virtual bool write(std::span<const double> command, std::span<const double> rate,
                                     int64_t sampled_ns) noexcept = 0;
    /// false = the snapshot was torn; `out` is not filled.
    [[nodiscard]] virtual bool read(DriveState& out) const noexcept = 0;
    virtual void reset() = 0;
    /// Non-null, with its RtSample and Event queues allocated, for the host's
    /// whole life; the node drains both without checking.
    [[nodiscard]] virtual plem::core::monitoring::MonitoringContext* monitoring_context() noexcept = 0;
    /// Per slave, in the model file's slaves[] order: "sha256:…" of the
    /// mapping this boot scanned, "" for a declared slave. Static.
    [[nodiscard]] virtual std::span<const std::string> pdo_mapping_digests() const noexcept = 0;
    /// Per axis, axes[] order: the declared 0x6060 Modes of operation
    /// (INTEGER8: 10 CST, 8 CSP, 9 CSV), the unit of that axis's write()
    /// value. Static.
    [[nodiscard]] virtual std::span<const int8_t> modes_of_operation() const noexcept = 0;
};

namespace detail {

/// The erasure itself. Nothing here decides anything: every call but
/// axis_names() and modes_of_operation() (copied from the spec) forwards.
template <size_t N>
class DriveHostAdapter final : public IDriveHost {
  public:
    /// Copies spec.axis_names, the only link between the user's spec and
    /// the joint names on the wire. NUL-termination inside each fixed array is
    /// a validated spec rule, so this is a plain copy.
    DriveHostAdapter(std::unique_ptr<plem::drive::DriveHost<N>> host,
                     const plem::drive::models::DriveModelSpec<N>& spec)
        : host_(std::move(host)) {
        names_.reserve(N);
        for (const auto& raw : spec.axis_names) {
            names_.emplace_back(raw.data());
        }
        modes_.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            modes_.push_back(static_cast<int8_t>(spec.axes[i].modes_of_operation));
        }
    }

    [[nodiscard]] std::span<const std::string> axis_names() const noexcept override {
        return names_;
    }
    [[nodiscard]] plem::core::Result<void> start() override {
        return host_->start();
    }
    plem::fieldbus::ethercat::BrakeQuiesceOutcome stop() override {
        return host_->stop();
    }
    [[nodiscard]] plem::core::Result<void> enable() override {
        return host_->enable();
    }
    void ramp_stop() override {
        host_->ramp_stop();
    }
    [[nodiscard]] bool write(std::span<const double> command, std::span<const double> rate,
                             int64_t sampled_ns) noexcept override {
        return host_->write(command, rate, sampled_ns);
    }
    [[nodiscard]] bool read(DriveState& out) const noexcept override {
        typename plem::drive::DriveHost<N>::State state{};
        if (!host_->read(state)) {
            return false;
        }
        out = DriveState{};
        for (size_t i = 0; i < N; ++i) {
            out.position[i] = state.actual.states[i].position_actual_value;
            out.velocity[i] = state.actual.states[i].velocity_actual_value;
            out.torque[i] = state.actual.states[i].torque_actual_value;
            out.statusword[i] = state.statusword[i];
        }
        out.protective_stop = state.protective_stop;
        out.operation_enabled = state.operation_enabled;
        out.brake_engaged = state.brake_engaged;
        out.brake_operation_pending = state.brake_operation_pending;
        out.process_data_complete = state.process_data_complete;
        return true;
    }
    void reset() override {
        host_->reset();
    }
    [[nodiscard]] plem::core::monitoring::MonitoringContext* monitoring_context() noexcept override {
        return host_->monitoring_context();
    }
    [[nodiscard]] std::span<const std::string> pdo_mapping_digests() const noexcept override {
        return host_->pdo_mapping_digests();
    }
    [[nodiscard]] std::span<const int8_t> modes_of_operation() const noexcept override {
        return modes_;
    }

  private:
    std::unique_ptr<plem::drive::DriveHost<N>> host_;
    std::vector<std::string> names_;
    std::vector<int8_t> modes_;
};

}  // namespace detail

/// Build a `DriveHost<N>` from its spec and hand it back with N erased. The
/// spec's own value gate runs inside create(); a refusal comes back as the
/// error message, untouched.
template <size_t N>
[[nodiscard]] plem::core::Result<std::unique_ptr<IDriveHost>> make_drive_host(
    const plem::drive::models::DriveModelSpec<N>& spec, std::shared_ptr<plem::fieldbus::ethercat::EtherCATBus> bus,
    const plem::drive::DriveHostParams& params) {
    using Out = plem::core::Result<std::unique_ptr<IDriveHost>>;
    auto host = plem::drive::DriveHost<N>::create(spec, std::move(bus), params);
    if (!host) {
        return Out::error(host.message());
    }
    return Out::ok(std::make_unique<detail::DriveHostAdapter<N>>(std::move(host).value(), spec));
}

}  // namespace plem::drive::ros
