// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <plem/core/result.hpp>
#include <plem/core/units.hpp>

#include "plem/fieldbus/cia402_profile.hpp"
#include "plem/fieldbus/cia402_types.hpp"
#include "plem/fieldbus/ethercat/pdo_layout.hpp"

namespace plem::fieldbus::ethercat {

using plem::core::units::Pulse;

/**
 * @brief A drive family's raw servo constants: the input of
 * make_axis_factors(), which composes an axis's `factors` from them. The
 * conversion functions read channels, not a ServoSpec.
 */
struct ServoSpec {
    double gear_ratio;       ///< Motor turns per output turn: the reducer's torque amplification (> 1 behind a reducer)
    double torque_constant;  ///< Motor torque constant in Nm/A
    uint32_t encoder_resolution;  ///< Encoder pulses per motor revolution (before gear)
    double counts_per_amp;  ///< Drive current counts per amp of the torque PDOs (counts/A, the SCALED_CURRENT unit's
                            ///< scale)

    /// @brief Drive velocity-count unit scale: position-pulse equivalents per
    /// velocity count (decode multiplies by it, encode divides: the exact
    /// direction the conversions and their round-trip tests pin). Drives are
    /// free to report/accept velocity in their own unit; omitting this
    /// under-reports speed by the factor, which an inner velocity loop cannot
    /// detect (it simply commands full torque forever).
    /// 1.0 (default) = velocity counts equal position pulses.
    double velocity_factor{1.0};
};

// Channel map
//
// Every axis declares, per channel, which unit its counts are in and the
// parameters that unit's formula needs. The unit is the contract: it names
// the physical dimension, the conversion, and the evidence a declaration
// owes (the model's provenance notes or approval record). Torque inside
// the core is Nm only; the conversion
// to the drive's number happens once, at the PDO boundary, and the unit
// declares what that number means for this drive. Only declarable units
// exist as enumerators. The reserved word `raw` has none, so a model naming
// it does not compile; any other number is refused by
// validate_axis_factors. Adding a unit requires its formula, validation,
// evidence and a bench, together.
//
// Offsets are not part of the map: the zero/calibration offset stays with
// ServoInfo::joint_offset (applied once at the PDO boundary, Pulse::to_hardware),
// and a host's offset correction adds a converted delta to it; a map-level offset would
// be applied twice. Sign (flip) stays at the PDO boundary as well.

/// @brief Torque-channel unit: what one count of 0x6071/0x6077 means.
enum class TorqueUnit : uint8_t {
    UNDECLARED = 0,  ///< absence sentinel, never a valid declaration
    /// Output-shaft Nm ↔ drive current counts (signed16): count = τ·scale/Kt/gear,
    /// τ = count·Kt·gear/scale. scale = counts per amp (vendor sheet), Kt = motor
    /// Nm/A, gear = motor turns per output turn. A vendor-specific unit of the
    /// standard objects (outside CiA 402), as the drive vendor's documentation
    /// and motor sheet give it.
    SCALED_CURRENT,
    /// Output-shaft Nm ↔ per mille of the motor rated torque (signed16), the
    /// CiA 402 standard unit: one count = 0x6076 Motor rated torque / 1000. It
    /// reuses the SCALED_CURRENT fields and formula: counts_per_unit = 1000
    /// (TorqueChannel::kPerMilleScale, required by defect()) and torque_constant
    /// = the motor rated torque in Nm, so count = τ·1000/T_rated/gear.
    RATED_PER_MILLE,
};

/// @brief Position/velocity-channel unit: what one count of 0x607A/0x6064 and
/// 0x60FF/0x606C means.
enum class MotionUnit : uint8_t {
    UNDECLARED = 0,  ///< absence sentinel, never a valid declaration
    /// Output-shaft rad ↔ motor encoder counts through the reducer:
    /// count = rad·E·gear/2π. Velocity counts are E·gear/(2π·velocity_factor)
    /// per rad/s; velocity_factor = position-pulse equivalents per
    /// velocity count (1.0 = the same unit as position; one tested drive reports 750.3).
    ROTARY_MOTOR_ENCODER,
};

namespace detail {
inline bool positive_finite(double v) noexcept {
    return std::isfinite(v) && v > 0.0;
}
}  // namespace detail

/// @brief What is wrong with a channel declaration, and with which value: the
/// channel validity rule stated once, as a value. The two projections read it
/// instead of re-implementing it: `is_valid()` is `kind == NONE` (the
/// conversions' debug precondition, allocation-free) and
/// validate_axis_factors() turns the same verdict into the diagnostic that
/// names the channel, its unit and the offending parameter. Order of the
/// parameter checks is the order the diagnostic reports a multi-defect channel
/// in. `value` is the offending parameter's value and travels with the kind;
/// the kinds that name no parameter leave it 0 and no reader looks at it.
struct ChannelDefect {
    enum class Kind : uint8_t {
        NONE,
        UNIT_UNDECLARED,
        UNIT_OUT_OF_RANGE,
        COUNTS_PER_UNIT,
        TORQUE_CONSTANT,
        GEAR_RATIO,
        ENCODER_RESOLUTION,
        VELOCITY_FACTOR,
        PER_MILLE_SCALE,  ///< RATED_PER_MILLE with counts_per_unit != 1000
    };

    Kind kind = Kind::NONE;
    double value = 0.0;
};

/// @brief Torque channel: unit + the parameters its formula needs (nothing else).
struct TorqueChannel {
    /// The one counts_per_unit RATED_PER_MILLE admits: 1000 counts per rated torque.
    static constexpr double kPerMilleScale = 1000.0;

    TorqueUnit unit = TorqueUnit::UNDECLARED;
    /// Counts per reference unit of this channel's TorqueUnit:
    ///   SCALED_CURRENT:  counts per 1 A (the drive's current scale);
    ///   RATED_PER_MILLE: kPerMilleScale counts per rated torque (1000 ‰).
    double counts_per_unit = 0.0;
    /// Motor Nm per reference unit. SCALED_CURRENT: Kt (Nm/A); RATED_PER_MILLE: motor rated torque (Nm).
    double torque_constant = 0.0;
    double gear_ratio = 0.0;  ///< motor turns per output turn
    [[nodiscard]] ChannelDefect defect() const noexcept {
        using Kind = ChannelDefect::Kind;
        if (unit == TorqueUnit::UNDECLARED) {
            return {Kind::UNIT_UNDECLARED};
        }
        if (unit != TorqueUnit::SCALED_CURRENT && unit != TorqueUnit::RATED_PER_MILLE) {
            return {Kind::UNIT_OUT_OF_RANGE};
        }
        if (unit == TorqueUnit::RATED_PER_MILLE && counts_per_unit != kPerMilleScale) {
            return {Kind::PER_MILLE_SCALE, counts_per_unit};
        }
        if (!detail::positive_finite(counts_per_unit)) {
            return {Kind::COUNTS_PER_UNIT, counts_per_unit};
        }
        if (!detail::positive_finite(torque_constant)) {
            return {Kind::TORQUE_CONSTANT, torque_constant};
        }
        if (!detail::positive_finite(gear_ratio)) {
            return {Kind::GEAR_RATIO, gear_ratio};
        }
        return {};
    }
    [[nodiscard]] bool is_valid() const noexcept {
        return defect().kind == ChannelDefect::Kind::NONE;
    }
    constexpr bool operator==(const TorqueChannel&) const = default;
};

/// @brief Position channel: unit + parameters.
struct PositionChannel {
    MotionUnit unit = MotionUnit::UNDECLARED;
    uint32_t encoder_resolution = 0;  ///< counts per motor revolution
    double gear_ratio = 0.0;          ///< motor turns per output turn
    [[nodiscard]] ChannelDefect defect() const noexcept {
        using Kind = ChannelDefect::Kind;
        if (unit == MotionUnit::UNDECLARED) {
            return {Kind::UNIT_UNDECLARED};
        }
        if (unit != MotionUnit::ROTARY_MOTOR_ENCODER) {
            return {Kind::UNIT_OUT_OF_RANGE};
        }
        if (encoder_resolution == 0) {
            return {Kind::ENCODER_RESOLUTION};
        }
        if (!detail::positive_finite(gear_ratio)) {
            return {Kind::GEAR_RATIO, gear_ratio};
        }
        return {};
    }
    [[nodiscard]] bool is_valid() const noexcept {
        return defect().kind == ChannelDefect::Kind::NONE;
    }
    constexpr bool operator==(const PositionChannel&) const = default;
};

/// @brief Velocity channel: unit + parameters.
struct VelocityChannel {
    MotionUnit unit = MotionUnit::UNDECLARED;
    uint32_t encoder_resolution = 0;
    double gear_ratio = 0.0;
    double velocity_factor = 0.0;  ///< position-pulse equivalents per velocity count
    [[nodiscard]] ChannelDefect defect() const noexcept {
        using Kind = ChannelDefect::Kind;
        if (unit == MotionUnit::UNDECLARED) {
            return {Kind::UNIT_UNDECLARED};
        }
        if (unit != MotionUnit::ROTARY_MOTOR_ENCODER) {
            return {Kind::UNIT_OUT_OF_RANGE};
        }
        if (encoder_resolution == 0) {
            return {Kind::ENCODER_RESOLUTION};
        }
        if (!detail::positive_finite(gear_ratio)) {
            return {Kind::GEAR_RATIO, gear_ratio};
        }
        if (!detail::positive_finite(velocity_factor)) {
            return {Kind::VELOCITY_FACTOR, velocity_factor};
        }
        return {};
    }
    [[nodiscard]] bool is_valid() const noexcept {
        return defect().kind == ChannelDefect::Kind::NONE;
    }
    constexpr bool operator==(const VelocityChannel&) const = default;
};

/// @brief One axis's channel map: the value the assembly carries from
/// AxisSpec through ServoInfo into DeviceServo. Each channel is validated
/// on its own; the helper below composes all three from one ServoSpec, but
/// nothing requires them to share parameters.
struct AxisFactors {
    TorqueChannel torque{};
    PositionChannel position{};
    VelocityChannel velocity{};
    constexpr bool operator==(const AxisFactors&) const = default;
};

/// @brief Compose a channel map from a drive family's raw constants
/// (ServoSpec): SCALED_CURRENT torque, ROTARY_MOTOR_ENCODER
/// position and velocity. Pure copy; the helper does no arithmetic; the
/// formulas and their evaluation order are owned by servo_conversions.hpp.
[[nodiscard]] constexpr AxisFactors make_axis_factors(const ServoSpec& spec) noexcept {
    return AxisFactors{
        .torque = {TorqueUnit::SCALED_CURRENT, spec.counts_per_amp, spec.torque_constant, spec.gear_ratio},
        .position = {MotionUnit::ROTARY_MOTOR_ENCODER, spec.encoder_resolution, spec.gear_ratio},
        .velocity = {MotionUnit::ROTARY_MOTOR_ENCODER, spec.encoder_resolution, spec.gear_ratio, spec.velocity_factor},
    };
}

namespace detail {

/// The torque unit a diagnostic names: the channel's own unit, or (for an
/// undeclared or out-of-range value, which names no unit) the implemented set.
[[nodiscard]] inline constexpr const char* torque_unit_name(TorqueUnit unit) noexcept {
    switch (unit) {
        case TorqueUnit::SCALED_CURRENT:
            return "SCALED_CURRENT";
        case TorqueUnit::RATED_PER_MILLE:
            return "RATED_PER_MILLE";
        default:
            return "SCALED_CURRENT or RATED_PER_MILLE";
    }
}

/// The parameter a parameter-defect names (diagnostics only).
[[nodiscard]] inline constexpr const char* channel_defect_param(ChannelDefect::Kind kind) noexcept {
    switch (kind) {
        case ChannelDefect::Kind::COUNTS_PER_UNIT:
            return "counts_per_unit";
        case ChannelDefect::Kind::TORQUE_CONSTANT:
            return "torque_constant";
        case ChannelDefect::Kind::GEAR_RATIO:
            return "gear_ratio";
        case ChannelDefect::Kind::VELOCITY_FACTOR:
            return "velocity_factor";
        default:
            return "";
    }
}

/// @brief The diagnostic projection of <channel>::defect(): the one place a
/// channel defect is worded. `unit` is the unit that channel implements and
/// `unit_number` is read only for UNIT_OUT_OF_RANGE. Allocates; init-path
/// use only.
[[nodiscard]] inline std::string channel_defect_message(const char* channel, const char* unit, ChannelDefect defect,
                                                        int unit_number) {
    std::ostringstream ss;
    switch (defect.kind) {
        case ChannelDefect::Kind::NONE:
            return {};
        case ChannelDefect::Kind::UNIT_UNDECLARED:
            ss << channel << " channel unit is UNDECLARED — declare factors." << channel
               << ".unit (make_axis_factors() composes it from a ServoSpec); no default unit exists";
            return ss.str();
        case ChannelDefect::Kind::UNIT_OUT_OF_RANGE:
            ss << channel << " channel unit holds an out-of-range value (" << unit_number << ") — only " << unit
               << " is implemented";
            return ss.str();
        case ChannelDefect::Kind::ENCODER_RESOLUTION:
            ss << channel << " channel (" << unit << ") encoder_resolution=0 must be > 0";
            return ss.str();
        case ChannelDefect::Kind::PER_MILLE_SCALE:
            ss << channel << " channel (" << unit << ") counts_per_unit=" << defect.value
               << " must be 1000 — one count is 1/1000 of the motor rated torque";
            return ss.str();
        case ChannelDefect::Kind::COUNTS_PER_UNIT:
        case ChannelDefect::Kind::TORQUE_CONSTANT:
        case ChannelDefect::Kind::GEAR_RATIO:
        case ChannelDefect::Kind::VELOCITY_FACTOR:
            break;
    }
    ss << channel << " channel (" << unit << ") " << channel_defect_param(defect.kind) << "=" << defect.value
       << " must be finite and > 0";
    return ss.str();
}

template <typename Channel>
[[nodiscard]] core::Result<void> channel_error(const char* name, const char* unit, const Channel& c) {
    std::string defect = channel_defect_message(name, unit, c.defect(), static_cast<int>(c.unit));
    if (defect.empty()) {
        return core::Result<void>::ok();
    }
    return core::Result<void>::error(std::move(defect));
}

}  // namespace detail

/// @brief Validate a channel map: ok, or an error naming the first defect
/// (channel, unit, parameter, value). It is the diagnostic projection of the
/// three channels' defect(): the rule lives there and is not restated here.
/// Structural validation only: it accepts exactly the implemented units (the
/// sentinel and any other number are refused) and requires each unit's
/// parameters to be finite and > 0 (RATED_PER_MILLE: counts_per_unit 1000). It does not, and cannot, check the
/// declaration's evidence; that is the model header's provenance and the
/// approval record.
[[nodiscard]] inline core::Result<void> validate_axis_factors(const AxisFactors& c) {
    if (core::Result<void> error = detail::channel_error("torque", detail::torque_unit_name(c.torque.unit), c.torque);
        !error) {
        return error;
    }
    if (core::Result<void> error = detail::channel_error("position", "ROTARY_MOTOR_ENCODER", c.position); !error) {
        return error;
    }
    return detail::channel_error("velocity", "ROTARY_MOTOR_ENCODER", c.velocity);
}

/// @brief The bool form of the same rule: each channel's defect() == NONE,
/// for ServoInfo::is_valid(). No diagnostic string is built on this path.
[[nodiscard]] inline bool axis_factors_valid(const AxisFactors& c) noexcept {
    return c.torque.is_valid() && c.position.is_valid() && c.velocity.is_valid();
}

/// @brief Longest fault-reset pulse a model may declare [ms]. 1 s is far above
/// the hold a vendor manual asks for (Mitsubishi MR-J5: 10/20 ms) and bounds
/// both the frame count and how long a mistyped value holds a drive in Fault
/// reset; no vendor figure backs the number itself.
inline constexpr uint32_t kMaxFaultResetPulseMs = 1000;

/// @brief CiA 402 channels one slave can host: the CANopen logical-device
/// scheme splits the profile area 0x6000–0x9FFF into 8 blocks of 0x800.
inline constexpr uint16_t kMaxCia402Channels = 8;

/// @brief Does the channel base sit on the CiA 402 multi-axis grid (channel N,
/// counted from 1 up to kMaxCia402Channels, at (N-1)*0x800)? One rule, read by
/// ServoInfo::is_valid at the bus boundary and by the model-spec gate before
/// any bus contact; an arbitrary offset would register PDO entries at indices
/// that are not the intended axis, which the master reports only as an opaque
/// ENOENT.
[[nodiscard]] inline constexpr bool od_offset_on_channel_grid(uint16_t od_offset) noexcept {
    return od_offset % 0x800 == 0 && od_offset / 0x800 < kMaxCia402Channels;
}

[[nodiscard]] inline std::string dump(const AxisFactors& c) {
    std::ostringstream ss;
    ss << "AxisFactors{torque=" << static_cast<int>(c.torque.unit) << "{counts_per_unit=" << c.torque.counts_per_unit
       << ", torque_constant=" << c.torque.torque_constant << ", gear_ratio=" << c.torque.gear_ratio
       << "}, position=" << static_cast<int>(c.position.unit) << "{encoder_resolution=" << c.position.encoder_resolution
       << ", gear_ratio=" << c.position.gear_ratio << "}, velocity=" << static_cast<int>(c.velocity.unit)
       << "{encoder_resolution=" << c.velocity.encoder_resolution << ", gear_ratio=" << c.velocity.gear_ratio
       << ", velocity_factor=" << c.velocity.velocity_factor << "}}";
    return ss.str();
}

struct ServoInfo {
    uint32_t vendor_id = 0;
    uint32_t product_code = 0;
    /// The axis's channel map, composed by the assembly from the
    /// model's ServoSpec; validated per channel at the config boundary.
    AxisFactors factors;
    bool flip = false;
    Pulse joint_offset;

    // The drive torque clamp at the output, Nm. Unlike the fields with no
    // default, an undeclared clamp is ±200 Nm; set both for the drive at hand
    // (the drive layer always does, from the file's limits).
    double max_torque_nm = 200.0;
    double min_torque_nm = -200.0;

    // Distributed-clocks drive properties (ESI/SDO-derived model constants).
    // 0 = not registered: enabling DC then fails fast at bus initialize();
    // an activation word or latch cycle is never guessed for an unverified
    // drive model.
    uint16_t dc_assign_activate = 0;  ///< ESI Dc/OpMode AssignActivate word (e.g. 0x0300 = SYNC0)
    int32_t dc_sync0_shift_ns = 0;    ///< master SYNC0 shift [ns]; SYNC0 must fall after the frame reaches the drive
    uint32_t dc_sync0_cycle_ns = 0;   ///< SYNC0 latch cycle [ns]: the drive's sync cycle, not the bus period
    /// The drive takes its cycle time from the master: with DC on, the bus
    /// registers dc_sync0_cycle_ns as a U32 startup SDO on 0x1C32:02, once
    /// per slave (a drive that derives 0x1C32:02 from its ESC keeps false).
    bool dc_write_cycle_time = false;

    /// @brief Model-declared CiA 402 requested-transition profile (mandatory;
    /// see cia402_profile.hpp for the scope + no-fallback contract). Injected
    /// by value into DeviceServo → ServoStateMachine; an empty (undeclared)
    /// profile is rejected fail-fast at EtherCATBus::initialize() before any
    /// hardware contact.
    Cia402TransitionProfile cia402_profile{};

    /// @brief Declared CiA 402 mode of operation: the 0x6060 command byte the
    /// device sends every cycle, and the value the controller's admission
    /// gates compare the 0x6061 display against (release readiness,
    /// all_drives_enabled, the in-service mismatch trip). Default CST.
    /// is_valid() accepts CST, CSP and CSV
    /// (operation_mode_supported) and rejects anything else, fail-fast at
    /// EtherCATBus::initialize() before any hardware contact.
    OperationMode modes_of_operation = OperationMode::CYCLIC_SYNC_TORQUE;

    /// @brief When the drive first reports the declared mode on 0x6061 (see
    /// ModeDisplayTiming). Mandatory; UNDECLARED fails is_valid(). Read by the
    /// controller's release admission only; all_drives_enabled and the
    /// in-service mismatch trip check the display strictly regardless.
    ModeDisplayTiming mode_display_timing = ModeDisplayTiming::UNDECLARED;

    // Multi-axis drive addressing. A single EtherCAT slave may host more than
    // one CiA 402 axis (e.g. two joints on one slave). Both defaults mean one
    // axis per slave.

    /// @brief CiA 402 object-dictionary channel offset added to every standard
    /// CiA 402 PDO index this axis registers.
    ///
    /// The multi-axis CiA 402 convention places channel N (counted from 1) at
    /// base + (N-1)*0x800: channel 1 controlword 0x6040, channel 2 controlword
    /// 0x6840. 0 = channel 1, the first (or only) axis on the slave.
    uint16_t od_offset = 0;

    /// @brief Bus position of the slave hosting this axis (0..65535); -1 = derive
    /// from the servo's index in the servo list (one axis per slave).
    ///
    /// Two axes sharing a slave resolve to the same effective ring position
    /// (declared, or derived from the index when negative) and differ only by
    /// od_offset. Explicit rather than computed, because the joint→(slave,
    /// channel) map is a per-model wiring fact, not something the bus can infer.
    int32_t slave_position = -1;

    /// @brief The slave's PDO policy (pdo_layout.hpp): a declared
    /// sync-manager / PDO mapping the bus writes at PREOP, or null = scanned:
    /// the bus reads the slave's current mapping from the master at
    /// initialize() Step 3 (after the master is requested, before any device)
    /// and uses that instead. Either way the bus registers only the standard
    /// entries the layout maps.
    ///
    /// The layout (declared at Step 0, scanned at Step 3) is validated with
    /// the same validator against every axis sharing the slave
    /// (pdo_layout_validation.hpp): the roles the mode requires,
    /// subindex/width/direction, byte alignment, duplicates and address
    /// collisions all fail fast before activation.
    ///
    /// Applied once per slave in PREOP; a scanned slave is never written.
    /// Every axis of a slave must point to the same layout object: all
    /// scanned (null) or all declaring one object. Disagreement is refused at
    /// initialize() (see SlavePdoMapping). The declaration must outlive the
    /// bus; model declarations use static constexpr storage.
    const SlavePdoMapping* pdo_mapping = nullptr;

    /// @brief ErrorCode optional-PDO role declaration; index 0 = undeclared.
    ///
    /// The only optional-PDO role. Objects outside the CiA 402 core set are
    /// optional: a compliant drive need not map them, and registering an
    /// unmapped one fails the whole entry list with an opaque ENOENT. This
    /// explicit declaration is the positive evidence that the slave maps the
    /// object; undeclared (index 0) = not registered.
    /// The declared index is the channel-1 base (normally 0x603F); the core
    /// applies od_offset like every other standard CiA 402 entry. Consumers:
    /// the fault-path diagnostic (SERVO_FAULT event carries the code). 0x603F
    /// is read from the fault frame's PDO; see ServoPDOData::error_code for why
    /// an SDO read can miss it.
    PdoEntryMapping error_code_pdo{0, 0, 0};

    /// @brief Shortest time [ms] a fault-reset pulse holds controlword bit 7
    /// high: a model scalar (drive model file `fault_reset.pulse_ms`, C++
    /// model kFaultResetPulseMs) carried per axis. The bus converts it to
    /// whole frames at its period, rounding up, at least one: 0 = one frame.
    /// The pulse
    /// ends early when the axis leaves FAULT (fault_reset_pulse.hpp).
    uint32_t fault_reset_pulse_ms = 0;

    /// @brief An endless axis: no position limits, so the device follows the
    /// int32 0x6064 word across its 2^32 wrap with an int64 count (see
    /// DeviceServo::read). Requires joint_offset 0. Default false: the
    /// position is the int32 word decoded directly.
    bool position_endless = false;

    /// @brief Object holding this axis's position before the drive's first
    /// enable, for a drive that reports 0x6064 = 0 until then (an absolute
    /// index, read by SDO with subindex 0 as int32 in the same counts as
    /// 0x6064). 0 = the drive reports 0x6064 while disabled; nothing is read.
    /// Read once before activation, and only when 0x6064 reads 0: the axis
    /// then reports this word, and mirrors it into 0x607A, until the drive's
    /// own 0x6064 turns nonzero. A failed read is a warning and leaves the axis
    /// as without it.
    uint16_t position_before_enable_index = 0;

    /// @brief Validate servo info
    /// @return true if all parameters are physically valid
    ///
    /// Torque limits must be finite as well as ordered: an infinite bound is
    /// ordered (min < max) yet leaves the set_command() clamp with no effective
    /// cap. NaN bounds already fail the `<` comparison; the isfinite checks add
    /// the inf case. The band must hold 0: a stop's zero torque command goes
    /// through the same clamp.
    /// od_offset must land on the CiA 402 multi-axis channel grid (channels
    /// 1..kMaxCia402Channels): an arbitrary offset would silently register PDO
    /// entries at indices that are not the intended axis, which the master
    /// reports only as an opaque ENOENT at registration time.
    /// modes_of_operation must be one of the supported cyclic-sync modes (CST, CSP,
    /// CSV, operation_mode_supported): it is the 0x6060 byte the device
    /// commands from the first frame.
    /// mode_display_timing must be one of the two declared timings: the release
    /// admission reads it, and an undeclared (or unknown-numbered) drive would
    /// be admitted under a reporting timing it never stated.
    /// pdo_mapping is not checked here: null is the scanned policy (see the
    /// field), and a declared layout is validated at initialize() Step 0.
    /// fault_reset_pulse_ms is capped at kMaxFaultResetPulseMs.
    /// An endless axis carries no joint_offset: its count is the drive's own.
    [[nodiscard]] bool is_valid() const noexcept {
        return axis_factors_valid(factors) && vendor_id != 0 && product_code != 0 && std::isfinite(min_torque_nm) &&
               std::isfinite(max_torque_nm) && min_torque_nm < max_torque_nm && min_torque_nm <= 0.0 &&
               max_torque_nm >= 0.0 && od_offset_on_channel_grid(od_offset) && slave_position >= -1 &&
               slave_position <= 0xFFFF && operation_mode_supported(modes_of_operation) &&
               mode_display_timing_declared(mode_display_timing) && fault_reset_pulse_ms <= kMaxFaultResetPulseMs &&
               (!position_endless || joint_offset.value() == 0);
    }

    /// @brief Generate human-readable dump of servo info
    /// @return Formatted string with all parameter values
    [[nodiscard]] std::string dump() const {
        std::ostringstream ss;
        ss << "ServoInfo{vendor_id=0x" << std::hex << vendor_id << ", product_code=0x" << product_code << std::dec
           << ", factors=" << ethercat::dump(factors) << ", flip=" << (flip ? "true" : "false")
           << ", joint_offset=" << joint_offset.value() << ", max_torque_nm=" << max_torque_nm
           << ", min_torque_nm=" << min_torque_nm << ", dc_assign_activate=0x" << std::hex << dc_assign_activate
           << std::dec << ", dc_sync0_shift_ns=" << dc_sync0_shift_ns << ", dc_sync0_cycle_ns=" << dc_sync0_cycle_ns
           << ", dc_write_cycle_time=" << (dc_write_cycle_time ? "true" : "false")
           << ", modes_of_operation=" << static_cast<int>(modes_of_operation)
           << ", mode_display_timing=" << static_cast<int>(mode_display_timing) << ", od_offset=0x" << std::hex
           << od_offset << std::dec << ", slave_position=" << slave_position
           << ", pdo_mapping=" << (pdo_mapping ? "declared" : "scanned") << ", error_code_pdo=0x" << std::hex
           << error_code_pdo.index << std::dec << ", fault_reset_pulse_ms=" << fault_reset_pulse_ms
           << ", position_endless=" << (position_endless ? "true" : "false") << ", position_before_enable_index=0x"
           << std::hex << position_before_enable_index << std::dec << "}";
        return ss.str();
    }
};

/// @brief One IO slave's identity. The device itself comes from the injected
/// IODeviceFactory (io_device_factory.hpp).
struct IOInfo {
    uint32_t vendor_id;
    uint32_t product_code;
};

/**
 * @brief One startup SDO write the bus reserves on a servo slave (drive-model
 * schema §4.7): IgH `ecrt_slave_config_sdo8/16/32`, which the master writes
 * when it configures the slave (PREOP → SAFEOP). `size` is the wire width in
 * bytes and `value` its unsigned bit pattern, never widened, never truncated.
 */
struct StartupSdo {
    uint16_t index = 0;
    uint8_t subindex = 0;
    uint8_t size = 0;  ///< 1, 2 or 4
    uint32_t value = 0;

    /// @brief `size` is 1, 2 or 4 and `value` fits in it.
    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return size == 4 || (size == 2 && value <= 0xFFFFu) || (size == 1 && value <= 0xFFu);
    }
    constexpr bool operator==(const StartupSdo&) const = default;
};

/**
 * @brief One servo slave's SDO list keyed by ring position, in list order.
 *
 * As startup SDOs (EtherCATBus::set_startup_sdos) these are writes: order and
 * repetition are data (a slave may write the same entry twice), the bus
 * reserves them as listed and never merges or sorts, and across slaves the
 * master gives no total order. As stored SDOs (EtherCATBus::set_stored_sdos,
 * drive-model schema §4.9) each is an expectation: `size` bytes, compared
 * little-endian with `value`; the bus reads them before activation and warns
 * on a difference, it never writes them.
 */
struct SlaveSdos {
    uint32_t slave_position = 0;
    std::vector<StartupSdo> sdos;
};

/**
 * @brief What a verified drive-model approval record claims a servo slave is
 * (drive-model schema §6): the SII serial number, the SII revision number
 * (the CoE 0x1018:03 value the master read from the SII), CoE 0x1009
 * (hardware version), 0x100A (software version) and (for a slave whose mapping the
 * bus scans, null ServoInfo::pdo_mapping) the digest of the scanned PDO
 * mapping (pdo_scan.hpp pdo_mapping_digest(): `sha256:` + 64 hex of the
 * format_pdo_mapping text, the value the bus prints in its INFO line). Each
 * field is a declaration: nullopt = the record does not claim that value (a
 * drive that does not implement the object; a declared-mapping slave has no
 * scan to digest), a value = it must match the live slave exactly, 0
 * included (some drives report serial 0; the value is compared as
 * declared; what it identifies is the record's identity_scope). Trivially
 * copyable: the drive-model spec carries one per slave.
 */
struct SlaveIdentity {
    static constexpr size_t kVersionLength = 32;  ///< NUL included; 31 bytes of text
    static constexpr size_t kDigestLength = 72;   ///< `sha256:` + 64 hex + NUL
    std::optional<uint32_t> serial_number{};
    std::optional<uint32_t> revision_number{};
    std::optional<std::array<char, kVersionLength>> hardware_version{};
    std::optional<std::array<char, kVersionLength>> software_version{};
    std::optional<std::array<char, kDigestLength>> pdo_mapping_digest{};
    constexpr bool operator==(const SlaveIdentity&) const = default;
};
static_assert(std::is_trivially_copyable_v<SlaveIdentity>,
              "SlaveIdentity must stay a plain VALUE — five optionals of number and fixed text");

/**
 * @brief A servo slave's expected identity keyed by ring position (the
 * SlaveSdos shape), EtherCATBus::set_expected_slave_identity.
 */
struct SlaveExpectedIdentity {
    uint32_t slave_position = 0;
    SlaveIdentity identity{};
};

/**
 * @brief EtherCATBus::initialize() Step 0: the injected configuration checked
 * before any master contact. ok = accepted; otherwise an error naming the first defect,
 * naming the servo (list index) or slave position.
 *
 * Pure (no ecrt call) so a file tool can run the same check offline. Checks:
 * each servo's channel factors, CiA 402 transition profile and ErrorCode role;
 * across servos, unique (slave, channel) pairs and one identity, one DC
 * setting and one PDO layout object per slave; each declared PDO layout
 * against every axis it hosts (validate_pdo_mapping; a scanned slave is
 * checked at Step 3, after the scan); every startup/stored SDO list and
 * expected identity names a servo slave, every SDO a 1/2/4-byte value that
 * fits. An invalid map would let servo_conversions fabricate a 0 command
 * every cycle with no warning. Absent config is not checked here (initialize
 * refuses it on its own).
 */
[[nodiscard]] core::Result<void> validate_bus_configuration(const std::vector<ServoInfo>& servos,
                                                            const std::vector<SlaveSdos>& startup,
                                                            const std::vector<SlaveSdos>& stored,
                                                            const std::vector<SlaveExpectedIdentity>& expected);

}  // namespace plem::fieldbus::ethercat
