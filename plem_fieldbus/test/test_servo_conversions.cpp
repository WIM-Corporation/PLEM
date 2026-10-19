// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include "plem/fieldbus/ethercat/config/configuration.hpp"
#include "plem/fieldbus/ethercat/utils/servo_conversions.hpp"
#include "plem/fieldbus/test/single_axis_cst_layout.hpp"

#include <cstdint>

using namespace plem::fieldbus::ethercat;
using namespace plem::core::units;

// Input adapters
// These tests speak ServoSpec, the model's raw material, and hand the
// conversions the channel the assembly helper composes from it. Test-side
// adapters only; production conversions take the channel.
namespace {
inline Pulse angle_to_pulse(const Radian& angle, const ServoSpec& spec) {
    return plem::fieldbus::ethercat::angle_to_pulse(angle, make_axis_factors(spec).position);
}
inline Radian to_angle(const Pulse& pulse, const ServoSpec& spec) {
    return plem::fieldbus::ethercat::to_angle(pulse, make_axis_factors(spec).position);
}
inline PulsePerSecond to_pulse_velocity(const RadianPerSecond& v, const ServoSpec& spec) {
    return plem::fieldbus::ethercat::to_pulse_velocity(v, make_axis_factors(spec).velocity);
}
inline RadianPerSecond to_angular_velocity(const PulsePerSecond& v, const ServoSpec& spec) {
    return plem::fieldbus::ethercat::to_angular_velocity(v, make_axis_factors(spec).velocity);
}
inline TorqueCount to_torque_count(const NewtonMeter& t, const ServoSpec& spec) {
    return plem::fieldbus::ethercat::to_torque_count(t, make_axis_factors(spec).torque);
}
inline NewtonMeter to_torque(const TorqueCount& c, const ServoSpec& spec) {
    return plem::fieldbus::ethercat::to_torque(c, make_axis_factors(spec).torque);
}
}  // namespace

struct ServoSpecTestData {
    ServoSpec spec;
    const char* description;
};

class ServoConversionsTest : public ::testing::Test {
  protected:
    void SetUp() override {
        typical_spec_.gear_ratio = 100.0;
        typical_spec_.torque_constant = 0.12;       // Nm/A
        typical_spec_.encoder_resolution = 131072;  // 17-bit encoder
        typical_spec_.counts_per_amp = 50.0;

        // High-resolution servo specifications
        high_res_spec_.gear_ratio = 100.0;
        high_res_spec_.torque_constant = 0.1;
        high_res_spec_.encoder_resolution = 524288;  // 19-bit encoder
        high_res_spec_.counts_per_amp = 96.0;

        // Edge case specifications
        edge_spec_.gear_ratio = 1.0;
        edge_spec_.torque_constant = 1.0;
        edge_spec_.encoder_resolution = 1000;
        edge_spec_.counts_per_amp = 1.0;
    }

    ServoSpec typical_spec_;
    ServoSpec high_res_spec_;
    ServoSpec edge_spec_;
    static constexpr double epsilon_ = 1e-6;
};

TEST_F(ServoConversionsTest, AngleToPulseConversion) {
    auto zero_angle = Radian(0.0);
    auto zero_pulse = angle_to_pulse(zero_angle, typical_spec_);
    EXPECT_EQ(zero_pulse.value(), 0);

    // One full rotation (2π radians)
    auto full_rotation = Radian(2.0 * M_PI);
    auto full_rotation_pulse = angle_to_pulse(full_rotation, typical_spec_);
    // Expected: encoder_resolution * gear_ratio = 131072 * 100
    EXPECT_EQ(full_rotation_pulse.value(), 131072 * 100);

    // Half rotation (π radians)
    auto half_rotation = Radian(M_PI);
    auto half_rotation_pulse = angle_to_pulse(half_rotation, typical_spec_);
    EXPECT_EQ(half_rotation_pulse.value(), 131072 * 100 / 2);

    auto neg_angle = Radian(-M_PI);
    auto neg_pulse = angle_to_pulse(neg_angle, typical_spec_);
    EXPECT_EQ(neg_pulse.value(), -131072 * 100 / 2);
}

TEST_F(ServoConversionsTest, PulseToAngleConversion) {
    auto zero_pulse = Pulse(0);
    auto zero_angle = to_angle(zero_pulse, typical_spec_);
    EXPECT_NEAR(zero_angle.value(), 0.0, epsilon_);

    auto full_rotation_pulse = Pulse(typical_spec_.encoder_resolution * typical_spec_.gear_ratio);
    auto full_rotation_angle = to_angle(full_rotation_pulse, typical_spec_);
    EXPECT_NEAR(full_rotation_angle.value(), 2.0 * M_PI, epsilon_);

    // Test negative pulse - use double calculation to avoid integer division issues
    int32_t half_rotation_count =
        static_cast<int32_t>(typical_spec_.encoder_resolution * typical_spec_.gear_ratio / 2.0);
    auto neg_pulse = Pulse(-half_rotation_count);
    auto neg_angle = to_angle(neg_pulse, typical_spec_);
    EXPECT_NEAR(neg_angle.value(), -M_PI, epsilon_);
}

TEST_F(ServoConversionsTest, AngularVelocityToPulseVelocity) {
    auto zero_vel = RadianPerSecond(0.0);
    auto zero_pulse_vel = to_pulse_velocity(zero_vel, typical_spec_);
    EXPECT_EQ(zero_pulse_vel.value(), 0);

    // 1 rad/s
    // The conversion narrows through the saturating boundary constructor,
    // which rounds to nearest; a bare static_cast would truncate toward zero
    // (a systematic up-to-1-count bias). Pin round-to-nearest.
    auto one_rad_per_sec = RadianPerSecond(1.0);
    auto pulse_vel = to_pulse_velocity(one_rad_per_sec, typical_spec_);
    int32_t expected =
        static_cast<int32_t>(std::round(typical_spec_.encoder_resolution * typical_spec_.gear_ratio / (2.0 * M_PI)));
    EXPECT_EQ(pulse_vel.value(), expected);
}

TEST_F(ServoConversionsTest, PulseVelocityToAngularVelocity) {
    auto zero_pulse_vel = PulsePerSecond(0);
    auto zero_ang_vel = to_angular_velocity(zero_pulse_vel, typical_spec_);
    EXPECT_NEAR(zero_ang_vel.value(), 0.0, epsilon_);

    // Test encoder_resolution * gear_ratio pulses/sec = 2π rad/s
    int32_t full_rotation_pulses = typical_spec_.encoder_resolution * typical_spec_.gear_ratio;
    auto full_rotation_pulse_vel = PulsePerSecond(full_rotation_pulses);
    auto full_rotation_ang_vel = to_angular_velocity(full_rotation_pulse_vel, typical_spec_);
    EXPECT_NEAR(full_rotation_ang_vel.value(), 2.0 * M_PI, epsilon_);
}

TEST_F(ServoConversionsTest, TorqueToRawConversion) {
    auto zero_torque = NewtonMeter(0.0);
    auto zero_raw = to_torque_count(zero_torque, typical_spec_);
    EXPECT_EQ(zero_raw, TorqueCount(0));

    // Test positive torque with counts_per_amp = 50
    auto torque_1nm = NewtonMeter(1.0);
    auto raw_50 = to_torque_count(torque_1nm, typical_spec_);
    // Formula: 1.0 * 50 / 0.12 / 100 ≈ 4.17
    int16_t expected_50 = static_cast<int16_t>(std::round(1.0 * 50.0 / 0.12 / 100.0));
    EXPECT_EQ(raw_50, TorqueCount(expected_50));

    // Test with counts_per_amp = 96
    auto raw_96 = to_torque_count(torque_1nm, high_res_spec_);
    // Formula: 1.0 * 96 / 0.1 / 100 = 9.6 ≈ 10
    int16_t expected_96 = static_cast<int16_t>(std::round(1.0 * 96.0 / 0.1 / 100.0));
    EXPECT_EQ(raw_96, TorqueCount(expected_96));

    auto neg_torque = NewtonMeter(-0.5);
    auto neg_raw = to_torque_count(neg_torque, typical_spec_);
    int16_t expected_neg = static_cast<int16_t>(std::round(-0.5 * 50.0 / 0.12 / 100.0));
    EXPECT_EQ(neg_raw, TorqueCount(expected_neg));

    // Large torque (overflow check)
    auto large_torque = NewtonMeter(100.0);
    auto large_raw = to_torque_count(large_torque, typical_spec_);
    int16_t expected_large = static_cast<int16_t>(std::round(100.0 * 50.0 / 0.12 / 100.0));
    EXPECT_EQ(large_raw, TorqueCount(expected_large));
}

TEST_F(ServoConversionsTest, RawToTorqueConversion) {
    TorqueCount zero_count(0);
    auto zero_torque = to_torque(zero_count, typical_spec_);
    EXPECT_NEAR(zero_torque.value(), 0.0, epsilon_);

    // Test positive torque count with counts_per_amp = 50
    TorqueCount count_val(100);
    auto torque = to_torque(count_val, typical_spec_);
    // Formula: 100 * 0.12 * 100 / 50 = 24
    double expected = 100.0 * 0.12 * 100.0 / 50.0;
    EXPECT_NEAR(torque.value(), expected, epsilon_);

    // Test with counts_per_amp = 96
    auto torque_96 = to_torque(count_val, high_res_spec_);
    // Formula: 100 * 0.1 * 100 / 96 ≈ 10.42
    expected = 100.0 * 0.1 * 100.0 / 96.0;
    EXPECT_NEAR(torque_96.value(), expected, epsilon_);

    TorqueCount neg_count(-50);
    auto neg_torque = to_torque(neg_count, typical_spec_);
    expected = -50.0 * 0.12 * 100.0 / 50.0;
    EXPECT_NEAR(neg_torque.value(), expected, epsilon_);

    TorqueCount max_count(INT16_MAX);
    auto max_torque = to_torque(max_count, typical_spec_);
    expected = static_cast<double>(INT16_MAX) * 0.12 * 100.0 / 50.0;
    EXPECT_NEAR(max_torque.value(), expected, epsilon_);
}

TEST_F(ServoConversionsTest, RoundTripConversions) {
    // Angle round-trip
    auto original_angle = Radian(1.234);
    auto pulse = angle_to_pulse(original_angle, typical_spec_);
    auto recovered_angle = to_angle(pulse, typical_spec_);
    // Some precision loss is expected due to integer pulse representation
    EXPECT_NEAR(recovered_angle.value(), original_angle.value(), 1e-4);

    // Angular velocity round-trip
    auto original_vel = RadianPerSecond(5.678);
    auto pulse_vel = to_pulse_velocity(original_vel, typical_spec_);
    auto recovered_vel = to_angular_velocity(pulse_vel, typical_spec_);
    EXPECT_NEAR(recovered_vel.value(), original_vel.value(), 1e-3);

    // Torque round-trip
    auto original_torque = NewtonMeter(2.5);
    auto torque_count = to_torque_count(original_torque, typical_spec_);
    auto recovered_torque = to_torque(torque_count, typical_spec_);
    // Some precision loss is expected due to int16_t representation: at most
    // half a count (one count = 0.12 * 100.0 / 50.0 Nm).
    EXPECT_NEAR(recovered_torque.value(), original_torque.value(), 0.5 * 0.12 * 100.0 / 50.0);
}

TEST_F(ServoConversionsTest, EdgeCaseSpecifications) {
    // Test with gear_ratio = 1, encoder_resolution = 1000, counts_per_amp = 1
    auto angle = Radian(2.0 * M_PI);
    auto pulse = angle_to_pulse(angle, edge_spec_);
    EXPECT_EQ(pulse.value(), 1000);

    auto torque = NewtonMeter(1.0);
    auto raw = to_torque_count(torque, edge_spec_);
    EXPECT_EQ(raw, TorqueCount(1));  // 1.0 * 1 / 1 / 1 = 1

    // Test very small torque constant
    ServoSpec small_kt_spec = edge_spec_;
    small_kt_spec.torque_constant = 0.001;
    auto raw_small_kt = to_torque_count(torque, small_kt_spec);
    EXPECT_EQ(raw_small_kt, TorqueCount(1000));  // 1.0 * 1 / 0.001 / 1 = 1000
}

TEST_F(ServoConversionsTest, BoundaryValues) {
    auto max_pulse = Pulse(INT32_MAX);
    auto max_angle = to_angle(max_pulse, typical_spec_);
    EXPECT_TRUE(std::isfinite(max_angle.value()));

    auto min_pulse = Pulse(INT32_MIN);
    auto min_angle = to_angle(min_pulse, typical_spec_);
    EXPECT_TRUE(std::isfinite(min_angle.value()));

    // Test a realistic maximum angle (±10 rotations = ±62.8 radians)
    // Most axes stay within ±360 degrees, but continuous-rotation axes might accumulate
    auto realistic_max = Radian(62.8);  // 10 rotations
    auto realistic_pulse = angle_to_pulse(realistic_max, typical_spec_);
    // Should be well within int32_t range
    EXPECT_LT(std::abs(realistic_pulse.value()), INT32_MAX / 2);
    // Past int32 the conversion saturates: AngleToPulseSaturatesAtInt32Boundary.
}

class ServoConversionsParameterizedTest : public ::testing::TestWithParam<ServoSpecTestData> {
  protected:
    static constexpr double epsilon_ = 1e-6;
};

TEST_P(ServoConversionsParameterizedTest, AngleRoundTripIsAccurate) {
    auto [spec, description] = GetParam();

    std::vector<Radian> test_angles = {
        Radian(0.0),
        Radian(M_PI / 4),  // 45 degrees
        Radian(M_PI / 2),  // 90 degrees
        Radian(M_PI),      // 180 degrees
        Radian(2 * M_PI),  // 360 degrees
        Radian(-M_PI),     // -180 degrees
    };

    for (const auto& angle : test_angles) {
        auto pulse = angle_to_pulse(angle, spec);
        auto recovered = to_angle(pulse, spec);

        // Expect accuracy within encoder resolution
        double expected_accuracy = (2.0 * M_PI) / (spec.encoder_resolution * spec.gear_ratio);
        EXPECT_NEAR(recovered.value(), angle.value(), expected_accuracy)
            << "Servo: " << description << ", Angle: " << angle.value();
    }
}

// A torque whose count exceeds the int16 register saturates instead of an
// out-of-range double→int16 cast (UB).
TEST_F(ServoConversionsTest, ToTorqueCountSaturatesAtInt16Boundary) {
    auto huge = to_torque_count(NewtonMeter(1.0e6), typical_spec_);
    EXPECT_EQ(huge.value(), INT16_MAX);
    auto huge_neg = to_torque_count(NewtonMeter(-1.0e6), typical_spec_);
    EXPECT_EQ(huge_neg.value(), INT16_MIN);
}

// to_torque_count is an installed public-header entry point, so it must be
// total on its own: non-finite torque maps to 0 (safe neutral) instead of a
// NaN reaching the int16 cast (UB). The production ingress
// (DeviceServo::set_command) additionally rejects non-finite fail-fast; this
// pins the release-mode backstop for direct header callers.
TEST_F(ServoConversionsTest, ToTorqueCountZeroesNonFiniteTorque) {
    EXPECT_EQ(to_torque_count(NewtonMeter(std::numeric_limits<double>::quiet_NaN()), typical_spec_).value(), 0);
    EXPECT_EQ(to_torque_count(NewtonMeter(std::numeric_limits<double>::infinity()), typical_spec_).value(), 0);
    EXPECT_EQ(to_torque_count(NewtonMeter(-std::numeric_limits<double>::infinity()), typical_spec_).value(), 0);
}

// angle_to_pulse routes its int32 narrowing through the saturating Pulse
// boundary constructor: a huge angle saturates, never a UB cast.
TEST_F(ServoConversionsTest, AngleToPulseSaturatesAtInt32Boundary) {
    EXPECT_EQ(angle_to_pulse(Radian(1.0e12), typical_spec_).value(), INT32_MAX);
    EXPECT_EQ(angle_to_pulse(Radian(-1.0e12), typical_spec_).value(), INT32_MIN);
}

// angle_to_pulse rounds to nearest through the boundary constructor; a
// bare static_cast would truncate toward zero. Pin a
// fractional count where the two disagree.
TEST_F(ServoConversionsTest, AngleToPulseRoundsToNearest) {
    double rad_to_pulse = typical_spec_.encoder_resolution * typical_spec_.gear_ratio / (2.0 * M_PI);
    auto angle = Radian(10.7 / rad_to_pulse);                     // exactly 10.7 pulses worth
    EXPECT_EQ(angle_to_pulse(angle, typical_spec_).value(), 11);  // truncation would give 10
}

TEST_P(ServoConversionsParameterizedTest, TorqueRoundTripIsAccurate) {
    auto [spec, description] = GetParam();

    std::vector<NewtonMeter> test_torques = {
        NewtonMeter(0.0), NewtonMeter(1.0), NewtonMeter(5.0), NewtonMeter(-1.0), NewtonMeter(-5.0),
    };

    for (const auto& torque : test_torques) {
        auto count = to_torque_count(torque, spec);
        auto recovered = to_torque(count, spec);

        // Expect accuracy within 16-bit quantization
        double max_torque = INT16_MAX * spec.torque_constant * spec.gear_ratio / spec.counts_per_amp;
        double expected_accuracy = max_torque / INT16_MAX;

        EXPECT_NEAR(recovered.value(), torque.value(), expected_accuracy * 2)
            << "Servo: " << description << ", Torque: " << torque.value();
    }
}

INSTANTIATE_TEST_SUITE_P(
    CommonServoSpecs, ServoConversionsParameterizedTest,
    ::testing::Values(ServoSpecTestData{{100.0, 0.12, 131072, 50.0}, "Typical servo (17-bit encoder, 100:1 gear)"},
                      ServoSpecTestData{{100.0, 0.1, 524288, 96.0}, "High-res servo (19-bit encoder, 100:1 gear)"},
                      ServoSpecTestData{{50.0, 0.05, 65536, 24.0}, "Low-res servo (16-bit encoder, 50:1 gear)"},
                      ServoSpecTestData{{160.0, 0.12, 262144, 64.0}, "High-gear servo (18-bit encoder, 160:1 gear)"}));

// RATED_PER_MILLE (the CiA 402 standard torque unit)

// One count = 1/1000 of the motor rated torque: the SCALED_CURRENT formula
// with counts_per_unit 1000 and torque_constant = rated torque (golden: τ 1 Nm,
// rated 1.27 Nm, gear 1 → 787.4 → 787).
TEST(ServoConversionsRatedPerMille, EncodesPerMilleOfRatedTorqueAndDecodesBack) {
    const TorqueChannel ch{TorqueUnit::RATED_PER_MILLE, TorqueChannel::kPerMilleScale, 1.27, 1.0};
    EXPECT_EQ(plem::fieldbus::ethercat::to_torque_count(NewtonMeter(1.0), ch).value(), 787);
    EXPECT_DOUBLE_EQ(plem::fieldbus::ethercat::to_torque(TorqueCount(787), ch).value(), 787 * 1.27 * 1.0 / 1000.0);
    EXPECT_NEAR(plem::fieldbus::ethercat::to_torque(TorqueCount(787), ch).value(), 1.0, 0.5 * 1.27 / 1000.0);
    // The gear divides as in SCALED_CURRENT: the output-shaft torque of 1000‰ is rated × gear.
    const TorqueChannel geared{TorqueUnit::RATED_PER_MILLE, TorqueChannel::kPerMilleScale, 1.27, 100.0};
    EXPECT_EQ(plem::fieldbus::ethercat::to_torque_count(NewtonMeter(1.27 * 100.0), geared).value(), 1000);
}

// velocity_factor

// A drive that reports velocity in its own unit: the scale multiplies the
// decode and divides the encode; omitting it under-reports speed by the
// factor, which an inner velocity loop cannot detect.
TEST(ServoConversionsVelocityScale, ScaleMultipliesDecodeAndDividesEncode) {
    ServoSpec spec{10.0, 0.05, 4096, 1.0};
    spec.velocity_factor = 4.0;

    ServoSpec unit_spec{10.0, 0.05, 4096, 1.0};  // default 1.0

    const PulsePerSecond wire(8192);
    const double decoded = to_angular_velocity(wire, spec).value();
    const double decoded_unit = to_angular_velocity(wire, unit_spec).value();
    EXPECT_DOUBLE_EQ(decoded, decoded_unit * 4.0);

    // Exact inverse pair: encode(decode(x)) == x for the same spec.
    const PulsePerSecond re_encoded = to_pulse_velocity(RadianPerSecond(decoded), spec);
    EXPECT_EQ(re_encoded.value(), wire.value());

    // Default 1.0 leaves the conversion bit-identical for models that declare
    // no velocity_factor.
    ASSERT_DOUBLE_EQ(unit_spec.velocity_factor, 1.0);
    EXPECT_DOUBLE_EQ(decoded_unit, 8192.0 * (2.0 * M_PI) / (4096.0 * 10.0));
}

// Debug contract: a direct caller cannot slip past the channel precondition
//
// The gates protect an assembled system; these pin what the installed inline
// functions promise a direct caller in a debug build: the assert covers the
// whole channel (unit and parameters), and a non-finite SI input into the
// position/velocity encoders. In a release build the assert is gone and the
// saturating boundary constructors map non-finite to 0 (the #else branch
// below pins that side).

TEST(ServoConversionsDebugContractDeathTest, ZeroParameterIsRefusedNotConvertedToZero) {
    auto c = make_axis_factors(plem::fieldbus::test::kSyntheticServoSpec);
    c.torque.torque_constant = 0.0;
    auto pos = make_axis_factors(plem::fieldbus::test::kSyntheticServoSpec).position;
    pos.encoder_resolution = 0;
#ifndef NDEBUG
    EXPECT_DEATH((void)plem::fieldbus::ethercat::to_torque_count(NewtonMeter(1.0), c.torque), "is_valid");
    EXPECT_DEATH((void)plem::fieldbus::ethercat::to_angle(Pulse(1), pos), "is_valid");
#else
    GTEST_SKIP() << "release build: the debug precondition is compiled out (the gates refuse the channel)";
#endif
}

TEST(ServoConversionsDebugContractDeathTest, NonFinitePositionOrVelocityInputIsRefused) {
    const auto c = make_axis_factors(plem::fieldbus::test::kSyntheticServoSpec);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
#ifndef NDEBUG
    EXPECT_DEATH((void)plem::fieldbus::ethercat::angle_to_pulse(Radian(nan), c.position), "isfinite");
    EXPECT_DEATH((void)plem::fieldbus::ethercat::to_pulse_velocity(RadianPerSecond(inf), c.velocity), "isfinite");
#else
    EXPECT_EQ(plem::fieldbus::ethercat::angle_to_pulse(Radian(nan), c.position).value(), 0);
    EXPECT_EQ(plem::fieldbus::ethercat::to_pulse_velocity(RadianPerSecond(inf), c.velocity).value(), 0);
#endif
}

// Checked conversions (the CSP/CSV command path)
// The command path must not saturate: a target the wire cannot carry is a
// violation (protective stop), never a nearby word. Offset and flip are applied
// exactly once, at the PDO boundary, and `si` is the angle/velocity the chosen
// count decodes back to, which is what the limit and step checks check.

namespace {
constexpr auto kSpec = plem::fieldbus::test::kSyntheticServoSpec;
const auto kChannels = make_axis_factors(kSpec);
constexpr double kPositionCountsPerTurn = kSpec.encoder_resolution * kSpec.gear_ratio;
}  // namespace

TEST(CheckedConversions, PositionAppliesOffsetAndFlipOnceAndReportsTheQuantizedAngle) {
    const auto& ch = kChannels.position;
    const int32_t count = plem::fieldbus::ethercat::angle_to_pulse(Radian(0.5), ch).value();
    const auto plain = encode_position_checked(0.5, ch, 1000, false);
    ASSERT_TRUE(plain);
    EXPECT_EQ(plain->word, count + 1000);
    EXPECT_DOUBLE_EQ(plain->si, plem::fieldbus::ethercat::to_angle(Pulse(count), ch).value());
    const auto flipped = encode_position_checked(0.5, ch, 1000, true);
    ASSERT_TRUE(flipped);
    EXPECT_EQ(flipped->word, 1000 - count);
    EXPECT_DOUBLE_EQ(flipped->si, plain->si);
    EXPECT_EQ(decode_position_word(plain->word, ch, 1000, false), plain->si);
    EXPECT_EQ(decode_position_word(flipped->word, ch, 1000, true), plain->si);
}

TEST(CheckedConversions, PositionRefusesWhatTheWireCannotCarry) {
    const auto& ch = kChannels.position;
    EXPECT_FALSE(encode_position_checked(std::numeric_limits<double>::quiet_NaN(), ch, 0, false));
    EXPECT_FALSE(encode_position_checked(std::numeric_limits<double>::infinity(), ch, 0, false));
    EXPECT_FALSE(encode_position_checked(1e6, ch, 0, false));  // ~1.3e12 counts
    // ~2e9 counts fits alone but not with the offset added (either sign convention).
    const double near_limit = 2e9 / (kPositionCountsPerTurn / (2.0 * M_PI));
    EXPECT_TRUE(encode_position_checked(near_limit, ch, 0, false));
    EXPECT_FALSE(encode_position_checked(near_limit, ch, 2'000'000'000, false));
    EXPECT_FALSE(encode_position_checked(near_limit, ch, -2'000'000'000, true));
}

TEST(CheckedConversions, PositionWordDecodeReportsSaturation) {
    const auto& ch = kChannels.position;
    EXPECT_FALSE(decode_position_word(-2'000'000'000, ch, 2'000'000'000, false));
    EXPECT_FALSE(decode_position_word(-2'000'000'000, ch, 2'000'000'000, true));
    EXPECT_TRUE(decode_position_word(123, ch, 2'000'000'000, false));
}

TEST(CheckedConversions, HalfCountBoundariesRoundLikeTheSaturatingEncoders) {
    const auto& pos = kChannels.position;
    const auto& vel = kChannels.velocity;
    const double pos_factor = kPositionCountsPerTurn / (2.0 * M_PI);
    for (const double counts : {100.5, -100.5, 0.5, -0.5, 7.4999, 7.5001}) {
        SCOPED_TRACE(counts);
        const double rad = counts / pos_factor;
        EXPECT_EQ(encode_position_checked(rad, pos, 0, false)->word,
                  plem::fieldbus::ethercat::angle_to_pulse(Radian(rad), pos).value());
        EXPECT_EQ(encode_velocity_checked(rad, vel, false)->word,
                  plem::fieldbus::ethercat::to_pulse_velocity(RadianPerSecond(rad), vel).value());
    }
}

// The encode factors' evaluation order is contract (servo_conversions.hpp):
// the factor is formed first, resolution·gear / (2π[·velocity_factor]), and
// only then multiplied by the SI value. These synthetic inputs sit a hair
// under a half count, where that order and the obvious rewrites land on
// different integers (checked below, so the pin cannot go vacuous if the
// numbers change). The expected words are literals, not recomputed: a
// reordered formula (in either the saturating encoder or its checked twin)
// turns this red.
TEST(CheckedConversions, EncodeFactorOrderIsPinnedAtAHalfCountBoundary) {
    const PositionChannel pos{MotionUnit::ROTARY_MOTOR_ENCODER, 10000, 7.0};
    const double rad = 0.00013463968515384827;  // 1.4999999999999998 counts by the contract order
    ASSERT_NE(std::round(rad * 10000 * 7.0 / (2.0 * M_PI)), 1.0) << "the input must sit on the boundary";
    ASSERT_NE(std::round(rad * (10000 * (7.0 / (2.0 * M_PI)))), 1.0) << "the input must sit on the boundary";
    EXPECT_EQ(plem::fieldbus::ethercat::angle_to_pulse(Radian(rad), pos).value(), 1);
    EXPECT_EQ(encode_position_checked(rad, pos, 0, false)->word, 1);

    const VelocityChannel vel{MotionUnit::ROTARY_MOTOR_ENCODER, 10000, 7.0, 3.0};
    const double rad_s = 0.002558154017923117;  // 9.5 counts by the contract order
    ASSERT_NE(std::round(rad_s * 10000 * 7.0 / (2.0 * M_PI * 3.0)), 10.0) << "the input must sit on the boundary";
    ASSERT_NE(std::round(rad_s * 10000 * 7.0 / (2.0 * M_PI) / 3.0), 10.0) << "the input must sit on the boundary";
    EXPECT_EQ(plem::fieldbus::ethercat::to_pulse_velocity(RadianPerSecond(rad_s), vel).value(), 10);
    EXPECT_EQ(encode_velocity_checked(rad_s, vel, false)->word, 10);
}

TEST(CheckedConversions, VelocityFlipsSignOnceAndRefusesOutOfRange) {
    const auto& ch = kChannels.velocity;
    const int32_t count = plem::fieldbus::ethercat::to_pulse_velocity(RadianPerSecond(1.25), ch).value();
    const auto plain = encode_velocity_checked(1.25, ch, false);
    ASSERT_TRUE(plain);
    EXPECT_EQ(plain->word, count);
    EXPECT_DOUBLE_EQ(plain->si, plem::fieldbus::ethercat::to_angular_velocity(PulsePerSecond(count), ch).value());
    const auto flipped = encode_velocity_checked(1.25, ch, true);
    ASSERT_TRUE(flipped);
    EXPECT_EQ(flipped->word, -count);
    EXPECT_DOUBLE_EQ(flipped->si, plain->si);
    EXPECT_FALSE(encode_velocity_checked(std::numeric_limits<double>::quiet_NaN(), ch, false));
    EXPECT_FALSE(encode_velocity_checked(1e6, ch, false));
}

// Endless position axis (limits.position: null)
// The host counts in int64 from the drive's int32 words: wrap_diff is the one
// place a word difference is formed, the domain is ±kEndlessCountDomain counts.

TEST(EndlessPosition, WrapDiffTakesTheShortWayAcrossTheInt32Wrap) {
    constexpr int32_t kMin = std::numeric_limits<int32_t>::min();
    constexpr int32_t kMax = std::numeric_limits<int32_t>::max();
    EXPECT_EQ(wrap_diff(kMin, kMax), 1) << "7FFFFFFF → 80000000 is one count forward";
    EXPECT_EQ(wrap_diff(kMax, kMin), -1);
    EXPECT_EQ(wrap_diff(kMin, 0), kMin) << "exactly half a period reads as −2^31";
    EXPECT_EQ(wrap_diff(0, kMin), kMin);
    EXPECT_EQ(wrap_diff(5, -3), 8);
    EXPECT_EQ(wrap_diff(-3, 5), -8);
}

TEST(EndlessPosition, EncodeFoldsTheCountIntoAWrappedWordAndReportsItsAngle) {
    const auto& ch = kChannels.position;
    const double per_count = 2.0 * M_PI / kPositionCountsPerTurn;
    const int64_t count = (int64_t{1} << 31) + 5;  // past INT32_MAX
    const auto plain = encode_endless_position_checked(static_cast<double>(count) * per_count, ch, false);
    ASSERT_TRUE(plain);
    EXPECT_EQ(plain->word, std::numeric_limits<int32_t>::min() + 5);
    EXPECT_DOUBLE_EQ(plain->si, static_cast<double>(count) * per_count);
    const auto flipped = encode_endless_position_checked(static_cast<double>(count) * per_count, ch, true);
    ASSERT_TRUE(flipped);
    EXPECT_EQ(flipped->word, std::numeric_limits<int32_t>::max() - 4) << "−(2^31 + 5) folded";
    EXPECT_DOUBLE_EQ(flipped->si, plain->si) << "si is the joint frame";
    EXPECT_EQ(endless_count_to_angle(count, ch), plain->si);
}

TEST(EndlessPosition, EncodeRefusesNonFiniteAndOutOfDomainBeforeRounding) {
    const auto& ch = kChannels.position;
    const double per_count = 2.0 * M_PI / kPositionCountsPerTurn;
    const auto domain = static_cast<double>(kEndlessCountDomain);
    EXPECT_FALSE(encode_endless_position_checked(std::numeric_limits<double>::quiet_NaN(), ch, false));
    EXPECT_FALSE(encode_endless_position_checked(std::numeric_limits<double>::infinity(), ch, false));
    EXPECT_FALSE(encode_endless_position_checked(-std::numeric_limits<double>::infinity(), ch, true));
    EXPECT_TRUE(encode_endless_position_checked(domain * per_count * (1.0 - 1e-12), ch, false));
    EXPECT_FALSE(encode_endless_position_checked(domain * per_count * (1.0 + 1e-9), ch, false));
    EXPECT_FALSE(encode_endless_position_checked(-domain * per_count * (1.0 + 1e-9), ch, true));
    EXPECT_FALSE(encode_endless_position_checked(7e15 * per_count, ch, false))
        << "7e15 counts: past the domain, refused before any step";
}

// The wire's sign convention lives in the saturating Pulse/PulsePerSecond
// boundary and again in the checked encoders and decoder. A unit channel (one
// count per radian, exactly) lets every integer joint count be passed as an
// angle, so the two can be compared over the int32 edges: where the wire word
// fits, both give the same word and the checked angle is that count; where it
// does not, the checked side refuses and the saturating side clamps.
TEST(CheckedConversions, CheckedAndSaturatingBoundariesAgreeOnTheWireConvention) {
    constexpr int64_t kMin = std::numeric_limits<int32_t>::min();
    constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
    const PositionChannel pos{MotionUnit::ROTARY_MOTOR_ENCODER, 1, 2.0 * M_PI};
    const VelocityChannel vel{MotionUnit::ROTARY_MOTOR_ENCODER, 1, 2.0 * M_PI, 1.0};
    ASSERT_EQ(pos.encoder_resolution * pos.gear_ratio / (2.0 * M_PI), 1.0) << "the channel must be one count per rad";
    ASSERT_EQ(vel.encoder_resolution * vel.gear_ratio / (2.0 * M_PI * vel.velocity_factor), 1.0);
    const auto clamp = [&](int64_t v) { return static_cast<int32_t>(std::clamp(v, kMin, kMax)); };
    const std::initializer_list<int64_t> values = {kMin, kMin + 1,      -2'000'000'000, -1,  0,
                                                   1,    2'000'000'000, kMax - 1,       kMax};
    for (const bool flip : {false, true}) {
        for (const int64_t a : values) {
            for (const int64_t b : values) {
                SCOPED_TRACE(::testing::Message() << "a=" << a << " b=" << b << " flip=" << flip);
                const auto count = static_cast<int32_t>(a);
                const auto offset = static_cast<int32_t>(b);
                // Joint count → 0x607A.
                const int64_t word = flip ? b - a : a + b;
                const auto encoded = encode_position_checked(static_cast<double>(count), pos, offset, flip);
                EXPECT_EQ(Pulse(count).to_hardware(offset, flip), clamp(word));
                ASSERT_EQ(encoded.has_value(), word == clamp(word));
                if (encoded) {
                    EXPECT_EQ(encoded->word, word);
                    EXPECT_EQ(encoded->si, static_cast<double>(count));
                }
                // 0x6064 word (a) → joint count.
                const int64_t joint = flip ? -(a - b) : a - b;
                const auto decoded = decode_position_word(count, pos, offset, flip);
                EXPECT_EQ(Pulse::from_hardware(count, offset, flip).value(), clamp(joint));
                ASSERT_EQ(decoded.has_value(), joint == clamp(joint));
                if (decoded) {
                    EXPECT_EQ(*decoded, static_cast<double>(joint));
                }
            }
            // Velocity: flip only, no offset.
            const auto count = static_cast<int32_t>(a);
            const int64_t word = flip ? -a : a;
            const auto encoded = encode_velocity_checked(static_cast<double>(count), vel, flip);
            EXPECT_EQ(PulsePerSecond(count).to_hardware(flip), clamp(word));
            EXPECT_EQ(PulsePerSecond::from_hardware(count, flip).value(), clamp(word));
            ASSERT_EQ(encoded.has_value(), word == clamp(word));
            if (encoded) {
                EXPECT_EQ(encoded->word, word);
                EXPECT_EQ(encoded->si, static_cast<double>(count));
                // DeviceServo::set_target_velocity_word stores the joint count
                // whose encode is the checked word: that round trip must return
                // the word itself.
                EXPECT_EQ(PulsePerSecond::from_hardware(encoded->word, flip).to_hardware(flip), encoded->word);
            }
        }
    }
}

// The endless encode folds the same sign convention (no offset) into the
// wrapped word.
TEST(EndlessPosition, EncodeUsesTheBoundedSignConventionModulo2To32) {
    const PositionChannel pos{MotionUnit::ROTARY_MOTOR_ENCODER, 1, 2.0 * M_PI};
    for (const bool flip : {false, true}) {
        for (const int64_t count : {int64_t{-5}, int64_t{0}, int64_t{7}, (int64_t{1} << 31) + 3, -(int64_t{1} << 40)}) {
            SCOPED_TRACE(::testing::Message() << "count=" << count << " flip=" << flip);
            const auto encoded = encode_endless_position_checked(static_cast<double>(count), pos, flip);
            ASSERT_TRUE(encoded);
            const int64_t hardware = flip ? -count : count;
            EXPECT_EQ(encoded->word, static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint64_t>(hardware))));
            EXPECT_EQ(encoded->si, static_cast<double>(count));
        }
    }
}
