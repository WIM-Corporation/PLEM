// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <plem/core/units.hpp>
#include <string>

using namespace plem::core::units;

class UnitsTest : public ::testing::Test {};

TEST_F(UnitsTest, UnitBaseComparisonOperators) {
    auto val1 = Radian(1.0);
    auto val2 = Radian(2.0);

    EXPECT_TRUE(val1 < val2);
    EXPECT_TRUE(val2 > val1);
    EXPECT_TRUE(val1 <= val2);
    EXPECT_TRUE(val2 >= val1);
    EXPECT_FALSE(val1 == val2);
    EXPECT_TRUE(val1 != val2);

    // NaN compares unequal and unordered, as the raw double does.
    const auto nan = Radian(std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(nan == nan);
    EXPECT_TRUE(nan != nan);
    EXPECT_FALSE(nan < val1);
    EXPECT_FALSE(nan >= val1);

    EXPECT_TRUE(Radian(1.0) == Radian(1.0));
    EXPECT_TRUE(Radian(0.0) == Radian(-0.0)) << "signed zeros compare equal, as the raw double does";

    // Integer units order like their raw value.
    EXPECT_TRUE(Pulse(1) < Pulse(2));
    EXPECT_TRUE(TorqueCount(100) == TorqueCount(100));
    EXPECT_TRUE(PulsePerSecond(-1) <= PulsePerSecond(0));

    EXPECT_EQ(TorqueCount{}.value(), 0) << "a default-constructed unit is zero";
}

// Pulse(double) range/non-finite policy: saturate finite out-of-range to the
// nearest int32 bound; map non-finite (NaN/inf) to 0 (a garbage magnitude has no
// meaningful pulse count, and casting it is UB). Proves the range before the cast.
TEST_F(UnitsTest, PulseFromDoubleSaturatesAndZeroesNonFinite) {
    EXPECT_EQ(Pulse(3e9).value(), std::numeric_limits<int32_t>::max());   // just past INT32_MAX
    EXPECT_EQ(Pulse(-3e9).value(), std::numeric_limits<int32_t>::min());  // just past INT32_MIN
    EXPECT_EQ(Pulse(std::numeric_limits<double>::infinity()).value(), 0);
    EXPECT_EQ(Pulse(-std::numeric_limits<double>::infinity()).value(), 0);
    EXPECT_EQ(Pulse(std::numeric_limits<double>::quiet_NaN()).value(), 0);
    // In-range values are unaffected (round-to-nearest).
    EXPECT_EQ(Pulse(2147483646.4).value(), 2147483646);
}

// Pulse::to_hardware saturates at the int32 register boundary instead of
// wrapping via signed-integer overflow (UB). The hardware sink is EC_WRITE_S32.
// A value outside int32 has no faithful representation at that boundary;
// saturate it.
TEST_F(UnitsTest, PulseToHardwareSaturatesOnOverflow) {
    // value_ + offset overflows past INT32_MAX → saturate, never wrap to INT32_MIN.
    EXPECT_EQ(Pulse(std::numeric_limits<int32_t>::max()).to_hardware(1, false), std::numeric_limits<int32_t>::max());
    // value_ + offset underflows past INT32_MIN → saturate.
    EXPECT_EQ(Pulse(std::numeric_limits<int32_t>::min()).to_hardware(-1, false), std::numeric_limits<int32_t>::min());
    // Negating INT32_MIN is UB in int32; must saturate to INT32_MAX.
    EXPECT_EQ(Pulse(std::numeric_limits<int32_t>::min()).to_hardware(0, true), std::numeric_limits<int32_t>::max());
}

// Pulse::from_hardware likewise widens + saturates (hw_value - offset, and the
// flip negation of INT32_MIN, are both signed-overflow UB unchecked).
TEST_F(UnitsTest, PulseFromHardwareSaturatesOnOverflow) {
    EXPECT_EQ(Pulse::from_hardware(std::numeric_limits<int32_t>::min(), 1, false).value(),
              std::numeric_limits<int32_t>::min());
    EXPECT_EQ(Pulse::from_hardware(std::numeric_limits<int32_t>::max(), -1, false).value(),
              std::numeric_limits<int32_t>::max());
    EXPECT_EQ(Pulse::from_hardware(std::numeric_limits<int32_t>::min(), 0, true).value(),
              std::numeric_limits<int32_t>::max());
}

// PulsePerSecond(double) shares the Pulse policy: saturate finite
// out-of-range to the nearest int32 bound, map non-finite to 0 (range proven
// before the cast; a bare static_cast would be UB).
TEST_F(UnitsTest, PulsePerSecondFromDoubleSaturatesAndZeroesNonFinite) {
    EXPECT_EQ(PulsePerSecond(3e9).value(), std::numeric_limits<int32_t>::max());
    EXPECT_EQ(PulsePerSecond(-3e9).value(), std::numeric_limits<int32_t>::min());
    EXPECT_EQ(PulsePerSecond(std::numeric_limits<double>::quiet_NaN()).value(), 0);
}

// The flip negation of INT32_MIN in PulsePerSecond::to_hardware/from_hardware
// is signed-overflow UB unchecked; widen + saturate exactly like Pulse.
TEST_F(UnitsTest, PulsePerSecondHardwareFlipSaturatesAtInt32Min) {
    EXPECT_EQ(PulsePerSecond(std::numeric_limits<int32_t>::min()).to_hardware(true),
              std::numeric_limits<int32_t>::max());
    EXPECT_EQ(PulsePerSecond::from_hardware(std::numeric_limits<int32_t>::min(), true).value(),
              std::numeric_limits<int32_t>::max());
}

// Negating INT16_MIN wraps back to INT16_MIN through the int16 conversion:
// a full-scale wrong-sign torque on a flipped joint. Saturate instead.
TEST(TorqueCountTest, HardwareFlipSaturatesAtInt16Min) {
    EXPECT_EQ(TorqueCount(std::numeric_limits<int16_t>::min()).to_hardware(true), std::numeric_limits<int16_t>::max());
    EXPECT_EQ(TorqueCount::from_hardware(std::numeric_limits<int16_t>::min(), true).value(),
              std::numeric_limits<int16_t>::max());
}

// saturate_round_to_int16: same double→int policy as the int32 sibling
// (non-finite → 0 safe neutral, finite out-of-range saturates, round to
// nearest). Pinned directly because no unit type owns a double→int16 boundary
// constructor; the fieldbus torque-command converter relies on this contract.
TEST(TorqueCountTest, SaturateRoundToInt16SaturatesAndZeroesNonFinite) {
    EXPECT_EQ(saturate_round_to_int16(1.0e6), std::numeric_limits<int16_t>::max());
    EXPECT_EQ(saturate_round_to_int16(-1.0e6), std::numeric_limits<int16_t>::min());
    EXPECT_EQ(saturate_round_to_int16(std::numeric_limits<double>::quiet_NaN()), 0);
    EXPECT_EQ(saturate_round_to_int16(std::numeric_limits<double>::infinity()), 0);
    EXPECT_EQ(saturate_round_to_int16(10.7), 11);  // round-to-nearest, not truncation
}

// Pulse Unit Tests

TEST_F(UnitsTest, PulseFromDouble) {
    auto p1 = Pulse(123.4);  // Should round to 123
    EXPECT_EQ(p1.value(), 123);

    auto p2 = Pulse(123.5);  // Should round to 124
    EXPECT_EQ(p2.value(), 124);

    auto p3 = Pulse(-123.5);  // Should round to -124
    EXPECT_EQ(p3.value(), -124);
}

TEST_F(UnitsTest, PulseHardwareConversions) {
    // Flip encode is offset - value, the exact inverse of from_hardware's
    // flip decode -(hw - offset).
    Pulse p1(1000);
    EXPECT_EQ(p1.to_hardware(500, false), 1500);  // 1000 + 500
    EXPECT_EQ(p1.to_hardware(500, true), -500);   // 500 - 1000

    Pulse p2(-1000);
    EXPECT_EQ(p2.to_hardware(500, false), -500);  // -1000 + 500
    EXPECT_EQ(p2.to_hardware(500, true), 1500);   // 500 - (-1000)

    auto p3 = Pulse::from_hardware(1500, 500, false);
    EXPECT_EQ(p3.value(), 1000);  // 1500 - 500

    auto p4 = Pulse::from_hardware(-1500, 500, true);
    EXPECT_EQ(p4.value(), 2000);  // 2000 = - (-1500 - 500)

    auto p5 = Pulse::from_hardware(500, 500, true);
    EXPECT_EQ(p5.value(), 0);  // 0 = - (500 - 500)
}

// Round-trip property: to_hardware/from_hardware are exact inverses in both
// compositions for every non-saturating flip × offset × value combination
// (saturation is covered by the dedicated tests). Grids include offsets of
// the magnitude a zero-offset calibration produces.
TEST_F(UnitsTest, PulseToHardwareIsExactInverseOfFromHardware) {
    constexpr std::array<int32_t, 9> kOffsets = {0, 1, -1, 500, -500000, 200000, 1727957, -1727957, 2000000};
    constexpr std::array<int32_t, 9> kValues = {0, 1, -1, 1000, -1000, 863978, -863978, 2000000, -2000000};
    for (bool flip : {false, true}) {
        for (int32_t offset : kOffsets) {
            for (int32_t value : kValues) {
                SCOPED_TRACE("flip=" + std::to_string(flip) + " offset=" + std::to_string(offset) +
                             " value=" + std::to_string(value));
                // Encode → decode returns the original pulse.
                const int32_t wire = Pulse(value).to_hardware(offset, flip);
                EXPECT_EQ(Pulse::from_hardware(wire, offset, flip).value(), value);
                // Decode → encode returns the original wire word.
                const Pulse decoded = Pulse::from_hardware(value, offset, flip);
                EXPECT_EQ(decoded.to_hardware(offset, flip), value);
            }
        }
    }
}

TEST_F(UnitsTest, PulsePerSecondHardwareConversions) {
    PulsePerSecond v1(1000);
    EXPECT_EQ(v1.to_hardware(false), 1000);
    EXPECT_EQ(v1.to_hardware(true), -1000);

    PulsePerSecond v2(-500);
    EXPECT_EQ(v2.to_hardware(false), -500);
    EXPECT_EQ(v2.to_hardware(true), 500);

    auto v3 = PulsePerSecond::from_hardware(1000, false);
    EXPECT_EQ(v3.value(), 1000);

    auto v4 = PulsePerSecond::from_hardware(1000, true);
    EXPECT_EQ(v4.value(), -1000);

    auto v5 = PulsePerSecond::from_hardware(-500, true);
    EXPECT_EQ(v5.value(), 500);
}

TEST(TorqueCountTest, HardwareConversions) {
    TorqueCount t1(200);
    EXPECT_EQ(t1.to_hardware(false), 200);

    EXPECT_EQ(t1.to_hardware(true), -200);

    auto t2 = TorqueCount::from_hardware(150, false);
    EXPECT_EQ(t2.value(), 150);

    auto t3 = TorqueCount::from_hardware(150, true);
    EXPECT_EQ(t3.value(), -150);

    auto t4 = TorqueCount::from_hardware(-100, true);
    EXPECT_EQ(t4.value(), 100);
}
