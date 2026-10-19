// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <plem/core/crypto/sha256.hpp>

#include <gtest/gtest.h>

namespace plem::core::crypto {

// NIST FIPS 180-2 known-answer vectors; they pin the digest itself.
TEST(Sha256Hex, MatchesNistVectorForAbc) {
    EXPECT_EQ(sha256_hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(Sha256Hex, MatchesNistVectorForEmptyInput) {
    EXPECT_EQ(sha256_hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

// A compact JSON int array (no spaces): the bytes
// hashlib.sha256(json.dumps(values, separators=(",", ":")).encode()) digests.
// Synthetic values; the digest was computed with Python's hashlib.
TEST(Sha256Hex, MatchesACompactJsonIntArrayDigestedInPython) {
    EXPECT_EQ(sha256_hex("[1000,-2000,3000,-4000,5000,-6000]"),
              "2b886f6671e2a688ff421a6051a89d1307108d3a514cd13e1c6c730cc59a9457");
}

}  // namespace plem::core::crypto
