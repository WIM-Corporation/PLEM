// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include "plem/core/crypto/sha256.hpp"

#include <array>

#include <sodium.h>

namespace plem::core::crypto {

std::string sha256_hex(std::string_view data) {
    std::array<unsigned char, crypto_hash_sha256_BYTES> hash{};
    crypto_hash_sha256(hash.data(), reinterpret_cast<const unsigned char*>(data.data()), data.size());

    std::array<char, crypto_hash_sha256_BYTES * 2 + 1> hex{};  // lowercase digits and a NUL
    sodium_bin2hex(hex.data(), hex.size(), hash.data(), hash.size());
    return {hex.data(), hex.size() - 1};
}

}  // namespace plem::core::crypto
