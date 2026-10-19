// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

// The C++ statusword decoder as the reference for plem.drive.cia402 (test only):
// one line per 16-bit statusword, "<word> <state name>", word 0 to 0xFFFF.
#include <cstdint>
#include <cstdio>

#include <plem/fieldbus/cia402_profile.hpp>
#include <plem/fieldbus/ethercat/state_machine.hpp>

int main() {
    for (uint32_t word = 0; word <= 0xFFFF; ++word) {
        const auto state = plem::fieldbus::ethercat::to_servo_state(static_cast<uint16_t>(word));
        std::printf("%u %s\n", word, plem::fieldbus::cia402_state_name(state));
    }
    return 0;
}
