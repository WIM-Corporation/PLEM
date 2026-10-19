// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

/// @file test_tri_state.cpp
/// @brief TriState compiles next to the TRUE/FALSE macros glib and
/// <rpc/types.h> define; a camera pipeline includes both in one TU.

#define TRUE 1
#define FALSE 0

#include <gtest/gtest.h>

#include <plem/core/types/tri_state.hpp>

namespace plem::core {
namespace {

TEST(TriState, CompilesBesideTrueFalseMacrosAndKeepsItsWireValues) {
    static_assert(static_cast<int8_t>(TriState::NO) == 0);
    static_assert(static_cast<int8_t>(TriState::YES) == 1);
    static_assert(static_cast<int8_t>(TriState::UNDEFINED) == -1);
    EXPECT_NE(TriState::YES, TriState::NO);
}

}  // namespace
}  // namespace plem::core
