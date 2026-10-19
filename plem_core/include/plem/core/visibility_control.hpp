// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// ROS 2 standard visibility control pattern.
// Apply PLEM_CORE_PUBLIC to symbols that must be visible to downstream
// packages (plem_fieldbus, plem_drive, higher layers, and ROS 2 nodes).
// Everything else is hidden: the library builds with -fvisibility=hidden.
//
// Usage:
//   #include <plem/core/visibility_control.hpp>
//
//   class PLEM_CORE_PUBLIC Foo { ... };
//   PLEM_CORE_PUBLIC void bar();
//
// Rationale: hidden-by-default keeps the exported ABI surface to the declared
// public API: nothing of plem's own leaves the .so unless it is marked PLEM_CORE_PUBLIC.
//
// Why C++11 attribute (`[[gnu::visibility]]`) instead of GCC legacy
// (`__attribute__((visibility))`, used elsewhere): a plem_core type may
// put `alignas(N)` between `class`/`struct` and the class-name (Event does);
// the legacy `__attribute__` form is rejected at that grammar position,
// whereas `[[gnu::visibility]]` places cleanly there. Both produce
// identical ELF visibility; the choice is purely about parser grammar.

#if defined(__GNUC__) || defined(__clang__)
#define PLEM_CORE_PUBLIC [[gnu::visibility("default")]]
#else
#define PLEM_CORE_PUBLIC
#endif
