// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// Package-private visibility marker (deliberately not an installed header).
//
// Every header under src/ethercat/ opens by declaring itself package-private,
// but that comment changes nothing at link time. When one of their templates or
// inline members does emit a linkable definition, it is emitted weak with
// default visibility, and such a definition is preemptible: an out-of-line
// call to it binds to whichever definition the dynamic loader's lookup scope
// reaches first (another object's copy of a header that has since diverged,
// or an LD_PRELOAD) rather than to the definition compiled beside it.
//
// PLEM_FIELDBUS_INTERNAL gives those definitions hidden visibility. Put it on
// every entity in these headers that can emit a linkable definition, whether or
// not it emits one at today's optimization level: a Debug build emits what
// RelWithDebInfo inlines away. For a class the marker goes on the class. For a
// type that a public header forward-declares and an exported class holds as a
// field, it goes on each member instead: marking the class there makes GCC warn
// (-Wattributes) that an exported class has a field of a hidden type. The type
// itself then stays default, and so does what the compiler instantiates on it;
// no member marker reaches those.
//
// Scalar constants are left unmarked. They emit a definition only when
// odr-used, and these are read by value.
//
// plem_fieldbus does not build with -fvisibility=hidden, so this is a per-name
// ratchet rather than a package-wide policy: an unmarked entity that emits is
// exported unless something else hides it for you, and the build succeeds either
// way. Two things do: a hidden type in its own signature (why RtCycleMonitor::emit
// stays hidden with or without its marker), and the target's
// VISIBILITY_INLINES_HIDDEN, which hides inline functions (in-class members,
// `inline`/constexpr functions, and the std instantiations above) except the
// members of an explicitly instantiated template (EtherCATController<N>)
// and anything with an explicit visibility attribute.
// The marker is therefore load-bearing on what is not inline (a plain function
// template such as run_brake_cycle_step, an out-of-class member definition), on
// those two exceptions, and on class-level entities other than functions; on
// other inline code it states intent the flag already enforces. test/check_internal_symbols.sh
// enforces the ratchet by checking exported ABI tokens (it does not parse
// these headers); its comment lists what it covers.
#define PLEM_FIELDBUS_INTERNAL [[gnu::visibility("hidden")]]
