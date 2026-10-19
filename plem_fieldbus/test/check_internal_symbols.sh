#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

# Package-private entities (src/ethercat/*.hpp, marked PLEM_FIELDBUS_INTERNAL)
# must not reach the shared library's export table. No behavior test can see
# this: it is an ELF property of the linked library, so the symbol table is the
# oracle.
set -euo pipefail

so="${1:?usage: $0 <path-to-libplem_fieldbus.so>}"
if [[ ! -r "$so" ]]; then
    echo "FAIL: cannot read $so"
    exit 1
fi

# Substrings matched against nm's mangled output, where each identifier appears
# verbatim. This is an ABI-token oracle, not a parse of the headers: a new
# entity whose name contains none of these goes unchecked.
forbidden=(JointDataPlane RtCycleMonitor RtEventEmitter ServoStatus run_brake_cycle_step decide_sequence_step commit_fault_reset axes_of_slave first_axis_index_of_slave)

# The subset that must also EXIST in the full symbol table. Without this the
# export check passes vacuously for anything the optimizer inlined away.
# `JointDataPlane` and `RtCycleMonitor` are deliberately absent
# from this list: JointDataPlane's
# members can all be inlined into the bus loop (GCC 11 at RelWithDebInfo emits
# no symbol, so nothing to export), and RtCycleMonitor is covered transitively
# (see below).
#
# Dropping the marker on run_brake_cycle_step (a non-inline template) turns
# this red. Dropping it from inline code (the in-class members of JointDataPlane
# and RtEventEmitter) does not: the target's VISIBILITY_INLINES_HIDDEN already
# hides inline functions (not the members of an explicitly instantiated
# template), so there the check pins the contract, not the marker.
# The same holds for RtCycleMonitor — emit() takes a `const JointDataPlane<DOF>&`,
# so GCC keeps it hidden whether or not it carries the marker itself.
anchors=(run_brake_cycle_step)

exported=$(nm -D --defined-only "$so")
defined=$(nm --defined-only "$so")

rc=0
for n in "${forbidden[@]}"; do
    if grep -q -- "$n" <<<"$exported"; then
        echo "FAIL: $n is exported from $(basename "$so")."
        echo "      Its declaration in src/ethercat/ lost PLEM_FIELDBUS_INTERNAL, or a new"
        echo "      code-emitting entity reached that header without the marker."
        grep -- "$n" <<<"$exported" | sed 's/^/        /'
        rc=1
    fi
done

for n in "${anchors[@]}"; do
    if ! grep -q -- "$n" <<<"$defined"; then
        echo "FAIL: $n is absent from $(basename "$so") altogether, so passing the export"
        echo "      check proves nothing about it. Either the entity was renamed or removed"
        echo "      (drop it from the anchor list), or every call was inlined away (pick a"
        echo "      name that still emits)."
        rc=1
    fi
done

if [[ $rc -eq 0 ]]; then
    echo "OK: ${#forbidden[@]} package-private tokens unexported, ${#anchors[@]} anchors present in $(basename "$so")."
fi
exit "$rc"
