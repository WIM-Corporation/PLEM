#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

# plem_fieldbus requires the IgH master: on a host without it (ecrt.h +
# libethercat.so) configure must stop with the install instruction — not get as far
# as a build that consumers then fail on at `#include <ecrt.h>`.
# A prefix that carries only the static libethercat.a must stop the same way (the
# library is LGPL — plem links the shared one only), and so must a static library
# handed in through the cache (-DETHERCAT_LIB=…/libethercat.a), which skips the search.
# ecrt.h and libethercat.so from two different prefixes must stop too — nested ones
# included (/usr and /usr/local are two installs). After either refusal, doing what
# the error says and configuring the SAME build directory again must succeed (the
# refusal leaves no found value in the cache). A prefix on CMAKE_PREFIX_PATH that
# carries ecrt.h + libethercat.so is used.
# IgH's absence is simulated by ignoring, in CMake's find_* searches, the two
# directories the calling configure found it in — /opt/etherlab, a system prefix or
# a CMAKE_PREFIX_PATH entry alike. A host with a SECOND IgH install elsewhere cannot
# simulate absence that way: the script says so and skips.
# usage: check_configure_without_igh.sh <plem_fieldbus source dir> <ecrt.h dir> <libethercat.so dir>
set -eo pipefail
src="$1"
igh_include="$2"
igh_lib="$3"
if [[ ! -f "${src}/package.xml" || -z "${igh_include}" || -z "${igh_lib}" ]]; then
    echo "usage: $0 <plem_fieldbus source dir> <ecrt.h dir> <libethercat.so dir>" >&2
    exit 2
fi
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
no_igh=(-GNinja -DBUILD_TESTING=OFF "-DCMAKE_IGNORE_PATH=${igh_include};${igh_lib}")

# A stand-in IgH prefix — configure only checks that the files exist.
fake_prefix() {  # <dir> <file under the prefix>...
    local dir="$1" file
    shift
    for file in "$@"; do
        mkdir -p "$(dirname "${dir}/${file}")"
        touch "${dir}/${file}"
    done
}

expect_refusal() {  # <label> <build dir> <instruction the error must carry> [cmake args...]
    local label="$1" build="$2" instruction="$3" flat want
    shift 3
    if cmake -S "${src}" -B "${build}" "${no_igh[@]}" "$@" >"${build}.log" 2>&1; then
        cat "${build}.log"
        echo "FAIL: plem_fieldbus configured ${label}"
        exit 1
    fi
    # CMake wraps a long message across lines — judge the whitespace-flattened text.
    flat="$(tr -s '[:space:]' ' ' <"${build}.log")"
    if [[ "${flat}" != *"${instruction}"* ]]; then
        cat "${build}.log"
        echo "FAIL: the configure error ${label} does not say '${instruction}'"
        exit 1
    fi
}

# Premise: with the found directories ignored, no IgH is left to find. A configure
# that still succeeds is a second install only if it FOUND a real libethercat.so —
# one that succeeds without it is the regression the first case below reports.
if cmake -S "${src}" -B "${tmp}/on" "${no_igh[@]}" >"${tmp}/on.log" 2>&1; then
    found="$(sed -n 's/^ETHERCAT_LIB:[A-Z]*=//p' "${tmp}/on/CMakeCache.txt")"
    if [[ -f "${found}" ]]; then
        echo "SKIP: CMake still finds IgH (${found}) with ${igh_include} and ${igh_lib} ignored — a second" \
            "install; absence cannot be simulated on this host"
        exit 77  # SKIP_RETURN_CODE (CMakeLists.txt): reported as skipped, never as a pass
    fi
fi
rm -rf "${tmp}/on" "${tmp}/on.log"

expect_refusal "without IgH" "${tmp}/on" "install IgH to /opt/etherlab"

fake_prefix "${tmp}/static" include/ecrt.h lib/libethercat.a
expect_refusal "with only the static libethercat.a" "${tmp}/on-static" "install IgH to /opt/etherlab" \
    "-DCMAKE_PREFIX_PATH=${tmp}/static"

fake_prefix "${tmp}/shared" include/ecrt.h lib/libethercat.a lib/libethercat.so

# expect_recovery <label> <build dir> [cmake args...] — the same build directory
# configures once the operator has done what the refusal said.
expect_recovery() {
    local label="$1" build="$2"
    shift 2
    if ! cmake -S "${src}" -B "${build}" "${no_igh[@]}" "$@" >"${build}.again.log" 2>&1; then
        cat "${build}.again.log"
        echo "FAIL: ${label} — the same build directory still refuses after following the instruction"
        exit 1
    fi
}

fake_prefix "${tmp}/cached" include/ecrt.h lib/libethercat.a
expect_refusal "with a cached static ETHERCAT_LIB" "${tmp}/on-cached" "not the shared libethercat.so" \
    "-DCMAKE_PREFIX_PATH=${tmp}/cached" "-DETHERCAT_LIB=${tmp}/cached/lib/libethercat.a"
expect_recovery "cached static ETHERCAT_LIB, then an IgH prefix" "${tmp}/on-cached" \
    "-DCMAKE_PREFIX_PATH=${tmp}/shared"

fake_prefix "${tmp}/headers" include/ecrt.h
fake_prefix "${tmp}/libs" lib/libethercat.so
expect_refusal "with ecrt.h and libethercat.so from different prefixes" "${tmp}/on-mixed" \
    "come from different IgH prefixes" "-DCMAKE_PREFIX_PATH=${tmp}/headers;${tmp}/libs"
expect_recovery "different prefixes, then the one IgH prefix first" "${tmp}/on-mixed" \
    "-DCMAKE_PREFIX_PATH=${tmp}/shared;${tmp}/headers;${tmp}/libs"

# Nested prefixes: ecrt.h only under <root>/usr, libethercat.so only under <root>/usr/local.
fake_prefix "${tmp}/nested" usr/include/ecrt.h usr/local/lib/libethercat.so
expect_refusal "with ecrt.h and libethercat.so from nested prefixes" "${tmp}/on-nested" \
    "come from different IgH prefixes" "-DCMAKE_PREFIX_PATH=${tmp}/nested/usr/local;${tmp}/nested/usr"

if ! cmake -S "${src}" -B "${tmp}/on-shared" "${no_igh[@]}" "-DCMAKE_PREFIX_PATH=${tmp}/shared" \
    >"${tmp}/on-shared.log" 2>&1; then
    cat "${tmp}/on-shared.log"
    echo "FAIL: an IgH prefix on CMAKE_PREFIX_PATH (ecrt.h + libethercat.so) does not configure"
    exit 1
fi
for want in "${tmp}/shared/lib/libethercat.so" "-I${tmp}/shared/include"; do
    if ! grep -qF -- "${want}" "${tmp}/on-shared/build.ninja"; then
        echo "FAIL: the prefix build does not use ${want}"
        exit 1
    fi
done
echo "OK: without IgH, with only libethercat.a, with a cached .a or with ecrt.h and libethercat.so from" \
    "different (or nested) prefixes configure stops with the instruction, and the same build directory configures" \
    "once it is followed; a prefix's ecrt.h + libethercat.so is used"
