#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

# This package configures on its own and bakes in the version its package.xml
# declares — the version `plem drive check` prints. The expected
# value is read from package.xml here, independently of the CMake under test.
# usage: check_configure_without_packaging.sh <plem_drive source dir>
set -eo pipefail
src="$1"
[[ -f "${src}/package.xml" ]] || { echo "usage: $0 <plem_drive source dir>" >&2; exit 2; }
want="$(sed -n 's:.*<version>\([^<]*\)</version>.*:\1:p' "${src}/package.xml" 2>/dev/null | head -1)"
[[ -n "${want}" ]] || { echo "usage: $0 <plem_drive source dir> (no <version> in its package.xml)" >&2; exit 2; }
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
cp -r "${src}" "${tmp}/plem_drive"
if ! cmake -S "${tmp}/plem_drive" -B "${tmp}/build" -GNinja -DBUILD_TESTING=OFF >"${tmp}/configure.log" 2>&1; then
    cat "${tmp}/configure.log"
    echo "FAIL: plem_drive does not configure on its own"
    exit 1
fi
if ! grep -qF "PLEM_PLATFORM_VERSION=\\\"${want}\\\"" "${tmp}/build/build.ninja"; then
    grep -o 'PLEM_PLATFORM_VERSION=[^ ]*' "${tmp}/build/build.ninja" | sort -u
    echo "FAIL: the baked-in version is not package.xml's ${want}"
    exit 1
fi
echo "OK: configured on its own, PLEM_PLATFORM_VERSION=${want}"
