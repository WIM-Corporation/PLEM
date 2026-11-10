#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# rt-initrd: put the RT kernel's modules into /boot/initrd, so a device whose rootfs is on NVMe can boot
# the RT kernel at all.
#
# Both extlinux labels (primary, real-time) load the same /boot/initrd. That initrd carries the storage
# modules and insmods them by absolute path, /lib/modules/<version>/..., so it only works for the kernel
# versions whose modules it contains. L4T builds it with /usr/sbin/nv-update-initrd, which loops over
# /boot/Image and /boot/Image.real-time — but on L4T 36.4.3 that loop deletes the shared file list after
# the first image (`rm -f "${temp_list}"` inside the loop), so the second kernel contributes nothing and
# the initrd ends up with the stock modules only.
#
# On eMMC that goes unnoticed (the MMC driver is built into the kernel). With the rootfs on an external
# NVMe the RT kernel cannot load nvme.ko, never finds the root partition, and the boot stops at
#   ERROR: mounting PARTUUID=<uuid> as /mnt fail...
# dropping into the initrd shell with no network. Three such boots and UEFI marks the slot unbootable
# (RootfsStatusSlotA=0xFF, RootfsRetryCountMax=3) and every later boot goes to the recovery initrd, which
# looks like a dead device.
#
# This tool runs a patched copy of nv-update-initrd from a temp file — the file NVIDIA ships is not
# touched, so a fixed version from a later BSP keeps working (the patch is skipped when the buggy line is
# gone). Modules for every installed kernel end up in one initrd, which both labels can use.
#
# Usage: rt-initrd.sh [--check]   (--check writes nothing and does not need root)
# The last stdout line is the state: applied | ok | n/a | failed (apply), ok | warn | n/a (--check);
# a usage error is failed. Exit codes: 0 = applied, ok or n/a, 1 = failed or warn, 2 = usage error or not root.
# RT_INITRD_ROOT (default empty) is put in front of every file path, for tests.
set -uo pipefail

R="${RT_INITRD_ROOT:-}"
INITRD="${R}/boot/initrd"
TOOL="${R}/usr/sbin/nv-update-initrd"
MODULES_DIR="${R}/lib/modules"

finish() { echo "$2"; exit "$1"; }  # finish <code> <state line>

# Newest installed RT kernel version, empty when the RT kernel is not installed.
rt_version() {
    local d v=()
    for d in "${MODULES_DIR}"/*-rt-tegra; do [[ -d "$d" ]] && v+=("$(basename "$d")"); done
    (( ${#v[@]} )) || return 0
    printf '%s\n' "${v[@]}" | sort -V | tail -1
}

# True when the initrd already carries that version's modules. The entries are written by `find . | cpio`
# and /lib is a symlink to usr/lib in the BSP rootfs, so they read ./lib/modules/..., lib/modules/... or
# usr/lib/modules/... depending on the build — match the tail, not a fixed prefix.
# grep must read the whole listing: with `set -o pipefail` an early exit (-q, -m1) kills gzip with SIGPIPE
# and the pipeline reports failure even though the modules are there. A 16 MB initrd hits this every time.
initrd_has() {
    local ver="$1" n
    n=$(gzip -dc "$INITRD" 2>/dev/null | cpio -t --quiet 2>/dev/null \
        | grep -cE "(^|/)lib/modules/${ver}(/|\$)") || true
    [[ "${n:-0}" -gt 0 ]]
}

case "$#:${1:-}" in
    0:) CHECK=false ;;
    1:--check) CHECK=true ;;
    *) finish 2 "failed: usage: rt-initrd.sh [--check]" ;;
esac

RT_VER="$(rt_version)"
[[ -n "$RT_VER" ]] || finish 0 "n/a: no RT kernel installed"
[[ -f "$INITRD" ]] || finish 1 "failed: ${INITRD} not found"

if $CHECK; then
    initrd_has "$RT_VER" || finish 1 "warn: /boot/initrd has no ${RT_VER} modules — the RT kernel cannot mount an NVMe rootfs"
    finish 0 "ok: /boot/initrd carries ${RT_VER} modules"
fi

initrd_has "$RT_VER" && finish 0 "ok: /boot/initrd already carries ${RT_VER} modules"

# RT_INITRD_ROOT points at a test tree, which needs no root.
[[ $EUID -eq 0 || -n "$R" ]] || finish 2 "failed: root is required"
[[ -x "$TOOL" ]] || finish 1 "failed: ${TOOL} not found — cannot rebuild the initrd"

patched=$(mktemp) || finish 1 "failed: could not create a temp file"
trap 'rm -f "$patched"' EXIT

# Drop the `rm -f "${temp_list}"` that sits directly after the copy call inside the per-image loop, so the
# list survives for the second kernel. Every other line is kept byte for byte.
awk '
    /copy_files_initrd "\$\{DESTDIR\}" "\$\{temp_list\}" "\$\{_kernel_version\}"/ {
        print; if ((getline nxt) > 0) { if (nxt !~ /^[[:space:]]*rm -f "\$\{temp_list\}"[[:space:]]*$/) print nxt }
        next
    }
    { print }
' "$TOOL" > "$patched" || finish 1 "failed: could not patch ${TOOL}"
chmod 755 "$patched"

if ! out=$("$patched" 2>&1); then
    echo "$out" >&2
    finish 1 "failed: nv-update-initrd exited non-zero"
fi
echo "$out" >&2

initrd_has "$RT_VER" || finish 1 "failed: /boot/initrd still has no ${RT_VER} modules"
finish 0 "applied: ${RT_VER} modules added to /boot/initrd — reboot to use it"
