#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# crash-logging: keep the system log across reboots and reboot after a kernel panic, so you can later see
# why a device in the field froze or rebooted.
#
# journald: the default (Storage=auto with no /var/log/journal) keeps logs only in RAM, lost on reboot.
# kernel.panic: the default 0 hangs after a kernel panic and waits for the hardware watchdog (120 s). With 10
#   it reboots itself after 10 seconds (whether the application resumes after reboot is up to its own startup policy).
# pstore (systemd-pstore) is on by default in the BSP, so it is left alone.
#
# Usage: crash-logging.sh [--check]   (--check writes nothing and does not need root)
# The last stdout line is the state: applied | ok | failed (apply), ok | warn (--check); a usage error is failed.
# Exit codes: 0 = applied or ok, 1 = failed or warn, 2 = usage error or not root.
# Outside a running system (chroot, container without systemd) it only writes the files; they apply at boot.
# CRASH_LOGGING_ROOT (default empty) is put in front of every file path, for tests. Commands (systemctl,
# journalctl, sysctl) still act on the running system.
set -uo pipefail

R="${CRASH_LOGGING_ROOT:-}"
DROPIN="${R}/etc/systemd/journald.conf.d/plem-persistent.conf"
PANIC_CONF="${R}/etc/sysctl.d/90-plem-panic.conf"
PANIC_SEC=10

finish() { echo "$2"; exit "$1"; }  # finish <code> <state line>

journal_on_disk() { [[ -d "${R}/var/log/journal/$(cat "${R}/etc/machine-id" 2>/dev/null)" ]]; }

# sysctl files other than PANIC_CONF that set kernel.panic to a different value (space separated).
# Symlinks to /etc/sysctl.conf (99-sysctl.conf) duplicate the original, so they are skipped.
panic_set_elsewhere() {
    local f v
    for f in "${R}/etc/sysctl.conf" "${R}"/etc/sysctl.d/*.conf "${R}"/usr/lib/sysctl.d/*.conf "${R}"/run/sysctl.d/*.conf; do
        [[ -f "$f" && "$f" != "$PANIC_CONF" ]] || continue
        [[ -L "$f" && "$(readlink -f "$f")" == "${R}/etc/sysctl.conf" ]] && continue
        v=$(sed -n 's/^[[:space:]]*kernel\.panic[[:space:]]*=[[:space:]]*\([^[:space:]]*\).*/\1/p' "$f" | tail -1)
        [[ -n "$v" && "$v" != "$PANIC_SEC" ]] && printf '%s ' "$f"
    done
}

case "$#:${1:-}" in
    0:) CHECK=false ;;
    1:--check) CHECK=true ;;
    *) finish 2 "failed: usage: crash-logging.sh [--check]" ;;
esac

if $CHECK; then
    journal_on_disk || finish 1 "warn: logs kept in RAM only"
    kp=$(sysctl -n kernel.panic 2>/dev/null || echo "?")
    if [[ "$kp" != "$PANIC_SEC" ]]; then
        ov=$(panic_set_elsewhere)
        [[ -n "$ov" ]] && finish 1 "warn: kernel.panic=${kp}, also set in ${ov% } (delete that line)"
        finish 1 "warn: kernel.panic=${kp}"
    fi
    finish 0 "ok: logs persistent, reboot ${PANIC_SEC} s after a panic"
fi

# CRASH_LOGGING_ROOT points at a test tree, which needs no root.
[[ $EUID -eq 0 || -n "$R" ]] || finish 2 "failed: root is required"
# Log cap: 2 % of the root filesystem, at most 2 GiB (some roots are on eMMC, so no fixed value).
size=$(df -B1 --output=size "${R}/" | tail -1)
[[ $size =~ ^[0-9]+$ ]] || finish 1 "failed: cannot read the root filesystem size"
size_mib=$(( size / 1024 / 1024 ))
cap=$(( size_mib * 2 / 100 ))
(( cap > 2048 )) && cap=2048
# A running system: booted with systemd and not in a chroot. Otherwise only the files are written.
live=false
[[ -d "${R}/run/systemd/system" ]] && ! systemd-detect-virt --quiet --chroot 2>/dev/null && live=true

changed=false tmp=$(mktemp)
trap 'rm -f "$tmp"' EXIT
write() {  # write <file>: install tmp as <file> if the content differs
    cmp -s "$tmp" "$1" && return
    if ! { mkdir -p "$(dirname "$1")" && install -m 0644 "$tmp" "$1"; }; then
        finish 1 "failed: writing $1 failed"
    fi
    changed=true
}
printf '[Journal]\nStorage=persistent\nSystemMaxUse=%sM\n' "$cap" > "$tmp"
write "$DROPIN"
# A restart is needed to load the settings, and a flush request to move logs to disk (journald does not open
# /var/log/journal before a flush request). Redo it if the file is the same but logs are not stored on disk.
if $live && { $changed || ! journal_on_disk; }; then
    if ! { systemctl restart systemd-journald && journalctl --flush; }; then
        finish 1 "failed: journald restart/flush failed"
    fi
    changed=true
fi
printf 'kernel.panic = %s\n' "$PANIC_SEC" > "$tmp"
write "$PANIC_CONF"
if $live && ! sysctl -p "$PANIC_CONF" > /dev/null; then
    finish 1 "failed: applying kernel.panic failed"
fi
# If another sysctl file sets kernel.panic, that file can win at boot.
others=$(panic_set_elsewhere)
[[ -z "$others" ]] || finish 1 "failed: kernel.panic is also set in ${others% } (delete that line and re-run)"
if $live && ! journal_on_disk; then
    finish 1 "failed: logs are not stored on disk (${R}/var/log/journal/$(cat "${R}/etc/machine-id" 2>/dev/null) missing)"
fi
$changed || finish 0 "ok: already set"
$live || finish 0 "applied: files written; they take effect at the next boot (not a running system)"
finish 0 "applied: persistent logs (cap ${cap} MiB), reboot ${PANIC_SEC} s after a panic"
