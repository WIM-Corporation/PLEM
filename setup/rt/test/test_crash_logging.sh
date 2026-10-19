#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# setup/rt/crash-logging.sh (journald persistence + kernel.panic reboot). Hermetic: every path sits under
# CRASH_LOGGING_ROOT (a temp dir) and systemctl/journalctl/sysctl/df are stubs on PATH, so nothing on this
# machine is read or changed.
#   ① apply: writes both files with the exact content, restarts+flushes journald, applies the sysctl file;
#      a second run reports ok without a restart; a missing on-disk journal is redone; failures exit 1.
#   ② --check: states and reasons, no writes, no restart.
# The wizard side (p1_s12 / Phase 2 mapping) is in test_setup.sh.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOL="${HERE}/crash-logging.sh"
fail() { echo "FAIL: $*" >&2; exit 1; }
[[ -x "$TOOL" ]] || fail "setup/rt/crash-logging.sh missing or not executable"

tmp="$(cd "$(mktemp -d)" && pwd -P)"   # canonical: the tool compares readlink -f results under this root
trap 'rm -rf "$tmp"' EXIT
R="${tmp}/root" BIN="${tmp}/bin" LOG="${tmp}/calls.log"
mkdir -p "$BIN"
# Stubs log every call. journalctl --flush creates the journal directory like journald does with
# Storage=persistent (unless NOFLUSH is set). sysctl -n reads the live value from KP.
cat >"${BIN}/systemctl" <<EOF
#!/bin/bash
echo "systemctl \$*" >>"$LOG"; [[ -z "\${FAIL_RESTART:-}" ]]
EOF
cat >"${BIN}/journalctl" <<EOF
#!/bin/bash
echo "journalctl \$*" >>"$LOG"
[[ -n "\${NOFLUSH:-}" ]] || mkdir -p "${R}/var/log/journal/\$(cat "${R}/etc/machine-id")"
EOF
cat >"${BIN}/sysctl" <<EOF
#!/bin/bash
echo "sysctl \$*" >>"$LOG"
[[ "\$1" == -n ]] && cat "${tmp}/kp"
[[ "\$1" == -p ]] && { [[ -z "\${FAIL_SYSCTL:-}" ]] || exit 1; echo "kernel.panic = 10"; }
exit 0
EOF
cat >"${BIN}/df" <<EOF
#!/bin/bash
echo "df \$*" >>"$LOG"; [[ -z "\${FAIL_DF:-}" ]] || exit 1; printf '1B-blocks\n%s\n' "\${DF_BYTES:-68719476736}"
EOF
# CHROOT set: systemd-detect-virt --chroot reports a chroot.
cat >"${BIN}/systemd-detect-virt" <<EOF
#!/bin/bash
[[ -n "\${CHROOT:-}" ]]
EOF
chmod 0755 "$BIN"/*

reset_root() { rm -rf "$R" "$LOG"; mkdir -p "${R}/etc/sysctl.d" "${R}/run/systemd/system"; echo 0123abcd >"${R}/etc/machine-id"; echo 0 >"${tmp}/kp"; }
run() {  # run [args] → OUT (stdout), RC; the tool runs with the stubs and the temp root
    set +e; OUT="$(PATH="${BIN}:${PATH}" CRASH_LOGGING_ROOT="$R" "$TOOL" "$@" 2>"${tmp}/err")"; RC=$?; set -e
}
DROPIN="${R}/etc/systemd/journald.conf.d/plem-persistent.conf"
PANIC="${R}/etc/sysctl.d/90-plem-panic.conf"

# ── ① apply ─────────────────────────────────────────────────────────────────
reset_root
run
[[ $RC -eq 0 && "$OUT" == "applied: persistent logs (cap 1310 MiB), reboot 10 s after a panic" ]] \
    || fail "fresh apply: rc=${RC} out=[${OUT}] err=$(cat "${tmp}/err")"
# 64 GiB root → 2 % = 1310 MiB.
[[ "$(cat "$DROPIN")" == $'[Journal]\nStorage=persistent\nSystemMaxUse=1310M' ]] || fail "journald drop-in content: $(cat "$DROPIN")"
[[ "$(cat "$PANIC")" == "kernel.panic = 10" ]] || fail "sysctl file content: $(cat "$PANIC")"
[[ "$(stat -c %a "$DROPIN" "$PANIC")" == $'644\n644' ]] || fail "file modes: $(stat -c %a "$DROPIN" "$PANIC")"
[[ "$(grep -v '^df ' "$LOG")" == "systemctl restart systemd-journald"$'\n'"journalctl --flush"$'\n'"sysctl -p ${PANIC}" ]] \
    || fail "fresh apply commands: $(cat "$LOG")"

# Second run: nothing to do, no restart.
rm -f "$LOG"; run
[[ $RC -eq 0 && "$OUT" == "ok: already set" ]] || fail "second run: rc=${RC} out=[${OUT}]"
grep -q '^systemctl' "$LOG" && fail "second run restarted journald: $(cat "$LOG")"

# Journal on disk but the drop-in content changed (smaller root → 655M): restart to load it.
rm -f "$LOG"; DF_BYTES=$((32 * 1024 ** 3)) run
[[ $RC -eq 0 && "$OUT" == "applied: persistent logs (cap 655 MiB), reboot 10 s after a panic" ]] \
    && grep -qx 'SystemMaxUse=655M' "$DROPIN" && grep -qx 'systemctl restart systemd-journald' "$LOG" \
    || fail "changed drop-in must be rewritten and loaded: rc=${RC} out=[${OUT}] $(cat "$LOG")"

# Same files but the journal is not on disk: restart + flush again.
rm -rf "${R}/var/log/journal" "$LOG"; DF_BYTES=$((32 * 1024 ** 3)) run   # same size as above: files unchanged
[[ $RC -eq 0 && "$OUT" == applied:* ]] && grep -qx 'systemctl restart systemd-journald' "$LOG" && grep -qx 'journalctl --flush' "$LOG" \
    || fail "journal not on disk must restart+flush: rc=${RC} out=[${OUT}] $(cat "$LOG")"

# Cap: at most 2 GiB.
reset_root; DF_BYTES=$((200 * 1024 ** 3)) run
grep -qx 'SystemMaxUse=2048M' "$DROPIN" || fail "cap must be 2048M on a 200 GiB root: $(cat "$DROPIN")"

# kernel.panic set to another value in another file → failed (that file can win at boot). A file with the
# same value is fine, and the 99-sysctl.conf symlink to sysctl.conf does not count twice.
reset_root
echo 'kernel.panic = 10' >"${R}/etc/sysctl.d/50-same.conf"
echo 'kernel.panic=0' >"${R}/etc/sysctl.conf"
ln -s ../sysctl.conf "${R}/etc/sysctl.d/99-sysctl.conf"
run
[[ $RC -eq 1 && "$OUT" == "failed: kernel.panic is also set in ${R}/etc/sysctl.conf (delete that line and re-run)" ]] \
    || fail "kernel.panic elsewhere: rc=${RC} out=[${OUT}]"

# Not a running system (no systemd booted, or a chroot): files written, nothing run on the live system.
for mode in no-systemd chroot; do
    reset_root
    if [[ $mode == no-systemd ]]; then rm -rf "${R}/run/systemd"; run; else CHROOT=1 run; fi
    [[ $RC -eq 0 && "$OUT" == "applied: files written; they take effect at the next boot (not a running system)" ]] \
        && grep -qx 'SystemMaxUse=1310M' "$DROPIN" && [[ "$(cat "$PANIC")" == "kernel.panic = 10" ]] \
        && [[ -z "$(grep -v '^df ' "$LOG")" ]] \
        || fail "not running (${mode}): rc=${RC} out=[${OUT}] $(cat "$LOG")"
done

# Root filesystem size unreadable → failed, nothing written.
reset_root; FAIL_DF=1 run
[[ $RC -eq 1 && "$OUT" == "failed: cannot read the root filesystem size" && ! -e "$DROPIN" ]] || fail "df failure: rc=${RC} out=[${OUT}]"
# A file cannot be written → failed.
reset_root; mkdir -p "${R}/etc/systemd"; : >"${R}/etc/systemd/journald.conf.d"
run
[[ $RC -eq 1 && "$OUT" == "failed: writing ${DROPIN} failed" ]] || fail "write failure: rc=${RC} out=[${OUT}]"
# sysctl -p fails → failed.
reset_root; FAIL_SYSCTL=1 run
[[ $RC -eq 1 && "$OUT" == "failed: applying kernel.panic failed" ]] || fail "sysctl -p failure: rc=${RC} out=[${OUT}]"

# journald restart fails → failed.
reset_root; FAIL_RESTART=1 run
[[ $RC -eq 1 && "$OUT" == "failed: journald restart/flush failed" ]] || fail "restart failure: rc=${RC} out=[${OUT}]"
# Flush leaves logs in RAM → failed.
reset_root; NOFLUSH=1 run
[[ $RC -eq 1 && "$OUT" == "failed: logs are not stored on disk (${R}/var/log/journal/0123abcd missing)" ]] \
    || fail "logs not on disk: rc=${RC} out=[${OUT}]"

# Usage.
for args in "--bogus" "--check --check"; do
    # shellcheck disable=SC2086  # split on purpose
    run $args
    [[ $RC -eq 2 && "$OUT" == "failed: usage: crash-logging.sh [--check]" ]] || fail "usage '${args}': rc=${RC} out=[${OUT}]"
done

# Not root, real paths: refused before anything runs. Only safe to try as non-root (as root it would write /etc).
if [[ $EUID -ne 0 ]]; then
    rm -f "$LOG"
    set +e; OUT="$(PATH="${BIN}:${PATH}" "$TOOL" 2>/dev/null)"; RC=$?; set -e
    [[ $RC -eq 2 && "$OUT" == "failed: root is required" && ! -e "$LOG" ]] || fail "non-root apply: rc=${RC} out=[${OUT}]"
else
    echo "NOTE: running as root, the non-root refusal case is skipped"
fi

# ── ② --check ───────────────────────────────────────────────────────────────
check() {  # check <rc> <state line>
    local before; before="$(find "$R" -printf '%p %s %T@\n' | sort)"; rm -f "$LOG"
    run --check
    [[ $RC -eq $1 && "$OUT" == "$2" ]] || fail "--check: want rc=$1 [$2], got rc=${RC} [${OUT}]"
    [[ "$(find "$R" -printf '%p %s %T@\n' | sort)" == "$before" ]] || fail "--check changed files"
    ! grep -qv '^sysctl -n kernel.panic$' "$LOG" 2>/dev/null || fail "--check ran more than sysctl -n: $(cat "$LOG")"
}
reset_root
check 1 "warn: logs kept in RAM only"
mkdir -p "${R}/var/log/journal/0123abcd"
check 1 "warn: kernel.panic=0"
echo 'kernel.panic = 0' >"${R}/etc/sysctl.d/10-vendor.conf"
check 1 "warn: kernel.panic=0, also set in ${R}/etc/sysctl.d/10-vendor.conf (delete that line)"
echo 10 >"${tmp}/kp"
check 0 "ok: logs persistent, reboot 10 s after a panic"

echo "PASS: crash logging — apply/ok/failed states, exact files, --check without writes"
