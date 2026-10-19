#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# The NIC rebind helper that setup/rt/setup.sh (menu 3) installs as ethercat.service ExecStartPost: cut it
# from the wizard's heredoc and RUN it hermetically against a fake /etc/ethercat.conf (ETHERCAT_CONF) with lsmod and
# modprobe shadowed on PATH — it never reaches sysfs or loads a module here.
# Pins the paths that decide before any PCI write: generic and empty
# DEVICE_MODULES are a no-op; a missing conf, a module list, no MASTER0_DEVICE (masters are read from 0 up to the
# first missing number, as ethercatctl does), a syntax error and an unloadable ec_<drv> fail; quoting and the conf's
# last-line status do not change what is read.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WIZ="${HERE}/setup.sh"

fail() { echo "FAIL: $*" >&2; exit 1; }

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
HELPER="${tmp}/native_bind.sh"
sed -n "/<< 'EOFNB'\$/,/^EOFNB\$/p" "$WIZ" | sed '1d;$d' >"$HELPER"
[[ -s "$HELPER" ]] || fail "rebind helper heredoc (EOFNB) not found in setup/rt/setup.sh"
mkdir -p "${tmp}/bin"
printf '%s\n' '#!/usr/bin/env bash' 'echo "Module Size Used by"' >"${tmp}/bin/lsmod"
printf '%s\n' '#!/usr/bin/env bash' 'echo "modprobe $*" >>"'"${tmp}"'/modprobe.log"' 'exit 1' >"${tmp}/bin/modprobe"
chmod 0755 "${tmp}/bin/lsmod" "${tmp}/bin/modprobe"

run() {  # run <conf path> → sets out, rc
    set +e
    out="$(env -i HOME="$HOME" PATH="${tmp}/bin:/usr/bin:/bin" ETHERCAT_CONF="$1" bash "$HELPER" 2>&1)"
    rc=$?
    set -e
}

# (a) generic: nothing to rebind, exit 0, no module touched
printf '%s\n' 'MASTER0_DEVICE="00:00:00:00:00:00"' 'DEVICE_MODULES="generic"' >"${tmp}/generic.conf"
run "${tmp}/generic.conf"
[[ "$rc" -eq 0 && "$out" == *"native rebind not required"* ]] || fail "(a) generic: ${rc} ${out}"

# (b) no DEVICE_MODULES line: same no-op
printf '%s\n' 'MASTER0_DEVICE=""' >"${tmp}/empty.conf"
run "${tmp}/empty.conf"
[[ "$rc" -eq 0 && "$out" == *"DEVICE_MODULES=empty"* ]] || fail "(b) empty: ${rc} ${out}"
[[ ! -e "${tmp}/modprobe.log" ]] || fail "(a)/(b) touched modprobe: $(cat "${tmp}/modprobe.log")"

# (c) a missing conf fails and says which file
run "${tmp}/absent.conf"
[[ "$rc" -eq 1 && "$out" == *"${tmp}/absent.conf missing"* ]] || fail "(c) missing conf: ${rc} ${out}"

# (d) a native driver whose ec_ module cannot be loaded fails before any sysfs write
printf '%s\n' 'MASTER0_DEVICE="00:00:00:00:00:00"' 'DEVICE_MODULES="igb"' >"${tmp}/igb.conf"
run "${tmp}/igb.conf"
[[ "$rc" -eq 1 && "$out" == *"failed to modprobe ec_igb"*"--enable-igb"* ]] || fail "(d) modprobe failure: ${rc} ${out}"
grep -qxF "modprobe ec_igb" "${tmp}/modprobe.log" || fail "(d) expected modprobe ec_igb, got: $(cat "${tmp}/modprobe.log")"

# (e) the conf is read the way ethercatctl reads it (sourced as shell): unquoted and
#     single-quoted values count the same as double-quoted ones
printf '%s\n' 'MASTER0_DEVICE=00:00:00:00:00:00' 'DEVICE_MODULES=igb' >"${tmp}/unquoted.conf"
printf '%s\n' "MASTER0_DEVICE='00:00:00:00:00:00'" "DEVICE_MODULES='igb'" >"${tmp}/single.conf"
for c in unquoted single; do
    rm -f "${tmp}/modprobe.log"
    run "${tmp}/${c}.conf"
    [[ "$rc" -eq 1 && "$out" == *"failed to modprobe ec_igb"* ]] || fail "(e) ${c}: ${rc} ${out}"
done

# (f) more than one module is a list ethercatctl accepts but this helper cannot
#     apply: fail before loading anything, and say how to fix it
for mods in "igb igc" "igb generic"; do
    printf '%s\n' 'MASTER0_DEVICE="00:00:00:00:00:00"' "DEVICE_MODULES=\"${mods}\"" >"${tmp}/multi.conf"
    rm -f "${tmp}/modprobe.log"
    run "${tmp}/multi.conf"
    [[ "$rc" -eq 1 && "$out" == *"DEVICE_MODULES=\"${mods}\""*"one native driver"* ]] || fail "(f) ${mods}: ${rc} ${out}"
    [[ ! -e "${tmp}/modprobe.log" ]] || fail "(f) ${mods} touched modprobe: $(cat "${tmp}/modprobe.log")"
done

# (g) a native driver with no MASTER0_DEVICE has nothing to rebind: fail, not a silent pass
printf '%s\n' 'DEVICE_MODULES="igb"' >"${tmp}/nomaster.conf"
rm -f "${tmp}/modprobe.log"
run "${tmp}/nomaster.conf"
[[ "$rc" -eq 1 && "$out" == *"no MASTER0_DEVICE"* ]] || fail "(g) no master: ${rc} ${out}"
[[ ! -e "${tmp}/modprobe.log" ]] || fail "(g) touched modprobe: $(cat "${tmp}/modprobe.log")"
#     a master after a gap is not read either (ethercatctl stops at the first missing number)
printf '%s\n' 'MASTER1_DEVICE="00:00:00:00:00:00"' 'DEVICE_MODULES="igb"' >"${tmp}/gap.conf"
run "${tmp}/gap.conf"
[[ "$rc" -eq 1 && "$out" == *"no MASTER0_DEVICE"* ]] || fail "(g) gap: ${rc} ${out}"
[[ ! -e "${tmp}/modprobe.log" ]] || fail "(g) gap touched modprobe: $(cat "${tmp}/modprobe.log")"

# (h) a conf is judged by its syntax, not by its last command's status: a valid conf
#     whose last line returns non-zero is read as ethercatctl reads it; a syntax error fails
printf '%s\n' 'MASTER0_DEVICE="00:00:00:00:00:00"' 'DEVICE_MODULES="igb"' '[ -n "$UNSET_X" ] && MASTER1_DEVICE=x' >"${tmp}/lastrc.conf"
run "${tmp}/lastrc.conf"
[[ "$rc" -eq 1 && "$out" == *"failed to modprobe ec_igb"* ]] || fail "(h) last line rc: ${rc} ${out}"
printf '%s\n' 'DEVICE_MODULES="igb' >"${tmp}/syntax.conf"
run "${tmp}/syntax.conf"
[[ "$rc" -eq 1 && "$out" == *"does not parse as shell"* ]] || fail "(h) syntax error: ${rc} ${out}"

echo "PASS: rebind helper reads the conf as ethercatctl does, is a no-op for generic, fails on a missing conf, a module list, no master device or an unloadable ec_ driver"
