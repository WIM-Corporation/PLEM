#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# setup/rt/setup.sh (Jetson RT setup wizard): the parts that run without root or hardware.
# - options: --help exits 0 before the root check, an unknown option exits 2, --baseline sets the default.
# - "Disable unused services" is opt-in: default no; yes with --baseline, when this device had it
#   applied before (a baseline unit is masked), or when all are already off. Step 13 changes nothing
#   when not selected. Phase 2 warns on drift only for a device that had it applied.
# - Phase 2 version hold: warns on unfinished installs, a missing pin file, an unheld package, or a
#   held package missing from the pin file.
# - Wiring: run_phase1/run_phase2/the entry point call these functions.
# - run_cmd reports the command's real exit code.
# - Step 12 and the Phase 2 crash logging check map crash-logging.sh's exit code and state line.
# - Step 15 / Phase 2 RT permission: realtime group, limits file (installed only when its bytes differ),
#   and the user who ran sudo added; Phase 2 warns when any is missing.
# - Menu 3 step 7 / Phase 2 EtherCAT device: rule 0660 group realtime after the same group/user step;
#   Phase 2 checks the device nodes and the user's membership.
# - booted_isol_cpu: the last CPU isolated at boot; menu 4 and every Phase 2 check use it.
# - Step 6 / CPU selection: the topology per isolated CPU, the selection menu, and the extlinux.conf edit
#   (RT-image labels only, idempotent, re-run with another CPU, one backup).
# - Step 1 counts only an uncommented rt-kernel line with the expected suite as registered.
# - /etc/ethercat.conf: comments come from a quoted heredoc (nothing in them runs), values unchanged.
# - The wizard directory is resolved once and stays right after a cd; Phase 3 leaves and removes
#   the build directory when a step fails.
# - Menu 5 (USB adapter drivers): upstream tag from the headers' Makefile, local includes, kernel source vs an
#   error page, vermagic, the per-driver skip decision, the fetch-build-install flow with its failure reports,
#   and the menu 2 check.
# Functions are sourced from a copy without the entry point, with system commands stubbed.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WIZ="${HERE}/setup.sh"

fail() { echo "FAIL: $*" >&2; exit 1; }
[[ -f "$WIZ" ]] || fail "setup/rt/setup.sh missing"

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# --- options -------------------------------------------------------------------------
set +e
out="$(bash "$WIZ" --help 2>&1)"; rc=$?
set -e
[[ $rc -eq 0 && "$out" == *"--baseline"* ]] || fail "--help: rc=${rc} ${out}"
set +e
out="$(bash "$WIZ" --no-such-option 2>&1)"; rc=$?
set -e
[[ $rc -eq 2 && "$out" == *"Unknown option: --no-such-option"* ]] || fail "unknown option: rc=${rc} ${out}"

# --- functions -------------------------------------------------------------------------
# Everything before the entry point, with the pin file, apt source and ethercat.conf moved into the temp dir.
grep -qx '#  Entry point' "$WIZ" || fail "entry point marker missing (the cut below would source main_menu)"
sed -e '/^#  Entry point$/,$d' \
    -e "s|^readonly RT_FREEZE_PREF=.*|readonly RT_FREEZE_PREF=\"${tmp}/freeze.pref\"|" \
    -e "s|^readonly RT_REPO_FILE=.*|readonly RT_REPO_FILE=\"${tmp}/l4t.list\"|" \
    -e "s|^readonly RT_LIMITS=.*|readonly RT_LIMITS=\"${tmp}/limits.conf\"|" \
    -e "s|^readonly EXTLINUX=.*|readonly EXTLINUX=\"${tmp}/extlinux.conf\"|" \
    -e "s|^readonly ETHERLAB_PREFIX=.*|readonly ETHERLAB_PREFIX=\"${tmp}/etherlab\"|" \
    -e "s|/etc/ld.so.conf.d/etherlab.conf|${tmp}/etherlab.conf|g" \
    -e "/^ec_release_native_nics() {$/,/^}$/ s|/sys/bus/pci|${tmp}/sys/bus/pci|g" \
    -e "s|^CMD_LOG=\$(mktemp /tmp/|CMD_LOG=\$(mktemp ${tmp}/|" \
    -e "s|^readonly EC_UDEV_RULES=.*|readonly EC_UDEV_RULES=\"${tmp}/99-ethercat.rules\"|" \
    -e "s|^readonly RT_SYSCTL_CONF=.*|readonly RT_SYSCTL_CONF=\"${tmp}/sysctl.conf\"|" \
    -e "s|^readonly EC_NATIVE_BIND_SERVICE=.*|readonly EC_NATIVE_BIND_SERVICE=\"${tmp}/plem-ethercat-native-bind.service\"|" \
    -e "s|^readonly EC_NATIVE_BIND_DROPIN=.*|readonly EC_NATIVE_BIND_DROPIN=\"${tmp}/ethercat.service.d/10-native-bind.conf\"|" \
    -e "s|^readonly EC_IRQ_TUNE_SCRIPT=.*|readonly EC_IRQ_TUNE_SCRIPT=\"${tmp}/plem_ethercat_nic_irq_tune.sh\"|" \
    -e "s|^readonly EC_IRQ_TUNE_SERVICE=.*|readonly EC_IRQ_TUNE_SERVICE=\"${tmp}/plem-ethercat-nic-irq-tune.service\"|" \
    -e "s|^readonly EC_IRQ_TUNE_DROPIN=.*|readonly EC_IRQ_TUNE_DROPIN=\"${tmp}/plem-ethercat-nic-irq-tune.service.d/10-native-bind.conf\"|" \
    -e "s|^    local conf=\"/etc/ethercat.conf\"$|    local conf=\"${tmp}/ethercat.conf\"|" \
    -e "s|^    local svc=\"/etc/systemd/system/jetson-clocks.service\"$|    local svc=\"${tmp}/jetson-clocks.service\"|" "$WIZ" >"${tmp}/lib.sh"
grep -q "^readonly RT_FREEZE_PREF=\"${tmp}/freeze.pref\"" "${tmp}/lib.sh" || fail "RT_FREEZE_PREF line not found"
grep -q "^readonly RT_REPO_FILE=\"${tmp}/l4t.list\"" "${tmp}/lib.sh" || fail "RT_REPO_FILE line not found"
grep -q "^readonly EXTLINUX=\"${tmp}/extlinux.conf\"" "${tmp}/lib.sh" || fail "EXTLINUX line not found (a case would read /boot)"
grep -q "^readonly ETHERLAB_PREFIX=\"${tmp}/etherlab\"" "${tmp}/lib.sh" || fail "ETHERLAB_PREFIX line not found (a case would run the real ethercatctl)"
grep -q "${tmp}/etherlab.conf" "${tmp}/lib.sh" || fail "ld.so.conf.d path not redirected (a case would write /etc)"
grep -qF "for d in ${tmp}/sys/bus/pci/drivers/ec_*" "${tmp}/lib.sh" || fail "ec_release_native_nics sysfs not redirected (a case would unbind a real NIC)"
grep -q "^readonly RT_LIMITS=\"${tmp}/limits.conf\"" "${tmp}/lib.sh" || fail "RT_LIMITS line not found"
grep -q "^CMD_LOG=\$(mktemp ${tmp}/" "${tmp}/lib.sh" || fail "CMD_LOG line not found (logs would stay in /tmp)"
grep -q "^readonly EC_UDEV_RULES=\"${tmp}/99-ethercat.rules\"" "${tmp}/lib.sh" || fail "EC_UDEV_RULES line not found (a case would write /etc)"
grep -q "^readonly RT_SYSCTL_CONF=\"${tmp}/sysctl.conf\"" "${tmp}/lib.sh" || fail "RT_SYSCTL_CONF line not found (a case would write /etc)"
grep -q "^readonly EC_NATIVE_BIND_SERVICE=\"${tmp}/" "${tmp}/lib.sh" || fail "EC_NATIVE_BIND_SERVICE line not found (a case would write /etc)"
grep -q "^readonly EC_NATIVE_BIND_DROPIN=\"${tmp}/" "${tmp}/lib.sh" || fail "EC_NATIVE_BIND_DROPIN line not found (a case would write /etc)"
for v in EC_IRQ_TUNE_SCRIPT EC_IRQ_TUNE_SERVICE EC_IRQ_TUNE_DROPIN; do
    grep -q "^readonly ${v}=\"${tmp}/" "${tmp}/lib.sh" || fail "${v} line not found (a case would write /etc)"
done
grep -q "^    local conf=\"${tmp}/ethercat.conf\"" "${tmp}/lib.sh" || fail "ethercat.conf path line not found"
grep -q "^    local svc=\"${tmp}/jetson-clocks.service\"" "${tmp}/lib.sh" || fail "jetson-clocks.service path line not found"

mkdir -p "${tmp}/bin"
# systemctl stub: logs every call; "is-enabled <unit>" answers "masked" for units listed in masked.txt.
cat >"${tmp}/bin/systemctl" <<EOF
#!/usr/bin/env bash
echo "systemctl \$*" >>"${tmp}/systemctl.log"
for a in "\$@"; do last="\$a"; done
[[ " \$* " == *" is-enabled "* ]] && grep -qx "\$last" "${tmp}/disabled.txt" 2>/dev/null && { echo disabled; exit 1; }
[[ " \$* " == *" is-enabled "* ]] && { grep -qx "\$last" "${tmp}/masked.txt" 2>/dev/null && echo masked || echo enabled; }
exit 0
EOF
: >"${tmp}/masked.txt"
printf '%s\n' '#!/usr/bin/env bash' 'cat "'"${tmp}"'/dpkg.out"' >"${tmp}/bin/dpkg-query"
chmod 0755 "${tmp}/bin/systemctl" "${tmp}/bin/dpkg-query"

# usage/parse_args live below the marker: source them separately.
sed -n '/^usage() {$/,/^}$/p; /^parse_args() {$/,/^}$/p' "$WIZ" >"${tmp}/args.sh"
grep -q '^parse_args() {' "${tmp}/args.sh" || fail "parse_args not found"

probe() {  # probe <shell code> → stdout of the code run with the wizard functions loaded
    PATH="${tmp}/bin:${PATH}" bash -c 'source "$1" >/dev/null 2>&1; shift; eval "$1"' _ "${tmp}/lib.sh" "$1" 2>&1
}

# parse_args
out="$(bash -c 'BASELINE_DEFAULT=false; source "$1"; parse_args --baseline; echo "D=${BASELINE_DEFAULT}"' _ "${tmp}/args.sh")"
[[ "$out" == *"D=true"* ]] || fail "parse_args --baseline: ${out}"
out="$(bash -c 'BASELINE_DEFAULT=false; source "$1"; parse_args; echo "D=${BASELINE_DEFAULT}"' _ "${tmp}/args.sh")"
[[ "$out" == *"D=false"* ]] || fail "parse_args without options: ${out}"

# Wiring (the functions above are only useful if these calls exist).
# Captured, not piped: grep -q exits early and pipefail would turn sed's SIGPIPE into a failure.
p1_body="$(sed -n '/^run_phase1() {$/,/^}$/p' "$WIZ")"
p2_body="$(sed -n '/^run_phase2() {$/,/^}$/p' "$WIZ")"
entry="$(sed -n '/^#  Entry point$/,$p' "$WIZ")"
grep -qx '    if ask_baseline; then APPLY_BASELINE=true; else APPLY_BASELINE=false; fi' <<<"$p1_body" \
    || fail "run_phase1 does not set APPLY_BASELINE from ask_baseline"
grep -qx '    check_version_hold' <<<"$p2_body" || fail "run_phase2 does not call check_version_hold"
grep -qx '    cleanup_legacy' <<<"$p1_body" || fail "run_phase1 does not call cleanup_legacy"
# p3_s9's body holds a heredoc whose helper functions end in "}" at column 0: cut at the next step.
grep -qx '    remove_old_native_bind_units' <<<"$(sed -n '/^p3_s9_native_bind() {$/,/^p3_s10_start_verify() {$/p' "$WIZ")" \
    || fail "p3_s9_native_bind does not call remove_old_native_bind_units"
grep -A1 -x '    elif baseline_selected; then' <<<"$p2_body" | grep -q 'p2_check "Unused services" "warn"' \
    || fail "run_phase2 does not warn on baseline drift"
grep -qx 'parse_args "\$@"' <<<"$entry" || fail "entry point does not call parse_args"

# ask_baseline: default answer (empty line) follows --baseline and the current state.
[[ "$(probe 'baseline_selected() { return 1; }; baseline_left_on() { echo "bluetooth.service "; }; ask_baseline <<<"" && echo ANS=Y || echo ANS=N')" == *ANS=N* ]] \
    || fail "ask_baseline: services on, no --baseline must default to no"
[[ "$(probe 'baseline_selected() { return 1; }; baseline_left_on() { echo "bluetooth.service "; }; BASELINE_DEFAULT=true; ask_baseline <<<"" && echo ANS=Y || echo ANS=N')" == *ANS=Y* ]] \
    || fail "ask_baseline: --baseline must default to yes"
[[ "$(probe 'baseline_left_on() { :; }; ask_baseline <<<"" && echo ANS=Y || echo ANS=N')" == *ANS=Y* ]] \
    || fail "ask_baseline: already off must default to yes"
[[ "$(probe 'baseline_selected() { return 0; }; baseline_left_on() { echo x; }; ask_baseline <<<"" && echo ANS=Y || echo ANS=N')" == *ANS=Y* ]] \
    || fail "ask_baseline: a device that had it applied must default to yes"
[[ "$(probe 'baseline_left_on() { :; }; ask_baseline <<<"n" && echo ANS=Y || echo ANS=N')" == *ANS=N* ]] \
    || fail "ask_baseline: an explicit no must win"
[[ "$(probe 'baseline_selected() { return 1; }; baseline_left_on() { echo x; }; ask_baseline <<<"y" && echo ANS=Y || echo ANS=N')" == *ANS=Y* ]] \
    || fail "ask_baseline: an explicit yes must win"

# baseline_selected: true once any unit the baseline masks is masked (system or user).
[[ "$(probe 'baseline_selected && echo SEL=1 || echo SEL=0')" == *SEL=0* ]] || fail "baseline_selected: nothing masked must be false"
echo packagekit.service >"${tmp}/masked.txt"
[[ "$(probe 'baseline_selected && echo SEL=1 || echo SEL=0')" == *SEL=1* ]] || fail "baseline_selected: a masked system unit must be true"
echo pipewire.socket >"${tmp}/masked.txt"
[[ "$(probe 'baseline_selected && echo SEL=1 || echo SEL=0')" == *SEL=1* ]] || fail "baseline_selected: a masked user unit must be true"
: >"${tmp}/masked.txt"

# p1_s13 not selected: skipped, no systemctl call.
rm -f "${tmp}/systemctl.log"
out="$(probe 'APPLY_BASELINE=false; p1_s13; echo "RESULT=${PHASE1_RESULTS[-1]}"')"
[[ "$out" == *"RESULT=skipped"* ]] || fail "p1_s13 not selected: ${out}"
[[ ! -e "${tmp}/systemctl.log" ]] || fail "p1_s13 not selected called systemctl: $(cat "${tmp}/systemctl.log")"

# p1_s12 / Phase 2 crash logging: the result follows crash-logging.sh's exit code and state line
# (its own behavior is in test_crash_logging.sh). A fake tool in the wizard directory answers.
printf '%s\n' '#!/bin/bash' 'echo "noise on stderr" >&2' 'printf %s "$*" >"${0%/*}/crash.args"' 'echo "$FAKE_ST"; exit "$FAKE_RC"' >"${tmp}/crash-logging.sh"
for c in "0|applied: persistent logs|done|✔ Persistent logs" "0|ok: already set|skipped|→ Already set" \
         "1|failed: journald restart/flush failed|failed|✘ journald restart/flush failed" "2|failed: root is required|failed|✘ root is required" \
         "0|failed: odd|failed|✘ odd" "1|ok: odd|failed|✘ odd"; do
    IFS='|' read -r frc fst want msg <<<"$c"
    out="$(FAKE_RC=$frc FAKE_ST=$fst probe 'p1_header() { :; }; p1_s12 </dev/null; echo "RESULT=${PHASE1_RESULTS[-1]}"')"
    [[ "$out" == *"RESULT=${want}"* && "$(sed 's/\x1b\[[0-9;]*m//g' <<<"$out")" == *"$msg"* ]] || fail "p1_s12 rc=${frc} '${fst}': ${out}"
    [[ ! -s "${tmp}/crash.args" ]] || fail "p1_s12 must run the tool without arguments: $(cat "${tmp}/crash.args")"
done
# Phase 2: run_phase2's own crash logging block, cut from the wizard and run in a function.
p2_crash="$(sed -n '/^    # Crash logging\. /,/^    fi$/p' <<<"$p2_body")"
grep -qF 'crash_logging --check' <<<"$p2_crash" || fail "run_phase2 crash logging block not found or does not use --check: ${p2_crash}"
for c in "0|ok: logs persistent, reboot 10 s after a panic|pass|logs persistent, reboot 10 s after a panic" \
         "1|warn: logs kept in RAM only|warn|logs kept in RAM only. Re-run menu 1" "2|failed: usage|warn|failed: usage. Re-run menu 1"; do
    IFS='|' read -r frc fst want msg <<<"$c"
    out="$(FAKE_RC=$frc FAKE_ST=$fst probe "p2_check() { echo \"P2 \$1|\$2|\$3\"; }; f() {
${p2_crash}
}; f")"
    [[ "$out" == *"P2 Crash logging|${want}|${msg}"* ]] || fail "Phase 2 crash logging rc=${frc}: ${out}"
    [[ "$(cat "${tmp}/crash.args")" == "--check" ]] || fail "Phase 2 must run the tool with --check"
done
rm -f "${tmp}/crash-logging.sh" "${tmp}/crash.args"

# p1_s15 / Phase 2 RT permission. The real limits file sits in the wizard directory (WIZARD_DIR = $tmp);
# getent/groupadd/id/usermod are functions: the group exists when rt.group exists, members are in rt.members.
cp "${HERE}/99-plem-realtime.conf" "${tmp}/99-plem-realtime.conf"
rt_stubs='getent() { [[ "$1 $2" == "group realtime" && -e "'"${tmp}"'/rt.group" ]]; }
groupadd() { echo "groupadd $*" >>"'"${tmp}"'/rt.log"; [[ -z "${RT_FAIL:-}" ]] && : >"'"${tmp}"'/rt.group"; [[ -z "${RT_FAIL:-}" ]]; }
id() { echo "$2 $(cat "'"${tmp}"'/rt.members" 2>/dev/null)"; }
usermod() { echo "usermod $*" >>"'"${tmp}"'/rt.log"; }
p1_header() { :; }'
rt_state() {  # rt_state <group: y|n> <limits: same|diff|none> <members>
    rm -f "${tmp}/rt.group" "${tmp}/rt.log" "${tmp}/limits.conf"
    if [[ "$1" == y ]]; then : >"${tmp}/rt.group"; fi
    case "$2" in same) cp "${tmp}/99-plem-realtime.conf" "${tmp}/limits.conf" ;; diff) echo 'op - rtprio 98' >"${tmp}/limits.conf" ;; esac
    echo "$3" >"${tmp}/rt.members"
}
rt_case() {  # rt_case <group> <limits> <members> <SUDO_USER> → output of p1_s15
    rt_state "$1" "$2" "$3"
    SUDO_USER="$4" probe "${rt_stubs}"'
p1_s15 </dev/null; echo "RESULT=${PHASE1_RESULTS[-1]}"'
}
# Fresh device: group created (--system), file installed, sudo user added, re-login said.
out="$(rt_case n none "" op)"
[[ "$out" == *RESULT=done* && "$out" == *"log out and in again"* ]] || fail "p1_s15 fresh: ${out}"
cmp -s "${tmp}/99-plem-realtime.conf" "${tmp}/limits.conf" || fail "p1_s15 fresh: limits file not installed"
[[ "$(cat "${tmp}/rt.log")" == $'groupadd --system realtime\nusermod -aG realtime op' ]] || fail "p1_s15 fresh: $(cat "${tmp}/rt.log")"
[[ "$(stat -c %a "${tmp}/limits.conf")" == 644 ]] || fail "p1_s15: limits file mode $(stat -c %a "${tmp}/limits.conf")"
# Already set: nothing written or run.
out="$(rt_case y same realtime op)"
[[ "$out" == *RESULT=skipped* && ! -e "${tmp}/rt.log" ]] || fail "p1_s15 already set: ${out} / $(cat "${tmp}/rt.log" 2>/dev/null)"
# File differs: rewritten with the source bytes, nothing else.
out="$(rt_case y diff realtime op)"
[[ "$out" == *RESULT=done* && ! -e "${tmp}/rt.log" ]] && cmp -s "${tmp}/99-plem-realtime.conf" "${tmp}/limits.conf" \
    || fail "p1_s15 file differs: ${out} / $(cat "${tmp}/limits.conf")"
# Not a member (a group with a longer name does not count): only usermod.
for mem in "" "realtime2 adm"; do
    out="$(rt_case y same "$mem" op)"
    [[ "$out" == *RESULT=done* && "$(cat "${tmp}/rt.log")" == 'usermod -aG realtime op' ]] || fail "p1_s15 not a member ('${mem}'): ${out}"
done
# No sudo user (unset or root): group and file set, no usermod, the manual command shown.
for u in "" root; do
    out="$(rt_case n none "" "$u")"
    [[ "$out" == *RESULT=done* && "$(cat "${tmp}/rt.log")" == 'groupadd --system realtime' && "$out" == *"usermod -aG realtime <user>"* ]] \
        || fail "p1_s15 SUDO_USER='${u}': ${out} / $(cat "${tmp}/rt.log")"
done
out="$(rt_case y same "" root)"
[[ "$out" == *RESULT=skipped* && ! -e "${tmp}/rt.log" ]] || fail "p1_s15 root, already set: ${out}"
# groupadd fails: failed, nothing written after it.
out="$(RT_FAIL=1 rt_case n none "" op)"
[[ "$out" == *RESULT=failed* && ! -e "${tmp}/limits.conf" ]] || fail "p1_s15 groupadd failure: ${out}"
# Phase 2: run_phase2's own RT permission block.
p2_rt="$(sed -n '/^    # RT permission (Phase 1 step 15)/,/^    fi$/p' <<<"$p2_body")"
[[ -n "$p2_rt" ]] || fail "run_phase2 RT permission block not found"
rt_p2() {  # rt_p2 <group> <limits> <members> <SUDO_USER> → "P2 name|status|msg"
    rt_state "$1" "$2" "$3"
    SUDO_USER="$4" probe "${rt_stubs}"'
p2_check() { echo "P2 $1|$2|$3"; }; f() {
'"${p2_rt}"'
}; f'
}
for c in "y|same|realtime|op|pass|realtime group and limits set, op is a member" "y|same||root|pass|realtime group and limits set" \
         "n|same|realtime|op|warn|no realtime group. Re-run menu 1" "y|diff|realtime|op|warn|${tmp}/limits.conf missing or different. Re-run menu 1" \
         "y|none|realtime|op|warn|${tmp}/limits.conf missing or different. Re-run menu 1" \
         "y|same||op|warn|op is not in the realtime group. Re-run menu 1, then log in again"; do
    IFS='|' read -r g l mem u want msg <<<"$c"
    out="$(rt_p2 "$g" "$l" "$mem" "$u")"
    grep -qxF "P2 RT permission|${want}|${msg}" <<<"$out" || fail "Phase 2 RT permission ${c}: ${out}"
done
# Menu 3 step 7: the EtherCAT device rule is 0660 group realtime; the step makes the group and adds the sudo
# user first (realtime_membership, shared with step 15). An existing different rule is replaced, kept as .plem-bak.
want_rule='KERNEL=="EtherCAT[0-9]*", MODE="0660", GROUP="realtime"'
udev_case() {  # udev_case <group> <rule file content, or empty for none> <members> <SUDO_USER> → output of p3_s7_udev + RC
    rt_state "$1" same "$3"
    rm -f "${tmp}/99-ethercat.rules" "${tmp}/99-ethercat.rules.plem-bak" "${tmp}/udevadm.log"
    if [[ -n "$2" ]]; then printf '%s\n' "$2" >"${tmp}/99-ethercat.rules"; fi
    SUDO_USER="$4" probe "${rt_stubs}"'
p3_header() { :; }; udevadm() { echo "udevadm $*" >>"'"${tmp}"'/udevadm.log"; [[ -z "${UDEV_FAIL:-}" ]]; }
[[ -n "${CP_FAIL:-}" ]] && cp() { return 1; }
p3_s7_udev </dev/null; echo "RC=$?"; echo "RESULT=${PHASE3_RESULTS[-1]}"'
}
out="$(udev_case n "" "" op)"
[[ "$out" == *RC=0*RESULT=done* && "$(cat "${tmp}/99-ethercat.rules")" == "$want_rule" && "$out" == *"log out and in again"* ]] \
    || fail "p3_s7_udev fresh: ${out} / $(cat "${tmp}/99-ethercat.rules" 2>/dev/null)"
[[ "$(cat "${tmp}/rt.log")" == $'groupadd --system realtime\nusermod -aG realtime op' ]] || fail "p3_s7_udev fresh: $(cat "${tmp}/rt.log")"
[[ -s "${tmp}/udevadm.log" ]] || fail "p3_s7_udev fresh: udev rules not reloaded"
# Root logged in directly (no SUDO_USER): group made, nobody added, the manual command shown.
out="$(udev_case n "" "" "")"
[[ "$out" == *RC=0*RESULT=done* && "$(cat "${tmp}/rt.log")" == 'groupadd --system realtime' && "$out" == *"usermod -aG realtime <user>"* ]] \
    || fail "p3_s7_udev as root: ${out} / $(cat "${tmp}/rt.log")"
out="$(udev_case y 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' realtime op)"
[[ "$out" == *RC=0*RESULT=done* && "$(cat "${tmp}/99-ethercat.rules")" == "$want_rule" && ! -e "${tmp}/rt.log" ]] \
    || fail "p3_s7_udev 0666 rule: ${out} / $(cat "${tmp}/99-ethercat.rules")"
[[ "$(cat "${tmp}/99-ethercat.rules.plem-bak")" == 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' ]] || fail "p3_s7_udev: the replaced rule must be kept as .plem-bak"
# Already set: nothing written, but udev is reloaded and triggered (an earlier run may have failed to).
out="$(udev_case y "$want_rule" realtime op)"
[[ "$out" == *RC=0*RESULT=skipped* && ! -e "${tmp}/rt.log" ]] || fail "p3_s7_udev already set: ${out}"
grep -q 'control --reload-rules' "${tmp}/udevadm.log" 2>/dev/null && grep -qx "udevadm trigger --sysname-match=EtherCAT\*" "${tmp}/udevadm.log" \
    || fail "p3_s7_udev already set: udev must still be reloaded and triggered: $(cat "${tmp}/udevadm.log" 2>/dev/null)"
out="$(udev_case y "$want_rule" "" op)"
[[ "$out" == *RC=0*RESULT=done* && "$(cat "${tmp}/rt.log")" == 'usermod -aG realtime op' ]] || fail "p3_s7_udev not a member: ${out}"
# Failures: RC 1 (so the runner stops), result failed.
out="$(RT_FAIL=1 udev_case n "" "" op)"
[[ "$out" == *RC=1*RESULT=failed* && ! -e "${tmp}/99-ethercat.rules" ]] || fail "p3_s7_udev groupadd failure: ${out}"
out="$(CP_FAIL=1 udev_case y 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' realtime op)"
[[ "$out" == *RC=1*RESULT=failed* && "$(cat "${tmp}/99-ethercat.rules")" == 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' ]] \
    || fail "p3_s7_udev backup failure must keep the old rule: ${out} / $(cat "${tmp}/99-ethercat.rules")"
for r in "" "$want_rule"; do
    out="$(UDEV_FAIL=1 udev_case y "$r" realtime op)"
    [[ "$out" == *RC=1*RESULT=failed* ]] || fail "p3_s7_udev udevadm failure (rule '${r}'): ${out}"
done
# Through the runner: a failed step 7 stops Phase 3 before the service steps.
rt_state n same ""
out="$(RT_FAIL=1 SUDO_USER=op probe "${rt_stubs}
cd '${tmp}'; RT_BOOTED=true; clear() { :; }; dpkg() { echo 'ii  x'; }; phase3_is_complete() { return 1; }
p3_header() { :; }; udevadm() { :; }
p3_s1_detect_interface() { PHASE3_RESULTS+=(done); }; p3_s2_dependencies() { PHASE3_RESULTS+=(done); }
p3_s3_clone() { PHASE3_RESULTS+=(done); }; p3_s4_build() { PHASE3_RESULTS+=(done); }
p3_s5_install() { PHASE3_RESULTS+=(done); }; p3_s6_configure() { PHASE3_RESULTS+=(done); }
p3_s8_systemd() { echo RAN-S8; PHASE3_RESULTS+=(done); }; p3_s9_native_bind() { echo RAN-S9; PHASE3_RESULTS+=(done); }
p3_s10_start_verify() { echo RAN-S10; PHASE3_RESULTS+=(done); }
run_phase3 <<<\$'y\n\n'")"
[[ "$out" == *"Failed to create the realtime group"* && "$out" != *RAN-S* ]] || fail "a failed step 7 must stop Phase 3: ${out}"
# ec_unload_master (step 5, and step 1 on the native backend): lsmod is stateful and rmmod honours module order —
# a user module leaves ec_master's Used-by list, and ec_master unloads only once that list is empty — so the
# device-module loop is load-bearing. A module a process holds (every rmmod fails) stops the step before make;
# one that unloads (also as a comma-separated pair) lets the install run; a user not named ec_* is left loaded and
# refused.
cat >"${tmp}/ec_stubs.sh" <<EOF
p3_header() { :; }; spin_start() { :; }; spin_stop() { :; }; depmod() { :; }; systemctl() { :; }; modprobe() { :; }
ldconfig() { :; }; ln() { :; }; make() { : >"${tmp}/make.ran"; }; ETHERCAT_SRC="${tmp}"
lsmod() { grep -v "^\$" "${tmp}/lsmod.state" || true; }
rmmod() {
    [[ "\$RMMOD_OK" == 1 && "\$1" != *,* ]] || return 1
    local size count users; read -r _ size count users < <(grep "^ec_master " "${tmp}/lsmod.state")
    if [[ "\$1" == ec_master ]]; then [[ "\$count" == 0 ]] || return 1; : >"${tmp}/lsmod.state"; return 0; fi
    users=",\${users},"; users="\${users/,\$1,/,}"; users="\${users#,}"; users="\${users%,}"
    count=0; [[ -n "\$users" ]] && count=\$(tr ',' '\n' <<<"\$users" | wc -l)
    { grep -v "^\$1 \|^ec_master " "${tmp}/lsmod.state"; echo "ec_master \$size \$count\${users:+ \$users}"; } >"${tmp}/lsmod.new"
    mv "${tmp}/lsmod.new" "${tmp}/lsmod.state"
}
EOF
s5_case() {  # s5_case <lsmod state: '' or 'ec_master …'> <rmmod succeeds: 0|1> → output of p3_s5_install + RC
    rm -f "${tmp}/make.ran"; printf '%s\n' "$1" >"${tmp}/lsmod.state"
    RMMOD_OK="$2" probe "source '${tmp}/ec_stubs.sh'; p3_s5_install </dev/null; echo \"RC=\$? RESULT=\${PHASE3_RESULTS[-1]}\""
}
out="$(s5_case 'ec_master 24576 1 ec_igb' 0)"
[[ "$out" == *"ec_master is still in use (lsmod: ec_master 24576 1 ec_igb)"*"run menu 3 again"* && "$out" == *"RC=1 RESULT=failed"* ]] \
    || fail "p3_s5: a held master must stop the install: ${out}"
[[ ! -e "${tmp}/make.ran" ]] || fail "p3_s5: a held master must stop the install before make"
for state in 'ec_master 24576 1 ec_igb' 'ec_master 24576 2 ec_generic,ec_igb'; do
    out="$(s5_case "$state" 1)"
    [[ -e "${tmp}/make.ran" && "$out" == *"RC=0 RESULT=done"* ]] || fail "p3_s5: modules that unload ('${state}') must let the install run: ${out}"
done
out="$(s5_case 'ec_master 24576 1 my_rt_app' 1)"
[[ ! -e "${tmp}/make.ran" && "$out" == *"ec_master is still in use (lsmod: ec_master 24576 1 my_rt_app)"* && "$out" == *"RC=1 RESULT=failed"* ]] \
    || fail "p3_s5: a user not named ec_* must not be unloaded: ${out}"
out="$(s5_case '' 1)"
[[ -e "${tmp}/make.ran" && "$out" == *"RC=0 RESULT=done"* && "$out" != *"still in use"* ]] || fail "p3_s5: with nothing loaded the install must proceed: ${out}"
# Step 1 (native backend): the real ec_release_native_nics on a fake sysfs (the harness points its /sys/bus/pci at
# ${tmp}). A held master refuses before any unbind or bind is written; one that unloads leaves the port unbound from
# ec_igb and bound back to igb.
pci="${tmp}/sys/bus/pci"
native_case() {  # native_case <rmmod succeeds: 0|1> → output of ec_release_native_nics + RC
    rm -rf "${tmp}/sys"; mkdir -p "${pci}/drivers/ec_igb" "${pci}/drivers/igb" "${pci}/devices/0000:01:00.0/net/eth9"
    : >"${pci}/drivers/ec_igb/0000:01:00.0"; ln -s "${pci}/drivers/ec_igb" "${pci}/devices/0000:01:00.0/driver"
    printf 'ec_master 24576 1 ec_igb\n' >"${tmp}/lsmod.state"
    RMMOD_OK="$1" probe "source '${tmp}/ec_stubs.sh'; ec_release_native_nics; echo \"RC=\$?\""
}
out="$(native_case 0)"
[[ "$out" == *"still in use"* && "$out" == *"RC=1"* && ! -e "${pci}/drivers/ec_igb/unbind" && ! -e "${pci}/drivers/igb/bind" ]] \
    || fail "native release: a held master must refuse before touching sysfs: ${out}"
out="$(native_case 1)"
[[ "$out" == *"RC=0"* && "$(cat "${pci}/drivers/ec_igb/unbind")" == "0000:01:00.0" && "$(cat "${pci}/drivers/igb/bind")" == "0000:01:00.0" ]] \
    || fail "native release: a port whose master unloads must return to igb: ${out}"
# Step 1: a native-backend release that finds the master still held (ec_release_native_nics returns 1) ends the
# step as failed before any port is detected. Detection's find is stubbed to stderr (its stdout is consumed by
# the read loop), so a step that went on to detect shows RAN-DETECT; ETHERCAT_IFACE is cleared so it would.
out="$(ETHERCAT_IFACE='' probe 'p3_header() { :; }; ec_release_native_nics() { return 1; }; find() { echo RAN-DETECT >&2; }
p3_s1_detect_interface </dev/null; echo "RC=$? RESULT=${PHASE3_RESULTS[-1]}"')"
[[ "$out" == *"RC=1 RESULT=failed"* && "$out" != *RAN-DETECT* ]] || fail "p3_s1: a held master must end the step before detection: ${out}"
# Step 10: after the service starts, a device node that does not have mode 0660 and group realtime is named (a warning, not a failure).
s10_case() {  # s10_case <"mode group"> → output of p3_s10_start_verify
    FAKE_STAT="$1" probe 'p3_header() { :; }; spin_start() { :; }; spin_stop() { :; }; depmod() { :; }; sleep() { :; }
lsmod() { echo "ec_master 1"; }; ethercat() { :; }; ETHERCAT_SRC=""
ec_device_nodes() { echo /dev/EtherCAT0; }; stat() { echo "$FAKE_STAT"; }
udevadm() { :; }
p3_s10_start_verify </dev/null; echo "RESULT=${PHASE3_RESULTS[-1]}"'
}
out="$(s10_case "666 root")"
[[ "$out" == *"/dev/EtherCAT0 does not have mode 0660 and group realtime: another udev rule may override it (grep -rl EtherCAT /etc/udev/rules.d /lib/udev/rules.d), then check with menu 2"* && "$out" == *RESULT=done* ]] \
    || fail "p3_s10: a 0666 node must be named: ${out}"
out="$(s10_case "660 realtime")"
[[ "$out" != *"does not have mode 0660"* && "$out" == *RESULT=done* ]] || fail "p3_s10: a 0660 realtime node is fine: ${out}"
# Step 10: the link comes up a few seconds after the service starts; the slave count is read once it has
# (bounded), so a link still coming up is not reported as "no slaves". The fake master shows 2 slaves from
# its LINK_AT-th call (a counter file, since each call runs in a subshell).
link_case() {  # link_case <LINK_AT> → output of p3_s10_start_verify
    LINK_AT="$1" CNT="${tmp}/ec.calls" probe 'p3_header() { :; }; spin_start() { :; }; spin_stop() { :; }; depmod() { :; }; sleep() { :; }
lsmod() { echo "ec_master 1"; }; ETHERCAT_SRC=""; : >"$CNT"
ethercat() { echo x >>"$CNT"; local n; n=$(wc -l <"$CNT")
  if (( n >= LINK_AT )); then printf "Master0\n  Phase: Idle\n  Slaves: 2\n  Link: UP\n"; else printf "Master0\n  Phase: Idle\n  Slaves: 0\n  Link: DOWN\n"; fi; }
ec_device_nodes() { :; }; udevadm() { :; }
p3_s10_start_verify </dev/null; echo "RESULT=${PHASE3_RESULTS[-1]}"'
}
out="$(link_case 4)"
[[ "$out" == *"Slaves=2"* && "$out" != *"no slaves connected"* ]] || fail "p3_s10: a link that comes up within the wait must show its slaves: ${out}"
out="$(link_case 999)"
[[ "$out" == *"Slaves=0"* && "$out" == *"no slaves connected"* ]] || fail "p3_s10: no slaves after the wait must still warn: ${out}"
grep -qF 'realtime_membership' <<<"$(sed -n '/^p1_s15() {$/,/^}$/p' "$WIZ")" || fail "p1_s15 must use realtime_membership"
# Phase 2: run_phase2's EtherCAT device block. Device nodes and their mode/group are stubbed.
p2_ec="$(sed -n '/^    # EtherCAT device permission/,/^    fi$/p' <<<"$p2_body")"
[[ -n "$p2_ec" ]] || fail "run_phase2 EtherCAT device permission block not found"
ec_p2() {  # ec_p2 <ethercat installed: y|n> <nodes> <"mode group"> <members> <SUDO_USER> → "P2 name|status|msg"
    rt_state y same "$4"
    SUDO_USER="$5" FAKE_NODES="$2" FAKE_STAT="$3" probe "${rt_stubs}
$([[ "$1" == y ]] && echo 'ethercat() { :; }')"'
command() { if [[ "$1 $2" == "-v ethercat" ]]; then declare -F ethercat >/dev/null; else builtin command "$@"; fi; }
ec_device_nodes() { [[ -n "$FAKE_NODES" ]] && echo "$FAKE_NODES"; }
stat() { echo "$FAKE_STAT"; }
p2_check() { echo "P2 $1|$2|$3"; }; f() {
'"${p2_ec}"'
}; f'
}
for c in "n||660 realtime|realtime|op|info|EtherCAT master not installed (menu 3)" \
         "y||660 realtime|realtime|op|warn|no /dev/EtherCAT* device: start the EtherCAT master (sudo systemctl start ethercat)" \
         "y|/dev/EtherCAT0|660 realtime|realtime|op|pass|/dev/EtherCAT* 0660 group realtime, op is a member (if menu 1 or 3 just added op, log out and in again: \`id -nG\` in your own shell must list realtime)" \
         "y|/dev/EtherCAT0|660 realtime||root|pass|/dev/EtherCAT* 0660 group realtime" \
         "y|/dev/EtherCAT0|666 root|realtime|op|warn|/dev/EtherCAT0 does not have mode 0660 and group realtime. Re-run menu 3" \
         "y|/dev/EtherCAT0|660 realtime||op|warn|op cannot open it: sudo usermod -aG realtime op, then log out and in again"; do
    IFS='|' read -r inst nodes st mem u want msg <<<"$c"
    out="$(ec_p2 "$inst" "$nodes" "$st" "$mem" "$u")"
    grep -qxF "P2 EtherCAT device|${want}|${msg}" <<<"$out" || fail "Phase 2 EtherCAT device ${c}: ${out}"
done
rm -f "${tmp}/rt.group" "${tmp}/rt.log" "${tmp}/rt.members" "${tmp}/limits.conf" "${tmp}/99-plem-realtime.conf" "${tmp}/99-ethercat.rules"*

# check_version_hold
printf '%s\n' 'hi  nvidia-l4t-core 1' 'hi  nvidia-l4t-rt-kernel 1' >"${tmp}/dpkg.out"
rm -f "${tmp}/freeze.pref"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"no version pin"* && "$out" == *"R=warn"* ]] || fail "hold: missing pin file must warn: ${out}"
: >"${tmp}/freeze.pref"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"not pinned to the installed version: nvidia-l4t-core nvidia-l4t-rt-kernel "* && "$out" == *"R=warn"* ]] \
    || fail "hold: an empty pin file must warn: ${out}"
printf 'Package: %s\nPin: version 1\nPin-Priority: 1001\n\n' nvidia-l4t-core nvidia-l4t-rt-kernel >"${tmp}/freeze.pref"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"R=pass"* ]] || fail "hold: all held + all pinned must pass: ${out}"
printf '%s\n' 'hi  nvidia-l4t-core 1' 'hU  nvidia-l4t-rt-kernel 1' >"${tmp}/dpkg.out"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"install not finished: nvidia-l4t-rt-kernel "* && "$out" == *"R=warn"* ]] \
    || fail "hold: an unfinished install must warn: ${out}"
printf '%s\n' 'hi  nvidia-l4t-core 1' 'ii  nvidia-l4t-cuda 1' 'rc  nvidia-l4t-old 1' >"${tmp}/dpkg.out"
printf 'Package: %s\n\n' nvidia-l4t-core nvidia-l4t-cuda >"${tmp}/freeze.pref"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"not held: nvidia-l4t-cuda "* && "$out" != *"nvidia-l4t-old"* && "$out" == *"R=warn"* ]] \
    || fail "hold: an installed unheld package must warn (removed ones ignored): ${out}"

# p1_s4 skips only when every installed package is held AND in the pin file — the same rule as
# check_version_hold, so its "re-run Phase 1" fixes a package held by hand but missing from the pin.
printf '%s\n' 'hi  nvidia-l4t-core 1' 'hi  nvidia-l4t-rt-kernel 1' >"${tmp}/dpkg.out"
hold_case() { probe 'apt-mark() { return 1; }   # reaching "apt-mark hold" proves the step did not skip
p1_header() { :; }; p1_s4 </dev/null; echo "RESULT=${PHASE1_RESULTS[-1]}"'; }
printf 'Package: %s\nPin: version 1\nPin-Priority: 1001\n\n' nvidia-l4t-core nvidia-l4t-rt-kernel >"${tmp}/freeze.pref"
[[ "$(hold_case)" == *RESULT=skipped* ]] || fail "p1_s4: all held + all pinned must skip: $(hold_case)"
printf 'Package: %s\nPin: version 1\nPin-Priority: 1001\n\n' nvidia-l4t-core >"${tmp}/freeze.pref"
[[ "$(hold_case)" == *RESULT=failed* ]] || fail "p1_s4: a held package missing from the pin file must not skip: $(hold_case)"
rm -f "${tmp}/freeze.pref"
[[ "$(hold_case)" == *RESULT=failed* ]] || fail "p1_s4: no pin file must not skip: $(hold_case)"
# dpkg writes triggers-pending as a lowercase t: both phases call it unfinished, neither skips past it.
printf '%s\n' 'hi  nvidia-l4t-core 1' 'it  nvidia-l4t-rt-kernel 1' >"${tmp}/dpkg.out"
printf 'Package: %s\n\n' nvidia-l4t-core nvidia-l4t-rt-kernel >"${tmp}/freeze.pref"
[[ "$(hold_case)" == *"not fully installed: nvidia-l4t-rt-kernel"*RESULT=failed* ]] || fail "p1_s4: triggers-pending must stop: $(hold_case)"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"install not finished: nvidia-l4t-rt-kernel "* ]] || fail "hold: triggers-pending must warn as unfinished: ${out}"
# Config-files only (ic) is not installed: neither phase asks to hold or pin it.
printf '%s\n' 'hi  nvidia-l4t-core 1' 'ic  nvidia-l4t-old 1' >"${tmp}/dpkg.out"
printf 'Package: %s\nPin: version 1\nPin-Priority: 1001\n\n' nvidia-l4t-core >"${tmp}/freeze.pref"
[[ "$(hold_case)" == *RESULT=skipped* ]] || fail "p1_s4: a config-files-only package must not count: $(hold_case)"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"R=pass"* ]] || fail "hold: a config-files-only package must not count: ${out}"

# A pin counts only as the exact installed version at Pin-Priority 1001: a bare Package line, a stale
# version or another priority does not freeze anything, so Phase 2 warns and Phase 1 writes the pin again.
printf '%s\n' 'hi  nvidia-l4t-core 2' >"${tmp}/dpkg.out"
for stanza in 'Package: nvidia-l4t-core\n' 'Package: nvidia-l4t-core\nPin: version 1\nPin-Priority: 1001\n' \
              'Package: nvidia-l4t-core\nPin: version 2\nPin-Priority: 500\n'; do
    printf "${stanza}" >"${tmp}/freeze.pref"
    out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
    [[ "$out" == *"nvidia-l4t-core"* && "$out" == *"R=warn"* ]] || fail "hold: pin '${stanza}' must warn: ${out}"
    [[ "$(hold_case)" == *RESULT=failed* ]] || fail "p1_s4: pin '${stanza}' must not skip: $(hold_case)"
done
printf 'Package: nvidia-l4t-core\nPin: version 2\nPin-Priority: 1001\n' >"${tmp}/freeze.pref"
out="$(probe 'check_version_hold; echo "R=${PHASE2_RESULTS[*]}"')"
[[ "$out" == *"R=pass"* ]] || fail "hold: the installed version at 1001 must pass: ${out}"

# p2_check info: shown, not counted.
out="$(probe 'p2_check "Unused services" "info" "left on"; echo "N=${#PHASE2_RESULTS[@]}"')"
[[ "$out" == *"left on"* && "$out" == *"N=0"* ]] || fail "p2_check info must not be counted: ${out}"

# run_cmd: the failure message carries the command's exit code, not the negation's.
out="$(probe 'run_cmd "step" bash -c "exit 3"; echo "RC=$?"')"
[[ "$out" == *"step failed (exit: 3)"* && "$out" == *"RC=1"* ]] || fail "run_cmd must report exit 3: ${out}"
out="$(probe 'run_cmd "step" false; echo "RC=$?"')"
[[ "$out" == *"step failed (exit: 1)"* ]] || fail "run_cmd must report exit 1: ${out}"

# booted_isol_cpu: last CPU of the boot-time isolated list, empty when none.
for c in "10-11:11" "5:5" "2,4-6:6" ":"; do
    out="$(probe "cat() { echo '${c%%:*}'; }; echo \"CPU=[\$(booted_isol_cpu)]\"")"
    [[ "$out" == *"CPU=[${c#*:}]"* ]] || fail "booted_isol_cpu '${c%%:*}' must give '${c#*:}': ${out}"
done
loadsweep_body="$(sed -n '/^run_loadsweep() {$/,/^}$/p' "$WIZ")"
grep -qF 'local rt_cpu; rt_cpu="$(booted_isol_cpu)"' <<<"$loadsweep_body" || fail "menu 4 does not use booted_isol_cpu"
grep -qF 'iso_cpu=$(booted_isol_cpu)' <<<"$p2_body" || fail "Phase 2 does not use booted_isol_cpu"
! grep -q 'RT_ISOL_CPU' <<<"$p2_body" || fail "Phase 2 must check the CPU isolated at boot, not one picked in this session"
# Phase 2 CPU isolation: the boot's isolated list against the CPU Phase 1 wrote to the RT boot label.
p2_iso="$(sed -n '/^    # 2\. CPU isolation/,/^    else p2_check "CPU isolation"/p' <<<"$p2_body")"
[[ -n "$p2_iso" ]] || fail "run_phase2 CPU isolation block not found"
iso_case() {  # iso_case <isolated at boot> <CPU in the RT boot label, or empty> → "P2 name|status|msg"
    printf 'LABEL real-time\n  LINUX /boot/Image.real-time\n  APPEND root=/dev/x%s\n' "${2:+ isolcpus=managed_irq,domain,$2}" >"${tmp}/extlinux.conf"
    probe "cat() { echo '$1'; }; p2_check() { echo \"P2 \$1|\$2|\$3\"; }; f() {
${p2_iso}
}; f"
}
for c in "11|11|pass|CPU 11" "11||pass|CPU 11" "|11|fail|no isolation" \
         "11|9|warn|CPU 11 is isolated but the RT boot label sets CPU 9: reboot, or re-run menu 1" \
         "10-11|11|warn|CPU 10-11 is isolated but the RT boot label sets CPU 11: reboot, or re-run menu 1" \
         "10-11|10-11|pass|CPU 10-11" "10-11|10,11|pass|CPU 10-11" "10-11|11,10|pass|CPU 10-11" \
         "10-11|10-11,10|pass|CPU 10-11" \
         "10-11|9-11|warn|CPU 10-11 is isolated but the RT boot label sets CPU 9-11: reboot, or re-run menu 1"; do
    IFS='|' read -r booted label want msg <<<"$c"
    out="$(iso_case "$booted" "$label")"
    grep -qxF "P2 CPU isolation|${want}|${msg}" <<<"$out" || fail "Phase 2 CPU isolation ${c}: ${out}"
done
rm -f "${tmp}/extlinux.conf"

# Step 1: a commented or other-suite rt-kernel line is not "registered"; the line is appended.
common='deb https://repo.download.nvidia.com/jetson/common r39.2 main'
want='deb https://repo.download.nvidia.com/jetson/rt-kernel r39.2 main'
for other in "# ${want}" "#${want}" 'deb https://repo.download.nvidia.com/jetson/rt-kernel r39.1 main'; do
    printf '%s\n' "$common" "$other" >"${tmp}/l4t.list"
    out="$(probe 'p1_s1; echo "RESULT=${PHASE1_RESULTS[-1]}"')"
    [[ "$out" == *"RESULT=done"* && "$(tail -1 "${tmp}/l4t.list")" == "$want" ]] \
        || fail "repo: '${other}' must not count as registered: ${out}"
    [[ "$(grep -cE '^deb .*jetson/rt-kernel ' "${tmp}/l4t.list")" -eq 1 ]] \
        || fail "repo: only the right rt-kernel line may stay active after '${other}': $(cat "${tmp}/l4t.list")"
    grep -qxF "$common" "${tmp}/l4t.list" || fail "repo: the jetson/common line must stay as it is"
done
printf '%s\n' "$common" "$want" >"${tmp}/l4t.list"
out="$(probe 'p1_s1; echo "RESULT=${PHASE1_RESULTS[-1]}"')"
[[ "$out" == *"RESULT=skipped"* && "$(wc -l <"${tmp}/l4t.list")" -eq 2 ]] || fail "repo: the right line must count as registered: ${out}"
# Variants of the right line count as registered and are left alone.
for right in "  ${want}" "deb [arch=arm64] https://repo.download.nvidia.com/jetson/rt-kernel r39.2 main" \
             "deb https://repo.download.nvidia.com/jetson/rt-kernel/ r39.2 main" "deb http://repo.download.nvidia.com/jetson/rt-kernel r39.2 main" $'deb https://repo.download.nvidia.com/jetson/rt-kernel r39.2 main\r'; do
    printf '%s\n' "$common" "$right" >"${tmp}/l4t.list"
    out="$(probe 'p1_s1; echo "RESULT=${PHASE1_RESULTS[-1]}"')"
    [[ "$out" == *"RESULT=skipped"* && "$(sed -n 2p "${tmp}/l4t.list")" == "$right" ]] \
        || fail "repo: '${right}' must count as registered and stay as it is: ${out}"
done
# The right line plus an indented line for another suite: registered, the other one disabled.
printf '%s\n' "$common" "$want" '  deb https://repo.download.nvidia.com/jetson/rt-kernel/ r39.1 main' >"${tmp}/l4t.list"
out="$(probe 'p1_s1; echo "RESULT=${PHASE1_RESULTS[-1]}"')"
[[ "$out" == *"RESULT=skipped"* && "$(sed -n 3p "${tmp}/l4t.list")" == "# disabled by the RT setup wizard (other suite):   deb"* ]] \
    || fail "repo: another suite next to the right line must be disabled: ${out} / $(cat "${tmp}/l4t.list")"
# A file without a final newline: the added line is its own line.
printf '%s' "$common" >"${tmp}/l4t.list"
probe 'p1_s1' >/dev/null
[[ "$(sed -n 1p "${tmp}/l4t.list")" == "$common" && "$(sed -n 2p "${tmp}/l4t.list")" == "$want" ]] \
    || fail "repo: appending must not join the last line: $(cat "${tmp}/l4t.list")"

# ethercat.conf: written without running anything (no systemctl), value lines unchanged.
p3_s6_body="$(sed -n '/^p3_s6_configure() {$/,/^}$/p' "$WIZ")"
grep -qF "cat << 'EOFCONF'" <<<"$p3_s6_body" || fail "ethercat.conf comments must use a quoted heredoc"
rm -f "${tmp}/systemctl.log" "${tmp}/ethercat.conf"
out="$(probe 'EC_MACS=(00:11:22:33:44:55 00:11:22:33:44:66); EC_INTERFACES=(eth1 eth2); EC_DEVICE_MODULES=igb
    p3_s6_configure; echo "RESULT=${PHASE3_RESULTS[-1]}"')"
[[ "$out" == *"RESULT=done"* ]] || fail "p3_s6_configure: ${out}"
[[ ! -e "${tmp}/systemctl.log" ]] || fail "writing ethercat.conf ran systemctl: $(cat "${tmp}/systemctl.log")"
[[ "$(grep -v '^#' "${tmp}/ethercat.conf")" == "$(printf '%s\n' '' 'MASTER0_DEVICE="00:11:22:33:44:55"' \
    'MASTER1_DEVICE="00:11:22:33:44:66"' 'DEVICE_MODULES="igb"' 'UPDOWN_INTERFACES="eth1 eth2"')" ]] \
    || fail "ethercat.conf value lines changed: $(cat "${tmp}/ethercat.conf")"

# Topology: the isolated CPU leaves every other possible CPU in irqaffinity (offline ones included).
topo() { probe "TOTAL_CPUS=$1; RT_ISOL_CPU=$2; compute_rt_topology; echo \"P=\${RT_PARAMS}\""; }
[[ "$(topo 12 11)" == *"P=isolcpus=managed_irq,domain,11 irqaffinity=0,1,2,3,4,5,6,7,8,9,10"* ]] || fail "topology 12/11: $(topo 12 11)"
[[ "$(topo 12 5)" == *"P=isolcpus=managed_irq,domain,5 irqaffinity=0,1,2,3,4,6,7,8,9,10,11"* ]] || fail "topology 12/5: $(topo 12 5)"
[[ "$(topo 8 0)" == *"P=isolcpus=managed_irq,domain,0 irqaffinity=1,2,3,4,5,6,7"* ]] || fail "topology 8/0: $(topo 8 0)"

# CPU selection lists every possible CPU, offline ones included (they come online after the MAXN
# reboot); the menu index picks the CPU, a cancel keeps the current one.
pick() { probe "TOTAL_CPUS=12; RT_ISOL_CPU=11; cpu_online_cores() { echo 0 1 2 3; }
    select_menu() { echo \"ITEMS=\$(( \$# / 3 ))\" >&2; while (( \$# )); do echo \"\$1|\$2\" >&2; shift 3; done; echo $1; }
    sel=\$(select_isolation_core 2>${tmp}/menu.txt); echo \"SEL=\${sel}\""; }
[[ "$(pick 10)" == *"SEL=10"* ]] || fail "select_isolation_core: index 10 must pick CPU 10 (offline now): $(pick 10)"
grep -qx 'ITEMS=12' "${tmp}/menu.txt" || fail "select_isolation_core must offer all 12 possible CPUs: $(cat "${tmp}/menu.txt")"
grep -q '^CPU 10|.*offline now' "${tmp}/menu.txt" || fail "an offline CPU must be tagged offline: $(cat "${tmp}/menu.txt")"
grep -q '^CPU 3|.*offline' "${tmp}/menu.txt" && fail "an online CPU must not be tagged offline: $(cat "${tmp}/menu.txt")"
[[ "$(pick -1)" == *"SEL=11"* ]] || fail "select_isolation_core: a cancel must keep CPU 11: $(pick -1)"

# Step 6 edits extlinux.conf: only labels that boot the RT image get the parameters, a re-run with
# the same CPU changes nothing, another CPU replaces both values, the first original is the only backup.
extlinux_stock() {
    cat <<'EOF'
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} root=/dev/nvme0n1p1 rw rootwait

LABEL real-time
      MENU LABEL real-time kernel
      LINUX /boot/Image.real-time
      INITRD /boot/initrd
      APPEND ${cbootargs} root=/dev/nvme0n1p1 rw rootwait
EOF
}
s6() { probe "TOTAL_CPUS=12; RT_ISOL_CPU=$1; compute_rt_topology; p1_s6; echo \"RESULT=\${PHASE1_RESULTS[-1]}\""; }
rm -f "${tmp}/extlinux.conf.bak"
extlinux_stock >"${tmp}/extlinux.conf"
out="$(s6 11)"
[[ "$out" == *"RESULT=done"* ]] || fail "p1_s6 fresh: ${out}"
expected="$(extlinux_stock | sed -e 's/^DEFAULT primary$/DEFAULT real-time/' \
    -e '$s/$/ isolcpus=managed_irq,domain,11 irqaffinity=0,1,2,3,4,5,6,7,8,9,10/')"
[[ "$(cat "${tmp}/extlinux.conf")" == "$expected" ]] || fail "p1_s6 fresh: $(diff <(echo "$expected") "${tmp}/extlinux.conf")"
[[ "$(cat "${tmp}/extlinux.conf.bak")" == "$(extlinux_stock)" ]] || fail "p1_s6 fresh: the backup must be the original"
out="$(s6 11)"
[[ "$out" == *"RESULT=skipped"* ]] || fail "p1_s6 re-run: ${out}"
[[ "$(cat "${tmp}/extlinux.conf")" == "$expected" ]] || fail "p1_s6 re-run changed the file: $(diff <(echo "$expected") "${tmp}/extlinux.conf")"
out="$(s6 5)"
[[ "$out" == *"RESULT=done"* ]] || fail "p1_s6 other CPU: ${out}"
expected="$(extlinux_stock | sed -e 's/^DEFAULT primary$/DEFAULT real-time/' \
    -e '$s/$/ isolcpus=managed_irq,domain,5 irqaffinity=0,1,2,3,4,6,7,8,9,10,11/')"
[[ "$(cat "${tmp}/extlinux.conf")" == "$expected" ]] || fail "p1_s6 other CPU: $(diff <(echo "$expected") "${tmp}/extlinux.conf")"
[[ "$(cat "${tmp}/extlinux.conf.bak")" == "$(extlinux_stock)" ]] || fail "p1_s6 re-run must keep the first backup"

# A third-party label booting the RT image (made DEFAULT) is updated and kept DEFAULT.
rm -f "${tmp}/extlinux.conf.bak"
{ extlinux_stock | sed 's/^DEFAULT primary$/DEFAULT camera/'
  printf '%s\n' '' 'LABEL camera' '      LINUX /boot/Image.real-time' \
      '      APPEND ${cbootargs} quiet isolcpus=managed_irq,domain,3 irqaffinity=0,1,2 fbcon=map:0'
} >"${tmp}/extlinux.conf"
out="$(s6 11)"
[[ "$out" == *"RESULT=done"* && "$out" == *"2 of 2 RT boot labels"* ]] || fail "p1_s6 two RT labels: ${out}"
expected="$(extlinux_stock | sed -e 's/^DEFAULT primary$/DEFAULT camera/' \
    -e '$s/$/ isolcpus=managed_irq,domain,11 irqaffinity=0,1,2,3,4,5,6,7,8,9,10/'
  printf '%s\n' '' 'LABEL camera' '      LINUX /boot/Image.real-time' \
      '      APPEND ${cbootargs} quiet isolcpus=managed_irq,domain,11 irqaffinity=0,1,2,3,4,5,6,7,8,9,10 fbcon=map:0')"
[[ "$(cat "${tmp}/extlinux.conf")" == "$expected" ]] || fail "p1_s6 two RT labels: $(diff <(echo "$expected") "${tmp}/extlinux.conf")"

# A label with isolcpus= but no irqaffinity= (written by hand, or by an older wizard) is completed, not failed.
rm -f "${tmp}/extlinux.conf.bak"
extlinux_stock | sed '$s/$/ isolcpus=managed_irq,domain,3/' >"${tmp}/extlinux.conf"
out="$(s6 11)"
[[ "$out" == *"RESULT=done"* ]] || fail "p1_s6 isolcpus-only label: ${out}"
expected="$(extlinux_stock | sed -e 's/^DEFAULT primary$/DEFAULT real-time/' \
    -e '$s/$/ isolcpus=managed_irq,domain,11 irqaffinity=0,1,2,3,4,5,6,7,8,9,10/')"
[[ "$(cat "${tmp}/extlinux.conf")" == "$expected" ]] || fail "p1_s6 isolcpus-only label: $(diff <(echo "$expected") "${tmp}/extlinux.conf")"

# A file without a DEFAULT line gets one, so the RT label boots by default.
rm -f "${tmp}/extlinux.conf.bak"
extlinux_stock | grep -v '^DEFAULT ' >"${tmp}/extlinux.conf"
out="$(s6 11)"
[[ "$out" == *"RESULT=done"* && "$out" == *"DEFAULT set to real-time"* ]] || fail "p1_s6 without a DEFAULT line: ${out}"
[[ "$(head -1 "${tmp}/extlinux.conf")" == "DEFAULT real-time" ]] || fail "p1_s6 must insert DEFAULT real-time: $(head -3 "${tmp}/extlinux.conf")"

# No label boots the RT image: fails, the file untouched.
rm -f "${tmp}/extlinux.conf.bak"
extlinux_stock | sed 's|/boot/Image.real-time|/boot/Image|' >"${tmp}/extlinux.conf"
out="$(s6 11)"
[[ "$out" == *"RESULT=failed"* && "$out" == *"no label boots the RT image"* ]] || fail "p1_s6 without an RT label: ${out}"
[[ "$(cat "${tmp}/extlinux.conf")" == "$(extlinux_stock | sed 's|/boot/Image.real-time|/boot/Image|')" ]] \
    || fail "p1_s6 without an RT label changed the file"

# Wizard directory: absolute, resolved at startup (a relative path still works after a cd), used by step 14.
out="$(cd "$tmp" && bash -c 'source ./lib.sh >/dev/null 2>&1; cd /; echo "DIR=${WIZARD_DIR}"')"
[[ "$(head -1 <<<"$out")" == "DIR=${tmp}" ]] || fail "WIZARD_DIR must be the absolute wizard directory: ${out}"
[[ "$(grep -c 'BASH_SOURCE' "$WIZ")" -eq 1 ]] || fail "BASH_SOURCE must be resolved only once (WIZARD_DIR)"
grep -qF 'local src="${WIZARD_DIR}/nvpps-tsc-unlock.sh"' "$WIZ" || fail "p1_s14 does not use WIZARD_DIR"
cds="$(grep -E '^[[:space:]]*cd "\$ETHERCAT_SRC"' "$WIZ" || true)"
[[ -n "$cds" ]] || fail "no cd into the build directory found (pattern out of date)"
grep -vqF '|| {' <<<"$cds" && fail "a cd into the build directory has no failure check"

# Phase 3 on an installed device: Reinstall? answered N (or Enter) runs no step; y runs them.
p3_prompt() {  # p3_prompt <answer> → output of run_phase3
    probe "cd '${tmp}'; RT_BOOTED=true; clear() { :; }; dpkg() { echo 'ii  x'; }; phase3_is_complete() { return 0; }
    p3_s1_detect_interface() { echo RAN-S1; PHASE3_RESULTS+=(failed); return 1; }
    run_phase3 <<<\$'$1\n\n'"
}
out="$(p3_prompt n)"   # read -p shows its prompt only on a terminal; the y case below proves the steps are reachable
[[ "$out" == *"EtherCAT already installed"* && "$out" != *RAN-S1* ]] || fail "Reinstall answered n must run no step: ${out}"
out="$(p3_prompt '')"
[[ "$out" != *RAN-S1* ]] || fail "Reinstall answered Enter must run no step: ${out}"
out="$(p3_prompt y)"
[[ "$out" == *RAN-S1* ]] || fail "Reinstall answered y must run the steps: ${out}"

# Phase 3: a failed step still leaves and removes the build directory.
out="$(probe "cd '${tmp}'; RT_BOOTED=true; clear() { :; }; dpkg() { echo 'ii  x'; }; phase3_is_complete() { return 1; }
    p3_s1_detect_interface() { PHASE3_RESULTS+=(done); }; p3_s2_dependencies() { PHASE3_RESULTS+=(done); }
    p3_s3_clone() { ETHERCAT_SRC=\$(mktemp -d '${tmp}/etherlab-build-XXXXXX'); PHASE3_RESULTS+=(done); }
    p3_s4_build() { cd \"\$ETHERCAT_SRC\" || return 1; PHASE3_RESULTS+=(failed); return 1; }
    run_phase3 <<<\$'y\n'; echo \"PWD=\$PWD\"; [[ -d \"\$ETHERCAT_SRC\" ]] && echo SRC=left || echo SRC=gone")"
[[ "$out" == *"SRC=gone"* ]] || fail "Phase 3 failure left the build directory: ${out}"
[[ "$out" == *"PWD=${tmp}"$'\n'* ]] || fail "Phase 3 did not restore the working directory: ${out}"
[[ -z "$(ls -d "${tmp}"/etherlab-build-* 2>/dev/null)" ]] || fail "build directory still on disk"

# Command log: kept on exit when it has content (a failed step printed its path), an empty one removed.
out="$(probe 'echo step-output >>"$CMD_LOG"; echo "LOG=$CMD_LOG"')"
log="$(sed -n 's/^LOG=//p' <<<"$out")"
[[ -n "$log" && -s "$log" && "$out" == *"Log: ${log}"* ]] || fail "a log with content must be kept and named on exit: ${out}"
rm -f "$log"
out="$(probe 'echo "LOG=$CMD_LOG"')"
log="$(sed -n 's/^LOG=//p' <<<"$out")"
[[ -n "$log" && ! -e "$log" ]] || fail "an empty log must be removed on exit: ${out}"

# Phase 1 does not set the Wi-Fi regulatory domain (a legal radio setting, not RT tuning).
! grep -qE 'iw reg set|timedatectl|ieee80211_regdom' "$WIZ" || fail "the wizard must not set the Wi-Fi regulatory domain"

# Phase 3 installs git before it clones IgH.
p3_s2_body="$(sed -n '/^p3_s2_dependencies() {$/,/^}$/p' "$WIZ")"
grep -qE '^    local pkgs=\(.* git( |\))' <<<"$p3_s2_body" || fail "p3_s2_dependencies must install git: ${p3_s2_body}"

# Phase 3 stops at step 2 when dpkg says build-essential is installed but cc is gone from PATH.
nocc="${tmp}/nocc"; mkdir -p "$nocc"; ln -sf "$(command -v grep)" "$(command -v make)" "$nocc/"
out="$(probe 'dpkg() { echo "ii  x"; }; PATH='"$nocc"'; p3_s2_dependencies; echo "RC=$? R=${PHASE3_RESULTS[*]}"')"
[[ "$out" == *"RC=1 R=failed"* && "$out" == *"--reinstall gcc"* ]] || fail "p3_s2 must fail when cc is missing: ${out}"
out="$(probe 'dpkg() { echo "ii  x"; }; p3_s2_dependencies; echo "RC=$? R=${PHASE3_RESULTS[*]}"')"
[[ "$out" == *"RC=0 R=skipped"* ]] || fail "p3_s2 must skip when everything is present: ${out}"

# A failure stays on screen: die() reports without waiting, and a menu item waits for Enter once,
# after its summary, so returning to the menu (which clears the screen) never hides the failure.
# read -p shows its prompt only on a terminal, so a read wrapper prints the prompt it was given.
show_prompts='read() { [[ "$1" == -rp ]] && echo "PROMPT: $2"; builtin read "$@"; }'
out="$(probe "${show_prompts}"'; die "boom" </dev/null; echo "RC=$?"')"
[[ "$out" == *"boom"* && "$out" == *"RC=1"* && "$out" != *"Press Enter"* ]] || fail "die must report and return 1 without a prompt: ${out}"
out="$(printf '\n\n\n\n' | probe "${show_prompts}"'
clear() { :; }; print_box() { :; }; print_env() { :; }; ask_baseline() { return 1; }
p1_s1() { die "repo step failed"; PHASE1_RESULTS+=("failed"); }; p1_s2() { echo "RAN p1_s2"; }
p1_summary() { echo "SUMMARY"; }
run_phase1; echo "RC=$?"')"
[[ "$out" != *"RAN p1_s2"* ]] || fail "a failed critical step must stop Phase 1: ${out}"
[[ "$(grep -c 'Press Enter to return to the menu' <<<"$out")" -eq 1 ]] || fail "Phase 1 stop: one menu prompt expected: ${out}"
tail_out="${out#*Menu 1 stopped}"
[[ "$out" == *"SUMMARY"*"Menu 1 stopped"* && "$tail_out" == *"Press Enter to return to the menu"* ]] \
    || fail "Phase 1 stop: summary and stop reason must come before the menu prompt: ${out}"
out="$(probe "${show_prompts}"'; command() { return 1; }; print_box() { :; }; run_loadsweep </dev/null; echo "RC=$?"')"
[[ "$out" == *"cyclictest not found"*"Press Enter to return to the menu"* \
    && "$(grep -c 'Press Enter to return to the menu' <<<"$out")" -eq 1 ]] \
    || fail "menu 4 refusal: the reason, then one menu prompt: ${out}"

# Menu 4 is a loaded test: a load generator that is gone makes the rate it ran under an error, never a pass.
# cyclictest is a function that always measures 5 µs; stress-ng is a stub on PATH.
sweep() {  # sweep <stress-ng body> [cyclictest extra code] [ps output: psr rtprio ppid pid comm]
    printf '#!/usr/bin/env bash\n%s\n' "$1" >"${tmp}/bin/stress-ng"; chmod 0755 "${tmp}/bin/stress-ng"
    rm -f "${tmp}/stress-ran"
    probe "${show_prompts}"'
print_box() { :; }; booted_isol_cpu() { echo 0; }; ask_duration() { echo 1; }; compute_hk() { HK_MASK=0; HK_NCPU=1; }
ethercat() { return 1; }; rm -f "'"${tmp}"'/calls"
ps() { printf "%s\n" '"'${3:-}'"' | grep -v "^$" || true; }
cyclictest() { echo x >>"'"${tmp}"'/calls"; n=$(wc -l <"'"${tmp}"'/calls"); '"${2:-}"'
    printf "%s\n" "000005 100" "# Avg Latencies: 00005" "# Max Latencies: 00005" "# Histogram Overflows: 00000"; }
run_loadsweep </dev/null; echo "RC=$?"'
    rm -f "${tmp}/bin/stress-ng"
}
out="$(sweep 'exit 0')"
[[ "$out" == *"load generator stopped"* && "$out" != *"All rates pass"* && "$out" == *"Some rates failed"* ]] \
    || fail "menu 4: a load generator that exits at once must not pass: ${out}"
out="$(sweep 'exec sleep 60' '[[ $n -eq 2 ]] && kill -KILL -- "-${HK_LOAD_PIDS[0]}" 2>/dev/null; sleep 0.2')"
[[ "$(grep -c 'kHz.*pass ' <<<"$out")" -eq 1 && "$out" == *"load generator stopped"* && "$out" != *"All rates pass"* ]] \
    || fail "menu 4: a load generator that exits mid-sweep must fail that rate: ${out}"
out="$(sweep 'exec sleep 60')"
[[ "$out" == *"All rates pass"* ]] || fail "menu 4: a live load and 5 µs must pass: ${out}"

# Menu 4 must not run beside a live control loop: a user RT thread on the isolated CPU refuses the
# sweep before any load or cyclictest starts (menu 2 step 12 rule); kernel threads (ppid 2) do not.
out="$(sweep 'touch "'"${tmp}"'/stress-ran"; exec sleep 60' '' '0 98 1234 1240 FieldbusCtrl')"
[[ "$out" == *"An RT thread is running on CPU 0 (FieldbusCtrl"* && "$out" != *"kHz"*"pass"* \
    && ! -e "${tmp}/calls" && ! -e "${tmp}/stress-ran" ]] \
    || fail "menu 4: a user RT thread on the isolated CPU must refuse the sweep: ${out}"
out="$(sweep 'exec sleep 60' '' '0 99 2 50 migration/0')"
[[ "$out" == *"All rates pass"* ]] || fail "menu 4: kernel RT threads must not block the sweep: ${out}"

# A failed write is a failed step: ethercat.conf here (a directory in its place makes the redirect fail).
rm -f "${tmp}/ethercat.conf"; mkdir "${tmp}/ethercat.conf"
out="$(probe 'EC_MACS=(00:11:22:33:44:55); EC_INTERFACES=(eth1); EC_DEVICE_MODULES=generic
    p3_s6_configure; echo "RC=$? RESULT=${PHASE3_RESULTS[-1]}"')"
[[ "$out" == *"RC=1 RESULT=failed"* && "$out" != *"Configured"* ]] || fail "p3_s6_configure: a failed write must fail: ${out}"
rmdir "${tmp}/ethercat.conf"

# Step 7 skips only an existing AND enabled clocks unit: a disabled one (or one whose enable failed) is enabled again.
clocks() { probe 'p1_header() { :; }; p1_s7 </dev/null; echo "RESULT=${PHASE1_RESULTS[-1]}"'; }
rm -f "${tmp}/jetson-clocks.service" "${tmp}/disabled.txt"
[[ "$(clocks)" == *RESULT=done* && -f "${tmp}/jetson-clocks.service" ]] || fail "p1_s7 fresh: $(clocks)"
[[ "$(clocks)" == *RESULT=skipped* ]] || fail "p1_s7: existing + enabled must skip: $(clocks)"
echo jetson-clocks.service >"${tmp}/disabled.txt"
rm -f "${tmp}/systemctl.log"
[[ "$(clocks)" == *RESULT=done* ]] && grep -q 'enable jetson-clocks.service' "${tmp}/systemctl.log" \
    || fail "p1_s7: existing but disabled must enable again: $(clocks)"
rm -f "${tmp}/disabled.txt" "${tmp}/jetson-clocks.service"

# Optional RT packages are added once, however often Phase 1 runs in one session.
out="$(probe 'apt-cache() { return 0; }; base=${#RT_PACKAGES[@]}; resolve_rt_packages >/dev/null
    once=${#RT_PACKAGES[@]}; resolve_rt_packages >/dev/null; echo "B=$base O=$once T=${#RT_PACKAGES[@]}"')"
[[ "$out" =~ O=([0-9]+)\ T=([0-9]+) && "${BASH_REMATCH[1]}" == "${BASH_REMATCH[2]}" ]] \
    || fail "resolve_rt_packages must not add a package twice: ${out}"

# Leftovers of older wizards that would still do harm are removed on a re-run:
# - menu 1: an old kernel.sched_rt_runtime_us line in sysctl.conf (RT throttling off at every boot),
#   other lines kept, the default written back;
# - menu 3 step 9: the standalone native-bind unit and ethercat.service.d/10-native-bind.conf
#   (a second ExecStartPost: the rebind ran twice), with the drop-in directory only when it is left
#   empty; and the NIC IRQ tuning unit, script and drop-in (on the generic backend it pinned the NIC
#   IRQ to a CPU chosen at install time, which a later menu 1 can make the isolated CPU).
printf '%s\n' 'vm.swappiness=10' 'kernel.sched_rt_runtime_us=-1' 'net.core.rmem_max=1' >"${tmp}/sysctl.conf"
printf '%s\n' '#!/usr/bin/env bash' 'echo "sysctl $*" >>"'"${tmp}"'/sysctl.log"' >"${tmp}/bin/sysctl"
chmod 0755 "${tmp}/bin/sysctl"
probe 'cleanup_legacy' >/dev/null
[[ "$(cat "${tmp}/sysctl.conf")" == $'vm.swappiness=10\nnet.core.rmem_max=1' ]] \
    || fail "cleanup_legacy must remove only the sched_rt_runtime_us line: $(cat "${tmp}/sysctl.conf")"
grep -qx "sysctl -w kernel.sched_rt_runtime_us=950000" "${tmp}/sysctl.log" \
    || fail "cleanup_legacy must write back the default: $(cat "${tmp}/sysctl.log" 2>/dev/null)"
mkdir -p "${tmp}/ethercat.service.d" "${tmp}/plem-ethercat-nic-irq-tune.service.d"
printf '[Service]\nExecStartPost=/usr/local/sbin/plem_ethercat_native_bind.sh\n' >"${tmp}/ethercat.service.d/10-native-bind.conf"
: >"${tmp}/ethercat.service.d/20-site.conf"   # someone else's drop-in: kept, and so is the directory
: >"${tmp}/plem-ethercat-native-bind.service"
: >"${tmp}/plem-ethercat-nic-irq-tune.service"
: >"${tmp}/plem_ethercat_nic_irq_tune.sh"
printf '[Unit]\nWants=plem-ethercat-native-bind.service\n' >"${tmp}/plem-ethercat-nic-irq-tune.service.d/10-native-bind.conf"
: >"${tmp}/systemctl.log"
probe 'remove_old_native_bind_units' >/dev/null
[[ ! -e "${tmp}/ethercat.service.d/10-native-bind.conf" && -e "${tmp}/ethercat.service.d/20-site.conf" ]] \
    || fail "remove_old_native_bind_units must remove only its own drop-in: $(ls "${tmp}/ethercat.service.d" 2>&1)"
[[ ! -e "${tmp}/plem-ethercat-native-bind.service" ]] || fail "remove_old_native_bind_units must remove the standalone unit"
[[ ! -e "${tmp}/plem-ethercat-nic-irq-tune.service" && ! -e "${tmp}/plem_ethercat_nic_irq_tune.sh" \
   && ! -e "${tmp}/plem-ethercat-nic-irq-tune.service.d" ]] \
    || fail "remove_old_native_bind_units must remove the NIC IRQ tuning unit, script and drop-in: $(ls "${tmp}")"
grep -qx "systemctl disable --now plem-ethercat-nic-irq-tune.service" "${tmp}/systemctl.log" \
    || fail "remove_old_native_bind_units must stop the NIC IRQ tuning unit: $(cat "${tmp}/systemctl.log")"

# --- Menu 5: USB adapter drivers ---------------------------------------------------------
# probe_val <expression> [setup code] → the expression's stdout alone (the wizard's exit trap writes terminal codes after it).
probe_val() {
    local o
    o="$(probe "${2:-}
echo \"<\$($1)>\"")" || true
    o="${o#*<}"; echo "${o%>*}"
}

# The upstream tag comes from the header tree's Makefile (not `make kernelversion`, which adds -rt-tegra);
# SUBLEVEL 0 is tag vX.Y, and a Makefile without the three numbers (a wrapper) is refused.
for c in "5|15|148|5.15.148" "6|8|0|6.8" "6|8|12|6.8.12"; do
    IFS='|' read -r mv mp ms want <<<"$c"
    printf '%s\n' '# SPDX-License-Identifier: GPL-2.0' "VERSION = ${mv}" "PATCHLEVEL = ${mp}" "SUBLEVEL = ${ms}" \
        'EXTRAVERSION = -rt-tegra' 'NAME = x' 'SUBLEVEL = 99' >"${tmp}/Makefile.k"
    out="$(probe_val "kernel_upstream_version '${tmp}/Makefile.k'")"
    [[ "$out" == "$want" ]] || fail "kernel_upstream_version must be ${want}: ${out}"
done
printf '%s\n' 'include /usr/src/linux-headers-x/Makefile' >"${tmp}/Makefile.k"
for f in Makefile.k nonexistent; do
    out="$(probe_val "kernel_upstream_version '${tmp}/${f}' && echo OK || echo NO")"
    [[ "$out" == NO ]] || fail "kernel_upstream_version: ${f} (a wrapper Makefile, a missing file) must fail: ${out}"
done

# Local includes: only #include "name" from the same directory, in order.
printf '%s\n' '// SPDX-License-Identifier: GPL-2.0+' '#include <linux/usb.h>' '#include "pl2303.h"' '  #  include "a.h" /* x */' \
    '#include "../up.h"' '// #include "c.h"' >"${tmp}/src.c"
out="$(probe_val "local_includes '${tmp}/src.c' | paste -sd' '")"
[[ "$out" == "pl2303.h a.h" ]] || fail "local_includes: ${out}"

# Kernel source vs an error page: the first line must be an SPDX tag.
printf '%s\n' '/* SPDX-License-Identifier: GPL-2.0+ */' '#define X 1' >"${tmp}/ok.h"
printf '%s\n' '<!DOCTYPE html>' '<html><body>SPDX-License-Identifier</body></html>' >"${tmp}/page.html"
: >"${tmp}/empty.c"
for c in "ok.h|Y" "src.c|Y" "page.html|N" "empty.c|N" "nonexistent|N"; do
    IFS='|' read -r f want <<<"$c"
    out="$(probe_val "is_kernel_source '${tmp}/${f}' && echo Y || echo N")"
    [[ "$out" == "$want" ]] || fail "is_kernel_source ${f} must be ${want}: ${out}"
done

# vermagic: the first word must be exactly the release.
for c in "5.15.148-rt-tegra SMP preempt_rt mod_unload modversions aarch64|5.15.148-rt-tegra|Y" \
         "5.15.148-tegra SMP mod_unload|5.15.148-rt-tegra|N" "5.15.148-rt-tegra-x SMP|5.15.148-rt-tegra|N" \
         "|5.15.148-rt-tegra|N" "5.15.148-rt-tegra|5.15.148-rt-tegra|Y" "|.|N"; do
    IFS='|' read -r vm rel want <<<"$c"
    out="$(probe_val "vermagic_matches '${vm}' '${rel}' && echo Y || echo N")"
    [[ "$out" == "$want" ]] || fail "vermagic_matches '${vm}' '${rel}' must be ${want}: ${out}"
done

# Driver state: =y built in, =m in-tree module, otherwise a module modinfo finds for this release, else missing.
# modinfo is a function: usb.vermagic holds what it reports for "-k <release> -F vermagic <name>".
modinfo_stub='modinfo() { [[ "$1 $3 $4" == "-k -F vermagic" ]] || return 9; [[ -s "'"${tmp}"'/usb.vermagic" ]] || return 1; cat "'"${tmp}"'/usb.vermagic"; }'
for c in "CONFIG_USB_SERIAL_CH341=y||built-in" "CONFIG_USB_SERIAL_CH341=m||in-tree" \
         "# CONFIG_USB_SERIAL_CH341 is not set|R SMP|installed" "# CONFIG_USB_SERIAL_CH341 is not set|R2 SMP|missing" \
         "# CONFIG_USB_SERIAL_CH341 is not set||missing" "CONFIG_USB_SERIAL_CH341X=y||missing"; do
    IFS='|' read -r line vm want <<<"$c"
    if [[ -n "$vm" ]]; then echo "$vm" >"${tmp}/usb.vermagic"; else : >"${tmp}/usb.vermagic"; fi
    printf '%s\n' CONFIG_USB_SERIAL=y "$line" CONFIG_X=m >"${tmp}/usb.config"
    out="$(probe_val "usb_driver_state ch341 CONFIG_USB_SERIAL_CH341 R <'${tmp}/usb.config'" "$modinfo_stub")"
    [[ "$out" == "$want" ]] || fail "usb_driver_state '${line}' modinfo '${vm}' must be ${want}: ${out}"
done

# Menu 5 end to end with every system command stubbed (nothing reaches /lib/modules or the network):
# gs_usb built in and ch341 installed are skipped; pl2303 is fetched with its local header, built, checked,
# installed, and depmod runs once; afterwards modinfo finds it.
cat >"${tmp}/usb_stubs.sh" <<EOF
clear() { :; }; print_env() { :; }; uname() { echo R; }; kernel_headers_match() { :; }
zcat() { printf '%s\n' \${USB_KCONFIG:-CONFIG_CAN_GS_USB=y '# CONFIG_USB_SERIAL_CH341 is not set' '# CONFIG_USB_SERIAL_PL2303 is not set'}; }
kernel_upstream_version() { echo 5.15.148; }
curl() {
    local o="" url=""
    while [[ \$# -gt 0 ]]; do case "\$1" in -o) o="\$2"; shift ;; http*) url="\$1" ;; esac; shift; done
    echo "\$url" >>"${tmp}/usb.urls"
    if [[ -n "\${USB_HTML:-}" ]]; then echo "<html>" >"\$o"; return 0; fi
    case "\$url" in
        *pl2303.c\?h=v5.15.148) printf '%s\n' '// SPDX-License-Identifier: GPL-2.0' '#include "pl2303.h"' >"\$o" ;;
        *pl2303.h\?h=v5.15.148) printf '%s\n' '/* SPDX-License-Identifier: GPL-2.0 */' >"\$o" ;;
        *) return 22 ;;
    esac
}
make() {
    echo "make \$*" >>"${tmp}/usb.log"
    local m; for m in "\$@"; do [[ "\$m" == M=* ]] && : >"\${m#M=}/pl2303.ko"; done
    [[ -z "\${USB_MAKE_FAIL:-}" ]]
}
modinfo() {
    case "\$*" in
        "-F vermagic "*.ko) echo "\${USB_KO_VM:-R SMP}" ;;
        "-k R -F vermagic ch341") echo "R SMP" ;;
        "-k R -F vermagic pl2303") [[ -e "${tmp}/usb.depmod" ]] && echo "R SMP" ;;
        *) return 1 ;;
    esac
}
install() { echo "install \$*" >>"${tmp}/usb.log"; }
depmod() { echo "depmod \$*" >>"${tmp}/usb.log"; : >"${tmp}/usb.depmod"; }
EOF
usb_run() {  # usb_run [extra shell code] → menu 5 output without colors, then R=<results>
    rm -f "${tmp}/usb.log" "${tmp}/usb.urls" "${tmp}/usb.depmod"
    probe "source '${tmp}/usb_stubs.sh'; ${1:-}
run_usb_drivers <<<\$'\n\n'; echo \"R=\${USB_RESULTS[*]}\"" | sed 's/\x1b\[[0-9;]*m//g'
}
out="$(usb_run)"
[[ "$out" == *"R=skipped skipped done"* ]] || fail "menu 5: built-in and installed skip, missing is built: ${out}"
kurl="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial"
[[ "$(cat "${tmp}/usb.urls")" == "${kurl}/pl2303.c?h=v5.15.148"$'\n'"${kurl}/pl2303.h?h=v5.15.148" ]] \
    || fail "menu 5 must fetch pl2303.c and its local header at the tag: $(cat "${tmp}/usb.urls")"
grep -q '^make -C /lib/modules/R/build M=.*/pl2303 modules$' "${tmp}/usb.log" || fail "menu 5 make: $(cat "${tmp}/usb.log")"
grep -q '^install -D -m 0644 .*/pl2303/pl2303.ko /lib/modules/R/updates/plem/pl2303.ko$' "${tmp}/usb.log" \
    || fail "menu 5 install: $(cat "${tmp}/usb.log")"
[[ "$(grep -c '^depmod' "${tmp}/usb.log")" == 1 ]] && grep -qx 'depmod R' "${tmp}/usb.log" \
    || fail "menu 5 must run depmod R once: $(cat "${tmp}/usb.log")"
# A run with nothing missing downloads and builds nothing.
out="$(USB_KCONFIG='CONFIG_CAN_GS_USB=y CONFIG_USB_SERIAL_CH341=m CONFIG_USB_SERIAL_PL2303=y' usb_run)"
[[ "$out" == *"R=skipped skipped skipped"* && ! -e "${tmp}/usb.urls" && ! -e "${tmp}/usb.log" ]] \
    || fail "menu 5 with nothing missing must change nothing: ${out}"
# Failures name what failed and how to fix it, on the spot and in the summary, and install nothing.
for c in "USB_HTML|is not kernel source" "USB_MAKE_FAIL|Build of pl2303" "USB_KO_VM|was built for 'R2'"; do
    IFS='|' read -r var msg <<<"$c"
    out="$(export "${var}=R2 SMP"; usb_run)"
    [[ "$out" == *"R=skipped skipped failed"* && "$(grep -c "$msg" <<<"$out")" -ge 2 && "$out" == *"Fix: "* ]] \
        || fail "menu 5 failure (${var}) must be reported with the reason and a fix, on the spot and in the summary: ${out}"
    ! grep -qE '^(install|depmod)' "${tmp}/usb.log" 2>/dev/null || fail "menu 5 failure (${var}) must install nothing: $(cat "${tmp}/usb.log")"
done
# depmod that does not make the module findable is a failure too.
out="$(usb_run 'depmod() { echo "depmod $*" >>"'"${tmp}"'/usb.log"; }')"
[[ "$out" == *"R=skipped skipped failed"* && "$out" == *"Fix: "* ]] || fail "menu 5: a module modprobe cannot find must fail: ${out}"
# Prerequisites: a failed header check fails every missing driver before any download.
out="$(usb_run 'kernel_headers_match() { return 1; }')"
[[ "$out" == *"R=skipped skipped failed"* && ! -e "${tmp}/usb.urls" ]] || fail "menu 5: failed header check: ${out}"

# Menu 2 reports each driver from the kernel config and modinfo; menu 5 is in the main menu.
p2_usb="$(sed -n '/^    # USB adapter drivers/,/^    done$/p' <<<"$p2_body")"
grep -qF 'usb_driver_state' <<<"$p2_usb" || fail "run_phase2 USB driver block not found: ${p2_usb}"
echo "R SMP" >"${tmp}/usb.vermagic"
out="$(probe "${modinfo_stub}; zcat() { printf '%s\n' CONFIG_CAN_GS_USB=y CONFIG_USB_SERIAL_CH341=m; }; k=R
modinfo() { [[ \"\$*\" == '-k R -F vermagic pl2303' ]] && return 1; echo 'R SMP'; }
p2_check() { echo \"P2 \$1|\$2|\$3\"; }; f() {
${p2_usb}
}; f")"
[[ "$out" == *"P2 USB driver gs_usb|pass|"* && "$out" == *"P2 USB driver ch341|pass|"* && "$out" == *"P2 USB driver pl2303|warn|"*"menu 5"* ]] \
    || fail "Phase 2 USB drivers: ${out}"
grep -qF 'run_usb_drivers' <<<"$(sed -n '/^main_menu() {$/,/^}$/p' "$WIZ")" || fail "main_menu does not offer run_usb_drivers"

echo "PASS: test_setup"
