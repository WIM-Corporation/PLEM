#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# setup/rt/setup.sh, device reporting: the report holds only step names and results, and the question comes
# before anything else runs. Needs no root or hardware: plem-fleet is a stub, the wizard a copy without its entry point.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WIZ="${HERE}/setup.sh"
fail() { echo "FAIL: $*" >&2; exit 1; }

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
grep -qx '#  Entry point' "$WIZ" || fail "entry point marker missing"
sed -e '/^#  Entry point$/,$d' -e "s|^CMD_LOG=\$(mktemp /tmp/|CMD_LOG=\$(mktemp ${tmp}/|" "$WIZ" >"${tmp}/wizard.sh"

cat >"${tmp}/fleet.py" <<STUB
import sys
open("${tmp}/calls", "a").write(" ".join(sys.argv[1:]) + "\n" + sys.stdin.read())
STUB

run() {  # run <bash commands>: in a shell with the wizard's functions, plem-fleet stubbed
    bash -c "trap - EXIT; source '${tmp}/wizard.sh'; trap - EXIT; FLEET_TOOL='${tmp}/fleet.py'; $1"
}

# --- the entry point asks before the menu ---
grep -q $'^fleet_consent\nmain_menu$' <(tail -2 "$WIZ" | tr -d '\r') 2>/dev/null \
    || [[ "$(tail -2 "$WIZ" | head -1)" == "fleet_consent" ]] || fail "fleet_consent does not run before main_menu"

# --- nothing ran: nothing is reported ---
run 'fleet_report'
[[ ! -e "${tmp}/calls" ]] || fail "reported although no step ran"

# --- names and results, never the message ---
run 'p2_check "RT permission" warn "alice is not in the realtime group" >/dev/null
     PHASE1_RESULTS=(ok failed); PHASE3_RESULTS=(skipped); CT_MAX_US=12 CT_AVG_US=3
     fleet_report'
grep -qx 'report-install' "${tmp}/calls" || fail "report-install not called: $(cat "${tmp}/calls")"
grep -qx '1|Add RT repository|ok' "${tmp}/calls" || fail "menu 1 step missing: $(cat "${tmp}/calls")"
grep -qx '1|apt update|failed' "${tmp}/calls" || fail "menu 1 result missing"
grep -qx '2|RT permission|warn' "${tmp}/calls" || fail "menu 2 check missing"
grep -qx '3|Detect interface|skipped' "${tmp}/calls" || fail "menu 3 step missing"
grep -qx 'cyclictest|12|3' "${tmp}/calls" || fail "cyclictest missing"
if grep -q alice "${tmp}/calls"; then fail "a check's message reached the report"; fi

# --- a failing reporter does not change the wizard's outcome ---
rm -f "${tmp}/calls"
run 'FLEET_TOOL=/nonexistent; PHASE1_RESULTS=(ok); fleet_report; echo survived' | grep -q survived \
    || fail "a missing tool ended the wizard"
run "printf 'import sys; sys.exit(3)' >'${tmp}/bad.py'; FLEET_TOOL='${tmp}/bad.py'; PHASE1_RESULTS=(ok); fleet_report; echo survived" | grep -q survived \
    || fail "a failing tool ended the wizard"

# --- the real screen, in a real terminal ---
python3 "$(dirname "${BASH_SOURCE[0]}")/fleet_consent_pty.py" "$WIZ" || fail "consent screen"

echo "PASS: test_fleet_wizard"
