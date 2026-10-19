#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# The `plem` dispatcher (tools/plem) RUN against a fake PATH: dispatch, argument passing, the command list,
# the no-argument start with and without plem-default, and the 127 answer for a command that is not installed.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLEM="${HERE}/plem"
fails=0
fail() { echo "FAIL: $*" >&2; fails=$((fails + 1)); }

[[ -x "$PLEM" ]] || { echo "FAIL: ${PLEM} missing or not executable" >&2; exit 1; }

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
a="${tmp}/a" b="${tmp}/dir with space"
mkdir -p "$a" "$b"
# Each fake command prints its name and each argument in brackets.
mk() {  # mk <dir> <file name> [mode]
    printf '#!/bin/sh\nprintf "%s" "%s"; for x in "$@"; do printf " [%%s]" "$x"; done; echo\n' "$2" > "$1/$2"
    chmod "${3:-0755}" "$1/$2"
}
mk "$a" plem-drive
mk "$b" plem-drive          # shadowed: the first on PATH answers
mk "$b" plem-alpha
mk "$a" plem-off 0644       # not executable — not a command
mk "$a" plem-Bad            # not a command name
mk "$a" plem-x.bak
mkdir "${a}/plem-dir"       # a directory — not a command
FAKE_PATH="${a}:${b}:/usr/bin:/bin"

run() {  # run <PATH> <args...> → globals out err rc
    local p="$1"; shift
    out="$(PATH="$p" "$PLEM" "$@" 2>"${tmp}/err")"; rc=$?
    err="$(cat "${tmp}/err")"
}

# dispatch + argument passing (spaces, empty, option-looking) — unchanged
run "$FAKE_PATH" drive run 'my file.yaml' '' --plot
[[ $rc -eq 0 && "$out" == "plem-drive [run] [my file.yaml] [] [--plot]" ]] \
    || fail "dispatch/args: rc=${rc} out='${out}'"
[[ "$out" != *"dir with space"* ]] || fail "the first plem-drive on PATH must answer"
run "$FAKE_PATH" alpha --help
[[ $rc -eq 0 && "$out" == "plem-alpha [--help]" ]] || fail "dispatch in a dir with a space: rc=${rc} out='${out}'"

# the exit code of the command is the dispatcher's
printf '#!/bin/sh\nexit 7\n' > "${a}/plem-seven"; chmod 0755 "${a}/plem-seven"
run "$FAKE_PATH" seven
[[ $rc -eq 7 ]] || fail "exit code not passed through: ${rc}"
rm -f "${a}/plem-seven"

# list: --help, -h, help — deduplicated, sorted, only executable files with command names
for h in --help -h help; do
    run "$FAKE_PATH" "$h"
    listed="$(sed -n 's/^  \([a-z0-9-]*\)$/\1/p' <<<"$out" | paste -sd' ')"
    [[ $rc -eq 0 && "$listed" == "alpha drive" ]] || fail "plem ${h}: rc=${rc} listed='${listed}' out='${out}'"
    [[ "$out" == *"depend on the installed packages"* ]] || fail "plem ${h}: no note that the commands depend on the packages"
done

# no argument, no plem-default: the list, rc 0 — another command (plem-beta) is not started by name
mk "$b" plem-beta
run "$FAKE_PATH"
[[ $rc -eq 0 && "$out" == *"Commands on this device:"* && "$out" == *"  drive"* ]] \
    || fail "plem without plem-default: rc=${rc} out='${out}'"
# no argument, plem-default installed: it runs, with no argument
mk "$b" plem-default
run "$FAKE_PATH"
[[ $rc -eq 0 && "$out" == "plem-default" ]] || fail "plem with plem-default: rc=${rc} out='${out}'"
# ...and it is not a command: --help does not list it (nor offer `plem default`)
run "$FAKE_PATH" --help
listed="$(sed -n 's/^  \([a-z0-9-]*\)$/\1/p' <<<"$out" | paste -sd' ')"
[[ $rc -eq 0 && "$listed" == "alpha beta drive" && "$out" != *"plem default"* ]] \
    || fail "plem --help with plem-default: rc=${rc} listed='${listed}' out='${out}'"
# nothing installed at all
run "/usr/bin:/bin"
[[ $rc -eq 0 && "$out" == *"No plem commands are installed on this device."* ]] \
    || fail "plem with nothing installed: rc=${rc} out='${out}'"

# a command that is not installed: one line on stderr, 127, nothing on stdout
for v in bogus off Bad x.bak dir ../a/plem-drive; do
    run "$FAKE_PATH" "$v" arg
    [[ $rc -eq 127 && -z "$out" && "$err" == "plem ${v}: not installed on this device (no plem-${v})" ]] \
        || fail "plem ${v}: rc=${rc} out='${out}' err='${err}'"
done

# an option before any command
run "$FAKE_PATH" --dry kill
[[ $rc -eq 2 && -z "$out" && "$err" == *"unknown option '--dry'"* ]] || fail "plem --dry: rc=${rc} err='${err}'"

# empty PATH entries are skipped (no command from the current directory)
mk "$tmp" plem-cwd
( cd "$tmp" && PATH=":${a}" "$PLEM" cwd >/dev/null 2>&1 ); cwd_rc=$?
[[ $cwd_rc -eq 127 ]] || fail "an empty PATH entry must not find ./plem-cwd (rc=${cwd_rc})"

if [[ $fails -ne 0 ]]; then
    echo "FAIL: ${fails} plem dispatcher check(s)" >&2
    exit 1
fi
echo "PASS: plem dispatcher — dispatch, arguments, list, no-argument start, 127 for a missing command"
