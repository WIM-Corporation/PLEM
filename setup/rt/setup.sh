#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
set -uo pipefail

# This wizard's directory, resolved once at startup: a relative $0 would resolve against a later cd.
WIZARD_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
readonly WIZARD_DIR

# ============================================================================
#  Jetson RT Kernel Setup Wizard
#  Targets: Jetson AGX Orin / Orin NX / Orin Nano (has run on AGX Orin only)
# ============================================================================

readonly RT_REPO_FILE="/etc/apt/sources.list.d/nvidia-l4t-apt-source.list"
# The rt-kernel repo uses the same suite name as common/som (for example, r39.2).
# Hardcoding it breaks on every JetPack release, so read it from the device's L4T apt source.
# Filled in by resolve_rt_repo().
RT_REPO_SUITE="" RT_REPO_LINE=""
readonly EXTLINUX="/boot/extlinux/extlinux.conf"
readonly RT_FREEZE_PREF="/etc/apt/preferences.d/plem-rt-freeze.pref"
# Phase 1 asks before disabling unused services. --baseline makes the default answer yes.
BASELINE_DEFAULT=false APPLY_BASELINE=false

# Isolated CPU: the one isolated at boot, else the last CPU (Phase 1 can change it).
# RT_OTHER_CPUS = all possible CPUs minus the isolated one. Not online CPUs: in a low-power
# mode some CPUs stay offline until the MAXN reboot and must still be in irqaffinity.
# The kernel ignores offline CPUs in the mask.
TOTAL_CPUS=$(( $(cat /sys/devices/system/cpu/possible | awk -F- '{print $NF}') + 1 ))

# Expand a cpulist like "0-3,6,8-11" to "0 1 2 3 6 8 9 10 11"
parse_cpu_list() {
    local spec="$1" part a b i _p out=()
    IFS=',' read -ra _p <<< "$spec"
    for part in "${_p[@]}"; do
        if [[ "$part" == *-* ]]; then
            a="${part%-*}"; b="${part#*-}"
            for (( i=a; i<=b; i++ )); do out+=("$i"); done
        elif [[ -n "$part" ]]; then
            out+=("$part")
        fi
    done
    echo "${out[@]}"
}

# A CPU list as a set: sorted, each CPU once (so 11,10 and 10-11,10 equal 10-11).
cpu_set() { parse_cpu_list "$1" | tr ' ' '\n' | sort -nu | paste -sd' '; }

# Currently online CPUs (space separated). Falls back to the possible range if /sys/online is missing.
# Note: use only for runtime measurement and load test. For persistent settings (boot parameters)
# use cpu_possible_cores (the online set changes with the power mode).
cpu_online_cores() {
    parse_cpu_list "$(cat /sys/devices/system/cpu/online 2>/dev/null || echo "0-$(( TOTAL_CPUS - 1 ))")"
}

# All possible CPUs (space separated), for persistent settings such as boot parameters.
cpu_possible_cores() {
    parse_cpu_list "0-$(( TOTAL_CPUS - 1 ))"
}

# Derive the housekeeping CPUs mask (RT_OTHER_CPUS) and boot parameters (RT_PARAMS) from RT_ISOL_CPU.
# Call again after changing the isolated CPU to update them all.
compute_rt_topology() {
    local c keep=() cores; read -ra cores <<< "$(cpu_possible_cores)"
    for c in "${cores[@]}"; do [[ "$c" == "$RT_ISOL_CPU" ]] || keep+=("$c"); done
    RT_OTHER_CPUS="$(IFS=,; echo "${keep[*]}")"
    RT_PARAMS="isolcpus=managed_irq,domain,${RT_ISOL_CPU} irqaffinity=${RT_OTHER_CPUS}"
}

# The CPU isolated at boot, or empty if none. If several CPUs are isolated (e.g. "10-11"), the last one.
booted_isol_cpu() {
    parse_cpu_list "$(cat /sys/devices/system/cpu/isolated 2>/dev/null)" | awk '{print $NF}'
}

# User RT threads on CPU $1 (names, space-separated; empty if none). Kernel threads (ppid 2) are
# excluded. A measurement there would disturb such a thread (e.g. a running control loop) or measure it.
iso_rt_threads() {
    ps -eLo psr=,rtprio=,ppid=,pid=,comm= \
        | awk -v c="$1" '$1==c && $2!="-" && $3!=2 && $4!=2 {$1=$2=$3=$4=""; sub(/^ +/,""); print}' | sort -u | tr '\n' ' '
}

RT_ISOL_CPU="$(booted_isol_cpu)"
[[ -n "$RT_ISOL_CPU" ]] || RT_ISOL_CPU=$(( TOTAL_CPUS - 1 ))
compute_rt_topology

RT_PACKAGES=(
    nvidia-l4t-rt-kernel
    nvidia-l4t-rt-kernel-headers
    nvidia-l4t-rt-kernel-oot-modules
    nvidia-l4t-display-rt-kernel
)

# Optional packages in the L4T 39.2 (JetPack 7.2.1) rt-kernel repo: add them to install and hold
# only when they actually exist in the repo index (resolve_rt_packages).
RT_PACKAGES_OPTIONAL=(
    nvidia-l4t-rt-kernel-nvgpu
    nvidia-l4t-rt-kernel-openrm
)

# --- Colors and symbols ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[1;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m'

SYM_OK="${GREEN}✔${NC}"
SYM_SKIP="${YELLOW}→${NC}"
SYM_FAIL="${RED}✘${NC}"
SYM_LOCK="${DIM}○${NC}"
SYM_READY="${CYAN}●${NC}"

# --- Log ---
CMD_LOG=$(mktemp /tmp/rt-wizard-XXXXXX.log)

# --- Utility functions ---
info()  { echo -e "         ${GREEN}✔${NC} $*"; }
skip()  { echo -e "         ${YELLOW}→${NC} $*"; }
warn()  { echo -e "         ${YELLOW}⚠${NC} $*"; }
fail()  { echo -e "         ${RED}✘${NC} $*"; }
# Reports a failure and returns 1; the caller decides whether to go on. Each menu item waits for
# Enter once, after its summary, so the failure stays on screen until the operator has read it.
die()   {
    fail "$*"
    echo -e "         ${DIM}Log: ${CMD_LOG}${NC}"
    echo ""
    return 1
}
detail(){ echo -e "         ${DIM}$*${NC}"; }

run_cmd() {
    local desc="$1" rc; shift
    "$@" >> "$CMD_LOG" 2>&1; rc=$?
    if [[ $rc -ne 0 ]]; then
        fail "${desc} failed (exit: ${rc})"
        echo -e "         ${DIM}── Recent log ──${NC}"
        tail -5 "$CMD_LOG" | while IFS= read -r line; do
            echo -e "         ${DIM}${line}${NC}"
        done
        return 1
    fi
    return 0
}

# --- Spinner ---
SPIN_PID=""
SPIN_CHARS="⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"

_spinner() {
    local msg="$1" i=0
    while true; do
        printf "\r         %s %s" "${SPIN_CHARS:i%${#SPIN_CHARS}:1}" "$msg"
        i=$(( i + 1 ))
        sleep 0.1
    done
}

spin_start() { _spinner "$1" & SPIN_PID=$!; }
spin_stop() {
    if [[ -n "$SPIN_PID" ]]; then
        kill "$SPIN_PID" 2>/dev/null
        wait "$SPIN_PID" 2>/dev/null || true
        printf "\r\033[K"
        SPIN_PID=""
    fi
}

cleanup() {
    spin_stop
    tput cnorm 2>/dev/null
    fleet_report
    # Keep a log with content: a failed step printed its path, and the user quits with q (exit 0) afterwards.
    if [[ -s "$CMD_LOG" ]]; then echo -e "  ${DIM}Log: ${CMD_LOG}${NC}" >&2; else rm -f "$CMD_LOG"; fi
}
trap 'cleanup' EXIT

# --- Box drawing ---
print_box() {
    local text="$1"
    local width=${2:-50}
    local pad=$(( (width - 2 - ${#text}) / 2 ))
    local pad_r=$(( width - 2 - ${#text} - pad ))
    echo ""
    echo -e "  ${BOLD}┌$(printf '─%.0s' $(seq 1 $((width-2))))┐${NC}"
    echo -e "  ${BOLD}│$(printf ' %.0s' $(seq 1 $pad))${text}$(printf ' %.0s' $(seq 1 $pad_r))│${NC}"
    echo -e "  ${BOLD}└$(printf '─%.0s' $(seq 1 $((width-2))))┘${NC}"
}

# --- Environment detection ---
DEVICE_MODEL="" L4T_VER="" CURRENT_KERNEL="" MEM_GB=""
RT_INSTALLED=false RT_BOOTED=false

detect_env() {
    DEVICE_MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo "Unknown")
    L4T_VER=$(dpkg -l nvidia-l4t-core 2>/dev/null | awk '/^[hi]i/{print $3}' | cut -d'-' -f1 || echo "Unknown")
    CURRENT_KERNEL=$(uname -r)
    MEM_GB=$(awk '/MemTotal/{printf "%.0f", $2/1024/1024}' /proc/meminfo)
    dpkg -l nvidia-l4t-rt-kernel 2>/dev/null | grep -q "^[hi]i" && RT_INSTALLED=true || RT_INSTALLED=false
    [[ "$CURRENT_KERNEL" == *"-rt-"* ]] && RT_BOOTED=true || RT_BOOTED=false
}

print_env() {
    local rt_boot_status
    if $RT_BOOTED; then rt_boot_status="${GREEN}RT kernel${NC}";
    else rt_boot_status="${YELLOW}stock kernel${NC}"; fi
    local rt_pkg_status
    if $RT_INSTALLED; then rt_pkg_status="${GREEN}installed${NC}";
    else rt_pkg_status="${YELLOW}not installed${NC}"; fi

    echo ""
    echo -e "  ${DIM}Device${NC}          ${DEVICE_MODEL}"
    echo -e "  ${DIM}L4T version${NC}     ${L4T_VER}"
    echo -e "  ${DIM}Current kernel${NC}  ${CURRENT_KERNEL} (${rt_boot_status}${NC})"
    echo -e "  ${DIM}Memory${NC}          ${MEM_GB}GB"
    echo -e "  ${DIM}RT packages${NC}     ${rt_pkg_status}"
}

# ============================================================================
#  Cursor menu
# ============================================================================

draw_menu() {
    local selected=$1; shift
    local items=("$@")

    echo "" >/dev/tty
    for (( i=0; i<${#items[@]}; i+=3 )); do
        local idx=$(( i / 3 ))
        local label="${items[$i]}"
        local status="${items[$((i+1))]}"
        local enabled="${items[$((i+2))]}"
        local cursor="  "

        [[ $idx -eq $selected ]] && cursor="${CYAN}▸${NC} "

        if [[ "$enabled" == "true" ]]; then
            echo -e "  ${cursor}${BOLD}${label}${NC}  ${status}" >/dev/tty
        else
            echo -e "  ${cursor}${DIM}${label}  ${status}${NC}" >/dev/tty
        fi
    done

    echo "" >/dev/tty
    echo -e "  ${DIM}↑↓ move  Enter select  q quit${NC}" >/dev/tty
}

select_menu() {
    local items=("$@")
    local count=$(( ${#items[@]} / 3 ))
    local selected=0
    local menu_lines=$(( count + 3 ))

    tput civis 2>/dev/null >/dev/tty

    draw_menu $selected "${items[@]}"

    while true; do
        IFS= read -rsn1 key </dev/tty
        case "$key" in
            $'\x1b')
                read -rsn2 -t 0.1 key2 </dev/tty
                case "$key2" in
                    '[A') (( selected > 0 )) && (( selected-- )) ;;
                    '[B') (( selected < count - 1 )) && (( selected++ )) ;;
                esac
                ;;
            '')
                local en_idx=$(( selected * 3 + 2 ))
                if [[ "${items[$en_idx]}" == "true" ]]; then
                    tput cnorm 2>/dev/null >/dev/tty
                    echo "$selected"
                    return
                fi
                ;;
            'q'|'Q')
                tput cnorm 2>/dev/null >/dev/tty
                echo "-1"
                return
                ;;
        esac

        printf "\033[${menu_lines}A" >/dev/tty
        draw_menu $selected "${items[@]}"
    done
}

# Select the RT CPU to isolate with the cursor. Lists every possible CPU (in a low-power mode,
# offline CPUs come online after the MAXN reboot, so they must be selectable; offline is
# shown only as a tag) and tags current / last (recommended) / cpu0 (not recommended).
# On cancel (q), keep the current RT_ISOL_CPU.
# Return only the selected CPU number on stdout (all other UI goes to /dev/tty).
select_isolation_core() {
    local cores online_pad; read -ra cores <<< "$(cpu_possible_cores)"
    online_pad=" $(cpu_online_cores) "
    local items=() c tag last=$(( TOTAL_CPUS - 1 ))
    for c in "${cores[@]}"; do
        tag=""
        [[ "$c" == "$RT_ISOL_CPU" ]] && tag="${GREEN}[current]${NC} "
        [[ "$c" == "$last" ]] && tag="${tag}${DIM}(last, recommended)${NC} "
        [[ "$c" == "0" ]] && tag="${tag}${YELLOW}(cpu0, not recommended)${NC}"
        [[ "$online_pad" == *" $c "* ]] || tag="${tag}${DIM}(offline now · online after MAXN reboot)${NC}"
        items+=("CPU $c" "$tag" "true")
    done
    echo "" >/dev/tty
    echo -e "  ${BOLD}Select the RT CPU to isolate${NC} ${DIM}(↑↓ move · Enter select · q keep)${NC}" >/dev/tty
    local sel; sel=$(select_menu "${items[@]}")
    if [[ "$sel" -ge 0 ]]; then echo "${cores[$sel]}"; else echo "$RT_ISOL_CPU"; fi
}

# ============================================================================
#  Phase 1 prerequisites
# ============================================================================

# RT throttling deliberately keeps the kernel default (950 ms per 1 s). RT control loops that
# sleep with clock_nanosleep every cycle never reach this limit, and if an RT thread runs away,
# the remaining 50 ms lets the per-CPU kernel threads (rcuc, ksoftirqd) on the isolated CPU run, so an RCU stall
# does not freeze the whole device. The wizard does not write this value; it only checks it.
readonly RT_RUNTIME_DEFAULT=950000

# Whether all deep idle states (state1+) on the isolated CPU are disabled. jetson_clocks disables them
# (it disables every idle state on every CPU). True for a CPU with no deep state.
isol_deep_idle_disabled() {  # <cpu>
    local f
    for f in /sys/devices/system/cpu/cpu${1}/cpuidle/state[1-9]*/disable; do
        [[ -f "$f" ]] || continue
        [[ "$(cat "$f" 2>/dev/null)" == "1" ]] || return 1
    done
    return 0
}

phase1_is_complete() {
    $RT_BOOTED \
        && [[ -f /etc/systemd/system/jetson-clocks.service ]] \
        && grep -q "isolcpus=" /proc/cmdline 2>/dev/null
}

phase2_is_complete() {
    # Quick check of the key items only (reads the effect, not whether a setting exists)
    $RT_BOOTED \
        && [[ -n "$(cat /sys/devices/system/cpu/isolated 2>/dev/null)" ]] \
        && [[ "$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null)" == "$RT_RUNTIME_DEFAULT" ]] \
        && [[ "$(awk '/SwapTotal/{print $2}' /proc/meminfo)" -eq 0 ]] \
        && systemctl is-active jetson-clocks.service &>/dev/null \
        && isol_deep_idle_disabled "$(booted_isol_cpu)" \
        && nvpps_lock_ok \
        && nvpmodel -q 2>/dev/null | grep -q "MAXN"
}

# ============================================================================
#  Phase 1: Install RT kernel
# ============================================================================

PHASE1_NAMES=(
    "Add RT repository" "apt update" "Install RT packages" "Package version hold"
    "Power mode MAXN" "Boot parameters" "jetson_clocks service" "Disable swap/zram"
    "Disable irqbalance" "Install rt-tests"
    "Wi-Fi tuning" "Crash logging" "Disable unused services" "nvpps TSC unlock"
    "RT permission" "RT initrd modules"
)
PHASE1_RESULTS=()

p1_header() {
    local n=$1 total=${#PHASE1_NAMES[@]}
    echo ""
    echo -e "  ${BLUE}${BOLD}[${n}/${total}]${NC} ${BOLD}${PHASE1_NAMES[$((n-1))]}${NC}"
}

p1_summary() {
    local width=53
    echo ""
    echo -e "  ${BOLD}┌$(printf '─%.0s' $(seq 1 $((width-2))))┐${NC}"
    echo -e "  ${BOLD}│                  Install summary                  │${NC}"
    echo -e "  ${BOLD}├──────┬─────────────────────────┬──────────────────┤${NC}"
    printf "  ${BOLD}│${NC} ${DIM}Step${NC} ${BOLD}│${NC} ${DIM}%-23s${NC} ${BOLD}│${NC} ${DIM}%-16s${NC} ${BOLD}│${NC}\n" "Item" "Result"
    echo -e "  ${BOLD}├──────┼─────────────────────────┼──────────────────┤${NC}"
    for i in "${!PHASE1_NAMES[@]}"; do
        local r="${PHASE1_RESULTS[$i]:-unknown}" sym label
        case "$r" in
            done)    sym="$SYM_OK";   label="done"    ;;
            skipped) sym="$SYM_SKIP"; label="skipped" ;;
            failed)  sym="$SYM_FAIL"; label="failed"  ;;
            *)       sym="?";         label="unknown" ;;
        esac
        printf "  ${BOLD}│${NC}  %2d  ${BOLD}│${NC} %-23s ${BOLD}│${NC} %b %-14s ${BOLD}│${NC}\n" \
            "$((i+1))" "${PHASE1_NAMES[$i]}" "$sym" "$label"
    done
    echo -e "  ${BOLD}└──────┴─────────────────────────┴──────────────────┘${NC}"
    echo ""
}

# --- Phase 1 step functions ---

# Use the L4T apt suite configured on the device for the rt-kernel repo as is (r39.2 for JetPack 7.2.1).
# The suite differs per release line. With a mismatched suite apt update
# still passes, but the kernel version does not match the installed userspace.
resolve_rt_repo() {
    RT_REPO_SUITE="$(awk '$1 == "deb" && $2 ~ /repo\.download\.nvidia\.com\/jetson\/common/ { print $3; exit }' \
        "$RT_REPO_FILE" 2>/dev/null)"
    [[ -n "$RT_REPO_SUITE" ]] || return 1
    RT_REPO_LINE="deb https://repo.download.nvidia.com/jetson/rt-kernel ${RT_REPO_SUITE} main"
}

# Add each optional package that exists in the repo index (call after apt update).
# Uses the exit code of apt-cache show; parsing apt-cache policy depends on format and locale.
resolve_rt_packages() {
    local pkg
    for pkg in "${RT_PACKAGES_OPTIONAL[@]}"; do
        [[ " ${RT_PACKAGES[*]} " == *" ${pkg} "* ]] && continue   # already added by an earlier run this session
        if apt-cache show "$pkg" &>/dev/null; then
            RT_PACKAGES+=("$pkg")
            detail "Optional package added: ${pkg}"
        else
            detail "Optional package not found: ${pkg} (not provided by this L4T repo)"
        fi
    done
}

# Active rt-kernel deb lines in RT_REPO_FILE, one "<line number> <suite>" per line. Accepts leading
# blanks, an [option] block and a trailing slash on the URL; commented lines are not active.
rt_repo_lines() {
    awk '{ s = $0; sub(/^[ \t]+/, "", s); n = split(s, f, /[ \t]+/)
           if (f[1] != "deb") next
           i = 2; if (f[i] ~ /^\[/) { while (i < n && f[i] !~ /\]$/) i++; i++ }
           if (f[i] ~ /^https?:\/\/repo\.download\.nvidia\.com\/jetson\/rt-kernel\/?$/) print NR, f[i + 1] }' \
        "$RT_REPO_FILE" 2>/dev/null
}

# True when an active rt-kernel deb line with the expected suite is present.
rt_repo_registered() {
    rt_repo_lines | awk -v suite="$RT_REPO_SUITE" '$2 == suite { f = 1 } END { exit !f }'
}

p1_s1() {
    p1_header 1
    [[ ! -f "$RT_REPO_FILE" ]] && { die "$RT_REPO_FILE not found. Check JetPack"; PHASE1_RESULTS+=("failed"); return; }
    if ! resolve_rt_repo; then
        die "No L4T suite found in ${RT_REPO_FILE} (no jetson/common entry)"
        PHASE1_RESULTS+=("failed"); return
    fi
    detail "L4T suite: ${RT_REPO_SUITE}"
    # An active rt-kernel line for another suite (left from an earlier JetPack) would mix kernel
    # releases, so comment it out.
    local other; other=$(rt_repo_lines | awk -v suite="$RT_REPO_SUITE" \
        '$2 != suite { printf "%ss|^|# disabled by the RT setup wizard (other suite): |;", $1 }')
    if [[ -n "$other" ]]; then
        if sed -i "$other" "$RT_REPO_FILE"; then
            detail "Disabled rt-kernel line(s) for another suite"
        else
            warn "Could not disable rt-kernel line(s) for another suite in ${RT_REPO_FILE}"
        fi
    fi
    if rt_repo_registered; then
        skip "Already registered"; PHASE1_RESULTS+=("skipped")
    else
        [[ -z "$(tail -c1 "$RT_REPO_FILE")" ]] || echo >> "$RT_REPO_FILE"   # file without a final newline
        echo "$RT_REPO_LINE" >> "$RT_REPO_FILE" || { die "Could not write $RT_REPO_FILE"; PHASE1_RESULTS+=("failed"); return; }
        info "Added"; PHASE1_RESULTS+=("done")
    fi
}

p1_s2() {
    p1_header 2
    spin_start "Updating package lists..."
    # Stop on apt update errors (RT repo 404, expired GPG key, no network) so step 3
    # never installs from a stale index.
    if apt update -qq >> "$CMD_LOG" 2>&1; then
        spin_stop; info "Update done"; PHASE1_RESULTS+=("done")
    else
        spin_stop
        die "apt update failed: check RT repo access / GPG key (${RT_REPO_LINE})"
        PHASE1_RESULTS+=("failed")
    fi
}

p1_s3() {
    p1_header 3
    resolve_rt_packages
    local all=true
    for pkg in "${RT_PACKAGES[@]}"; do
        if dpkg -l "$pkg" 2>/dev/null | grep -q "^[hi]i"; then
            detail "├─ ${pkg} ... ${GREEN}installed${NC}"
        else
            detail "├─ ${pkg} ... ${YELLOW}not installed${NC}"; all=false
        fi
    done
    if ! $all; then
        spin_start "Installing packages... (takes several minutes)"
        if apt install -y "${RT_PACKAGES[@]}" >> "$CMD_LOG" 2>&1; then
            spin_stop; info "Packages installed"
        else
            spin_stop; die "Install failed"; PHASE1_RESULTS+=("failed"); return
        fi
    fi
    # Check the RT kernel files on disk, not just apt's exit code (uname shows the stock
    # kernel until reboot). Runs on the already-installed path too.
    local rt_mod_dir; rt_mod_dir="$(ls -d /lib/modules/*-rt-tegra 2>/dev/null | head -1 || true)"
    if [[ -f /boot/Image.real-time && -n "$rt_mod_dir" ]]; then
        info "RT kernel artifacts present (/boot/Image.real-time · ${rt_mod_dir##*/})"
        if $all; then PHASE1_RESULTS+=("skipped"); else PHASE1_RESULTS+=("done"); fi
    else
        [[ -f /boot/Image.real-time ]] || fail "verify: /boot/Image.real-time missing"
        [[ -n "$rt_mod_dir" ]]         || fail "verify: /lib/modules/*-rt-tegra missing"
        die "apt reported success for the RT kernel, but the artifacts are missing. A reboot would return to the stock kernel. Check the log and retry."
        PHASE1_RESULTS+=("failed"); return
    fi
}

# nvidia-l4t packages that a simulated full-upgrade would install, upgrade, or remove (space separated). Must be
# empty for a complete hold. If apt fails the simulation itself (broken deps etc.), print a marker so it does not read as "clean".
_l4t_upgrade_leaks() {
    local out
    out=$(apt-get -s -o Debug::NoLocking=1 full-upgrade 2>&1) || { echo "<apt simulation failed>"; return; }
    awk '($1 == "Inst" || $1 == "Remv") && $2 ~ /^nvidia-l4t-/ {print $1" "$2}' <<< "$out" | tr '\n' ' '
}

p1_s4() {
    p1_header 4

    # Hold every installed L4T package. Holding only kernel and bootloader lets `apt full-upgrade`
    # upgrade or remove the L4T userspace (optee, cuda, init …) to the next release, mixing releases with the kernel
    # (the RT kernel comes from the rt repo) and bootloader (TOS). Packages whose names differ per release
    # (rt-kernel-nvgpu in 39.2, etc.) are caught by the prefix too.
    l4t_hold_scan
    if [[ -n "$L4T_BROKEN" ]]; then
        die "L4T packages not fully installed: ${L4T_BROKEN}(fix with 'sudo dpkg --configure -a', then re-run)"
        PHASE1_RESULTS+=("failed"); return
    fi
    local hold_packages=("${L4T_INSTALLED[@]}")
    if (( ${#hold_packages[@]} == 0 )); then
        die "No installed nvidia-l4t packages: install the RT kernel (menu 1, step 3) first"
        PHASE1_RESULTS+=("failed"); return
    fi

    # Full hold: `apt-mark hold` can be ignored by `apt full-upgrade` / `apt install --reinstall`.
    # So pin the *exact installed version* with apt preferences (Pin-Priority
    # 1001: blocks upgrades and also unintended downgrades), and add hold as well.
    # A version that has disappeared from the repo still gets 1001 from the dpkg status.
    local pref_file="$RT_FREEZE_PREF" pkg
    # Skip only when every installed package is held and in the pin file (l4t_hold_scan, the rule Phase 2
    # checks). A newly installed package, or one held by hand but not pinned, makes the next run write both again.
    if [[ -z "${L4T_UNHELD}${L4T_UNPINNED}" ]]; then
        skip "Already held (${#hold_packages[@]} nvidia-l4t packages held + version pin)"; PHASE1_RESULTS+=("skipped"); return
    fi

    # 1) apt-mark hold
    if ! apt-mark hold "${hold_packages[@]}" >> "$CMD_LOG" 2>&1; then
        die "apt-mark hold failed"; PHASE1_RESULTS+=("failed"); return
    fi

    # 2) Exact version pin
    local pref_body ver
    pref_body="# RT setup wizard: all installed nvidia-l4t-* packages pinned to their installed versions (Pin-Priority 1001)
# To regenerate, delete this file and re-run the wizard."
    for pkg in "${hold_packages[@]}"; do
        ver=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null)
        [[ -n "$ver" ]] || { die "Version lookup failed for ${pkg}"; PHASE1_RESULTS+=("failed"); return; }
        pref_body+="

Package: $pkg
Pin: version $ver
Pin-Priority: 1001"
    done
    if ! printf '%s\n' "$pref_body" > "$pref_file" 2>>"$CMD_LOG"; then
        die "Failed to write version pin file: $pref_file"
        PHASE1_RESULTS+=("failed"); return
    fi
    chmod 0644 "$pref_file" 2>/dev/null || true

    # 3) readback: hold and pin applied, and a simulated full-upgrade does not touch L4T.
    local held_now; held_now=$(apt-mark showhold 2>/dev/null || echo "")
    local verify_fail=false
    for pkg in "${hold_packages[@]}"; do
        grep -qxF "$pkg" <<< "$held_now" || { verify_fail=true; fail "verify: ${pkg} hold missing"; }
    done
    l4t_hold_scan
    [[ -z "$L4T_UNPINNED" ]] || { verify_fail=true; fail "verify: not pinned to the installed version: ${L4T_UNPINNED}"; }
    spin_start "Checking the hold with a simulated full-upgrade... (tens of seconds)"
    local leaks; leaks=$(_l4t_upgrade_leaks)
    spin_stop
    [[ -z "$leaks" ]] || { verify_fail=true; fail "verify: L4T packages touched by full-upgrade: ${leaks}"; }
    if $verify_fail; then
        die "Hold verification failed: re-check apt-mark showhold / ${pref_file}"
        PHASE1_RESULTS+=("failed"); return
    fi

    info "${#hold_packages[@]} nvidia-l4t packages held + version pinned"
    detail "Versions frozen on apt upgrade, full-upgrade, --reinstall (downgrades blocked too)"
    detail "Pin file: ${pref_file}"
    PHASE1_RESULTS+=("done")
}

p1_s5() {
    p1_header 5
    # The MAXN mode ID can differ per board, so read it from the conf (AGX Orin = 0).
    local conf="/etc/nvpmodel.conf" status_file="/var/lib/nvpmodel/status" id cur_default m
    id=$(sed -n 's/^< POWER_MODEL ID=\([0-9]*\) NAME=MAXN[A-Z_]* >.*/\1/p' "$conf" 2>/dev/null | head -1)
    if [[ -z "$id" ]]; then
        die "No MAXN power mode found in ${conf}"; PHASE1_RESULTS+=("failed"); return
    fi
    # Default mode: boot into MAXN even if the status file is lost or nvpmodel is reset.
    cur_default=$(sed -n 's/^< PM_CONFIG DEFAULT=\([0-9]*\) >.*/\1/p' "$conf" | head -1)
    local changed=false
    if [[ "$cur_default" != "$id" ]]; then
        sed -i --follow-symlinks "s/^< PM_CONFIG DEFAULT=[0-9]* >/< PM_CONFIG DEFAULT=${id} >/" "$conf"
        detail "${conf} default mode ${cur_default:-?} → ${id}"; changed=true
    fi
    # Current mode: nvpmodel -m shows a reboot prompt and does not save if declined, so
    # write the status file directly to apply it on the next boot.
    m=$(nvpmodel -q 2>/dev/null | grep -oP 'NV Power Mode: \K\w+' || true)
    if [[ "$m" != MAXN* ]]; then
        # Same format nvpmodel writes (no newline).
        printf 'pmode:%04d' "$id" > "$status_file" \
            || { die "Failed to write ${status_file}"; PHASE1_RESULTS+=("failed"); return; }
        detail "Current ${m:-unknown} → MAXN (ID ${id}) on reboot"; changed=true
    fi
    if [[ "$(sed -n 's/^< PM_CONFIG DEFAULT=\([0-9]*\) >.*/\1/p' "$conf" | head -1)" != "$id" ]]; then
        die "Failed to change the default mode in ${conf}"; PHASE1_RESULTS+=("failed"); return
    fi
    if $changed; then info "MAXN set (applies on reboot)"; PHASE1_RESULTS+=("done")
    else skip "Already MAXN (current and default mode)"; PHASE1_RESULTS+=("skipped"); fi
}

# Print the LINUX image of an extlinux LABEL (empty if none), so a label counts as RT
# by the image it boots, not by its name.
_extlinux_label_image() {
    awk -v want="$1" '
        /^LABEL /                { cur=$2; next }
        cur==want && $1=="LINUX" { print $2; exit }
    ' "$EXTLINUX"
}

# Print the APPEND line number of every label that boots /boot/Image.real-time. Match by
# image, not label name: third-party packages (e.g. a camera driver) add a label copied from DEFAULT
# and make it DEFAULT, so that label needs the parameters too. "MENU LABEL" lines have
# $1 == MENU and do not match.
_rt_append_lines() {
    awk '
        /^[[:space:]]*LABEL /                                { inlab=1; rt=0 }
        inlab && $1=="LINUX" && $2=="/boot/Image.real-time"  { rt=1 }
        inlab && rt && $1=="APPEND"                          { print NR; inlab=0 }
    ' "$EXTLINUX"
}

p1_s6() {
    p1_header 6
    [[ ! -f "$EXTLINUX" ]] && { die "$EXTLINUX not found"; PHASE1_RESULTS+=("failed"); return; }
    if [[ ! -f "${EXTLINUX}.bak" ]]; then
        cp "$EXTLINUX" "${EXTLINUX}.bak" || { die "Could not back up $EXTLINUX"; PHASE1_RESULTS+=("failed"); return; }
        detail "Backup: ${EXTLINUX}.bak"
    fi

    local -a _lines=(); mapfile -t _lines < <(_rt_append_lines)
    if (( ${#_lines[@]} == 0 )); then
        die "No APPEND line: no label boots the RT image (check the RT kernel package install)"
        PHASE1_RESULTS+=("failed"); return
    fi

    local ln line changed=0 _mod
    for ln in "${_lines[@]}"; do
        _mod=false
        line="$(sed -n "${ln}p" "$EXTLINUX")"
        if ! grep -q "isolcpus=" <<< "$line"; then
            sed -i "${ln}s|$| ${RT_PARAMS}|" "$EXTLINUX"
            _mod=true
        elif ! grep -qE "isolcpus=managed_irq,domain,${RT_ISOL_CPU}( |$)" <<< "$line" \
          || ! grep -qE "irqaffinity=${RT_OTHER_CPUS}( |$)"              <<< "$line"; then
            # Isolated CPU changed on re-run, or irqaffinity= missing (a label written by hand or
            # an older wizard): set both parameters to the current values
            sed -i "${ln}s|isolcpus=[^[:space:]]*|isolcpus=managed_irq,domain,${RT_ISOL_CPU}|" "$EXTLINUX"
            if grep -q "irqaffinity=" <<< "$line"; then
                sed -i "${ln}s|irqaffinity=[^[:space:]]*|irqaffinity=${RT_OTHER_CPUS}|" "$EXTLINUX"
            else
                sed -i "${ln}s|$| irqaffinity=${RT_OTHER_CPUS}|" "$EXTLINUX"
            fi
            _mod=true
        fi
        $_mod && changed=$(( changed + 1 ))
    done

    # Verify: every RT boot label must carry the current topology
    local _ok=true
    mapfile -t _lines < <(_rt_append_lines)
    for ln in "${_lines[@]}"; do
        line="$(sed -n "${ln}p" "$EXTLINUX")"
        grep -qE "isolcpus=managed_irq,domain,${RT_ISOL_CPU}( |$)" <<< "$line" || _ok=false
        grep -qE "irqaffinity=${RT_OTHER_CPUS}( |$)"               <<< "$line" || _ok=false
    done
    if [[ "$_ok" != "true" ]]; then
        die "Apply failed. Restore from ${EXTLINUX}.bak"; PHASE1_RESULTS+=("failed"); return
    fi
    if (( changed > 0 )); then
        info "Applied (${changed} of ${#_lines[@]} RT boot labels updated)"
        detail "isolcpus=managed_irq,domain,${RT_ISOL_CPU}"
        detail "irqaffinity=${RT_OTHER_CPUS}"
        PHASE1_RESULTS+=("done")
    else
        skip "Already set (${#_lines[@]} RT boot labels)"; PHASE1_RESULTS+=("skipped")
    fi

    # Keep DEFAULT if it already boots the RT image: another tool may have made its own
    # RT label DEFAULT, and switching back would drop its FDT/overlays. Otherwise set
    # DEFAULT to real-time.
    local cur_default cur_default_img
    cur_default=$(awk '/^DEFAULT/{print $2; exit}' "$EXTLINUX")
    cur_default_img=$(_extlinux_label_image "$cur_default")
    if [[ "$cur_default_img" == "/boot/Image.real-time" ]]; then
        detail "DEFAULT (${cur_default}) already boots the RT image: kept"
    elif grep -q "^LABEL real-time" "$EXTLINUX"; then
        if grep -q '^DEFAULT ' "$EXTLINUX"; then
            sed -i 's/^DEFAULT .*/DEFAULT real-time/' "$EXTLINUX"
        else
            sed -i '1i DEFAULT real-time' "$EXTLINUX"  # no DEFAULT line: the first label would boot
        fi
        if grep -q '^DEFAULT real-time$' "$EXTLINUX"; then
            info "DEFAULT set to real-time"
        else
            warn "DEFAULT could not be set to real-time: check ${EXTLINUX}"
        fi
    else
        warn "No LABEL real-time. Check the RT package install"
    fi
}

p1_s7() {
    p1_header 7
    local svc="/etc/systemd/system/jetson-clocks.service"
    # Skip only a unit that is also enabled: a failed enable (or a later disable) is repaired by re-running.
    if [[ -f "$svc" ]] && systemctl is-enabled --quiet jetson-clocks.service 2>/dev/null; then
        skip "Already exists and enabled"; PHASE1_RESULTS+=("skipped"); return
    fi
    if ! cat > "$svc" << 'UNIT'
[Unit]
Description=Jetson Clocks
After=multi-user.target nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
UNIT
    then
        die "Failed to write ${svc}"; PHASE1_RESULTS+=("failed"); return
    fi
    run_cmd "daemon-reload" systemctl daemon-reload || { die "daemon-reload failed"; PHASE1_RESULTS+=("failed"); return; }
    run_cmd "enable" systemctl enable jetson-clocks.service || { die "enable failed"; PHASE1_RESULTS+=("failed"); return; }
    info "Created and enabled"; PHASE1_RESULTS+=("done")
}

p1_s8() {
    p1_header 8
    local found=false persist_failed=false
    # Swap still on at run time is a change too, even with the services and fstab already clean.
    [[ "$(awk '/SwapTotal/{print $2}' /proc/meminfo 2>/dev/null)" != "0" ]] && found=true
    swapoff -a 2>/dev/null || true
    # Disable swap services when present: nvzramconfig for zram and nvfb-swapfile for /swapfile.
    local svc svc_found=false
    for svc in nvzramconfig nvfb-swapfile; do
        if systemctl is-enabled "$svc" &>/dev/null; then
            run_cmd "$svc" systemctl disable --now "$svc" || warn "Failed to disable ${svc}"
            detail "├─ ${svc} disabled"; found=true; svc_found=true
        fi
    done
    $svc_found || detail "├─ no swap service, or already disabled"
    # fstab swap entries turn swap back on at boot even with the services off. Comment them out
    # instead of deleting so they can be restored (the original is backed up once).
    if grep -qE '^[^#].*[[:space:]]swap[[:space:]]' /etc/fstab 2>/dev/null; then
        if [[ -e /etc/fstab.plem-rt.bak ]] || cp /etc/fstab /etc/fstab.plem-rt.bak; then
            if sed -i -E 's|^([^#].*[[:space:]]swap[[:space:]].*)$|# [plem-rt] &|' /etc/fstab \
                && ! grep -qE '^[^#].*[[:space:]]swap[[:space:]]' /etc/fstab; then
                detail "├─ /etc/fstab swap entries commented out (backup: /etc/fstab.plem-rt.bak)"; found=true
            else
                warn "Could not comment out the swap entries in /etc/fstab (swap returns at boot)"; persist_failed=true
            fi
        else
            rm -f /etc/fstab.plem-rt.bak  # a partial copy must not pass as the backup next time
            warn "Cannot back up /etc/fstab: its swap entries are left as they are (swap returns at boot)"; persist_failed=true
        fi
    fi
    # Read back SwapTotal so a failed disable is not reported as done.
    local _swaptotal; _swaptotal=$(awk '/SwapTotal/{print $2}' /proc/meminfo 2>/dev/null || echo "?")
    if [[ "$_swaptotal" == "0" ]] && $persist_failed; then
        die "Swap is off now but /etc/fstab could not be changed: it comes back at boot (see above)"; PHASE1_RESULTS+=("failed")
    elif [[ "$_swaptotal" == "0" ]]; then
        info "Disabled (SwapTotal=0)"
        if $found; then PHASE1_RESULTS+=("done"); else PHASE1_RESULTS+=("skipped"); fi
    else
        die "Failed to disable swap (SwapTotal=${_swaptotal}kB): check swapoff and the swap services above"; PHASE1_RESULTS+=("failed")
    fi
}

p1_s9() {
    p1_header 9
    if dpkg -l irqbalance &>/dev/null; then
        if systemctl disable --now irqbalance >> "$CMD_LOG" 2>&1; then
            info "Disabled"; PHASE1_RESULTS+=("done")
        else
            die "Failed to disable irqbalance: check systemctl disable --now irqbalance"; PHASE1_RESULTS+=("failed")
        fi
    else
        skip "Not installed (Jetson default)"; PHASE1_RESULTS+=("skipped")
    fi
}

p1_s10() {
    p1_header 10
    if dpkg -l rt-tests 2>/dev/null | grep -q "^[hi]i"; then
        skip "Already installed"; PHASE1_RESULTS+=("skipped")
    else
        spin_start "Installing rt-tests..."
        if apt install -y rt-tests >> "$CMD_LOG" 2>&1; then
            spin_stop; info "Installed"; PHASE1_RESULTS+=("done")
        else
            spin_stop; warn "Install failed (install rt-tests manually before menu 2)"; PHASE1_RESULTS+=("skipped")
        fi
    fi
}

p1_s11() {
    p1_header 11

    # Detect the Wi-Fi interface
    local wifi_iface=""
    for _wdir in /sys/class/net/*/wireless; do
        [[ -d "$_wdir" ]] || continue
        wifi_iface=$(basename "$(dirname "$_wdir")")
        break
    done

    if [[ -z "$wifi_iface" ]]; then
        skip "No Wi-Fi interface"; PHASE1_RESULTS+=("skipped"); return
    fi
    detail "Interface: ${wifi_iface}"
    local changed=false

    # (a) Disable NetworkManager wifi.powersave
    local nm_conf="/etc/NetworkManager/conf.d/wifi-powersave-off.conf"
    if [[ -f "$nm_conf" ]] && grep -q "wifi.powersave = 2" "$nm_conf"; then
        detail "├─ NM powersave already disabled"
    else
        cat > "$nm_conf" << 'NMEOF'
[connection]
wifi.powersave = 2
NMEOF
        detail "├─ NM powersave disabled"; changed=true
    fi

    # (b) iw power_save off dispatcher
    local dispatcher="/etc/NetworkManager/dispatcher.d/99-wifi-powersave"
    if [[ -f "$dispatcher" ]]; then
        detail "├─ dispatcher already exists"
    else
        cat > "$dispatcher" << DISPEOF
#!/bin/bash
iw dev ${wifi_iface} set power_save off
DISPEOF
        chmod +x "$dispatcher"
        detail "├─ dispatcher created (${wifi_iface})"; changed=true
    fi

    # (c) Per-chipset modprobe.d settings (RTL8822CE only)
    local driver=""
    driver=$(basename "$(readlink -f "/sys/class/net/${wifi_iface}/device/driver")" 2>/dev/null || echo "")
    if [[ "$driver" == "rtl88x2ce" ]] || [[ "$driver" == "rtl8822ce" ]]; then
        local modprobe_conf="/etc/modprobe.d/rtl8822ce-no-powersave.conf"
        if [[ -f "$modprobe_conf" ]]; then
            detail "└─ RTL8822CE modprobe already set"
        else
            cat > "$modprobe_conf" << 'MODEOF'
# Disable all power saving for RTL8822CE WiFi (RT latency optimization)
options rtl8822ce rtw_power_mgnt=0 rtw_ips_mode=0 rtw_lps_chk_by_tp=0 rtw_max_roaming_times=0
MODEOF
            detail "└─ RTL8822CE power saving disabled"; changed=true
        fi
        # Apply at runtime right away
        for param in rtw_power_mgnt rtw_ips_mode rtw_lps_chk_by_tp; do
            local sysfs="/sys/module/rtl8822ce/parameters/${param}"
            [[ -f "$sysfs" ]] && echo 0 > "$sysfs" 2>/dev/null || true
        done
    else
        detail "└─ driver: ${driver:-unknown} (generic settings only)"
    fi

    # iw power_save off at runtime
    iw dev "$wifi_iface" set power_save off 2>/dev/null || true

    info "Wi-Fi tuning done (persistent after reboot)"
    if $changed; then PHASE1_RESULTS+=("done"); else PHASE1_RESULTS+=("skipped"); fi
}

# Older versions of this wizard turned RT throttling off in sysctl.conf at every boot. Remove that line.
# The other units and boot parameters they added do no harm and are left in place.
readonly RT_SYSCTL_CONF="/etc/sysctl.conf"
cleanup_legacy() {
    if grep -q '^kernel.sched_rt_runtime_us' "$RT_SYSCTL_CONF" 2>/dev/null; then
        sed -i '/^kernel.sched_rt_runtime_us/d' "$RT_SYSCTL_CONF"
        sysctl -w kernel.sched_rt_runtime_us="$RT_RUNTIME_DEFAULT" >> "$CMD_LOG" 2>&1 || true
        detail "Removed old sched_rt_runtime_us setting → restored default ${RT_RUNTIME_DEFAULT}"
    fi
}

# Crash logging: makes it possible to see later why a device in the field froze or rebooted.
# crash-logging.sh does the work; see its header.
crash_logging() {  # crash_logging [--check] → the tool's state line; its full output goes to CMD_LOG
    local out rc; out=$(bash "${WIZARD_DIR}/crash-logging.sh" "$@" 2>&1); rc=$?
    echo "$out" >> "$CMD_LOG"
    tail -1 <<< "$out"
    return $rc
}

p1_s12() {
    p1_header 12
    local st rc msg; st=$(crash_logging); rc=$?; msg=${st#*: }
    case "$rc:$st" in
        0:applied:*) info "${msg^}"; PHASE1_RESULTS+=("done") ;;
        0:ok:*)      skip "${msg^}"; PHASE1_RESULTS+=("skipped") ;;
        *)           die "$msg";     PHASE1_RESULTS+=("failed") ;;
    esac
}

# Service baseline (optional, see run_phase1): turn off desktop features a dedicated control computer does
# not use. gdm/graphical.target is left alone for a local GUI. Only units present on the device are touched
# (names differ per release).
BASELINE_OFF=(bluetooth.service ModemManager.service avahi-daemon.socket avahi-daemon.service
              apport.service kerneloops.service
              apt-daily.timer apt-daily-upgrade.timer fwupd-refresh.timer motd-news.timer man-db.timer
              update-notifier-download.timer update-notifier-motd.timer)
# Units that dbus activation points at directly, so they wake up again after disable, are masked
# (dbus activation of bluetooth, ModemManager, and avahi goes through the alias that disable removes).
BASELINE_MASK=(packagekit.service rtkit-daemon.service power-profiles-daemon.service)
# User session audio (desktop audio gets RT priority): masked for all users.
BASELINE_MASK_USER=(pipewire.socket pipewire.service pipewire-pulse.socket pipewire-pulse.service
                    pipewire-media-session.service wireplumber.service pulseaudio.socket pulseaudio.service)

_unit_exists()      { [[ -n "$(systemctl list-unit-files --no-legend "$1" 2>/dev/null)" ]]; }
_user_unit_exists() { [[ -n "$(systemctl --global list-unit-files --no-legend "$1" 2>/dev/null)" ]]; }

# Service baseline units that are not off yet (space separated). Must be empty to count as done.
baseline_left_on() {
    local u st
    for u in "${BASELINE_OFF[@]}"; do
        _unit_exists "$u" || continue
        st=$(systemctl is-enabled "$u" 2>/dev/null)
        [[ "$st" == disabled || "$st" == masked || "$st" == static || "$st" == indirect ]] \
            && ! systemctl is-active -q "$u" && continue
        printf '%s ' "$u"
    done
    for u in "${BASELINE_MASK[@]}"; do
        _unit_exists "$u" || continue
        [[ "$(systemctl is-enabled "$u" 2>/dev/null)" == masked ]] || printf '%s ' "$u"
    done
    for u in "${BASELINE_MASK_USER[@]}"; do
        _user_unit_exists "$u" || continue
        [[ "$(systemctl --global is-enabled "$u" 2>/dev/null)" == masked ]] || printf '%s ' "$u"
    done
}

# Heuristic: a stock install masks none of these units, so a masked one means Phase 1 applied the baseline here.
baseline_selected() {
    local u
    for u in "${BASELINE_MASK[@]}"; do
        [[ "$(systemctl is-enabled "$u" 2>/dev/null)" == masked ]] && return 0
    done
    for u in "${BASELINE_MASK_USER[@]}"; do
        [[ "$(systemctl --global is-enabled "$u" 2>/dev/null)" == masked ]] && return 0
    done
    return 1
}

p1_s13() {
    p1_header 13
    if ! $APPLY_BASELINE; then
        skip "Not selected (services left as they are)"; PHASE1_RESULTS+=("skipped"); return
    fi
    local left; left=$(baseline_left_on)
    if [[ -z "$left" ]]; then
        skip "Already disabled"; PHASE1_RESULTS+=("skipped"); return
    fi
    local u
    for u in "${BASELINE_OFF[@]}"; do
        _unit_exists "$u" || continue
        systemctl disable --now "$u" >> "$CMD_LOG" 2>&1
        # For units generated from SysV scripts (e.g. apport), disable hands off to
        # systemd-sysv-install and the --now stop is lost. Stop it separately if still running.
        systemctl is-active -q "$u" && systemctl stop "$u" >> "$CMD_LOG" 2>&1
    done
    for u in "${BASELINE_MASK[@]}"; do
        _unit_exists "$u" && systemctl mask --now "$u" >> "$CMD_LOG" 2>&1
    done
    for u in "${BASELINE_MASK_USER[@]}"; do
        _user_unit_exists "$u" && systemctl --global mask "$u" >> "$CMD_LOG" 2>&1
    done
    left=$(baseline_left_on)
    if [[ -n "$left" ]]; then
        die "Failed to disable: ${left}"; PHASE1_RESULTS+=("failed"); return
    fi
    info "Unused services, timers and desktop audio disabled"
    detail "Desktop audio is off from the next login (or reboot). To revert: systemctl enable/unmask, systemctl --global unmask"
    PHASE1_RESULTS+=("done")
}

# nvpps TSC unlock. A standalone script does the work (installed as NVPPS_TOOL so it can be
# re-run on its own); see its header.
readonly NVPPS_NODE="/proc/device-tree/bus@0/nvpps"
readonly NVPPS_KEEP="/etc/plem/nvpps-lock-keep"
readonly NVPPS_TOOL="/usr/local/sbin/plem-nvpps-tsc-unlock"

# True when the desired state matches the live DT. True on boards without the node.
nvpps_lock_ok() {
    [[ -d "$NVPPS_NODE" ]] || return 0
    if [[ -e "$NVPPS_KEEP" ]]; then [[ ! -e "${NVPPS_NODE}/ptp_tsc_sync_dis" ]]
    else [[ -e "${NVPPS_NODE}/ptp_tsc_sync_dis" ]]; fi
}

# Tool output lines with the prefix removed (last line is the status: "applied: …"). Arguments are passed to the tool as is.
nvpps_tool() {
    local out rc; out=$("$NVPPS_TOOL" "$@" 2>&1); rc=$?
    echo "$out" >> "$CMD_LOG"
    sed 's/^plem-nvpps-tsc-unlock: //' <<< "$out"
    return $rc
}

p1_s14() {
    p1_header 14
    local src="${WIZARD_DIR}/nvpps-tsc-unlock.sh"
    if ! install -m 755 "$src" "$NVPPS_TOOL"; then
        die "${src} install failed"; PHASE1_RESULTS+=("failed"); return
    fi
    local out rc st; out=$(nvpps_tool); rc=$?
    st=$(tail -1 <<< "$out")
    grep '^warning:' <<< "$out" | while read -r l; do warn "${l#warning: }"; done
    case "$rc:$st" in
        0:applied:*)        info "${st#applied: }"; PHASE1_RESULTS+=("done") ;;
        0:ok:*|0:pending:*|0:n/a:*) skip "${st#*: }"; PHASE1_RESULTS+=("skipped") ;;
        *)                  die "${st#*: }";        PHASE1_RESULTS+=("failed") ;;
    esac
}

# RT permission: members of the realtime group may use SCHED_FIFO and lock memory (99-plem-realtime.conf
# in this directory). Phase 1 adds the user who ran sudo.
readonly RT_LIMITS="/etc/security/limits.d/99-plem-realtime.conf"

readonly EC_UDEV_RULES="/etc/udev/rules.d/99-ethercat.rules"
ec_device_nodes() { compgen -G '/dev/EtherCAT[0-9]*'; }
ec_bad_nodes() { local n; for n in $1; do [[ "$(stat -c '%a %G' "$n")" == "660 realtime" ]] || printf '%s ' "$n"; done; }  # nodes not 0660 realtime

rt_user() { [[ "${SUDO_USER:-root}" != root ]] && echo "$SUDO_USER"; }   # empty when run as root directly
in_realtime() { [[ " $(id -nG "$1" 2>/dev/null) " == *" realtime "* ]]; }

# The realtime group exists and the user who ran sudo is a member. Returns 1 after die() on a failure;
# RT_MEMBERSHIP_CHANGED=true when it created the group or added the user.
realtime_membership() {
    RT_MEMBERSHIP_CHANGED=false
    local user; user=$(rt_user)
    if ! getent group realtime >/dev/null; then
        groupadd --system realtime >> "$CMD_LOG" 2>&1 || { die "Failed to create the realtime group"; return 1; }
        RT_MEMBERSHIP_CHANGED=true
    fi
    if [[ -z "$user" ]]; then
        warn "No user was added to the realtime group (the wizard runs as root, not through sudo). Run it with sudo from your own account, or run: sudo usermod -aG realtime <user>"
    elif ! in_realtime "$user"; then
        usermod -aG realtime "$user" >> "$CMD_LOG" 2>&1 || { die "Failed to add ${user} to the realtime group"; return 1; }
        RT_MEMBERSHIP_CHANGED=true
        detail "${user} added to the realtime group: log out and in again for it to take effect"
    fi
}

p1_s15() {
    p1_header 15
    local changed=false
    realtime_membership || { PHASE1_RESULTS+=("failed"); return; }
    $RT_MEMBERSHIP_CHANGED && changed=true
    if ! cmp -s "${WIZARD_DIR}/99-plem-realtime.conf" "$RT_LIMITS"; then
        install -m 644 "${WIZARD_DIR}/99-plem-realtime.conf" "$RT_LIMITS" \
            || { die "Failed to write ${RT_LIMITS}"; PHASE1_RESULTS+=("failed"); return; }
        changed=true
    fi
    if $changed; then info "realtime group and ${RT_LIMITS} set"; PHASE1_RESULTS+=("done")
    else skip "Already set"; PHASE1_RESULTS+=("skipped"); fi
}

# RT initrd modules: without them the RT kernel cannot mount a rootfs on NVMe and the device boots into the
# recovery initrd instead. rt-initrd.sh does the work; see its header.
rt_initrd() {  # rt_initrd [--check] → the tool's state line; its full output goes to CMD_LOG
    local out rc; out=$(bash "${WIZARD_DIR}/rt-initrd.sh" "$@" 2>&1); rc=$?
    echo "$out" >> "$CMD_LOG"
    tail -1 <<< "$out"
    return $rc
}

p1_s16() {
    p1_header 16
    local st rc msg; st=$(rt_initrd); rc=$?; msg=${st#*: }
    case "$rc:$st" in
        0:applied:*)     info "${msg^}"; PHASE1_RESULTS+=("done") ;;
        0:ok:*|0:n/a:*)  skip "${msg^}"; PHASE1_RESULTS+=("skipped") ;;
        *)               die "$msg";     PHASE1_RESULTS+=("failed") ;;
    esac
}

# Disabling unused services suits a dedicated control computer, not a desktop, so Phase 1 asks.
# Default yes with --baseline, or when this device already had them disabled (a re-run keeps it so).
ask_baseline() {
    local ans q="Disable unused services (Bluetooth, ModemManager, Avahi, desktop audio, background timers)?"
    if $BASELINE_DEFAULT || baseline_selected || [[ -z "$(baseline_left_on)" ]]; then
        read -rp "  ${q} (Y/n): " ans
        [[ ! "$ans" =~ ^[Nn]$ ]]
    else
        read -rp "  ${q} (y/N): " ans
        [[ "$ans" =~ ^[Yy]$ ]]
    fi
}

run_phase1() {
    clear
    print_box "Menu 1: Install RT kernel"
    print_env
    echo ""
    read -rp "  Continue? (Y/n): " confirm
    [[ "$confirm" =~ ^[Nn]$ ]] && return

    # Choose the isolated CPU (default: current RT_ISOL_CPU). The operator can change it with the cursor.
    # On change, compute_rt_topology recomputes RT_OTHER_CPUS/RT_PARAMS together.
    echo ""
    detail "CPU to isolate: CPU ${RT_ISOL_CPU}  ·  housekeeping CPUs (from possible): ${RT_OTHER_CPUS}"
    local _chg
    read -rp "  Choose the isolated CPU yourself? (y/N): " _chg
    if [[ "$_chg" =~ ^[Yy]$ ]]; then
        RT_ISOL_CPU="$(select_isolation_core)"
        compute_rt_topology
        info "Isolated CPU = CPU ${RT_ISOL_CPU}  ·  housekeeping CPUs = ${RT_OTHER_CPUS}"
    fi

    if ask_baseline; then APPLY_BASELINE=true; else APPLY_BASELINE=false; fi

    PHASE1_RESULTS=()
    # Steps 1-3 (repo, update, install) form a chain. If one fails, stop: later steps would
    # hold packages and edit boot parameters on a system without an RT kernel. Each step
    # already called die(), so only the table is shown here.
    local step
    for step in p1_s1 p1_s2 p1_s3; do
        "$step"
        if [[ "${PHASE1_RESULTS[-1]:-}" == "failed" ]]; then
            p1_summary
            fail "Menu 1 stopped: step ${step#p1_s} (${PHASE1_NAMES[${step#p1_s}-1]}) failed; the remaining steps were not run"
            echo ""
            read -rp "  Press Enter to return to the menu..." _
            return 1
        fi
    done
    # Remove legacy settings only after the critical chain passes, so a stop in steps 1-3 leaves them in place.
    cleanup_legacy
    # Independent tuning steps: continue-on-error is fine (each result in the table).
    p1_s4; p1_s5; p1_s6
    p1_s7; p1_s8; p1_s9; p1_s10; p1_s11; p1_s12; p1_s13; p1_s14; p1_s15; p1_s16
    p1_summary

    echo -e "  After reboot, verify the install in ${BOLD}menu 2${NC}."
    echo ""
    read -rp "  Reboot now? (y/N): " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        echo -e "\n  Rebooting...\n"; reboot
    else
        echo -e "\n  When ready, run ${BOLD}sudo reboot${NC}."
    fi
    echo ""
    read -rp "  Press Enter to return to the menu..." _
}

# ============================================================================
#  Phase 2: Verify setup
# ============================================================================

PHASE2_RESULTS=()

P2_LOG=()  # name|status of every check, for the device report: never msg, it can name users and paths
p2_check() {
    local name="$1" status="$2" msg="$3"
    local sym
    P2_LOG+=("${name}|${status}")
    case "$status" in
        pass) sym="${GREEN}✔${NC}"; PHASE2_RESULTS+=("pass") ;;
        fail) sym="${RED}✘${NC}"; PHASE2_RESULTS+=("fail") ;;
        warn) sym="${YELLOW}⚠${NC}"; PHASE2_RESULTS+=("warn") ;;
        info) sym="${DIM}○${NC}" ;;   # optional item: shown, not counted
    esac
    printf "  %b  %-24s %s\n" "$sym" "$name" "$msg"
}

# One pass over the nvidia-l4t-* packages, shared by Phase 1 step 4 and Phase 2 so both judge "held and pinned"
# the same way. Sets L4T_INSTALLED (array) and the space-separated lists L4T_BROKEN, L4T_UNHELD, L4T_UNPINNED.
# dpkg status letters: 1st = wanted (h = hold), 2nd = state (U/F/H/W/t = install not finished), 3rd R = reinstall required.
# A package counts as pinned only by a stanza with its installed version at Pin-Priority 1001 (what step 4 writes).
l4t_hold_scan() {
    local rows pins; rows=$(dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' 'nvidia-l4t-*' 2>/dev/null)
    L4T_BROKEN=$(awk 'substr($1,2,1) ~ /[UFHWt]/ || substr($1,3,1) == "R" {printf "%s ", $2}' <<< "$rows")
    mapfile -t L4T_INSTALLED < <(awk '$1 ~ /^[hi]i$/ {print $2}' <<< "$rows")
    L4T_UNHELD=$(awk '$1 == "ii" {printf "%s ", $2}' <<< "$rows")
    pins=$(awk '/^Package: / {p = $2; v = ""} /^Pin: version / {v = $3} /^Pin-Priority: 1001$/ && p != "" {print p, v; p = ""}' \
        "$RT_FREEZE_PREF" 2>/dev/null)
    L4T_UNPINNED=$(awk 'NR == FNR {pin[$0]; next} $1 ~ /^[hi]i$/ && !(($2 " " $3) in pin) {printf "%s ", $2}' \
        <(printf '%s\n' "$pins") <(printf '%s\n' "$rows"))
}

# Every installed nvidia-l4t-* package is held and named in the version pin file (what Phase 1 writes).
check_version_hold() {
    l4t_hold_scan
    if [[ -n "$L4T_BROKEN" ]]; then
        p2_check "Version hold" "warn" "install not finished: ${L4T_BROKEN}(run sudo dpkg --configure -a, then menu 1)"
    elif [[ ! -f "$RT_FREEZE_PREF" ]]; then
        p2_check "Version hold" "warn" "no version pin (${RT_FREEZE_PREF}). Re-run menu 1"
    elif [[ -n "$L4T_UNHELD" ]]; then
        p2_check "Version hold" "warn" "not held: ${L4T_UNHELD}(re-run menu 1)"
    elif [[ -n "$L4T_UNPINNED" ]]; then
        p2_check "Version hold" "warn" "not pinned to the installed version: ${L4T_UNPINNED}(re-run menu 1)"
    else
        p2_check "Version hold" "pass" "nvidia-l4t-* held and version-pinned"
    fi
}

run_phase2() {
    clear
    print_box "Menu 2: Verify setup (after reboot)"
    print_env
    PHASE2_RESULTS=()

    echo ""
    echo -e "  ${BOLD}Checks${NC}"
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"

    # 1. RT kernel
    local k; k=$(uname -r)
    [[ "$k" == *"-rt-"* ]] && p2_check "RT kernel boot" "pass" "$k" \
        || p2_check "RT kernel boot" "fail" "$k (not RT)"

    # 2. CPU isolation (isolcpus). Phase 2 checks this boot, so every check and test below uses the CPU
    # isolated at boot (iso_cpu). Expected is the CPU Phase 1 wrote to the RT boot label, not one picked
    # earlier in this session: a different value means the label changed and the device has not rebooted.
    local iso iso_cpu want_cpu ln
    iso=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo ""); iso_cpu=$(booted_isol_cpu)
    ln=$(_rt_append_lines 2>/dev/null | head -1)
    want_cpu=$([[ -n "$ln" ]] && sed -n "${ln}p" "$EXTLINUX" | grep -oE 'isolcpus=managed_irq,domain,[0-9,-]+' | sed 's/.*domain,//')
    if [[ -z "$iso" ]]; then p2_check "CPU isolation" "fail" "no isolation"
    elif [[ -n "$want_cpu" && "$(cpu_set "$iso")" != "$(cpu_set "$want_cpu")" ]]; then
        p2_check "CPU isolation" "warn" "CPU $iso is isolated but the RT boot label sets CPU ${want_cpu}: reboot, or re-run menu 1"
    else p2_check "CPU isolation" "pass" "CPU $iso"; fi

    # 3. Boot parameters
    # Check 2 already proved the isolcpus effect via /sys/.../isolated. Here we check that both
    # parameters exist and the applied irqaffinity value (default_smp_affinity).
    local cmd; cmd=$(cat /proc/cmdline)
    local _bootok=true _bootmsg=""
    grep -q "isolcpus=" <<< "$cmd"   || { _bootok=false; _bootmsg+="isolcpus missing "; }
    grep -q "irqaffinity=" <<< "$cmd" || { _bootok=false; _bootmsg+="irqaffinity missing "; }
    local _dsa; _dsa=$(cat /proc/irq/default_smp_affinity 2>/dev/null || echo "?")
    if ! $_bootok; then
        p2_check "Boot parameters" "fail" "${_bootmsg}"
    else
        p2_check "Boot parameters" "pass" "isolcpus+irqaffinity applied (default_smp_affinity=${_dsa})"
    fi

    # 4. Power mode
    local pm; pm=$(nvpmodel -q 2>/dev/null | grep -oP 'NV Power Mode: \K.*' || echo "unknown")
    [[ "$pm" == *"MAXN"* ]] && p2_check "Power mode" "pass" "$pm" \
        || p2_check "Power mode" "warn" "$pm"

    # 5. jetson_clocks service
    if systemctl is-active jetson-clocks.service &>/dev/null; then
        p2_check "jetson_clocks" "pass" "active"
    elif systemctl is-enabled jetson-clocks.service &>/dev/null; then
        p2_check "jetson_clocks" "warn" "enabled (not active)"
    else
        p2_check "jetson_clocks" "fail" "not set"
    fi

    # 6. Swap
    local st; st=$(awk '/SwapTotal/{print $2}' /proc/meminfo)
    if [[ "$st" -eq 0 ]]; then
        p2_check "Swap/zram" "pass" "swap=0"
    else
        p2_check "Swap/zram" "fail" "swap=${st}kB active"
    fi

    # 7. CPU idle: check that deep idle (state1+) is off on the isolated CPU (iso_cpu).
    # jetson_clocks in jetson-clocks.service turns it off (every idle state on every core).
    # WFI is still used as the cpuidle governor fallback, whatever the disable flag says.
    if [[ -z "$iso_cpu" ]]; then
        p2_check "CPU idle" "fail" "no isolated CPU"
    elif isol_deep_idle_disabled "$iso_cpu"; then
        p2_check "CPU idle" "pass" "isolated CPU ${iso_cpu} deep idle disabled (jetson_clocks)"
    else
        p2_check "CPU idle" "fail" "Deep idle states are enabled on isolated CPU ${iso_cpu}. Check jetson-clocks.service. If it is active and idle is still enabled, jetson_clocks on this platform does not disable idle"
    fi

    # 8. RT throttling: the policy is to keep the default (see the RT_RUNTIME_DEFAULT comment).
    local rr; rr=$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null || echo "?")
    if [[ "$rr" == "$RT_RUNTIME_DEFAULT" ]]; then
        p2_check "RT throttling" "pass" "default kept (${rr}µs/1s)"
    else
        local _src; _src=$(grep -ls '^[[:space:]]*kernel.sched_rt_runtime_us' "$RT_SYSCTL_CONF" /etc/sysctl.d/*.conf 2>/dev/null | tr '\n' ' ')
        p2_check "RT throttling" "warn" "${rr}. Policy is to keep the default ${RT_RUNTIME_DEFAULT} (set in: ${_src:-no sysctl file, changed at runtime})"
    fi

    # 9. irqbalance
    if ! dpkg -l irqbalance &>/dev/null; then
        p2_check "irqbalance" "pass" "not installed"
    elif ! systemctl is-active irqbalance &>/dev/null; then
        p2_check "irqbalance" "pass" "disabled"
    else
        p2_check "irqbalance" "fail" "active"
    fi

    check_version_hold

    # Crash logging. The tool's --check (no writes) decides.
    local _cl
    if _cl=$(crash_logging --check); then
        p2_check "Crash logging" "pass" "${_cl#ok: }"
    else
        p2_check "Crash logging" "warn" "${_cl#warn: }. Re-run menu 1"
    fi

    # RT initrd modules. The tool's --check (no writes) decides.
    local _ri
    if _ri=$(rt_initrd --check); then
        p2_check "RT initrd modules" "pass" "${_ri#*: }"
    else
        p2_check "RT initrd modules" "warn" "${_ri#warn: }. Re-run menu 1"
    fi

    # Unused services
    local _bl; _bl=$(baseline_left_on)
    if [[ -z "$_bl" ]]; then
        p2_check "Unused services" "pass" "unused services, timers and desktop audio off"
    elif baseline_selected; then
        p2_check "Unused services" "warn" "back on: ${_bl}(re-run menu 1)"
    else
        p2_check "Unused services" "info" "left on (optional; answer yes in menu 1 to disable)"
    fi

    # nvpps TSC unlock (boards with an nvpps node only). The tool's --check (no writes) decides.
    if [[ -d "$NVPPS_NODE" ]]; then
        if [[ ! -x "$NVPPS_TOOL" ]]; then
            p2_check "nvpps TSC unlock" "warn" "${NVPPS_TOOL} missing. Re-run menu 1"
        else
            local _st; _st=$(nvpps_tool --check | tail -1)
            local _ptp; _ptp=$(systemctl list-units --no-legend --state=active 'ptp4l*' 'phc2sys*' 2>/dev/null)
            case "$_st" in
                ok:*)
                    if [[ ! -e "$NVPPS_KEEP" && -n "$_ptp" ]]; then
                        p2_check "nvpps TSC unlock" "warn" "TSC lock off but ptp4l/phc2sys running. If this device uses PTP on the built-in RJ45 port, create ${NVPPS_KEEP} and re-run menu 1"
                    else
                        p2_check "nvpps TSC unlock" "pass" "${_st#ok: }"
                    fi ;;
                n/a:*) p2_check "nvpps TSC unlock" "pass" "${_st#n/a: }" ;;
                *)     p2_check "nvpps TSC unlock" "warn" "${_st}" ;;
            esac
        fi
    fi

    # RT permission (Phase 1 step 15). Membership is checked for the user who ran sudo.
    local _ru; _ru=$(rt_user)
    if ! getent group realtime >/dev/null; then
        p2_check "RT permission" "warn" "no realtime group. Re-run menu 1"
    elif ! cmp -s "${WIZARD_DIR}/99-plem-realtime.conf" "$RT_LIMITS"; then
        p2_check "RT permission" "warn" "${RT_LIMITS} missing or different. Re-run menu 1"
    elif [[ -n "$_ru" ]] && ! in_realtime "$_ru"; then
        p2_check "RT permission" "warn" "${_ru} is not in the realtime group. Re-run menu 1, then log in again"
    else
        p2_check "RT permission" "pass" "realtime group and limits set${_ru:+, ${_ru} is a member}"
    fi

    # EtherCAT device permission (menu 3 step 7): every device node 0660 group realtime, the user who ran sudo a member.
    local _ec_user _ec_nodes _ec_bad; _ec_user=$(rt_user)
    if ! command -v ethercat >/dev/null; then
        p2_check "EtherCAT device" "info" "EtherCAT master not installed (menu 3)"
    elif ! _ec_nodes=$(ec_device_nodes); then
        p2_check "EtherCAT device" "warn" "no /dev/EtherCAT* device: start the EtherCAT master (sudo systemctl start ethercat)"
    else
        _ec_bad=$(ec_bad_nodes "$_ec_nodes")
        if [[ -n "$_ec_bad" ]]; then
            p2_check "EtherCAT device" "warn" "${_ec_bad}does not have mode 0660 and group realtime. Re-run menu 3"
        elif [[ -n "$_ec_user" ]] && ! in_realtime "$_ec_user"; then
            p2_check "EtherCAT device" "warn" "${_ec_user} cannot open it: sudo usermod -aG realtime ${_ec_user}, then log out and in again"
        else
            p2_check "EtherCAT device" "pass" "/dev/EtherCAT* 0660 group realtime${_ec_user:+, ${_ec_user} is a member (if menu 1 or 3 just added ${_ec_user}, log out and in again: \`id -nG\` in your own shell must list realtime)}"
        fi
    fi

    # 10. Wi-Fi power save (only when Wi-Fi is present)
    local _wifi_iface=""
    for _wdir in /sys/class/net/*/wireless; do
        [[ -d "$_wdir" ]] || continue
        _wifi_iface=$(basename "$(dirname "$_wdir")")
        break
    done
    if [[ -n "$_wifi_iface" ]]; then
        local ps_status
        ps_status=$(iw dev "$_wifi_iface" get power_save 2>/dev/null | awk '{print $NF}' || echo "?")
        [[ "$ps_status" == "off" ]] \
            && p2_check "Wi-Fi power save" "pass" "${_wifi_iface}: power_save off" \
            || p2_check "Wi-Fi power save" "warn" "${_wifi_iface}: power_save ${ps_status}"
    fi

    # USB adapter drivers (menu 5 builds the missing ones). Reads the kernel config and modinfo only.
    local _kcfg _d _mod _sym _st
    _kcfg=$(zcat /proc/config.gz 2>/dev/null)
    for _d in "${USB_DRIVERS[@]}"; do
        IFS='|' read -r _mod _sym _ <<<"$_d"
        _st="$(usb_driver_state "$_mod" "$_sym" "$k" <<<"$_kcfg")"
        p2_check "USB driver ${_mod}" "$([[ $_st == missing ]] && echo warn || echo pass)" "$(usb_driver_note "$_st" "$k")"
    done

    # Verify summary
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"
    local pc=0 fc=0 wc=0
    for r in "${PHASE2_RESULTS[@]}"; do
        case "$r" in pass) (( pc++ )) ;; fail) (( fc++ )) ;; warn) (( wc++ )) ;; esac
    done
    echo ""
    echo -e "  ${GREEN}✔ ${pc} pass${NC}  ${RED}✘ ${fc} fail${NC}  ${YELLOW}⚠ ${wc} warn${NC}"

    if [[ $fc -ne 0 ]]; then
        echo ""
        echo -e "  ${RED}${BOLD}Some checks failed.${NC} Re-run menu 1 or check manually."
        echo ""
        read -rp "  Press Enter to return to the menu..." _
        return
    fi

    # === Live tests ===
    echo ""
    echo -e "  ${BOLD}Live tests${NC}"
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"

    # 11. CPU isolation test: check that device IRQs are not routed to the isolated CPU
    #     Essential kernel interrupts (IPI, arch_timer, LOC, PMU, etc.) cannot be removed even with isolcpus
    #     → count only lines with a numeric IRQ number (hardware/device IRQs), excluding arch_timer
    # /proc/interrupts has a column per online CPU only, so find the CPU's column by its header name.
    irq_device_count() {  # <cpu>
        awk -v cpu="CPU$1" '
            NR==1 { for (i = 1; i <= NF; i++) if ($i == cpu) col = i + 1; next }
            col && $1 ~ /^[0-9]+:$/ && !/arch_timer/ {sum += $col}
            END { if (col) print sum+0; else print "offline" }
        ' /proc/interrupts
    }
    local irq_before=""
    [[ -n "$iso_cpu" ]] && irq_before=$(irq_device_count "$iso_cpu")
    if [[ -z "$iso_cpu" ]]; then
        p2_check "CPU isolation test" "fail" "no isolated CPU"
    elif [[ "$irq_before" == offline ]]; then
        p2_check "CPU isolation test" "fail" "CPU ${iso_cpu} is offline"
    else
        printf "  %b  %-24s %s" "${DIM}…${NC}" "CPU isolation test" "load test on CPU ${iso_cpu}..."
        taskset -c "$iso_cpu" dd if=/dev/urandom of=/dev/null bs=1M count=100 >> "$CMD_LOG" 2>&1 || true
        sleep 1

        local irq_after; irq_after=$(irq_device_count "$iso_cpu")
        local irq_diff=$(( irq_after - irq_before ))
        printf "\r\033[K"

        if [[ $irq_diff -eq 0 ]]; then
            p2_check "CPU isolation test" "pass" "device IRQs: +${irq_diff}"
        elif [[ $irq_diff -lt 5 ]]; then
            p2_check "CPU isolation test" "pass" "device IRQs: +${irq_diff} (under 5)"
        else
            p2_check "CPU isolation test" "fail" "device IRQs: +${irq_diff} (isolation broken)"
        fi
    fi

    # 12. cyclictest: 30 s at 1 kHz on the isolated CPU (no load). The loaded measurement is menu 4.
    # Skipped while a user RT thread (e.g. a running control loop) is on the isolated CPU.
    local _rt_busy=""
    [[ -n "$iso_cpu" ]] && _rt_busy=$(iso_rt_threads "$iso_cpu")
    if [[ -n "$_rt_busy" ]]; then
        p2_check "RT latency" "warn" "An RT thread is running on CPU ${iso_cpu} (${_rt_busy}). Stop it and measure again"
    elif command -v cyclictest &>/dev/null && [[ -n "$iso_cpu" ]]; then
        printf "  %b  %-24s %s" "${DIM}…${NC}" "RT latency" "running cyclictest on CPU ${iso_cpu} for 30 s..."
        local ct_output
        ct_output=$(cyclictest -t1 -a "$iso_cpu" -p95 -i1000 -D30 --mlockall -q 2>&1) || true
        printf "\r\033[K"

        local max_lat
        max_lat=$(echo "$ct_output" | grep -oP 'Max:\s+\K[0-9]+' | head -1 || echo "")
        local avg_lat
        avg_lat=$(echo "$ct_output" | grep -oP 'Avg:\s+\K[0-9]+' | head -1 || echo "")

        if [[ -n "$max_lat" ]]; then
            CT_MAX_US=$max_lat CT_AVG_US=$avg_lat
            if [[ "$max_lat" -lt 100 ]]; then
                p2_check "RT latency" "pass" "CPU ${iso_cpu} Max: ${max_lat}µs, Avg: ${avg_lat}µs"
            elif [[ "$max_lat" -lt 500 ]]; then
                p2_check "RT latency" "warn" "CPU ${iso_cpu} Max: ${max_lat}µs, Avg: ${avg_lat}µs"
            else
                p2_check "RT latency" "fail" "CPU ${iso_cpu} Max: ${max_lat}µs (>500µs)"
            fi
        else
            p2_check "RT latency" "warn" "could not parse result"
        fi
    elif ! command -v cyclictest &>/dev/null; then
        p2_check "RT latency" "warn" "cyclictest not installed (sudo apt install rt-tests)"
    fi

    # Final summary (including live tests)
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"
    pc=0; fc=0; wc=0
    for r in "${PHASE2_RESULTS[@]}"; do
        case "$r" in pass) (( pc++ )) ;; fail) (( fc++ )) ;; warn) (( wc++ )) ;; esac
    done
    echo ""
    echo -e "  ${GREEN}✔ ${pc} pass${NC}  ${RED}✘ ${fc} fail${NC}  ${YELLOW}⚠ ${wc} warn${NC}"

    if [[ $fc -eq 0 ]]; then
        echo ""
        echo -e "  ${GREEN}${BOLD}All checks and tests passed.${NC}"
    else
        echo ""
        echo -e "  ${RED}${BOLD}Some checks failed.${NC} Check manually."
    fi

    if ! command -v cyclictest &>/dev/null; then
        echo ""
        echo -e "  ${DIM}Long-run RT performance test:${NC}"
        echo -e "  ${DIM}  sudo apt install rt-tests${NC}"
        echo -e "  ${DIM}  sudo cyclictest --mlockall --smp --priority=98 --interval=200 --distance=0${NC}"
    fi

    echo ""
    read -rp "  Press Enter to return to the menu..." _
}

# ============================================================================
#  Phase 3: Install EtherCAT master
# ============================================================================

# Pin IgH to a release tag, not a moving branch, so builds are reproducible. IgH 1.6.13 has
# a kernel 6.8 copy of igb, so the native backend builds unmodified on JetPack 7.2.1.
# When bumping, update ETHERLAB_EXPECTED_SHA and run a long test on a real bus.
readonly ETHERLAB_VERSION="1.6.13"         # git tag
readonly ETHERLAB_EXPECTED_SHA="8fd340e5c3dea8120c84256439813b27bf42c13d"  # `git ls-remote --tags ... 1.6.13` (tag object); "" to skip
readonly ETHERLAB_REPO="https://gitlab.com/etherlab.org/ethercat.git"
readonly ETHERLAB_PREFIX="/opt/etherlab"
ETHERCAT_SRC=""  # set by mktemp in p3_s3_clone

PHASE3_NAMES=(
    "Detect interface"
    "Build dependencies"
    "Download source"
    "Build"
    "Install"
    "EtherCAT config"
    "udev rules"
    "systemd service"
    "native NIC bind"
    "Start and verify"
)
PHASE3_RESULTS=()
EC_INTERFACES=()   # selected NICs → mapped in order to MASTER0, MASTER1, ...
EC_MACS=()         # MACs, 1:1 with EC_INTERFACES

# EC_IRQ_TUNE_*, EC_NATIVE_BIND_SERVICE and EC_NATIVE_BIND_DROPIN are paths of older setups that p3_s9 removes.
readonly EC_IRQ_TUNE_SCRIPT="/usr/local/sbin/plem_ethercat_nic_irq_tune.sh"
readonly EC_IRQ_TUNE_SERVICE="/etc/systemd/system/plem-ethercat-nic-irq-tune.service"
readonly EC_IRQ_TUNE_DROPIN="/etc/systemd/system/plem-ethercat-nic-irq-tune.service.d/10-native-bind.conf"
# EC_NATIVE_BIND_SCRIPT is the current rebind helper.
readonly EC_NATIVE_BIND_SCRIPT="/usr/local/sbin/plem_ethercat_native_bind.sh"
readonly EC_NATIVE_BIND_SERVICE="/etc/systemd/system/plem-ethercat-native-bind.service"
readonly EC_NATIVE_BIND_DROPIN="/etc/systemd/system/ethercat.service.d/10-native-bind.conf"

phase3_is_complete() {
    command -v ethercat &>/dev/null \
        && systemctl is-enabled ethercat.service &>/dev/null \
        && [[ $(lsmod | grep -cw "ec_master") -gt 0 ]]
}

p3_header() {
    local n=$1 total=${#PHASE3_NAMES[@]}
    echo ""
    echo -e "  ${BLUE}${BOLD}[${n}/${total}]${NC} ${BOLD}${PHASE3_NAMES[$((n-1))]}${NC}"
}

p3_summary() {
    local width=53
    echo ""
    echo -e "  ${BOLD}┌$(printf '─%.0s' $(seq 1 $((width-2))))┐${NC}"
    echo -e "  ${BOLD}│             EtherCAT install summary              │${NC}"
    echo -e "  ${BOLD}├──────┬─────────────────────────┬──────────────────┤${NC}"
    for i in "${!PHASE3_NAMES[@]}"; do
        local r="${PHASE3_RESULTS[$i]:-unknown}" sym label
        case "$r" in
            done)    sym="$SYM_OK";   label="done"   ;;
            skipped) sym="$SYM_SKIP"; label="skipped" ;;
            failed)  sym="$SYM_FAIL"; label="failed"   ;;
            *)       sym="?";         label="unknown" ;;
        esac
        printf "  ${BOLD}│${NC}  %2d  ${BOLD}│${NC} %-23s ${BOLD}│${NC} %b %-14s ${BOLD}│${NC}\n" \
            "$((i+1))" "${PHASE3_NAMES[$i]}" "$sym" "$label"
    done
    echo -e "  ${BOLD}└──────┴─────────────────────────┴──────────────────┘${NC}"
    echo ""
}

# Verify the installed kernel headers match the RUNNING kernel. Kernel modules such as
# ec_master build against headers in /lib/modules/$(uname -r)/build; if that path is missing or
# points at another kernel version, vermagic mismatches. That failure only shows up later as a
# modprobe failure at service start (and gets swallowed), so fail fast here.
kernel_headers_match() {  # <module named in the mismatch message>
    local _krel _kbuild
    _krel="$(uname -r)"
    _kbuild="/lib/modules/${_krel}/build"
    if [[ ! -e "$_kbuild" ]]; then
        die "Kernel headers missing: no ${_kbuild}. Install the headers package of the running kernel (${_krel}): nvidia-l4t-rt-kernel-headers for the RT kernel, nvidia-l4t-kernel-headers for the stock one"
        return 1
    fi
    # Compare the kernel release of the header tree the build symlink points to with the running kernel.
    local _hdr_rel=""
    if [[ -r "${_kbuild}/include/config/kernel.release" ]]; then
        _hdr_rel="$(cat "${_kbuild}/include/config/kernel.release" 2>/dev/null || echo "")"
    fi
    if [[ -n "$_hdr_rel" && "$_hdr_rel" != "$_krel" ]]; then
        die "Kernel header version mismatch: headers=${_hdr_rel} vs running kernel=${_krel}. ${1} would fail to load (vermagic)"
        return 1
    fi
    detail "Kernel headers match: ${_krel}"
}

# --- Phase 3 step functions ---

# Drivers with an IgH 1.6 native backend; others use generic. Each name is both the Linux driver and the
# IgH one (ec_<name>): ethercatctl's std_module_name maps only ccat, whose Linux driver is ec_bhf, so ccat
# (the CCAT FPGA of Beckhoff PCs) is left out. A NIC already on the native driver shows as ec_<drv>
# (e.g. ec_igb), so the ec_ prefix is stripped.
ec_native_capable() {
    case "${1#ec_}" in
        igb|igc|e1000|e1000e|r8169|8139too|e100|genet|dwmac-intel|stmmac-pci) return 0 ;;
        *) return 1 ;;
    esac
}

# Decide whether a NIC is an EtherCAT candidate. EtherCAT runs only over Ethernet
# (ARPHRD_ETHER=1), so CAN (type 280), USB gadget, virtual and empty-MAC interfaces are excluded.
ec_is_candidate() {
    local ifc="$1"
    [[ -d "/sys/class/net/${ifc}/device" ]] || return 1
    case "$ifc" in
        lo|wl*|docker*|br-*|veth*|tap*|tun*|usb*|can*) return 1 ;;
    esac
    [[ "$(cat "/sys/class/net/${ifc}/type" 2>/dev/null)" == "1" ]] || return 1
    local mac; mac="$(cat "/sys/class/net/${ifc}/address" 2>/dev/null || echo "")"
    [[ -n "$mac" && "$mac" != "00:00:00:00:00:00" ]] || return 1
    return 0
}

# Choose EC_DEVICE_MODULES from the drivers of the selected EC_INTERFACES[].
# A single native-capable driver means native; anything else (mixed/unsupported) means generic.
# IgH applies one DEVICE_MODULES to every master, so native multi-master works
# only when all ports use the same driver.
ec_resolve_backend() {
    local ifc drv
    declare -A _drvset=()
    # _native_possible: some selected NIC has a native-capable driver. Tells "fell back from
    # native" (ask first) apart from "native never possible" (proceed).
    local _native_possible=false _empty_drv=false
    for ifc in "${EC_INTERFACES[@]}"; do
        drv="$(basename "$(readlink -f "/sys/class/net/${ifc}/device/driver" 2>/dev/null)" 2>/dev/null || echo "")"
        # Normalize a NIC already bound to native (ec_igb) to the same driver as stock (igb).
        # On reinstall this stops a mix of ec_<drv> and stock ports being misread as "mixed drivers".
        drv="${drv#ec_}"
        if [[ -n "$drv" ]]; then
            _drvset["$drv"]=1
            ec_native_capable "$drv" && _native_possible=true
        else
            _empty_drv=true
        fi
    done
    if [[ "${#_drvset[@]}" -eq 1 ]] && ec_native_capable "${!_drvset[*]}"; then
        EC_DEVICE_MODULES="${!_drvset[*]}"
        return 0
    fi
    # Generic fallback. If no selected driver is native-capable (and all could be read), proceed
    # silently. Otherwise (mixed drivers, or a driver could not be read) ask first: generic goes
    # through the network stack, so cycle timing jitters more than native and an occasional frame
    # can be lost at 1 kHz (IgH docs: slower than native).
    if ! $_native_possible && ! $_empty_drv; then
        EC_DEVICE_MODULES="generic"
        detail "Using the generic backend: IgH has no native driver for the selected port(s). It works, but cycle timing jitters more than with the native driver."
        return 0
    fi
    if [[ "${#_drvset[@]}" -gt 1 ]]; then
        warn "Selected NIC drivers are mixed (${!_drvset[*]}), so the native backend cannot be used → downgrading to generic"
        detail "native multi-master works only when every selected NIC uses the same driver (e.g. a 4-port igb card)."
    elif $_empty_drv; then
        warn "Could not read some NIC drivers (empty readlink), native cannot be decided → downgrading to generic"
    fi
    detail "generic sends frames through the Linux network stack: more jitter, and an occasional frame can be lost at 1 kHz. Native (ec_<drv>) is recommended for 1 kHz."
    local _ans
    read -rp "  Use the generic backend instead of native? (y/N): " _ans </dev/tty
    if [[ ! "$_ans" =~ ^[Yy]$ ]]; then
        die "generic downgrade declined. Reselect NICs that share one native driver"
        return 1
    fi
    EC_DEVICE_MODULES="generic"
    return 0
}

# Checkbox multi-select TUI. Draws the UI on /dev/tty and prints the selected indices
# (space-separated) to stdout (same calling convention as select_menu). Prints "-1" on cancel.
# Color codes stay literal (\033...) and are interpreted once, at the final echo -e.
EC_MS_LABELS=()
EC_MS_CHECKED=()
EC_MS_CURSOR=0
ec_ms_draw() {
    local n=${#EC_MS_LABELS[@]} i mark cur
    echo "" >/dev/tty
    for (( i=0; i<n; i++ )); do
        if [[ "${EC_MS_CHECKED[$i]}" == "1" ]]; then mark="[${GREEN}x${NC}]"; else mark="[ ]"; fi
        if [[ $i -eq $EC_MS_CURSOR ]]; then cur="${CYAN}▸${NC} "; else cur="  "; fi
        echo -e "  ${cur}${mark} ${EC_MS_LABELS[$i]}" >/dev/tty
    done
    echo "" >/dev/tty
    echo -e "  ${DIM}↑↓ move  Space select/clear  a toggle all  Enter confirm  q cancel${NC}" >/dev/tty
}
ec_multiselect() {
    EC_MS_LABELS=("$@")
    local n=${#EC_MS_LABELS[@]} i key key2 allon nv out cnt
    local menu_lines=$(( n + 3 ))
    EC_MS_CHECKED=(); for (( i=0; i<n; i++ )); do EC_MS_CHECKED[$i]=0; done
    EC_MS_CURSOR=0

    tput civis 2>/dev/null >/dev/tty
    ec_ms_draw
    while true; do
        IFS= read -rsn1 key </dev/tty
        case "$key" in
            $'\x1b')
                read -rsn2 -t 0.1 key2 </dev/tty
                case "$key2" in
                    '[A') (( EC_MS_CURSOR > 0 )) && (( EC_MS_CURSOR-- )) ;;
                    '[B') (( EC_MS_CURSOR < n - 1 )) && (( EC_MS_CURSOR++ )) ;;
                esac ;;
            ' ')
                if [[ "${EC_MS_CHECKED[$EC_MS_CURSOR]}" == "1" ]]; then
                    EC_MS_CHECKED[$EC_MS_CURSOR]=0
                else
                    EC_MS_CHECKED[$EC_MS_CURSOR]=1
                fi ;;
            'a'|'A')
                allon=1
                for (( i=0; i<n; i++ )); do [[ "${EC_MS_CHECKED[$i]}" == "0" ]] && allon=0; done
                nv=1; (( allon == 1 )) && nv=0
                for (( i=0; i<n; i++ )); do EC_MS_CHECKED[$i]=$nv; done ;;
            '')
                out=""; cnt=0
                for (( i=0; i<n; i++ )); do
                    if [[ "${EC_MS_CHECKED[$i]}" == "1" ]]; then out+="$i "; cnt=$((cnt+1)); fi
                done
                if (( cnt >= 1 )); then
                    tput cnorm 2>/dev/null >/dev/tty
                    echo "${out% }"
                    return 0
                fi ;;
            'q'|'Q')
                tput cnorm 2>/dev/null >/dev/tty
                echo "-1"; return 0 ;;
        esac
        printf "\033[${menu_lines}A" >/dev/tty
        ec_ms_draw
    done
}

# Reinstall: stop the master and unload its modules. Returns 1, with the operator message, when
# ec_master stays loaded: a process (plem drive run, a robot control node) holds it, and only
# stopping that process frees it. A stop that fails for another reason (a hand-edited conf that
# does not parse, a conf naming a driver other than the loaded ec_<drv>) is repaired by the
# rmmods below. No-op when ec_master is not loaded.
ec_unload_master() {
    [[ $(lsmod | grep -cw "ec_master") -gt 0 ]] || return 0
    systemctl stop ethercat.service >> "$CMD_LOG" 2>&1 || true
    [[ -x "${ETHERLAB_PREFIX}/sbin/ethercatctl" ]] && "${ETHERLAB_PREFIX}/sbin/ethercatctl" stop >> "$CMD_LOG" 2>&1 || true
    # What is left: ec_master's users named ec_* (IgH's own modules: the ec_<drv> device modules,
    # and its example kernel applications such as ec_mini), then the master. A requested master
    # pins its device module, so a user-space application's hold survives both rmmods and shows
    # in the check below. A user not named ec_* is left loaded and refused there.
    local _user
    for _user in $(lsmod | awk '$1 == "ec_master" { gsub(",", " ", $4); print $4 }'); do
        [[ "$_user" == ec_* ]] || continue
        rmmod "$_user" >> "$CMD_LOG" 2>&1 || true
    done
    rmmod ec_master >> "$CMD_LOG" 2>&1 || true
    [[ $(lsmod | grep -cw "ec_master") -gt 0 ]] || return 0
    fail "ec_master is still in use (lsmod: $(lsmod | grep -w ec_master)). An application may hold the EtherCAT master (plem drive run, a robot control node): stop it, then run menu 3 again. ethercat.service is stopped now; sudo systemctl start ethercat brings the previous setup back"
    return 1
}

# Reinstall: a NIC on ec_<drv> has no Linux net device, so detection would miss it. Stop the
# master and bind those PCI devices back to the stock driver. The built-in driver does not
# reclaim them after rmmod, so bind explicitly. No-op when no native backend holds a NIC.
ec_release_native_nics() {
    lsmod | awk '{print $1}' | grep -qx 'ec_master' || return 0
    # Which native driver holds the ports comes from sysfs, not /etc/ethercat.conf: that is what has to be
    # undone, and a hand-edited conf may not even parse. The wizard writes one native driver for every port.
    local d dm="" ecdrv
    for d in /sys/bus/pci/drivers/ec_*; do
        [[ -d "$d" ]] && ec_native_capable "${d##*/}" && dm="${d##*/ec_}"
    done
    [[ -n "$dm" ]] || return 0
    ecdrv="ec_${dm}"

    # stop rmmods ec_<drv> and the driver directory goes away, so collect the BDFs first.
    local -a bdfs=(); local p b
    for p in "/sys/bus/pci/drivers/$ecdrv/"*; do
        b="$(basename "$p")"
        [[ "$b" =~ ^[0-9a-f]{4}:[0-9a-f]{2}:[0-9a-f]{2}\.[0-9a-f]$ ]] && bdfs+=("$b")
    done
    [[ "${#bdfs[@]}" -eq 0 ]] && return 0

    warn "Existing native EtherCAT (${ecdrv}) holds ${#bdfs[@]} NIC(s) → restoring stock (${dm}) for detection"
    # An application holding the master keeps ec_master (and the NIC's ec_<drv>) loaded through
    # the stop: unbinding the NIC under it would take its bus away. Refuse before touching sysfs.
    ec_unload_master || return 1
    detail "EtherCAT communication is stopped now; it is set up again after the reinstall."
    detail "Cancelling below leaves EtherCAT stopped: systemctl start ethercat restores the previous setup."

    # A modular stock driver was unloaded by ethercatctl start; load it again before binding.
    modprobe "$dm" >> "$CMD_LOG" 2>&1 || true
    local cur
    for b in "${bdfs[@]}"; do
        cur="$(basename "$(readlink "/sys/bus/pci/devices/$b/driver" 2>/dev/null)" 2>/dev/null || echo "")"
        [[ "$cur" == "$dm" ]] && continue
        [[ -n "$cur" ]] && echo "$b" > "/sys/bus/pci/drivers/$cur/unbind" 2>>"$CMD_LOG" || true
        echo "$b" > "/sys/bus/pci/drivers/$dm/bind" 2>>"$CMD_LOG" || true
    done

    # Poll until the released ports have net devices again (up to ~5s, including udev rename).
    # Count only those ports: another port of the same card may never have left the stock driver.
    released_up() { local n=0 b; for b in "${bdfs[@]}"; do compgen -G "/sys/bus/pci/devices/$b/net/*" >/dev/null && n=$((n + 1)); done; echo "$n"; }
    local i back
    for i in $(seq 1 10); do
        back=$(released_up); (( back >= ${#bdfs[@]} )) && break
        sleep 0.5
    done
    if (( back >= ${#bdfs[@]} )); then
        detail "stock ${dm} restored: ${#bdfs[@]} port net device(s) visible again"
    else
        warn "Only ${back} of ${#bdfs[@]} port(s) came back on ${dm}: the others do not appear below"
    fi
}

p3_s1_detect_interface() {
    p3_header 1

    EC_INTERFACES=(); EC_MACS=()
    local ifc

    # Reinstall: if a native backend already holds the NIC (ec_<drv>), that port's net device
    # is gone and detection below would miss it. Return to stock first to expose every port.
    ec_release_native_nics || { PHASE3_RESULTS+=("failed"); return 1; }

    if [[ -n "${ETHERCAT_IFACE:-}" ]]; then
        # Comma-separated multiple values allowed: ETHERCAT_IFACE="eno1,enP5p1s0f0"
        local -a want=()
        IFS=',' read -r -a want <<< "$ETHERCAT_IFACE"
        for ifc in "${want[@]}"; do
            ifc="${ifc// /}"
            [[ -z "$ifc" ]] && continue
            if ! ip link show "$ifc" &>/dev/null || [[ ! -d "/sys/class/net/${ifc}/device" ]]; then
                fail "ETHERCAT_IFACE='${ifc}' is not a usable physical NIC"
                PHASE3_RESULTS+=("failed"); return 1
            fi
            EC_INTERFACES+=("$ifc")
        done
        if [[ "${#EC_INTERFACES[@]}" -eq 0 ]]; then
            fail "No valid NIC found in ETHERCAT_IFACE"
            PHASE3_RESULTS+=("failed"); return 1
        fi
    else
        local -a candidates=(); local excluded=0
        while IFS= read -r ifc; do
            if ec_is_candidate "$ifc"; then
                candidates+=("$ifc")
            elif [[ -d "/sys/class/net/${ifc}/device" && "$ifc" != "lo" ]]; then
                excluded=$((excluded + 1))
            fi
        done < <(find /sys/class/net -maxdepth 1 -mindepth 1 -printf '%f\n' | sort)

        if [[ "${#candidates[@]}" -eq 0 ]]; then
            fail "No candidate physical NIC for EtherCAT found"
            detail "Set it with the ETHERCAT_IFACE=<iface>[,<iface>...] environment variable, or check the NIC connection."
            PHASE3_RESULTS+=("failed"); return 1
        elif [[ "${#candidates[@]}" -eq 1 ]]; then
            # A single port (no PCIe NIC) may be the device's network link: ask before taking it for EtherCAT.
            local only="${candidates[0]}" addr ans
            addr=$(ip -4 -o addr show dev "$only" 2>/dev/null | awk '{print $4}' | paste -sd' ')
            warn "Only one candidate NIC: ${only}${addr:+ (address ${addr})}. As an EtherCAT master port it carries EtherCAT only."
            read -rp "         Use ${only} for EtherCAT? [y/N]: " ans
            if [[ ! "$ans" =~ ^[Yy]$ ]]; then
                fail "No NIC selected. Add a NIC for EtherCAT, or name one with ETHERCAT_IFACE=<iface>"
                PHASE3_RESULTS+=("failed"); return 1
            fi
            EC_INTERFACES+=("$only")
        else
            echo "         Select the NIC(s) to use as EtherCAT master. For multi-master (one master per bus) pick several;"
            echo "         they are configured as MASTER0, MASTER1, ... in the order listed."
            [[ "$excluded" -gt 0 ]] && detail "Excluded ${excluded} non-EtherCAT interface(s) (CAN/USB/Wi-Fi/non-Ethernet)"
            local -a labels=()
            for ifc in "${candidates[@]}"; do
                local mac state driver tag
                mac=$(cat "/sys/class/net/${ifc}/address" 2>/dev/null || echo "?")
                state=$(cat "/sys/class/net/${ifc}/operstate" 2>/dev/null || echo "?")
                driver=$(basename "$(readlink -f "/sys/class/net/${ifc}/device/driver" 2>/dev/null)" 2>/dev/null || echo "?")
                if ec_native_capable "$driver"; then tag=" ${GREEN}[native]${NC}"; else tag=" ${DIM}[generic]${NC}"; fi
                labels+=("$(printf '%-14s mac=%s  state=%s  driver=%s%s' "$ifc" "$mac" "$state" "$driver" "$tag")")
            done
            local picks idx
            picks=$(ec_multiselect "${labels[@]}")
            if [[ "$picks" == "-1" || -z "$picks" ]]; then
                fail "Selection cancelled"; PHASE3_RESULTS+=("failed"); return 1
            fi
            for idx in $picks; do
                EC_INTERFACES+=("${candidates[$idx]}")
            done
        fi
    fi

    # Collect each interface MAC (1:1 with EC_INTERFACES)
    for ifc in "${EC_INTERFACES[@]}"; do
        local mac
        mac=$(cat "/sys/class/net/${ifc}/address" 2>/dev/null || echo "")
        if [[ -z "$mac" ]]; then
            fail "Cannot read the MAC address of ${ifc}"; PHASE3_RESULTS+=("failed"); return 1
        fi
        EC_MACS+=("$mac")
    done

    # Decide the native/generic backend (mixed drivers → generic)
    # ec_resolve_backend may ask to accept a generic fallback; a refusal fails this step.
    if ! ec_resolve_backend; then
        PHASE3_RESULTS+=("failed"); return 1
    fi

    local n=${#EC_INTERFACES[@]} k
    if [[ "$n" -gt 1 ]]; then
        info "${n} NICs selected → multi-master (IgH backend=${EC_DEVICE_MODULES})"
        for k in "${!EC_INTERFACES[@]}"; do
            detail "MASTER${k}: ${EC_INTERFACES[$k]} (${EC_MACS[$k]})"
        done
    else
        info "Interface: ${EC_INTERFACES[0]} (${EC_MACS[0]}, IgH backend=${EC_DEVICE_MODULES})"
    fi
    PHASE3_RESULTS+=("done")
}

p3_s2_dependencies() {
    p3_header 2

    local pkgs=(build-essential autoconf automake libtool pkg-config git)
    local need_install=false
    for pkg in "${pkgs[@]}"; do
        dpkg -l "$pkg" 2>/dev/null | grep -q "^[hi]i" || { need_install=true; break; }
    done

    local result=skipped
    if $need_install; then
        spin_start "Installing build dependencies..."
        apt-get update -qq >> "$CMD_LOG" 2>&1 || true
        if apt-get install -y -qq "${pkgs[@]}" >> "$CMD_LOG" 2>&1; then
            spin_stop; info "Install done"; result=done
        else
            spin_stop; fail "Install failed"; PHASE3_RESULTS+=("failed"); return 1
        fi
    else
        skip "Already installed"
    fi

    # The package state does not prove the binaries exist: a package can stay "installed" after its
    # file is gone (e.g. /usr/bin/gcc removed with an alternatives group), and configure then fails late.
    if ! command -v cc >/dev/null || ! command -v make >/dev/null; then
        fail "cc or make is not in PATH although build-essential is installed"
        detail "Restore with: sudo apt-get install --reinstall gcc make, then run menu 3 again"
        PHASE3_RESULTS+=("failed"); return 1
    fi
    PHASE3_RESULTS+=("$result")
}

p3_s3_clone() {
    p3_header 3

    # Download source into a temp directory (removed in step 10)
    ETHERCAT_SRC=$(mktemp -d /tmp/etherlab-build-XXXXXX)
    detail "Build directory: ${ETHERCAT_SRC}"

    spin_start "Downloading EtherLab ${ETHERLAB_VERSION} (tag)..."
    if git clone -b "$ETHERLAB_VERSION" --depth 1 "$ETHERLAB_REPO" "$ETHERCAT_SRC" >> "$CMD_LOG" 2>&1; then
        spin_stop; info "Download done"
    else
        spin_stop; fail "Download failed (check that tag ${ETHERLAB_VERSION} exists)"; PHASE3_RESULTS+=("failed"); return 1
    fi

    # Check the cloned tag against the expected SHA. A match on either HEAD or the tag
    # object is accepted, so annotated and lightweight tags both work.
    if [[ -n "$ETHERLAB_EXPECTED_SHA" ]]; then
        local head_sha tag_sha
        head_sha="$(git -C "$ETHERCAT_SRC" rev-parse HEAD 2>/dev/null || echo "")"
        tag_sha="$(git -C "$ETHERCAT_SRC" rev-parse "$ETHERLAB_VERSION" 2>/dev/null || echo "")"
        if [[ "$head_sha" == "${ETHERLAB_EXPECTED_SHA}"* || "$tag_sha" == "${ETHERLAB_EXPECTED_SHA}"* ]]; then
            detail "Integrity verified: ${ETHERLAB_VERSION} @ ${head_sha:0:8}"
        else
            fail "EtherLab integrity mismatch: expected ${ETHERLAB_EXPECTED_SHA}.. / HEAD ${head_sha:0:8} tag ${tag_sha:0:8}"
            PHASE3_RESULTS+=("failed"); return 1
        fi
    fi
    PHASE3_RESULTS+=("done")
}

p3_s4_build() {
    p3_header 4

    cd "$ETHERCAT_SRC" || { fail "Cannot enter build directory ${ETHERCAT_SRC}"; PHASE3_RESULTS+=("failed"); return 1; }

    kernel_headers_match ec_master || { PHASE3_RESULTS+=("failed"); return 1; }

    [[ -f "Makefile" ]] && { make clean >> "$CMD_LOG" 2>&1 || true; }

    detail "Running bootstrap..."
    if ! ./bootstrap >> "$CMD_LOG" 2>&1; then
        fail "bootstrap failed"; PHASE3_RESULTS+=("failed"); return 1
    fi

    # IgH default is "all device drivers disabled"; we enable generic
    # unconditionally as a fallback, and add a native backend
    # (e.g. --enable-igb) iff the selected NIC has an IgH equivalent
    # (decided in step 1).
    detail "Running configure..."
    local -a extra_enables=()
    if [[ -n "${EC_DEVICE_MODULES:-}" && "$EC_DEVICE_MODULES" != "generic" ]]; then
        extra_enables+=("--enable-${EC_DEVICE_MODULES}")
        detail "Enabling native EtherCAT backend build: ec_${EC_DEVICE_MODULES}"
    fi
    local log_mark; log_mark=$(wc -l < "$CMD_LOG")
    if ! ./configure \
        --prefix="$ETHERLAB_PREFIX" \
        --sysconfdir=/etc \
        --enable-generic \
        "${extra_enables[@]}" >> "$CMD_LOG" 2>&1; then
        # If IgH has no native copy for the running kernel, it stops here with "kernel X.Y not available for
        # <drv> driver!". We do not fall back to generic (1kHz jitter). Show the reason.
        fail "configure failed: $(tail -n +"$(( log_mark + 1 ))" "$CMD_LOG" | grep 'configure: error' | tail -1 | sed 's/.*configure: error: //')"
        PHASE3_RESULTS+=("failed"); return 1
    fi

    # make (keep 1 CPU core free for the spinner)
    local jobs=$(( $(nproc) - 1 ))
    [[ $jobs -lt 1 ]] && jobs=1
    spin_start "Compiling... (takes 1-3 min)"
    if make -j"$jobs" all modules >> "$CMD_LOG" 2>&1; then
        spin_stop; info "Build done"; PHASE3_RESULTS+=("done")
    else
        spin_stop; fail "Build failed"; PHASE3_RESULTS+=("failed"); return 1
    fi
}

p3_s5_install() {
    p3_header 5

    cd "$ETHERCAT_SRC" || { fail "Cannot enter build directory ${ETHERCAT_SRC}"; PHASE3_RESULTS+=("failed"); return 1; }

    # Reinstall (generic backend; the native one unloaded in step 1): old modules must go first.
    # Installing over a module a process holds would leave it running, and step 10 would still
    # report it as verified.
    ec_unload_master || { PHASE3_RESULTS+=("failed"); return 1; }

    spin_start "Installing..."
    if make modules_install install >> "$CMD_LOG" 2>&1; then
        spin_stop
    else
        spin_stop; fail "Install failed"; PHASE3_RESULTS+=("failed"); return 1
    fi

    depmod >> "$CMD_LOG" 2>&1

    echo "${ETHERLAB_PREFIX}/lib" > /etc/ld.so.conf.d/etherlab.conf \
        || { fail "Could not write /etc/ld.so.conf.d/etherlab.conf"; PHASE3_RESULTS+=("failed"); return 1; }
    ldconfig >> "$CMD_LOG" 2>&1

    ln -sf "${ETHERLAB_PREFIX}/bin/ethercat" /usr/local/bin/ethercat 2>/dev/null || true

    info "Install done (${ETHERLAB_PREFIX})"
    PHASE3_RESULTS+=("done")
}

p3_s6_configure() {
    p3_header 6

    local conf="/etc/ethercat.conf"

    # Auto-generate MASTER0..N + UPDOWN_INTERFACES from the selected NICs
    local master_lines="" updown="" k
    for k in "${!EC_MACS[@]}"; do
        master_lines+="MASTER${k}_DEVICE=\"${EC_MACS[$k]}\""$'\n'
        updown+="${EC_INTERFACES[$k]} "
    done
    updown="${updown% }"

    {
        cat << 'EOFCONF'
# EtherCAT master configuration (generated by the RT setup wizard). ethercatctl sources it as shell.
#   MASTER<X>_DEVICE  : MAC of each EtherCAT NIC port, one line per master.
#   DEVICE_MODULES    : driver backend (igb | generic | ...). Native needs all ports on one driver.
#   UPDOWN_INTERFACES : interfaces brought up/down with the service, 1:1 with MASTER<X>_DEVICE.
# With a native backend the port's net device disappears after the rebind, so a restart logs
# "Cannot find device <iface>". This is expected.
# To add ports, append MASTER<N>_DEVICE lines (numbered from 0 without gaps; ethercatctl stops at the
# first missing number) and the interface here; the ExecStartPost rebind handles every entry.

EOFCONF
        printf '%sDEVICE_MODULES="%s"\nUPDOWN_INTERFACES="%s"\n' "$master_lines" "$EC_DEVICE_MODULES" "$updown"
    } > "$conf" || { die "Failed to write ${conf}"; PHASE3_RESULTS+=("failed"); return 1; }

    for k in "${!EC_MACS[@]}"; do
        detail "MASTER${k}_DEVICE=${EC_MACS[$k]} (${EC_INTERFACES[$k]})"
    done
    detail "DEVICE_MODULES=${EC_DEVICE_MODULES}"
    detail "UPDOWN_INTERFACES=${updown}"
    info "Configured (${#EC_MACS[@]} masters)"; PHASE3_RESULTS+=("done")
}

p3_s7_udev() {
    p3_header 7

    # The EtherCAT device opens for members of realtime only (the group that may run the bus loop real-time).
    local rule='KERNEL=="EtherCAT[0-9]*", MODE="0660", GROUP="realtime"' wrote=false
    realtime_membership || { PHASE3_RESULTS+=("failed"); return 1; }
    if [[ "$(cat "$EC_UDEV_RULES" 2>/dev/null)" != "$rule" ]]; then
        if [[ -f "$EC_UDEV_RULES" ]]; then
            cp -p "$EC_UDEV_RULES" "${EC_UDEV_RULES}.plem-bak" \
                || { die "Failed to back up ${EC_UDEV_RULES}: left as it is"; PHASE3_RESULTS+=("failed"); return 1; }
            detail "Previous rule kept in ${EC_UDEV_RULES}.plem-bak"   # udev reads only *.rules
        fi
        echo "$rule" > "$EC_UDEV_RULES" || { die "Failed to write ${EC_UDEV_RULES}"; PHASE3_RESULTS+=("failed"); return 1; }
        wrote=true
    fi
    # Reload even when the rule was already there: an earlier run may have written it and failed here.
    if ! udevadm control --reload-rules >> "$CMD_LOG" 2>&1 || ! udevadm trigger --sysname-match='EtherCAT*' >> "$CMD_LOG" 2>&1; then
        die "udev reload failed: /dev/EtherCAT* may keep its old mode"; PHASE3_RESULTS+=("failed"); return 1
    fi
    if $wrote || $RT_MEMBERSHIP_CHANGED; then info "udev rule: /dev/EtherCAT* mode 0660, group realtime"; PHASE3_RESULTS+=("done")
    else skip "Already set"; PHASE3_RESULTS+=("skipped"); fi
}

p3_s8_systemd() {
    p3_header 8

    local svc="/etc/systemd/system/ethercat.service"

    if ! cat > "$svc" << EOFSVC
[Unit]
Description=EtherCAT Master Kernel Modules
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=${ETHERLAB_PREFIX}/sbin/ethercatctl start
# Move each EtherCAT NIC from the built-in driver to ec_<drv> after ec_master loads.
# Running it here re-applies the current /etc/ethercat.conf on every start and restart.
# Needed because the Jetson NIC driver is built in (cannot be rmmod'ed). No-op for generic.
ExecStartPost=${EC_NATIVE_BIND_SCRIPT}
ExecStop=${ETHERLAB_PREFIX}/sbin/ethercatctl stop

[Install]
WantedBy=multi-user.target
EOFSVC
    then
        fail "Failed to write ${svc}"; PHASE3_RESULTS+=("failed"); return 1
    fi

    run_cmd "daemon-reload" systemctl daemon-reload || { fail "daemon-reload failed"; PHASE3_RESULTS+=("failed"); return 1; }
    run_cmd "enable service" systemctl enable ethercat.service || { fail "enable failed"; PHASE3_RESULTS+=("failed"); return 1; }
    info "Service configured"; PHASE3_RESULTS+=("done")
}

# Older setups ran the rebind from a standalone unit or an ethercat.service drop-in. Either one
# left in place runs the bind a second time next to the ExecStartPost, so both are removed. Their
# NIC IRQ tuning unit goes too: on the generic backend it pinned the NIC IRQ to a CPU chosen at
# install time, which a later menu 1 can make the isolated CPU.
remove_old_native_bind_units() {
    if [[ -f "$EC_IRQ_TUNE_SERVICE" ]]; then
        systemctl disable --now plem-ethercat-nic-irq-tune.service >> "$CMD_LOG" 2>&1 || true
    fi
    rm -f "$EC_IRQ_TUNE_SERVICE" "$EC_IRQ_TUNE_SCRIPT" "$EC_IRQ_TUNE_DROPIN"
    rmdir "$(dirname "$EC_IRQ_TUNE_DROPIN")" 2>/dev/null || true
    if systemctl list-unit-files 2>/dev/null | grep -q '^plem-ethercat-native-bind\.service'; then
        systemctl disable --now plem-ethercat-native-bind.service >> "$CMD_LOG" 2>&1 || true
    fi
    rm -f "$EC_NATIVE_BIND_SERVICE" "$EC_NATIVE_BIND_DROPIN"
    rmdir --ignore-fail-on-non-empty "$(dirname "$EC_NATIVE_BIND_DROPIN")" 2>/dev/null || true
}

p3_s9_native_bind() {
    p3_header 9

    # Native PCIe rebind helper, run as ethercat.service ExecStartPost (see p3_s8_systemd).
    # The script header explains it.
    if ! cat > "$EC_NATIVE_BIND_SCRIPT" << 'EOFNB'
#!/usr/bin/env bash
# Rebind EtherCAT NIC(s) from the kernel network driver (igb/igc/...) to
# IgH's EtherCAT-aware driver (ec_igb/ec_igc/...). Idempotent.
# ETHERCAT_CONF overrides the config path (tests).
set -euo pipefail

ETHERCAT_CONF="${ETHERCAT_CONF:-/etc/ethercat.conf}"

if [[ ! -f "$ETHERCAT_CONF" ]]; then
    echo "ERROR: $ETHERCAT_CONF missing." >&2
    exit 1
fi

# Read the conf the way ethercatctl does: source it as shell (any quoting), take
# DEVICE_MODULES as a space-separated list, and MASTER0_DEVICE, MASTER1_DEVICE, ...
# up to the first unset one.
# The syntax check is separate: sourcing returns the status of the conf's last
# command, which ethercatctl ignores too.
if ! bash -n "$ETHERCAT_CONF"; then
    echo "ERROR: $ETHERCAT_CONF does not parse as shell (ethercatctl sources it). Fix the file, then: systemctl restart ethercat." >&2
    exit 1
fi
DEVICE_MODULES=""
set +u
# shellcheck source=/dev/null
. "$ETHERCAT_CONF" >/dev/null || true
set -u
read -ra modules <<<"$DEVICE_MODULES"
if [[ ${#modules[@]} -eq 0 || "${modules[*]}" == "generic" ]]; then
    echo "info: DEVICE_MODULES=${DEVICE_MODULES:-empty}. native rebind not required."
    exit 0
fi
if [[ ${#modules[@]} -gt 1 ]]; then
    echo "ERROR: DEVICE_MODULES=\"${DEVICE_MODULES}\" in $ETHERCAT_CONF lists more than one module. This helper moves every MASTER<N>_DEVICE NIC to one native driver, so it needs DEVICE_MODULES set to exactly one native driver (for example \"igb\"), or to \"generic\" (no rebind). Set one, then: systemctl restart ethercat." >&2
    exit 1
fi
module="${modules[0]}"

devices=()
while var="MASTER${#devices[@]}_DEVICE"; [[ -n "${!var:-}" ]]; do
    devices+=("${!var}")
done
# For a manual run: as ExecStartPost this is never reached, ethercatctl start
# already fails on a missing MASTER0_DEVICE.
if [[ ${#devices[@]} -eq 0 ]]; then
    echo "ERROR: no MASTER0_DEVICE in $ETHERCAT_CONF, so there is no NIC to move to ec_${module}. Set MASTER0_DEVICE to the EtherCAT NIC's MAC (ip -o link), then: systemctl restart ethercat." >&2
    exit 1
fi

EC_DRIVER="ec_${module}"

if ! lsmod | awk '{print $1}' | grep -qE "^${EC_DRIVER}$"; then
    if ! modprobe "$EC_DRIVER" 2>/dev/null; then
        echo "ERROR: failed to modprobe $EC_DRIVER. Is IgH built with --enable-${module}?" >&2
        exit 1
    fi
fi

if [[ ! -d "/sys/bus/pci/drivers/$EC_DRIVER" ]]; then
    echo "ERROR: /sys/bus/pci/drivers/$EC_DRIVER missing after modprobe." >&2
    exit 1
fi

mac_to_iface() {
    local mac="${1,,}" net name addr
    for net in /sys/class/net/*; do
        [[ -f "$net/address" ]] || continue
        name="$(basename "$net")"
        addr="$(tr '[:upper:]' '[:lower:]' <"$net/address")"
        if [[ "$addr" == "$mac" ]]; then
            echo "$name"; return 0
        fi
    done
    return 1
}

# Is this MAC already served by the EtherCAT master? Once a NIC is rebound to
# ec_<drv> its kernel netdev disappears, so mac_to_iface() can no longer find
# it, but the master reports it as "(attached)". A stale or wrong MAC (no such
# NIC present) instead shows "(waiting...)" or nothing. This lets us tell
# "already done" apart from "misconfigured". Returns: 0=attached (already
# bound), 1=not attached, 2=cannot tell (ethercat CLI unavailable).
mac_attached_to_master() {
    local mac="${1,,}"
    command -v ethercat >/dev/null 2>&1 || return 2
    ethercat master 2>/dev/null | tr '[:upper:]' '[:lower:]' \
        | grep -qE "main:[[:space:]]*${mac}[[:space:]]*\(attached\)"
}

rebound=0
skipped=0
stale=0
for master_idx in "${!devices[@]}"; do
    val="${devices[$master_idx]}"

    iface=""
    if [[ "$val" =~ ^([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}$ ]]; then  # ethercatctl's MAC test
        iface="$(mac_to_iface "$val" 2>/dev/null || true)"
    else
        iface="$val"
    fi

    if [[ -z "$iface" ]]; then
        # No net device has this MAC. Two causes:
        #   (a) the NIC is already on ec_<drv> (its netdev is gone): normal
        #   (b) no NIC with this MAC is present (stale/wrong conf): broken
        att_rc=0
        mac_attached_to_master "$val" || att_rc=$?
        if [[ "$att_rc" -eq 0 ]]; then
            echo "info: MASTER${master_idx}_DEVICE=$val: already bound to $EC_DRIVER (attached to master)."
            skipped=$((skipped + 1)); continue
        elif [[ "$att_rc" -eq 2 ]]; then
            echo "WARN: MASTER${master_idx}_DEVICE=$val: no net device and master state unknown (ethercat CLI not found). If slaves are missing, verify this MAC matches a present NIC (ip -o link) in $ETHERCAT_CONF." >&2
            skipped=$((skipped + 1)); continue
        fi
        echo "WARN: MASTER${master_idx}_DEVICE=$val: stale or absent MAC: no present NIC has this MAC and it is not attached to the master. EtherCAT master ${master_idx} will NOT work. Set MASTER${master_idx}_DEVICE in $ETHERCAT_CONF to a current NIC MAC (ip -o link), then: systemctl restart ethercat." >&2
        stale=$((stale + 1)); continue
    fi

    bdf="$(basename "$(readlink "/sys/class/net/$iface/device" 2>/dev/null || true)")"
    if [[ -z "$bdf" ]]; then
        echo "WARN: MASTER${master_idx}_DEVICE=$val (iface=$iface) has no PCI BDF, skipping." >&2
        continue
    fi

    cur_driver="$(basename "$(readlink "/sys/bus/pci/devices/$bdf/driver" 2>/dev/null || true)")"
    if [[ "$cur_driver" == "$EC_DRIVER" ]]; then
        echo "info: $bdf ($iface) already on $EC_DRIVER."
        skipped=$((skipped + 1)); continue
    fi

    if [[ -n "$cur_driver" ]]; then
        echo "info: $bdf ($iface): unbinding from $cur_driver"
        echo "$bdf" > "/sys/bus/pci/drivers/$cur_driver/unbind" || { echo "ERROR: unbind failed" >&2; exit 1; }
    fi

    # Rollback: if the bind to ec_<drv> fails after we already unbound the
    # original kernel driver, the NIC is left driverless and `set -e` would
    # abort here, stranding it. Re-bind the original driver before exiting so
    # the NIC keeps a working driver instead of going dark.
    echo "info: $bdf ($iface): binding to $EC_DRIVER"
    if ! echo "$bdf" > "/sys/bus/pci/drivers/$EC_DRIVER/bind"; then
        echo "ERROR: bind to $EC_DRIVER failed" >&2
        if [[ -n "$cur_driver" ]]; then
            echo "info: $bdf ($iface): rolling back, re-binding to $cur_driver" >&2
            echo "$bdf" > "/sys/bus/pci/drivers/$cur_driver/bind" 2>/dev/null \
                || echo "ERROR: rollback re-bind to $cur_driver also failed, NIC has no driver" >&2
        fi
        exit 1
    fi
    rebound=$((rebound + 1))
done

echo "summary: rebound=$rebound skipped=$skipped stale=$stale target_driver=$EC_DRIVER"

# A stale/absent MASTER MAC means that master cannot run. If nothing was
# rebound at all yet some entries were stale, the config is effectively broken,
# so fail loudly and mark ethercat.service failed instead of silently
# "succeeding" with no working EtherCAT. (A partial multi-master setup where at
# least one NIC came up stays successful so it is not torn down.)
if [[ "$stale" -gt 0 && "$rebound" -eq 0 ]]; then
    echo "ERROR: ${stale} EtherCAT master device(s) have a stale/absent MAC and none were rebound. Fix MASTER*_DEVICE in $ETHERCAT_CONF (ip -o link), then: systemctl restart ethercat." >&2
    exit 1
fi
EOFNB
    then
        fail "Failed to write ${EC_NATIVE_BIND_SCRIPT}"; PHASE3_RESULTS+=("failed"); return 1
    fi
    chmod 0755 "$EC_NATIVE_BIND_SCRIPT" \
        || { fail "Failed to make ${EC_NATIVE_BIND_SCRIPT} executable"; PHASE3_RESULTS+=("failed"); return 1; }

    # The rebind runs only as ethercat.service ExecStartPost. The unit turns active only after
    # ExecStartPost finishes, so no ordering drop-in is needed.
    remove_old_native_bind_units

    run_cmd "daemon-reload" systemctl daemon-reload || { fail "daemon-reload failed"; PHASE3_RESULTS+=("failed"); return 1; }

    detail "RT loop CPU: $(booted_isol_cpu) (isolated at boot)"
    detail "interfaces: ${EC_INTERFACES[*]}"
    if [[ -n "${EC_DEVICE_MODULES:-}" && "$EC_DEVICE_MODULES" != "generic" ]]; then
        detail "DEVICE_MODULES: ${EC_DEVICE_MODULES} (native → PCIe rebind to ec_${EC_DEVICE_MODULES})"
    else
        detail "DEVICE_MODULES: ${EC_DEVICE_MODULES:-generic} (generic → the bind helper changes nothing)"
    fi
    # Keep NetworkManager from running DHCP/IPv6 on EtherCAT ports (the generic backend keeps
    # its netdev; for native this only applies early in boot, before the rebind).
    # `+=` appends to the existing unmanaged list (distro default); `=` would replace the whole list.
    if [[ -d /etc/NetworkManager/conf.d ]]; then
        local _m _macs=()
        for _m in "${EC_MACS[@]}"; do _macs+=("mac:${_m}"); done
        if ! printf '[keyfile]\nunmanaged-devices+=%s\n' "$(IFS=';'; echo "${_macs[*]}")" \
            > /etc/NetworkManager/conf.d/plem-ethercat.conf; then
            fail "Cannot write /etc/NetworkManager/conf.d/plem-ethercat.conf"; PHASE3_RESULTS+=("failed"); return 1
        fi
        systemctl reload NetworkManager >> "$CMD_LOG" 2>&1 || true
        detail "NetworkManager unmanaged: ${EC_MACS[*]}"
    fi
    info "bind helper installed as ethercat.service ExecStartPost ($(basename "$EC_NATIVE_BIND_SCRIPT"))"
    PHASE3_RESULTS+=("done")
}

p3_s10_start_verify() {
    p3_header 10

    spin_start "Starting EtherCAT service..."
    systemctl stop ethercat >> "$CMD_LOG" 2>&1 || true
    depmod >> "$CMD_LOG" 2>&1 || true
    systemctl start ethercat >> "$CMD_LOG" 2>&1 || true

    # Wait for the module to load (up to 15 s)
    local w=0
    while [[ $(lsmod | grep -cw "ec_master") -eq 0 ]] && [[ $w -lt 15 ]]; do
        sleep 1; w=$(( w + 1 ))
    done

    # The native rebind already ran as ExecStartPost of the start above.
    spin_stop

    local verify_ok=true

    if systemctl is-active --quiet ethercat; then
        detail "├─ service: active"
    else
        detail "├─ service: ${RED}inactive${NC}"; verify_ok=false
    fi

    if [[ $(lsmod | grep -cw "ec_master") -gt 0 ]]; then
        detail "├─ module: ec_master loaded"
    else
        detail "├─ module: ${RED}ec_master not loaded${NC}"; verify_ok=false
    fi

    # The native backend polls, so there is no NIC IRQ. generic runs without IRQ tuning.
    if [[ -n "${EC_DEVICE_MODULES:-}" && "$EC_DEVICE_MODULES" != "generic" ]]; then
        detail "├─ backend: native ${EC_DEVICE_MODULES} (polling, no NIC IRQ)"
    else
        detail "├─ backend: ${YELLOW}generic${NC} (works; more jitter than native, 1 kHz without lost frames not guaranteed)"
    fi

    # The link comes up a few seconds after the service starts: read the slave count once it has (up to 10 s).
    local master_out _wl=0
    while $verify_ok && [[ $_wl -lt 10 ]] && master_out=$(ethercat master 2>/dev/null) \
        && [[ ! "$master_out" =~ Slaves:[[:space:]]*[1-9] ]]; do
        [[ $_wl -eq 0 ]] && detail "├─ waiting for the EtherCAT link (up to 10 s)"
        sleep 1; _wl=$(( _wl + 1 ))
    done
    if master_out=$(ethercat master 2>&1); then
        # Multi-master: `ethercat master` prints Master0..N blocks in order. A single
        # grep for Phase/Slaves puts several lines into one variable and breaks the output
        # (Phase=Idle\nIdle..., Slaves=7\n7...), so parse per master block and print one line each.
        local _ml _total=0 _mc=0
        _ml=$(echo "$master_out" | awk '
            /^Master[0-9]+/        { if(i!=""){print i"|"p"|"s"|"l} i=$1; p="?"; s="0"; l="?" }
            /^[[:space:]]*Phase:/  { p=$2 }
            /^[[:space:]]*Slaves:/ { s=$2 }
            /Link:/                { if(l=="?") l=$2 }
            END                    { if(i!=""){print i"|"p"|"s"|"l} }
        ')
        while IFS='|' read -r _idx _ph _sl _lk; do
            [[ -z "$_idx" ]] && continue
            _mc=$((_mc + 1)); _total=$((_total + ${_sl:-0}))
        done <<< "$_ml"
        detail "├─ masters: ${_mc} (total slaves ${_total})"
        while IFS='|' read -r _idx _ph _sl _lk; do
            [[ -z "$_idx" ]] && continue
            local _lkc
            [[ "$_lk" == "UP" ]] && _lkc="${GREEN}UP${NC}" || _lkc="${DIM}${_lk}${NC}"
            detail "│    ${_idx}  Phase=${_ph}  Slaves=${_sl}  Link=${_lkc}"
        done <<< "$_ml"
        if [[ "$_total" -eq 0 ]]; then
            detail "├─ ${YELLOW}no slaves connected (connect EtherCAT devices, then check)${NC}"
        fi
    else
        detail "├─ master: ${YELLOW}not accessible${NC}"
    fi

    # The device nodes the service created: step 7's rule applies to them (a warning, the master itself works).
    local _ec_nodes _ec_bad
    udevadm settle --timeout=10 >> "$CMD_LOG" 2>&1 || true   # let step 7's rule reach the new nodes first
    if _ec_nodes=$(ec_device_nodes) && _ec_bad=$(ec_bad_nodes "$_ec_nodes") && [[ -n "$_ec_bad" ]]; then
        warn "${_ec_bad}does not have mode 0660 and group realtime: another udev rule may override it (grep -rl EtherCAT /etc/udev/rules.d /lib/udev/rules.d), then check with menu 2"
    fi

    # Clean up sources (including root-owned files from the build)
    if [[ -d "$ETHERCAT_SRC" ]]; then
        rm -rf "$ETHERCAT_SRC"
        detail "└─ source directory removed (${ETHERCAT_SRC})"
    fi

    if $verify_ok; then
        info "Started and verified"; PHASE3_RESULTS+=("done")
    else
        fail "Verification failed"; PHASE3_RESULTS+=("failed")
    fi
}

run_phase3() {
    clear
    print_box "Menu 3: Install EtherCAT master"
    print_env

    echo ""
    echo -e "  ${BOLD}Checking prerequisites${NC}"
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"

    local prereq_ok=true

    if $RT_BOOTED; then
        printf "  %b  %-28s %s\n" "${GREEN}✔${NC}" "Booted RT kernel" "$CURRENT_KERNEL"
    else
        printf "  %b  %-28s %s\n" "${RED}✘${NC}" "Booted RT kernel" "not an RT kernel"
        prereq_ok=false
    fi

    if dpkg -l nvidia-l4t-rt-kernel-headers 2>/dev/null | grep -q "^[hi]i"; then
        printf "  %b  %-28s %s\n" "${GREEN}✔${NC}" "Kernel headers" "installed"
    else
        printf "  %b  %-28s %s\n" "${RED}✘${NC}" "Kernel headers" "nvidia-l4t-rt-kernel-headers not installed"
        prereq_ok=false
    fi

    if phase3_is_complete; then
        printf "  %b  %-28s %s\n" "${YELLOW}⚠${NC}" "Existing install" "EtherCAT already installed"
    fi

    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"

    if ! $prereq_ok; then
        echo ""
        echo -e "  ${RED}${BOLD}Prerequisites not met.${NC}"
        echo ""
        read -rp "  Press Enter to return to the menu..." _
        return
    fi

    echo ""
    if phase3_is_complete; then
        read -rp "  Already installed. Reinstall? (y/N): " answer
        [[ ! "$answer" =~ ^[Yy]$ ]] && return
    else
        read -rp "  Install the EtherCAT master? (Y/n): " confirm
        [[ "$confirm" =~ ^[Nn]$ ]] && return
    fi

    PHASE3_RESULTS=()
    ETHERCAT_SRC=""
    local p3_failed=false _cwd="$PWD"

    p3_run_step() {
        $p3_failed && { PHASE3_RESULTS+=("failed"); return; }
        "$@" || p3_failed=true
    }

    p3_run_step p3_s1_detect_interface
    p3_run_step p3_s2_dependencies
    p3_run_step p3_s3_clone
    p3_run_step p3_s4_build
    p3_run_step p3_s5_install
    p3_run_step p3_s6_configure
    p3_run_step p3_s7_udev
    p3_run_step p3_s8_systemd
    p3_run_step p3_s9_native_bind
    p3_run_step p3_s10_start_verify

    # The build steps cd into the source directory. Go back, and remove it if a failed step left it.
    cd "$_cwd" 2>/dev/null || cd /
    [[ -n "$ETHERCAT_SRC" && -d "$ETHERCAT_SRC" ]] && rm -rf "$ETHERCAT_SRC"

    p3_summary

    local has_fail=false
    for r in "${PHASE3_RESULTS[@]}"; do
        [[ "$r" == "failed" ]] && { has_fail=true; break; }
    done

    if ! $has_fail; then
        echo -e "  ${GREEN}${BOLD}EtherCAT master installation is complete.${NC}"
        echo ""
        echo -e "  ${DIM}Useful commands:${NC}"
        echo -e "  ${DIM}  ethercat master     # master status${NC}"
        echo -e "  ${DIM}  ethercat slaves     # slave list${NC}"
    else
        echo -e "  ${RED}${BOLD}There was a problem with the installation.${NC}"
        echo -e "  ${DIM}Log: ${CMD_LOG}${NC}"
    fi

    echo ""
    read -rp "  Press Enter to return to the menu..." _
}

# ============================================================================
#  Menu 4: RT jitter load test
# ============================================================================
# Purpose: using this repo alone, quickly measure "does the isolated RT CPU keep
# deterministic wakeups at 1→5kHz cycle periods?". cyclictest runs alongside a representative
# load (on the housekeeping CPUs) while the cycle period (-i) steps up from 1kHz to 5kHz.
#
# Scope (important): this is a smoke test of the prerequisite, "RT scheduling jitter",
# not of the EtherCAT bus itself, and it runs before the application: it refuses while a user RT
# thread is on the isolated CPU. Real WKC/frame loss/DC drift need live slaves + a cyclic PDO
# app to measure. The master may be up: with no application its slaves stay short of OP (the
# configuration that takes them there is applied only after an application calls
# ecrt_master_activate(), ecrt.h), and a change of their states under load is still observed
# best-effort as a sign of link or scan trouble. The short run (default 20s/step) is for fast detection of regressions and misconfiguration; it does not
# replace production qualification (runs of hours to tens of hours).

# --- Load test helpers -------------------------------------------------------
# Cycle sweep table. Rate / period (µs) / warn and fail max-latency thresholds (µs).
# To adjust thresholds, change only this one place (index 0-4 = 1-5kHz).
readonly RT_SWEEP_RATES=(1 2 3 4 5)
readonly RT_SWEEP_PERIODS=(1000 500 333 250 200)
readonly RT_SWEEP_WARNS=(100 80 50 50 40)
readonly RT_SWEEP_FAILS=(300 150 100 80 60)

# Compute the housekeeping cpulist excluding the isolated CPU → HK_MASK (comma list)/HK_NCPU.
compute_hk() {
    local rt="$1" c keep=() onl
    read -ra onl <<< "$(cpu_online_cores)"
    for c in "${onl[@]}"; do [[ "$c" == "$rt" ]] || keep+=("$c"); done
    HK_MASK="$(IFS=,; echo "${keep[*]}")"; HK_NCPU="${#keep[@]}"
}
# Put a representative system load on the housekeeping CPUs. It runs in its own process group via setsid
# so stop_hk_load reliably cleans up the whole group (child workers included). Do not use a bare `wait`:
# if a stress-ng --io worker gets stuck in sync() D-state, wait blocks forever (cleanup hangs).
HK_LOAD_PIDS=()
start_hk_load() {
    local mask="$1" ncpu="$2" c
    HK_LOAD_PIDS=()
    if command -v stress-ng &>/dev/null; then
        detail "Load generator: stress-ng (--cpu ${ncpu} --vm 1 --io 1) @ CPU ${mask}"
        setsid taskset -c "$mask" stress-ng --cpu "$ncpu" --vm 1 --vm-bytes 512M --io 1 >> "$CMD_LOG" 2>&1 &
        HK_LOAD_PIDS+=($!); disown   # drop job tracking so kill does not leak a "Killed" notice to the terminal
    else
        detail "Load generator: built-in fallback (busy-loop ×${ncpu} + dd) @ CPU ${mask}"
        for (( c=0; c<ncpu; c++ )); do
            setsid taskset -c "$mask" bash -c 'while :; do :; done' >/dev/null 2>&1 &
            HK_LOAD_PIDS+=($!); disown
        done
        setsid taskset -c "$mask" bash -c 'while :; do dd if=/dev/zero of=/dev/null bs=1M count=2048 2>/dev/null; done' >/dev/null 2>&1 &
        HK_LOAD_PIDS+=($!); disown
    fi
    sleep 1   # warm up the load
}
# True while every load process started above still runs (a rate measured without them was not loaded).
hk_load_alive() {
    local p
    for p in "${HK_LOAD_PIDS[@]}"; do kill -0 "$p" 2>/dev/null || return 1; done
}
stop_hk_load() {
    [[ ${#HK_LOAD_PIDS[@]} -gt 0 ]] || return 0
    local p
    for p in "${HK_LOAD_PIDS[@]}"; do kill -TERM "-${p}" 2>/dev/null; kill -TERM "${p}" 2>/dev/null; done
    for p in "${HK_LOAD_PIDS[@]}"; do kill -KILL "-${p}" 2>/dev/null; kill -KILL "${p}" 2>/dev/null; done
    HK_LOAD_PIDS=()
}
# Return a summary of EtherCAT slave AL (Application Layer) states as "N×STATE".
# AL state = each slave's EtherCAT operating stage: INIT → PREOP (preparation) →
# SAFEOP (communicating, outputs not applied = no-motion) → OP (normal operation). E.g. "7×PREOP" = all 7 in PREOP.
ec_al_summary() {
    ethercat slaves 2>/dev/null \
        | awk '$3 ~ /^(INIT|PREOP|SAFEOP|BOOT|OP)$/ {print $3}' \
        | sort | uniq -c | awk '{printf "%s×%s ", $1, $2}'
}
# Prompt for the run time per step (seconds) and return it on stdout. Enter=default
# (the caller seeds the default from env SWEEP_SEC). Invalid input falls back to the default.
# Note: the return value is captured with $(), so prompts/warnings go only to /dev/tty, not stdout.
ask_duration() {
    local def="${1:-20}" ans
    read -rp "  Run time per step (s) [Enter=default ${def}]: " ans </dev/tty
    if [[ -z "$ans" ]]; then echo "$def"
    elif [[ "$ans" =~ ^[0-9]+$ && "$ans" -ge 1 ]]; then echo "$ans"
    else echo -e "         ${YELLOW}⚠ invalid input → using default ${def}s${NC}" >/dev/tty; echo "$def"; fi
}
# ---------------------------------------------------------------------------
run_loadsweep() {
    print_box "RT jitter load test (cyclictest, 1→5 kHz)"
    echo ""

    if ! command -v cyclictest &>/dev/null; then
        die "cyclictest not found. Finish the rt-tests install in menu 1 first"
        read -rp "  Press Enter to return to the menu..." _; return
    fi
    local rt_cpu; rt_cpu="$(booted_isol_cpu)"   # same CPU as menu 1/2
    if [[ -z "$rt_cpu" ]]; then
        die "No isolated CPU. Finish menus 1 and 2 (isolcpus) first"
        read -rp "  Press Enter to return to the menu..." _; return
    fi
    # A check before the application runs: cyclictest at priority 99 on the loop's CPU would preempt
    # a running control loop (98) and measure it instead of the CPU.
    local rt_busy; rt_busy="$(iso_rt_threads "$rt_cpu")"
    if [[ -n "$rt_busy" ]]; then
        die "An RT thread is running on CPU ${rt_cpu} (${rt_busy% }). Stop it (e.g. plem drive run), then measure again"
        read -rp "  Press Enter to return to the menu..." _; return
    fi
    local dur; dur="$(ask_duration "${SWEEP_SEC:-20}")"   # seconds per step (interactive, Enter=default)

    # housekeeping = online CPUs - isolated CPU (compute_hk → HK_MASK/HK_NCPU)
    compute_hk "$rt_cpu"
    detail "Measuring on isolated CPU ${rt_cpu} · load applied to housekeeping CPUs (${HK_MASK})"
    detail "${dur}s per step (set with SWEEP_SEC). A short smoke test, not production qualification"
    detail "It measures only whether the scheduler wakes an empty timer thread on time. No EtherCAT traffic or control"
    detail "computation runs, so a real loop jitters more. max is a peak that grows the longer it runs."
    echo ""

    # Thresholds come from the shared table above (RT_SWEEP_*) (warn threshold / fail limit).
    local rates=("${RT_SWEEP_RATES[@]}")
    local periods=("${RT_SWEEP_PERIODS[@]}")
    local warns=("${RT_SWEEP_WARNS[@]}")
    local fails=("${RT_SWEEP_FAILS[@]}")

    # Start the representative load (shared helper; Ctrl-C also cleans up via group kill)
    trap 'stop_hk_load; trap - INT TERM HUP; echo; return 130' INT TERM
    trap 'stop_hk_load; exit 129' HUP   # terminal closed: nobody reads the menu, so stop the load and exit
    start_hk_load "$HK_MASK" "$HK_NCPU"

    # --- EtherCAT best-effort state before (only when the master is up) ---
    local ec_live=false ec_before=""
    if command -v ethercat &>/dev/null && systemctl is-active ethercat.service &>/dev/null; then
        ec_live=true
        ec_before="$(ec_al_summary)"
        detail "EtherCAT master active: watching for slave state changes under load (slaves reach OP only under a running application)"
    fi

    # ASCII headers only: µs is multi-byte, so printf widths would not match display width.
    # Columns: avg=typical · p99/p99.9=upper tail (occasional spikes) · max=peak. Verdict uses max (µs).
    echo ""
    printf "  ${BOLD}%-6s %7s %6s %6s %7s %6s  %s${NC}\n" "rate" "period" "avg" "p99" "p99.9" "max" "verdict"
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 64))${NC}"

    local worst="pass" i
    for i in "${!rates[@]}"; do
        local khz="${rates[$i]}" per="${periods[$i]}" w="${warns[$i]}" f="${fails[$i]}"
        printf "  %-6s %7s ${DIM}measuring...${NC}" "${khz}kHz" "$per"
        # -h 1000: 0-1000µs histogram → avg/p99/p99.9/max and overflow (>1ms) counts
        local out; out=$(cyclictest -t1 -a "$rt_cpu" -p99 -i "$per" -m -h 1000 -q -D "${dur}" 2>/dev/null) || true
        local stats; stats=$(awk '
            /^[0-9]+[ \t]+[0-9]+$/ { u=$1+0; b[u]=$2+0; tot+=$2; if(u>mu)mu=u; next }
            /^# Avg Latencies/ { avg=$NF+0 }
            /^# Max Latencies/ { mx=$NF+0 }
            /^# Histogram Overflows/ { ovf=$NF+0 }
            END {
                if (tot == 0) { print "ERR"; exit }
                t99=tot*0.99; t999=tot*0.999; cum=0; p99=-1; p999=-1
                for (u=0; u<=mu; u++) { cum+=b[u]; if(p99<0&&cum>=t99)p99=u; if(p999<0&&cum>=t999)p999=u }
                printf "%d %d %d %d %d", avg, p99, p999, mx, ovf
            }' <<< "$out")
        printf "\r\033[K"
        local avg p99 p999 mx ovf verdict color
        if ! hk_load_alive || [[ "$stats" == "ERR" || -z "$stats" ]]; then
            avg="?"; p99="?"; p999="?"; mx="?"; verdict="ERR"; color="$YELLOW"; worst="fail"
        else
            read -r avg p99 p999 mx ovf <<< "$stats"
            if   (( mx > f )); then verdict="FAIL"; color="$RED"; worst="fail"
            elif (( mx > w )); then verdict="warn"; color="$YELLOW"; [[ "$worst" == "pass" ]] && worst="warn"
            else verdict="pass"; color="$GREEN"; fi
            [[ "${ovf:-0}" -gt 0 ]] && verdict="${verdict}!"   # samples >1ms exist (tail clipped)
        fi
        # "w<"/"f>" stay abbreviated to keep the row narrow; the summary below spells them out.
        printf "  %-6s %7s %6s %6s %7s ${color}%6s${NC}  ${color}%-5s${NC} ${DIM}(w<%s f>%s)${NC}\n" \
            "${khz}kHz" "$per" "$avg" "$p99" "$p999" "$mx" "$verdict" "$w" "$f"
        [[ "$verdict" == FAIL* ]] && { detail "Jitter limit exceeded at this rate. Skipping the higher rates."; break; }
        hk_load_alive || { fail "The load generator stopped during this rate (see ${CMD_LOG}). Stopping the sweep."; break; }
    done

    # --- EtherCAT best-effort state after ---
    if $ec_live; then
        local ec_after; ec_after="$(ec_al_summary)"
        echo ""
        if [[ "$ec_before" == "$ec_after" ]]; then
            info "EtherCAT slave states held: ${ec_after:-N/A} (no change under load)"
        else
            fail "EtherCAT slave states changed during the load test: '${ec_before}'→'${ec_after}'"
            worst="fail"
        fi
    fi

    stop_hk_load
    trap - INT TERM HUP

    echo ""
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 56))${NC}"
    case "$worst" in
        pass) echo -e "  ${GREEN}${BOLD}✔ All rates pass${NC}: every rate up to 5 kHz (200 µs) stayed within its warn threshold in this run." ;;
        warn) echo -e "  ${YELLOW}${BOLD}⚠ pass (warn)${NC}: above the warn threshold but within the fail limit. A real soak is recommended for higher rates." ;;
        fail) echo -e "  ${RED}${BOLD}✘ Some rates failed${NC}: do not use those rates; recheck the isolation and power settings." ;;
    esac
    detail "Verdict is based on max (worst case). p99.9=worst of about 1 in 1000 cycles, max=overall peak (${dur}s observed)."
    detail "(w<N f>N) = warn threshold / fail limit for that rate, in µs."
    detail "For precise tail and thermal stability, run longer with a larger SWEEP_SEC (e.g. 600). '!' = some samples exceeded 1 ms (outside the histogram)."
    echo -e "  ${DIM}Note: this menu is an RT jitter load test run before the application. It does not measure bus metrics${NC}"
    echo -e "  ${DIM}(WKC/frame loss/DC drift); check those with your control application.${NC}"
    echo ""
    read -rp "  Press Enter to return to the menu..." _
}

# ============================================================================
#  Menu 5: USB adapter drivers
# ============================================================================
# L4T kernels ship some common USB adapter drivers disabled. Menu 5 builds them on the device for the
# running kernel, from the upstream stable source of the same kernel version (never a prebuilt module),
# and installs them under updates/. Their USB ID aliases load them when an adapter is plugged in.
# Everything comes from the running kernel: uname -r → its header tree → the upstream tag in its Makefile.

# module|CONFIG symbol|in-tree source: candleLight/gs_usb-compatible USB-CAN, CH340/CH341 and PL2303 USB-serial.
readonly USB_DRIVERS=(
    "gs_usb|CONFIG_CAN_GS_USB|drivers/net/can/usb/gs_usb.c"
    "ch341|CONFIG_USB_SERIAL_CH341|drivers/usb/serial/ch341.c"
    "pl2303|CONFIG_USB_SERIAL_PL2303|drivers/usb/serial/pl2303.c"
)
readonly USB_DRIVER_SUBDIR="plem"   # /lib/modules/<release>/updates/<subdir>
readonly KERNEL_STABLE_GIT="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git"
USB_RESULTS=() USB_NOTES=()   # per USB_DRIVERS index: done/skipped/failed, and what the summary says

# Upstream version of a kernel tree from its top Makefile: 5.15.148, or 6.8 when SUBLEVEL is 0 (the tag is v6.8).
# `make kernelversion` adds the local suffix (5.15.148-rt-tegra), which is no upstream tag.
kernel_upstream_version() {  # <Makefile>
    awk '$2 == "=" && $1 ~ /^(VERSION|PATCHLEVEL|SUBLEVEL)$/ && !($1 in v) { v[$1] = $3 }
         END {
             if (v["VERSION"] !~ /^[0-9]+$/ || v["PATCHLEVEL"] !~ /^[0-9]+$/ || v["SUBLEVEL"] !~ /^[0-9]+$/) exit 1
             printf "%s.%s%s\n", v["VERSION"], v["PATCHLEVEL"], (v["SUBLEVEL"] == 0 ? "" : "." v["SUBLEVEL"])
         }' "$1" 2>/dev/null
}

# Files a source includes with #include "name" from its own directory.
local_includes() {  # <source file>
    sed -nE 's/^[[:space:]]*#[[:space:]]*include[[:space:]]*"([^"/]+)".*/\1/p' "$1"
}

# git.kernel.org can answer a path it does not have with an HTML page and status 200, so a fetched file
# counts as kernel source only when its first line is an SPDX tag (every file of these drivers has one).
is_kernel_source() {  # <file>
    local first=""
    IFS= read -r first 2>/dev/null <"$1"
    [[ "$first" == *SPDX-License-Identifier* ]]
}

vermagic_matches() {  # <vermagic> <kernel release>: the first vermagic word is the release a module was built for
    [[ -n "$2" && "${1%% *}" == "$2" ]]
}

# One driver for kernel <release>, from that kernel's config on stdin: built-in, in-tree (a module of the kernel
# itself), installed (a module modinfo finds with this release's vermagic), or missing. Reads only.
usb_driver_state() {  # <module> <CONFIG symbol> <release>
    case "$(sed -n "s/^${2}=\([ym]\)$/\1/p")" in
        y) echo built-in; return ;;
        m) echo in-tree; return ;;
    esac
    if vermagic_matches "$(modinfo -k "$3" -F vermagic "$1" 2>/dev/null)" "$3"; then echo installed; else echo missing; fi
}

usb_driver_note() {  # <state from usb_driver_state> <release>: what menus 2 and 5 say about it
    case "$1" in
        built-in)  echo "built into the kernel" ;;
        in-tree)   echo "module of this kernel" ;;
        installed) echo "installed for ${2}" ;;
        *)         echo "missing for ${2}: run menu 5" ;;
    esac
}

# Names of the drivers missing for the running kernel (space separated). Reads only.
usb_drivers_missing() {
    local krel kcfg d mod sym
    krel="$(uname -r)"; kcfg="$(zcat /proc/config.gz 2>/dev/null)"
    for d in "${USB_DRIVERS[@]}"; do
        IFS='|' read -r mod sym _ <<<"$d"
        [[ "$(usb_driver_state "$mod" "$sym" "$krel" <<<"$kcfg")" == missing ]] && printf '%s ' "$mod"
    done
}

# Records drivers as failed: what failed and why ($1), how to fix it ($2). Shown now and again in the summary.
usb_failed() {  # <why> <fix> <index>...
    local why="$1" fix="$2" i; shift 2
    fail "$why"; detail "Fix: ${fix}"
    for i in "$@"; do USB_RESULTS[i]="failed"; USB_NOTES[i]="${why}. Fix: ${fix}"; done
}

usb_fetch() {  # <index> <tree path> <tag> <out file>
    run_cmd "Download of ${2}" curl -fsSL --retry 2 -o "$4" "${KERNEL_STABLE_GIT}/plain/${2}?h=${3}" || {
        usb_failed "Download of ${2} at ${3} failed" \
            "check the connection to git.kernel.org and that tag ${3} exists in ${KERNEL_STABLE_GIT}, then run menu 5 again" "$1"
        return 1
    }
    is_kernel_source "$4" || {
        usb_failed "${2} at ${3} is not kernel source (the server sent another page)" \
            "check that ${3} is a tag of ${KERNEL_STABLE_GIT}, then run menu 5 again" "$1"
        return 1
    }
}

usb_driver_build() {  # <index> <release> <tag> <work dir>: fetch, build, check and install one driver
    local i=$1 krel=$2 tag=$3 mod path inc
    IFS='|' read -r mod _ path <<<"${USB_DRIVERS[$1]}"
    local dir="${4}/${mod}" dest="/lib/modules/${krel}/updates/${USB_DRIVER_SUBDIR}"
    mkdir -p "$dir" || { usb_failed "Cannot create ${dir}" "check free space in /tmp, then run menu 5 again" "$i"; return 1; }
    usb_fetch "$i" "$path" "$tag" "${dir}/${mod}.c" || return 1
    for inc in $(local_includes "${dir}/${mod}.c"); do
        usb_fetch "$i" "${path%/*}/${inc}" "$tag" "${dir}/${inc}" || return 1
    done
    echo "obj-m := ${mod}.o" >"${dir}/Makefile"
    run_cmd "Build of ${mod}" make -C "/lib/modules/${krel}/build" M="$dir" modules || {
        usb_failed "Build of ${mod} against /lib/modules/${krel}/build failed" \
            "see the build output in ${CMD_LOG}; the kernel headers for ${krel} must be complete" "$i"
        return 1
    }
    local vm; vm="$(modinfo -F vermagic "${dir}/${mod}.ko" 2>/dev/null)"
    vermagic_matches "$vm" "$krel" || {
        usb_failed "${mod}.ko was built for '${vm%% *}', not the running kernel ${krel}" \
            "reinstall the kernel headers for ${krel}, then run menu 5 again" "$i"
        return 1
    }
    run_cmd "Install of ${mod}.ko" install -D -m 0644 "${dir}/${mod}.ko" "${dest}/${mod}.ko" || {
        usb_failed "Could not install ${mod}.ko into ${dest}" "check that ${dest} is writable, then run menu 5 again" "$i"
        return 1
    }
    info "Built from ${tag} and installed"
    USB_RESULTS[i]="done"; USB_NOTES[i]="built from ${tag}"
}

usb_summary() {
    local i mod sym
    echo ""
    echo -e "  ${BOLD}USB adapter drivers for $(uname -r)${NC}"
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"
    for i in "${!USB_DRIVERS[@]}"; do
        mod="${USB_DRIVERS[$i]%%|*}"
        case "${USB_RESULTS[$i]:-}" in
            done)    sym="$SYM_OK" ;;
            skipped) sym="$SYM_SKIP" ;;
            failed)  sym="$SYM_FAIL" ;;
            *)       sym="?" ;;
        esac
        printf "  %b  %-8s %s\n" "$sym" "$mod" "${USB_NOTES[$i]:-}"
    done
    echo -e "  ${DIM}$(printf '─%.0s' $(seq 1 48))${NC}"
}

run_usb_drivers() {
    clear
    print_box "Menu 5: Install USB adapter drivers"
    print_env
    echo ""
    detail "Builds the USB-CAN (gs_usb) and USB-serial (ch341, pl2303) drivers this kernel lacks,"
    detail "from the upstream source of the same kernel version, downloaded from git.kernel.org."
    echo ""
    read -rp "  Continue? (Y/n): " confirm
    [[ "$confirm" =~ ^[Nn]$ ]] && return

    USB_RESULTS=(); USB_NOTES=()
    local krel kcfg i mod sym st todo=()
    krel="$(uname -r)"
    kcfg="$(zcat /proc/config.gz 2>/dev/null)"
    echo ""
    if [[ -z "$kcfg" ]]; then
        usb_failed "Cannot read the running kernel's configuration (/proc/config.gz)" \
            "boot a kernel built with CONFIG_IKCONFIG_PROC, then run menu 5 again" "${!USB_DRIVERS[@]}"
    else
        for i in "${!USB_DRIVERS[@]}"; do
            IFS='|' read -r mod sym _ <<<"${USB_DRIVERS[$i]}"
            st="$(usb_driver_state "$mod" "$sym" "$krel" <<<"$kcfg")"
            if [[ "$st" == missing ]]; then todo+=("$i"); continue; fi
            USB_RESULTS[i]="skipped"; USB_NOTES[i]="$(usb_driver_note "$st" "$krel")"
            skip "${mod}: ${USB_NOTES[$i]}"
        done
    fi

    if [[ ${#todo[@]} -gt 0 ]]; then
        local ver="" work=""
        if ! kernel_headers_match "These drivers"; then
            usb_failed "The kernel headers for ${krel} are missing or do not match it (see above)" \
                "install the kernel headers package of the running kernel, then run menu 5 again" "${todo[@]}"
        elif ! command -v curl &>/dev/null || ! command -v make &>/dev/null || ! command -v gcc &>/dev/null; then
            usb_failed "curl, make or gcc is not installed" "sudo apt install curl build-essential, then run menu 5 again" "${todo[@]}"
        elif ! ver="$(kernel_upstream_version "/lib/modules/${krel}/build/Makefile")"; then
            usb_failed "Cannot read the kernel version from /lib/modules/${krel}/build/Makefile" \
                "reinstall the kernel headers for ${krel}, then run menu 5 again" "${todo[@]}"
        elif ! work="$(mktemp -d /tmp/plem-usb-drivers-XXXXXX)"; then
            usb_failed "Cannot create a build directory in /tmp" "check free space in /tmp, then run menu 5 again" "${todo[@]}"
        else
            detail "Kernel ${krel}: upstream tag v${ver}"
            for i in "${todo[@]}"; do
                echo ""; echo -e "  ${BOLD}${USB_DRIVERS[$i]%%|*}${NC}"
                usb_driver_build "$i" "$krel" "v${ver}" "$work"
            done
            rm -rf "$work"
        fi

        # One depmod for all, then check that modprobe now finds each new module for this kernel.
        local built=()
        for i in "${todo[@]}"; do [[ "${USB_RESULTS[$i]}" == "done" ]] && built+=("$i"); done
        if [[ ${#built[@]} -gt 0 ]]; then
            echo ""
            if ! run_cmd "depmod" depmod "$krel"; then
                usb_failed "depmod ${krel} failed: modprobe does not know the new modules" "run sudo depmod ${krel}" "${built[@]}"
            else
                for i in "${built[@]}"; do
                    IFS='|' read -r mod sym _ <<<"${USB_DRIVERS[$i]}"
                    [[ "$(usb_driver_state "$mod" "$sym" "$krel" <<<"$kcfg")" == installed ]] \
                        || usb_failed "${mod} was installed but modinfo -k ${krel} does not find it" \
                            "check ${CMD_LOG} and /lib/modules/${krel}/updates/${USB_DRIVER_SUBDIR}" "$i"
                done
            fi
        fi
    fi

    usb_summary
    if [[ " ${USB_RESULTS[*]} " == *" failed "* ]]; then
        echo -e "  ${RED}${BOLD}Some drivers are not available.${NC} Fix the cause above and run menu 5 again."
        echo -e "  ${DIM}Log: ${CMD_LOG}${NC}"
    else
        echo -e "  ${GREEN}${BOLD}All drivers are available.${NC} Each one carries its adapters' USB IDs (modinfo -F alias)."
    fi
    echo ""
    read -rp "  Press Enter to return to the menu..." _
}

# ============================================================================
#  Main menu
# ============================================================================

main_menu() {
    while true; do
        detect_env
        clear
        print_box "Jetson RT Kernel Setup Wizard"
        print_env

        local p1_status p1_en p2_status p2_en p3_status p3_en p4_status p4_en p5_status p5_en

        # Phase 1 status
        # Tell apart the reasons for phase1_is_complete=false. If already booted into RT
        # (RT_BOOTED), what remains is CPU tuning (services/isolcpus), so a re-run is needed,
        # not a reboot. "reboot required" is right only when RT is installed but the stock kernel is still booted.
        if phase1_is_complete; then
            p1_status="${SYM_OK} done"
        elif $RT_BOOTED; then
            p1_status="${YELLOW}⚠${NC} incomplete ${DIM}(re-run required)${NC}"
        elif $RT_INSTALLED; then
            p1_status="${YELLOW}⚠${NC} reboot required"
        else
            p1_status="${SYM_READY} ready"
        fi
        p1_en="true"

        # Phase 2 status
        if phase1_is_complete && phase2_is_complete; then
            p2_status="${SYM_OK} done"
            p2_en="true"
        elif phase1_is_complete; then
            p2_status="${SYM_READY} ready"
            p2_en="true"
        else
            # Booted into RT but incomplete means a tuning problem (Phase 1 incomplete), not a reboot problem.
            if $RT_INSTALLED && ! $RT_BOOTED; then
                p2_status="${SYM_LOCK} locked ${DIM}(reboot required)${NC}"
            else
                p2_status="${SYM_LOCK} locked ${DIM}(menu 1 incomplete)${NC}"
            fi
            p2_en="false"
        fi

        # Phase 3 status
        # Be precise about the lock reason: if booted into RT, the RT kernel requirement is met and Phase 1
        # tuning is what remains. Show "RT kernel required" only when there is no RT kernel, to avoid confusion.
        if phase3_is_complete; then
            p3_status="${SYM_OK} done"
            p3_en="true"
        elif phase1_is_complete; then
            p3_status="${SYM_READY} ready"
            p3_en="true"
        else
            if $RT_BOOTED; then
                p3_status="${SYM_LOCK} locked ${DIM}(menu 1 incomplete)${NC}"
            else
                p3_status="${SYM_LOCK} locked ${DIM}(RT kernel required)${NC}"
            fi
            p3_en="false"
        fi

        # Menu 4 needs a CPU isolated at runtime (/sys/devices/system/cpu/isolated); run_loadsweep
        # uses it and refuses while a user RT thread runs there. EtherCAT is optional: if the master is up,
        # slave AL states are watched too.
        if [[ -n "$(cat /sys/devices/system/cpu/isolated 2>/dev/null)" ]]; then
            p4_status="${SYM_READY} ready"; p4_en="true"
        elif $RT_INSTALLED && ! $RT_BOOTED; then
            p4_status="${SYM_LOCK} locked ${DIM}(reboot required)${NC}"; p4_en="false"
        else
            p4_status="${SYM_LOCK} locked ${DIM}(no isolated CPU)${NC}"; p4_en="false"
        fi

        # Menu 5 builds for whichever kernel is running, RT or not.
        if [[ -z "$(usb_drivers_missing)" ]]; then
            p5_status="${SYM_OK} done"; p5_en="true"
        else
            p5_status="${SYM_READY} ready"; p5_en="true"
        fi

        local items=(
            "1. Install RT kernel (before reboot)"           "$p1_status"  "$p1_en"
            "2. Verify setup (after reboot)"                 "$p2_status"  "$p2_en"
            "3. Install EtherCAT master"                     "$p3_status"  "$p3_en"
            "4. RT jitter load test (cyclictest, 1→5 kHz)"   "$p4_status"  "$p4_en"
            "5. Install USB adapter drivers (CAN, serial)"   "$p5_status"  "$p5_en"
        )

        local choice
        choice=$(select_menu "${items[@]}")

        case "$choice" in
            0) run_phase1 ;;
            1) run_phase2 ;;
            2) run_phase3 ;;
            3) run_loadsweep ;;
            4) run_usb_drivers ;;
            *) clear; exit 0 ;;
        esac
    done
}

# ============================================================================
#  Device reporting (docs/telemetry.md)
# ============================================================================

FLEET_TOOL="${WIZARD_DIR}/../../plem_drive/plem/drive/fleet.py"
CT_MAX_US="" CT_AVG_US=""

# Asked once. Nothing is collected unless the operator picks Yes.
fleet_consent() {
    [[ -f "$FLEET_TOOL" && -t 0 && ! -e /etc/plem/fleet.json && ! -e /etc/plem/license.key ]] || return 0
    [[ "${PLEM_TELEMETRY:-}" != "off" ]] || return 0
    local choice
    while true; do
        clear
        python3 "$FLEET_TOOL" notice
        read -rsN 1000 -t 0.05 _ </dev/tty || true  # keys pressed before the question must not answer it; -n would stop at the first Enter
        choice=$(select_menu "Yes" "" true "No" "" true "View this device's report" "" true)
        case "$choice" in
            0) python3 "$FLEET_TOOL" on >>"$CMD_LOG" 2>&1 || warn "Could not turn device reporting on"; break ;;
            2) clear; python3 "$FLEET_TOOL" show; read -rp "  Press Enter to go back..." _ ;;
            *) python3 "$FLEET_TOOL" decline >>"$CMD_LOG" 2>&1 || true; break ;;
        esac
    done
    clear
    echo "  Optional: get notices about newly validated JetPack versions and early access to PLEM Pro."
    echo "  WIM may email you about these. Reply to any of them to stop. If device reporting is on,"
    echo "  the address is linked to this device. Remove it any time: sudo plem-fleet forget-email"
    local email=""
    read -rp "  Email (Enter to skip): " email </dev/tty || true
    if [[ -n "$email" ]]; then
        python3 "$FLEET_TOOL" contact "$email" >>"$CMD_LOG" 2>&1 || warn "Could not register that address"
    fi
}

# Runs on exit. Only step names and results: a failure here never changes the wizard's outcome.
fleet_report() {
    [[ -f "$FLEET_TOOL" ]] || return 0
    (( ${#PHASE1_RESULTS[@]} + ${#P2_LOG[@]} + ${#PHASE3_RESULTS[@]} > 0 )) || return 0
    local i entry
    {
        for i in "${!PHASE1_RESULTS[@]}"; do echo "1|${PHASE1_NAMES[$i]}|${PHASE1_RESULTS[$i]}"; done
        for entry in "${P2_LOG[@]}"; do echo "2|${entry}"; done
        for i in "${!PHASE3_RESULTS[@]}"; do echo "3|${PHASE3_NAMES[$i]}|${PHASE3_RESULTS[$i]}"; done
        [[ -z "$CT_MAX_US" ]] || echo "cyclictest|${CT_MAX_US}|${CT_AVG_US}"
    } | timeout 15 python3 "$FLEET_TOOL" report-install >>"$CMD_LOG" 2>&1 || true
}

# ============================================================================
#  Entry point
# ============================================================================

usage() {
    echo "Usage: sudo bash $0 [--baseline]"
    echo "  --baseline  make menu 1 default to disabling unused services (for a dedicated control computer)"
}
parse_args() {
    local arg
    for arg in "$@"; do
        case "$arg" in
            --baseline) BASELINE_DEFAULT=true ;;
            -h|--help)  usage; exit 0 ;;
            *)          echo "Unknown option: ${arg}" >&2; usage >&2; exit 2 ;;
        esac
    done
}

parse_args "$@"

if [[ $EUID -ne 0 ]]; then
    echo -e "\n  ${RED}${BOLD}Error:${NC} root is required. Run with ${BOLD}sudo bash $0${NC}.\n"
    exit 1
fi


fleet_consent
main_menu
