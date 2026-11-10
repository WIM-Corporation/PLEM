#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# plem-nvpps-tsc-unlock: turn off the nvpps PTP-TSC hardware lock (takes effect on reboot).
#
# At boot, nvpps locks the TSC (the base of every Linux clock) to the PHC of the built-in RJ45 port (MGBE0:
# end0 on JetPack 7.2.1). With no PTP grandmaster on that port, the counter periodically
# slips by ±140 ppm, and on an EtherCAT master that feeds host time to the DC reference clock, the DC error
# grows to hundreds of µs. The DT property ptp_tsc_sync_dis is read only at probe time. There is no runtime
# switch.
#
# The property is written directly into /boot/dtb/kernel_*.dtb, and each label without FDT (except
# primary) gets an FDT line pointing at that file. On a USB (Jetson ISO) install, which has neither, the file is
# first created from the /boot DTB that matches the booted slot's kernel-dtb partition (the DTB the firmware
# boots there), so the label keeps that DTB, now with the property.
# Labels that a third-party camera driver package creates also use this file as FDT, so the setting
# survives that package's install and upgrade. If the property is lost (the L4T dtbs package restores the original, or an extlinux
# without FDT is restored), the lock simply turns back on and the system still boots. Never add FDT to primary:
# the nvidia-l4t-bootloader postinst then deletes every FDT line in the file.
#
# On a device that uses the built-in port for PTP, create /etc/plem/nvpps-lock-keep and run again to remove the property.
#
# Usage: plem-nvpps-tsc-unlock [--check]   (--check writes nothing and only reports the state)
# The last stderr line is the state: applied | ok | pending | warn | n/a | failed
#   applied/pending = next boot is correct, reboot required. warn = next boot is not in the wanted state either.
#   The next-boot result comes from applying the DEFAULT label's OVERLAYS to its FDT offline.
# Exit codes: 0 = check done (see the state line), 2 = precondition not met or failed before writing (DTB and extlinux unchanged),
#            3 = file replacement failed.
set -uo pipefail

NODE=/proc/device-tree/bus@0/nvpps
KEEP=/etc/plem/nvpps-lock-keep
EXT=/boot/extlinux/extlinux.conf
SECUREBOOT=/sys/firmware/efi/efivars/SecureBoot-8be4df61-93ca-11d2-aa0d-00e098032b8c
CHECK=false; [[ "${1:-}" == "--check" ]] && CHECK=true

say()    { echo "plem-nvpps-tsc-unlock: $*" >&2; }
finish() { say "$2"; exit "$1"; }  # finish <code> <state line>

[[ $# -eq 0 ]] || $CHECK || finish 2 "failed: usage: plem-nvpps-tsc-unlock [--check]"
[[ $EUID -eq 0 ]] || $CHECK || finish 2 "failed: root is required"
$CHECK || { exec 9> /run/lock/plem-nvpps.lock && flock 9; }

[[ -d "$NODE" ]] || finish 0 "n/a: no nvpps node (this board does not lock the TSC, or it boots with ACPI)"
# With Secure Boot, extlinux and the DTB must be signed, so editing them breaks boot verification.
# The efivar holds 4 bytes of attributes, then 1 byte of value.
if [[ -f "$SECUREBOOT" ]] && [[ "$(od -An -tu1 -j4 -N1 "$SECUREBOOT" | tr -d ' ')" == 1 ]]; then
    finish 0 "n/a: Secure Boot is on; set this in a signed DTB instead"
fi
for t in fdtget fdtput fdtoverlay dtc; do
    command -v "$t" &>/dev/null || finish 2 "failed: ${t} not found (device-tree-compiler)"
done

# Temp files and a seeded DTB that is not committed yet are removed on any exit, so a failure before the
# replacement leaves /boot as it was (exit 2).
dtmp="" etmp="" seeded="" seeded_dir=false
cleanup() {
    rm -f "$dtmp" "$etmp"
    [[ -n "$seeded" ]] && rm -f "$seeded" && $seeded_dir && rmdir /boot/dtb 2>/dev/null
    return 0
}
$CHECK || trap cleanup EXIT   # --check runs without the lock, so it must not delete the writer's temp files

mapfile -t dtbs < <(compgen -G '/boot/dtb/kernel_*.dtb')
# A USB (Jetson ISO) install has no /boot/dtb/ and no FDT line: the firmware boots the DTB in the booted slot's
# kernel-dtb partition. Start /boot/dtb/kernel_<board>.dtb, the name a flash uses, from the /boot/<board>.dtb
# (installed by the kernel dtbs package) whose bytes that partition starts with. The partition is padded, so
# only the file's length is compared. The compatible check below then runs on it.
if (( ${#dtbs[@]} == 0 )); then
    case "$(nvbootctrl get-current-slot 2>/dev/null)" in
        0) part=/dev/disk/by-partlabel/A_kernel-dtb ;;
        1) part=/dev/disk/by-partlabel/B_kernel-dtb ;;
        *) finish 2 "failed: no /boot/dtb/kernel_*.dtb, and nvbootctrl get-current-slot did not report the boot slot" ;;
    esac
    board=""
    for f in /boot/*.dtb; do
        [[ -f "$f" ]] && cmp -s -n "$(stat -c %s "$f")" "$f" "$part" && { board=$(basename "$f" .dtb); break; }
    done
    [[ -n "$board" ]] \
        || finish 2 "failed: no /boot/dtb/kernel_*.dtb, and no /boot/*.dtb matches ${part}, the DTB the firmware boots"
    if $CHECK; then
        dtbs=("/boot/${board}.dtb")   # judged as it would be seeded; nothing is written
    else
        [[ -d /boot/dtb ]] || seeded_dir=true
        mkdir -p /boot/dtb && cp -p "/boot/${board}.dtb" "/boot/dtb/kernel_${board}.dtb" \
            || finish 2 "failed: could not create /boot/dtb/kernel_${board}.dtb"
        seeded="/boot/dtb/kernel_${board}.dtb"; dtbs=("$seeded")
    fi
fi
(( ${#dtbs[@]} == 1 )) || finish 2 "failed: expected exactly one /boot/dtb/kernel_*.dtb (found ${#dtbs[@]})"
DTB=${dtbs[0]}
live_compat=$(tr '\0' ' ' < /proc/device-tree/compatible | sed 's/ *$//')
[[ "$(fdtget "$DTB" / compatible 2>/dev/null)" == "$live_compat" ]] \
    || finish 2 "failed: ${DTB} is not the DTB for this board (${live_compat})"

want=1; [[ -e "$KEEP" ]] && want=0
has_prop() { fdtget "$1" /bus@0/nvpps ptp_tsc_sync_dis &>/dev/null; }

# Every awk on extlinux strips CRLF before reading (if primary is not recognized, every FDT line gets deleted).
fdt_missing() {  # names of labels with no FDT line (primary excluded)
    awk '{ sub(/\r$/, "") }
        $1=="LABEL" { if (n && !f) print n; n=($2=="primary" ? "" : $2); f=0; next }
        $1=="FDT"   { f=1 }
        END         { if (n && !f) print n }
    ' "$1"
}
label_field() {  # label_field <conf> <label> <key>: the value of key in that label
    awk -v want="$2" -v key="$3" '{ sub(/\r$/, "") }
        $1=="LABEL" { cur=$2 } cur==want && $1==key { print $2; exit }' "$1"
}

# 1. Build everything to be written as temp files, and replace only after they are verified.
changed="" dtmp=/boot/dtb/.plem-nvpps.tmp dbak="/boot/dtb/.plem-bak-${DTB##*/}" etmp="${EXT}.plem-tmp"   # leading dot: outside the L4T and driver package globs
need_dtb=false; [[ $(has_prop "$DTB" && echo 1 || echo 0) != "$want" ]] && need_dtb=true
need_ext=false; (( want )) && [[ -n "$(fdt_missing "$EXT")" ]] && need_ext=true

if ! $CHECK && $need_ext; then
    awk -v dtb="$DTB" '{ sub(/\r$/, "") }
        NR==FNR { if ($1=="LABEL") { n++; lab[n]=$2 } if ($1=="FDT") hasfdt[n]=1; next }
        $1=="LABEL" { m++ }
        { print }
        $1=="LINUX" && lab[m]!="primary" && !hasfdt[m] {
            match($0, /^[[:space:]]*/); print substr($0, 1, RLENGTH) "FDT " dtb
        }
    ' "$EXT" "$EXT" > "$etmp" || finish 2 "failed: extlinux edit failed"
    [[ -z "$(fdt_missing "$etmp")" ]] \
        || finish 2 "failed: a label has no LINUX line, so FDT could not be added (DTB and extlinux unchanged)"
fi
if ! $CHECK && $need_dtb; then
    cp -p "$DTB" "$dtmp" || finish 2 "failed: DTB temp copy failed"
    cp -p "$DTB" "$dbak" || finish 2 "failed: DTB backup copy failed"   # previous DTB, like extlinux's .plem-bak
    if (( want )); then fdtput "$dtmp" /bus@0/nvpps ptp_tsc_sync_dis
    else fdtput -d "$dtmp" /bus@0/nvpps ptp_tsc_sync_dis; fi
    got=0; has_prop "$dtmp" && got=1
    if (( got != want )) || ! dtc -I dtb -O dtb -o /dev/null "$dtmp" 2>/dev/null; then
        finish 2 "failed: DTB edit verification failed (DTB and extlinux unchanged)"
    fi
fi

# 2. Replace. A rename within the same directory, so an interruption leaves either the old file or the new one.
if ! $CHECK && $need_dtb; then
    sync "$dtmp" && mv "$dtmp" "$DTB" || finish 3 "failed: replacing ${DTB} failed"
    [[ -n "$seeded" ]] && changed+="${DTB} created from /boot/${board}.dtb, "
    seeded=""   # committed: kept on exit
    changed+="DTB (previous content in ${dbak}) "
fi
if ! $CHECK && $need_ext; then
    cp -p "$EXT" "${EXT}.plem-bak" && chmod 644 "$etmp" && sync "$etmp" && mv "$etmp" "$EXT" \
        || finish 3 "failed: replacing extlinux failed"
    changed+="extlinux (previous content in ${EXT}.plem-bak) "
fi

# 3. Next-boot check: the result of applying the DEFAULT label's FDT + OVERLAYS offline.
state=$([[ $want == 1 ]] && echo "TSC lock off" || echo "TSC lock kept (${KEEP})")
if $CHECK && { $need_dtb || $need_ext; }; then
    finish 0 "warn: ${state} is not applied; run sudo plem-nvpps-tsc-unlock and reboot"
fi
def=$(awk '{ sub(/\r$/, "") } $1=="DEFAULT" { print $2; exit }' "$EXT")
def_fdt=$(label_field "$EXT" "$def" FDT)
def_ovl=$(label_field "$EXT" "$def" OVERLAYS)
if [[ "$def" == primary || -z "$def_fdt" ]]; then
    finish 0 "warn: ${changed:+(${changed% } written) }${state} will not take effect at the next boot: DEFAULT(${def:-none}) does not use FDT (it is primary or has no FDT line)"
fi
next=0
if [[ -z "$def_ovl" ]]; then
    has_prop "$def_fdt" && next=1
else
    ptmp=$(mktemp); IFS=',' read -ra ovls <<< "$def_ovl"
    if ! fdtoverlay -i "$def_fdt" -o "$ptmp" "${ovls[@]}" 2>/dev/null; then
        rm -f "$ptmp"; finish 0 "warn: ${changed:+(${changed% } written) }test apply of OVERLAYS for DEFAULT(${def}) failed, extlinux may be ignored at boot"
    fi
    has_prop "$ptmp" && next=1; rm -f "$ptmp"
fi
if (( next != want )); then
    why="FDT=${def_fdt} of DEFAULT(${def})"; [[ -n "$def_ovl" ]] && why+=", OVERLAYS change the property (${def_ovl})"
    finish 0 "warn: ${changed:+(${changed% } written) }${state} will not take effect at the next boot: ${why}"
fi
for o in $(awk -v dtb="$DTB" '{ sub(/\r$/, "") } $1=="FDT" && $2!=dtb { print $2 }' "$EXT" | sort -u); do
    say "warning: labels that use another DTB (${o}) do not get this setting"
done
awk '{ sub(/\r$/, "") } $1=="LABEL" { p=($2=="primary") } p && $1=="FDT" { f=1 } END { exit !f }' "$EXT" \
    && say "warning: the primary label has an FDT line; an L4T bootloader update will delete every FDT line"

live=0; [[ -e "${NODE}/ptp_tsc_sync_dis" ]] && live=1
if [[ -n "$changed" ]]; then finish 0 "applied: ${state}, takes effect on reboot (${changed% })"
elif (( live != want )); then finish 0 "pending: ${state}, written to disk, reboot required"
else finish 0 "ok: ${state}"; fi
