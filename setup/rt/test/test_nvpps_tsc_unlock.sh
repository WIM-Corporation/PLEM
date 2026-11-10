#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# Behavior test of nvpps-tsc-unlock.sh on small fake device trees. The script runs from a copy whose
# fixed paths (/proc/device-tree, /boot, /etc/plem, /run/lock, the Secure Boot efivar) point into a temp
# directory, so no real system file is touched. Needs root (the script refuses otherwise) and the
# device-tree-compiler tools; without them it skips and says why.
#
# Layouts:
#   flash  /boot/dtb/kernel_<board>.dtb exists (flash script install)
#   usb    no /boot/dtb/, the board DTB only in /boot/<board>.dtb, no FDT line (USB installer). The
#          A/B_kernel-dtb partitions hold the DTB the firmware boots; a fake nvbootctrl reports the slot.
# What this does not see: the real partitions and nvbootctrl, and the boot itself.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${HERE}/../nvpps-tsc-unlock.sh"
fail() { echo "FAIL: $*" >&2; exit 1; }

for t in dtc fdtget fdtput fdtoverlay; do
    command -v "$t" >/dev/null || { echo "SKIP: ${t} not found (install device-tree-compiler)"; exit 0; }
done
[[ $EUID -eq 0 ]] || { echo "SKIP: needs root (the script refuses to write as a normal user)"; exit 0; }

BOARD="tegra234-p3737-0000+p3701-0005-nv"
COMPAT="nvidia,p3737-0000+p3701-0005 nvidia,tegra234"

# make_dtb <out> <compatible words…>
make_dtb() {
    local out="$1"; shift
    local c="" w; for w in "$@"; do c+="${c:+, }\"${w}\""; done
    printf '/dts-v1/;\n/ {\n\tcompatible = %s;\n\tbus@0 {\n\t\tnvpps {\n\t\t\tstatus = "okay";\n\t\t};\n\t};\n};\n' "$c" \
        | dtc -I dts -O dtb -o "$out" - 2>/dev/null || fail "dtc could not build ${out}"
}

# setup <layout> → prints the temp root; builds the fake tree, boot files and the patched script copy
setup() {
    local layout="$1" r; r="$(mktemp -d)"
    mkdir -p "$r/dt/bus@0/nvpps" "$r/boot/extlinux" "$r/etc" "$r/run" "$r/part" "$r/bin"
    # shellcheck disable=SC2086  # word-split the compatible list into NUL-separated entries
    printf '%s\0' $COMPAT > "$r/dt/compatible"
    # shellcheck disable=SC2086
    make_dtb "$r/boot/${BOARD}.dtb" $COMPAT
    # Another DTB with the same compatible but other content (a variant such as -super).
    cp "$r/boot/${BOARD}.dtb" "$r/boot/${BOARD}-super.dtb"; fdtput -c "$r/boot/${BOARD}-super.dtb" /super
    set_part "$r" A "$r/boot/${BOARD}.dtb"; set_part "$r" B "$r/boot/${BOARD}-super.dtb"
    printf '#!/bin/sh\n[ "$1" = get-current-slot ] && echo 0\n' > "$r/bin/nvbootctrl"; chmod +x "$r/bin/nvbootctrl"
    if [[ "$layout" == flash ]]; then
        mkdir -p "$r/boot/dtb"; cp "$r/boot/${BOARD}.dtb" "$r/boot/dtb/kernel_${BOARD}.dtb"
    fi
    cat > "$r/boot/extlinux/extlinux.conf" <<'EOF'
TIMEOUT 30
DEFAULT real-time

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} root=PARTUUID=0 rw

LABEL real-time
	MENU LABEL real-time kernel
	LINUX /boot/Image.real-time
	INITRD /boot/initrd
	APPEND ${cbootargs} root=PARTUUID=0 rw isolcpus=managed_irq,domain,11
EOF
    sed -e "s#/proc/device-tree#$r/dt#g" -e "s#/boot/#$r/boot/#g" -e "s#=/boot\$#=$r/boot#g" \
        -e "s#/dev/disk/by-partlabel/#$r/part/#g" \
        -e "s#/etc/plem/#$r/etc/#g" -e "s#/run/lock/#$r/run/#g" \
        -e "s#/sys/firmware/efi/efivars/#$r/efivars/#g" "$SRC" > "$r/unlock.sh"
    grep -qE "(^|[[:space:]\"'=(])/(proc|boot|dev/disk|etc/plem|run/lock|sys/firmware)/" <(grep -v '^#' "$r/unlock.sh") \
        && fail "a fixed path in nvpps-tsc-unlock.sh was not redirected — update this test's substitutions"
    echo "$r"
}
# set_part <root> <A|B> <dtb>: that slot's kernel-dtb partition holds <dtb>, zero-padded like the real one
set_part() { { cat "$3"; head -c 4096 /dev/zero; } > "$1/part/${2}_kernel-dtb"; }
run() { local r="$1"; shift; PATH="$r/bin:$PATH" bash "$r/unlock.sh" "$@" 2>&1 | tail -1; echo "rc=${PIPESTATUS[0]}"; }
has_prop() { fdtget "$1" /bus@0/nvpps ptp_tsc_sync_dis >/dev/null 2>&1; }
label_fdt() { awk -v want="$2" '$1=="LABEL" { cur=$2 } cur==want && $1=="FDT" { print $2 }' "$1"; }

# 1. USB layout: --check writes nothing and says the setting is not applied.
r="$(setup usb)"
out="$(run "$r" --check)"
[[ "$out" == *"warn:"*"not applied"*"rc=0"* ]] || fail "usb --check: ${out}"
[[ ! -e "$r/boot/dtb" ]] || fail "usb --check created $r/boot/dtb"

# 2. USB layout: seeds /boot/dtb/kernel_<board>.dtb from the /boot DTB that the booted slot's kernel-dtb
#    partition holds (not the -super variant), sets the property, adds FDT to every label but primary. The
#    package's /boot/<board>.dtb itself stays as it was.
out="$(run "$r")"
seed="$r/boot/dtb/kernel_${BOARD}.dtb"
[[ "$out" == *"applied:"*"rc=0"* ]] || fail "usb apply: ${out}"
[[ -f "$seed" ]] && has_prop "$seed" || fail "usb apply: ${seed} missing or without ptp_tsc_sync_dis"
has_prop "$r/boot/${BOARD}.dtb" && fail "usb apply: changed the package DTB /boot/${BOARD}.dtb"
[[ "$(label_fdt "$r/boot/extlinux/extlinux.conf" real-time)" == "$seed" ]] || fail "usb apply: real-time label FDT is not ${seed}"
[[ -z "$(label_fdt "$r/boot/extlinux/extlinux.conf" primary)" ]] || fail "usb apply: primary got an FDT line"

# 3. Second run: nothing left to write; the running tree still has the lock, so a reboot is pending.
out="$(run "$r")"
[[ "$out" == *"pending:"*"rc=0"* ]] || fail "usb rerun: ${out}"
rm -rf "$r"

# 4. USB layout where no /boot DTB is the one in the booted slot's partition: stops before writing.
r="$(setup usb)"; rm -f "$r/boot/${BOARD}.dtb"
cp "$r/boot/extlinux/extlinux.conf" "$r/ext.before"
out="$(run "$r")"
[[ "$out" == *"failed:"*"A_kernel-dtb"*"rc=2"* ]] || fail "usb without board DTB: ${out}"
[[ ! -e "$r/boot/dtb" ]] || fail "usb without board DTB: created $r/boot/dtb"
cmp -s "$r/ext.before" "$r/boot/extlinux/extlinux.conf" || fail "usb without board DTB: extlinux changed"
rm -rf "$r"

# 5. USB layout whose /boot/<board>.dtb is for another board: stops, leaves no seeded file behind.
r="$(setup usb)"
make_dtb "$r/boot/${BOARD}.dtb" "nvidia,p3767-0000" "nvidia,tegra234"; set_part "$r" A "$r/boot/${BOARD}.dtb"
cp "$r/boot/extlinux/extlinux.conf" "$r/ext.before"
out="$(run "$r")"
[[ "$out" == *"failed:"*"not the DTB for this board"*"rc=2"* ]] || fail "usb wrong board: ${out}"
[[ -z "$(compgen -G "$r/boot/dtb/kernel_*.dtb")" ]] || fail "usb wrong board: left a seeded kernel_*.dtb"
cmp -s "$r/ext.before" "$r/boot/extlinux/extlinux.conf" || fail "usb wrong board: extlinux changed"
rm -rf "$r"

# 4b. Slot B booted: the DTB in B_kernel-dtb is the one seeded.
r="$(setup usb)"
printf '#!/bin/sh\n[ "$1" = get-current-slot ] && echo 1\n' > "$r/bin/nvbootctrl"
out="$(run "$r")"
[[ "$out" == *"applied:"*"rc=0"* ]] || fail "usb slot B: ${out}"
[[ -f "$r/boot/dtb/kernel_${BOARD}-super.dtb" ]] || fail "usb slot B: did not seed from ${BOARD}-super.dtb"
rm -rf "$r"

# 6. Flash layout (regression): the existing kernel_*.dtb is edited in place, as before.
r="$(setup flash)"
out="$(run "$r")"
[[ "$out" == *"applied:"*"rc=0"* ]] || fail "flash apply: ${out}"
has_prop "$r/boot/dtb/kernel_${BOARD}.dtb" || fail "flash apply: property missing"
[[ "$(label_fdt "$r/boot/extlinux/extlinux.conf" real-time)" == "$r/boot/dtb/kernel_${BOARD}.dtb" ]] || fail "flash apply: FDT line"
rm -rf "$r"

echo "PASS: test_nvpps_tsc_unlock"
