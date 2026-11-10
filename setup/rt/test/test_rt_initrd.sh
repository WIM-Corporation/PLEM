#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# setup/rt/rt-initrd.sh (RT kernel modules in /boot/initrd). Hermetic: every path sits under
# RT_INITRD_ROOT (a temp dir) and the nv-update-initrd it runs is a fixture that behaves like the real
# one, so nothing on this machine is read or changed.
#   ① no RT kernel installed → n/a, nothing written.
#   ② initrd without the RT modules: --check warns, apply rebuilds it and the modules are there.
#   ③ the fixture carries NVIDIA's bug (the list is deleted after the first image); the patched copy must
#      beat it, and the file NVIDIA ships must stay byte for byte the same.
#   ④ already-correct initrd → ok without running the tool; a tool that cannot fix it → failed.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOL="${HERE}/rt-initrd.sh"
fail() { echo "FAIL: $*" >&2; exit 1; }
[[ -x "$TOOL" ]] || fail "setup/rt/rt-initrd.sh missing or not executable"

RT=5.15.148-rt-tegra
STOCK=5.15.148-tegra

# A tree with /boot/initrd (a cpio.gz holding the listed module versions), /lib/modules/<versions> and a
# fixture nv-update-initrd that repacks the initrd the way the real one does.
make_root() {  # make_root <dir> <initrd versions, space separated> <installed versions>
    local root="$1" in_initrd="$2" installed="$3" v
    mkdir -p "${root}/boot" "${root}/lib/modules" "${root}/usr/sbin" "${root}/stage"
    for v in $installed; do mkdir -p "${root}/lib/modules/${v}/kernel/drivers/nvme/host"
        echo "$v" > "${root}/lib/modules/${v}/kernel/drivers/nvme/host/nvme.ko"; done
    ( cd "${root}/stage"
      for v in $in_initrd; do mkdir -p "lib/modules/${v}/kernel/drivers/nvme/host"
          echo "$v" > "lib/modules/${v}/kernel/drivers/nvme/host/nvme.ko"; done
      find . | cpio -H newc -o --quiet | gzip -9 -n > "${root}/boot/initrd" )
    rm -rf "${root}/stage"
}

# Fixture: loops over both kernels like the real tool. With BUGGY=1 it drops the list after the first one,
# which is what L4T 36.4.3 does, so only the stock modules land in the initrd.
write_fixture() {  # write_fixture <root> <buggy: true|false>
    local root="$1" buggy="$2"
    cat > "${root}/usr/sbin/nv-update-initrd" <<EOF
#!/bin/bash
set -e
DESTDIR=\$(mktemp -d)
temp_list=\$(mktemp)
printf 'lib/modules/<KERNEL_VERSION>/kernel/drivers/nvme/host/nvme.ko\n' > "\$temp_list"
for image_path in ${STOCK} ${RT}; do
    _kernel_version="\$image_path"
    copy_files_initrd "\${DESTDIR}" "\${temp_list}" "\${_kernel_version}"
EOF
    $buggy && printf '\t\trm -f "${temp_list}"\n' >> "${root}/usr/sbin/nv-update-initrd"
    cat >> "${root}/usr/sbin/nv-update-initrd" <<EOF
done
( cd "\$DESTDIR" && find . | cpio -H newc -o --quiet | gzip -9 -n > "${root}/boot/initrd" )
rm -rf "\$DESTDIR" "\$temp_list"
EOF
    # copy_files_initrd is defined above its use in the real tool too; prepend it.
    cat > "${root}/usr/sbin/.fn" <<EOF
copy_files_initrd() {
    local _d="\$1" _list="\$2" _ver="\$3"
    [ -s "\$_list" ] || return 0
    sed "s|<KERNEL_VERSION>|\${_ver}|g" "\$_list" | while read -r f; do
        mkdir -p "\$_d/\$(dirname "\$f")"; echo "\$_ver" > "\$_d/\$f"
    done
}
EOF
    sed -i.bak "2r ${root}/usr/sbin/.fn" "${root}/usr/sbin/nv-update-initrd"
    rm -f "${root}/usr/sbin/.fn" "${root}/usr/sbin/nv-update-initrd.bak"
    chmod 755 "${root}/usr/sbin/nv-update-initrd"
}

initrd_versions() {  # initrd_versions <root>
    gzip -dc "$1/boot/initrd" | cpio -t --quiet 2>/dev/null \
        | sed -n 's|.*lib/modules/\([^/]*\)/.*|\1|p' | sort -u
}

tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT

# ① no RT kernel installed
R1="${tmp}/r1"; make_root "$R1" "$STOCK" "$STOCK"; write_fixture "$R1" false
out=$(RT_INITRD_ROOT="$R1" "$TOOL") || fail "① exit ${?} for a box without the RT kernel"
[[ "$out" == n/a:* ]] || fail "① expected n/a, got: $out"
out=$(RT_INITRD_ROOT="$R1" "$TOOL" --check) || fail "① --check exit ${?}"
[[ "$out" == n/a:* ]] || fail "① --check expected n/a, got: $out"

# ② + ③ RT installed, initrd missing its modules, NVIDIA's tool buggy
R2="${tmp}/r2"; make_root "$R2" "$STOCK" "$STOCK $RT"; write_fixture "$R2" true
before="$(md5 -q "${R2}/usr/sbin/nv-update-initrd" 2>/dev/null || md5sum "${R2}/usr/sbin/nv-update-initrd" | cut -d' ' -f1)"
set +e; out=$(RT_INITRD_ROOT="$R2" "$TOOL" --check 2>/dev/null); rc=$?; set -e
[[ $rc -eq 1 && "$out" == warn:* ]] || fail "② --check expected warn/1, got ${rc}: $out"
[[ "$(initrd_versions "$R2")" == "$STOCK" ]] || fail "② --check must not write"
out=$(RT_INITRD_ROOT="$R2" "$TOOL" 2>/dev/null) || fail "② apply exit ${?}: $out"
[[ "$out" == applied:* ]] || fail "② expected applied, got: $out"
[[ "$(initrd_versions "$R2" | tr '\n' ' ')" == "${RT} ${STOCK} " ]] \
    || fail "③ both kernels must be in the initrd, got: $(initrd_versions "$R2" | tr '\n' ' ')"
after="$(md5 -q "${R2}/usr/sbin/nv-update-initrd" 2>/dev/null || md5sum "${R2}/usr/sbin/nv-update-initrd" | cut -d' ' -f1)"
[[ "$before" == "$after" ]] || fail "③ /usr/sbin/nv-update-initrd was modified"
out=$(RT_INITRD_ROOT="$R2" "$TOOL" --check) || fail "③ --check after apply exit ${?}: $out"
[[ "$out" == ok:* ]] || fail "③ --check after apply expected ok, got: $out"

# ④ already correct → ok without running the tool (the fixture is replaced by one that would fail)
R4="${tmp}/r4"; make_root "$R4" "$STOCK $RT" "$STOCK $RT"
printf '#!/bin/bash\nexit 1\n' > "${R4}/usr/sbin/nv-update-initrd"; chmod 755 "${R4}/usr/sbin/nv-update-initrd"
out=$(RT_INITRD_ROOT="$R4" "$TOOL") || fail "④ exit ${?} on an already-correct initrd"
[[ "$out" == ok:* ]] || fail "④ expected ok, got: $out"

# a tool that runs but does not fix the initrd → failed
R5="${tmp}/r5"; make_root "$R5" "$STOCK" "$STOCK $RT"
printf '#!/bin/bash\nexit 0\n' > "${R5}/usr/sbin/nv-update-initrd"; chmod 755 "${R5}/usr/sbin/nv-update-initrd"
set +e; out=$(RT_INITRD_ROOT="$R5" "$TOOL" 2>/dev/null); rc=$?; set -e
[[ $rc -eq 1 && "$out" == failed:* ]] || fail "⑤ expected failed/1, got ${rc}: $out"

# ⑦ the real BSP initrd lists its modules under usr/lib/modules (/lib is a symlink there) — recognise it
R6="${tmp}/r6"; mkdir -p "${R6}/boot" "${R6}/lib/modules/${RT}" "${R6}/usr/sbin" "${R6}/stage/usr/lib/modules/${RT}"
( cd "${R6}/stage" && find . | cpio -H newc -o --quiet | gzip -9 -n > "${R6}/boot/initrd" )
out=$(RT_INITRD_ROOT="$R6" "$TOOL" --check) || fail "⑦ --check exit ${?} on a usr/lib initrd: $out"
[[ "$out" == ok:* ]] || fail "⑦ expected ok for usr/lib/modules entries, got: $out"

# ⑧ the listing is read to the end. A reader that stops at the first match (grep -q, -m1) kills the
# writer with SIGPIPE, and under `set -o pipefail` the whole pipeline then reports failure — on the real
# 16 MB initrd that turned a correct initrd into "no modules". The stub writes the matching line first and
# keeps writing, so the bug shows up here without depending on how fast a big archive streams.
R7="${tmp}/r7"; make_root "$R7" "$STOCK $RT" "$STOCK $RT"
STUB="${tmp}/stub"; mkdir -p "$STUB"
cat >"${STUB}/cpio" <<EOF
#!/bin/bash
cat >/dev/null                                   # drain the decompressed archive
echo "usr/lib/modules/${RT}/kernel/drivers/nvme/host/nvme.ko"
sleep 0.3                                        # a reader that already left closes the pipe here
for i in \$(seq 1 2000); do echo "usr/lib/firmware/blob\${i}"; done
EOF
chmod 755 "${STUB}/cpio"
out=$(PATH="${STUB}:$PATH" RT_INITRD_ROOT="$R7" "$TOOL" --check) \
    || fail "⑧ --check exit ${?} although the modules are listed: $out"
[[ "$out" == ok:* ]] || fail "⑧ expected ok, got: $out"

# usage error
set +e; out=$("$TOOL" --nope 2>/dev/null); rc=$?; set -e
[[ $rc -eq 2 && "$out" == failed:* ]] || fail "⑥ expected failed/2 for a bad argument, got ${rc}: $out"

echo "PASS: test_rt_initrd.sh"
