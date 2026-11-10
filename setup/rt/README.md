# Jetson RT kernel and EtherCAT setup and verification wizard

> **Target devices:** NVIDIA Jetson AGX Orin Developer Kit (where it has run). Written to handle Orin NX and Orin Nano too; not run on them.
> **Software:** JetPack 7.2.1 (L4T r39.2.1, RT kernel `6.8.x-rt-tegra`). Menus 1 to 3 ran on an AGX Orin devkit with both the built-in RJ45 port (generic driver) and an Intel I210 (native driver; that devkit was set up with an earlier version of this wizard, see [`docs/hardware-evidence.md`](../../docs/hardware-evidence.md)). Menu 4 has not run a measurement, and menu 3's reinstall path (`Reinstall? (y/N)`) has not been run there. Menu 5 has not run on a device; its downloads and builds were run by hand on a JetPack 6 RT kernel (`5.15.148-rt-tegra`), without installing or loading the modules. This is the platform of the drive software in this repository ([`docs/install.md`](../../docs/install.md)).
> The wizard reads the L4T release line from the device's apt sources and picks the matching rt-kernel repository.

`setup.sh` is an interactive script that installs the official NVIDIA RT kernel (`nvidia-l4t-rt-kernel`,
`CONFIG_PREEMPT_RT`), isolates a CPU, sets up the IgH EtherCAT master, runs RT load tests and builds the USB adapter drivers the kernel lacks.
It installs what it needs with apt (menu 3 also clones IgH with git, and menu 5 downloads kernel driver sources from git.kernel.org): this directory alone is enough to install, verify, and run the load tests.

## Contents

| File | Description |
| --- | --- |
| `setup.sh` | Menu-driven wizard: install, verify, EtherCAT, load tests, USB adapter drivers |
| `nvpps-tsc-unlock.sh` | nvpps TSC unlock (DTB property + extlinux FDT). Menu 1 installs and runs it. It can also be run on its own (`sudo plem-nvpps-tsc-unlock`) |
| `crash-logging.sh` | Persistent logs + reboot after a kernel panic. Menu 1 runs it, menu 2 runs it with `--check` (no writes). It can also be run on its own (`sudo bash setup/rt/crash-logging.sh`) |
| `rt-initrd.sh` | Puts the RT kernel's modules into `/boot/initrd`. Menu 1 runs it, menu 2 runs it with `--check` (no writes). It can also be run on its own (`sudo bash setup/rt/rt-initrd.sh`) |
| `99-plem-realtime.conf` | Real-time scheduling limits for the `realtime` group. Menu 1 installs it as `/etc/security/limits.d/99-plem-realtime.conf`. If the file goes missing, run menu 1 again |
| `test/` | Hermetic tests of `setup.sh`, `crash-logging.sh`, `rt-initrd.sh` and the menu 3 NIC rebind helper (`bash setup/rt/test/test_setup.sh` and the others); they touch no real system file |
| `LICENSE` | License for the files in this directory |

## Quick start

```bash
sudo bash setup/rt/setup.sh              # from the repository root
sudo bash setup/rt/setup.sh --baseline   # a Jetson used only as a controller: menu 1 defaults to disabling unused services
```

`↑↓` move, `Enter` select, `q` quit. The header shows the device, L4T version, kernel, memory, and RT package state.
Command output goes to `/tmp/rt-wizard-<random>.log`; a failed step prints its path, and on exit the wizard keeps the file and prints its path if anything was written to it.

## Menu

| Menu | When | What it does |
| --- | --- | --- |
| **1. Install RT kernel** | Before reboot | Installs the RT kernel and applies RT tuning (CPU isolation, power, memory, and more) |
| **2. Verify setup** | After reboot | Checks automatically that the menu 1 settings took effect |
| **3. Install EtherCAT master** | After booting the RT kernel | Builds, configures, and registers the IgH EtherCAT master service |
| **4. RT jitter load test** | When an isolated CPU exists | Measures scheduling jitter on the isolated CPU under load with `cyclictest`, 1→5 kHz |
| **5. Install USB adapter drivers** | On the kernel to build for (RT or stock) | Builds and installs the USB-CAN and USB-serial adapter drivers the kernel ships disabled |

Typical order: **1 → reboot → 2 → 3 → 4**, and **5** on a device that uses a USB-CAN or USB-serial adapter.

## Device reporting

Before the menu, the wizard asks once whether to share device data with WIM, and offers an optional email address.
[`docs/telemetry.md`](../../docs/telemetry.md) lists what is sent and how it is kept. Either answer is saved in
`/etc/plem/fleet.json`. A Yes also adds `/usr/local/sbin/plem-fleet`, the `plem-fleet` timer and service, and
`/var/lib/plem/fleet`. Giving an email address also installs `/usr/local/sbin/plem-fleet`, so the address can be
removed later. `sudo plem-fleet off` turns it off and removes the timer and the queued reports.

The question needs the rest of this repository. Run from a copy of this directory alone, or with `PLEM_TELEMETRY=off`,
the wizard does not ask and nothing is sent.

---

## 1. Install RT kernel (before reboot)

The wizard applies the following.

| Item | Details | Purpose |
| --- | --- | --- |
| RT kernel apt install | `nvidia-l4t-rt-kernel`, `-headers`, `-oot-modules`, `nvidia-l4t-display-rt-kernel` (+ on the JetPack 7 line, the `nvidia-l4t-rt-kernel-nvgpu` and `-openrm` GPU kernel modules are added automatically, only when the repository has them) | `PREEMPT_RT` kernel |
| Version hold and pin | `apt-mark hold` + exact version pin on every installed `nvidia-l4t-*` | Prevents mixing kernel, bootloader, and L4T userspace releases (checked with a simulated full-upgrade) |
| Power mode | MAXN: default mode in `/etc/nvpmodel.conf` + current-mode state file (the ID is read from the conf) | All cores, maximum clocks. Boots in MAXN even if the state file is reset |
| Fixed clocks | `jetson_clocks` systemd service | Keeps maximum clocks after reboot. Also disables CPU idle states (C-states) on all cores |
| CPU isolation | Boot parameters `isolcpus=managed_irq,domain,<N> irqaffinity=…` | Removes the RT CPU from scheduler load balancing and IRQs |
| Disable swap/zram | `swapoff`, disable `nvzramconfig` and `nvfb-swapfile` when present, comment out swap lines in `/etc/fstab` | Prevents jitter from the CPU cost of zram compression |
| Disable irqbalance | Only if installed | Prevents IRQ rebalancing from breaking isolation |
| Install rt-tests | `apt install rt-tests` (`cyclictest`) | Used by the latency tests in menus 2 and 4 |
| Wi-Fi tuning | NetworkManager `wifi.powersave = 2`, a dispatcher running `iw ... power_save off`, RTL8822CE modprobe options | Prevents Wi-Fi power-save wakeups and latency spikes |
| Disable unused services (optional) | Menu 1 asks first (default no; yes with `--baseline`, on a device where it was applied before, or when all are already off). Menu 2 warns if a unit comes back on after it was applied. Turns off bluetooth, ModemManager, avahi, apport, kerneloops, power-profiles-daemon, packagekit, rtkit, background timers such as apt-daily, and desktop audio (pipewire, pulseaudio). gdm is kept. Masking packagekit makes `apt update` and every package install or removal end with a harmless `GDBus.Error … packagekit.service is masked` line: Ubuntu's apt hook still tries to notify it | Prevents features a control computer does not use from consuming CPU and disk or getting RT priority |
| Crash logging | Persistent journald storage (`Storage=persistent`, limit = 2 % of the root filesystem, at most 2 GiB) + `kernel.panic=10` (`crash-logging.sh`) | Logs survive a reboot, and after a kernel panic the system reboots after 10 seconds instead of waiting 120 seconds for the watchdog |
| RT initrd modules | Rebuilds `/boot/initrd` so it carries the modules of every installed kernel, including `<version>-rt-tegra` (`rt-initrd.sh`) | Both extlinux labels load the same initrd, and it insmods storage drivers by absolute path under `/lib/modules/<version>`. L4T's `nv-update-initrd` loops over `/boot/Image` and `/boot/Image.real-time`. On L4T 36.4.3 (JetPack 6) it deletes the shared file list after the first image, so the RT modules never make it in. With the rootfs on NVMe the RT kernel then cannot load `nvme.ko` and the boot stops at `ERROR: mounting PARTUUID=... fail`; after three such boots UEFI marks the slot unbootable (`RootfsStatusSlotA=0xFF`) and only the recovery initrd comes up. On L4T 39.2.1 (JetPack 7.2.1) both kernels' modules are already in the initrd, and the step changes nothing |
| nvpps TSC unlock | Adds the nvpps `ptp_tsc_sync_dis` property to `/boot/dtb/kernel_*.dtb`, and adds an `FDT` line for that DTB to each label without FDT except primary (`nvpps-tsc-unlock.sh`, installed as `/usr/local/sbin/plem-nvpps-tsc-unlock`). On a USB (Jetson ISO) install, which has no `/boot/dtb/`, it first creates `/boot/dtb/kernel_<board>.dtb` from the `/boot` DTB that the firmware boots | When nvpps locks the TSC to the PHC of the built-in RJ45 port (MGBE0: `end0` on JetPack 7.2.1) and that port has no PTP grandmaster, the clock periodically slips by ±140 ppm and the EtherCAT DC error grows to hundreds of µs |
| RT permission | Creates the `realtime` system group, installs `99-plem-realtime.conf` (members get `SCHED_FIFO` priority up to 99 and unlimited locked memory), and adds the user who ran `sudo` to the group. Takes effect at that user's next login | Lets control loops run real-time as a normal user instead of root |

- **Isolated CPU selection:** The default is the CPU isolated at boot (the last one if several). When none is isolated
  (a first run, or before the reboot after menu 1), it is the last core (AGX Orin 12 cores → CPU11, Orin NX 8 cores →
  CPU7, Orin Nano 6 cores → CPU5). Menu 2 checks the CPU isolated at boot against the one menu 1 wrote to the boot label. You can pick another core with the cursor in menu 1.
- **Reboot required:** Boot parameters and the kernel switch take effect after reboot. When the install finishes,
  `DEFAULT` in `/boot/extlinux/extlinux.conf` is set to `real-time`, unless the current DEFAULT already boots the RT image.

RT throttling (`kernel.sched_rt_runtime_us`) is left at the kernel default of 950000 (950 ms out of every second).
An RT loop that sleeps every cycle never reaches this limit, and if an RT thread runs away, the kernel threads
on the isolated CPU still run during the remaining 50 ms, so the whole device does not freeze. The wizard does not set this value: menu 2
only checks it. If `/etc/sysctl.conf` sets it, re-running menu 1 removes that line and restores the default.

**nvpps TSC unlock** applies only to boards that have an nvpps node (such as the AGX Orin devkit carrier board). Because the property goes into
the DTB file rather than a boot label, a label that a third-party driver package (for example a camera card driver) creates
on install and upgrade uses the same DTB, so the setting carries over.
If the setting is lost (someone releases the L4T version hold and updates the DTB package, or removes such a driver package with apt or dpkg),
the lock turns back on and the system still boots, and menu 2 warns about it. In that case, run `sudo plem-nvpps-tsc-unlock` and reboot.
`plem-nvpps-tsc-unlock --check` writes nothing and reports the next-boot state from the FDT and OVERLAYS of the DEFAULT label.
Each edit keeps the previous files: the DTB as `/boot/dtb/.plem-bak-<name>.dtb` and extlinux as
`/boot/extlinux/extlinux.conf.plem-bak`.

A USB (Jetson ISO) install has neither `/boot/dtb/` nor an `FDT` line, so the firmware boots the DTB in the
`kernel-dtb` partition of the boot slot (`nvbootctrl get-current-slot`). There the script first creates
`/boot/dtb/kernel_<board>.dtb` from the `/boot/<board>.dtb` whose bytes that partition starts with, and stops without
writing anything if no `/boot/*.dtb` matches or the match's `compatible` is not the running board's. After that, the labels boot this file instead of the firmware's copy. Restoring
`/boot/extlinux/extlinux.conf.plem-bak` removes the `FDT` lines and returns to the firmware's DTB.

Devices that synchronize time over PTP on the built-in RJ45 port must keep the lock. Run `sudo mkdir -p /etc/plem && sudo touch
/etc/plem/nvpps-lock-keep`, then run the script (or menu 1) again to remove the property. Devices with Secure Boot
enabled are skipped, because the DTB would need to be signed.

`kthread_cpus` is not a mainline kernel parameter and the kernel ignores it, so the wizard does not add it.

## 2. Verify setup (after reboot)

After rebooting, run menu 2 to check that the items above took effect (`PREEMPT_RT` kernel,
isolated CPU, boot parameters the kernel ignored, RT throttling default, swap, CPU idle (deep idle states), power mode and clocks, and more; once menu 3 has run, also that `/dev/EtherCAT*` is mode 0660 group `realtime` and that the user who ran `sudo` is a member; and whether each menu 5 driver is available for the running kernel, a warning naming menu 5 when one is missing).
Finally, it runs a 1 kHz cyclictest on the isolated CPU for 30 seconds (no load; load measurement is menu 4). For commands to check by hand,
see the [verification cheat sheet](#verification-cheat-sheet).

## 3. Install EtherCAT master

Menu 3 builds the IgH EtherCAT master (EtherLab, pinned to git tag `1.6.13`) from source and configures
it, including the service.

1. **NIC auto-detection:** Selects the NIC dedicated to EtherCAT and identifies its kernel driver
   (can be set with the `ETHERCAT_IFACE` environment variable: `sudo ETHERCAT_IFACE=<iface>[,<iface>...] bash
   setup/rt/setup.sh`. Wi-Fi is excluded from the candidates).
2. **Backend selection:** If the selected NIC driver is on the IgH native backend support list
   (`igb`/`igc`/`e1000e`/`r8169`/`stmmac-pci`, and others), `ec_<driver>` is built as well.
   Otherwise it falls back to `generic`. If the driver is on the support list but IgH does not ship a copy
   for the running kernel, the wizard stops at configure (it does not fall back to `generic`, see below).
3. Creates `/etc/ethercat.conf`, the udev rule that makes `/dev/EtherCAT*` mode 0660 group `realtime` (after creating the group and adding the user who ran `sudo`; a different existing rule is kept as `99-ethercat.rules.plem-bak`), and `ethercat.service` (including the native rebind).
4. Starts the service and verifies it: the unit is active and `ec_master` is loaded (either missing fails the
   step). It then reports the backend and, once the link is up, the slave count (no slaves is a note, not a failure).
5. The EtherCAT port is removed from NetworkManager control by `/etc/NetworkManager/conf.d/plem-ethercat.conf`.
   To use that port as a normal network port again, delete this file and run `systemctl reload NetworkManager`.

On a device where the master is already installed, menu 3 offers a reinstall; what it stops and how to cancel is in
[`docs/install.md`](../../docs/install.md#2-get-the-source-and-run-the-setup-wizard).

### Native backend vs generic

| Item | `generic` | `ec_<driver>` (for example, `igb`) |
| --- | --- | --- |
| Data path | IgH → ec_generic → **Linux network stack** → driver → NIC | IgH → **ec_\<driver\>** → NIC (bypasses the stack) |
| Determinism | Depends on NIC IRQ latency | RT thread polling, no interrupts on the data path |
| Use | Built-in ports and NICs without an IgH driver (for example, the Jetson RJ45). Works; 1 kHz without lost frames is not guaranteed | Recommended for 1 kHz on supported NICs (for example, PCIe Intel I210) |

- On L4T, built-in drivers such as `igb` cannot be removed with `rmmod`, so native mode hands the NIC to
  `ec_<driver>` with a PCI sysfs unbind/bind. This rebind runs from `ExecStartPost` (idempotent) in
  `ethercat.service`, so it is applied again from the current `/etc/ethercat.conf` not only at boot but on every
  `systemctl restart ethercat`. If the NIC or MAC changes, run menu 3 (Install EtherCAT master) again. If you only edit the conf and restart, EtherCAT works, but the NetworkManager exclusion (`plem-ethercat.conf`) stays on the old MAC.
- In native mode the NIC does not appear as a Linux net device (not listed in `ip link`). This is expected:
  the master polls the NIC, so its MSI-X interrupt counts stay at 0.
- For a multi-port NIC, add `MASTER<X>_DEVICE`/`UPDOWN_INTERFACES` entries to `/etc/ethercat.conf`
  (numbered from 0 without gaps, as ethercatctl reads them) and the ExecStartPost rebind helper iterates over all of them
  (no code changes needed).
- Native backends ship per kernel version. IgH keeps a separate patched copy of each driver
  for each kernel version (both a subdirectory form such as `devices/igb/igb_main-6.1-ethercat.c`
  and a flat form such as `devices/r8169-4.4-ethercat.c` exist). On a kernel with no bundled copy,
  passing `--enable-<driver>` makes configure fail with `kernel X.Y not available for <driver> driver!`,
  and the wizard prints that reason and stops. It does not quietly switch such a NIC to `generic`: a native-capable
  card is normally there for its timing, and `generic` would give that up without telling you. Choosing a port that
  only `generic` can drive, such as the built-in RJ45, is fine and the wizard proceeds without asking.
- As of IgH 1.6.13, the kernels bundled for `igb` are `3.18 4.4 4.19 5.10 5.14 5.15 6.1 6.4 6.8 6.12`.
  JetPack 7.2.1 (L4T 39.2.1, kernel 6.8) builds native mode without changes to
  the IgH source (the 6.8 copy has been bundled since IgH 1.6.10).

## 4. RT jitter load test (cyclictest, 1→5 kHz)

Runs `cyclictest` on the isolated CPU **together with concurrent system load** and measures scheduling jitter at 1→5 kHz
(available only when an isolated CPU exists, and uses that CPU automatically). This is a smoke test of
RT scheduling alone, run before the application: it refuses while a user RT thread (such as a running
`plem drive run`) is on the isolated CPU, because cyclictest at priority 99 would preempt that loop.

## 5. Install USB adapter drivers (RT or stock kernel)

The JetPack 6 and JetPack 7 kernels (stock and RT) ship these USB adapter drivers disabled (`is not set` in `/proc/config.gz`); menu 5 reads the running kernel's configuration and skips any driver it already has:

| Module | Kernel option | Adapters |
| --- | --- | --- |
| `gs_usb` | `CONFIG_CAN_GS_USB` | candleLight/gs_usb-compatible USB-CAN adapters |
| `ch341` | `CONFIG_USB_SERIAL_CH341` | CH340/CH341 USB-serial adapters |
| `pl2303` | `CONFIG_USB_SERIAL_PL2303` | PL2303 USB-serial adapters |

Menu 5 builds them on the device for the running kernel. Nothing is fixed to a JetPack or kernel version: it takes the
upstream version from the Makefile of `/lib/modules/$(uname -r)/build` (for example `5.15.148`, tag `v5.15.148`, on JetPack 6 and `6.8.12` on JetPack 7), downloads
each driver's source file of that tag, and the files it includes with `#include "…"` from its own directory, from the stable
kernel tree on git.kernel.org, and builds it against the installed headers. No prebuilt module is shipped.

For each driver, it:

1. Skips it when the running kernel already has it (`=y` or `=m` in `/proc/config.gz`), or when a module for this kernel
   is already installed (`modinfo -k $(uname -r)` finds it with a matching vermagic). A re-run downloads nothing then.
2. Otherwise checks, before downloading, that the kernel headers match the running kernel (the same check as menu 3) and
   that `curl`, `make` and `gcc` are installed.
3. Refuses a downloaded file that is not kernel source (its first line must be an SPDX tag: the server can answer with
   an HTML page), builds the module, and checks that its vermagic is the running kernel's release.
4. Installs it as `/lib/modules/$(uname -r)/updates/plem/<module>.ko`, then runs `depmod` once and checks that
   `modinfo` now finds each new module.

Menu 5 builds for whichever kernel is running, RT or stock, and is not locked before the RT reboot. It writes no
`/etc/modules-load.d` entry: each module carries its adapters' USB IDs (`modinfo -F alias`), which the kernel matches to
pick a driver when a device is plugged in; loading on a plugged-in adapter has not been tried yet. A build may print a
"compiler differs" warning (the system gcc is not the one the kernel was built with); the test builds completed with it,
and the built modules were not loaded. Any failure is shown at the step
and again in the summary, with what failed and how to fix it. The modules belong to one kernel release: after a kernel
update, run menu 5 again (menu 2 reports a missing driver).

---

## Verification cheat sheet

Commands to check by hand after reboot (expected output depends on hardware and settings).

```bash
# Confirm the RT kernel booted
uname -a                                   # JetPack 7.2.1: 6.8.x-rt-tegra ... PREEMPT_RT
zcat /proc/config.gz | grep PREEMPT_RT     # CONFIG_PREEMPT_RT=y

# CPU isolation / RT throttling
cat /sys/devices/system/cpu/isolated       # e.g. 11
sysctl kernel.sched_rt_runtime_us          # 950000 (default kept)

# Swap / CPU idle / irqbalance
free -h | grep -i swap                      # Swap: 0B
cat /sys/devices/system/cpu/cpu<N>/cpuidle/state1/disable   # 1 (disabled by jetson_clocks)
systemctl is-active irqbalance 2>/dev/null  # inactive or not found

# Power mode / clocks
sudo nvpmodel -q                            # NV Power Mode: MAXN
systemctl is-active jetson-clocks.service   # active

# EtherCAT (after menu 3)
sudo ethercat master
sudo ethercat slaves
systemctl is-active ethercat

# USB adapter drivers (after menu 5)
modinfo -F vermagic gs_usb ch341 pl2303     # each line starts with $(uname -r)
```

## Removing the RT kernel

The wizard puts an apt hold and an exact version pin on every installed `nvidia-l4t-*` package. To remove the RT
kernel or to upgrade JetPack, release the hold and the pin first. This removes only the kernel packages: boot
parameters, the `DEFAULT` label, jetson_clocks, the nvpps DTB property and the other menu 1 settings stay until you
revert them.

```bash
sudo rm /etc/apt/preferences.d/plem-rt-freeze.pref
sudo apt-mark unhold $(apt-mark showhold | grep '^nvidia-l4t-')
sudo apt remove nvidia-l4t-rt-kernel nvidia-l4t-rt-kernel-headers \
  nvidia-l4t-rt-kernel-oot-modules nvidia-l4t-display-rt-kernel
sudo reboot
```

The wizard backs up these files before editing them (it never deletes the backups); the other files it writes have no
backup:

| Backup | What it holds |
| --- | --- |
| `/boot/extlinux/extlinux.conf.bak` | extlinux.conf before the wizard's first boot-parameter edit (made only when no file of that name exists) |
| `/boot/extlinux/extlinux.conf.plem-bak` | extlinux.conf before the latest nvpps TSC unlock edit |
| `/boot/dtb/.plem-bak-<name>.dtb` | the DTB before the latest nvpps TSC unlock edit |
| `/etc/fstab.plem-rt.bak` | `/etc/fstab` before its swap entries were commented out (made once) |
| `/etc/udev/rules.d/99-ethercat.rules.plem-bak` | a different `99-ethercat.rules` the wizard replaced |

To put a file back, copy its backup over it: run only the lines you need (the two extlinux lines are alternatives),
then reboot. A backup is the file as it was when the backup was made, so any later change to that file is undone too;
read it before you copy it.

```bash
sudo cp /boot/extlinux/extlinux.conf.bak /boot/extlinux/extlinux.conf             # before the first boot-parameter edit
sudo cp /boot/extlinux/extlinux.conf.plem-bak /boot/extlinux/extlinux.conf        # or: before the last nvpps edit
ls /boot/dtb/.plem-bak-*                                                          # find <name>
sudo cp -p /boot/dtb/.plem-bak-<name>.dtb /boot/dtb/<name>.dtb                     # the DTB before the last nvpps edit
sudo cp /etc/fstab.plem-rt.bak /etc/fstab                                          # swap entries
sudo mv /etc/udev/rules.d/99-ethercat.rules.plem-bak /etc/udev/rules.d/99-ethercat.rules
sudo reboot
```

To turn the nvpps TSC lock back on, use the keep file (`/etc/plem/nvpps-lock-keep`, above) rather than the DTB backup:
the next run of menu 1 edits a restored DTB again.

Because the wizard changed the default mode in `/etc/nvpmodel.conf` to MAXN (a conffile of `nvidia-l4t-nvpmodel`),
dpkg asks about this file during a JetPack upgrade. Take the new release's file, then run the wizard again.

## References

- [NVIDIA Jetson Real-Time Kernel (r39.2, JetPack 7.2)](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/Kernel/RealTimeKernel.html)
- [IgH EtherCAT master (EtherLab)](https://gitlab.com/etherlab.org/ethercat)
- [Ubuntu Real-Time Kernel Docs](https://documentation.ubuntu.com/real-time/latest/)
- [OSADL — Latency plots / cyclictest methodology](https://www.osadl.org/Latency-plots.latency-plots.0.html)
- [Red Hat — Optimizing RHEL for Real Time](https://docs.redhat.com/en/documentation/red_hat_enterprise_linux_for_real_time/)
