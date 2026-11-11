# Installing on Jetson AGX Orin

This page prepares a Jetson AGX Orin Developer Kit to run a drive bus (`plem drive run`) and installs the drive
software. Sections 0 to 4 set up the device: a real-time kernel, one isolated CPU for the bus loop, the IgH EtherCAT
master (EtherLab's open-source EtherCAT master), and the network port the drives are wired to. Then install ROS 2
(section 5) and build the drive software from this source tree (section 6).

The setup wizard in [`setup/rt/`](../setup/rt/) does the device setup. [`setup/rt/README.md`](../setup/rt/README.md)
describes what it changes.

## Supported platform

| Platform | Status |
|---|---|
| Jetson AGX Orin Developer Kit, JetPack 7.2.1 (L4T R39.2.1, Ubuntu 24.04, arm64), ROS 2 Jazzy, IgH EtherCAT master 1.6.13 | Supported. Both the built-in RJ45 port (Case 1) and Intel I210 with the native driver (Case 2) have run on this platform. This page was followed on a freshly flashed devkit with Case 1. Two drives ran the bench, `plem drive run` and the A6B example's demo ([`docs/hardware-evidence.md`](hardware-evidence.md)); Case 2's three 1 kHz timing runs are in [`docs/benchmark/README.md`](benchmark/README.md). Menu 4 has not run a measurement (with the bus running it refused, as designed) |
| Jetson Thor | Not verified yet. The setup wizard is written for Orin |
| Orin NX, Orin Nano | The setup wizard is written for them but has run only on AGX Orin; the drive software is not supported |

The hardware runs ([`docs/hardware-evidence.md`](hardware-evidence.md)) were measured on JetPack 7.2.1 with ROS 2 Jazzy,
using Case 2 for the earlier sessions and Case 1 for the last three.

## 0. Choose the EtherCAT port

EtherCAT is a line: the master port connects to the first drive's IN port, and each drive's OUT port to the next
drive's IN. One port runs any number of drives.

| | Case 1 — built-in RJ45 | Case 2 — Intel I210 PCIe card |
|---|---|---|
| Hardware | Nothing extra. The devkit's RJ45 port (Tegra MGBE MAC, driver `nvethernet`) | An Intel I210-T1 card in the devkit's PCIe slot |
| IgH driver | `generic`: frames go through the Linux network stack. IgH has no native driver for `nvethernet` | `ec_igb`: native, polled by the bus loop, no network stack |
| Use it for | Evaluation and wiring checks. 1 kHz without lost frames is **not** guaranteed; a recommended cycle time is pending measurement | Running at 1 kHz |
| Network access | The built-in port becomes the EtherCAT port, so **SSH over it stops.** Use Wi-Fi or USB-C device mode | The built-in port stays your network port |

The wizard picks the driver from the port you select. It uses the native driver when IgH has one for the port's Linux
driver (`igb` for the I210), and `generic` otherwise. IgH also has a native driver for Intel I225/I226 (`ec_igc`),
which was not tested here. Case 1 gets no interface tuning: in our tests, changing IRQ affinity, offload and interrupt
coalescing did not remove the occasional lost frame at 1 kHz.

## 1. Flash JetPack 7.2.1

Flash JetPack 7.2.1 with NVIDIA's instructions (SDK Manager or the devkit's flashing guide). Then check:

```bash
cat /etc/nv_tegra_release                         # check: "# R39 (release), REVISION: 2.1..."
grep VERSION_CODENAME /etc/os-release             # check: VERSION_CODENAME=noble
```

## 2. Get the source and run the setup wizard

Clone this source tree, which contains the wizard, and start the wizard:

```bash
sudo apt update && sudo apt install -y git
cd ~ && git clone https://github.com/WIM-Corporation/PLEM.git
cd ~/PLEM
sudo bash setup/rt/setup.sh
```

Run menu **1** (install the RT kernel) and reboot. After the reboot, start the wizard again:

```bash
cd ~/PLEM
sudo bash setup/rt/setup.sh
```

Then run menu **2** (verify) and menu **3** (install the EtherCAT master). Menu 4, a `cyclictest` jitter load test on the
isolated CPU, is optional.

If you run menu 3 again while the master is loaded, it asks `Already installed. Reinstall? (y/N)`. Enter (N) returns
to the menu and changes nothing; `y` runs all of menu 3 again: it downloads and rebuilds IgH, picks the EtherCAT port
again and restarts the EtherCAT service. (If the master is not loaded, for example after a cancelled reinstall, it
asks `Install the EtherCAT master? (Y/n)` instead, and there Enter installs: answer `n` to return.) Stop anything
using the bus first (`plem drive run`, a robot control node): while an application holds the master the old module
cannot be unloaded, and menu 3 stops and says so — at port selection with the native backend, at the install step with
the generic one. From that point ethercat.service is stopped (an application that holds the master keeps its bus). If
you cancel after that point, it stays stopped; `sudo systemctl start ethercat` brings the previous setup back. Answer
`y` to change the port or to repair the install.

> **Case 1: keep a second network link before menu 3.** Menu 3 writes
> `/etc/NetworkManager/conf.d/plem-ethercat.conf`, which tells NetworkManager to stop managing the port you select
> (matched by its MAC address). From then on that port carries EtherCAT only. In Case 1 that port is the built-in RJ45,
> so the device loses its wired network there. Menu 3 itself downloads the IgH source, and sections 5 and 6 download
> packages, so connect Wi-Fi (or a second wired port) before running menu 3. When the built-in RJ45 is the only
> candidate port, menu 3 warns and asks before taking it.

| Menu | What it does |
|---|---|
| **1. Install RT kernel** (before reboot) | Installs NVIDIA's real-time kernel and makes it the default boot entry, isolates one CPU (the one isolated at boot, else the last one: CPU 11 on AGX Orin), applies RT tuning (including edits to the device tree, the boot entries and `/etc/fstab`), and gives your user real-time scheduling permission (section 3). It also holds the installed `nvidia-l4t-*` packages at their versions. [`setup/rt/README.md`](../setup/rt/README.md) has the full list (section 1) and, under "Removing the RT kernel", how to release them before a JetPack upgrade |
| **2. Verify setup** (after reboot) | Checks that menu 1 took effect. If none of those checks fails, it runs two live tests on the isolated CPU: a device-interrupt count around a short load, and a 30 s `cyclictest` at 1 kHz (skipped when `cyclictest` is missing or a real-time thread already runs on that CPU) |
| **3. Install EtherCAT master** | Builds IgH 1.6.13 into `/opt/etherlab`, selects the port (asking when there are several), writes `/etc/ethercat.conf`, takes the port away from NetworkManager, and enables and starts `ethercat.service`. Locked until menu 1 is complete and the RT kernel is booted |
| **4. RT jitter load test** | `cyclictest` on the isolated CPU under load, 1 to 5 kHz. Optional; run it before the application (it refuses while a control loop runs on that CPU) |

Before the menu, the wizard asks once whether to share device data with WIM. Nothing is sent unless you answer Yes
([`docs/telemetry.md`](telemetry.md)).

The build needs IgH under `/opt/etherlab` (menu 3); it is not a rosdep key.

The bus loop runs as `SCHED_FIFO` priority 98 on the last isolated CPU. `plem drive run --rt-cpu` picks another CPU.

[`setup/rt/README.md`](../setup/rt/README.md) covers the rest of the wizard: the `--baseline` option for a Jetson used
only as a controller (menu 1 then defaults to turning off services such a device does not use) and
[`setup/rt/crash-logging.sh`](../setup/rt/crash-logging.sh), which menu 1 runs (`bash setup/rt/crash-logging.sh --check`
reports its state without changing anything).

<details><summary>Two buses on one Jetson (not yet run on hardware)</summary>

Two buses on one Jetson have not yet run on hardware. Each bus is its own `plem drive run` process, one per master,
each in its own terminal with a different `bus.name`, `--master` and `--rt-cpu`. So each bus needs its own port and
its own isolated CPU. Setting up and running two buses:

- In menu 3, select both ports. They become `MASTER0` and `MASTER1` in the order the wizard lists them. The native
  driver needs both on the same Linux driver (two I210 cards, for example).
- Menu 1 isolates one CPU, so isolate the second one by hand. In `/boot/extlinux/extlinux.conf`, on the `APPEND` line
  of each label that boots `/boot/Image.real-time`, change the two parameters to
  `isolcpus=managed_irq,domain,10-11 irqaffinity=0-9`, then reboot. Menu 2 then checks the isolated CPUs against that
  range. Running menu 1 again sets them back to one CPU.
- Without `--rt-cpu`, both runs take the last isolated CPU.
- A bus cannot take a master that another EtherCAT application holds; it fails at start.
- The bench runs one bus at a time. Stop the other buses first: it refuses while another drive bus runs or the master
  is not Idle.

</details>

## 3. Real-time permission for your user

`plem drive run` raises its bus thread to `SCHED_FIFO` and locks memory. A non-root user needs limits for that, and
menu 1 sets them: it creates the `realtime` group, installs
[`setup/rt/99-plem-realtime.conf`](../setup/rt/99-plem-realtime.conf) in `/etc/security/limits.d/`, and adds the user
who ran `sudo` to the group. To add another user, run `sudo usermod -aG realtime <user>`.

Log out and in again, then:

```bash
ulimit -r        # check: 99
ulimit -l        # check: unlimited
```

Without these limits the bus still starts, but its log says that RT scheduling failed (it then runs with the default
policy) or that `mlockall` failed, and real-time timing is not guaranteed. Check the two values above and `id -nG`
in the shell that starts the bus. A process with `CAP_SYS_NICE` and `CAP_IPC_LOCK`, or run as root, is not bound by
these limits; the setup wizard does not set capabilities.

The same group opens the bus. Menu 3 writes `/etc/udev/rules.d/99-ethercat.rules` so that `/dev/EtherCAT0` (and any
further master) is mode 0660, group `realtime`. Only members can open it, so only they can write to the motion bus.
Before it writes the rule, menu 3 creates the group and adds the user who ran `sudo`, as menu 1 does; log out and in
again if it says it added you. A different existing rule (an older 0666 one, for example) is replaced and kept as
`99-ethercat.rules.plem-bak`. Menu 2 checks the device nodes and your membership, and prints the command to fix
either:

```bash
ls -l /dev/EtherCAT0     # check: crw-rw---- 1 root realtime ...
id -nG                   # check: lists realtime
```

## 4. Check the master

Wire the drives and power them, then:

```bash
ethercat master      # check: "Main: <MAC> (attached)" and "Link: UP"
ethercat slaves      # check: one line per drive, state PREOP
```

Case 1 only:

```bash
nmcli -t -f DEVICE,STATE device | grep end0   # check: end0:unmanaged (end0 is the built-in RJ45 port)
```

Case 2: the Jetson kernel has the `igb` driver built in, so IgH's `ethercatctl` cannot unload it, and `ec_igb` never
gets the card on its own. Menu 3 adds a helper (`/usr/local/sbin/plem_ethercat_native_bind.sh`) as
`ExecStartPost` of `ethercat.service`. On every start and restart it reads `/etc/ethercat.conf` and moves the
card from `igb` to `ec_igb` through PCI sysfs.

```bash
journalctl -u ethercat -b | grep summary   # check: "summary: rebound=1 ..." on the first start
ip -o link                                 # check: the I210 interface is gone — ec_igb owns the card
```

If the card or its MAC changes, run menu 3 again. Editing `/etc/ethercat.conf` alone leaves NetworkManager's
exclusion on the old MAC.

`plem drive run` fails at start while another EtherCAT application holds the same master.

## 5. Add the ROS 2 Jazzy apt repository

The build uses ROS 2 Jazzy from packages.ros.org. These commands are the repository setup from the official ROS 2
Jazzy install page, "Ubuntu (deb packages)" (https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). If
they fail, check that page.

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

If menu 1 turned off unused services, `apt update` and every package install or removal end with this line, printed
as an error: `GDBus.Error:org.freedesktop.systemd1.UnitMasked: Unit packagekit.service is masked.` It is harmless.
Menu 1 masks PackageKit, and Ubuntu's apt hook still tries to notify it; the apt command itself succeeded.

## 6. Build the drive software

Install the build tools, then build this tree in place:

```bash
sudo apt install -y ros-jazzy-ros-base ros-dev-tools ninja-build # ros-dev-tools brings colcon and rosdep
sudo rosdep init                                                # once per machine; "already exists" is fine
rosdep update
cd ~/PLEM
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -y
colcon build --cmake-args -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
sudo install -m 0755 tools/plem /usr/local/bin/plem             # the `plem` command
plem drive --help                                               # check: lists the verbs run and check
```

The build includes the tests. To run them (optional):

```bash
colcon test && colcon test-result --verbose
```

- **Each new terminal needs `source ~/PLEM/install/setup.bash`** before `plem drive` or a Python script that imports
  `plem.drive`. Without it `plem drive` says `plem drive: not installed on this device (no plem-drive)`, and Python
  `No module named 'plem'`.
- Building `plem_drive` prints a CMake deprecation warning for `tl_expected`. It comes from ROS 2 Jazzy's
  `controller_manager` dependencies, not from this tree, and does not affect the build.
- IgH is not a rosdep key. Without it under `/opt/etherlab` (menu 3), the `plem_fieldbus` configure step stops and
  says where it looked.
- The build needs Ninja: with Unix Makefiles, the ROS 2 message generators can race on generated sources. `RelWithDebInfo` keeps symbols and turns asserts off; an empty build type compiles without
  optimization.
- [`tools/plem`](../tools/plem) is the `plem` dispatcher. `plem <command> [arguments]` runs the `plem-<command>`
  executable on the PATH with the arguments unchanged, and `plem --help` lists the commands found. These pages
  write the drive tool as `plem drive <verb>`. Without the dispatcher installed, type `plem-drive <verb>`
  (`plem-drive check <file>`), the same tool.
- For `plem drive run --plot`, install PlotJuggler: `sudo apt install ros-jazzy-plotjuggler-ros`.
- For the `ros2 control` commands in [`docs/getting-started.md`](getting-started.md) (`list_controllers`,
  `switch_controllers`), install `sudo apt install ros-jazzy-ros2controlcli`. Without it `ros2` says
  `invalid choice: 'control'`.

Next: [`docs/getting-started.md`](getting-started.md). The rest of the platform is PLEM Pro
([`README.md`](../README.md#plem-pro)).
