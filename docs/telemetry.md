# Device reporting

The setup wizard asks once whether to share device data with WIM Inc. ("WIM"), the company that makes PLEM. Nothing is
collected unless someone answers Yes. Answering No, or turning reporting off later, does not change how PLEM works.
A device where nobody answered, such as an unattended install, reports nothing. A disk image cloned from a device that
answered Yes does not report either, because the consent was given on another device.

The code that builds every report is [`plem_drive/plem/drive/fleet.py`](../plem_drive/plem/drive/fleet.py) and
[`plem_drive/plem/drive/cli/fleet_session.py`](../plem_drive/plem/drive/cli/fleet_session.py). To see this device's
device report before deciding, pick *View this device's report* in the wizard, or run `python3 plem_drive/plem/drive/fleet.py show`.

## What is sent

| Report | When | Contents |
|---|---|---|
| Setup | When the wizard exits | Each menu step's name and result, and the cyclictest maximum and average |
| Device | Between 5 and 35 minutes after boot, then about every 2 hours | The module serial number, board and SoC model, the device tree compatible string, SoC revision, memory and swap, NVMe and eMMC model and size, Ethernet PCI IDs, the EtherCAT network driver's name, L4T version and whether its package is held, kernel version, whether the kernel is PREEMPT_RT, `isolcpus` and `irqaffinity`, IgH version and backend, ROS 2 and Python versions, uptime, boot count, the last reset reason, whether a kernel panic record exists, load, GPU load, temperatures, power mode, nvpps lock state, jetson-clocks state, free disk space and eMMC wear |
| EtherCAT session | For each `plem drive run` | Vendor ID, product code, ring position and axis count of each slave, the scope of the drive identity check, each axis's control mode, cycle time and sync mode, approval state, software revision, digests of the drive model file and profile, run time, cycle counts, overruns, working-counter errors, lost samples, cycle work (mean, maximum), period deviation (p99, maximum), DC deviation maximum, and counts of events, limit stops, protective stops by source and drive fault codes by axis |

A session running longer than 2 hours is reported in 2-hour parts. A session is saved every 10 minutes, so a power loss can lose up to the last 10 minutes of its statistics.

## What is not sent

Hostnames, user names, file paths, MAC addresses, the PLEM license key, the kernel command line other than the two values above, log text,
drive model file contents (axis names, limits, startup values), joint positions, velocities, torques and commands.

WIM's server logs can record the IP address of a connection.

Each report also carries a random install identifier, the time it was made, and when and to which version of the
wizard's question consent was given.

## How it is used and kept

WIM uses this data to find and fix problems, improve PLEM and analyze how PLEM is used. If this device is later
registered with WIM, the data may be linked to it.

WIM keeps the data while the device keeps reporting. When you turn reporting off, or 24 months after the device's last
report, WIM removes the serial number and the drive model file digests, and refuses further reports under that install
identifier. The remaining statistics are kept. They still include hardware details, and one install's reports stay
grouped together.

The data is stored on servers WIM operates in South Korea.

## Turning it off

```bash
sudo plem-fleet off      # stops collection, deletes queued reports, and asks WIM's server to remove the identifiers
plem-fleet status
```

If the server cannot be reached, collection still stops. Run the command again when online to finish removing the
identifiers. To skip the question on a device you set up yourself, run the wizard with `PLEM_TELEMETRY=off`.

## Email address

The wizard also offers an optional email address for notices about newly validated JetPack versions and early access to
PLEM Pro. It is separate from device reporting: you can give one without the other. If device reporting is on, the
address is linked to this device. `sudo plem-fleet forget-email` deletes it at any time, and turning reporting off
deletes one linked to this device. WIM deletes an address it still holds 24 months after the device's last report, or
24 months after it was given if reporting is off.

## Questions and requests

To ask what WIM holds about a device, or to have it corrected or deleted, email <william@wimcorp.co.kr>. If the device
is gone and `sudo plem-fleet off` can no longer be run, include its module serial number.
