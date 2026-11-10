#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem-fleet: device data sent to WIM with the operator's consent (docs/telemetry.md).

Standard library only and importing nothing from plem: the setup wizard runs this
file before ROS or plem_drive is built, and installs it as /usr/local/sbin/plem-fleet.

Everything a report holds is built field by field here or in fleet_session.py.
Never add raw command output, a log line, /proc/cmdline or a file path to a report:
those carry hostnames, user names and MAC addresses.

    plem-fleet on | off | status | notice | show | send | tick | forget-email
    plem-fleet decline | contact <email> | report-install   (used by the wizard)

PLEM_FLEET_ROOT (default empty) is put in front of every file path, for tests.
"""

from __future__ import annotations

import datetime
import glob
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
import uuid

_ROOT = os.environ.get("PLEM_FLEET_ROOT", "")
STATE_FILE = f"{_ROOT}/etc/plem/fleet.json"
FLEET_DIR = f"{_ROOT}/var/lib/plem/fleet"
SPOOL_DIR = f"{FLEET_DIR}/spool"
WORK_DIR = f"{FLEET_DIR}/work"
SERVER_STOP_FILE = f"{FLEET_DIR}/server-stop"
INSTALLED_TOOL = f"{_ROOT}/usr/local/sbin/plem-fleet"
SERVICE_FILE = f"{_ROOT}/etc/systemd/system/plem-fleet.service"
TIMER_FILE = f"{_ROOT}/etc/systemd/system/plem-fleet.timer"
LICENSED_DEVICE_FILE = f"{_ROOT}/etc/plem/license.key"

ENDPOINT = os.environ.get(
    "PLEM_FLEET_ENDPOINT", "https://depot.wimcorp.dev/api/fleet/v1"
)
NOTICE_VERSION = "1"
SCHEMA_VERSION = 1
SPOOL_CAP = 256
SERVER_STOP_RETRY_S = 7 * 24 * 3600
HTTP_TIMEOUT_S = 10
COMMAND_TIMEOUT_S = 10

NOTICE = """\
Help improve PLEM by sharing device data with WIM?
This helps us find and fix setup, hardware and drive problems on devices like this one.

WIM collects this device's serial number, hardware and software versions, setup results,
device health (uptime, restarts, temperature, storage), and for each EtherCAT session the
drive models, timing statistics and fault counts.
WIM keeps it while this device keeps reporting and may use it to fix problems, improve PLEM
and analyze how PLEM is used. If this device is later registered with WIM, the data may be
linked to it. Server logs may record your IP address.
When you turn this off, or 24 months after the device's last report, WIM removes the serial
number and the drive file digests, and keeps the remaining statistics.
Not collected: hostnames, file paths, joint data, drive configuration files.

If you are not responsible for this device, choose No.
Turn it off any time: sudo plem-fleet off
Details: docs/telemetry.md
"""

SERVICE_UNIT = """\
[Unit]
Description=PLEM fleet report (docs/telemetry.md)

[Service]
Type=oneshot
ExecStart=/usr/bin/python3 /usr/local/sbin/plem-fleet tick
DynamicUser=yes
SupplementaryGroups=realtime adm
Nice=19
IOSchedulingClass=idle
ProtectSystem=strict
ProtectHome=yes
PrivateTmp=yes
NoNewPrivileges=yes
ReadWritePaths=/var/lib/plem/fleet
"""

TIMER_UNIT = """\
[Unit]
Description=PLEM fleet report after boot and about every two hours

[Timer]
OnBootSec=5min
OnUnitActiveSec=2h
RandomizedDelaySec=30min
Persistent=true

[Install]
WantedBy=timers.target
"""


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _read(path: str) -> str | None:
    try:
        with open(_ROOT + path, encoding="utf-8", errors="replace") as file:
            return file.read().replace("\x00", " ").strip()
    except OSError:
        return None


def _run(*argv: str) -> str | None:
    """stdout of a command, or None if it is missing, fails or hangs."""
    try:
        done = subprocess.run(
            argv, capture_output=True, text=True, timeout=COMMAND_TIMEOUT_S, check=True
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return done.stdout.strip()


def read_serial() -> str | None:
    return _read("/proc/device-tree/serial-number") or None


def load_state() -> dict | None:
    try:
        with open(STATE_FILE, encoding="utf-8") as file:
            state = json.load(file)
    except (OSError, ValueError):
        return None
    return state if isinstance(state, dict) else None


def _server_stopped() -> bool:
    """WIM's server said to stop, recently: collect nothing until it is time to ask again."""
    try:
        return time.time() - os.path.getmtime(SERVER_STOP_FILE) < SERVER_STOP_RETRY_S
    except OSError:
        return False


def active_state() -> dict | None:
    """The consent record if collection may run on this device, else None.

    The serial check stops a cloned disk image from reporting: the consent was
    given on another device.
    """
    state = load_state()
    if not state or state.get("enabled") is not True:
        return None
    if state.get("serial") != read_serial() or os.path.exists(LICENSED_DEVICE_FILE):
        return None
    return None if _server_stopped() else state


def is_current(state: dict) -> bool:
    """Whether the consent `state` was read from still stands."""
    now = active_state()
    return now is not None and now["install_id"] == state["install_id"]


def _write_json(path: str, value: dict, mode: int = 0o644) -> None:
    """Whole file or nothing: a crash leaves the old file or none."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    handle, temp = tempfile.mkstemp(dir=os.path.dirname(path), prefix=".tmp-")
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as file:
            json.dump(value, file)
        os.chmod(temp, mode)
        os.replace(temp, path)
    except BaseException:
        os.unlink(temp)
        raise


def new_report(state: dict, kind: str, payload: dict) -> dict:
    return {
        "install_id": state["install_id"],
        "report_id": str(uuid.uuid4()),
        "kind": kind,
        "schema_version": SCHEMA_VERSION,
        "occurred_at": utc_now(),
        "reported_serial": state["serial"],
        "notice_version": state["notice_version"],
        "consented_at": state["consented_at"],
        "payload": payload,
    }


def spool(report: dict) -> None:
    _write_json(f"{SPOOL_DIR}/{report['report_id']}.json", report, 0o664)
    files = _spool_files()
    for old in files[: max(0, len(files) - SPOOL_CAP)]:
        os.unlink(old)


def _spool_files() -> list[str]:
    return sorted(glob.glob(f"{SPOOL_DIR}/*.json"), key=os.path.getmtime)


def clear_spool() -> None:
    for path in glob.glob(f"{SPOOL_DIR}/*.json") + glob.glob(f"{WORK_DIR}/*.json"):
        os.unlink(path)


def _http(
    method: str, path: str, body: dict | None = None
) -> tuple[int | None, dict, bytes]:
    request = urllib.request.Request(
        ENDPOINT + path,
        data=None if body is None else json.dumps(body).encode(),
        method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_S) as response:
            return response.status, dict(response.headers), response.read(1 << 16)
    except urllib.error.HTTPError as error:
        return error.code, dict(error.headers), b""
    except OSError:
        return None, {}, b""


def _is_disabled(status: int | None, headers: dict) -> bool:
    verdict = {key.lower(): value for key, value in headers.items()}.get("x-telemetry")
    return status == 403 and verdict == "disabled"


def _delivered(status: int | None, body: bytes, report_id: str) -> bool:
    """A misrouted request can answer 200 with a web page: only the echo proves delivery."""
    if status != 200:
        return False
    try:
        return json.loads(body).get("report_id") == report_id
    except (ValueError, AttributeError):
        return False


def send() -> int:
    """Send the spool, oldest first; the count delivered. Stops at the first network failure.

    Only reports of the current consent go out: a session that was still running when
    reporting was turned off must not bring its identifiers back.
    """
    state = active_state()
    delivered = 0
    for path in _spool_files():
        try:
            with open(path, encoding="utf-8") as file:
                report = json.load(file)
        except ValueError:
            os.unlink(path)
            continue
        if state is None or report.get("install_id") != state["install_id"]:
            os.unlink(path)
            continue
        status, headers, body = _http("POST", "/reports", report)
        if _is_disabled(status, headers):
            clear_spool()
            # Always a new file: the timer's user changes per run and cannot touch an old one.
            _write_json(SERVER_STOP_FILE, {}, 0o664)
            return delivered
        if _delivered(status, body, report["report_id"]):
            os.unlink(path)
            delivered += 1
        elif status in (410, 413, 422):  # this server will never take it
            os.unlink(path)
        else:
            break
    return delivered


def adopt_orphan_sessions() -> None:
    """Move session files whose writer died (power loss, kill) into the spool as they are."""
    for path in glob.glob(f"{WORK_DIR}/*.json"):
        match = re.fullmatch(r"(.+)-(\d+)\.json", os.path.basename(path))
        if not match or os.path.exists(f"/proc/{match.group(2)}"):
            continue
        finished = f"{SPOOL_DIR}/{match.group(1)}.json"
        if os.path.exists(
            finished
        ):  # it died between saving the final copy and cleaning up
            os.unlink(path)
        else:
            os.replace(path, finished)


def _kernel_args() -> dict:
    wanted = {}
    for word in (_read("/proc/cmdline") or "").split():
        key, _, value = word.partition("=")
        if key in ("isolcpus", "irqaffinity"):  # the rest can carry ids and addresses
            wanted[key] = value
    return wanted


def _meminfo_mib(key: str) -> int | None:
    match = re.search(rf"^{key}:\s+(\d+) kB", _read("/proc/meminfo") or "", re.M)
    return int(match.group(1)) // 1024 if match else None


def _thermal_c() -> dict:
    zones = {}
    for path in glob.glob(f"{_ROOT}/sys/class/thermal/thermal_zone*"):
        kind = _read(path[len(_ROOT) :] + "/type")
        milli = _read(path[len(_ROOT) :] + "/temp")
        if kind and milli and milli.lstrip("-").isdigit():
            zones[re.sub(r"\W", "_", kind) + "_c"] = int(milli) / 1000
    return zones


def _uptime_s() -> int | None:
    raw = _read("/proc/uptime")
    return int(float(raw.split()[0])) if raw else None


def _boot_count() -> int | None:
    listing = _run("journalctl", "--list-boots", "--no-pager", "-q")
    return len(listing.splitlines()) if listing else None


def _emmc_life() -> str | None:
    for path in glob.glob(f"{_ROOT}/sys/class/mmc_host/mmc*/mmc*:*/life_time"):
        return _read(path[len(_ROOT) :])
    return None


def _ethercat_backend() -> str | None:
    match = re.search(
        r'^DEVICE_MODULES="?([\w ]+)', _read("/etc/ethercat.conf") or "", re.M
    )
    return (
        match.group(1).strip() if match else None
    )  # never MASTER0_DEVICE: it is the MAC


def _ethercat_nic_driver() -> str | None:
    """Only the driver's name: the address finds the interface and goes no further."""
    match = re.search(
        r'^MASTER0_DEVICE="?([0-9A-Fa-f:-]+)', _read("/etc/ethercat.conf") or "", re.M
    )
    if not match:
        return None
    wanted = match.group(1).replace("-", ":").lower()
    for path in glob.glob(f"{_ROOT}/sys/class/net/*/address"):
        link = os.path.dirname(path) + "/device/driver"
        if (_read(path[len(_ROOT) :]) or "").lower() == wanted and os.path.islink(link):
            return os.path.basename(os.readlink(link))
    return None


def _block_size_mib(path: str) -> int | None:
    sectors = _read(path)  # always counted in 512-byte units
    return (
        int(sectors) * 512 // (1024 * 1024) if sectors and sectors.isdigit() else None
    )


def _l4t_core_held() -> bool | None:
    status = _run("dpkg-query", "-W", "-f=${Status}", "nvidia-l4t-core")
    return status.startswith("hold") if status else None


def _pstore_has_panic() -> bool:
    return bool(
        glob.glob(f"{_ROOT}/var/lib/systemd/pstore/**/dmesg-ramoops-*", recursive=True)
    )


def _disk_free_mib() -> int | None:
    try:
        return shutil.disk_usage(_ROOT or "/").free // (1024 * 1024)
    except OSError:
        return None


def _igh_version() -> str | None:
    match = re.search(
        r"\d+\.\d+(?:\.\d+)*", _run("/opt/etherlab/bin/ethercat", "version") or ""
    )
    return match.group(0) if match else None


def _nvpps_lock_state() -> str | None:
    """The state word of the wizard's nvpps tool. Only the word: its other lines name paths."""
    try:
        done = subprocess.run(
            [f"{_ROOT}/usr/local/sbin/plem-nvpps-tsc-unlock", "--check"],
            capture_output=True,
            text=True,
            timeout=COMMAND_TIMEOUT_S,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    lines = done.stderr.strip().splitlines()
    match = lines and re.match(
        r"(?:plem-nvpps-tsc-unlock: )?(applied|ok|pending|warn|n/a|failed)\b", lines[-1]
    )
    return match.group(1) if match else None


def _nvpmodel() -> str | None:
    match = re.search(r"NV Power Mode: (\S+)", _run("nvpmodel", "-q") or "")
    return match.group(1) if match else None


def _ethernet_pci_ids() -> list[str]:
    listing = _run("lspci", "-n", "-d", "::0200") or ""
    return re.findall(r"\b[0-9a-f]{4}:[0-9a-f]{4}\b", listing)


def collect_device() -> dict:
    thermal = _thermal_c()
    gpu_load = _read("/sys/devices/platform/bus@0/17000000.gpu/load")
    return {
        "board_model": _read("/proc/device-tree/model"),
        "dt_compatible": _read("/proc/device-tree/compatible"),
        "soc_id": _read("/sys/devices/soc0/soc_id"),
        "soc_revision": _read("/sys/devices/soc0/revision"),
        "mem_total_mib": _meminfo_mib("MemTotal"),
        "mem_available_mib": _meminfo_mib("MemAvailable"),
        "swap_total_mib": _meminfo_mib("SwapTotal"),
        "swap_free_mib": _meminfo_mib("SwapFree"),
        "nvme_model": _read("/sys/class/nvme/nvme0/model"),
        "nvme_size_mib": _block_size_mib("/sys/class/nvme/nvme0/nvme0n1/size"),
        "emmc_model": _read("/sys/block/mmcblk0/device/name"),
        "emmc_size_mib": _block_size_mib("/sys/block/mmcblk0/size"),
        "ethernet_pci_ids": _ethernet_pci_ids(),
        "l4t_core_version": _run(
            "dpkg-query", "-W", "-f=${Version}", "nvidia-l4t-core"
        ),
        "l4t_core_held": _l4t_core_held(),
        "kernel_release": platform.release(),
        "preempt_rt": _read("/sys/kernel/realtime") == "1",
        **_kernel_args(),
        "igh_version": _igh_version(),
        "ethercat_backend": _ethercat_backend(),
        "ethercat_nic_driver": _ethercat_nic_driver(),
        "ros_distros": sorted(
            os.path.basename(p) for p in glob.glob(f"{_ROOT}/opt/ros/*")
        ),
        "python_version": platform.python_version(),
        "uptime_s": _uptime_s(),
        "boot_count": _boot_count(),
        "reset_reason": _read("/sys/devices/platform/bus@0/c360000.pmc/reset_reason"),
        "pstore_panic_seen": _pstore_has_panic(),
        "load_avg_1m": float(os.getloadavg()[0]),
        "gpu_load_permille": int(gpu_load) if gpu_load and gpu_load.isdigit() else None,
        "thermal": thermal,
        "temp_max_c": max(thermal.values(), default=None),
        "power_mode": _nvpmodel(),
        "nvpps_lock_state": _nvpps_lock_state(),
        # The file, not systemctl: the timer's sandbox has no connection to systemd.
        "jetson_clocks_enabled": os.path.exists(
            f"{_ROOT}/etc/systemd/system/multi-user.target.wants/jetson-clocks.service"
        ),
        "disk_free_mib": _disk_free_mib(),
        "emmc_life_time": _emmc_life(),
    }


def tick() -> int:
    """One timer run: report this device's state, then send what is queued."""
    state = active_state()
    if state is None:
        return 0
    adopt_orphan_sessions()
    spool(new_report(state, "device", collect_device()))
    send()
    return 0


def _kept_contact(state: dict) -> dict:
    """The way to remove an email given separately from reporting."""
    contact_id = state.get("contact_install_id")
    return {} if contact_id is None else {"contact_install_id": contact_id}


def _record_decision(enabled: bool) -> dict:
    previous = load_state() or {}
    state = _kept_contact(previous) | {
        "enabled": enabled,
        "install_id": previous.get("install_id") or str(uuid.uuid4()),
        "serial": read_serial(),
        "notice_version": NOTICE_VERSION,
        "consented_at": utc_now(),
    }
    _write_json(STATE_FILE, state)
    return state


def _systemctl(*args: str) -> None:
    if not _ROOT:
        subprocess.run(["systemctl", *args], check=False, capture_output=True)


def _make_fleet_dirs() -> None:
    if (
        not _ROOT
    ):  # the wizard asks before menu 1, which is what normally creates the group
        subprocess.run(
            ["groupadd", "--system", "-f", "realtime"], check=True, capture_output=True
        )
    for path in (FLEET_DIR, SPOOL_DIR, WORK_DIR):
        os.makedirs(path, exist_ok=True)
        os.chmod(
            path, 0o2775
        )  # sessions are written by the operator, in group realtime
        if not _ROOT:
            shutil.chown(path, group="realtime")


def _install_tool() -> None:
    os.makedirs(os.path.dirname(INSTALLED_TOOL), exist_ok=True)
    if os.path.realpath(__file__) != os.path.realpath(INSTALLED_TOOL):
        shutil.copy(__file__, INSTALLED_TOOL)
    os.chmod(INSTALLED_TOOL, 0o755)


def turn_on() -> int:
    _record_decision(True)
    _make_fleet_dirs()
    _install_tool()
    os.makedirs(os.path.dirname(SERVICE_FILE), exist_ok=True)
    with open(SERVICE_FILE, "w", encoding="utf-8") as file:
        file.write(SERVICE_UNIT)
    with open(TIMER_FILE, "w", encoding="utf-8") as file:
        file.write(TIMER_UNIT)
    _systemctl("daemon-reload")
    _systemctl("enable", "--now", "plem-fleet.timer")
    print("Device reporting is on. Turn it off any time: sudo plem-fleet off")
    return 0


def turn_off() -> int:
    """Collection stops here whether or not the server answers. The install id is
    kept until the server confirms, so running this again can finish the job."""
    state = load_state()
    _systemctl("disable", "--now", "plem-fleet.timer")
    for path in (SERVICE_FILE, TIMER_FILE, SERVER_STOP_FILE):
        if os.path.exists(path):
            os.unlink(path)
    _systemctl("daemon-reload")
    clear_spool()
    if not state or not state.get("install_id"):
        print("Device reporting is off.")
        return 0
    _write_json(STATE_FILE, state | {"enabled": False})
    status, _, _ = _http("DELETE", f"/installs/{state['install_id']}")
    if status != 204:
        print(
            "Device reporting is off on this device, but WIM's server could not be reached, "
            "so the serial number it holds was not removed. Run this again when online.",
            file=sys.stderr,
        )
        return 1
    # The server just deleted an email tied to this install; one given separately stays.
    separate = state.get("contact_install_id") not in (None, state["install_id"])
    _write_json(
        STATE_FILE, {"enabled": False} | (_kept_contact(state) if separate else {})
    )
    print("Device reporting is off. WIM removed this device's serial number.")
    return 0


def contact(email: str) -> int:
    """Optional and separate from reporting: works whether reporting is on or off."""
    active = active_state()
    previous = load_state() or {"enabled": False}
    state = active or {
        "install_id": previous.get("contact_install_id") or str(uuid.uuid4()),
        "serial": None,
        "notice_version": NOTICE_VERSION,
        "consented_at": utc_now(),
    }
    report = new_report(
        state, "contact", {"email": email, "purpose": "notices_and_early_access"}
    )
    status, headers, body = _http("POST", "/reports", report)
    if _is_disabled(status, headers) or not _delivered(
        status, body, report["report_id"]
    ):
        print("Could not register that address right now.", file=sys.stderr)
        return 1
    _write_json(STATE_FILE, previous | {"contact_install_id": state["install_id"]})
    _install_tool()  # the wizard tells the operator to remove the address with it
    return 0


def forget_email() -> int:
    state = load_state() or {}
    install_id = state.get("contact_install_id")
    if not install_id:
        print("No email address was registered from this device.")
        return 0
    status, _, _ = _http("DELETE", f"/installs/{install_id}/contact")
    if status != 204:
        print(
            "WIM's server could not be reached, so the address was not removed. "
            "Run this again when online.",
            file=sys.stderr,
        )
        return 1
    del state["contact_install_id"]
    _write_json(STATE_FILE, state)
    print("WIM removed the email address.")
    return 0


def report_install(lines: list[str]) -> int:
    """From the wizard: `menu|step|result` lines, and `cyclictest|max_us|avg_us`."""
    state = active_state()
    if state is None:
        return 0
    steps, cyclictest = [], None
    for line in lines:
        fields = line.strip().split("|")
        if fields[0] == "cyclictest" and len(fields) == 3:
            cyclictest = {"max_us": _number(fields[1]), "avg_us": _number(fields[2])}
        elif len(fields) == 3:
            steps.append({"menu": fields[0], "step": fields[1], "result": fields[2]})
    _make_fleet_dirs()
    spool(new_report(state, "install", {"steps": steps, "cyclictest": cyclictest}))
    send()
    return 0


def _number(text: str) -> float | None:
    try:
        return float(text)
    except ValueError:
        return None


def _status() -> int:
    state = load_state()
    active = active_state() is not None
    print(f"reporting: {'on' if active else 'off'}")
    if state and state.get("install_id"):
        print(f"install id: {state['install_id']}")
    print(f"queued reports: {len(_spool_files())}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    verb = args[0] if args else ""
    if verb == "notice":
        print(NOTICE)
        return 0
    if verb == "show":
        print(json.dumps(collect_device(), indent=2))
        print(
            "\nThis is the device report, sent about every two hours. The setup and EtherCAT session reports are in docs/telemetry.md."
        )
        return 0
    if verb == "status":
        return _status()
    if verb == "tick":
        return tick()
    if verb == "send":
        print(f"delivered: {send()}")
        return 0
    if (
        verb in ("on", "off", "decline", "report-install", "contact", "forget-email")
        and os.geteuid() != 0
        and not _ROOT
    ):
        print("plem-fleet: this needs root: run it with sudo", file=sys.stderr)
        return 1
    if verb == "on":
        return turn_on()
    if verb == "off":
        return turn_off()
    if verb == "decline":
        _write_json(STATE_FILE, {"enabled": False} | _kept_contact(load_state() or {}))
        return 0
    if verb == "report-install":
        return report_install(sys.stdin.readlines())
    if verb == "contact" and len(args) == 2:
        return contact(args[1])
    if verb == "forget-email":
        return forget_email()
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
