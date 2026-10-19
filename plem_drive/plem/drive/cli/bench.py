# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The drive bench — measure a drive model file not yet approved and approve it.

    plem drive run <model.yaml>     (on a terminal, for a file not approved)

`plem drive run` offers it and calls main([<model.yaml>, --master N
[, --rt-cpu N]]) on a terminal. The bus runs on a private namespace
(/plem_drive_bench), not the file's bus.name, so no Bus("<bus.name>") client
attaches to it.

Starts the bus the way `plem drive run --unapproved` does (the ros2_control
chain, as a child it owns) and, with an operator at the drive, releases the
declared axes' brakes three or four times: held where they rest (a release
latches csp at its position and csv at 0 rad/s; a bus with a csv or cst axis
streams 0 to its stream controller), and, on a bus with no cst axis, once while
moving: every csp and csv axis travels a short way and then takes a requested
stop (stop-decel, the bench's only motion). It measures the bench items of docs/cia402-drive-model.md §6.1
and passes them to the C++ `drive_model_tool approve` in a bench file in a
private temporary directory, removed afterwards. `drive_model_tool approve`
decides and records the approval with its evidence in the model file itself, so
the model file is the only file to keep. A file whose approval record no longer
holds (it was edited after its bench) is benched the same way and its record replaced, and so is an approved file
(`plem drive run` offers it when the drives on the bus differ from the record).
Nothing is written before the confirmation: a refusal or a decline is one
line and leaves the last `<stem>.bench.log` as it was. A confirmed run that
does not end approved keeps `<stem>.bench.log`, prints its path and leaves the
model file as it was: re-run the bench.

It drives the bus through the user's client, `plem.drive.Bus`, for every
release and stop and for stop-decel's motion (the csp move and the csv ramp
stream). It sends `reset_error` and the second `release_brake`, which must be
a no-op, directly through the control service; `Bus.release()` would restart
the trajectory controller first. The stream at rest runs as a separate
`ros2 topic pub` process so stale-engage can kill the sender alone.

It measures each item and marks it PASS or FAIL against the thresholds below,
or NA where the item does not apply to the axes. The required item ids, the
digests, the axis/slave addressing and the approval come from the C++ `drive_model_tool` (check/approve), which checks
the ids, digests and record when it approves; what the bus saw comes from its
topics (the joint state broadcaster's joint states, the drive bus's status,
events and info). It reads neither the YAML nor the bus's log. The thresholds
are the bench's own pass bars, chosen from bench runs on real drives; they are
not the axes' safety limits. Rename rule (docs/cia402-drive-model.md §6.1): a
change that makes an item's pass contract stricter renames that id in the C++
kRequiredBenchItems.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
from fractions import Fraction
import io
import json
import math
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Callable

from plem_drive_msgs.msg import DriveStatus
from plem_rt_msgs.msg import RtEvent
from plem.drive.cia402 import RESTING, state
from plem.drive.cli.run import (
    BENCH_NAMESPACE,
    BENCH_NAMESPACE_ENV,
    CHECK_FORMAT,
    CHECK_MISMATCH,
    drive_model_tool,
    own_pids,
    plem_drive_command,
    run_pids,
)

# Abort bars, holds and timeouts — change one only under the rename rule above.
DRIFT_ABORT_RAD = (
    0.005  # a released axis held at rest moving farther aborts and engages
)
STATUS_AGE_ABORT_S = 0.35  # ~/status is published at 10 Hz
JOINT_STATES_AGE_ABORT_S = 0.2
STALE_MARGIN_S = (
    0.01  # over check's command_stale_timeout_ns: the RT command age at the 0x54 event
)
HOLD_S = 2.0  # first release hold
SECOND_HOLD_S = 1.0  # after reset, second release
PUBLISH_HZ = 200  # the streams the bench publishes
COMMAND_WAIT_S = 3.0
# The controller manager, its hardware (up once every slave is in OP — the bus
# waits up to 60 s for that, DriveBus::kOpTimeout) and the spawned controllers.
NODE_START_S = 75.0
NODE_STOP_S = 15.0
CLI_TIMEOUT_S = 10.0
# stop-decel: each csp and csv axis moves at most STOP_DISTANCE_RAD toward its
# roomier side (nine tenths of the room) at a cruise of min(CRUISE_FRACTION x
# velocity_cap_rad_s, the speed that leaves half the distance at cruise, the
# speed held at least MIN_CRUISE_S), then the requested stop; its average
# deceleration to rest may exceed limits.acceleration_rad_s2 by DECEL_MARGIN.
STOP_DISTANCE_RAD = 0.5
MIN_STOP_DISTANCE_RAD = 0.05
CRUISE_FRACTION = 0.3
# joint_states trails the command by the drive's response (~33 ms measured on
# an A6B) plus up to one broadcaster period (10 ms at 100 Hz): asked mid-cruise
# (50 ms in), this leaves that much and a thin margin.
MIN_CRUISE_S = 0.1
DECEL_MARGIN = 1.1
REST_RAD_S = 0.01  # at rest: slower than this, or than 2 % of the speed at the stop
# What the hardware plugin (drive_system_interface.cpp) logs when its stop confirmed
# every brake engaged (INFO) or did not (ERROR) — stop_brakes_line;
# its test pins all three lines word for word and a bench test checks
# the source carries these literals. ros2_control exits 0 either way, so these lines
# are the only report on the brakes: the confirming line must be present, and its
# absence (a plugin that never stopped, or a lost line) is not confirmation. The
# bench runs the chain with --verbose so the INFO line is printed.
CONFIRMED_BRAKES = "brakes confirmed engaged on stop"
UNCONFIRMED_BRAKES = "brakes were NOT confirmed engaged on stop"
NEVER_RELEASED_BRAKES = "brakes were never released this run"

QUIET_STATES = (
    "SWITCH_ON_DISABLED",
    "NOT_READY_TO_SWITCH_ON",
    "READY_TO_SWITCH_ON",
    "SWITCHED_ON",
    "OPERATION_ENABLED",
    "QUICK_STOP_ACTIVE",
)
SLAVE_HINT = (
    "`sudo dmesg | grep -i ethercat` shows the AL status code or a refused startup SDO "
    "(startup_sdo, or dc.write_cycle_time's 0x1C32:02)"
)
# SDO aborts that mean "this drive has no such object" — every other upload
# failure (permission, timeout, another abort) is an infrastructure error.
ABSENT_ABORTS = (0x06020000, 0x06090011)
# An upload that fails without an SDO abort is read again after these delays:
# IgH 1.6 can hand an upload the answer to the master's own mailbox traffic
# (its SDO dictionary read after a bus rescan). A heuristic, the same as the
# fieldbus's init-time reads: it does not prove that was the cause. The text
# is what IgH 1.6.13's `ethercat upload` printed right after a bus rescan;
# other wording aborts at once.
UPLOAD_NO_ABORT = "Failed to upload SDO: Input/output error"
UPLOAD_RETRY_DELAYS_S = (0.1, 0.3, 0.5)


class BenchError(Exception):
    """A refusal or an abort; the message is what the operator reads."""


class CheckFailed(BenchError):
    """A measured check did not hold; the message is its pass condition."""


# check output


@dataclasses.dataclass(frozen=True)
class Axis:
    slave: int  # the model file's slaves[] index (the approval record's `slave`)
    position: int  # ring position (the bus, `ethercat -p`)
    channel: int  # 1..8
    od_offset: int  # the channel's object offset (channel n = 0x800·(n-1))
    name: str
    lo: float
    hi: float
    vmax: float
    mode: str  # cst | csp | csv
    encoder_resolution: int  # counts per motor turn the file claims 0x6064 carries
    flip: bool
    # this axis's slave's startup_sdo: {(index, subindex): value} — what the
    # drive runs with once the node starts, whatever it holds before
    startup_sdo: dict = dataclasses.field(default_factory=dict, compare=False)
    # a rated_per_mille axis: the file's motor rated torque, in 0x6076's unit (mNm)
    rated_torque_mnm: float | None = None
    vcap: float = 0.0  # velocity_cap_rad_s — what a planner may command
    accel: float | None = None  # limits.acceleration_rad_s2: the stop's (csp, csv)

    def sdo(self, base: int) -> int:
        """The object this axis owns at `base` (check's od_offset added)."""
        return base + self.od_offset


@dataclasses.dataclass(frozen=True)
class Slave:
    position: int
    scanned: bool


@dataclasses.dataclass(frozen=True)
class Check:
    config_digest: str
    profile_digest: str
    revision: str
    claimed: str
    reasons: tuple[str, ...]  # why an approved claim is not honored
    approve_blocker: str  # '' = approve would take the file
    warnings: tuple[str, ...]  # shown, never a refusal
    required: tuple[str, ...]
    cycle_time_ns: int
    command_stale_timeout_ns: int  # the node's command freshness
    fault_reset_policy: str
    axes: tuple[Axis, ...]
    slaves: tuple[Slave, ...]


def parse_check(text: str) -> Check:
    """`plem drive check --json` stdout (one JSON document) → Check. Another format
    or a missing key is a refusal: the runner and the tool must come from the
    same release."""
    mismatch = BenchError(CHECK_MISMATCH)
    try:
        c = json.loads(text)
    except ValueError:
        raise mismatch from None
    if not isinstance(c, dict) or c.get("format") != CHECK_FORMAT:
        raise mismatch
    try:
        slaves = tuple(
            Slave(s["ring_position"], s["pdo_mapping"] == "scanned")
            for s in c["slaves"]
        )
        writes = [
            {(index, sub): value for index, sub, value in s["startup_sdo"]}
            for s in c["slaves"]
        ]
        axes = tuple(
            Axis(
                slave=a["slave"],
                position=a["ring_position"],
                channel=a["channel"],
                od_offset=a["od_offset"],
                name=a["name"],
                lo=-math.inf
                if a["position_min_rad"] is None
                else a["position_min_rad"],
                hi=math.inf if a["position_max_rad"] is None else a["position_max_rad"],
                vmax=a["velocity_limit_rad_s"],
                mode=a["mode"],
                encoder_resolution=a["encoder_resolution"],
                flip=a["flip"],
                startup_sdo=writes[a["slave"]] if 0 <= a["slave"] < len(writes) else {},
                rated_torque_mnm=a["rated_torque_mnm"],
                vcap=a["velocity_cap_rad_s"],
                accel=a["acceleration_limit_rad_s2"],
            )
            for a in c["axes"]
        )
        # In the format but unread here: a missing one still means another release.
        for key in ("bus_name", "approval_effective"):
            c[key]
        check = Check(
            config_digest=c["config_digest"],
            profile_digest=c["profile_digest"],
            revision=c["implementation_revision"],
            claimed=c["approval_claimed"],
            reasons=tuple(c["reasons"]),
            approve_blocker=c["approve_blocker"] or "",
            warnings=tuple(c["warnings"]),
            required=tuple(c["required_bench_items"]),
            cycle_time_ns=c["cycle_time_ns"],
            command_stale_timeout_ns=c["command_stale_timeout_ns"],
            fault_reset_policy=c["fault_reset_policy"],
            axes=axes,
            slaves=slaves,
        )
    except KeyError as e:
        raise BenchError(
            f"plem drive check printed no {e} — the plem packages are from different releases; "
            "install matching versions"
        ) from None
    except TypeError:
        raise mismatch from None
    for a in axes:
        if (
            not 0 <= a.slave < len(slaves)
            or slaves[a.slave].position != a.position
            or not 1 <= a.channel <= 8
        ):
            raise BenchError(
                f"plem drive check printed an inconsistent axis entry for {a.name}"
            )
    if not axes:
        raise BenchError("plem drive check printed no axes")
    return check


def refusal_text(checked) -> str:
    """Why check refused: the approval reasons of its JSON when stdout carries
    one (stderr then holds only the loader's mapping line), else its stderr."""
    try:
        reasons = json.loads(checked.stdout)["reasons"]
    except (ValueError, KeyError, TypeError):
        reasons = None
    return "\n".join(reasons) if reasons else checked.stderr or checked.stdout


MODE_BITS = {"csp": 7, "csv": 8, "cst": 9}  # 0x6502 supported drive modes
# (base, name, size) a csp/csv axis's stop depends on.
STOP_OBJECTS = (
    (0x605A, "quick stop option code", 2),
    (0x605C, "disable operation option code", 2),
    (0x6084, "profile deceleration", 4),
    (0x6085, "quick stop deceleration", 4),
)


SCALING_OBJECTS = (0x607E, 0x608F, 0x6091, 0x6092)  # a write to one rescales 0x6064


def drive_object_report(axis: Axis, read) -> tuple[str, list[str], list[str]]:
    """The axis's CiA 402 objects read before the node starts — `read(index,
    subindex, typ)` returns the value, None for an absent object, or raises
    BenchError (any other failure: "not compared", never an abort). A value
    the file's startup_sdo writes overrides the read one (the drive runs with
    it). Returns (failure, warnings, information): failure = the drive cannot
    run the file's mode (0x6502 — the only refusal); warnings = drive-side
    polarity against flip, or scaling objects that disagree with
    encoder_resolution; information = what was not compared, and the stop
    values the drive holds now that the file does not declare, as
    startup_sdo lines ready to paste."""
    failure, warns, info, failed = "", [], [], set()

    def get(base: int, sub: int, typ: str) -> int | None:
        index = axis.sdo(base)
        if (index, sub) in axis.startup_sdo:
            return axis.startup_sdo[(index, sub)]
        try:
            return read(index, sub, typ)
        except (BenchError, ValueError, IndexError) as e:
            failed.add((base, sub))
            info.append(
                f"{axis.name}: 0x{index:04X}:{sub} not read ({e}) — not compared"
            )
            return None

    supported = get(0x6502, 0, "uint32")
    if supported is None:
        if (0x6502, 0) not in failed:
            info.append(f"{axis.name}: no 0x6502 — mode support not compared")
    elif not supported >> MODE_BITS[axis.mode] & 1:
        failure = (
            f"{axis.name}: 0x{axis.sdo(0x6502):04X} supported drive modes 0x{supported:08X} lacks "
            f"{axis.mode} (bit {MODE_BITS[axis.mode]}) — the drive cannot run the file's mode; choose a mode "
            "the drive supports"
        )
    else:
        info.append(
            f"{axis.name}: 0x{axis.sdo(0x6502):04X} supported drive modes 0x{supported:08X} include {axis.mode}"
        )
    polarity = get(0x607E, 0, "uint8")
    if polarity and axis.flip:
        warns.append(
            f"WARN {axis.name}: 0x{axis.sdo(0x607E):04X} polarity 0x{polarity:02X} — the drive inverts on its "
            "own and the file's flip (true) inverts again; set the direction in one place (0x607E = 0 with flip: true, "
            "or keep 0x607E and set flip: false)"
        )
    elif polarity == 0:
        info.append(
            f"{axis.name}: 0x{axis.sdo(0x607E):04X} polarity 0 — the drive does not invert"
        )
    elif polarity and (axis.sdo(0x607E), 0) not in axis.startup_sdo:
        info.append(
            f"{axis.name}: the drive inverts on its own (0x{axis.sdo(0x607E):04X} = 0x{polarity:02X}) — declare it "
            "in startup_sdo so a replacement drive keeps the direction"
        )
    pair = lambda base: tuple(get(base, sub, "uint32") for sub in (1, 2))  # noqa: E731
    feed, gear, encoder = pair(0x6092), pair(0x6091), pair(0x608F)
    one_to_one = not all(gear) or gear[0] == gear[1]
    if all(feed):
        gear = gear if all(gear) else (1, 1)
        counts = Fraction(feed[0], feed[1]) / Fraction(gear[0], gear[1])
        source = f"0x6092 feed {feed[0]}/{feed[1]} over 0x6091 gear {gear[0]}/{gear[1]}"
    elif all(encoder) and one_to_one:
        counts = Fraction(encoder[0], encoder[1])
        source = f"0x608F encoder increments {encoder[0]}/{encoder[1]}"
    else:
        counts, source = None, "no usable 0x6092 feed or 0x608F encoder resolution"
    if counts is None or counts.denominator != 1 or counts <= 1:
        # Drives often leave these objects at defaults or use them otherwise.
        info.append(
            f"{axis.name}: scaling not compared ("
            + (source if counts is None else f"{source} gives {float(counts):g}")
            + ")"
        )
    elif counts == axis.encoder_resolution:
        info.append(
            f"{axis.name}: scaling {source} = {int(counts)} counts per motor turn, as encoder_resolution says"
        )
    elif counts != axis.encoder_resolution:
        warns.append(
            f"WARN {axis.name}: the drive's scaling objects give {int(counts)} counts per motor turn in 0x6064 "
            f"({source}); the file's factors.encoder_resolution is {axis.encoder_resolution} — positions would be "
            f"off by {float(counts / axis.encoder_resolution):.6g}×. Set encoder_resolution to {int(counts)}, or the "
            f"drive's scaling to {axis.encoder_resolution} (a vendor-specific electronic gear is not read)"
        )
    objects = STOP_OBJECTS if axis.mode in ("csp", "csv") else ()
    holds = []
    for base, _, size in objects:
        if (axis.sdo(base), 0) in axis.startup_sdo:
            continue
        value = get(base, 0, "uint16" if size == 2 else "uint32")
        if value is not None:
            holds.append(
                f"      - {{ index: 0x{axis.sdo(base):04x}, subindex: 0, size: {size}, value: {value} }}"
            )
    if holds:
        info.append(f"{axis.name}: the drive holds now (not declared in startup_sdo):")
        info.extend(holds)
    return failure, warns, info


ERROR_CLASSES = {
    0x0: "no error",
    0x1: "generic",
    0x2: "current",
    0x3: "voltage",
    0x4: "temperature",
    0x5: "device hardware",
    0x6: "device software",
    0x7: "additional modules",
    0x8: "monitoring",
    0x9: "external",
    0xF: "additional functions",
}
ERROR_CODES = {0x8611: "following error", 0x8612: "reference limit"}  # CiA 402


def error_code_text(code: int) -> str:
    """0x603F with its CiA 301/402 class — the drive manual names the rest."""
    if code == 0:
        return "0x0000 no error"
    if code in ERROR_CODES:
        return f"0x{code:04X} {ERROR_CODES[code]}"
    what = (
        "device specific"
        if code >= 0xFF00
        else ERROR_CLASSES.get(code >> 12, "unclassified")
    )
    return f"0x{code:04X} {what} — see the drive manual"


def status_bit_notes(history, axes: tuple[Axis, ...], since_ns: int = 0) -> list[str]:
    """Statusword bit 11 (internal limit active) on any axis, in the statuses
    the bench saw from `since_ns` on (sampled at the status rate — a short
    pulse can be missed). Not bit 13 (following error): PLEM does not use it,
    and a drive may set it through normal motion."""
    seen = [0] * len(axes)
    for t, (*_, words) in history:
        if t < since_ns:
            continue
        for i, w in enumerate(words[: len(axes)]):
            seen[i] |= w
    notes = []
    for a, w in zip(axes, seen):
        if w & 0x0800:
            notes.append(
                f"WARN {a.name}: statusword bit 11 (internal limit active) seen during the bench"
            )
    return notes


def position_continuity_note(
    axis: Axis, before: int | None, after: int | None, moved: bool = False
) -> str:
    """0x6064 read before the node started and after it stopped. A held axis
    moves far below a turn (the drift abort), so a turn or more means the
    drive lost or rebuilt its position. An axis stop-decel `moved` is not
    checked: its 0.5 rad can be many motor turns through a gearhead, and
    check's JSON does not carry the gear ratio."""
    if any(
        (axis.sdo(base), sub) in axis.startup_sdo
        for base in SCALING_OBJECTS
        for sub in (0, 1, 2)
    ):
        return f"{axis.name}: startup_sdo rescales 0x6064 — continuity not compared"
    if before is None or after is None:
        return (
            f"{axis.name}: 0x6064 not read before and after — continuity not compared"
        )
    counts = after - before
    turns = abs(counts) / axis.encoder_resolution
    line = f"{axis.name}: 0x6064 moved {counts} counts ({turns:.3g} motor turns) between the reads before the node and after it"
    if moved:
        # Not checked on the csp and csv axes of a bus with no cst axis (stop-decel
        # moves them): that needs gear_ratio in check's JSON, compared against
        # joint_states' net motion.
        return f"{line} — stop-decel moved it, continuity not judged"
    return (
        f"WARN {line} — the drive lost or rebuilt its position" if turns >= 1 else line
    )


def rated_torque_note(axis: Axis, raw: bytes | None) -> str:
    """0x6076 Motor rated torque (an `ethercat upload -t uint32` answer, None =
    absent) against the file's rated_per_mille value — a log line, never a
    refusal. A wrong value scales every torque command and reading; the bench
    commands only a neutral torque, so nothing else here would show it."""
    index = axis.sdo(0x6076)
    if raw is None:
        return f"{axis.name}: the drive has no 0x{index:04X} motor rated torque — not compared"
    live = int(raw.split()[-1], 0)
    if live in (0, 0xFFFFFFFF):
        return f"{axis.name}: 0x{index:04X} motor rated torque reads 0x{live:08X} (unset) — not compared"
    file = axis.rated_torque_mnm  # > 0: the loader refuses anything else
    if abs(live - file) <= max(1, 0.01 * file):  # 1 %, at least 1 mNm
        return f"{axis.name}: 0x{index:04X} motor rated torque {live} mNm matches factors.torque.motor_rated_torque_nm"
    ratio = live / file
    effect = (
        f"every torque command is {1 / ratio:.3g}× weaker than asked and every reported torque {1 / ratio:.3g}× higher "
        "than the drive's"
        if ratio < 1
        else f"every torque command is {ratio:.3g}× stronger than asked (it can saturate 0x6071 at its int16 limit, "
        f"−32768…32767 counts) and every reported torque {ratio:.3g}× lower than the drive's"
    )
    return (
        f"WARN {axis.name}: 0x{index:04X} motor rated torque reads {live} mNm, {ratio:.3g}× the file's "
        f"factors.torque.motor_rated_torque_nm ({file:g} mNm) — {effect}. Either the file is wrong (0x6076 is in mNm: "
        "divide by 1000 for Nm) or the drive's motor setting is. The bench does not stop on this"
    )


def bench_log(model: Path) -> Path:
    """<stem>.bench.log — next to the model."""
    return model.parent / f"{model.stem}.bench.log"


# bus observations


def parse_slave_info(text: str) -> tuple[int | None, int | None]:
    """(serial number, revision number) from `ethercat slaves -v` — None when the line is absent."""

    def number(label):
        m = re.search(rf"^\s*{label}:\s*(0x[0-9a-fA-F]+|\d+)\s*$", text, re.M)
        return int(m[1], 0) if m else None

    return number("Serial number"), number("Revision number")


def upload_failure_is_absence(stderr: str) -> bool:
    """True only for an SDO abort that says the object does not exist.
    IgH returns 1 for an abort and for a device it cannot open alike, so the
    code in the text is the only discriminator."""
    m = re.search(r"abort\w*[^\n]*?(0x[0-9a-fA-F]{8})", stderr, re.I)
    return bool(m) and int(m[1], 16) in ABSENT_ABORTS


@dataclasses.dataclass
class Identity:
    serial_number: int | None = None
    revision_number: int | None = None
    hardware_version: str | None = None
    software_version: str | None = None
    pdo_mapping_digest: str | None = None

    def observes_something(self) -> bool:
        """The loader's rule: serial number 0 is the SII 'not recorded' value (empty),
        a revision number of 0 is a read value, a scanned digest counts."""
        return (
            bool(self.serial_number)
            or self.revision_number is not None
            or self.hardware_version is not None
            or self.software_version is not None
            or self.pdo_mapping_digest is not None
        )


def digests_match_layout(digests: list[str], slaves: tuple[Slave, ...]) -> str:
    """'' when ~/info carries one entry per slave, a sha256 for each scanned
    slave and '' for each declared one — the expectation comes from check."""
    if len(digests) != len(slaves):
        return f"~/info carries {len(digests)} digests for {len(slaves)} slaves"
    for s, (slave, digest) in enumerate(zip(slaves, digests)):
        if slave.scanned and not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
            return f"slave {s} is scanned but ~/info carries {digest!r}"
        if not slave.scanned and digest:
            return f"slave {s} is declared but ~/info carries {digest!r}"
    return ""


# the bench file


class Quoted(str):
    """A text value the bench file writes double-quoted, so the C++ reader
    never takes it for a number or a boolean (`1e3`, `on`)."""


def bench_document(
    check: Check,
    identities: list[Identity],
    items: dict[str, tuple[str, str]],
    conditions: dict[str, str],
) -> dict:
    def text(value):
        return None if value is None else Quoted(value)

    return {
        "schema": "cia402_bench/v1",
        "claims": {
            "config_digest": Quoted(check.config_digest),
            "profile_digest": Quoted(check.profile_digest),
            "implementation_revision": Quoted(check.revision),
            "slaves": [
                {
                    "slave": s,
                    "serial_number": ident.serial_number,
                    "revision_number": ident.revision_number,
                    "hardware_version": text(ident.hardware_version),
                    "software_version": text(ident.software_version),
                    "pdo_mapping_digest": text(ident.pdo_mapping_digest),
                }
                for s, ident in enumerate(identities)
            ],
        },
        "conditions": {k: Quoted(v) for k, v in conditions.items()},
        "items": [
            {"id": i, "result": r, "evidence": Quoted(e)} for i, (r, e) in items.items()
        ],
    }


def dump_bench(document: dict) -> str:
    import yaml

    class Dumper(yaml.SafeDumper):
        pass

    Dumper.add_representer(
        Quoted,
        lambda d, v: d.represent_scalar("tag:yaml.org,2002:str", str(v), style='"'),
    )
    return yaml.dump(
        document, Dumper=Dumper, sort_keys=False, allow_unicode=True, width=10_000
    )


# the observer


class Guard:
    """The bench's observer contract for N axes, rclpy-free: callbacks latch the
    first violation, `guard()` raises it and checks freshness. RLock makes a
    phase change and a release gate atomic relative to callbacks."""

    def __init__(
        self,
        axes: tuple[Axis, ...],
        clock: Callable[[], float] = time.monotonic,
    ):
        self.axes, self.n, self.clock = axes, len(axes), clock
        # The stream controller's vector: one value per csv or cst axis, at rest
        # (0 rad/s, 0 Nm); None while stop-decel streams its ramp.
        self.streamed = sum(a.mode != "csp" for a in axes)
        self.neutral: tuple[float, ...] | None = (0.0,) * self.streamed
        # (RT time, velocities) of every cycle (rt_raw) while stop-decel records,
        # and the cycles rt_raw lost meanwhile.
        self.samples: list | None = None
        self.rt_seq, self.rt_lost = None, 0
        self.mask = (1 << self.n) - 1
        self.lock = threading.RLock()
        self.status = self.joint_states = None
        self.status_t = self.joint_states_t = self.command_t = 0.0
        self.history, self.events, self.commands = [], [], 0
        self.error, self.phase, self.allowed_stops = None, "startup", (0,)
        self.monitor, self.monitor_ns = False, 0
        self.baseline, self.worst = None, [0.0] * self.n
        # The axes stop-decel moves (file order): exempt from the drift abort
        # until the next release; every other axis stays held to the baseline.
        self.moving: frozenset[int] = frozenset()
        self.settled_count = 0
        # axis index -> its idle SDO reading, for each axis the idle read saw in
        # FAULT (a fault already latched before the bench started, e.g. left by
        # the previous exit — whatever caused it): FAULT is tolerated for these
        # axes during startup only.
        self.latched: dict[int, str] = {}
        # Release refusals seen on rt_events, as cause text, in arrival order.
        self.refusals: dict[str, None] = {}
        self.slave_refused = False  # a 0x62 arrived: the slave hint applies

    def latch(self, ok: bool, why: str) -> bool:
        if not ok and self.error is None:
            self.error = f"{self.phase}: {why}"
        return ok

    def settled(self, s, stop: int) -> bool:
        return (
            s.protective_stop == stop
            and not s.operation_enabled
            and s.brake_engaged == self.mask
            and s.brake_operation_pending == 0
            and len(s.statusword) == self.n
            and all(state(w) in RESTING for w in s.statusword)
        )

    def faulted(self) -> list[int]:
        """The axes in FAULT in the latest status."""
        with self.lock:
            s = self.status
            return (
                []
                if s is None
                else [i for i, w in enumerate(s.statusword) if state(w) == "FAULT"]
            )

    def ready(self, s) -> bool:
        return (
            s.protective_stop == 0
            and s.operation_enabled
            and s.brake_engaged == 0
            and s.brake_operation_pending == 0
            and len(s.statusword) == self.n
            and all(state(w) == "OPERATION_ENABLED" for w in s.statusword)
        )

    def on_status(self, m) -> None:
        with self.lock:
            now = self.clock()
            gap = now - self.status_t
            self.latch(
                not self.monitor or gap <= STATUS_AGE_ABORT_S, "status arrival gap"
            )
            self.status, self.status_t = m, now
            if not self.latch(
                len(m.statusword) == self.n, f"statusword length {len(m.statusword)}"
            ):
                return
            self.latch(
                not ((m.brake_engaged | m.brake_operation_pending) & ~self.mask),
                f"mask outside {self.mask:#x}",
            )
            self.latch(
                m.protective_stop in self.allowed_stops,
                f"unexpected protective stop {m.protective_stop}",
            )
            self.latch(
                all(
                    state(w) in QUIET_STATES
                    or (
                        self.phase == "startup"
                        and i in self.latched
                        and state(w) == "FAULT"
                    )
                    for i, w in enumerate(m.statusword)
                ),
                f"fault/unrecognized statusword {list(m.statusword)}",
            )
            self.latch(
                self.phase != "hold" or self.ready(m),
                f"hold lost: not every axis in Operation enabled with brakes released, nothing pending and no protective stop "
                f"(operation_enabled={m.operation_enabled}, protective_stop={m.protective_stop}, brake_engaged={m.brake_engaged}, "
                f"pending={m.brake_operation_pending}, statusword={list(m.statusword)})",
            )
            self.latch(
                self.phase != "braked" or self.settled(m, m.protective_stop),
                "braked state lost",
            )
            fresh = gap <= STATUS_AGE_ABORT_S
            self.settled_count = (
                (self.settled_count + 1 if fresh else 1) if self.settled(m, 0) else 0
            )
            self.history.append(
                (
                    time.monotonic_ns(),
                    (
                        m.protective_stop,
                        m.operation_enabled,
                        m.brake_engaged,
                        m.brake_operation_pending,
                        tuple(m.statusword),
                    ),
                )
            )

    def on_event(self, m) -> None:
        with self.lock:
            self.events.append(m)
            # A release refused at the node's readiness gate: release_brake still
            # answers success, so these events are the only cause the bench can name.
            if m.event_type in (
                RtEvent.SLAVE_NOT_OPERATIONAL,
                RtEvent.MODES_OF_OPERATION_MISMATCH,
            ):
                from plem.drive.bus import describe_refusal

                axis = (
                    self.axes[m.joint_id].name
                    if m.joint_id < self.n
                    else f"axis {m.joint_id}"
                )
                if m.event_type == RtEvent.SLAVE_NOT_OPERATIONAL:
                    self.slave_refused = True
                self.refusals[describe_refusal(m, axis)] = None
            self.latch(
                m.event_type != RtEvent.COMMAND_STALE
                or DriveStatus.PROTECTIVE_STOP_SOURCE_EXTERNAL in self.allowed_stops,
                "unexpected command stale event",
            )
            self.latch(
                m.event_type != RtEvent.OPERATION_ENABLED_LOST, "drive enable/mode lost"
            )
            self.latch(
                not (
                    m.event_type == RtEvent.WKC_MISMATCH
                    and self.monitor_ns
                    and m.monotonic_ns >= self.monitor_ns
                ),
                "WKC mismatch after startup",
            )

    def refusal_note(self) -> str:
        """The release refusals rt_events named, for a timeout message ("" if none)."""
        return self._note(with_idle=False)

    def start_note(self, slaves_not_op: str | None = None) -> str:
        """For the start timeout: the release refusals, then the slaves the
        master reports outside OP (`slaves_not_op`, EthercatCli.slaves_not_in_op — "" =
        every file slave in OP). Without that answer (None), a statusword of 0
        in the latest status is named as one possible cause (a drive still
        initializing, or a slave that sends no process data — PREOP)."""
        return self._note(with_idle=slaves_not_op is None, slaves_not_op=slaves_not_op)

    def _note(self, with_idle: bool, slaves_not_op: str | None = None) -> str:
        with self.lock:
            causes, hint = list(self.refusals), self.slave_refused
            idle = (
                with_idle
                and self.status is not None
                and any(w == 0 for w in self.status.statusword)
            )
        parts = ["rt_events: " + "; ".join(causes)] if causes else []
        if slaves_not_op:
            parts.append("slaves not in OP: " + slaves_not_op)
            hint = True
        if idle:
            parts.append(
                "a statusword of 0: the drive may still be initializing (Not ready to switch on), "
                "or a slave is not exchanging process data (not in SAFEOP/OP)"
            )
            hint = True
        note = " — " + "; ".join(parts) if parts else ""
        return note + (". " + SLAVE_HINT if hint else "")

    def on_rt_raw(self, m) -> None:
        with self.lock:
            if self.samples is None:
                self.rt_seq = None
                return
            n = m.axis_count
            if not self.latch(
                n == self.n, f"rt_raw carries {n} axes, the file {self.n}"
            ):
                return
            for i in range(m.batch_size):
                if self.rt_seq is not None:
                    self.rt_lost += max(0, m.sequence[i] - self.rt_seq - 1)
                self.rt_seq = m.sequence[i]
                self.samples.append(
                    (
                        m.monotonic_ns[i] * 1e-9,
                        tuple(m.actual_velocity[i * n : (i + 1) * n]),
                    )
                )

    def on_joint_states(self, m) -> None:
        """The joint state broadcaster's sample, kept in file order (it
        publishes the joints in an order of its own)."""
        with self.lock:
            now = self.clock()
            self.latch(
                not self.monitor
                or now - self.joint_states_t <= JOINT_STATES_AGE_ABORT_S,
                "joint_states arrival gap",
            )
            self.joint_states_t = now
            names = tuple(a.name for a in self.axes)
            if not self.latch(
                sorted(m.name) == sorted(names)
                and all(len(v) == self.n for v in (m.position, m.velocity, m.effort)),
                f"joint identity/length: {list(m.name)}",
            ):
                return
            at = [list(m.name).index(name) for name in names]
            m = self.joint_states = SimpleNamespace(
                name=names,
                position=tuple(m.position[i] for i in at),
                velocity=tuple(m.velocity[i] for i in at),
                effort=tuple(m.effort[i] for i in at),
            )
            self.latch(
                all(
                    math.isfinite(v)
                    for seq in (m.position, m.velocity, m.effort)
                    for v in seq
                ),
                "nonfinite joint sample",
            )
            self.latch(
                all(
                    a.lo <= q <= a.hi and abs(v) <= a.vmax
                    for q, v, a in zip(m.position, m.velocity, self.axes)
                ),
                f"position/velocity outside the declared limits: q={list(m.position)} v={list(m.velocity)}",
            )
            if self.baseline is not None:
                self.worst = [
                    w if i in self.moving else max(w, abs(q - q0))
                    for i, (w, q, q0) in enumerate(
                        zip(self.worst, m.position, self.baseline)
                    )
                ]
                self.latch(
                    max(self.worst) <= DRIFT_ABORT_RAD,
                    f"drift {self.worst} rad > {DRIFT_ABORT_RAD} — a released axis moved while held at "
                    "rest; engaged",
                )

    def on_command(self, m) -> None:
        with self.lock:
            self.latch(len(m.data) == self.streamed, f"command length {len(m.data)}")
            self.latch(
                self.neutral is None
                or tuple(m.data) == self.neutral,  # NaN is never equal
                f"command {list(m.data)} is not the neutral {list(self.neutral or ())}",
            )
            if len(m.data) == self.streamed:
                self.commands += 1
                self.command_t = self.clock()

    def guard(self) -> None:
        with self.lock:
            if self.monitor:
                now = self.clock()
                self.latch(
                    now - self.status_t <= STATUS_AGE_ABORT_S, "status observation lost"
                )
                self.latch(
                    now - self.joint_states_t <= JOINT_STATES_AGE_ABORT_S,
                    "joint observation lost",
                )
            if self.error:
                raise BenchError(self.error)

    def phase_to(self, phase: str, stops: tuple[int, ...] = (0,)) -> None:
        with self.lock:
            self.guard()
            if phase in ("hold", "braked"):
                ok = self.status is not None and (
                    self.ready(self.status)
                    if phase == "hold"
                    else self.settled(self.status, 0)
                )
                self.latch(ok, f"current status changed before entering {phase}")
                self.guard()
            self.phase, self.allowed_stops = phase, stops

    def release_gate(self) -> None:
        """Every release: settled, inside the limits and, on a bus with a csv
        or cst axis, with a fresh stream (a cst axis has no command without it;
        the release latches csp and csv). The drift baseline is taken here and
        kept through the next engage."""
        self.guard()
        if not (self.status is not None and self.settled(self.status, 0)):
            raise BenchError(
                "release: the axes are not settled (Switch on disabled or Ready to switch on, engaged, nothing pending, no protective stop)"
            )
        q = tuple(self.joint_states.position)
        if not all(a.lo <= x <= a.hi for x, a in zip(q, self.axes)):
            raise BenchError(f"release: a position is outside the declared limits: {q}")
        if self.streamed and self.clock() - self.command_t > JOINT_STATES_AGE_ABORT_S:
            raise BenchError("release: the stream is not fresh")
        self.baseline, self.worst, self.moving = q, [0.0] * self.n, frozenset()
        self.phase_to("release")


# stop-decel


@dataclasses.dataclass(frozen=True)
class Move:
    index: int  # the axis, in file order
    direction: int  # +1 or -1
    distance: float  # rad
    cruise: float  # rad/s


def stop_moves(axes: tuple[Axis, ...], q) -> list[Move]:
    """stop-decel's move of each csp and csv axis from `q`: toward its roomier
    side, STOP_DISTANCE_RAD or nine tenths of that room, whichever is shorter;
    cruise = min(CRUISE_FRACTION x velocity_cap_rad_s, sqrt(a x distance / 2),
    the v with v^2 / a + v x MIN_CRUISE_S = distance) — accelerating and
    stopping at the limit take at most half the distance, and the cruise lasts
    at least MIN_CRUISE_S. An axis without MIN_STOP_DISTANCE_RAD of room is a
    refusal."""
    moves = []
    for i, (a, x) in enumerate(zip(axes, q)):
        if a.mode == "cst":
            continue
        up, down = a.hi - x, x - a.lo
        distance = min(STOP_DISTANCE_RAD, 0.9 * max(up, down))
        if distance < MIN_STOP_DISTANCE_RAD:
            raise BenchError(
                f"stop-decel: {a.name} at {x:.4f} rad has no room to move between its limits "
                f"[{a.lo:g}, {a.hi:g}] (needs {MIN_STOP_DISTANCE_RAD:g} rad on one side) — move it "
                "off its limits by hand and re-run"
            )
        held = (
            a.accel
            / 2
            * (math.sqrt(MIN_CRUISE_S**2 + 4 * distance / a.accel) - MIN_CRUISE_S)
        )
        cruise = min(CRUISE_FRACTION * a.vcap, math.sqrt(a.accel * distance / 2), held)
        moves.append(Move(i, 1 if up >= down else -1, distance, cruise))
    return moves


def ramp_velocity(move: Move, accel: float, t: float) -> float:
    """A csv axis's stop-decel command t s after its ramp starts: up to cruise
    at `accel`, cruise, back down to 0 — `move.distance` in all, so the axis
    stays inside its room even if the stop came late."""
    v = move.cruise
    rise = v / accel
    hold = move.distance / v - rise
    if t <= 0:
        speed = 0.0
    elif t < rise:
        speed = accel * t
    elif t < rise + hold:
        speed = v
    else:
        speed = max(0.0, v - accel * (t - rise - hold))
    return move.direction * speed


def stop_times(phases: list[tuple[float, float]]) -> tuple[float, float]:
    """From the start, when stop-decel asks for the stop (the middle of the
    shortest cruise) and when its wait for every axis at cruise ends (three
    quarters of it — the request still lands inside the cruise), given each
    (time to cruise, time at cruise); every cruise starts at the latest rise."""
    begin = max(rise for rise, _ in phases)
    shortest = min(hold for _, hold in phases)
    return begin + shortest / 2, begin + 0.75 * shortest


def stop_settle_s(moves: list[Move], axes: tuple[Axis, ...]) -> float:
    """How long stop-decel's requested stop may take to settle: the slowest
    axis's time from its cruise to rest at its limit, plus COMMAND_WAIT_S
    (the settle every other request gets)."""
    return max(m.cruise / axes[m.index].accel for m in moves) + COMMAND_WAIT_S


class CspMove:
    """stop-decel's csp move (`move`, plem.drive.Bus.move_to) on a thread of
    its own, started after `delay` s. The requested stop ends it with an
    error, so an error is the move failing only when it came before the
    stop request."""

    def __init__(self, move: Callable[[], None], delay: float):
        self.error: tuple[float, Exception] | None = None

        def go():
            try:
                move()
            except Exception as e:
                self.error = (time.monotonic(), e)

        self.thread = threading.Timer(delay, go)
        self.thread.daemon = True
        self.thread.start()

    def check(self, before: float = math.inf) -> None:
        """A refusal when the move failed before `before`."""
        if self.error is not None and self.error[0] < before:
            e = self.error[1]
            raise BenchError(
                f"stop-decel: the csp move failed: {type(e).__name__}: {e}"
            )

    def end(self, t_request: float) -> str:
        """Once the move has ended (Bench.end_move waits for it): the error the
        stop ended the move with ('' = none); a refusal when the move failed
        before the stop request."""
        self.thread.join()
        self.check(t_request)
        return (
            ""
            if self.error is None
            else f"stop-decel: the stop ended the csp move ({self.error[1]})"
        )


def decelerations(samples, t_stop: float, moves: list[Move]):
    """Per move: (|velocity| in the last sample at or before `t_stop`, seconds
    from `t_stop` to the first sample at rest — None if none is), from
    (t, velocities in file order) samples. At rest: |v| <= REST_RAD_S or 2 % of
    the speed at the stop."""
    before = [v for t, v in samples if t <= t_stop][-1]
    out = []
    for m in moves:
        speed = abs(before[m.index])
        rest = max(REST_RAD_S, 0.02 * speed)
        when = next(
            (t - t_stop for t, v in samples if t > t_stop and abs(v[m.index]) <= rest),
            None,
        )
        out.append((speed, when))
    return out


def require(ok: bool, what: str) -> None:
    if not ok:
        raise CheckFailed(what)


def judge_stale_engage(events, t_kill: int, check: Check, need=require) -> str:
    """stale-engage from the events after the stream was killed at `t_kill`
    (monotonic ns): the first 0x54 at or after it, its RT command age within
    the command stale timeout (in s) + STALE_MARGIN_S, and a 0x42 within two
    cycles of it. `need` takes each check in order and raises on the first that fails."""
    stale = next(
        (
            e
            for e in events
            if e.event_type == RtEvent.COMMAND_STALE and e.monotonic_ns >= t_kill
        ),
        None,
    )
    need(stale is not None, "stale: 0x54 after the stream was killed")
    stale_bar = check.command_stale_timeout_ns * 1e-9 + STALE_MARGIN_S
    need(
        0 <= stale.value <= stale_bar,
        f"stale: RT command age {stale.value:.6f} s <= {stale_bar:g} s",
    )
    bar = 2 * check.cycle_time_ns
    engages = sorted(
        e.monotonic_ns - stale.monotonic_ns
        for e in events
        if e.event_type == RtEvent.BRAKE_ENGAGE and e.monotonic_ns >= stale.monotonic_ns
    )
    need(
        bool(engages) and engages[0] <= bar,
        f"stale: 0x54 -> first 0x42 {engages[:1]} ns <= {bar} ns",
    )
    return (
        f"stream killed -> 0x54 at age {stale.value:.3f} s -> EXTERNAL protective stop -> 0x42 in {engages[0]} ns "
        f"(<= 2 cycles) -> every axis settled"
    )


def stop_decel_na(modes) -> str:
    """Why check left stop-decel out for a bus of these axis modes."""
    if all(m == "cst" for m in modes):
        return "no csp or csv axis — the bench moves no torque axis"
    return "a cst axis makes this bus's stop an immediate disable, not the ramp stop-decel measures"


def judge_stop_decel(
    axes: tuple[Axis, ...],
    moves: list[Move],
    cruise: dict,
    t_request: float,
    events,
    samples,
    lost: int,
    need=require,
) -> str:
    """stop-decel from what was recorded after the stop requested at
    `t_request`: no 0x4F among `events`, rt_raw `samples` (t, velocities in file
    order) from the request on with no cycle `lost`, and each moving axis at
    cruise at the request, then at rest at no more than DECEL_MARGIN x its
    acceleration on average (decelerations). `need` takes each check in order
    and raises on the first that fails."""
    need(
        not any(e.event_type == RtEvent.RAMP_STOP_TIMEOUT for e in events),
        "stop-decel: no stop timeout (0x4F) — every axis confirmed at rest",
    )
    need(
        any(t <= t_request for t, _ in samples),
        "stop-decel: rt_raw delivered the cycle at the stop request (is rt_raw flowing?)",
    )
    # A lost batch would hide the first cycle at rest and pass a stop too sharp.
    need(lost == 0, f"stop-decel: rt_raw lost no cycle while recording ({lost} lost)")
    lines = []
    for m, (speed, seconds_to_rest) in zip(
        moves, decelerations(samples, t_request, moves)
    ):
        a = axes[m.index]
        need(
            seconds_to_rest is not None,
            f"stop-decel: {a.name} came to rest within the recorded rt_raw cycles",
        )
        need(
            speed >= 0.9 * cruise[m.index],
            f"stop-decel: {a.name} at cruise at the stop ({speed:.4f} of {cruise[m.index]:.4f} rad/s)",
        )
        decel = speed / max(seconds_to_rest, 1e-9)
        bar = DECEL_MARGIN * a.accel
        need(
            decel <= bar,
            f"stop-decel: {a.name} {speed:.3f} rad/s -> rest in {seconds_to_rest:.3f} s = "
            f"{decel:.3f} rad/s^2 <= {bar:.3f} ({DECEL_MARGIN:g} x limits.acceleration_rad_s2)",
        )
        lines.append(
            f"{a.name} {speed:.3f} rad/s -> rest in {seconds_to_rest:.3f} s ({decel:.3f} <= {bar:.3f} rad/s^2)"
        )
    return (
        "protective_stop at cruise: "
        + "; ".join(lines)
        + "; no 0x4F; every axis engaged"
    )


# hardware side


def log_line(sink, msg: str) -> None:
    """Best effort: a line that cannot be written (full disk, closed pipe) never
    stops the teardown or the gates — the record and the safety path are separate."""
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    with contextlib.suppress(OSError, ValueError):
        print(line, flush=True)
    if sink is not None:
        with contextlib.suppress(OSError, ValueError):
            sink.write(line + "\n")
            sink.flush()


def string_payload(stdout: bytes) -> tuple[str | None, str]:
    """An `ethercat upload -t string` answer → (the value to record, note).
    IgH prints the payload bytes and one newline; the bus later compares the
    payload byte for byte, so nothing else is trimmed. A payload a record
    cannot carry (not UTF-8, a control byte such as NUL padding, over 31
    bytes) is recorded null — the bus then does not compare it (schema §6)."""
    payload = stdout[:-1] if stdout.endswith(b"\n") else stdout
    try:
        text = payload.decode("utf-8")
    except UnicodeDecodeError:
        return None, f"not UTF-8 ({payload.hex()}) — recorded null"
    if len(payload) > 31 or any(ord(c) < 0x20 or ord(c) == 0x7F for c in text):
        return (
            None,
            f"{payload!r} cannot be carried by a record (control byte or over 31 bytes) — recorded null",
        )
    return text, ""


class EthercatCli:
    """The IgH `ethercat` CLI on one master — read-only uses, bounded."""

    def __init__(self, master: int):
        self.master = master
        self.cli = shutil.which("ethercat") or (
            "/opt/etherlab/bin/ethercat"
            if os.access("/opt/etherlab/bin/ethercat", os.X_OK)
            else None
        )
        if self.cli is None:
            raise BenchError(
                "the IgH ethercat CLI is not installed (looked in PATH and /opt/etherlab/bin) — install "
                "the EtherCAT master with menu 3 of the RT setup wizard (setup/rt/setup.sh)"
            )

    def run(self, *args: str, binary: bool = False) -> subprocess.CompletedProcess:
        """`ethercat <verb> -m N <rest>`."""
        try:
            return subprocess.run(
                [self.cli, args[0], "-m", str(self.master), *args[1:]],
                capture_output=True,
                text=not binary,
                timeout=CLI_TIMEOUT_S,
            )
        except subprocess.TimeoutExpired:
            raise BenchError(
                f"`ethercat {' '.join(args)}` on master {self.master} did not answer in "
                f"{CLI_TIMEOUT_S:g} s"
            ) from None

    def check_access(self) -> None:
        r = self.run("master")
        if r.returncode != 0:
            raise BenchError(
                f"cannot read EtherCAT master {self.master}: {(r.stderr or r.stdout).strip()} — check "
                "the master index and that this user may open the master device: `id -nG` must list "
                "the group of the udev rule /etc/udev/rules.d/99-ethercat.rules (the RT setup "
                "wizard's default: realtime); if not, `sudo usermod -aG realtime $USER`, then log "
                "in again (docs/install.md §3). Run nothing as root to get around it"
            )

    def idle(self) -> bool:
        return "Phase: Idle" in self.run("master").stdout

    def upload(
        self, position: int, index: int, typ: str, subindex: int = 0
    ) -> bytes | None:
        """stdout bytes, or None when the drive has no such object; anything else aborts."""
        args = (
            "upload",
            "-p",
            str(position),
            "-t",
            typ,
            f"0x{index:04x}",
            str(subindex),
        )
        for attempt, delay in enumerate((*UPLOAD_RETRY_DELAYS_S, None), 1):
            r = self.run(*args, binary=True)
            if r.returncode == 0:
                return r.stdout
            err = r.stderr.decode("utf-8", "replace")
            if upload_failure_is_absence(err):
                return None
            if delay is None or UPLOAD_NO_ABORT not in err:
                break
            time.sleep(delay)
        fix = (
            "start again once the bus has been idle for a while; if it keeps failing, "
            "check the drive's mailbox (CoE) and the cabling"
            if UPLOAD_NO_ABORT in err
            else "check permissions and the bus"
        )
        raise BenchError(
            f"SDO upload 0x{index:04x} from the slave at ring position {position} failed: "
            f"{err.strip()} (attempts: {attempt}) — not an absent object; {fix}"
        )

    def statusword(self, axis: Axis) -> int:
        raw = self.upload(axis.position, axis.sdo(0x6041), "uint16")
        if raw is None:
            raise BenchError(
                f"{axis.name}: the drive has no 0x{axis.sdo(0x6041):04x} statusword object"
            )
        return int(raw.split()[-1], 0)

    def identity(self, position: int) -> tuple[Identity, list[str]]:
        r = self.run("slaves", "-p", str(position), "-v")
        if r.returncode != 0:
            raise BenchError(
                f"cannot read the slave at ring position {position}: {r.stderr.strip()}"
            )
        serial_number, revision = parse_slave_info(r.stdout)
        values, notes = [], []
        for index in (0x1009, 0x100A):
            raw = self.upload(position, index, "string")
            value, note = (None, "absent") if raw is None else string_payload(raw)
            values.append(value)
            if note:
                notes.append(f"0x{index:04X}: {note}")
        return Identity(
            serial_number=serial_number,
            revision_number=revision,
            hardware_version=values[0],
            software_version=values[1],
        ), notes

    def slaves_not_in_op(self, positions: list[int]) -> str | None:
        """`ethercat slaves`: the given ring positions whose AL state is not OP,
        as "0 PREOP, 1 SAFEOP" ("" = all in OP; a position the master does not
        list reads "absent"). None when the CLI gives no answer — then nothing is
        known, and the caller falls back to what the node reported."""
        try:
            r = self.run("slaves")
        except BenchError:
            return None
        if r.returncode != 0:
            return None
        states = parse_slave_states(r.stdout)
        return ", ".join(
            f"{p} {states.get(p, 'absent')}" for p in positions if states.get(p) != "OP"
        )

    def version(self) -> str:
        r = subprocess.run(
            [self.cli, "version"], capture_output=True, text=True, timeout=CLI_TIMEOUT_S
        )
        return (r.stdout.splitlines() or ["IgH (version unknown)"])[0]


def parse_slave_states(text: str) -> dict[int, str]:
    """`ethercat slaves` lines (`<pos>  <alias>:<rel>  <state>  <+|E>  <name>`)
    → {ring position: AL state text, e.g. "PREOP" or "SAFEOP+ERROR"}."""
    states = {}
    for line in text.splitlines():
        m = re.match(r"\s*(\d+)\s+\d+:\d+\s+(\S+)\s+[+E]\s", line)
        if m:
            states[int(m.group(1))] = m.group(2)
    return states


def run_argv(args, tool: str) -> list[str]:
    """`plem drive run --unapproved` of the benched file: the bench is the operator at the drives.
    Verbose: its output is the bench log, kept for diagnosis, not a terminal."""
    return [
        "env",
        f"{BENCH_NAMESPACE_ENV}={args.namespace}",
        tool,
        "run",
        str(args.model),
        "--master",
        str(args.master),
        "--unapproved",
        "--verbose",
        *(["--rt-cpu", str(args.rt_cpu)] if args.rt_cpu is not None else []),
    ]


def bus_users() -> list[int]:
    # Known bus clients by executable; any other application holding the
    # master is caught by the master-state check (ethercat.idle()) beside this.
    # Not seen: a stack still starting (launched, its control node not yet
    # spawned) — the master reservation then refuses whichever side comes second.
    return own_pids(lambda exe, _: os.path.basename(exe) == "ros2_control_node")


class DriveProcess:
    """The run chain: `plem drive run` execs the controller manager in its own
    place, so this child's pid is the controller manager's and SIGINT reaches
    it directly: its requested stop, then the bus shuts down. The kernel also
    sends it SIGINT if the runner dies first; that is only a fallback."""

    def __init__(self, sink, argv: list[str]):
        # Its output goes to the bench log from here on: stop_lines() reads it.
        with contextlib.suppress(AttributeError, OSError, ValueError):
            sink.flush()
        try:
            self.output = (sink.name, sink.tell())
        except (AttributeError, OSError, ValueError):
            self.output = None
        self.p = subprocess.Popen(
            ["setpriv", "--pdeathsig", "INT", *argv],
            stdout=sink,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def stop_lines(self) -> tuple[str, str, str]:
        """The chain's CONFIRMED_BRAKES, UNCONFIRMED_BRAKES and
        NEVER_RELEASED_BRAKES lines in its output ('' = none) — read after end(). Output that cannot be read is a
        refusal: the brakes are then not confirmed."""
        try:
            path, start = self.output
            with open(path, "rb") as f:
                f.seek(start)
                lines = f.read().decode("utf-8", "replace").splitlines()
        except (TypeError, OSError) as e:
            raise BenchError(
                f"the run chain's output cannot be read back ({e}) — its brakes are not confirmed"
            ) from None

        def first(marker: str) -> str:
            return next((line.strip() for line in lines if marker in line), "")

        return (
            first(CONFIRMED_BRAKES),
            first(UNCONFIRMED_BRAKES),
            first(NEVER_RELEASED_BRAKES),
        )

    def check_sigint(self) -> None:
        ignored = int(
            re.search(
                r"SigIgn:\s*(\w+)", Path(f"/proc/{self.p.pid}/status").read_text()
            )[1],
            16,
        )
        if ignored & (1 << (signal.SIGINT - 1)):
            raise BenchError(
                "the run chain started with SIGINT ignored, so the bench could not stop it with Ctrl-C "
                "(SIGINT); start the bench from a shell that does not ignore SIGINT"
            )

    def end(self) -> int:
        """SIGINT and wait; a chain still alive after NODE_STOP_S is killed and that is a failure."""
        if self.p.poll() is not None:
            return self.p.returncode
        self.p.send_signal(signal.SIGINT)
        try:
            return self.p.wait(timeout=NODE_STOP_S)
        except subprocess.TimeoutExpired:
            self.p.kill()
            self.p.wait()
            raise BenchError(
                f"the controller manager was still alive {NODE_STOP_S:g} s after SIGINT — killed; its brakes "
                "were NOT confirmed engaged, check the drive"
            ) from None


class Publisher:
    """The user's stream at rest: `ros2 topic pub`, a process of its own so it can be killed alone."""

    def __init__(self, topic: str, values: tuple[float, ...]):
        import yaml

        # `ros2 topic pub` parses the payload with yaml.safe_load, whose YAML 1.1
        # reads `1e-05` (repr, no dot) as a string: PyYAML's own float spelling.
        dumper = yaml.SafeDumper(None)
        data = ", ".join(dumper.represent_float(float(v)).value for v in values)
        self.p = subprocess.Popen(
            [
                "setpriv",
                "--pdeathsig",
                "KILL",
                "ros2",
                "topic",
                "pub",
                "-r",
                str(PUBLISH_HZ),
                topic,
                "std_msgs/msg/Float64MultiArray",
                "{data: [" + data + "]}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    def end(self) -> int:
        self.p.kill()
        t = time.monotonic_ns()
        self.p.wait()
        return t


class Ramp:
    """stop-decel's stream: `publish(values())` (plem.drive Bus.stream) at
    PUBLISH_HZ from a thread of this process; end() stops it. A refused
    command is logged and ends the publishing (the bus's command watch then
    stops it); end() raises it."""

    def __init__(self, publish, values, log):
        self.stopped = threading.Event()
        self.error: Exception | None = None

        def loop():
            while not self.stopped.wait(1 / PUBLISH_HZ):
                try:
                    publish(values())
                except Exception as e:
                    self.error = e
                    log(f"stop-decel: the ramp stream stopped: {e}")
                    return

        self.thread = threading.Thread(target=loop, daemon=True)
        self.thread.start()

    def end(self) -> int:
        self.stopped.set()
        self.thread.join()
        t = time.monotonic_ns()
        if self.error is not None:
            raise BenchError(f"stop-decel: the ramp stream stopped: {self.error}")
        return t


STREAM_START_S = 15.0
STREAM_READY = (
    20  # new commands at rest the observer must receive before a stream counts as live
)


class Bench:
    """The N-axis bench: every axis runs the same order and gates.
    Every child and ROS resource is owned as soon as its constructor returns, so
    cleanup() reaches it whatever raised after that. A signal inside the
    constructor can leave one unowned: the bench is void anyway, and a node's
    pdeathsig ends it when the runner exits."""

    def __init__(self, args, check: Check, ethercat: EthercatCli | None, sink):
        self.args, self.check, self.ethercat, self.sink = args, check, ethercat, sink
        self.axes = check.axes
        self.guard = Guard(self.axes)
        self.all_cst = all(a.mode == "cst" for a in self.axes)
        self.items: dict[str, tuple[str, str]] = {}
        self.notes: list[str] = []  # the bench file's conditions.notes
        self.node: DriveProcess | None = None
        self.pub: Publisher | Ramp | None = None
        self.info = None
        self.ros: dict = {}
        self.client = self.request = None
        self.publisher = Publisher  # the stream factory (a test seam)
        # plem.drive.Bus: releases, stops, stop-decel's motion (a test seam)
        self.drive = None
        self.interrupted: list[int] = []  # signals held during cleanup

    def log(self, msg: str) -> None:
        log_line(self.sink, msg)

    def need(self, ok: bool, what: str) -> None:
        self.log(f"{'PASS' if ok else 'FAIL'} {what}")
        require(ok, what)

    def wait_until(self, pred, deadline: float, step: float = 0.01) -> bool:
        """True only if `pred` held and the absolute deadline had not passed by the
        time that was known (a state seen only after the deadline counts as missed)."""
        while True:
            self.guard.guard()
            held = pred()
            if time.monotonic() >= deadline:
                return False
            if held:
                return True
            time.sleep(step)

    def wait_for(self, pred, timeout: float, step: float = 0.01) -> bool:
        return self.wait_until(pred, time.monotonic() + timeout, step)

    def status_is(self, pred):
        return lambda: self.guard.status is not None and pred(self.guard.status)

    # ROS side
    def start_ros(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from plem_drive_msgs.msg import DriveInfo, DriveStatus
        from plem_rt_msgs.msg import RtRawBatch
        from plem_rt_msgs.srv import SetControlCommand
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        ns = self.args.namespace

        # The runner's own SIGINT/SIGTERM handling decides the shutdown order;
        # rclpy's handlers would shut the context down under the executor.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.ros["rclpy"] = rclpy
        node = self.ros["node"] = rclpy.create_node("plem_drive_bench")
        base = f"{ns}/plem_drive"  # the drive bus's own surface, beside the controllers
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        retained = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        g = self.guard
        node.create_subscription(DriveStatus, f"{base}/status", g.on_status, latched)
        node.create_subscription(RtEvent, f"{base}/rt_events", g.on_event, retained)
        node.create_subscription(JointState, f"{ns}/joint_states", g.on_joint_states, 1)
        node.create_subscription(
            RtRawBatch,
            f"{ns}/rt_raw",
            g.on_rt_raw,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        if g.streamed:
            node.create_subscription(
                Float64MultiArray,
                self.stream_topic(),
                g.on_command,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        node.create_subscription(
            DriveInfo, f"{base}/info", lambda m: setattr(self, "info", m), latched
        )
        self.client = node.create_client(SetControlCommand, f"{base}/control_command")
        self.request = SetControlCommand.Request
        executor = self.ros["executor"] = SingleThreadedExecutor()
        executor.add_node(node)
        closing = self.ros["closing"] = threading.Event()

        def spin():
            try:
                executor.spin()
            except (
                BaseException
            ) as e:  # a dead observer must stop the bench, never go quiet
                with g.lock:
                    g.latch(False, f"observer exception {type(e).__name__}: {e}")
            finally:
                if not closing.is_set():
                    with g.lock:
                        g.latch(False, "observer executor stopped")

        thread = self.ros["thread"] = threading.Thread(target=spin, daemon=True)
        thread.start()

    def stop_ros(self) -> bool:
        """executor.shutdown → spin thread join → node destroy → context shutdown;
        each step tolerates a partial start and runs even if an earlier one failed.
        True only if the observer is confirmed stopped (its latch is then final)."""
        r = self.ros
        if "closing" in r:
            r["closing"].set()

        def joined(t):
            t.join(timeout=2.0)
            return not t.is_alive()

        steps = (
            ("executor", lambda e: e.shutdown(timeout_sec=2.0)),
            ("thread", joined),
            ("node", lambda n: n.destroy_node()),
            ("rclpy", lambda rc: rc.try_shutdown()),
        )
        ok = True
        for key, step in steps:
            if key in r:
                ok &= self.teardown(f"ROS {key}", lambda: step(r[key]))
        self.ros = {}
        return ok

    def teardown(self, what: str, step) -> bool:
        """One cleanup step: a raise or a False fails it without stopping the steps after it."""
        try:
            done = step()
        except Exception as e:
            self.log(f"cleanup: {what}: {type(e).__name__}: {e}")
            return False
        if done is False:
            self.log(f"cleanup: {what} did not complete")
        return done is not False

    def call(self, **fields):
        """One raw control_command request — what plem.drive has no call for:
        reset_error, and the second release_brake that must be a no-op
        (Bus.release would restart the trajectory controller first)."""
        self.need(
            self.wait_for(self.client.service_is_ready, 5.0),
            "control_command service is up",
        )
        with self.guard.lock:
            self.guard.guard()
            deadline = time.monotonic() + 5.0
            future = self.client.call_async(self.request(**fields))
        self.need(self.wait_until(future.done, deadline), f"{fields} answered")
        r = future.result()
        self.log(f'service {fields} -> success={r.success} message="{r.message}"')
        return r

    def through_drive(self, tag, request, target, what, wait=COMMAND_WAIT_S) -> None:
        """One plem.drive request (Bus.release, Bus.stop) on a thread of its
        own, so the observer keeps its watch while the client waits; one
        deadline, `wait` s from the request, covers its answer and the state
        the bench checks (`target` on the observer's status) — the client's
        own waits are longer (start_drive), so the bench's deadline decides."""
        self.guard.guard()
        deadline = time.monotonic() + wait
        error: list[Exception] = []

        def send():
            try:
                request()
            except Exception as e:  # DriveError: the bus's reason
                error.append(e)

        worker = threading.Thread(target=send, daemon=True)
        worker.start()
        settled = self.wait_until(
            lambda: not worker.is_alive() and (bool(error) or self.status_is(target)()),
            deadline,
        )
        if error:
            raise BenchError(f"{tag}: {error[0]}{self.guard.refusal_note()}")
        self.need(
            settled,
            f"{tag}: {what} within {wait:g} s of the request"
            + self.guard.refusal_note(),
        )

    def stop(self, tag: str, wait: float = COMMAND_WAIT_S) -> None:
        """Bus.stop: protective_stop, cancelling a csp move after it."""
        g = self.guard
        self.through_drive(
            tag,
            self.drive.stop,
            lambda s: g.settled(s, 0),
            "protective_stop: every axis at rest"
            + (", 0 torque" if self.all_cst else "")
            + ", disabled, engaged, nothing pending, protective stop 0",
            wait,
        )

    def stream_topic(self) -> str:
        return f"{self.args.namespace}/stream_controller/commands"

    def start_stream(self, publisher=None) -> None:
        """A stream (`publisher(topic, values at rest)`, the bench's `publisher`
        by default) starting at rest (csv 0 rad/s, cst 0 Nm, one value per
        stream axis) counts as live only once the observer has received
        STREAM_READY new commands from it (a reset before that races the bus's
        stale re-assertion)."""
        values = (0.0,) * self.guard.streamed
        with self.guard.lock:
            self.guard.neutral = values
            before = self.guard.commands
        self.pub = (publisher or self.publisher)(self.stream_topic(), values)

        def error():  # a Ramp whose client refused (Ramp.error)
            return getattr(self.pub, "error", None)

        live = self.wait_for(
            lambda: error() is not None or self.guard.commands - before >= STREAM_READY,
            STREAM_START_S,
        )
        self.need(
            live and error() is None,
            f"stream at rest {list(values)} at {PUBLISH_HZ} Hz: {STREAM_READY} new commands received"
            + (f" — the stream stopped: {error()}" if error() is not None else ""),
        )

    def end_stream(self) -> int:
        """Ends the stream; the time it ended (monotonic ns)."""
        pub, self.pub = self.pub, None
        return pub.end()

    def start_node(self, argv: list[str]) -> None:
        self.node = DriveProcess(self.sink, argv)  # owned before anything can raise
        self.node.check_sigint()

    def end_node(self) -> None:
        """SIGINT to the run chain: it must exit 0 and its stop must have
        confirmed every brake engaged — the CONFIRMED_BRAKES line present and
        no UNCONFIRMED_BRAKES line in its output (the chain exits 0 either
        way)."""
        rc = self.node.end()  # a kill raises; the node stays owned
        node, self.node = self.node, None
        self.need(rc == 0, f"stop: controller manager RC {rc}")
        confirmed, unconfirmed, never_released = node.stop_lines()
        self.need(
            not unconfirmed,
            "stop: the hardware plugin logged no unconfirmed-brakes line"
            + (f" — it logged: {unconfirmed}" if unconfirmed else ""),
        )
        self.need(
            bool(confirmed),
            "stop: the hardware plugin reported its brakes confirmed engaged"
            + (
                ""
                if confirmed
                # Brakes that stayed engaged: no released brake was seen to engage.
                else f" — it reported instead: {never_released}"
                if never_released
                else f" — it did not report its brakes confirmed engaged ('{CONFIRMED_BRAKES}' "
                "is not in the run's output): the plugin was not stopped, or its line was lost, so "
                "the brake state at the stop is unknown"
            ),
        )

    def start_drive(self) -> None:
        """plem.drive.Bus, the user's client: every release (on a bus with a
        csp axis it restarts the trajectory controller first) and stop, and
        stop-decel's motion — the csp move and the csv ramp stream. Its own
        waits outlast the bench's longest deadline (stop-decel's settle: a
        cruise is at most CRUISE_FRACTION x the cap), so the bench's deadline
        is the one that fails a step."""
        from plem.drive import Bus

        slowest = max(
            (CRUISE_FRACTION * a.vcap / a.accel for a in self.axes if a.accel),
            default=0.0,
        )
        self.drive = Bus(self.args.namespace, timeout=slowest + COMMAND_WAIT_S + 2.0)

    def end_move(self, move: CspMove, t_request: float) -> str:
        """CspMove.end once the move has ended, the observer watching meanwhile;
        a move not ended COMMAND_WAIT_S + 3 s from now is a refusal."""
        timeout = COMMAND_WAIT_S + 3.0
        if not self.wait_until(
            lambda: not move.thread.is_alive(), time.monotonic() + timeout
        ):
            raise BenchError(
                f"stop-decel: the csp move did not end {timeout:g} s after the stop"
            )
        return move.end(t_request)

    # steps
    def idle_statuswords(self, tag: str) -> dict[int, str]:
        """Every axis not Operation enabled, with bit 2 clear. Returns the axes in FAULT (index ->
        statusword and, best effort, 0x603F Error code) — Guard.latched."""
        self.need(self.ethercat.idle(), f"{tag}: master Idle before SDO reads")
        faults = {}
        for i, a in enumerate(self.axes):
            sw = self.ethercat.statusword(a)
            self.need(
                state(sw) != "OPERATION_ENABLED" and not sw & 4,
                f"{tag}: {a.name} idle statusword {state(sw)} (0x{sw:04x}) not Operation enabled, bit 2 clear",
            )
            if state(sw) == "FAULT":
                faults[i] = f"{a.name} sw 0x{sw:04x}{self.error_code(a)}"
        return faults

    def error_code(self, a: Axis) -> str:
        """' 0x603F 0x….' — '' when the drive has no such object or the read fails."""
        index = a.sdo(0x603F)
        try:
            raw = self.ethercat.upload(a.position, index, "uint16")
            return (
                ""
                if raw is None
                else f" 0x{index:04X} {error_code_text(int(raw.split()[-1], 0))}"
            )
        except (BenchError, ValueError, IndexError):
            return ""

    def await_start(self) -> bool:
        """Two consecutive settled statuses, joint_states and ~/info. An axis the
        idle read saw in FAULT (a fault already latched before the bench started,
        e.g. left by the previous exit — its cause is not checked) may show FAULT
        until the node is up; then, once every axis reports a status word (on
        distributed clocks a slave can reach OP seconds after another, and a
        reset sent before it reports does not reach it), one reset_error, logged
        as a WARN and kept for the notes, and a FAULT still there fails.
        True = that reset was sent."""
        g = self.guard

        def up():
            return (
                self.node.p.poll() is None
                and g.joint_states is not None
                and self.info is not None
            )

        seen = []  # the FAULT axes of the snapshot the wait ended on

        def started():
            # One snapshot under the lock: a status landing after it cannot turn
            # "a latched FAULT" into "settled" with one settled sample behind it.
            with g.lock:
                seen[:] = g.faulted()
                reporting = g.status is not None and all(g.status.statusword)
                return up() and (g.settled_count >= 2 or (bool(seen) and reporting))

        up_in_time = self.wait_for(started, NODE_START_S)
        # The cause is read only on a timeout: one `ethercat slaves` call for the
        # facts (AL state per file slave), the node's own report as the fallback.
        self.need(
            up_in_time,
            "start: joint_states, ~/info and two consecutive settled statuses (or the Fault the idle "
            "read saw)"
            + (
                ""
                if up_in_time
                else g.start_note(
                    self.ethercat.slaves_not_in_op(
                        [s.position for s in self.check.slaves]
                    )
                )
            ),
        )
        g.guard()  # a FAULT on any other axis is latched: it stops here
        faulted = list(seen)
        if not faulted:
            return False
        note = (
            "WARN fault already latched before the bench started: "
            + ", ".join(g.latched[i] for i in faulted)
            + " — reset_error sent once"
        )
        self.log(note)
        self.notes.append(note)
        r = self.call(reset_error=True)
        if not r.success:
            raise BenchError(
                f'start: reset_error refused ("{r.message}") — check the run chain\'s output in this '
                "log, clear the drive alarm and re-run"
            )
        settled = self.wait_for(lambda: up() and g.settled_count >= 2, COMMAND_WAIT_S)
        with g.lock:
            if not settled and (still := g.faulted()):
                raise BenchError(
                    "start: "
                    + ", ".join(
                        f"{self.axes[i].name} 0x{g.status.statusword[i]:04x}"
                        for i in still
                    )
                    + " still in Fault after one reset_error — clear the drive alarm (its 0x603F error "
                    "code is in the WARN above when the drive reported one) and re-run"
                )
        self.need(settled, "start: two consecutive settled statuses after the reset")
        return True

    def release(self, tag: str) -> None:
        with self.guard.lock:
            self.guard.release_gate()
        self.through_drive(
            tag,
            self.drive.release,
            self.guard.ready,
            "every axis Operation enabled, released, nothing pending",
        )
        self.guard.phase_to("hold")

    def hold(self, seconds: float, tag: str) -> None:
        self.wait_for(lambda: False, seconds)
        self.log(f"{tag}: {seconds:g} s hold, max drift {self.guard.worst} rad")

    def stale(self, mark: int, t_kill: int) -> str:
        g = self.guard
        self.need(
            self.wait_for(
                self.status_is(
                    lambda s: g.settled(s, DriveStatus.PROTECTIVE_STOP_SOURCE_EXTERNAL)
                ),
                3.0,
            ),
            "stale: EXTERNAL protective stop, every axis Switch on disabled or Ready to switch on, disabled, engaged, nothing pending",
        )
        self.wait_for(
            lambda: False, 0.3
        )  # drain the event publisher with the guards still active
        return judge_stale_engage(g.events[mark:], t_kill, self.check, self.need)

    def await_cruise(
        self, moves: list[Move], cruise: dict, deadline: float, move=None
    ) -> None:
        """Every moving axis at 0.9 of its cruise (joint_states) by `deadline`,
        else a refusal naming the axes short of it; a csp move (CspMove) that
        failed ends the wait at once."""
        g = self.guard

        def short():
            v = g.joint_states.velocity
            return [m for m in moves if abs(v[m.index]) < 0.9 * cruise[m.index]]

        def at_cruise():
            if move is not None:
                move.check()
            return not short()

        reached = self.wait_until(at_cruise, deadline)
        self.need(
            reached,
            "stop-decel: every moving axis at cruise before the stop request"
            + (
                ""
                if reached
                else " — short of it: "
                + ", ".join(
                    f"{self.axes[m.index].name} {abs(g.joint_states.velocity[m.index]):.4f} of "
                    f"{cruise[m.index]:.4f} rad/s"
                    for m in short()
                )
            ),
        )

    def stop_decel(self) -> str:
        """The bench's one motion. Released, every csp axis (plem.drive.Bus,
        Trapezoid) and every csv axis (a stream ramped at its acceleration
        limit) moves its stop_moves() way; the cruises start together. Once
        every moving axis is seen at cruise (a csp move starts with its goal's
        latency), the stop is requested in the middle of the
        shortest cruise, or at three quarters of it at the latest (stop_times),
        and gets the plan's time to rest (stop_settle_s) to settle. Only the
        moving axes leave the drift check.
        PASS: no stop timeout (0x4F), every axis at rest with its brake
        engaged, and each axis's average deceleration from the request to
        rest (|v at the request| / time to rest, from rt_raw's every cycle) at most
        DECEL_MARGIN x its limits.acceleration_rad_s2."""
        from plem.drive.profile import Limits, Scale, Trapezoid

        g = self.guard
        with g.lock:
            g.guard()
            moves = stop_moves(self.axes, g.joint_states.position)
        csp = [m for m in moves if self.axes[m.index].mode == "csp"]
        csv = [m for m in moves if self.axes[m.index].mode == "csv"]
        cruise = {m.index: m.cruise for m in csv}  # the speed each axis reaches
        # (time to cruise, time at cruise) per csv axis and for the csp axes
        # together (one phase-synchronized plan); every cruise starts at `begin`.
        phases = [
            (
                m.cruise / self.axes[m.index].accel,
                m.distance / m.cruise - m.cruise / self.axes[m.index].accel,
            )
            for m in csv
        ]
        if csp:
            names = [a.name for a in self.axes if a.mode == "csp"]
            start = [g.joint_states.position[m.index] for m in csp]
            goal = [x + m.direction * m.distance for x, m in zip(start, csp)]
            scale = min(m.cruise / self.axes[m.index].vcap for m in csp)
            plan = Trapezoid().plan(
                start,
                goal,
                [
                    Limits(self.axes[m.index].vcap, self.axes[m.index].accel)
                    for m in csp
                ],
                Scale(scale, 1.0),
            )
            seconds = [
                p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                for p in plan.points
            ]
            phases.append((seconds[1], seconds[2] - seconds[1]))
            cruise.update(
                (m.index, abs(v)) for m, v in zip(csp, plan.points[1].velocities)
            )
        begin = max(rise for rise, _ in phases)
        t_stop, t_cruise_seen = stop_times(phases)
        settle_s = stop_settle_s(moves, self.axes)

        go = [math.inf]  # when the motion starts: the ramp streams rest until then
        if g.streamed:
            ramps = {m.index: m for m in csv}
            delay = {m.index: begin - m.cruise / self.axes[m.index].accel for m in csv}

            def values():
                t = time.monotonic() - go[0]
                return {
                    a.name: ramp_velocity(ramps[i], a.accel, t - delay[i])
                    if i in ramps
                    else 0.0
                    for i, a in enumerate(self.axes)
                    if a.mode != "csp"
                }

            self.end_stream()  # the stream at rest makes way for the ramp
            self.start_stream(
                lambda topic, rest: Ramp(self.drive.stream, values, self.log)
            )
        self.release("stop-decel")
        with g.lock:
            g.phase_to("moving")
            g.moving, g.neutral, g.samples = frozenset(m.index for m in moves), None, []
            g.rt_lost = 0
        t0 = go[0] = time.monotonic()
        move = (
            CspMove(
                lambda: self.drive.move_to(
                    dict(zip(names, goal)), velocity_scale=scale
                ),
                begin - seconds[1],
            )
            if csp
            else None
        )
        self.wait_until(lambda: False, t0 + t_stop)
        self.await_cruise(moves, cruise, t0 + t_cruise_seen, move)
        mark = len(g.events)
        g.phase_to("stopping")
        t_request = time.monotonic()
        self.stop("stop-decel", settle_s)
        self.wait_for(lambda: False, 0.3)  # the events and samples after rest
        if g.streamed:
            self.end_stream()
        if move is not None and (note := self.end_move(move, t_request)):
            self.log(note)
        with g.lock:
            samples, g.samples, lost = g.samples, None, g.rt_lost
            g.neutral = (0.0,) * g.streamed
            events = g.events[mark:]
        g.phase_to("braked")
        return judge_stop_decel(
            self.axes, moves, cruise, t_request, events, samples, lost, self.need
        )

    def rated_torques(self) -> None:
        """0x6076 of each rated_per_mille axis against its file value — logged,
        a WARN also kept for the bench file's notes, never refused (a failed read
        is "not compared", not an abort)."""
        for a in self.axes:
            if a.rated_torque_mnm is None:
                continue
            try:
                raw = self.ethercat.upload(a.position, a.sdo(0x6076), "uint32")
                note = rated_torque_note(a, raw)
            except (BenchError, ValueError, IndexError) as e:
                note = f"{a.name}: 0x{a.sdo(0x6076):04X} motor rated torque not compared ({e})"
            self.log(note)
            if note.startswith("WARN "):
                self.notes.append(note)

    def read_object(self, a: Axis, index: int, subindex: int, typ: str) -> int | None:
        raw = self.ethercat.upload(a.position, index, typ, subindex)
        return None if raw is None else int(raw.split()[-1], 0)

    def drive_objects(self) -> None:
        """Each axis's CiA 402 objects before the node starts: a mode the drive
        cannot run refuses the bench (every such axis named at once);
        disagreements are WARN lines kept for the bench file's notes; the rest
        is logged (values to paste included)."""
        failures = []
        for a in self.axes:
            failure, warns, info = drive_object_report(
                a, lambda index, sub, typ, a=a: self.read_object(a, index, sub, typ)
            )
            for line in info + warns:
                self.log(line)
            self.notes.extend(warns)
            if failure:
                failures.append(failure)
        if failures:
            raise BenchError("\n".join(failures))

    def positions(self) -> list[int | None]:
        """0x6064 per axis, read with the node stopped (None = not read)."""
        out = []
        for a in self.axes:
            try:
                out.append(self.read_object(a, a.sdo(0x6064), 0, "int32"))
            except (BenchError, ValueError, IndexError):
                out.append(None)
        return out

    def run(self) -> list[Identity]:
        g = self.guard
        g.latched = self.idle_statuswords("idle")
        self.items["idle-non-oe"] = (
            "PASS",
            "idle SDO 0x6041 per axis: not Operation enabled, bit 2 clear",
        )
        identities = []
        for s, slave in enumerate(self.check.slaves):
            ident, notes = self.ethercat.identity(slave.position)
            self.log(
                f"slave {s} (ring {slave.position}): {ident}"
                + (f" — {'; '.join(notes)}" if notes else "")
            )
            identities.append(ident)
        self.rated_torques()
        self.drive_objects()
        before = self.positions()

        self.start_ros()
        self.start_node(run_argv(self.args, plem_drive_command()))
        reset = self.await_start()
        self.start_drive()
        with g.lock:
            g.monitor, g.monitor_ns = True, time.monotonic_ns()
            g.guard()
        q = tuple(g.joint_states.position)
        self.need(
            all(a.lo <= x <= a.hi for x, a in zip(q, self.axes)),
            f"start: positions inside the limits {q}",
        )
        g.phase_to("braked")
        self.items["start-rtso"] = (
            "PASS",
            "two consecutive statuses: Switch on disabled or Ready to switch on, engaged, pending 0, protective stop 0; "
            "positions inside the declared limits"
            + (
                "; after one reset_error sent by the bench for a fault latched before it"
                if reset
                else ""
            ),
        )

        digests = list(self.info.pdo_mapping_digest)
        problem = digests_match_layout(digests, self.check.slaves)
        self.need(
            not problem,
            f"assembly: {problem or '~/info matches the declared mappings'}",
        )
        for s, digest in enumerate(digests):
            identities[s].pdo_mapping_digest = digest or None
        for s, ident in enumerate(identities):
            self.need(
                ident.observes_something(),
                f"slave {s}: the bench observed at least one identity value (an approval binds to one)",
            )
        # The observer latches joint_states naming other joints than the file's axes.
        names = list(g.joint_states.name)
        self.items["assembly-match"] = (
            "PASS",
            f"joint_states carries the file's axes {names}; ~/info digests "
            "match the declared scanned/declared mappings",
        )

        # What is measured is check's required list (the C++ decides it); the
        # axis modes only say how (which axes stream, which move).
        stale = "stale-engage" in self.check.required
        moving = "stop-decel" in self.check.required
        if g.streamed:
            self.start_stream()
        self.release("release 1")
        r = self.call(release_brake=True)
        self.need(
            r.success and g.ready(g.status),
            "release 1: a second release_brake is a no-op (still Operation enabled)",
        )
        self.hold(HOLD_S, "release 1")
        self.items["release-oe"] = (
            "PASS",
            ("stream at rest + " if g.streamed else "")
            + f"release_brake -> Operation enabled, engaged 0, pending 0; second call no-op; {HOLD_S:g} s hold, "
            f"max drift {max(g.worst):.6f} rad",
        )

        if stale:
            mark = len(g.events)
            g.phase_to("stopping", (0, DriveStatus.PROTECTIVE_STOP_SOURCE_EXTERNAL))
            self.items["stale-engage"] = ("PASS", self.stale(mark, self.end_stream()))
            self.start_stream()
            self.need(self.call(reset_error=True).success, "reset request accepted")
            self.need(
                self.wait_for(self.status_is(lambda s: g.settled(s, 0)), 3.0),
                "reset: protective stop 0, every axis settled",
            )
            g.phase_to("braked")
            self.release("release 2")
            self.hold(SECOND_HOLD_S, "release 2")
        else:
            self.items["stale-engage"] = (
                "NA",
                "csp lane: the trajectory controller writes every cycle; a stalled controller manager's "
                "stale stop is not measured by the bench",
            )
        g.phase_to("stopping")
        self.stop("disable-engage")
        self.items["disable-engage"] = (
            "PASS",
            f"protective_stop service -> disabled{', 0 torque' if self.all_cst else ''}, every axis "
            "engaged, pending 0, protective stop 0",
        )
        g.phase_to("braked")
        if moving:
            self.items["stop-decel"] = ("PASS", self.stop_decel())
            if g.streamed:
                self.start_stream()
        else:
            self.items["stop-decel"] = (
                "NA",
                stop_decel_na(a.mode for a in self.axes),
            )
        self.release("last release")

        g.phase_to("stopping", g.allowed_stops)
        with g.lock:
            g.guard()
            g.monitor = (
                False  # shutdown ends observation on purpose; no release follows
            )
        self.end_node()
        if self.pub is not None:
            self.end_stream()
        self.need(
            self.wait_for(lambda: not run_pids(self.args.namespace), 5.0, 0.2),
            "stop: no process of the run chain left",
        )
        self.need(
            self.wait_for(self.ethercat.idle, 15.0, 0.2), "stop: master back to Idle"
        )
        self.idle_statuswords("stop")
        moved = "stop-decel" in self.check.required  # every csp and csv axis
        for a, b, after in zip(self.axes, before, self.positions()):
            note = position_continuity_note(a, b, after, moved and a.mode != "cst")
            self.log(note)
            if note.startswith("WARN "):
                self.notes.append(note)
        for note in status_bit_notes(g.history, self.axes, g.monitor_ns):
            self.log(note)
            self.notes.append(note)
        self.items["stop-quiesce"] = (
            "PASS",
            "SIGINT to the controller manager while released: RC 0, no unconfirmed-brakes line was logged "
            f"('{UNCONFIRMED_BRAKES}'), no remnants, master Idle, and the drives report not Operation enabled with "
            "bit 2 clear on an idle SDO read",
        )
        self.items["fault-reset"] = (
            "NA",
            "fault_reset.policy none — no automatic reset to measure"
            if self.check.fault_reset_policy == "none"
            else "automatic reset is not measured by the v1 bench",
        )
        return identities

    def cleanup(self) -> bool:
        """Node first (SIGINT — its quiesce confirms the brakes) with the stream
        alive, then the stream, then ROS; last, the observer's final latch (a
        violation after the last guard check still voids the bench: its error
        is read directly, without the freshness check). SIGINT/SIGTERM are held
        for the teardown only: each is recorded and voids the bench, the
        teardown runs to the end, and the caller's handlers are back on return.
        True = everything ended clean, nothing latched, no signal."""
        held = {
            s: signal.signal(s, lambda n, _f: self.interrupted.append(n))
            for s in (signal.SIGINT, signal.SIGTERM)
        }
        try:
            ok = self.end_everything()
        finally:
            for s, handler in held.items():
                signal.signal(s, handler)
        if self.interrupted:
            ok = False
            self.log(
                f"cleanup: signal {self.interrupted} held until the teardown ended — no approval"
            )
        return ok

    def end_everything(self) -> bool:
        ok = True
        if self.node is not None:
            ok &= self.teardown("run chain", self.end_node)
        if self.pub is not None:
            ok &= self.teardown("stream", self.end_stream)
        if self.drive is not None:
            drive, self.drive = self.drive, None
            ok &= self.teardown("plem.drive client", drive.close)
        ok &= self.stop_ros()
        with self.guard.lock:
            if self.guard.error:
                ok = False
                self.log(f"observer latched: {self.guard.error}")
        if run_pids(self.args.namespace):
            ok = False
            self.log("a process of the run chain is still running")
        return ok


CONFIRM = """The drive bench releases the brakes of every axis in {model} {releases} times, {held}.
Confirm only if all of the following hold:
  - You are at the drive, with the emergency stop in reach.
  - Every axis under gravity is propped: a released axis that moves more than {drift} rad while held aborts the bench.
  - Axes not in this file are held by their own brakes or mechanically.
  - No drive in this file has a brake shared by several axes (this procedure does not cover that case).
{moves}{replaced}Proceed? [y/N] """

REPLACED = "  - The file's current approval is replaced when this bench passes.\n"

MOVES = """  - Once, every csp and csv axis MOVES up to {distance:g} rad in either direction from where it rests
    (the bench picks the side with room) at up to {fraction:g} of its velocity cap, then stops:
    clear the workspace on both sides of every such axis.
"""


def confirm_text(model: str, check: Check) -> str:
    """What the operator confirms: the releases, and the one motion when
    check requires stop-decel."""
    moving = "stop-decel" in check.required
    return CONFIRM.format(
        model=model,
        # release 1, (release 2 after stale-engage), (stop-decel's), the last
        releases=2 + ("stale-engage" in check.required) + moving,
        held="each held where it rests (cst at 0 Nm, csv at 0 rad/s, csp at its position)",
        drift=DRIFT_ABORT_RAD,
        moves=MOVES.format(distance=STOP_DISTANCE_RAD, fraction=CRUISE_FRACTION)
        if moving
        else "",
        replaced=REPLACED if check.claimed == "approved" else "",
    )


MEASURED = (
    "assembly-match",
    "idle-non-oe",
    "start-rtso",
    "release-oe",
    "stale-engage",
    "stop-decel",
    "disable-engage",
    "stop-quiesce",
    "fault-reset",
)


def run_tool(binary: str, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run([binary, *args], capture_output=True, text=True, timeout=60)


def preflight(args, tool: str, say) -> tuple[EthercatCli, Check]:
    """Everything before any bus write or brake — refusals only."""
    checked = run_tool(tool, "check", "--json", str(args.model))
    # Exit 1 with the JSON = an approved claim not honored: benched like an
    # unapproved file. Exit 1 without it = a refusal.
    if checked.returncode != 0 and not checked.stdout.strip():
        raise BenchError(f"plem drive check refused the file:\n{refusal_text(checked)}")
    check = parse_check(checked.stdout)
    for reason in check.reasons:
        say(f"the approval it claims no longer holds — the bench replaces it: {reason}")
    for warning in check.warnings:
        say(f"WARN {warning}")
    ethercat = EthercatCli(args.master)
    ethercat.check_access()
    if check.approve_blocker:
        raise BenchError(f"approve would refuse this file: {check.approve_blocker}")
    unknown = [i for i in check.required if i not in MEASURED]
    if unknown:
        raise BenchError(
            f"this bench cannot measure the required item(s) {unknown} — install plem packages "
            "from one release"
        )
    if bus_users() or not ethercat.idle():
        raise BenchError(
            "the bus is in use: a ros2_control_node of this user is running, or `ethercat master` does not "
            "show Phase: Idle (Operation = another program holds the master; Orphaned = no network device attached)"
        )
    try:  # run calls the bench on a terminal only
        answer = input(confirm_text(args.model.name, check))
    except (EOFError, KeyboardInterrupt):
        answer = ""
    if answer.strip().lower() != "y":
        raise BenchError("not confirmed — nothing was touched")
    if bus_users() or not ethercat.idle():  # the prompt may have waited
        raise BenchError("the bus became busy while waiting for the confirmation")
    say("preflight passed")
    return ethercat, check


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="plem drive run", description=__doc__.splitlines()[0]
    )
    parser.add_argument("model", type=Path)
    parser.add_argument("--master", type=int, default=0)
    parser.add_argument("--rt-cpu", type=int)
    args = parser.parse_args(argv)
    args.model = args.model.resolve()
    log_path = bench_log(args.model)
    if not os.access(args.model.parent, os.W_OK) or not os.access(args.model, os.W_OK):
        print(
            f"plem drive run: cannot write {args.model} or next to it — the approval is recorded in "
            "the file and the log beside it; make both writable",
            file=sys.stderr,
        )
        return 1
    if (owner := args.model.stat().st_uid) != os.geteuid():
        print(
            f"plem drive run: {args.model} is owned by uid {owner}, not by you (uid {os.geteuid()}) — "
            "recording the approval would make it yours; bench it as its owner",
            file=sys.stderr,
        )
        return 1
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    # Nothing is written before the confirmation: a refusal or a decline is one
    # line and leaves the last bench's log as it was; once confirmed, the lines
    # said so far open the log.
    early = io.StringIO()
    try:
        tool = drive_model_tool()
        ethercat, check = preflight(args, tool, lambda msg: log_line(early, msg))
        args.namespace = BENCH_NAMESPACE
    except (Exception, KeyboardInterrupt, SystemExit) as e:
        reason = e if isinstance(e, BenchError) else f"{type(e).__name__}: {e}"
        print(f"plem drive run: {reason}", file=sys.stderr)
        return 1
    approved = False
    with open(log_path, "w") as sink:
        sink.write(early.getvalue())

        def say(msg):
            log_line(sink, msg)

        bench = None
        try:
            bench = Bench(args, check, ethercat, sink)
            identities = bench.run()
        except (Exception, KeyboardInterrupt, SystemExit) as e:
            say(
                f"ABORT: check failed, expected: {e}"
                if isinstance(e, CheckFailed)
                else f"ABORT: {e}"
                if isinstance(e, BenchError)
                else f"ABORT: {type(e).__name__}: {e}"
            )
            identities = None
        finally:
            if bench is not None and not bench.cleanup():
                say(
                    "cleanup did not finish cleanly (brakes unconfirmed, a process left, the observer latched or a "
                    "signal) — no approval; check the drive"
                )
                identities = None
        if identities is not None:
            try:
                conditions = {
                    "host": socket.gethostname(),
                    "master": ethercat.version(),
                    "date": time.strftime("%Y-%m-%d"),
                    "measured_by": "plem drive run (bench)",
                    "notes": "; ".join(bench.notes),
                }
                with tempfile.TemporaryDirectory(prefix="plem-drive-bench.") as private:
                    evidence = Path(private) / "bench.yaml"
                    evidence.write_text(
                        dump_bench(
                            bench_document(check, identities, bench.items, conditions)
                        )
                    )
                    done = run_tool(tool, "approve", str(args.model), str(evidence))
                say(done.stdout.strip())
                if done.returncode == 0:
                    approved = True
                    say(f"approval recorded in {args.model}")
                else:
                    say(f"approve refused:\n{done.stderr.strip()}")
            except (OSError, subprocess.SubprocessError, BenchError) as e:
                say(
                    f"cannot finish the bench file or approval: {type(e).__name__}: {e}"
                )
            except (KeyboardInterrupt, SystemExit) as e:
                approved = False  # an interrupted finish is reported as one, never as a success
                say(
                    f"interrupted ({type(e).__name__}) while finishing — the approval may or may not "
                    f"have been written: `plem drive check {args.model}` shows whether it is approved"
                )
    if approved:
        log_path.unlink(missing_ok=True)
        return 0
    print(f"plem drive run: not approved — the log is {log_path}", file=sys.stderr)
    return 1
