# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The drive monitor — bus timing from a running drive bus's rt_raw, one row a second.

    plem drive run <model.yaml> --monitor          (in the terminal of the bus it starts)
    python3 -m plem.drive.cli.monitor <bus>      (another terminal, a bus already running)

<bus> is the model file's bus.name. Subscribes to /<bus>/rt_raw (every RT
cycle, batched), /<bus>/plem_drive/status and /<bus>/plem_drive/rt_events (a
fault names the drive's error code the bus reported, while that event is
still in the topic's 50-event history).
Every second it prints a row of what arrived in that second, under a header
it repeats every 20 rows, and a line of its own whenever the bus's state
changes; on Ctrl-C it prints the total. A recording can be measured the same
way — `ros2 bag play` it and run this against the same bus (a sample lost
while recording counts as lost; the one-second windows follow the replay). For a fixed-length run:
`timeout -s INT 1800 python3 -m plem.drive.cli.monitor <bus>`.

Columns (each name carries its unit; /s = counted in that second):
  exec-mean, exec-max  [us] loop_exec_us — processing time of one RT cycle.
  pdev-p99, pdev-max   [us] |loop_jitter_us| — deviation of one wake-to-wake period
                       from the nominal period. It is not lateness against a
                       schedule.
  overrun              /s deadline_miss — cycles whose processing took longer
                       than the period.
  wkc-err              /s wkc_mismatch — cycles whose working counter was wrong
                       (every cycle while a slave is not yet in OP).
  lost                 /s gaps in the per-sample sequence. A producer overflow and
                       a DDS drop count the same; a restarted node (sequence
                       back to 0) is a new session, not loss.
  dc-max               [ns] dc_sync_monitor_ns max. 0 is DC off or a true 0 (rt_raw
                       does not tell them apart); cycles without a value are
                       skipped.
  state                the last DriveStatus in words.

Percentiles come from 0.1 us bins up to 1000 us and report the bin's upper edge
(never above the exact max); a value beyond 1000 us reports the exact max.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from plem.drive.cia402 import fault_bit

DC_NO_VALUE = 0xFFFFFFFF  # rt_raw's "no DC monitor value this cycle"
# A row's values are right-aligned under these names, each with its unit.
COLUMNS = tuple(
    (name, max(len(name), 8))
    for name in (
        "time",
        "exec-mean[us]",
        "exec-max[us]",
        "pdev-p99[us]",
        "pdev-max[us]",
        "overrun/s",
        "wkc-err/s",
        "lost/s",
        "dc-max[ns]",
    )
)
HEADER = " ".join(f"{name:>{width}}" for name, width in COLUMNS) + "  state"
HEADER_EVERY = 20  # rows
_BIN_PER_US = 10  # 0.1 us bins
_BINS = 1000 * _BIN_PER_US  # 0..1000 us, then one overflow bin


class Fold:
    """Folds rt_raw batches into counters and a period-deviation histogram.
    ROS-free: a batch is anything with the RtRawBatch per-sample fields."""

    def __init__(self) -> None:
        self.samples = 0
        self.lost = 0
        self.overruns = 0
        self.wkc_errors = 0
        self.exec_max_us = 0.0
        self._exec_sum_us = 0.0
        self.dev_max_us = 0.0
        self.dc_max_ns: int | None = None
        self._hist = [0] * (_BINS + 1)
        self._last_seq: int | None = None

    def fold(self, batch) -> None:
        for i, seq in enumerate(batch.sequence):
            if self._last_seq is not None and seq > self._last_seq + 1:
                self.lost += seq - self._last_seq - 1
            self._last_seq = seq
            self.samples += 1
            exec_us = batch.loop_exec_us[i]
            self._exec_sum_us += exec_us
            self.exec_max_us = max(self.exec_max_us, exec_us)
            dev = abs(batch.loop_jitter_us[i])
            self.dev_max_us = max(self.dev_max_us, dev)
            self._hist[min(int(dev * _BIN_PER_US), _BINS)] += 1
            self.overruns += batch.deadline_miss[i]
            self.wkc_errors += batch.wkc_mismatch[i]
            dc = batch.dc_sync_monitor_ns[i]
            if dc != DC_NO_VALUE:
                self.dc_max_ns = (
                    dc if self.dc_max_ns is None else max(self.dc_max_ns, dc)
                )

    def carry(self, other: Fold) -> None:
        """Keep the sequence position across a window reset, so a gap that
        straddles two windows is counted once."""
        self._last_seq = other._last_seq

    @property
    def exec_mean_us(self) -> float | None:
        return self._exec_sum_us / self.samples if self.samples else None

    def dev_percentile(self, q: float) -> float | None:
        if not self.samples:
            return None
        rank = max(1, math.ceil(q * self.samples - 1e-9))  # 0.999 * 1000 must stay 999
        seen = 0
        for b, count in enumerate(self._hist):
            seen += count
            if seen >= rank:
                if b == _BINS:
                    return self.dev_max_us
                return min((b + 1) / _BIN_PER_US, self.dev_max_us)
        return self.dev_max_us

    def row(self, clock: str, state: str) -> str:
        """One row under HEADER: this window's numbers, then `state`."""
        if not self.samples:
            return f"{clock:>8}  no samples  {state}"
        values = (
            clock,
            f"{self.exec_mean_us:.1f}",
            f"{self.exec_max_us:.1f}",
            f"{self.dev_percentile(0.99):.1f}",
            f"{self.dev_max_us:.1f}",
            self.overruns,
            self.wkc_errors,
            self.lost,
            "-" if self.dc_max_ns is None else self.dc_max_ns,
        )
        cells = " ".join(f"{v:>{w}}" for v, (_, w) in zip(values, COLUMNS))
        return f"{cells}  {state}"

    @property
    def troubled(self) -> bool:
        """This window had an overrun, a WKC error or a lost sample."""
        return bool(self.overruns or self.wkc_errors or self.lost)

    def summary(self, seconds: float) -> str:
        head = f"total over {seconds:.1f} s: "
        if not self.samples:
            return head + "no samples"
        return (
            head
            + f"samples {self.samples}  lost {self.lost}\n"
            + f"  exec max {self.exec_max_us:.1f} us\n"
            + f"  period_dev max {self.dev_max_us:.1f}  p99 {self.dev_percentile(0.99):.1f}"
            + f"  p99.9 {self.dev_percentile(0.999):.1f} us\n"
            + f"  overruns {self.overruns}  wkc_errors {self.wkc_errors}"
        )


def state_text(status, fault_codes: dict[int, int] | None = None) -> tuple[str, bool]:
    """The bus's state in words from a DriveStatus, and whether it is an alarm.
    fault_codes: axis index -> the drive's error code from rt_events' SERVO_FAULT."""
    if status is None:
        return "no status yet", False
    words = list(status.statusword)
    if words and not any(words):
        return "every statusword 0 (drives not reporting yet)", False
    # rclpy: imported once a status arrives
    from plem.drive.bus import STOP_SOURCES, fault_detail

    codes = fault_codes or {}
    said = [what for bit, what in STOP_SOURCES if status.protective_stop & bit]
    said += [
        f"axis{i} fault ({fault_detail(w, codes.get(i))})"
        for i, w in enumerate(words)
        if fault_bit(w)
    ]
    if said:
        return ", ".join(said), True
    if status.brake_operation_pending:
        return "brake operation pending", False
    if status.operation_enabled and not status.brake_engaged:
        return "released", False
    every = (1 << len(words)) - 1
    if status.brake_engaged == every:
        return "held, brakes on", False
    loose = [i for i in range(len(words)) if not status.brake_engaged >> i & 1]
    return ", ".join(f"axis{i} brake not confirmed on" for i in loose), True


def _paint(text: str, color: str) -> str:
    """ANSI color on a terminal, unless NO_COLOR is set (no-color.org)."""
    if not sys.stdout.isatty() or os.environ.get("NO_COLOR"):
        return text
    return f"\033[{color}m{text}\033[0m"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python3 -m plem.drive.cli.monitor",
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="\n".join(__doc__.splitlines()[2:]),
    )
    parser.add_argument("bus", help="the drive bus's name (the model file's bus.name)")
    args = parser.parse_args(argv)
    ns = "/" + args.bus.strip("/")

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from plem_drive_msgs.msg import DriveStatus
    from plem_rt_msgs.msg import RtEvent, RtRawBatch

    from plem.drive.bus import forget_cleared

    rclpy.init()
    node = rclpy.create_node("plem_drive_monitor")
    total, window = Fold(), Fold()
    state, _ = state_text(None)
    fault_codes: dict[int, int] = {}
    rows = 0
    started = time.monotonic()

    def on_batch(msg) -> None:
        total.fold(msg)
        window.fold(msg)

    def on_status(msg) -> None:
        nonlocal state
        forget_cleared(fault_codes, msg.statusword)
        said, alarm = state_text(msg, fault_codes)
        if said != state:
            state = said
            line = f"{time.strftime('%H:%M:%S')}  -> {said}"
            print(_paint(line, "1;31" if alarm else "1"), flush=True)

    def on_event(msg) -> None:
        if msg.event_type == RtEvent.SERVO_FAULT:
            fault_codes[msg.joint_id] = int(msg.value)

    def on_tick() -> None:
        nonlocal window, rows
        if rows % HEADER_EVERY == 0:
            print(_paint(HEADER, "2"), flush=True)
        rows += 1
        row = window.row(time.strftime("%H:%M:%S"), state)
        print(_paint(row, "33") if window.troubled else row, flush=True)
        fresh = Fold()
        fresh.carry(window)
        window = fresh

    node.create_subscription(
        RtRawBatch,
        f"{ns}/rt_raw",
        on_batch,
        QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    node.create_subscription(
        DriveStatus,
        f"{ns}/plem_drive/status",
        on_status,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    node.create_subscription(
        RtEvent,
        f"{ns}/plem_drive/rt_events",
        on_event,
        QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    node.create_timer(1.0, on_tick)
    print(f"monitor: {ns} timing, one row a second — Ctrl-C for the total", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(total.summary(time.monotonic() - started), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
