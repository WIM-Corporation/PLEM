#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""Timing report of a drive bus run: figures and a summary from its rt_raw.

    python3 bench_report.py <bag_dir> <out_dir>                # both steps: needs ROS 2 + plem_rt_msgs
    python3 bench_report.py export <bag_dir> <cycles.csv.gz>   # needs ROS 2 + plem_rt_msgs
    python3 bench_report.py report <cycles.csv.gz> <out_dir>   # numpy + matplotlib only

`export` flattens the bag's /<bus>/rt_raw batches (plem_rt_msgs/RtRawBatch)
into one row per RT cycle. `report` writes summary.json and three SVG figures.
With a bag and a folder, it does both and keeps cycles.csv.gz in that folder.

Wake latency: the RT loop sleeps to absolute deadlines (start + n x period), so a
cycle's lateness is its wake time minus its deadline, up to one constant (the
lateness of the first cycle). It is reported above the smallest lateness seen:
how much later than its best cycle the loop woke. That holds while the loop
never fell a period behind (that re-bases the deadlines); `report` refuses a run
whose lateness spreads over half a period or more, or that overran.

Phases: "holding" is the run before any axis first turns faster than
MOVING_RAD_S (bus reset and brake release included), "moving" the span from that
cycle to the last one that does.
"""

import csv
import gzip
import json
import re
import sys
from pathlib import Path

import numpy as np

DC_NO_VALUE = 0xFFFFFFFF
MOVING_RAD_S = 0.1
COLUMNS = (
    "sequence",
    "wake_ns",
    "exec_us",
    "overrun",
    "wkc_error",
    "dc_ns",
    "speed",
)


def cycle_rows(m):
    """One row per cycle of an RtRawBatch, in COLUMNS order; speed is the fastest
    axis's absolute velocity."""
    axes = m.axis_count
    for i in range(m.batch_size):
        yield (
            m.sequence[i],
            m.monotonic_ns[i],
            m.loop_exec_us[i],
            m.deadline_miss[i],
            m.wkc_mismatch[i],
            m.dc_sync_monitor_ns[i],
            max(
                (abs(v) for v in m.actual_velocity[i * axes : (i + 1) * axes]),
                default=0.0,
            ),
        )


def export(bag: str, out: str) -> None:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from plem_rt_msgs.msg import RtRawBatch

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=""), ConverterOptions("", ""))
    topics = sorted(
        t.name for t in reader.get_all_topics_and_types() if t.name.endswith("/rt_raw")
    )
    if len(topics) != 1:
        sys.exit(
            f"{bag}: needs exactly one <bus>/rt_raw topic, found {topics or 'none'}"
        )
    rows, period = {}, None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topics[0]:
            continue
        m = deserialize_message(data, RtRawBatch)
        period = m.period_ns
        for row in cycle_rows(m):
            if row[0] in rows:
                sys.exit(
                    f"{bag}: sequence {row[0]} repeats (the bus restarted) — record one run per bag"
                )
            rows[row[0]] = row
    if not rows:
        sys.exit(f"{bag}: {topics[0]} holds no samples")
    with gzip.open(out, "wt", newline="") as f:
        f.write(f"# period_ns {period}\n")
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for seq in sorted(rows):
            r = rows[seq]
            w.writerow(
                (
                    r[0],
                    r[1],
                    f"{r[2]:.2f}",
                    r[3],
                    r[4],
                    r[5],
                    f"{r[6]:.4f}",
                )
            )
    print(f"{len(rows)} cycles -> {out}")


def load(path: str):
    with gzip.open(path, "rt") as f:
        period = int(f.readline().split()[2])
        data = np.genfromtxt(f, delimiter=",", names=True, dtype=None)
    return data, period


def stats(x):
    if len(x) == 0:
        return None
    return {
        "mean": round(float(np.mean(x)), 2),
        "p99": round(float(np.percentile(x, 99)), 2),
        "p99_9": round(float(np.percentile(x, 99.9)), 2),
        "max": round(float(np.max(x)), 2),
    }


def save_svg(fig, path: Path) -> None:
    """Save the figure as SVG, rewriting each color whose hex code is all decimal
    digits (black, for one) as rgb() so issue trackers do not autolink "#<digits>"."""
    fig.savefig(path)
    svg = re.sub(
        r"#([0-9]{2})([0-9]{2})([0-9]{2})\b",
        lambda m: "rgb({},{},{})".format(*(int(g, 16) for g in m.groups())),
        path.read_text(),
    )
    path.write_text(svg)


def report(path: str, out_dir: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d, period = load(path)
    seq = d["sequence"].astype(np.int64)
    span = int(seq[-1] - seq[0]) + 1
    if d["overrun"].sum():
        sys.exit(
            f"{path}: the loop overran {int(d['overrun'].sum())} time(s), so its deadlines were "
            "re-based and the wake latency cannot be computed; record a run without overruns"
        )
    wake = d["wake_ns"].astype(np.int64)
    late = (wake - wake[0]) - (seq - seq[0]) * period
    if late.max() - late.min() >= period // 2:
        # The timer re-bases its deadlines when it wakes past the next one, which
        # need not set the overrun flag: the lateness then jumps by whole periods.
        sys.exit(
            f"{path}: the wake times jump by about a period, so the deadlines were re-based and "
            "the wake latency cannot be computed; record a run without overruns"
        )
    late_us = (late - late.min()) / 1000.0
    exec_us = d["exec_us"].astype(float)
    dc = d["dc_ns"].astype(np.int64)
    dc_ok = dc != DC_NO_VALUE
    fast = np.flatnonzero(d["speed"] > MOVING_RAD_S)
    moving = np.zeros(len(seq), dtype=bool)
    holding = np.ones(len(seq), dtype=bool)
    if len(fast):
        moving[fast[0] : fast[-1] + 1] = True
        holding[fast[0] :] = False
    t = (seq - seq[0]) * period / 1e9

    summary = {
        "period_us": period / 1000,
        "seconds": round(span * period / 1e9, 1),
        "cycles": int(len(seq)),
        "cycles_not_recorded": span - int(len(seq)),
        "overruns": int(d["overrun"].sum()),
        "wkc_errors": int(d["wkc_error"].sum()),
        "wake_latency_us": stats(late_us),
        "cycle_work_us": stats(exec_us),
        "dc_deviation_ns": stats(dc[dc_ok].astype(float)),
        "phases": {
            name: {
                "seconds": round(int(mask.sum()) * period / 1e9, 1),
                "wake_latency_us": stats(late_us[mask]),
                "cycle_work_us": stats(exec_us[mask]),
            }
            for name, mask in (("holding", holding), ("moving", moving))
        },
    }
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    # Per-second maxima for the timelines (one point per second keeps the SVG small).
    sec = np.floor(t).astype(int)
    edges = np.flatnonzero(np.diff(sec)) + 1
    groups = np.split(np.arange(len(sec)), edges)
    ts = np.array([sec[g[0]] for g in groups])
    per = lambda x: np.array([x[g].max() for g in groups])  # noqa: E731
    plt.rcParams.update({"font.size": 9, "svg.fonttype": "none"})

    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(ts, per(late_us), lw=1)
    ax[0].set_ylabel("wake latency\nmax per s [µs]")
    ax[1].plot(ts, per(exec_us), lw=1, color="tab:orange")
    ax[1].set_ylabel("cycle work\nmax per s [µs]")
    ax[2].plot(ts, per(d["speed"]), lw=1, color="tab:green")
    ax[2].set_ylabel("axis speed\nmax per s [rad/s]")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(f"RT loop timing over the run ({1e9 / period:.0f} Hz)")
    fig.tight_layout()
    save_svg(fig, out / "timeline.svg")

    fig, ax = plt.subplots(1, 2, figsize=(8, 3))
    for a, x, name, color in (
        (ax[0], late_us, "wake latency [µs]", "tab:blue"),
        (ax[1], exec_us, "cycle work [µs]", "tab:orange"),
    ):
        a.hist(x, bins=200, color=color, log=True)
        for q, ls in ((99, "--"), (99.9, ":")):
            a.axvline(np.percentile(x, q), color="k", ls=ls, lw=0.8, label=f"p{q}")
        a.axvline(x.max(), color="tab:red", lw=1.0, label="max")
        a.set_xlabel(name)
        a.set_ylabel("cycles")
        a.legend(frameon=False)
    fig.suptitle(f"Distributions over {len(seq)} cycles")
    fig.tight_layout()
    save_svg(fig, out / "distributions.svg")

    fig, a = plt.subplots(figsize=(8, 2.5))
    dct = np.where(dc_ok, dc, 0)
    a.plot(ts, per(dct) / 1000.0, lw=1, color="tab:purple")
    a.set_ylabel("DC deviation\nmax per s [µs]")
    a.set_xlabel("time [s]")
    a.grid(alpha=0.3)
    fig.tight_layout()
    save_svg(fig, out / "dc-deviation.svg")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] not in ("export", "report"):
        out = Path(sys.argv[2])
        out.mkdir(parents=True, exist_ok=True)
        export(sys.argv[1], str(out / "cycles.csv.gz"))
        report(str(out / "cycles.csv.gz"), str(out))
    elif len(sys.argv) == 4 and sys.argv[1] in ("export", "report"):
        (export if sys.argv[1] == "export" else report)(sys.argv[2], sys.argv[3])
    else:
        sys.exit(__doc__)
