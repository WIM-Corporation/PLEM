#!/bin/bash
# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
#
# tools/bench_report.py without ROS: a three-axis batch flattens to rows whose speed is the fastest axis,
# and `report` on such rows finds the moving phase when only the third axis moves.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! python3 -c 'import numpy, matplotlib' 2>/dev/null; then
    echo "SKIP: test_bench_report.sh needs numpy and matplotlib" >&2
    exit 0
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
PYTHONDONTWRITEBYTECODE=1 python3 - "$HERE" "$tmp" <<'EOF'
import csv, gzip, json, sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, sys.argv[1])
import bench_report

tmp = Path(sys.argv[2])
period = 1_000_000
n = 3000
# Three axes; only the third moves (2 rad/s) in cycles 1000-1999.
batch = SimpleNamespace(
    batch_size=n,
    axis_count=3,
    sequence=list(range(n)),
    monotonic_ns=[i * period for i in range(n)],
    loop_exec_us=[12.0] * n,
    deadline_miss=[0] * n,
    wkc_mismatch=[0] * n,
    dc_sync_monitor_ns=[500] * n,
    actual_velocity=[v for i in range(n) for v in (0.0, 0.0, -2.0 if 1000 <= i < 2000 else 0.0)],
)
rows = list(bench_report.cycle_rows(batch))
assert len(rows) == n, len(rows)
assert rows[1500][-1] == 2.0, rows[1500]
assert rows[0][-1] == 0.0, rows[0]

cycles = tmp / "cycles.csv.gz"
with gzip.open(cycles, "wt", newline="") as f:
    f.write(f"# period_ns {period}\n")
    w = csv.writer(f)
    w.writerow(bench_report.COLUMNS)
    w.writerows(rows)
bench_report.report(str(cycles), str(tmp / "out"))
summary = json.loads((tmp / "out" / "summary.json").read_text())
assert summary["phases"]["moving"]["seconds"] == 1.0, summary["phases"]
for svg in ("timeline.svg", "distributions.svg", "dc-deviation.svg"):
    assert (tmp / "out" / svg).is_file(), svg
EOF
rc=$?
[[ $rc -eq 0 ]] && echo "PASS: test_bench_report.sh" || echo "FAIL: test_bench_report.sh" >&2
exit $rc
