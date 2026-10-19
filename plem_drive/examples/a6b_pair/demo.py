# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""Coupled pair, mirrored back and forth: axis1 = S - axis0.

Start the bus first: `plem drive run drive.yaml --plot --monitor`, then `python3 demo.py`.
Leaving the `with` block stops both axes, Ctrl-C included. README.md beside this file: what to check before the first run.
"""

from plem.drive import Bus, SCurve

TRAVEL_RAD = 5.0  # each way from where the axes rest, at the gearhead output
CYCLES = 10
SCALE = 0.4  # of the file's velocity and acceleration limits: about 8 rad/s

with Bus("a6b_pair") as bus:
    bus.reset()  # clears a fault or protective stop left from the last run
    bus.release()
    p = bus.positions
    start, s = p["axis0"], p["axis0"] + p["axis1"]  # S stays constant
    for x in [start + TRAVEL_RAD, start - TRAVEL_RAD] * CYCLES + [start]:
        bus.move_to(
            {"axis0": x, "axis1": s - x},
            velocity_scale=SCALE,
            acceleration_scale=SCALE,
            # Jerk-limited: the acceleration ramps in over 0.1 s instead of stepping.
            profile=SCurve(),
        )
    print(
        f"done: {CYCLES} times ±{TRAVEL_RAD} rad, back at axis0 {bus.positions['axis0']:.3f} rad"
    )
