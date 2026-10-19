# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem.drive.profile against Ruckig (ruckig_reference.cpp), thousands of random moves.

SCurve is compared with Ruckig exactly: duration, every sample through the
spline the JTC fits between our points, our points on Ruckig's trajectory, and
each Ruckig segment ending at one of our points.

Ruckig 0.9.2 has no infinite jerk, so Trapezoid is compared with Ruckig at a
jerk that reaches each axis' acceleration in EPS seconds (the jerk-limited
optimum is then within a few EPS of the trapezoid), and its exact 1-D values
with an independent oracle that inverts distance(duration) by bisection.

Ruckig's Phase synchronisation keeps the straight line only when the limiting
axis' scaled profile fits every other axis; otherwise it falls back to time
synchronisation (axes leave the line and the move can be shorter). We always
keep the line, so for those cases we only require that our duration is not
shorter than Ruckig's.
"""

import math
import os
import random
import subprocess
import sys
from pathlib import Path
from typing import NamedTuple

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_profile import sample, times  # noqa: E402

from plem.drive import Limits, Scale, SCurve, Trapezoid  # noqa: E402

REFERENCE = os.environ.get("PLEM_RUCKIG_REFERENCE")

SAMPLES = 40
EPS = 1e-6  # Trapezoid against Ruckig: jerk ramp time in s
# Ruckig's profiles are numerical (its own checks hold positions to ~1e-8 and
# times to ~1e-12 s); ours are closed form with points at whole nanoseconds.
# Errors are measured relative to each axis's distance, velocity limit and
# acceleration limit (TOL_REL) plus what TOL_TIME of time moves that quantity
# (velocity, acceleration, jerk times 1 ns); times to TOL_TIME + TOL_REL * T.
TOL_REL = 1e-8
TOL_TIME = 1e-9


class Case(NamedTuple):
    profile: object
    start: list
    goal: list
    vel: list  # scaled limits per axis, as given to Ruckig
    acc: list
    jerk: list
    traj: object


def random_cases(rng, n_single, n_multi):
    """SCurve and Trapezoid cases spanning distances, limits and both reduced regimes."""
    cases = []
    for dofs in [1] * n_single + [rng.randint(2, 6) for _ in range(n_multi)]:
        start, goal, limits = [], [], []
        for _ in range(dofs):
            s = rng.uniform(-10, 10)
            zero = dofs > 1 and rng.random() < 0.1
            start.append(s)
            goal.append(
                s if zero else s + rng.choice((-1, 1)) * 10 ** rng.uniform(-6, 2)
            )
            limits.append(Limits(10 ** rng.uniform(-1, 1.5), 10 ** rng.uniform(-1, 3)))
        if start == goal:
            continue
        scale = Scale(
            rng.choice((1.0, rng.uniform(0.05, 1))),
            rng.choice((1.0, rng.uniform(0.05, 1))),
        )
        jerk = rng.choice((None, 10 ** rng.uniform(-1, 4)))
        vel = [lm.velocity * scale.velocity for lm in limits]
        acc = [lm.acceleration * scale.acceleration for lm in limits]
        for profile, jerks in (
            (SCurve(jerk), [jerk or a / 0.1 for a in acc]),
            (Trapezoid(), [a / EPS for a in acc]),
        ):
            traj = profile.plan(start, goal, limits, scale)
            cases.append(Case(profile, start, goal, vel, acc, jerks, traj))
    return cases


def run_reference(cases):
    """Ruckig for every case in one process: None where it could not calculate."""
    lines = []
    for c in cases:
        axes = " ".join(
            f"{x!r}"
            for axis in zip(c.start, c.goal, c.vel, c.acc, c.jerk)
            for x in axis
        )
        ours = times(c.traj)
        lines.append(
            f"{len(c.start)} {axes} {SAMPLES} {len(ours)} " + " ".join(map(repr, ours))
        )
    out = subprocess.run(
        [REFERENCE],
        input="\n".join(lines) + "\n",
        capture_output=True,
        text=True,
        check=True,
        timeout=600,
    ).stdout.splitlines()
    refs, i = [], 0
    while i < len(out):
        head = out[i].split()
        i += 1
        if head[0] == "error":
            refs.append(None)
            continue
        assert head[0] == "case", out[i - 1]
        sizes = [int(x) for x in head[3:6]]
        rows = [[float(x) for x in line.split()] for line in out[i : i + sum(sizes)]]
        i += sum(sizes)
        bounds, samples, at_ours = (
            rows[: sizes[0]],
            rows[sizes[0] : -sizes[2]],
            rows[-sizes[2] :],
        )
        refs.append((float(head[1]), head[2] == "1", bounds, samples, at_ours))
    return refs


@pytest.mark.skipif(
    not REFERENCE,
    reason="PLEM_RUCKIG_REFERENCE (the ruckig_reference binary) is not set",
)
def test_matches_ruckig():
    cases = random_cases(random.Random(20261005), n_single=1500, n_multi=1500)
    refs = run_reference(cases)
    assert len(refs) == len(cases)
    counts = dict.fromkeys(
        (
            "scurve",
            "trapezoid",
            "scurve_off_line",
            "trapezoid_off_line",
            "ruckig_error",
        ),
        0,
    )
    worst = {}

    def check(key, err, tol):
        worst[key] = max(worst.get(key, 0.0), err / tol)
        assert err <= tol, key

    for c, ref in zip(cases, refs):
        name = type(c.profile).__name__.lower()
        T = times(c.traj)[-1]
        if ref is None:
            # only at the extreme jerk standing in for the trapezoid (a / EPS)
            assert name == "trapezoid", c
            counts["ruckig_error"] += 1
            continue
        T_ref, phase, bounds, samples, at_ours = ref
        if not phase:
            counts[f"{name}_off_line"] += 1
            assert T >= T_ref * (1 - TOL_REL) - TOL_TIME - (
                4 * EPS if name == "trapezoid" else 0
            ), c
            continue
        counts[name] += 1
        d = [g - s for s, g in zip(c.start, c.goal)]
        moving = [i for i, di in enumerate(d) if di]
        if name == "trapezoid":
            check("trapezoid_time", abs(T - T_ref), 4 * EPS + TOL_REL * T)
            for t, *state in samples:
                for i in moving:
                    p, v, _ = sample(c.traj, t, i)
                    check(
                        "trapezoid_pos",
                        abs(p - state[3 * i]),
                        4 * c.vel[i] * EPS + TOL_REL * abs(d[i]),
                    )
                    check(
                        "trapezoid_vel",
                        abs(v - state[3 * i + 1]),
                        4 * c.acc[i] * EPS + TOL_REL * c.vel[i],
                    )
            continue
        check("scurve_time", abs(T - T_ref), TOL_TIME + TOL_REL * T)
        for i in moving:
            # per axis: relative error, plus what 1 ns of time quantisation moves
            tol = (
                TOL_REL * abs(d[i]) + TOL_TIME * c.vel[i],
                TOL_REL * c.vel[i] + TOL_TIME * c.acc[i],
                TOL_REL * c.acc[i] + TOL_TIME * c.jerk[i],
            )
            # every Ruckig sample, through the quintic the JTC fits between our points
            for t, *state in samples:
                got = sample(c.traj, t, i)
                for k, what in enumerate(("pos", "vel", "acc")):
                    check(f"scurve_{what}", abs(got[k] - state[3 * i + k]), tol[k])
            # our points lie on Ruckig's trajectory
            for p, (t, *state) in zip(c.traj.points, at_ours):
                got = p.positions[i], p.velocities[i], p.accelerations[i]
                for k, what in enumerate(("pos", "vel", "acc")):
                    check(
                        f"scurve_point_{what}", abs(got[k] - state[3 * i + k]), tol[k]
                    )
        # every Ruckig segment longer than a nanosecond ends at one of our points
        ours = times(c.traj)
        for prev, b in zip(bounds, bounds[1:]):
            if b[0] - prev[0] > 1e-9:
                check(
                    "scurve_boundary_time",
                    min(abs(t - b[0]) for t in ours),
                    0.5e-9 + TOL_REL * T,
                )
    print(
        f"\ncompared: {counts}\nworst error / tolerance: { {k: f'{v:.3g}' for k, v in worst.items()} }"
    )
    assert counts["scurve"] > 2000 and counts["trapezoid"] > 2000
    assert counts["ruckig_error"] < 0.01 * len(cases)


def oracle_trapezoid_duration(distance, v, a):
    """Shortest T whose best reachable distance covers `distance`, by bisection."""

    def reach(T):
        return v * (T - v / a) if T >= 2 * v / a else a * T * T / 4

    lo, hi = 0.0, 1.0
    while reach(hi) < distance:
        hi *= 2
    for _ in range(200):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if reach(mid) < distance else (lo, mid)
    return hi


def test_trapezoid_matches_an_independent_oracle():
    rng = random.Random(1005)
    for _ in range(3000):
        start = rng.uniform(-10, 10)
        d = rng.choice((-1, 1)) * 10 ** rng.uniform(-6, 2)
        lim = Limits(10 ** rng.uniform(-1, 1.5), 10 ** rng.uniform(-1, 3))
        traj = Trapezoid().plan([start], [start + d], [lim], Scale())
        T = oracle_trapezoid_duration(abs(d), lim.velocity, lim.acceleration)
        assert times(traj)[-1] == pytest.approx(T, rel=TOL_REL, abs=TOL_TIME)
        # the velocity is the lowest of: speeding up at a, the peak, slowing down at a
        peak = min(lim.velocity, lim.acceleration * T / 2)
        for k in range(SAMPLES + 1):
            t = T * k / SAMPLES
            _, v, _ = sample(traj, t, 0)
            expect = math.copysign(
                min(lim.acceleration * t, peak, lim.acceleration * (T - t)), d
            )
            assert v == pytest.approx(
                expect, abs=TOL_REL * lim.velocity + lim.acceleration * TOL_TIME
            )
