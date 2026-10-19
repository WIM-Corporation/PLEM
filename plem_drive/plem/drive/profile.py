# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""Rest-to-rest motion profiles, planned host-side as a JointTrajectory.

Every axis moves on one straight line in axis space (phase synchronisation):
the move is planned once on a path parameter s in [0, 1] and axis i follows
start_i + d_i * s, so all axes start, peak and stop together. The limits on s
are the tightest of the moving axes (v_i / |d_i|, a_i / |d_i|, j_i / |d_i|).

Only the segment boundaries are emitted. Between two points the trajectory
controller fits a spline through the end states, and that spline reproduces
each segment exactly (up to the nanosecond time grid, see _plan):
- Trapezoid: the acceleration jumps at a boundary, so one point cannot carry
  both sides of it. Its points leave accelerations empty; the controller then
  uses a cubic through positions and velocities, which is exact for the
  constant-acceleration (parabola) and cruise (line) segments.
- SCurve: the acceleration is continuous, so its points carry it; the quintic
  through positions, velocities and accelerations is exact for each
  constant-jerk (cubic) segment.
"""

from __future__ import annotations

import abc
import math
from dataclasses import dataclass
from typing import Sequence

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass(frozen=True)
class Limits:
    """One axis' limits: velocity in rad/s, acceleration in rad/s^2 (both > 0)."""

    velocity: float
    acceleration: float

    def __post_init__(self):
        for name in ("velocity", "acceleration"):
            value = getattr(self, name)
            if not (
                isinstance(value, (int, float)) and math.isfinite(value) and value > 0
            ):
                raise ValueError(
                    f"Limits.{name} must be a finite number > 0, got {value!r}"
                )


@dataclass(frozen=True)
class Scale:
    """Fractions of the limits a move uses, each in (0, 1]."""

    velocity: float = 1.0
    acceleration: float = 1.0

    def __post_init__(self):
        for name in ("velocity", "acceleration"):
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and 0 < value <= 1):
                raise ValueError(f"Scale.{name} must be in (0, 1], got {value!r}")


# A 1-D plan: (duration, acceleration at the segment's start, jerk) per segment,
# starting at rest at s = 0.
Segments = list[tuple[float, float, float]]


def trapezoid_1d(distance: float, v: float, a: float) -> Segments:
    """Time-optimal rest-to-rest move over distance > 0 under |v| and |a|."""
    if distance * a >= v * v:  # the cruise velocity is reached
        ta, tc = v / a, distance / v - v / a
    else:  # triangle
        ta, tc = math.sqrt(distance / a), 0.0
    return [(ta, a, 0.0), (tc, 0.0, 0.0), (ta, -a, 0.0)]


def scurve_1d(distance: float, v: float, a: float, j: float) -> Segments:
    """Time-optimal rest-to-rest move over distance > 0 under |v|, |a| and |j|."""

    # Accelerating to a velocity w takes a jerk ramp tj and a constant-acceleration
    # part ta: tj = sqrt(w / j), ta = 0 when w * j <= a^2 (a not reached), else
    # tj = a / j, ta = w / a - a / j. Speeding up and slowing down cover w * (2 tj + ta).
    def ramp(w: float) -> tuple[float, float]:
        return (math.sqrt(w / j), 0.0) if w * j <= a * a else (a / j, w / a - a / j)

    tj, ta = ramp(v)
    tc = (distance - v * (2 * tj + ta)) / v
    if tc < 0:  # the cruise velocity is not reached: solve for the peak w
        if distance * j * j >= 2 * a**3:  # a is reached: w^2 / a + w a / j = distance
            w = 0.5 * a * (math.sqrt(a * a / (j * j) + 4 * distance / a) - a / j)
        else:  # a is not reached: distance = 2 j tj^3
            w = j * (distance / (2 * j)) ** (2 / 3)
        (tj, ta), tc = ramp(w), 0.0
    peak = j * tj
    return [
        (tj, 0.0, j), (ta, peak, 0.0), (tj, peak, -j),
        (tc, 0.0, 0.0),
        (tj, 0.0, -j), (ta, -peak, 0.0), (tj, -peak, j),
    ]  # fmt: skip


class Profile(abc.ABC):
    """How a rest-to-rest move is timed.

    Subclass it and pass an instance to ``move_to(profile=)`` or set it as
    ``bus.default_profile``. ``plan`` returns a JointTrajectory starting at
    ``start`` at t = 0 and ending at rest at ``goal``; joint_names stay empty
    (the caller fills them). Each point carries one position and one velocity
    per axis (accelerations: one per axis or none), with strictly increasing
    times. Whatever the profile, the caller checks every point, and the speed
    change between points, against the drive file's limits before sending it;
    it does not sample the spline between points.
    """

    @abc.abstractmethod
    def plan(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        limits: Sequence[Limits],
        scale: Scale,
    ) -> JointTrajectory: ...


class Trapezoid(Profile):
    """Acceleration- and velocity-limited, no jerk limit (the default)."""

    def plan(self, start, goal, limits, scale):
        def plan_s(d, lim):
            return trapezoid_1d(
                1.0,
                _tightest(d, [lm.velocity * scale.velocity for lm in lim]),
                _tightest(d, [lm.acceleration * scale.acceleration for lm in lim]),
            )

        return _plan(start, goal, limits, plan_s, with_acceleration=False)


class SCurve(Profile):
    """Jerk-limited (7 segments). By default each axis reaches its acceleration in 0.1 s."""

    def __init__(self, jerk: float | None = None):
        if jerk is not None and not (
            isinstance(jerk, (int, float)) and math.isfinite(jerk) and jerk > 0
        ):
            raise ValueError(
                f"SCurve jerk must be a finite number > 0 in rad/s^3 or None, got {jerk!r}"
            )
        self.jerk = jerk

    def plan(self, start, goal, limits, scale):
        def plan_s(d, lim):
            acc = [lm.acceleration * scale.acceleration for lm in lim]
            jerk = [self.jerk or ai / 0.1 for ai in acc]
            return scurve_1d(
                1.0,
                _tightest(d, [lm.velocity * scale.velocity for lm in lim]),
                _tightest(d, acc),
                _tightest(d, jerk),
            )

        return _plan(start, goal, limits, plan_s, with_acceleration=True)


def _plan(start, goal, limits, plan_s, with_acceleration: bool) -> JointTrajectory:
    start, goal = _check(start, goal, limits)
    d = [g - s for s, g in zip(start, goal)]
    moving = [i for i, di in enumerate(d) if di != 0]
    traj = JointTrajectory()

    def point(ns: int, s: float, sv: float, sa: float) -> JointTrajectoryPoint:
        p = JointTrajectoryPoint(
            positions=[s0 + di * s for s0, di in zip(start, d)],
            velocities=[di * sv for di in d],
            time_from_start=Duration(
                sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000
            ),
        )
        if with_acceleration:
            p.accelerations = [di * sa for di in d]
        return p

    traj.points.append(point(0, 0.0, 0.0, 0.0))
    if not moving:  # nothing to move: the start point alone
        return traj
    segments = [
        g
        for g in plan_s([d[i] for i in moving], [limits[i] for i in moving])
        if g[0] > 0
    ]
    # Each segment's polynomial: (start time, s, ds/dt, d2s/dt2, jerk) at its start.
    pieces, t, s, v = [], 0.0, 0.0, 0.0
    for duration, a, j in segments:
        pieces.append((t, s, v, a, j))
        t += duration
        s, v, _ = _extend(pieces[-1], t)
    # Times are whole nanoseconds, so a point sits up to 0.5 ns off its boundary.
    # It takes the state of the longer neighbouring segment's polynomial extended
    # to that time: the spline over the long segment stays exact and the error
    # (about jerk * 0.5 ns in acceleration) is confined to the short one. A
    # boundary that rounds onto an earlier point's time is dropped.
    last = max(round(t * 1e9), 1)
    for k in range(len(segments) - 1):
        ns = round(pieces[k + 1][0] * 1e9)
        if _ns(traj.points[-1]) < ns < last:
            longer = k if segments[k][0] >= segments[k + 1][0] else k + 1
            traj.points.append(point(ns, *_extend(pieces[longer], ns * 1e-9)))
    end = point(last, 1.0, 0.0, 0.0)
    end.positions = list(goal)  # exactly, not start + d
    traj.points.append(end)
    return traj


def _extend(
    piece: tuple[float, float, float, float, float], t: float
) -> tuple[float, float, float]:
    """(s, ds/dt, d2s/dt2) at time t of one segment's polynomial."""
    t0, s, v, a, j = piece
    x = t - t0
    return s + x * (v + x * (a / 2 + x * j / 6)), v + x * (a + x * j / 2), a + x * j


def _ns(p: JointTrajectoryPoint) -> int:
    return p.time_from_start.sec * 1_000_000_000 + p.time_from_start.nanosec


def _tightest(d: list[float], per_axis: list[float]) -> float:
    """The limit on s: an axis moving d_i under limit x_i allows ds/dt^k <= x_i / |d_i|."""
    return min(x / abs(di) for di, x in zip(d, per_axis))


def _check(start, goal, limits) -> tuple[list[float], list[float]]:
    if not (len(start) == len(goal) == len(limits)):
        raise ValueError(
            f"start ({len(start)}), goal ({len(goal)}) and limits ({len(limits)}) must have "
            "one entry per axis; pass the same axes in the same order to each"
        )
    for name, values in (("start", start), ("goal", goal)):
        for i, x in enumerate(values):
            if not (isinstance(x, (int, float)) and math.isfinite(x)):
                raise ValueError(
                    f"{name}[{i}] must be a finite number in rad, got {x!r}"
                )
    for i, lm in enumerate(limits):
        if not isinstance(lm, Limits):
            raise ValueError(f"limits[{i}] must be a Limits, got {lm!r}")
    return [float(x) for x in start], [float(x) for x in goal]
