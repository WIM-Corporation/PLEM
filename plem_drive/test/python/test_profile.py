# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem.drive.profile — rest-to-rest plans as the JointTrajectory a JTC executes."""

import bisect
import math
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from plem.drive import Limits, Profile, Scale, SCurve, Trapezoid  # noqa: E402


def times(traj):
    return [
        p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in traj.points
    ]


def column(traj, field, axis=0):
    return [getattr(p, field)[axis] for p in traj.points]


def sample(traj, t, axis):
    """(p, v, a) of one axis at t, interpolated as the JTC does between points:
    quintic when both points carry accelerations, cubic otherwise."""
    ts = times(traj)
    pts = traj.points
    if t >= ts[-1]:
        return pts[-1].positions[axis], 0.0, 0.0
    k = bisect.bisect_right(ts, t) - 1
    T, x = ts[k + 1] - ts[k], t - ts[k]
    p0, v0, p1, v1 = (
        pts[k].positions[axis],
        pts[k].velocities[axis],
        pts[k + 1].positions[axis],
        pts[k + 1].velocities[axis],
    )
    if pts[k].accelerations and pts[k + 1].accelerations:
        a0, a1 = pts[k].accelerations[axis], pts[k + 1].accelerations[axis]
        c = [
            p0, v0, a0 / 2,
            (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T**2) / (2 * T**3),
            (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T**2) / (2 * T**4),
            (12 * (p1 - p0) - 6 * (v1 + v0) * T + (a1 - a0) * T**2) / (2 * T**5),
        ]  # fmt: skip
    else:
        c = [
            p0,
            v0,
            (3 * (p1 - p0) - (2 * v0 + v1) * T) / T**2,
            (2 * (p0 - p1) + (v0 + v1) * T) / T**3,
        ]
    p = sum(ci * x**i for i, ci in enumerate(c))
    v = sum(i * ci * x ** (i - 1) for i, ci in enumerate(c) if i >= 1)
    a = sum(i * (i - 1) * ci * x ** (i - 2) for i, ci in enumerate(c) if i >= 2)
    return p, v, a


def test_trapezoid_with_cruise():
    # 1 rad, v=1, a=2: accelerate 0.5 s (0.25 rad), cruise 0.5 s, decelerate 0.5 s.
    traj = Trapezoid().plan(
        [0.0], [1.0], [Limits(velocity=1.0, acceleration=2.0)], Scale()
    )
    assert times(traj) == pytest.approx([0.0, 0.5, 1.0, 1.5])
    assert column(traj, "positions") == pytest.approx([0.0, 0.25, 0.75, 1.0])
    assert column(traj, "velocities") == pytest.approx([0.0, 1.0, 1.0, 0.0])
    # accelerations jump at every boundary, so the points leave them to the cubic
    assert all(not p.accelerations for p in traj.points)


def test_trapezoid_triangle_when_the_cruise_is_not_reached():
    # 1 rad, v=10, a=1: half the distance in sqrt(0.5 / 0.5) = 1 s, peak 1 rad/s.
    traj = Trapezoid().plan(
        [2.0], [1.0], [Limits(velocity=10.0, acceleration=1.0)], Scale()
    )
    assert times(traj) == pytest.approx([0.0, 1.0, 2.0])
    assert column(traj, "positions") == pytest.approx([2.0, 1.5, 1.0])
    assert column(traj, "velocities") == pytest.approx([0.0, -1.0, 0.0])


def test_scurve_full_seven_segments():
    # 1 rad, v=1, a=2, j=10. Jerk ramp tj = a/j = 0.2 s, constant acceleration
    # ta = v/a - tj = 0.3 s, so speeding up takes 0.7 s and covers v*0.7/2 = 0.35 rad;
    # the cruise covers 1 - 2*0.35 = 0.3 rad in 0.3 s. T = 0.7 + 0.3 + 0.7 = 1.7 s.
    # p(0.2) = j tj^3/6 = 1/75; p(0.5) = 1/75 + 0.2*0.3 + 2*0.3^2/2 = 0.163333...
    traj = SCurve(jerk=10.0).plan(
        [0.0], [1.0], [Limits(velocity=1.0, acceleration=2.0)], Scale()
    )
    assert times(traj) == pytest.approx([0.0, 0.2, 0.5, 0.7, 1.0, 1.2, 1.5, 1.7])
    assert column(traj, "positions") == pytest.approx(
        [0.0, 1 / 75, 0.49 / 3, 0.35, 0.65, 1 - 0.49 / 3, 1 - 1 / 75, 1.0]
    )
    assert column(traj, "velocities") == pytest.approx([0, 0.2, 0.8, 1, 1, 0.8, 0.2, 0])
    assert column(traj, "accelerations") == pytest.approx([0, 2, 2, 0, 0, -2, -2, 0])


def test_scurve_acceleration_reached_cruise_not():
    # 1 rad, v=10, a=2, j=10: tj = 0.2 s; the peak w solves w^2/a + w a/j = 1, i.e.
    # w^2 + 0.4 w - 2 = 0, w = sqrt(2.04) - 0.2. T = 2 (2 tj + w/a - tj) = w + 0.4.
    w = math.sqrt(2.04) - 0.2
    traj = SCurve(jerk=10.0).plan(
        [0.0], [1.0], [Limits(velocity=10.0, acceleration=2.0)], Scale()
    )
    assert times(traj)[-1] == pytest.approx(w + 0.4)
    assert len(traj.points) == 7  # no cruise boundary
    assert max(column(traj, "velocities")) == pytest.approx(w)
    assert max(column(traj, "accelerations")) == pytest.approx(2.0)


def test_scurve_neither_acceleration_nor_cruise_reached():
    # 0.1 rad, v=a=j=10: 2 a^3/j^2 = 20 > 0.1, so four jerk ramps of tj with
    # 0.1 = 2 j tj^3: tj = 0.005^(1/3), peak acceleration j tj < a.
    tj = 0.005 ** (1 / 3)
    traj = SCurve(jerk=10.0).plan(
        [0.0], [0.1], [Limits(velocity=10.0, acceleration=10.0)], Scale()
    )
    assert times(traj) == pytest.approx([0, tj, 2 * tj, 3 * tj, 4 * tj])
    # points sit at whole nanoseconds: the state there, j * 0.5 ns off a boundary at most
    assert column(traj, "accelerations") == pytest.approx(
        [0, 10 * tj, 0, -10 * tj, 0], abs=1e-8
    )
    assert column(traj, "positions")[2] == pytest.approx(0.05)


def test_scurve_default_jerk_reaches_the_acceleration_in_a_tenth_of_a_second():
    traj = SCurve().plan(
        [0.0], [10.0], [Limits(velocity=1.0, acceleration=2.0)], Scale()
    )
    assert times(traj)[1] == pytest.approx(0.1)
    assert traj.points[1].accelerations[0] == pytest.approx(2.0)


AXES = (
    [0.5, -1.0, 2.0, 0.3],
    [2.0, -1.0, -1.0, 0.31],
    [Limits(1.0, 2.0), Limits(0.5, 9.0), Limits(3.0, 1.5), Limits(0.01, 0.01)],
)


@pytest.mark.parametrize("profile", [Trapezoid(), SCurve(), SCurve(jerk=3.0)])
def test_all_axes_move_on_one_line_and_end_together(profile):
    start, goal, limits = AXES
    traj = profile.plan(start, goal, limits, Scale())
    assert traj.joint_names == []
    d = [g - s for s, g in zip(start, goal)]
    for p in traj.points:
        # axis 1 does not move; the others are at the same fraction of their distance
        assert p.positions[1] == -1.0 and p.velocities[1] == 0.0
        fractions = [(p.positions[i] - start[i]) / d[i] for i in (0, 2, 3)]
        assert fractions == pytest.approx([fractions[0]] * 3, abs=1e-12)
    assert list(traj.points[-1].positions) == goal
    assert list(traj.points[-1].velocities) == [0.0] * 4


@pytest.mark.parametrize("profile", [Trapezoid(), SCurve(), SCurve(jerk=3.0)])
@pytest.mark.parametrize("scale", [Scale(), Scale(0.3, 0.7), Scale(1.0, 0.05)])
def test_limits_hold_between_points_and_one_axis_reaches_one(profile, scale):
    start, goal, limits = AXES
    traj = profile.plan(start, goal, limits, scale)
    end = times(traj)[-1]
    peak_v, peak_a = [0.0] * 4, [0.0] * 4
    for k in range(4001):
        for i in range(4):
            _, v, a = sample(traj, end * k / 4000, i)
            peak_v[i], peak_a[i] = max(peak_v[i], abs(v)), max(peak_a[i], abs(a))
    use_v = [peak_v[i] / (limits[i].velocity * scale.velocity) for i in range(4)]
    use_a = [
        peak_a[i] / (limits[i].acceleration * scale.acceleration) for i in range(4)
    ]
    # boundary times are whole nanoseconds; the spline over a rounded interval
    # differs from the exact segment by parts per billion
    assert max(use_v + use_a) <= 1 + 1e-6
    # time-optimal on the line: some axis is at a limit (a sampled peak may miss it slightly)
    assert max(use_v + use_a) > 0.99


def test_scale_stretches_the_move():
    one = [Limits(velocity=1.0, acceleration=2.0)]
    # v halved: cruise reached at 0.5 rad/s, accelerate 0.25 s, cruise (1 - 0.125)/0.5 s
    traj = Trapezoid().plan([0.0], [1.0], one, Scale(velocity=0.5))
    assert times(traj)[-1] == pytest.approx(0.25 + 1.75 + 0.25)
    # a quartered: 0.5 rad/s^2, accelerate 2 s to 1 rad/s over 1 rad > half the move: triangle 2 sqrt(2)
    traj = Trapezoid().plan([0.0], [1.0], one, Scale(acceleration=0.25))
    assert times(traj)[-1] == pytest.approx(2 * math.sqrt(2))


@pytest.mark.parametrize("profile", [Trapezoid(), SCurve()])
def test_no_axis_moves_gives_the_start_point_alone(profile):
    traj = profile.plan([1.0, 2.0], [1.0, 2.0], [Limits(1, 1)] * 2, Scale())
    assert len(traj.points) == 1
    assert times(traj) == [0.0]
    assert list(traj.points[0].positions) == [1.0, 2.0]
    assert list(traj.points[0].velocities) == [0.0, 0.0]


@pytest.mark.parametrize("profile", [Trapezoid(), SCurve(jerk=1e12)])
def test_a_move_shorter_than_a_nanosecond_still_ends_after_the_start(profile):
    traj = profile.plan([0.0], [1e-30], [Limits(1e3, 1e6)], Scale())
    assert [p.time_from_start.nanosec for p in traj.points] == [0, 1]
    assert list(traj.points[-1].positions) == [1e-30]


def test_profile_is_the_extension_point():
    class Hold(Profile):
        def plan(self, start, goal, limits, scale):
            return Trapezoid().plan(start, start, limits, scale)

    assert len(Hold().plan([0.0], [1.0], [Limits(1, 1)], Scale()).points) == 1
    with pytest.raises(TypeError):
        Profile()


@pytest.mark.parametrize(
    "make, words",
    [
        (lambda: Limits(velocity=0.0, acceleration=1.0), "Limits.velocity"),
        (lambda: Limits(velocity=1.0, acceleration=math.inf), "Limits.acceleration"),
        (lambda: Scale(velocity=1.5), "Scale.velocity"),
        (lambda: Scale(acceleration=0.0), "Scale.acceleration"),
        (lambda: SCurve(jerk=-1.0), "jerk"),
        (
            lambda: Trapezoid().plan([0.0], [1.0, 2.0], [Limits(1, 1)], Scale()),
            "one entry per axis",
        ),
        (
            lambda: Trapezoid().plan([math.nan], [1.0], [Limits(1, 1)], Scale()),
            "start[0]",
        ),
        (lambda: SCurve().plan([0.0], ["1"], [Limits(1, 1)], Scale()), "goal[0]"),
        (lambda: SCurve().plan([0.0], [1.0], [(1, 1)], Scale()), "limits[0]"),
    ],
)
def test_bad_input_is_refused_with_what_and_how(make, words):
    with pytest.raises(ValueError, match=re.escape(words)):
        make()
