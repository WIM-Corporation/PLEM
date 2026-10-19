# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem.drive.Bus against a fake bus graph: what a user's call sends, waits for
and refuses. The fake is the drive bus's public ROS surface as `plem drive run`
starts it — the robot description, joint states, the drive status, the
control command service, the trajectory controller's action and the
controller manager's list and switch, and the stream controller's commands topic — served in-process."""

import os
import pathlib
import sys
import threading
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from plem_drive_msgs.msg import DriveStatus
from plem_rt_msgs.msg import RtEvent
from plem_rt_msgs.srv import SetControlCommand
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from plem.drive import Bus, DriveError, SCurve, Trapezoid  # noqa: E402
from plem.drive.bus import STATUS_SILENCE_S  # noqa: E402
from plem.drive.cli import run  # noqa: E402
from ros_test_domain import private_domain_id, wait_until  # noqa: E402

# Off the user's live domain: the fake bus and the client both read it at init.
os.environ["ROS_DOMAIN_ID"] = private_domain_id()

NS = "testbus"


def description(fault_reset_policy="none"):
    axes = [
        {
            "name": "a0",
            "mode": "csp",
            "position_min_rad": -1.0,
            "position_max_rad": 1.0,
            "velocity_limit_rad_s": 2.0,
            "velocity_cap_rad_s": 2.0,
            "acceleration_limit_rad_s2": 4.0,
        },
        {
            "name": "a1",
            "mode": "csp",
            "position_min_rad": None,
            "position_max_rad": None,
            "velocity_limit_rad_s": 1.0,
            "velocity_cap_rad_s": 1.0,
            "acceleration_limit_rad_s2": 2.0,
        },
        {
            "name": "t",
            "mode": "cst",
            "position_min_rad": -1.0,
            "position_max_rad": 1.0,
            "velocity_limit_rad_s": 1.0,
            "velocity_cap_rad_s": 1.0,
            "acceleration_limit_rad_s2": None,
            "torque_min_nm": -3.0,
            "torque_max_nm": 4.0,
        },
        {
            "name": "v",
            "mode": "csv",
            "position_min_rad": -1.0,
            "position_max_rad": 1.0,
            "velocity_limit_rad_s": 1.0,
            "velocity_cap_rad_s": 1.0,
            "acceleration_limit_rad_s2": 2.0,
        },
    ]
    return run.robot_description(
        {"axes": axes, "fault_reset_policy": fault_reset_policy}, "/m/testbus.yaml"
    )


class FakeBus:
    """The bus side. `positions` are what joint_states reports."""

    def __init__(self, urdf=None):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("fake_bus", namespace=NS, context=self.context)
        self.positions = {"a0": 0.1, "a1": 5.0, "t": 0.0, "v": 0.0}
        self.velocities = {"a0": 0.0, "a1": 0.0, "t": 0.0, "v": 0.0}
        self.silent = False  # True: the bus went away (nothing is published)
        self.joints_silent = False  # True: only joint_states stopped
        self.status = DriveStatus(brake_engaged=0b1111)
        self.axes_mask = 0b1111  # the brakes a stop engages: one bit per axis
        self.goals = []
        self.streamed = []
        self.commands = []
        self.switches = []
        # The trajectory controller as the manager lists it: None until its
        # spawner has loaded it (`plem drive run` starts the spawners in parallel).
        self.trajectory_state = "active"
        self.order = []
        self.goal_result = FollowJointTrajectory.Result.SUCCESSFUL
        self.arrive = True  # a successful goal moves joint_states to its end
        self.land = {}  # positions a successful goal leaves, over its end
        self.hold_goal = threading.Event()
        self.release_ok = True
        self.reset_clears = True
        self.stale = False  # a silent stream's stop: re-asserted after every request
        self.fault_on_release = False  # a drive that faults when enabled
        self.refuse_release = None  # an RtEvent the bus emits instead of releasing
        # An earlier request's refusal, still in transit: published while the
        # release request is served.
        self.late_refusal = None
        # A refusal of two axes split across two requests: the first event
        # answers the first request, the second arrives while the retry is served.
        self.split_refusal = None
        self._held_refusal = None
        # The statuses an accepted release goes through, one per tick, the last
        # one standing (None: released at once; []: the status stays as it is).
        self.release_statuses = None
        self._script = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_publisher(String, "robot_description", latched).publish(
            String(data=urdf or description())
        )
        self.joint_pub = self.node.create_publisher(JointState, "joint_states", 10)
        self.status_pub = self.node.create_publisher(
            DriveStatus, "plem_drive/status", latched
        )
        self.event_pub = self.node.create_publisher(
            RtEvent,
            "plem_drive/rt_events",
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.node.create_service(
            SetControlCommand, "plem_drive/control_command", self.on_command
        )
        self.node.create_service(
            SwitchController, "controller_manager/switch_controller", self.on_switch
        )
        self.node.create_service(
            ListControllers, "controller_manager/list_controllers", self.on_list
        )
        ActionServer(
            self.node,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            self.on_goal,
            cancel_callback=self.on_cancel,
        )
        self.node.create_subscription(
            Float64MultiArray,
            "stream_controller/commands",
            lambda m: self.streamed.append(list(m.data)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.node.create_timer(0.01, self.tick)
        self.executor = MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def tick(self):
        if self.silent:
            return
        js = JointState(
            name=list(self.positions),
            position=list(self.positions.values()),
            velocity=list(self.velocities.values()),
        )
        if not self.joints_silent:
            self.joint_pub.publish(js)
        if self._script:
            self.status = self._script.pop(0)
        self.status.header.stamp = self.node.get_clock().now().to_msg()
        self.status_pub.publish(self.status)

    def on_command(self, request, response):
        self.commands.append(request)
        if request.protective_stop:
            self.order.append("stop")
        if request.release_brake:
            response.success = self.release_ok
            response.message = (
                "release_brake: requested"
                if self.release_ok
                else "release_brake: refused (P-STOP)"
            )
            if self.split_refusal is not None:
                first, self._held_refusal = self.split_refusal
                self.split_refusal = None
                self.event_pub.publish(first)
                response.success = True
                return response
            if self._held_refusal is not None:
                self.event_pub.publish(self._held_refusal)
                self._held_refusal = None
            if self.refuse_release is not None:
                # accepted, then refused by the loop: the brakes stay, nothing is
                # pending, and the reason is on rt_events only
                self.refuse_release.header.stamp = self.node.get_clock().now().to_msg()
                self.event_pub.publish(self.refuse_release)
            elif self.release_ok:
                if self.late_refusal is not None:
                    self.late_refusal.header.stamp = (
                        self.node.get_clock().now().to_msg()
                    )
                    self.event_pub.publish(self.late_refusal)
                if self.release_statuses is None:
                    self.status = DriveStatus(operation_enabled=True, brake_engaged=0)
                else:
                    self._script = list(self.release_statuses)
            if self.fault_on_release:
                self.status = DriveStatus(
                    brake_engaged=0b1111, statusword=[0x0250, 0x0238, 0x0250, 0x0250]
                )
        if request.protective_stop:
            response.success = True
            self.status = DriveStatus(brake_engaged=self.axes_mask)
        if request.reset_error:
            response.success = True
            if self.reset_clears:
                self.status = DriveStatus(brake_engaged=0b1111, statusword=[0x0250] * 4)
        if self.stale:
            self.status = DriveStatus(brake_engaged=0b1111, protective_stop=2)
        return response

    def on_cancel(self, _):
        self.order.append("cancel")
        return CancelResponse.ACCEPT

    def on_list(self, _, response):
        if self.trajectory_state is not None:
            response.controller = [
                ControllerState(
                    name="joint_trajectory_controller", state=self.trajectory_state
                )
            ]
        return response

    def on_switch(self, request, response):
        self.switches.append(
            (list(request.deactivate_controllers), list(request.activate_controllers))
        )
        # The manager refuses a switch naming a controller it has not loaded.
        response.ok = self.trajectory_state is not None
        return response

    def on_goal(self, handle):
        self.goals.append(handle.request.trajectory)
        while self.hold_goal.is_set() and not handle.is_cancel_requested:
            time.sleep(0.01)
        result = FollowJointTrajectory.Result()
        if handle.is_cancel_requested:
            handle.canceled()
            return result
        result.error_code = self.goal_result
        if self.goal_result == FollowJointTrajectory.Result.SUCCESSFUL:
            handle.succeed()
            if self.arrive:
                last = handle.request.trajectory.points[-1].positions
                self.positions.update(zip(handle.request.trajectory.joint_names, last))
            self.positions.update(self.land)
        else:
            result.error_string = "path tolerance violated"
            handle.abort()
        return result

    def close(self):
        self.executor.shutdown()
        self.thread.join()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.fixture
def fake():
    bus = FakeBus()
    yield bus
    bus.close()


@pytest.fixture
def bus(fake):
    b = Bus(NS, timeout=5.0)
    yield b
    b.close()


def test_the_axes_and_their_positions_come_from_the_bus(bus):
    assert bus.axes == ("a0", "a1", "t", "v")
    assert bus["a0"].position == pytest.approx(0.1)
    assert bus.positions["a1"] == pytest.approx(5.0)
    with pytest.raises(KeyError, match="a9"):
        bus["a9"]


def test_release_restarts_the_trajectory_controller_then_releases_and_waits(bus, fake):
    bus.release()
    assert fake.switches == [
        (["joint_trajectory_controller"], []),
        ([], ["joint_trajectory_controller"]),
    ]
    assert [c.release_brake for c in fake.commands] == [True]


def test_release_waits_for_the_trajectory_controller_its_spawner_starts(bus, fake):
    # `plem drive run` reports the bus up before its spawners have started the
    # trajectory controller; a release in that window waits for it.
    fake.trajectory_state = None
    threading.Timer(0.2, setattr, (fake, "trajectory_state", "inactive")).start()
    threading.Timer(0.4, setattr, (fake, "trajectory_state", "active")).start()
    bus.release()
    assert fake.switches == [
        (["joint_trajectory_controller"], []),
        ([], ["joint_trajectory_controller"]),
    ]
    assert [c.release_brake for c in fake.commands] == [True]


def test_release_times_out_naming_the_trajectory_controller(fake):
    fake.trajectory_state = None
    with (
        Bus(NS, timeout=0.5) as b,
        pytest.raises(DriveError, match="joint_trajectory_controller to become active"),
    ):
        b.release()
    assert fake.switches == []
    assert not any(c.release_brake for c in fake.commands)


def test_a_refused_release_raises_with_the_bus_message(bus, fake):
    fake.release_ok = False
    with pytest.raises(DriveError, match="P-STOP"):
        bus.release()


def test_move_to_sends_one_planned_trajectory_and_returns_on_arrival(bus, fake):
    bus.release()
    bus.move_to({"a0": 0.5}, velocity_scale=0.5)
    (trajectory,) = fake.goals
    assert list(trajectory.joint_names) == ["a0", "a1"]
    first, last = trajectory.points[0], trajectory.points[-1]
    assert list(first.positions) == pytest.approx([0.1, 5.0])
    assert list(last.positions) == pytest.approx([0.5, 5.0]), (
        "an axis not named keeps its position"
    )
    assert max(abs(p.velocities[0]) for p in trajectory.points) == pytest.approx(1.0), (
        "half of 2 rad/s"
    )
    deadline = time.monotonic() + 2.0
    while bus["a0"].position != pytest.approx(0.5) and time.monotonic() < deadline:
        time.sleep(0.01)  # joint states are a topic: the next sample after arrival
    assert bus["a0"].position == pytest.approx(0.5)


def test_the_profile_is_the_buses_default_or_the_one_passed(bus, fake):
    bus.release()
    assert isinstance(bus.default_profile, Trapezoid)
    bus["a0"].move_to(0.3, profile=SCurve())
    assert len(fake.goals[0].points[0].accelerations) == 2, (
        "an S-curve carries accelerations"
    )
    bus.default_profile = SCurve()
    bus.move_to({"a0": 0.2})
    assert len(fake.goals[1].points[0].accelerations) == 2


def test_a_move_past_a_limit_is_refused_before_anything_moves(bus, fake):
    bus.release()
    with pytest.raises(ValueError, match="a0.*1.5.*limit"):
        bus.move_to({"a0": 1.5})
    with pytest.raises(ValueError, match="'t' is not a position axis"):
        bus.move_to({"t": 0.1})
    with pytest.raises(KeyError, match="a9"):
        bus.move_to({"a9": 0.1})
    assert fake.goals == []


class Overshoot(Trapezoid):
    """A user profile that plans too fast — the bus checks every point anyway."""

    def plan(self, start, goal, limits, scale):
        trajectory = super().plan(start, goal, limits, scale)
        for point in trajectory.points:
            point.velocities = [3.0 * v for v in point.velocities]
        return trajectory


def test_a_user_profile_is_checked_against_the_files_limits(bus, fake):
    bus.release()
    with pytest.raises(ValueError, match="a0.*velocity"):
        bus.move_to({"a0": 0.5}, profile=Overshoot())
    assert fake.goals == []


def test_a_failed_move_raises_with_the_controllers_reason(bus, fake):
    bus.release()
    fake.goal_result = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
    with pytest.raises(DriveError, match="path tolerance violated"):
        bus.move_to({"a0": 0.5})


def test_stop_cancels_the_move_asks_for_the_stop_and_waits_for_the_brakes(bus, fake):
    bus.release()
    fake.hold_goal.set()
    raised = []

    def mover():
        try:
            bus.move_to({"a0": 0.9})
        except DriveError as error:
            raised.append(str(error))

    thread = threading.Thread(target=mover)
    thread.start()
    while not fake.goals:
        time.sleep(0.01)
    bus.stop()
    thread.join(timeout=5.0)
    assert raised and "cancelled" in raised[0], "the move returned, cancelled"
    assert [c.protective_stop for c in fake.commands] == [False, True]
    # The stop first: it ramps the axes down while the plan is still running; a
    # cancel first would freeze the plan and leave the stop nothing to ramp.
    assert fake.order == ["stop", "cancel"]


def test_ctrl_c_during_a_move_in_a_with_block_cancels_the_move(fake, monkeypatch):
    # Leaving the block stops the axes; the stop must also end the running plan,
    # or the trajectory controller keeps it should the protective stop fail.
    fake.hold_goal.set()
    try:
        with pytest.raises(KeyboardInterrupt):
            with Bus(NS, timeout=5.0) as b:
                b.release()

                first = [b._reporting]  # move_to checks once before it sends the goal

                def interrupted():
                    if first:
                        first.pop()()
                        return
                    raise KeyboardInterrupt  # Ctrl-C while the move runs

                monkeypatch.setattr(b, "_reporting", interrupted)
                b.move_to({"a0": 0.9})
        assert wait_until(lambda: "cancel" in fake.order, 5.0), fake.order
        assert fake.order == ["stop", "cancel"]
    finally:
        fake.hold_goal.clear()


def test_a_move_without_a_release_is_refused(bus, fake):
    with pytest.raises(DriveError, match="release"):
        bus.move_to({"a0": 0.5})


class Hurried(Trapezoid):
    """A user profile that keeps every point's speed in bounds but squeezes the
    time between them — its acceleration is over the limit, though no point says so."""

    def plan(self, start, goal, limits, scale):
        trajectory = super().plan(start, goal, limits, scale)
        for point in trajectory.points:
            t = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) * 0.7
            point.time_from_start.sec, point.time_from_start.nanosec = (
                int(t),
                int((t - int(t)) * 1e9),
            )
            point.velocities = [v / 0.7 for v in point.velocities]
        return trajectory


def test_the_acceleration_between_points_is_checked_too(bus, fake):
    bus.release()
    with pytest.raises(ValueError, match="a0.*acceleration"):
        bus.move_to({"a0": 0.5}, velocity_scale=0.5, profile=Hurried())
    assert fake.goals == []


def test_a_plan_that_ran_out_while_the_bus_stopped_is_no_arrival(bus, fake):
    # The trajectory controller reports success when its plan's time is up — the
    # bus may have stopped on its own meanwhile (a limit, a lost stream).
    bus.release()
    fake.arrive = False
    fake.status = DriveStatus(protective_stop=1, brake_engaged=0b1111)
    with pytest.raises(DriveError, match="protective stop"):
        bus.move_to({"a0": 0.5})


class Malformed(Trapezoid):
    """A user profile whose plan is broken in the way `edit` says."""

    def __init__(self, edit):
        super().__init__()
        self.edit = edit

    def plan(self, start, goal, limits, scale):
        trajectory = super().plan(start, goal, limits, scale)
        self.edit(trajectory.points)
        return trajectory


def _nan_velocity(points):
    points[1].velocities[0] = float("nan")


def _no_velocities(points):
    points[1].velocities = []


def _short_positions(points):
    points[1].positions = points[1].positions[:1]


def _same_time(points):
    points[1].time_from_start = points[0].time_from_start


@pytest.mark.parametrize(
    ("edit", "message"),
    [
        (_nan_velocity, "not a finite number"),
        (_no_velocities, "velocities"),
        (_short_positions, "positions"),
        (_same_time, "not later than point 0"),
    ],
)
def test_a_malformed_user_plan_is_refused_before_anything_moves(
    bus, fake, edit, message
):
    bus.release()
    with pytest.raises(ValueError, match=message):
        bus.move_to({"a0": 0.5}, profile=Malformed(edit))
    assert fake.goals == []


def test_a_move_to_where_the_axes_are_is_one_point_and_is_sent(bus, fake):
    # Nothing to move: the plan is its start point alone, at t = 0.
    bus.release()
    bus.move_to({"a0": bus.positions["a0"]})
    assert len(fake.goals) == 1 and len(fake.goals[0].points) == 1


def test_a_position_that_is_not_a_number_is_no_arrival(fake):
    with Bus(NS, timeout=1.0) as bus:
        bus.release()
        fake.land = {"a0": float("nan")}
        try:
            with pytest.raises(DriveError, match="a0.*did not arrive"):
                bus.move_to({"a0": 0.5})
        finally:
            fake.land = {}
            fake.positions["a0"] = 0.5


def test_a_move_whose_joint_states_stopped_fails_promptly(fake):
    with Bus(NS, timeout=30.0) as bus:
        bus.release()
        fake.hold_goal.set()
        fake.joints_silent = True
        t0 = time.monotonic()
        try:
            with pytest.raises(DriveError, match="no joint states"):
                bus.move_to({"a0": 0.5})
            assert time.monotonic() - t0 < 10.0, "not the 30 s timeout past the plan"
        finally:
            fake.hold_goal.clear()
            fake.joints_silent = False


def test_a_plan_whose_axes_never_get_there_is_no_arrival(fake):
    with Bus(NS, timeout=3.0) as bus:
        bus.release()
        fake.arrive = False
        with pytest.raises(DriveError, match="a0.*did not arrive"):
            bus.move_to({"a0": 0.5})


def test_stream_sends_every_stream_axis_in_the_stream_controllers_order(bus, fake):
    # Before the release: a bus with a cst axis releases only on a live stream.
    # Best effort: a command sent before the subscription matched is lost, so
    # keep streaming (as a caller does) until one arrives.
    deadline = time.monotonic() + 2.0
    while not fake.streamed and time.monotonic() < deadline:
        bus.stream({"v": 0.5, "t": -1.0})
        time.sleep(0.01)
    assert fake.streamed[0] == [-1.0, 0.5], "the joints parameter order: t, then v"


def test_a_stream_command_past_a_limit_or_for_another_axis_is_refused(bus, monkeypatch):
    published = []  # what reaches the publisher, recorded in the calling thread
    monkeypatch.setattr(bus._stream, "publish", published.append)
    for commands, error, match in (
        ({"v": 1.5, "t": 0.0}, ValueError, "v: 1.5 rad/s .*limits.velocity_rad_s"),
        ({"v": 0.0, "t": 4.5}, ValueError, "t: 4.5 Nm .*limits.torque_nm"),
        ({"v": 0.0, "t": -3.5}, ValueError, "t: -3.5 Nm"),
        ({"v": float("nan"), "t": 0.0}, ValueError, "v: nan"),
        ({"v": 0.0}, ValueError, "no command for the stream axis 't'"),
        ({"v": 0.0, "t": 0.0, "a0": 0.1}, ValueError, "'a0' is a position axis"),
        ({"v": 0.0, "t": 0.0, "a9": 0.1}, KeyError, "a9"),
    ):
        with pytest.raises(error, match=match):
            bus.stream(commands)
    assert published == [], "nothing is sent"
    bus.stream({"v": 0.0, "t": 0.0})
    assert len(published) == 1, "a valid command is"


def edited(edit):
    """description() with `edit(ros2_control element)` applied."""
    robot = ET.fromstring(description())
    edit(robot.find("ros2_control"))
    return ET.tostring(robot, encoding="unicode")


def test_stream_on_a_bus_without_a_stream_axis_is_refused():
    def position_axes_only(hw):
        for joint in hw.findall("joint"):
            if joint.get("name") in ("t", "v"):
                hw.remove(joint)

    fake = FakeBus(urdf=edited(position_axes_only))
    fake.axes_mask = 0b11  # a0 and a1
    try:
        with Bus(NS, timeout=5.0) as bus:
            with pytest.raises(ValueError, match="no stream axis"):
                bus.stream({})
    finally:
        fake.close()


def test_a_cst_axis_from_an_older_run_asks_for_a_restart():
    # A robot description written before it carried the torque clamp.
    def without_torque(hw):
        for joint in hw.iter("joint"):
            for param in joint.findall("param"):
                if param.get("name").startswith("torque_"):
                    joint.remove(param)

    fake = FakeBus(urdf=edited(without_torque))
    try:
        with Bus(NS, timeout=5.0) as bus:
            with pytest.raises(DriveError, match="restart plem drive run"):
                bus.stream({"v": 0.0, "t": 0.0})
    finally:
        fake.close()


def test_velocities_and_status_are_the_buses_latest(bus, fake):
    fake.status = DriveStatus(
        operation_enabled=True, brake_engaged=0, protective_stop=0
    )
    fake.velocities["a0"] = 0.25
    deadline = time.monotonic() + 2.0
    while (
        not (bus.velocities["a0"] == 0.25 and bus.status.operation_enabled)
        and time.monotonic() < deadline
    ):
        time.sleep(0.01)
    assert bus.velocities["a0"] == 0.25 and bus["a0"].velocity == 0.25
    assert bus.status.operation_enabled and bus.status.brake_engaged == 0


def test_reset_clears_a_protective_stop_and_a_drive_fault_and_waits(bus, fake):
    fake.status = DriveStatus(
        brake_engaged=0b1111, protective_stop=2, statusword=[0x0238] * 4
    )
    assert wait_until(lambda: bus.status.protective_stop == 2)
    bus.reset()
    assert [c.reset_error for c in fake.commands] == [True]
    assert bus.status.protective_stop == 0


def test_a_reset_logs_the_faults_it_clears_with_the_drive_error_code(bus, fake, capfd):
    fake.status = DriveStatus(
        brake_engaged=0b1111, statusword=[0x0250, 0x0238, 0x0250, 0x0250]
    )
    fake.event_pub.publish(
        RtEvent(
            event_type=RtEvent.SERVO_FAULT,
            joint_id=1,
            error_code=0x0238,
            value=float(0xFF50),
        )
    )
    assert wait_until(
        lambda: list(bus.status.statusword) == [0x0250, 0x0238, 0x0250, 0x0250]
    )
    assert wait_until(lambda: bus._fault_codes.get(1) == 0xFF50)
    bus.reset()
    assert "a1 in fault (statusword 0x0238, error 0xff50)" in capfd.readouterr().err
    assert wait_until(lambda: not bus._fault_codes)  # cleared: the code is forgotten


def test_a_reset_that_does_not_clear_names_what_stands(bus, fake):
    fake.reset_clears = False
    fake.status = DriveStatus(
        brake_engaged=0b1111, protective_stop=2, statusword=[0x0250, 0x0238, 0, 0]
    )
    with (
        Bus(NS, timeout=0.5) as b,
        pytest.raises(DriveError, match=r"protective stop EXTERNAL.*a1 in fault"),
    ):
        b.reset()


def test_a_move_on_a_bus_that_stopped_reporting_fails_promptly(fake):
    # A timeout far past the 1 s silence keeps the two apart on a loaded machine.
    with Bus(NS, timeout=30.0) as bus:
        bus.release()
        fake.hold_goal.set()  # the move would last until the timeout
        fake.silent = True
        t0 = time.monotonic()
        try:
            with pytest.raises(DriveError, match="stopped reporting"):
                bus.move_to({"a0": 0.5})
            assert time.monotonic() - t0 < 10.0, "not the 30 s timeout past the plan"
        finally:
            fake.hold_goal.clear()
            fake.silent = False  # back, so leaving the block stops it


def test_a_move_on_a_bus_whose_feedback_went_stale_sends_no_goal(fake):
    # Positions a second old would plan a move the bus is not reporting on, and the
    # goal, once accepted, may run: refused before anything is sent.
    with Bus(NS, timeout=30.0) as bus:
        bus.release()
        fake.silent = True
        time.sleep(STATUS_SILENCE_S + 0.3)
        try:
            with pytest.raises(DriveError, match="stopped reporting"):
                bus.move_to({"a0": 0.5})
            assert fake.goals == [], "a goal was sent from stale feedback"
        finally:
            fake.silent = False  # back, so leaving the block stops it


@pytest.mark.parametrize("request_", ["release", "reset"])
def test_a_request_while_this_clients_stream_is_silent_fails_at_once(
    bus, fake, request_
):
    bus.stream({"v": 0.0, "t": 0.0})
    time.sleep(0.2)  # silent: the bus's stop stands
    fake.stale = True
    fake.status = DriveStatus(brake_engaged=0b1111, protective_stop=2)
    assert wait_until(lambda: bus.status.protective_stop == 2)  # the client has seen it
    t0 = time.monotonic()
    with pytest.raises(DriveError, match=r"stream is silent .* stream\(\) "):
        getattr(bus, request_)()
    assert time.monotonic() - t0 < 1.0, "not the 5 s timeout"


def test_a_reset_while_streaming_from_a_thread_still_waits_for_the_bus(bus, fake):
    fake.status = DriveStatus(brake_engaged=0b1111, protective_stop=2)
    done = threading.Event()
    streamer = threading.Thread(
        target=lambda: [
            bus.stream({"v": 0.0, "t": 0.0}) or time.sleep(0.005)
            for _ in iter(done.is_set, True)
        ]
    )
    streamer.start()
    try:
        bus.reset()
    finally:
        done.set()
        streamer.join()
    assert bus.status.protective_stop == 0


def test_a_stop_after_the_stream_went_silent_still_waits_for_the_brakes(bus, fake):
    bus.stream({"v": 0.0, "t": 0.0})
    time.sleep(0.2)
    fake.status = DriveStatus(brake_engaged=0, protective_stop=2)
    # the client has seen it: brakes not yet engaged
    assert wait_until(
        lambda: bus.status.protective_stop == 2 and bus.status.brake_engaged == 0
    )
    bus.stop()  # the brakes engage whether or not the stream is live
    assert fake.order[-1] == "stop"
    assert bus.status.brake_engaged == fake.axes_mask


def test_a_reset_judges_the_silent_stream_on_the_status_after_it(bus, fake):
    # A bus stop()ped after its stream went silent no longer re-asserts the stop:
    # the cached status from before the reset still shows it, the reset clears it.
    bus.stream({"v": 0.0, "t": 0.0})
    time.sleep(0.2)
    fake.status = DriveStatus(brake_engaged=0b1111, protective_stop=2)
    assert wait_until(lambda: bus.status.protective_stop == 2)
    bus.reset()
    assert bus.status.protective_stop == 0


def test_a_release_with_an_axis_in_fault_asks_for_a_reset_at_once(bus, fake):
    # A drive's latched fault (CiA 402 Fault) does not enable: reset() is the
    # deliberate fault reset, release() does not do it behind the caller's back.
    fake.status = DriveStatus(
        brake_engaged=0b1111, statusword=[0x0238, 0x0250, 0x0238, 0x0250]
    )
    assert wait_until(
        lambda: list(bus.status.statusword) == [0x0238, 0x0250, 0x0238, 0x0250]
    )
    t0 = time.monotonic()
    with pytest.raises(
        DriveError, match=r"a0 in fault .*0x0238.*t in fault .*call reset\(\) first"
    ):
        bus.release()
    assert time.monotonic() - t0 < 1.0, "not the 5 s timeout"
    assert not any(c.release_brake for c in fake.commands), "nothing sent"
    bus.reset()
    bus.release()


def test_a_drive_that_faults_on_the_release_fails_it_at_once(bus, fake):
    fake.fault_on_release = True
    t0 = time.monotonic()
    with pytest.raises(DriveError, match=r"a1 in fault .*reset\(\)"):
        bus.release()
    assert time.monotonic() - t0 < 1.0, "not the 5 s timeout"


def test_a_release_the_bus_refuses_for_a_slave_out_of_op_names_it_at_once(bus, fake):
    fake.refuse_release = RtEvent(
        event_type=RtEvent.SLAVE_NOT_OPERATIONAL, joint_id=1, error_code=4, value=1.0
    )
    t0 = time.monotonic()
    with pytest.raises(
        DriveError,
        match=r"release was refused or tripped — a1: its slave \(ring 1\) is not in OP \(SAFEOP\)",
    ):
        bus.release()
    assert time.monotonic() - t0 < 1.0, "not the 5 s timeout"


def test_a_release_the_bus_refuses_for_the_mode_names_the_displayed_mode(bus, fake):
    fake.refuse_release = RtEvent(
        event_type=RtEvent.MODES_OF_OPERATION_MISMATCH,
        joint_id=0,
        error_code=0x0237,
        value=0.0,
    )
    with pytest.raises(
        DriveError,
        match=r"a0: not in its declared mode \(0x6061 reads 0, statusword 0x0237\)",
    ):
        bus.release()


def test_a_negative_vendor_mode_is_named_signed(bus, fake):
    # 0x6061 is INTEGER8: a vendor mode -1 reads -1, not 255.
    fake.refuse_release = RtEvent(
        event_type=RtEvent.MODES_OF_OPERATION_MISMATCH,
        joint_id=0,
        error_code=0x0237,
        value=-1.0,
    )
    with pytest.raises(DriveError, match=r"0x6061 reads -1, statusword"):
        bus.release()


def test_a_mode_refusal_at_display_0_names_the_post_enable_fix():
    # 0 at rest is how a drive that shows its mode only after enable looks
    # under the pre_enable default; any other display is a plain mismatch.
    from plem.drive.bus import describe_refusal

    def mismatch(display: float, statusword: int = 0x0250) -> RtEvent:
        return RtEvent(
            event_type=RtEvent.MODES_OF_OPERATION_MISMATCH,
            joint_id=0,
            error_code=statusword,
            value=display,
        )

    assert describe_refusal(mismatch(0.0), "a0") == (
        "a0: not in its declared mode (0x6061 reads 0, statusword 0x0250) — if this drive shows "
        "its mode only after enable, write mode_display_timing: post_enable on this axis in the "
        "drive model file"
    )
    assert describe_refusal(mismatch(8.0), "a0") == (
        "a0: not in its declared mode (0x6061 reads 8, statusword 0x0250)"
    )
    # The bus admits a 0 display under post_enable only at rest: SWITCH_ON_DISABLED
    # (0x0250, 0x0270) or READY_TO_SWITCH_ON (0x0231). No frame (0x0000), a fault
    # (0x0218) or an enabled axis (0x0237) is not fixed by the key: no hint.
    for rest in (0x0270, 0x0231):
        assert "post_enable" in describe_refusal(mismatch(0.0, rest), "a0")
    for other in (0x0000, 0x0218, 0x0237):
        assert describe_refusal(mismatch(0.0, other), "a0") == (
            f"a0: not in its declared mode (0x6061 reads 0, statusword 0x{other:04x})"
        )


PENDING = DriveStatus(brake_engaged=0b1111, brake_operation_pending=0b1111)
RELEASED = DriveStatus(operation_enabled=True, brake_engaged=0)


def test_an_earlier_requests_refusal_arriving_after_the_request_does_not_fail_it(
    bus, fake
):
    # Published while this request is served, so it surely arrives after it:
    # the bus's own state, pending and then released, decides.
    fake.late_refusal = RtEvent(
        event_type=RtEvent.SLAVE_NOT_OPERATIONAL, joint_id=1, error_code=4, value=1.0
    )
    fake.release_statuses = [PENDING] * 5 + [RELEASED]
    bus.release()
    assert [c.release_brake for c in fake.commands] == [True]


def test_a_status_sampled_before_the_request_is_not_its_refusal(bus, fake):
    # The first status after the answer may predate the request: brakes
    # engaged and nothing pending there is not the bus's answer yet.
    fake.release_statuses = (
        [DriveStatus(brake_engaged=0b1111)] + [PENDING] * 5 + [RELEASED]
    )
    bus.release()


def test_a_status_torn_while_the_release_settles_is_not_a_refusal(bus, fake):
    # The bus writes the brakes, then the pending mask, then operation enabled:
    # one status can read the brakes released and nothing pending, not yet enabled.
    torn = DriveStatus(operation_enabled=False, brake_engaged=0)
    fake.release_statuses = [PENDING] * 2 + [torn] * 3 + [RELEASED]
    bus.release()


def test_a_release_refused_without_a_reason_on_rt_events_fails_at_once(bus, fake):
    # Nothing pending and the brakes engaged: the bus discarded the request.
    fake.release_statuses = []
    t0 = time.monotonic()
    with pytest.raises(
        DriveError, match=r"release was refused or tripped — see rt_events"
    ):
        bus.release()
    assert time.monotonic() - t0 < 1.0, "not the 5 s timeout"


def test_the_rest_of_an_earlier_two_axis_refusal_does_not_fail_a_retry(bus, fake):
    # The second event, delivered while the retry is served, answered the
    # first request.
    fake.split_refusal = (
        RtEvent(event_type=RtEvent.SLAVE_NOT_OPERATIONAL, joint_id=0, error_code=4),
        RtEvent(event_type=RtEvent.SLAVE_NOT_OPERATIONAL, joint_id=1, error_code=4),
    )
    with pytest.raises(DriveError, match=r"release was refused or tripped — a0: "):
        bus.release()
    bus.release()


def test_a_refusal_from_before_the_request_does_not_fail_it(fake):
    fake.event_pub.publish(
        RtEvent(
            event_type=RtEvent.SLAVE_NOT_OPERATIONAL,
            joint_id=0,
            error_code=2,
        )
    )
    with Bus(NS, timeout=5.0) as b:
        b.release()


def test_a_bus_that_resets_faults_itself_releases_through_a_fault():
    # fault_reset: bounded_auto_reset — the bus resets a fault on the release.
    fake = FakeBus(urdf=description("bounded_auto_reset"))
    try:
        fake.status = DriveStatus(
            brake_engaged=0b1111, statusword=[0x0238, 0x0250, 0x0238, 0x0250]
        )
        with Bus(NS, timeout=5.0) as b:
            assert wait_until(
                lambda: list(b.status.statusword) == [0x0238, 0x0250, 0x0238, 0x0250]
            )
            b.release()
        assert [c.release_brake for c in fake.commands] == [
            True,
            False,
        ]  # release, the leaving stop
    finally:
        fake.close()


def test_a_with_block_left_by_an_exception_stops_the_axes(fake):
    with pytest.raises(ValueError, match="the user's own error"):
        with Bus(NS, timeout=5.0) as b:
            b.release()
            raise ValueError("the user's own error")
    assert [c.protective_stop for c in fake.commands] == [False, True]


def test_a_with_block_left_normally_stops_the_axes(fake):
    with Bus(NS, timeout=5.0) as b:
        b.release()
    assert [c.protective_stop for c in fake.commands] == [False, True]


def test_a_stop_that_fails_on_a_normal_exit_is_raised(fake):
    with pytest.raises(DriveError, match="the stop"):
        with Bus(NS, timeout=0.5) as b:
            b.release()
            fake.silent = True  # the stop's brakes are never reported engaged


def test_a_failed_stop_on_the_way_out_never_masks_the_exception(fake, capfd):
    with pytest.raises(ValueError, match="the user's own error"):
        with Bus(NS, timeout=0.5) as b:
            b.release()
            fake.silent = True  # the stop's brakes are never reported engaged
            raise ValueError("the user's own error")
    assert fake.commands[-1].protective_stop
    assert "the stop on the way out failed" in capfd.readouterr().err, (
        "the failed stop left no trace"
    )


def test_a_description_that_is_no_drive_bus_leaves_nothing_running():
    fake = FakeBus(urdf="<robot name='r'/>")
    try:
        with pytest.raises(DriveError, match="no ros2_control hardware"):
            Bus(NS, timeout=5.0)
        # The fake's own executor may add pool workers; the client's is `(spin)`.
        left = [t.name for t in threading.enumerate() if "(spin)" in t.name]
        assert left == [fake.thread.name], "the client's spin thread was left"
    finally:
        fake.close()


def test_no_bus_of_that_name_names_it_and_asks_whether_run_is_up():
    with pytest.raises(DriveError) as error:
        Bus("nobody_here", timeout=0.5)
    assert str(error.value).startswith("/nobody_here: timed out after 0.5 s")
    assert "is `plem drive run` up with bus.name nobody_here?" in str(error.value)
