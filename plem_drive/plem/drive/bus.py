# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""Bus — one drive bus started by `plem drive run`, moved from Python.

The client speaks only the bus's standard ROS surface: the robot description
(axes and their limits), joint states, the drive status and its RT events (a
release the loop refused), the control command service, the joint trajectory
controller's FollowJointTrajectory action, the controller manager's switch
service and the stream controller's commands topic. It spins its own ROS
context on its own thread, so its calls block the caller only, never a callback.
"""

from __future__ import annotations

import math
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Literal

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from plem_drive_msgs.msg import DriveStatus
from plem_rt_msgs.msg import RtEvent
from plem_rt_msgs.srv import SetControlCommand
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from plem.drive.cia402 import RESTING, fault_bit, state
from plem.drive.profile import Limits, Profile, Scale, Trapezoid

TRAJECTORY_CONTROLLER = "joint_trajectory_controller"
# A move has arrived when every axis's measured position is this close to its goal.
ARRIVAL_TOLERANCE_RAD = 1e-3
# The bus reports its status ten times a second; this long without one, it is gone.
STATUS_SILENCE_S = 1.0
# Past this since this client's last stream() call the bus has stopped on the silence
# (50 ms stream let-go + 50 ms command watch).
STREAM_SILENCE_S = 0.1
# DriveStatus.protective_stop bits, with the causes each one covers.
STOP_SOURCES = (
    (
        DriveStatus.PROTECTIVE_STOP_SOURCE_INTERNAL,
        "protective stop INTERNAL (a limit still broken)",
    ),
    (
        DriveStatus.PROTECTIVE_STOP_SOURCE_EXTERNAL,
        "protective stop EXTERNAL (the bus, the command stream or a drive leaving"
        " Operation enabled or its declared mode — see rt_events)",
    ),
)


# SLAVE_NOT_OPERATIONAL's error_code: the slave's EtherCAT AL state as the master reports it.
AL_STATES = {
    0: "offline, or the state could not be read",
    1: "INIT",
    2: "PREOP",
    4: "SAFEOP",
    8: "OP with a reconfiguration pending",
}


def fault_detail(statusword: int, error_code: int | None) -> str:
    """'statusword 0x…', with the drive's error code when the bus reported one
    (SERVO_FAULT's value; 0 = no code reported, e.g. no error-code PDO mapped)."""
    error = f", error 0x{error_code:04x}" if error_code else ""
    return f"statusword 0x{statusword:04x}{error}"


def forget_cleared(codes: dict[int, int], statusword) -> None:
    """Drop the codes of axes no longer in fault (statusword bit 3 clear), so a
    later fault shows its own code or none, never the previous one."""
    for i, word in enumerate(statusword):
        if not fault_bit(word):
            codes.pop(i, None)


def describe_refusal(event: RtEvent, axis: str) -> str:
    """A SLAVE_NOT_OPERATIONAL or MODES_OF_OPERATION_MISMATCH event, in words."""
    if event.event_type == RtEvent.SLAVE_NOT_OPERATIONAL:
        al_state = AL_STATES.get(event.error_code, f"AL state 0x{event.error_code:x}")
        return f"{axis}: its slave (ring {int(event.value)}) is not in OP ({al_state})"
    text = (
        f"{axis}: not in its declared mode (0x6061 reads {int(event.value)}, "
        f"statusword 0x{event.error_code:04x})"
    )
    # The bus admits a 0 display under post_enable only at rest (SWITCH_ON_DISABLED or
    # READY_TO_SWITCH_ON, the core's to_servo_state); elsewhere the key fixes nothing.
    if int(event.value) == 0 and state(event.error_code) in RESTING:
        text += (
            " — if this drive shows its mode only after enable, write "
            "mode_display_timing: post_enable on this axis in the drive model file"
        )
    return text


class DriveError(RuntimeError):
    """The bus refused or failed a request; the message is the bus's reason."""


@dataclass(frozen=True)
class _Axis:
    name: str
    # position (csp, the trajectory controller), velocity (csv) or effort (cst)
    command: Literal["position", "velocity", "effort"]
    lower: float  # -inf/inf on an axis without end stops
    upper: float
    velocity: float
    acceleration: float | None
    torque: tuple[float, float] | None  # a cst axis's clamp (min, max) in Nm


def _axes(urdf: str) -> list[_Axis]:
    robot = ET.fromstring(urdf)
    hw = {j.get("name"): j for j in robot.find("ros2_control").iter("joint")}
    out = []
    # The file's axes order: run writes the joints and the stream controller's
    # `joints` parameter from it, so the stream axes here are in the controller's order.
    for joint in robot.iter("joint"):
        name = joint.get("name")
        if joint.get("type") is None or name not in hw:
            continue
        limit = joint.find("limit")
        commands = [c.get("name") for c in hw[name].iter("command_interface")]
        params = {p.get("name"): p.text for p in hw[name].iter("param")}
        out.append(
            _Axis(
                name=name,
                command=commands[0],
                lower=float(limit.get("lower", "-inf")),
                upper=float(limit.get("upper", "inf")),
                velocity=float(limit.get("velocity")),
                acceleration=float(params["acceleration_rad_s2"])
                if "acceleration_rad_s2" in params
                else None,
                torque=(float(params["torque_min_nm"]), float(params["torque_max_nm"]))
                if "torque_min_nm" in params
                else None,
            )
        )
    return out


class Axis:
    """One axis of a bus: its state and a single-axis move."""

    def __init__(self, bus: Bus, name: str):
        self._bus = bus
        self.name = name

    @property
    def position(self) -> float:
        return self._bus.positions[self.name]

    @property
    def velocity(self) -> float:
        return self._bus.velocities[self.name]

    def move_to(self, position: float, **kwargs) -> None:
        self._bus.move_to({self.name: position}, **kwargs)


class Bus:
    """A drive bus by its name — the model file's bus.name (a leading / is
    accepted too). Leaving `with Bus(...)` stops the axes (stop()), however
    the block ends; close() detaches without stopping."""

    def __init__(self, name: str, *, timeout: float = 10.0):
        self._ns = "/" + name.lstrip("/")
        self._timeout = timeout
        self.default_profile: Profile = Trapezoid()
        self._context = rclpy.Context()
        rclpy.init(context=self._context)
        self._node = rclpy.create_node(
            "plem_drive_client", namespace=self._ns, context=self._context
        )
        self._lock = threading.Lock()
        self._joints: JointState | None = None
        self._joints_at = 0.0
        # None only until __init__ returns: it waits for the first status.
        self._status: DriveStatus | None = None
        self._status_at = 0.0
        self._statuses = 0  # statuses received, counted
        self._streamed_at: float | None = None
        self._refusals: list[RtEvent] = []
        self._fault_codes: dict[int, int] = {}  # axis index -> last SERVO_FAULT code
        self._description = threading.Event()
        self._urdf = ""
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(
            String, "robot_description", self._on_description, latched
        )
        self._node.create_subscription(JointState, "joint_states", self._on_joints, 10)
        self._node.create_subscription(
            DriveStatus, "plem_drive/status", self._on_status, latched
        )
        self._node.create_subscription(
            RtEvent,
            "plem_drive/rt_events",
            self._on_event,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._command = self._node.create_client(
            SetControlCommand, "plem_drive/control_command"
        )
        self._switch = self._node.create_client(
            SwitchController, "controller_manager/switch_controller"
        )
        self._controllers = self._node.create_client(
            ListControllers, "controller_manager/list_controllers"
        )
        self._trajectory = ActionClient(
            self._node,
            FollowJointTrajectory,
            f"{TRAJECTORY_CONTROLLER}/follow_joint_trajectory",
        )
        self._stream = self._node.create_publisher(
            Float64MultiArray,
            "stream_controller/commands",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._goal = None
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        try:
            self._wait(
                lambda: (
                    self._description.is_set()
                    and self._joints is not None
                    and self._status is not None
                ),
                f"the bus (is `plem drive run` up with bus.name {self._ns[1:]}? It comes up "
                "once every slave is in OP — its terminal then says `every slave in OP`)",
            )
            hardware = ET.fromstring(self._urdf).find("ros2_control/hardware")
            if hardware is None:
                raise DriveError(
                    f"{self._ns}: the robot description has no ros2_control hardware — "
                    "not a drive bus (is it the description `plem drive run` published?)"
                )
            self._by_name = {axis.name: axis for axis in _axes(self._urdf)}
            # A fault fails a release at once only when nothing else resets it. Under
            # bounded_auto_reset the bus resets it (and the timeout covers the bus giving
            # up). A description written by an older run has no fault_reset_policy, so the
            # release keeps waiting.
            self._fault_is_final = any(
                p.get("name") == "fault_reset_policy" and p.text == "none"
                for p in hardware.iter("param")
            )
        except BaseException:
            self.close()
            raise
        self.axes = tuple(self._by_name)

    # State
    def __getitem__(self, name: str) -> Axis:
        if name not in self._by_name:
            raise KeyError(
                f"no axis '{name}' on {self._ns} — its axes: {', '.join(self.axes)}"
            )
        return Axis(self, name)

    @property
    def positions(self) -> dict[str, float]:
        return self._latest_joints()[0]

    @property
    def velocities(self) -> dict[str, float]:
        return self._latest_joints()[1]

    @property
    def status(self) -> DriveStatus:
        """The bus's latest status: operation_enabled (every drive in Operation
        enabled and in its declared mode), brake_engaged and protective_stop
        (bit masks, 0 = none), statusword."""
        return self._status

    # Commands
    def release(self) -> None:
        """Release the brakes and enable every axis.

        The axes then hold where they are until moved. On a bus with a csp axis,
        release() first waits (up to the timeout) for the trajectory controller
        that `plem drive run` starts once the bus is up, then restarts it so it
        holds the current position rather than an earlier goal. On a bus without
        a cst axis, a csv axis holds 0 rad/s until the first stream() command
        after the release. A bus with a cst axis releases only while stream() is
        running, and its axes follow the streamed commands at once.

        Raises DriveError when the bus refuses or trips the release (for example
        a slave not in OP, or an axis not in its declared mode); the message
        names the reasons rt_events carried since the request. The refusal is
        read once the bus settles: from the second status after the service
        answer, brakes engaged with no brake operation pending. While an earlier
        brake operation is still pending, release() does not decide and may wait
        out the timeout. With fault_reset policy none, an axis in fault (CiA 402
        Fault, for example an alarm latched when the bus last stopped) fails the
        release at once, before and after the request: call reset() first. Under
        bounded_auto_reset the bus clears the fault itself."""
        if self._fault_is_final and (fault := self._in_fault()):
            raise DriveError(
                f"{self._ns}: {fault} — call reset() first, then release()"
            )
        if any(axis.command == "position" for axis in self._by_name.values()):
            # `plem drive run` reports the bus up before its spawner has started the
            # controller; restarting it earlier races the spawner and is refused.
            self._wait(
                self._trajectory_active,
                f"{TRAJECTORY_CONTROLLER} to become active (plem drive run spawns it once the "
                "bus is up; if it was deactivated by hand, activate it again with: ros2 control "
                f"switch_controllers --activate {TRAJECTORY_CONTROLLER} -c {self._ns}/controller_manager)",
            )
            # A trajectory controller re-activated holds where the axes are (run sets
            # it so), not where an earlier goal or a stop left its command.
            # Two requests: one that names the controller in both lists is refused.
            for deactivate, activate, strictness in (
                ([TRAJECTORY_CONTROLLER], [], SwitchController.Request.BEST_EFFORT),
                ([], [TRAJECTORY_CONTROLLER], SwitchController.Request.STRICT),
            ):
                request = SwitchController.Request(
                    deactivate_controllers=deactivate,
                    activate_controllers=activate,
                    strictness=strictness,
                )
                if not self._call(self._switch, request).ok:
                    raise DriveError(
                        f"{self._ns}: the trajectory controller could not be restarted"
                        " (controller_manager refused the switch; see its log)"
                    )
        with self._lock:  # earlier refusals answered earlier releases
            self._refusals.clear()
        self._control(SetControlCommand.Request(release_brake=True))
        answered = self._answered()

        def released() -> bool:
            # Status and the service answer travel unordered: two statuses sampled
            # before the request but delivered after the answer can be taken for it,
            # hence the second status. The count first: the status read after it is
            # at least as new.
            answer = self._statuses >= answered
            status = self._status
            if status.operation_enabled and status.brake_engaged == 0:
                return True
            if not answer:
                return False
            if self._fault_is_final and (fault := self._in_fault()):
                raise DriveError(
                    f"{self._ns}: an axis faulted during the release: {fault} (see rt_events) — "
                    "call reset() first, then release()"
                )
            if status.brake_engaged and status.brake_operation_pending == 0:
                raise DriveError(
                    f"{self._ns}: the release was refused or tripped — "
                    f"{self._refusal_reasons() or 'see rt_events'}"
                )
            return False

        self._wait(
            released,
            "the release (every axis Operation enabled, every brake released)",
            answered=answered,
        )

    def move_to(
        self,
        goals: dict[str, float],
        *,
        velocity_scale: float = 1.0,
        acceleration_scale: float = 1.0,
        profile: Profile | None = None,
    ) -> None:
        """Move position axes to `goals` (rad) and return on arrival; the other
        position axes keep their positions. Every planned point is checked
        against the file's limits before anything moves."""
        for name in goals:
            if self._by_name[self[name].name].command != "position":
                raise ValueError(
                    f"'{name}' is not a position axis (csp) — stream its commands instead"
                )
        if not self._status.operation_enabled:
            raise DriveError(
                f"{self._ns}: the axes are not released — call release() first"
            )
        # A bus that stopped reporting would be planned from stale positions and sent a
        # goal it may still run: refuse before anything is sent.
        self._reporting()
        axes = [axis for axis in self._by_name.values() if axis.command == "position"]
        now = self.positions
        start = [now[axis.name] for axis in axes]
        goal = [float(goals.get(axis.name, now[axis.name])) for axis in axes]
        limits = [Limits(axis.velocity, axis.acceleration) for axis in axes]
        trajectory = (profile or self.default_profile).plan(
            start, goal, limits, Scale(velocity_scale, acceleration_scale)
        )
        trajectory.joint_names = [axis.name for axis in axes]
        _check(trajectory, axes)
        self._send(trajectory)
        self._arrive(dict(zip(trajectory.joint_names, goal)))

    def stream(self, commands: dict[str, float]) -> None:
        """Send one cycle's command to every stream axis — rad/s on a csv axis,
        Nm on a cst axis. Checked first, per value: finite, and inside the
        file's limits.velocity_rad_s on a csv axis (the speed only, not the
        acceleration) or limits.torque_nm's min and max on a cst axis. Call it
        every cycle: about 100 ms after the last call (50 ms stream let-go + 50 ms
        command watch) the bus takes a protective stop, so a client that hangs
        stops the axes. On a bus with a cst axis, stream before release(): it
        refuses without a live command. A cst axis whose robot description
        carries no torque limits (written by an older plem drive run) raises
        DriveError: restart plem drive run."""
        if all(axis.command == "position" for axis in self._by_name.values()):
            raise ValueError(
                f"{self._ns} has no stream axis (csv or cst) — move its axes with move_to"
            )
        for name in commands:
            if self._by_name[self[name].name].command == "position":
                raise ValueError(
                    f"'{name}' is a position axis (csp) — move it with move_to"
                )
        values = []
        for axis in self._by_name.values():
            if axis.command == "position":
                continue
            if axis.name not in commands:
                raise ValueError(
                    f"no command for the stream axis '{axis.name}' — every call commands every stream axis"
                )
            value = float(commands[axis.name])
            if axis.command == "effort":
                if axis.torque is None:
                    raise DriveError(
                        f"{self._ns}: the robot description carries no torque limits for '{axis.name}' — "
                        "it was written by an older plem drive run; restart plem drive run"
                    )
                (lower, upper), unit, key = axis.torque, "Nm", "limits.torque_nm"
            else:
                lower, upper = -axis.velocity, axis.velocity
                unit, key = "rad/s", "limits.velocity_rad_s"
            if not lower <= value <= upper:  # NaN too
                raise ValueError(
                    f"{axis.name}: {value:.6g} {unit} is outside {key} [{lower:.6g}, {upper:.6g}]"
                )
            values.append(value)
        self._stream.publish(Float64MultiArray(data=values))
        self._streamed_at = time.monotonic()

    def reset(self) -> None:
        """Clear drive faults and the protective stop (reset_error), then wait
        until the bus reports neither. A stop whose cause still stands stays,
        and the DriveError names its source and the axes in fault — the cause
        itself is on rt_events. A silent stream on a bus with a csv or cst axis
        is such a cause: stream first, then reset. Once this client has
        streamed, release() and reset() raise at once when the bus, after the
        request, still reports the EXTERNAL stop while this client's stream is
        silent — keep stream() running through them. The axes in fault when it
        is called are logged first, as one warning with each drive's error code
        when the bus reported one: a reset clears a real alarm as readily as
        the one a bus stop leaves."""
        if fault := self._in_fault():
            self._node.get_logger().warning(f"{self._ns}: resetting {fault}")
        self._control(SetControlCommand.Request(reset_error=True))
        answered = self._answered()
        try:
            self._wait(lambda: not self._standing(), "the reset", answered=answered)
        except DriveError:
            if self._silent_stream(answered):
                raise
            if standing := self._standing():
                raise DriveError(
                    f"{self._ns}: the reset did not clear — {standing} (see rt_events)"
                ) from None

    def stop(self) -> None:
        """Requested stop. A bus without a cst axis ramps its csp and csv axes
        to rest, then engages the brakes; a bus with any cst axis takes an
        immediate disable instead: every drive is disabled at once (the drive's
        own disable reaction stops it) and the brakes engage."""
        # The stop first: it ramps the axes down from the plan's speed. A cancel
        # first would freeze the plan and leave the stop no speed to ramp from.
        # A refused stop still cancels the move.
        try:
            self._control(SetControlCommand.Request(protective_stop=True))
        finally:
            if (goal := self._goal) is not None:
                goal.cancel_goal_async()
        all_axes = (1 << len(self._by_name)) - 1
        self._wait(
            lambda: (
                self._status.brake_engaged == all_axes
                and self._status.brake_operation_pending == 0
            ),
            "the stop (every brake engaged)",
        )

    def _standing(self) -> str:
        """What a reset leaves: the protective stop's sources and the axes in fault."""
        status = self._status
        out = [text for bit, text in STOP_SOURCES if status.protective_stop & bit]
        if fault := self._in_fault():
            out.append(fault)
        return ", ".join(out)

    def _in_fault(self) -> str:
        """The axes the latest status reports in CiA 402 Fault (statusword bit 3)."""
        with self._lock:
            codes = dict(self._fault_codes)
        return ", ".join(
            f"{name} in fault ({fault_detail(word, codes.get(i))})"
            for i, (name, word) in enumerate(zip(self.axes, self._status.statusword))
            if fault_bit(word)
        )

    def _reporting(self) -> None:
        """A bus that stopped reporting its status or joint states is gone (run exited)."""
        now = time.monotonic()
        for what, at in (
            ("status", self._status_at),
            ("joint states", self._joints_at),
        ):
            if (silent := now - at) > STATUS_SILENCE_S:
                raise DriveError(
                    f"{self._ns}: the bus stopped reporting (no {what} for {silent:.1f} s — "
                    "is `plem drive run` still up?)"
                )

    def _arrive(self, goals: dict[str, float]) -> None:
        """The trajectory controller's success only means its plan's time ran out:
        the bus must still be moving the axes and they must reach their goals."""
        deadline = time.monotonic() + self._timeout
        while True:
            self._reporting()
            if (reason := _bus_moving(self._status)) is not None:
                raise DriveError(
                    f"{self._ns}: the move ended without arriving — {reason}"
                )
            now = self.positions
            off = {
                n: now[n] - g
                for n, g in goals.items()
                if not abs(now[n] - g) <= ARRIVAL_TOLERANCE_RAD
            }
            if not off:
                return
            if time.monotonic() > deadline:
                name, error = next(iter(off.items()))
                raise DriveError(
                    f"{self._ns}: {name} did not arrive: still {error:+.6g} rad from its goal"
                    f" {self._timeout} s after the planned end of the move"
                )
            time.sleep(0.01)

    def close(self) -> None:
        """Detach from the bus without stopping it: the axes keep their state
        (released brakes stay released) and the bus keeps running."""
        self._executor.shutdown()
        self._thread.join()
        self._node.destroy_node()
        rclpy.shutdown(context=self._context)

    def __enter__(self) -> Bus:
        return self

    def __exit__(self, kind, *_) -> None:
        try:
            if kind is None:
                self.stop()  # a failed stop is raised: the axes may still be released
            else:  # the block's own exception is the one raised
                try:
                    self.stop()
                except Exception as error:  # the axes may still be released: say so
                    self._node.get_logger().warning(
                        f"{self._ns}: the stop on the way out failed — {error}"
                    )
        finally:
            self.close()

    # Internals
    def _on_description(self, msg: String) -> None:
        self._urdf = msg.data
        self._description.set()

    def _on_joints(self, msg: JointState) -> None:
        with self._lock:
            self._joints = msg
            self._joints_at = time.monotonic()

    def _on_event(self, msg: RtEvent) -> None:
        if msg.event_type == RtEvent.SERVO_FAULT:
            with self._lock:
                self._fault_codes[msg.joint_id] = int(msg.value)
        # The loop's refusals of a release: the service only says it was requested.
        if msg.event_type in (
            RtEvent.SLAVE_NOT_OPERATIONAL,
            RtEvent.MODES_OF_OPERATION_MISMATCH,
        ):
            with self._lock:
                self._refusals.append(msg)

    def _refusal_reasons(self) -> str:
        """The refusals rt_events carried since the release was asked, in words."""
        with self._lock:
            events = list(self._refusals)
        return "; ".join(
            describe_refusal(
                e,
                self.axes[e.joint_id]
                if e.joint_id < len(self.axes)
                else f"axis {e.joint_id}",
            )
            for e in events
        )

    def _on_status(self, msg: DriveStatus) -> None:
        with self._lock:
            forget_cleared(self._fault_codes, msg.statusword)
        self._status = msg
        self._status_at = time.monotonic()
        self._statuses += 1

    def _latest_joints(self) -> tuple[dict[str, float], dict[str, float]]:
        with self._lock:
            js = self._joints
        return dict(zip(js.name, js.position)), dict(
            zip(js.name, js.velocity or [0.0] * len(js.name))
        )

    def _answered(self) -> int:
        """The status count from which statuses answer a request just answered:
        the first one after may have been sampled before the request was
        posted, the second normally a publish period later (status and the
        answer travel unordered, so a delayed status can still predate it)."""
        return self._statuses + 2

    def _silent_stream(self, answered: int) -> str | None:
        """The EXTERNAL stop, reported after the request (status `answered` on),
        while this client's stream is silent: the bus re-asserts it every
        cycle, so the wait cannot end."""
        fresh = (
            self._statuses >= answered
        )  # the count first: the status read after it is as new
        status = self._status
        if (
            self._streamed_at is None
            or not fresh
            or not status.protective_stop & DriveStatus.PROTECTIVE_STOP_SOURCE_EXTERNAL
        ):
            return None
        silent = time.monotonic() - self._streamed_at
        if silent <= STREAM_SILENCE_S:
            return None
        return (
            f"the stream is silent ({silent:.1f} s since the last stream() call) and the bus's "
            "protective stop (EXTERNAL) stands after the request — keep calling stream() from a "
            "thread while release() and reset() run. If the stream was live, the remaining "
            "cause is a bus communication loss (see rt_events)"
        )

    def _wait(self, done, what: str, answered: int | None = None) -> None:
        """Poll done() until the timeout. With `answered` (release, reset), a
        silent stream's EXTERNAL stop reported after the request fails at once."""
        deadline = time.monotonic() + self._timeout

        def silent() -> None:
            if answered is not None and (why := self._silent_stream(answered)):
                raise DriveError(f"{self._ns}: {what} cannot finish — {why}") from None

        while True:
            try:
                if done():
                    return
            except DriveError:
                silent()  # that stop's standing brakes read as a refusal: name the stop
                raise
            silent()
            if time.monotonic() > deadline:
                raise DriveError(
                    f"{self._ns}: timed out after {self._timeout} s waiting for {what}"
                )
            time.sleep(0.01)

    def _trajectory_active(self) -> bool:
        listed = self._call(self._controllers, ListControllers.Request()).controller
        return any(
            c.name == TRAJECTORY_CONTROLLER and c.state == "active" for c in listed
        )

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=self._timeout):
            raise DriveError(
                f"{self._ns}: {client.srv_name} is not offered (is `plem drive run` up?)"
            )
        return self._result(client.call_async(request), client.srv_name)

    def _result(self, future, what: str):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout):
            raise DriveError(f"{self._ns}: no answer from {what} in {self._timeout} s")
        return future.result()

    def _control(self, request: SetControlCommand.Request) -> None:
        response = self._call(self._command, request)
        if not response.success:
            raise DriveError(f"{self._ns}: {response.message}")

    def _send(self, trajectory) -> None:
        if not self._trajectory.wait_for_server(timeout_sec=self._timeout):
            raise DriveError(f"{self._ns}: the trajectory controller is not running")
        goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        handle = self._result(
            self._trajectory.send_goal_async(goal), "the trajectory controller"
        )
        if not handle.accepted:
            raise DriveError(f"{self._ns}: the trajectory controller refused the move")
        self._goal = handle
        last = trajectory.points[-1].time_from_start
        duration = last.sec + last.nanosec * 1e-9
        # Until the result arrives the move may still run: self._goal stays set on
        # any exit from here (Ctrl-C, a silent bus), so a stop() can cancel it.
        done = threading.Event()
        future = handle.get_result_async()
        future.add_done_callback(lambda _: done.set())
        # The move's own time (a stop() cancels it), and no more than the timeout
        # past it — while the bus still reports.
        deadline = time.monotonic() + duration + self._timeout
        while not done.wait(0.05):
            # No cancel on silence: a cancel freezes a live bus's axes without
            # deceleration; the move runs on to its checked goal.
            self._reporting()
            if time.monotonic() > deadline:
                raise DriveError(
                    f"{self._ns}: no result from the trajectory controller {self._timeout} s past the move's end"
                )
        outcome = future.result()
        self._goal = None
        if outcome.status == GoalStatus.STATUS_CANCELED:
            raise DriveError(f"{self._ns}: the move was cancelled (stop)")
        if outcome.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise DriveError(
                f"{self._ns}: the move failed — {outcome.result.error_string or outcome.result.error_code}"
            )


def _bus_moving(status: DriveStatus) -> str | None:
    """Why the bus is not moving the axes, or None."""
    if status.protective_stop:
        return "the bus took a protective stop (see rt_events)"
    if not status.operation_enabled:
        return "the axes are no longer released"
    return None


def _check(trajectory, axes: list[_Axis]) -> None:
    """A well-formed plan (points later one after another, each with a finite
    position and velocity per axis), every point and the speed change between
    points inside the file's limits — or ValueError. The motion between points
    is the controller's spline and is not sampled."""
    if not trajectory.points:
        raise ValueError("the plan has no points")
    times = [
        p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        for p in trajectory.points
    ]
    n = len(axes)
    for k, point in enumerate(trajectory.points):
        if k > 0 and not times[k] > times[k - 1]:
            raise ValueError(f"point {k} is not later than point {k - 1}")
        if (
            len(point.positions) != n
            or len(point.velocities) != n
            or len(point.accelerations) not in (0, n)
        ):
            raise ValueError(
                f"point {k} must carry {n} positions and {n} velocities (and {n} "
                "accelerations, or none): one per position axis"
            )
        values = (*point.positions, *point.velocities, *point.accelerations)
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"point {k} holds a value that is not a finite number")
    for k, point in enumerate(trajectory.points):
        for i, axis in enumerate(axes):
            position = point.positions[i]
            if not axis.lower <= position <= axis.upper:
                raise ValueError(
                    f"{axis.name}: point {k} position {position:.6g} rad is outside the limit "
                    f"[{axis.lower:.6g}, {axis.upper:.6g}]"
                )
            limits = [("velocity", abs(point.velocities[i]), axis.velocity)]
            if axis.acceleration is not None:
                if point.accelerations:
                    limits.append(
                        ("acceleration", abs(point.accelerations[i]), axis.acceleration)
                    )
                if k > 0:
                    change = abs(
                        point.velocities[i] - trajectory.points[k - 1].velocities[i]
                    )
                    limits.append(
                        (
                            "acceleration",
                            # The points sit on a nanosecond grid: allow the rounding of both.
                            change / (times[k] - times[k - 1] + 2e-9),
                            axis.acceleration,
                        )
                    )
            for quantity, value, bound in limits:
                if value > bound * (1 + 1e-6):  # the planner's nanosecond time grid
                    raise ValueError(
                        f"{axis.name}: point {k} {quantity} {value:.6g} is over the limit {bound:.6g}"
                    )
