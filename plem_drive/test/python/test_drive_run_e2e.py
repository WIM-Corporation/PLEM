# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The whole `plem drive run` chain without a bus: the robot description and
controller parameters run writes, the controller manager with the drive system
interface over a simulated host (test plugin), the joint state broadcaster and
trajectory controller or stream controller spawned as run spawns them, and the
Python client — release, a planned move (Trapezoid and SCurve), a stop, a
release after the stop and a move back; and a stream to a csv and a cst axis."""

import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import pytest
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from plem.drive import Bus, DriveError, SCurve  # noqa: E402
from plem.drive.cli import run  # noqa: E402
from ros_test_domain import private_domain_id, wait_until  # noqa: E402

PREFIX = os.environ["PLEM_DRIVE_TEST_PREFIX"]
NODE = os.environ["PLEM_ROS2_CONTROL_NODE"]
LIB = pathlib.Path(NODE).parents[1]  # <prefix>/lib of the controller manager's ROS

CHECK = {
    "approval_effective": "approved",
    "cycle_time_ns": 10_000_000,  # 100 Hz: light on a test box
    "fault_reset_policy": "none",
    "axes": [
        {
            "name": name,
            "mode": "csp",
            "position_min_rad": -2.0,
            "position_max_rad": 2.0,
            "velocity_limit_rad_s": 2.0,
            "velocity_cap_rad_s": 2.0,
            "acceleration_limit_rad_s2": 8.0,
        }
        for name in ("a0", "a1")
    ],
}
# A stream bus: the cst axis first, so a swapped order shows on the csv axis.
STREAMED = {
    "approval_effective": "approved",
    "cycle_time_ns": 10_000_000,
    "fault_reset_policy": "none",
    "axes": [
        {
            "name": "t",
            "mode": "cst",
            "position_min_rad": -2.0,
            "position_max_rad": 2.0,
            "velocity_limit_rad_s": 2.0,
            "velocity_cap_rad_s": 2.0,
            "acceleration_limit_rad_s2": None,
            "torque_min_nm": -3.0,
            "torque_max_nm": 3.0,
        },
        {
            "name": "v",
            "mode": "csv",
            "position_min_rad": -2.0,
            "position_max_rad": 2.0,
            "velocity_limit_rad_s": 2.0,
            "velocity_cap_rad_s": 2.0,
            "acceleration_limit_rad_s2": 8.0,
        },
    ],
}


def simulated(urdf: str, record: pathlib.Path) -> str:
    robot = ET.fromstring(urdf)
    hardware = robot.find("ros2_control/hardware")
    hardware.find("plugin").text = "plem_drive/test/RecordingDriveSystemInterface"
    ET.SubElement(hardware, "param", name="record_file").text = str(record)
    return ET.tostring(robot, encoding="unicode")


@pytest.fixture
def bus_up(monkeypatch, request):
    check = getattr(request, "param", CHECK)
    with tempfile.TemporaryDirectory() as tmp:
        record = pathlib.Path(tmp) / "calls"
        ns = "/e2e"
        description = simulated(run.robot_description(check, "/unused.yaml"), record)
        params = run.controller_params(check, cpus=range(1), isolated=set())
        params["robot_state_publisher"] = {
            "ros__parameters": {"robot_description": description}
        }
        params_file = pathlib.Path(tmp) / "params.yaml"
        params_file.write_text(yaml.safe_dump(run.namespaced(params, ns)))
        domain = private_domain_id()
        monkeypatch.setenv("ROS_DOMAIN_ID", domain)
        env = dict(
            os.environ,
            AMENT_PREFIX_PATH=PREFIX + os.pathsep + os.environ["AMENT_PREFIX_PATH"],
        )
        ros_args = [
            "--ros-args",
            "-r",
            f"__ns:={ns}",
            "--params-file",
            str(params_file),
        ]
        # setpriv: the nodes die with this process, however it ends (it execs —
        # proc.pid stays the node's).
        procs = [
            subprocess.Popen(
                [
                    "setpriv",
                    "--pdeathsig",
                    "KILL",
                    NODE,
                    *ros_args,
                    *run.DESCRIPTION_FROM_TOPIC,
                ],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ),
            subprocess.Popen(
                [
                    "setpriv",
                    "--pdeathsig",
                    "KILL",
                    str(LIB / "robot_state_publisher" / "robot_state_publisher"),
                    *ros_args,
                ],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ),
        ]
        try:
            for controller in run.lanes(check):
                spawner = subprocess.run(
                    [
                        str(LIB / "controller_manager" / "spawner"),
                        controller,
                        "-c",
                        f"{ns}/controller_manager",
                    ],
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=60,
                )
                assert spawner.returncode == 0, spawner.stdout + spawner.stderr
            yield ns, record
        finally:
            for proc in procs:
                proc.send_signal(signal.SIGINT)
            for proc in procs:
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    proc.kill()


def test_release_move_stop_midway_release_and_move_back(bus_up):
    ns, record = bus_up
    with Bus(ns, timeout=20.0) as bus:
        assert bus.axes == ("a0", "a1")
        bus.release()
        bus.move_to({"a0": 0.5, "a1": -0.25}, velocity_scale=0.5)
        assert wait_until(
            lambda: (
                abs(bus.positions["a0"] - 0.5) < 1e-6
                and abs(bus.positions["a1"] + 0.25) < 1e-6
            )
        )
        # A stop in the middle of a move: the axes come to rest off the plan.
        outcome = []

        def move():
            try:
                bus.move_to({"a0": -1.5}, velocity_scale=0.2)
                outcome.append(None)
            except Exception as error:  # noqa: BLE001 — asserted below
                outcome.append(error)

        mover = threading.Thread(target=move)
        mover.start()
        assert wait_until(lambda: bus.positions["a0"] < 0.3)
        bus.stop()
        mover.join(timeout=10.0)
        assert not mover.is_alive()
        assert isinstance(outcome[0], DriveError), outcome
        time.sleep(0.3)  # the simulated axes come to rest past the hold
        stopped = bus.positions["a0"]
        bus.release()  # holds where the axes are — the next command starts there, no jump
        bus.move_to({"a0": 0.0, "a1": 0.0}, profile=SCurve())
        assert wait_until(
            lambda: abs(bus.positions["a0"]) < 1e-6 and abs(bus.positions["a1"]) < 1e-6
        )
        assert stopped != pytest.approx(0.0)
    # The last stop: leaving the with block.
    assert record.read_text().split() == [
        "start",
        "enable",
        "ramp_stop",
        "enable",
        "ramp_stop",
    ], "no jump after the release"


@pytest.mark.parametrize("bus_up", [STREAMED], indirect=True)
def test_stream_a_velocity_and_a_torque_axis(bus_up):
    ns, record = bus_up
    with Bus(ns, timeout=20.0) as bus:
        command = {"t": -1.0, "v": 0.0}
        running = threading.Event()
        running.set()

        def control():  # the client's loop: one command per cycle
            while running.is_set():
                bus.stream(command)
                time.sleep(0.005)

        loop = threading.Thread(target=control)
        loop.start()
        try:
            bus.release()  # a bus with a cst axis releases on a live stream
            command["v"] = 0.5
            # The bus writes a cycle only when every axis has a command: the csv
            # axis's speed shows both values arrived, each on its own axis.
            assert wait_until(lambda: bus["v"].velocity == pytest.approx(0.5))
            bus.stop()
        finally:
            running.clear()
            loop.join()
    # The second stop: leaving the with block (on a bus already stopped).
    assert record.read_text().split() == ["start", "enable", "ramp_stop", "ramp_stop"]
