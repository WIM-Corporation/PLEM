# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The controller manager over the drive system interface, stopped the way an
operator stops `plem drive run` (SIGINT): the bus must be asked for its stop
(ramp_stop — the axes decelerate) and then stopped, in that order, before the
process ends — also while the bus still waits for every slave to reach OP.
A test-only plugin records the host calls (no bus)."""

import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

import pytest
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from plem.drive.cli import run  # noqa: E402
from ros_test_domain import private_domain_id, wait_until  # noqa: E402

PREFIX = os.environ["PLEM_DRIVE_TEST_PREFIX"]
NODE = os.environ["PLEM_ROS2_CONTROL_NODE"]
# robot_state_publisher beside the controller manager: the description reaches
# the manager by topic, as `plem drive run` hands it.
PUBLISHER = (
    pathlib.Path(NODE).parents[1] / "robot_state_publisher" / "robot_state_publisher"
)


def description(record_file, op_after_ms):
    return f"""<robot name="bus">
  <link name="world"/>
  <link name="p_link"/>
  <joint name="p" type="revolute"><parent link="world"/><child link="p_link"/>
    <limit lower="-1" upper="1" velocity="1" effort="0"/></joint>
  <ros2_control name="bus" type="system">
    <hardware>
      <plugin>plem_drive/test/RecordingDriveSystemInterface</plugin>
      <param name="record_file">{record_file}</param>
      <param name="op_after_ms">{op_after_ms}</param>
    </hardware>
    <joint name="p">
      <command_interface name="position"/><command_interface name="velocity"/>
      <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
    </joint>
  </ros2_control>
</robot>"""


@pytest.mark.parametrize(
    "op_after_ms, settle_s",
    [
        # Running: past the activation.
        (0, 1.0),
        # Still waiting for OP: the SIGINT lands in the activation itself.
        (5000, 0.0),
    ],
    ids=["running", "waiting-for-op"],
)
def test_sigint_asks_the_bus_for_its_stop_then_stops_it(op_after_ms, settle_s):
    with tempfile.TemporaryDirectory() as tmp:
        record = pathlib.Path(tmp) / "calls"
        params = pathlib.Path(tmp) / "params.yaml"
        params.write_text(
            yaml.safe_dump(
                {
                    "controller_manager": {"ros__parameters": {"update_rate": 100}},
                    "robot_state_publisher": {
                        "ros__parameters": {
                            "robot_description": description(record, op_after_ms)
                        }
                    },
                }
            )
        )
        env = dict(
            os.environ,
            AMENT_PREFIX_PATH=PREFIX + os.pathsep + os.environ["AMENT_PREFIX_PATH"],
        )
        env["ROS_DOMAIN_ID"] = private_domain_id()  # alone on its domain
        publisher = subprocess.Popen(
            [
                "setpriv",
                "--pdeathsig",
                "KILL",
                str(PUBLISHER),
                "--ros-args",
                "--params-file",
                str(params),
            ],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        proc = subprocess.Popen(
            # setpriv: the node dies with this process, however it ends (it
            # execs — proc.pid stays the node's).
            [
                "setpriv",
                "--pdeathsig",
                "KILL",
                NODE,
                "--ros-args",
                "--params-file",
                str(params),
                *run.DESCRIPTION_FROM_TOPIC,
            ],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            assert wait_until(
                lambda: record.exists() and "start" in record.read_text(), 20
            ), "the hardware never started"
            time.sleep(settle_s)
            proc.send_signal(signal.SIGINT)
            out, _ = proc.communicate(timeout=20)
        finally:
            if proc.poll() is None:
                proc.kill()
            publisher.kill()
        calls = record.read_text().split()
        assert calls == ["start", "ramp_stop", "stop"], (
            f"host calls {calls}\n--- node output ---\n{out}"
        )
