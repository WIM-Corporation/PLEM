# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""One EtherCAT (CoE) session's statistics for plem-fleet (docs/telemetry.md).

The report describes CoE slaves and axes, whichever host runs them, so another
host publishing RtRawBatch and RtEvent can use `Session` with its own `tier`.

Started by `plem drive run` when reporting is on. A separate process that
subscribes to the same topics as the monitor, and never keeps positions,
velocities, torques or commands.
"""

from __future__ import annotations

import json
import os
import signal
import sys
import time
import uuid

from plem.drive import fleet
from plem.drive.cli.monitor import Fold

SAVE_PERIOD_S = 600.0
ROLL_PERIOD_S = 2 * 3600.0
CONSENT_CHECK_PERIOD_S = 30.0
_PROTECTIVE_STOP_TRIGGERED = (
    0  # RtEvent.error_code of PROTECTIVE_STOP; 1 = a clear refused
)


def session_context(check: dict, tier: str = "drive") -> dict:
    """What identifies this session's hardware and setup. Names, limits and startup
    SDO values stay out: they are the user's configuration."""
    slaves = []
    for index, slave in enumerate(check["slaves"]):
        slaves.append(
            {
                "ring_position": slave["ring_position"],
                "vendor_id": slave["vendor_id"],
                "product_code": slave["product_code"],
                "axis_count": sum(
                    1 for axis in check["axes"] if axis["slave"] == index
                ),
            }
        )
    return {
        "tier": tier,
        "cycle_time_ns": check.get("cycle_time_ns"),
        "sync_mode": check["sync_mode"],
        "approval_effective": check.get("approval_effective"),
        "identity_scope": check.get("identity_scope"),
        "implementation_revision": check.get("implementation_revision"),
        "config_digest": check.get("config_digest"),
        "profile_digest": check.get("profile_digest"),
        "slaves": slaves,
        "axes": [
            {"slave": axis["slave"], "mode": axis["mode"]} for axis in check["axes"]
        ],
    }


class Session:
    """Folds the bus's topics into one report. ROS-free: messages are anything with
    the fields of RtRawBatch, DriveStatus and RtEvent.

    `event_names` maps an RtEvent type to its name; `limit_reasons` a LIMIT_STOP
    error_code to its name; `stop_sources` a DriveStatus protective_stop bit to a
    source name.
    """

    def __init__(
        self, context, event_names, limit_reasons, stop_sources, clock=time.monotonic
    ):
        self.context = context
        self.report_id = str(uuid.uuid4())
        self._event_names = event_names
        self._limit_reasons = limit_reasons
        self._stop_sources = stop_sources
        self._clock = clock
        self._started = clock()
        self._fold = Fold()
        self._in_op = False
        self._events: dict[str, int] = {}
        self._limit_stops: dict[str, int] = {}
        self._stops: dict[str, int] = {}
        self._last_stop_bits = 0
        self._faults: dict[tuple[int, int], int] = {}
        self._last_event = -1

    def on_batch(self, batch) -> None:
        # The working counter is wrong on every cycle until the slaves reach OP.
        self._in_op = self._in_op or not any(batch.wkc_mismatch)
        if self._in_op:
            self._fold.fold(batch)

    def on_status(self, status) -> None:
        for bit, name in self._stop_sources.items():
            if status.protective_stop & bit and not self._last_stop_bits & bit:
                self._stops[name] = self._stops.get(name, 0) + 1
        self._last_stop_bits = status.protective_stop

    def on_event(
        self, event, *, servo_fault: int, limit_stop: int, protective_stop: int
    ) -> None:
        if event.event_sequence <= self._last_event:  # a retained event seen again
            return
        self._last_event = event.event_sequence
        if (
            event.event_type == protective_stop
            and event.error_code != _PROTECTIVE_STOP_TRIGGERED
        ):
            return
        name = self._event_names.get(event.event_type, f"type_{event.event_type}")
        self._events[name] = self._events.get(name, 0) + 1
        if event.event_type == limit_stop:
            reason = self._limit_reasons.get(
                event.error_code, f"code_{event.error_code}"
            )
            self._limit_stops[reason] = self._limit_stops.get(reason, 0) + 1
        elif event.event_type == servo_fault:
            key = (event.joint_id, int(event.value))
            self._faults[key] = self._faults.get(key, 0) + 1

    def payload(self, complete: bool) -> dict:
        fold = self._fold
        return {
            **self.context,
            "complete": complete,
            "run_s": round(self._clock() - self._started, 1),
            "samples": fold.samples,
            "lost_samples": fold.lost,
            "overruns": fold.overruns,
            "wkc_errors": fold.wkc_errors,
            "exec_mean_us": fold.exec_mean_us,
            "exec_max_us": fold.exec_max_us,
            "period_dev_p99_us": fold.dev_percentile(0.99),
            "period_dev_max_us": fold.dev_max_us,
            "dc_max_ns": fold.dc_max_ns,
            "events": self._events,
            "limit_stops": self._limit_stops,
            "protective_stop_sources": self._stops,
            # code 0: the drive reported none, or no error-code PDO is mapped.
            "faults": [
                {"axis": axis, "code": code, "count": count}
                for (axis, code), count in sorted(self._faults.items())
            ],
        }


def _names(cls, prefix: str = "", skip: tuple[str, ...] = ()) -> dict[int, str]:
    return {
        getattr(cls, name): name[len(prefix) :].lower()
        for name in dir(cls)
        if name.isupper()
        and name.startswith(prefix)
        and not name.startswith(skip)
        and isinstance(getattr(cls, name), int)
    }


def main(argv: list[str]) -> int:
    state = fleet.active_state()
    if state is None or len(argv) != 2:
        return 0
    ns, context = argv[0], json.loads(argv[1])

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from plem_drive_msgs.msg import DriveStatus
    from plem_rt_msgs.msg import RtEvent, RtRawBatch

    event_names = _names(
        RtEvent, skip=("SEVERITY_", "SOURCE_", "LIMIT_STOP_REASON_", "JOINT_ID_")
    )
    limit_reasons = _names(RtEvent, "LIMIT_STOP_REASON_")
    stop_sources = _names(DriveStatus, "PROTECTIVE_STOP_SOURCE_")

    def new_session() -> Session:
        return Session(context, event_names, limit_reasons, stop_sources)

    session = new_session()

    def work_path(current: Session) -> str:
        return f"{fleet.WORK_DIR}/{current.report_id}-{os.getpid()}.json"

    def report_of(current: Session, complete: bool) -> dict:
        report = fleet.new_report(state, "coe_session", current.payload(complete))
        report["report_id"] = current.report_id  # later saves replace, never add
        return report

    def finish(current: Session) -> None:
        if fleet.is_current(state):  # it may have been turned off during this session
            fleet.spool(report_of(current, True))
        if os.path.exists(work_path(current)):
            os.unlink(work_path(current))

    def on_save() -> None:
        nonlocal session
        if session.payload(False)["run_s"] >= ROLL_PERIOD_S:
            finish(session)
            session = new_session()
        fleet._write_json(work_path(session), report_of(session, False), 0o664)

    def check_consent() -> None:
        if not fleet.is_current(state):
            raise KeyboardInterrupt

    def stop(_signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)  # `plem drive run` ending sends this
    rclpy.init()
    node = rclpy.create_node("plem_drive_fleet")
    latched = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(
        RtRawBatch,
        f"{ns}/rt_raw",
        lambda msg: session.on_batch(msg),
        QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    node.create_subscription(
        DriveStatus,
        f"{ns}/plem_drive/status",
        lambda msg: session.on_status(msg),
        latched,
    )
    node.create_subscription(
        RtEvent,
        f"{ns}/plem_drive/rt_events",
        lambda msg: session.on_event(
            msg,
            servo_fault=RtEvent.SERVO_FAULT,
            limit_stop=RtEvent.LIMIT_STOP,
            protective_stop=RtEvent.PROTECTIVE_STOP,
        ),
        latched,
    )
    node.create_timer(SAVE_PERIOD_S, on_save)
    node.create_timer(CONSENT_CHECK_PERIOD_S, check_consent)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        finish(session)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
