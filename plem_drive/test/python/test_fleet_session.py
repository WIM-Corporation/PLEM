# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""One CoE session's report: the ROS-free folding, and what it must never carry."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from plem.drive.cli.fleet_session import Session, session_context  # noqa: E402

SERVO_FAULT, LIMIT_STOP, PROTECTIVE_STOP, BRAKE = 80, 78, 64, 66
KINDS = {
    "servo_fault": SERVO_FAULT,
    "limit_stop": LIMIT_STOP,
    "protective_stop": PROTECTIVE_STOP,
}
CHECK = {
    "bus_name": "secret_line_3",
    "cycle_time_ns": 1000000,
    "approval_effective": "approved",
    "identity_scope": "model",
    "implementation_revision": "0.11.0",
    "config_digest": "sha256:aa",
    "profile_digest": "sha256:bb",
    "bus_name": "secret_line_3",
    "sync_mode": "drive_default",
    "slaves": [
        {
            "ring_position": 0,
            "vendor_id": 0x1234,
            "product_code": 0x5678,
            "startup_sdo": [{"value": 12345}],
        }
    ],
    "axes": [
        {"name": "secret_joint", "slave": 0, "mode": "cst", "position_max_rad": 3.25},
        {"name": "other_joint", "slave": 0, "mode": "csp", "position_max_rad": 3.25},
    ],
}


def batch(seq, wkc=None, exec_us=10.0, jitter=1.0, miss=0):
    n = len(seq)
    return SimpleNamespace(
        sequence=list(seq),
        loop_exec_us=[exec_us] * n,
        loop_jitter_us=[jitter] * n,
        deadline_miss=[miss] * n,
        wkc_mismatch=list(wkc if wkc is not None else [0] * n),
        dc_sync_monitor_ns=[500] * n,
    )


def event(seq, kind, code=0, joint=0, value=0.0):
    return SimpleNamespace(
        event_sequence=seq,
        event_type=kind,
        error_code=code,
        joint_id=joint,
        value=value,
    )


def session(clock=lambda: 100.0):
    return Session(
        {"tier": "drive"},
        {
            SERVO_FAULT: "servo_fault",
            LIMIT_STOP: "limit_stop",
            PROTECTIVE_STOP: "protective_stop",
            BRAKE: "brake_engage",
        },
        {1: "position"},
        {1: "internal", 2: "external"},
        clock=clock,
    )


def test_context_describes_coe_hardware_and_no_user_configuration():
    context = session_context(CHECK)
    assert context["slaves"] == [
        {
            "ring_position": 0,
            "vendor_id": 0x1234,
            "product_code": 0x5678,
            "axis_count": 2,
        }
    ]
    assert context["axes"] == [{"slave": 0, "mode": "cst"}, {"slave": 0, "mode": "csp"}]
    assert context["tier"] == "drive"
    assert context["sync_mode"] == "drive_default"
    text = json.dumps(context)
    for private in ("secret_line_3", "secret_joint", "12345", "3.25"):
        assert private not in text


def test_a_host_can_name_its_own_tier():
    assert session_context(CHECK, tier="robot")["tier"] == "robot"


def test_cycles_before_op_do_not_count_as_errors():
    s = session()
    s.on_batch(batch([0, 1], wkc=[1, 1]))
    s.on_batch(batch([2, 3]))
    s.on_batch(batch([4, 5], wkc=[1, 0]))
    payload = s.payload(True)
    assert payload["samples"] == 4
    assert payload["wkc_errors"] == 1


def test_timing_is_folded():
    s = session()
    s.on_batch(batch([0, 1, 4], exec_us=20.0, jitter=2.0, miss=1))
    payload = s.payload(True)
    assert payload["overruns"] == 3
    assert payload["lost_samples"] == 2
    assert payload["exec_max_us"] == 20.0
    assert payload["period_dev_max_us"] == 2.0
    assert payload["dc_max_ns"] == 500


def test_events_are_counted_by_name_and_a_retained_one_only_once():
    s = session()
    s.on_event(event(1, BRAKE), **KINDS)
    s.on_event(event(1, BRAKE), **KINDS)
    s.on_event(event(2, BRAKE), **KINDS)
    assert s.payload(True)["events"] == {"brake_engage": 2}


def test_a_refused_clear_is_not_a_protective_stop():
    s = session()
    s.on_event(event(1, PROTECTIVE_STOP, code=1), **KINDS)
    assert s.payload(True)["events"] == {}
    s.on_event(event(2, PROTECTIVE_STOP, code=0), **KINDS)
    assert s.payload(True)["events"] == {"protective_stop": 1}


def test_limit_stops_keep_their_reason():
    s = session()
    s.on_event(event(1, LIMIT_STOP, code=1), **KINDS)
    s.on_event(event(2, LIMIT_STOP, code=9), **KINDS)
    assert s.payload(True)["limit_stops"] == {"position": 1, "code_9": 1}


def test_faults_are_counted_per_axis_and_code():
    s = session()
    s.on_event(event(1, SERVO_FAULT, joint=1, value=0x2310), **KINDS)
    s.on_event(event(2, SERVO_FAULT, joint=1, value=0x2310), **KINDS)
    assert s.payload(True)["faults"] == [{"axis": 1, "code": 0x2310, "count": 2}]


def test_a_stop_source_counts_on_its_rising_edge():
    s = session()
    for bits in (0, 2, 2, 3, 0, 2):
        s.on_status(SimpleNamespace(protective_stop=bits))
    assert s.payload(True)["protective_stop_sources"] == {"external": 2, "internal": 1}


def test_run_time_and_completeness():
    now = [100.0]
    s = session(clock=lambda: now[0])
    now[0] = 160.5
    payload = s.payload(False)
    assert payload["run_s"] == 60.5
    assert payload["complete"] is False


def test_an_idle_session_reports_without_numbers_rather_than_failing():
    payload = session().payload(False)
    assert payload["samples"] == 0
    assert payload["exec_mean_us"] is None
    json.dumps(payload)
