# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The drive bench (bench.py): the rclpy-free parts, and its bench file through the real
drive_model_tool approve/check (no hardware, no ROS context)."""

import importlib.util
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from plem_rt_msgs.msg import RtEvent

_spec = importlib.util.spec_from_file_location(
    "plem_drive_bench", os.environ["PLEM_DRIVE_BENCH_SCRIPT"]
)
bench = importlib.util.module_from_spec(_spec)
sys.modules["plem_drive_bench"] = bench  # dataclasses resolve their module
_spec.loader.exec_module(bench)

PLEM_DRIVE = os.environ["PLEM_DRIVE_BIN"]
EXAMPLE = Path(os.environ["PLEM_DRIVE_EXAMPLE_YAML"])


def check_text(edit=None):
    """A three-slave, five-axis `plem drive check` JSON (drive_synthetic.yaml's
    addressing: ring 5, 0, 2; j0 on channel 2) with a scanned middle slave;
    `edit` changes the dict before it is dumped."""

    def axis(name, slave, ring, channel):
        return {
            "name": name,
            "slave": slave,
            "ring_position": ring,
            "channel": channel,
            "od_offset": 0x800 * (channel - 1),
            "mode": "cst",
            "position_min_rad": -1.0,
            "position_max_rad": 1.5,
            "velocity_limit_rad_s": 2.0,
            "velocity_cap_rad_s": 2.0,
            "acceleration_limit_rad_s2": None,
            "encoder_resolution": 65536,
            "flip": False,
            "rated_torque_mnm": None,
        }

    c = {
        "format": "plem-drive-check/1",
        "schema": "cia402_drive/v1",
        "bus_name": "synthetic",
        "config_digest": "sha256:c0",
        "profile_digest": "sha256:p0",
        "profile_source": "built_in",
        "implementation_revision": "0.9.0",
        "approval_claimed": "unapproved",
        "approval_effective": "unapproved",
        "identity_scope": None,
        "reasons": [],
        "warnings": [],
        "approve_blocker": None,
        "cycle_time_ns": 1000000,
        "command_stale_timeout_ns": 100000000,
        "fault_reset_policy": "none",
        "required_bench_items": "assembly-match idle-non-oe start-rtso release-oe stale-engage disable-engage stop-quiesce".split(),
        "axes": [
            axis("j0", 0, 5, 2),
            axis("j1", 0, 5, 1),
            axis("j2", 1, 0, 1),
            axis("j3", 2, 2, 1),
            axis("j4", 2, 2, 8),
        ],
        "slaves": [
            {"ring_position": 5, "pdo_mapping": "declared", "startup_sdo": []},
            {"ring_position": 0, "pdo_mapping": "scanned", "startup_sdo": []},
            {"ring_position": 2, "pdo_mapping": "declared", "startup_sdo": []},
        ],
        "not_judged": "...",
    }
    if edit:
        edit(c)
    return json.dumps(c)


CHECK = check_text()

# `ethercat slaves -v` output shape, trimmed — a synthetic drive identity
# (the fieldbus test fixtures' kSyntheticVendorId / kSyntheticProductCode).
SLAVES_V = """=== Master 0, Slave 0 ===
Device: Main
State: PREOP
Flag: +
Identity:
  Vendor Id:       0x00c0ffee
  Product code:    0x00001001
  Revision number: 0x00000002
  Serial number:   0x00000000
DL information:
  FMMU bit operation: no
"""


# check output


def test_check_output_parses_into_file_order_addressing():
    c = bench.parse_check(CHECK)
    assert c.required[0] == "assembly-match" and len(c.required) == 7
    assert (
        c.approve_blocker == ""
        and c.cycle_time_ns == 1000000
        and c.fault_reset_policy == "none"
    )
    assert [a.name for a in c.axes] == ["j0", "j1", "j2", "j3", "j4"]
    assert (c.axes[0].slave, c.axes[0].position, c.axes[0].channel) == (0, 5, 2)
    assert (c.axes[0].lo, c.axes[0].hi, c.axes[0].vmax) == (-1.0, 1.5, 2.0)
    assert [(s.position, s.scanned) for s in c.slaves] == [
        (5, False),
        (0, True),
        (2, False),
    ]


def with_modes(*modes):
    """CHECK with axis i declared modes[i] (the rest stay cst)."""

    def edit(c):
        for i, mode in enumerate(modes):
            c["axes"][i]["mode"] = mode

    return check_text(edit)


def test_axis_lines_carry_the_mode():
    assert [a.mode for a in bench.parse_check(with_modes("csp", "csv")).axes] == [
        "csp",
        "csv",
        "cst",
        "cst",
        "cst",
    ]


def test_a_csp_axis_parses_with_its_limits():
    # A csp axis's command is a position, so the runner's neutral — the braked
    # position — holds.
    def edit(c):
        c["axes"][0].update(mode="csp")

    axes = bench.parse_check(check_text(edit)).axes
    assert [a.mode for a in axes] == ["csp", "cst", "cst", "cst", "cst"]
    assert (axes[0].lo, axes[0].hi, axes[0].vmax) == (-1.0, 1.5, 2.0)


def test_sdo_address_adds_the_channels_object_offset():
    c = bench.parse_check(CHECK)
    assert c.axes[1].sdo(0x6041) == 0x6041  # channel 1
    assert c.axes[0].sdo(0x6041) == 0x6841  # channel 2
    assert c.axes[4].sdo(0x6041) == 0x6041 + 7 * 0x800  # channel 8


def test_another_format_or_a_missing_key_is_a_release_mismatch():
    for text in (
        "schema: cia402_drive/v1\n",  # a non-JSON check output
        check_text(lambda c: c.update(format="plem-drive-check/2")),
        check_text(lambda c: c.pop("format")),
    ):
        with pytest.raises(bench.BenchError, match="different releases"):
            bench.parse_check(text)
    for key in (
        "required_bench_items",
        "approve_blocker",
        "command_stale_timeout_ns",
        "bus_name",
        "approval_effective",
    ):
        with pytest.raises(bench.BenchError, match="different releases"):
            bench.parse_check(check_text(lambda c: c.pop(key)))
    for bad in ({"slave": 3}, {"ring_position": 9}, {"channel": 9}):
        with pytest.raises(bench.BenchError, match="inconsistent axis entry for j4"):
            bench.parse_check(check_text(lambda c: c["axes"][4].update(bad)))


def test_a_refusal_shows_the_approval_reasons_of_the_json():
    out = check_text(lambda c: c.update(reasons=["config_digest differs"]))
    refused = SimpleNamespace(stdout=out, stderr="loader mapping line")
    assert bench.refusal_text(refused) == "config_digest differs"
    assert (
        bench.refusal_text(SimpleNamespace(stdout="", stderr="bus refusal"))
        == "bus refusal"
    )


def test_the_approve_blocker_carries_the_reason():
    c = bench.parse_check(
        check_text(
            lambda c: c.update(approve_blocker="the approval block is not canonical")
        )
    )
    assert c.approve_blocker == "the approval block is not canonical"
    assert bench.parse_check(CHECK).approve_blocker == ""


def test_warnings_and_the_stale_timeout_are_read():
    warning = (
        "axes[0] (j0, csp): slaves[0].startup_sdo sets no 0x6884 (Profile deceleration)"
    )
    c = bench.parse_check(check_text(lambda c: c.update(warnings=[warning])))
    assert c.warnings == (warning,)
    assert bench.parse_check(CHECK).warnings == ()
    assert bench.parse_check(CHECK).command_stale_timeout_ns == 100000000


def test_the_log_sits_next_to_the_model(tmp_path):
    assert bench.bench_log(tmp_path / "my.yaml") == tmp_path / "my.bench.log"
    assert bench.bench_log(tmp_path / "my.yml") == tmp_path / "my.bench.log"


# bus observations


def test_slave_info_reads_serial_and_revision():
    assert bench.parse_slave_info(SLAVES_V) == (0, 2)
    assert bench.parse_slave_info("Identity:\n  Vendor Id: 0x1\n") == (None, None)


def test_only_an_absent_object_abort_is_a_null():
    assert bench.upload_failure_is_absence(
        "SDO transfer aborted with code 0x06020000: This object does not exist"
    )
    assert bench.upload_failure_is_absence(
        "SDO transfer aborted with code 0x06090011: Subindex does not exist"
    )
    assert not bench.upload_failure_is_absence(
        "SDO transfer aborted with code 0x05040000: SDO protocol timeout"
    )
    assert not bench.upload_failure_is_absence(
        "Failed to open master device /dev/EtherCAT0: Permission denied"
    )
    assert not bench.upload_failure_is_absence("")


def test_identity_completeness_follows_the_loader():
    assert not bench.Identity(
        serial_number=0
    ).observes_something()  # serial number 0 = not recorded
    assert bench.Identity(
        serial_number=0, revision_number=0
    ).observes_something()  # revision 0 is a read value
    assert bench.Identity(pdo_mapping_digest="sha256:" + "a" * 64).observes_something()
    assert bench.Identity(serial_number=7).observes_something()


def test_info_digests_are_judged_against_the_declared_mappings():
    slaves = bench.parse_check(CHECK).slaves
    ok = ["", "sha256:" + "b" * 64, ""]
    assert bench.digests_match_layout(ok, slaves) == ""
    assert "is scanned" in bench.digests_match_layout(["", "", ""], slaves)
    assert "is declared" in bench.digests_match_layout(
        ["sha256:" + "b" * 64, "sha256:" + "b" * 64, ""], slaves
    )
    assert "2 digests for 3" in bench.digests_match_layout(ok[:2], slaves)


# the observer contract


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def status(n, *, stop=0, operation_enabled=False, engaged=None, pending=0, word=0x0221):
    mask = (1 << n) - 1
    return SimpleNamespace(
        protective_stop=stop,
        operation_enabled=operation_enabled,
        brake_engaged=mask if engaged is None else engaged,
        brake_operation_pending=pending,
        statusword=list(word) if isinstance(word, (list, tuple)) else [word] * n,
    )


def refusal(event_type, joint, code, value, ns):
    """A release-refusal RtEvent (0x62 / 0x5F) as the node publishes it."""
    return SimpleNamespace(
        event_type=event_type,
        joint_id=joint,
        error_code=code,
        value=value,
        monotonic_ns=ns,
    )


def joints(names, q, v=None):
    n = len(names)
    return SimpleNamespace(
        name=list(names),
        position=list(q),
        velocity=list(v or [0.0] * n),
        effort=[0.0] * n,
    )


def guard_for_two():
    c = bench.parse_check(CHECK)
    clock = Clock()
    g = bench.Guard(c.axes[:2], clock=clock)
    return g, clock


def settle(g, clock, q=(0.0, 0.0)):
    g.on_status(status(2))
    g.on_joint_states(joints(["j0", "j1"], q))
    g.on_command(SimpleNamespace(data=[0.0, 0.0]))


def test_silence_blocks_the_next_release():
    g, clock = guard_for_two()
    settle(g, clock)
    g.monitor = True
    clock.t += bench.STATUS_AGE_ABORT_S + 0.01
    with pytest.raises(bench.BenchError, match="observation lost"):
        g.release_gate()


def test_malformed_and_nonfinite_samples_latch():
    g, clock = guard_for_two()
    g.on_joint_states(joints(["j0"], [0.0]))
    with pytest.raises(bench.BenchError, match="joint identity/length"):
        g.guard()
    g, clock = guard_for_two()
    g.on_joint_states(joints(["j0", "j1"], [0.0, float("nan")]))
    with pytest.raises(bench.BenchError, match="nonfinite"):
        g.guard()


def test_a_transient_violation_stays_latched():
    g, clock = guard_for_two()
    g.on_joint_states(
        joints(["j0", "j1"], [0.0, 0.0], v=[5.0, 0.0])
    )  # over velocity_limit 2
    g.on_joint_states(joints(["j0", "j1"], [0.0, 0.0]))  # back inside
    with pytest.raises(bench.BenchError, match="outside the declared limits"):
        g.guard()


def test_a_nonzero_command_or_unknown_state_latches():
    g, clock = guard_for_two()
    g.on_command(SimpleNamespace(data=[0.0, 0.1]))
    with pytest.raises(bench.BenchError, match="neutral"):
        g.guard()
    g, clock = guard_for_two()
    g.on_status(status(2, word=0x0008))  # FAULT
    with pytest.raises(bench.BenchError, match="fault"):
        g.guard()


def test_drift_baseline_is_taken_at_the_release_and_kept():
    g, clock = guard_for_two()
    settle(g, clock, q=(0.2, -0.3))
    g.release_gate()
    assert g.baseline == (0.2, -0.3)
    g.on_joint_states(joints(["j0", "j1"], [0.203, -0.3]))  # 3 mrad: inside
    g.guard()
    g.on_joint_states(joints(["j0", "j1"], [0.2, -0.306]))  # 6 mrad from the baseline
    with pytest.raises(bench.BenchError, match="drift"):
        g.guard()


def test_during_stop_decel_only_the_moving_axes_leave_the_drift_check():
    g, clock = guard_for_two()
    settle(g, clock)
    g.release_gate()
    g.moving = frozenset({0})  # stop-decel moves j0; j1 (a cst axis) rests at 0 Nm
    g.on_joint_states(joints(["j0", "j1"], [0.4, 0.0], v=[1.0, 0.0]))
    g.guard()
    g.on_joint_states(joints(["j0", "j1"], [0.4, 0.006]))
    with pytest.raises(bench.BenchError, match="drift"):
        g.guard()
    g, clock = guard_for_two()
    g.moving = frozenset({0})
    settle(g, clock)
    g.release_gate()
    assert g.moving == frozenset()  # every release starts with every axis checked


def test_a_release_needs_settled_axes_inside_the_limits_and_a_fresh_stream():
    g, clock = guard_for_two()
    settle(g, clock, q=(1.6, 0.0))  # j0 over its 1.5 upper limit
    with pytest.raises(bench.BenchError):
        g.release_gate()
    g, clock = guard_for_two()
    settle(g, clock)
    clock.t += bench.JOINT_STATES_AGE_ABORT_S + 0.01
    g.on_status(status(2))
    g.on_joint_states(joints(["j0", "j1"], [0.0, 0.0]))
    with pytest.raises(bench.BenchError, match="not fresh"):
        g.release_gate()


def test_an_unexpected_stop_or_stale_event_latches():
    g, clock = guard_for_two()
    g.on_status(status(2, stop=2))
    with pytest.raises(bench.BenchError, match="unexpected protective stop"):
        g.guard()
    g, clock = guard_for_two()
    g.on_event(SimpleNamespace(event_type=RtEvent.COMMAND_STALE, monotonic_ns=1))
    with pytest.raises(bench.BenchError, match="stale"):
        g.guard()


def test_a_release_refusal_event_is_kept_as_the_cause_not_as_an_abort():
    # The node answers release_brake with success and refuses at the readiness
    # gate; the only trace is rt_events. The bench keeps 0x62/0x5F as the cause
    # for its timeout messages — they are not violations on their own.
    g, _ = guard_for_two()
    assert g.refusal_note() == ""
    g.on_event(refusal(RtEvent.SLAVE_NOT_OPERATIONAL, 1, 4, 1.0, 1))
    g.on_event(refusal(RtEvent.MODES_OF_OPERATION_MISMATCH, 0, 0x0270, 0.0, 2))
    g.on_event(  # the same refusal again is one cause
        refusal(RtEvent.SLAVE_NOT_OPERATIONAL, 1, 4, 1.0, 3)
    )
    g.guard()
    note = g.refusal_note()
    assert note == (
        " — rt_events: j1: its slave (ring 1) is not in OP (SAFEOP); j0: not in its declared mode "
        "(0x6061 reads 0, statusword 0x0270) — if this drive shows its mode only after enable, write "
        "mode_display_timing: post_enable on this axis in the drive model file. `sudo dmesg | grep -i ethercat` shows the AL status "
        "code or a refused startup SDO (startup_sdo, or dc.write_cycle_time's 0x1C32:02)"
    )
    g2, _ = guard_for_two()
    g2.on_event(refusal(RtEvent.SLAVE_NOT_OPERATIONAL, 0, 0, 0.0, 1))
    assert (
        "j0: its slave (ring 0) is not in OP (offline, or the state could not be read)"
        in g2.refusal_note()
    )


def test_a_start_timeout_with_a_zero_statusword_names_the_missing_process_data():
    # A slave that never leaves PREOP sends no process data: its statusword
    # reads 0 (decoded as NOT_READY_TO_SWITCH_ON, so nothing latches) and the start
    # just times out. The start note names that as one possible cause (a drive still
    # initializing reads 0 too), with the slave hint.
    g, _ = guard_for_two()
    assert g.start_note() == ""
    g.on_status(status(2, word=[0x0270, 0x0000]))
    assert g.start_note() == (
        " — a statusword of 0: the drive may still be initializing (Not ready to switch on), or a slave is not exchanging process data (not in SAFEOP/OP). "
        + bench.SLAVE_HINT
    )
    g.on_status(status(2, word=0x0270))
    assert g.start_note() == ""


def fake_bus(result):
    """An EthercatCli whose `ethercat` calls are answered by `result` (never the real CLI)."""
    b = bench.EthercatCli.__new__(bench.EthercatCli)
    b.master, b.calls = 0, []

    def run(*args, binary=False):
        b.calls.append(args)
        if isinstance(result, Exception):
            raise result
        return result

    b.run = run
    return b


SLAVES_OUT = (
    "0  0:0  PREOP         +  MDDLN55BE\n"
    "1  0:1  OP            +  MDDLN55BE\n"
    "2  0:2  SAFEOP+ERROR  E  0x0000066f:0x60380009\n"
    "xxx LINK FAILURE xxx\n"
)


def test_the_bus_names_the_slaves_not_in_op_from_the_cli():
    ok = SimpleNamespace(returncode=0, stdout=SLAVES_OUT, stderr="")
    b = fake_bus(ok)
    assert b.slaves_not_in_op([0, 1, 2, 3]) == "0 PREOP, 2 SAFEOP+ERROR, 3 absent"
    assert b.calls == [("slaves",)]
    assert fake_bus(ok).slaves_not_in_op([1]) == "", "every file slave in OP"
    failed = SimpleNamespace(returncode=1, stdout="", stderr="Failed to open master")
    assert fake_bus(failed).slaves_not_in_op([0]) is None, "no answer = no fact"
    assert fake_bus(bench.BenchError("timeout")).slaves_not_in_op([0]) is None


def test_a_start_timeout_names_the_slaves_not_in_op_instead_of_guessing():
    g, _ = guard_for_two()
    g.on_status(status(2, word=[0x0270, 0x0000]))
    assert g.start_note("0 PREOP, 1 PREOP") == (
        " — slaves not in OP: 0 PREOP, 1 PREOP. " + bench.SLAVE_HINT
    )
    assert g.start_note("") == "", "the CLI says every slave is in OP: no guess"
    assert "a statusword of 0" in g.start_note(None), "no CLI answer: the guess stays"


def test_the_slave_hint_is_printed_once():
    # A slave refusal and a zero statusword both call for the same hint; the
    # start note carries it once, at the end.
    g, _ = guard_for_two()
    g.on_event(refusal(RtEvent.SLAVE_NOT_OPERATIONAL, 1, 2, 1.0, 1))
    g.on_status(status(2, word=[0x0270, 0x0000]))
    note = g.start_note()
    assert note.count(bench.SLAVE_HINT) == 1, note
    assert note.endswith(bench.SLAVE_HINT)
    assert note.startswith(" — rt_events: ") and "; a statusword of 0:" in note, note
    # A mode refusal alone needs no slave hint.
    g2, _ = guard_for_two()
    g2.on_event(refusal(RtEvent.MODES_OF_OPERATION_MISMATCH, 0, 0x0270, 10.0, 1))
    assert bench.SLAVE_HINT not in g2.refusal_note()


def test_a_drive_resting_in_switch_on_disabled_counts_as_settled():
    # Before its first release the node writes controlword 0x0000: a standard
    # drive rests in Switch on disabled (0x0270), some families in READY_TO_SWITCH_ON.
    g, clock = guard_for_two()
    g.on_status(status(2, word=0x0270))
    g.on_status(status(2, word=0x0270))
    assert g.settled_count == 2
    g.guard()


def test_a_statusword_of_zero_never_counts_as_settled():
    # 0 (no feedback yet) decodes as NOT_READY_TO_SWITCH_ON — never a rest.
    g, clock = guard_for_two()
    g.on_status(status(2, word=0))
    g.on_status(status(2, word=0))
    assert g.settled_count == 0
    g.guard()


def test_only_an_axis_latched_at_idle_may_show_fault_and_only_at_startup():
    g, clock = guard_for_two()
    g.latched = {0: "j0 sw 0x0238"}
    g.on_status(status(2, word=[0x0238, 0]))  # a fault latched before the bench
    g.guard()
    g.on_status(status(2, word=[0x0270, 0x0238]))  # a new fault on another axis
    with pytest.raises(bench.BenchError, match="fault"):
        g.guard()
    g, clock = guard_for_two()
    g.latched = {0: "j0 sw 0x0238"}
    g.phase = "braked"  # after the start, a fault aborts as before
    g.on_status(status(2, word=[0x0238, 0x0270]))
    with pytest.raises(bench.BenchError, match="fault"):
        g.guard()


def test_the_idle_read_records_a_faulted_axis_with_its_error_code(tmp_path):
    b = a_bench(tmp_path)
    words = {
        0x6841: 0x0238,
        0x6041: 0x0270,
    }  # j0 (channel 2) FAULT, j1 SWITCH_ON_DISABLED
    b.ethercat = SimpleNamespace(
        idle=lambda: True,
        statusword=lambda a: words[a.sdo(0x6041)],
        upload=lambda pos, index, typ: b"0xff50 65360\n" if index == 0x683F else None,
    )
    b.log = lambda msg: None
    assert b.idle_statuswords("idle") == {
        0: "j0 sw 0x0238 0x683F 0xFF50 device specific — see the drive manual"
    }


def a_starting_bench(tmp_path, monkeypatch, after_reset):
    """A node that is up with j0 in the FAULT the idle read saw; each
    reset_error is answered at once and followed by two `after_reset` statuses."""
    monkeypatch.setattr(bench, "COMMAND_WAIT_S", 0.3)
    b = a_bench(tmp_path)
    b.node = SimpleNamespace(p=SimpleNamespace(poll=lambda: None))
    b.info = object()
    g = b.guard
    g.latched = {0: "j0 sw 0x0238 0x683F 0xFF50"}
    g.on_joint_states(joints(["j0", "j1"], [0.0, 0.0]))
    g.on_status(status(2, word=[0x0238, 0x0270]))
    sent = []

    def call_async(req):
        sent.append(req)
        for _ in range(2):
            g.on_status(status(2, word=after_reset))
        return FakeFuture(0)

    b.client = SimpleNamespace(service_is_ready=lambda: True, call_async=call_async)
    b.request = lambda **f: f
    lines = []
    b.log = lines.append
    return b, sent, lines


def test_a_start_timeout_reads_the_slave_states_once_and_names_them(
    tmp_path, monkeypatch
):
    # The node never settles (no process data: statusword 0 on every axis).
    # Only then is the master asked, once, and its answer is the cause — the
    # file's slaves in file order (CHECK: ring positions 5, 0, 2).
    monkeypatch.setattr(bench, "NODE_START_S", 0.05)
    b = a_bench(tmp_path)
    b.node = SimpleNamespace(p=SimpleNamespace(poll=lambda: None))
    b.info = object()
    b.guard.on_joint_states(joints(["j0", "j1"], [0.0, 0.0]))
    b.guard.on_status(status(2, word=0x0000))
    out = "".join(f"{p}  0:{p}  PREOP  +  drive\n" for p in range(8))
    b.ethercat = fake_bus(SimpleNamespace(returncode=0, stdout=out, stderr=""))
    b.log = lambda msg: None
    with pytest.raises(
        bench.BenchError, match="slaves not in OP: 5 PREOP, 0 PREOP, 2 PREOP"
    ) as caught:
        b.await_start()
    assert b.ethercat.calls == [("slaves",)]
    assert "a statusword of 0" not in str(caught.value), "the fact replaces the guess"
    assert str(caught.value).endswith(bench.SLAVE_HINT)


def test_a_fault_latched_before_the_bench_is_reset_once_with_a_warning(
    tmp_path, monkeypatch
):
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0270, 0x0270])
    assert b.await_start() is True
    assert sent == [{"reset_error": True}]
    warn = (
        "WARN fault already latched before the bench started: j0 sw 0x0238 0x683F 0xFF50 — "
        "reset_error sent once"
    )
    assert warn in lines and b.notes == [warn]


def test_the_one_reset_waits_until_every_axis_reports(tmp_path, monkeypatch):
    # On distributed clocks a slave can reach OP seconds after another: j0 shows
    # the FAULT the idle read saw while j1 has no process data yet (status word
    # 0). A reset sent then does not reach j1, which stays in FAULT — so the one
    # reset waits until every axis reports; here j1 never does, so the start
    # times out naming the slave states, with no reset sent.
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0270, 0x0270])
    monkeypatch.setattr(bench, "NODE_START_S", 0.05)
    b.guard.latched = {0: "j0 sw 0x0238 0x683F 0xFF50", 1: "j1 sw 0x0238 0x683F 0xFF50"}
    b.guard.on_status(status(2, word=[0x0238, 0x0000]))
    out = "".join(f"{p}  0:{p}  OP  +  drive\n" for p in range(8))
    b.ethercat = fake_bus(SimpleNamespace(returncode=0, stdout=out, stderr=""))
    with pytest.raises(bench.BenchError, match="start:"):
        b.await_start()
    assert sent == []


def test_the_one_reset_goes_once_the_late_axis_reports(tmp_path, monkeypatch):
    # The same late slave, now reporting a moment later: the reset is sent
    # exactly once, after j1's first status word, and the start settles.
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0270, 0x0270])
    b.guard.latched = {0: "j0 sw 0x0238 0x683F 0xFF50", 1: "j1 sw 0x0238 0x683F 0xFF50"}
    b.guard.on_status(status(2, word=[0x0238, 0x0000]))
    at_reset = []  # the status words when the reset went out
    settle = b.client.call_async

    def call_async(req):
        at_reset.append(list(b.guard.status.statusword))
        return settle(req)

    b.client = SimpleNamespace(service_is_ready=lambda: True, call_async=call_async)
    late = threading.Timer(
        0.05, lambda: b.guard.on_status(status(2, word=[0x0238, 0x0238]))
    )
    late.start()
    try:
        assert b.await_start() is True
    finally:
        late.cancel()
    assert sent == [{"reset_error": True}]
    assert at_reset == [[0x0238, 0x0238]], "the reset waited for j1's first status word"
    assert b.guard.settled_count >= 2


def test_a_fault_still_there_after_the_one_reset_fails_fast(tmp_path, monkeypatch):
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0238, 0x0270])
    with pytest.raises(
        bench.BenchError,
        match="j0 0x0238 still in Fault after one reset_error .* when the drive reported one",
    ):
        b.await_start()
    assert sent == [{"reset_error": True}]  # no retry


def test_a_fault_clearing_between_samples_never_starts_on_one_settled_status(
    tmp_path, monkeypatch
):
    # One healthy status lands right after the wait saw the latched FAULT: the
    # start must not go on with a single settled sample behind it.
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0270, 0x0270])

    def log(msg):
        lines.append(msg)
        if msg.startswith("PASS start:"):
            b.guard.on_status(status(2, word=0x0270))

    b.log = log
    b.await_start()
    assert b.guard.settled_count >= 2


def test_settled_statuses_known_only_at_the_reset_deadline_are_late(
    tmp_path, monkeypatch
):
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0270, 0x0270])
    monkeypatch.setattr(bench, "COMMAND_WAIT_S", 0.0)  # the deadline passes at once
    with pytest.raises(bench.BenchError, match="settled statuses after the reset"):
        b.await_start()
    assert sent == [{"reset_error": True}]


def test_a_reset_that_clears_the_fault_but_never_settles_fails(tmp_path, monkeypatch):
    # j0 leaves FAULT but reads 0 (NOT_READY_TO_SWITCH_ON) — not a rest: the generic
    # start failure.
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0000, 0x0270])
    with pytest.raises(bench.BenchError, match="settled statuses after the reset"):
        b.await_start()
    assert sent == [{"reset_error": True}]


def test_a_refused_reset_names_a_remedy(tmp_path, monkeypatch):
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0238, 0x0270])
    b.client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda req: FakeFuture(0, success=False),
    )
    with pytest.raises(
        bench.BenchError, match="reset_error refused .* clear the drive alarm"
    ):
        b.await_start()


def test_a_clean_start_sends_no_reset(tmp_path, monkeypatch):
    b, sent, lines = a_starting_bench(tmp_path, monkeypatch, [0x0270, 0x0270])
    b.guard.on_status(status(2, word=0x0270))
    b.guard.on_status(status(2, word=0x0270))
    assert b.await_start() is False
    assert sent == [] and b.notes == []


# the bench file through the real tool


def run(*args):
    return subprocess.run(
        [PLEM_DRIVE, *args], capture_output=True, text=True, timeout=60
    )


def test_the_bench_file_is_approved_by_the_real_tool(tmp_path):
    assert os.access(PLEM_DRIVE, os.X_OK), (
        f"{PLEM_DRIVE} missing — the build must provide plem-drive"
    )
    model = tmp_path / "my_drive.yaml"
    shutil.copy(EXAMPLE, model)
    checked = run("check", "--json", str(model))
    assert checked.returncode == 0, checked.stderr
    c = bench.parse_check(checked.stdout)
    assert c.required and not c.approve_blocker
    identities = [
        bench.Identity(
            serial_number=0,
            revision_number=2,
            hardware_version="1e3",  # reads as a number unless quoted
            software_version="on",  # reads as a boolean unless quoted
            pdo_mapping_digest="sha256:" + "a" * 64 if s.scanned else None,
        )
        for s in c.slaves
    ]
    items = {
        i: ("PASS", f'{i}: "quoted" \\ back\\slash, \ud55c\uae00') for i in c.required
    }
    items["fault-reset"] = ("NA", "fault_reset.policy none")
    conditions = {
        "host": "test",
        "master": "IgH test",
        "date": "2026-09-24",
        "measured_by": "plem drive run (bench)",
        "notes": "",
    }
    bench_path = tmp_path / "evidence" / "bench.yaml"
    bench_path.parent.mkdir()
    bench_path.write_text(
        bench.dump_bench(bench.bench_document(c, identities, items, conditions))
    )
    done = run("approve", str(model), str(bench_path))
    assert done.returncode == 0, done.stderr
    assert sorted(p.name for p in tmp_path.iterdir()) == ["evidence", "my_drive.yaml"]
    shutil.rmtree(bench_path.parent)  # the evidence is inside the model file now
    back = run("check", "--json", str(model))
    assert back.returncode == 0, back.stdout + back.stderr
    assert json.loads(back.stdout)["approval_effective"] == "approved"


def test_a_failed_item_is_refused_by_approve(tmp_path):
    model = tmp_path / "m.yaml"
    shutil.copy(EXAMPLE, model)
    c = bench.parse_check(run("check", "--json", str(model)).stdout)
    identities = [
        bench.Identity(
            serial_number=0,
            revision_number=2,
            pdo_mapping_digest="sha256:" + "a" * 64 if s.scanned else None,
        )
        for s in c.slaves
    ]
    items = {i: ("PASS", "ok") for i in c.required}
    items["release-oe"] = ("FAIL", "did not reach OPERATION_ENABLED")
    conditions = {
        "host": "h",
        "master": "m",
        "date": "2026-09-24",
        "measured_by": "b",
        "notes": "",
    }
    before = model.read_text()
    bench_path = tmp_path / "bench.yaml"
    bench_path.write_text(
        bench.dump_bench(bench.bench_document(c, identities, items, conditions))
    )
    done = run("approve", str(model), str(bench_path))
    assert done.returncode != 0 and "'release-oe' as FAIL" in done.stderr
    assert model.read_text() == before


# lifecycle, deadlines, ownership, payloads


def a_bench(tmp_path, axes=2):
    args = SimpleNamespace(namespace="/t", master=0, model=tmp_path / "m.yaml")
    c = bench.parse_check(CHECK)
    c = bench.Check(**{**c.__dict__, "axes": c.axes[:axes]})
    return bench.Bench(args, c, None, None)


def test_a_violation_latched_after_the_last_check_voids_the_bench(tmp_path):
    b = a_bench(tmp_path)
    b.guard.latch(False, "drift during the final SDO read")
    assert b.cleanup() is False


class FakeFuture:
    def __init__(self, after, success=True):
        self.t = time.monotonic() + after
        self.r = SimpleNamespace(success=success, message="")

    def done(self):
        return time.monotonic() >= self.t

    def result(self):
        return self.r


def settle_later(b, after):
    def go():
        time.sleep(after)
        b.guard.status = status(
            2, operation_enabled=True, engaged=0, word=0x0227
        )  # every axis OPERATION_ENABLED

    threading.Thread(target=go, daemon=True).start()


def test_one_deadline_covers_the_answer_and_the_settled_state(tmp_path):
    # A plem.drive request: the bench's deadline runs from the request, not
    # from the client's return.
    b = a_bench(tmp_path)
    b.log = lambda msg: None
    settle_later(b, 2.0)
    b.through_drive(
        "t", lambda: time.sleep(1.0), b.guard.ready, "released"
    )  # 2.0 s < 3 s from the request
    b = a_bench(tmp_path)
    b.log = lambda msg: None
    settle_later(b, 3.5)  # 2 s answer + 1.5 s settle each fit alone, not together
    with pytest.raises(bench.BenchError, match="released within 3 s of the request"):
        b.through_drive("t", lambda: time.sleep(2.0), b.guard.ready, "released")


def test_the_observer_aborts_while_the_client_still_waits(tmp_path):
    # Bus.release waits on its own (seconds); a violation the observer latches
    # meanwhile — an axis falling with its brake open — aborts at once.
    b = a_bench(tmp_path)
    b.log = lambda msg: None
    released = threading.Event()
    threading.Timer(0.1, lambda: b.guard.latch(False, "drift on j0")).start()
    t0 = time.monotonic()
    with pytest.raises(bench.BenchError, match="drift on j0"):
        b.through_drive("t", lambda: released.wait(5.0), b.guard.ready, "released")
    assert time.monotonic() - t0 < 0.5, (
        "within the observer's polling, not the client's wait"
    )
    released.set()


def test_a_ramp_whose_client_refuses_stops_and_names_the_reason():
    lines, sent = [], []

    def publish(values):
        sent.append(values)
        raise ValueError("v: 9 rad/s is outside limits.velocity_rad_s")

    ramp = bench.Ramp(publish, lambda: {"v": 9.0}, lines.append)
    ramp.thread.join(timeout=5.0)
    assert not ramp.thread.is_alive(), "the refusal ends the publishing by itself"
    assert len(sent) == 1, "it stops publishing: the bus's command watch trips"
    assert any("outside limits.velocity_rad_s" in line for line in lines)
    with pytest.raises(bench.BenchError, match="ramp stream stopped: v: 9 rad/s"):
        ramp.end()


def test_a_ramp_refused_before_the_release_names_its_reason(tmp_path, monkeypatch):
    monkeypatch.setattr(bench, "STREAM_START_S", 5.0)
    b = a_bench_of(tmp_path, "csp", "csv")
    b.log = lambda msg: None

    def refused(values):
        raise ValueError("j1: 9 rad/s is outside limits.velocity_rad_s")

    t0 = time.monotonic()
    with pytest.raises(bench.BenchError, match="stopped: j1: 9 rad/s"):
        b.start_stream(lambda topic, rest: bench.Ramp(refused, lambda: {}, b.log))
    assert time.monotonic() - t0 < 1.0, "not the whole stream start wait"


def test_the_observer_keeps_watching_while_the_csp_move_ends(tmp_path):
    b = a_bench(tmp_path)
    b.log = lambda msg: None
    move = bench.CspMove(lambda: time.sleep(3.0), 0.0)
    threading.Timer(0.1, lambda: b.guard.latch(False, "drift on j0")).start()
    t0 = time.monotonic()
    with pytest.raises(bench.BenchError, match="drift on j0"):
        b.end_move(move, time.monotonic())
    assert time.monotonic() - t0 < 0.5


def test_a_csp_move_that_does_not_end_is_a_refusal(tmp_path, monkeypatch):
    monkeypatch.setattr(bench, "COMMAND_WAIT_S", -2.95)  # 0.05 s to end
    b = a_bench(tmp_path)
    b.log = lambda msg: None
    with pytest.raises(bench.BenchError, match="did not end"):
        b.end_move(bench.CspMove(lambda: time.sleep(1.0), 0.0), time.monotonic())


def test_a_refused_drive_request_fails_with_the_buses_reason(tmp_path):
    b = a_bench(tmp_path)

    def refused():
        raise RuntimeError("/t: release_brake: refused (P-STOP)")

    with pytest.raises(bench.BenchError, match="release 1: /t: release_brake: refused"):
        b.through_drive("release 1", refused, b.guard.ready, "released")


def test_a_stream_counts_only_its_own_new_commands(tmp_path, monkeypatch):
    monkeypatch.setattr(bench, "STREAM_START_S", 0.5)
    b = a_bench(tmp_path)
    b.guard.commands = 500  # an earlier stream's count must not satisfy the new one
    b.publisher = lambda topic, n: SimpleNamespace(end=lambda: 0)
    with pytest.raises(bench.BenchError, match="new commands"):
        b.start_stream()

    def feeding(topic, n):
        def go():
            time.sleep(0.1)
            for _ in range(
                bench.STREAM_READY
            ):  # through the observer's callback and its lock
                b.guard.on_command(SimpleNamespace(data=[0.0] * len(b.axes)))

        threading.Thread(target=go, daemon=True).start()
        return SimpleNamespace(end=lambda: 0)

    b.publisher = feeding
    b.start_stream()


def test_the_node_is_owned_before_its_checks_so_cleanup_reaps_it(tmp_path, monkeypatch):
    b = a_bench(tmp_path)
    monkeypatch.setattr(
        bench.DriveProcess,
        "check_sigint",
        lambda self: (_ for _ in ()).throw(bench.BenchError("status unreadable")),
    )
    with pytest.raises(bench.BenchError):
        b.start_node(["sleep", "30"])
    assert b.node is not None
    proc = b.node.p
    assert (
        b.cleanup() is False
    )  # SIGINT ends `sleep` with a nonzero RC — not a confirmed quiesce
    assert proc.returncode is not None and b.node is None  # signalled and reaped


FAKE_CHAIN = """
import signal, sys, time
def stop(*_):
    print({line!r}, file=sys.stderr, flush=True)
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
print("ready", flush=True)
time.sleep(30)
"""
UNCONFIRMED = (
    "[ERROR] [1.0] [DriveSystemInterface]: brakes were NOT confirmed engaged on stop "
    "(engaged mask 0x01 of 0x03, timeout mask 0x02, statuswords j0=0x0250 j1=0x0237)"
)


CONFIRMED = (
    "[INFO] [1.0] [DriveSystemInterface]: brakes confirmed engaged on stop "
    "(engaged mask 0x03 of 0x03)"
)
NEVER_RELEASED = (
    "[INFO] [1.0] [DriveSystemInterface]: brakes were never released this run — the stop observed no brake "
    "(2 joints, none released)"
)


@pytest.mark.parametrize(
    "logged, clean",
    [
        (CONFIRMED, True),
        (UNCONFIRMED, False),
        # Silence is not confirmation: a plugin that never stopped (or whose
        # line was lost) leaves the brakes unknown.
        ("stopped", False),
        (CONFIRMED + "\n" + UNCONFIRMED, False),
        # A stop with nothing released observed no brake: not a confirmation.
        (NEVER_RELEASED, False),
    ],
)
def test_a_stop_qualifies_only_on_the_plugins_confirmation_though_the_chain_exits_0(
    tmp_path, logged, clean
):
    # ros2_control exits 0 either way: the hardware plugin's line in the
    # chain's output (the bench log) is the only word on its brakes — the
    # affirmative line must be there and the negative one absent.
    b = a_bench(tmp_path)
    log = tmp_path / "m.bench.log"
    with open(log, "w") as sink:
        b.sink = sink
        b.log("a bench line before the chain")
        b.start_node([sys.executable, "-c", FAKE_CHAIN.format(line=logged)])
        deadline = time.monotonic() + 10
        while "ready" not in log.read_text() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert keep_handlers(b.cleanup) is clean
    if logged == "stopped":
        assert "did not report its brakes confirmed engaged" in log.read_text()
    if logged == NEVER_RELEASED:  # the plugin's word, not "unknown"
        text = log.read_text()
        assert "it reported instead: " + NEVER_RELEASED in text
        assert "unknown" not in text


def test_the_bench_looks_for_the_hardware_plugins_own_words():
    # Each marker is one fact: drive_system_interface.cpp writes it (its test
    # pins the prefix), the bench reads the same literal.
    source = (
        Path(__file__).resolve().parents[2]
        / "src"
        / "ros"
        / "drive_system_interface.cpp"
    ).read_text()
    assert '"' + bench.UNCONFIRMED_BRAKES in source
    assert '"' + bench.CONFIRMED_BRAKES in source
    assert '"' + bench.NEVER_RELEASED_BRAKES in source


def test_the_bench_runs_the_chain_verbose_so_the_info_confirmation_is_printed():
    argv = bench.run_argv(
        SimpleNamespace(namespace="b", model="m.yaml", master=0, rt_cpu=None), "tool"
    )
    assert "--verbose" in argv


def test_string_payloads_keep_their_bytes_or_are_recorded_null():
    assert bench.string_payload(b"2.07\n") == ("2.07", "")
    assert (
        bench.string_payload(b" 2.07 \n")[0] == " 2.07 "
    )  # the bus compares bytes: no trimming
    assert (
        bench.string_payload(b"0.2\x00\x00\n")[0] is None
    )  # NUL padding cannot be recorded
    assert bench.string_payload(b"\xff\xfe\n")[0] is None
    assert bench.string_payload(b"x" * 32 + b"\n")[0] is None


def test_a_csp_or_csv_file_is_benched_not_refused(tmp_path, monkeypatch):
    # Preflight goes on to the master for any mode.
    touched = []

    def bus(master):
        touched.append(master)
        raise bench.BenchError("stop here")

    monkeypatch.setattr(bench, "EthercatCli", bus)
    calls = []

    def tool(*a):
        calls.append(a)
        return SimpleNamespace(returncode=0, stdout=with_modes("csp", "csv"), stderr="")

    monkeypatch.setattr(bench, "run_tool", tool)
    args = SimpleNamespace(model=tmp_path / "m.yaml", master=0)
    with pytest.raises(bench.BenchError, match="stop here"):
        bench.preflight(args, "drive_model_tool", print)
    assert touched == [0]
    assert calls == [("drive_model_tool", "check", "--json", str(args.model))]


@pytest.mark.parametrize("answer", ["n", EOFError, KeyboardInterrupt])
def test_a_declined_or_abandoned_confirmation_touches_nothing(
    tmp_path, monkeypatch, answer
):
    monkeypatch.setattr(bench, "run_tool", checked_as(0, CHECK))
    monkeypatch.setattr(
        bench,
        "EthercatCli",
        lambda m: SimpleNamespace(check_access=lambda: None, idle=lambda: True),
    )
    monkeypatch.setattr(bench, "bus_users", lambda: [])

    def ask(prompt):
        if isinstance(answer, str):
            return answer
        raise answer()

    monkeypatch.setattr("builtins.input", ask)
    args = SimpleNamespace(model=tmp_path / "m.yaml", master=0)
    with pytest.raises(bench.BenchError, match="not confirmed — nothing was touched"):
        bench.preflight(args, "drive_model_tool", print)


def test_the_bench_speaks_as_plem_drive_run_and_takes_no_yes(tmp_path, capsys):
    with pytest.raises(SystemExit):
        bench.main([str(tmp_path / "m.yaml"), "--yes"])
    err = capsys.readouterr().err
    assert "plem drive run" in err and "--yes" in err


def checked_as(returncode, stdout, stderr=""):
    return lambda *a: SimpleNamespace(
        returncode=returncode, stdout=stdout, stderr=stderr
    )


def test_preflight_benches_an_approved_file(tmp_path, monkeypatch):
    # plem drive run offers it when the drives differ from the record: a
    # re-bench replaces the approval.
    approved = check_text(
        lambda c: c.update(approval_claimed="approved", approval_effective="approved")
    )
    monkeypatch.setattr(bench, "run_tool", checked_as(0, approved))

    def bus(master):
        raise bench.BenchError("stop here")

    monkeypatch.setattr(bench, "EthercatCli", bus)
    args = SimpleNamespace(model=tmp_path / "m.yaml", master=0)
    with pytest.raises(bench.BenchError, match="stop here"):
        bench.preflight(args, "drive_model_tool", print)


def test_the_confirmation_says_an_approval_is_replaced():
    c = bench.parse_check(CHECK)
    replaced = "The file's current approval is replaced when this bench passes."
    assert replaced not in bench.confirm_text("m.yaml", c)
    approved = bench.Check(**{**c.__dict__, "claimed": "approved"})
    assert replaced in bench.confirm_text("m.yaml", approved)


def test_preflight_benches_a_file_whose_record_no_longer_holds(tmp_path, monkeypatch):
    # check exits 1 with its JSON: the claim is approved, the record does not
    # hold (an edit after the bench) — a re-bench replaces it.
    stale = check_text(
        lambda c: c.update(
            approval_claimed="approved",
            approval_effective="unapproved",
            reasons=["approval.record.config_digest is 'sha256:c9' but …"],
        )
    )
    monkeypatch.setattr(bench, "run_tool", checked_as(1, stale))

    def bus(master):
        raise bench.BenchError("stop here")

    monkeypatch.setattr(bench, "EthercatCli", bus)
    said = []
    args = SimpleNamespace(model=tmp_path / "m.yaml", master=0)
    with pytest.raises(bench.BenchError, match="stop here"):
        bench.preflight(args, "plem-drive", said.append)
    assert any(
        "approval.record.config_digest is 'sha256:c9'" in line for line in said
    ), said


def test_preflight_refuses_what_check_refuses(tmp_path, monkeypatch):
    monkeypatch.setattr(bench, "run_tool", checked_as(1, "", "bus refusal"))
    args = SimpleNamespace(model=tmp_path / "m.yaml", master=0)
    with pytest.raises(
        bench.BenchError, match="plem drive check refused the file:\nbus refusal"
    ):
        bench.preflight(args, "drive_model_tool", print)


def test_preflight_shows_checks_warnings_before_the_bus(tmp_path, monkeypatch):
    warned = check_text(
        lambda c: c.update(
            warnings=["axes[0] (j0, csp): slaves[0].startup_sdo sets no 0x6884"]
        )
    )
    monkeypatch.setattr(
        bench,
        "run_tool",
        lambda *a: SimpleNamespace(returncode=0, stdout=warned, stderr=""),
    )

    def no_cli(master):
        raise bench.BenchError("stop here")

    monkeypatch.setattr(bench, "EthercatCli", no_cli)
    said = []
    args = SimpleNamespace(model=tmp_path / "m.yaml", master=0)
    with pytest.raises(bench.BenchError, match="stop here"):
        bench.preflight(args, "plem-drive", said.append)
    assert said == ["WARN axes[0] (j0, csp): slaves[0].startup_sdo sets no 0x6884"]


def preflight_through_main(tmp_path, monkeypatch, bus):
    """main() up to its confirmation: check answers CHECK, `bus` stands in for EthercatCli."""
    model = tmp_path / "m.yaml"
    model.write_text("x")
    monkeypatch.setattr(
        bench, "drive_model_tool", lambda: str(tmp_path / "drive_model_tool")
    )
    monkeypatch.setattr(
        bench,
        "run_tool",
        lambda *a: SimpleNamespace(returncode=0, stdout=CHECK, stderr=""),
    )
    monkeypatch.setattr(bench, "EthercatCli", bus)
    monkeypatch.setattr(bench, "bus_users", lambda: [])
    monkeypatch.setattr(
        bench.signal, "signal", lambda *a: None
    )  # keep pytest's own handlers
    return model


def test_a_refusal_before_the_confirmation_writes_no_log(tmp_path, monkeypatch, capsys):
    def no_cli(master):
        raise bench.BenchError("the IgH ethercat CLI is not installed")

    model = preflight_through_main(tmp_path, monkeypatch, no_cli)
    assert bench.main([str(model)]) == 1
    assert sorted(p.name for p in tmp_path.iterdir()) == ["m.yaml"]
    assert capsys.readouterr().err == (
        "plem drive run: the IgH ethercat CLI is not installed\n"
    )


@pytest.mark.parametrize("answer", ["n", EOFError, KeyboardInterrupt])
def test_a_declined_confirmation_says_one_line_and_keeps_the_last_log(
    tmp_path, monkeypatch, capsys, answer
):
    model = preflight_through_main(
        tmp_path,
        monkeypatch,
        lambda m: SimpleNamespace(check_access=lambda: None, idle=lambda: True),
    )
    last = tmp_path / "m.bench.log"
    last.write_text("the last bench's log\n")

    def ask(prompt):
        if isinstance(answer, str):
            return answer
        raise answer()

    monkeypatch.setattr("builtins.input", ask)
    assert bench.main([str(model)]) == 1
    assert last.read_text() == "the last bench's log\n"
    assert capsys.readouterr().err == (
        "plem drive run: not confirmed — nothing was touched\n"
    )
    last.unlink()
    assert bench.main([str(model)]) == 1
    assert sorted(p.name for p in tmp_path.iterdir()) == ["m.yaml"]


def test_a_file_of_another_user_is_refused_before_anything(
    tmp_path, monkeypatch, capsys
):
    # approve refuses it (a replacement would make it the caller's): said
    # before the bus is touched, not after the bench.
    model = tmp_path / "m.yaml"
    model.write_text("x")
    monkeypatch.setattr(bench.os, "geteuid", lambda: model.stat().st_uid + 1)
    monkeypatch.setattr(bench, "run_tool", lambda *a: pytest.fail("ran check"))
    assert bench.main([str(model)]) == 1
    err = capsys.readouterr().err
    assert f"is owned by uid {model.stat().st_uid}, not by you" in err, err
    assert sorted(p.name for p in tmp_path.iterdir()) == ["m.yaml"]


# deadline boundary, teardown result, cleanup under fault/signal


def test_a_deadline_that_passed_fails_even_if_the_state_arrives_right_after(tmp_path):
    b = a_bench(tmp_path)
    deadline = time.monotonic() + 0.05
    assert b.wait_until(lambda: time.monotonic() >= deadline, deadline) is False

    def slow_true():  # true, but only known after the deadline
        time.sleep(0.1)
        return True

    assert b.wait_until(slow_true, time.monotonic() + 0.05) is False
    assert b.wait_until(lambda: True, time.monotonic() + 1.0) is True


class FakeRos:
    def __init__(self, shutdown=True, alive=False, destroy_raises=False):
        self.shutdown_ok, self.alive, self.destroy_raises = (
            shutdown,
            alive,
            destroy_raises,
        )
        self.done = []

    def resources(self):
        me = self

        class Executor:
            def shutdown(self, timeout_sec):
                me.done.append("executor")
                return me.shutdown_ok

        class Thread:
            def join(self, timeout):
                me.done.append("thread")

            def is_alive(self):
                return me.alive

        class Node:
            def destroy_node(self):
                me.done.append("node")
                if me.destroy_raises:
                    raise RuntimeError("destroy failed")

        class Rclpy:
            def try_shutdown(self):
                me.done.append("rclpy")

        return {
            "closing": threading.Event(),
            "executor": Executor(),
            "thread": Thread(),
            "node": Node(),
            "rclpy": Rclpy(),
        }


@pytest.mark.parametrize(
    "fault", [dict(shutdown=False), dict(alive=True), dict(destroy_raises=True)]
)
def test_an_observer_not_confirmed_stopped_voids_the_bench(tmp_path, fault):
    b = a_bench(tmp_path)
    ros = FakeRos(**fault)
    b.ros = ros.resources()
    assert b.cleanup() is False
    assert ros.done == ["executor", "thread", "node", "rclpy"]  # every step still ran


def test_a_clean_ros_teardown_leaves_cleanup_clean(tmp_path):
    b = a_bench(tmp_path)
    ros = FakeRos()
    b.ros = ros.resources()
    assert b.cleanup() is True


def keep_handlers(fn):
    saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        return fn()
    finally:
        for s, h in saved.items():
            signal.signal(s, h)


@pytest.mark.parametrize("fault", ["exception", "sigterm", "sigint"])
def test_cleanup_runs_every_step_through_a_fault_or_a_signal(tmp_path, fault):
    b = a_bench(tmp_path)
    ros = FakeRos()
    b.ros = ros.resources()

    def end():
        if fault == "exception":
            raise OSError("kill failed")
        os.kill(os.getpid(), signal.SIGTERM if fault == "sigterm" else signal.SIGINT)
        time.sleep(0.05)  # the handler runs here
        return 0

    b.pub = SimpleNamespace(end=end)

    def go():
        signal.signal(signal.SIGINT, signal.default_int_handler)
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
        return b.cleanup()

    assert (
        keep_handlers(go) is False
    )  # an interrupted or faulted cleanup never approves
    assert ros.done == ["executor", "thread", "node", "rclpy"]


# a failing log, handler restore, a signal before approval


class BrokenSink:
    def write(self, text):
        raise OSError(28, "No space left on device")

    def flush(self):
        pass


def test_a_log_that_cannot_be_written_does_not_cut_cleanup_short(tmp_path):
    b = a_bench(tmp_path)
    b.sink = BrokenSink()
    ros = FakeRos(shutdown=False)  # a failure that wants to be logged
    b.ros = ros.resources()
    b.pub = SimpleNamespace(end=lambda: (_ for _ in ()).throw(OSError("kill failed")))
    assert keep_handlers(b.cleanup) is False
    assert ros.done == ["executor", "thread", "node", "rclpy"]


def test_cleanup_holds_signals_only_for_the_teardown(tmp_path):
    def go():
        def mine(*_):
            sys.exit(143)

        signal.signal(signal.SIGTERM, mine)
        a_bench(tmp_path).cleanup()
        return signal.getsignal(signal.SIGTERM) is mine

    assert keep_handlers(go)


def test_the_bench_runs_on_its_private_namespace(tmp_path, monkeypatch):
    model = tmp_path / "m.yaml"
    model.write_text("x")
    monkeypatch.setattr(
        bench, "drive_model_tool", lambda: str(tmp_path / "drive_model_tool")
    )
    monkeypatch.setattr(
        bench,
        "preflight",
        lambda *a: (SimpleNamespace(version=lambda: "1.6"), bench.parse_check(CHECK)),
    )
    seen = []

    class Recording:
        def __init__(self, args, *rest):
            seen.append(args.namespace)
            raise bench.BenchError("recorded")

    monkeypatch.setattr(bench, "Bench", Recording)
    assert keep_handlers(lambda: bench.main([str(model)])) == 1
    # Private: a Bus("<bus.name>") client cannot attach to the bench's bus.
    assert seen == ["/plem_drive_bench"]


def test_a_signal_after_cleanup_stops_the_approval_and_keeps_the_log(
    tmp_path, monkeypatch, capsys
):
    model = tmp_path / "m.yaml"
    model.write_text("x")
    monkeypatch.setattr(
        bench, "drive_model_tool", lambda: str(tmp_path / "drive_model_tool")
    )
    monkeypatch.setattr(
        bench,
        "preflight",
        lambda *a: (SimpleNamespace(version=lambda: "1.6"), bench.parse_check(CHECK)),
    )

    class Passing:
        items = {}
        notes = []

        def __init__(self, *a):
            pass

        def run(self):
            return []

        def cleanup(self):
            return True

    monkeypatch.setattr(bench, "Bench", Passing)
    monkeypatch.setattr(bench, "bench_document", lambda *a: {})

    def terminated(document):  # SIGTERM lands while the evidence is written
        os.kill(os.getpid(), signal.SIGTERM)
        time.sleep(0.05)
        return ""

    monkeypatch.setattr(bench, "dump_bench", terminated)
    approvals = []
    monkeypatch.setattr(
        bench,
        "run_tool",
        lambda *a: (
            approvals.append(a) or SimpleNamespace(returncode=0, stdout="", stderr="")
        ),
    )
    assert keep_handlers(lambda: bench.main([str(model)])) == 1
    assert approvals == []
    assert "interrupted" in (tmp_path / "m.bench.log").read_text()
    assert "the log is" in capsys.readouterr().err


def test_a_signal_right_after_approve_keeps_the_log_and_names_the_model(
    tmp_path, monkeypatch, capsys
):
    model = tmp_path / "m.yaml"
    model.write_text("x")
    monkeypatch.setattr(
        bench, "drive_model_tool", lambda: str(tmp_path / "drive_model_tool")
    )
    monkeypatch.setattr(
        bench,
        "preflight",
        lambda *a: (SimpleNamespace(version=lambda: "1.6"), bench.parse_check(CHECK)),
    )

    class Passing:
        items = {}
        notes = []

        def __init__(self, *a):
            pass

        def run(self):
            return []

        def cleanup(self):
            return True

    monkeypatch.setattr(bench, "Bench", Passing)
    monkeypatch.setattr(bench, "bench_document", lambda *a: {})
    monkeypatch.setattr(
        bench,
        "run_tool",
        lambda *a: SimpleNamespace(returncode=0, stdout="approved", stderr=""),
    )
    real = bench.log_line

    def log_line(sink, msg):  # SIGTERM lands while the approval is reported
        real(sink, msg)
        if msg.startswith("approval recorded in"):
            os.kill(os.getpid(), signal.SIGTERM)
            time.sleep(0.05)

    monkeypatch.setattr(bench, "log_line", log_line)
    assert keep_handlers(lambda: bench.main([str(model)])) == 1
    log = (tmp_path / "m.bench.log").read_text()
    assert "interrupted" in log and f"`plem drive check {model}`" in log


# the neutral command per axis mode


def test_the_cst_stream_at_rest_is_a_zero_vector(monkeypatch):
    argv = []
    monkeypatch.setattr(bench.subprocess, "Popen", lambda a, **kw: argv.append(a))
    bench.Publisher("/t/plem_drive/command", (0.0, 0.0))
    assert argv[0] == [  # the exact ros2 topic pub command line
        "setpriv",
        "--pdeathsig",
        "KILL",
        "ros2",
        "topic",
        "pub",
        "-r",
        str(bench.PUBLISH_HZ),
        "/t/plem_drive/command",
        "std_msgs/msg/Float64MultiArray",
        "{data: [0.0, 0.0]}",
    ]


@pytest.mark.parametrize(
    "values",
    [
        (1e-05, 0.0),
        (-2e-07, 0.0),
        (0.123456789012345, -0.0),
        (1e22, 3.0),
        (5e-324, -5e-324),
        (1e-300, sys.float_info.max),
    ],
)
def test_the_payload_reaches_the_node_as_the_exact_floats(monkeypatch, values):
    # `ros2 topic pub` reads the payload with yaml.safe_load (YAML 1.1: `1e-05`
    # without a dot is a string, and the message cannot be built from it).
    import yaml

    argv = []
    monkeypatch.setattr(bench.subprocess, "Popen", lambda a, **kw: argv.append(a))
    bench.Publisher("/t/plem_drive/command", values)
    data = yaml.safe_load(argv[0][-1])["data"]
    assert all(type(x) is float for x in data)
    assert [x.hex() for x in data] == [v.hex() for v in values]  # -0.0 keeps its sign


def a_bench_of(tmp_path, *modes):
    """A bench of the first len(modes) axes of CHECK, declared `modes`."""
    args = SimpleNamespace(namespace="/t", master=0, model=tmp_path / "m.yaml")
    c = bench.parse_check(with_modes(*modes))
    return bench.Bench(
        args, bench.Check(**{**c.__dict__, "axes": c.axes[: len(modes)]}), None, None
    )


def test_the_stream_carries_the_stream_axes_only_at_rest(tmp_path):
    # csp holds where it is released (the bus latches it); the stream controller
    # takes one value per csv or cst axis, in file order.
    b = a_bench_of(tmp_path, "csp", "csv", "cst")
    assert b.guard.neutral == (0.0, 0.0)  # csv 0 rad/s, cst 0 Nm
    published = []

    def feeding(topic, values):
        published.append((topic, values))
        for _ in range(bench.STREAM_READY):
            b.guard.on_command(SimpleNamespace(data=list(values)))
        return SimpleNamespace(end=lambda: 0)

    b.publisher = feeding
    b.start_stream()
    assert published == [("/t/stream_controller/commands", (0.0, 0.0))]


def test_the_observer_holds_the_stream_to_its_length_and_neutral_vector():
    c = bench.parse_check(with_modes("csp", "csv"))
    g = bench.Guard(c.axes[:3], clock=Clock())
    g.on_command(SimpleNamespace(data=[0.0, 0.0]))
    g.guard()
    g.on_command(SimpleNamespace(data=[0.0, 0.0, 0.0]))
    with pytest.raises(bench.BenchError, match="command length 3"):
        g.guard()
    g = bench.Guard(c.axes[:3], clock=Clock())
    g.on_command(SimpleNamespace(data=[0.1, 0.0]))
    with pytest.raises(bench.BenchError, match="neutral"):
        g.guard()


def test_a_bus_without_a_stream_axis_releases_without_a_stream():
    # csp only: the release latches each axis where it rests — no stream to be fresh.
    c = bench.parse_check(with_modes("csp", "csp"))
    g = bench.Guard(c.axes[:2], clock=Clock())
    g.on_status(status(2))
    g.on_joint_states(joints(["j0", "j1"], [0.2, 0.0]))
    g.release_gate()
    assert g.baseline == (0.2, 0.0)


def test_joint_states_in_another_order_are_read_by_name():
    # The joint state broadcaster does not keep the file's order.
    g, clock = guard_for_two()
    g.on_joint_states(joints(["j1", "j0"], [-0.3, 0.2], v=[0.0, 0.1]))
    g.guard()
    assert tuple(g.joint_states.name) == ("j0", "j1")
    assert tuple(g.joint_states.position) == (0.2, -0.3)
    assert tuple(g.joint_states.velocity) == (0.1, 0.0)
    g.on_joint_states(joints(["j0", "j9"], [0.0, 0.0]))
    with pytest.raises(bench.BenchError, match="joint identity/length"):
        g.guard()


def test_a_bus_releases_through_the_client_after_the_gate(tmp_path):
    # Bus.release (on a csp bus it restarts the trajectory controller: it then
    # holds where the axes are, not where a stop left its command) — after the
    # bench's gate.
    b = a_bench_of(tmp_path, "csp", "cst")
    released = []
    b.drive = SimpleNamespace(release=lambda: released.append(True))
    b.guard.on_status(status(2))
    b.guard.on_joint_states(joints(["j0", "j1"], [1.6, 0.0]))  # j0 over its 1.5 limit
    b.guard.on_command(SimpleNamespace(data=[0.0]))
    with pytest.raises(bench.BenchError, match="outside the declared limits"):
        b.release("release 1")
    assert released == []


def test_an_endless_axis_releases_anywhere():
    c = bench.parse_check(
        check_text(
            lambda c: c["axes"][0].update(
                position_min_rad=None, position_max_rad=None, mode="csp"
            )
        )
    )
    assert (c.axes[0].lo, c.axes[0].hi) == (float("-inf"), float("inf"))
    g = bench.Guard(c.axes[:1], clock=Clock())
    g.on_status(status(1))
    g.on_joint_states(
        joints(["j0"], [5000.0])
    )  # past the ±1608 rad an int32 word folds at
    g.release_gate()


# stale-engage: the verdict on the events after the stream was killed

KILL_NS = 5_000_000_000


def rt_event(kind, ns, value=0.0):
    return SimpleNamespace(event_type=kind, monotonic_ns=ns, value=value)


STALE, ENGAGE = RtEvent.COMMAND_STALE, RtEvent.BRAKE_ENGAGE


# CHECK: 100 ms command_stale_timeout, 1 ms cycle -> age bar 0.11 s, engage bar 2 ms.
@pytest.mark.parametrize(
    "events, refused",
    [
        ([], "0x54 after the stream was killed"),
        # a 0x54 before the kill belongs to another stop
        (
            [rt_event(STALE, KILL_NS - 1, 0.1), rt_event(ENGAGE, KILL_NS)],
            "0x54 after the stream was killed",
        ),
        (
            [rt_event(STALE, KILL_NS, 0.111), rt_event(ENGAGE, KILL_NS)],
            "RT command age 0.111000 s <= 0.11 s",
        ),
        (
            [rt_event(STALE, KILL_NS, -0.001), rt_event(ENGAGE, KILL_NS)],
            "RT command age -0.001000 s",
        ),
        ([rt_event(STALE, KILL_NS, 0.1)], "0x54 -> first 0x42 [] ns <= 2000000 ns"),
        # an engage before the 0x54 is not the one it caused
        (
            [rt_event(ENGAGE, KILL_NS), rt_event(STALE, KILL_NS + 1, 0.1)],
            "0x54 -> first 0x42 [] ns",
        ),
        (
            [rt_event(STALE, KILL_NS, 0.1), rt_event(ENGAGE, KILL_NS + 2_000_001)],
            "0x54 -> first 0x42 [2000001] ns <= 2000000 ns",
        ),
    ],
)
def test_stale_engage_refuses_a_missing_or_late_0x54_or_engage(events, refused):
    with pytest.raises(bench.BenchError) as e:
        bench.judge_stale_engage(events, KILL_NS, bench.parse_check(CHECK))
    assert str(e.value).startswith("stale: " + refused)


def test_stale_engage_passes_at_its_bars_and_names_the_first_engage():
    events = [
        rt_event(ENGAGE, KILL_NS + 3_000_000),
        rt_event(STALE, KILL_NS, 0.11),
        rt_event(ENGAGE, KILL_NS + 2_000_000),
    ]
    assert bench.judge_stale_engage(events, KILL_NS, bench.parse_check(CHECK)) == (
        "stream killed -> 0x54 at age 0.110 s -> EXTERNAL protective stop -> 0x42 in 2000000 ns "
        "(<= 2 cycles) -> every axis settled"
    )


# stop-decel: the one motion of the bench


def axes_of(*modes, **over):
    def edit(c):
        for i, mode in enumerate(modes):
            c["axes"][i].update(
                {
                    "mode": mode,
                    "velocity_cap_rad_s": 1.9,
                    "acceleration_limit_rad_s2": None if mode == "cst" else 4.0,
                    **over,
                }
            )

    return bench.parse_check(check_text(edit)).axes[: len(modes)]


def test_each_moving_axis_goes_to_its_roomier_side_a_short_way_at_cruise():
    # Limits -1.0 .. 1.5, cap 1.9 rad/s, 4 rad/s^2.
    moves = bench.stop_moves(axes_of("csp", "csv", "cst"), (1.4, -0.2, 0.0))
    assert [(m.index, m.direction) for m in moves] == [(0, -1), (1, 1)]  # no cst move
    assert all(m.distance == bench.STOP_DISTANCE_RAD for m in moves)
    # min(0.3 x 1.9, sqrt(4 x 0.5 / 2)) — half the distance is left for cruise.
    assert moves[0].cruise == pytest.approx(0.57)
    near = bench.stop_moves(axes_of("csp", acceleration_limit_rad_s2=0.5), (-0.8,))
    # 0.9 of the 0.2 rad toward -1.0 vs 2.3 rad toward 1.5: up, capped at 0.5 rad.
    assert (near[0].direction, near[0].distance) == (1, 0.5)
    assert near[0].cruise == pytest.approx(math.sqrt(0.5 * 0.5 / 2))
    tight = bench.stop_moves(
        axes_of("csp", position_min_rad=-0.1, position_max_rad=0.2), (0.0,)
    )
    assert tight[0].direction == 1 and tight[0].distance == pytest.approx(0.18)
    endless = bench.stop_moves(
        axes_of("csv", position_min_rad=None, position_max_rad=None), (0.0,)
    )
    assert endless[0].distance == bench.STOP_DISTANCE_RAD


def test_a_stiff_axis_still_cruises_long_enough_to_be_seen_at_cruise():
    # The A6B example's output: cap 19.9 rad/s, 95 rad/s^2 over 0.5 rad. Half the
    # distance at cruise alone gives 4.87 rad/s for 51 ms — shorter than the
    # drive's measured lag (~33 ms) lets joint_states reach 0.9 of it.
    # The cruise is held at least MIN_CRUISE_S.
    a = 95.0
    (move,) = bench.stop_moves(
        axes_of("csp", velocity_cap_rad_s=19.9, acceleration_limit_rad_s2=a), (0.0,)
    )
    hold = (move.distance - move.cruise**2 / a) / move.cruise
    assert hold == pytest.approx(bench.MIN_CRUISE_S)
    assert move.cruise == pytest.approx(3.62, abs=0.01)


def test_an_axis_with_no_room_to_move_refuses_before_any_release():
    with pytest.raises(bench.BenchError, match="no room"):
        bench.stop_moves(
            axes_of("csp", position_min_rad=-0.01, position_max_rad=0.02), (0.0,)
        )


def test_a_csv_ramp_reaches_cruise_at_its_limit_and_travels_its_distance():
    move = bench.Move(index=0, direction=-1, distance=0.5, cruise=1.0)
    v = [bench.ramp_velocity(move, 4.0, k * 1e-3) for k in range(-100, 2000)]
    assert v[0] == 0.0 and min(v) == -1.0 and v[-1] == 0.0
    assert max(abs(b - a) for a, b in zip(v, v[1:])) <= 4.0 * 1e-3 + 1e-12
    assert sum(v) * 1e-3 == pytest.approx(-0.5, abs=2e-3)


def test_the_deceleration_is_the_speed_at_the_stop_over_the_time_to_rest():
    moves = [bench.Move(0, 1, 0.5, 0.6), bench.Move(2, -1, 0.5, 0.6)]
    # (t, velocities in file order): at the request (t = 1.0) 0.6 and -0.6 rad/s.
    samples = [(0.9, (0.6, 0.0, -0.6)), (1.0, (0.6, 0.0, -0.6))]
    samples += [
        (1.0 + k * 0.01, (max(0.0, 0.6 - 4 * k * 0.01), 0.0, -0.6 + 3 * k * 0.01))
        for k in range(1, 30)
    ]
    got = bench.decelerations(samples, 1.0, moves)
    assert got[0] == (0.6, pytest.approx(0.15))  # 4 rad/s^2
    assert got[1][0] == 0.6 and got[1][1] == pytest.approx(0.2)  # 3 rad/s^2
    assert bench.decelerations(samples[:3], 1.0, moves)[0][1] is None  # never at rest


def stopped(*rest_s, at_stop=(0.6, -0.6)):
    """rt_raw samples (t, velocities) for a stop requested at t = 1.0: each axis
    at `at_stop` until its rest_s after the request (None = never at rest)."""
    ts = sorted({1.0 + r for r in rest_s if r is not None})
    return [(1.0, at_stop)] + [
        (
            t,
            tuple(
                0.0 if r is not None and t >= 1.0 + r else v
                for v, r in zip(at_stop, rest_s)
            ),
        )
        for t in ts
    ]


def judge_stop(samples, events=(), lost=0):
    # j0 csp and j1 csv at 4 rad/s^2 -> deceleration bar 1.1 x 4 = 4.4 rad/s^2
    moves = [bench.Move(0, 1, 0.5, 0.6), bench.Move(1, -1, 0.5, 0.6)]
    return bench.judge_stop_decel(
        axes_of("csp", "csv"), moves, {0: 0.6, 1: 0.6}, 1.0, list(events), samples, lost
    )


@pytest.mark.parametrize(
    "samples, events, lost, refused",
    [
        (
            stopped(0.15, 0.2),
            [rt_event(RtEvent.RAMP_STOP_TIMEOUT, 1)],
            0,
            "no stop timeout (0x4F)",
        ),
        (
            stopped(0.15, 0.2)[1:],
            [],
            0,
            "rt_raw delivered the cycle at the stop request",
        ),
        (stopped(0.15, 0.2), [], 1, "rt_raw lost no cycle while recording (1 lost)"),
        (
            stopped(0.15, None),
            [],
            0,
            "j1 came to rest within the recorded rt_raw cycles",
        ),
        (
            stopped(0.15, 0.2, at_stop=(0.539, -0.6)),
            [],
            0,
            "j0 at cruise at the stop (0.5390 of 0.6000 rad/s)",
        ),
        (
            stopped(0.136, 0.2),
            [],
            0,
            "j0 0.600 rad/s -> rest in 0.136 s = 4.412 rad/s^2 <= 4.400",
        ),
    ],
)
def test_stop_decel_refuses_a_timeout_a_gap_in_rt_raw_or_a_stop_off_cruise_or_too_sharp(
    samples, events, lost, refused
):
    with pytest.raises(bench.BenchError) as e:
        judge_stop(samples, events, lost)
    assert str(e.value).startswith("stop-decel: " + refused)


def test_stop_decel_passes_at_nine_tenths_of_cruise_and_just_under_the_deceleration_bar():
    assert judge_stop(stopped(0.137, 0.2, at_stop=(0.6, -0.54))) == (
        "protective_stop at cruise: j0 0.600 rad/s -> rest in 0.137 s (4.380 <= 4.400 rad/s^2); "
        "j1 0.540 rad/s -> rest in 0.200 s (2.700 <= 4.400 rad/s^2); no 0x4F; every axis engaged"
    )


def test_the_stop_settles_within_the_slowest_deceleration_plus_the_command_wait():
    axes = axes_of("csp", "csv")  # 4 rad/s^2 each
    moves = [bench.Move(0, 1, 0.5, 0.6), bench.Move(1, 1, 0.5, 2.0)]
    assert bench.stop_settle_s(moves, axes) == pytest.approx(
        2.0 / 4.0 + bench.COMMAND_WAIT_S
    )


def test_the_stop_is_requested_only_with_every_moving_axis_at_cruise(tmp_path):
    b = a_bench(tmp_path)
    moves = [bench.Move(0, 1, 0.5, 0.6), bench.Move(1, -1, 0.5, 0.6)]
    cruise = {0: 0.6, 1: 0.6}
    b.guard.on_joint_states(joints(["j0", "j1"], [0.0, 0.0], v=[0.6, -0.3]))
    with pytest.raises(bench.BenchError, match="j1 0.3000 of 0.6000"):
        b.await_cruise(moves, cruise, time.monotonic() + 0.05)
    b.guard.on_joint_states(joints(["j0", "j1"], [0.0, 0.0], v=[0.6, -0.58]))
    b.await_cruise(moves, cruise, time.monotonic() + 0.05)
    # A csp move that already failed ends the wait at once, with its error.
    failed = bench.CspMove(
        lambda: (_ for _ in ()).throw(RuntimeError("refused the move")), 0.0
    )
    failed.thread.join()
    b.guard.on_joint_states(joints(["j0", "j1"], [0.0, 0.0], v=[0.0, 0.0]))
    with pytest.raises(bench.BenchError, match="refused the move"):
        b.await_cruise(moves, cruise, time.monotonic() + 5.0, failed)


def test_the_stop_is_asked_mid_cruise_and_the_wait_for_cruise_ends_inside_it():
    # (time to cruise, time at cruise): every cruise starts at the latest rise
    # (1.0 s); the shortest lasts 2.0 s. The request is due at its middle, and a
    # late axis may delay it to three quarters of it, never past its end.
    assert bench.stop_times([(1.0, 2.0), (0.5, 4.0)]) == (2.0, 2.5)


def test_a_csp_move_failing_before_the_stop_fails_the_item_never_swallowed():
    def refused():
        raise RuntimeError("the trajectory controller refused the move")

    move = bench.CspMove(refused, 0.0)
    with pytest.raises(bench.BenchError, match="refused the move"):
        move.end(time.monotonic() + 1.0)
    t_request = time.monotonic()

    def cancelled():  # the protective stop ends the move: expected, logged
        time.sleep(0.05)
        raise RuntimeError("the move was cancelled (stop)")

    assert "cancelled (stop)" in bench.CspMove(cancelled, 0.0).end(t_request)


def test_a_failed_check_is_told_apart_from_a_refusal():
    # main() prints a CheckFailed as "ABORT: check failed, expected: <condition>",
    # any other BenchError as the operator message it already is.
    with pytest.raises(bench.CheckFailed, match="^stale: x$"):
        bench.require(False, "stale: x")
    assert issubclass(bench.CheckFailed, bench.BenchError)
    bench.require(True, "never raised")


def test_the_bench_measures_every_required_item_including_the_stop():
    assert "stop-decel" in bench.MEASURED


@pytest.mark.parametrize(
    ("modes", "says"),
    [
        (("cst", "cst"), "no csp or csv axis"),
        (("csp", "cst"), "immediate disable"),
        (("csv", "cst"), "immediate disable"),
    ],
)
def test_stop_decel_is_na_for_the_reason_the_bus_has(modes, says):
    # check leaves stop-decel out on any bus with a cst axis: that bus's stop
    # is an immediate disable, not the ramp stop-decel measures.
    assert says in bench.stop_decel_na(modes)


def test_the_confirmation_follows_checks_required_items():
    # What the bench measures is check's required list (the C++ decides it);
    # the axis modes only say how.
    c = bench.parse_check(CHECK)  # stale-engage required, stop-decel not
    text = bench.confirm_text("m.yaml", c)
    assert "MOVE" not in text and " 3 times" in text

    def requiring(*ids):
        return bench.Check(**{**c.__dict__, "required": ids})

    text = bench.confirm_text("m.yaml", requiring("stale-engage", "stop-decel"))
    assert "MOVE" in text and "either direction" in text and " 4 times" in text
    assert f"{bench.STOP_DISTANCE_RAD:g} rad" in text and "clear" in text
    assert " 3 times" in bench.confirm_text("m.yaml", requiring("stop-decel"))


def test_the_run_chain_is_started_unapproved_on_the_benchs_master(tmp_path):
    # Its namespace is the bench's own, handed to run through its environment.
    args = SimpleNamespace(
        namespace="/b", master=2, model=tmp_path / "m.yaml", rt_cpu=None
    )
    argv = bench.run_argv(args, "/p/bin/plem-drive")
    assert argv == [
        "env",
        "PLEM_DRIVE_BENCH_NAMESPACE=/b",
        "/p/bin/plem-drive",
        "run",
        str(tmp_path / "m.yaml"),
        "--master",
        "2",
        "--unapproved",
        "--verbose",  # the bench log keeps every node's lines
    ]
    # run's --rt-cpu, forwarded: the bench's bus runs on the CPU the operator chose.
    args.rt_cpu = 3
    assert bench.run_argv(args, "p")[-2:] == ["--rt-cpu", "3"]


# rated_per_mille: 0x6076 against the file, a warning only


def per_mille(mnm):
    """CHECK with j0 (channel 2 of ring 5) a rated_per_mille axis of `mnm` mNm."""
    return check_text(lambda c: c["axes"][0].update(rated_torque_mnm=mnm))


def uint32(value):
    return f"0x{value:08x} {value}\n".encode()  # `ethercat upload -t uint32`


def test_a_per_mille_axis_carries_its_rated_torque_and_the_rest_none():
    c = bench.parse_check(per_mille(1270.4))
    assert [a.rated_torque_mnm for a in c.axes] == [1270.4, None, None, None, None]


@pytest.mark.parametrize(
    "file_mnm, raw, says",
    [
        (1270, uint32(1270), "j0: 0x6876 motor rated torque 1270 mNm matches"),
        # within 1 %
        (1270, uint32(1282), "j0: 0x6876 motor rated torque 1282 mNm matches"),
        # a small motor: 1 mNm floor
        (1.5, uint32(2), "j0: 0x6876 motor rated torque 2 mNm matches"),
        (1270, uint32(1283), "WARN j0: 0x6876 motor rated torque reads 1283 mNm"),
        (1270, None, "j0: the drive has no 0x6876 motor rated torque — not compared"),
        (
            1270,
            uint32(0xFFFFFFFF),
            "j0: 0x6876 motor rated torque reads 0xFFFFFFFF (unset) — not compared",
        ),
        (
            1270,
            uint32(0),
            "j0: 0x6876 motor rated torque reads 0x00000000 (unset) — not compared",
        ),
    ],
)
def test_the_rated_torque_note_warns_on_a_mismatch_only(file_mnm, raw, says):
    axis = bench.parse_check(per_mille(file_mnm)).axes[0]
    assert bench.rated_torque_note(axis, raw).startswith(says)


def test_a_1000x_error_is_named_in_its_direction():
    # The file 1000× too small (Nm written as kNm): a sub-mNm value, never a division by 0.
    small = bench.rated_torque_note(
        bench.parse_check(per_mille(0.16)).axes[0], uint32(160)
    )
    assert small.startswith("WARN j0: 0x6876 motor rated torque reads 160 mNm, 1e+03×")
    assert (
        "1e+03× stronger than asked" in small and "int16 limit, −32768…32767" in small
    )
    assert "1e+03× lower" in small
    # The file 1000× too large (0x6076's mNm written as Nm).
    large = bench.rated_torque_note(
        bench.parse_check(per_mille(1270000)).axes[0], uint32(1270)
    )
    assert (
        "0.001×" in large
        and "1e+03× weaker than asked" in large
        and "1e+03× higher" in large
    )
    for text in (small, large):
        assert (
            "Either the file is wrong" in text and "the drive's motor setting" in text
        )


def a_per_mille_bench(tmp_path, upload):
    args = SimpleNamespace(namespace="/t", master=0, model=tmp_path / "m.yaml")
    b = bench.Bench(
        args, bench.parse_check(per_mille(1270)), SimpleNamespace(upload=upload), None
    )
    lines = []
    b.log = lines.append
    return b, lines


def test_the_bench_reads_0x6076_once_per_per_mille_axis_and_keeps_the_warning(tmp_path):
    reads = []
    b, lines = a_per_mille_bench(tmp_path, lambda *a: reads.append(a) or uint32(127))
    b.rated_torques()
    assert reads == [(5, 0x6876, "uint32")]
    assert len(lines) == 1 and lines[0].startswith("WARN j0:")
    assert b.notes == lines


def test_a_failed_0x6076_read_is_not_compared_and_does_not_abort(tmp_path):
    def upload(*a):
        raise bench.BenchError("SDO upload 0x6876 failed: SDO protocol timeout")

    b, lines = a_per_mille_bench(tmp_path, upload)
    b.rated_torques()
    assert lines == [
        "j0: 0x6876 motor rated torque not compared (SDO upload 0x6876 failed: SDO protocol timeout)"
    ]
    assert b.notes == []


def test_an_approved_run_keeps_the_warning_in_the_bench_notes(
    tmp_path, monkeypatch, capsys
):
    model = tmp_path / "m.yaml"
    model.write_text("x")
    monkeypatch.setattr(
        bench, "drive_model_tool", lambda: str(tmp_path / "drive_model_tool")
    )
    monkeypatch.setattr(
        bench,
        "preflight",
        lambda *a: (SimpleNamespace(version=lambda: "1.6"), bench.parse_check(CHECK)),
    )

    class Warned:
        items = {}
        notes = ["WARN j0: 0x6876 motor rated torque reads 127 mNm"]

        def __init__(self, *a):
            pass

        def run(self):
            return []

        def cleanup(self):
            return True

    conditions = []
    monkeypatch.setattr(bench, "Bench", Warned)
    monkeypatch.setattr(
        bench, "bench_document", lambda *a: conditions.append(a[3]) or {}
    )
    handed = []

    def approve(binary, verb, model_arg, evidence):
        handed.append((Path(evidence), Path(evidence).read_text()))
        return SimpleNamespace(returncode=0, stdout="approved", stderr="")

    monkeypatch.setattr(bench, "run_tool", approve)
    assert keep_handlers(lambda: bench.main([str(model)])) == 0
    assert conditions[0]["notes"] == "WARN j0: 0x6876 motor rated torque reads 127 mNm"
    # approve got the evidence in a private directory, gone afterwards: the model file is the only output.
    ((evidence, text),) = handed
    assert evidence.parent != tmp_path and not evidence.parent.exists()
    assert text == "{}\n"
    assert sorted(p.name for p in tmp_path.iterdir()) == ["m.yaml"]
    assert f"approval recorded in {model.resolve()}" in capsys.readouterr().out


# drive objects read before the node starts (CiA 402)


def axis_of(mode="csp", encoder_resolution=65536, flip=False, channel=1, startup=()):
    def edit(c):
        c["axes"][0].update(
            mode=mode,
            encoder_resolution=encoder_resolution,
            flip=flip,
            channel=channel,
            od_offset=0x800 * (channel - 1),
        )
        c["slaves"][0]["startup_sdo"] = [list(w) for w in startup]

    return bench.parse_check(check_text(edit)).axes[0]


def reader(objects, failing=()):
    """A fake SDO read: {(index, subindex): value}; absent = None; `failing`
    addresses raise like an SDO abort that is not an absence."""
    seen = []

    def read(index, subindex, typ):
        seen.append((index, subindex, typ))
        if (index, subindex) in failing:
            raise bench.BenchError("SDO upload failed: Data type mismatch")
        return objects.get((index, subindex))

    read.seen = seen
    return read


def test_a_drive_without_the_files_mode_fails_before_any_brake_is_released():
    failure, _, _ = bench.drive_object_report(
        axis_of("csp"), reader({(0x6502, 0): 1 << 8 | 1 << 9})
    )
    assert failure == (
        "j0: 0x6502 supported drive modes 0x00000300 lacks csp (bit 7) — the drive cannot run the file's mode; "
        "choose a mode the drive supports"
    )
    for mode, bit in (("csp", 7), ("csv", 8), ("cst", 9)):
        failure, _, _ = bench.drive_object_report(
            axis_of(mode), reader({(0x6502, 0): 1 << bit})
        )
        assert failure == "", mode
    failure, _, info = bench.drive_object_report(axis_of("csp"), reader({}))
    assert failure == "" and "j0: no 0x6502 — mode support not compared" in info


def test_a_failed_read_is_not_compared_never_an_abort():
    failure, warns, info = bench.drive_object_report(
        axis_of("csp"), reader({}, failing={(0x6502, 0), (0x6092, 1), (0x6085, 0)})
    )
    assert failure == "" and warns == []
    assert (
        "j0: 0x6502:0 not read (SDO upload failed: Data type mismatch) — not compared"
        in info
    )
    assert any(line.startswith("j0: 0x6085:0 not read") for line in info)


def test_the_object_address_follows_the_channel():
    read = reader({})
    bench.drive_object_report(axis_of("csp", channel=2), read)
    assert (0x6D02, 0, "uint32") in read.seen and (0x6884, 0, "uint32") in read.seen


def test_a_drive_side_polarity_conflicts_only_with_flip():
    _, warns, _ = bench.drive_object_report(
        axis_of("csp", flip=True), reader({(0x607E, 0): 0xC0})
    )
    assert warns == [
        "WARN j0: 0x607E polarity 0xC0 — the drive inverts on its own and the file's flip (true) inverts "
        "again; set the direction in one place (0x607E = 0 with flip: true, or keep 0x607E and set flip: false)"
    ]
    _, warns, info = bench.drive_object_report(
        axis_of("csp"), reader({(0x607E, 0): 0xC0})
    )
    assert warns == [] and any("declare it in startup_sdo" in line for line in info)
    # The file's startup write is what the drive runs with, not what it holds now.
    _, warns, info = bench.drive_object_report(
        axis_of("csp", flip=True, startup=[(0x607E, 0, 0)]), reader({(0x607E, 0): 0xC0})
    )
    assert warns == []
    _, warns, _ = bench.drive_object_report(
        axis_of("csp", flip=True, startup=[(0x607E, 0, 0x80)]), reader({})
    )
    assert len(warns) == 1


def test_the_drives_position_scaling_is_compared_with_encoder_resolution():
    # CiA 402: counts per motor turn in 0x6064 = feed constant / gear ratio.
    scaled = {(0x6091, 1): 1, (0x6091, 2): 1, (0x6092, 1): 8388608, (0x6092, 2): 1}
    _, warns, _ = bench.drive_object_report(axis_of("cst"), reader(scaled))
    assert warns == [
        "WARN j0: the drive's scaling objects give 8388608 counts per motor turn in 0x6064 (0x6092 feed "
        "8388608/1 over 0x6091 gear 1/1); the file's factors.encoder_resolution is 65536 — positions would be "
        "off by 128×. Set encoder_resolution to 8388608, or the drive's scaling to 65536 (a vendor-specific "
        "electronic gear is not read)"
    ]
    _, warns, _ = bench.drive_object_report(
        axis_of("cst", encoder_resolution=8388608), reader(scaled)
    )
    assert warns == []
    # Without a feed constant the encoder increments decide — only under a 1:1 gear.
    encoder = {(0x608F, 1): 131072, (0x608F, 2): 1}
    assert len(bench.drive_object_report(axis_of("cst"), reader(encoder))[1]) == 1
    geared = {**encoder, (0x6091, 1): 3, (0x6091, 2): 1}
    _, warns, info = bench.drive_object_report(axis_of("cst"), reader(geared))
    assert warns == [] and any("scaling not compared" in line for line in info)
    # Defaults (1/1) and non-integer results are not compared: drives use these objects loosely.
    for objects in (
        {(0x6092, 1): 1, (0x6092, 2): 1},
        {(0x6092, 1): 836837551, (0x6092, 2): 262217903},
        {},
    ):
        _, warns, info = bench.drive_object_report(axis_of("cst"), reader(objects))
        assert warns == [] and any("scaling not compared" in line for line in info), (
            objects
        )
    # A startup write overrides what the drive holds before the node.
    _, warns, _ = bench.drive_object_report(
        axis_of("cst", startup=[(0x6092, 1, 65536)]), reader(scaled)
    )
    assert warns == []


def test_undeclared_stop_values_are_printed_ready_to_paste():
    holds = {
        (0x605A, 0): 2,
        (0x605C, 0): 0xFFFF,
        (0x6084, 0): 1000,
        (0x6085, 0): 5000,
        (0x6065, 0): 3000,
        (0x6066, 0): 40000,
    }
    read = reader(holds)
    _, _, info = bench.drive_object_report(
        axis_of("csp", startup=[(0x6084, 0, 7)]), read
    )
    paste = [line for line in info if line.startswith("      - {")]
    assert (
        paste
        == [
            "      - { index: 0x605a, subindex: 0, size: 2, value: 2 }",
            "      - { index: 0x605c, subindex: 0, size: 2, value: 65535 }",
            "      - { index: 0x6085, subindex: 0, size: 4, value: 5000 }",
        ]
    )  # the following-error window (0x6065/0x6066) is the drive's own business: not offered
    assert (
        0x605C,
        0,
        "uint16",
    ) in read.seen  # the unsigned wire pattern startup_sdo takes
    _, _, info = bench.drive_object_report(axis_of("cst"), reader(holds))
    assert not any("0x6084" in line for line in info)  # csp/csv only


def test_an_error_code_names_its_cia_class():
    assert bench.error_code_text(0x8611) == "0x8611 following error"
    assert bench.error_code_text(0x2310) == "0x2310 current — see the drive manual"
    assert (
        bench.error_code_text(0xFF01) == "0xFF01 device specific — see the drive manual"
    )
    assert bench.error_code_text(0x0000) == "0x0000 no error"


def test_statusword_bit_11_seen_on_the_bench_is_noted_bit_13_is_not():
    # Bit 13 (following error) is not noted: PLEM does not use it, and a drive
    # may set it through normal motion (an A6B over its 0x6065 window).
    axes = (axis_of("csp"), bench.parse_check(CHECK).axes[1])  # j0 csp, j1 cst
    history = [
        (1, (0, False, 3, 0, (0x2A38, 0x0238))),  # startup, before monitoring: ignored
        (5, (0, True, 0, 0, (0x0237, 0x0237))),
        (6, (0, True, 0, 0, (0x2A37, 0x2237))),  # j0 bits 11+13, j1 bit 13
    ]
    assert bench.status_bit_notes(history, axes, since_ns=5) == [
        "WARN j0: statusword bit 11 (internal limit active) seen during the bench",
    ]
    assert bench.status_bit_notes(history[:2], axes, since_ns=5) == []


def test_an_encoder_that_moved_a_turn_is_warned():
    a = axis_of("cst")
    assert bench.position_continuity_note(a, 1000, 1010) == (
        "j0: 0x6064 moved 10 counts (0.000153 motor turns) between the reads before the node and after it"
    )
    assert bench.position_continuity_note(a, 1000, 1000 + 70000).startswith(
        "WARN j0: 0x6064 moved 70000 counts (1.07 motor turns)"
    )
    assert "not compared" in bench.position_continuity_note(a, None, 5)
    # An axis stop-decel moved: up to 0.5 rad at the output can be many motor
    # turns through a gearhead, and check's JSON does not carry the gear ratio.
    assert bench.position_continuity_note(a, 1000, 1000 + 70000, moved=True) == (
        "j0: 0x6064 moved 70000 counts (1.07 motor turns) between the reads before the node and after it"
        " — stop-decel moved it, continuity not judged"
    )
    rescaled = axis_of("cst", startup=[(0x607E, 0, 0x80)])
    assert "startup_sdo rescales 0x6064" in bench.position_continuity_note(
        rescaled, 0, 10**6
    )


class FakeBus:
    def __init__(self, objects):
        self.objects, self.calls = objects, []

    def upload(self, position, index, typ, subindex=0):
        self.calls.append((position, index, subindex, typ))
        v = self.objects.get((index, subindex))
        return None if v is None else f"0x{v:x} {v}\n".encode()


def test_drive_objects_names_every_axis_the_drive_cannot_run_and_keeps_warnings():
    c = bench.parse_check(
        check_text(lambda c: c["axes"][0].update(mode="csp", flip=True))
    )
    b = bench.Bench(
        SimpleNamespace(namespace="/b"),
        c,
        FakeBus({(0x6D02, 0): 0, (0x6502, 0): 0, (0x687E, 0): 0x80}),
        [],
    )
    b.log = lambda msg: None
    with pytest.raises(bench.BenchError) as e:
        b.drive_objects()
    lines = str(e.value).splitlines()
    assert len(lines) == 4 and lines[0].startswith("j0: 0x6D02 supported drive modes")
    assert b.notes and b.notes[0].startswith("WARN j0: 0x687E polarity 0x80")


def test_the_upload_carries_the_subindex(monkeypatch):
    calls = []
    monkeypatch.setattr(bench.shutil, "which", lambda name: "/bin/ethercat")
    bus = bench.EthercatCli(0)
    monkeypatch.setattr(
        bus,
        "run",
        lambda *a, binary=False: (
            calls.append(a) or SimpleNamespace(returncode=0, stdout=b"0x1 1\n")
        ),
    )
    assert bus.upload(3, 0x6091, "uint32", 2) == b"0x1 1\n"
    assert calls == [("upload", "-p", "3", "-t", "uint32", "0x6091", "2")]


def upload_answering(monkeypatch, answers):
    """An EthercatCli whose uploads answer `answers` in turn; returns it and the sleeps."""
    sleeps = []
    monkeypatch.setattr(bench.time, "sleep", sleeps.append)
    b = fake_bus(None)
    replies = iter(answers)
    b.run = lambda *a, binary=False: b.calls.append(a) or next(replies)
    return b, sleeps


NO_ABORT = SimpleNamespace(
    returncode=1, stdout=b"", stderr=b"Failed to upload SDO: Input/output error\n"
)


def test_an_upload_failing_without_an_abort_is_read_again(monkeypatch):
    ok = SimpleNamespace(returncode=0, stdout=b"0x1 1\n", stderr=b"")
    b, sleeps = upload_answering(monkeypatch, [NO_ABORT, NO_ABORT, ok])
    assert b.upload(0, 0x6041, "uint16") == b"0x1 1\n"
    assert len(b.calls) == 3 and sleeps == [0.1, 0.3]


def test_only_a_no_abort_failure_is_read_again_and_at_most_three_more_times(
    monkeypatch,
):
    b, sleeps = upload_answering(monkeypatch, [NO_ABORT] * 4)
    with pytest.raises(
        bench.BenchError,
        match=r"Input/output error \(attempts: 4\) .*start again once the bus has been idle",
    ):
        b.upload(0, 0x6041, "uint16")
    assert sleeps == [0.1, 0.3, 0.5]
    denied = SimpleNamespace(
        returncode=1,
        stdout=b"",
        stderr=b"Failed to open master device: Permission denied\n",
    )
    b, sleeps = upload_answering(monkeypatch, [denied])
    with pytest.raises(bench.BenchError, match="check permissions and the bus"):
        b.upload(0, 0x6041, "uint16")
    assert sleeps == [] and len(b.calls) == 1


def test_a_passing_check_says_what_it_compared():
    a6b_like = {
        (0x6502, 0): 0x3AD,
        (0x607E, 0): 0,
        (0x6092, 1): 65536,
        (0x6092, 2): 1,
        (0x6091, 1): 1,
        (0x6091, 2): 1,
    }
    failure, warns, info = bench.drive_object_report(axis_of("csp"), reader(a6b_like))
    assert failure == "" and warns == []
    assert info[:3] == [
        "j0: 0x6502 supported drive modes 0x000003AD include csp",
        "j0: 0x607E polarity 0 — the drive does not invert",
        "j0: scaling 0x6092 feed 65536/1 over 0x6091 gear 1/1 = 65536 counts per motor turn, as "
        "encoder_resolution says",
    ]
    _, _, info = bench.drive_object_report(
        axis_of("csp"), reader({}, failing={(0x6502, 0)})
    )
    assert not any(
        "no 0x6502" in line for line in info
    )  # the failed read already says it


def rt_raw(seqs, velocities, n=2):
    return SimpleNamespace(
        axis_count=n,
        batch_size=len(seqs),
        sequence=list(seqs),
        monotonic_ns=[int(1e9 + 1e6 * q) for q in seqs],
        actual_velocity=[v for row in velocities for v in row],
    )


def test_stop_decel_samples_every_rt_cycle_from_rt_raw_and_counts_losses():
    g, clock = guard_for_two()
    g.on_rt_raw(rt_raw([0, 1], [(1.0, 0.0), (1.0, 0.0)]))  # not recording
    g.samples = []
    g.on_joint_states(joints(["j0", "j1"], [0.0, 0.0], v=[9.0, 9.0]))  # not a sample
    g.on_rt_raw(rt_raw([5, 6], [(0.5, 0.0), (0.4, 0.1)]))  # 2..4 before recording
    g.on_rt_raw(rt_raw([9], [(0.0, 0.0)]))  # 7 and 8 lost
    assert [t for t, _ in g.samples] == pytest.approx([1.005, 1.006, 1.009])
    assert [v for _, v in g.samples] == [
        (0.5, 0.0),
        (pytest.approx(0.4), pytest.approx(0.1)),
        (0.0, 0.0),
    ]
    assert g.rt_lost == 2
