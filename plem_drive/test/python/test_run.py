# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem drive run — what it generates from `plem drive check` (the robot
description and the controller manager's parameters), which lanes it starts,
the host checks it warns about, and when it refuses before starting anything.
ROS-free: the check JSON is written here, or printed by the real
`plem drive check` of a fixture file — never read from a drive."""

import json
import os
import pathlib
import re
import sys
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from plem.drive.cli import run  # noqa: E402

REAL_PLOTJUGGLER = run._plotjuggler
REAL_EXECUTABLE = run._executable

PACKAGE = pathlib.Path(__file__).resolve().parents[2]
EXAMPLES = PACKAGE / "examples"
# The a6b_pair example's own layout: its sync tab is for a mirrored pair only.
PAIR_LAYOUT = EXAMPLES / "a6b_pair" / "plotjuggler.xml"


def axis(name, mode, **over):
    a = {
        "name": name,
        "mode": mode,
        "position_min_rad": -1.0,
        "position_max_rad": 1.5,
        "velocity_limit_rad_s": 2.0,
        "velocity_cap_rad_s": 1.9,
        "acceleration_limit_rad_s2": None if mode == "cst" else 4.0,
        "torque_min_nm": -3.0 if mode == "cst" else None,
        "torque_max_nm": 4.0 if mode == "cst" else None,
    }
    a.update(over)
    return a


def check(*axes, cycle_ns=1_000_000, approval="approved", bus="pair"):
    return {
        "format": "plem-drive-check/1",
        "bus_name": bus,
        "approval_effective": approval,
        "cycle_time_ns": cycle_ns,
        "fault_reset_policy": "none",
        "axes": list(axes),
    }


def joints(urdf):
    return {
        j.get("name"): j for j in ET.fromstring(urdf).iter("joint") if j.get("type")
    }


def hw_joints(urdf):
    return {
        j.get("name"): j for j in ET.fromstring(urdf).find("ros2_control").iter("joint")
    }


# robot description


def test_each_axis_is_a_joint_with_the_files_limits_planned_under_the_cap():
    urdf = run.robot_description(
        check(axis("a0", "csp"), axis("a1", "csp")), "/m/a.yaml", master=1
    )
    j = joints(urdf)
    assert list(j) == ["a0", "a1"]
    limit = j["a0"].find("limit")
    assert j["a0"].get("type") == "revolute"
    assert float(limit.get("lower")) == -1.0 and float(limit.get("upper")) == 1.5
    # A planner keeps under the cap (the limit less the count rounding of a cycle's step).
    assert float(limit.get("velocity")) == 1.9
    hw = ET.fromstring(urdf).find("ros2_control")
    params = {p.get("name"): p.text for p in hw.find("hardware").iter("param")}
    assert hw.find("hardware/plugin").text == "plem_drive/DriveSystemInterface"
    assert params == {
        "model_file": "/m/a.yaml",
        "master_index": "1",
        "fault_reset_policy": "none",
    }
    unapproved = run.robot_description(
        check(axis("a0", "csp")), "/m/a.yaml", unapproved=True
    )
    hardware = ET.fromstring(unapproved).find("ros2_control/hardware")
    assert {p.get("name"): p.text for p in hardware.iter("param")}[
        "unapproved"
    ] == "true"


def test_an_axis_without_end_stops_is_continuous():
    urdf = run.robot_description(
        check(axis("a0", "csp", position_min_rad=None, position_max_rad=None)),
        "/m/a.yaml",
    )
    j = joints(urdf)["a0"]
    assert j.get("type") == "continuous"
    assert j.find("limit").get("lower") is None


def test_each_mode_commands_its_own_interface_and_reads_all_three():
    urdf = run.robot_description(
        check(axis("p", "csp"), axis("v", "csv"), axis("t", "cst")), "/m/a.yaml"
    )
    hw = hw_joints(urdf)
    commands = {
        n: [c.get("name") for c in j.iter("command_interface")] for n, j in hw.items()
    }
    # csp also takes the planned velocity: the loop advances a late sample by it.
    assert commands == {
        "p": ["position", "velocity"],
        "v": ["velocity"],
        "t": ["effort"],
    }
    for j in hw.values():
        assert [s.get("name") for s in j.iter("state_interface")] == [
            "position",
            "velocity",
            "effort",
        ]


def test_an_axis_carries_its_stop_deceleration_or_its_torque_clamp():
    urdf = run.robot_description(check(axis("p", "csp"), axis("t", "cst")), "/m/a.yaml")
    hw = hw_joints(urdf)
    assert {p.get("name"): p.text for p in hw["p"].iter("param")} == {
        "acceleration_rad_s2": "4.0"
    }
    # A stream client checks a cst axis's commands against the file's clamp.
    assert {p.get("name"): p.text for p in hw["t"].iter("param")} == {
        "torque_min_nm": "-3.0",
        "torque_max_nm": "4.0",
    }


def test_the_real_checks_output_carries_a_cst_axis_clamp_into_the_description(
    monkeypatch,
):
    # The real plem drive check of this build's drive_model_tool (CMake sets
    # both paths): the keys run reads are the ones check prints.
    tool = os.environ["PLEM_DRIVE_BIN"]
    monkeypatch.setattr(run, "drive_model_tool", lambda: tool)
    model = os.environ["PLEM_DRIVE_EXAMPLE_YAML"]
    printed = run._check(model)
    urdf = run.robot_description(printed, model)
    hw = hw_joints(urdf)
    assert set(hw) == {a["name"] for a in printed["axes"]}
    assert {p.get("name"): p.text for p in hw["j0"].iter("param")} == {
        "torque_min_nm": "-30.0",
        "torque_max_nm": "40.0",
    }
    stream = run.controller_params(printed, cpus=range(2), isolated=set())[
        run.STREAM_CONTROLLER
    ]["ros__parameters"]
    assert stream["joints"] == [a["name"] for a in printed["axes"]]
    assert set(stream["interface_names"]) == {"effort"}


# controller manager parameters


def test_joint_states_is_published_at_100_hz_not_every_cycle():
    # Every cycle is on rt_raw; a 1 kHz joint_states starved Python clients (Bus.stream
    # loops) of the GIL on a loaded host.
    for cycle_ns, hz in ((1_000_000, 100), (250_000, 100), (20_000_000, 50)):
        params = run.controller_params(
            check(axis("p0", "csp"), cycle_ns=cycle_ns), cpus=range(1), isolated=set()
        )
        jsb = params["joint_state_broadcaster"]["ros__parameters"]
        assert jsb["update_rate"] == hz


def test_position_axes_get_the_trajectory_controller_the_rest_the_stream():
    params = run.controller_params(
        check(axis("p0", "csp"), axis("v", "csv"), axis("p1", "csp"), axis("t", "cst")),
        cpus=range(4),
        isolated={3},
    )
    cm = params["controller_manager"]["ros__parameters"]
    assert cm["update_rate"] == 1000
    # The manager's update thread stays off the isolated CPU the bus loop owns.
    assert cm["cpu_affinity"] == [0, 1, 2]
    assert (
        cm["joint_state_broadcaster"]["type"]
        == "joint_state_broadcaster/JointStateBroadcaster"
    )
    jtc = params[run.TRAJECTORY_CONTROLLER]["ros__parameters"]
    assert jtc["joints"] == ["p0", "p1"]
    assert jtc["command_interfaces"] == ["position", "velocity"]
    assert jtc["open_loop_control"] is True
    assert jtc["allow_nonzero_velocity_at_trajectory_end"] is False
    # Re-activated after a stop it holds where the axes are, not its last command.
    assert jtc["set_last_command_interface_value_as_state_on_activation"] is False
    stream = params[run.STREAM_CONTROLLER]["ros__parameters"]
    assert stream["joints"] == ["v", "t"]
    assert stream["interface_names"] == ["velocity", "effort"]
    assert run.lanes(check(axis("p0", "csp"), axis("v", "csv"), axis("t", "cst"))) == [
        "joint_state_broadcaster",
        run.TRAJECTORY_CONTROLLER,
        run.STREAM_CONTROLLER,
    ]


def test_a_bus_of_one_kind_starts_only_its_lane():
    assert run.lanes(check(axis("p", "csp"))) == [
        "joint_state_broadcaster",
        run.TRAJECTORY_CONTROLLER,
    ]
    params = run.controller_params(
        check(axis("t", "cst")), cpus=range(2), isolated=set()
    )
    assert run.TRAJECTORY_CONTROLLER not in params
    assert "cpu_affinity" not in params["controller_manager"]["ros__parameters"], (
        "nothing to keep off"
    )


def test_the_update_rate_is_the_bus_cycle():
    params = run.controller_params(
        check(axis("p", "csp"), cycle_ns=500_000), cpus=range(2), isolated=set()
    )
    assert params["controller_manager"]["ros__parameters"]["update_rate"] == 2000


def test_the_parameters_file_names_each_node_in_the_namespace():
    params = run.controller_params(
        check(axis("p", "csp")), cpus=range(2), isolated=set()
    )
    assert set(run.namespaced(params, "/bus")) == {
        "/bus/controller_manager",
        "/bus/joint_state_broadcaster",
        f"/bus/{run.TRAJECTORY_CONTROLLER}",
    }


# host checks (warnings only)


def test_a_host_without_rt_isolation_or_with_the_tsc_lock_is_warned_with_the_fix():
    warnings = run.host_warnings(realtime=False, isolated=set(), tsc_lock=True)
    assert len(warnings) == 3
    assert "PREEMPT_RT" in warnings[0]
    assert "menu 1" in warnings[0] and "setup/rt/setup.sh" in warnings[0]
    assert "isolcpus" in warnings[1] and "menu 1" in warnings[1]
    assert "sudo plem-nvpps-tsc-unlock" in warnings[2]
    assert run.host_warnings(realtime=True, isolated={3}, tsc_lock=False) == []
    assert run.host_warnings(realtime=True, isolated={3}, tsc_lock=None) == [], (
        "no nvpps node: nothing to judge"
    )


def test_the_tsc_lock_is_judged_only_on_an_enabled_nvpps_node(tmp_path):
    node = tmp_path / "nvpps"
    assert run._tsc_lock(node) is None, "no node"
    node.mkdir()
    assert run._tsc_lock(node) is True, (
        "the lock is on unless the property says otherwise"
    )
    (node / "ptp_tsc_sync_dis").write_bytes(b"")
    assert run._tsc_lock(node) is False
    (node / "status").write_bytes(b"disabled\0")
    assert run._tsc_lock(node) is None


# refusals before anything starts


def test_an_unapproved_file_is_refused_naming_the_bench_offer_unless_unapproved_is_asked():
    error = run.refusal(
        check(axis("p", "csp"), approval="unapproved"), "/m/a.yaml", unapproved=False
    )
    assert "`plem drive run /m/a.yaml` on a terminal offers the bench" in error
    assert "plem drive bench" not in error
    assert (
        run.refusal(
            check(axis("p", "csp"), approval="unapproved"), "/m/a.yaml", unapproved=True
        )
        is None
    )
    assert run.refusal(check(axis("p", "csp")), "/m/a.yaml", unapproved=False) is None


class Started(Exception):
    pass


@pytest.fixture
def chain(monkeypatch, tmp_path):
    """run.main up to the start: check answers from `checks` (one per call),
    nothing is spawned or exec'd — `seen` records what would be."""
    seen = SimpleNamespace(
        spawned=[], outputs={}, execd=None, benched=[], asked=[], checks=[]
    )
    monkeypatch.setattr(run, "_check", lambda f: seen.checks.pop(0))
    monkeypatch.setattr(
        run,
        "_this_host",
        lambda: {"realtime": True, "isolated": set(), "tsc_lock": None},
    )
    monkeypatch.setattr(run, "_executable", lambda pkg, name: f"/x/{name}")

    def spawn(argv, output=None):
        seen.spawned.append(argv)
        seen.outputs[argv[0]] = output

    monkeypatch.setattr(run, "_spawn_child", spawn)
    monkeypatch.delenv("PLEM_LOG_LEVEL", raising=False)
    monkeypatch.setattr(run, "run_pids", lambda ns: [])
    monkeypatch.setattr(run, "_plotjuggler", lambda: "/x/plotjuggler")

    def execv(path, argv):
        seen.execd = argv
        raise Started

    monkeypatch.setattr(run.os, "execv", execv)
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))

    def bench(argv):
        seen.benched.append(argv)
        return seen.bench_rc

    monkeypatch.setattr(run, "_bench", bench)
    # The drives on the bus match the approval record unless a test says otherwise.
    seen.probe = (0, "")
    seen.probed = []

    def probe(model_file, master):
        seen.probed.append((model_file, master))
        return seen.probe

    monkeypatch.setattr(run, "_probe", probe)
    seen.tty = True
    seen.answer = "y"
    monkeypatch.setattr(run.sys.stdin, "isatty", lambda: seen.tty, raising=False)

    def ask(prompt):
        seen.asked.append(prompt)
        return seen.answer

    monkeypatch.setattr("builtins.input", ask)
    return seen


def test_on_a_terminal_an_unapproved_file_is_benched_first_then_started(chain, capsys):
    chain.checks = [
        check(axis("p", "csp"), approval="unapproved"),
        check(axis("p", "csp")),
    ]
    chain.bench_rc = 0
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--master", "2"])
    # One question: the bench's confirmation (it carries the conditions), not run's.
    assert chain.asked == []
    assert (
        "/m/drive.yaml is not approved for motion. Starting the drive bench."
        in capsys.readouterr().out
    )
    assert chain.benched == [["/m/drive.yaml", "--master", "2"]]
    assert chain.execd[0] == "/x/ros2_control_node"
    assert "unapproved" not in (pathlib.Path(chain.execd[-1]).read_text())


def test_a_declined_or_failed_bench_starts_nothing_and_says_why(chain, capsys):
    chain.checks = [
        check(axis("p", "csp"), approval="unapproved"),
        check(axis("p", "csp")),
    ]
    chain.bench_rc = 1  # declined at its confirmation, or not approved
    assert run.main(["/m/drive.yaml"]) == 1
    assert chain.execd is None and chain.spawned == [] and len(chain.benched) == 1
    # The bench said why on its own line: run adds nothing after it.
    assert capsys.readouterr().err == ""


def test_an_approval_that_no_longer_holds_is_refused_with_checks_reasons():
    stale = check(axis("p", "csp"), approval="unapproved")
    stale["reasons"] = [
        "approval.record.config_digest is 'sha256:c9' but the file's is 'sha256:c0'"
    ]
    error = run.refusal(stale, "/m/a.yaml", unapproved=False)
    assert "no longer holds: approval.record.config_digest is 'sha256:c9'" in error


def test_without_a_terminal_an_unapproved_file_is_refused_with_the_reason(
    chain, capsys
):
    chain.checks = [check(axis("p", "csp"), approval="unapproved")]
    chain.tty = False
    assert run.main(["/m/drive.yaml"]) == 1
    assert chain.asked == [] and chain.benched == [] and chain.execd is None
    assert "on a terminal offers the bench" in capsys.readouterr().err


def test_unapproved_starts_without_asking(chain):
    chain.checks = [check(axis("p", "csp"), approval="unapproved")]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--unapproved"])
    assert chain.asked == [] and chain.benched == []
    assert chain.probed == [], "a bench run compares no record"


def test_an_unapproved_start_says_bench_mode_even_for_an_approved_file(chain, capsys):
    chain.checks = [check(axis("p", "csp"))]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--unapproved"])
    out = capsys.readouterr().out
    assert "master 0, bench mode, its approval record unused (/m/drive.yaml)" in out


def test_each_start_of_a_bus_reuses_one_private_directory(chain, tmp_path):
    for _ in range(2):
        chain.checks = [check(axis("p", "csp"), bus="left_arm")]
        with pytest.raises(Started):
            run.main(["/m/drive.yaml", "--plot"])
    dirs = [d for d in tmp_path.iterdir() if d.name.startswith("plem-drive-run")]
    assert [d.name for d in dirs] == ["plem-drive-run-left_arm"]
    assert dirs[0].stat().st_mode & 0o777 == 0o700
    assert {f.name for f in dirs[0].iterdir()} == {"params.yaml", "plotjuggler.xml"}


def test_a_runtime_directory_others_can_write_gets_a_fresh_private_one(
    chain, tmp_path, monkeypatch
):
    shared = tmp_path / "shared"
    shared.mkdir(mode=0o777)
    shared.chmod(0o777)
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(shared))
    monkeypatch.setattr(run.tempfile, "tempdir", str(tmp_path))
    chain.checks = [check(axis("p", "csp"), bus="left_arm")]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml"])
    assert list(shared.iterdir()) == []
    fresh = [d for d in tmp_path.iterdir() if d.name.startswith("plem-drive-run-")]
    assert len(fresh) == 1 and fresh[0].name != "plem-drive-run-left_arm"
    assert fresh[0].stat().st_mode & 0o777 == 0o700


def test_a_runtime_directory_of_another_user_gets_a_fresh_private_one(
    chain, tmp_path, monkeypatch
):
    # sudo -E or su keeps the other user's XDG_RUNTIME_DIR.
    monkeypatch.setattr(run.os, "getuid", lambda: os.stat(tmp_path).st_uid + 1)
    monkeypatch.setattr(run.tempfile, "tempdir", str(tmp_path / "tmp"))
    (tmp_path / "tmp").mkdir()
    chain.checks = [check(axis("p", "csp"), bus="left_arm")]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml"])
    assert not (tmp_path / "plem-drive-run-left_arm").exists()
    assert len(list((tmp_path / "tmp").iterdir())) == 1


def test_the_buses_rebench_reasons_carry_the_file_placeholder_run_fills_in():
    # run names the file in the bus's reason instead of repeating its advice.
    fieldbus = (
        pathlib.Path(__file__).resolve().parents[3]
        / "plem_fieldbus"
        / "src"
        / "ethercat"
    )
    advice = "(`plem drive run <file>` on a terminal offers the bench)"
    for source in ("bus.cpp", "pdo_scan.cpp"):
        assert advice in (fieldbus / source).read_text(), source


# An approved file: the drives on the bus are compared with its record

SWAPPED = (
    "slave 0: SII serial number is 67890 but the approval record declares 12345 — … "
    "re-bench this one (`plem drive run <file>` on a terminal offers the bench)."
)


def test_an_approved_file_on_its_drives_starts_without_asking(chain):
    chain.checks = [check(axis("p", "csp"))]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--master", "2"])
    assert chain.probed == [("/m/drive.yaml", 2)]
    assert chain.asked == [] and chain.benched == []


def test_other_drives_of_the_same_model_are_benched_on_a_terminal_then_started(
    chain, capsys
):
    chain.checks = [check(axis("p", "csp")), check(axis("p", "csp"))]
    chain.probe = (3, SWAPPED)
    chain.bench_rc = 0
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--master", "2"])
    out = capsys.readouterr().out
    assert (
        "/m/drive.yaml: the drives on the bus differ from its approval record"
        ". Starting the drive bench:" in out
    )
    assert SWAPPED.replace("<file>", "/m/drive.yaml") in out
    assert chain.benched == [["/m/drive.yaml", "--master", "2"]]
    assert chain.execd[0] == "/x/ros2_control_node"


def test_other_drives_without_a_terminal_are_refused_with_the_reason(chain, capsys):
    chain.checks = [check(axis("p", "csp"))]
    chain.probe = (3, SWAPPED)
    chain.tty = False
    assert run.main(["/m/drive.yaml"]) == 1
    err = capsys.readouterr().err
    assert "/m/drive.yaml: the drives on the bus differ from its approval record" in err
    # The bus's reason says how to re-bench, with the file named; run adds only
    # what a terminal-less start can do instead.
    assert err.count("on a terminal") == 1
    assert "`plem drive run /m/drive.yaml` on a terminal offers the bench" in err
    assert "--unapproved" in err
    assert chain.benched == [] and chain.execd is None and chain.spawned == []


def test_the_probe_runs_the_tool_with_plem_logging_off(monkeypatch):
    # Its stderr is the reason shown to the operator: the bus's words only,
    # none of the C++ logger's lines.
    seen = {}

    def fake_run(argv, **kwargs):
        seen.update(argv=argv, **kwargs)
        return run.subprocess.CompletedProcess(argv, 3, "", "slave 0: differs\n")

    monkeypatch.setattr(run, "drive_model_tool", lambda: "/x/drive_model_tool")
    monkeypatch.setattr(run.subprocess, "run", fake_run)
    monkeypatch.setenv("PLEM_LOG_LEVEL", "debug")
    monkeypatch.setenv("KEEP_ME", "1")
    assert run._probe("/m/drive.yaml", 2) == (3, "slave 0: differs")
    assert seen["argv"] == [
        "/x/drive_model_tool",
        "probe",
        "/m/drive.yaml",
        "--master",
        "2",
    ]
    assert seen["env"]["PLEM_LOG_LEVEL"] == "off" and seen["env"]["KEEP_ME"] == "1"
    assert seen["capture_output"] and seen["text"]


def test_drives_a_bench_cannot_settle_are_refused_in_the_buss_words(chain, capsys):
    chain.checks = [check(axis("p", "csp"))]
    chain.probe = (
        1,
        "the drives are in another order than at the bench: restore the cabling",
    )
    assert run.main(["/m/drive.yaml"]) == 1
    err = capsys.readouterr().err
    assert "plem drive run: the drives are in another order than at the bench" in err
    assert chain.benched == [] and chain.execd is None and chain.spawned == []


def test_monitor_runs_beside_the_bus_on_its_namespace(chain):
    chain.checks = [check(axis("p", "csp"))]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--monitor"])
    assert [sys.executable, "-m", "plem.drive.cli.monitor", "/pair"] in chain.spawned
    chain.checks = [check(axis("p", "csp"))]
    chain.spawned.clear()
    with pytest.raises(Started):
        run.main(["/m/drive.yaml"])
    assert not any("plem.drive.cli.monitor" in argv for argv in chain.spawned)


def test_a_log_level_the_logger_does_not_know_is_said_before_the_start():
    said = run.log_level_warning("verbose")
    assert said.startswith(
        "PLEM_LOG_LEVEL='verbose' is not a level (trace, debug, info, "
    )
    assert said.endswith(
        "); ignored, the default applies (warn, or info with --verbose)"
    )
    for fine in (None, "", "debug", "warn", "off"):
        assert run.log_level_warning(fine) is None, fine


def test_by_default_only_warnings_and_the_buses_own_lines_reach_the_terminal(
    chain, capsys
):
    chain.checks = [check(axis("p", "csp"), axis("q", "csv"), bus="left_arm")]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--plot"])
    execd = " ".join(chain.execd)
    assert "--log-level warn --log-level left_arm.plem_drive:=info" in execd
    for argv in chain.spawned:
        quiet = argv[-2:] == ["--log-level", "warn"]
        # The spawners keep their lines: "Configured and activated" is the ready sign.
        assert quiet == (argv[0] in ("/x/robot_state_publisher", "/x/rt_raw_relay")), (
            argv
        )
    assert run.os.environ["PLEM_LOG_LEVEL"] == "warn"
    log = chain.outputs["/x/plotjuggler"]
    assert log is not None and log.name == "plotjuggler.log"
    out = capsys.readouterr().out
    assert "left_arm: 2 axes, 1 ms cycle, master 0, approved (/m/drive.yaml)" in out
    assert "p  csp" in out and "q  csv" in out
    assert str(log) in out and "--verbose" in out


def test_verbose_leaves_every_log_as_it_is(chain, monkeypatch):
    monkeypatch.setenv("PLEM_LOG_LEVEL", "debug")
    chain.checks = [check(axis("p", "csp"))]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--verbose", "--plot"])
    assert "--log-level" not in chain.execd
    assert not any("--log-level" in argv for argv in chain.spawned)
    assert run.os.environ["PLEM_LOG_LEVEL"] == "debug"
    assert chain.outputs["/x/plotjuggler"] is None


def test_a_level_the_user_set_is_kept(chain, monkeypatch):
    monkeypatch.setenv("PLEM_LOG_LEVEL", "info")
    chain.checks = [check(axis("p", "csp"))]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml"])
    assert run.os.environ["PLEM_LOG_LEVEL"] == "info"


def test_the_namespace_is_the_files_bus_name_not_its_file_name(chain):
    chain.checks = [check(axis("p", "csp"), bus="left_arm")]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml"])
    assert "__ns:=/left_arm" in chain.execd
    import yaml

    params = yaml.safe_load(pathlib.Path(chain.execd[-1]).read_text())
    assert "/left_arm/controller_manager" in params
    # The manager reads the description from robot_state_publisher's topic
    # (Humble through the remap), never from a parameter (Jazzy reads none).
    manager = params["/left_arm/controller_manager"]["ros__parameters"]
    assert "robot_description" not in manager
    assert "~/robot_description:=robot_description" in chain.execd
    assert (
        "robot_description"
        in params["/left_arm/robot_state_publisher"]["ros__parameters"]
    )
    assert [
        "/x/spawner",
        "joint_state_broadcaster",
        "-c",
        "/left_arm/controller_manager",
        "--controller-manager-timeout",
        "90",
    ] in chain.spawned


def test_there_is_no_ns_option_the_file_names_the_bus(chain, capsys):
    with pytest.raises(SystemExit) as exc:
        run.main(["/m/drive.yaml", "--ns", "/b"])
    assert exc.value.code == 2
    assert "--ns" not in run.USAGE
    assert "bus.name" in run.USAGE


def test_a_bus_of_that_name_already_up_is_refused_before_anything_starts(
    chain, capsys, monkeypatch
):
    chain.checks = [check(axis("p", "csp"))]
    monkeypatch.setattr(run, "run_pids", lambda ns: [4242] if ns == "/pair" else [])
    assert run.main(["/m/drive.yaml"]) == 1
    assert chain.execd is None and chain.spawned == [] and chain.benched == []
    err = capsys.readouterr().err
    assert "a bus named /pair is already up (pid 4242)" in err
    assert "bus.name" in err


def test_the_plotted_message_has_no_bounded_sequence():
    # PlotJuggler's ROS 2 plugin reads `T[<=N]` as a fixed array of N and fails
    # to decode every message ("decode: not enough data").
    from plem_rt_msgs.msg import RtRawBatch
    from rosidl_parser.definition import BoundedSequence

    bounded = [
        name
        for name, kind in zip(
            RtRawBatch.get_fields_and_field_types(), RtRawBatch.SLOT_TYPES
        )
        if isinstance(kind, BoundedSequence)
    ]
    assert bounded == []


def test_plot_starts_the_relay_and_plotjuggler_on_this_buses_layout(chain):
    chain.checks = [check(axis("p", "csp"), bus="left_arm")]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--plot"])
    assert [
        "/x/rt_raw_relay",
        "--ros-args",
        "-r",
        "__ns:=/left_arm",
        "--log-level",
        "warn",
    ] in chain.spawned
    plot = [argv for argv in chain.spawned if argv[0] == "/x/plotjuggler"]
    assert len(plot) == 1 and plot[0][1] == "--layout"
    assert pathlib.Path(plot[0][2]).read_text() == run.plot_layout("left_arm", ["p"])


SAMPLES = "/{}/rt_raw/samples/{}"


def tab(root, name):
    (found,) = [t for t in root.iter("Tab") if t.get("tab_name") == name]
    return found


def curves(area):
    return [c.get("name") for c in area.iter("curve")]


def test_the_plot_layout_of_a_one_axis_bus_plots_that_axis():
    root = ET.fromstring(run.plot_layout("solo", ["j0"]))
    (area,) = tab(root, "position").iter("DockArea")
    assert area.get("name") == "j0"
    assert curves(area) == [
        SAMPLES.format("solo", f"{field}[0]")
        for field in ("actual_position", "applied_command", "command_position")
    ]
    velocity, torque = tab(root, "velocity_torque").iter("DockArea")
    assert curves(velocity) == [SAMPLES.format("solo", "actual_velocity[0]")]
    assert curves(torque) == [SAMPLES.format("solo", "actual_torque[0]")]
    # The bus's own timing and health, sample 0 of the batch's per-cycle fields.
    for name, fields in (
        ("timing", ["loop_exec_us[0]", "loop_jitter_us[0]"]),
        ("bus", ["wkc_mismatch[0]", "deadline_miss[0]", "dc_sync_monitor_ns[0]"]),
    ):
        got = [c for a in tab(root, name).iter("DockArea") for c in curves(a)]
        assert got == [SAMPLES.format("solo", f) for f in fields]
    assert {t.get("value") for t in root.iter("selected_topics")} == {
        "/solo/rt_raw/samples"
    }
    assert not list(root.iter("snippet")), (
        "no pair arithmetic on a bus that is not a pair"
    )
    assert "[1]" not in ET.tostring(root, encoding="unicode")


def test_the_plot_layout_of_a_three_axis_bus_plots_every_axis():
    root = ET.fromstring(run.plot_layout("trio", ["a", "b", "c"]))
    position = list(tab(root, "position").iter("DockArea"))
    assert [a.get("name") for a in position] == ["a", "b", "c"]
    for i, area in enumerate(position):
        assert SAMPLES.format("trio", f"actual_position[{i}]") in curves(area)
    velocity, torque = tab(root, "velocity_torque").iter("DockArea")
    assert curves(velocity) == [
        SAMPLES.format("trio", f"actual_velocity[{i}]") for i in range(3)
    ]
    assert curves(torque) == [
        SAMPLES.format("trio", f"actual_torque[{i}]") for i in range(3)
    ]
    splitter = tab(root, "position").find("Container/DockSplitter/DockSplitter")
    assert splitter.get("count") == "3"
    assert sum(float(x) for x in splitter.get("sizes").split(";")) == pytest.approx(1.0)


def test_plot_without_plotjuggler_is_refused_naming_the_package_before_anything(
    chain, capsys, monkeypatch
):
    monkeypatch.setattr(run, "_plotjuggler", lambda: None)
    monkeypatch.setenv("ROS_DISTRO", "humble")
    chain.checks = [check(axis("p", "csp"), approval="unapproved")]
    chain.bench_rc = 0
    assert run.main(["/m/drive.yaml", "--plot"]) == 1
    assert chain.execd is None and chain.spawned == [] and chain.benched == []
    assert len(chain.checks) == 1, "refused before the file is even checked"
    assert "sudo apt install ros-humble-plotjuggler-ros" in capsys.readouterr().err
    monkeypatch.delenv(
        "ROS_DISTRO"
    )  # the release's distro when the environment names none
    assert run.main(["/m/drive.yaml", "--plot"]) == 1
    assert "sudo apt install ros-jazzy-plotjuggler-ros" in capsys.readouterr().err


def test_plotjuggler_without_its_ros2_streamer_counts_as_missing(
    chain, capsys, monkeypatch, tmp_path
):
    # plotjuggler alone opens no ROS 2 topic: the layout needs plotjuggler_ros.
    from ament_index_python import packages

    exe = tmp_path / "lib" / "plotjuggler" / "plotjuggler"
    exe.parent.mkdir(parents=True)
    exe.write_text("")
    exe.chmod(0o755)

    def prefix(package):
        if package == "plotjuggler":
            return str(tmp_path)
        raise packages.PackageNotFoundError(package)

    monkeypatch.setattr(packages, "get_package_prefix", prefix)
    monkeypatch.setattr(run, "_executable", REAL_EXECUTABLE)
    monkeypatch.setattr(run, "_plotjuggler", REAL_PLOTJUGGLER)
    assert run._plotjuggler() is None
    chain.checks = [check(axis("p", "csp"))]
    assert run.main(["/m/drive.yaml", "--plot"]) == 1
    assert len(chain.checks) == 1, "refused before the file is even checked"
    assert chain.execd is None and chain.spawned == []
    assert "plotjuggler-ros" in capsys.readouterr().err
    # Both present: the plotjuggler executable.
    monkeypatch.setattr(packages, "get_package_prefix", lambda package: str(tmp_path))
    assert run._plotjuggler() == str(exe)


def test_the_benchs_private_namespace_overrides_the_bus_name(chain, monkeypatch):
    # The bench's child run (only it sets this) keeps the bus away from a
    # Bus("<bus.name>") client such as the example's demo.
    monkeypatch.setenv(run.BENCH_NAMESPACE_ENV, run.BENCH_NAMESPACE)
    chain.checks = [check(axis("p", "csp"), approval="unapproved")]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--unapproved"])
    assert f"__ns:={run.BENCH_NAMESPACE}" in chain.execd


def test_run_pids_matches_this_namespace_token_only(tmp_path):
    def entry(pid, exe, ns):
        d = tmp_path / str(pid)
        d.mkdir()
        (d / "exe").symlink_to(f"/opt/ros/humble/lib/x/{exe}")
        (d / "cmdline").write_bytes(
            b"\0".join([exe.encode(), b"--ros-args", b"-r", f"__ns:={ns}".encode()])
            + b"\0"
        )

    entry(101, "ros2_control_node", "/pair")
    entry(102, "ros2_control_node", "/pair2")
    entry(103, "robot_state_publisher", "/pair")
    entry(104, "python3", "/pair")
    (tmp_path / "self").mkdir()
    assert sorted(run.run_pids("/pair", proc=tmp_path)) == [101, 103]
    assert run.run_pids("/pair2", proc=tmp_path) == [102]


def test_run_reads_checks_json(monkeypatch):
    # check prints a summary for people; run asks for the machine contract.
    calls = []

    def ran(argv, **_):
        calls.append(argv)
        return SimpleNamespace(
            returncode=0, stdout=json.dumps(check(axis("p", "csp"))), stderr=""
        )

    monkeypatch.setattr(
        run, "drive_model_tool", lambda: "/p/lib/plem_drive/drive_model_tool"
    )
    monkeypatch.setattr(run.subprocess, "run", ran)
    run._check("/m/a.yaml")
    # The tool of this install, never a plem-drive found on PATH.
    assert calls == [
        ["/p/lib/plem_drive/drive_model_tool", "check", "--json", "/m/a.yaml"]
    ]


def test_the_executables_come_from_their_packages_install_prefixes(monkeypatch):
    from ament_index_python import packages

    monkeypatch.setattr(
        packages, "get_package_prefix", lambda package: f"/opt/{package}"
    )
    monkeypatch.setattr(run, "_executable", REAL_EXECUTABLE)
    assert run.drive_model_tool() == "/opt/plem_drive/lib/plem_drive/drive_model_tool"
    assert run.plem_drive_command() == "/opt/plem_drive/bin/plem-drive"


def test_check_without_plem_drive_or_its_tool_is_a_refusal_not_a_traceback(monkeypatch):
    from ament_index_python import packages

    def missing(package):
        raise packages.PackageNotFoundError(package)

    monkeypatch.setattr(packages, "get_package_prefix", missing)
    monkeypatch.setattr(run, "_executable", REAL_EXECUTABLE)
    with pytest.raises(
        RuntimeError,
        match="package plem_drive not found — source the setup.bash of the install that has plem_drive$",
    ):
        run._check("/m/a.yaml")
    monkeypatch.setattr(
        run, "drive_model_tool", lambda: "/nonexistent/drive_model_tool"
    )
    with pytest.raises(
        RuntimeError,
        match="cannot run the drive model tool /nonexistent/drive_model_tool .* — rebuild or reinstall plem_drive$",
    ):
        run._check("/m/a.yaml")


def test_a_check_without_a_bus_name_is_a_release_mismatch(monkeypatch):
    monkeypatch.setattr(run, "drive_model_tool", lambda: "/x/drive_model_tool")
    printed = check(axis("p", "csp"))
    del printed["bus_name"]
    monkeypatch.setattr(
        run.subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(
            returncode=0, stdout=json.dumps(printed), stderr=""
        ),
    )
    with pytest.raises(RuntimeError, match="install matching versions"):
        run._check("/m/a.yaml")


def test_a_file_whose_approval_no_longer_holds_is_read_as_unapproved(monkeypatch):
    monkeypatch.setattr(run, "drive_model_tool", lambda: "/x/drive_model_tool")
    # check exits 1 with its JSON for an approval claim it does not honour.
    printed = json.dumps(check(axis("p", "csp"), approval="unapproved"))
    monkeypatch.setattr(
        run.subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(returncode=1, stdout=printed, stderr="mapping"),
    )
    assert run._check("/m/a.yaml")["approval_effective"] == "unapproved"
    monkeypatch.setattr(
        run.subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(returncode=1, stdout="", stderr="bad file"),
    )
    with pytest.raises(RuntimeError, match="bad file"):
        run._check("/m/a.yaml")


def test_a_check_of_another_format_is_refused_naming_matching_versions(monkeypatch):
    monkeypatch.setattr(run, "drive_model_tool", lambda: "/x/drive_model_tool")
    printed = json.dumps({**check(axis("p", "csp")), "format": "plem-drive-check/2"})
    monkeypatch.setattr(
        run.subprocess,
        "run",
        lambda *a, **k: SimpleNamespace(returncode=0, stdout=printed, stderr=""),
    )
    with pytest.raises(RuntimeError, match="install matching versions"):
        run._check("/m/a.yaml")


def test_the_manager_keeps_off_the_isolated_cpus_it_may_run_on():
    params = run.controller_params(
        check(axis("p", "csp")), cpus={0, 1, 3}, isolated={3}
    )
    assert params["controller_manager"]["ros__parameters"]["cpu_affinity"] == [0, 1]
    params = run.controller_params(check(axis("p", "csp")), cpus={3}, isolated={3})
    assert "cpu_affinity" not in params["controller_manager"]["ros__parameters"], (
        "nowhere else to go"
    )


def test_rt_cpu_pins_the_bus_loop_and_keeps_the_manager_off_it():
    urdf = run.robot_description(check(axis("p", "csp")), "/m/a.yaml", rt_cpu=2)
    hardware = ET.fromstring(urdf).find("ros2_control/hardware")
    assert {p.get("name"): p.text for p in hardware.iter("param")}["rt_cpu"] == "2"
    params = run.controller_params(
        check(axis("p", "csp")), cpus={0, 1, 2, 3}, isolated={3}, rt_cpu=1
    )
    assert params["controller_manager"]["ros__parameters"]["cpu_affinity"] == [0, 2]
    params = run.controller_params(
        check(axis("p", "csp")), cpus={0, 1, 2}, isolated=set(), rt_cpu=2
    )
    assert params["controller_manager"]["ros__parameters"]["cpu_affinity"] == [0, 1]


def test_rt_cpu_reaches_the_hardware(chain):
    chain.checks = [check(axis("p", "csp"))]
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--rt-cpu", "2"])
    import yaml

    params = yaml.safe_load(pathlib.Path(chain.execd[-1]).read_text())
    urdf = params["/pair/robot_state_publisher"]["ros__parameters"]["robot_description"]
    hardware = ET.fromstring(urdf).find("ros2_control/hardware")
    assert {p.get("name"): p.text for p in hardware.iter("param")}["rt_cpu"] == "2"
    assert "--rt-cpu" in run.USAGE


def test_the_bench_runs_on_the_same_rt_cpu(chain, monkeypatch):
    monkeypatch.setattr(run.os, "sched_getaffinity", lambda pid: {0, 1})
    chain.checks = [
        check(axis("p", "csp"), approval="unapproved"),
        check(axis("p", "csp")),
    ]
    chain.bench_rc = 0
    with pytest.raises(Started):
        run.main(["/m/drive.yaml", "--rt-cpu", "1"])
    assert chain.benched == [["/m/drive.yaml", "--master", "0", "--rt-cpu", "1"]]


def test_an_rt_cpu_this_process_may_not_run_on_is_refused_before_anything(
    chain, capsys, monkeypatch
):
    monkeypatch.setattr(run.os, "sched_getaffinity", lambda pid: {0, 1})
    assert run.main(["/m/drive.yaml", "--rt-cpu", "7"]) == 1
    assert chain.execd is None and chain.spawned == [] and chain.asked == []
    assert run.RT_CPU_REFUSAL.format(7) in capsys.readouterr().err


def test_the_rt_cpu_refusal_is_the_hardware_plugins_own_text():
    source = (
        pathlib.Path(__file__).resolve().parents[2]
        / "src"
        / "ros"
        / "drive_system_interface.cpp"
    ).read_text()
    throw = source[source.index('throw std::invalid_argument("rt_cpu') :]
    throw = throw[: throw.index(";")]
    assert "".join(
        re.findall(r'"((?:[^"\\]|\\.)*)"', throw)
    ) == run.RT_CPU_REFUSAL.format("")


def test_the_pair_example_names_one_bus_in_its_file_layout_and_demo():
    folder = EXAMPLES / "a6b_pair"
    assert "bus: { name: a6b_pair," in (folder / "drive.yaml").read_text()
    layout = PAIR_LAYOUT.read_text()
    root = ET.fromstring(layout)
    topics = {t for t in re.findall(r"/[A-Za-z0-9_]+/rt_raw/samples", layout)}
    assert topics == {"/a6b_pair/rt_raw/samples"}
    snippets = {s.get("name") for s in root.iter("snippet")}
    assert snippets == {"position_mismatch", "velocity_sum", "opposing_torque"}
    demo = (folder / "demo.py").read_text()
    compile(demo, "demo.py", "exec")
    assert 'Bus("a6b_pair")' in demo


def test_an_argument_error_shows_the_synopsis_once_not_the_help(capsys):
    with pytest.raises(SystemExit) as exit_:
        run.main(["/m/drive.yaml", "--master", "x"])
    assert exit_.value.code == 2
    err = capsys.readouterr().err
    assert err.startswith("usage: plem drive run <model.yaml> ")
    assert "usage: usage:" not in err
    assert "--rt-cpu N      the CPU" not in err, "the option help stays with -h"
    assert "plem drive run: error: argument --master: invalid int value: 'x'" in err


def test_a_side_process_ends_with_the_controller_manager(tmp_path):
    """run's monitor, relay and PlotJuggler get SIGTERM when the process that
    spawned them (the controller manager, after run's exec) dies, however it dies."""
    import contextlib
    import signal
    import subprocess
    import textwrap

    from ros_test_domain import wait_until

    pidfile = tmp_path / "child.pid"
    parent = subprocess.Popen(
        [
            sys.executable,
            "-c",
            textwrap.dedent(f"""
                import time
                from plem.drive.cli import run
                run._spawn_child(["sh", "-c", "echo $$ > {pidfile}; exec sleep 60"])
                time.sleep(60)
            """),
        ],
        env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        start_new_session=True,  # its own group: cleanup reaches a child whose pid was never written
    )
    child = None

    def gone():
        """Ended: no such process, or a zombie its new parent has not reaped yet."""
        try:
            stat = pathlib.Path(f"/proc/{child}/stat").read_text()
        except FileNotFoundError:
            return True
        return stat.rsplit(")", 1)[1].split()[0] == "Z"

    try:
        assert wait_until(lambda: pidfile.exists() and pidfile.read_text().strip()), (
            "the child never started"
        )
        child = int(pidfile.read_text())
        parent.kill()
        parent.wait()
        assert wait_until(gone), "the child outlived its parent"
    finally:
        with contextlib.suppress(ProcessLookupError):
            os.killpg(parent.pid, signal.SIGKILL)
        parent.wait()


@pytest.mark.parametrize("spawner_alive", [True, False])
def test_a_side_process_starts_only_while_its_spawner_lives(
    tmp_path, monkeypatch, spawner_alive
):
    """A spawner that died before setpriv set the death signal has made the child
    another's (here: a pid not its parent); the child then exits without running."""
    import subprocess

    started = []
    real_popen = subprocess.Popen
    monkeypatch.setattr(
        run.subprocess, "Popen", lambda *a, **k: started.append(real_popen(*a, **k))
    )
    if not spawner_alive:
        monkeypatch.setattr(run.os, "getpid", lambda: 1)
    ran = tmp_path / "ran"
    run._spawn_child(["touch", str(ran)])
    assert started[0].wait(timeout=5) == (0 if spawner_alive else 1)
    assert ran.exists() == spawner_alive
