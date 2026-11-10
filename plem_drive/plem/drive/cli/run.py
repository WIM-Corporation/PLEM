# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem drive run — start a drive bus under ros2_control from its model file.

`plem drive check` validates the file; from its JSON this writes the robot
description (one joint per axis, the drive system interface as its hardware)
and the controller manager's parameters, then starts the bus: the
robot_state_publisher and the controller spawners as children, and the
controller manager itself in this process's place (exec), so Ctrl-C reaches it
directly and the system interface stops the axes (a requested stop) before
it shuts the bus down.

The bus is named by the file's `bus.name` and runs in the namespace
/<bus.name>; a second run of a bus already up on this host is refused.

A csp axis is moved by the joint trajectory controller (move_to, MoveIt); a
csv or cst axis follows a stream through the stream controller. About 100 ms
of stream silence (the controller's 50 plus the bus's 50) is a protective
stop. Host checks (RT kernel, isolated CPU, nvpps lock) print warnings and
never stop the run.

When the file is not approved for motion and `run` is on a terminal, `run`
says so and offers the bench at the drives (its confirmation is the only
prompt). The bench records the approval in the file in place, and `run`
starts the bus once the file is approved. If the bench is declined or fails,
or there is no terminal, `run` refuses. `--unapproved` starts the bus anyway;
it is meant for an operator at the drives, for example the bench itself.

For an approved file, `run` first compares the drives on the bus with the
approval record (the bus's start-up check, without starting the bus). If only
an identity value differs (another unit of the same model), `run` offers the
bench on a terminal (the bench replaces the approval) and refuses without
one. A different model, a missing drive or drives in a different order are
refused with the bus's own message.

--monitor prints the bus timing in this terminal, alongside the bus's output; --plot starts rt_raw_relay and
PlotJuggler on this bus. Each is a child that ends with the bus.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import platform
import shlex
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

TRAJECTORY_CONTROLLER = "joint_trajectory_controller"
STREAM_CONTROLLER = "stream_controller"
_COMMAND = {"csp": ("position", "velocity"), "csv": ("velocity",), "cst": ("effort",)}
_NVPPS = pathlib.Path("/sys/firmware/devicetree/base/bus@0/nvpps")
# The hardware plugin's own refusal (drive_system_interface.cpp rt_cpu_affinity;
# a test compares the two), said here before anything starts.
RT_CPU_REFUSAL = (
    "rt_cpu must be a CPU this process may run on (got '{}'), or unset for the last "
    "isolated CPU — the isolated CPUs are listed in /sys/devices/system/cpu/isolated"
)

USAGE = """usage: plem drive run <model.yaml> [--master N] [--rt-cpu N] [--monitor] [--plot]
                       [--unapproved] [--verbose]

Start the drive bus the file describes under ros2_control, in this terminal;
Ctrl-C stops every axis (decelerating at limits.acceleration_rad_s2; on a
bus with a cst axis, every drive is disabled at once), requests the brakes,
reports whether they confirmed engaged, and then shuts the bus down. The bus
runs in the namespace /<bus.name> (the file's `bus: { name: … }`). A file not yet approved for motion: on a terminal, run
offers to bench it at the drives first (the approval is recorded in the file)
and then starts it.

  --master N      EtherCAT master index (default 0)
  --rt-cpu N      the CPU the bus loop is pinned to (default: the last
                  isolated CPU) — two buses on one host need different CPUs
  --monitor       print the bus timing (rt_raw) once a second in this terminal
  --plot          plot every cycle: rt_raw_relay and PlotJuggler on this bus
  --unapproved    start the bus even if the file is not approved (its approval
                  record is ignored); only with an operator at the drives,
                  for example to collect bench evidence by hand
  --verbose       show every node's log (by default warnings and errors, and
                  the bus's own lines; PlotJuggler's output goes to a file)

Moving: from Python, `from plem.drive import Bus` (Bus("<bus.name>").move_to(...));
or a FollowJointTrajectory goal to /<bus.name>/joint_trajectory_controller.
"""

# The terminal by default: warnings and errors, the bus node's own lines
# (waiting for OP, bench evidence) and the spawners' (controllers active).
# Native knobs only: the ROS log level and plem's PLEM_LOG_LEVEL (a value the
# user set is kept).
QUIET = ["--log-level", "warn"]
# The controller manager takes the description from robot_state_publisher's
# topic, never a parameter (Jazzy reads none). Humble subscribes
# ~/robot_description, remapped here; Jazzy subscribes robot_description
# itself, where the remap is inert.
DESCRIPTION_FROM_TOPIC = ["-r", "~/robot_description:=robot_description"]

PLOTJUGGLER_MISSING = (
    "--plot needs PlotJuggler and its ROS 2 streamer (plotjuggler_ros), which are not "
    "installed — sudo apt install ros-{}-plotjuggler-ros"
)
# The bench's bus runs here, never on the file's bus.name, so a Bus("<bus.name>")
# client cannot attach to it. Set only by the bench, for its child run.
BENCH_NAMESPACE = "/plem_drive_bench"
BENCH_NAMESPACE_ENV = "PLEM_DRIVE_BENCH_NAMESPACE"


PLOT_COLORS = ("#1f77b4", "#d62728", "#1ac938", "#ff7f0e", "#9467bd", "#8c564b")


def plot_layout(bus: str, axes: list[str]) -> str:
    """--plot's PlotJuggler layout for this bus: a position pane per axis, every
    axis's velocity and torque, the loop timing and the bus health. Sample index
    i is the i-th axis of the file (the bus's joint order)."""
    samples = f"/{bus}/rt_raw/samples"
    root = ET.Element("root")
    tabs = ET.SubElement(
        root, "tabbed_widget", parent="main_window", name="Main Window"
    )

    def tab(name: str, panes: list[tuple[str, list[str]]]) -> None:
        split = ET.SubElement(
            ET.SubElement(
                ET.SubElement(tabs, "Tab", containers="1", tab_name=name), "Container"
            ),
            "DockSplitter",
            orientation="-",
            sizes="1",
            count="1",
        )
        share = round(1 / len(panes), 6)
        sizes = [share] * (len(panes) - 1) + [1 - share * (len(panes) - 1)]  # sum to 1
        inner = ET.SubElement(
            split,
            "DockSplitter",
            orientation="-",
            count=str(len(panes)),
            sizes=";".join(f"{x:.6f}" for x in sizes),
        )
        for title, fields in panes:
            plot = ET.SubElement(
                ET.SubElement(inner, "DockArea", name=title),
                "plot",
                mode="TimeSeries",
                line_width="1.0",
                flip_x="false",
                flip_y="false",
            )
            ET.SubElement(
                plot,
                "range",
                left="0.000000",
                bottom="-1.000000",
                top="1.000000",
                right="10.000000",
            )
            ET.SubElement(plot, "limitY")
            for k, field in enumerate(fields):
                ET.SubElement(
                    plot,
                    "curve",
                    color=PLOT_COLORS[k % len(PLOT_COLORS)],
                    name=f"{samples}/{field}",
                )

    each = range(len(axes))
    tab(
        "position",
        [
            (
                name,
                [
                    f"{f}[{i}]"
                    for f in ("actual_position", "applied_command", "command_position")
                ],
            )
            for i, name in enumerate(axes)
        ],
    )
    tab(
        "velocity_torque",
        [
            ("velocity", [f"actual_velocity[{i}]" for i in each]),
            ("torque", [f"actual_torque[{i}]" for i in each]),
        ],
    )
    tab("timing", [("exec", ["loop_exec_us[0]"]), ("jitter", ["loop_jitter_us[0]"])])
    tab(
        "bus",
        [
            ("errors", ["wkc_mismatch[0]", "deadline_miss[0]"]),
            ("dc", ["dc_sync_monitor_ns[0]"]),
        ],
    )
    ET.SubElement(tabs, "currentTabIndex", index="0")
    ET.SubElement(root, "use_relative_time_offset", enabled="1")
    plugins = ET.SubElement(root, "Plugins")
    for plugin in ("DataLoad ROS2 bags", "ROS2 Topic Subscriber"):
        p = ET.SubElement(plugins, "plugin", ID=plugin)
        for key, value in (
            ("use_header_stamp", "true"),
            ("discard_large_arrays", "true"),
            ("max_array_size", "100"),
            ("boolean_strings_to_number", "true"),
            ("remove_suffix_from_strings", "true"),
            ("selected_topics", samples),
        ):
            ET.SubElement(p, key, value=value)
    ET.SubElement(root, "previouslyLoaded_Datafiles")
    ET.SubElement(root, "previouslyLoaded_Streamer", name="ROS2 Topic Subscriber")
    ET.SubElement(root, "customMathEquations")
    ET.SubElement(root, "snippets")
    ET.indent(root, " ")
    return (
        "<?xml version='1.0' encoding='UTF-8'?>\n"
        + ET.tostring(root, encoding="unicode")
        + "\n"
    )


def own_pids(match, proc: pathlib.Path = pathlib.Path("/proc")) -> list[int]:
    """This user's processes (another user's /proc/<pid>/exe is unreadable)
    for which match(exe, cmdline) holds."""
    found = []
    for p in proc.iterdir():
        if p.name.isdigit() and int(p.name) != os.getpid():
            try:
                if match(os.readlink(p / "exe"), (p / "cmdline").read_bytes()):
                    found.append(int(p.name))
            except OSError:
                pass
    return found


def run_pids(namespace: str, proc: pathlib.Path = pathlib.Path("/proc")) -> list[int]:
    """The run chain's long-lived processes on `namespace` (its controller
    manager and robot_state_publisher)."""
    remap = f"__ns:={namespace}".encode()
    return own_pids(
        lambda exe, cmdline: (
            os.path.basename(exe) in ("ros2_control_node", "robot_state_publisher")
            and remap in cmdline.split(b"\0")
        ),
        proc,
    )


def refusal(check: dict, model_file: str, *, unapproved: bool) -> str | None:
    """Why run must not start, or None — with check's reasons when the file
    claims an approval that no longer holds."""
    if check["approval_effective"] != "approved" and not unapproved:
        return (
            f"{model_file} is not approved for motion (approval: "
            f"{check['approval_effective']}). Bench it at the drives "
            f"(`plem drive run {shlex.quote(model_file)}` on a terminal offers the bench, which "
            "records the approval in the file), or run with --unapproved with an "
            "operator at the drives."
        ) + "".join(
            f"\n  the approval it claims no longer holds: {reason}"
            for reason in check.get("reasons", ())
        )
    return None


def robot_description(
    check: dict,
    model_file: str,
    *,
    master: int = 0,
    unapproved: bool = False,
    rt_cpu: int | None = None,
) -> str:
    """A URDF of the bus: world, one joint per axis, the drive system interface."""
    robot = ET.Element("robot", name="drive_bus")
    ET.SubElement(robot, "link", name="world")
    hw = ET.Element("ros2_control", name="drive_bus", type="system")
    hardware = ET.SubElement(hw, "hardware")
    ET.SubElement(hardware, "plugin").text = "plem_drive/DriveSystemInterface"
    ET.SubElement(hardware, "param", name="model_file").text = model_file
    ET.SubElement(hardware, "param", name="master_index").text = str(master)
    # For a client's release: under bounded_auto_reset the bus resets a fault itself.
    ET.SubElement(hardware, "param", name="fault_reset_policy").text = check[
        "fault_reset_policy"
    ]
    if unapproved:
        ET.SubElement(hardware, "param", name="unapproved").text = "true"
    if rt_cpu is not None:
        ET.SubElement(hardware, "param", name="rt_cpu").text = str(rt_cpu)
    for axis in check["axes"]:
        name = axis["name"]
        endless = axis["position_min_rad"] is None
        ET.SubElement(robot, "link", name=f"{name}_link")
        joint = ET.SubElement(
            robot, "joint", name=name, type="continuous" if endless else "revolute"
        )
        ET.SubElement(joint, "parent", link="world")
        ET.SubElement(joint, "child", link=f"{name}_link")
        ET.SubElement(joint, "axis", xyz="0 0 1")
        # The planner's velocity: the limit less a cycle's count rounding (the loop's step check).
        limit = {"velocity": str(axis["velocity_cap_rad_s"]), "effort": "0"}
        if not endless:
            limit.update(
                lower=str(axis["position_min_rad"]), upper=str(axis["position_max_rad"])
            )
        ET.SubElement(joint, "limit", limit)
        hw_joint = ET.SubElement(hw, "joint", name=name)
        for interface in _COMMAND[axis["mode"]]:
            ET.SubElement(hw_joint, "command_interface", name=interface)
        for interface in ("position", "velocity", "effort"):
            ET.SubElement(hw_joint, "state_interface", name=interface)
        if axis["acceleration_limit_rad_s2"] is not None:
            ET.SubElement(hw_joint, "param", name="acceleration_rad_s2").text = str(
                axis["acceleration_limit_rad_s2"]
            )
        if axis["mode"] == "cst":  # its torque clamp, for a stream client's checks
            for key in ("torque_min_nm", "torque_max_nm"):
                ET.SubElement(hw_joint, "param", name=key).text = str(axis[key])
    robot.append(hw)
    return ET.tostring(robot, encoding="unicode")


# joint_states serves observers (Bus, MoveIt, rviz) at 100 Hz; rt_raw carries every
# cycle. Publishing joint_states at the 1 kHz cycle rate starved Python clients'
# stream loops of the GIL on a loaded host.
JOINT_STATES_HZ = 100


def lanes(check: dict) -> list[str]:
    """The controllers run spawns, in order."""
    modes = {axis["mode"] for axis in check["axes"]}
    out = ["joint_state_broadcaster"]
    if "csp" in modes:
        out.append(TRAJECTORY_CONTROLLER)
    if modes & {"csv", "cst"}:
        out.append(STREAM_CONTROLLER)
    return out


def controller_params(
    check: dict, *, cpus, isolated: set[int], rt_cpu: int | None = None
) -> dict:
    """The controller manager's parameters file, as a dict."""
    cm = {
        "update_rate": round(1e9 / check["cycle_time_ns"]),
        "lock_memory": True,
        "joint_state_broadcaster": {
            "type": "joint_state_broadcaster/JointStateBroadcaster"
        },
    }
    # The bus loop runs on an isolated CPU (or rt_cpu); the manager's update thread
    # stays off them (when there is somewhere else to run).
    rt = isolated | ({rt_cpu} if rt_cpu is not None else set())
    housekeeping = sorted(cpu for cpu in cpus if cpu not in rt)
    if rt and housekeeping:
        cm["cpu_affinity"] = housekeeping
    params = {
        "controller_manager": {"ros__parameters": cm},
        "joint_state_broadcaster": {
            "ros__parameters": {"update_rate": min(JOINT_STATES_HZ, cm["update_rate"])}
        },
    }
    position = [a["name"] for a in check["axes"] if a["mode"] == "csp"]
    stream = [a for a in check["axes"] if a["mode"] != "csp"]
    if position:
        cm[TRAJECTORY_CONTROLLER] = {
            "type": "joint_trajectory_controller/JointTrajectoryController"
        }
        params[TRAJECTORY_CONTROLLER] = {
            "ros__parameters": {
                "joints": position,
                "command_interfaces": ["position", "velocity"],
                "state_interfaces": ["position", "velocity"],
                # The plan is the command (no feedback correction). A cancelled goal
                # holds the measured position at once — stop a moving bus with
                # protective_stop (Bus.stop), which decelerates first.
                "open_loop_control": True,
                "allow_nonzero_velocity_at_trajectory_end": False,
                # On activation (each release) it holds the measured position, because a
                # stop brings the axes to rest past its last command.
                "set_last_command_interface_value_as_state_on_activation": False,
            }
        }
    if stream:
        cm[STREAM_CONTROLLER] = {"type": "plem_drive/StreamController"}
        params[STREAM_CONTROLLER] = {
            "ros__parameters": {
                "joints": [a["name"] for a in stream],
                "interface_names": [_COMMAND[a["mode"]][0] for a in stream],
            }
        }
    return params


def namespaced(params: dict, ns: str) -> dict:
    """A parameters file for nodes in `ns`: each node's key fully qualified (a
    bare node name matches that node only outside any namespace)."""
    return {f"{ns}/{node}": values for node, values in params.items()}


# The level names the bus's logger (spdlog) takes from PLEM_LOG_LEVEL.
LOG_LEVELS = (
    "trace",
    "debug",
    "info",
    "warning",
    "warn",
    "error",
    "err",
    "critical",
    "off",
)


def log_level_warning(value: str | None) -> str | None:
    """None for an unset, empty or known PLEM_LOG_LEVEL; else what to tell the operator."""
    if not value or value in LOG_LEVELS:
        return None
    return (
        f"PLEM_LOG_LEVEL={value!r} is not a level ({', '.join(LOG_LEVELS)}); "
        "ignored, the default applies (warn, or info with --verbose)"
    )


def host_warnings(
    *, realtime: bool, isolated: set[int], tsc_lock: bool | None
) -> list[str]:
    """What on this host can upset the bus timing — each with the fix."""
    out = []
    if not realtime:
        out.append(
            "the kernel is not PREEMPT_RT — the bus loop can miss its cycle; "
            "boot the RT kernel: menu 1 of the RT setup wizard (setup/rt/setup.sh in "
            "https://github.com/WIM-Corporation/PLEM, docs/install.md), then reboot"
        )
    if not isolated:
        out.append(
            "no isolated CPU — the bus loop shares its CPU with everything else; "
            "isolate one: menu 1 of the same setup wizard sets "
            "isolcpus=managed_irq,domain,<cpu>; reboot after it"
        )
    if tsc_lock:
        out.append(
            "the nvpps PTP-TSC lock is on — it slews the host clock and the drives' DC "
            "drift follows (hundreds of µs); run sudo plem-nvpps-tsc-unlock and reboot "
            "(menu 1 of the setup wizard installs it; it sets ptp_tsc_sync_dis, and its "
            "last line says why if the lock stays)"
        )
    return out


def _tsc_lock(nvpps: pathlib.Path) -> bool | None:
    """The nvpps PTP-TSC lock from the device tree: None without an enabled nvpps node."""
    status = nvpps / "status"
    if not nvpps.is_dir() or (
        status.exists() and status.read_bytes().rstrip(b"\0") not in (b"okay", b"ok")
    ):
        return None
    return not (nvpps / "ptp_tsc_sync_dis").exists()


def _isolated_cpus() -> set[int]:
    text = pathlib.Path("/sys/devices/system/cpu/isolated").read_text().strip()
    cpus = set()
    for part in filter(None, text.split(",")):
        lo, _, hi = part.partition("-")
        cpus.update(range(int(lo), int(hi or lo) + 1))
    return cpus


def _this_host() -> dict:
    realtime = (
        pathlib.Path("/sys/kernel/realtime").exists()
        or "PREEMPT_RT" in platform.version()
    )
    tsc_lock = _tsc_lock(_NVPPS)
    return {"realtime": realtime, "isolated": _isolated_cpus(), "tsc_lock": tsc_lock}


# `plem drive check --json`'s stdout contract; the bench reads the same document.
CHECK_FORMAT = "plem-drive-check/1"
CHECK_MISMATCH = (
    "plem drive check printed another format — the plem packages are from different releases; "
    "install matching versions"
)


def _check(model_file: str) -> dict:
    from ament_index_python.packages import PackageNotFoundError

    try:
        tool = drive_model_tool()
        done = subprocess.run(
            [tool, "check", "--json", model_file], capture_output=True, text=True
        )
    except PackageNotFoundError:
        raise RuntimeError(
            "package plem_drive not found — source the setup.bash of the install that has plem_drive"
        ) from None
    except OSError as error:
        raise RuntimeError(
            f"cannot run the drive model tool {tool} ({error.strerror or error})"
            " — rebuild or reinstall plem_drive"
        ) from None
    # Exit 1 with the JSON is an approval claim this software does not honor:
    # the file reads as unapproved, and the JSON carries the reasons.
    if done.returncode != 0 and not done.stdout.strip():
        raise RuntimeError(
            done.stderr.strip() or f"plem drive check {model_file} failed"
        )
    try:
        check = json.loads(done.stdout)
    except ValueError:
        raise RuntimeError(CHECK_MISMATCH) from None
    if (
        not isinstance(check, dict)
        or check.get("format") != CHECK_FORMAT
        or "bus_name" not in check
    ):
        raise RuntimeError(CHECK_MISMATCH)
    return check


def _yaml(params: dict) -> str:
    import yaml

    return yaml.safe_dump(params, sort_keys=False)


def main(argv: list[str]) -> int:
    # An argument error shows the synopsis once; -h prints all of USAGE.
    synopsis = USAGE.split("\n\n", 1)[0].removeprefix("usage: ")
    parser = argparse.ArgumentParser(
        prog="plem drive run", usage=synopsis, add_help=False
    )
    parser.add_argument("model_file")
    parser.add_argument("--master", type=int, default=0)
    parser.add_argument("--rt-cpu", type=int)
    parser.add_argument("--monitor", action="store_true")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--unapproved", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    if any(a in ("-h", "--help") for a in argv):
        sys.stdout.write(USAGE)
        return 0
    args = parser.parse_args(argv)
    if args.rt_cpu is not None and args.rt_cpu not in os.sched_getaffinity(0):
        sys.stderr.write(f"plem drive run: {RT_CPU_REFUSAL.format(args.rt_cpu)}\n")
        return 1
    # Every --plot refusal comes before anything starts.
    plotjuggler = None
    if args.plot and (plotjuggler := _plotjuggler()) is None:
        distro = os.environ.get("ROS_DISTRO", "jazzy")
        sys.stderr.write(f"plem drive run: {PLOTJUGGLER_MISSING.format(distro)}\n")
        return 1
    model_file = os.path.abspath(args.model_file)
    try:
        check = _check(model_file)
        ns = os.environ.get(BENCH_NAMESPACE_ENV) or "/" + check["bus_name"]
        # This host only: a bus of the same name on another host of the ROS
        # domain, or another user's, is not seen.
        if pids := run_pids(ns):
            raise RuntimeError(
                f"a bus named {ns} is already up (pid {', '.join(map(str, pids))}) — "
                f"stop it (Ctrl-C in its terminal), or give this file another bus.name"
            )
        rt_cpu = ["--rt-cpu", str(args.rt_cpu)] if args.rt_cpu is not None else []
        bench_args = [model_file, "--master", str(args.master), *rt_cpu]
        # One question: the bench's own confirmation, which states its conditions.
        offer = None
        if (
            refusal(check, model_file, unapproved=args.unapproved)
            and sys.stdin.isatty()
        ):
            offer = (
                f"{model_file} is not approved for motion. Starting the drive bench."
            )
        elif check["approval_effective"] == "approved" and not args.unapproved:
            probed, reason = _probe(model_file, args.master)
            # The bus's reason says how to re-bench; name the file in it.
            reason = reason.replace("<file>", shlex.quote(model_file))
            if probed == PROBE_REBENCH and sys.stdin.isatty():
                offer = (
                    f"{model_file}: the drives on the bus differ from its approval record"
                    f". Starting the drive bench:\n  {reason}"
                )
            elif probed == PROBE_REBENCH:
                raise RuntimeError(
                    f"{model_file}: the drives on the bus differ from its approval record:\n  {reason}\n"
                    "  Or run with --unapproved with an operator at the drives."
                )
            elif probed != 0:
                raise RuntimeError(
                    reason
                    or f"comparing the drives with {model_file} failed (exit {probed})"
                )
        if offer:
            print(offer, flush=True)
            if _bench(bench_args) != 0:
                return 1  # the bench said why
            check = _check(model_file)
    except (ValueError, RuntimeError) as error:
        sys.stderr.write(f"plem drive run: {error}\n")
        return 1
    if error := refusal(check, model_file, unapproved=args.unapproved):
        sys.stderr.write(f"plem drive run: {error}\n")
        return 1
    host = _this_host()
    for warning in host_warnings(**host):
        sys.stderr.write(f"plem drive run: warning: {warning}\n")
    if warning := log_level_warning(os.environ.get("PLEM_LOG_LEVEL")):
        sys.stderr.write(f"plem drive run: warning: {warning}\n")
        del os.environ["PLEM_LOG_LEVEL"]  # said once here, not again by the bus

    workdir = _work_dir(ns)
    description = robot_description(
        check,
        model_file,
        master=args.master,
        unapproved=args.unapproved,
        rt_cpu=args.rt_cpu,
    )
    params = controller_params(
        check,
        cpus=os.sched_getaffinity(0),
        isolated=host["isolated"],
        rt_cpu=args.rt_cpu,
    )
    params["robot_state_publisher"] = {
        "ros__parameters": {"robot_description": description}
    }
    params_file = workdir / "params.yaml"
    params_file.write_text(_yaml(namespaced(params, ns)))
    ros_args = ["--ros-args", "-r", f"__ns:={ns}", "--params-file", str(params_file)]
    quiet, own = [], []
    plot_log = None
    if not args.verbose:
        os.environ.setdefault("PLEM_LOG_LEVEL", "warn")
        quiet = QUIET
        own = [*QUIET, "--log-level", f"{ns[1:].replace('/', '.')}.plem_drive:=info"]
        plot_log = workdir / "plotjuggler.log"
    print(start_summary(check, model_file, args, plot_log), flush=True)
    _spawn_child(
        [
            _executable("robot_state_publisher", "robot_state_publisher"),
            *ros_args,
            *quiet,
        ]
    )
    for controller in lanes(check):
        _spawn_child(
            [
                _executable("controller_manager", "spawner"),
                controller,
                "-c",
                f"{ns}/controller_manager",
                # The manager answers once its hardware is up — every slave in OP,
                # which the bus waits up to 60 s for: wait quietly past that.
                "--controller-manager-timeout",
                "90",
            ]
        )
    if args.monitor:  # its lines in this terminal; Ctrl-C ends it with its totals
        _spawn_child([sys.executable, "-m", "plem.drive.cli.monitor", ns])
    _start_fleet_session(ns, check)
    if args.plot:
        layout = workdir / "plotjuggler.xml"
        layout.write_text(plot_layout(ns[1:], [a["name"] for a in check["axes"]]))
        relay = _executable("plem_drive", "rt_raw_relay")
        _spawn_child([relay, "--ros-args", "-r", f"__ns:={ns}", *quiet])
        _spawn_child([plotjuggler, "--layout", str(layout)], output=plot_log)
    # This process becomes the controller manager: Ctrl-C reaches it directly.
    node = _executable("controller_manager", "ros2_control_node")
    os.execv(node, [node, ros_args[0], *own, *DESCRIPTION_FROM_TOPIC, *ros_args[1:]])
    return 1  # not reached


# drive_model_tool probe: 0 the drives match the approval record, 3 only an
# identity value differs (a bench at these drives replaces the record), 1 refused.
PROBE_REBENCH = 3


def _probe(model_file: str, master: int) -> tuple[int, str]:
    """The bus's start-up check of the drives against the file's approval
    record, stopped there (nothing configured or started): (exit, reason).
    plem's logging is off: stderr is then the bus's refusal alone."""
    done = subprocess.run(
        [drive_model_tool(), "probe", model_file, "--master", str(master)],
        capture_output=True,
        text=True,
        env={**os.environ, "PLEM_LOG_LEVEL": "off"},
    )
    return done.returncode, done.stderr.strip()


def _bench(argv: list[str]) -> int:
    """The drive bench, in this process: 0 = the file is approved now."""
    from plem.drive.cli.bench import main as bench

    return bench(argv)


def _plotjuggler() -> str | None:
    """PlotJuggler's executable, or None when it or its ROS 2 streamer
    (plotjuggler_ros, which the layout subscribes with) is not installed."""
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        path = _executable("plotjuggler", "plotjuggler")
        get_package_prefix("plotjuggler_ros")
    except PackageNotFoundError:
        return None
    return path if os.access(path, os.X_OK) else None


def _executable(package: str, name: str) -> str:
    from ament_index_python.packages import get_package_prefix

    return os.path.join(get_package_prefix(package), "lib", package, name)


def _work_dir(ns: str) -> pathlib.Path:
    """Private to this user (0700): the files name the hardware plugin the bus
    loads. This process becomes the controller manager and cannot remove the
    directory, so each bus reuses one in the user's own runtime directory (the
    files this run writes replace the last run's). A runtime directory that is
    missing, or not this user's alone, gets a fresh temporary one instead."""
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    try:
        st = os.stat(runtime) if runtime else None
    except OSError:
        st = None
    if st is None or st.st_uid != os.getuid() or st.st_mode & 0o077:
        return pathlib.Path(tempfile.mkdtemp(prefix="plem-drive-run-"))
    workdir = pathlib.Path(runtime) / f"plem-drive-run-{ns[1:]}"
    workdir.mkdir(mode=0o700, exist_ok=True)
    return workdir


def drive_model_tool() -> str:
    """plem_drive's C++ drive model tool: `check`, the bench's `approve` and run's `probe`."""
    return _executable("plem_drive", "drive_model_tool")


def plem_drive_command() -> str:
    """This package's `plem-drive` command (in bin/), which the bench starts its run with."""
    from ament_index_python.packages import get_package_prefix

    return os.path.join(get_package_prefix("plem_drive"), "bin", "plem-drive")


def _start_fleet_session(ns: str, check: dict) -> None:
    """Report this session if the operator agreed (docs/telemetry.md). Reporting
    must never stop a bus, so a problem here means this session is not reported."""
    try:
        from plem.drive import fleet
        from plem.drive.cli.fleet_session import session_context

        if fleet.active_state() is None:
            return
        context = session_context(check)
        _spawn_child(
            [
                sys.executable,
                "-m",
                "plem.drive.cli.fleet_session",
                ns,
                json.dumps(context),
            ],
            output=pathlib.Path(os.devnull),
        )
    except Exception:  # noqa: BLE001 — whatever went wrong, the bus still starts
        return


def _spawn_child(argv: list[str], output: pathlib.Path | None = None) -> None:
    """Start a child that gets SIGTERM when this process (the controller manager
    after exec) ends: setpriv sets the death signal, then a shell execs `argv`
    only if its parent is still this process; a parent that died before the
    signal was set has made it the child of another, and the shell exits.
    `output`: a file for its stdout and stderr, else this terminal."""
    sink = open(output, "w") if output else None  # the child keeps it open
    alive = 'test "$PPID" = "$0" && exec "$@"'
    subprocess.Popen(
        ["setpriv", "--pdeathsig", "TERM", "sh", "-c", alive, str(os.getpid()), *argv],
        stdout=sink,
        stderr=sink,
    )


def start_summary(check: dict, model_file: str, args, plot_log) -> str:
    """What run is about to start, in a few lines."""
    axes = check["axes"]
    cycle_ms = check["cycle_time_ns"] / 1e6
    lines = [
        f"{check['bus_name']}: {len(axes)} {'axis' if len(axes) == 1 else 'axes'}, {cycle_ms:g} ms cycle, "
        f"master {args.master}, "
        f"{'bench mode, its approval record unused' if args.unapproved else check['approval_effective']}"
        f" ({model_file})"
    ]
    width = max(len(a["name"]) for a in axes)
    lines += [f"  {a['name']:<{width}}  {a['mode']}" for a in axes]
    if args.plot:
        lines.append(
            "  plots: PlotJuggler" + (f", its output in {plot_log}" if plot_log else "")
        )
    lines.append(
        "  Ctrl-C stops the axes, then shuts the bus down."
        + ("" if args.verbose else " Add --verbose to see every node's log.")
    )
    return "\n".join(lines)
