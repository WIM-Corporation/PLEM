# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem-drive — the dispatcher: help and usage codes, and which verb goes where
(check execs plem_drive's C++ tool, run runs in-process)."""

import os
import pathlib
import sys
import types

import pytest

# This source tree's package, so cli and the run module it imports come from the same tree.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from plem.drive import cli  # noqa: E402


def test_the_dispatcher_starts_without_the_ros_client():
    # `plem drive check` and `--help` work where rclpy cannot load: importing
    # the dispatcher imports neither rclpy nor plem.drive's Bus.
    import subprocess

    probe = "import sys, plem.drive.cli; print(sorted({'rclpy', 'plem.drive.bus'} & set(sys.modules)))"
    env = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join(
            [
                str(pathlib.Path(__file__).resolve().parents[2]),
                os.environ.get("PYTHONPATH", ""),
            ]
        ),
    }
    done = subprocess.run(
        [sys.executable, "-c", probe],
        capture_output=True,
        text=True,
        env=env,
        check=True,
    )
    assert done.stdout.strip() == "[]", done.stdout + done.stderr


def test_no_verb_is_usage_on_stderr_with_code_2(capsys):
    assert cli.main([]) == 2
    out, err = capsys.readouterr()
    assert out == "" and err.startswith("usage: plem drive <verb>")


@pytest.mark.parametrize("word", ["help", "-h", "--help"])
def test_help_is_usage_on_stdout_with_code_0(capsys, word):
    assert cli.main([word]) == 0
    out, err = capsys.readouterr()
    assert err == "" and out == cli.USAGE
    # The two verbs, in the order a user meets them.
    verbs = [
        line.split()[0]
        for line in out.splitlines()
        if line.startswith("  ") and line[2] != " "
    ]
    assert verbs == ["run", "check"]


@pytest.mark.parametrize("verb", ["verify", "bench", "monitor", "approve"])
def test_an_unknown_verb_is_named_with_code_2(capsys, verb):
    # bench is offered by run, monitor is run --monitor, approve is the bench's own step.
    assert cli.main([verb, "x.yaml"]) == 2
    out, err = capsys.readouterr()
    assert out == "" and err.startswith(
        f"plem drive: unknown verb '{verb}'\n\nusage: plem drive"
    )


class Exec(Exception):
    pass


def test_check_execs_the_cpp_tool_this_build_installed(monkeypatch):
    verb = "check"
    seen = []

    def execv(path, argv):
        seen.append((path, argv))
        raise Exec

    monkeypatch.setattr(cli.os, "execv", execv)
    with pytest.raises(Exec):
        cli.main([verb, "a.yaml", "b.yaml", "--help"])
    ((path, argv),) = seen
    assert os.path.samefile(path, os.environ["PLEM_DRIVE_BIN"])
    assert argv == [path, verb, "a.yaml", "b.yaml", "--help"]


def test_run_runs_in_process_with_the_rest_of_the_arguments(monkeypatch):
    verb = "run"
    got = []
    module = types.ModuleType(f"plem.drive.cli.{verb}")
    module.main = lambda argv: got.append(argv) or 7
    monkeypatch.setitem(sys.modules, f"plem.drive.cli.{verb}", module)
    assert cli.main([verb, "m.yaml", "--yes"]) == 7
    assert got == [["m.yaml", "--yes"]]


def test_check_without_plem_drive_says_so_with_code_1(monkeypatch, capsys):
    monkeypatch.setenv("AMENT_PREFIX_PATH", "/nonexistent")
    assert cli.main(["check", "a.yaml"]) == 1
    out, err = capsys.readouterr()
    assert out == "" and err.startswith("plem drive: package plem_drive not found")
    # Valid for a source build and a packaged install alike: no package name to install.
    assert (
        "source the setup.bash of the install that has plem_drive" in err
        and "plem-drive" not in err
    )


@pytest.mark.parametrize("error", [FileNotFoundError, PermissionError])
def test_an_unrunnable_executable_says_so_with_code_1(monkeypatch, capsys, error):
    def execv(path, argv):
        raise error(2, "refused", path)

    monkeypatch.setattr(cli.os, "execv", execv)
    assert cli.main(["check", "a.yaml"]) == 1
    out, err = capsys.readouterr()
    assert out == "" and err.startswith("plem drive: cannot run the drive model tool")
    assert err.rstrip().endswith("— rebuild or reinstall plem_drive"), err
