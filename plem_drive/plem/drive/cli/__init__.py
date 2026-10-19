# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem-drive — the drive model file tool: one command, two verbs.

`check` execs the C++ tool of plem_drive (the loader's own checks); `run`
runs here — it offers the bench for a file not yet approved, and `--monitor`
prints the bus timing in the same terminal. Each verb's arguments pass through untouched,
and each verb explains itself with `--help`.
"""

from __future__ import annotations

import os
import sys

USAGE = """usage: plem drive <verb> [arguments]

  run <model.yaml>     start the bus under ros2_control; for a file not yet approved,
                       it first offers the drive bench (terminal only). Ctrl-C stops
                       the axes, then shuts the bus down
  check <model.yaml>   read a drive model file the way the bus does (no hardware);
                       --json prints the machine-readable result for scripts

`plem drive <verb> --help` explains a verb. Without the `plem` command, every
`plem drive <verb>` in these messages is `plem-drive <verb>`.
"""


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if not args:
        sys.stderr.write(USAGE)
        return 2
    verb, rest = args[0], args[1:]
    if verb in ("help", "-h", "--help"):
        sys.stdout.write(USAGE)
        return 0
    if verb == "check":
        from ament_index_python.packages import PackageNotFoundError

        from plem.drive.cli.run import drive_model_tool

        try:
            tool = drive_model_tool()
        except PackageNotFoundError:
            sys.stderr.write(
                "plem drive: package plem_drive not found — source the setup.bash"
                " of the install that has plem_drive\n"
            )
            return 1
        try:
            os.execv(tool, [tool, verb, *rest])
        except OSError as error:
            sys.stderr.write(
                f"plem drive: cannot run the drive model tool {tool}"
                f" ({error.strerror or error}) — rebuild or reinstall plem_drive\n"
            )
            return 1
    if verb == "run":
        from plem.drive.cli.run import main as run

        return run(rest)
    sys.stderr.write(f"plem drive: unknown verb '{verb}'\n\n{USAGE}")
    return 2
