# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The installed plem.drive is what Python finds once the install is sourced.

The other tests import from this source tree; this one runs a fresh
interpreter outside it, so a plem.drive that installs off PYTHONPATH (the
CMake hook that puts it there) fails here.
"""

import os
import subprocess
import sys


def test_python_finds_the_installed_plem_drive():
    # Paths as Python reports them, not resolved: a --symlink-install points
    # the installed files back at this tree, and that is still the install.
    prefix = os.path.abspath(os.environ["PLEM_DRIVE_PREFIX"])
    probe = "import plem.drive, plem.drive.cli; print(plem.drive.__file__); print(plem.drive.cli.__file__)"
    done = subprocess.run(
        [sys.executable, "-c", probe],
        capture_output=True,
        text=True,
        cwd="/",
        check=False,
    )
    assert done.returncode == 0, done.stderr
    for found in done.stdout.split():
        assert os.path.abspath(found).startswith(prefix + os.sep), (found, prefix)
