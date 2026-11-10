# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""Drives the wizard's consent screen through a real terminal, with plem-fleet stubbed.

Run by test_fleet_wizard.sh: fleet_consent_pty.py <setup.sh>. Exit code = failed cases.
"""

import os
import pty
import re
import select
import subprocess
import sys
import tempfile
import time

WIZ = sys.argv[1]
tmp = tempfile.mkdtemp()
wizard = open(WIZ).read().split("#  Entry point")[0]
open(f"{tmp}/wizard.sh", "w").write(wizard)
open(f"{tmp}/fleet.py", "w").write(
    f'import sys\nopen("{tmp}/calls","a").write(" ".join(sys.argv[1:])+"\\n")\nprint("NOTICE TEXT")\n'
)


def run(keys: list[bytes], typeahead: bytes = b"") -> list[str]:
    calls = f"{tmp}/calls"
    if os.path.exists(calls):
        os.unlink(calls)
    script = (
        f"trap - EXIT; source {tmp}/wizard.sh; trap - EXIT; FLEET_TOOL={tmp}/fleet.py; "
        # The guards look at /etc/plem, which a device under test may already have.
        + re.search(r"^fleet_consent\(\) \{.*?^\}", wizard, re.M | re.S)
        .group(0)
        .replace("! -e /etc/plem/fleet.json", f"! -e {tmp}/none")
        .replace("! -e /etc/plem/license.key", f"! -e {tmp}/none")
        + "\nsleep 0.5; fleet_consent"
    )
    pid, fd = pty.fork()
    if pid == 0:
        os.execvp("bash", ["bash", "-c", script])
    if typeahead:
        os.write(fd, typeahead)  # arrives before the screen asks
    output = b""
    deadline = time.time() + 12
    pending = list(keys)
    seen_menu = False
    last_send = 0.0
    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.2)
        if ready:
            try:
                data = os.read(fd, 65536)
            except OSError:
                break
            if not data:
                break
            output += data
        seen_menu = seen_menu or b"Enter select" in output
        if seen_menu and pending and time.time() - last_send > 1.0:
            os.write(fd, pending.pop(0))
            last_send = time.time()
    try:
        os.close(fd)
    except OSError:
        pass
    os.waitpid(pid, 0)
    return open(calls).read().split("\n")[:-1] if os.path.exists(calls) else []


cases = {
    "Enter on the focused Yes, skip email": ([b"\r", b"\r"], b""),
    "Down, Enter = No": ([b"\x1b[B", b"\r", b"\r"], b""),
    "q = No": ([b"q", b"\r"], b""),
    "View, then back, then Yes": (
        [b"\x1b[B", b"\x1b[B", b"\r", b"\r", b"\r", b"\r"],
        b"",
    ),
    "typed-ahead Enter does not answer": ([b"\x1b[B", b"\r", b"\r"], b"\r\r\r"),
    "an email is passed on": ([b"\r", b"me@example.invalid\r"], b""),
    "a second Enter after View does not answer": (
        [b"\x1b[B", b"\x1b[B", b"\r", b"\r\r", b"\x1b[B", b"\r", b"\r"],
        b"",
    ),
}
failed = 0
expected = {
    "Enter on the focused Yes, skip email": ["notice", "on"],
    "Down, Enter = No": ["notice", "decline"],
    "q = No": ["notice", "decline"],
    "View, then back, then Yes": ["notice", "show", "notice", "on"],
    "typed-ahead Enter does not answer": ["notice", "decline"],
    "an email is passed on": ["notice", "on", "contact me@example.invalid"],
    "a second Enter after View does not answer": [
        "notice",
        "show",
        "notice",
        "decline",
    ],
}
for name, (keys, ahead) in cases.items():
    got = run(keys, ahead)
    ok = got == expected[name]
    failed += not ok
    print(("PASS" if ok else "FAIL"), name, got)
subprocess.run(["rm", "-rf", tmp])
sys.exit(failed)
