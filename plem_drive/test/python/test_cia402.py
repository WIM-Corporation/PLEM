# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem.drive.cia402 against the C++ decoder (statusword_reference.cpp): every
16-bit statusword names the same CiA 402 state on both sides. A word the C++
side cannot place (UNKNOWN) is a hex string here."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from plem.drive.cia402 import state  # noqa: E402

REFERENCE = os.environ.get("PLEM_STATUSWORD_REFERENCE")


@pytest.mark.skipif(
    not REFERENCE, reason="PLEM_STATUSWORD_REFERENCE not set (run through colcon test)"
)
def test_every_statusword_decodes_to_the_same_state_as_the_cpp_decoder():
    out = subprocess.run([REFERENCE], check=True, capture_output=True, text=True).stdout
    table = dict(line.split(" ", 1) for line in out.splitlines())
    assert len(table) == 0x10000
    mismatches = []
    for word in range(0x10000):
        cpp = table[str(word)]
        expected = f"0x{word:04x}" if cpp == "UNKNOWN" else cpp
        if state(word) != expected:
            mismatches.append(f"0x{word:04x}: C++ {cpp}, Python {state(word)}")
    assert not mismatches, f"{len(mismatches)} words differ, first: {mismatches[:5]}"
