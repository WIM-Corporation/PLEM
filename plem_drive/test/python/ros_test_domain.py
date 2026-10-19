# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0
"""The ROS tests' shared helpers: a domain of their own, off the user's live one,
and a bounded wait for what another thread or process makes true."""

import os
import time

# ROS 2 gives 0..101 as the domain IDs whose DDS ports stay out of Linux's
# ephemeral range; the tests take the top of it.
FIRST, LAST = 60, 101


def private_domain_id() -> str:
    """This process's domain (by pid) inside FIRST..LAST, as ROS_DOMAIN_ID text."""
    return str(FIRST + os.getpid() % (LAST - FIRST + 1))


def wait_until(predicate, seconds=5.0):
    """True once `predicate()` holds, polled every 20 ms; its last value at `seconds`."""
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.02)
    return predicate()
