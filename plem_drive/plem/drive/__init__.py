# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem.drive — move CiA 402 axes from Python.

The names below load on first use, so the `plem drive` command
(plem.drive.cli) starts without the ROS client that Bus needs.
"""

from __future__ import annotations

import importlib

_EXPORTS = {
    "Axis": "plem.drive.bus",
    "Bus": "plem.drive.bus",
    "DriveError": "plem.drive.bus",
    "Limits": "plem.drive.profile",
    "Profile": "plem.drive.profile",
    "Scale": "plem.drive.profile",
    "SCurve": "plem.drive.profile",
    "Trapezoid": "plem.drive.profile",
}

__all__ = list(_EXPORTS)


def __getattr__(name: str):
    if name not in _EXPORTS:
        raise AttributeError(f"module 'plem.drive' has no attribute {name!r}")
    value = getattr(importlib.import_module(_EXPORTS[name]), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted([*globals(), *_EXPORTS])
