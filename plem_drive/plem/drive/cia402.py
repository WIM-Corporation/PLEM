# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The CiA 402 statusword in words, without ROS: the one decoder the bench,
the client and the monitor share."""

# (mask, value, name), in the order a statusword is matched.
STATES = (
    (0x4F, 0x08, "FAULT"),
    (0x4F, 0x0F, "FAULT_REACTION_ACTIVE"),
    (0x4F, 0x40, "SWITCH_ON_DISABLED"),
    (0x4F, 0x00, "NOT_READY_TO_SWITCH_ON"),
    (0x6F, 0x21, "READY_TO_SWITCH_ON"),
    (0x6F, 0x23, "SWITCHED_ON"),
    (0x6F, 0x27, "OPERATION_ENABLED"),
    (0x6F, 0x07, "QUICK_STOP_ACTIVE"),
)
# Brake-closed rest: SWITCH_ON_DISABLED under the node's pre-release
# controlword 0x0000 (a standard drive), READY_TO_SWITCH_ON on some families
# and after an engage (early settle). Never NOT_READY_TO_SWITCH_ON: a
# statusword of 0 (no feedback yet) decodes as NOT_READY_TO_SWITCH_ON.
RESTING = ("SWITCH_ON_DISABLED", "READY_TO_SWITCH_ON")


def state(sw: int) -> str:
    return next(
        (name for mask, value, name in STATES if sw & mask == value), f"0x{sw:04x}"
    )


def fault_bit(sw: int) -> bool:
    """Statusword bit 3, Fault: set in FAULT and in FAULT_REACTION_ACTIVE."""
    return bool(sw & 0x0008)
