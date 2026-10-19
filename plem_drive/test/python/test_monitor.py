# SPDX-FileCopyrightText: 2025-2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""The drive monitor — the ROS-free fold of rt_raw batches (no ROS context)."""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from plem.drive.cli.monitor import DC_NO_VALUE, Fold  # noqa: E402


def batch(seq, jitter=None, exec_us=None, miss=None, wkc=None, dc=None):
    n = len(seq)
    return SimpleNamespace(
        sequence=list(seq),
        loop_jitter_us=list(jitter if jitter is not None else [0.0] * n),
        loop_exec_us=list(exec_us if exec_us is not None else [10.0] * n),
        deadline_miss=list(miss if miss is not None else [0] * n),
        wkc_mismatch=list(wkc if wkc is not None else [0] * n),
        dc_sync_monitor_ns=list(dc if dc is not None else [0] * n),
    )


def test_a_sequence_gap_is_loss_across_batches_and_a_restart_is_not():
    f = Fold()
    f.fold(batch([0, 1, 2]))
    f.fold(batch([5, 6]))  # 3 and 4 never arrived
    f.fold(batch([0, 1]))  # the node restarted: a new session, not loss
    f.fold(batch([3]))  # 2 lost in the new session
    assert f.samples == 8
    assert f.lost == 3


def test_the_dc_sentinel_is_skipped_and_zero_is_a_value():
    f = Fold()
    f.fold(batch([0, 1, 2], dc=[10, DC_NO_VALUE, 20]))
    assert f.dc_max_ns == 20
    g = Fold()
    g.fold(batch([0], dc=[DC_NO_VALUE]))
    assert g.dc_max_ns is None
    h = Fold()
    h.fold(batch([0], dc=[0]))
    assert h.dc_max_ns == 0


def test_period_deviation_is_absolute_and_percentiles_report_the_bin_upper_edge():
    f = Fold()
    # 999 samples at 1.0 us and one at -5.0 us: p99.9 (rank 999) is the 1.0 bin,
    # reported as its upper edge 1.1; the max stays exact.
    f.fold(batch(range(1000), jitter=[1.0] * 999 + [-5.0]))
    assert f.dev_max_us == pytest.approx(5.0)
    assert f.dev_percentile(0.999) == pytest.approx(1.1)
    assert f.dev_percentile(1.0) == pytest.approx(
        5.0
    )  # the edge never exceeds the exact max
    assert f.dev_percentile(0.99) == pytest.approx(1.1)


def test_a_value_on_a_bin_boundary_lands_in_the_upper_bin():
    f = Fold()
    f.fold(batch([0], jitter=[0.3]))
    assert f.dev_percentile(0.5) == pytest.approx(
        0.3
    )  # capped by the max, bin 3 = [0.3, 0.4)
    g = Fold()
    g.fold(batch([0, 1], jitter=[0.3, 0.9]))
    assert g.dev_percentile(0.5) == pytest.approx(0.4)


def test_a_deviation_beyond_the_histogram_reports_the_exact_max():
    f = Fold()
    f.fold(batch([0, 1], jitter=[1.0, 1500.0]))
    assert f.dev_percentile(0.999) == pytest.approx(1500.0)


def test_counters_and_exec_time_add_up():
    f = Fold()
    f.fold(batch([0, 1, 2], exec_us=[10.0, 30.0, 20.0], miss=[0, 1, 0], wkc=[1, 0, 1]))
    assert (f.overruns, f.wkc_errors) == (1, 2)
    assert f.exec_max_us == pytest.approx(30.0)
    assert f.exec_mean_us == pytest.approx(20.0)


def test_an_empty_window_says_so():
    f = Fold()
    assert f.samples == 0
    assert f.dev_percentile(0.99) is None
    assert f.exec_mean_us is None
    assert "no samples" in f.row("15:04:48", "released")
    assert "no samples" in f.summary(0.0)


def test_the_summary_carries_the_numbers():
    f = Fold()
    f.fold(batch([0, 1, 3], jitter=[0.5, -1.2, 0.8], miss=[0, 0, 1], wkc=[0, 1, 0]))
    text = f.summary(12.5)
    for part in (
        "12.5 s",
        "samples 3",
        "lost 1",
        "overruns 1",
        "wkc_errors 1",
        "period_dev max 1.2",
    ):
        assert part in text, (part, text)


def test_a_gap_across_a_window_reset_counts_in_the_new_window():
    first = Fold()
    first.fold(batch([0, 1]))
    second = Fold()
    second.carry(first)
    second.fold(batch([4, 5]))
    assert second.lost == 2


def test_every_number_sits_right_under_its_column_name():
    from plem.drive.cli.monitor import HEADER

    f = Fold()
    f.fold(batch([0, 1, 3], jitter=[0.5, -1.2, 0.8], miss=[0, 0, 1], wkc=[0, 1, 0]))
    row = f.row("15:04:48", "released")
    for label in (
        "exec-mean[us]",
        "exec-max[us]",
        "pdev-p99[us]",
        "pdev-max[us]",
        "overrun/s",
        "wkc-err/s",
        "lost/s",
        "dc-max[ns]",
    ):
        end = HEADER.index(label) + len(label)
        assert row[end - 1] != " " and row[end] == " ", (label, HEADER, row)
    assert row.startswith("15:04:48") and row.endswith("released")


def status(sw, oe=False, pstop=0, brakes=0b11, pending=0):
    return SimpleNamespace(
        statusword=sw,
        operation_enabled=oe,
        protective_stop=pstop,
        brake_engaged=brakes,
        brake_operation_pending=pending,
    )


@pytest.mark.parametrize(
    "msg, said, alarm",
    [
        (None, "no status yet", False),
        (status([0, 0]), "every statusword 0 (drives not reporting yet)", False),
        (status([0x0231, 0x0238]), "axis1 fault (statusword 0x0238)", True),
        (
            status([0x0231, 0x0231], pstop=1),
            "protective stop INTERNAL (a limit still broken)",
            True,
        ),
        (status([0x1237, 0x1237], oe=True, brakes=0), "released", False),
        (status([0x0231, 0x0231], pending=0b01), "brake operation pending", False),
        (status([0x0231, 0x0231]), "held, brakes on", False),
        # An engage that ended unconfirmed: disabled, nothing pending, bit 1 still 0.
        (status([0x0231, 0x0231], brakes=0b01), "axis1 brake not confirmed on", True),
    ],
)
def test_the_state_is_said_in_words(msg, said, alarm):
    from plem.drive.cli.monitor import state_text

    assert state_text(msg) == (said, alarm)


def test_a_fault_names_the_drive_error_code_the_bus_reported():
    from plem.drive.cli.monitor import state_text

    # rt_events' SERVO_FAULT carried 0xFF50 for axis 1; 0 = no error-code PDO mapped.
    said, alarm = state_text(status([0x0238, 0x0238]), {0: 0, 1: 0xFF50})
    assert (
        said
        == "axis0 fault (statusword 0x0238), axis1 fault (statusword 0x0238, error 0xff50)"
    )
    assert alarm


def test_the_bus_name_is_required(monkeypatch, capsys):
    from plem.drive.cli import monitor

    # Past the parser it would import rclpy: here that import fails.
    monkeypatch.setitem(sys.modules, "rclpy", None)
    with pytest.raises(SystemExit) as exc:
        monitor.main([])
    err = capsys.readouterr().err
    assert exc.value.code == 2 and "bus" in err and "--ns" not in err
    assert err.startswith("usage: python3 -m plem.drive.cli.monitor "), err


def test_a_cleared_fault_forgets_its_code_so_a_new_fault_never_shows_the_old_one():
    from plem.drive.bus import forget_cleared

    codes = {0: 0xFF50, 1: 0x7500}
    forget_cleared(codes, [0x0250, 0x0238])  # axis0 reset, axis1 still in fault
    assert codes == {1: 0x7500}
