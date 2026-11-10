# SPDX-FileCopyrightText: 2026 WIM Inc.
# SPDX-License-Identifier: Apache-2.0

"""plem-fleet: what reaches the server, and when nothing may."""

import importlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

SERIAL = "SERIAL-A"


@pytest.fixture
def fleet(tmp_path, monkeypatch):
    monkeypatch.setenv("PLEM_FLEET_ROOT", str(tmp_path))
    (tmp_path / "proc/device-tree").mkdir(parents=True)
    (tmp_path / "proc/device-tree/serial-number").write_text(SERIAL + "\x00")
    import plem.drive.fleet as module

    yield importlib.reload(module)
    monkeypatch.delenv("PLEM_FLEET_ROOT")
    importlib.reload(module)


class Server:
    """Stands in for the server: answers each request from a script."""

    def __init__(self, fleet, monkeypatch):
        self.calls = []
        self.answer = lambda method, path, body: (
            (200, {}, json.dumps({"report_id": body["report_id"]}).encode())
            if method == "POST"
            else (204, {}, b"")
        )
        monkeypatch.setattr(fleet, "_http", self._http)

    def _http(self, method, path, body=None):
        self.calls.append((method, path, body))
        return self.answer(method, path, body)


@pytest.fixture
def server(fleet, monkeypatch):
    return Server(fleet, monkeypatch)


def spooled(fleet):
    return sorted(Path(fleet.SPOOL_DIR).glob("*.json"))


def test_nothing_is_active_before_a_decision(fleet):
    assert fleet.active_state() is None


def test_tick_without_consent_collects_and_sends_nothing(fleet, server):
    assert fleet.tick() == 0
    assert server.calls == []
    assert not Path(fleet.SPOOL_DIR).exists()


def test_declining_keeps_it_off(fleet, server):
    assert fleet.main(["decline"]) == 0
    assert fleet.active_state() is None
    fleet.tick()
    assert server.calls == []


def test_on_writes_state_and_installs_the_timer(fleet):
    assert fleet.turn_on() == 0
    state = fleet.active_state()
    assert state["serial"] == SERIAL
    assert state["notice_version"] == fleet.NOTICE_VERSION
    assert Path(fleet.TIMER_FILE).exists() and Path(fleet.SERVICE_FILE).exists()
    assert Path(fleet.INSTALLED_TOOL).stat().st_mode & 0o111


def test_a_cloned_image_does_not_report(fleet, server, tmp_path):
    fleet.turn_on()
    (tmp_path / "proc/device-tree/serial-number").write_text("SERIAL-B\x00")
    assert fleet.active_state() is None
    fleet.tick()
    assert server.calls == []


def test_a_licensed_device_does_not_use_this_path(fleet, tmp_path):
    fleet.turn_on()
    (tmp_path / "etc/plem/license.key").write_text("key")
    assert fleet.active_state() is None


def test_tick_sends_a_device_report_and_clears_it(fleet, server):
    fleet.turn_on()
    fleet.tick()
    [(method, path, body)] = server.calls
    assert (method, path) == ("POST", "/reports")
    assert body["kind"] == "device"
    assert body["reported_serial"] == SERIAL
    assert spooled(fleet) == []


@pytest.mark.parametrize(
    "answer",
    [
        (None, {}, b""),
        (500, {}, b""),
        (200, {}, b"<html>a web page</html>"),
    ],
)
def test_an_unproven_delivery_keeps_the_report(fleet, server, answer):
    fleet.turn_on()
    server.answer = lambda *_: answer
    fleet.tick()
    assert len(spooled(fleet)) == 1


def test_a_report_the_server_refuses_for_good_is_dropped(fleet, server):
    fleet.turn_on()
    server.answer = lambda *_: (422, {}, b"")
    fleet.tick()
    assert spooled(fleet) == []


def test_the_server_can_stop_collection_and_clear_the_queue(fleet, server):
    fleet.turn_on()
    server.answer = lambda *_: (403, {"X-Telemetry": "disabled"}, b"")
    fleet.tick()
    assert spooled(fleet) == []


def test_the_spool_is_bounded(fleet):
    fleet.turn_on()
    state = fleet.active_state()
    for _ in range(fleet.SPOOL_CAP + 5):
        fleet.spool(fleet.new_report(state, "device", {}))
    assert len(spooled(fleet)) == fleet.SPOOL_CAP


def test_off_stops_collection_and_tells_the_server(fleet, server):
    fleet.turn_on()
    install_id = fleet.active_state()["install_id"]
    fleet.spool(fleet.new_report(fleet.active_state(), "device", {}))
    assert fleet.turn_off() == 0
    assert ("DELETE", f"/installs/{install_id}", None) in server.calls
    assert fleet.active_state() is None
    assert spooled(fleet) == []
    assert not Path(fleet.TIMER_FILE).exists()
    assert fleet.load_state() == {"enabled": False}  # the identifier is gone too


def test_off_while_offline_stops_locally_and_can_be_finished_later(fleet, server):
    fleet.turn_on()
    install_id = fleet.active_state()["install_id"]
    server.answer = lambda *_: (None, {}, b"")
    assert fleet.turn_off() == 1
    assert fleet.active_state() is None
    assert fleet.load_state()["install_id"] == install_id

    server.answer = lambda *_: (204, {}, b"")
    assert fleet.turn_off() == 0
    assert fleet.load_state() == {"enabled": False}


def test_contact_needs_no_reporting_and_carries_no_serial(fleet, server):
    assert fleet.contact("operator@example.invalid") == 0
    [(_, _, body)] = server.calls
    assert body["kind"] == "contact"
    assert body["reported_serial"] is None
    assert body["payload"]["email"] == "operator@example.invalid"


def test_contact_is_linked_to_the_device_when_reporting_is_on(fleet, server):
    fleet.turn_on()
    fleet.contact("operator@example.invalid")
    assert server.calls[-1][2]["reported_serial"] == SERIAL


def test_install_report_holds_only_step_results(fleet, server):
    fleet.turn_on()
    fleet.report_install(
        ["1|apt update|ok\n", "2|RT kernel|pass\n", "cyclictest|12|3.5\n", "junk\n"]
    )
    payload = server.calls[-1][2]["payload"]
    assert payload["cyclictest"] == {"max_us": 12.0, "avg_us": 3.5}
    assert payload["steps"][0] == {"menu": "1", "step": "apt update", "result": "ok"}
    assert len(payload["steps"]) == 2


def test_device_report_leaves_out_what_identifies_people_or_networks(fleet, tmp_path):
    (tmp_path / "proc/cmdline").write_text(
        "root=PARTUUID=abc hostname=bench-host ip=203.0.113.9 isolcpus=11 irqaffinity=0-10\n"
    )
    (tmp_path / "etc").mkdir(exist_ok=True)
    (tmp_path / "etc/ethercat.conf").write_text(
        'MASTER0_DEVICE="02-00-00-00-00-01"\nDEVICE_MODULES="igb"\n'
    )
    payload = fleet.collect_device()
    assert payload["isolcpus"] == "11"
    assert payload["ethercat_backend"] == "igb"
    text = json.dumps(payload)
    for secret in ("bench-host", "203.0.113.9", "PARTUUID", "02-00-00"):
        assert secret not in text


def test_device_report_names_the_ethercat_driver_not_its_address(fleet, tmp_path):
    address = ":".join(["02", "00", "00", "00", "00", "01"])
    (tmp_path / "etc").mkdir(exist_ok=True)
    (tmp_path / "etc/ethercat.conf").write_text(
        f'MASTER0_DEVICE="{address.upper()}"\nDEVICE_MODULES="generic"\n'
    )
    drivers = tmp_path / "bus/drivers/nvethernet"
    drivers.mkdir(parents=True)
    device = tmp_path / "bus/devices/nic0"
    device.mkdir(parents=True)
    (device / "driver").symlink_to(drivers)
    for name, mac, target in (
        ("other0", "02-00-00-00-00-02".replace("-", ":"), None),
        ("end0", address, device),
    ):
        interface = tmp_path / "sys/class/net" / name
        interface.mkdir(parents=True)
        (interface / "address").write_text(mac + "\n")
        if target:
            (interface / "device").symlink_to(target)
    payload = fleet.collect_device()
    assert payload["ethercat_nic_driver"] == "nvethernet"
    assert address not in json.dumps(payload).lower()


def test_an_ethercat_address_on_no_interface_has_no_driver(fleet, tmp_path):
    (tmp_path / "etc").mkdir(exist_ok=True)
    (tmp_path / "etc/ethercat.conf").write_text('MASTER0_DEVICE="02-00-00-00-00-03"\n')
    assert fleet.collect_device()["ethercat_nic_driver"] is None


def test_device_report_has_storage_swap_and_the_l4t_hold(fleet, tmp_path, monkeypatch):
    nvme = tmp_path / "sys/class/nvme/nvme0/nvme0n1"
    nvme.mkdir(parents=True)
    (nvme / "size").write_text("2097152\n")  # 512-byte sectors
    mmc = tmp_path / "sys/block/mmcblk0/device"
    mmc.mkdir(parents=True)
    (mmc.parent / "size").write_text("4194304\n")
    (mmc / "name").write_text("G1M15M\n")
    (tmp_path / "proc/meminfo").write_text(
        "SwapTotal:     2048 kB\nSwapFree:      1024 kB\n"
    )
    real_run = fleet._run
    monkeypatch.setattr(
        fleet,
        "_run",
        lambda *argv: (
            "hold ok installed"
            if argv[:1] == ("dpkg-query",) and "-f=${Status}" in argv
            else real_run(*argv)
        ),
    )
    payload = fleet.collect_device()
    assert payload["nvme_size_mib"] == 1024
    assert payload["emmc_size_mib"] == 2048
    assert payload["emmc_model"] == "G1M15M"
    assert payload["swap_total_mib"] == 2
    assert payload["swap_free_mib"] == 1
    assert payload["l4t_core_held"] is True


def test_storage_and_hold_are_empty_where_the_device_has_none(fleet, monkeypatch):
    monkeypatch.setattr(fleet, "_run", lambda *argv: None)
    payload = fleet.collect_device()
    assert payload["nvme_size_mib"] is None
    assert payload["emmc_model"] is None
    assert payload["l4t_core_held"] is None


def test_device_report_fits_the_server_limits(fleet):
    def depth(value, level=0):
        if isinstance(value, dict):
            return max((depth(v, level + 1) for v in value.values()), default=level + 1)
        return level

    assert depth(fleet.collect_device()) <= 4


def test_a_panic_needs_a_dmesg_record_not_just_a_console_log(fleet, tmp_path):
    store = tmp_path / "var/lib/systemd/pstore"
    store.mkdir(parents=True)
    (store / "console-ramoops-0").write_text("routine")
    assert fleet.collect_device()["pstore_panic_seen"] is False
    (store / "dmesg-ramoops-0").write_text("panic")
    assert fleet.collect_device()["pstore_panic_seen"] is True


def test_a_dead_writers_session_is_adopted_and_a_live_ones_is_left(fleet):
    fleet.turn_on()
    work = Path(fleet.WORK_DIR)
    (work / "aaa-999999999.json").write_text("{}")
    live = work / "bbb-1.json"  # pid 1 is always alive
    live.write_text("{}")
    fleet.adopt_orphan_sessions()
    assert [p.name for p in spooled(fleet)] == ["aaa.json"]
    assert live.exists()


def test_turning_on_again_from_the_installed_tool_works(fleet, monkeypatch):
    fleet.turn_on()
    monkeypatch.setattr(fleet, "__file__", fleet.INSTALLED_TOOL)
    assert fleet.turn_on() == 0
    assert Path(fleet.TIMER_FILE).exists()


def test_a_report_made_before_turning_off_is_never_sent_afterwards(fleet, server):
    fleet.turn_on()
    state = fleet.active_state()
    fleet.turn_off()
    server.calls.clear()
    fleet.spool(fleet.new_report(state, "coe_session", {"run_s": 1}))
    assert fleet.send() == 0
    assert server.calls == []

    fleet.turn_on()  # a new install id: the old report still must not go out
    fleet.send()
    assert server.calls == []
    assert spooled(fleet) == []


def test_a_finished_session_beats_its_unfinished_copy(fleet):
    fleet.turn_on()
    work = Path(fleet.WORK_DIR)
    (work / "aaa-999999999.json").write_text('{"payload": {"complete": false}}')
    Path(fleet.SPOOL_DIR, "aaa.json").write_text('{"payload": {"complete": true}}')
    fleet.adopt_orphan_sessions()
    assert (
        json.loads(Path(fleet.SPOOL_DIR, "aaa.json").read_text())["payload"]["complete"]
        is True
    )
    assert list(work.glob("*.json")) == []
    assert len(spooled(fleet)) == 1


def test_command_output_is_narrowed_before_it_is_reported(fleet, monkeypatch):
    answers = {"/opt/etherlab/bin/ethercat": "IgH EtherCAT master 1.6.13 on host-a"}
    monkeypatch.setattr(fleet, "_run", lambda *argv: answers.get(argv[0]))
    payload = fleet.collect_device()
    assert payload["igh_version"] == "1.6.13"


def test_consent_can_be_rechecked_for_a_running_session(fleet):
    fleet.turn_on()
    state = fleet.active_state()
    assert fleet.is_current(state)
    Path(fleet.STATE_FILE).write_text('{"enabled": false}')
    assert not fleet.is_current(state)


def test_a_retired_install_is_not_retried(fleet, server):
    fleet.turn_on()
    fleet.spool(fleet.new_report(fleet.active_state(), "device", {}))
    server.answer = lambda *_: (410, {}, b"")
    fleet.send()
    assert spooled(fleet) == []


def test_nvpps_state_is_one_of_the_known_words(fleet, monkeypatch):
    class Done:
        stderr = "some path /opt/some/path\napplied\n"

    monkeypatch.setattr(fleet.subprocess, "run", lambda *a, **k: Done())
    assert fleet._nvpps_lock_state() == "applied"
    Done.stderr = "/opt/some/path\n"
    assert fleet._nvpps_lock_state() is None


def test_a_server_stop_holds_off_collection_until_the_retry_time(fleet, server):
    import os
    import time

    fleet.turn_on()
    server.answer = lambda *_: (403, {"X-Telemetry": "disabled"}, b"")
    fleet.tick()
    assert len(server.calls) == 1
    assert fleet.active_state() is None
    fleet.tick()
    assert len(server.calls) == 1
    assert spooled(fleet) == []

    old = time.time() - fleet.SERVER_STOP_RETRY_S - 1
    os.utime(fleet.SERVER_STOP_FILE, (old, old))
    assert fleet.active_state() is not None
    fleet.tick()
    assert len(server.calls) == 2


def test_turning_off_clears_a_server_stop(fleet, server):
    fleet.turn_on()
    server.answer = lambda *_: (403, {"X-Telemetry": "disabled"}, b"")
    fleet.tick()
    server.answer = lambda *_: (204, {}, b"")
    fleet.turn_off()
    assert not Path(fleet.SERVER_STOP_FILE).exists()


def test_an_email_given_without_reporting_can_be_removed(fleet, server):
    fleet.main(["decline"])
    fleet.contact("operator@example.invalid")
    install_id = fleet.load_state()["contact_install_id"]
    assert server.calls[-1][2]["install_id"] == install_id
    assert fleet.active_state() is None
    assert Path(
        fleet.INSTALLED_TOOL
    ).exists()  # the command the screen names must exist

    assert fleet.forget_email() == 0
    assert server.calls[-1] == ("DELETE", f"/installs/{install_id}/contact", None)
    assert fleet.load_state() == {"enabled": False}


def test_an_email_given_with_reporting_can_be_removed_and_reporting_stays_on(
    fleet, server
):
    fleet.turn_on()
    fleet.contact("operator@example.invalid")
    install_id = fleet.active_state()["install_id"]
    assert fleet.forget_email() == 0
    assert server.calls[-1][1] == f"/installs/{install_id}/contact"
    assert fleet.active_state() is not None


def test_removing_an_email_while_offline_can_be_finished_later(fleet, server):
    fleet.contact("operator@example.invalid")
    server.answer = lambda *_: (None, {}, b"")
    assert fleet.forget_email() == 1
    assert "contact_install_id" in fleet.load_state()


def test_removing_when_no_email_was_given_says_so_and_calls_nothing(fleet, server):
    assert fleet.forget_email() == 0
    assert server.calls == []


def test_turning_reporting_off_keeps_the_way_to_remove_a_separate_email(fleet, server):
    fleet.contact("operator@example.invalid")
    contact_id = fleet.load_state()["contact_install_id"]
    fleet.turn_on()
    assert fleet.load_state()["contact_install_id"] == contact_id
    fleet.turn_off()
    assert fleet.load_state() == {"enabled": False, "contact_install_id": contact_id}
