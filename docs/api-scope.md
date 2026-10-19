# Public API scope

Most of the installed headers, messages and services are what `plem drive run` and `plem.drive` use. This page lists
the rest: surface that exists for a separate stack, and headers installed only for
tests.

## Surface for a separate, non-public stack

These packages are also the base layer of a separate robot stack that is not published; code comments call it "the
robot tier" or "a higher-layer host". The surface below exists for that stack; unless an item says otherwise, this tree does not exercise it:

- In `RtRawBatch`, `command_position` on an axis that is not `csp` and `command_velocity` on an axis that is not
  `csv` are always 0: they report a motion target the drive bus never writes.
- `SetControlCommand`'s `free_drive` is always refused, and `ControlModeFlags::freedrive_enabled` is never set by the
  drive host.
- The `ControllerSignal` and `RtDiagnostics` monitoring streams are not allocated by the drive host.
- `freedrive_enabled`, `ControllerSignal` and `RtDiagnostics` are marked in their headers as internal: not part of
  the public API, they may change without notice.
- `AxisSpec`'s `position_before_enable_index` is set only by that stack; a drive model file does not carry it.
- That stack builds its own bus with `plem_drive`'s `bus_setup.hpp` and `models/validation.hpp` (the drive host uses
  them too), and the drive bus never calls `EtherCATBus::set_io_device_factory`.
- `plem_fieldbus`'s `ICiA402Bus` (`cia402_bus.hpp`) is the bus interface that stack assembles through;
  `EtherCATBus` implements it. The drive host and the RT loop take the concrete `EtherCATBus`.

## Headers installed for tests

- `plem_core`: `test/include/plem/core/test/seqlock_data_test_peer.hpp` (`plem::core::test::SeqlockDataTestPeer`, a
  peer that freezes a `SeqlockData` writer mid-write).
- `plem_fieldbus`: the headers under `test/include/` (`<plem/fieldbus/test/…>`, namespace `plem::fieldbus::test`: the
  fake IgH master's controls, test probes and fixtures) and the `plem_fieldbus_fake_master` library, a stand-in for
  the IgH master.

They are for tests, not for applications. The default colcon build, which has `BUILD_TESTING` on, installs them;
`-DBUILD_TESTING=OFF` leaves them out. Some of `plem_drive`'s tests include `plem_fieldbus`'s, so they build only
against a `plem_fieldbus` built with `BUILD_TESTING` on.
