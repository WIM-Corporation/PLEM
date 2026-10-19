# plem_fieldbus

A C++20 library that runs CiA 402 servo drives over EtherCAT with the IgH EtherCAT master. It does not use
rclcpp.

## What it provides

Library `plem_fieldbus` (shared), headers under `include/plem/fieldbus/`:

| Header | What it is |
|---|---|
| `cia402_bus.hpp` | `ICiA402Bus` — the bus interface for assembly and fault reset (the RT loop and the status read take the concrete `EtherCATBus`) |
| `cia402_profile.hpp`, `ethercat/state_machine.hpp` | The CiA 402 state-machine transitions and `ServoStateMachine`, which walks a drive through them |
| `watchdog.hpp` | The working-counter watchdog and command-freshness checks |
| `ethercat/bus.hpp` | `EtherCATBus`, the `ICiA402Bus` on the IgH master |
| `ethercat/controller.hpp`, `ethercat/controller_config.hpp` | `EtherCATController<DOF>`, the RT EtherCAT loop (its thread lifecycle and RT scheduling), and its `ControllerConfig` |
| `ethercat/brake_logic.hpp`, `ethercat/drive_safety.hpp` | Brake decisions, and the protective-stop decision the RT loop makes from the declared drive limits (`set_drive_limits()`, which `start()` requires on a `csp` or `csv` axis) |
| `ethercat/pdo_layout_validation.hpp`, `ethercat/pdo_scan.hpp` | Checks of a declared PDO mapping, and reading back the mapping a slave reports |
| `ethercat/device_base.hpp`, `ethercat/io_device_factory.hpp` | The IO extension point: an IO slave's device is a `DeviceBase` the application supplies through an `IODeviceFactory` (the bus has no default IO device) |

Headers under `test/include/` (`<plem/fieldbus/test/…>`, namespace `plem::fieldbus::test`: the fake IgH master's
controls, test probes and fixtures) and the `plem_fieldbus_fake_master` library are for tests, not for applications;
[`docs/api-scope.md`](../docs/api-scope.md#headers-installed-for-tests) says when they are installed.

## Dependencies

`plem_core` and fmt (`package.xml`), resolved by `rosdep`, and the IgH EtherCAT master, which is not a
rosdep key and has to be installed by hand (below).

## The IgH EtherCAT master

IgH is required. The CMake configure step looks for `ecrt.h` and the shared `libethercat.so` (the unversioned
development link) on `CMAKE_PREFIX_PATH` first, then under `/opt/etherlab` (IgH's default prefix), then in the system prefixes. Configure
stops with an error when:

- either file is not found;
- `ETHERCAT_LIB` is not a shared library: the path must end in `.so`, so a static `.a` or a versioned `.so.N` is
  refused — libethercat is LGPL and is linked dynamically only;
- the header and the library come from different install prefixes.

Each error says what to do. The library links `libethercat.so` and gets the IgH library directory as its install
RPATH.

Installing IgH 1.6.13 on the Jetson: [`docs/install.md`](../docs/install.md), section 2 (the setup wizard), at the root of this repository. Other hosts:
follow IgH's own install instructions; this package needs only `ecrt.h` and the shared `libethercat.so`.

At run time the IgH kernel master (`ec_master` with a NIC driver) must be loaded and bound to the EtherCAT port, and
the user running the node needs read and write access to `/dev/EtherCAT0`. The setup wizard sets up the service and the
device rebind; [`docs/install.md`](../docs/install.md), section 3, covers the real-time limits and the device permission.

## Build and test

From the repository root (the colcon workspace — [`docs/install.md`](../docs/install.md), section 6), with IgH installed:

```bash
colcon build --packages-up-to plem_fieldbus --cmake-args -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select plem_fieldbus && colcon test-result --verbose
```

The tests need no EtherCAT network or drives: bus bring-up is tested against a fake IgH master
compiled into the test, and the controller against that fake master.

## Documentation

`README.md` and [`docs/`](../docs/) at the root of this repository: installing on the Jetson ([`docs/install.md`](../docs/install.md)), running a
drive ([`docs/getting-started.md`](../docs/getting-started.md)) and the drive model file reference ([`docs/cia402-drive-model.md`](../docs/cia402-drive-model.md)).
