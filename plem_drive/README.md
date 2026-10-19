# plem_drive

Runs CiA 402 servo drives on an EtherCAT bus from one drive model file (a YAML file describing the bus). `plem drive
check` reads the file the way the bus does; `plem drive run` brings the bus up under ros2_control and runs each axis in
torque, position or velocity (`cst`, `csp`, `csv`); `plem.drive` moves the axes from Python. On hardware it has run two
A6B drives in `csp`, and a copy of that file in `csv`; both were benched and approved. A `cst` copy had short runs
without the bench. [`docs/hardware-evidence.md`](../docs/hardware-evidence.md) lists what has and has not run.

The RT event and raw cycle messages and the control command service it uses are in [`plem_rt_msgs`](../plem_rt_msgs/);
its own status and facts messages are in [`plem_drive_msgs`](../plem_drive_msgs/).

## What it provides

| Part | What it is |
|---|---|
| `plem-drive` (`bin/`) | The command behind `plem drive`: `check` runs `drive_model_tool`, `run` ([`plem/drive/cli/run.py`](plem/drive/cli/run.py)) starts the bus and offers the bench for a file not yet approved (for an approved file it first compares the drives on the bus with the approval record, `drive_model_tool probe`) |
| `plem.drive` | The Python client: `Bus` (`release`, `move_to`, `stream`, `stop`) and the motion profiles |
| `plem_drive/DriveSystemInterface` | The ros2_control hardware interface (`SystemInterface`): one joint per axis, the bus's status, events, `rt_raw` and the control command service beside the controllers |
| `plem_drive/StreamController` | The controller that takes `~/commands` for the `csv` and `cst` joints |
| Bench and monitor | [`plem/drive/cli/bench.py`](plem/drive/cli/bench.py) (the approval run `plem drive run` offers) and [`plem/drive/cli/monitor.py`](plem/drive/cli/monitor.py) (`--monitor`; also `python3 -m plem.drive.cli.monitor <bus.name>` for a bus already running) |
| `rt_raw_relay` | Republishes each `rt_raw` batch as one message per cycle on `rt_raw/samples`, for plotting (`plem drive run --plot` opens PlotJuggler on a layout built from the bus's axes) |
| `drive_model_tool` | The C++ tool behind `plem drive check`, the bench's approval step and `run`'s check of the drives, in `lib/plem_drive/` (not on `PATH`) |
| `share/plem_drive/examples/` | The examples to start from ([`examples/README.md`](examples/README.md)) |
| `share/skills/add-drive/` | An agent skill that writes a drive model file with a source for every value |

The command's internals are `plem.drive.cli`; the Python API is what `plem.drive` exports.

`drive_model_tool`'s verbs:

- `plem drive check [--json] <model.yaml>` — reads the file the way the bus does and prints a summary (bus, axes,
  approval, warnings, next step), or with `--json` one JSON document (`"format": "plem-drive-check/1"`: digests,
  approval state, required bench items, axes, slaves); exit 0 when the approval the file claims is the one in effect.
  `plem drive check --help` describes it.
- `approve <model.yaml> <bench.yaml>` (internal — the bench's approval step, not a verb of `plem-drive`) — writes the
  approval record from a bench result into the model file itself: its `approval` block is replaced, or appended when
  it has none; every other byte is kept, and the write is atomic. Refused when the bench does not match the file as it
  is now or a required item is not PASS.
- `probe <model.yaml> --master N` (internal — `plem drive run` calls it before it starts an approved file) — the bus's
  own start-up check of the drives on master N against the file: their model, presence and the approval record's
  identities. Nothing is configured, written or activated. Exit 0 when the drives match, 3 when only identity values
  differ (on a terminal `run` then offers the bench, which replaces the record), 2 for a malformed command, 1 for anything else; the reason goes
  to stderr.

## C++

Two libraries, linked by target name:

| Target | Headers, namespace | What it is |
|---|---|---|
| `plem_drive::plem_drive` | `<plem/drive/…>`, `plem::drive` | The drive model file loader and the drive host. Links no ROS library, so a process without ROS can run drives with it (building against it still needs the package's ROS 2 dependencies installed) |
| `plem_drive::plem_drive_ros` | `<plem/drive/ros/…>`, `plem::drive::ros` | The ROS layer: the hardware interface and the stream controller (loaded by name, their headers are not installed), and the helpers another hardware interface links (`ControlCommandService`, `RtRawBatcher`, `to_rt_event`) |

Link the one you need: `target_link_libraries(my_target plem_drive::plem_drive)`. `ament_target_dependencies(my_target
plem_drive)` links both.

`plem_drive::plem_drive`'s headers:

| Header | What it is |
|---|---|
| `drive_model_file.hpp` | `load_drive_model_file(path)` — reads and checks a drive model file of 1 to 8 axes and returns the model, or an error naming the file and the line or key |
| `models/` | The loaded model types |
| `drive_host.hpp` | `DriveHost<N>` — runs N drives on one EtherCAT bus: start and stop, enable and disable, fault reset, and `write()`, the command path into the RT loop |
| `bus_setup.hpp`, `models/validation.hpp` | For a program that runs its own EtherCAT bus from the same spec types: check the slave and axis values, build the PDO mappings and one `ServoInfo` per axis, set up the bus before `initialize()` |

`plem_drive::plem_drive_ros`'s headers, in `plem/drive/ros/`: `control_command_service.hpp`, `rt_event.hpp` and
`rt_raw_batcher.hpp`. Headers under `src/` are the package's own and are not installed.

### Running drives without ROS

The hardware interface is the caller in this package (`build_drive_host` in [`src/ros/drive_bus.cpp`](src/ros/drive_bus.cpp)).
`drive_host.hpp` documents each call; in order:

1. `load_drive_model_file(path)` returns a `core::Result<DriveModelSpecAny>`. Check `is_ok()` (the reason is
   `message()`); `value()` is a `std::variant` with one `models::DriveModelSpec<N>` per axis count.
2. `std::visit` the variant and call `DriveHost<N>::create(spec, bus, params)`. The host never creates its bus: pass
   `std::make_shared<plem::fieldbus::ethercat::EtherCATBus>(master_index)`. It returns a
   `core::Result<std::unique_ptr<DriveHost<N>>>`: check it the same way.
3. `start()` runs the RT loop (a `Result`). `enable()` enables motion (a `Result`; refused for an unapproved model unless
   `DriveHostParams::bench_unapproved` is set). The order of `enable()` and the first `write()` depends on the bus:
   - a bus with a `cst` axis: `enable()` is refused unless a `write()` landed within `kCommandStaleTimeoutNs`
     (50 ms) before it — start writing commands, then call `enable()` while they keep coming;
   - a bus without one: `enable()` holds the position it finds, and only `write()` calls made after it are applied.
4. Call `write(command, rate, sampled_ns)` every cycle (false = rejected, nothing written) and `read(state)` for the
   last cycle's state (false = retry). When the writes stop for `kCommandStaleTimeoutNs`, the bus takes a protective
   stop (on a bus without a `cst` axis, only while the drives are released).
5. `stop()` ends the run: on a bus without a `cst` axis, enabled axes first ramp to rest (with one, the drives are disabled
   at once — [`docs/getting-started.md`](../docs/getting-started.md), section 6); it then disables the drives,
   requests brake engagement, waits for it within a timeout and stops the loop. It returns a
   `fieldbus::ethercat::BrakeQuiesceOutcome`: an unconfirmed engagement is reported there, not raised. Check
   `never_released` first (nothing was released, so nothing was observed), then `fully_engaged()` and `timeout_mask`.

## Dependencies

`plem_core`, `plem_fieldbus`, `plem_drive_msgs`, `plem_rt_msgs`, `libyaml_vendor`, nlohmann-json, fmt, ros2_control
(`hardware_interface`, `controller_interface`, `controller_manager` and the controllers `plem drive run` starts),
`rclcpp`, `rclpy` and the message packages in `package.xml`, resolved by `rosdep`.

## Build and test

From the repository root (the colcon workspace — [`docs/install.md`](../docs/install.md), section 6), with the IgH
EtherCAT master installed (see `plem_fieldbus`):

```bash
colcon build --packages-up-to plem_drive --cmake-args -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select plem_drive && colcon test-result --verbose
```

The tests need no drives: gtest suites for the library, the plugins and the bus surface, pytest for the command, the
bench, the monitor and `plem.drive`, and an end-to-end `plem drive run` over a simulated host. Some include
`plem_fieldbus`'s test-support headers ([`docs/api-scope.md`](../docs/api-scope.md#headers-installed-for-tests)).

## Documentation

`README.md` and [`docs/`](../docs/) at the root of this repository:
[`docs/getting-started.md`](../docs/getting-started.md) (the example, `plem drive run`, `plem.drive`, stopping, bench
and approval, monitoring, the two-axis example), [`docs/cia402-drive-model.md`](../docs/cia402-drive-model.md) (every
key of the drive model file and where each rule is checked; loader errors cite its sections as "schema §N") and
[`docs/install.md`](../docs/install.md) (setting up the Jetson).
