# plem_rt_msgs

The ROS 2 interfaces every PLEM RT host shares. The drive bus in `plem_drive` publishes and serves them, and a
higher-layer host built on the same RT loop publishes and serves the same types.

## What it provides

| Interface | What it carries | Where the drive bus uses it |
|---|---|---|
| `msg/RtEvent` | Discrete RT events (deadline misses, communication errors, state transitions, faults) with RT monotonic time stamps | `~/rt_events`, latched |
| `msg/RtRawBatch` | Every RT cycle's data (up to 8 axes per sample; each batch states its own `axis_count`), in batches of K samples | `rt_raw`, best effort |
| `srv/SetControlCommand` | Flags `release_brake`, `protective_stop`, `reset_error`, `free_drive` (the drive bus refuses `free_drive`). A protective stop in the same request as a release wins: only the stop runs, and the response is `success=false` and names what was dropped | `/<bus>/plem_drive/control_command` |

Each file's comments describe its fields. The drive bus's own facts and status are in
[`plem_drive_msgs`](../plem_drive_msgs/README.md).

## Dependencies

`std_msgs`, `rosidl_default_generators` and `rosidl_default_runtime` (`package.xml`).

## Build

From the repository root (the colcon workspace — [`docs/install.md`](../docs/install.md), section 6):

```bash
colcon build --packages-up-to plem_rt_msgs --cmake-args -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The package has no tests of its own; `plem_drive`'s tests use these interfaces.

## Documentation

`README.md` and [`docs/`](../docs/) at the root of this repository; [`docs/getting-started.md`](../docs/getting-started.md) shows the topics and the
service in use.
