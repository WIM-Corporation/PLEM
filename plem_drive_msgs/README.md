# plem_drive_msgs

The ROS 2 interfaces specific to the drive bus in `plem_drive` (published by `plem drive run` alongside its
controllers, on a node named `plem_drive`). The event stream, the raw cycle stream and the control command service
that every RT host shares are in [`plem_rt_msgs`](../plem_rt_msgs/README.md).

## What it provides

| Interface | What it carries | Where the drive bus uses it |
|---|---|---|
| `msg/DriveInfo` | Facts that do not change while the bus runs: the PDO mapping digest per slave and the declared mode of operation per axis | `~/info`, published once, latched |
| `msg/DriveStatus` | Protective-stop source, whether every drive is in Operation enabled in its declared mode, brake state, the statusword per axis — at a low constant rate, not for timing | `~/status`, latched |

Each file's comments describe its fields.

## Dependencies

`std_msgs`, `rosidl_default_generators` and `rosidl_default_runtime` (`package.xml`).

## Build

From the repository root (the colcon workspace — [`docs/install.md`](../docs/install.md), section 6):

```bash
colcon build --packages-up-to plem_drive_msgs --cmake-args -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The package has no tests of its own; `plem_drive`'s tests use these interfaces.

## Documentation

`README.md` and [`docs/`](../docs/) at the root of this repository; [`docs/getting-started.md`](../docs/getting-started.md) shows the topics in use.
