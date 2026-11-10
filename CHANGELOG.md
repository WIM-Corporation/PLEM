# Changelog

Releases of this repository. Each one is a tag `vX.Y.Z`. Documentation-only updates between releases are not
listed.

## [0.11.0] - 2026-11-11

First public release, for Jetson AGX Orin with JetPack 7.2.1 and ROS 2 Jazzy.

- Packages: `plem_core`, `plem_fieldbus`, `plem_drive`, `plem_drive_msgs`, `plem_rt_msgs`
  (Apache License 2.0; `plem_core` also carries an MIT-licensed queue, see [`NOTICE`](NOTICE)).
- `plem drive check` and `plem drive run`, the drive model file ([`docs/cia402-drive-model.md`](docs/cia402-drive-model.md)), the Python
  `plem.drive` API and the `a6b_pair` example. Besides refusing invalid files, `check` warns when an axis's
  `limits.position` reaches past what the drive's 32-bit position can report with its `zero_offset_count` and `flip`,
  and when a slave still has `example_drive.yaml`'s placeholder `vendor_id`.
- The setup wizard for the RT kernel, CPU isolation and the IgH EtherCAT master ([`setup/rt/`](setup/rt/)).
- Build from source: [`docs/install.md`](docs/install.md).
- Optional device reporting, asked once by the setup wizard ([`docs/telemetry.md`](docs/telemetry.md)).
