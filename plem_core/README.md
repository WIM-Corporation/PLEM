# plem_core

A C++20 library of real-time utilities shared by the other drive packages. It does not use rclcpp.

## What it provides

Library `plem_core` (shared), headers under `include/plem/core/`:

| Header | What it is |
|---|---|
| `units.hpp`, `units/` | Typed units: angle, angular velocity, torque, pulse counts and pulse velocity, torque counts |
| `result.hpp` | `Result<T>` — either a value or an error |
| `constants.hpp` | `kMaxBusAxes`, the maximum number of axes on one fieldbus |
| `timing/` | `periodic_timer_rt.hpp` (a periodic timer on `clock_nanosleep` with absolute deadlines), cycle-count conversion, `timespec` helpers |
| `threading/` | Namespace `plem::core::threading`: RT thread configuration (scheduling policy, CPU affinity, isolated-CPU detection; `rt_thread_config.hpp`), RT memory hardening (memory locking, heap and stack prefault, page-fault counts; `rt_memory.hpp`), a byte-level seqlock (`seqlock::`, whose protocol also serves memory shared between processes) and `SeqlockData<T>`, a seqlock-protected value within one process (`seqlock.hpp`) |
| `logging/logger.hpp` | Logging on spdlog |
| `monitoring.hpp`, `monitoring/` | Namespace `plem::core::monitoring`: the RT monitoring API, single-producer, single-consumer queues (with overflow counts) into which the RT thread pushes samples, events and metrics, and from which a publishing thread reads them |
| `crypto/sha256.hpp` | SHA-256 as a lowercase hex string (libsodium) |
| `types/` | Shared value types: per-joint targets and actuals (`JointTarget`/`JointActual`), joint commands, motion limits, control mode flags with the protective-stop decision shared by every RT command owner (`control_mode_flags.hpp`), `TriState`, and the shared RT data block (`SharedJointData`, `shared_joint_data.hpp`) |
| `third_party/rigtorp/SPSCQueue.h` | Erik Rigtorp's single-producer single-consumer queue, MIT license (notice in the file) |

`test/include/plem/core/test/seqlock_data_test_peer.hpp` (`plem::core::test::SeqlockDataTestPeer`) is for tests, not for
applications; [`docs/api-scope.md`](../docs/api-scope.md#headers-installed-for-tests) says when it is installed.

## Dependencies

spdlog and libsodium (`package.xml`), resolved by `rosdep`.

## Build and test

From the repository root (the colcon workspace — [`docs/install.md`](../docs/install.md), section 6):

```bash
colcon build --packages-up-to plem_core --cmake-args -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select plem_core && colcon test-result --verbose
```

The tests are gtest suites under `test/`.

## Documentation

`README.md` and [`docs/`](../docs/) at the root of this repository: installing on the Jetson ([`docs/install.md`](../docs/install.md)), running a
drive ([`docs/getting-started.md`](../docs/getting-started.md)) and the drive model file reference ([`docs/cia402-drive-model.md`](../docs/cia402-drive-model.md)).
