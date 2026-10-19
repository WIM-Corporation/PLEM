# Getting started — run a servo drive

This page gets a CiA 402 servo drive moving on an EtherCAT bus: you describe the drive in one YAML file, and
`plem drive run` reads it and runs the bus under ros2_control (section 4). Each axis runs torque (`cst`), position
(`csp`) or velocity (`csv`), chosen per axis in the file, and one bus may mix them. Every key is
defined in the drive model schema, [`docs/cia402-drive-model.md`](cia402-drive-model.md), called "schema"
below (`schema §4.1` is its section 4.1).

You need a prepared Jetson with the drive software built from source and the workspace sourced ([`docs/install.md`](install.md)).

> **Two Panasonic A6B drives?** Follow the example's own guide,
> [`plem_drive/examples/a6b_pair/README.md`](../plem_drive/examples/a6b_pair/README.md): it runs the same steps with
> real values. This page is for any other drive.

<details><summary>Terms used on this page</summary>

| Term | Meaning |
|---|---|
| EtherCAT | The fieldbus: one Ethernet line from the Jetson's port through every drive, read and written every cycle |
| Master, slave | The Jetson's EtherCAT side (the IgH master) and each device on the line (a drive is a slave) |
| Ring position | A slave's place on the line, counted from 0 at the port (`ethercat slaves` lists them) |
| CiA 402 | The standard servo-drive profile: the state machine (enable, fault, …) and the motion objects |
| CoE | CANopen over EtherCAT: how a drive's objects (`0x6064` and so on) are reached over EtherCAT |
| Object, SDO | A drive setting or value at an index such as `0x6064`; an SDO reads or writes one, outside the cycle (`ethercat upload`, `ethercat download`) |
| PDO | The objects exchanged every cycle (command out, position and status in) |
| Statusword, controlword | The drive's state (`0x6041`) and the master's request (`0x6040`) |
| DC, SYNC0 | Distributed clocks: the drives' shared clock, and the pulse at which each applies the cycle's command |
| ESI | The drive vendor's XML file describing the drive; some values in the file come from it |
| Working counter | How many slaves handled a cycle's frame; a mismatch means a slave did not exchange that cycle's data (every cycle until all slaves reach OP) |
| `csp`, `csv`, `cst` | Cyclic synchronous position, velocity and torque: the axis mode |

</details>

Four steps for your own drive:

```bash
mkdir -p "$HOME/drives" && cp "$(ros2 pkg prefix --share plem_drive)/examples/example_drive.yaml" "$HOME/drives/my_drive.yaml"   # copy an example (section 1)
# fill in your drive's values (section 2)
plem drive check "$HOME/drives/my_drive.yaml"   # check the file (section 3)
plem drive run "$HOME/drives/my_drive.yaml"     # bench it once (section 8), then run the bus (section 4)
```

> **Open the first axis without a gravity load.** The steps below release the brakes under a neutral command (0 Nm
> on a torque axis). The drive layer has no gravity compensation: a loaded torque axis falls as soon as its brake
> opens. Position and velocity axes are held by the drive's own loop, but how that behaves differs per drive and has
> been seen on one unloaded rig only. Test first on an unloaded axis, with the emergency stop within reach.

> **What has run on hardware:** JetPack 7.2.1 with ROS 2 Jazzy, using both Intel I210 with the native driver (Case 2) and the built-in RJ45 port (Case 1). The A6B example (section 10) and three copies of it have run: one with `flip`, one in `csv` and one with a `generated` mapping ([`docs/install.md`](install.md#supported-platform)).
> [`docs/hardware-evidence.md`](hardware-evidence.md) lists each session and what has not run.

## 0. The `plem drive` command

`plem drive` has two verbs. `plem drive check` reads a file (section 3). `plem drive run` runs the bus
(section 4), and for a file not yet approved it offers the bench first (section 8). `plem drive --help` lists the
verbs, and `plem drive <verb> --help` explains one.

`plem` ([`tools/plem`](../tools/plem), installed in [`docs/install.md`](install.md), section 6) passes its arguments
unchanged to the `plem-drive` command of `plem_drive`, so `plem-drive <verb>` is the same tool. It is on the PATH once
the workspace is sourced (`source install/setup.bash`).

## 1. Copy an example

The package installs two examples:

- `example_drive.yaml` — one position axis. Every value is synthetic.
- `a6b_pair/` — two Panasonic MINAS A6B drives, one position axis each, in `drive.yaml`. Its values come from
  Panasonic's documents (cited in the example's [`README.md`](../plem_drive/examples/a6b_pair/README.md#the-examples-values-and-where-they-come-from)) and were read from the maintainer's drives. The shipped file is not approved: bench it (section 8). What has run
  on those drives is in its [`README.md`](../plem_drive/examples/a6b_pair/README.md#the-drives-the-checks-and-the-values). The folder also holds `demo.py` (section 10).

```bash
mkdir -p "$HOME/drives"
cp "$(ros2 pkg prefix --share plem_drive)/examples/example_drive.yaml" "$HOME/drives/my_drive.yaml"
```

Replace the values with your drive's and keep the shape. The synthetic values fit no real drive.

Or let a coding agent write it. The package installs an agent skill, `add-drive`, that takes the copy through
section 2. It is told to read your drives with read-only `ethercat` commands only (review the commands your agent
asks to run). It asks for the drive, motor and gearhead, reads their documents, writes each value with its source as
a comment beside it, and asks you for every decision (limits, stop decelerations, cycle). Where nothing gives a
value it leaves the key out, marked `# OPEN`; a file with a missing required key is refused. It stops early when a
premise of the schema does not hold (a holding brake per axis that the drive itself operates, an absolute encoder;
schema §4.4). Otherwise it runs `plem drive check` until the file passes or fails only on its open keys (the rest
judged on a scratch copy). The bench (section 8) stays yours. For Claude Code, link it into your skills:

```bash
mkdir -p ~/.claude/skills
ln -s "$(ros2 pkg prefix plem_drive)/share/skills/add-drive" ~/.claude/skills/
```

Another agent reads the same `SKILL.md` as its instructions. Check what it wrote: the comments say where each value
came from, and a wrong value still loads.

## 2. Fill in the values

A file with a missing key is refused. Only these may be left out:

- `cia402_profile`: the standard CiA 402 transition table is used.
- An axis's `mode_display_timing`: `pre_enable` is used.
- The sub-block of a switched-off feature: `dc` with `sync_mode: drive_default`, `pacing` with `policy: none`.
  Likewise, `limits.torque_nm` is written on `cst` axes only and `limits.acceleration_rad_s2` on `csp` and `csv` axes only.
- A slave's `stored_sdo`: no stored value is read.
- A `dc` block's `write_cycle_time`: 0x1C32:02 is not written.
- `approval`: the file is unapproved.
- The position and velocity `unit`: it has one value.
- `error_code_pdo`, when the mapping carries Error code 0x603F (a `generated` mapping, or a declared one that maps it).

"No value" is written as `null` or `[]`.

Identical drives (the same drive, motor, gearhead and settings) share one slave entry: write their ring positions as
a list (`ring_position: [0, 1]`) and give `name`, `flip` and `zero_offset_count` one value per drive
(`name: [axis0, axis1]`). The example in section 10 does this (schema §4.1).

| Key | Where it comes from |
|---|---|
| `vendor_id`, `product_code`, `ring_position` | `ethercat slaves -v` |
| `pdo_mapping` | `scanned` (the drive's power-on mapping), `generated` (PLEM writes the CiA 402 objects it uses in PREOP; run on hardware only with the A6B, `csp`) or a declared mapping PLEM writes in PREOP, taken from the drive's ESI file, `ethercat pdos` or the log of a `scanned` start (`PLEM_LOG_LEVEL=debug`; a rejected scan prints it anyway; schema §4.1) |
| `bus` | The bus's `name` (it runs as `/<name>`; schema §4.5), the cycle `cycle_time_ns` (a multiple of 1000) and `sync_mode`: `drive_default` (the drive's own synchronization) or `dc_sync0` (schema §5) |
| `dc` | Only with `sync_mode: dc_sync0`, on every slave: `assign_activate` from the drive's ESI (`Dc/OpMode`), `sync0_shift_ns` so that SYNC0 falls after the cycle's frame has reached the drive (300000 at 1 ms on the A6B), `sync0_cycle_ns` a cycle the drive supports and a multiple of `cycle_time_ns`, and `write_cycle_time: true` only when the drive's ESI or manual says the master sets 0x1C32:02 (schema §5) |
| `name` | Your choice: lower case, digits and `_`, at most 31 bytes, unique in the file |
| `channel` | Which CiA 402 axis inside the drive (1–8) |
| `mode` | `cst`, `csp` or `csv` per axis. Check that the drive supports it: the mode's target object (0x6071, 0x607A, 0x60FF) must be in the mapping |
| `mode_display_timing` | Leave it out (= `pre_enable`) unless the drive shows its mode on 0x6061 only after enable. Such a drive's release is refused, and the refusal says to write `post_enable` on that axis (schema §4.2a) |
| `factors` | From the drive manual and motor nameplate: `gear_ratio` and `encoder_resolution`, once per axis. Torque unit: `rated_per_mille` when the manual gives 0x6071/0x6077 in 0.1 % of rated torque (`motor_rated_torque_nm` in Nm), and `scaled_current` when it gives current counts |
| `velocity_factor` | The 0x606C unit in the drive manual. A wrong value still loads and silently skews the velocity limit and `joint_states` velocities |
| `flip`, `zero_offset_count` | Measure them (one per drive when the entry lists its ring positions) |
| `limits` | **Your responsibility.** No defaults. An axis without end stops writes `position: null` (schema §4.4). A position or velocity axis also needs `acceleration_rad_s2`, the deceleration at which a requested stop ramps it to rest (section 6): what the axis can brake at, and no more than the drive's 0x6085 |
| `startup_sdo` | Drive manual; `[]` for none |
| `stored_sdo` | Optional: values the drive must hold in its own stored parameters (input assignment, encoder mode, tuning). Read at start; a difference gives one warning, nothing is written and the bus starts anyway. Leave the key out for none (schema §4.9) |
| `fault_reset` | Whether to reset faults automatically (`policy`) is your choice; `pulse_ms` from the drive manual, or `0` (one cycle) when it states nothing |
| `cia402_profile` | Leave it out for a drive that follows the standard. A drive that takes Shutdown in `not_ready_to_switch_on` after power-on needs a table with that row (schema §4.8) |
| `approval` | Leave it out. The bench writes the record into the file (section 8); it is never written by hand |

**The encoder must be absolute.** PLEM has no homing. `zero_offset_count` and the position limits locate an axis only
if the drive reports the same position after every power-on. An incremental encoder restarts its count at power-on,
so the limits would then guard a different range. The file has no key for the encoder type, and nothing can detect
it. Do not use an incremental encoder (or an absolute one set to an incremental mode) on an axis with position
limits. This is a premise of the schema, like the brake premises (schema §4.4). An endless axis (below) has no
position limits to protect, and its position is relative after a restart anyway.

**An endless axis** (an output that keeps turning one way) writes `position: null` with `zero_offset_count: 0`, in any
mode, `csp` too. Its position stays one continuous coordinate across the drive's 32-bit wrap, but after a PLEM restart
it starts again from the drive's own value, so command a `csp` axis relative to where it is. The drive must meet the
requirements in [schema §4.4](cia402-drive-model.md#44-per-axis-physical-safety--required-declarations) (wrap
handling, software position limit, counter overflow at start; it gives the A6B's settings). Do coordinate-changing
maintenance (homing, multi-turn clear) while stopped, then restart PLEM. Net one-way travel of more than 2⁴⁸ counts
since start causes a protective stop that lasts until PLEM restarts; that stop was not verified on hardware. Crossing
the wrap ran on two A6B drives ([`docs/hardware-evidence.md`](hardware-evidence.md)).

## 3. Check the file

No drive needed:

```bash
plem drive check "$HOME/drives/my_drive.yaml"
```

A refusal says what is wrong and how to fix it, by the file's key path, for a format error
(`slaves[1].axes[0].channel`) and a value error (`slaves[1].axes[0].limits.position`) alike. For a `ring_position`
list the message first names which drive. A message's `schema §4.1` is that section of the schema. `check` also
refuses, without the drives, a file the bus would refuse at startup (a mode's required PDO object missing, two axes
on one channel).

A passing file prints a summary: the bus; each axis with its mode, ring position and channel; whether the file is
approved (if a claimed approval does not hold, each reason on its own line); the warnings; and the next step. The
warnings name a position or velocity axis whose file leaves the drive's stop objects (0x605A, 0x605C, 0x6084, 0x6085;
section 6) to stored values, a DC slave with a SYNC0 shift of 0, an axis whose `limits.position` reaches past what the
drive's 32-bit position can report with its `zero_offset_count` and `flip` (the range is printed), and a slave that
still has the example's `vendor_id` 0x00c0ffee. They do not refuse the file or change the exit code. With that
`vendor_id`, where the next step would offer `plem drive run` on an unapproved file, it says to replace the example's
values instead.

For scripts, `plem drive check --json <file>` prints the same judgment as one JSON document. The exit code is the
same with or without `--json`.

| Exit code | Meaning |
|---|---|
| 0 | The file's claimed approval state holds |
| 1 | The file is refused, or its claimed approval does not hold (the reason is on stderr, or under `approval:`; in the JSON, `reasons`) |
| 2 | Wrong usage |

**Passing `check` is not passing start-up.** The JSON's `not_judged` lists what `check` does not judge: a scanned slave's
mapping, the live drive identity (SII serial number, 0x1009, 0x100A, the scanned mapping's digest), fault-reset pacing
against the cycle, DC rules. The bus judges those when `plem drive run` attaches it.

<details><summary>The <code>--json</code> fields</summary>

The document (`"format": "plem-drive-check/1"`) holds two digests, `implementation_revision`, the claimed and
effective approval, and what the bench needs: the required bench items, `approve_blocker` (null, or why recording an
approval would be refused), and the axis and slave addresses.

The JSON's `axes[i]` also carry what `plem drive run` builds from:

- `mode`
- `position_min_rad`/`position_max_rad`: null on an axis without end stops
- `velocity_limit_rad_s`
- `velocity_cap_rad_s`: the speed a planner may command. On a `csp` axis that is the velocity limit less one
  position count per cycle (two on an axis without end stops), so a rounded target stays inside the one-cycle step
  check; on other axes, the velocity limit. A `csp` axis whose cap would not be positive is refused (raise
  `limits.velocity_rad_s`, lengthen `bus.cycle_time_ns` or use a finer position channel).
- `acceleration_limit_rad_s2`: null on a `cst` axis
- `torque_min_nm`/`torque_max_nm`: a `cst` axis's `limits.torque_nm`; null on other axes

</details>

## 4. Run the bus — `plem drive run`

This section has run on hardware ([`docs/hardware-evidence.md`](hardware-evidence.md), sixth session and later). Not
run yet: two coupled axes turning together under torque, `--rt-cpu`, and two buses.

```bash
plem drive run "$HOME/drives/my_drive.yaml"
```

`plem drive run <file> [--master N] [--rt-cpu N] [--monitor] [--plot] [--unapproved] [--verbose]` runs in
the terminal it is started from. The bus runs in the namespace `/<bus.name>` (the file's
`bus: { name: … }`; `my_drive` in the example, so `/my_drive` below).

1. It runs `plem drive check` on the file. It refuses when a bus of the same name is already up on this host (another
   `plem drive run` of yours on that namespace; another user's bus, or one on another host of the ROS domain, is not
   seen). For a file not approved for motion, on a terminal it says so and the bench follows (section 8), and it starts once the bench has recorded the approval. If the bench is declined or
   fails, or there is no terminal, it refuses and says why. `--unapproved` runs the file anyway as a bench
   run, with a `BENCH MODE` warning at start and on every release. Use it only when you are at the drives; the run's
   evidence is bench evidence only. For an approved file, the drives on the bus are first compared with its approval
   record (section 8).
2. It warns, and starts anyway, when the kernel is not PREEMPT_RT, when no CPU is isolated, and when the Jetson's
   nvpps PTP-TSC lock is on (it slews the host clock, and the drives' DC drift follows). Each warning says the fix.
3. It writes the robot description and the controller manager's parameters. The description has one joint per
   axis, in the file's order, with the file's position limits (`continuous` on an axis without end stops) and
   `velocity_cap_rad_s` as its velocity limit. In the parameters the update rate is the bus cycle. The manager's
   update thread is kept off the isolated CPUs and the `--rt-cpu` CPU, when there is another CPU it may run on.
4. It starts `robot_state_publisher` and the controller spawners, then becomes the controller manager itself (exec).
   Ctrl-C reaches it directly, and the bus stops every axis before it lets go: with the requested stop of section 6
   (ramp to rest at `limits.acceleration_rad_s2`, then the brakes) on a bus without a torque axis, otherwise with the
   immediate disable. An engage it cannot confirm is logged as an ERROR ("brakes were NOT confirmed engaged on stop").
5. The bus comes up once every slave is in OP. With the loop running, it waits until the domain's working counter is
   complete and then logs `every slave in OP (working counter complete) … s after the loop started`. Move after that
   line. On a bus with a `csp` axis the trajectory controller becomes active a moment later, once its spawner has
   started it: `Bus.release()` waits for it (up to the `Bus` timeout), and a `FollowJointTrajectory` client of your
   own should wait until the controller is active (`ros2 control list_controllers`) or retry a rejected goal. How long
   this takes is up to the master: IgH configures the slaves one at a time and holds a DC slave short of OP until its
   clock is within 10 µs of the reference clock, for up to 5 s. So the time depends on how far each slave's clock
   starts from the reference (one measured start: 42 µs, converging at about 20 µs/s; 2 to 11 s in all on the two
   reference drives). A slave still out of OP after 60 s stops the bus and fails the start, pointing at
   `ethercat slaves` (each slave's state) and the kernel log. A Ctrl-C while it waits stops the bus at once: nothing was
   released, so no brake waits to be confirmed.

| Option | Meaning |
|---|---|
| `--master N` | The IgH master number (default 0) |
| `--rt-cpu N` | The CPU the bus loop is pinned to. Default: the last isolated CPU (`/sys/devices/system/cpu/isolated`). A CPU this process may not run on is refused at start; whether it is isolated is not checked |
| `--monitor` | Print the bus timing beside it, one line a second (section 9) |
| `--plot` | Plot every cycle: start `rt_raw_relay` and PlotJuggler on this bus (section 9). Refused before anything starts when PlotJuggler or its ROS 2 streamer `plotjuggler_ros` is not installed |
| `--unapproved` | Run any file in bench mode, its approval record unused, without the bench procedure (item 1) |
| `--verbose` | Show every node's log. By default the terminal gets a short summary before the start, then warnings, errors, the bus node's own lines (waiting for OP, bench evidence) and the spawners' lines (controllers loaded and active): `run` sets the ROS log level (`--log-level warn`) and `PLEM_LOG_LEVEL=warn`, keeping a value already set in the environment (one that is not a level is reported and dropped). PlotJuggler's output goes to `plotjuggler.log` in the run's directory, named in the summary. The bench's own run is always verbose: its log is kept for diagnosis |

The bus loop runs at `SCHED_FIFO` priority 98, its default; `run` has no option to change it.

To run two buses on one Jetson, start two `plem drive run` processes, one per master. [`docs/install.md`](install.md),
section 2, says how to set them up and run them.

The axis's `mode` picks its controller:

| Axes | Controller | Interface |
|---|---|---|
| `csp` | `joint_trajectory_controller` (open loop; every trajectory ends at rest) | `/<bus.name>/joint_trajectory_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`). `plem.drive`'s `Bus.move_to` (below) uses it, and so can any other `FollowJointTrajectory` client (MoveIt speaks it; not tried here) |
| `csv`, `cst` | `stream_controller` | `/<bus.name>/stream_controller/commands` (`std_msgs/Float64MultiArray`, best effort): one value per joint, in the order of the controller's `joints` parameter, in that axis's unit (rad/s, Nm) |

When the stream goes silent, the stream controller stops writing 50 ms after the last message, and the bus's
own 50 ms command watch then latches the protective stop, about 100 ms after the last message. Deactivating the
stream controller (`ros2 control switch_controllers`) also leaves no command, so the watch latches the protective
stop of the whole bus, every axis, about 50 ms after the switch. On a bus with no torque axis the watch runs only
while the bus is released, so before the release (when nothing moves) neither latches. On Jazzy, if every command
interface is locked at that moment, the deactivation fails and logs it, and the last commands stay: stop the bus.

Before the first message a velocity axis is commanded 0 rad/s. A torque axis gets nothing, so on a bus with a torque axis start the stream
before the release (section 5). A cycle writes the bus only when every axis has a command, so
on a mixed bus a silent stream holds back the position axes' commands too.

The bus also has its own interfaces:

| Interface | Type | Meaning |
|---|---|---|
| `/<bus.name>/joint_states` | `sensor_msgs/JointState` | Measured position, velocity, torque (from `joint_state_broadcaster`, at about 100 Hz, the cycle rate when slower; every cycle is on `rt_raw`); names are the file's axis names |
| `/<bus.name>/plem_drive/status` | `plem_drive_msgs/DriveStatus` | Protective stop, `operation_enabled` (every axis in Operation enabled and in its declared mode), brakes, `statusword` (0x6041). A level report, not a timing source |
| `/<bus.name>/plem_drive/rt_events` | `plem_rt_msgs/RtEvent` | Events. An event's time is its `monotonic_ns` (`header.stamp` is the publish time) |
| `/<bus.name>/rt_raw` | `plem_rt_msgs/RtRawBatch` | Every control cycle, up to 40 cycles (or 40 ms) per message, best effort: measured values, `applied_command` (the target actually sent that cycle, in the axis's mode unit, after clamping and rounding), `command_position`/`command_velocity` (the command given to `csp`/`csv` axes), timing, working counter, `dc_sync_monitor_ns` (always 0 with `sync_mode: drive_default`; 4294967295 = no value that cycle). Every axis of the bus (up to 8); `axis_count` says how many |
| `/<bus.name>/plem_drive/info` | `plem_drive_msgs/DriveInfo` | Published once and latched: the scanned mapping digests and each axis's `modes_of_operation` (10 `cst`, 8 `csp`, 9 `csv`) |
| `/<bus.name>/plem_drive/control_command` | `plem_rt_msgs/srv/SetControlCommand` | `release_brake` = enable. `protective_stop` = stop (section 6: on a bus without a torque axis a ramp to rest, then the brakes; otherwise disable at once). `reset_error` = drive fault reset and protective-stop clear (asynchronous: watch `status`). `free_drive` is always refused. `protective_stop` with `release_brake` or `free_drive` in one request: the stop runs; `release_brake`, `free_drive` and `reset_error` in that request are ignored, and the response is `success: false` naming them. |

The controller manager runs on its own clock beside the bus loop, so the loop can read a sample late; how a `csp`
command is kept from tripping the one-cycle step check then is in
[schema §4.2b](cia402-drive-model.md#42b-mode--the-axiss-operating-mode-and-command).

From Python, with `from plem.drive import Bus, SCurve`:

```python
from plem.drive import Bus, SCurve

with Bus("my_drive") as bus:              # the file's bus.name; leaving the block stops the axes
    print(bus.axes, bus.positions)        # axis names; positions in rad
    bus.reset()                            # clear a fault latched when the bus last stopped
    bus.release()                          # brakes off; the axes hold where they are
    start = bus.positions
    bus.move_to({name: start[name] + 0.5 for name in bus.axes},
                velocity_scale=0.2, acceleration_scale=0.2)   # returns on arrival
    bus.move_to(start, profile=SCurve())
```

Velocity and torque axes take a command each cycle from your control loop (here one axis, `j0`):

```python
import threading, time
from plem.drive import Bus

with Bus("my_drive") as bus:
    command = {"j0": 0.0}                  # per stream axis: rad/s (csv) or Nm (cst)
    running = True

    def control():                         # your loop: each call is one cycle's command
        while running:
            bus.stream(command)
            time.sleep(0.005)

    loop = threading.Thread(target=control)
    loop.start()                           # a bus with a torque axis releases only on a live stream
    try:
        bus.reset()                        # with the stream live (a silent stream's stop would stay)
        bus.release()
        command["j0"] = 0.3
        time.sleep(2.0)
        command["j0"] = 0.0
        bus.stop()
    finally:
        running = False                    # an exception above must end the loop too
        loop.join()
```

No thread re-sends `stream()` for you: call it every cycle. When the calls stop (your loop stalls, or dies on an
exception), the bus takes the protective stop described above about 100 ms after the last one, plus a controller
cycle and the message's delivery; on a bus with no torque axis, only while it is released.

Cancelling a FollowJointTrajectory goal yourself (MoveIt, a CLI) is not a stop: the trajectory controller holds the
measured position at once. Stop a moving bus with `protective_stop` (`Bus.stop()`).

On a bus with a `csv` or `cst` axis, stream first, then reset, and keep `stream()` running (from a thread) through
`release()` and `reset()`. `close()` alone detaches without stopping: released axes stay released, holding; leave the
`with` block (or call `stop()`) to stop.

<details><summary>The full <code>Bus</code> API</summary>

- `Bus(name, *, timeout=10.0)` connects to the running `plem drive run` of the file whose `bus.name` is `name` (a
  leading `/` is accepted). It waits for its robot description, joint states and status; with no such bus it
  raises `DriveError` naming the bus once the timeout passes. `bus.axes` (names), `bus.positions` and
  `bus.velocities` (dicts), `bus["axis"].position`, `.velocity` and `.move_to(x)`; `bus.status` is the bus's
  latest `plem_drive_msgs/DriveStatus` (`operation_enabled` as a bool, the `brake_engaged` and `protective_stop` bit
  masks, `statusword`).
- `reset()` sends `reset_error` (drive faults and the protective stop) and waits until the bus reports neither. It
  raises `DriveError` naming the protective stop's source and the axes in fault. The source is INTERNAL (a limit
  still broken) or EXTERNAL (the bus, the command stream or a drive leaving Operation enabled or its declared mode); the cause itself is on
  `rt_events`. Before the request it logs the axes in fault as a ROS warning (`<bus>: resetting axisN in fault
  (statusword 0x…, error 0x…)`, the error code when the bus reported one). On a bus with a `csv` or `cst` axis,
  stream first, then reset. Once this client has streamed, keep
  `stream()` running (from a thread) through `release()` and `reset()`. Stream 0 rad/s to hold a `csv` axis; on a bus
  with a `cst` axis the streamed command takes effect at the release. If the bus, after the request, still reports
  the EXTERNAL stop while this client's stream is silent, both raise `DriveError` at once saying so, rather than
  waiting out the timeout.
- `release()` (on a bus with position axes) first waits, up to the timeout, until the trajectory controller is active
  (its spawner starts it after the bus is up; a controller you deactivated yourself must be activated again first). It
  then re-activates the controller, so it holds where the axes are rather than where an earlier goal left it. Then it
  releases the brakes and waits until every axis is in Operation enabled with its brake released. On a file with
  `fault_reset.policy: none`, an axis in fault (CiA 402 Fault; many drives latch an alarm when the bus stops) fails it
  at once, before or after the request, naming the axes and their statusword: call `reset()` first. The client never
  resets a fault itself; under `bounded_auto_reset` the bus does, and `release()` waits for it. A release that the bus
  refuses, or that trips (a slave not in OP, an axis not in its declared mode), fails once the bus settles. From the
  second `status` message after the service responds, a status with brakes engaged and no brake operation pending is
  taken as the release's outcome (a refusal). While an earlier brake operation is still pending, that status does not
  decide, and the release may wait out the timeout. The error names the
  `SLAVE_NOT_OPERATIONAL`/`MODES_OF_OPERATION_MISMATCH` events received since the request, or says to see `rt_events`.
- `move_to({axis: rad, …}, velocity_scale=1.0, acceleration_scale=1.0, profile=None)` (scales in (0, 1]) plans one
  move of every position axis, sends it and returns on arrival. Axes not named keep their positions. Before anything
  moves, it raises `ValueError` if a name is not a `csp` axis, or a planned point leaves the file's position
  limits, velocity (`velocity_cap_rad_s`) or acceleration (at the points that carry one, and as the average speed
  change between points — the controller's spline between them is not sampled); an unknown name raises
  `KeyError`. It raises `DriveError`, with the bus's reason, when:
  - the axes are not released;
  - the controller refuses or fails the move;
  - a `stop()` cancels it;
  - the bus stopped on its own before the axes arrived;
  - the bus stopped reporting its status or joint states for a second. If this happens before the move is sent,
    nothing is sent; if it happens during the move, check whether `plem drive run` is still up: if it is, the move
    continues, so call `stop()`;
  - the axes are not within 1 mrad of their goals by the timeout (a coarse encoder may not settle that close).
- `stream({axis: value, …})` sends one cycle's command to every stream axis (`csv`, `cst`) by name, in rad/s on a
  `csv` axis and Nm on a `cst` axis; the client puts them in the stream controller's order. Before sending it raises
  `ValueError` if a value is outside the file's limits (`limits.velocity_rad_s` on a `csv` axis, `limits.torque_nm`'s
  min and max on a `cst` axis; a non-finite value too), a name is a position axis, a stream axis is missing, or the
  bus has no stream axis. An unknown name raises `KeyError`. A `cst` axis whose robot description has no torque
  limits raises `DriveError`: restart `plem drive run`. Nothing is sent
  then. Each value is checked alone; a `csv` axis's acceleration is not. It may be called before `release()`, and on a bus
  with a torque axis it must be (section 5).
- `stop()` calls `protective_stop`, cancels the running move and waits until every brake is engaged. On a bus
  without a torque axis the axes decelerate from the plan's speed; with one, every drive is disabled at once
  (section 6).
- Leaving the `with` block calls `stop()`, then closes the client, however the block ends (Ctrl-C included). On a
  normal exit a failing stop is raised; on an exception it does not replace the exception. `close()` alone detaches
  without stopping: released axes stay released, holding.
- Profiles: `Trapezoid()` is the default (`bus.default_profile`); `SCurve(jerk=None)` is jerk-limited (by default
  each axis reaches its acceleration in 0.1 s). A move of several axes is one straight line in joint space: they
  start and stop together. Only the segment boundary points are sent, and the trajectory controller's spline between
  them follows each segment. Your own profile subclasses `Profile` and implements
  `plan(start, goal, limits, scale) -> trajectory_msgs/JointTrajectory`, starting at `start` at t = 0 and ending at
  rest at `goal`. Its points, and the speed change between them, get the same limit check; what the spline does
  between points is the profile's own responsibility.

</details>

## 5. Release the brakes

```bash
ros2 service call /my_drive/plem_drive/control_command plem_rt_msgs/srv/SetControlCommand '{release_brake: true}'
```

(`Bus.release()` does the same.) With `fault_reset.policy: none`, a drive in fault does not enable: many drives latch an alarm when the bus stops, so
call `reset_error` first and wait for `status`.

On a bus with a torque (`cst`) axis, order matters: **start the stream first, then release**: 0 Nm on a torque axis,
0 rad/s on a velocity axis, one value per stream joint:

```bash
# keep streaming (one value per stream axis; for one torque axis, 0 Nm), then release in another terminal
ros2 topic pub -r 200 /my_drive/stream_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.0]}'
```

On such a bus a release without a live command is refused, with the reason in the response. A command whose length
is not the axis count, or that holds a non-finite value, is not accepted; sending only that is the same as sending
nothing. `ros2 topic pub` reads YAML, where `1e-05` is not a number: write `1.0e-05`.

On a bus with only position and velocity axes the release does not wait for commands. From the release until the first command after it,
a position axis holds the position it found at the release (it does not follow a sagging axis) and a velocity
axis 0 rad/s. Commands sent before the release are not applied. The 50 ms command watch starts with
that first command. A release is refused while a stop is still running (section 6).

A release the bus accepts can still be refused a cycle later, at the readiness gate: the response says
`success: true` while the brakes stay engaged. Confirm the release on `status` (`operation_enabled`, brakes) and read
the cause on `rt_events` (the bench prints it in its timeout messages). `RtEvent` names each event code as a constant
(`RtEvent.SLAVE_NOT_OPERATIONAL` in Python, `RtEvent::SLAVE_NOT_OPERATIONAL` in C++); compare against those, not the
numbers:

| `event_type` | Name | Fields | What to do |
|---|---|---|---|
| 98 | `SLAVE_NOT_OPERATIONAL` | `joint_id` = axis, `error_code` = the slave's AL state (0 offline, or the state could not be read; 1 INIT; 2 PREOP; 4 SAFEOP; 8 OP with a reconfiguration pending), `value` = ring position | `plem drive run` comes up only once every slave is in OP (section 4), so the slave has usually left OP since (8: a reconfiguration is still pending). Check the kernel log (`sudo dmesg \| grep -i ethercat`) for an AL status code, or a refused startup SDO (`startup_sdo`, or `dc.write_cycle_time`'s 0x1C32:02) |
| 95 | `MODES_OF_OPERATION_MISMATCH` | `joint_id` = axis, `error_code` = statusword, `value` = 0x6061 display | The axis has no frame yet, an unrecognized statusword, or a display other than its declared `mode`. Check `mode` and `mode_display_timing` (section 2). The same event names an enabled axis that leaves its mode (a protective stop) |

Position and velocity commands are **not clipped.** A position command outside the position limits or farther than
one cycle's travel (`velocity_rad_s` × cycle) from the last one, or a velocity command above `velocity_rad_s`,
causes a protective stop. Both are judged after conversion to drive counts, so a value right at a limit can fall
either side by rounding. The first position command after release is compared with the position held since the
release: farther than one cycle's travel causes a stop, closer **is applied and the axis moves**. A trajectory planned
within `velocity_cap_rad_s` (as `Bus.move_to` does) stays inside the step check.

## 6. Stopping

| Situation | What happens | Who does it |
|---|---|---|
| Commands stop arriving | Stale command → protective stop → neutral on every axis → forced brake engage | PLEM (50 ms after the last command) |
| A position or velocity command exceeds a limit | Protective stop in that cycle (`csp` axes keep the previous cycle's target, the others go neutral) → neutral and forced brake engage from the next cycle | PLEM |
| Ctrl-C (SIGINT) or SIGTERM to `plem drive run` (the controller manager) | Stop (on a bus without a torque axis the axes first ramp to rest, as for `protective_stop` below) → brakes engaged → exit. An engage it cannot confirm is logged as an ERROR with the mask of the engaged brakes; this is always the case for a drive still in Fault at the end of the stop (call `reset_error` and wait for the status to leave Fault first) | PLEM |
| The process is killed (SIGKILL) | Communication stops and the drive's output sync manager watchdog acts | **The drive**, by its own setting, outside PLEM. A declared mapping must enable the watchdog to pass, but its time is the drive's default; a `scanned` slave is not checked at all. **Do not read this as a guarantee** |
| You just want to stop | `protective_stop` service call (`Bus.stop()`). On a bus without a torque axis it is the requested stop below (a ramp to rest, then the brakes), after which it can be released again without a reset unless it times out (below); with a torque axis the drives are disabled at once. The service answers at once; watch the brakes in `status` | PLEM |

A stop on a limit, a command gap or lost communication is the standard CiA 402 disable sequence, not Quick stop. So
is every stop on a bus with a torque axis. Deceleration, brake timing and the behaviour of a gravity-loaded axis
differ per drive. **On that path PLEM does not slow a moving axis down before the disable.** The drive decides how it
stops: on Disable operation by its disable operation option code (0x605C) and deceleration (typically 0x6084), on
lost communication by its fault reaction (0x605E, typically at 0x6085). Set them for your drive in
`startup_sdo`. With slow factory values, an axis stopped at speed keeps turning for a long time ([`docs/hardware-evidence.md`](hardware-evidence.md)).

On a bus of position and velocity axes, `protective_stop` and the end of a run (SIGINT, SIGTERM) are a requested
stop. PLEM ramps each command from its current speed to rest at `limits.acceleration_rad_s2`, never back past rest,
and ignores your commands meanwhile. Once the commands rest and every measured speed is within 1 % of its velocity
limit, the brakes engage.

If the stop is not confirmed within the planned deceleration + 0.5 s, every drive in Operation enabled gets Quick stop.
It slows at its own 0x6085 and ends in the state its 0x605A selects (2 = Switch on disabled), which latches a
protective stop (`RAMP_STOP_TIMEOUT`, 0x4F, in `rt_events`, value = seconds since the request; reset first).
After 2 s more, a drive still in Quick stop active gets Disable voltage: if its 0x6085 has not
stopped the axis by then, the axis loses power while moving. So set 0x6085 and 0x605A in `startup_sdo` too, and keep
`acceleration_rad_s2` no higher than 0x6085 (schema §4.10). The bench measures this stop once from cruise
(`stop-decel`, section 8). The requested stop ran in the sixth session ([`docs/hardware-evidence.md`](hardware-evidence.md))
without reaching its Quick stop fallback, which has not run.

A position axis's neutral is that cycle's measurement, not a promise to hold a fixed position. 0 rad/s on a velocity
axis is no evidence of zero torque or an engaged brake. `limits.torque_nm` is a `cst` axis's key (required there,
refused on `csp` and `csv` axes) and clips torque commands; the torque a drive produces in `csp` or `csv` is bounded
by the drive's own 0x6072. A stop caused by a limit emits one `LIMIT_STOP` (0x4E) event per axis in `rt_events`:
`joint_id` = axis, `error_code` = reason (schema §4.2b), `value` = the offending value.

## 7. Recover from a protective stop

1. On a bus with a torque axis, start streaming again: a reset without commands trips again at once. Resending the
   command that broke a limit trips again (on a bus of position and velocity axes, once it is applied after the
   next release).
2. Call `reset_error`.
3. Wait until `protective_stop` in `status` reads 0 (the request is asynchronous).
4. Call `release_brake` (`Bus.release()`).

An axis stopped outside its position limits cannot be released this way: reset is refused while it is outside, and
the drive tool cannot move it back, so recover the axis outside PLEM.

## 8. Bench and approval

Only the file's `approval` block enables motion, and the software judges that record. The bench measures the
file at the drives and records the approval in the file itself. On a terminal, `plem drive run` offers the bench for
a file not approved for motion:

```text
$ plem drive run "$HOME/drives/my_drive.yaml"
/home/…/drives/my_drive.yaml is not approved for motion. Starting the drive bench.
```

The bench has run on hardware since the sixth session ([`docs/hardware-evidence.md`](hardware-evidence.md)), on two
A6B drives: position axes, coupled and free, and a `csv` copy with the outputs free. It runs on the `plem drive run` chain; its tests use a simulated bus.

1. **Confirm.** The bench asks one question (`[y/N]`; no, end of input or Ctrl-C touches nothing): whether you are at
   the drive with the emergency stop in reach, gravity-loaded axes are propped, axes not in the file are held by their
   own brakes or mechanics, and no brake is shared by several axes. On a bus with a position or velocity axis and no
   torque axis it also says that each such axis **moves** once, up to 0.5 rad either way from where it rests. Clear
   the workspace on both sides.
2. **Bench.** It starts the bus the way `plem drive run --unapproved` does (the controller manager as its child) and
   releases and engages the brakes three times, or four on a bus with a `csv` axis and no `cst` axis. A release
   holds every axis where it rests: a position axis at its position, a velocity axis at 0 rad/s, a torque axis at
   the 0 Nm the bench streams. A released axis held at rest that moves more than 0.005 rad aborts and engages.
   Every axis must start inside the file's position limits. The one motion is `stop-decel` (schema §6.1), on a bus
   with no torque axis.
3. **Result.** On success the approval, with the bench evidence inside it, is written into the model file. Its
   `approval` block is replaced, or added at the end when it has none; every other byte is kept, and the write is
   atomic (a temporary file next to it, then a rename). Otherwise it leaves `my_drive.bench.log`, whose path it
   prints, and the model file as it was: bench again. The log is rewritten on every confirmed run (a refusal or a
   decline writes nothing), so move a log you need before running again. The recording step judges the approval; the
   bench only carries measurements.
4. **Start.** `run` checks the file again and starts the bus. If the live drive (serial number, revision number,
   0x1009, 0x100A, the scanned mapping) differs from the record, the bus refuses before activation.

An approved file is compared with the drives on the bus before every start: the bus's own identity check, then the
master is released; nothing is configured or moved. Another unit of the same model (only an identity value differs) is
benched like an unapproved file on a terminal, and the bench replaces the approval; without a terminal `run` refuses
with the reason. Another model, a missing drive, or drives in another order than at the bench (restore the cabling)
are refused with the bus's own error message. `--unapproved` runs any file, an approved one included, in bench mode:
its approval record is not used, so no identity is checked.

The required bench items follow the axes' modes (schema §6.1); `plem drive check --json` prints the file's
`required_bench_items`.

The bench refuses before touching the bus if the master is in use. A file whose approval no longer holds (edited
after its bench), or an approved file benched again (other drives), is benched like an unapproved one and its record
replaced; the confirmation says so. It shows `check`'s warnings first and does not stop on them. The model
file must be yours, and both it and its directory writable. If you edit the file while it is being benched,
recording is refused (the evidence no longer matches it).

> **One file.** The approval lives in the model file; there is no second copy to keep. An approval survives software
> updates. Changing a value, replacing the drive, or an update that raises the bench requirements voids it; then
> bench the file again.

Approval judges that the records (the file, the bench evidence it carries and the live drive) agree, not that
the measurements are true. At start only the recorded fields are compared: a revision number or version
left `null` is not compared, so the approval still holds after that field changes (another firmware, for a version).
A drive whose serial number is 0 or not recorded has no unit to bind to. Its approval applies to every drive with the
same vendor and product code (and, for a `scanned` slave, the same mapping), within the fields the record keeps.

## 9. Watch the bus

**Levels and events:**

```bash
ros2 topic echo /my_drive/plem_drive/status
ros2 topic echo /my_drive/plem_drive/rt_events
```

**Timing — `plem drive run --monitor`.** One row a second from `rt_raw` in `run`'s terminal, under a
header repeated every 20 rows. Each change of the bus's state gets a line of its own (`HH:MM:SS  -> <state>`), and
Ctrl-C prints the total. On a terminal, rows with overruns, working-counter errors or lost samples are yellow, and a
state line for a fault, a protective stop or a brake not confirmed on is red (`NO_COLOR` turns colour off).

Columns, each name with its unit (`/s`: counted in that second):

- `time`
- `exec-mean[us]`, `exec-max[us]`: loop execution time
- `pdev-p99[us]`, `pdev-max[us]`: period deviation = |actual − nominal| wake-to-wake period, not lateness against a schedule
- `overrun/s`: execution longer than the period
- `wkc-err/s`: working-counter errors; every cycle while a slave is not yet in OP
- `lost/s`: gaps in the sample sequence; a restarted bus, detected when the sequence goes back, starts a new count
- `dc-max[ns]`: the DC deviation max
- `state`: the last `status` in words: `every statusword 0 (drives not reporting yet)`, `held, brakes on` (every
  brake confirmed engaged), `brake operation pending`, `released`, and in red `protective stop INTERNAL|EXTERNAL (…)`,
  `axisN fault (statusword 0x…, error 0x…)` (the error code when the bus reported one), `axisN brake not confirmed on`

The total covers the whole run, the wait for OP included: samples, lost, execution max, period deviation max, p99 and
p99.9, overruns and working-counter errors (every cycle of that wait counts as one). Percentiles use 0.1 µs bins and
report the bin's upper edge.

**Every cycle in PlotJuggler: `plem drive run --plot`.** `rt_raw` packs up to 40 cycles (or 40 ms) into one message. The relay
republishes one message per cycle on `/<bus.name>/rt_raw/samples`, stamped with that cycle's RT time; it runs with
normal scheduling and does not touch the control loop. `--plot` starts the relay and PlotJuggler beside the bus, on a
layout built from this bus's axes, and both end with the bus. Install PlotJuggler and its ROS 2 streamer once:

```bash
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
plem drive run "$HOME/drives/my_drive.yaml" --plot
```

The layout uses the message's `header.stamp` as the time axis and has four tabs: position (one pane per axis, titled
with its name: `actual_position`, `applied_command`, `command_position`), velocity and torque (every axis),
timing (`loop_exec_us`, `loop_jitter_us`), and bus (`wkc_mismatch`, `deadline_miss`, `dc_sync_monitor_ns`). The
`a6b_pair` example adds its own layout with a sync tab for its mirrored pair (section 10).

The first time, PlotJuggler may ask questions (the header-stamp choice is not checked on a live bus). To start the
previous streaming plugin, answer Yes. In the ROS 2 subscriber dialog, select `/<bus.name>/rt_raw/samples` and keep
the header stamp on. Asked about missing curves, create empty placeholders. Export CSV from PlotJuggler. Measured
position, velocity and torque alone are on `/<bus.name>/joint_states`, no relay needed.

**Messages that look like errors in normal use.** These showed up with ROS 2 Jazzy and the A6B example:

| Message | When | What it is |
|---|---|---|
| `[deprecated] 'open_loop_control' parameter is deprecated` | Every start of a bus with a position (`csp`) axis | The trajectory controller's warning about a setting `plem drive run` gives it; Jazzy still applies the setting |
| `[….pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!` (printed twice, as ERROR) | At Ctrl-C (without `--monitor`, the last lines) | ros2_control's statistics thread ending with the process. The stop and brake engage come before it; a failed engage is logged as an ERROR of its own (section 6). `ethercat master` then shows `Phase: Idle` |
| `no status yet` in the `--monitor` state | Before the bus's first status message | The bus is still starting; the state follows once the drives report |
| `[….controller_manager]: Overrun detected! The controller manager missed its desired rate of 1000 Hz. The loop took … ms (missed cycles : …)` (WARN) | Seen once while the axes were held, as another terminal started `ros2 topic echo` | The controller manager runs beside the bus loop (section 4) and ran late; the bus loop's own overruns are the `--monitor` `overrun/s` column (0 then). What delayed it was not measured. A controller manager 50 ms late while the axes are released latches a protective stop (section 6) |
| `0x1009: b'…' cannot be carried by a record … recorded null` (also `0x100A`) | During the bench | The drive's version text cannot go into the approval record; the approval binds to the identity values the bench could read (a `PASS slave N` line follows) |
| `axisN fault (statusword 0x…, error 0x…)` in the `--monitor` state | From the start until a reset, when an earlier stop left the drives in fault | A latched drive fault, often the communication alarm a bus stop leaves (on an A6B, 0x603F reads 0xFF50 and the display shows Err80.4; the kernel log has `Sync manager watchdog`). **Read the code before resetting**: the state shows it (and `bus.reset()` logs it in a warning as it clears), or, when none is shown, `ethercat upload -p <ring position> -t uint16 0x603f 0` reads it. Any other code than the stop alarm 0xFF50 is a real drive alarm. Then send `reset_error` (`bus.reset()`); with `fault_reset.policy: none` nothing else clears it. `bus.reset()` clears whichever fault it is, and the A6B demo calls it first |

## 10. Two-axis example — Panasonic A6B

`a6b_pair/` runs two Panasonic MINAS A6B drives on one bus, one position axis each, on distributed clocks. On the
maintainer's bench their gearhead outputs are coupled so that they turn as mirror images ([video of the
bench](media/a6b-pair-demo.mp4), 35 s, no sound). Its [`README.md`](../plem_drive/examples/a6b_pair/README.md) has the
steps: the per-drive checks, every value of `drive.yaml` with its source, the bench, the plots and `demo.py`, which
moves the pair mirrored.

## Not supported

- **Modes are `cst`, `csp` and `csv`, fixed at start.** No runtime mode switch, trajectory interpolation in the bus
  loop, feed-forward (0x60B1, 0x60B2) or homing; Quick stop only as the fallback of a requested stop (section 6).
  Trajectories are made on the host: by the trajectory controller and `plem.drive` under `plem drive run` (section 4),
  or by your own stream client. `cst` does not hold position.
- **The encoder must be absolute** (section 2). With no homing, an incremental encoder's position restarts at every
  power-on and the position limits no longer guard the range they name. The file has no key for this, so the software cannot
  refuse such a drive. Do not use one.
- **Only a holding brake that the drive itself releases and engages with the servo is supported.** A brake opened by a
  0x60FE digital output, a brake controlled outside the drive, or no brake is not supported. The file has no key for
  this, so the software cannot refuse such a drive. Do not use one.
- **Do not use a drive where one brake releases several axes.** Brakes are treated as independent per axis. There is
  no key to declare sharing and no way to detect it, so one axis's brake action could release another axis. Several
  axes on one drive with independent brakes are supported (up to 8, not verified on hardware; the second channel's
  objects are at +0x800, e.g. 0x6841).
- **No gravity compensation** and no hand guiding (`free_drive`) in the drive layer.
- **No IO slaves.** The bus configures only the drives in the file and leaves other slaves unconfigured (PREOP).
