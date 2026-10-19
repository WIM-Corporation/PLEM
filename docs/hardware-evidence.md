# Hardware evidence

The hardware runs of the drive software: the setup, what was observed, and what has not run yet. Section numbers are
those of [`docs/getting-started.md`](getting-started.md); "schema" is the drive model file reference, [`docs/cia402-drive-model.md`](cia402-drive-model.md).

All sessions ran on Jetson AGX Orin with JetPack 7.2.1 and ROS 2 Jazzy. The earlier sessions used an Intel I210 with
IgH's native driver (Case 2); the last three ran the same drives on a freshly flashed devkit using the built-in RJ45
port (Case 1; [`docs/install.md`](install.md#supported-platform)). The first five sessions ran a predecessor front end
on an earlier build of the RT layer; the sixth and the later ones ran `plem drive run`.

## Earlier sessions (predecessor front end)

That front end took one command stream for the whole bus, had an RT trapezoidal ramp for `csp` axes, and had its own
bench, monitor and demo position publisher ("the front end", "the bench", "the monitor" and "the demo" below). Setup:

- Two Panasonic A6B MDDLN55BE drives (product code 0x60380009 — the example's drive since the third session) with
  MSMF102L1H5 motors (holding brake, battery-backed absolute encoder), each driving a 21:1 planetary gearhead (Nidec
  ABLE VRG, frame size 120). The files used `gear_ratio: 1.0` until the last item of the fourth session, so the values
  before it are at the motor shaft (positions, speeds and accelerations ÷ 21 for the output, torques × 21).
- Case 2 network (Intel I210, native EtherCAT driver), 1 ms, `sync_mode: drive_default` — except the distributed-clocks runs
  below.
- Drive software version (0x100A) V1.00 on both. Nothing beyond the gearheads was measured as a load.
- Drive model files derived from the A6B example (declared PDO mapping, `csp` with the ramp), a source build. In the
  first two sessions the derived files raised the limits with the peak — up to ±45 rad, 240 rad/s and a 640 rad/s²
  ramp. In the third the example ran with 640 rad/s² stopping decelerations and its zero offsets set, and for the
  cruise and deceleration steps with the limit at 1.25 × the cruise.
- Started in the front end's bench mode (its counterpart of `plem drive run --unapproved`).

The front end took `protective_stop` as an immediate disable (the path a limit or a command gap takes) and watched
commands at 100 ms, so the protective stops below show only that immediate path. The requested stop (section 6) and
the 50 ms command watch (section 5) ran from the sixth session on; the release hold ran there only on position axes
whose controller wrote from the next cycle.

### First to third sessions

- 0x6061 reads 8 at rest (`mode_display_timing: pre_enable` is right for these drives).
- Enable, a 15 s hold without motion and the brake engage on stop completed.
- Both axes followed ramped position commands in opposite directions. Peaks were 5 to 160 rad/s with no drive alarm
  while moving. The two timed runs had no lost cycle and a p99 cycle jitter of 0.7–1.2 µs. The following error was about 30 ms × velocity. Exceeding the
  following error window (0x6065) only set statusword bit 13 — no alarm.
- A velocity-limit protective stop (`LIMIT_STOP`, reason 2) came from the drive overshooting a ramp that cruised at
  its limit.
- **A protective stop at speed did not slow the axes down.** With factory 0x6084/0x6085 (about 0.75 rad/s²), a stop
  requested while an axis slowed through about 46 rad/s left it turning at about 30 rad/s for about 9 s, until the
  front end exited. The lost bus then raised a communication alarm, and the drives slowed down, consistent with the
  factory rate (0x605E = 2 uses 0x6085): the axes came to rest after about 98 and 44 revolutions in all. That is why the example sets them (section 6).
- With the example's stopping decelerations (0x6084 = 0x6085 = 640 rad/s², a second run), protective stops at
  speed stopped both axes together. From about 1, 9, 36 and 144 rad/s, the axes were below 0.05 rad/s within 14, 59,
  115 and 289 ms, after 0.005, 0.33, 2.2 and 21.5 rad. The two axes' distances differed by at most 0.07 rad. The
  distance matches v²/2a plus the drive's following lag, about 30–45 ms of travel.
- Third session, ramp cruise `velocity_rad_s` at 2000 rad/s²: over cruises of 50, 100, 200, 300 and 366 rad/s the peak
  measured speed was 4.5–5.8, 1.2–2.0, 0.5–1.0, 0.5–0.7 and 0.5–0.7 % over the cruise, the following error 1.6 to
  11.7 rad (about 32 ms × velocity), the torque at most 1.3 Nm, with no drive alarm and no `LIMIT_STOP`. The example's
  file peaked at 368.7 rad/s under its 418 rad/s limit. The auto-tuning (Pr0.02 = 1) had been on through the
  earlier runs; the shaft load was not measured. In the 366 rad/s step (limit 457.5 rad/s), a protective stop at
  332 rad/s had both axes below 0.05 rad/s (the same criterion as above) in 0.58 s after 98 rad, within 0.08 rad of
  each other. The working-counter mismatches of those runs were all in the
  first 8 s, with the axes at rest.
- Third session, stopping deceleration steps (0x6084 = 0x6085, limit 1.25 × the cruise, a protective stop at
  0.9 × the cruise, no load): from about 46 rad/s at 640, 1280, 2560 and 3200 rad/s² the axes were below 0.05 rad/s
  after 3.4, 2.4, 2.0 and 1.8 rad; from about 140 rad/s at 1280, 2560 and 3200 after 12.6, 8.3 and 7.6 rad; from
  about 340–367 rad/s after 55.7, 37.5 and 29.7 rad (185 ms at 3200). The two axes were about 0.2 rad or less apart,
  with no drive alarm. Counted from the stop service call (so including the host's path to the bus), the speed stayed
  above 95 % of its value for 16 to 45 ms over the steps, 16 to 30 ms at 3200; from there to
  rest, the speed divided by the remaining time was, at 3200, about 660 rad/s² from 46 rad/s, 1310–1560 from 140 and
  2220 from 344 — the drive's command smoothing is the likely reason the stops take longer than the setting implies
  (not measured). The example writes 3200 rad/s²,
  near the top of 0x6084's U32 range at 1:1.
- Third session, the bench on the example with 3200 rad/s² decelerations and its zero offsets set, the drives' alarms cleared
  first: every required item passed and the bench wrote the approved file (`identity_scope: unit`). Under the neutral
  command the released axes drifted at most 1.2e-5 rad; after the command stream was killed the first brake engage
  came 1.0 ms after the stale event. 0x1009 and 0x100A read `V1.00` padded with spaces and two trailing NUL bytes, which a record cannot carry,
  so the record leaves both versions null — of the identity fields, the approval checks only the serial number and the revision number. The approved
  file then started without bench mode: both drives matched the record, and after one `reset_error`,
  `release_brake` was accepted and ±1 rad commands ran.
- Most exits of the front end (14 of 19 observed) left a communication alarm (Err80.4, 0x603F = 0xFF50; AL status 0x001B, sync
  manager watchdog) on the drives. The front end stopped the cyclic exchange with the drives still in OP and left their exit
  from OP to IgH 1.6.13; when that transition (a bus rescan, an SDO dictionary read, a few times a cause not found)
  outlasts the drives' process-data watchdog (100 ms, ESC 0x0400/0x0420), the alarm fires. The next run cleared it with `reset_error` before the release.
- One drive: power-cycling it left the next drive's mailbox unanswered. `ethercat states INIT` then `PREOP`
  recovered it; the cause is not known. Separately, an SDO upload right after an exit twice got a reply meant for
  another request: IgH reads each drive's SDO dictionary after a rescan through the same mailbox without serialising
  it with other SDO requests (from the IgH source) — the likely cause of that, not reproduced.
- Synchronization with `sync_mode: drive_default`: the drives ran Free Run, not SM2-synchronous — in OP, 0x1C32:01 read 0 and
  0x0981 read 0 (without DC the A6B keeps its default; PLEM wrote no 0x1C32).
- Distributed clocks (`dc_sync0`, the example's `dc` values, 1 ms, a derived file): 0x1C32:02 read 0 (the A6B's
  unset value). With 0x1C32:02 = 1000000 written by hand (`ethercat download`, RAM) the drives set 0x1C32:01 = 2 and read 0x0981 = 3,
  and reached OP with no drive alarm. `dc_sync_monitor_ns` reached about 1 ms in the first 15–30 s (IgH 1.6.13 sends
  no dedicated static-drift burst and does not reset the drift filter), then p50 0.1–0.3 µs and at most about
  1.5 µs. At 368 rad/s, outputs free, the lag was 33–34 ms against 32 ms in Free Run: DC did not materially change it.
  With the two gearhead outputs joined by the printed jaw (torque limit 30 %, up to 407 rad/s at the motors), they
  pushed against each other with at most 0.37 N·m, their positions at most 0.41 rad apart (about 1.1° at the output,
  inside the jaw's clearance — see the fourth session) and their lags
  equal within 0.3 ms; motion started inside the settling window in every run. No Err81 and no missed cycle. In two
  2-minute runs the deviation jumped once each (151 µs, 393 µs) and decayed over about 16 s with the control loop on
  time; the cause is not known. `dc.write_cycle_time: true` makes the same write.

### Fourth session

Gearhead outputs free unless stated, `dc_sync0` with `write_cycle_time` (PLEM writing 0x1C32:02), the example as
shipped and files derived from it:

- Before the first start one drive held 0x1C32:01 = 2, 0x1C32:02 = 2000000 and 0x6085 = 0xFFFFFFFF from earlier
  writes; after the start both read 0x1C32:02 = 1000000 and the file's 0x6085. 0x605A read 2 and 3.
- The example as shipped (zero offsets 0; the axes rested at about 1142 and −1139 rad): `plem drive check`, then
  the bench passed every required item and wrote the approved file, which then ran without bench mode with
  the demo at ±0.5 rad: both drives reached OP, no new drive fault while enabled, 144,840 cycles with no lost cycle or
  deadline miss, p99 cycle jitter 1.2 µs. The working-counter mismatches were all outside Operation enabled.
- `dc_sync_monitor_ns` (IgH's upper estimate from one broadcast read, not a per-drive value): in the shipped-example
  run below 62 µs in the first 10 s (a later 6-minute run started at 754 µs), then p50 0.2–0.3 µs between excursions. In every run longer than 90 s an excursion came about every
  110 s (a 10-minute run: eight above 100 µs, the largest 721 µs), rising over a few seconds (about 100–120 µs/s at
  first) and decaying within 10–30 s, with the axes at rest or moving; no lost cycle, deadline miss or drive fault
  came with them. A log of the kernel's clock correction over a 6-minute run showed no change at its three onsets.
  The cause is not known.
- Endless axes (`limits.position: null`): both axes swung ±480 rad at up to 375 rad/s and crossed the 32-bit count's
  wrap (±1608.5 rad at 23 bits) six times each, with continuous positions and commands and no `LIMIT_STOP`. In that
  run 0x6064 repeated its previous value in 0.12 % and 0.05 % of the cycles above 20 rad/s (the next cycle carried two
  cycles' travel).
- Endless axes after a long one-way run (a derived `csp` file, `gear_ratio: 21.0`): both axes turned 840 rad one way at the output (about 2900 motor turns) in about 60 s at 14 rad/s with no
  fault and continuous positions, exit 0. At the next start both drives faulted with 0x603F = 0xFF1D (Err29.1,
  counter overflow protection 1) and stayed in Fault after `reset_error`: their multi-turn data read 2914 and −2873
  turns with Pr6.98 = 0, past the ±256 turns a 32-bit position holds at 23 bits with a 1:1 electronic gear (A6B
  manual p.255-256). The earlier endless runs swung both ways and ended inside that range. Not yet run: the same axes with Pr6.98 bit 3 = 1.
- With a wrong `product_code` the start was refused; the error named slave 0's declared and found identity. The DC reference clock was
  slave 0.
- `csv` (0x60FF declared, limit 20 rad/s), one run: ±2 rad/s steps followed, peaks 2.4 and 3.5 rad/s. `cst` (0x6071
  declared, `torque_nm` ±0.3), two runs: at ±0.05 Nm neither axis turned more than 0.01 rad; at ±0.1 Nm axis 0 turned (up to
  6.1 rad/s) and axis 1 did not. The measured torque followed the command (0.054 Nm at 0.051).
- Command checks: a position target outside `limits.position` (±0.5 rad against ±0.3) gave `LIMIT_STOP` reason 4 on
  both axes, a `csv` command above the limit (±25 against 20 rad/s) reason 6; neither command was applied. Axes left
  outside the position limits by an earlier run gave reason 1 at start, release was refused, and `reset_error` was
  rejected (`PROTECTIVE_STOP`, error code 1) — section 7.
- SIGTERM to the front end while released: exit 0 (its stop confirmed every brake engaged; the engage time was not
  measured). SIGKILL while released: both drives faulted by themselves (0x603F = 0xFF58). After a normal exit the
  drives held 0xFF50, cleared each time with `reset_error`.
- The monitor printed its lines and total on the live bus (14.7 s).
- Coupled: the two gearhead outputs joined by the printed jaw (about 1.5 mm total clearance between its teeth). A
  guarded sweep — one axis held, the other stepped — found its two 0.25 N·m opposing-torque points 1.94 rad apart at
  the motor, about 5.3° at the output; over the first 1.5 rad from rest the held axis took almost no torque, and its
  torque rose before the far point (0.05 N·m at +1.6 rad, 0.17 N·m at +1.78). These points are thresholds, not the
  teeth's contact points. `startup_sdo` lowered Pr0.13 to 30 % (0x4FA8/0x4FA9 read ±30 % in Operation enabled: about
  0.95 N·m at the motor, about 20 N·m at the output before the gearhead's losses). A monitor of the recorded stream
  would have taken a protective stop at 0.5 N·m opposing torque or 0.5 rad relative position (it never did); it
  judged only samples with both axes in Operation enabled, measured the relative position from the first such
  sample, and did not detect a gap in the stream. The ramp at 10 rad/s (50 rad/s²) and at 50, 100
  and 200 rad/s (500 rad/s²) each ran 30 s and took a protective stop at 0.9 × the cruise: no drive fault after
  enable, opposing torque at most 0.21 N·m and relative position at most 0.18 rad (motor side). At 366 rad/s the run
  completed (0.33 N·m, 0.37 rad) but the motor torque reached 0.935 N·m, 98 % of the limit, and the stop step was not
  run; its end-of-run protective stop from about 267 rad/s stopped both axes in 170 ms and 20 rad. Much of the 366 rad/s
  relative position came from feedback timing (one axis's 0x6064 a cycle behind). The relative position reached the sweep's near
  threshold in none of the 10–200 rad/s runs and in 0.75 % of the 366 rad/s run (a count sensitive to that feedback
  timing): these runs show the two drives moving together, not the jaw carrying load.
- The example with `gear_ratio: 21.0` (its values at the output), the gearhead outputs joined by the jaw:
  the bench approved it (positions read 57.0 and −56.9 rad at the output), and the approved file ran the demo
  at ±0.05, ±0.5 and ±2 rad — the last reaching 17.3 rad/s, the cruise — with no drive fault after enable, no
  `LIMIT_STOP`, at most 27 N·m of drive-reported torque scaled to the output (1.3 N·m at the motor), at most 4.6 N·m
  opposing torque and 0.007 rad relative position, exit 0; these do not measure the jaw's load. The ±0.5 rad run
  first faulted both drives, read by SDO as 0x603F = 0xFF18 (Err24.0, position deviation excess), with Pr0.13 = 30 %
  and Pr0.14 = 2670177 counts (2 rad at the motor) still in their RAM from an earlier test file (read by SDO; the
  console readings were not kept). The recorded tracking error passed 2 motor rad about 10 ms before the fault —
  consistent with that Pr0.14 (the `a6b_pair` example's README). After a power cycle (Pr0.13 = 300 %, Pr0.14 = 83886080 read back) it ran as
  above; the ±0.05 rad run came before the power cycle. A reset of the fault the previous exit left cleared axis 0 but not
  axis 1, most likely because it was sent before axis 1 exchanged data on DC (its status at that moment was not
  recorded); the bench sends that reset once every axis reports.

### Fifth session

`dc_sync0`, 1 ms, files derived from the example, gearhead outputs free unless stated:

- SYNC0 shift (`dc.sync0_shift_ns`). PLEM sets the DC application time when its RT loop wakes and sent the frame
  about 12 µs later (p99 15 µs, max 55 µs). With shift 0 the ESC's SYNC0 start time (0x0990) put SYNC0 about 4.7 µs
  after the wake — before the cycle's frame; with 20000 and 300000 at 25 and 304 µs. At 0 the drives' position demand
  value (0x6062) now and then skipped or repeated a cycle's target (steps of 0 or 2 cycles, or 1/16 of one) while the
  host's target was regular, and torque spikes (up to about 25 N·m at the output) followed 1–2 cycles later. In
  alternated runs this showed in 6 of 10 at 0 and in 0 of 10 at 300 µs (Fisher's exact test p ≈ 0.01); 20 µs still
  showed it. The example sets 300000 (schema §5). With the outputs joined, a run at 23.5 rad/s (output) and 300 µs completed
  31 s without a trip of the coupling guard, where a run at 0 tripped after about 10 s; it still showed a short burst
  during a DC deviation excursion of about 300 µs, so those excursions (fourth session) remain open. The next day,
  with the nvpps PTP-TSC lock disabled, three more coupled runs of about 33 s at 23.5 rad/s kept the DC deviation at
  most 2.7, 2.8 and 5.2 µs while moving, with no guard trip and no lost cycle; no run with the lock on was made the
  same day to compare. Other cycle times were not run.

## Sixth session

`plem drive run`, its bench and `plem.drive`, the same two drives, `dc_sync0` with
`write_cycle_time`, 1 ms, the gearhead outputs joined by the jaw. The file was the example with one change: axis 1's
lower position limit −46 rad instead of −59.5, so that the bench's move (each axis toward its roomier side) went
opposite ways on the two coupled axes. Every other move commanded the axes as mirror images (axis 1 = a fixed sum −
axis 0). A guard on the recorded stream would have taken a protective stop at 12 N·m opposing torque (output) or
0.03 rad relative position; it never did (at most 2.9 N·m and 0.014 rad). One rig, position axes and one velocity
copy; these runs show the two drives moving together, not the jaw carrying load.

- The bench and the approved-file run below ran on the session's final build; the first `--unapproved` runs ran on an
  earlier build of it with the same stop and release paths.
- The bench: every required item passed and the approval was recorded in the file. Held releases
  drifted at most 6.8e-7 rad; `stop-decel` requested the stop at 3.48 rad/s of a 3.62 rad/s cruise, both axes at rest 0.100 s later
  (34.7 rad/s², bound 1.1 × 95). Its end check saw 0x6064 move 1.16 motor turns and statusword bit 13,
  both from the bench's own move (bit 13 lights in normal motion on these drives); the bench does not warn on either.
- The approved file, without `--unapproved`: 10 minutes of `Bus.move_to` between −14 and 26 rad at
  0.9 × the velocity limit (17.9 rad/s), Trapezoid and SCurve alternating — 230 moves, every one arrived; no
  `LIMIT_STOP`, no protective stop, no lost cycle or deadline miss, period deviation at most 11.8 µs,
  `dc_sync_monitor_ns` at most 5.4 µs after the first 20 s. At cruise the one-cycle target step ranged
  0.0170–0.0181 rad (standard deviation 0.07 mrad).
- `--unapproved` (the earlier build): `Bus.move_to`
  ±3 rad with Trapezoid and SCurve at 0.5 × the limit arrived within 1 mrad; a release moved the axes at most
  5.2 µrad; `Bus.stop()` from 11.93 and 17.88 rad/s brought both axes to rest at an average 61 and 68 rad/s² (limit 95)
  and returned with every brake engaged after 0.21 and 0.35 s, 1.15 and 2.33 rad on; SIGINT to the controller manager
  while moving at about 9.9 rad/s: the hardware interface's deactivation (the stop) took 196 ms, no unconfirmed-brakes line, the
  master back to Idle, no process left. No stop reached the Quick stop fallback, so 0x605A = 2 was not observed.
- A `csv` copy of the file (0x60FF declared), `--unapproved`: `Bus.stream` refused 25 rad/s (limit 19.9), NaN and a
  missing axis without sending. In the 3 rad/s run a 126 ms pause of the Python stream loop (200 Hz) stopped the
  axes first — keep the stream loop light; the silence that followed raised the stale-command event (0x54, age
  0.051 s) about 107 ms after the client's last call, on a bus already in protective stop. From motion, a silence at
  10 rad/s stopped the axes (the client saw the stop within about 190 ms; the RT-side timing was not recorded). After
  a stale stop `reset_error` took effect only once a stream was live again.
- At one start one drive's latched fault read 0x603F = 0xA000 instead of the usual 0xFF50; `reset_error` cleared it.

## Later sessions

The same rig with the same file and copies of it, on software that followed the sixth session (fail-fast checks in
`plem.drive`, the 100 Hz `joint_states`, the bench reading `rt_raw`):

- Client-side failures and rates: with this client's stream silent and the protective stop it raised standing,
  `release()` and `reset()` failed in 0.20 and 0.08 s naming the silent stream. On a latched
  fault (`fault_reset.policy: none`) `release()` failed before sending; after `reset()` it released. `joint_states` ran
  at 100.2 Hz; the bench's `stop-decel`, read from `rt_raw` with no cycle lost, passed at 34.2 and 34.8 rad/s² and
  approved the file. A 200 Hz `csv` stream for 60 s on a host at load average 14.6 (11 busy loops) held 185 Hz (longest gap 13.4 ms)
  with no protective stop.
- A `cst` copy of the file (0x6071 declared, torque limit ±3 N·m): `Bus.stream` refused 3.5 N·m without sending.
  Mirrored 1.5 N·m pulses were applied as 1.47 N·m and measured +1.54/−1.60 N·m, but the axes did not move
  (3 µrad); the gearheads' static friction is the likely cause, not established. At 3 N·m per axis one axis turned within the coupling's backlash (about 0.09 rad);
  the coupled pair did not turn together. A silenced stream raised the protective stop with an immediate disable and
  the brakes engaged; with the stream back `reset()` and `release()` succeeded, and `Bus.stop()` disabled at once and
  engaged.
- Start-up wait and the A6B example `demo.py`: the bus came up only once every slave was in OP (2.4–5.8 s after the loop
  started), and a Ctrl-C while waiting for OP stopped it in 0.38 s. `demo.py` (±5 rad ten times, 0.4 × the limits) on
  the coupled pair finished in 33.0 s with Trapezoid and 35.4 s with SCurve; the largest position mismatch, sampled
  about every 5 ms, was 0.00215 and 0.00212 rad and the largest velocity sum 0.156 and 0.143 rad/s (one run each), with
  no protective stop, lost cycle or overrun. PlotJuggler from `plem drive run --plot` plotted
  `/<bus.name>/rt_raw/samples`. Actual position followed the command with a visible lag, not
  measured in that run (about 32 ms × velocity, of which the command filters are about 16 ms; the `a6b_pair` example's README,
  "The example's values and where they come from").
- `pdo_mapping: scanned` after a power cycle of both drives (the first of three starts had no bus run before it): the
  master's scan held the 13 entries the A6B example declares (RxPDO 0x1600 ×4, TxPDO 0x1A01 ×9), the mapping digest
  matched the one the bench had recorded, and both drives reached OP in each start. Whether that power-on mapping is the drive's
  factory default or one saved earlier was not established.

## JetPack 7.2.1 session

A freshly flashed Jetson AGX Orin Developer Kit, JetPack 7.2.1 (L4T 39.2.1), set up by following
[`docs/install.md`](install.md) sections 1 to 6 (the page of that day; its port name and revision check were corrected
after this run) on the built-in RJ45 port (Case 1, IgH `generic` driver), ROS 2 Jazzy,
built from source. The same two A6B drives and gearheads, with the `a6b_pair` example as shipped (zero offsets 0).
Whether the gearhead outputs were coupled was not recorded.

- Setup: menus 1 to 3 of the setup wizard passed; menu 2's 30 s `cyclictest` at 1 kHz on the isolated CPU showed
  4 µs maximum and 1 µs mean latency. The full `colcon test` suite passed (one test skipped by design in a
  `RelWithDebInfo` build). `ulimit -r` 99, `ulimit -l` unlimited and `/dev/EtherCAT0` mode 0660, group `realtime`, as
  section 3 of the install page says; `plem drive run` ran as that non-root user.
- Bench (offered by `plem drive run`): every required item passed and the approval was recorded. Hold drift at most
  2.5e-7 rad; `stop-decel` 32.7 and 34.3 rad/s² (bound 104.5).
- Two `plem drive run` starts after the bench reached OP in 9.2 and 7.5 s. During the second, the kernel logged `Slave did not sync after 5000 ms` for
  the first drive, about 5 s of the 7.5.
- `demo.py` (`SCurve`, 0.4 × the limits) ran to the end (exit 0) in 34.9 s. From release to the stop: no overrun,
  working-counter error or lost cycle in `--monitor`, period deviation at most 1.7 µs. Each of its 21 moves ended
  within 1e-3 rad of the target (the arrival check of `move_to`); positions between them were not recorded.
- The bench's stop left both drives in fault (statusword 0x0238 at the bench's last read, CoE emergency 0xFF50 and
  the sync manager watchdog in the kernel log), as in the sessions above. The demo run started with both drives in
  fault 0x0238; whether that was the bench's fault never reset or the earlier run's stop (which logged the sync manager
  watchdog) was not established. `reset()` cleared it. All three bus stops logged the sync manager watchdog.

## JetPack 7.2.1 session, gearhead outputs free

The devkit of the session above on its built-in RJ45 port (Case 1, IgH `generic` driver), ROS 2 Jazzy, IgH 1.6.13,
built from a development snapshot of this tree (the full `colcon test` suite passed, one test skipped). The same two
A6B drives, gearhead outputs free and no load. Three copies of the `a6b_pair` example, each with zero offsets
measured at the axes' rest positions and `max_rad` narrowed to 52.0 to fit them: the example, a copy with `flip`, and
a `csv` copy. Execution, working-counter and DC figures are those of the generic path and do not compare with Case 2.

- Bench: every required item passed and the approval was recorded on all three files. `stop-decel` 32.9 to 42.6 rad/s²
  (bound 104.5), no `RAMP_STOP_TIMEOUT`. On the `csv` copy `stale-engage` passed.
- Two of the four benches of `csp` files ended at their first release: the bench tried to restart the trajectory
  controller before its spawner had started it. Nothing moved and no approval was written. `Bus.release()` now waits
  for the controller to be active (next section).
- `--monitor` over about 644 s (three complete demo runs and one stopped by Ctrl-C, a 15 s hold): no lost sample and no
  overrun, execution at most 70.5 µs (right after a release), period deviation at most 2.7 µs (p99 0.5 µs). After
  OP, working-counter mismatches in 4 cycles (3 events); the cause was not isolated.
- Release hold: a `csp` release held 15 s moved at most 2.9e-7 rad. On the `csv` copy a release without a stream
  held 5 s, moved at most 2.9e-6 rad, raised no protective stop and kept both axes in Operation enabled.
- Stale stream: a 200 Hz `csv` stream from another process, killed with SIGKILL, raised the stale event about
  100 ms after its last command (age 0.051 s), then the brake engage and the protective stop.
- `flip`: the reported position matched −(count − zero) × 2π ÷ (resolution × gear) within one count, and a positive
  `move_to` lowered the drive's count.
- Stops: SIGINT to a held bus confirmed both brakes engaged, the master back to Idle and no process left. Ctrl-C in
  `demo.py` while moving: the protective stop was accepted and the axes held with brakes on, no `RAMP_STOP_TIMEOUT`.
- `demo.py` (`SCALE` 0.4) ran to the end (exit 0) in 35.7 s.

Not run in this session: the outputs coupled, a stop under load, a run longer than about 11 minutes, the demo on
the flipped file.

## JetPack 7.2.1 session, fixes rechecked

The same devkit, port, driver and drives as the session above, with a later snapshot of this tree (built on the
devkit; the test suite was not run there). The gearhead outputs were taken to be still free and unloaded as the
session above left them; this was not rechecked.

- `pdo_mapping: generated` on a copy of the `a6b_pair` example: every required bench item passed and the approval was recorded.
  With the bus in OP, both drives carried RxPDO 0x1600 (0x6040, 0x6060, 0x607A) and TxPDO 0x1A00 (0x6041, 0x6061,
  0x6064, 0x606C, 0x6077, 0x603F), the layout of schema §4.1 for a `csp` axis, with no entry of the other drive's axis.
- `Bus.release()` waiting for the trajectory controller: with the controller unloaded by hand, `release()` waited
  about 4 s, and the release was accepted 12 ms after a spawner made the controller active. The spawner race of the
  session above did not recur in nine starts (three benches of the `flip` copy with `--verbose`, where two of three
  had ended at their first release before, and six releases called as soon as the bus came up). In the six the
  controller was already active when the release came, and the benches kept no log of the order, so the race itself
  was not reproduced.
- Ctrl-C inside `with Bus` during a `move_to`, one try: the protective stop was accepted first and the trajectory goal
  was cancelled 1.0 ms later; the axes stopped short of the goal and held with both brakes engaged.
- Setup wizard ([`setup/rt/setup.sh`](../setup/rt/setup.sh)): with the bus running, menu 4 refused to measure and named
  the bus thread on the isolated CPU. After the current `99-plem-realtime.conf` was installed (the command menu 1
  runs) and the user logged in again, the login shell's nice value was 0 instead of 19, with `rtprio` 99 and
  unlimited locked memory. A session already open, the processes it started and the user's systemd manager keep
  nice 19 until that manager restarts (a login after every session of the user has closed).

## Not yet verified

Not yet verified on hardware with this software:

- Of `plem drive run` and `plem.drive` (section 4): two coupled axes turning together under torque, commanded-vs-measured
  torque above 1.5 N·m, `--rt-cpu` and two buses on one Jetson.
- The bench on a bus with a `cst` axis. (The bench with `stale-engage`, and a `csv` release held without a stream,
  ran on a `csv` copy on JetPack 7.2.1 with the outputs free.)
- The requested stop's Quick stop fallback (`RAMP_STOP_TIMEOUT`), and with it the example's 0x605A = 2.
- `example_drive.yaml` as shipped; a `scanned` mapping that is refused (it differs from the one the approval recorded
  (schema §6), lacks an object an axis needs, or cannot be read in full (schema §4.1)); a `generated` mapping on any drive but the A6B; `cst` and `csv` beyond the runs listed above, a protective stop of a
  fast `csv` axis, and a silenced stream on a moving `cst` axis.
- The automatic fault reset (`bounded_auto_reset`) and a non-zero `pulse_ms` (`reset_error` with `pulse_ms: 0`
  cleared Err80 before every run), several axes on one drive, three or more drives, distributed clocks with PLEM
  writing 0x1C32:02 with the jaw carrying load, and distributed clocks on any other drive. A SYNC0 shift at any cycle
  other than 1 ms, and the drive-side input shift (0x1C33).
- A protective stop on a measured position crossing a limit while moving (reason 1 was seen only at start).
- The bench and the approval path beyond two A6B drives (declared and `generated` mappings, `csp` axes and a `csv`
  copy; approved with `drive_default` in the third session, in the fourth with DC (gearhead outputs free with
  `gear_ratio: 1.0`, joined with 21.0), and in the later Case 1 sessions).

The full procedure in [`docs/install.md`](install.md) was followed once with Case 1 (the first built-in-port session above). The JetPack 7.2.1 devkit of benchmark runs 1 to 3
([`docs/benchmark/README.md`](benchmark/README.md)) was set up with an earlier version of the setup wizard ([`setup/rt/setup.sh`](../setup/rt/setup.sh), menus 1 to 3,
Case 2). Not verified:

- A measurement by menu 4 (only its refusal with the bus running was seen).
- A recommended cycle time for Case 1, and lost-frame rates beyond the counts recorded ([`docs/benchmark/README.md`](benchmark/README.md): none in three
  five-minute Case 2 runs; on Case 1 between 0 and 6 working-counter errors per run, cause not isolated).
