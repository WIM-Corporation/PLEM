# a6b_pair: two coupled axes moved in sync

To adapt `drive.yaml` to your drives, see "The drives, the checks and the values" below. Where every value comes
from is in "The example's values and where they come from". "Getting-started section N" means section N
of [`docs/getting-started.md`](../../../docs/getting-started.md), and "schema §N" means section N of
[`docs/cia402-drive-model.md`](../../../docs/cia402-drive-model.md).

| File | What it is |
| --- | --- |
| `drive.yaml` | The drive model file: two Panasonic MINAS A6B drives, one position (`csp`) axis each, behind 21:1 gearheads. `bus.name: a6b_pair`, so the bus runs as `/a6b_pair`. |
| `demo.py` | Moves the pair back and forth, mirrored. |
| `plotjuggler.xml` | The PlotJuggler layout for this pair, with its sync tab (the plots, below). |

`plem drive run --plot` opens a layout built from the bus's axes. For the sync tab, load `plotjuggler.xml` from this
folder into that PlotJuggler window with its Load Layout button.

## The rig

Two drives whose gearhead outputs are mechanically coupled and mirrored: when axis0 turns one way, axis1 turns
the other way by the same angle. Call the sum at rest `S = axis0 + axis1`. The demo keeps it by commanding
`axis1 = S - axis0` at every move, and plans both axes as one move, so they start and arrive together. Any difference
from `S` is a position mismatch the coupling has to absorb, and the drives then push against each other through it.

The values in `drive.yaml` are for the drives, motors and gearheads named in "The drives, the checks and the values"
below; another drive, motor, gearhead or coupling needs its own values. A video of that bench is
[`docs/media/a6b-pair-demo.mp4`](../../../docs/media/a6b-pair-demo.mp4) in this repository.

## Run it

Copy the folder somewhere you can write, because the bench records its approval in `drive.yaml`:

```bash
mkdir -p "$HOME/drives"
cp -r "$(ros2 pkg prefix --share plem_drive)/examples/a6b_pair" "$HOME/drives/"
cd "$HOME/drives/a6b_pair"
```

0. **Check each drive first.** Follow "Check each drive" below: the electronic gear, `zero_offset_count`, the stored
   settings and the stopping decelerations.
1. **Bench and approve the file.** It is not approved yet. On a terminal, run `plem drive run drive.yaml`; it offers
   the bench. The bench moves every position axis together in one move, each up to 0.5 rad at the output, toward
   the side with more room to its limit. On a coupled pair those moves go opposite ways only when the limits and resting
   positions make them; otherwise the two drives push against each other through the coupling. So either:
   - Bench uncoupled: remove the coupling and any load, so the outputs turn freely.
   - Narrow one axis's limit so that the moves mirror each other. A copy with axis1's lower position limit at −46 rad
     was benched that way on a coupled rig
     ([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md), sixth session).

   The bench records the approval in `drive.yaml`. Its first checks can read `PASS idle: axis1 idle statusword FAULT
   (0x0238)` and then `WARN fault already latched before the bench started: … 0x603F 0xFF50 … reset_error sent once`:
   the drive starts in the alarm the last stop left (getting-started section 9), and the bench resets it, whatever the
   code. If the WARN shows another code than 0xFF50, it was a real drive alarm: stop the bus and find its cause before
   benching again (getting-started section 9).
   After the bench the bus keeps running; stop it with Ctrl-C before the next step.
2. **Couple the outputs**, mirrored, with the axes at rest. The demo also runs with them uncoupled (each output
   turns freely); the sync plots then show two free axes, not a coupled pair.
3. **Start the bus and the plots** with `plem drive run drive.yaml --plot --monitor`. This starts the bus, the
   per-cycle relay and PlotJuggler on `/a6b_pair`, and prints the loop timing (period deviation, overruns) in this terminal once
   a second. PlotJuggler and its ROS 2 streamer come from `sudo apt install ros-$ROS_DISTRO-plotjuggler-ros`. Ctrl-C in
   this terminal stops both axes, requests the brakes (it warns when they are not confirmed engaged) and closes the
   plots. On its first start PlotJuggler may ask some
   questions (we have not checked these dialogs on a live bus):
   - Whether to start the previously used streaming plugin: answer Yes.
   - In the ROS 2 subscriber dialog: select `/a6b_pair/rt_raw/samples` and keep the header stamp option on.
   - If it reports curves that do not exist yet: choose to create empty placeholders.
4. **Run the demo** in another terminal: `source ~/PLEM/install/setup.bash`, then `python3 demo.py` in this folder. Each axis needs 5 rad of room to its limits from
   where it rests; otherwise `move_to` refuses before anything moves. The demo resets any drive fault, releases the
   brakes, moves the pair 5 rad each way from where it rests, ten times, returns it to where it started, prints `done: …` with where axis0 ended, and stops it. The moves
   run at 40 % of the file's velocity and acceleration limits (about 8 rad/s at the output) and are jerk-limited
   (`SCurve`: the acceleration ramps in over 0.1 s instead of stepping, as the default trapezoid does). Ctrl-C stops
   both axes too. The plots show whether the pair stays in sync: `position_mismatch` and `opposing_torque` stay small.

## The plots

| Tab | Shows |
| --- | --- |
| position | Per axis: measured position (`actual_position`), the command the loop applied (`applied_command`) and the planned position (`command_position`). |
| sync | `position_mismatch`: `p0 + p1` minus its value at the first sample, in rad. `velocity_sum`: `v0 + v1`, in rad/s. `opposing_torque`: `(tq0 + tq1) / 2`, in N·m at the output, the torque the drives push against each other with. All three are near 0 while the pair is in sync. |
| velocity_torque | Measured velocity and torque of both axes. |
| timing | `loop_exec_us` (processing time of one cycle) and `loop_jitter_us` (deviation of one period from the nominal period). |
| bus | `wkc_mismatch` and `deadline_miss` per cycle, and the DC sync deviation `dc_sync_monitor_ns`. |

Start the stream with the pair at rest and in sync: `position_mismatch` is relative to the first sample PlotJuggler
receives after its streaming starts. Clearing PlotJuggler's buffer resets that reference.

## Safety

- On a new rig, lower the demo's `SCALE` first and watch the plots. Its 0.4 is about 8 rad/s at the output. The
  reference pair ran coupled at 0.9 (17.9 rad/s) for 10 minutes
  ([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md), sixth session). On a new rig stay at about 0.85
  at most (16.9 of the file's 19.9 rad/s velocity limit, `limits.velocity_rad_s`; full scale also reaches 95 rad/s²).
- Leave room under the velocity limit, which is a protective stop. The drives overshoot the commanded speed at the
  start of a cruise (at most 5.8 % on the reference drives, more on short moves; see "Limits and stop distance" below).
- Watch `opposing_torque` and stop (Ctrl-C) if it grows. A coupled pair fights itself when it goes out of sync.
- Keep hands off the outputs and the coupling while the bus runs. The bench video shows the coupling opened and
  closed by hand while the outputs turn: a deliberate test on that bench, not a step of this procedure.
- Stopping decelerates both axes at 95 rad/s², then requests the brakes and reports whether they confirmed engaged. A
  command gap over 50 ms or a broken limit stops at once instead: the drive slows on its own deceleration setting, then
  the brake is requested.
- A holding brake is not a service brake. Do not use it to stop a moving axis.
- Bench with the outputs free, or with one limit narrowed as in step 1. The bench's moves point each axis toward
  its roomier side, not toward the mirrored one, so on a coupled pair they can push one drive against the other.

## The drives, the checks and the values

`a6b_pair/drive.yaml` declares two A6B drives on one bus (`bus.name: a6b_pair`; master → drive 0 X2A, drive 0 X2B →
drive 1 X2A), each running one position axis. They run on distributed clocks (`dc_sync0`, `sync0_shift_ns: 300000`),
and PLEM writes the drives' 0x1C32:02 (`write_cycle_time`). Its values ran on two MDDLN55BE drives with
MSMF102L1H5 motors, where files without its stop settings (`limits.acceleration_rad_s2`, 0x605A) were benched and
approved ([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md)). The shipped file is not approved; a copy
differing in one position limit was benched, approved and run in the sixth session ("Run it", step 1), and the
file as shipped was benched and run on JetPack 7.2.1.

If a slave does not reach OP with DC (release refused with `SLAVE_NOT_OPERATIONAL`, getting-started section 5), set
`sync_mode: drive_default` and delete both `dc` blocks to get the Free Run configuration the reference drives were benched and approved in.
Bench it again: `sync_mode` and `dc` are in the config digest, so the change voids an approval.

On the bench each motor drives a 21:1 planetary gearhead (Nidec ABLE VRG, frame size 120). The two gearhead outputs
face each other, and a printed jaw coupling can join them (the video above). The file says so with `gear_ratio: 21.0`,
so its values come in two layers:

- **Axis values** (`limits`, the commands and `joint_states`) are rad, rad/s and N·m at the gearhead output.
  `factors` converts them; `encoder_resolution` and `motor_rated_torque_nm` stay the motor's.
- **Drive values** are raw, as the drive's manual gives them, and `gear_ratio` does not apply. PLEM writes
  `startup_sdo` values and compares `stored_sdo` values literally (their position units count at the motor).
  `zero_offset_count` is the motor's 0x6064 count PLEM subtracts, and `dc` holds timing settings.

A position, speed or acceleration at the motor is 21 × the output's. Torque goes the other way: output = motor × 21,
less the gearhead's losses, which PLEM does not model. Keep fast moves short, because the gearhead's rated input speed
is below the motor's maximum (its `gear_ratio` row below). With another gearhead, change `gear_ratio` and every axis
value; with none, `gear_ratio: 1.0` makes the axis the motor shaft.

The file assumes motors with a holding brake that the A6B itself releases and engages with the servo, and
battery-backed absolute encoders (getting-started section 2). These are premises a file cannot state (schema §4.4),
like the other two listed there: losing communication is a protective stop, and brakes are independent per axis. The
file also assumes that the A6B's brake release output, BRK-OFF, is assigned with 3410h–3412h [SX-DSV03242 §6-9-3]
and wired to the brake. How
the drive times the brake is its own setting, and the "set brake" through 60FEh is not supported. On the reference
drives the brakes released (the axes turned); holding and timing were not measured.

**Another drive, motor or gearhead.** The file is for MDDLN55BE drives with MSMF102L1H5 motors and 21:1 gearheads. If
your drive or motor model differs, change `product_code` and `motor_rated_torque_nm` (one entry holds both drives), and
scale the speed values to your motor's maximum speed. If your gearhead differs, change `gear_ratio` and every axis
value.

### Check each drive

With the bus stopped (`-p` is the drive's ring position):
- **Electronic gear.** Read `0x608f`, `0x6091` and `0x6092` subindices 1 and 2
  (`ethercat upload -p 0 -t uint32 0x6091 1`, and so on). The factory 1:1 means
  (608Fh:01 ÷ 608Fh:02) × (6091h:01 ÷ 6091h:02) = 6092h:01 ÷ 6092h:02 [SX-DSV03242 §9]. If yours is not 1:1, restore
  the factory gear with the drive's setup software. Every value in the file counts encoder pulses. A drive set up for
  another controller may use another gear, and then positions and limits all scale by that ratio. The bench compares
  6092h/6091h (else 608Fh) with `encoder_resolution` when they give an integer count; it cannot see a
  vendor-specific gear.
- **Zero.** Where you want 0 rad, read `ethercat upload -p 0 -t int32 0x6064 0` (motor counts) and write it as
  `zero_offset_count`. Both axes must start inside the position limits (±59.5 rad at the output, narrowed for
  your zero as described below), or release is refused. For example, a zero of 700000000 counts moves the reachable
  range's top to 51.6 rad (76.6 − 700000000/28036857 = 51.6), so that axis needs `max_rad` 51.6 or less; "Limits
  and stop distance" below has the rule. Both axes share one `limits` entry, so the narrower bound applies to both.
- **Stored settings.** Make each drive's stored parameters match the file's `stored_sdo`. The bus reads each value
  at start. A difference gives one warning line with the commands to store the file's value, or advice to change the
  file if the drive is right. It writes nothing and starts anyway (schema §4.9). A match reads the current value only:
  it does not show the value is saved, or applied. The drive manual says whether an object can be written and stored,
  and whether the drive implements CiA 301 0x1010. If it can: stop the bus, power-cycle the drive (its RAM goes back
  to the stored values), download the value, save with 0x1010:01, and power-cycle again if the object takes effect at
  power-up. Power-cycle first because a save usually stores **every** storable value the drive holds at that moment;
  otherwise the stopping decelerations this file writes at every start are saved with it.
- **What an earlier file wrote stays.** Power-cycle the drives when you switch to a file that writes less.
  `startup_sdo` writes go to the drive's RAM and stay there after the bus stops, until the drive is power-cycled, so a
  file run after another inherits every object the earlier file wrote that it does not write itself. On the reference
  drives a test file's Pr0.13 = 30 % and Pr0.14 = 2 rad at the motor stayed, and an ordinary move with this file then
  faulted with Err24.0 (position deviation excess). The A6B specifics:
  - **Overtravel inputs.** The file expects 0x3401–0x3403 = 0 (cleared). Wire the limit switches and change the file,
    or clear the assignment. The input assignment takes effect only after a power cycle. On the reference drives, the
    factory input assignment put POT and NOT (normally closed) on SI2 and SI3 (0x3401, 0x3402). Unwired, they inhibit
    both directions: the drive enables but does not follow the commands.
  - **Tuning.** The file expects the maintainer's values (0x3002 = 1, 0x3003 = 11, 0x3222 = 139, 0x3647 = 1). A
    drive tuned on its own machine differs; change the file, not the drive.
- **Stopping decelerations.** The manual asks you to set these for your machine. Choose them from the load's inertia,
  the motor's torque and the drive's regeneration capacity, and try the first stops without a load. At every start the
  file writes 0x605C = 1, 0x605E = 2 and 0x6084 = 0x6085 = 3200 rad/s² at the motor (152 rad/s² at the output; the
  objects are in drive units; getting-started section 6). 3200 rad/s² is not a Panasonic value; we chose it on the
  bench, gearhead outputs free. With a load the peak torque and the regenerated energy rise and can raise a drive alarm
  or shock the machine.
- **The drive's own settings.** Read 0x6065 (Following error window), 0x6080 (Max motor speed), 0x6072 (Max
  torque), Pr0.13 (1st torque limit) and Pr0.14 (Position deviation excess) (`ethercat upload -p 0 -t int16 0x3013 0`
  and `ethercat upload -p 0 -t int32 0x3014 0`; the reference drives read 300 and 83886080. Pr0.13 is 300, not the
  default 500, because the drive caps it at the connected motor's maximum torque; see the table below). Drive objects
  count at the motor. These settings stay as they are (`startup_sdo` writes only what it lists, `stored_sdo` only
  reads). The factory defaults, the lag at speed and statusword bit 13 are in "The example's values and where they
  come from" below.
- **Absolute encoder.** Check Pr0.15 (0x3015); the file expects 2. 1 is the incremental mode: the position
  restarts at every power-on and `zero_offset_count` no longer holds (getting-started section 2). 0, 2 or 4 keep it
  with the encoder battery [SX-DSV03242 §6-9-4]. One of the reference drives came set to 1. After switching to an
  absolute mode that drive raised Err40 until its multi-turn data was cleared. Clear it with the servo off:
  `ethercat -p <ring position> download -t uint16 0x4d01 0 0x0031`, then a 0→1 edge on bit 9 of 0x4D00:01
  (`download -t uint32 0x4d00 1 0x200`, then `0`). See the drive's manual.
- Then run `plem drive check` on the file.

### Limits and stop distance

The file's values at the output: a velocity limit of 19.9 rad/s (80 % of the motor's 5000 r/min, 418 rad/s at the
motor) and 95 rad/s². A measured speed above the limit is a protective stop, so leave room under it for the drive's
overshoot. On the reference drives (real-time auto-tuning on, gearhead outputs free) the overshoot over cruises of 2.4 to
17.4 rad/s (50 to 366 at the motor) was at most 5.8 %, the largest at the slowest
([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md), third session). Short moves can overshoot more, so
measure yours.

±59.5 rad is inside the drive's 32-bit position count only for a zero near count 0. The count reaches ±76.6 rad at the
output (2³¹ counts, 23 bits, 21:1). So with your `zero_offset_count` of z counts, the reachable joint range is
−76.6 − z/28036857 to 76.6 − z/28036857 rad, and the joint value itself must also stay within ±76.6 rad. Narrow
`limits.position` to both; a command outside is a protective stop. `plem drive check` warns when a limit reaches past
the range your `zero_offset_count` and `flip` leave, and prints that range.

A protective stop at speed travels on, and how far depends on your machine. Measure it and keep more than that to the
limit. For scale only: a limit or command-gap stop slows at 0x6084, 152 rad/s² at the output (measured stop below), and
a command gap adds up to 50 ms at speed before the stop. `protective_stop` and `Bus.stop()` ramp at 95 rad/s². From
17.88 rad/s the reference axes took 2.33 rad at an average 68 rad/s²
([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md), sixth session), more than v²/2a at 95 rad/s² (about
1.7 rad). Other drive alarms (a position deviation excess, over-speed) follow the drive's own alarm stop sequence,
which can use its dynamic brake; 605Eh covers only the EtherCAT alarms (table below). See the drive's manual.

## The example's values and where they come from

The values come from Panasonic's published documents (cited per value as [document §section]), from the motor maker's
product page for the motor, and from the two reference drives the file was written on, read with `ethercat slaves -v`
and SDO uploads: MDDLN55BE drives (basic type, D frame, 200 V, 1.5 kW) with MSMF102L1H5 motors. Only the stopping
decelerations are PLEM's bench choice. Each value says whether it is at the motor or
at the output.

- [SX-DSV03190] MINAS A6B Series for Rotary motor, Specifications, R11.0
- [SX-DSV03241] MINAS A6B Technical Reference, Functional Specification, R10.0 (section numbers below); the
  Pr0.13 entry cites R14.0
- [SX-DSV03242] MINAS A6B Technical Reference, EtherCAT Communication Specification, R10.1

**Drive, motor and gearhead**

| Key or object | Value | Source / why |
|---|---|---|
| `vendor_id` | 0x0000066f | "fixed at 0000066Fh" [SX-DSV03242, object 1018h-01h] |
| `product_code` | 0x60380009 | MDDLN55BE [SX-DSV03190 §3.1], read on the reference drives. Another model: that table or `ethercat slaves -v` |
| Motor | MSMF102L1H5 | 1 kW, rated 3000 r/min, max 5000 r/min = 523.6 rad/s, rated torque 3.18 N·m, peak 9.55 N·m, rotor inertia 2.47e-4 kg·m², holding brake, 23-bit encoder (product page) |
| `motor_rated_torque_nm` | 3.18 | The motor's rated torque; the reference drives read 6076h = 3180 mN·m. Yours: 0x6076 in mN·m ÷ 1000 |
| 6502h (supported modes) | 0x3AD | Read on the reference drives: csp, csv and cst are supported |
| `gear_ratio` | 21.0 | Nidec ABLE VRG, frame size 120, nameplate VRGH-21D120-19FB19; its catalog 72050 rates the input at 3000 r/min, 6000 r/min at most, and the output at 126 N·m. 17.4 rad/s at the output is 3490 r/min at the motor, above the rated input speed: short moves, not continuous duty |
| `encoder_resolution` | 8388608 | 23-bit/r encoder. The factory electronic gear is 1:1 for it, so one position unit is one encoder pulse [SX-DSV03242 §6-9-4]. Read on the reference drives: 0x608F 8388608/1, 0x6091 1/1, 0x6092 8388608/1, so 1:1 ("Check each drive") |
| `torque.unit` | `rated_per_mille` | 6071h/6077h are in 0.1 % of the rated torque [SX-DSV03242 §9] |
| `velocity_factor` | 1.0 | 606Ch is in command units/s, the same unit as position [SX-DSV03242 §9] |
| `mode_display_timing` | `pre_enable` | 6061h follows 6060h about 2 ms after a change, independent of enable [SX-DSV03242 §6-5-4]; 0x6061 read 8 at rest on the reference drives. A drive that shows its mode only after enable needs `post_enable`; its release refusal says so (schema §4.2a) |

**Process data.** The file declares its mapping instead of `scanned`: the shipment RxPDO mapping 1 (1600h) and TxPDO
mapping 2 (1A01h), entry for entry [SX-DSV03242 §5-4-3]. PLEM writes the declared mapping in PREOP. TxPDO mapping 1
(1A00h) has no 606Ch/6077h, which the bus requires. The RxPDO of mapping 2 carries 6080h (Max motor speed), which
nothing here writes, so it would stay 0. `pdo_mapping: generated` would write only the objects PLEM uses (without the
touch probe and digital inputs); a copy of this file with it passed the bench once on the A6B pair, outputs
free and no load ([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md)).

**Speed and torque.** Plan moves below the 19.9 rad/s velocity limit with a margin for the drive's tracking overshoot:
17.4 rad/s at the output, about 70 % (366 rad/s at the motor), leaves 2.5 rad/s. On the reference drives the overshoot
at 17.4 rad/s was 0.7 %. Another motor, load or tuning overshoots differently; measure yours by running below the limit
and reading the peak velocity around each start of cruise. The bare rotor needs about 0.49 N·m at the motor at
95 rad/s² at the output (1995 at the motor, 15 % of rated). The gearhead adds its own inertia and losses (the reference
drives, gearhead outputs free, reported at most 1.3 N·m at the motor), and a load adds more. `joint_states` reports the
torque at the output (21 ×). The ±59.5 rad position limits are about ±9.5 output turns.

**Stopping.** `startup_sdo` writes these to the drive's RAM at every start. 0x1010 is not written, so they are not
stored. Once PLEM lets go, the drive decides how a moving axis stops (getting-started section 6):

| Object | Value | Source / why |
|---|---|---|
| 0x605A Quick stop option code | 2 | Slow at 0x6085, then Switch on disabled (CiA 402). The A6B's 0x605A value table is not in the A6B manuals cited above, so the first hardware stop is the check. It has not been observed ([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md)) |
| 0x605C Disable operation option code | 1 | Slow at 0x6084, then Switched on |
| 0x605E Fault reaction option code | 2 | Slow at 0x6085 on the alarm, then fault [SX-DSV03242 §6-9-2]. It covers only the EtherCAT-related alarms (Err80, 81, 85, 88) |
| 0x6084, 0x6085 decelerations | 4272282972 | 3200 rad/s² at the motor (152 at the output) in command units/s² with the 1:1 electronic gear; `gear_ratio` does not apply. Near the U32 top (about 3217 rad/s² at 23 bits). PLEM's bench choice, not Panasonic's: set yours for your load ("Check each drive"). Factory: about 0.75 rad/s², tens of seconds of coasting from speed |

On the reference drives with the gearhead outputs free, a stop on the disable path (Disable operation, 0x6084) from
about 344 rad/s at the motor (limit raised to 457.5 rad/s) brought the axes below 0.05 rad/s in about 0.19 s and 30 rad
(16.4 rad/s and 1.4 rad at the output; that stop call disabled the drives at once, the path a limit or a command gap takes). The speed was
still above 95 % of its value 30 ms after that stop call, so about 10 rad (0.5 at the output) of that travel was at
this speed ([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md), third session). The requested stop's
Quick stop fallback (getting-started section 6) has not run. A holding brake is not a service brake: "for maintenance
purposes and should not be used as a stopping (braking) device" [SX-DSV03190 §10, item 34].

**Alarms after a run.** Most exits in the first five sessions (14 of 19, on the reference drives with an earlier version
of the front end) left Err80.4 (AL status 001Bh, sync manager watchdog [SX-DSV03242 §8-2]); the mechanism is in
[`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md). In the stop runs the axes had stopped and the brakes
had engaged before the exit. Clear it with `reset_error` before the next release (getting-started section 5). A killed
host left a different alarm (0x603F = 0xFF58). A drive still in Fault at the end of a stop cannot have its brake
confirmed (getting-started section 6).

**The drive's own settings** ("Check each drive"), at the motor:

| Object | Value | Source / why |
|---|---|---|
| 6065h Following error window | 100000 counts, about 0.075 rad | Default [SX-DSV03190 Appendix 1]. Passing it sets statusword bit 13 (following error [SX-DSV03242 §6-4]) |
| Pr0.14 Position deviation excess setup | 83886080 counts, about 62.8 rad | Default [SX-DSV03190 Appendix 1]. The alarm. Panasonic sizes it Vc / Kp × (1.2 to 2.0) [SX-DSV03241 §7-4]: with Pr1.00 (position loop gain) = 32.0 /s read on the reference drives, about 16 to 26 rad at 418 rad/s |
| 6080h Max motor speed | 6500 | Default [SX-DSV03190 Appendix 1], with the drive's over-speed protection |
| 6072h Max torque | 5000 | Default [SX-DSV03190 Appendix 1] |
| Pr0.13 1st torque limit | 500, read as 300 | Default 500 [SX-DSV03190 Appendix 1]. The drive limits the value to the connected motor's maximum torque [SX-DSV03241 R14.0, Pr0.13]: 300 % for the MSMF102L1H5 (peak 9.55 ÷ rated 3.18 N·m). On the reference drives a write of 500 reads back 300 |

The lag at speed is far larger than the following error window and is not a fault: about 32 ms × velocity here,
11.7 rad at 366 rad/s (0.56 rad at the output's 17.4 rad/s). 60F4h is taken before the command filters
[SX-DSV03242 §6-6-1], so the factory two-degree-of-freedom control (Pr6.47 = 1, Pr2.22 = 139 = 13.9 ms, read on the
reference drives [SX-DSV03241 §5-2-16]) shows in it. The command filters read on the reference drives (3222h 13.9 ms
smoothing, 3223h 1.0 ms FIR, 3648h 1.7 ms tuning filter) add up to about 16 ms; the rest is not explained. It is not
the position loop's own lag: 3110h velocity feed forward reads 100.0 % (3111h filter 0) [SX-DSV03242 §9]. To shorten
it, tune with the load on, raising Pr0.03 stiffness step by step [SX-DSV03241 §5-1-3].

On the reference drives passing the window raised no alarm from 5 to 366 rad/s, and PLEM does not read bit 13. With
this window, bit 13 is on whenever the axis moves faster than about 0.1 rad/s at the output (on the coupled rig at 14
and 23.5 rad/s at the output: set in every cruise cycle, clear at rest). That is normal here. The file leaves
6065h/6066h as they are, since a wider window would only change the bit.

**Stored values.** `stored_sdo` sizes follow the object table [SX-DSV03242 §9]: 3401h–3403h are I32, the rest I16
(read as 2 bytes on the reference drives). The factory 818181h puts POT and NOT (normally closed) on SI2 and SI3
("Check each drive").

**Distributed clocks.** With `sync_mode: drive_default`, the reference drives ran Free Run, not
SM2-synchronous (in OP, 0x1C32:01 = 0 and 0x0981 = 0). Without DC the A6B keeps its 0x1C32:01 default
[SX-DSV03242 §5-5], and PLEM writes no 0x1C32 there.

On DC the A6B takes 0x1C32:02 from the master and raises Err81.0 (AL 0x0035) at PREOP to SAFEOP unless it equals the
SYNC0 cycle [SX-DSV03242 §5-5, §8-2]. So each slave declares `write_cycle_time: true`, and PLEM writes
0x1C32:02 = 1000000 in PREOP. The drive sets 0x1C32:01 = 2 itself [SX-DSV03242 §5-5]. `assign_activate: 0x0300` is
SYNC0 on (0x0981 read 3 on the reference drives; the A6B takes 000b or 011b in its bits 2-0 [SX-DSV03242 §8-2,
Err81.5]). SYNC0 at 1 ms is the bus cycle, a cycle the A6B supports [SX-DSV03242 §5-5].

`sync0_shift_ns` is the master's SYNC0 shift (schema §5). SYNC0 fires 300 µs after the moment each cycle starts, so
that cycle's frame (sent about 12 µs later, at most 55 µs measured) has reached the drive. At 0, SYNC0 came before the
frame, and the drives now and then skipped or repeated a cycle's target, with torque spikes
([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md), fifth session). 300 µs was measured for this 1 ms
cycle only. It is not the drive's own Shift time (0x1C32:03), which the A6B does not support.

With the 0x1C32:02 write made by hand, and then by PLEM (`gear_ratio: 1.0` with the gearhead outputs free, and 21.0
with them joined), the reference drives reached OP with no new drive fault while enabled. The DC sync deviation
settled after 10–30 s and then rose for several seconds about every 110 s, for a cause not known
([`docs/hardware-evidence.md`](../../../docs/hardware-evidence.md)).
