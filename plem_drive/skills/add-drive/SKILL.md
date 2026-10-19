---
name: add-drive
description: >-
  Writes a PLEM CiA 402 drive model file (`cia402_drive/v1`, the `drive.yaml` that `plem drive check` and
  `plem drive run` read) for the servo drives on an EtherCAT bus, for any vendor: starts from the installed example,
  reads the real drives without changing them, researches the drive's manual and ESI, and writes each value with its
  source next to it — or asks the user, or leaves the key open. Use it whenever someone wants to set up, onboard or
  configure a new servo drive or EtherCAT motor for `plem drive`, write or finish a drive model file, or adapt a
  drive.yaml to another drive model — even when they do not name the skill.
---

# Drive model file for the drives on a bus

The goal is a drive model file that `plem drive check` accepts and in which **every value can be traced**: to what
the bus reported, to a document, to a read on the drive, or to the user's own decision. A wrong value can turn a motor the
wrong way, stop it too late, or make a limit guard the wrong range. So a value nobody can source stays open: leave its key out and put `# OPEN: <key> — <why, who closes it>` where
it goes. A missing required key refuses the file, which keeps an unfinished file from ever running. Open the key, not
an entry inside it: an unsourced stop deceleration opens the whole `startup_sdo`, because a shorter list or `[]` still
loads. Leaving a key open is a correct outcome; guessing is not.

The bench (`plem drive run` on an unapproved file) is the physical check that comes after your work; it moves the
axes and is the user's to run at the drive. Your part ends at a file that passes `check`, or fails only on its open
keys, and a list of what is still open.

Talk to the user in their language. Keep the YAML comments in one language, English unless the user asks otherwise.

## What you may run

Reading the bus does not change it; writing to it can move, reconfigure or permanently alter a drive. Run only:

- `plem drive check [--json] <file>`
- `ethercat master`, `ethercat slaves [-v]`, `ethercat pdos`, `ethercat sdos`, `ethercat sii_read`, `ethercat xml`
- `ethercat upload -p <ring position> <index> <subindex>` — an SDO read. Read with no bus running (the drives rest in PREOP).
  If it fails with "Failed to determine SDO entry data type", pass the type the manual gives: `-t uint16`, `-t int32`.

Nothing else on the bus: no `ethercat download`, `states`, `sii_write`, `alias`, `reg_write`, `foe_write`,
`soe_write`, no `plem drive run`, no bench, nothing that enables a drive. If `ethercat` is refused for permission, ask
the user (the udev rule sets who may access the master — `docs/install.md`, section 3) — do not use `sudo`. A value
that can only be had by writing or moving goes on the open list for the user.

## Addresses on a multi-axis drive

A drive can carry several CiA 402 axes (channels 1–8); channel c's profile objects (0x6000–0x67FF) sit at
`<index> + (c−1) × 0x800` (0x6076 on channel 2 is 0x6876). Communication objects (0x1C32) and vendor objects are per
drive — the manual says where they are. Read a profile object at its channel address, and write it at that address in
`startup_sdo`/`stored_sdo`; the mapping entries in `pdo_mapping` are those absolute addresses too. The one exception
is `error_code_pdo`: it always takes the channel-1 base (`0x603f`), and PLEM adds the channel offset itself (schema
§4.3).

## Steps

### 1. Read the bus, then ask once

`ethercat master` must show `Phase: Idle` and `ethercat slaves` every drive in PREOP; a drive in SAFEOP or OP means a
bus is running — tell the user and do not stop it yourself. Then read each drive: `ethercat slaves -v` (ring
position, vendor ID, product code, revision, name) and `ethercat pdos -p <ring position>` (its PDO mapping).

Then ask the user, in one message, for what the bus cannot tell you:

- the drive model, motor model, gearhead and its ratio (nameplate or catalog), per drive;
- the mode of each axis (`csp` position, `csv` velocity, `cst` torque) and, on a multi-axis drive, which channels;
- the brake: a holding brake per axis that the drive itself releases and engages;
- the encoder: absolute (battery-backed or multi-turn) per the motor data sheet, and set to an absolute mode — confirm it from the documents in step 2 and cite it; if the manual names an
  encoder-mode parameter, put it in `stored_sdo`;
- whether a bus ran since the drives were powered on — it wrote its mapping into the drives, which keep it until
  power-off, so they must be power-cycled before the mapping you read is the drive's own;
- the drive's manual, ESI file and motor data sheet if they have them.

If the user does not answer, carry on with what you have and open what depends on the answer.

**Stop here and tell the user** when a premise of PLEM does not hold (schema §4.4); the file has no key for these,
and nothing detects them later:

- no brake, an external brake, or a brake released through 0x60FE, or one brake shared by several axes — not supported;
- an incremental encoder (or an absolute one set to an incremental mode) on an axis with position limits — PLEM has
  no homing, so the limits would guard a different range after every power-on. Only an endless axis (`position:
  null`, `zero_offset_count: 0`) may run without an absolute position.

### 2. Research

Get the drive manual (its CoE object dictionary chapter), the motor data sheet and the drive's ESI XML. Prefer the
user's files; otherwise the maker's published documents. Note each document's title, number and revision — every
documented value cites it. An ESI gives objects, defaults, access rights and PDO sets, not the meaning of units: a
value only a manual could source (the torque or velocity unit) stays open when the ESI is all you have.

What each key is and where it comes from: the plem drive getting-started guide, section 2 ("Fill in the values"), and
the schema `docs/cia402-drive-model.md`, both in the plem drive source repository's `docs/`. `plem drive check`'s refusals also name each key's
rule. The installed `$(ros2 pkg prefix --share plem_drive)/examples/a6b_pair/drive.yaml` and the `README.md` beside it show a finished file
with every value's source — read it for form, never copy its values: they are for one drive and motor.

### 3. Fill the values

Start from the user's file if they have one; otherwise copy the installed example:

```bash
mkdir -p ~/drives
cp "$(ros2 pkg prefix --share plem_drive)/examples/example_drive.yaml" ~/drives/<name>.yaml
```

Its values are synthetic and load as they are — `flip: false`, `zero_offset_count: 0`, `sync_mode: drive_default` look like
defaults but are not. Every one gets replaced or opened; step 4 audits it. Drives that are the same drive, motor,
gearhead and settings share one slave entry with a `ring_position` list. The example is a `csp` axis: a `cst` axis
takes `limits.torque_nm` instead of `limits.acceleration_rad_s2`.

Write each value's source as a YAML comment on its line or in the comment block above it. Comments do not change the
file's digest, so they cost nothing and let the user check each value.

| Kind | Keys | Comment |
|---|---|---|
| Reported by the bus | `ring_position`, `vendor_id`, `product_code`, `pdo_mapping: scanned` | `# ethercat slaves -v` / `# ethercat pdos` |
| From a document | `gear_ratio` (nameplate, catalog), `encoder_resolution`, torque `unit`, `velocity_factor`, a declared `pdo_mapping`, `dc.assign_activate`, `dc.write_cycle_time`, which stop objects the drive has, `fault_reset.pulse_ms` (0 when the manual states none), `cia402_profile` (only when the manual says the drive takes Shutdown in `not_ready_to_switch_on` — schema §4.8), `stored_sdo` entries (the encoder-mode and input-assignment parameters the manual names) | `# [<doc no.> §<section or object>]` |
| Read on the drive | 0x6076 rated torque, 0x6502 supported modes, 0x608F/0x6091/0x6092 (encoder increments, gear), 0x1C32:02 access, 0x6064 for `zero_offset_count` | `# read: 0x6076 = 3180 (mN·m)` |
| The user's decision | `cycle_time_ns`, `sync_mode`, `dc.sync0_cycle_ns`, `dc.sync0_shift_ns`, `fault_reset.policy`, every `limits` value, the stop objects' values (0x605A 1–3, 0x605C, 0x605E, 0x6084, 0x6085), `flip` | `# user: <their words>` |
| A label | `model.name`, `bus.name`, axis `name` | pick a descriptive one and say "rename freely" — names change no behaviour |
| Left out unless the manual says otherwise | `mode_display_timing` | leave it out (= `pre_enable`); write `post_enable`, cited, only when the manual says 0x6061 shows the mode only after enable. A drive that does so without saying is refused at release, and the refusal names this key |

**The mapping.** `pdo_mapping: scanned` fits when the drive's mapping carries every object its axes need, at the
widths the bus checks (schema §4.1): out 0x6040 (16), 0x6060 (8) and the mode's target — 0x607A (32), 0x60FF (32) or
0x6071 (16); in 0x6041 (16), 0x6061 (8), 0x6064 (32), 0x606C (32), 0x6077 (16); each at its channel's address. If it
lacks one, declare a mapping the drive's ESI or manual offers that carries them all, cited; `generated` only with the
user's agreement: it has run on hardware only with the A6B, and a drive that does not accept PLEM's PDO
numbers refuses it at PREOP→SAFEOP (schema §4.1). `error_code_pdo` may be left out only with `generated` or a declared
mapping that carries Error code; otherwise write `{ index: 0x603f, subindex: 0, bits: 16 }` when the mapping carries
it, or `null`.

**The electronic gear.** `encoder_resolution` is 0x6064 counts per motor turn, which equals the encoder's increments
only with a 1:1 gear: (0x608F:01 ÷ 0x608F:02) × (0x6091:01 ÷ 0x6091:02) = 0x6092:01 ÷ 0x6092:02, and no vendor gear
parameter the manual names away from its factory value. Otherwise open `encoder_resolution` and `velocity_factor` and
tell the user: restoring 1:1 is a write to the drive, theirs to do; write a counts-per-turn value only when the manual
states how the gear sets it.

**Distributed clocks.** With `sync_mode: dc_sync0` every slave gets a `dc` block. `sync0_cycle_ns` is a cycle the
drive supports and a multiple of `cycle_time_ns`; `sync0_shift_ns` must place SYNC0 after the cycle's frame has reached
the drive (schema §5) — it depends on the drive, the cycle and the bus, and a value measured elsewhere does not carry
over. `write_cycle_time: true` only when the documents list 0x1C32:02 as writable and say the master sets it.

**Limits and stop decelerations are the user's.** They decide where the axis may go and how hard it stops. Collect
your proposals and ask for them in one message, each with its arithmetic and sources, so the user can confirm them in
one reply:

- velocity: for example, 80 % of the motor's maximum speed ÷ `gear_ratio` (data sheet cited);
- position: inside the range the drive's 32-bit count reaches from the user's zero, and inside the machine's travel
  (`plem drive check` warns and prints that range when a limit reaches past it);
- the stop deceleration `limits.acceleration_rad_s2` (rad/s² at the output) must not exceed the drive's 0x6085. The
  drive objects count at the motor: when the manual gives 0x6084/0x6085 in position units/s² (and the gear is 1:1),
  counts/s² = rad/s² at the output × `gear_ratio` × `encoder_resolution` ÷ 2π; otherwise use the manual's unit, cited.

Write a value only after the user confirms it; if they do not, it stays open. When the user explicitly hands you the
choice ("pick something sensible"), that is their decision: write your value with
`# chosen at the user's request: <arithmetic and sources>`, and put it first in the hand-over as a value to confirm
before the bench.

`zero_offset_count`: the user puts the axis where they want 0, and you read 0x6064 on it. Without that, it stays open.

When a document and a read on the drive disagree, write the read value, cite both and tell the user. A drive's
settings can differ from its factory defaults.

### 4. Check and audit

```bash
plem drive check ~/drives/<name>.yaml
```

`check` stops at the first refusal, so an open key hides everything after it. To have the rest judged, run `check`
on a scratch copy in which each open key holds a placeholder, fix in the real file whatever it names, and delete the
copy. A refusal names the key and the fix. The keys that may be left out (schema §1 — `stored_sdo`, `cia402_profile`,
…) do not block `check` when open: name them in the hand-over too. Passing `check` is not passing start-up: `not_judged` in
`plem drive check --json <file>` lists what the bus judges only when it runs (a scanned mapping among them). Its
`warnings` name stop objects the file leaves to the drive, a `dc.sync0_shift_ns` of 0, a `limits.position` past the
range the drive's count can report, and a slave that still has the example's `vendor_id`. Clear the last two before the
hand-over: past that range the limit does not hold, and the example's `vendor_id` means the file still holds the
example's identity.

Then audit the real file line by line: every value carries a source comment (`ethercat …`, `[doc]`, `read:`, `user:`,
`chosen at the user's request:`) on its line or in the block above it, or its key is gone and
marked `# OPEN`. The example's own comments (`# yours: …`) are not sources. A value with neither is a leftover from
the example: source it or open it.

### 5. Hand over

First the values chosen at the user's request, to confirm. Then a table of every value you filled — key, value,
source — and the open list: each `# OPEN` key, why, and who closes it. Then the user's next steps at the drive:

1. Close the open keys and run `plem drive check` again.
2. Bench the file with `plem drive run <file>`, the outputs unloaded and the emergency stop in reach; it moves each
   position or velocity axis up to 0.5 rad.
