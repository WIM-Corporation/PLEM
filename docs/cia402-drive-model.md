# CiA 402 drive model file — schema (`cia402_drive/v1`)

A drive model file is one YAML file that declares one EtherCAT bus of CiA 402 servo drives: the slaves, the axes on
them, their unit conversions and safety limits, and an approval record. `plem drive run <path>` (the `plem_drive`
hardware interface (a ros2_control `SystemInterface`), hardware parameter `model_file`) and `plem drive check` read it
through one loader, `load_drive_model_file`
([`plem_drive/include/plem/drive/drive_model_file.hpp`](../plem_drive/include/plem/drive/drive_model_file.hpp)).

Loader and tool messages cite this document by section number ("schema §4.1"). The numbers here are stable.

Every refusal rule below names the layer that checks it (file layer, value gate, bus Step 0 …). A
guarantee that names no check is not claimed.

`plem drive <verb>` below is one command: `plem` runs the `plem-drive` command of the `plem_drive` package,
on the PATH once the workspace's `install/setup.bash` is sourced, and callable directly as `plem-drive <verb>`
(`plem drive --help` lists the verbs). Its verbs are `check`, which execs `drive_model_tool`, the C++ tool of the
`plem_drive` package (`lib/plem_drive/`, not on the PATH), and `run`, which runs in the dispatcher and offers the
bench (§6.1) for a file not approved for motion. The bench records the approval with that tool's `approve` (the
bench's internal step, not a user command).

## 1. Scope

- One file = one drive model: CiA 402 CoE drive slaves 1..N and the axes on them. The file declares drives
  only. It has no robot kinematics, masses, gains or controller settings.
- The file holds data, not procedures. There are no handshake branches, polling, retries, custom state
  machines or unverified torque constants. Code owns the procedures; the file owns constants, roles and the
  choice among verified policies.
- **Every key is required except where listed below, and no default stands in for a value you must
  choose.** Leaving a key out means something in these places only:
  - `cia402_profile` left out = the core's built-in transition table (§4.8);
  - the sub-block of a feature a switch turns off is not written (`dc` when `bus.sync_mode: drive_default`, `pacing`
    when `fault_reset.policy: none`), nor `limits.torque_nm` on an axis whose mode is not `cst` (a `cst` axis's
    key, §4.4), nor `limits.acceleration_rad_s2` on a `cst` axis (a `csp` or `csv` axis's key, §4.4); writing
    it is an error;
  - `stored_sdo` left out of a slave = no stored value is read (§4.9);
  - `write_cycle_time` left out of a `dc` block = 0x1C32:02 is not written (§5);
  - `approval` left out = unapproved (§6);
  - `mode_display_timing` left out = `pre_enable` (§4.2a);
  - `factors.position`, and the `unit` of `factors.position` and `factors.velocity`, left out =
    `rotary_motor_encoder`, the only value (§4.2);
  - `error_code_pdo` left out = Error code (0x603F) when the slave's mapping is `generated` or a declared mapping
    that maps 0x603F at the axis's channel; with any other mapping the key is required (§4.2);
  - `bus.name` left out = the loader accepts the file, but `plem drive check` and `plem drive run` refuse it (§4.5).

  Any other missing key is an error.
- Absence of a value is written with a fixed literal: `null` (`error_code_pdo`, `approval.record`, identity
  fields, `limits.position` for an axis with no end stops) or `[]` (`startup_sdo`, `pdos` of a mailbox SM).
- Anything this version does not support (§9) is **refused**. Unsupported features are not accepted early as
  future keys.

## 2. File format — a strict subset of YAML

The loader reads a narrow YAML subset. The syntax layer (documents, nodes, keys, encoding) is a tree builder
on the libyaml event API; the schema layer checks scalar types ([`plem_drive/src/loader/yaml_tree.cpp`](../plem_drive/src/loader/yaml_tree.cpp),
[`plem_drive/src/loader/reader.cpp`](../plem_drive/src/loader/reader.cpp)). A syntax error names the line and column (an encoding error names the
byte offset; a file with no document has no position). A schema error names the key path
(`slaves[1].axes[0].channel`).

| Rule | Content |
|---|---|
| Document | One document. No directives (`%YAML`) and no second document. One leading `---` is allowed |
| Nodes | Mappings, sequences and scalars only. No anchors or aliases, no explicit tags (not even `!!int`), no block scalars (`\|`, `>`). Nesting up to 16 levels. A merge key (`<<`) is refused as an unknown key |
| Keys | String keys only. **A duplicate key is refused** (before any value is stored, in nested mappings too) |
| Unknown keys | A key the schema does not list is refused (reserved keys too, §9). A typo cannot read as an omission |
| bool | `true` / `false` only, unquoted. YAML 1.1 forms (`yes`, `on`, `Y`) are refused |
| null | `null` only. `~` and empty values are refused |
| Integer | Decimal (no leading zero) or `0x` hex. `+`, `_`, `0X`, negative hex, real notation (`1536.0`) and quotes are refused. Negative values only in signed fields (`zero_offset_count`, `dc.sync0_shift_ns`). Read as 64-bit and range-checked **before** narrowing to the C++ width: out of range is refused, never wrapped or zeroed |
| Real | JSON number syntax `-?(0\|[1-9][0-9]*)(\.[0-9]+)?([eE][-+]?[0-9]+)?`, finite only. `010`, `+1.0`, `.5`, `1.`, `.inf`, `nan` are refused; an integer literal (`121`) is accepted. `-0.0` is accepted and digests the same as `0.0` (§7) |
| String | Kept as a string: a string that looks like a number (`"1"`) is not turned into one. Quoted or plain; a plain value that the loader would read as null, bool or number (`12`, `true`, `0x1f`) must be quoted, as must an empty value or `~`. ASCII control characters (C0 and DEL, including ones written as quoted escapes) are refused. YAML 1.1 bools, timestamps and `.inf` pass as strings where a string is expected |
| Encoding | UTF-8 only (UTF-16 is refused, not converted). Control bytes other than tab and line breaks are refused by the reader. Allowed characters in names: §4.5 |
| Size | Up to 1 MiB, several times a file that fills every capacity (eight slaves with full PDO and startup SDO lists) |

## 3. Top-level shape

```yaml
schema: cia402_drive/v1                        # §10 — the only accepted string
model: { name: bench_3slave_cst }              # §4.5. Keep the file's own revision in a comment
bus: { name: bench_bus, cycle_time_ns: 1000000, sync_mode: drive_default } # §4.5 name; §5 — dc_sync0: every slave writes dc: {...}
fault_reset: { policy: none, pulse_ms: 0 }     # §4.6 — with bounded_auto_reset write pacing: {...}
# cia402_profile: [...]                        # §4.8 — only for a drive that differs from the standard
slaves:                                        # §4.1 · file order = index (not sorted)
  - ring_position: 5
    ...
    axes: [ ... ]                              # §4.2–§4.4 · this slave's axes; document order = joint order
# approval: {...}                              # §6 — optional: left out = unapproved; the bench writes it
```

The keys shown in this document are **all** the keys version 1 accepts. The mapping to value-spec fields is
the table in internals §3.1 (linked from §3.1). Not in the file: the file's own revision (write it as a comment; the product does not
read it), hold/communication-loss/brake-sharing behaviour and the encoder type (fixed premises, §4.4), and
the reserved keys (`brake.coupling`, `extra_sdo`, §9) and an axis's `ramp` (refused, §4.2b).

Counts come from sequence lengths; there is no separate `count` key. The value spec `DriveModelSpec<N>` has
N = the number of axes = Σ len(`slaves[].axes`) after `ring_position` lists are expanded, 1..`kMaxAxes` (8: the brake mask is a uint8). The loader returns a
`DriveModelSpecAny` (a variant over N = 1..8); the hardware interface erases N with `std::visit`. An empty `axes`
list is refused before axes are counted, so the slave count is at most N.

RT observation covers every axis of the bus: each `rt_raw` sample carries `axis_count` axes (up to 8), and
`~/rt_events` reports a `SERVO_FAULT` onset for any of them.

**Limit of the nested shape.** Joint order = document order, so an order that goes back and forth between
slaves (j0 on slave A, j1 on B, j2 on A) cannot be written. The value spec allows it; the file shape does
not. The DC reference clock is chosen by ring position, not document order (§5).

### 3.1 Key → value spec

The loader projects the file onto one flat value spec. Nothing outside `DriveModelSpecAny` reaches
execution. The approval material (the two digests, the profile source, claimed/effective status, reasons)
is the return value of `inspect_drive_model`, which `plem drive check --json` prints (§6).

The value gate, the bus and the runtime speak in flat axis indices. The loader rewrites a value-gate refusal's
`axes[i]` / `slaves[k]` to the file path (`slaves[s].axes[k]`, `slaves[s]`, for a `ring_position` list led by
the drive). At debug log level (`PLEM_LOG_LEVEL=debug`) it also logs the mapping `axes[i] = slaves[s].axes[k]
(name)` and the profile source (built-in or inline, row count).

Which value spec field each file key becomes: [`docs/internals/drive-model-contracts.md`](internals/drive-model-contracts.md), §3.1.

### 3.2 Loader lifetime contract

The file is read once, when the bus starts: no environment variable or host parameter overrides a value from it,
and it is not re-read while the bus runs: replacing it means stop and restart, and the new start checks everything
again. A relative path is relative to the process working directory and `~` is not expanded: use an absolute path.
Where each check runs and what a loaded result guarantees: [`docs/internals/drive-model-contracts.md`](internals/drive-model-contracts.md), §3.2.

## 4. Slaves and axes

### 4.1 `slaves[]` — one physical drive slave = one `ec_slave_config_t`

```yaml
- ring_position: 5                     # ring position (the address the master uses). Gaps and any order are allowed; a duplicate is refused.
                                       # A list ([0, 1]) declares identical drives in one entry (below)
  vendor_id: 0x00c0ffee
  product_code: 0x00001001
  pdo_mapping:                         # required. This mapping (declared: the core writes it in PREOP), the plain word
                                       # `generated` (the core builds and writes the CiA 402 objects the axes use) or the
                                       # plain word `scanned` (the bus reads the slave's current mapping).
    sync_managers:
      - sm: 2                          # SM number 0..15 (kSyncManagerCount)
        direction: output              # output = master→slave (Rx) / input = slave→master (Tx)
        watchdog: true
        pdos:
          - index: 0x1600
            entries:                   # absolute object addresses. index 0x0000 = gap (its width takes image space)
              - { index: 0x6040, subindex: 0, bits: 16 }
              - { index: 0x6060, subindex: 0, bits: 8 }
              - { index: 0x6071, subindex: 0, bits: 16 }
      - sm: 3
        direction: input
        watchdog: false
        pdos: [ ... ]
  dc:                                  # §5 — only with bus.sync_mode: dc_sync0 (refused with drive_default)
    assign_activate: 0x0300
    sync0_cycle_ns: 1000000
    sync0_shift_ns: 0
  startup_sdo: []                      # §4.7 — write [] for none
  axes: [ ... ]                        # §4.2 — this slave's axes (one or more); document order = joint order
```

- The value form is `SlaveSpec{ring_position, vendor_id, product_code, pdo_mapping, dc_*}`. Storage limits:
  at most 4 SMs, 4 PDOs per SM, 16 entries per PDO (the `kMax*` constants of `SlavePdoMappingSpec`,
  `SyncManagerAssignmentSpec`, `PdoMappingSpec`). Do not confuse this with the SM **number** range (0..15).
- `ring_position` is separate from the list index. A slave with `axes: []` is refused (file layer, before
  axes are counted). The file accepts 0..2³¹−1; values above the master's 16-bit address are refused by the
  value gate. A `vendor_id` or `product_code` of 0 is refused by the file layer; write the values that
  `ethercat slaves -v` shows. `plem drive check` warns of a slave with the example's `vendor_id` 0x00c0ffee
  (`plem_drive/examples/example_drive.yaml`, synthetic values); where its next step would offer `plem drive run` on
  an unapproved file, it says to replace the example's values instead.
- An empty `sync_managers` list is refused (file layer). An explicit PDO with empty `entries` is refused
  (entry count 0 would let the master's default mapping in; bus Step 0). An SM with no PDO (a mailbox SM,
  `pdos: []`) is allowed. A gap entry still needs `bits > 0`. Lists above the storage limits are refused by
  the file layer.
- Other values for `pdo_mapping` (`null`, a quoted `"scanned"`, another word, an empty list) are refused.
  A drive that does not accept reconfiguration uses `scanned`.
- **`pdo_mapping: generated`** is a declared mapping the loader builds from the slave's axes, so the core writes
  it in PREOP and everything said of a declared mapping applies. Per axis, in channel order: RxPDO
  `0x1600 + channel − 1` in SM 2 (watchdog on) with Controlword (0x6040), Modes of operation (0x6060) and the
  mode's target (`cst` 0x6071, `csp` 0x607A, `csv` 0x60FF); TxPDO `0x1A00 + channel − 1` in SM 3 with Statusword
  (0x6041), Modes of operation display (0x6061), Position, Velocity and Torque actual value (0x6064, 0x606C,
  0x6077) and Error code (0x603F), each at the axis's channel offset. These are the objects the core reads and
  writes (§8, `validate_pdo_mapping`), nothing more: the file for a drive whose mapping must carry other objects (touch
  probe, digital inputs) declares that mapping. A drive with more than four axes on one slave declares its mapping
  (one PDO per axis, four per SM). The PDO numbers per channel (`0x1600 + channel − 1`) are PLEM's choice, not a
  CiA 402 rule; a drive that does not accept these PDO indices or entries refuses the mapping at PREOP→SAFEOP; for such a
  drive, declare instead the mapping its manual allows, or use `scanned`. Verified on hardware: on two A6B drives,
  `csp` (a copy of the A6B example, which itself declares its mapping;
  [`hardware-evidence.md`](hardware-evidence.md)).
- **A `ring_position` list** declares two or more drives that are the same in every other key: the entry stands
  for one slave per listed position, in list order, and its axes repeat on each. Three axis keys differ by
  drive and take one value per drive, as a list as long as the positions: `name`, `flip` and
  `zero_offset_count` (`name: [axis0, axis1]`). Every other key holds for all of them; a drive that differs in
  any other key gets its own entry. The loader expands the list, so the value spec, the digests (§7) and the
  approval record (§6, `slaves[]` by expanded index) are those of the entries written out one by one.
  A refusal names the file's entry index (`slaves[e]`): a per-drive key adds `[copy]` (`name[1]`), and a
  value-gate refusal for one listed drive starts with `for the drive at slaves[e].ring_position[copy]:`. The debug
  mapping line counts `slaves[k]` and `axes[i]` after the expansion.
- **`pdo_mapping: scanned`** is carried in the value spec as `sync_count 0` (one normalized line
  `slaves[s].pdo_mapping.sync_count=0`; the digest of a declared file does not change). The slave builds no
  graph. In bus initialize Step 3 (after the master is acquired, before devices are created) the bus reads
  the mapping with `ecrt_master_get_sync_manager/get_pdo/get_pdo_entry` and runs `validate_pdo_mapping` for
  every axis of that slave. It refuses before activation, the refusal saying: a missing role names the slave,
  revision and role; a failed read names where (SM/PDO/entry); an incomplete scan (sync_count 0) names
  vendor/product, AL state and three possible causes.
- The output SM watchdog rule (value gate `validate_drive_model_spec`, §4.4) applies to declared mappings only. The core writes no SM on a scanned
  slave, so it runs on the drive's own settings and is not judged.
- For a scanned slave the bus logs one INFO line (SM/PDO/entry counts, revision, sha256) and, with
  `PLEM_LOG_LEVEL=debug`, the full mapping text in the §4.1 form. Paste that text under `pdo_mapping:` with
  matching indentation and it becomes the declaration of the same mapping (a loader round trip is pinned by a
  test).
- Which PDO roles are required (the common roles plus the mode's target: `cst` 0x6071, `csp` 0x607A, `csv`
  0x60FF) and whether addresses overlap is judged by the bus at startup; `plem drive check` runs the same
  judgment without the drives (a scanned slave's mapping excepted).

### 4.2 `slaves[].axes[]` — one CiA 402 axis (= one joint the host sees)

```yaml
- name: j0                             # §4.5 — the host interface name; a duplicate anywhere in the file is refused
  channel: 2                           # which CiA 402 axis inside the drive (from 1) — od_offset = (channel − 1) × 0x800
  mode: cst                            # cst | csp | csv — per axis; one bus may mix them (§4.2b)
  mode_display_timing: post_enable     # pre_enable | post_enable — left out = pre_enable (§4.2a)
  factors:                             # unit conversion — formulas and parameters per unit, no precomputed factor
    gear_ratio: 100.0                  # once per axis — the torque, position and velocity conversions use it
    encoder_resolution: 65536          # once per axis — position and velocity use it
    torque:   { unit: scaled_current, counts_per_amp: 50.0, torque_constant: 0.12 }
    velocity: { velocity_factor: 750.3 }  # unit: rotary_motor_encoder may be written; it is the only value
  flip: false                          # direction — sign inversion at the PDO boundary
  zero_offset_count: 0                 # §4.4 — the hardware position count at q = 0 (int32)
  error_code_pdo: null                 # or { index: 0x603F, subindex: 0, bits: 16 } — channel 1's base address (§4.3);
                                       # left out = 0x603F when the mapping is generated or maps it (below)
  limits: { ... }                      # §4.4
```

The slave is the parent, so an axis has no `slave` key. There is one profile per file (top-level
`cia402_profile`, §4.8). Hold, communication loss, brake sharing and the encoder type are not keys but fixed
premises (§4.4).
PDO mapping indices (0x1600, 0x1610 …) are not computed from `channel`; write them in `pdo_mapping` as they
are (a `generated` mapping is the exception, §4.1).

`error_code_pdo` names the entry the drive's error code arrives in. Left out, it is Error code (0x603F) for
a `generated` mapping, and for a declared mapping that maps 0x603F at the axis's channel. A declared mapping
without it, or a `scanned` one (read only at startup), needs the key: the drive's own error code entry, or
`null` for none.

Every `factors` key is required (file layer), except the position and velocity `unit` (and so a `position`
block that would hold nothing else): it has one value. The check covers only the `unit` values (file layer) and that
numbers are finite and positive (value gate), so **a wrong value still loads.** Gear ratio and encoder
resolution are written once per axis; the loader passes gear ratio to all three conversions and encoder
resolution to the position and velocity conversions. `factors` is a host-side conversion; PLEM does not write the
drive's Factor group objects (0x608F–0x60A9). Inside the core, torque is always Nm; conversion to the drive's
numbers happens once, at the PDO boundary, and `torque.unit` declares what those numbers mean for each drive.
The unit definitions are `TorqueUnit` and `MotionUnit` in
[`plem_fieldbus/include/plem/fieldbus/ethercat/config/configuration.hpp`](../plem_fieldbus/include/plem/fieldbus/ethercat/config/configuration.hpp); the formulas are in
[`plem_fieldbus/include/plem/fieldbus/ethercat/utils/servo_conversions.hpp`](../plem_fieldbus/include/plem/fieldbus/ethercat/utils/servo_conversions.hpp).

| Key | Meaning |
|---|---|
| `torque.unit` | What one count of 0x6071/0x6077 means: `scaled_current` or `rated_per_mille`. The value selects the key set below; keys of the other unit are refused as unknown keys. `scaled_current` = a drive current count, a vendor unit on a standard object (outside CiA 402): count = τ × `counts_per_amp` ÷ (`torque_constant` × `gear_ratio`). `rated_per_mille` = the CiA 402 standard unit, one count = 1/1000 of 0x6076 Motor rated torque: count = τ × 1000 ÷ (`motor_rated_torque_nm` × `gear_ratio`) |
| `torque.counts_per_amp` | (`scaled_current`) current counts per ampere (drive manual) |
| `torque.torque_constant` | (`scaled_current`) motor torque constant in Nm/A (motor nameplate) |
| `torque.motor_rated_torque_nm` | (`rated_per_mille`) motor rated torque in **Nm** at the motor shaft. The drive's 0x6076 is in mNm, so divide by 1000 (0x6076 = 1270 → `1.27`). A wrong value scales every torque command and measured torque: too large makes commands weaker, too small makes them stronger, up to the end of the drive's torque range. `plem drive check --json` prints `rated_torque_mnm` in each axis (mNm, six significant digits, not rounded to an integer; null for other units). The bench reads 0x6076 once (at the axis's channel address) and warns when it differs by more than max(1 mNm, 1 %), and records the warning in the bench evidence's `conditions.notes`. It does not stop the bench. A missing object, 0, 0xFFFFFFFF or a failed read is not compared |
| `gear_ratio` | Motor turns per output-shaft turn (the mechanical reduction), once per axis. Not the drive's 0x6091 value (see the precondition under `encoder_resolution`) |
| `position.unit` · `velocity.unit` | Only `rotary_motor_encoder` in version 1: output-shaft rad ↔ motor encoder counts. Position count = rad × `encoder_resolution` × `gear_ratio` ÷ 2π. May be left out, with the `position` block |
| `encoder_resolution` | 0x6064 counts per motor turn (integer), once per axis. `encoder_resolution` × `gear_ratio` must be the 0x6064 counts per output-shaft turn. `encoder_resolution` alone equals the motor's 0x608F encoder resolution only while the drive's electronic gear (0x6091/0x6092, or the vendor's equivalent) is 1:1; the bench compares 0x6092/0x6091 (else 0x608F) when they give an integer count, and does not see a vendor-specific electronic gear object |
| `velocity.velocity_factor` | Position counts per velocity count (1.0 = the same unit as position). Velocity counts per rad/s = `encoder_resolution` × `gear_ratio` ÷ (2π × `velocity_factor`). Take it from the 0x606C unit in the drive manual; a wrong value silently skews the velocity limit and the `joint_states` velocity |

#### 4.2a `mode_display_timing` — when 0x6061 follows 0x6060

- **Optional; left out = `pre_enable`.** Accepted values:
  - `pre_enable` — the drive sets 0x6061 (mode display) to the requested 0x6060 before enable;
  - `post_enable` — 0x6061 stays 0 until the first enable.

  An unknown value is refused by the file layer. Left out, the loader writes `pre_enable`: the same spec, and so
  the same §7 digests, as a file that writes it. `pre_enable` is the stricter reading: release is admitted only
  once 0x6061 shows the declared mode. The assembly and bus gates still refuse `AxisSpec::mode_display_timing`
  `UNDECLARED` from code that builds a spec without the file.
- Carry: `AxisSpec::mode_display_timing` → `ServoInfo` → `DeviceServo`. The loader projects the value as it is. The
  execution meaning (the first zero admission under `POST_ENABLE`, grace, trip episodes) is a core contract
  the file does not change.
- It is an input of the §7 normalization and the §6 approval digest; changing it voids an approval.
- When to write `post_enable`: a drive whose 0x6061 reads 0 at rest and shows the mode only once enabled. Under
  `pre_enable` its release is refused with `MODES_OF_OPERATION_MISMATCH` and a display of 0 on `rt_events`, and
  the refusal text of `Bus.release` and the bench names this key. CiA 402 does not say when 0x6061 follows, so
  this key declares a vendor characteristic; it does not mark a standard violation.

#### 4.2b `mode` — the axis's operating mode and command

`mode` is the axis's 0x6060 value: `cst` 10, `csp` 8, `csv` 9. It is sent from startup on every cycle; there
is no runtime switch. The host command (`IDriveHost::write`; under `plem drive run` the hardware interface
writes the controllers' commands) has one value per axis, and each value's unit is its axis's mode: `cst` Nm, `csp` rad, `csv` rad/s (joint coordinates relative
to `zero_offset_count`; `flip` is applied once at the PDO boundary). The bus reports the modes in `~/info`
as `modes_of_operation` (int8, the 0x6060 values); `plem drive check` prints it per axis (`axes[i].mode` with `--json`).
The required PDO roles are the common roles plus the mode's target (0x6071, 0x607A, 0x60FF); other targets
are optional (source of truth: `validate_pdo_mapping`). `csp` ran in every hardware session (two A6B drives,
[`docs/hardware-evidence.md`](hardware-evidence.md)); `csv` and `cst` in shorter runs listed there.

**When a command is applied** (`EtherCATController::run_drive_command_cycle`). A command goes to the drive
only in a cycle where the torque gate is open (host enable ∧ every axis Operation enabled ∧ no protective
stop), the host has written a command (stamp ≠ 0; on a bus without a `cst` axis, a command written **after**
the last release; "Release hold" below), and **every** axis's command passes its check. On such a bus the
release hold and a requested stop (§4.10) send targets the core makes, in place of the host's, through the same
checks. In every
other cycle all axes get a neutral command: `cst` 0 Nm, `csp` 0x607A = that cycle's raw 0x6064 word
(mirror), `csv` 0 rad/s. Other mapped targets get 0 (0x6071, 0x60FF) or the mirror (0x607A).

Checks do not clip `csp` and `csv` values: a violation is a protective stop; a `cst` torque is clipped:

- `csp`: the command is encoded without saturation (outside int32 is a violation; on an endless axis,
  outside ±2⁴⁸ counts, §4.4); the angle it decodes back
  to must be inside `limits.position` and within `limits.velocity_rad_s × period` of the target this axis
  applied last cycle (or of the measured position, on the first application after a gap; on an endless axis,
  of the hold word last transmitted, §4.4). An axis whose
  measured position does not decode (on an axis with a finite `limits.position`, a word that does not decode
  to an int32 joint count, from a large `zero_offset_count`; on an endless axis, a count that left its domain,
  §4.4) is treated like a position limit violation, in **every** mode: the protective stop holds whether
  or not the axis is enabled, and reset is refused while it does not decode (on an endless axis, until PLEM
  restarts).
  A host that writes its planned speed with the command (`JointCommand::rate` in `DriveHost::write(command, rate,
  sampled_ns)`; under `plem drive run` the trajectory controller's velocity command) has its `csp` target advanced by
  that speed × the sample's age (from `sampled_ns`, the instant the host read the bus and planned from rather than when it
  writes, to the cycle that reads it) before these checks: the controller manager runs on its own clock, and a
  sample read late must not trip the one-cycle step check. A stamp taken at the write instead carries the host's
  own update jitter × speed into every target (at speed, a `CSP_STEP` stop). A write without a speed is applied as
  written.
  The advance follows the loop's actual wake, the step check the nominal period: between two frames the target
  moves by about speed × the wake-to-wake gap, so a gap longer than period × `limits.velocity_rad_s` / speed takes
  the step check's protective stop (`LIMIT_STOP` 5): an axis at 90 % of its limit trips on a gap of about 1.1
  periods. That is the safe side: the frame carries the whole move and the drive would get it within one cycle. With
  the bus loop at FIFO priority on an isolated CPU such a gap is rare; it shows as `loop_jitter_us` (gap − period: about 111 µs at 90 % on a 1 ms cycle), and an overrun
  of the cycle before (`deadline_miss`) is one cause.
- `csv`: the magnitude of the velocity the encoded word decodes back to is at most `limits.velocity_rad_s`.
- `cst`: clipped to `limits.torque_nm`.

A violation sets an internal protective stop in that cycle and goes to the forced brake engage. In that cycle
`cst` and `csv` axes get their neutral command, and a `csp` axis keeps the target it was sent the cycle before:
the stop controlword goes out only in the next cycle, and the raw 0x6064 word under Enable operation would step
the target back by the following lag, which a drive can trip on (two A6B drives faulted with Err27 on such a
step). From the next cycle every axis is neutral. The engage is not skipped even if an operator reset is consumed in the next cycle.
The same command after a reset trips again.

**Release hold** (a bus without a `cst` axis). A release (`DriveHost::enable`) does not need a command stream
first. The release is marked (`EtherCATController::open_command_epoch`) and a command written before it is not
applied. Until the host writes its first command after the release, a `csp` axis holds the position it found at
the release (a target taken from the measurement of the first cycle with the gate open and then kept; it does not
follow a sagging actual) and a `csv` axis commands 0 rad/s; these targets get the checks above. The host's first
`csp` command is judged against that held target for the one-cycle step. The
command freshness watch (§4.4, 50 ms) arms on that first command and rests while motion is off or a stop runs
(§4.10): a host that writes nothing keeps the hold, with no stale stop. A bus with any `cst` axis keeps the old
contract: fresh torque must stream before the release (otherwise `enable` refuses), and the watch arms on the
first host write. On hardware a `csp` release kept its position with the trajectory controller writing from the
next cycle, and a `csv` release held 5 s without a stream
([`docs/hardware-evidence.md`](hardware-evidence.md)); a bus with a `cst` axis has not run on this path.

**Planning speed.** `plem drive check --json` reports per axis `axes[i].velocity_cap_rad_s`: on a `csp` axis
`limits.velocity_rad_s` minus one position count per cycle (two on an endless axis; the second absorbs the binary64
error of angles far past int32 counts, §4.4), so a rounded target planned at that speed never crosses the one-cycle
step limit; on another axis `limits.velocity_rad_s` as is. A `csp` axis whose cap would be 0 or less is refused (raise
`limits.velocity_rad_s`, lengthen `bus.cycle_time_ns` or use a finer position channel). It is the speed a planner may
command, which `plem drive run` writes as the joint's velocity limit in its robot description. The same `axes[i]`
carry `velocity_limit_rad_s`, `acceleration_limit_rad_s2` (null on a `cst` axis) and
`position_min_rad`/`position_max_rad` (null on an endless axis). The measured-velocity check (`LIMIT_STOP` 2) uses
`limits.velocity_rad_s`, so a cruise just under the limit can turn the drive's tracking overshoot into a protective
stop. Plan below the limit to leave a margin (measure the peak measured velocity, `actual_velocity` in `rt_raw`, at
the transition to cruise).

**No ramp in the bus loop.** An axis's `ramp` block is refused with a message that says to delete it: motion is planned by the client (the trajectory
controller under `plem drive run`, `plem.drive`'s `Bus.move_to`), and a requested stop (§4.10) decelerates at
`limits.acceleration_rad_s2`.

**Cause event `LIMIT_STOP`** (0x4E). It is emitted once per violating axis **at the place that newly sets**
the internal protective stop: `joint_id` = axis, `error_code` = `LimitStopReason`, `value` = in the reason's
unit (rad, rad/s or the command unit; NaN for reason 3).

| `LimitStopReason` | Meaning |
|---|---|
| 1 | measured position |
| 2 | measured velocity |
| 3 | position does not decode (§4.4) |
| 4 | `csp` range |
| 5 | `csp` one-cycle step |
| 6 | `csv` velocity |
| 7 | command does not encode |

The safety stage emits only when no internal latch was set at the start of the cycle (a lasting violation or
a refused reset emits nothing; an external stop already set does not suppress it). The command stage runs
only in cycles with the gate open, so a violation there is always a new stop. With several reasons on one
axis, the first in judgment order is reported (decode → position → velocity; `csp` range → step), not the
lowest number. Pairing and order with the brake stage's `PROTECTIVE_STOP` are not guaranteed (a command
violation comes one cycle earlier). **Reason numbers are never reused** (a public header contract).

### 4.3 Three kinds of address

| Place | Meaning | Offset |
|---|---|---|
| `pdo_mapping.*.entries[].index` | the slave's **absolute** object address | none: channel 2's statusword is written `0x6841` |
| `slaves[].axes[].error_code_pdo.index` | **channel 1's base** (for example `0x603F`) | the loader **carries the base unchanged** (`ServoInfo` and `AxisPdoRequirements` take the base). Each consumer adds the offset **once**: the validator computes the mapped address as `base + od_offset` (`pdo_layout_validation.cpp`), and registration adds the offset too (`servo/device.cpp`). If the loader stored `0x683F`, consumers would use `0x703F` |
| `slaves[].startup_sdo[].index` | **absolute** address | none: the file computes any vendor stride itself (the core has no notion of stride) |

A refusal example, limited to what the code checks: when only `0x683F` is mapped and the base is written as
`0x683F`, the resolved address `0x703F` is unmapped and `validate_pdo_mapping` refuses it. `0x703F` itself
is not a forbidden address: if the mapping maps it as a correct 16-bit Tx role with no conflict, the code
does not refuse it.

`channel` is compared as `1 ≤ channel ≤ 8` on the parsed integer **before** multiplying (so a literal cannot
wrap around to look like channel 1). Eight is the CANopen logical-device layout: the profile area
0x6000–0x9FFF split into eight 0x800 blocks. The same bound is applied to C++ models by
`od_offset_on_channel_grid` (`configuration.hpp`, `kMaxCia402Channels`) at bus Step 0 and in the model spec
gate. It does not prove the physical channel exists: a one-axis model that uses only `channel: 2` is valid.

`error_code_pdo.index: 0` is refused by the file layer. 0 is the "no role" sentinel that the value gate and
Step 0 skip, so a declared Error code role would silently disappear. To declare none, write
`error_code_pdo: null`.

### 4.4 Per-axis physical safety — required declarations

Direction, zero, travel, velocity/torque limits and the stop deceleration are declared with the keys below. **Every one is required
and has no default**; the torque limit is required on `cst` axes only (it clamps torque commands, which `csp` and `csv`
axes do not send). Hold means, communication-loss behaviour and brake sharing each accept one value in
this version, so they are not keys but the **premises** listed after the table. A missing safety fact (a
shared brake, for example) cannot be declared in the file; such drives are kept out by the approval procedure
(§6.1).

| Fact | Key | Coordinates and unit | Consumer | Checked |
|---|---|---|---|---|
| Direction | `flip` | Sign inversion at the PDO boundary. With flip `hw = offset − value`, otherwise `value + offset` (`Pulse::to_hardware`, saturating) | `ServoInfo::flip` → `DeviceServo`. The sign is inverted on the host; PLEM does not write 0x607E (Polarity) | bool. The physical direction = the bench |
| Zero | `zero_offset_count` (int32) | The **hardware position count** at q = 0, in position encoder counts. Flip is not applied to it first. It is not the drive's Home offset (0x607C) or Position offset (0x60B0); PLEM does not write those | `ServoInfo::joint_offset` via `AxisSafety::zero_offset_count` | int32. Where the value came from and whether the reference survives a restart = the bench plus the §6 binding. `0` too only as a measured, bound value |
| Travel | `limits.position: {min_rad, max_rad}` or `null` | Output-shaft rad, joint coordinates relative to `zero_offset_count`. `null` = an axis with no end stops (±inf). The check runs on the host; PLEM does not write 0x607D (Software position limit) | `AxisSafety::position_min/max_rad` → `EtherCATController::set_drive_limits` → the RT drive safety step `run_drive_safety_cycle` (every cycle: limit check → internal protective stop; the forced engage sequence starts in the same cycle, and when the brake actually engages differs per drive) + the `csp` command check (§4.2b) (on hardware: the command check, and an axis outside the limits at start) | finite and min < max, or `null`; `null` requires `zero_offset_count: 0` and is an endless axis in every mode (below; `validate_drive_model_spec`) |
| Velocity limit | `limits.velocity_rad_s` | Output-shaft rad/s, > 0. The check runs on the host; PLEM does not write 0x607F or 0x6080 | the same step (`|v| ≤ limit`) + the `csp` one-cycle step and `csv` command checks (§4.2b) (on hardware: the measured-velocity stop and the `csv` command check) | finite and positive (`validate_drive_model_spec`); on a `csp` axis also above one position count per cycle (two on an endless axis), so that `velocity_cap_rad_s` > 0 (§4.2b) |
| Stop deceleration | `limits.acceleration_rad_s2`, **`csp` and `csv` axes only**: required there (missing is refused with a fix message), refused on a `cst` axis | Output-shaft rad/s², > 0 (the same joint frame as the other limits). The deceleration a requested stop brings the commands to rest at. The deceleration is applied on the host; PLEM does not write 0x6084 or 0x6085 (`startup_sdo` sets the drive's). Write no more than the drive's 0x6085 (§4.10) | `AxisSafety::acceleration_rad_s2` → `EtherCATController::set_drive_stop` → the requested stop (§4.10) | `csp`, `csv`: finite and positive; `cst`: 0 (`validate_drive_model_spec`; the file layer refuses a missing key, a value of 0 or less and the key on a `cst` axis first). Whether the axis really stops at it = hardware: the bench's `stop-decel` item checks only that the stop is, on average, no sharper than 1.1 × this value; on the A6B example (95 rad/s², no load) it measured about 33 to 43 rad/s² ([`hardware-evidence.md`](hardware-evidence.md)) |
| Torque limit | `limits.torque_nm: {min, max}`, **`cst` axes only**: required on a `cst` axis, refused by the loader on a `csp` or `csv` axis | Output-shaft Nm, **min < 0 < max required** (a clamp that cannot carry 0 Nm would distort a stop command). A host-side clamp; PLEM does not write 0x6072 (Max torque) | the clamp of `ServoInfo::min/max_torque_nm` (`DeviceServo::set_command`); on a `cst` axis `AxisSafety::torque_min/max_nm` is carried, so a default has no way in. A `csp` or `csv` axis sends no torque command, so it has no clamp: the drive's own loop produces its torque, bounded by the drive's 0x6072 (value spec 0/0, carried as the widest finite band, `MotionLimit::effort_limit` = +inf) | `cst`: finite, min < 0 < max; otherwise 0/0 (`validate_drive_model_spec`). Physical plausibility = the bench |

A position limit stop is not released while the axis is outside the limit.

**Saturation of the position.** On an axis with a finite `limits.position` the measured position is 0x6064
(int32), turned into an int32 joint count with `zero_offset_count` and `flip`, and **saturated** on overflow
(`Pulse`). A saturated position stays one value while the drive moves, so neither limits nor bench drift can
judge it. Such an axis therefore takes a protective stop, like a limit violation, while its word does not
decode (§4.2b). The value gate checks that the position conversion maps an int32 count to a finite, non-zero
angle (a diverging conversion would give an infinite measurement; a collapsing one would give 0 for every
position). The joint counts that decode are hw − z (−(hw − z) with `flip`) for every int32 hw, themselves within
int32; `plem drive check` warns (never refuses) when `limits.position` reaches past their angles and prints that
range, rounded inward to 4 decimals. Past that range the measured position is not the joint's: the joint count
saturates, or, with `zero_offset_count` 0 (−1 with `flip`), the drive's count wraps straight to the range's other end
and decodes there, so a limit past the range does not hold. Velocity and torque are separate PDOs and are not affected by position
saturation or wrap.

**Endless axis** (`limits.position: null`). The same in every mode (`cst`, `csp`, `csv`). There is no new key:
`drive_servo_infos` sets `ServoInfo::position_endless` (the config digest does not change). The host follows
the raw 0x6064 word with an int64 count (`DeviceServo::read`):

- **Observation.** Only a frame with a complete working counter (`note_received_frame`) and a non-zero
  statusword is an observation. Any other frame moves nothing, the zero image of a lost frame included. The
  first observation is the seed: right after start the position is the drive's word as it is (within about
  ±1608 rad on a 23-bit encoder at gear 1). No origin is stored.
- **Advance.** The word difference between two observations is read the short way modulo 2³² (`wrap_diff`)
  and added. So on every valid frame count ≡ word (mod 2³²), and every 0x607A word PLEM writes is congruent
  with the drive's own frame. Only the reported absolute can be off by k·2³² counts: after a restart, a drive
  re-initialisation or an over-long gap. The `csp` raw hold writes the last **observed** word (never an invalid
  frame's word), so the held word and the reported angle are always the same count. The first application
  after a hold (after a restart, after a gap with the gate closed, during or right after invalid frames)
  measures its step from the hold word last **transmitted**: a drive that moved
  further than one cycle's travel across invalid frames makes that resume a `csp` step violation (the stop
  frame's neutral is the newly observed word). Not so when the drive re-derives 0x6064 in OP while the axis is enabled: application continues
  from the previous target (`csp_applied_rad_`), does not see the jump, and the drive receives the old target
  in its new frame. Only the operator procedure below (coordinate-changing maintenance while stopped, then a
  PLEM restart) avoids that case.
- **Gap ceiling.** The displacement between valid observations must stay under 2³¹ counts (exactly 2³¹ reads
  as −2³¹): on a 23-bit encoder, 256 motor revolutions, or 2.56 s of invalid frames at 6000 rpm. Beyond it the
  absolute shifts by k·2³². This is a documented limit; it is not detected.
- **Domain ceiling.** |count| ≤ 2⁴⁸ (`kEndlessCountDomain`): on a 23-bit encoder 2²⁵ ≈ 33.5 M motor
  revolutions of net one-way travel since PLEM started, about 6.1 days at 400 rad/s motor speed, 3.9 days at
  6000 rpm, 122 days at 20 rad/s. Beyond it the candidate is dropped and the axis takes a protective stop as
  undecodable (`LIMIT_STOP` 3); reset is refused until PLEM restarts, and the restart folds again. There is
  no running re-fold.
- **Angle and target.** angle = (flip ? −count : count) × 2π ÷ (`encoder_resolution` × `gear_ratio`),
  computed once per valid `read()`. A `csp` target that is non-finite or more than 2⁴⁸ counts away is
  unencodable (`LIMIT_STOP` 7); otherwise the rounded count goes on 0x607A folded modulo 2³², and that count's
  angle is the applied command (`applied_command` in `rt_raw`).
- **Value gate.** The position conversion must map −2⁴⁸ counts to a finite, non-zero angle (int32 alone is not
  enough: `encoder_resolution` 1 with `gear_ratio` 1e-298 is finite at 2³¹ counts and infinite at 2³²). On a
  `csp` axis one cycle's largest increment, `limits.velocity_rad_s` × period ÷ count + 2, must stay under 2³¹
  counts (the drive takes 0x607A as differences of successive words; about 1.6e6 rad/s at 1 ms on a 23-bit
  encoder, so it does not bind on a real drive). The planning speed keeps two counts per cycle (§4.2b).
  `zero_offset_count: 0` is required.
- **Precision** (estimates; the tests pin it). Inside 2⁴⁸ the binary64 error of one step is about 0.13 count
  or less, inside the second spare count. At 2⁴⁸ `joint_states` resolves about 1/16 count or finer; the float32 RT
  sample (`rt_raw`) resolves 1.2e-4 rad at 1600 rad and about 16 rad near 2⁴⁸ (gear 1); monitoring only.
- **Drive re-initialisation.** When the drive re-derives 0x6064 (A6B: p.18 and p.247: power-on, ESM
  INIT→PREOP, homing, multi-turn clear, PANATERM operations and pin assignment, Err27.4; p.258: a driver
  reset after the pulse position wrapped with a drive electronic gear other than 1:1), a change seen on valid
  frames is integrated like motion; a change of 2³¹ counts or more aliases. The congruence still holds; there
  is no re-initialisation detector. The count freezes only while communication actually fails (WKC), and only
  consecutive failures past the watchdog threshold are a protective stop. **Operator procedure:** do any
  coordinate-changing maintenance (homing, multi-turn clear, PANATERM operations, electronic-gear or polarity
  changes) while stopped, then restart PLEM.

**Drive requirements for an endless axis** (compatibility conditions; PLEM does not check them):

- R1: 0x6064 wraps over the full 2³²-count period: no smaller drive-side modulo (no narrowed 0x607B, no A6B
  continuous rotating absolute mode 3688h, whose wrap boundary depends on settings and is not a 2³² period).
- R2: the drive takes a 0x607A word wrap as continuous motion.
- R3: no drive-side software position limit trips on the wrap.
- R4: the drive starts again after net travel past its 32-bit range. A drive with a multi-turn absolute encoder
  derives 0x6064 from the encoder at power-on and at other initializations; when that absolute position no longer
  fits 32 bits, some drives raise a counter-overflow alarm and do not start. Run such a drive with its encoder in
  incremental mode or with a setting that ignores that overflow while keeping the full 2³² wrap (R1), or give
  the axis position limits so its net travel stays inside the drive's range. This holds in every operation mode
  (`csp`, `csv`, `cst`): the encoder counts every turn. PLEM warns once per axis at stop when its count for that
  run (counted from the drive's value at start) ended outside the 32-bit range; it does not see wraps from
  earlier runs, the drive's own offsets or a drive-side electronic gear other than 1:1 (which moves the drive's
own limit), so no warning does not prove the next start is fine.

The evidence is one drive, the A6B (SX-DSV03242 R10.1; page = the printed `No. SX-DSV03242 - N -` label):
p.261: outside continuous rotating absolute mode 0x607B is treated as 80000000h/7FFFFFFFh, and 0x607A going
7FFFFFFFh → 80000000h is a 1-unit move in the positive direction (wraparound processing; R1, R2); p.100-101:
0x607D is stored in EEPROM, is disabled when 01h ≥ 02h (for example 0/0), and a wrap while it is enabled raises
Err88.3 (R3); p.255-256: with an electronic gear under 128 the actual position must fit 32 bits (±256 motor turns
at 23 bits with a 1:1 electronic gear; the range scales with the gear), otherwise Err29.1 (counter overflow
protection 1) at the next initialization, and Pr6.98 bit 3 = 1 (function extended version 3 and later) ignores
the upper 7 bits of the multi-turn data so a one-way axis does not raise it; depending on the electronic gear, the
position after a power cycle may then differ from before (R4). For any other drive, check its manual: the CiA 402 wording, other vendors' wrap handling and
following-error windows, and whether the A6B's 0x6065 check is continuous across the wrap are not verified.
PLEM does not write 0x607D; write the expected values in `stored_sdo` so the bus reads them at start (a
warning only, §4.9). For an A6B axis without position limits the entries would be (vendor-specific; the installed
`a6b_pair` example has position limits and does not carry it):

```yaml
    stored_sdo:   # A6B: software position limit off (607Dh-01h >= 607Dh-02h)
      - { index: 0x607D, subindex: 1, size: 4, value: 0 }
      - { index: 0x607D, subindex: 2, size: 4, value: 0 }
```

**Premises** (the file does not write them):

- **Hold = a power-coupled brake.** After disable the load is held by the brake. The engage sequence (the
  release-step contract of `brake_logic.hpp`) is the only hold means: `DriveHost::disable` posts it and the
  quiesce of `stop` confirms it settled (disable itself does not confirm). `cst` does not hold position. The
  `csp` mirror is not a promise to keep a fixed position (it is each cycle's measurement), and `csv` 0 rad/s
  is a velocity-loop target during Operation enabled, not evidence of zero torque or of an engaged brake.
  A stop on a limit, a command gap or WKC loss, and every stop on a bus with a `cst` axis, is the standard CiA 402
  disable sequence (Disable operation, not Quick stop 0x0002), so deceleration, brake timing and the behaviour
  of a gravity-loaded axis differ per drive. On that path the core does not slow a moving axis before the
  disable: the drive's 0x605C and its deceleration (typically 0x6084) decide, and on lost communication its
  0x605E (typically at 0x6085); `startup_sdo` sets them. `protective_stop` and the end of a run on a bus without
  a `cst` axis are a requested stop (§4.10): the core decelerates first at `limits.acceleration_rad_s2` and, if
  that is not confirmed in time, commands Quick stop and leaves it to the drive's 0x6085 and 0x605A. If the drive
  enters Quick stop active and stays there after stopping (0x605A 5–8), the engage on the disable path does not
  complete: there the core does not command Disable voltage (transition 12) from that state, because `csp`, `csv`
  and `cst` carry no standard deceleration-complete bit (the A6B leaves bit 10 unused in these modes) and a drive
  may cut torque at once on a Disable voltage received mid-ramp. Only the requested stop sends Disable voltage, to
  an axis still in Quick stop active 2 s after its Quick stop command (§4.10). Write a 0x605A value of 1–3 that the
  drive supports in `startup_sdo` (the drive then goes to Switch on disabled by itself after stopping). `plem drive check` adds to its warnings (`warnings` with `--json`), one line per axis, what a `csp` or `csv` axis's slave leaves out of `startup_sdo` at the axis's channel (subindex 0): the
  stop objects 0x605A, 0x605C, 0x6084, 0x6085; the drive keeps its stored values. 0x605E is not checked. The
  following-error objects 0x6065, 0x6066 are not warned: exceeding the window sets statusword bit 13, PLEM does not act
  on that bit, and on the A6B the window only sets the bit (the alarm is the drive's own position deviation setting, A6B
  Pr0.14). The drive's following lag (on the A6B coupled rig about 32 ms × velocity, drive side; its command filters explain
  about 16 ms, the rest is not explained) exceeds the factory window, so bit 13 can be set in normal motion. The warning
  leaves the file accepted and the exit code unchanged, and a declared write says nothing about whether
  its value suits the machine. On hardware (A6B, `csp`; motion disabled at once, as a limit or a command gap does),
  with factory values of about 0.75 rad/s², an axis stopped while it was slowing through about 46 rad/s kept turning
  at about 30 rad/s for seconds. A stop of a fast `csv` axis on this path was not checked on hardware (a silenced
  stream at 10 rad/s was). A 0x60FE output brake, an external brake or no brake is not supported, and nothing detects
  one (neither the loader nor the bench sees the brake type). Do not use such a drive.
- **Communication loss = protective stop.** The WKC watchdog (5 ms of sustained working-counter mismatch,
  `EtherCATController::kWkcLossThresholdNs`) and command freshness 50 ms
  (`plem::drive::kCommandStaleTimeoutNs`; on a bus with a `cst` axis armed **after
  the first host write**, on a bus without one armed on the first host write after the release and at rest while
  motion is off or a requested stop runs; §4.2b "Release hold") → external protective stop → neutral commands on
  all axes (§4.2b) and forced engage. The basis of 50 ms: on JetPack 7.2.1 with ROS 2 Jazzy under load (10 minutes,
  load 8–9) an rclpy publisher's largest gap between messages was 41 ms at 200 Hz and 35 ms at 1 kHz, never over
  50 ms. The margin of another host, load or publishing path was not measured. On hardware the stop has run from paused or
  silenced `csv` streams (up to 10 rad/s) and a silenced `cst` stream on an axis at rest; the stale event's age read 0.051 s each time it was recorded (the sixth session, and twice in the later Case 1 sessions)
  ([`docs/hardware-evidence.md`](hardware-evidence.md)). A stale command stream's stop controlword goes out in the same cycle as its neutral
  commands. If the master itself dies, the drive's output SM watchdog applies. The validator
  (`validate_drive_model_spec`) enforces only its declaration (`watchdog: true` on an output SM, declared
  slaves only; a scanned slave is not judged because the core does not write it, §4.1). Its time is the
  master default (`EC_WD_DEFAULT` = the drive's ESI value). That this watchdog stops the drive when the master
  dies is a premise, not measured; no required bench item exercises a dead master.
  Recovery = operator reset (`DriveHost::reset`).
- **Brakes are independent per axis.** A drive where one brake releases several axes together is **not
  supported**. The file has no key for that fact, so **the loader cannot refuse it**; the approval procedure
  excludes it (§6.1). Sharing is not assumed just because axes are on the same slave.
- **The encoder is absolute.** The core has no homing: `zero_offset_count` and `limits.position` locate an axis
  only if the drive reports the same 0x6064 position for the same physical position after every power-on (a
  battery-backed or multi-turn absolute encoder in an absolute mode). An incremental encoder (or an absolute one set
  to an incremental mode) restarts its count at power-on, so the limits would guard a different range than they
  name and the bench's evidence would bind to a coordinate that no longer exists. The file has no key for the encoder
  type and the drive does not report it, so **nothing refuses such a drive**. Do not use one on an axis with
  position limits. An endless axis (`limits.position: null`, below) has no position limits for this to protect;
  after a restart it starts again from the drive's own value anyway.

### 4.5 Names

- `model.name`: stored in a 32-byte array with NUL (`DriveModelSpec::model_name`) → **at most 31 bytes**,
  counted in UTF-8 bytes. It is never cut by characters, and never cut so that two names become equal:
  too long is refused. An empty or whitespace-only name is refused (the file layer refuses whitespace-only
  first). ASCII control characters (C0, DEL) are refused by the loader: an inner NUL would silently cut
  `abc\0def` to `abc`, and a line break would split the log line that prints the name.
- `slaves[].axes[].name`: an ASCII identifier `[a-z_][a-z0-9_]*`. Empty, whitespace and NUL are not allowed,
  and a duplicate in the file is refused. Stored in 32 bytes with NUL (`kMaxAxisNameLength`) → **at most 31
  bytes**. It becomes the name in `joint_states`. Source of truth: `validate_drive_model_spec`.
- `bus.name`: the bus's ROS name: `plem drive run` starts the bus in the namespace `/<name>`, and
  `plem.drive.Bus(name)` connects to it. One ROS name token: letters, digits and `_`, not starting with a
  digit, at most 64 bytes. Optional to the loader (a file read by another host, such as a robot tier, carries
  none); `plem drive check` and `run` refuse a file without it. It is not part of the spec, so it is not in
  config_digest: renaming a bus keeps its approval (§7).

### 4.6 `fault_reset` — one setting for the model

The policy and pacing are set once for the whole controller (`controller.cpp`); there is no per-axis form. A bus whose
drives need different pacing is not supported.

| Policy | `pacing` | Rules (source of truth: `validate_fault_reset`) |
|---|---|---|
| `none` | not written | declared pacing is refused by the file layer (`pacing: null` too: a null in a mapping place) |
| `bounded_auto_reset` | `{ interval_ms, max_attempts }` required (a missing `pacing` is refused by the file layer) | `interval_ms > 0` (uint32), `1 <= max_attempts <= 10000` (a count limit, not a wall-clock limit and not a promise that that many attempts are safe); `validate_fault_reset` (`brake_logic.hpp`, value gate). `interval_ms` must also be expressible as a cycle count at the bus period (§5, a setter predicate after bus initialize). Both checks apply |

**`pulse_ms`** (required, 0..1000, file layer) is the **minimum time** (ms) controlword bit 7 stays high for one fault
reset. It applies to every reset (manual and automatic) whatever the policy.

- The bus turns it into a frame count rounded up at the period (at least 1), so **`0` = one frame at any
  period**.
- If the drive manual asks for a hold time, write that value (some drives ask for 10–20 ms or more).
- The pulse ends early in the frame where the axis leaves FAULT, even with width left (that frame is the
  falling edge).
- While the pulse holds bit 7, automatic attempts are not taken and not counted. A frame with only the
  falling edge left is taken like a one-frame pulse, and the attempt waits until after that edge.
- After stop and restart, the remaining width is not continued; only the one-frame falling edge is used.
- The upper bound of 1000 ms is not a vendor value. It guards typos and the frame-count range.
- What is claimed is only the shape of the word the host writes. Whether the drive accepted it or reset is
  not claimed; check `status`.

### 4.7 SDO — a static startup write list

`slaves[].startup_sdo[]` = an **ordered list** of `{ index, subindex, size, value }`, executed with IgH
`ecrt_slave_config_sdo*`.

- `size` is in bytes and only 1, 2 and 4 are accepted (`ecrt_slave_config_sdo8/16/32`); up to 16 writes per
  slave. `value` is the **unsigned wire bit pattern** of that width: `0 <= value < 2^(8·size)`, checked before
  loading. Negative values are not supported (no silent wrapping of `-1` until a sign convention is chosen on
  purpose). The width is never inferred from the value.
- Order and repetition are part of the data: for example the same address written several times with
  different values, in order. No automatic merging, sorting or last-wins. Order is kept **within** a slave;
  there is no guaranteed total wire order **between** slaves.
- There are no `wait`, `if`, `retry`, `read-until` or callback keys.
- A parameter that takes effect only after a power cycle gains nothing from a startup write that lands after
  power-up; declare the value the drive must hold in `stored_sdo` instead (§4.9).
- **Writes to objects the core owns are refused.** Owned = the core writes it, or another key of the file
  declares it. The core does not write limit or zero objects to the drive; they are refused only so the file
  never has two sources for one fact. Checked by `validate_drive_model_spec`. Addresses in `startup_sdo` are
  **absolute**, so the check expands `base + od_offset` for **every axis of that slave** (width-checked before
  adding):
  - for the slave: PDO assignment and mapping (0x1C10–0x1C2F, the range IgH `ecrt.h` lists as assignment
    objects; 0x1600–0x17FF, 0x1A00–0x1BFF), and synchronization (0x1C32, 0x1C33: synchronization is
    declared by `sync_mode` and `dc`, and the core writes only 0x1C32:02, for `dc.write_cycle_time`, §5);
  - for each axis, at its channel base: controlword 0x6040, mode 0x6060 (owned by `mode`), command targets
    (0x6071, 0x607A, 0x60FF), the limit objects 0x607D, 0x6072, 0x6080 (competing with `limits`) and the zero
    0x607C (competing with `zero_offset_count`). Example: in a file with a channel 2 axis,
    `{index: 0x6860, size: 1, value: 8}` competes with that axis's `mode: cst` and is refused.
  - Undeclared physical channels and vendor extension objects are not classified: they are accepted, and
    their effect is the declarer's responsibility. Structural validation does not promise to judge the
    physical effect of an arbitrary vendor object.
- **Side effects before approval.** Startup writes are scheduled for PREOP→SAFEOP, so they change the
  hardware state before the motion mode is entered.
  - Scheduling belongs to assembly. `DriveHost::create` passes the value-gated list to the bus; the bus
    schedules all of it before master activation and logs one line per write.
  - An unapproved spec schedules them too (the motion gate is `enable()`). What a write does to the drive
    is the declarer's responsibility; the validator checks only addresses.
  - A host is single-use: re-enabling after stop is a new host that re-checks and re-schedules the whole list.
    `ecrt.h` documents that if the master reconfigures a slave during one host's life (after a power cycle,
    for example) the scheduled writes are written again; this was not measured.
  - If the drive refuses a write (SDO abort), the core does not report the startup SDO as the cause (it does
    not read per-slave configuration state). How that shows up was not checked.

Registering runtime SDO endpoints (`axes[].extra_sdo`) is not a version 1 key (§9).

### 4.8 `cia402_profile` — left out = the core's built-in table; written = the drive's own table

- **Without the key** every axis uses the core constant `fieldbus::kStandardCia402Profile` (12 rows: a
  subset of CiA 402 transitions 2–12 and the one-word 3+4 row; 16 is not declared). It has no non-standard
  rows.
- A drive that accepts Shutdown in `NOT_READY_TO_SWITCH_ON` (and reports 0x0020 there) writes a table with
  that row in front (the C++ constant `kNotReadyShutdownCia402Profile`, 13 rows).
- That constant is not a "fully hardware-verified profile". Its documentation states, per row group, that
  **acceptance of the disable-voltage rows into SWITCH_ON_DISABLED (transitions 7/9/10/12) and the row out
  of SWITCH_ON_DISABLED (transition 2, Shutdown) was not confirmed on hardware.** A file that leaves the key out inherits
  the same unconfirmed rows.
- The loader logs that it used the built-in table at debug level (`PLEM_LOG_LEVEL=debug`), and
  `plem drive check --json` reports it as `profile_source: built_in`. That and the bench are what show a vendor
  difference the omission may hide.
- **Only a drive that differs from the standard** writes a top-level row list, one per file, applied to every
  axis:

  ```yaml
  cia402_profile:
    - { from: switch_on_disabled, to: ready_to_switch_on, controlword: 0x0006 }
    - { from: ready_to_switch_on, to: switched_on, controlword: 0x0007 }
    # …
  ```

  State names are the core's names (`cia402_state_name`) in lower case: `not_ready_to_switch_on`,
  `switch_on_disabled`, `ready_to_switch_on`, `switched_on`, `operation_enabled`, `quick_stop_active`,
  `fault_reaction_active`, `fault`.
- Validation is the same as for a C++ table: `validate_cia402_profile` checks in the value gate the row
  count, valid states, self rows, bits 7 and 10, duplicate (from, to), that a command leads to its target, and
  that the ten required pairs are complete. It does not guarantee physical progress, vendor shortcuts or a
  full standard implementation. Controlword bits 4–6, 9 (mode), 8 (halt) and 11–15 (manufacturer) are not
  checked and go to the drive as written; not checked does not mean safe.
- The file layer refuses first only what can be wrong in the file alone: an unknown state name, more than 16
  rows, an empty list (leave the key out instead) and `cia402_profile: null` (null does not select the
  built-in table; leaving the key out does).
- A table that passes the code checks still needs its own approval for a new model. How the table enters the
  digest: §6 and §7.

### 4.9 stored_sdo — values the drive must have stored (read at start, warn only)

`slaves[].stored_sdo[]` (optional) = a list of `{ index, subindex, size, value }`, the same element as
`startup_sdo`, read as "this drive should hold this value". Left out = nothing is read; `[]` is refused (leave
the key out).

```yaml
    stored_sdo:
      - { index: 0x3401, subindex: 0, size: 4, value: 0 }
      - { index: 0x3015, subindex: 0, size: 2, value: 2 }
```

- **Width and value** follow §4.7: `size` 1, 2 or 4 bytes; `value` is the unsigned bit pattern of that width
  (a negative value is written as its two's complement in hex); the width is never inferred from the value.
  The bus compares the answer's bytes, read little-endian at `size` width, with `value`.
- **Refused only when malformed.** Everything is judged from the file alone:
  1. a `size` other than 1, 2 or 4, a value that does not fit it, or more than 16 values per slave (file
     layer for the count and the integer ranges, value gate for size × value);
  2. the same `(index, subindex)` twice in one slave: two expected values contradict each other (value gate);
  3. an address this slave's `startup_sdo` writes: the drive holds that write in RAM until the next power
     cycle, so reading it proves nothing and the file would have two sources for one fact. **An object moved
     to `startup_sdo` leaves `stored_sdo`** (value gate);
  4. a list on a slave past the slave count (value gate, for a spec assembled in C++).

  There is no static refusal of command, limit or mapping objects: what is exchanged every cycle is known
  only to the bus (the materialized layout, declared or scanned). Addresses are absolute (§4.3).
- **When it is read.** `DriveHost::create` hands the lists to the bus (`EtherCATBus::set_stored_sdos`). The
  bus reads them in `initialize()` Step 3.6, after the identity check (§6), before any PDO or SDO
  configuration and before master activation: one blocking `ecrt_master_sdo_upload` per value.
- **Not read, with one WARN each** (not a refusal, no upload):
  - an object mapped in an **output** sync manager of the slave's actual layout (declared or scanned): the
    master sends that whole image every cycle, including entries the axis mode does not use;
  - a PDO mapping or assignment object (0x1600–0x17FF, 0x1A00–0x1BFF, 0x1C10–0x1C2F): PDO configuration,
    not a stored setting.
- **Warn only.** A difference is one WARN line naming both directions: store the file's value (if the drive
  manual says the object is writable and storable: power-cycle the drive, download the value, save with CiA 301
  0x1010:01 if the drive implements it, power-cycle again if the object takes effect at power-up), or, if the drive is right (for example tuned on this machine), change
  the file. A read that fails is one WARN and the next value is read; an upload that fails without an SDO
  abort (no answer, a slave without CoE, or a mailbox error) skips the rest of that slave; a read interrupted by a signal ends the check. None of these refuses
  to start, and the check performs no writes. A slave whose values all match gets one INFO line.
- **The check reads each object's current value. It does not verify that a value is saved or in effect;
  neither a match nor a mismatch proves either.** A value downloaded but not saved matches until the next
  power cycle; a match on a parameter that takes effect at power-up does not mean it is applied.
- **Sync-manager parameters (0x1C32/0x1C33) include drive-maintained runtime entries; list only stored
  configuration entries.**
- **Startup wait.** The reads can add a startup wait with no upper bound, possibly the first blocking read
  of an init that had none (identity and DC reads run only when declared). PLEM sets no timeout of its own. A
  read still queued in the master ends on a signal (and the check with it). Once the transfer runs, only IgH's
  CoE handling bounds it (a per-step response timeout while the slave's mailbox stays empty), so a slave that
  keeps answering with other mailbox traffic can hold the start. An upload that fails without an SDO abort is
  read again after 0.1, 0.3 and 0.5 s (below); if the fourth attempt fails too, that slave's reads end. A slave
  lists at most 16 values: a longer list is refused (item 1 above).
- **Reads right after a bus scan.** No earlier than 3 s after each slave reaches PREOP, the IgH 1.6.13 master reads its
  SDO dictionary once (`EC_WAIT_SDO_DICT`) over the same mailbox; how long that takes depends on the drive's dictionary
  (tens of seconds on the A6B). A `stored_sdo` read that overlaps it can receive a dictionary reply and fail without an
  SDO abort (`ret=-5` in the WARN; `Received unknown response while uploading SDO` in the kernel log). On the
  maintainer's A6B pair the master sometimes scanned the bus again when a run stopped, and each scan opens this window;
  a start 3 s after the previous stop hit it, and so did one 34 s after a scan (load average about 11); one 1.5 s after
  a scan, before the dictionary read began, did not.
- **Retries.** Such a read (`-EIO` without an abort) is tried again after 0.1, 0.3 and 0.5 s, as are the approval
  record's hardware and software version reads and the position read before the first enable; a read that succeeds on a
  later attempt logs one INFO line. The delays add up to 0.9 s and each attempt is a full transfer again, so a slave
  that never answers this way costs 0.9 s plus three more transfers per read. A signal during a delay does not end the
  reads. If all four attempts fail, the message says `attempts: 4`. A `stored_sdo` failure does not refuse the start; a
  declared version that cannot be read does, and for this failure says to start again rather than to re-bench. A success
  after a retry does not prove the dictionary read was the cause. To compare, start again once the bus has been idle for
  a while.
- **Saving with 0x1010.** Whether an object can be written and stored, and whether the drive implements
  0x1010, is in the drive manual. "Store parameters" (subindex 1) usually saves **every** storable value the
  drive holds at that moment, including writes an earlier start made through `startup_sdo`. So the order is: power-cycle the drive (RAM
  back to the stored values) → download the value → save → power-cycle again if the manual says the object
  takes effect at power-up. Do it with the bus stopped.
- **Not in the approval.** `stored_sdo` is outside the config digest (§7): editing the list in an approved
  file keeps the approval, and the approval does not vouch for the list, nor that the drive held these
  values during the bench.
- `plem drive check` cannot read the bus: it applies the refusals above and prints nothing more.

### 4.10 Requested stop — the drive layer's stop category 1 sequence

`protective_stop` (`<ns>/plem_drive/control_command`, `SetControlCommand`) and the end of a run (Ctrl-C in `plem drive run`'s terminal: the hardware
interface's deactivation → `DriveHost::stop`) are `DriveHost::ramp_stop`: the core decelerates the moving axes first, then switches motion off and
engages the brakes (the sequence of an IEC 60204-1 stop category 1; it is not a safety-rated function). Source of
truth: `decide_ramp_stop` and `stop_velocity_step` in `drive_safety.hpp`, and
`EtherCATController::request_ramp_stop` and `run_ramp_stop_cycle` (`controller.cpp`).

- **When it applies.** Only on a bus without a `cst` axis, with motion live (released). On a bus with any `cst`
  axis, or with nothing released, `ramp_stop` is `disable`: motion goes off at once and the brake engage is posted; a
  moving axis stops by its drive's 0x605C reaction. A limit violation, a command gap (stale) and WKC
  loss still stop at once on every bus (Disable operation → the drive's 0x605C/0x6084, §4.4).
- **Deceleration.** In the cycle that takes the request, each `csp` or `csv` command starts from the speed it was
  last applied at: a `csp` axis's last target step over the cycle, a `csv` axis's last applied velocity, 0 after
  a cycle that applied nothing. Every cycle it then falls by `limits.acceleration_rad_s2` × cycle toward 0 and
  rests there; it never comes back past 0 (`stop_velocity_step`). A `csp` target advances from the previous target
  at that speed; a `csv` target is that speed. Host commands are not applied meanwhile. The stop's targets get the
  §4.2b checks.
- **End.** Once every command rests and every axis's measured speed (0x606C) is within 1 % of its
  `limits.velocity_rad_s`, motion goes off and every brake engage starts. No protective stop is latched; the next
  release needs no reset.
- **Not confirmed → Quick stop.** The deadline is the planned deceleration at the request (the largest |start
  speed| ÷ deceleration over the axes) plus 0.5 s (`kRampStopSettleTimeoutNs`). If the end condition does not hold by
  then, Quick stop is commanded to every axis in Operation enabled and the RT event `RAMP_STOP_TIMEOUT` (0x4F, `value` =
  seconds since the request) is emitted. The drive then ramps down at its own 0x6085 and ends in the state its
  0x605A selects (2 = Switch on disabled). Its leaving Operation enabled latches the external protective stop, so
  the next release needs `reset_error`. The stop ends when every axis reads a state that proves its brake closed,
  or after 2 s more (`kQuickStopTimeoutNs`); an axis still in Quick stop active then gets Disable voltage
  (transition 12), and the brake engage starts; if the drive's 0x6085 has not stopped the axis within those 2 s,
  power is cut on a moving axis. The Quick stop command needs the operation_enabled → quick_stop_active row
  (transition 11, 0x0002) in a `csp` or `csv` axis's transition table; the value gate refuses such an axis without
  it (the built-in table has it).
- **Motion ending on another path.** If another protective stop (a limit, WKC loss) closes the gate, the
  requested stop ends in that cycle and the engage follows. The command freshness watch rests while a stop runs:
  a host that stops writing during the stop does not trip stale.
- **While it runs.** `enable` (`release_brake`) is refused (the refusal says why). The request is asynchronous:
  the service response only means it was taken; read the end from the brakes engaged in `~/status`
  (`DriveStatus` has no stopping flag; a C++ host reads `DriveHost::State::stopping`). `DriveHost::stop` calls
  `ramp_stop`, waits for it to end (capped at 30 s, for a loop that stopped cycling), then disables and quiesces. A
  run that never released (no `enable` reached its release since start) only stops the loop: no brake was opened,
  and a slave short of OP could not confirm one anyway. Its outcome has `never_released` set (the masks show every
  brake engaged, as before a start, but nothing was observed); check it before reading `fully_engaged()` as a
  confirmation.
- **Choosing the value.** A deceleration the axis can really brake at, no more than the drive's 0x6085 (Quick stop
  deceleration): a stop that cannot follow it is not confirmed in time and falls to Quick stop. PLEM does not write
  0x6085 or 0x605A; `startup_sdo` sets them (the `check` warning of §4.4 names a missing one).
- **Scope of the claim.** Unit tests (a fake master among them) pin this sequence. On hardware it ran on two rigs with the
  same two A6B drives on JetPack 7.2.1 (`csp` axes from up to 17.9 rad/s with the outputs coupled, and with outputs free;
  [`docs/hardware-evidence.md`](hardware-evidence.md)) without reaching the Quick stop fallback, which has not run. The bench item
  `stop-decel` (§6.1) measures it once from cruise on every bus with a `csp` or `csv` axis and no `cst` axis; it ran
  on `csp` buses on both rigs and on a `csv` copy on JetPack 7.2.1.
  The 1 % band, 0.5 s and 2 s are design values, not measurements. How a drive reacts to a Quick stop command in
  motion, and what its 0x605A values mean, differ per drive (the A6B's 0x605A value table is not in the manuals
  held; §10).

## 5. Cycle and distributed clocks

- `bus.cycle_time_ns`: the unit of ETG 0x1C32:02 Cycle time (ns), and a multiple of 1000 (the core runs in µs;
  a non-multiple is refused by the file layer, never rounded). The value must be in the range this release
  accepts: 200000–1000000 ns (value gate; the constants are
  `kMinCycleTimeUs`/`kMaxCycleTimeUs` in [`plem_drive/include/plem/drive/models/ethercat_slave_spec.hpp`](../plem_drive/include/plem/drive/models/ethercat_slave_spec.hpp), checked
  in [`plem_drive/src/models/value_gate.hpp`](../plem_drive/src/models/value_gate.hpp)). That is not a universal range for all CiA 402
  drives and controllers; other periods are for the bench to show. The drive layer has one period source (the
  value spec's `cycle_time_us`; `DriveHost` has no separate knob), so "declared ≠ executed" cannot happen.
- DC (declared once per slave). The checks of `configure_DC()`: active period ≠ 0, `assign_activate ≠ 0`,
  `sync0_cycle_ns ≠ 0`, and **`sync0_cycle_ns % bus.cycle_time_ns == 0`** (a multiple, not necessarily equal)
  (`bus.cpp`). `sync0_shift_ns` keeps its int32 sign. PLEM does not check that it is at least 0 or below the
  period; that is for the drive, the master and the bench. `plem drive check` warns on a DC
  slave whose `sync0_shift_ns` is 0 (see below: SYNC0 then fires before that cycle's frame; a negative shift is
  the same phase as period minus it and is not warned).
- **Choosing `sync0_shift_ns`.** It is the master-side SYNC0 shift, passed with the SYNC0 cycle to the EtherCAT
  master (`ecrt_slave_config_dc`): SYNC0 fires that long after the DC application time, which PLEM sets when its
  RT loop wakes, before it sends that cycle's frame. It is not the slave's own Shift time (ETG 0x1C32:03), a
  different object a drive may not support (the A6B does not). SYNC0 must fall after the cycle's frame has reached
  the slave, with margin: choose a shift above the host's worst wake-to-send time plus the frame's transit to the
  slave plus a margin, and below the cycle minus the drive's own processing time (its manual). Write it relative
  to the cycle. Too small a shift shows as intermittent torque spikes or following-error jumps while the host's
  target is regular. Measured on two A6B drives at 1 ms ([`docs/hardware-evidence.md`](hardware-evidence.md), fifth session): the
  frame left about 12 µs after the wake (p99 15 µs, max 55 µs); at 0 SYNC0 fired about 4.7 µs after the wake,
  before the frame, and the drives' position demand (0x6062) now and then skipped or repeated a cycle's target;
  20 µs still did; 300 µs did not in 10 runs. The example's 300000 is that measurement; it does not carry to
  cycles of about 300 µs or less. Coupled operation (the jaw-joined pair) ran at 300 µs too, about 33 s of
  23.5 rad/s motion per run: with the nvpps PTP-TSC lock disabled three such runs showed no DC transient and no
  guard trip; one earlier run with the lock on showed a short burst during a DC deviation excursion of about 300 µs. Not run: other cycle times, other drives, the drive-side input shift (0x1C33).
- 0x1C32:02 (Cycle time) differs from drive to drive. In the reference slave stack model (Beckhoff AN ET9300;
  the ETG.1020 text itself was not opened), a slave whose 0x1C32:01 the master does not write takes its
  synchronization from the activation register 0x0981, and :02 mirrors 0x09A0 (the SYNC0 cycle). Some drives
  have :02 read-only or no 0x1C32 at all; some take :02 from the master. So the core does not write 0x1C32 by
  default, and the drive file says when to:
  - `dc.write_cycle_time: true` (optional: left out = not written; an explicit `false` is refused by the file
    layer). The bus reserves one U32 startup SDO 0x1C32:02 = `sync0_cycle_ns` **per
    physical slave**, written at PREOP → SAFEOP. It comes from the same field that programs SYNC0 (0x09A0), so
    the two requests cannot disagree (this does not show the hardware kept both). 0x1C32:01 and 0x1C33 are not
    written. A refused reservation (`ecrt_slave_config_sdo32` < 0) fails `initialize()`; an abort on the wire
    stops that slave before SAFEOP while `ecrt_master_activate()` has already returned success (IgH 1.6.13);
    the release gate below keeps it from being released. The key enters the digest only when true (§7). Axes
    of one slave that disagree on DC (`assign_activate`, SYNC0 cycle, shift, `write_cycle_time`) are refused by
    the bus's Step 0 (the drive layer carries one slave declaration to all of them, so they always agree).
  - **When to set it**: only when the drive's ESI or manual lists 0x1C32:02 as rw and says the master sets it
    (the A6B; MR-J4-TM has :02 rw, but its manual does not say :01 follows the DC registers (default Free Run),
    so it is unconfirmed there, and PLEM does not write :01). Leave it out where the drive's ESI or manual shows
    :02 read-only or has no 0x1C32: set there, the write is expected to be refused (CoE abort on a read-only
    object), the slave then stays in PREOP (the kernel log shows an SDO abort on 0x1C32:02) and release is
    refused.
  - The core **never reads** the drive's 0x1C32: before DC is programmed a drive that mirrors SYNC0 reads 0 or
    a stale value, which cannot be judged. A cycle the drive cannot run is refused by the drive: at PREOP →
    SAFEOP, or as a sync error that drops the slave out of OP (AL status 0x002C with the slave
    falling back to SAFEOP in a loop is an example of the second kind). The cause is the AL status code in the kernel log. That is documented A6B
    behaviour and the reference model, not a guarantee for every vendor.
- Whether DC is used. A slave declares DC **capability** (`dc`); the required key `bus.sync_mode` decides
  whether it runs. `dc_sync0` follows the ETG 0x1C32:01 Sync mode naming: turn on slave DC (AssignActivate,
  SYNC0 period). `drive_default` is PLEM's own name: PLEM configures no synchronization (it writes no
  0x1C32:01 and turns no DC on), so the drive keeps its own setting, Free Run or SM-synchronous as that drive
  does (the example's A6B drives ran Free Run: 0x1C32:01 and 0x0981 read 0 in OP). Other values are refused.
- **`dc_sync0` rests on one drive model.** On the example's two A6B drives, with 0x1C32:02 written by hand
  (CLI, RAM), the drives reached OP and moved together, their gearhead outputs joined by a jaw, without a drive
  alarm ([`docs/hardware-evidence.md`](hardware-evidence.md); whether the jaw carried load was not established); with PLEM making that write (`write_cycle_time`) the example ran
  there with the gearhead outputs free (`gear_ratio: 1.0`) and joined (`gear_ratio: 21.0`; the jaw's load was not measured). PLEM does not claim DC support for another drive until its
  DC values (assign_activate, SYNC0 period, how it treats 0x1C32) are confirmed and that configuration has run
  on hardware. With `dc_sync0`, every slave writes a `dc` block. A missing `dc` is refused by the file layer,
  an `assign_activate` 0 by the value gate (`validate_drive_model_spec`); `sync0_cycle_ns` 0 and the multiple rule are
  refused by `configure_DC` after the master is acquired. With `drive_default`, `dc` is not written (writing it is
  refused by the file layer, §1).
- A slave that does not reach OP. The release gate requires every axis's slave to be operational (the
  master's slave configuration state: in OP, no reconfiguration pending): in SAFEOP the inputs read valid, so a
  statusword and mode display can pass, while the drive ignores the outputs. The state is read once per slave
  in every brake step that consumes a release request, after the consume (nothing is read without a request);
  a refused axis is named with `SLAVE_NOT_OPERATIONAL`, which carries the AL state and the ring position (AL state 0
  offline, or the state could not be read; 1 INIT; 2 PREOP; 4 SAFEOP; 8 OP with a reconfiguration pending). The event does
  not tell the causes apart. `plem drive run` comes up only once every slave is in OP, so the slave has usually left
  OP since: check the kernel log (`sudo dmesg | grep -i ethercat`) for an AL status code, or a refused
  startup SDO (`startup_sdo`, or `dc.write_cycle_time`'s 0x1C32:02). The gate holds on a bus without DC too.
- DC settling. After start, `dc_sync_monitor_ns` on the A6B drives reached about 1 ms for 15–30 s (IgH 1.6.13
  sends no dedicated static-drift burst and does not reset the drift filter), then p50 0.1–0.3 µs and at most
  about 1.5 µs. It does not stay there: in every run longer than 90 s an excursion of up to a few hundred µs (721 µs at
  most in a 10-minute run) came about every 110 s and decayed within 10–30 s, with no lost cycle or drive fault; the
  cause is not known ([`docs/hardware-evidence.md`](hardware-evidence.md)). PLEM has no gate that waits for settling (motion started inside the settling window ran without
  an alarm there; that does not show settling never matters).
- Reference clock = the servo slave with the **lowest ring position** (`bus.cpp`): the first DC slave the
  frame meets on the bus. IgH compensates propagation delay only downstream of the reference clock, and
  slaves ahead of it would be written before the reference time is read. File order does not matter. There is no separate key.
- Pacing and period: `ceil(interval_ms · 10⁶ / cycle_time_ns) <= UINT32_MAX` must hold (the same rule as the
  setter's division predicate: `interval_ms = UINT32_MAX` with `cycle_time_ns = 200000` is expressible on
  each side but overflows the cycle count). Judged after bus initialize. The µs→ns conversion is done in a
  wide enough integer.

## 6. Approval record — in the file, bound by a semantic hash

`approval` is optional. A file without it (or with an explicit `status: unapproved` / `record: null`) is
unapproved, the normal state before approval: assembly and startup are allowed, `enable()` is refused.
`approved` holds only when the loader **judges** the record the bench wrote into the file itself (§6.1;
the bench's recording step is the `approve` verb of `drive_model_tool`). The record carries the whole
bench evidence, so the model file is the only file to keep. The words `unverified` and `verified` are refused as
unknown values (no alias).

```yaml
# unapproved: no approval block, or
#   approval:
#     status: unapproved
#     record: null
# approved — written by the bench:
approval:
  status: approved
  record:
    config_digest: "sha256:…"          # SHA-256 of the §7 normalization (approval, expected_identity and stored_sdo excluded)
    profile_digest: "sha256:…"         # digest of the resolved cia402_profile values (all axes)
    implementation_revision: "…"       # the platform version the bench ran on (copied from the bench claims) — provenance, not judged
    slaves:                            # one identity per declared slave, by slaves[] index
      - { slave: 0, serial_number: 0, revision_number: 0x00000002, hardware_version: "1.3", software_version: "2.07", pdo_mapping_digest: null }
      #   a `pdo_mapping: scanned` slave carries "sha256:…" (the bus's ~/info at the bench start — the same value as the bus INFO line); a declared slave carries null
    bench:                             # the bench evidence approve read — the §6.1 shape without its schema line
      claims: { config_digest: "sha256:…", profile_digest: "sha256:…", implementation_revision: "…", slaves: [ … ] }
      conditions: { host: "…", master: "…", date: "YYYY-MM-DD", measured_by: "…", notes: "" }
      items:
        - { id: "assembly-match", result: PASS, evidence: "…" }
```

- **Identity fields.**
  - `serial_number` (the SII serial number): an integer (decimal or `0x`) or `null`.
  - `revision_number` (the SII revision number, the CoE 0x1018:03 value): an integer or `null`.
  - `hardware_version` (CoE 0x1009) and `software_version` (0x100A): a string of up to 31 bytes or `null`.
    A drive that returns a longer string can only be recorded with `null` there.
  - `pdo_mapping_digest`: exactly `sha256:` + 64 lower-case hex digits, or `null`. It is the SHA-256 of a
    scanned slave's mapping text (the §4.1 form, `format_pdo_mapping`), **copied** from the bus's
    `~/info` (`plem_drive_msgs/DriveInfo`, `pdo_mapping_digest`) on the bench start, the same value as the bus
    INFO line `using the scanned PDO mapping — … sha256:…` (the runner and the tool do not recompute it).
  - `null` = "this drive does not give that value; do not compare". Values are compared **as declared**, and
    0 is a value.
  - **No observed value at all is refused**: `serial_number` 0 or `null` with `revision_number`, both versions and
    the digest all `null`. Serial 0 counts as empty (the SII convention for "not recorded"); a revision
    number of 0 counts as read. Write at least one observable value.
  - **A digest on a declared slave is refused**: a slave whose `pdo_mapping` is a declared mapping has no
    scan to compare with (`config_digest` binds a declared mapping).
- **No downgrade — loading for execution fails closed.** If a file claims `approved` and any judgment below
  does not hold, `load_drive_model_file` fails with the first reason (plus the count of the rest). The
  read-only `plem drive check` reports whether the file is approved and every reason (`approval_claimed`, `approval_effective` and `reasons` with `--json`). The
  only tool that writes the file is the bench's recording step, and it replaces the `approval` block only (below);
  it never edits a declaration.
- **Judgment order** (all reasons are collected):
  1. The record's shape (three fields + `slaves[]` + `bench` with its claims, conditions and items; unknown keys
     refused): file layer. A malformed embedded bench (a duplicate item id, a result other than PASS, FAIL or
     NA) is a file error at its key path (`approval.record.bench.items[7].id …`), not a reason.
  2. `config_digest` == this declaration's computed value (§7).
  3. `profile_digest` == the resolved profile's computed value.
  4. (No comparison with the current release: `implementation_revision` records the release on which the bench
     ran. An approval survives releases.)
  5. `record.slaves` has exactly one entry per declared slave (missing, duplicate or out-of-range entries
     refused) + no-observed-value refusal + declared-slave digest refusal.
  6. (No separate bench-file hash: the evidence is embedded in the record.)
  7. **Back reference**: the embedded `bench.claims` equal the record's `config_digest`, `profile_digest`,
     `implementation_revision` and set of slave identities. It catches a **one-sided** edit: editing a limit
     and then pasting the new digest that `check` printed into the record (2 and 3 would pass), or attaching
     a bench of configuration A to configuration B. It does not prove that a bench ran, and an edit that
     changes both copies, or the evidence's conditions and items, is not detected.
  8. Every bench item required for this file's axes is `PASS` (§6.1); the loader and `approve` use the same
     function. At runtime,
     step 8 re-judges **the stored bench evidence against today's list of required ids**. An old bench lacks
     any id added or renamed later, so tightening the criteria is the only way a release
     voids an approval whose resolved declaration (including the built-in table it uses), identities,
     evidence and serialization are unchanged (§6.1 rename
     rule; the other ways are under "Lifetime" below).
- **Scope — `identity_scope`** (printed by `approve` and `check --json`; the `check` summary says `approved for
  this unit` or `approved for the model`).
  - If any `serial_number` in the record is 0 or `null`, the scope is `model`: the approval binds **a model with the
    same VID/PID and hardware and software versions**, not a unit. Motion also opens on another unit with the
    same firmware.
  - A `pdo_mapping: scanned` slave also needs the same scanned-mapping digest, so a fleet with revisions that
    map differently needs one approval per revision. (On some drives the SII and 0x1018:04 serials are both 0.)
  - If every slave has a non-zero serial number, the scope is `unit`. Version 1 has no alternative unit evidence
    (vendor objects).
- **Online check at startup.** The record's `slaves` go to `DriveModelSpec::expected_identity` →
  `DriveHost::create` → `EtherCATBus::set_expected_slave_identity` → a separate step of `initialize()` (after
  the master is acquired, before activation, independent of DC).
  - It reads the SII serial, SII revision number, 0x1009 and 0x100A and compares **only the declared values**
    exactly.
  - `pdo_mapping_digest` is not read; it is compared with the digest of the mapping that Step 3 of the same
    startup scanned. On a mismatch the message shows both digests and does not narrow the cause (drive or
    firmware replaced, a unit with a different device profile or variant, a wrong record, an assignment written by another tool, a
    CoE read failure that left SII values, a rescan needed). What to do: check the nameplate, then bench and
    approve again.
  - Any difference, or a declared value that cannot be read (for the digest: no scan), fails initialize →
    create is refused → `plem drive run`'s hardware interface fails to initialize and nothing starts. Nothing is read when
    nothing is declared (an unapproved file).
- **`stored_sdo` is not approved.** It is outside the config digest (§4.9, §7): anyone can edit a
  `stored_sdo` list in an approved file and the approval holds. The approval does not vouch for the list, and
  does not show that the drive held those values during the bench.
- **Lifetime — an approval survives releases.** `implementation_revision` is the **provenance** of the release
  the bench ran on (the value `plem drive check --json` printed). It is not judged. A development tree and a release
  package of the same version print the same string, so it does not identify a binary either. An empty value
  is a file error (the runner copies it from `check`); any other value is provenance nobody checks. An approval
  is voided only by:
  - a change of the declaration's **meaning** → judgment 2 (comments, indentation and key order are outside
    the digest, §7). A change of the core's built-in table → judgments 2 and 3 for files that use it (the
    resolved profile is also part of the config serialization);
  - a recorded identity value or scanned mapping that differs, or a declared value that cannot be read →
    the online check. Only **recorded fields** are compared, so a replacement with the same observed values
    passes (`identity_scope: model`);
  - a defect in the embedded bench evidence (judgments 7 and 8): restoring the record reopens it; no re-bench
    needed;
  - adding or renaming a required item → judgment 8 (§6.1 rename rule);
  - a change of the normalization or of the mapping text serialization → the digest (the internals §7 compatibility
    invariant: a risk to prevent, not an ordinary re-bench reason).

  Bench the file again; the bench replaces its record in place.
- **Bench bootstrap.** The only way to move an unapproved model is a bench run: `plem drive run <file>
  --unapproved` (the hardware interface's hardware parameter `unapproved`, default false; the loader call is
  `load_drive_model_file(path, ApprovalRecord::IGNORE)`; the bench starts the bus the same way). It opens only the approval gate of `decide_enable`; everything else (protective stop, command
  freshness, brake state) stays. A WARN "BENCH MODE" is logged at startup and at every actual release. The evidence
  of that run is bench evidence, not approved motion. In a bench run every file loads as unapproved and its approval
  record is never used, so there is no identity to compare (a file edited after its bench, or an approved file on
  other drives, can be benched again). There is no fake
  `approved` and no implicit bypass.
- The hashes **detect change** of the declaration. They are not proof that measurements are true, and not
  author authentication. The embedded evidence is not hashed; judgment 7 (claims vs the record's head), judgment 8
  (required items PASS) and the file layer's shape check are its only checks. Approval records require neither
  signatures nor PKI. Normal execution needs at least:
  valid structure ∧ supported declared policies ∧ a valid approval binding ∧ matching unit/execution
  conditions ∧ the readiness gates. An unsupported declaration is refused whether or not it is approved.

### 6.1 Bench file (`cia402_bench/v1`) and the scope of version 1 approval

The bench writes its evidence to this file (in a private temporary directory, removed afterwards) and hands it to
the `approve` verb of the drive model tool (`lib/plem_drive/drive_model_tool approve <model.yaml> <bench.yaml>`;
not a user command). It uses the same YAML subset as §2 (the output of `yaml.safe_dump` reads as is). `approve`
copies everything after the `schema` line (claims, every condition, every item) into `approval.record.bench`
(§6) and writes that record into the model file:

- the file's `approval` block is replaced (an approved one too, in a re-bench), or appended at the end when the file
  has none; every other byte is kept;
- the write is atomic: a temporary file in the same directory, then a rename, keeping the file's permissions (a
  symbolic link is written through to its target);
- it refuses when the bench's claimed digests are not this software's digests of the file as it is now (the file
  was edited during the bench), when a required item is missing or not `PASS`, when the result would exceed the
  loader's file size cap (1 MiB), and when the `approval` block cannot be found in place (a quoted key, for example;
  `check` prints that reason as `approve_blocker`). It judges its output text with the loader before writing and
  writes nothing unless the result is approved.

```yaml
schema: cia402_bench/v1
claims:                                # what the runner measured — the record's head is copied from it (§6 judgment 7)
  config_digest: "sha256:…"            # from `plem drive check --json` output
  profile_digest: "sha256:…"
  implementation_revision: "…"
  slaves: [{ slave: 0, serial_number: 0, revision_number: 0x00000002, hardware_version: "1.3", software_version: "2.07", pdo_mapping_digest: null }]
  #   values read over SII/SDO; serial_number and revision_number are integers (the runner copies Revision number
  #   from `ethercat slaves -v`). pdo_mapping_digest: for a scanned slave, the "sha256:…" from the bus's ~/info at the bench start
conditions: { host: "…", master: "…", date: "YYYY-MM-DD", measured_by: "…", notes: "" }
items:
  - { id: assembly-match, result: PASS, evidence: "…" }   # result: PASS | FAIL | NA; a duplicate id is refused
```

Required items in version 1 (`kRequiredBenchItems`), **by the file's axis modes** (`required_bench_items`):

| id | Required on | What it checks |
|---|---|---|
| `assembly-match` | every bus | the bus assembled the file's axes (`joint_states` names) and mappings (`~/info` digests) |
| `idle-non-oe` | every bus | before startup, every axis is not Operation enabled (idle SDO read of 0x6041) |
| `start-rtso` | every bus | after create and start, every axis is inside its position limits and rests brake-closed (Switch on disabled or Ready to switch on) with brakes engaged, no pending operations and no protective stop. The bus commands no transition before the first release (it writes controlword 0x0000); the state is the drive's own rest under that word (Switch on disabled for a standard drive, Ready to switch on on some families). One exception: an axis the idle read saw in Fault (a fault already latched before the bench started) gets one `reset_error` from the bench after startup, before this item is judged, and the evidence says so. It does not show that the drive accepts Shutdown in general: for a drive that starts in Switch on disabled, the first release (`release-oe`) also passes through Shutdown, but the bench does not record that 0x0006 was sent. The id keeps its v1 name: the criterion was loosened, not tightened (rename rule below) |
| `release-oe` | every bus | enable (with a stream at rest on a bus with a `csv` or `cst` axis) → every axis Operation enabled; calling again does nothing; then a hold at rest, where a released axis that moves more than 0.005 rad aborts the bench and engages |
| `stale-engage` | a bus with a `csv` or `cst` axis | stopping the stream → stale → external protective stop → forced engage |
| `stop-decel` | a bus with a `csp` or `csv` axis and no `cst` axis (a `cst` axis makes the stop an immediate disable, so there is no ramp to measure; a mixed bus's stop in motion has no bench evidence) | each `csp` and `csv` axis moves toward the side with more room: at most 0.5 rad (and nine tenths of the room, at least 0.05 rad) at a cruise of min(0.3 × `velocity_cap_rad_s`, the speed that leaves half the distance at cruise, the speed held at least 0.1 s; `joint_states` trails the command by the drive's response); a `csp` axis by `plem.drive`'s trapezoid, a `csv` axis by a stream ramped at its acceleration, the cruises starting together; `protective_stop` is called in the middle of the shortest one, once every moving axis is seen at cruise (three quarters into it at the latest, else the item fails). PASS: no `RAMP_STOP_TIMEOUT` (0x4F), every axis at rest with its brake engaged, and each axis's average deceleration (its speed at the request ÷ the time to rest, from every cycle on `rt_raw`; a cycle lost while it records fails the item) at most 1.1 × `limits.acceleration_rad_s2` (the requested stop of §4.10, from cruise) |
| `disable-engage` | every bus | `protective_stop` at rest → disabled, every brake engaged, nothing pending, no protective stop (on a bus of `cst` axes only, also 0 torque; on a bus without a torque axis, the requested-stop path at rest) |
| `stop-quiesce` | every bus | SIGINT to the controller manager while released → exit code 0, the hardware interface logged `brakes confirmed engaged on stop` and not `brakes were NOT confirmed engaged on stop` (the exit code is 0 either way, so these lines are the only word on the brakes; a run whose output carries neither (the hardware interface was not stopped, or its line was lost) fails, and so does a run that never released, whose hardware interface logs `brakes were never released this run` instead), no process of the run left, the master back to Idle, every axis non-enabled on an idle SDO read |

- An item not required for the file's axes is optional, like any other id (`fault-reset`, for example); the
  bench writes it as `NA` with `evidence`; leaving it out is also accepted. On a bus of `csp` axes only,
  `stale-engage` is `NA`: the trajectory controller writes every cycle, and a stalled controller manager's
  stale stop is not measured by the bench.
- If a required item is missing or not `PASS`, the file is not approved. **`FAIL` is refused for any id** (a
  measured failure is not optional; to opt out of an item the drive does not offer, write `NA`).
- `plem drive check --json` prints today's required list for that file as `required_bench_items`.
- An approval record without `stop-decel` still holds on a bus with a `cst` axis (the item is not required
  there; a record that carries it from an earlier bench holds too); on a bus with a `csp` or `csv` axis and no
  `cst` axis judgment 8 refuses it: bench again to add the item.

**Rename rule** (because approvals survive releases). A change that makes an item's **pass criteria
stricter** (a runner threshold, a C++ budget such as the command-freshness budget in `drive_host.hpp` or the
requested stop's timeouts in `drive_safety.hpp`, or this
specification, wherever it comes from) renames that id in `kRequiredBenchItems` (`stale-engage` →
`stale-engage-2`) and puts "drive re-bench required" in the changelog. An old bench does not have the new id,
so judgment 8 refuses it. Adding a required item works the same way. A change that keeps the contract, or that
loosens a criterion, is not renamed. Tightening an optional item follows this rule when the item becomes
required. The runner copies `claims.implementation_revision` from `check` output before the bench start.

**Runner.** The bench (the rclpy runner in `plem_drive`) is offered by `plem drive run <yaml>` on a terminal
for a file not approved for motion (the getting-started page, section 8). It starts the bus the way
`plem drive run --unapproved` does (the controller manager as its child), measures the items, writes the bench file
into a private temporary directory (`measured_by: "plem drive run (bench)"`), runs `approve` on it and removes it: a run
that ends approved leaves the approval in the model file only (`approval recorded in <file>`); a refusal or a declined confirmation is one line and writes nothing; any other
run leaves `<stem>.bench.log` and the model file as it was (bench again).
A file whose approval claim no longer holds, and an approved file (`plem drive run` offers the bench when the drives on
the bus differ from its record by an identity value only), is benched and its record replaced; the confirmation says
so. Before the bench start it shows the `warnings` of `check` as `WARN` lines (they do not stop
it). Before the bus starts (master Idle) it reads each axis's CiA 402 objects over SDO: if 0x6502 supported drive
modes lacks the bit of the file's mode (csp 7, csv 8, cst 9) it refuses **before any brake is released** (an absent
object or a failed read is not compared). A non-zero 0x607E polarity with `flip: true` (inverted twice) and counts
per motor turn in 0x6064 (0x6092 feed ÷ 0x6091 gear, else 0x608F encoder increments under a 1:1 gear) that are an
integer above 1 and differ from `encoder_resolution` are warned and kept in the bench evidence's `conditions.notes`
(1/1 defaults and non-integer results are not compared; drives use these objects loosely). An object the file's
`startup_sdo` writes is judged by that value, not by what the drive holds before the bus starts (its RAM may keep
the last run's writes). Any read failure, 0x6502's included, is "not compared" and does not stop the bench. The
stop objects (0x605A, 0x605C, 0x6084, 0x6085) of a `csp`/`csv` axis that the file does not declare are logged with
the value the drive holds now, as lines to paste into `startup_sdo`. After the bus stops it reads 0x6064 again and
warns if it differs from the read before the start by a motor turn or more (a held axis stays far below that; not
judged on an axis `stop-decel` moved: through a gearhead its motion can be many motor turns, so on a bus with a `csp` or `csv`
axis and no `cst` axis its `csp` and `csv` axes are not compared; an axis whose `startup_sdo` writes a scaling object is not compared either); statusword bit 11 (internal limit active) on any axis seen during the bench
is warned too (not bit 13, following error: PLEM does not use it, and a drive may set it in normal motion). A faulted axis's 0x603F is
printed with its CiA 301/402 class (current, voltage, temperature, monitoring, device specific …; 0x8611 following
error). Warnings do not block the approval.

- Its only inputs are the JSON output of `plem drive check --json` (`"format": "plem-drive-check/1"`; another format is
  refused as a release mismatch; `required_bench_items`, `approve_blocker`, `cycle_time_ns`,
  `command_stale_timeout_ns`, `fault_reset_policy`, `axes`, `slaves`) and the bus's topics (`<ns>/joint_states`
  from the joint state broadcaster, matched to the file's axes by name, `<ns>/plem_drive/status`, `rt_events` and `info`). It reads neither the YAML
  nor the bus's log. It does not judge (`approve` decides).
- The 0x6061 mode display is not a separate item: the release admission (`release_admissible`) refuses a
  display that differs from the declaration, so `release-oe` shows the mismatch as a FAIL.
- Releases hold each axis where it rests: the release hold of §4.2b holds a `csp` axis at its position and a `csv`
  axis at 0 rad/s without a stream; on a bus with a `csv` or `cst` axis the runner streams 0 (0 rad/s, 0 Nm) to the
  stream controller (`<ns>/stream_controller/commands`). Where the bus has a `csp` axis, every release goes through `plem.drive`'s `Bus.release`,
  which waits for the trajectory controller to be active and then restarts it, so it holds where the axes are, not where
  a stop left its command. It
  releases three times, or four on a bus with a `csv` axis and no `cst` axis.
- The confirmation names the release count and, on a bus with a `csp` or `csv` axis and no `cst` axis, that each such axis moves once
  up to 0.5 rad either way (`stop-decel`, the bench's only motion), so the workspace must be clear on both sides.
  It is the bench's only question; no, end of input or Ctrl-C touches nothing.
- **Scope of the claim.** This runner, on the `plem drive run` chain with `stop-decel`, has approved files on hardware
  on two A6B drives on JetPack 7.2.1: `csp` axes and a `csv` copy
  ([`docs/hardware-evidence.md`](hardware-evidence.md)). A bus with a `cst` axis has not.

**Procedure.** `plem drive check <yaml>` (optional: shows the approval state and warnings, the digests with `--json`; an
approval claim that does not hold exits 1 with its reasons and counts as unapproved) → `plem drive run <yaml>` on a terminal → answer the bench offer → the approval is recorded in the file, and `run` checks it again and starts the
bus.

**Excluded.** Drives with shared brakes are not in scope of the version 1 approval procedure: `approve`
cannot check physical sharing (§4.4 premises).

## 7. Normalization (the input of config_digest)

What is normalized is **the validated typed meaning**, not the YAML text and not C++ struct memory.

- Mapping key order is not in the struct and not in the digest (lines are written in a fixed order). The
  **sequence order** of `slaves`, `axes`, `sync_managers`, `pdos`, `entries` and `startup_sdo` is kept (axis
  order, gap positions and repeated writes are execution meaning).
- Integers by value (`0x0600` and `1536` digest the same). Reals as the shortest round-trip form of the
  parsed binary64 value (`std::to_chars`), with **`-0.0` written as `0.0`** (the value gate requires torque
  min < 0, so the real boundary is only offsets/positions, and they are physically equal). NaN is refused
  earlier. The only infinities are the (`-inf`, `inf`) pair of a `null` travel, written by `std::to_chars` as
  `-inf`/`inf` (different from every finite value). Strings as strings (`"1"` ≠ `1`). Enums as integers.
- Comments, indentation and mapping key order are not in the digest: **a comment change does not void an
  approval**. Names (`model.name`, `axes[].name`) are data and do void it; `bus.name` is not (§4.5) and does not.
- How the validated value is serialized, and why an unchanged file keeps its digests across releases:
  [`docs/internals/drive-model-contracts.md`](internals/drive-model-contracts.md), §7.

## 8. Where each rule is checked

Which function or step checks each rule (value gate, bus Step 0, file layer): [`docs/internals/drive-model-contracts.md`](internals/drive-model-contracts.md), §8.

## 9. Not provided

- No `auto_fault_reset: bool` → `fault_reset.policy` + explicit pacing. The request, the pulse sent and the
  drive's actual reset are different things.
- No arbitrary `factor / offset / mask` PDO conversions → the formulas and parameters of the supported units,
  one zero (`zero_offset_count`) and `flip`. `counts_per_amp` is a physical counts-per-ampere parameter, not
  an affine factor. The statusword mask belongs to the core; it is not a user feature.
- No user PDO `default`, no arbitrary command/state interface wiring, no general IO plugins, no transition
  tables, branches or retries outside the verified walk (an inline table gets the §4.8 checks as is). The
  core's no-command and stop behaviours are execution contracts, not defaults synthesized for missing data.
- IO slaves: version 1 declares **drive slaves only** (no IO onboarding). The bus configures only the
  declared drive slaves and leaves other slaves on the bus at the master default (PREOP). Holes in
  non-contiguous drive positions are not compressed, and IO positions are not guessed.
- `unit: raw` is reserved: the loader refuses it as an unknown value.
- Only three modes: `cst`, `csp`, `csv` (§4.2b). No runtime mode switch, no interpolation or trajectory
  generation in the bus loop (the client plans motion: `plem drive run`'s trajectory controller, `plem.drive`), no feed-forward (0x60B1, 0x60B2), no
  Quick stop (beyond the one the core commands when a requested stop is not confirmed in time, §4.10), no homing. `hold`, `comm_loss` and `brake` (premises, §4.4) are unknown keys in version 1.
- Keys refused as unknown keys, whatever their value:
  - `axes[].brake.coupling` — power coupling is the release-step contract of `brake_logic.hpp`, not a
    declaration.
  - `axes[].extra_sdo` — runtime endpoint registration; the core has no delivery path or checks for it.
- `stored_sdo` (§4.9) never writes, saves or corrects a drive value, never refuses to start, never retries, has
  no cancellation or bound on its wait, and does not decide which side is right. It does not compare drives
  with each other; a per-axis difference shows when two slaves list the same expected value.

## 10. Version and examples

- `schema: cia402_drive/v1` — the kind and version on one line. The loader accepts only this string (anything
  else is refused with "this loader reads cia402_drive/v1"). A change that every existing version 1 file
  still loads under, to the same declaration and digests, stays in version 1: a shorter spelling such as a
  `ring_position` list or a left-out unit (§1). Any other change
  of key structure is version 2. What blocks motion is the approval (§6), not the version mark.
- Example: [`plem_drive/test/data/drive_synthetic.yaml`](../plem_drive/test/data/drive_synthetic.yaml) — three slaves `{5, 0, 2}`; slave 5's two axes are in
  reverse channel order (the first axis in the document = channel 2); the two-channel slaves use a complete
  `cst` layout (with a gap and Error code). The loader golden test (`test_drive_model_file.cpp`) loads it and
  compares the whole expected spec, so it cannot rot apart from the schema; it is not installed. The C++ fixture's interleaved joint order and its IO
  devices cannot be written in a file (§3), so the example leaves them out. A synthetic
  example is not evidence of physical approval.
- Installed example: [`plem_drive/examples/example_drive.yaml`](../plem_drive/examples/example_drive.yaml) → `share/plem_drive/examples/` —
  one position (`csp`) axis of a CiA 402 servo drive with **synthetic values** (not measured or taken from any
  drive manual), `pdo_mapping: scanned`, the stop objects (0x605A, 0x605C, 0x6084, 0x6085) in `startup_sdo`,
  `limits.acceleration_rad_s2`, no `cia402_profile` (standard table), unapproved. It is the file you copy to start. The file as shipped was not run on hardware; a `scanned` mapping was,
  on the A6B pair after a power cycle ([`docs/hardware-evidence.md`](hardware-evidence.md)), and starting from a file was
  (the A6B example below).
- Installed example: [`plem_drive/examples/a6b_pair/drive.yaml`](../plem_drive/examples/a6b_pair/drive.yaml) — two `csp` axes of a Panasonic A6B with
  real values, the two drives in one entry (a `ring_position` list, §4.1), each behind a 21:1 gearhead (`gear_ratio: 21.0`: axis values at the gearhead output, drive objects in
  drive units at the motor), run, benched and approved on two drives ([`docs/hardware-evidence.md`](hardware-evidence.md)). The
  file as shipped, with `limits.acceleration_rad_s2: 95.0` on both axes and the startup_sdo 0x605A (Quick
  stop option code) = 2 on both slaves, is not approved: bench it before motion. The deceleration ran ([`docs/hardware-evidence.md`](hardware-evidence.md),
  a copy with one position limit changed); 0x605A = 2 has not been observed: no stop there
  reached Quick stop, and the A6B's 0x605A value table is not in the manuals held. It is moved with
  `plem drive run` and `plem.drive`.
