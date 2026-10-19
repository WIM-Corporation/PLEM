# Drive model file — implementation contracts

For contributors changing `plem_drive`: how the loader projects a drive model file onto its value spec, what a
loaded result guarantees, how the approval digests are computed, and which function checks each rule. The
section numbers are those of the schema, [`docs/cia402-drive-model.md`](../cia402-drive-model.md), whose sections
point here. A user of the file needs only the schema.

## 3.1 Key → value spec

The loader projects the file onto one flat value spec (`DriveModelSpec<N>`):

| File key | Value spec field |
|---|---|
| `model.name` | `model_name` |
| `bus.name` | not in the spec — `FileProjection::bus_name` beside it (§4.5), so not in the digest |
| `bus.cycle_time_ns` · `bus.sync_mode` | `cycle_time_us` (= ns ÷ 1000 — a value that is not a multiple of 1000 is refused by the file layer) · `dc_enable` (`drive_default` = false, `dc_sync0` = true) |
| `fault_reset.policy` · `fault_reset.pacing` | `fault_reset` · `fault_reset_pacing` (`pacing` is required under `bounded_auto_reset` and refused under `none`, by the file layer) |
| `fault_reset.pulse_ms` | `fault_reset_pulse_ms` → every axis's `ServoInfo::fault_reset_pulse_ms` (the bus converts it to frames) |
| `slaves[s].ring_position` | `slaves[s].ring_position` |
| `slaves[s].{vendor_id, product_code, pdo_mapping}` | `slaves[s].{vendor_id, product_code, pdo_mapping}` (`sync_managers[].sm`, `direction`, `pdos[].entries[].bits` ↔ `syncs[].sm_index`, `output`, `entries[].bit_length`) |
| `slaves[s].dc.{assign_activate, sync0_cycle_ns, sync0_shift_ns, write_cycle_time}` | `slaves[s].dc_*` |
| `slaves[s].startup_sdo` | `startup_sdo[s]` |
| `slaves[s].stored_sdo` (optional) | `stored_sdo[s]` (count 0 when the key is absent — nothing is read; not in the config digest, §7) |
| `slaves[s].axes[k]` | joint i (counted in document order) — `axes[i].slave_index = s` |
| `…axes[k].name` | `axis_names[i]` |
| `…axes[k].channel` | `axes[i].od_offset = (channel − 1) × 0x800` |
| `…axes[k].{mode, mode_display_timing, factors, flip, error_code_pdo}` | `axes[i].{modes_of_operation, mode_display_timing, factors, flip, error_code_pdo}` (`bits` ↔ `bit_length`; `factors.gear_ratio` goes to the torque, position and velocity conversions and `encoder_resolution` to the position and velocity conversions, each as the same value; `counts_per_amp` ↔ `counts_per_unit`; for `rated_per_mille`, `motor_rated_torque_nm` ↔ `torque_constant` with `counts_per_unit` = 1000) |
| `…axes[k].{zero_offset_count, limits}` | `safety[i]` |
| `…axes[k].ramp` | (none: refused, §4.2b) |
| `cia402_profile[r]` (top level) | row r of every `axes[i].cia402_profile` (`kStandardCia402Profile` when the key is absent) |
| `approval` (optional) | `approval` (the §6 result — absent or `status: unapproved` = `Approval::UNAPPROVED`; `approved` becomes `APPROVED` only when every judgment on the record holds, otherwise loading fails) · `expected_identity[s]` (from `record.slaves` when approved) |

## 3.2 Loader lifetime contract

- **Single source.** No environment variable or host parameter overrides a value from the file.
- **When things are judged.** Loader success = file layer + `validate_drive_model_spec` (the value gate).
  Everything else is judged **where a hand-assembled spec is judged too**:
  - bus Step 0 (per-slave `validate_pdo_mapping`, duplicate (ring position, `od_offset`), OD overflow,
    Error code width) before `DriveHost::create` calls the master;
  - the fault-reset pacing × period predicate after bus initialize;
  - the DC sync0 rules after the master is acquired and before activation.

  These checks are not copied into the loader. Bus Step 0 is one pure function, `validate_bus_configuration`
  (fieldbus `configuration.hpp`): `DriveHost::create` calls it before the master, and `plem drive check` calls it
  on the same injection (`drive_bus_injection`) with no drive — only a scanned slave's mapping waits for startup.
- **Atomic result.** A successful result is one value that passed the checks above. A failed load leaves
  nothing behind in the executor.
- **No reload while active.** The file is not re-read while the host is active. Replacing it means stop and
  restart; the new host re-checks everything.
- **PDO graph lifetime.** The executor consumes a pointer graph: one graph per slave, shared by that slave's
  axes, living at least as long as the bus (`DriveHost::create` rebuilds it from the value). The loader
  never returns addresses of parser nodes or local graphs.
- **Error context.** A file-layer refusal names the file path and position — line for syntax (byte offset
  for encoding), key path for schema. A value-gate refusal names the same file path
  (`slaves[s].axes[k].limits.position`, `slaves[s].axes[k].factors.gear_ratio`, `slaves[s].startup_sdo[w]`,
  `bus.cycle_time_ns`); the loader maps the flat index back (schema §3.1).
- **Paths.** A relative path is relative to the process working directory, and `~` is not expanded. Use an
  absolute path.

## 7. Normalization — implementation and compatibility

- **Implementation** ([`plem_drive/src/loader/digest.cpp`](../../plem_drive/src/loader/digest.cpp), `norm/1`). The YAML is not
  re-serialized. The **validated `DriveModelSpec` value** is serialized as fixed-order `key=value` lines:
  - the first line is `schema=cia402_drive/v1 norm/1`; sequences are written up to their count, in order;
    `approval`, `expected_identity` and `stored_sdo` are excluded (`stored_sdo` is a warn-only expectation
    nothing executes — editing it keeps an approval, and the approval does not vouch for it, §4.9);
  - most line keys use the file key vocabulary (`bus.cycle_time_ns`, `bus.sync_mode`,
    `slaves[s].ring_position`, `axes[i].factors.gear_ratio`, `axes[i].limits.*` …). Two lines
    differ: `bus.sync_mode` writes `drive_default` as `sm`, and an axis's `mode_display_timing` is the line
    `axes[i].mode_display`. Lines with no matching
    file key, or a different shape, use spec field names (the complete list is the `put` lines of
    `digest.cpp`) — for example `model_name` (file `model.name`), `fault_reset`,
    `fault_reset_pacing`, `fault_reset_pulse_ms` (file `fault_reset.{policy,pacing,pulse_ms}`) and
    `slave_count`; axes use the flat `axes[i]` instead of the file's nested `slaves[].axes[]`, with the lines
    `slave_index`, `slave_position_derived` (a retired field, kept and always written `false` so the digest does not change),
    `od_offset` and `mode_of_operation` (file `mode`, §4.2b);
  - **a normalized key, once chosen, is never renamed** — even a rename to match the file vocabulary changes
    the digest and voids every approval (compatibility invariant below);
  - gear ratio and encoder resolution are one line per axis; `limits.torque_nm` is two lines on a `cst` axis only
    (none on a `csp` or `csv` axis); `limits.acceleration_rad_s2` is one line on a `csp` or `csv` axis only (on
    a `cst` axis it is omitted, and a `csp` or `csv` axis must carry the key); no ramp line is serialized;
  - one line `slaves[s].dc_write_cycle_time=true` for a slave with `dc.write_cycle_time: true` only (the line is
    omitted at its default `false`);
  - one line `cia402_profile.source=built_in|inline` (an inline copy of the built-in table is a different
    declaration and so a different approval — a change of the core table voids only files that use the
    built-in table).

  `profile_digest` covers only each axis's `axes[i].cia402_profile` values, by the same rules (`profile/1`).
  The loader's `inspect` and `drive_model_tool` (`check`, `approve`) call the same function. A `sizeof(DriveModelSpec<1>)` pin, checked on LP64
  builds only, catches a new field that changes the struct size at compile time. It cannot catch a small field that fits in padding
  (bool, uint8, uint16), so a review of a spec header change looks at `normalize()` too. The golden values are
  the two hex digests of `drive_synthetic.yaml` in `test_drive_model_file.cpp`.
- **Compatibility invariant** (approvals survive releases — §6 "Lifetime"): **an unchanged declaration and an
  unchanged mapping produce the same digest bytes across releases** — the config normalization (`norm/1`),
  the profile normalization (`profile/1`) and the scanned PDO mapping text (`format_pdo_mapping` →
  `pdo_mapping_digest`, which the bus compares with the record).
  - A change to any of these serializations voids every approval that holds such a digest. It is a
    **compatibility risk**: review must justify it, and if it lands, the changelog says "drive re-bench
    required".
  - A new field may omit its line at its default only when that fixed default means what the old behaviour
    meant; any non-default value must be serialized.
  - An omission is never applied retroactively to a field that is already serialized (`mode_of_operation` is
    always serialized — a new mode only widens the accepted values).
  - There is no compatibility layer that accepts several normalizations at once.

## 8. Where each rule is checked

"Value gate" is `validate_drive_model_spec`, called by the loader. The other checks run where a
hand-assembled spec is judged too (Step 0, setters, `configure_DC` — §3.2).

| Rule | Checked by | File layer |
|---|---|---|
| Control period | value gate (`kMinCycleTimeUs`..`kMaxCycleTimeUs`, §5) | multiple of 1000 ns |
| Output SM watchdog | value gate (a declared mapping needs `watchdog: true` on an output SM; a scanned slave is not judged, §4.1) | — |
| Axis and slave count | value gate (slave_count 1..N) | N = Σ len(`axes`) after expansion ∈ 1..8; empty `slaves` or `axes` refused before counting |
| Ring position and identity | `check_slaves` (position ≥ 0, VID/PID ≠ 0) + value gate (`ring_position` ≤ 65535) | reads 0..2³¹−1; VID/PID 0 refused first |
| Duplicate ring position, orphan slave | `check_slaves` | duplicates at the value gate; orphans by the empty-`axes` rule first. Gaps and any order allowed |
| Mapping policy and storage | `check_slaves` (`kMax*` capacity; sync_count 0 = scanned passes); the SM number range (0..15) at Step 0 | empty `sync_managers`, over capacity, other values; `generated` on a slave with more than four axes |
| Mapping structure, per-mode roles, direction, width, address overlap | `validate_pdo_mapping` (declared: bus Step 0; scanned: bus Step 3; per slave, for all its axes). The required-role list is that validator's — PLEM's minimum, not a universal one | — |
| Channel grid, mode, `mode_display_timing`, profile | `check_axes` | `channel` 1..8 before multiplying |
| Duplicate (ring position, `od_offset`), standard OD overflow | `validate_bus_configuration` (bus initialize Step 0) | — |
| Error code | `check_axes` (width 0 refused) + Step 0 (16-bit width, address overflow, mapped) | `index: 0` refused |
| Factor parameters | value gate (positive, file-key wording) + `validate_axis_factors` | — |
| Torque limit | value gate (`cst`: finite, min < 0 < max; otherwise 0/0 — the loader refuses the key off `cst`) | — |
| Velocity limit | value gate (finite and positive; `csp`: `velocity_cap_rad_s` > 0, `check_csp_step`) → `set_drive_limits` (finite, > 0) | missing or not a number |
| Stop deceleration | value gate (`csp`, `csv`: finite and positive; `cst`: 0 — file-key wording) → `set_drive_stop` | missing, 0 or less on a `csp` or `csv` axis; the key on a `cst` axis |
| `ramp` | file layer: refused, with the message to delete it (§4.2b) | — |
| Reset policy × pacing, period | `validate_fault_reset` (value gate) + setter predicate (after bus initialize) | `pacing` missing under `bounded_auto_reset` or written under `none` |
| DC values and divisibility | value gate (`dc_sync0` → every `assign_activate ≠ 0`; `write_cycle_time` refused under `drive_default`) + `configure_DC()` (including a refused reservation of the 0x1C32:02 write) | `dc` under `drive_default`, a missing `dc` under `dc_sync0` and `write_cycle_time: false` refused |
| Names | `check_model_name` + value gate (axis names, §4.5) | over 31 bytes, control characters, whitespace-only model name |
| Profile | `validate_cia402_profile` (value gate) | unknown state, over 16 rows, empty, `null` |
| `schema` | — | exactly `cia402_drive/v1` |
| `bus.name` | `plem drive check` (required there; the loader takes a file without it) | a ROS name token, at most 64 bytes, when present |
| Startup SDO width, value, conflicts | value gate (size 1/2/4, value range, 16 per slave, core-owned objects) + Step 0 | range-checked before narrowing |
| Stored SDO width, value, duplicates | value gate (size 1/2/4, value range, 16 per slave, duplicate address, address in the slave's `startup_sdo`, slave past the count) + Step 0; output-PDO and mapping objects are skipped with a WARN at Step 3.6 (§4.9) | `[]` refused; range-checked before narrowing |
| §4.4 execution | value gate + `DriveHost` (carry, enable gate) + `run_drive_safety_cycle` | — |
| Approval binding, digests, identity | `inspect_drive_model` (§6 judgments 1–8) + `EtherCATBus::verify_slave_identity` (initialize Step 3.5) | record shape (judgment 1); approved fails closed |
