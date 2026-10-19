# Drive examples

Two examples come with `plem drive`. Copy one somewhere you can write: the bench records its approval in the file.

| Example | Use it to |
| --- | --- |
| `example_drive.yaml` | Start the file for your own drive. One position (`csp`) axis on one CiA 402 drive. Every value is synthetic, so it runs on no real drive until you replace them all. |
| `a6b_pair/` | See a finished setup. Two Panasonic MINAS A6B drives in position (`csp`) on a rig whose outputs are coupled, moved in sync, with a demo script and a PlotJuggler layout. Its own `README.md` walks through it. |

The installed copies are in `$(ros2 pkg prefix --share plem_drive)/examples/`.

## Modes

Each axis picks one mode in the file, and one bus may mix them:

- `csp`: the drive follows a target position. `plem.drive` plans the moves (`Bus.move_to`).
- `csv`: the drive follows a target velocity that your program streams (`Bus.stream`).
- `cst`: the drive follows a target torque, also streamed (`Bus.stream`). It does not hold position.

All three load and run. On hardware, `csp` has run the most, on the A6B pair; `csv` has been benched and had short
runs, and `cst` short, low-speed runs only. [`docs/hardware-evidence.md`](../../docs/hardware-evidence.md) lists what has run and what has not.

## Your own drive

Run the first axis with no load on it, with the emergency stop within reach: the bench that `plem drive run`
offers moves the axes.

```bash
mkdir -p "$HOME/drives"
cp "$(ros2 pkg prefix --share plem_drive)/examples/example_drive.yaml" "$HOME/drives/my_drive.yaml"
cd "$HOME/drives"
plem drive check my_drive.yaml   # reads the file; a missing or invalid key refuses it, saying which
plem drive run my_drive.yaml     # offers the bench, then runs the bus once the file is approved
```

For a velocity (`csv`) or torque (`cst`) axis, change `mode` — a `cst` axis takes `limits.torque_nm` instead of
`limits.acceleration_rad_s2`; section 2 of [`docs/getting-started.md`](../../docs/getting-started.md) says which keys go with each mode.

A coding agent can write the file with you: the agent skill installed with this package,
`$(ros2 pkg prefix plem_drive)/share/skills/add-drive`, reads your drives without writing to them, cites a source
for every value and asks you for the decisions ([`docs/getting-started.md`](../../docs/getting-started.md) section 1).

## Where to read next

These are in this repository:

- [`docs/getting-started.md`](../../docs/getting-started.md): the steps in order, from filling in the values (section 2) to watching the bus
  (section 9).
- [`docs/cia402-drive-model.md`](../../docs/cia402-drive-model.md): every key of the file.
