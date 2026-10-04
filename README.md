# wg-lo-filter
Control software for FINER/Tunable waveguide LO filter

## Configuration

Python 3.11 or later is required (`tomllib`). Keep `config.toml` in the
project root alongside `scripts/`, including when deploying to Raspberry Pi.
The configuration path is resolved relative to `scripts/utils.py`, not the
current working directory.

`config.toml` defines:

- `receivers.<name>.bpf_id`: receiver-to-BPF mapping.
- `bpfs.<id>.hpf_ids`: HPFs to prepare and halt, in the listed order.
- `bpfs.<id>.move_order`: HPF motion order; each member must appear once.
- `hpfs.<id>.slave_id`: zero-based EtherCAT slave ID.
- `hpfs.<id>.cutoff_side`: `hpf` for the lower cutoff, `lpf` for the upper cutoff.
- `hpfs.<id>.A`, `B`, `X0`: coefficients for
  `position_mm = X0 - A / (cutoff_GHz - B)`.
- `hpfs.<id>.FREQ`, `FRQ2`: actuator excitation frequencies in Hz.
- `motion`: default/index velocity, acceleration/deceleration (existing controller
  command units), and encoder resolution in micrometres per encoder unit.
- `timing`: PDO exchange pauses, polling intervals, and reset/settling waits in seconds.
- `timeouts`: Python wait limits in seconds; `ethercat_state_check_us` is in microseconds.
- `pdo_settle`: exchange counts and pause interval used while awaiting state updates.
- `controller_defaults`: shared controller settings, including `LLIM`/`HLIM` in
  encoder units. Parameter transmission order is explicitly retained in the code.

The values were transferred from the existing scripts on 2026-10-04 without
retuning. When tuning a value, add the actual measurement date, conditions,
and reason beside that entry. Commit the comment and value together.
CLI display/post-action pauses use named constants at each script's top.
These are distinct from the configurable controller/PDO waits.

The BPF cancellation boundary retains the existing expression
`calculated_position >= MAX_DESIRED_POSITION_MM - RESOLUTION_MM`, with
`MAX_DESIRED_POSITION_MM = 2.5` in `utils.py`. With the current resolution,
positions at or above 2.49875 mm are canceled; this is separate from `HLIM`.
`move_abs_plot()` internals are unchanged. The legacy `command(delay=...)`
argument remains unused and does not introduce a new pause.

Configuration is loaded and validated when `scripts.utils` is imported.
Restart the script or interactive Python session after editing it.
Each HPF must belong to exactly one BPF, and slave IDs must be unique.
The existing HPF #1–#6 and B45/B67 names remain available as compatibility
constants; use the lookup functions for control logic.

CLI `--bpf_ID` choices are derived from the configured BPF IDs. For example,
from the project root:

```sh
uv run python -m scripts.prepare_bpf --bpf_ID 2
uv run python -m scripts.move_bpf --bpf_ID 2 --central_freq 100 --band_width 10
uv run python -m scripts.halt_bpf --bpf_ID 2
```

Python callers can resolve a receiver or actuator using
`get_bpf_id_for_receiver("B67")`, `get_bpf_slave_ids(bpf_id)`, and
`get_bpf_id_for_slave(slave_id)`. The CLI continues to accept BPF IDs directly.

## Logging

Every CLI accepts `--log-level DEBUG|INFO|WARNING|ERROR|CRITICAL`
(case-insensitive, default `INFO`). Standard Python logging emits timestamped
messages to stderr; importing `scripts.utils` does not configure console output.

```sh
# Normal progress and movement results
uv run python -m scripts.move_bpf --bpf_ID 2 --central_freq 100 --band_width 10
# Include full controller statuses
uv run python -m scripts.prepare_bpf --bpf_ID 2 --log-level DEBUG
# Only warnings and errors
uv run python -m scripts.all_hpf --slaveID 3 --dpos_mm 1.0 --log-level WARNING
```

`INFO` includes progress and position results; `DEBUG` includes full statuses
and acknowledgement diagnostics. `WARNING` includes canceled motion and
missing diagnostic data; `ERROR` includes movement/HALT failures and timeouts.
Because status inspection is its primary purpose, `read_status_hpf` reports
its status at `INFO`. Changing the log threshold does not skip status reads,
motion commands, or HALT, and does not suppress Python exceptions/tracebacks.

For IPython/notebook use, configure logging explicitly:

```python
from scripts.logging_utils import configure_logging
configure_logging("DEBUG")
import scripts.utils as u
```

Repeated configuration replaces the application's console handler; it does
not change the root logger or unrelated libraries' logging configuration.

## EtherCAT master lifetime

Hardware functions now take a PySOEM master as their first argument.
The module no longer stores a global master, and the old `u.init()` API has
been replaced by a context manager:

```python
import scripts.utils as u

with u.ethercat_master("eth0") as master:
    u.prepare_actuator(master, 3, bpf_id=2)
    u.find_index(master, 3, direction=0)
    u.dpos(master, 3, target_pos_mm=1.0)
    status = u.read_status(master, 3)
# The master has been closed here; use a new with block for later commands.
```

The established initialization sequence and PDO timings are unchanged.
On normal exit, initialization failure, an exception, or Ctrl+C, the context
manager attempts INIT and then closes the adapter. If opening failed, it
skips the INIT request but still attempts close. Close is attempted even if
the INIT request fails. Cleanup failures propagate on normal exit; if
another exception is already active, cleanup failures are logged and attached
as notes without replacing that original exception.

Do not call `u.close(master)` manually inside the with block.
Closing the master is not a HALT command or a guaranteed emergency stop.
Existing timeout/HALT handling runs inside the block before cleanup.
Forced termination (SIGKILL), power loss, or a failure of the close operation
cannot be protected against by a Python context manager.

Pure configuration, calculation, conversion, and formatting functions
(for example `get_bpf_slave_ids()`, `calc_bpf_positions()`, and
`encoder_to_mm()`) do not require a master. CLI arguments are unchanged;
the command-line examples above still apply.

Dependency-free lifecycle/caller tests (no hardware communication):

```sh
python -m unittest discover -s tests -v
```
