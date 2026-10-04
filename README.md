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
