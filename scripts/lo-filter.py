"""FINER LO filter CLI. Run: uv run lo-filter.py <command> [options]."""
from typing import Annotated
import logging

import typer

app = typer.Typer(no_args_is_help=True, add_completion=False)
IFNAME = "eth0"  # Established EtherCAT interface; USB-LAN is for SSH, not control.
LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")


def _log_level(value: str) -> str:
    level = value.upper()
    if level not in LOG_LEVELS:
        raise typer.BadParameter("Choose: " + ", ".join(LOG_LEVELS))
    return level


Rx = Annotated[str | None, typer.Option("--rx", help="Receiver: 4+5 or 6+7; exclusive with --hpf-id.")]
Hpf = Annotated[int | None, typer.Option("--hpf-id", help="HPF ID from config.toml; exclusive with --rx.")]
LogLevel = Annotated[str, typer.Option("--log-level", callback=_log_level, help="Minimum log level.")]


def _execute(operation, log_level: str) -> None:
    """CLI boundary: human logs only; no application-specific exit-code scheme."""
    # Lazy import lets --help work without config/PySOEM and keeps imports inert.
    try:
        import utils as u
    except Exception:
        logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")
        logging.getLogger("wg_lo_filter.cli").exception("Failed to load control configuration/dependencies")
        return
    u.configure_logging(log_level)
    logger = u.get_logger("cli")
    try:
        operation(u)
    except KeyboardInterrupt:
        logger.warning("Operation interrupted by Ctrl+C")
    except Exception as error:
        logger.error("%s: %s", type(error).__name__, error)
        logger.debug("Exception details", exc_info=True)


def _target_operation(rx, hpf_id, action):
    def operate(u):
        bpf_id, slave_ids = u.resolve_target(rx, hpf_id)
        with u.ethercat_master(IFNAME) as master:
            action(u, master, bpf_id, slave_ids)
    return operate


@app.command()
def prepare(rx: Rx = None, hpf_id: Hpf = None, log_level: LogLevel = "INFO") -> None:
    """Reset, configure, enable and find index for the selected receiver/HPF."""
    _execute(_target_operation(
        rx, hpf_id, lambda u, master, bpf_id, ids: u.prepare_selected(master, bpf_id, ids)
    ), log_level)


@app.command()
def set(
    rx: Annotated[str, typer.Option("--rx", help="Receiver: 4+5 or 6+7.")],
    lo: Annotated[float, typer.Option("--lo", help="RF first LO frequency [GHz].")],
    bw: Annotated[float, typer.Option("--bw", help="Full filter bandwidth [GHz].")],
    log_level: LogLevel = "INFO",
) -> None:
    """Move a prepared BPF; never automatically prepare it."""
    def operate(u):
        request = u.build_lo_request(rx, lo, bw)
        logger = u.get_logger("cli")
        logger.info("Receiver %s / RF LO %.5f GHz / multiplier %s / filter center %.5f GHz",
                    rx, lo, request["multiplier"], request["central_frequency_GHz"])
        with u.ethercat_master(IFNAME) as master:
            result = u.move_bpf(master, request["bpf_id"], request["central_frequency_GHz"], bw)
            if result is not None:
                logger.info("BPF #%s setting completed", request["bpf_id"])
    _execute(operate, log_level)


@app.command("read-status")
def read_status(rx: Rx = None, hpf_id: Hpf = None, log_level: LogLevel = "INFO") -> None:
    """Read PDO status without sending enable/reset/prepare commands."""
    _execute(_target_operation(
        rx, hpf_id, lambda u, master, bpf_id, ids: u.log_selected_status(master, ids)
    ), log_level)


@app.command()
def halt(rx: Rx = None, hpf_id: Hpf = None, log_level: LogLevel = "INFO") -> None:
    """HALT the selected receiver/HPF; separate from automatic error handling."""
    _execute(_target_operation(
        rx, hpf_id, lambda u, master, bpf_id, ids: u.halt_selected(master, ids)
    ), log_level)


@app.command()
def reset(rx: Rx = None, hpf_id: Hpf = None, log_level: LogLevel = "INFO") -> None:
    """Reset the selected receiver/HPF; does not prepare or find index."""
    _execute(_target_operation(
        rx, hpf_id, lambda u, master, bpf_id, ids: u.reset_selected(master, ids)
    ), log_level)


if __name__ == "__main__":
    app()
