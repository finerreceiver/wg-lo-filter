"""FINER LO filter CLI. Run: uv run lo-filter.py <command> [options]."""
from typing import Annotated
import logging

import typer

app = typer.Typer(no_args_is_help=True, add_completion=False)
IFNAME = "eth0"  # Established EtherCAT interface; USB-LAN is for SSH, not control.
LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")


def _log_level(value: str) -> str:
    """Normalize and validate the CLI logging threshold.

    Args:
        value: Case-insensitive log-level string supplied by Typer.

    Returns:
        The uppercase level name.

    Raises:
        typer.BadParameter: The name is not in LOG_LEVELS.
    """
    level = value.upper()
    if level not in LOG_LEVELS:
        raise typer.BadParameter("Choose: " + ", ".join(LOG_LEVELS))
    return level


Rx = Annotated[str | None, typer.Option("--rx", help="Receiver: 4+5 or 6+7; exclusive with --hpf-id.")]
Hpf = Annotated[int | None, typer.Option("--hpf-id", help="HPF ID from config.toml; exclusive with --rx.")]
LogLevel = Annotated[str, typer.Option("--log-level", callback=_log_level, help="Minimum log level.")]


def _execute(operation, log_level: str) -> None:
    """Load utilities, configure logging, and run a CLI operation.

    Args:
        operation: Callable receiving the imported utils module.
        log_level: Validated uppercase logging threshold.

    Returns:
        None; control outcomes are communicated through human-readable logs.

    Notes:
        Imports utils lazily so help does not require controller dependencies
        or configuration. Import failures are logged with a traceback.
        Operation Exceptions are logged at ERROR (tracebacks at DEBUG), and
        KeyboardInterrupt at WARNING, then suppressed at this CLI boundary.
        Does not define application-specific exit codes or emit JSON.
        HALT and master cleanup are the operation's responsibility, not this
        wrapper's. configure_logging errors occur before the operation handler.
    """
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
    """Build a deferred operation with one owned EtherCAT master.

    Args:
        rx: Receiver CLI selector, or None when hpf_id is supplied.
        hpf_id: One-based HPF ID, or None when rx is supplied.
        action: Callable taking (utils, master, bpf_id, slave_ids).

    Returns:
        A callable taking the utils module; selection and connection are
        deferred until that callable runs.

    Notes:
        Resolves the target before opening IFNAME. The action runs inside
        ethercat_master(), which owns cleanup. Does not open a connection when
        constructing the callback or add a motion-error guard itself.
    """
    def operate(u):
        """Resolve the captured selector and run the action within one master.

        Args:
            u: Imported utilities module providing target and lifecycle helpers.

        Returns:
            None.

        Notes:
            Selection, connection, and action errors propagate to _execute().
            The context manager attempts cleanup if the action raises.
        """
        bpf_id, slave_ids = u.resolve_target(rx, hpf_id)
        with u.ethercat_master(IFNAME) as master:
            action(u, master, bpf_id, slave_ids)
    return operate


@app.command()
def prepare(rx: Rx = None, hpf_id: Hpf = None, log_level: LogLevel = "INFO") -> None:
    """Reset, configure, enable, and reference the selected receiver or HPF.

    Args:
        rx: Receiver CLI selector ('4+5' or '6+7'), mutually exclusive with hpf_id.
        hpf_id: One-based HPF ID, mutually exclusive with rx.
        log_level: Case-insensitive threshold; defaults to INFO.

    Returns:
        None; status and failures are reported through logs.

    Notes:
        Exactly one target selector is required. Receiver selection prepares
        all configured BPF members; HPF selection prepares only that actuator.
        Driving errors or Ctrl+C attempt HALT of the entire containing BPF,
        including for single-HPF preparation, before master cleanup.
        This is an explicit operation; set() never calls it automatically.
    """
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
    """Set a prepared receiver's filter from RF LO and filter bandwidth.

    Args:
        rx: Receiver CLI selector ('4+5' or '6+7'); required.
        lo: Positive finite RF first-LO frequency in GHz; required.
        bw: Positive full filter-side bandwidth in GHz; required.
        log_level: Case-insensitive threshold; defaults to INFO.

    Returns:
        None; progress, cancellation, and failures are reported through logs.

    Notes:
        Divides lo by the configured RF multiplier, but does not scale bw.
        Frequency calculations precede connection. Requires the current
        enabled/encoder_valid/has_error readiness checks; never auto-prepares
        or reads back parameters. Software-limit cancellation does not move.
        Driving errors or Ctrl+C attempt BPF-wide HALT on the same master.
        HPF-level selection is not supported for this command.
    """
    def operate(u):
        """Calculate the captured LO request and move its BPF with one master.

        Args:
            u: Imported utilities module providing calculation and control helpers.

        Returns:
            None.

        Notes:
            Logs completion only when move_bpf() returns a non-None result.
            Errors propagate to _execute(); the master context attempts cleanup.
        """
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
    """Read and log PDO status without sending enable/reset/prepare commands.

    Args:
        rx: Receiver CLI selector ('4+5' or '6+7'), mutually exclusive with hpf_id.
        hpf_id: One-based HPF ID, mutually exclusive with rx.
        log_level: Case-insensitive threshold; defaults to INFO.

    Returns:
        None; status and failures are reported through logs.

    Notes:
        Exposed as read-status on the CLI. Position in millimetres and full
        decoded status are logged at INFO for each selected slave.
        The master is still initialized and closed, so this is not a guarantee
        that EtherCAT bus state is unchanged. Does not validate readiness.
    """
    _execute(_target_operation(
        rx, hpf_id, lambda u, master, bpf_id, ids: u.log_selected_status(master, ids)
    ), log_level)


@app.command()
def halt(rx: Rx = None, hpf_id: Hpf = None, log_level: LogLevel = "INFO") -> None:
    """Send a normal stop to the selected receiver or individual HPF.

    Args:
        rx: Receiver CLI selector ('4+5' or '6+7'), mutually exclusive with hpf_id.
        hpf_id: One-based HPF ID, mutually exclusive with rx.
        log_level: Case-insensitive threshold; defaults to INFO.

    Returns:
        None; status and failures are reported through logs.

    Notes:
        Uses user-requested halt_selected(), not the automatic motion-error
        guard. Attempts all selected HALTs, waits 0.5 seconds, and logs status.
        Individual command/read failures are logged. Does not verify physical
        stopping or provide emergency preemption of another control process.
    """
    _execute(_target_operation(
        rx, hpf_id, lambda u, master, bpf_id, ids: u.halt_selected(master, ids)
    ), log_level)


@app.command()
def reset(rx: Rx = None, hpf_id: Hpf = None, log_level: LogLevel = "INFO") -> None:
    """Reset the selected receiver or HPF without re-preparing it.

    Args:
        rx: Receiver CLI selector ('4+5' or '6+7'), mutually exclusive with hpf_id.
        hpf_id: One-based HPF ID, mutually exclusive with rx.
        log_level: Case-insensitive threshold; defaults to INFO.

    Returns:
        None; status and failures are reported through logs.

    Notes:
        Resets slaves sequentially, waits 0.5 seconds after each, and logs
        status. Does not enable or find index; run prepare before moving.
        A command/status failure stops further resets and is logged by the
        CLI boundary. No automatic HALT guard is added to reset.
    """
    _execute(_target_operation(
        rx, hpf_id, lambda u, master, bpf_id, ids: u.reset_selected(master, ids)
    ), log_level)


if __name__ == "__main__":
    app()
