from __future__ import annotations

from contextlib import contextmanager
from typing import Any, Callable, Iterator
import pysoem
import time
import struct
import math
from pathlib import Path
import tomllib
import logging

# 2026-10-04: CLI共通ロギングを統合。importだけでコンソールを設定しない。
LOGGER_NAMESPACE = "wg_lo_filter"
LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")
CONSOLE_HANDLER_NAME = "wg-lo-filter-console"
LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
LOG_DATE_FORMAT = "%H:%M:%S"
logging.getLogger(LOGGER_NAMESPACE).addHandler(logging.NullHandler())


def get_logger(name):
    """Return a named logger in the application's logging namespace.

    Args:
        name: Logger suffix, such as 'cli' or 'utils'.

    Returns:
        A logging.Logger; no console handler is configured here.
    """
    return logging.getLogger(f"{LOGGER_NAMESPACE}.{name}")


def configure_logging(level="INFO"):
    """Configure timestamped application logs on standard error.

    Args:
        level: Case-insensitive log threshold; defaults to INFO.

    Returns:
        None.

    Raises:
        ValueError: The level is not one of LOG_LEVELS.

    Notes:
        Replaces only the application's named console handler. Does not change
        root-logger configuration; repeated calls do not duplicate that handler.
    """
    if not isinstance(level, str) or level.upper() not in LOG_LEVELS:
        raise ValueError(f"Invalid log level: {level}")
    application_logger = logging.getLogger(LOGGER_NAMESPACE)
    application_logger.setLevel(level.upper())
    application_logger.propagate = False
    for handler in list(application_logger.handlers):
        if handler.get_name() == CONSOLE_HANDLER_NAME:
            application_logger.removeHandler(handler)
            handler.close()
    console = logging.StreamHandler()
    console.set_name(CONSOLE_HANDLER_NAME)
    console.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=LOG_DATE_FORMAT))
    application_logger.addHandler(console)

logger = get_logger("utils")

# 固定の通信仕様。運用設定と区別し、TOMLで変更しない。
PDO_COMMAND_FORMAT = "<4siiHHB"
PDO_POSITION_FORMAT = "<i"
PDO_POSITION_BYTES = struct.calcsize(PDO_POSITION_FORMAT)
PDO_STATUS_BYTES = 3
COMMAND_NAME_BYTES = 4
EXECUTE_PREPARE = 0
EXECUTE_RUN = 1
MM_PER_UM = 1e-3
SIGNED_INT32_MIN = -(2 ** 31)
SIGNED_INT32_MAX = 2 ** 31 - 1
UNSIGNED_INT16_MAX = 2 ** 16 - 1

# Python側BPF判定は既存の >= 2.5 - RESOLUTION_MM を維持する。
# これはコントローラーのHLIMとは別の運用制限。
MAX_DESIRED_POSITION_MM = 2.5
INDEX_RESULT_DISPLAY_PAUSE_S = 1.0 # 原点探索結果を読むための表示待ち
LEGACY_UNUSED_COMMAND_DELAY_S = 0.02 # 現在command()はこの引数を使用しない

# 送信順はプロトコル手順としてコードに保持。値はTOMLから取得する。
CONTROLLER_PARAMETER_ORDER = (
    "ELIM", "TOU2", "TOU3", "ZON1", "ZON2", "PTOL", "PTO2", "ILIM",
    "ACTD", "ENCD", "ENCO", "LLIM", "HLIM", "PRO2", "PROP", "INTF", "INDA",
)
STATUS_BITS = {
    "enabled": 0, "end_stop": 1, "thermal_protection1": 2,
    "thermal_protection2": 3, "force_zero": 4, "motor_on": 5,
    "closed_loop": 6, "encoder_index": 7, "encoder_valid": 8,
    "searching_index": 9, "position_reached": 10, "error_compensation": 11,
    "encoder_error": 12, "scanning": 13, "left_end_stop": 14,
    "right_end_stop": 15, "error_limit": 16, "searching_optimal_freq": 17,
    "safety_timeout": 18, "ecat_ack": 19, "emergency_stop": 20,
    "position_fail": 21,
}

# =========================
# Actuator Parameter
# =========================

# BPF #1: HPF #1（下側）+ LPF #1 = HPF #2・#3（上側）
CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.toml"


def load_config(path=CONFIG_PATH):
    """Read and validate the TOML configuration without hardware access.

    Args:
        path: Path to the configuration file; defaults to CONFIG_PATH.

    Returns:
        The validated configuration dictionary.

    Raises:
        OSError: The configuration file cannot be read.
        tomllib.TOMLDecodeError: The TOML syntax is invalid.
        ValueError: Required entries, numeric ranges, or mappings are invalid.

    Notes:
        Checks receiver/BPF/HPF mappings, unique slave IDs, motion/timing values,
        PDO-compatible parameter ranges, and calibration coefficient finiteness.
        Does not validate measured calibration coverage or controller state.
    """
    with Path(path).open("rb") as file:
        config = tomllib.load(file)

    try:
        # 数値の型・有限性とPDOに収まる範囲を通信開始前に検査する。
        positive_fields = {
            "motion": ("default_velocity", "index_velocity", "acceleration",
                       "deceleration", "encoder_resolution_um"),
            "timing": ("status_poll_interval_s",),
            "timeouts": ("default_wait_s", "enable_s", "index_search_s",
                         "index_landing_s", "absolute_move_s", "ethercat_state_check_us"),
            "pdo_settle": ("default_cycles", "interval_s", "after_settings_cycles",
                           "after_enable_cycles", "after_index_cycles"),
        }
        nonnegative_timing = ("pdo_exchange_pause_s", "position_settle_s", "reset_wait_s",
                              "settings_apply_wait_s", "motion_status_wait_s")
        for section, keys in positive_fields.items():
            for key in keys:
                value = config[section][key]
                if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{section}.{key} must be a positive finite number")
        for key in nonnegative_timing:
            value = config["timing"][key]
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"timing.{key} must be a nonnegative finite number")
        for key, maximum in (("default_velocity", SIGNED_INT32_MAX),
                             ("index_velocity", SIGNED_INT32_MAX),
                             ("acceleration", UNSIGNED_INT16_MAX),
                             ("deceleration", UNSIGNED_INT16_MAX)):
            value = config["motion"][key]
            if type(value) is not int or value > maximum:
                raise ValueError(f"motion.{key} must be an integer <= {maximum}")
        for key in ("default_cycles", "after_settings_cycles", "after_enable_cycles", "after_index_cycles"):
            if type(config["pdo_settle"][key]) is not int:
                raise ValueError(f"pdo_settle.{key} must be an integer")
        if type(config["timeouts"]["ethercat_state_check_us"]) is not int:
            raise ValueError("timeouts.ethercat_state_check_us must be an integer")
        defaults = config["controller_defaults"]
        for key in CONTROLLER_PARAMETER_ORDER:
            value = defaults[key]
            if type(value) is not int or not SIGNED_INT32_MIN <= value <= SIGNED_INT32_MAX:
                raise ValueError(f"controller_defaults.{key} must be a signed 32-bit integer")
        if set(defaults) != set(CONTROLLER_PARAMETER_ORDER):
            raise ValueError("Unknown controller_defaults parameter")
        if defaults["LLIM"] >= defaults["HLIM"]:
            raise ValueError("LLIM must be smaller than HLIM")
        hpfs = config["hpfs"]
        bpfs = config["bpfs"]
        receivers = config["receivers"]
        if not hpfs or not bpfs or not receivers:
            raise ValueError("hpfs, bpfs and receivers must not be empty")
        slave_ids = []
        for hpf_id, hpf in hpfs.items():
            if not hpf_id.isdecimal() or str(int(hpf_id)) != hpf_id or int(hpf_id) < 1:
                raise ValueError(f"Invalid HPF ID: {hpf_id}")
            slave_id = hpf["slave_id"]
            if type(slave_id) is not int or slave_id < 0:
                raise ValueError(f"HPF #{hpf_id}: slave_id must be a nonnegative integer")
            slave_ids.append(slave_id)
            if hpf["cutoff_side"] not in ("hpf", "lpf"):
                raise ValueError(f"HPF #{hpf_id}: cutoff_side must be hpf or lpf")
            for key in ("FREQ", "FRQ2"):
                if type(hpf[key]) is not int or not 0 < hpf[key] <= SIGNED_INT32_MAX:
                    raise ValueError(f"HPF #{hpf_id}: {key} must be a positive signed 32-bit integer")
            for key in ("A", "B", "X0"):
                value = hpf[key]
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(f"HPF #{hpf_id}: {key} must be a finite number")
        if len(set(slave_ids)) != len(slave_ids):
            raise ValueError("HPF slave_id values must be unique")

        assigned_hpfs = []
        for bpf_id, bpf in bpfs.items():
            if not bpf_id.isdecimal() or str(int(bpf_id)) != bpf_id or int(bpf_id) < 1:
                raise ValueError(f"Invalid BPF ID: {bpf_id}")
            ids = bpf["hpf_ids"]
            order = bpf["move_order"]
            if (not isinstance(ids, list) or not ids
                    or any(type(i) is not int or str(i) not in hpfs for i in ids)
                    or len(set(ids)) != len(ids)):
                raise ValueError(f"BPF #{bpf_id}: invalid or duplicate hpf_ids")
            if (not isinstance(order, list)
                    or any(type(i) is not int for i in order)
                    or sorted(order) != sorted(ids)):
                raise ValueError(f"BPF #{bpf_id}: move_order must contain each member once")
            assigned_hpfs.extend(ids)
        if (len(set(assigned_hpfs)) != len(assigned_hpfs)
                or set(assigned_hpfs) != {int(i) for i in hpfs}):
            raise ValueError("Each HPF must belong to exactly one BPF")
        for name, receiver in receivers.items():
            bpf_id = receiver["bpf_id"]
            if type(bpf_id) is not int or str(bpf_id) not in bpfs:
                raise ValueError(f"Receiver {name}: unknown bpf_id {bpf_id}")
            if not isinstance(receiver["cli_name"], str) or not receiver["cli_name"]:
                raise ValueError(f"Receiver {name}: cli_name must be a nonempty string")
            multiplier = receiver["rf_to_filter_multiplier"]
            if type(multiplier) is not int or multiplier <= 0:
                raise ValueError(f"Receiver {name}: rf_to_filter_multiplier must be a positive integer")
        names = [receiver["cli_name"] for receiver in receivers.values()]
        if len(names) != len(set(names)):
            raise ValueError("Receiver cli_name values must be unique")
    except KeyError as exc:
        raise ValueError(f"Missing configuration entry in {path}: {exc}") from exc
    return config


CONFIG = load_config()
# 既存の公開定数名を維持。値はimport時に設定から一度読み込む。
DEFAULT_VEL = CONFIG["motion"]["default_velocity"]
INDEX_VEL = CONFIG["motion"]["index_velocity"]
ACCEL = CONFIG["motion"]["acceleration"]
DECEL = CONFIG["motion"]["deceleration"]
POLL_DT = CONFIG["timing"]["status_poll_interval_s"]
SETTLE_TIME = CONFIG["timing"]["position_settle_s"]
PDO_EXCHANGE_PAUSE_S = CONFIG["timing"]["pdo_exchange_pause_s"]
RESET_WAIT_S = CONFIG["timing"]["reset_wait_s"]
SETTINGS_APPLY_WAIT_S = CONFIG["timing"]["settings_apply_wait_s"]
MOTION_STATUS_WAIT_S = CONFIG["timing"]["motion_status_wait_s"]
DEFAULT_WAIT_TIMEOUT_S = CONFIG["timeouts"]["default_wait_s"]
ENABLE_TIMEOUT_S = CONFIG["timeouts"]["enable_s"]
INDEX_SEARCH_TIMEOUT_S = CONFIG["timeouts"]["index_search_s"]
INDEX_LANDING_TIMEOUT_S = CONFIG["timeouts"]["index_landing_s"]
ABSOLUTE_MOVE_TIMEOUT_S = CONFIG["timeouts"]["absolute_move_s"]
ETHERCAT_STATE_CHECK_TIMEOUT_US = CONFIG["timeouts"]["ethercat_state_check_us"]
PDO_SETTLE_CYCLES = CONFIG["pdo_settle"]["default_cycles"]
PDO_SETTLE_INTERVAL_S = CONFIG["pdo_settle"]["interval_s"]
AFTER_SETTINGS_PDO_CYCLES = CONFIG["pdo_settle"]["after_settings_cycles"]
AFTER_ENABLE_PDO_CYCLES = CONFIG["pdo_settle"]["after_enable_cycles"]
AFTER_INDEX_PDO_CYCLES = CONFIG["pdo_settle"]["after_index_cycles"]
BPF_IDS = tuple(sorted(int(i) for i in CONFIG["bpfs"]))
BPF_B45 = CONFIG["receivers"]["B45"]["bpf_id"]
BPF_B67 = CONFIG["receivers"]["B67"]["bpf_id"]


def get_bpf_hpf_ids(bpf_id, *, motion_order=False):
    """Return the configured HPF members of a BPF in the requested order.

    Args:
        bpf_id: BPF ID defined in config.toml.
        motion_order: Use move_order when True, otherwise hpf_ids.

    Returns:
        A new list of one-based HPF IDs.

    Raises:
        ValueError: bpf_id is not a configured integer BPF ID.
    """
    if type(bpf_id) is not int or bpf_id not in BPF_IDS:
        raise ValueError(f"Unknown BPF ID: {bpf_id}; configured IDs: {BPF_IDS}")
    key = "move_order" if motion_order else "hpf_ids"
    return list(CONFIG["bpfs"][str(bpf_id)][key])


def get_bpf_slave_ids(bpf_id):
    """Resolve BPF membership to EtherCAT slave indices.

    Args:
        bpf_id: BPF ID defined in config.toml.

    Returns:
        A new list of zero-based slave IDs in configured hpf_ids order.

    Raises:
        ValueError: bpf_id is not configured.
    """
    return [CONFIG["hpfs"][str(i)]["slave_id"] for i in get_bpf_hpf_ids(bpf_id)]


def get_hpf_id_for_slave(slave_id):
    """Find the HPF mapped to a zero-based EtherCAT slave index.

    Args:
        slave_id: Zero-based EtherCAT slave index.

    Returns:
        The one-based HPF ID.

    Raises:
        ValueError: No HPF maps to the supplied integer slave ID.
    """
    if type(slave_id) is int:
        for hpf_id, hpf in CONFIG["hpfs"].items():
            if hpf["slave_id"] == slave_id:
                return int(hpf_id)
    raise ValueError(f"No HPF configured for slave ID {slave_id}")


def get_bpf_id_for_slave(slave_id):
    """Find the BPF containing the HPF mapped to a slave.

    Args:
        slave_id: Zero-based EtherCAT slave index.

    Returns:
        The configured BPF ID.

    Raises:
        ValueError: The slave ID is not mapped to an HPF.
    """
    hpf_id = get_hpf_id_for_slave(slave_id)
    return next(bpf_id for bpf_id in BPF_IDS if hpf_id in get_bpf_hpf_ids(bpf_id))

# Keep the existing public names for callers of scripts.utils.
SLAVE_ID_HPF_1 = CONFIG["hpfs"]["1"]["slave_id"]
SLAVE_ID_HPF_2 = CONFIG["hpfs"]["2"]["slave_id"]
SLAVE_ID_HPF_3 = CONFIG["hpfs"]["3"]["slave_id"]

# BPF #2: LPF #2 = HPF #4・#6（上側）+ HPF #5（下側）
SLAVE_ID_HPF_4 = CONFIG["hpfs"]["4"]["slave_id"]
SLAVE_ID_HPF_5 = CONFIG["hpfs"]["5"]["slave_id"]
SLAVE_ID_HPF_6 = CONFIG["hpfs"]["6"]["slave_id"]

RESOLUTION_UM = CONFIG["motion"]["encoder_resolution_um"]
RESOLUTION_MM = RESOLUTION_UM * MM_PER_UM


def encoder_to_mm(position_encoder):
    """Convert an encoder position to millimetres.

    Args:
        position_encoder: Position in encoder units.

    Returns:
        position_encoder multiplied by the configured RESOLUTION_MM.
    """
    return position_encoder * RESOLUTION_MM

# LPF #1のActual positionと実測LPF cutoffの−3 dB fitting結果
# 2026年8月5日測定。70 GHzの測定点はfittingから除外。
A_LPF_HPF_2 = CONFIG["hpfs"]["2"]["A"]
B_LPF_HPF_2 = CONFIG["hpfs"]["2"]["B"]
X0_LPF_HPF_2 = CONFIG["hpfs"]["2"]["X0"]

A_LPF_HPF_3 = CONFIG["hpfs"]["3"]["A"]
B_LPF_HPF_3 = CONFIG["hpfs"]["3"]["B"]
X0_LPF_HPF_3 = CONFIG["hpfs"]["3"]["X0"]

# HPF #1単体の−3 dB fitting結果（2026年8月4日測定）
A_HPF_1 = CONFIG["hpfs"]["1"]["A"]
B_HPF_1 = CONFIG["hpfs"]["1"]["B"]
X0_HPF_1 = CONFIG["hpfs"]["1"]["X0"]

# LPF #2 / HPF #4のActual positionと実測LPF cutoffのfitting結果
A_LPF_HPF_4 = CONFIG["hpfs"]["4"]["A"]
B_LPF_HPF_4 = CONFIG["hpfs"]["4"]["B"]
X0_LPF_HPF_4 = CONFIG["hpfs"]["4"]["X0"]

# LPF #2 / HPF #6のActual positionと実測LPF cutoffのfitting結果
A_LPF_HPF_6 = CONFIG["hpfs"]["6"]["A"]
B_LPF_HPF_6 = CONFIG["hpfs"]["6"]["B"]
X0_LPF_HPF_6 = CONFIG["hpfs"]["6"]["X0"]

# HPF #5単体のfitting結果
A_HPF_5 = CONFIG["hpfs"]["5"]["A"]
B_HPF_5 = CONFIG["hpfs"]["5"]["B"]
X0_HPF_5 = CONFIG["hpfs"]["5"]["X0"]

# =========================
# EtherCAT basic functions
# =========================
@contextmanager
def ethercat_master(ifname: str) -> Iterator[pysoem.Master]:
    """Open, initialize, and own an EtherCAT master for one with block.

    Args:
        ifname: Network interface connected to the EtherCAT controller.

    Raises:
        RuntimeError: No slaves are found or SAFEOP/OP is not reached.
        Exception: PySOEM initialization or normal-exit cleanup fails.

    Notes:
        Maps PDOs, checks SAFEOP, then requests OP using the established timing.
        On exit, attempts INIT and close, including during initialization failure
        or an active exception. If open fails, skips INIT but still attempts close.
        Cleanup errors during an existing exception are logged and added as notes;
        otherwise they propagate. Do not manually close inside the with block.
        Does not send HALT, serialize processes, or protect against power loss/SIGKILL.

    Yields:
        The initialized pysoem.Master; usable only within the with block.
    """
    master = pysoem.Master()
    opened = False
    primary_error = None
    try:
        master.open(ifname)
        opened = True
        if master.config_init() < 1:
            raise RuntimeError(f"No EtherCAT slaves found on {ifname}")
        master.config_map()
        master.state_check(pysoem.SAFEOP_STATE, timeout=ETHERCAT_STATE_CHECK_TIMEOUT_US)
        if master.state != pysoem.SAFEOP_STATE:
            raise RuntimeError("Failed to reach EtherCAT SAFEOP_STATE")
        master.state = pysoem.OP_STATE
        master.write_state()
        master.state_check(pysoem.OP_STATE, timeout=ETHERCAT_STATE_CHECK_TIMEOUT_US)
        if master.state != pysoem.OP_STATE:
            raise RuntimeError("Failed to reach EtherCAT OP_STATE")
        logger.info("Master is in OP_STATE")
        yield master
    except BaseException as error:
        # Also preserve KeyboardInterrupt/SystemExit while closing the adapter.
        primary_error = error
        raise
    finally:
        try:
            close(master, request_init=opened)
        except BaseException as cleanup_error:
            if primary_error is None:
                raise
            logger.error("Master cleanup also failed: %s", cleanup_error)
            primary_error.add_note(f"EtherCAT master cleanup failed: {cleanup_error}")


def send_cmd(
    master: pysoem.Master,
    slave_id: int,
    cmd_bytes: bytes,
    execute: int,
    v1: int = 0,
    v2: int = 0,
    v3: int = 0,
    v4: int = 0,
) -> None:
    """Write an Xeryon command payload to a slave's PDO output buffer.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        cmd_bytes: Four-byte ASCII command identifier, for example b'DPOS'.
        execute: Execute byte; normally 0 for prepare or 1 for execute.
        v1: Main parameter, packed as signed 32-bit integer.
        v2: Command-specific second parameter, packed as signed 32-bit integer.
        v3: Command-specific third parameter, packed as unsigned 16-bit integer.
        v4: Command-specific fourth parameter, packed as unsigned 16-bit integer.

    Returns:
        None.

    Raises:
        IndexError: slave_id is outside master.slaves.
        struct.error: A value cannot be packed into PDO_COMMAND_FORMAT.

    Notes:
        Numeric values are converted with int(). Output is padded to the existing
        buffer length. This function does not exchange PDOs; call trig() to send.
        The command's interpretation and units depend on the controller command.
    """
    payload = struct.pack(
        PDO_COMMAND_FORMAT,
        cmd_bytes,
        int(v1),
        int(v2),
        int(v3),
        int(v4),
        int(execute),
    )

    slave = master.slaves[slave_id]
    slave.output = payload.ljust(len(slave.output), b"\x00")

def trig(master: pysoem.Master) -> None:
    """Exchange process data once, then apply the configured PDO pause.

    Args:
        master: Open, mapped PySOEM master owned by the caller.

    Returns:
        None.

    Notes:
        Calls send_processdata() and receive_processdata(), then sleeps for
        PDO_EXCHANGE_PAUSE_S seconds. Does not check the returned working counter.
        The effective cycle time also includes communication and processing time.
    """
    master.send_processdata()
    master.receive_processdata()
    time.sleep(PDO_EXCHANGE_PAUSE_S)
    


def command(
    master: pysoem.Master,
    slave_id: int,
    cmd_bytes: bytes,
    v1: int = 0,
    v2: int = 0,
    v3: int = 0,
    v4: int = 0,
    delay: float = LEGACY_UNUSED_COMMAND_DELAY_S,
) -> None:
    """Send a command using the prepare-then-execute PDO sequence.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        cmd_bytes: Four-byte ASCII controller command.
        v1: Command-specific main parameter in controller units.
        v2: Command-specific second parameter in controller units.
        v3: Command-specific third parameter in controller units.
        v4: Command-specific fourth parameter in controller units.
        delay: Legacy compatibility argument; currently unused.

    Returns:
        None.

    Notes:
        Writes execute=0 and exchanges PDOs, then writes execute=1 and exchanges
        PDOs again. int conversion and packing are delegated to send_cmd().
        Does not verify command acknowledgement; underlying errors propagate.
    """
    send_cmd(master, slave_id, cmd_bytes, EXECUTE_PREPARE, v1, v2, v3, v4)
    trig(master)

    send_cmd(master, slave_id, cmd_bytes, EXECUTE_RUN, v1, v2, v3, v4)
    trig(master)


# =========================
# Status handling
# =========================
def read_status(master: pysoem.Master, slave_id: int) -> dict[str, Any]:
    """Exchange PDOs and decode one slave's position and status bits.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.

    Returns:
        A dictionary with 'pos' in encoder units, boolean STATUS_BITS fields,
        and hexadecimal 'status_raw'.

    Raises:
        IndexError: The slave index is outside master.slaves.
        struct.error: The position bytes cannot be decoded.

    Notes:
        Does not send enable/reset/prepare commands. It does transmit the current
        PDO output buffers and does not validate the receive working counter.
    """
    trig(master)

    data = master.slaves[slave_id].input

    actual_position = struct.unpack(PDO_POSITION_FORMAT, data[:PDO_POSITION_BYTES])[0]
    status = int.from_bytes(data[PDO_POSITION_BYTES:PDO_POSITION_BYTES + PDO_STATUS_BYTES], "little")

    return {
        "pos": actual_position,

        **{name: bool((status >> bit) & 1) for name, bit in STATUS_BITS.items()},

        "status_raw": hex(status),
    }


def has_error(st):
    """Test the status fields currently classified as controller errors.

    Args:
        st: Status dictionary produced by read_status().

    Returns:
        True if encoder_error, error_limit, safety_timeout, emergency_stop,
        position_fail, end_stop, left_end_stop, or right_end_stop is set.

    Raises:
        KeyError: A checked status field is missing.

    Notes:
        Does not check thermal protection, busy-state flags, or parameter values.
    """
    return (
        st["encoder_error"]
        or st["error_limit"]
        or st["safety_timeout"]
        or st["emergency_stop"]
        or st["position_fail"]
        or st["end_stop"]
        or st["left_end_stop"]
        or st["right_end_stop"]
    )


def wait_until(
    master: pysoem.Master,
    slave_id: int,
    condition: Callable[[dict[str, Any]], bool],
    timeout: float = DEFAULT_WAIT_TIMEOUT_S,
    label: str = 'condition',
) -> dict[str, Any]:
    """Poll controller status until a predicate succeeds or the wait fails.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        condition: Callable receiving a status dictionary and returning a boolean.
        timeout: Polling deadline in seconds; uses the current wall-clock time.
        label: Description included in error messages.

    Returns:
        The first status satisfying condition.

    Raises:
        RuntimeError: has_error() is True, checked before the predicate.
        TimeoutError: The condition is not observed before the deadline.

    Notes:
        Polls read_status() and sleeps POLL_DT between unsuccessful checks.
        Requires a positive timeout to obtain a status. Timeout is not a hard
        real-time bound; communication and sleeps can extend elapsed time.
        Does not send HALT itself.
    """
    t0 = time.time()

    while time.time() - t0 < timeout:
        st = read_status(master, slave_id)

        if has_error(st):
            raise RuntimeError(f"Controller error while waiting for {label}: {st}")

        if condition(st):
            return st

        time.sleep(POLL_DT)

    raise TimeoutError(f"Timeout while waiting for {label}. Last status: {st}")


# =========================
# Motion commands
# =========================
def enable(master: pysoem.Master, slave_id: int) -> None:
    """Send ENBL=1 and wait for the enabled status bit.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.

    Returns:
        None.

    Raises:
        RuntimeError: A checked controller error is observed while waiting.
        TimeoutError: enabled is not observed within ENABLE_TIMEOUT_S.

    Notes:
        Exchanges one more PDO cycle after the wait. This changes controller
        state and must not be used as a prerequisite for passive status inspection.
    """
    command(master, slave_id, b"ENBL", v1=1)
    wait_until(master, slave_id, lambda st: st["enabled"], timeout=ENABLE_TIMEOUT_S, label="enabled")
    trig(master)


def halt(master: pysoem.Master, slave_id: int) -> None:
    """Send a normal HALT command to one actuator.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.

    Returns:
        None.

    Notes:
        Exchanges an additional PDO cycle after the command. Does not wait for
        motor_on=False or prove that physical motion stopped. Communication errors
        propagate to the caller.
    """
    command(master, slave_id, b"HALT")
    trig(master)


def reset(master: pysoem.Master, slave_id: int) -> None:
    """Send RSET to one controller and exchange an additional PDO cycle.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.

    Returns:
        None.

    Notes:
        Does not wait for reboot, reapply settings, enable, or find index.
        Treat the actuator as requiring preparation before the next absolute move.
    """
    command(master, slave_id, b"RSET")
    trig(master)


def find_index(
    master: pysoem.Master,
    slave_id: int,
    direction: int = 0,
    vel: int = INDEX_VEL,
    accel: int = ACCEL,
    decel: int = DECEL,
) -> dict[str, Any]:
    """Start INDX and wait for index validity and landing completion.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        direction: 0 for descending, 1 for ascending encoder direction.
        vel: Velocity passed in raw controller command units.
        accel: Acceleration passed in raw controller command units.
        decel: Deceleration passed in raw controller command units.

    Returns:
        The final status after the settling wait.

    Raises:
        ValueError: direction is not 0 or 1.
        RuntimeError: A checked error occurs or search stops before encoder_valid.
        TimeoutError: Index search or landing exceeds its configured timeout.

    Notes:
        First exchanges AFTER_INDEX_PDO_CYCLES status updates. Waits for
        encoder_valid, then position_reached, and applies settling/display waits.
        Uses INDEX_SEARCH_TIMEOUT_S and INDEX_LANDING_TIMEOUT_S seconds.
        Does not HALT by itself; prepare_selected() supplies the error guard.
    """
    if direction not in (0, 1):
        raise ValueError("INDX direction must be 0 or 1")

    command(master, slave_id, b"INDX", v1=direction, v2=vel, v3=accel, v4=decel)

    # INDX の直後は PDO の状態反映に数周期かかることがある。
    # 公式ライブラリと同様に数回更新を待ってから、探索が原点未検出の
    # まま終了していないか確認する。
    for _ in range(AFTER_INDEX_PDO_CYCLES):
        time.sleep(POLL_DT)
        st = read_status(master, slave_id)

    t0 = time.time()
    while time.time() - t0 < INDEX_SEARCH_TIMEOUT_S:
        st = read_status(master, slave_id)

        if has_error(st):
            raise RuntimeError(f"Controller error while waiting for index: {st}")

        if st["encoder_valid"]:
            break

        if not st["searching_index"]:
            raise RuntimeError(
                "Index search stopped without finding an index: "
                f"{st}"
            )

        time.sleep(POLL_DT)
    else:
        raise TimeoutError(
            "Timeout while waiting for encoder_valid / index found. "
            f"Last status: {st}"
        )

    wait_until(
        master,
        slave_id,
        lambda st: st["position_reached"],
        timeout=INDEX_LANDING_TIMEOUT_S,
        label="position_reached after index",
    )

    time.sleep(SETTLE_TIME)
    st = read_status(master, slave_id)
    logger.info("Index found for slave %s", slave_id)
    logger.debug("Index status: %s", st)
    time.sleep(INDEX_RESULT_DISPLAY_PAUSE_S)
    return st

def move_abs(
    master: pysoem.Master,
    slave_id: int,
    target_pos: int,
    vel: int = DEFAULT_VEL,
    accel: int = ACCEL,
    decel: int = DECEL,
) -> dict[str, Any]:
    """Move one prepared actuator to an absolute encoder position using DPOS.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        target_pos: Absolute target in encoder units.
        vel: Velocity passed in raw controller command units.
        accel: Acceleration passed in raw controller command units.
        decel: Deceleration passed in raw controller command units.

    Returns:
        The final status read after SETTLE_TIME.

    Raises:
        RuntimeError: The actuator is disabled, unreferenced, or reports a checked error.
        TimeoutError: position_reached with motor_on=False is not observed in time.

    Notes:
        Waits MOTION_STATUS_WAIT_S before polling, then uses ABSOLUTE_MOVE_TIMEOUT_S.
        Does not separately verify a motion-start edge or final numeric tolerance.
        The BPF software position limit and error-triggered HALT are handled by
        move_bpf(), not by this low-level function.
    """
    st = read_status(master, slave_id)

    if not st["enabled"]:
        raise RuntimeError("Controller is not enabled. Run enable() first.")

    if not st["encoder_valid"]:
        raise RuntimeError("Index is not found yet. Run find_index() first.")

    command(master, slave_id, b"DPOS", v1=target_pos, v2=vel, v3=accel, v4=decel)
    time.sleep(MOTION_STATUS_WAIT_S)
    # wait_until(
    #     master,
    #     slave_id,
    #     lambda s: (not s["position_reached"]) or s["motor_on"],
    #     timeout=1.0,
    #     label=f"motion started target={target_pos}",
    # )

    st = wait_until(
        master,
        slave_id,
        lambda s: s["position_reached"] and not s["motor_on"],
        timeout=ABSOLUTE_MOVE_TIMEOUT_S,
        label=f"position_reached target={target_pos}",
    )

    time.sleep(SETTLE_TIME)
    st = read_status(master, slave_id)

    logger.info(f"Target={target_pos}, Actual={st['pos']}, Error={st['pos'] - target_pos}")
    return st

def set_param(
    master: pysoem.Master,
    slave_id: int,
    param_name: str,
    value: int | float,
    delay: float = LEGACY_UNUSED_COMMAND_DELAY_S,
) -> None:
    """Send a four-character controller parameter name and its value.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        param_name: Four-character ASCII parameter identifier, such as 'FREQ'.
        value: Value in that parameter's controller units; converted with int().
        delay: Legacy argument forwarded to command(); currently unused.

    Returns:
        None.

    Raises:
        ValueError: param_name is not four characters long.
        UnicodeEncodeError: param_name is not ASCII.

    Notes:
        Uses the standard command sequence without reading back the parameter.
        PDO packing and communication errors propagate.
    """
    if len(param_name) != COMMAND_NAME_BYTES:
        raise ValueError("param_name must be 4 characters, e.g. 'PROP', 'FREQ'")

    cmd_bytes = param_name.encode("ascii")
    command(master, slave_id, cmd_bytes, v1=value, delay=delay)


def apply_default_settings(
    master: pysoem.Master,
    slave_id: int,
    bpf_id: int | None = None,
    ecat_ack_check: bool = False,
) -> None:
    """Apply one HPF's TOML settings in the established transmission order.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        bpf_id: Optional expected BPF ID; mismatched membership is rejected.
        ecat_ack_check: Log ecat_ack after each write when True.

    Returns:
        None.

    Raises:
        ValueError: The slave is unmapped or does not belong to the expected BPF.

    Notes:
        Writes FREQ, FRQ2, then CONTROLLER_PARAMETER_ORDER; waits
        SETTINGS_APPLY_WAIT_S afterwards. Does not reset, enable, or find index.
        Acknowledgement logging is diagnostic only; it neither enforces an ACK
        nor reads back or verifies the actual parameter values.
    """
    configured_bpf = get_bpf_id_for_slave(slave_id)
    if bpf_id is not None and bpf_id != configured_bpf:
        raise ValueError(f"Slave {slave_id} belongs to BPF #{configured_bpf}, not #{bpf_id}")
    hpf = CONFIG["hpfs"][str(get_hpf_id_for_slave(slave_id))]
    freq, frq2 = hpf["FREQ"], hpf["FRQ2"]

    parameters = [
        ("FREQ", freq),
        ("FRQ2", frq2),
    ] + [(name, CONFIG["controller_defaults"][name]) for name in CONTROLLER_PARAMETER_ORDER]
    for param_name, value in parameters:
        set_param(master, slave_id, param_name, value)
        if ecat_ack_check:
            logger.debug("Slave %s / %s: ecat_ack=%s", slave_id, param_name,
                         read_status(master, slave_id)['ecat_ack'])
    time.sleep(SETTINGS_APPLY_WAIT_S)


def pdo_settle(
    master: pysoem.Master,
    cycles: int = PDO_SETTLE_CYCLES,
    dt: float = PDO_SETTLE_INTERVAL_S,
) -> None:
    """Repeat PDO exchanges with an additional pause after each cycle.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        cycles: Number of calls to trig(); defaults to PDO_SETTLE_CYCLES.
        dt: Additional sleep in seconds after each trig() call.

    Returns:
        None.

    Notes:
        Each cycle includes communication time, PDO_EXCHANGE_PAUSE_S, and dt.
        Does not prove that a controller state transition has completed.
    """
    for _ in range(cycles):
        trig(master)
        time.sleep(dt)


def prepare_actuator(master: pysoem.Master, slave_id: int, bpf_id: int | None = None) -> None:
    """Reset, configure, and enable one actuator without searching for index.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_id: Zero-based EtherCAT slave index.
        bpf_id: Optional expected BPF ID; membership is checked before reset.

    Returns:
        None.

    Raises:
        ValueError: The slave is unmapped or belongs to another BPF.
        RuntimeError: A checked error occurs while enabling.
        TimeoutError: The enable wait expires.

    Notes:
        Waits RESET_WAIT_S after reset, applies settings, and exchanges the
        configured settling cycles after settings and enable. find_index() is
        separate. This function does not provide its own HALT-on-error guard.
    """
    configured_bpf = get_bpf_id_for_slave(slave_id)
    if bpf_id is not None and bpf_id != configured_bpf:
        raise ValueError(f"Slave {slave_id} belongs to BPF #{configured_bpf}, not #{bpf_id}")
    reset(master, slave_id)
    time.sleep(RESET_WAIT_S)

    apply_default_settings(master, slave_id, bpf_id=bpf_id)
    pdo_settle(master, cycles=AFTER_SETTINGS_PDO_CYCLES, dt=PDO_SETTLE_INTERVAL_S)
    
    enable(master, slave_id)
    pdo_settle(master, cycles=AFTER_ENABLE_PDO_CYCLES, dt=PDO_SETTLE_INTERVAL_S)


def dpos(
    master: pysoem.Master,
    slaveId: int,
    target_pos_mm: float,
    vel: int = DEFAULT_VEL,
    accel: int = ACCEL,
    decel: int = DECEL,
) -> None:
    """Convert an absolute millimetre target to encoder units and move.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slaveId: Zero-based EtherCAT slave index.
        target_pos_mm: Absolute position in millimetres, not a relative displacement.
        vel: Velocity passed in raw controller command units.
        accel: Acceleration passed in raw controller command units.
        decel: Deceleration passed in raw controller command units.

    Returns:
        None; logs the actual position in millimetres after the move.

    Raises:
        RuntimeError: move_abs() rejects readiness or observes a checked error.
        TimeoutError: move_abs() does not observe completion in time.

    Notes:
        Uses round(target_pos_mm / RESOLUTION_MM), including Python's rounding
        rule. Delegates motion to move_abs(); does not enforce the BPF limit or
        provide an independent HALT guard.
    """
    target_pos_encoder = round(target_pos_mm / RESOLUTION_MM)
    move_abs(master, slaveId, target_pos_encoder, vel, accel, decel)
    apos_encoder = read_status(master, slaveId)["pos"]
    apos_mm = encoder_to_mm(apos_encoder)
    logger.info(f"APOS = {apos_mm:.5f} mm")

def calc_pos_mm_from_fcut(f_cut_GHz, A, B, X0):
    """Invert the calibrated cutoff-frequency relation to a position.

    Args:
        f_cut_GHz: Required cutoff frequency in GHz.
        A: Calibration coefficient in GHz * mm.
        B: Calibration frequency offset in GHz.
        X0: Calibration position offset in millimetres.

    Returns:
        X0 - A / (f_cut_GHz - B), in millimetres.

    Raises:
        ValueError: f_cut_GHz equals B, making the denominator zero.

    Notes:
        No hardware access. Does not validate calibration coverage, finiteness,
        or the position safety limit.
    """
    if f_cut_GHz == B:
        raise ValueError("f_cut_GHz - Bが0になるため位置を計算できません。")

    return X0 - A / (f_cut_GHz - B)


def calc_bpf_positions(bpf_id, central_freq_GHz, bandwidth_GHz):
    """Calculate filter cutoffs and all BPF actuator targets without moving.

    Args:
        bpf_id: BPF ID defined in config.toml.
        central_freq_GHz: Filter-side center frequency in GHz, not RF LO.
        bandwidth_GHz: Positive full bandwidth at the filter in GHz.

    Returns:
        A dictionary with BPF ID, filter center/bandwidth/cutoffs in GHz, and
        each 'HPF #<id> position_mm' target in millimetres.

    Raises:
        ValueError: BPF ID, finite-frequency/bandwidth requirements, positive
            lower cutoff, or a calibration denominator is invalid.

    Notes:
        Uses center +/- bandwidth/2 and each HPF's cutoff_side and A/B/X0.
        Does not enforce the BPF software position limit or access hardware.
    """
    hpf_ids = get_bpf_hpf_ids(bpf_id)

    central_freq_GHz = float(central_freq_GHz)
    bandwidth_GHz = float(bandwidth_GHz)

    if not math.isfinite(central_freq_GHz):
        raise ValueError("central_freq_GHzには有限の数値を指定してください。")
    if not math.isfinite(bandwidth_GHz) or bandwidth_GHz <= 0:
        raise ValueError("bandwidth_GHzは0より大きい有限の数値を指定してください。")

    freq_hpf_GHz = central_freq_GHz - bandwidth_GHz/2
    freq_lpf_GHz = central_freq_GHz + bandwidth_GHz/2

    if freq_hpf_GHz <= 0:
        raise ValueError("HPF側cutoffが0 GHz以下になっています。")

    positions = {
        "bpf_id": bpf_id,
        "central_frequency_GHz": central_freq_GHz,
        "bandwidth_GHz": bandwidth_GHz,
        "HPF_cutoff_GHz": freq_hpf_GHz,
        "LPF_cutoff_GHz": freq_lpf_GHz,
    }

    for hpf_id in hpf_ids:
        hpf = CONFIG["hpfs"][str(hpf_id)]
        cutoff = freq_hpf_GHz if hpf["cutoff_side"] == "hpf" else freq_lpf_GHz
        positions[f"HPF #{hpf_id} position_mm"] = calc_pos_mm_from_fcut(
            cutoff, hpf["A"], hpf["B"], hpf["X0"]
        )

    return positions

def check_bpf_actuator_ready(master: pysoem.Master, name: str, slave_id: int) -> dict[str, Any]:
    """Read one actuator and enforce the current BPF readiness predicate.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        name: Human-readable actuator label used in errors.
        slave_id: Zero-based EtherCAT slave index.

    Returns:
        The observed status dictionary.

    Raises:
        RuntimeError: enabled or encoder_valid is False, or has_error() is True.

    Notes:
        Missing preparation produces a message instructing the caller to prepare.
        Does not reset/enable, read back settings, or check for ongoing motion.
    """
    status = read_status(master, slave_id)

    if not status["enabled"]:
        raise RuntimeError(f"{name}がenableされていません。準備が完了していません。prepareを実行してください。")
    if not status["encoder_valid"]:
        raise RuntimeError(f"{name}のindexが見つかっていません。準備が完了していません。prepareを実行してください。")
    if has_error(status):
        raise RuntimeError(f"{name}にエラーがあります: {status}")

    return status


def halt_bpf(master: pysoem.Master, bpf_id: int) -> None:
    """Attempt HALT for every configured member of a BPF.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        bpf_id: BPF ID defined in config.toml.

    Returns:
        None; each success or failure is logged.

    Raises:
        ValueError: bpf_id is not configured.

    Notes:
        Individual Exception failures are logged and suppressed so remaining
        members are attempted. Does not wait or read status, return a collective
        success flag, or verify physical stopping.
    """
    actuator_list = [
        (f"HPF #{i}", CONFIG["hpfs"][str(i)]["slave_id"])
        for i in get_bpf_hpf_ids(bpf_id)
    ]

    for name, slave_id in actuator_list:
        try:
            halt(master, slave_id)
            logger.info(f"✅ {name} halted")
        except Exception as error:
            logger.error(f"🔴 {name} could not be halted: {error}")


def print_bpf_movement_result(name, slave_id, calculated_position_mm, status):
    """Log a movement summary and return its position/status fields.

    Args:
        name: Human-readable actuator label.
        slave_id: Zero-based slave index used for display.
        calculated_position_mm: Unrounded target from the frequency calibration.
        status: Final read_status() dictionary.

    Returns:
        A dictionary containing the calculated target, rounded encoder-grid
        target, actual position in mm, error in encoder units, and status.

    Notes:
        Error is actual minus the unrounded calculated target, expressed in encoder
        units; it need not be an integer. Despite its name, uses logging, not print.
        Does not acquire status or assert that the target was physically reached.
    """
    target_position_mm = round(calculated_position_mm / RESOLUTION_MM) * RESOLUTION_MM
    actual_position_mm = encoder_to_mm(status["pos"])
    error_encoder = (actual_position_mm - calculated_position_mm)/RESOLUTION_MM

    logger.debug("")
    logger.info(f"🔵 {name} movement result")
    logger.info(f"Slave ID            = {slave_id}")
    logger.info(f"Target position     = {calculated_position_mm:.8f} mm")
    logger.info(f"Input position      = {target_position_mm:.5f} mm")
    logger.info(f"Actual position     = {actual_position_mm:.5f} mm")
    logger.info(f"Actual - Target     = {error_encoder:.1f} eu")
    logger.debug("Full status =")
    logger.debug(status)
    logger.debug(status["status_raw"])

    return {
        "slaveId": slave_id,
        "calculated_position_mm": calculated_position_mm,
        "target_position_mm": target_position_mm,
        "actual_position_mm": actual_position_mm,
        "error_encoder_unit": error_encoder,
        "status": status,
    }


def move_bpf(
    master: pysoem.Master,
    bpf_id: int,
    central_freq_GHz: float,
    bandwidth_GHz: float,
    vel: int = DEFAULT_VEL,
    accel: int = ACCEL,
    decel: int = DECEL,
) -> dict[str, Any] | None:
    ## frequency -> position
    """Move a prepared BPF using filter-side frequencies and configured order.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        bpf_id: BPF ID defined in config.toml.
        central_freq_GHz: Filter-side center in GHz, not the RF LO frequency.
        bandwidth_GHz: Full filter-side bandwidth in GHz.
        vel: Velocity passed in raw controller command units.
        accel: Acceleration passed in raw controller command units.
        decel: Deceleration passed in raw controller command units.

    Returns:
        A result dictionary with filter frequencies and per-HPF summaries;
        None if a target meets or exceeds the software cancellation boundary.

    Raises:
        ValueError: Frequency inputs, BPF ID, or calibration calculation is invalid.
        RuntimeError: Master/slave availability, readiness, or motion checks fail.
        TimeoutError: An actuator move does not complete in its wait window.
        KeyboardInterrupt: Motion is interrupted; the guard attempts BPF-wide HALT.

    Notes:
        Checks all members before driving; never auto-prepares. Cancels when any
        target >= MAX_DESIRED_POSITION_MM - RESOLUTION_MM (2.49875 mm for the
        current resolution), independently of controller HLIM/LLIM.
        The motion loop is guarded: failures or Ctrl+C attempt HALT for the whole
        BPF on the same master. Pre-motion rejection and post-loop summary reads
        are outside that guard. There is no automatic rollback or numeric final
        position-tolerance verification.
    """
    positions = calc_bpf_positions(bpf_id, central_freq_GHz, bandwidth_GHz)

    actuator_list = [
        (f"HPF #{i}", CONFIG["hpfs"][str(i)]["slave_id"], positions[f"HPF #{i} position_mm"])
        for i in get_bpf_hpf_ids(bpf_id, motion_order=True)
    ]

    ## status check
    if master is None:
        raise RuntimeError("ethercat_master(IFNAME)で取得したmasterを渡してください。")

    max_slave_id = max(slave_id for _, slave_id, _ in actuator_list)
    if len(master.slaves) <= max_slave_id:
        raise RuntimeError(
            f"slave ID = {max_slave_id}まで必要ですが、"
            f"接続されているslave数は{len(master.slaves)}です。"
        )

    status_before = {}
    for name, slave_id, calculated_position in actuator_list:
        status_before[name] = check_bpf_actuator_ready(master, name, slave_id)

    ## pre-motion setting check
    logger.info(f"🔵 BPF #{bpf_id} setting")
    logger.info(f"Center frequency = {positions['central_frequency_GHz']:.5f} GHz")
    logger.info(f"Bandwidth        = {positions['bandwidth_GHz']:.5f} GHz")
    logger.info(f"HPF cutoff       = {positions['HPF_cutoff_GHz']:.5f} GHz")
    logger.info(f"LPF cutoff       = {positions['LPF_cutoff_GHz']:.5f} GHz")
    logger.debug("")

    logger.info("🔵 Actuator movement preview")
    for name, slave_id, calculated_position in actuator_list:
        current_position = encoder_to_mm(status_before[name]["pos"])
        logger.info(f"{name} / slave {slave_id}: "
            f"current {current_position:.5f} mm -> target {calculated_position:.5f} mm")

    logger.debug("")

    # soft limit
    if any(
        calculated_position >= MAX_DESIRED_POSITION_MM - RESOLUTION_MM
        for _, _, calculated_position in actuator_list
    ):
        logger.warning("🔴 Motion canceled")
        return None

    with halt_on_motion_error(master, bpf_id):
        # Follow the explicit move_order in config.toml.
        for name, slave_id, calculated_position in actuator_list:
            dpos(master, slave_id, calculated_position, vel=vel, accel=accel, decel=decel)

    result = {
        "bpf_id": bpf_id,
        "central_frequency_GHz": positions["central_frequency_GHz"],
        "bandwidth_GHz": positions["bandwidth_GHz"],
        "HPF_cutoff_GHz": positions["HPF_cutoff_GHz"],
        "LPF_cutoff_GHz": positions["LPF_cutoff_GHz"],
    }

    print_section("Final status")
    for name, slave_id, calculated_position in actuator_list:
        status = read_status(master, slave_id)
        result[name] = print_bpf_movement_result(
            name,
            slave_id,
            calculated_position,
            status,
        )

    return result

def close(master: pysoem.Master, *, request_init: bool = True) -> None:
    """Request INIT and close the adapter, even if the INIT request fails.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        request_init: False if adapter opening did not complete; skips INIT only.

    Returns:
        None.

    Raises:
        BaseException: INIT or adapter cleanup fails; close is still attempted.

    Notes:
        Normally owned by ethercat_master(); do not call manually inside its block.
        If INIT and close both fail, the close error carries a note about INIT.
        If only INIT fails, it is re-raised after successful close. Does not HALT.
    """
    state_error = None
    try:
        if request_init:
            master.state = pysoem.INIT_STATE
            master.write_state()
    except BaseException as error:
        state_error = error
        logger.error("Failed to request EtherCAT INIT during cleanup: %s", error)
    finally:
        try:
            master.close()
        except BaseException as error:
            if state_error is not None:
                error.add_note(f"INIT request also failed: {state_error}")
            logger.error("Failed to close EtherCAT master: %s", error)
            raise
    logger.info("Master has been closed")
    if state_error is not None:
        raise state_error


# 2026-10-04: CLI統合用の上位制御。Typerには依存しない。
HALT_STATUS_WAIT_S = 0.5
RESET_STATUS_WAIT_S = 0.5


def get_receiver_config(rx: str) -> dict[str, Any]:
    """Look up receiver settings by its CLI selector.

    Args:
        rx: Configured cli_name, currently '4+5' or '6+7', not 'B45'/'B67'.

    Returns:
        The receiver dictionary from CONFIG, including bpf_id and
        rf_to_filter_multiplier; this is not a copy.

    Raises:
        ValueError: No receiver has the specified CLI name.
    """
    for receiver in CONFIG["receivers"].values():
        if receiver["cli_name"] == rx:
            return receiver
    choices = ", ".join(receiver["cli_name"] for receiver in CONFIG["receivers"].values())
    raise ValueError(f"Unknown receiver: {rx}. Choose: {choices}")


def resolve_target(rx: str | None, hpf_id: int | None) -> tuple[int, list[int]]:
    """Resolve exactly one receiver or HPF selector to a BPF and slave list.

    Args:
        rx: CLI receiver selector, or None when hpf_id is supplied.
        hpf_id: One-based configured HPF ID, or None when rx is supplied.

    Returns:
        A (bpf_id, slave_ids) tuple. Receiver selection includes all members;
        HPF selection includes only its mapped zero-based slave ID.

    Raises:
        ValueError: Both/neither selectors are supplied, or the target is unknown.

    Notes:
        No hardware access and no implicit all-devices target.
    """
    if (rx is None) == (hpf_id is None):
        raise ValueError("--rx または --hpf-id のどちらか一方を指定してください。")
    if rx is not None:
        bpf_id = get_receiver_config(rx)["bpf_id"]
        return bpf_id, get_bpf_slave_ids(bpf_id)
    if str(hpf_id) not in CONFIG["hpfs"]:
        raise ValueError(f"Unknown HPF ID: {hpf_id}")
    slave_id = CONFIG["hpfs"][str(hpf_id)]["slave_id"]
    return get_bpf_id_for_slave(slave_id), [slave_id]


def build_lo_request(rx: str, lo_rf_GHz: float, bandwidth_GHz: float) -> dict[str, Any]:
    """Convert an RF LO request to filter frequencies and calibrated targets.

    Args:
        rx: Receiver CLI selector.
        lo_rf_GHz: Positive finite RF first-LO frequency in GHz.
        bandwidth_GHz: Positive full filter-side bandwidth in GHz; not divided
        by the RF multiplier.

    Returns:
        A dictionary with rx, lo_rf_GHz, multiplier and calc_bpf_positions() fields.

    Raises:
        ValueError: Receiver, frequency inputs, or calibration calculation is invalid.

    Notes:
        Filter center is lo_rf_GHz / rf_to_filter_multiplier. Performs no hardware
        access, readiness checks, or software position-limit enforcement.
    """
    if not math.isfinite(lo_rf_GHz) or lo_rf_GHz <= 0:
        raise ValueError("--loには0より大きい有限の数値を指定してください。")
    receiver = get_receiver_config(rx)
    center = lo_rf_GHz / receiver["rf_to_filter_multiplier"]
    positions = calc_bpf_positions(receiver["bpf_id"], center, bandwidth_GHz)
    return {
        "rx": rx, "lo_rf_GHz": lo_rf_GHz,
        "multiplier": receiver["rf_to_filter_multiplier"],
        **positions,
    }


def validate_connected_slaves(master: pysoem.Master, slave_ids: list[int]) -> None:
    """Check that each requested index exists in master.slaves.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_ids: List of zero-based EtherCAT slave indices.

    Returns:
        None.

    Raises:
        RuntimeError: An index is negative or beyond the discovered slave count.

    Notes:
        Checks indices only, not device identity, PDO validity, or readiness.
    """
    for slave_id in slave_ids:
        if not 0 <= slave_id < len(master.slaves):
            raise RuntimeError(
                f"slave ID {slave_id}が必要ですが、接続slave数は{len(master.slaves)}です。"
            )


def log_selected_status(master: pysoem.Master, slave_ids: list[int]) -> None:
    """Read and log positions and full status for selected slaves at INFO.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_ids: Zero-based indices in the order to inspect.

    Returns:
        None.

    Raises:
        RuntimeError: A requested index is unavailable.

    Notes:
        Does not enable, reset, prepare, or test readiness. read_status() exchanges
        current PDO buffers. Read/decoding errors propagate.
    """
    validate_connected_slaves(master, slave_ids)
    for slave_id in slave_ids:
        status = read_status(master, slave_id)
        logger.info("HPF #%s / slave %s: position %.5f mm",
                    get_hpf_id_for_slave(slave_id), slave_id, encoder_to_mm(status["pos"]))
        logger.info("%s", status)


def halt_after_motion_error(master: pysoem.Master, bpf_id: int) -> None:
    """Attempt BPF-wide HALT, pause, then log every member's status.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        bpf_id: BPF ID defined in config.toml.

    Returns:
        None.

    Raises:
        ValueError: bpf_id is not configured.

    Notes:
        Used even for single-HPF preparation failures. halt_bpf() attempts all
        members, then waits HALT_STATUS_WAIT_S seconds (currently 0.5).
        Individual status-read Exceptions are logged and suppressed. Neither
        HALT acknowledgement nor physical stopping is verified.
    """
    halt_bpf(master, bpf_id)
    time.sleep(HALT_STATUS_WAIT_S)
    for slave_id in get_bpf_slave_ids(bpf_id):
        try:
            logger.info("Status after HALT / slave %s: %s", slave_id, read_status(master, slave_id))
        except Exception as error:
            logger.error("HALT後のステータス取得失敗 / slave %s: %s", slave_id, error)


@contextmanager
def halt_on_motion_error(master: pysoem.Master, bpf_id: int) -> Iterator[None]:
    """Guard driving operations with best-effort BPF-wide HALT.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        bpf_id: BPF ID defined in config.toml.

    Raises:
        Exception: The original guarded operation's exception is re-raised.
        KeyboardInterrupt: Ctrl+C is re-raised after the HALT attempt.

    Notes:
        Catches Exception and KeyboardInterrupt, not SystemExit. Uses the same
        master, does not close it, and preserves the original failure if HALT
        handling also fails by logging and attaching a note. Place pre-motion
        input/readiness checks outside this guard. No rollback is performed.

    Yields:
        None; the guarded driving operation runs inside the with block.
    """
    try:
        yield
    except (Exception, KeyboardInterrupt) as error:
        logger.error("BPF #%sの駆動を中断します (%s: %s)。全3台へHALTを送信します。",
                     bpf_id, type(error).__name__, error)
        try:
            halt_after_motion_error(master, bpf_id)
        except BaseException as halt_error:
            logger.error("内部HALT処理にも失敗しました: %s", halt_error)
            error.add_note(f"Internal HALT failed: {halt_error}")
        raise


def prepare_selected(master: pysoem.Master, bpf_id: int, slave_ids: list[int]) -> None:
    """Prepare and reference selected actuators sequentially within a BPF.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        bpf_id: BPF ID defined in config.toml.
        slave_ids: Selected zero-based slave indices, normally from resolve_target().

    Returns:
        None; logs preparation progress and completion.

    Raises:
        RuntimeError: A selected index is unavailable or preparation fails.
        ValueError: Slave/BPF membership or direction validation fails.
        TimeoutError: Enabling, index search, or landing exceeds its wait limit.
        KeyboardInterrupt: Ctrl+C interrupts preparation.

    Notes:
        Validates selected indices before the guard. For each slave, runs
        prepare_actuator() then find_index(direction=0). Guarded failures attempt
        HALT for all BPF members, even when preparing only one selected HPF.
    """
    validate_connected_slaves(master, slave_ids)
    with halt_on_motion_error(master, bpf_id):
        for slave_id in slave_ids:
            logger.info("HPF #%s / slave %s: preparation started",
                        get_hpf_id_for_slave(slave_id), slave_id)
            prepare_actuator(master, slave_id, bpf_id=bpf_id)
            find_index(master, slave_id, direction=0)
            logger.debug("Status after prepare: %s", read_status(master, slave_id))
    logger.info("Preparation completed")


def halt_selected(master: pysoem.Master, slave_ids: list[int]) -> None:
    """Attempt user-requested HALT for selected slaves and log their status.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_ids: Zero-based indices in the order to halt.

    Returns:
        None.

    Raises:
        RuntimeError: A selected index is unavailable; rejected before HALT.

    Notes:
        Attempts all selected HALTs before waiting HALT_STATUS_WAIT_S seconds.
        Individual command/status Exceptions are logged and suppressed. Does not
        HALT other BPF members, verify motor_on=False, or return a success flag.
    """
    validate_connected_slaves(master, slave_ids)
    for slave_id in slave_ids:
        try:
            halt(master, slave_id)
            logger.info("HPF #%s / slave %s halted", get_hpf_id_for_slave(slave_id), slave_id)
        except Exception as error:
            logger.error("HALT failed / slave %s: %s", slave_id, error)
    time.sleep(HALT_STATUS_WAIT_S)
    for slave_id in slave_ids:
        try:
            log_selected_status(master, [slave_id])
        except Exception as error:
            logger.error("HALT後のステータス取得失敗 / slave %s: %s", slave_id, error)


def reset_selected(master: pysoem.Master, slave_ids: list[int]) -> None:
    """Reset selected controllers sequentially and log the resulting status.

    Args:
        master: Open, mapped PySOEM master owned by the caller.
        slave_ids: Zero-based indices in the order to reset.

    Returns:
        None.

    Raises:
        RuntimeError: A requested index is unavailable.

    Notes:
        Waits RESET_STATUS_WAIT_S seconds after each reset (currently 0.5).
        Does not prepare, enable, find index, or supply an automatic HALT guard.
        Command/status errors propagate and stop further resets. Run prepare
        before the next absolute move.
    """
    validate_connected_slaves(master, slave_ids)
    for slave_id in slave_ids:
        reset(master, slave_id)
        time.sleep(RESET_STATUS_WAIT_S)
        log_selected_status(master, [slave_id])
    logger.info("Reset completed. 次の移動前にprepareを実行してください。")


def print_section(title):
    """Log a section heading at INFO without printing directly.

    Args:
        title: Human-readable heading.

    Returns:
        None.
    """
    logger.info("%s", title)
