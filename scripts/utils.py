from __future__ import annotations

from contextlib import contextmanager
from typing import Any, Callable, Iterator
import pysoem
import time
import struct
import math
import numpy as np
from pathlib import Path
import tomllib
from scripts.logging_utils import get_logger

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
RECORD_POLL_INTERVAL_S = 0.005 # 診断ログの既存取得周期

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
    """Load and validate configuration before any EtherCAT communication."""
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


def get_bpf_id_for_receiver(receiver_name):
    try:
        return CONFIG["receivers"][receiver_name]["bpf_id"]
    except KeyError as exc:
        raise ValueError(f"Unknown receiver: {receiver_name}") from exc


def get_bpf_hpf_ids(bpf_id, *, motion_order=False):
    if type(bpf_id) is not int or bpf_id not in BPF_IDS:
        raise ValueError(f"Unknown BPF ID: {bpf_id}; configured IDs: {BPF_IDS}")
    key = "move_order" if motion_order else "hpf_ids"
    return list(CONFIG["bpfs"][str(bpf_id)][key])


def get_bpf_slave_ids(bpf_id):
    return [CONFIG["hpfs"][str(i)]["slave_id"] for i in get_bpf_hpf_ids(bpf_id)]


def get_hpf_id_for_slave(slave_id):
    if type(slave_id) is int:
        for hpf_id, hpf in CONFIG["hpfs"].items():
            if hpf["slave_id"] == slave_id:
                return int(hpf_id)
    raise ValueError(f"No HPF configured for slave ID {slave_id}")


def get_bpf_id_for_slave(slave_id):
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
    """Convert a position from encoder units to millimetres."""
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
    """Own a master for one with block, including initialization-failure cleanup.

    Preserve the established configuration/state-transition sequence and timing.
    Cleanup failures are raised on normal exit; during an existing exception
    they are logged and attached as a note without replacing the original error.
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
    """
    Xeryon EtherCAT PDO:
      cmd_bytes : 4-byte ASCII command, e.g. b'DPOS'
      v1        : main parameter
      v2        : velocity
      v3        : acceleration
      v4        : deceleration
      execute   : 0 -> prepare, 1 -> execute
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
    """
    Xeryon EtherCAT commands are sent in two steps:
      1. execute = 0
      2. execute = 1
    delay is retained for compatibility and is currently unused.
    """
    send_cmd(master, slave_id, cmd_bytes, EXECUTE_PREPARE, v1, v2, v3, v4)
    trig(master)

    send_cmd(master, slave_id, cmd_bytes, EXECUTE_RUN, v1, v2, v3, v4)
    trig(master)


# =========================
# Status handling
# =========================
def read_status(master: pysoem.Master, slave_id: int) -> dict[str, Any]:
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
    """
    ENBL=1:
      - enables motor signals
      - also recovers from errors
    """
    command(master, slave_id, b"ENBL", v1=1)
    wait_until(master, slave_id, lambda st: st["enabled"], timeout=ENABLE_TIMEOUT_S, label="enabled")
    trig(master)


def disable(master: pysoem.Master, slave_id: int) -> None:
    command(master, slave_id, b"ENBL", v1=0)


def halt(master: pysoem.Master, slave_id: int) -> None:
    """
    Normal stop.
    STOP is more like emergency stop and blocks following commands.
    """
    command(master, slave_id, b"HALT")
    trig(master)


def reset(master: pysoem.Master, slave_id: int) -> None:
    """
    RSET resets controller and settings to saved values.
    After this, ENBL and INDX are needed again.
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
    """
    direction:
      0 -> descending encoder direction
      1 -> ascending encoder direction
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
    """
    Absolute move by DPOS.
    target_pos is in encoder units.
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

def scan(
    master: pysoem.Master,
    slave_id: int,
    direction: int,
    vel: int = DEFAULT_VEL,
    accel: int = ACCEL,
    decel: int = DECEL,
) -> None:
    """
    Continuous closed-loop motion.
    direction:
      -1 -> negative direction
       0 -> stop
      +1 -> positive direction
    """
    if direction not in (-1, 0, 1):
        raise ValueError("SCAN direction must be -1, 0, or 1")

    command(master, slave_id, b"SCAN", v1=direction, v2=vel, v3=accel, v4=decel)


def stop_scan(master: pysoem.Master, slave_id: int) -> None:
    scan(master, slave_id, direction=0)

def set_param(
    master: pysoem.Master,
    slave_id: int,
    param_name: str,
    value: int | float,
    delay: float = LEGACY_UNUSED_COMMAND_DELAY_S,
) -> None:
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
    """Apply the known actuator settings after every controller reset.

    These values were previously duplicated in setup.py, test.py, and all.py.
    Calling this function after reset() avoids relying on RAM-resident settings
    left over from a previous script invocation.
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
    """Exchange PDOs for several cycles so the preceding command can settle."""
    for _ in range(cycles):
        trig(master)
        time.sleep(dt)


def prepare_actuator(master: pysoem.Master, slave_id: int, bpf_id: int | None = None) -> None:
    """Reset, configure, and enable one actuator for a fresh run.

    Index search is deliberately separate: callers must invoke find_index()
    explicitly when an absolute position reference is required.
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


def save_rows_csv(rows, csv_path):
    """
    rows: list of dict
    csv_path: 保存先CSV
    """
    if len(rows) == 0:
        logger.warning("No data to save.")
        return

    fieldnames = list(rows[0].keys())

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    logger.info(f"Saved: {csv_path}  ({len(rows)} samples)")


def record_until(
    master: pysoem.Master,
    slave_id: int,
    stop_condition: Callable[[dict[str, Any], float], bool],
    csv_path: str | Path = 'motion_log.csv',
    target_pos: int | None = None,
    poll_dt: float = RECORD_POLL_INTERVAL_S,
    timeout: float = DEFAULT_WAIT_TIMEOUT_S,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """
    駆動中のPDO入力を読み続けてCSV保存する関数。

    Parameters
    ----------
    slave_id : int
        EtherCAT slave ID

    stop_condition : callable
        stop_condition(st, t) が True になったら記録終了

    csv_path : str
        保存するCSVファイル名

    target_pos : int or None
        目標位置 [encoder unit]。
        指定すると error も保存する。

    poll_dt : float
        記録周期 [s]
        例: 0.005 -> 約200 Hz
            0.02  -> 約50 Hz

    timeout : float
        最大記録時間 [s]
    """

    rows = []

    t0 = time.perf_counter()
    next_t = t0

    prev_t = None
    prev_pos_um = None

    last_status = None

    try:
        while True:
            now = time.perf_counter()
            t = now - t0

            st = read_status(master, slave_id)
            last_status = st

            pos_enc = st["pos"]
            pos_um = pos_enc * RESOLUTION_UM

            # 推定速度 [um/s]
            if prev_t is None:
                vel_est_um_s = math.nan
            else:
                dt_meas = now - prev_t
                if dt_meas > 0:
                    vel_est_um_s = (pos_um - prev_pos_um) / dt_meas
                else:
                    vel_est_um_s = math.nan

            if target_pos is None:
                error_enc = math.nan
                error_um = math.nan
            else:
                error_enc = target_pos - pos_enc
                error_um = error_enc * RESOLUTION_UM

            row = {
                "t_s": t,
                "pos_enc": pos_enc,
                "pos_um": pos_um,
                "vel_est_um_s": vel_est_um_s,
                "target_pos_enc": target_pos if target_pos is not None else "",
                "error_enc": error_enc,
                "error_um": error_um,

                "enabled": int(st["enabled"]),
                "motor_on": int(st["motor_on"]),
                "closed_loop": int(st["closed_loop"]),
                "encoder_index": int(st["encoder_index"]),
                "encoder_valid": int(st["encoder_valid"]),
                "searching_index": int(st["searching_index"]),
                "position_reached": int(st["position_reached"]),
                "scanning": int(st["scanning"]),

                "end_stop": int(st["end_stop"]),
                "left_end_stop": int(st["left_end_stop"]),
                "right_end_stop": int(st["right_end_stop"]),

                "encoder_error": int(st["encoder_error"]),
                "error_limit": int(st["error_limit"]),
                "safety_timeout": int(st["safety_timeout"]),
                "emergency_stop": int(st["emergency_stop"]),
                "position_fail": int(st["position_fail"]),

                "ecat_ack": int(st["ecat_ack"]),
                "status_raw": st["status_raw"],
            }

            rows.append(row)

            prev_t = now
            prev_pos_um = pos_um

            if has_error(st):
                raise RuntimeError(f"Controller error during recording: {st}")

            if stop_condition(st, t):
                break

            if t > timeout:
                raise TimeoutError(f"Recording timeout. Last status: {st}")

            next_t += poll_dt
            sleep_time = next_t - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        save_rows_csv(rows, csv_path)

    return rows, last_status

def plot_motion_log_encoder(csv_path, save_path=None, smooth_window=1):
    """
    Xeryonの駆動ログCSVを読み込んでプロットする。
    位置と誤差は encoder unit のまま表示する。

    Parameters
    ----------
    csv_path : str
        record_until(), move_abs_record(), scan_record() などで保存したCSV

    save_path : str or None
        図を保存する場合のファイル名。
        例: "motion_plot.png"

    smooth_window : int
        速度の移動平均窓。
        1なら平滑化なし。
    """

    df = pd.read_csv(csv_path)

    # ==========================
    # 数値列を安全に変換
    # ==========================
    for col in df.columns:
        if col != "status_raw":
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # ==========================
    # time
    # ==========================
    t = df["t_s"].to_numpy()

    # ==========================
    # position [encoder unit]
    # ==========================
    pos_enc = df["pos_enc"].to_numpy()

    # ==========================
    # velocity [encoder unit/s]
    # ==========================
    if "vel_est_enc_s" in df.columns:
        vel_enc_s = df["vel_est_enc_s"].to_numpy()
    else:
        vel_enc_s = np.gradient(pos_enc, t)

    if smooth_window > 1:
        vel_enc_s = (
            pd.Series(vel_enc_s)
            .rolling(window=smooth_window, center=True, min_periods=1)
            .mean()
            .to_numpy()
        )

    # ==========================
    # error [encoder unit]
    # ==========================
    has_error_col = "error_enc" in df.columns and df["error_enc"].notna().any()

    if has_error_col:
        error_enc = df["error_enc"].to_numpy()
    else:
        error_enc = None

    # ==========================
    # target [encoder unit]
    # ==========================
    has_target = (
        "target_pos_enc" in df.columns
        and df["target_pos_enc"].notna().any()
    )

    if has_target:
        target_enc = df["target_pos_enc"].dropna().iloc[0]
    else:
        target_enc = None

    # ==========================
    # plot
    # ==========================
    nrows = 4 if error_enc is not None else 3
    fig, axes = plt.subplots(nrows, 1, figsize=(10, 8), sharex=True)

    ax_pos = axes[0]
    ax_vel = axes[1]

    if error_enc is not None:
        ax_err = axes[2]
        ax_status = axes[3]
    else:
        ax_status = axes[2]

    # --------------------------
    # position
    # --------------------------
    ax_pos.plot(t, pos_enc, label="Actual position")

    if target_enc is not None:
        ax_pos.axhline(target_enc, linestyle="--", label="Target position")

    ax_pos.set_ylabel("Position [encoder unit]")
    ax_pos.grid(True)
    ax_pos.legend()

    # --------------------------
    # velocity
    # --------------------------
    ax_vel.plot(t, vel_enc_s, label="Estimated velocity")
    ax_vel.axhline(0, linestyle=":")

    ax_vel.set_ylabel("Velocity [encoder unit/s]")
    ax_vel.grid(True)
    ax_vel.legend()

    # --------------------------
    # error
    # --------------------------
    if error_enc is not None:
        ax_err.plot(t, error_enc, label="Target - actual")
        ax_err.axhline(0, linestyle=":")

        ax_err.set_ylabel("Error [encoder unit]")
        ax_err.grid(True)
        ax_err.legend()

    # --------------------------
    # status bits
    # --------------------------
    status_cols = [
        "motor_on",
        "closed_loop",
        "position_reached",
        "encoder_valid",
        "error_limit",
        "safety_timeout",
        "position_fail",
    ]

    y_offset = 0
    yticks = []
    ylabels = []

    for col in status_cols:
        if col in df.columns:
            y = df[col].fillna(0).to_numpy()
            ax_status.step(t, y + y_offset, where="post", label=col)

            yticks.append(y_offset + 0.5)
            ylabels.append(col)
            y_offset += 1.5

    ax_status.set_yticks(yticks)
    ax_status.set_yticklabels(ylabels)
    ax_status.set_xlabel("Time [s]")
    ax_status.set_ylabel("Status")
    ax_status.grid(True)

    plt.tight_layout()

    if save_path is not None:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        logger.info(f"Saved figure: {save_path}")

    plt.show()

    return df

def move_abs_plot(
    master: pysoem.Master,
    slave_id: int,
    target_pos: int,
    vel: int = DEFAULT_VEL,
    accel: int = ACCEL,
    decel: int = DECEL,
    csv_path: str | Path | None = None,
    plot: bool = True,
    plot_save_path: str | Path | None = None,
    poll_dt: float = 0.005,
    timeout: float = 30.0,
    smooth_window: int = 5,
    min_record_time: float = 0.05,
    target_tolerance: int = 2,
) -> tuple[list[dict[str, Any]], dict[str, Any], Any]:
    """
    DPOSで絶対位置移動しながらデータを保存し、必要なら自動でプロットする。

    Parameters
    ----------
    slave_id : int
        EtherCAT slave ID

    target_pos : int
        目標位置 [encoder unit]

    vel : int
        速度 [um/s]

    accel : int
        加速度 [mm/s^2 or controller unit]

    decel : int
        減速度 [mm/s^2 or controller unit]

    csv_path : str or None
        保存するCSVファイル名。
        Noneなら target_pos から自動生成。

    plot : bool
        True なら移動後に自動でプロットする。
        False ならCSV保存のみ。

    plot_save_path : str or None
        プロット画像を保存する場合のファイル名。
        Noneなら画像保存なし。

    poll_dt : float
        データ取得周期 [s]

    timeout : float
        最大待ち時間 [s]

    smooth_window : int
        速度プロットの移動平均窓。

    min_record_time : float
        最低限記録する時間 [s]。
        position_reached が最初から立っている場合の即終了を防ぐ。

    target_tolerance : int
        目標到達判定用の許容誤差 [encoder unit]
    """

    # ==========================
    # check status
    # ==========================
    st0 = read_status(master, slave_id)
    start_pos = st0["pos"]

    if not st0["enabled"]:
        raise RuntimeError("Controller is not enabled. Run enable() first.")

    if not st0["encoder_valid"]:
        raise RuntimeError("Index is not found yet. Run find_index() first.")

    if csv_path is None:
        csv_path = f"move_abs_{start_pos}_to_{target_pos}.csv"

    # ==========================
    # send DPOS command
    # ==========================
    send_cmd(master, slave_id, b"DPOS", 0, target_pos, vel, accel, decel)
    trig(master)
    time.sleep(0.02)

    send_cmd(master, slave_id, b"DPOS", 1, target_pos, vel, accel, decel)
    trig(master)

    # ==========================
    # record during motion
    # ==========================
    state = {
        "motion_started": False,
    }

    def stop_condition(st, t):
        # motor_on が立つ、position_reached が落ちる、または位置が変われば
        # 実際に移動が始まったとみなす
        if (
            st["motor_on"]
            or not st["position_reached"]
            or abs(st["pos"] - start_pos) > 0
        ):
            state["motion_started"] = True

        reached = st["position_reached"]
        close_to_target = abs(st["pos"] - target_pos) <= target_tolerance

        # 通常の終了条件
        if state["motion_started"] and reached and t > min_record_time:
            return True

        # すでに目標位置近傍にいる場合の終了条件
        if reached and close_to_target and t > min_record_time:
            return True

        return False

    rows, last_status = record_until(
        master,
        slave_id=slave_id,
        stop_condition=stop_condition,
        csv_path=csv_path,
        target_pos=target_pos,
        poll_dt=poll_dt,
        timeout=timeout,
    )

    time.sleep(SETTLE_TIME)
    st = read_status(master, slave_id)

    final_error = st["pos"] - target_pos

    logger.info(f"Target={target_pos}, "
        f"Actual={st['pos']}, "
        f"Error={final_error} encoder unit")
    logger.info(f"Saved log: {csv_path}")

    # ==========================
    # plot automatically
    # ==========================
    df = None

    if plot:
        df = plot_motion_log_encoder(
            csv_path,
            save_path=plot_save_path,
            smooth_window=smooth_window,
        )

    return rows, st, df

def dpos(
    master: pysoem.Master,
    slaveId: int,
    target_pos_mm: float,
    vel: int = DEFAULT_VEL,
    accel: int = ACCEL,
    decel: int = DECEL,
) -> None:
    target_pos_encoder = round(target_pos_mm / RESOLUTION_MM)
    move_abs(master, slaveId, target_pos_encoder, vel, accel, decel)
    apos_encoder = read_status(master, slaveId)["pos"]
    apos_mm = encoder_to_mm(apos_encoder)
    logger.info(f"APOS = {apos_mm:.5f} mm")

def calc_pos_mm_from_fcut(f_cut_GHz, A, B, X0):
    if f_cut_GHz == B:
        raise ValueError("f_cut_GHz - Bが0になるため位置を計算できません。")

    return X0 - A / (f_cut_GHz - B)


def calc_bpf_positions(bpf_id, central_freq_GHz, bandwidth_GHz):
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
    status = read_status(master, slave_id)

    if not status["enabled"]:
        raise RuntimeError(f"{name}がenableされていません。")
    if not status["encoder_valid"]:
        raise RuntimeError(f"{name}のindexが見つかっていません。")
    if has_error(status):
        raise RuntimeError(f"{name}にエラーがあります: {status}")

    return status


def halt_bpf(master: pysoem.Master, bpf_id: int) -> None:
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

    try:
        # Follow the explicit move_order in config.toml.
        for name, slave_id, calculated_position in actuator_list:
            dpos(master, slave_id, calculated_position, vel=vel, accel=accel, decel=decel)
    except Exception:
        logger.error("🔴 移動中にエラーが発生しました。")
        raise

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
    """Request INIT and close the adapter even if the state request fails.

    Normally called by ethercat_master(), not explicitly inside its with block.
    request_init=False is used when opening the adapter did not complete.
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

def print_section(title):
    logger.info("%s", title)
