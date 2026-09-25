import atexit
import pysoem
import time
import struct
import math
import numpy as np

master = None
_close_registered = False

DEFAULT_VEL = 500000   
INDEX_VEL = 50000       
ACCEL = 65000            
DECEL = 65000

POLL_DT = 0.02
SETTLE_TIME = 1.0

# =========================
# Actuator Parameter
# =========================

# BPF #1: HPF #1（下側）+ LPF #1 = HPF #2・#3（上側）
SLAVE_ID_HPF_1 = 0
SLAVE_ID_HPF_2 = 1
SLAVE_ID_HPF_3 = 2

# BPF #2: LPF #2 = HPF #4・#6（上側）+ HPF #5（下側）
SLAVE_ID_HPF_4 = 3
SLAVE_ID_HPF_5 = 4
SLAVE_ID_HPF_6 = 5

RESOLUTION_UM = 1.25
RESOLUTION_MM = RESOLUTION_UM * 1e-3


def encoder_to_mm(position_encoder):
    """Convert a position from encoder units to millimetres."""
    return position_encoder * RESOLUTION_MM

# LPF #1のActual positionと実測LPF cutoffの−3 dB fitting結果
# 2026年8月5日測定。70 GHzの測定点はfittingから除外。
A_LPF_HPF_2 = 139.77990996621924
B_LPF_HPF_2 = 4.146514102344519
X0_LPF_HPF_2 = 3.6831305389197464

A_LPF_HPF_3 = 150.00098052196947
B_LPF_HPF_3 = 1.3454350285068244
X0_LPF_HPF_3 = 3.424692246476261

# HPF #1単体の−3 dB fitting結果（2026年8月4日測定）
A_HPF_1 = 140.96792648249118
B_HPF_1 = 3.2159676191402906
X0_HPF_1 = 3.3405032839438187

# LPF #2 / HPF #4のActual positionと実測LPF cutoffのfitting結果
A_LPF_HPF_4 = 134.2169506476309
B_LPF_HPF_4 = 6.079161691165807
X0_LPF_HPF_4 = 2.562860947465932

# LPF #2 / HPF #6のActual positionと実測LPF cutoffのfitting結果
A_LPF_HPF_6 = 127.51463334495108
B_LPF_HPF_6 = 8.130669597652863
X0_LPF_HPF_6 = 2.760685100749963

# HPF #5単体のfitting結果
A_HPF_5 = 154.6067528526659
B_HPF_5 = -0.7844167254033043
X0_HPF_5 = 3.0330052831635563

# =========================
# EtherCAT basic functions
# =========================
def init(ifname):
    global master, _close_registered

    master = pysoem.Master()
    master.open(ifname)

    if not _close_registered:
        atexit.register(close)
        _close_registered = True

    if master.config_init() < 1:
        raise RuntimeError(f"No EtherCAT slaves found on {ifname}")

    master.config_map()

    master.state_check(pysoem.SAFEOP_STATE, timeout=50000)
    if master.state != pysoem.SAFEOP_STATE:
        raise RuntimeError("Failed to reach EtherCAT SAFEOP_STATE")

    master.state = pysoem.OP_STATE
    master.write_state()
    master.state_check(pysoem.OP_STATE, timeout=50000)
    if master.state != pysoem.OP_STATE:
        raise RuntimeError("Failed to reach EtherCAT OP_STATE")

    print("Master is in OP_STATE")


def send_cmd(slave_id, cmd_bytes, execute, v1=0, v2=0, v3=0, v4=0):
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
        "<4siiHHB",
        cmd_bytes,
        int(v1),
        int(v2),
        int(v3),
        int(v4),
        int(execute),
    )

    slave = master.slaves[slave_id]
    slave.output = payload.ljust(len(slave.output), b"\x00")

def trig():
    master.send_processdata()
    master.receive_processdata()
    time.sleep(0.002)
    


def command(slave_id, cmd_bytes, v1=0, v2=0, v3=0, v4=0, delay=0.02):
    """
    Xeryon EtherCAT commands are sent in two steps:
      1. execute = 0
      2. execute = 1
    """
    send_cmd(slave_id, cmd_bytes, 0, v1, v2, v3, v4)
    trig()

    send_cmd(slave_id, cmd_bytes, 1, v1, v2, v3, v4)
    trig()


# =========================
# Status handling
# =========================
def read_status(slave_id):
    trig()

    data = master.slaves[slave_id].input

    actual_position = struct.unpack("<i", data[0:4])[0]
    status = int.from_bytes(data[4:7], "little")

    return {
        "pos": actual_position,

        "enabled": bool((status >> 0) & 1),
        "end_stop": bool((status >> 1) & 1),
        "thermal_protection1": bool((status >> 2) & 1),
        "thermal_protection2": bool((status >> 3) & 1),
        "force_zero": bool((status >> 4) & 1),
        "motor_on": bool((status >> 5) & 1),
        "closed_loop": bool((status >> 6) & 1),
        "encoder_index": bool((status >> 7) & 1),
        "encoder_valid": bool((status >> 8) & 1),
        "searching_index": bool((status >> 9) & 1),
        "position_reached": bool((status >> 10) & 1),
        "error_compensation": bool((status >> 11) & 1),
        "encoder_error": bool((status >> 12) & 1),
        "scanning": bool((status >> 13) & 1),
        "left_end_stop": bool((status >> 14) & 1),
        "right_end_stop": bool((status >> 15) & 1),
        "error_limit": bool((status >> 16) & 1),
        "searching_optimal_freq": bool((status >> 17) & 1),
        "safety_timeout": bool((status >> 18) & 1),
        "ecat_ack": bool((status >> 19) & 1),
        "emergency_stop": bool((status >> 20) & 1),
        "position_fail": bool((status >> 21) & 1),

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


def wait_until(slave_id, condition, timeout=30.0, label="condition"):
    t0 = time.time()

    while time.time() - t0 < timeout:
        st = read_status(slave_id)

        if has_error(st):
            raise RuntimeError(f"Controller error while waiting for {label}: {st}")

        if condition(st):
            return st

        time.sleep(POLL_DT)

    raise TimeoutError(f"Timeout while waiting for {label}. Last status: {st}")


# =========================
# Motion commands
# =========================
def enable(slave_id):
    """
    ENBL=1:
      - enables motor signals
      - also recovers from errors
    """
    command(slave_id, b"ENBL", v1=1)
    wait_until(slave_id, lambda st: st["enabled"], timeout=3.0, label="enabled")
    trig()


def disable(slave_id):
    command(slave_id, b"ENBL", v1=0)


def halt(slave_id):
    """
    Normal stop.
    STOP is more like emergency stop and blocks following commands.
    """
    command(slave_id, b"HALT")
    trig()


def reset(slave_id):
    """
    RSET resets controller and settings to saved values.
    After this, ENBL and INDX are needed again.
    """
    command(slave_id, b"RSET")
    trig()


def find_index(slave_id, direction=0, vel=INDEX_VEL, accel=ACCEL, decel=DECEL):
    """
    direction:
      0 -> descending encoder direction
      1 -> ascending encoder direction
    """
    if direction not in (0, 1):
        raise ValueError("INDX direction must be 0 or 1")

    command(slave_id, b"INDX", v1=direction, v2=vel, v3=accel, v4=decel)

    # INDX の直後は PDO の状態反映に数周期かかることがある。
    # 公式ライブラリと同様に数回更新を待ってから、探索が原点未検出の
    # まま終了していないか確認する。
    for _ in range(3):
        time.sleep(POLL_DT)
        st = read_status(slave_id)

    t0 = time.time()
    while time.time() - t0 < 10.0:
        st = read_status(slave_id) #1.0s

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
        slave_id,
        lambda st: st["position_reached"],
        timeout=5.0,
        label="position_reached after index",
    )

    time.sleep(SETTLE_TIME)
    st = read_status(slave_id)
    print("Index found:", st)
    time.sleep(1)
    return st

def move_abs(slave_id, target_pos, vel=DEFAULT_VEL, accel=ACCEL, decel=DECEL):
    """
    Absolute move by DPOS.
    target_pos is in encoder units.
    """
    st = read_status(slave_id)

    if not st["enabled"]:
        raise RuntimeError("Controller is not enabled. Run enable() first.")

    if not st["encoder_valid"]:
        raise RuntimeError("Index is not found yet. Run find_index() first.")

    command(slave_id, b"DPOS", v1=target_pos, v2=vel, v3=accel, v4=decel)
    time.sleep(0.1)
    # wait_until(
    #     slave_id,
    #     lambda s: (not s["position_reached"]) or s["motor_on"],
    #     timeout=1.0,
    #     label=f"motion started target={target_pos}",
    # )

    st = wait_until(
        slave_id,
        lambda s: s["position_reached"] and not s["motor_on"],
        timeout=5.0,
        label=f"position_reached target={target_pos}",
    )

    time.sleep(SETTLE_TIME)
    st = read_status(slave_id)

    print(f"Target={target_pos}, Actual={st['pos']}, Error={st['pos'] - target_pos}")
    return st

def scan(slave_id, direction, vel=DEFAULT_VEL, accel=ACCEL, decel=DECEL):
    """
    Continuous closed-loop motion.
    direction:
      -1 -> negative direction
       0 -> stop
      +1 -> positive direction
    """
    if direction not in (-1, 0, 1):
        raise ValueError("SCAN direction must be -1, 0, or 1")

    command(slave_id, b"SCAN", v1=direction, v2=vel, v3=accel, v4=decel)


def stop_scan(slave_id):
    scan(slave_id, direction=0)

def set_param(slave_id, param_name, value, delay=0.02):
    if len(param_name) != 4:
        raise ValueError("param_name must be 4 characters, e.g. 'PROP', 'FREQ'")

    cmd_bytes = param_name.encode("ascii")
    command(slave_id, cmd_bytes, v1=value, delay=delay)


def apply_default_settings(slave_id, bpf_id=1, ecat_ack_check=False):
    """Apply the known actuator settings after every controller reset.

    These values were previously duplicated in setup.py, test.py, and all.py.
    Calling this function after reset() avoids relying on RAM-resident settings
    left over from a previous script invocation.
    """
    frequency_settings = {
        1: {
            SLAVE_ID_HPF_1: (172000, 170000),
            SLAVE_ID_HPF_2: (172000, 169000),
            SLAVE_ID_HPF_3: (173000, 170000),
        },
        2: {
            SLAVE_ID_HPF_4: (174000, 171000),
            SLAVE_ID_HPF_5: (172000, 169000),
            SLAVE_ID_HPF_6: (173000, 169000),
        },
    }

    try:
        freq, frq2 = frequency_settings[bpf_id][slave_id]
    except KeyError as exc:
        raise ValueError(
            f"No default settings defined for BPF #{bpf_id}, slave ID {slave_id}"
        ) from exc

    for param_name, value in [
        ("FREQ", freq),
        ("FRQ2", frq2),
        ("ELIM", 0),
        ("TOU2", 10),
        ("TOU3", 0),
        ("ZON1", 100),
        ("ZON2", 1000),
        ("PTOL", 2),
        ("PTO2", 4),
        ("ILIM", 3000),
        ("ACTD", 0),
        ("ENCD", 0),
        ("ENCO", 0),
        ("LLIM", -4000),
        ("HLIM", 4000),
        ("PRO2", 150),
        ("PROP", 350),
        ("INTF", 60),
        ("INDA", 1),
    ]:
        set_param(slave_id, param_name, value)
        if ecat_ack_check:
            print(read_status(slave_id)['ecat_ack'])
    time.sleep(1)


def pdo_settle(cycles=10, dt=0.02):
    """Exchange PDOs for several cycles so the preceding command can settle."""
    for _ in range(cycles):
        trig()
        time.sleep(dt)


def prepare_actuator(slave_id, bpf_id=1):
    """Reset, configure, and enable one actuator for a fresh run.

    Index search is deliberately separate: callers must invoke find_index()
    explicitly when an absolute position reference is required.
    """
    reset(slave_id)
    time.sleep(3)

    apply_default_settings(slave_id, bpf_id=bpf_id)
    pdo_settle(cycles=10, dt=0.02)
    
    enable(slave_id)
    pdo_settle(cycles=5, dt=0.02)


def save_rows_csv(rows, csv_path):
    """
    rows: list of dict
    csv_path: 保存先CSV
    """
    if len(rows) == 0:
        print("No data to save.")
        return

    fieldnames = list(rows[0].keys())

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Saved: {csv_path}  ({len(rows)} samples)")


def record_until(
    slave_id,
    stop_condition,
    csv_path="motion_log.csv",
    target_pos=None,
    poll_dt=0.005,
    timeout=30.0,
):
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

            st = read_status(slave_id)
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
        print(f"Saved figure: {save_path}")

    plt.show()

    return df

def move_abs_plot(
    slave_id,
    target_pos,
    vel=DEFAULT_VEL,
    accel=ACCEL,
    decel=DECEL,
    csv_path=None,
    plot=True,
    plot_save_path=None,
    poll_dt=0.005,
    timeout=30.0,
    smooth_window=5,
    min_record_time=0.05,
    target_tolerance=2,
):
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
    st0 = read_status(slave_id)
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
    send_cmd(slave_id, b"DPOS", 0, target_pos, vel, accel, decel)
    trig()
    time.sleep(0.02)

    send_cmd(slave_id, b"DPOS", 1, target_pos, vel, accel, decel)
    trig()

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
        slave_id=slave_id,
        stop_condition=stop_condition,
        csv_path=csv_path,
        target_pos=target_pos,
        poll_dt=poll_dt,
        timeout=timeout,
    )

    time.sleep(SETTLE_TIME)
    st = read_status(slave_id)

    final_error = st["pos"] - target_pos

    print(
        f"Target={target_pos}, "
        f"Actual={st['pos']}, "
        f"Error={final_error} encoder unit"
    )
    print(f"Saved log: {csv_path}")

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

def dpos(slaveId, target_pos_mm, vel=DEFAULT_VEL, accel=ACCEL, decel=DECEL):
    target_pos_encoder = round(target_pos_mm / RESOLUTION_MM)
    move_abs(slaveId, target_pos_encoder, vel, accel, decel)
    apos_encoder = read_status(slaveId)["pos"]
    apos_mm = encoder_to_mm(apos_encoder)
    print(f"APOS = {apos_mm:.5f} mm")

def calc_pos_mm_from_fcut(f_cut_GHz, A, B, X0):
    if f_cut_GHz == B:
        raise ValueError("f_cut_GHz - Bが0になるため位置を計算できません。")

    return X0 - A / (f_cut_GHz - B)


def calc_bpf_positions(bpf_id, central_freq_GHz, bandwidth_GHz):
    if bpf_id not in (1, 2):
        raise ValueError("bpf_idには1または2を指定してください。")

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

    if bpf_id == 1:
        positions.update({
            "HPF #1 position_mm": calc_pos_mm_from_fcut(
                freq_hpf_GHz, A_HPF_1, B_HPF_1, X0_HPF_1
            ),
            "HPF #2 position_mm": calc_pos_mm_from_fcut(
                freq_lpf_GHz, A_LPF_HPF_2, B_LPF_HPF_2, X0_LPF_HPF_2
            ),
            "HPF #3 position_mm": calc_pos_mm_from_fcut(
                freq_lpf_GHz, A_LPF_HPF_3, B_LPF_HPF_3, X0_LPF_HPF_3
            ),
        })
    else:
        positions.update({
            "HPF #4 position_mm": calc_pos_mm_from_fcut(
                freq_lpf_GHz, A_LPF_HPF_4, B_LPF_HPF_4, X0_LPF_HPF_4
            ),
            "HPF #5 position_mm": calc_pos_mm_from_fcut(
                freq_hpf_GHz, A_HPF_5, B_HPF_5, X0_HPF_5
            ),
            "HPF #6 position_mm": calc_pos_mm_from_fcut(
                freq_lpf_GHz, A_LPF_HPF_6, B_LPF_HPF_6, X0_LPF_HPF_6
            ),
        })

    return positions

def check_bpf_actuator_ready(name, slave_id):
    status = read_status(slave_id)

    if not status["enabled"]:
        raise RuntimeError(f"{name}がenableされていません。")
    if not status["encoder_valid"]:
        raise RuntimeError(f"{name}のindexが見つかっていません。")
    if has_error(status):
        raise RuntimeError(f"{name}にエラーがあります: {status}")

    return status


def halt_bpf(bpf_id):
    if bpf_id == 1:
        actuator_list = [
            ("HPF #1", SLAVE_ID_HPF_1),
            ("HPF #2", SLAVE_ID_HPF_2),
            ("HPF #3", SLAVE_ID_HPF_3),
        ]
    elif bpf_id == 2:
        actuator_list = [
            ("HPF #4", SLAVE_ID_HPF_4),
            ("HPF #5", SLAVE_ID_HPF_5),
            ("HPF #6", SLAVE_ID_HPF_6),
        ]
    else:
        raise ValueError("bpf_idには1または2を指定してください。")

    for name, slave_id in actuator_list:
        try:
            halt(slave_id)
            print(f"✅ {name} halted")
        except Exception as error:
            print(f"🔴 {name} could not be halted: {error}")


def print_bpf_movement_result(name, slave_id, calculated_position_mm, status):
    target_position_mm = round(calculated_position_mm / RESOLUTION_MM) * RESOLUTION_MM
    actual_position_mm = encoder_to_mm(status["pos"])
    error_encoder = (actual_position_mm - calculated_position_mm)/RESOLUTION_MM

    print()
    print(f"🔵 {name} movement result")
    print(f"Slave ID            = {slave_id}")
    print(f"Target position     = {calculated_position_mm:.8f} mm")
    print(f"Input position      = {target_position_mm:.5f} mm")
    print(f"Actual position     = {actual_position_mm:.5f} mm")
    print(f"Actual - Target     = {error_encoder:.1f} eu")
    print("Full status =")
    print(status)
    print(status["status_raw"])

    return {
        "slaveId": slave_id,
        "calculated_position_mm": calculated_position_mm,
        "target_position_mm": target_position_mm,
        "actual_position_mm": actual_position_mm,
        "error_encoder_unit": error_encoder,
        "status": status,
    }


def move_bpf(
    bpf_id,
    central_freq_GHz,
    bandwidth_GHz,
    vel=DEFAULT_VEL,
    accel=ACCEL,
    decel=DECEL,
):
    ## frequency -> positon
    positions = calc_bpf_positions(bpf_id, central_freq_GHz, bandwidth_GHz)

    if bpf_id == 1:
        # 先頭を単体HPF、後ろ2台をLPF側にそろえる。
        actuator_list = [
            ("HPF #1", SLAVE_ID_HPF_1, positions["HPF #1 position_mm"]),
            ("HPF #2", SLAVE_ID_HPF_2, positions["HPF #2 position_mm"]),
            ("HPF #3", SLAVE_ID_HPF_3, positions["HPF #3 position_mm"]),
        ]
    elif bpf_id == 2:
        # 先頭を単体HPF、後ろ2台をLPF側にそろえる。
        # actuator_list[1:] + actuator_list[:1]により、移動順は#4→#6→#5となる。
        actuator_list = [
            ("HPF #5", SLAVE_ID_HPF_5, positions["HPF #5 position_mm"]),
            ("HPF #4", SLAVE_ID_HPF_4, positions["HPF #4 position_mm"]),
            ("HPF #6", SLAVE_ID_HPF_6, positions["HPF #6 position_mm"]),
        ]
    else:
        raise ValueError("bpf_idには1または2を指定してください。")

    ## status check
    if master is None:
        raise RuntimeError("EtherCAT masterが初期化されていません。init(IFNAME)を実行してください。")

    max_slave_id = max(slave_id for _, slave_id, _ in actuator_list)
    if len(master.slaves) <= max_slave_id:
        raise RuntimeError(
            f"slave ID = {max_slave_id}まで必要ですが、"
            f"接続されているslave数は{len(master.slaves)}です。"
        )

    status_before = {}
    for name, slave_id, calculated_position in actuator_list:
        status_before[name] = check_bpf_actuator_ready(name, slave_id)

    ## pre-motion setting check
    print(f"🔵 BPF #{bpf_id} setting")
    print(f"Center frequency = {positions['central_frequency_GHz']:.5f} GHz")
    print(f"Bandwidth        = {positions['bandwidth_GHz']:.5f} GHz")
    print(f"HPF cutoff       = {positions['HPF_cutoff_GHz']:.5f} GHz")
    print(f"LPF cutoff       = {positions['LPF_cutoff_GHz']:.5f} GHz")
    print()

    print("🔵 Actuator movement preview")
    for name, slave_id, calculated_position in actuator_list:
        current_position = encoder_to_mm(status_before[name]["pos"])
        print(
            f"{name} / slave {slave_id}: "
            f"current {current_position:.5f} mm -> target {calculated_position:.5f} mm"
        )

    print()
    answer = input("🟡 The actuators will move to the position described above. OK? ---> [y/N] ")

    if answer != "y":
        print("🔴 Motion canceled")
        return None

    try:
        # LPF側の2台を先にそろえ、その後に単体HPFを移動する。
        for name, slave_id, calculated_position in actuator_list[1:] + actuator_list[:1]:
            dpos(slave_id, calculated_position, vel=vel, accel=accel, decel=decel)
    except Exception:
        print("🔴 移動中にエラーが発生しました。")
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
        status = read_status(slave_id)
        result[name] = print_bpf_movement_result(
            name,
            slave_id,
            calculated_position,
            status,
        )

    return result

def close():
    global master

    if master is None:
        return

    try:
        master.state = pysoem.INIT_STATE
        master.write_state()
    finally:
        master.close()
        master = None

def print_section(title):
    print(f"\n{'═' * 72}\n  {title}\n{'═' * 72}")
