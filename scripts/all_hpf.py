import scripts.utils as u
import argparse
import time

# 2026-10-04: 既存の待ち時間を用途別に命名。秒数・呼び出し順は維持。
# DISPLAYはログを読む間隔、WAIT/AFTERは処理後の既存待ち時間。
AFTER_INIT_PAUSE_S = 1
STATUS_DISPLAY_PAUSE_S = 1
AFTER_MOTION_PAUSE_S = 2
STEP_DISPLAY_PAUSE_S = 0.5
HALT_STATUS_WAIT_S = 0.5

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--slaveID", type=int, required=True)
    parser.add_argument("--dpos_mm", type=float, required=True)
    args = parser.parse_args()

    SLAVE_ID = args.slaveID
    BPF_ID = u.get_bpf_id_for_slave(SLAVE_ID)
    dpos_mm = args.dpos_mm

    IFNAME = "eth0"

    ## Initializing
    u.init(IFNAME)
    time.sleep(AFTER_INIT_PAUSE_S)
    print("✅ Controller has been initialized")

    try:

        u.prepare_actuator(SLAVE_ID, bpf_id=BPF_ID) # コントローラーをresetしてから、制御パラメーターを送り、enableする
        print("✅ Parameters applied and controller enabled")

        ## Read status(1)
        print("✅ Status before index search")
        print(u.read_status(SLAVE_ID))
        time.sleep(STATUS_DISPLAY_PAUSE_S)

        ## Index search
        print("▶️ Index search started")
        u.find_index(SLAVE_ID, direction=0)
        print("✅ Index has been found")


        ## Read status(2)
        print("✅ Status before motion")
        print(u.read_status(SLAVE_ID))
        time.sleep(STATUS_DISPLAY_PAUSE_S)

        # DPOS
        print("▶️ Motion started")
        u.dpos(SLAVE_ID, target_pos_mm=dpos_mm)
        time.sleep(AFTER_MOTION_PAUSE_S)
        print("✅ DPOS has been done")
        time.sleep(STEP_DISPLAY_PAUSE_S)
        

        ## Read status(3)
        print("✅ Status after motion")
        print(u.read_status(SLAVE_ID))

    except TimeoutError:
        print("⚠️ Timeout detected. Sending HALT to the actuator.")
        try:
            ## errorが出ているので、motorの停止信号を送る
            u.halt(SLAVE_ID)
            time.sleep(HALT_STATUS_WAIT_S)
            print("✅ Status after HALT")
            print(u.read_status(SLAVE_ID))
        except Exception as halt_error:
            print(f"⚠️ HALT command or status read failed: {halt_error}")
        raise
    finally:
        ## masterのclose処理
        u.close()
        print("✅ Master has been closed")
