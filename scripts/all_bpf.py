import scripts.utils as u
import argparse
import time

# 2026-10-04: 既存の待ち時間を用途別に命名。秒数・呼び出し順は維持。
# DISPLAYはログを読む間隔、WAIT/AFTERは処理後の既存待ち時間。
CONFIG_DISPLAY_PAUSE_S = 3
SECTION_DISPLAY_PAUSE_S = 3
STEP_DISPLAY_PAUSE_S = 2
AFTER_PREPARATION_PAUSE_S = 2
AFTER_INDEX_PAUSE_S = 2
AFTER_MOTION_PAUSE_S = 2

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--bpf_ID", type=int, choices=u.BPF_IDS, required=True)
    parser.add_argument("--central_freq", type=float, required=True)
    parser.add_argument("--band_width", type=float, required=True)
    parser.add_argument("--prepare", action="store_true", help="指定すると、移動前にパラメーター設定とindex探索を実行します。")
    args = parser.parse_args()

    BPF_ID = args.bpf_ID
    CENTRAL_FREQ_GHZ = args.central_freq
    BANDWIDTH_GHZ = args.band_width
    PREPARE = args.prepare

    u.print_section("BPF configuration")
    print(f"  BPF ID     : #{BPF_ID}")
    print(f"  central frequency : {CENTRAL_FREQ_GHZ:.3f} GHz")
    print(f"  bandwidth         : {BANDWIDTH_GHZ:.3f} GHz")
    print()
    time.sleep(CONFIG_DISPLAY_PAUSE_S)

    IFNAME = "eth0"

    u.init(IFNAME)
    try:
        slave_list = u.get_bpf_slave_ids(BPF_ID)

        if PREPARE:
            for slave_id in slave_list:
                u.print_section(f"Slave ID {slave_id}: pre-motion setup")
                time.sleep(SECTION_DISPLAY_PAUSE_S)

                ## Parameter setting
                print(f"▶️  [Slave ID = {slave_id}]: Now setting parameter...")
                u.prepare_actuator(slave_id, bpf_id=BPF_ID)
                print(f"✅ [Slave ID = {slave_id}]: Parameters applied and controller enabled")
                time.sleep(AFTER_PREPARATION_PAUSE_S)

                ## Read status(1)
                print(f"📊 [Slave ID = {slave_id}]: Status before index search")
                time.sleep(STEP_DISPLAY_PAUSE_S)
                print(u.read_status(slave_id))
                time.sleep(STEP_DISPLAY_PAUSE_S)

                ## Index search
                print(f"▶️ [Slave ID = {slave_id}]: Index search started")
                time.sleep(STEP_DISPLAY_PAUSE_S)
                u.find_index(slave_id, direction=0)
                print(f"✅ [Slave ID = {slave_id}]: Index has been found")
                time.sleep(AFTER_INDEX_PAUSE_S)

                ## Read status(2)
                print(f"📊 [Slave ID = {slave_id}]: Status before motion")
                time.sleep(STEP_DISPLAY_PAUSE_S)
                print(u.read_status(slave_id))
                time.sleep(STEP_DISPLAY_PAUSE_S)

                print(f"✅ [Slave ID = {slave_id}]: Pre-motion settinfg completed")
                time.sleep(STEP_DISPLAY_PAUSE_S)

        else:
            print("✅ We'll skip the actuator preparation")

        u.print_section(f"BPF motion")
        time.sleep(STEP_DISPLAY_PAUSE_S)
        print("▶️  BPF making started")
        result = u.move_bpf(BPF_ID, CENTRAL_FREQ_GHZ, BANDWIDTH_GHZ)
        if result == None:
            time.sleep(STEP_DISPLAY_PAUSE_S)
            pass
        else:
            time.sleep(AFTER_MOTION_PAUSE_S)
            print(f"✅ BPF#{BPF_ID} has been made")
            time.sleep(STEP_DISPLAY_PAUSE_S)

    finally:
        ## masterのclose処理
        u.close()
        print("✅ Master has been closed")
