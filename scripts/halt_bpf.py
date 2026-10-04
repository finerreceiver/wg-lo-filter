import scripts.utils as u
import argparse
import time

# 2026-10-04: 既存の待ち時間を用途別に命名。秒数・呼び出し順は維持。
# DISPLAYはログを読む間隔、WAIT/AFTERは処理後の既存待ち時間。
CLOSE_DISPLAY_PAUSE_S = 0.2

if __name__ == "__main__":

    IFNAME = "eth0"

    parser = argparse.ArgumentParser()
    parser.add_argument("--bpf_ID", type=int, choices=u.BPF_IDS, required=True)
    args = parser.parse_args()

    BPF_ID = args.bpf_ID
    u.init(IFNAME)

    try:
        slave_list = u.get_bpf_slave_ids(BPF_ID)

        print("▶️ We'll halt the actuators")
        print("📊 Status before HALT")
        for SLAVE_ID in slave_list:
            print(f"📊 [Slave ID = {SLAVE_ID}]: Status before halt")
            print(u.read_status(SLAVE_ID))

        u.halt_bpf(BPF_ID)

        for SLAVE_ID in slave_list:
            print(f"📊 [Slave ID = {SLAVE_ID}]: Status after halt")
            print(u.read_status(SLAVE_ID))
        
        # motor_on bit を読み込んでHALTが成功したか判定したいね

    finally:
        u.close()
        time.sleep(CLOSE_DISPLAY_PAUSE_S)

           
  
