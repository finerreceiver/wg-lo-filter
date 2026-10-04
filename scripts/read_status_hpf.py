import scripts.utils as u
import argparse
import time

# 2026-10-04: 既存の待ち時間を用途別に命名。秒数・呼び出し順は維持。
# DISPLAYはログを読む間隔、WAIT/AFTERは処理後の既存待ち時間。
ENABLE_STATUS_WAIT_S = 0.5
CLOSE_DISPLAY_PAUSE_S = 0.5

if __name__ == "__main__":

    IFNAME = "eth0"

    parser = argparse.ArgumentParser()
    parser.add_argument("--slaveID", type=int, required=True)
    args = parser.parse_args()

    SLAVE_ID = args.slaveID
    print()
    u.init(IFNAME)
    try:
        u.enable(SLAVE_ID)
        time.sleep(ENABLE_STATUS_WAIT_S)
        print(u.read_status(SLAVE_ID))
        position_mm = u.encoder_to_mm(u.read_status(SLAVE_ID)['pos'])
        print(f"Position: {position_mm} [mm]")
    finally:
        u.close()
        time.sleep(CLOSE_DISPLAY_PAUSE_S)
        print("✅ Master has been closed")


           
  
