import scripts.utils as u
import argparse
import time

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
        time.sleep(0.5)
        print(u.read_status(SLAVE_ID))
        position_mm = u.encoder_to_mm(u.read_status(SLAVE_ID)['pos'])
        print(f"Position: {position_mm} [mm]")
    finally:
        u.close()
        time.sleep(0.5)
        print("✅ Master has been closed")


           
  

