import scripts.utils as u
import argparse
import time

if __name__ == "__main__":

    IFNAME = "eth0"

    parser = argparse.ArgumentParser()
    parser.add_argument("--slaveID", type=int, required=True)
    args = parser.parse_args()

    SLAVE_ID = args.slaveID
    u.init(IFNAME)

    try:
        print("▶️ We'll halt the actuator")
        print("📊 Status before HALT")
        print(u.read_status(SLAVE_ID))

        u.halt(SLAVE_ID)

        print("📊 Status after HALT")
        print(u.read_status(SLAVE_ID))
        # motor_on bit を読み込んでHALTが成功したか判定したいね

    finally:
        u.close()

           
  

