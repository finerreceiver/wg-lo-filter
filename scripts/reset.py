import scripts.utils as u
import argparse
import time

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--slaveID", type=int, required=True)
    args = parser.parse_args()

    SLAVE_ID = args.slaveID

    IFNAME = "eth0"

    u.init(IFNAME)
    try:
        u.reset(SLAVE_ID)

        time.sleep(0.5)
        
        print(u.read_status(SLAVE_ID))
    finally:
        u.close()
        time.sleep(0.5)