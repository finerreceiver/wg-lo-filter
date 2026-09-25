import scripts.utils as u
import argparse
import time

if __name__ == "__main__":

    IFNAME = "eth0"

    parser = argparse.ArgumentParser()
    parser.add_argument("--bpf_ID", type=int, choices=[1, 2], required=True)
    args = parser.parse_args()

    BPF_ID = args.bpf_ID
    u.init(IFNAME)

    try:
        if BPF_ID == 1:
            slave_list = [u.SLAVE_ID_HPF_1, u.SLAVE_ID_HPF_2, u.SLAVE_ID_HPF_3]
        if BPF_ID == 2:
            slave_list = [u.SLAVE_ID_HPF_4, u.SLAVE_ID_HPF_5, u.SLAVE_ID_HPF_6]

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
        time.sleep(0.2)

           
  

