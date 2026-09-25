import scripts.utils as u
import argparse
import time
import math

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--bpf_ID", type=int, choices=[1, 2], required=True)
    args = parser.parse_args()

    BPF_ID = args.bpf_ID

    u.print_section("BPF configuration")
    print(f"  BPF ID     : #{BPF_ID}")
    time.sleep(3)

    IFNAME = "eth0"

    u.init(IFNAME)
    try:
        if BPF_ID == 1:
            slave_list = [u.SLAVE_ID_HPF_1, u.SLAVE_ID_HPF_2, u.SLAVE_ID_HPF_3]
        if BPF_ID == 2:
            slave_list = [u.SLAVE_ID_HPF_4, u.SLAVE_ID_HPF_5, u.SLAVE_ID_HPF_6]

        for slave_id in slave_list:
            u.print_section(f"Slave ID {slave_id}: pre-motion setup")
            time.sleep(3)

            ## Parameter setting
            print(f"▶️  [Slave ID = {slave_id}]: Now setting parameter...")
            u.prepare_actuator(slave_id, bpf_id=BPF_ID)
            print(f"✅ [Slave ID = {slave_id}]: Parameters applied and controller enabled")
            time.sleep(2)

            ## Read status(1)
            print(f"📊 [Slave ID = {slave_id}]: Status before index search")
            time.sleep(2)
            print(u.read_status(slave_id))
            time.sleep(2)

            ## Index search
            print(f"▶️ [Slave ID = {slave_id}]: Index search started")
            time.sleep(2)
            u.find_index(slave_id, direction=0)
            print(f"✅ [Slave ID = {slave_id}]: Index has been found")
            time.sleep(2)

            ## Read status(2)
            print(f"📊 [Slave ID = {slave_id}]: Status before motion")
            time.sleep(2)
            print(u.read_status(slave_id))
            time.sleep(2)

            print(f"✅ [Slave ID = {slave_id}]: Pre-motion settinfg completed")
            time.sleep(2)

        print(f"✅ BPF#{BPF_ID} setup completed!")
        
    finally:
        ## masterのclose処理
        u.close()
        time.sleep(1.5)
        print("✅ Master has been closed")


