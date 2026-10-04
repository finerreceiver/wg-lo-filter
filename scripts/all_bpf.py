import scripts.utils as u
import argparse
import time

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
    time.sleep(3)

    IFNAME = "eth0"

    u.init(IFNAME)
    try:
        slave_list = u.get_bpf_slave_ids(BPF_ID)

        if PREPARE:
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

        else:
            print("✅ We'll skip the actuator preparation")

        u.print_section(f"BPF motion")
        time.sleep(2)
        print("▶️  BPF making started")
        result = u.move_bpf(BPF_ID, CENTRAL_FREQ_GHZ, BANDWIDTH_GHZ)
        if result == None:
            time.sleep(2)
            pass
        else:
            time.sleep(2)
            print(f"✅ BPF#{BPF_ID} has been made")
            time.sleep(2)

    finally:
        ## masterのclose処理
        u.close()
        print("✅ Master has been closed")
