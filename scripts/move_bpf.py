import scripts.utils as u
import argparse
import time
import math

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--bpf_ID", type=int, choices=[1, 2], required=True)
    parser.add_argument("--central_freq", type=float, required=True)
    parser.add_argument("--band_width", type=float, required=True)
    args = parser.parse_args()

    BPF_ID = args.bpf_ID
    CENTRAL_FREQ_GHZ = args.central_freq
    BANDWIDTH_GHZ = args.band_width

    u.print_section("BPF configuration")
    print(f"  BPF ID     : #{BPF_ID}")
    print(f"  central frequency : {CENTRAL_FREQ_GHZ:.3f} GHz")
    print(f"  bandwidth         : {BANDWIDTH_GHZ:.3f} GHz")
    print()
    time.sleep(3)

    IFNAME = "eth0"

    u.init(IFNAME)
    try:
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


