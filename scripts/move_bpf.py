import scripts.utils as u
from scripts.logging_utils import add_logging_arguments, configure_logging, get_logger

logger = get_logger("move_bpf")
import argparse
import time
import math

# 2026-10-04: 既存の待ち時間を用途別に命名。秒数・呼び出し順は維持。
# DISPLAYはログを読む間隔、WAIT/AFTERは処理後の既存待ち時間。
CONFIG_DISPLAY_PAUSE_S = 3
STEP_DISPLAY_PAUSE_S = 2
AFTER_MOTION_PAUSE_S = 2

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--bpf_ID", type=int, choices=u.BPF_IDS, required=True)
    parser.add_argument("--central_freq", type=float, required=True)
    parser.add_argument("--band_width", type=float, required=True)
    add_logging_arguments(parser)
    args = parser.parse_args()
    configure_logging(args.log_level)

    BPF_ID = args.bpf_ID
    CENTRAL_FREQ_GHZ = args.central_freq
    BANDWIDTH_GHZ = args.band_width

    u.print_section("BPF configuration")
    logger.info(f"  BPF ID     : #{BPF_ID}")
    logger.info(f"  central frequency : {CENTRAL_FREQ_GHZ:.3f} GHz")
    logger.info(f"  bandwidth         : {BANDWIDTH_GHZ:.3f} GHz")
    logger.debug("")
    time.sleep(CONFIG_DISPLAY_PAUSE_S)

    IFNAME = "eth0"

    with u.ethercat_master(IFNAME) as master:
        u.print_section(f"BPF motion")
        time.sleep(STEP_DISPLAY_PAUSE_S)
        logger.info("▶️  BPF making started")
        result = u.move_bpf(master, BPF_ID, CENTRAL_FREQ_GHZ, BANDWIDTH_GHZ)
        if result == None:
            time.sleep(STEP_DISPLAY_PAUSE_S)
            pass
        else:
            time.sleep(AFTER_MOTION_PAUSE_S)
            logger.info(f"✅ BPF#{BPF_ID} has been made")
            time.sleep(STEP_DISPLAY_PAUSE_S)
