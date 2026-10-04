import scripts.utils as u
from scripts.logging_utils import add_logging_arguments, configure_logging, get_logger

logger = get_logger("halt_bpf")
import argparse
import time

# 2026-10-04: 既存の待ち時間を用途別に命名。秒数・呼び出し順は維持。
# DISPLAYはログを読む間隔、WAIT/AFTERは処理後の既存待ち時間。
CLOSE_DISPLAY_PAUSE_S = 0.2

if __name__ == "__main__":

    IFNAME = "eth0"

    parser = argparse.ArgumentParser()
    parser.add_argument("--bpf_ID", type=int, choices=u.BPF_IDS, required=True)
    add_logging_arguments(parser)
    args = parser.parse_args()
    configure_logging(args.log_level)

    BPF_ID = args.bpf_ID
    try:
        with u.ethercat_master(IFNAME) as master:

            slave_list = u.get_bpf_slave_ids(BPF_ID)

            logger.info("▶️ We'll halt the actuators")
            logger.info("📊 Status before HALT")
            for SLAVE_ID in slave_list:
                logger.info(f"📊 [Slave ID = {SLAVE_ID}]: Status before halt")
                logger.debug(u.read_status(master, SLAVE_ID))

            u.halt_bpf(master, BPF_ID)

            for SLAVE_ID in slave_list:
                logger.info(f"📊 [Slave ID = {SLAVE_ID}]: Status after halt")
                logger.debug(u.read_status(master, SLAVE_ID))
    finally:
        time.sleep(CLOSE_DISPLAY_PAUSE_S)

           
  
