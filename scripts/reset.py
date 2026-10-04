import scripts.utils as u
from scripts.logging_utils import add_logging_arguments, configure_logging, get_logger

logger = get_logger("reset")
import argparse
import time

# 2026-10-04: 既存の待ち時間を用途別に命名。秒数・呼び出し順は維持。
# DISPLAYはログを読む間隔、WAIT/AFTERは処理後の既存待ち時間。
RESET_STATUS_WAIT_S = 0.5
CLOSE_DISPLAY_PAUSE_S = 0.5

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--slaveID", type=int, required=True)
    add_logging_arguments(parser)
    args = parser.parse_args()
    configure_logging(args.log_level)

    SLAVE_ID = args.slaveID

    IFNAME = "eth0"

    u.init(IFNAME)
    try:
        u.reset(SLAVE_ID)

        time.sleep(RESET_STATUS_WAIT_S)
        
        logger.info(u.read_status(SLAVE_ID))
    finally:
        u.close()
        time.sleep(CLOSE_DISPLAY_PAUSE_S)
