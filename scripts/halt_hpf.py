import scripts.utils as u
from scripts.logging_utils import add_logging_arguments, configure_logging, get_logger

logger = get_logger("halt_hpf")
import argparse
import time

if __name__ == "__main__":

    IFNAME = "eth0"

    parser = argparse.ArgumentParser()
    parser.add_argument("--slaveID", type=int, required=True)
    add_logging_arguments(parser)
    args = parser.parse_args()
    configure_logging(args.log_level)

    SLAVE_ID = args.slaveID
    u.init(IFNAME)

    try:
        logger.info("▶️ We'll halt the actuator")
        logger.info("📊 Status before HALT")
        logger.debug(u.read_status(SLAVE_ID))

        u.halt(SLAVE_ID)

        logger.info("📊 Status after HALT")
        logger.debug(u.read_status(SLAVE_ID))
        # motor_on bit を読み込んでHALTが成功したか判定したいね

    finally:
        u.close()

           
  
