import scripts.utils as u
import argparse
import time

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--slaveID", type=int, required=True)
    parser.add_argument("--dpos_mm", type=float, required=True)
    args = parser.parse_args()

    SLAVE_ID = args.slaveID
    dpos_mm = args.dpos_mm

    IFNAME = "eth0"

    ## Initializing
    u.init(IFNAME)
    time.sleep(1)
    print("✅ Controller has been initialized")

    try:

        u.prepare_actuator(SLAVE_ID) # コントローラーをresetしてから、制御パラメーターを送り、enableする
        print("✅ Parameters applied and controller enabled")

        ## Read status(1)
        print("✅ Status before index search")
        print(u.read_status(SLAVE_ID))
        time.sleep(1)

        ## Index search
        print("▶️ Index search started")
        u.find_index(SLAVE_ID, direction=0)
        print("✅ Index has been found")


        ## Read status(2)
        print("✅ Status before motion")
        print(u.read_status(SLAVE_ID))
        time.sleep(1)

        # DPOS
        print("▶️ Motion started")
        u.dpos(SLAVE_ID, target_pos_mm=dpos_mm)
        time.sleep(2)
        print("✅ DPOS has been done")
        time.sleep(0.5)
        

        ## Read status(3)
        print("✅ Status after motion")
        print(u.read_status(SLAVE_ID))

    except TimeoutError:
        print("⚠️ Timeout detected. Sending HALT to the actuator.")
        try:
            ## errorが出ているので、motorの停止信号を送る
            u.halt(SLAVE_ID)
            time.sleep(0.5)
            print("✅ Status after HALT")
            print(u.read_status(SLAVE_ID))
        except Exception as halt_error:
            print(f"⚠️ HALT command or status read failed: {halt_error}")
        raise
    finally:
        ## masterのclose処理
        u.close()
        print("✅ Master has been closed")
