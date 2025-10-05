#!/usr/bin/env python3
from servo_common import get_driver, RelativeServo, drive_to_stop
import threading, time

SPEED = 120.0
STEP_DEG = 5.0
HOLD = 5

def run():
    drv = get_driver()
    try:
        s2 = RelativeServo(drv, 2)
        print("\n=== C CHORD: 2 → CCW end-stop ===")
        t = threading.Thread(target=drive_to_stop, args=(s2, True, SPEED, STEP_DEG))
        t.start(); t.join()
        time.sleep(HOLD)

        print("↩ Reset C: 2 → CW end-stop")
        t = threading.Thread(target=drive_to_stop, args=(s2, False, SPEED, STEP_DEG))
        t.start(); t.join()

    finally:
        drv.cleanup()

if __name__ == "__main__":
    run()
