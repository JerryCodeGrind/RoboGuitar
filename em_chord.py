#!/usr/bin/env python3
from servo_common import get_driver, RelativeServo, drive_to_stop
import threading, time

SPEED = 120.0
STEP_DEG = 5.0
HOLD = 5

def run():
    drv = get_driver()
    try:
        s1 = RelativeServo(drv, 1)
        print("\n=== E MINOR: 1 → CW end-stop ===")
        t = threading.Thread(target=drive_to_stop, args=(s1, False, SPEED, STEP_DEG))
        t.start(); t.join()
        time.sleep(HOLD)
    finally:
        drv.cleanup()

if __name__ == "__main__":
    run()
