#!/usr/bin/env python3
from servo_common import get_driver, RelativeServo, drive_to_stop
import threading, time

SPEED = 120.0
STEP_DEG = 5.0
STAGGER = 0.05
HOLD = 5

def run():
    drv = get_driver()
    try:
        s5, s6 = RelativeServo(drv, 5), RelativeServo(drv, 6)
        print("\n=== G CHORD: 5 & 6 → CCW end-stops (no reset here) ===")
        threads = []
        for s in (s5, s6):
            t = threading.Thread(target=drive_to_stop, args=(s, True, SPEED, STEP_DEG))
            t.start(); threads.append(t); time.sleep(STAGGER)
        for t in threads: t.join()
        time.sleep(HOLD)
    finally:
        drv.cleanup()

if __name__ == "__main__":
    run()
