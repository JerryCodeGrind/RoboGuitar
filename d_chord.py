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
        s1, s2, s6, s5 = RelativeServo(drv,1), RelativeServo(drv,2), RelativeServo(drv,6), RelativeServo(drv,5)

        print("\n=== D CHORD: 1 & 2 → CCW end-stops ===")
        t1 = threading.Thread(target=drive_to_stop, args=(s1, True, SPEED, STEP_DEG))
        t2 = threading.Thread(target=drive_to_stop, args=(s2, True, SPEED, STEP_DEG))
        t1.start(); time.sleep(STAGGER); t2.start(); t1.join(); t2.join()
        time.sleep(HOLD)

        print("↩ Reset from G: 6 & 5 → CW end-stops")
        t3 = threading.Thread(target=drive_to_stop, args=(s6, False, SPEED, STEP_DEG))
        t4 = threading.Thread(target=drive_to_stop, args=(s5, False, SPEED, STEP_DEG))
        t3.start(); time.sleep(STAGGER); t4.start(); t3.join(); t4.join()

    finally:
        drv.cleanup()

if __name__ == "__main__":
    run()
