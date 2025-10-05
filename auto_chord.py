#!/usr/bin/env python3
"""
servo_common.py — Expanded max_deg version (2025-10)
-----------------------------------------------------
✓ Uses increased STRING_MAX_DEG for wider travel
✓ Keeps calibrated PWM ranges
✓ 60°/s default speed (overridden by chord scripts)
✓ No startup 'middle' motion
✓ Persists /tmp/servo_state.json
"""

import sys, time, json, os
from typing import Optional

# ==========================================================
# USER SETTINGS
# ==========================================================

SPEED_DEG_PER_SEC = 60.0
STEP_DEG = 3.0

# Calibrated pulse ranges
PULSE_US_180 = (700, 2300)
PULSE_US_270 = (600, 2400)

# Optional per-string overrides
PULSE_OVERRIDE = {
    6: (720, 2320),
    5: (710, 2310),
    4: (700, 2300),
    3: (610, 2380),
    2: (620, 2390),
    1: (620, 2390),
}

# Direction viewed from servo horn side
CW_IS_INCREASE = {6: True, 5: True, 4: True, 3: True, 2: True, 1: True}

# GPIO map
STRING_TO_GPIO = {6: 6, 5: 26, 4: 5, 3: 22, 2: 17, 1: 27}

# ==========================================================
# ⚙️ MAIN CHANGE — expanded mechanical mapping
# ==========================================================
STRING_MAX_DEG = {
    6: 240,  # 180° servos extended
    5: 240,
    4: 180,
    3: 360,  # 270° servos extended
    2: 360,
    1: 360,
}

STRING_PULSE_RANGE = {
    s: PULSE_OVERRIDE.get(
        s, PULSE_US_270 if STRING_MAX_DEG[s] >= 270 else PULSE_US_180
    )
    for s in STRING_TO_GPIO
}

STATE_PATH = "/tmp/servo_state.json"

# ==========================================================
# DRIVER LAYER (pigpio → fallback RPi.GPIO)
# ==========================================================

class ServoDriverBase:
    def setup_pin(self, gpio, pulse_us_initial): raise NotImplementedError
    def write_pulse(self, gpio, pulse_us): raise NotImplementedError
    def read_current_pulse(self, gpio) -> int: return 0
    def cleanup(self): pass

class PigpioDriver(ServoDriverBase):
    def __init__(self):
        import pigpio
        self.pg = pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "pigpio daemon not running. Try: sudo pkill pigpiod; sudo pigpiod"
            )
    def setup_pin(self, gpio, pulse_us_initial):
        self.pi.set_mode(gpio, 1)
        self.pi.set_servo_pulsewidth(gpio, 0)
    def write_pulse(self, gpio, pulse_us):
        self.pi.set_servo_pulsewidth(gpio, int(pulse_us))
    def read_current_pulse(self, gpio) -> int:
        try: return int(self.pi.get_servo_pulsewidth(gpio))
        except Exception: return 0
    def cleanup(self):
        for gpio in STRING_TO_GPIO.values():
            try: self.pi.set_servo_pulsewidth(gpio, 0)
            except Exception: pass
        self.pi.stop()

class RPiGPIODriver(ServoDriverBase):
    def __init__(self):
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        self.GPIO.setmode(GPIO.BCM)
        self.freq = 50.0
        self.period_ms = 1000.0 / self.freq
        self.pwms = {}
    def _us_to_dc(self, us): return (us / 1000.0) / self.period_ms * 100.0
    def setup_pin(self, gpio, pulse_us_initial):
        self.GPIO.setup(gpio, self.GPIO.OUT)
        pwm = self.GPIO.PWM(gpio, self.freq)
        pwm.start(0)
        self.pwms[gpio] = pwm
    def write_pulse(self, gpio, pulse_us):
        self.pwms[gpio].ChangeDutyCycle(self._us_to_dc(pulse_us))
    def cleanup(self):
        for pwm in self.pwms.values(): pwm.stop()
        self.GPIO.cleanup()

def get_driver():
    try: return PigpioDriver()
    except Exception as e:
        print(f"[info] pigpio unavailable ({e}); falling back to RPi.GPIO.", file=sys.stderr)
        return RPiGPIODriver()

# ==========================================================
# STATE HANDLING
# ==========================================================

def _load_state():
    try:
        if os.path.exists(STATE_PATH):
            with open(STATE_PATH, "r") as f:
                return json.load(f)
    except Exception: pass
    return {}

def _save_state(state):
    try:
        with open(STATE_PATH, "w") as f: json.dump(state, f)
    except Exception as e:
        print(f"[warn] Failed saving state: {e}", file=sys.stderr)

# ==========================================================
# SERVO MODEL
# ==========================================================

class RelativeServo:
    def __init__(self, driver, string_num):
        self.string = string_num
        self.gpio = STRING_TO_GPIO[string_num]
        self.max_deg = float(STRING_MAX_DEG[string_num])
        self.pulse_min, self.pulse_max = map(float, STRING_PULSE_RANGE[string_num])
        self.cw_is_increase = CW_IS_INCREASE[string_num]
        self.pulse_per_deg = (self.pulse_max - self.pulse_min) / self.max_deg
        self.drv = driver
        driver.setup_pin(self.gpio, 0)

        state = _load_state()
        saved = state.get(f"s{self.string}")
        self.current_pulse: Optional[float] = None
        if isinstance(saved, (int, float)) and self.pulse_min <= saved <= self.pulse_max:
            self.current_pulse = float(saved)
        else:
            try:
                cur = self.drv.read_current_pulse(self.gpio)
                if cur and self.pulse_min <= cur <= self.pulse_max:
                    self.current_pulse = float(cur)
            except Exception: pass

    def _safe_start(self, sign: float) -> float:
        margin = (self.pulse_max - self.pulse_min) * 0.05
        return (self.pulse_min + margin) if sign > 0 else (self.pulse_max - margin)

    def rotate(self, degrees, clockwise=True,
               speed_deg_per_sec=SPEED_DEG_PER_SEC, step_deg=STEP_DEG):
        degrees = float(degrees)
        if degrees <= 0: return
        inc_for_cw = 1.0 if self.cw_is_increase else -1.0
        direction_sign = inc_for_cw if clockwise else -inc_for_cw
        if self.current_pulse is None:
            self.current_pulse = self._safe_start(direction_sign)

        remaining = degrees
        per_step = min(step_deg, degrees)
        sleep_s = 0 if speed_deg_per_sec <= 0 else per_step / float(speed_deg_per_sec)

        while remaining > 1e-6:
            step = min(per_step, remaining)
            self.current_pulse += self.pulse_per_deg * step * direction_sign
            self.current_pulse = max(self.pulse_min, min(self.current_pulse, self.pulse_max))
            self.drv.write_pulse(self.gpio, self.current_pulse)
            remaining -= step
            if sleep_s > 0: time.sleep(sleep_s)

        state = _load_state()
        state[f"s{self.string}"] = round(self.current_pulse, 2)
        _save_state(state)

def run_single_move(string_num, degrees, clockwise):
    drv = get_driver()
    try:
        s = RelativeServo(drv, string_num)
        s.rotate(degrees, clockwise=clockwise)
    finally:
        drv.cleanup()
