#!/usr/bin/env python3
"""
servo_common.py — clean/quiet revamp
- RPi.GPIO as default driver (pigpio optional, silent fallback)
- No startup 'middle'
- Persistent state /tmp/servo_state.json
- True end-stop helpers for CW/CCW
- String max-deg scaling kept (1,2,3→360; 5,6→240) for angle math if ever used
"""

import os, sys, time, json
from typing import Optional

# =================== USER CONFIG ===================

# SPEED used only if caller doesn't pass one
SPEED_DEG_PER_SEC = 60.0
STEP_DEG = 3.0
STATE_PATH = "/tmp/servo_state.json"

# GPIO map (after your 22/17 swap): 3→22, 2→17
STRING_TO_GPIO = {6: 6, 5: 26, 4: 5, 3: 22, 2: 17, 1: 27}

# Direction sense (viewed from the horn/outside). Flip any single value if a string moves the wrong way.
CW_IS_INCREASE = {6: True, 5: True, 4: True, 3: True, 2: True, 1: True}

# Expanded mechanical mapping so 240°/360° spans do more work if you use rotate()
STRING_MAX_DEG = {6: 240, 5: 240, 4: 180, 3: 360, 2: 360, 1: 360}

# Safe default pulse ranges; override per string if needed
PULSE_US_180 = (700, 2300)
PULSE_US_270 = (600, 2400)
PULSE_OVERRIDE = {
    # 180° class
    6: (720, 2320),
    5: (710, 2310),
    4: (700, 2300),
    # 270° class
    3: (610, 2380),
    2: (620, 2390),
    1: (620, 2390),
}

# ===================================================

def _load_state():
    try:
        if os.path.exists(STATE_PATH):
            with open(STATE_PATH, "r") as f:
                return json.load(f)
    except Exception:
        pass
    return {}

def _save_state(state):
    try:
        with open(STATE_PATH, "w") as f:
            json.dump(state, f)
    except Exception:
        pass

# Build pulse ranges respecting overrides and class
STRING_PULSE_RANGE = {}
for s in STRING_TO_GPIO:
    default = PULSE_US_270 if STRING_MAX_DEG[s] >= 270 else PULSE_US_180
    STRING_PULSE_RANGE[s] = PULSE_OVERRIDE.get(s, default)

# =================== DRIVERS ===================

class ServoDriverBase:
    def setup_pin(self, gpio): raise NotImplementedError
    def write_pulse(self, gpio, pulse_us): raise NotImplementedError
    def read_current_pulse(self, gpio) -> int: return 0
    def cleanup(self): pass

class RPiGPIODriver(ServoDriverBase):
    def __init__(self):
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        self.GPIO.setmode(GPIO.BCM)
        self.freq = 50.0
        self.period_ms = 1000.0 / self.freq
        self.pwms = {}
    def _us_to_dc(self, us):  # duty cycle % for 50 Hz
        return (us / 1000.0) / self.period_ms * 100.0
    def setup_pin(self, gpio):
        self.GPIO.setup(gpio, self.GPIO.OUT)
        pwm = self.GPIO.PWM(gpio, self.freq)
        pwm.start(0.0)  # no signal until commanded
        self.pwms[gpio] = pwm
    def write_pulse(self, gpio, pulse_us):
        dc = 0.0 if pulse_us <= 0 else self._us_to_dc(pulse_us)
        self.pwms[gpio].ChangeDutyCycle(dc)
    def cleanup(self):
        for pwm in self.pwms.values(): pwm.stop()
        self.GPIO.cleanup()

class PigpioDriver(ServoDriverBase):
    def __init__(self):
        import pigpio
        self.pi = pigpio.pi()  # do NOT print if fail; caller will fallback
        if not self.pi.connected:
            raise RuntimeError("pigpio not connected")
    def setup_pin(self, gpio):
        self.pi.set_mode(gpio, 1)
        self.pi.set_servo_pulsewidth(gpio, 0)
    def write_pulse(self, gpio, pulse_us):
        self.pi.set_servo_pulsewidth(gpio, int(pulse_us))
    def read_current_pulse(self, gpio) -> int:
        try: return int(self.pi.get_servo_pulsewidth(gpio))
        except Exception: return 0
    def cleanup(self):
        try:
            for gpio in STRING_TO_GPIO.values():
                self.pi.set_servo_pulsewidth(gpio, 0)
            self.pi.stop()
        except Exception:
            pass

def get_driver():
    # Default to RPi.GPIO for quiet reliability; allow opting into pigpio via env
    prefer = os.getenv("SERVO_DRIVER", "RPIGPIO").upper()
    if prefer == "PIGPIO":
        try: return PigpioDriver()
        except Exception: pass
    # fallback to RPi.GPIO
    try: return RPiGPIODriver()
    except Exception as e:
        sys.stderr.write(f"[fatal] No GPIO driver available: {e}\n")
        raise

# =================== MODEL ===================

class RelativeServo:
    def __init__(self, driver, string_num):
        self.string = int(string_num)
        self.gpio = int(STRING_TO_GPIO[self.string])
        self.max_deg = float(STRING_MAX_DEG[self.string])
        self.pulse_min, self.pulse_max = map(float, STRING_PULSE_RANGE[self.string])
        self.cw_is_increase = bool(CW_IS_INCREASE[self.string])
        self.pulse_per_deg = (self.pulse_max - self.pulse_min) / self.max_deg if self.max_deg > 0 else 10.0
        self.drv = driver
        self.drv.setup_pin(self.gpio)

        self.current_pulse: Optional[float] = None
        st = _load_state()
        saved = st.get(f"s{self.string}")
        if isinstance(saved, (int, float)) and self.pulse_min <= float(saved) <= self.pulse_max:
            self.current_pulse = float(saved)

    # Low-level pulse stepper (used by rotate and end-stop helpers)
    def _step_to(self, target_pulse, speed_deg_per_sec=SPEED_DEG_PER_SEC, step_deg=STEP_DEG):
        # initialize if unknown (no middle jump; pick safe side)
        if self.current_pulse is None:
            margin = 0.05 * (self.pulse_max - self.pulse_min)
            # start away from target to ensure motion
            self.current_pulse = self.pulse_min + margin if target_pulse > (self.pulse_min + self.pulse_max)/2 else self.pulse_max - margin

        pulse_per_deg = self.pulse_per_deg if self.pulse_per_deg != 0 else 10.0
        total_deg = abs((target_pulse - self.current_pulse) / pulse_per_deg)
        if total_deg < 1e-6:
            return
        per_step = max(1.0, min(step_deg, total_deg))
        sleep_s = 0 if speed_deg_per_sec <= 0 else per_step / float(speed_deg_per_sec)
        sign = 1.0 if target_pulse > self.current_pulse else -1.0

        remaining = total_deg
        while remaining > 1e-6:
            step = min(per_step, remaining)
            self.current_pulse += sign * step * pulse_per_deg
            if self.current_pulse < self.pulse_min: self.current_pulse = self.pulse_min
            if self.current_pulse > self.pulse_max: self.current_pulse = self.pulse_max
            self.drv.write_pulse(self.gpio, self.current_pulse)
            remaining -= step
            if sleep_s > 0: time.sleep(sleep_s)

        # snap to exact target, persist, then leave PWM running (caller may cut)
        self.current_pulse = float(max(self.pulse_min, min(target_pulse, self.pulse_max)))
        self.drv.write_pulse(self.gpio, self.current_pulse)
        st = _load_state(); st[f"s{self.string}"] = round(self.current_pulse, 2); _save_state(st)

    # Angle API (kept for compatibility)
    def rotate(self, degrees, clockwise=True, speed_deg_per_sec=SPEED_DEG_PER_SEC, step_deg=STEP_DEG):
        inc_for_cw = 1.0 if self.cw_is_increase else -1.0
        sign = inc_for_cw if clockwise else -inc_for_cw
        delta = degrees * sign * self.pulse_per_deg
        target = (self.current_pulse if self.current_pulse is not None else (self.pulse_min + self.pulse_max)/2) + delta
        self._step_to(target, speed_deg_per_sec, step_deg)

# =================== END-STOP HELPERS ===================

def target_pulse_for_direction(s: RelativeServo, ccw: bool) -> float:
    # If CW increases pulse, then CCW wants pulse_min; else pulse_max
    if ccw:
        return s.pulse_min if s.cw_is_increase else s.pulse_max
    else:
        return s.pulse_max if s.cw_is_increase else s.pulse_min

def drive_to_stop(s: RelativeServo, ccw: bool, speed=SPEED_DEG_PER_SEC, step_deg=STEP_DEG):
    tgt = target_pulse_for_direction(s, ccw)
    s._step_to(tgt, speed, step_deg)
    # cut PWM so it doesn't buzz at the stop
    s.drv.write_pulse(s.gpio, 0)
