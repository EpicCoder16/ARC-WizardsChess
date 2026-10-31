"""Wizard Chess Task 1: three-wheel TB6612FNG motor driver.

Runs on Raspberry Pi Pico 2 W using MicroPython.

Interface:
    set_motor(wheel, power)  # wheel 1..3, power -1.0..1.0
    stop_all()               # coast all motors

NOTE: set_motor() is intentionally non-blocking so the future PID loop can
call it at 50 Hz. Ramp motor commands in the caller (see test_motors.py).
"""

from machine import Pin, PWM

PWM_FREQUENCY_HZ = 20_000
MAX_DUTY = 65_535

# Set an entry to True if positive power makes that wheel run backward.
REVERSED = {
    1: False,
    2: False,
    3: False,
}

# wheel: (PWM, IN1, IN2), using the handoff's current pin map.
_PIN_MAP = {
    1: (2, 3, 4),
    2: (5, 6, 7),
    3: (8, 9, 10),
}

_motors = {}
_directions = {}

for _wheel, (_pwm_pin, _in1_pin, _in2_pin) in _PIN_MAP.items():
    _in1 = Pin(_in1_pin, Pin.OUT, value=0)
    _in2 = Pin(_in2_pin, Pin.OUT, value=0)
    _pwm = PWM(Pin(_pwm_pin))
    _pwm.freq(PWM_FREQUENCY_HZ)
    _pwm.duty_u16(0)
    _motors[_wheel] = (_pwm, _in1, _in2)
    _directions[_wheel] = 0


def set_motor(wheel, power):
    """Apply signed PWM power to one wheel, with no deliberate delay.

    wheel is 1, 2, or 3. power is between -1.0 and +1.0.
    Zero power coasts the wheel. This is OPEN-LOOP POWER, not wheel speed.
    """
    if wheel not in _motors:
        raise ValueError("wheel must be 1, 2, or 3")
    if not isinstance(power, (int, float)) or not (-1.0 <= power <= 1.0):
        raise ValueError("power must be a number from -1.0 to +1.0")

    pwm, in1, in2 = _motors[wheel]
    if REVERSED[wheel]:
        power = -power

    direction = 1 if power > 0 else (-1 if power < 0 else 0)

    if direction == 0:
        pwm.duty_u16(0)
        in1.value(0)
        in2.value(0)
    else:
        # Remove power before switching direction, but don't interrupt PWM
        # unnecessarily when a PID loop updates power in the same direction.
        if direction != _directions[wheel]:
            pwm.duty_u16(0)
            in1.value(1 if direction > 0 else 0)
            in2.value(0 if direction > 0 else 1)
        pwm.duty_u16(int(abs(power) * MAX_DUTY))

    _directions[wheel] = direction


def stop_all():
    """Immediately command all motors to coast (not active braking)."""
    for wheel in _motors:
        pwm, in1, in2 = _motors[wheel]
        pwm.duty_u16(0)
        in1.value(0)
        in2.value(0)
        _directions[wheel] = 0
