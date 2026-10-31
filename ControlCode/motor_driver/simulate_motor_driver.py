"""Offline simulated hardware checks for Wizard Chess Task 1.

Run on a laptop with Python 3: python3 simulate_motor_driver.py
This substitutes fake GPIO and PWM devices for the Pico's machine module.
It does NOT test electrical wiring, actual motors, driver temperature, or current.
"""

import sys
import types


class FakePin:
    OUT = 1

    def __init__(self, number, mode=None, value=0):
        self.number = number
        self.state = value

    def value(self, new_value=None):
        if new_value is not None:
            self.state = new_value
        return self.state


class FakePWM:
    def __init__(self, pin):
        self.pin = pin
        self.frequency = None
        self.duty = None

    def freq(self, hz):
        self.frequency = hz

    def duty_u16(self, duty):
        assert 0 <= duty <= 65535, "PWM duty out of range"
        self.duty = duty


fake_machine = types.ModuleType("machine")
fake_machine.Pin = FakePin
fake_machine.PWM = FakePWM
sys.modules["machine"] = fake_machine

from motor_driver import REVERSED, _motors, set_motor, stop_all  # noqa: E402


def verify(wheel, duty, in1, in2):
    pwm, first, second = _motors[wheel]
    assert pwm.duty == duty, f"Wheel {wheel}: unexpected duty {pwm.duty}"
    assert first.value() == in1 and second.value() == in2, (
        f"Wheel {wheel}: unexpected direction pins"
    )


def main():
    expected_pins = {1: (2, 3, 4), 2: (5, 6, 7), 3: (8, 9, 10)}
    for wheel, pins in expected_pins.items():
        pwm, a, b = _motors[wheel]
        assert (pwm.pin.number, a.number, b.number) == pins
        assert pwm.frequency == 20_000
        verify(wheel, 0, 0, 0)
    print("PASS: All wheel pins, 20 kHz PWM, and initial stop state")

    set_motor(1, 0.30)
    verify(1, int(0.30 * 65535), 1, 0)
    print("PASS: Wheel 1 forward at 30%")

    set_motor(2, -0.60)
    verify(2, int(0.60 * 65535), 0, 1)
    print("PASS: Wheel 2 backward at 60%")

    set_motor(3, 1.0)
    verify(3, 65535, 1, 0)
    print("PASS: Wheel 3 forward at 100%")

    set_motor(1, -1.0)
    verify(1, 65535, 0, 1)
    set_motor(1, 0.0)
    verify(1, 0, 0, 0)
    print("PASS: Direction changes and zero-power coast")

    REVERSED[2] = True
    set_motor(2, 0.50)
    verify(2, int(0.50 * 65535), 0, 1)
    REVERSED[2] = False
    print("PASS: Per-wheel reverse flag")

    stop_all()
    for wheel in expected_pins:
        verify(wheel, 0, 0, 0)
    print("PASS: stop_all() coasts all three wheels")

    for wheel, power in ((4, 0.5), (1, 1.2), (1, -1.2), (1, "fast")):
        try:
            set_motor(wheel, power)
        except ValueError:
            continue
        raise AssertionError(f"Expected ValueError for {wheel=}, {power=}")
    print("PASS: Invalid input rejected")

    print("\nAll offline tests passed. Real hardware testing still required.")


if __name__ == "__main__":
    main()
