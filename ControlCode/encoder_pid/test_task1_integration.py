
import sys
import types
from pathlib import Path

from speed_control import WheelSpeedControl


# Fake Raspberry Pi Pico GPIO pin
class FakePin:
    OUT = 1

    def __init__(self, pin, mode=None, value=0):
        self.number = pin
        self.state = value

    def value(self, new_value=None):
        if new_value is not None:
            self.state = new_value
        return self.state


# Fake Pico PWM controller
class FakePWM:
    def __init__(self, pin):
        self.pin = pin.number
        self.frequency = 0
        self.duty = 0

    def freq(self, value):
        self.frequency = value

    def duty_u16(self, value):
        self.duty = value


# Pretend that the Pico's machine module exists
fake_machine = types.ModuleType("machine")
fake_machine.Pin = FakePin
fake_machine.PWM = FakePWM
sys.modules["machine"] = fake_machine



# Read Task 1's motor driver from the repository
repo = Path(__file__).resolve().parents[2]

motor_driver_path = (
    repo / "ControlCode" / "motor_driver" / "motor_driver.py"
)

source = motor_driver_path.read_text()

motor_driver = types.ModuleType("motor_driver")
exec(compile(source, "motor_driver.py", "exec"),
     motor_driver.__dict__)


# Connect Task 2 PID to Task 1 motor output
controller = WheelSpeedControl(motor_driver.set_motor)

controller.set_wheel_speed(1, 100)
controller.set_wheel_speed(2, -80)
controller.set_wheel_speed(3, 0)

controller.update(0.02)

print("Testing Task 1 + Task 2 integration...")

# Check the actual Task 1 GPIO/PWM outputs
for wheel, expected_direction in [
    (1, (1, 0)),
    (2, (0, 1)),
    (3, (0, 0))
]:
    pwm, in1, in2 = motor_driver._motors[wheel]

    assert pwm.frequency == 20000
    assert (in1.value(), in2.value()) == expected_direction

    if wheel == 3:
        assert pwm.duty == 0
    else:
        assert pwm.duty > 0

    print("PASS: Wheel", wheel, "motor output")

motor_driver.stop_all()

for wheel in range(1, 4):
    pwm, in1, in2 = motor_driver._motors[wheel]
    assert pwm.duty == 0
    assert in1.value() == 0
    assert in2.value() == 0

print("PASS: All motors stopped")
print("Task 1 + Task 2 offline integration passed!")
