
import math

from pico_encoder import QuadratureIRQEncoder
from speed_control import WheelSpeedControl


class EncoderAdapter:
    """
    Connect an IRQ encoder to the speed controller.

    Converts encoder counts into distance and speed.
    """

    def __init__(self, encoder, diameter_mm=70,
                 counts_per_rev=700):

        if diameter_mm <= 0 or counts_per_rev <= 0:
            raise ValueError("Invalid wheel measurements")

        self.encoder = encoder
        self.mm_per_count = (
            math.pi * diameter_mm / counts_per_rev
        )
        self.last_count = encoder.read_count()

    def get_distance(self):
        return self.encoder.read_count() * self.mm_per_count

    def get_speed(self, dt_ms):
        if dt_ms <= 0:
            raise ValueError("Time must be positive")

        current_count = self.encoder.read_count()
        change = current_count - self.last_count
        self.last_count = current_count

        return change * self.mm_per_count * 1000 / dt_ms

    def reset(self):
        self.encoder.reset()
        self.last_count = self.encoder.read_count()


# Test the integration without physical hardware
if __name__ == "__main__":

    class FakePin:
        IN = 0
        IRQ_RISING = 1
        IRQ_FALLING = 2
        pins = {}

        def __init__(self, number, mode):
            self.state = 0
            self.handler = None
            FakePin.pins[number] = self

        def value(self):
            return self.state

        def irq(self, trigger, handler):
            self.handler = handler

        def drive(self, value):
            if value != self.state:
                self.state = value
                self.handler(self)

    motor_commands = {}

    def fake_motor(wheel, power):
        motor_commands[wheel] = power

    controller = WheelSpeedControl(fake_motor)

    # Test-only pin identifiers, NOT actual GPIO assignments
    pin_pairs = {
        1: (101, 102),
        2: (103, 104),
        3: (105, 106)
    }

    # Connect Pico-style encoders to all three wheels
    for wheel, (pin_a, pin_b) in pin_pairs.items():

        hardware_encoder = QuadratureIRQEncoder(
            pin_a,
            pin_b,
            pin_class=FakePin,
            disable_irq=lambda: 0,
            enable_irq=lambda state: None
        )

        controller.encoders[wheel] = EncoderAdapter(
            hardware_encoder
        )

    print("Testing Pico encoder + PID integration...")

    # Simulate a quadrature encoder sequence
    def rotate(pin_a, pin_b, sequence):
        for a, b in sequence:
            FakePin.pins[pin_a].drive(a)
            FakePin.pins[pin_b].drive(b)

    forward = [
        (0, 1), (1, 1), (1, 0), (0, 0)
    ]

    backward = [
        (1, 0), (1, 1), (0, 1), (0, 0)
    ]

    rotate(101, 102, forward)
    rotate(103, 104, backward)

    assert controller.get_wheel_distance(1) > 0
    assert controller.get_wheel_distance(2) < 0
    assert controller.get_wheel_distance(3) == 0

    print("PASS: Three independent IRQ encoders")

    # Send target wheel speeds
    controller.set_wheel_speed(1, 100)
    controller.set_wheel_speed(2, -80)
    controller.set_wheel_speed(3, 0)

    # Run one 20 ms PID update
    controller.update(0.02)

    assert motor_commands[1] > 0
    assert motor_commands[2] < 0
    assert motor_commands[3] == 0

    print("PASS: PID responds to encoder readings")
    print("PASS: Motor directions correct")

    # Reset all counts
    controller.reset_counts()

    for wheel in range(1, 4):
        assert controller.get_wheel_distance(wheel) == 0

    print("PASS: Reset all three encoder counts")
    print("Pico encoder + PID integration passed!")
