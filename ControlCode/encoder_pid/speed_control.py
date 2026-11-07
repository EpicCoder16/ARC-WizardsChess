
from encoder import WheelEncoder
from pid_controller import PIDController


class WheelSpeedControl:
    def __init__(self, motor_output):
        # motor_output is a function that sets motor power
        self.motor_output = motor_output

        # One encoder and PID controller per wheel
        self.encoders = {}
        self.pids = {}
        self.targets = {}

        for wheel in range(1, 4):
            self.encoders[wheel] = WheelEncoder()
            self.pids[wheel] = PIDController(
                kp=0.005,
                ki=0.04
            )
            self.targets[wheel] = 0.0

    def check_wheel(self, wheel):
        if wheel not in self.encoders:
            raise ValueError("Wheel must be 1, 2, or 3")

    def set_wheel_speed(self, wheel, mm_per_s):
        self.check_wheel(wheel)
        self.targets[wheel] = mm_per_s

    def get_wheel_distance(self, wheel):
        self.check_wheel(wheel)
        return self.encoders[wheel].get_distance()

    def encoder_transition(self, wheel, a, b):
        self.check_wheel(wheel)
        self.encoders[wheel].update(a, b)

    def reset_counts(self):
        for encoder in self.encoders.values():
            encoder.reset()

    def update(self, dt=0.02):
        # Called every 20 ms = 50 times per second
        if dt <= 0:
            raise ValueError("Time must be positive")

        for wheel in range(1, 4):
            encoder = self.encoders[wheel]
            pid = self.pids[wheel]

            actual_speed = encoder.get_speed(dt * 1000)
            target_speed = self.targets[wheel]

            power = pid.update(
                target_speed,
                actual_speed,
                dt
            )

            self.motor_output(wheel, power)


# Offline test using a fake motor driver
if __name__ == "__main__":
    motor_commands = {}

    def fake_motor(wheel, power):
        motor_commands[wheel] = power

    controller = WheelSpeedControl(fake_motor)

    controller.set_wheel_speed(1, 100)
    controller.set_wheel_speed(2, -80)
    controller.set_wheel_speed(3, 0)

    controller.update()

    assert round(motor_commands[1], 2) == 0.58
    assert round(motor_commands[2], 2) == -0.46
    assert motor_commands[3] == 0.0

    print("PASS: Three independent wheel controllers")

    # Simulate one forward encoder cycle on wheel 1
    for a, b in [
        (0, 0), (0, 1), (1, 1),
        (1, 0), (0, 0)
    ]:
        controller.encoder_transition(1, a, b)

    assert controller.get_wheel_distance(1) > 0
    assert controller.get_wheel_distance(2) == 0

    print("PASS: Independent encoder tracking")

    controller.reset_counts()

    for wheel in range(1, 4):
        assert controller.get_wheel_distance(wheel) == 0

    print("PASS: Reset all encoder counts")
    print("All three-wheel controller tests passed!")
