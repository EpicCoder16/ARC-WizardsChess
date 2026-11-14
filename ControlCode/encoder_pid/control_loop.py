
import time
from speed_control import WheelSpeedControl


def current_time_ms():
    # Works on both MicroPython and regular Python
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()

    return int(time.monotonic() * 1000)


def elapsed_ms(now, previous):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, previous)

    return now - previous


class ControlLoop:
    def __init__(self, controller, clock=None):
        self.controller = controller
        self.clock = clock or current_time_ms

        self.interval_ms = 20  # 50 Hz
        self.last_update = None
        self.update_count = 0

    def step(self):
        now = self.clock()

        if self.last_update is None:
            self.last_update = now
            return False

        elapsed = elapsed_ms(now, self.last_update)

        # Only update when at least 20 ms have passed
        if elapsed < self.interval_ms:
            return False

        dt = elapsed / 1000.0

        # Update all three wheel controllers
        self.controller.update(dt)

        self.last_update = now
        self.update_count += 1

        return True


# Offline test without hardware
if __name__ == "__main__":

    motor_commands = []

    def fake_motor(wheel, power):
        motor_commands.append((wheel, power))

    controller = WheelSpeedControl(fake_motor)

    controller.set_wheel_speed(1, 100)
    controller.set_wheel_speed(2, -80)
    controller.set_wheel_speed(3, 0)

    # Fake clock starts at zero milliseconds
    fake_time = [0]

    loop = ControlLoop(
        controller,
        clock=lambda: fake_time[0]
    )

    loop.step()

    # Simulate one second, advancing 1 ms at a time
    for i in range(1000):
        fake_time[0] += 1
        loop.step()

    assert loop.update_count == 50

    print("PASS: 50 PID updates in 1 simulated second")

    # Each update should control all three motors
    assert len(motor_commands) == 150

    print("PASS: All three wheels updated")
    print("PASS: 150 total motor commands")
    print("50 Hz control loop simulation passed!")
