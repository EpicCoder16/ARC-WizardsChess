
import math


class WheelEncoder:
    # One quadrature transition counts as one tick.
    # Positive sequence: 00 -> 01 -> 11 -> 10 -> 00
    STEPS = {
        (0, 1): 1, (1, 3): 1,
        (3, 2): 1, (2, 0): 1,
        (1, 0): -1, (3, 1): -1,
        (2, 3): -1, (0, 2): -1
    }

    def __init__(self, wheel_diameter_mm=70,
                 counts_per_rev=700, reverse=False):
        if wheel_diameter_mm <= 0 or counts_per_rev <= 0:
            raise ValueError("Wheel measurements must be positive")

        # Temporary values until Sanyam confirms hardware specs
        self.mm_per_count = (
            math.pi * wheel_diameter_mm / counts_per_rev
        )
        self.reverse = reverse
        self.count = 0
        self.last_count = 0
        self.last_state = None
        self.speed = 0.0

    def update(self, a, b):
        # Read encoder channels A and B
        state = (int(a) << 1) | int(b)

        if self.last_state is not None:
            change = self.STEPS.get(
                (self.last_state, state), 0
            )

            if self.reverse:
                change = -change

            self.count += change

        self.last_state = state

    def get_distance(self):
        # Signed distance in millimeters
        return self.count * self.mm_per_count

    def get_speed(self, dt_ms):
        # Calculate speed over a sampling interval
        if dt_ms <= 0:
            raise ValueError("Time interval must be positive")

        change = self.count - self.last_count
        self.speed = (
            change * self.mm_per_count * 1000 / dt_ms
        )
        self.last_count = self.count
        return self.speed

    def reset(self):
        self.count = 0
        self.last_count = 0
        self.speed = 0.0


# Simple test without hardware
if __name__ == "__main__":
    encoder = WheelEncoder()

    # Simulate one forward quadrature cycle
    encoder.update(0, 0)
    encoder.update(0, 1)
    encoder.update(1, 1)
    encoder.update(1, 0)
    encoder.update(0, 0)

    print("Encoder counts:", encoder.count)
    print("Distance (mm):", round(encoder.get_distance(), 2))
    print("Speed (mm/s):", round(encoder.get_speed(20), 2))
