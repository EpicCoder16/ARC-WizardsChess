"""CAUTION: Moves physical motors. Use only after hardware/current approval.

Start with one wheel lifted off the floor and a current-limited supply.
After confirming the hardware/driver rating, add 0.60 and 1.00 below.
"""

import time
from motor_driver import set_motor, stop_all

# Safe-to-review default: ONLY 30% tests; not permission to power the motor.
# After Sanyam approves the motor/driver combination and current limits,
# use (0.30, 0.60, 1.00) for the full handoff acceptance test.
POWER_LEVELS = (0.30,)
WHEELS = (1, 2, 3)
HOLD_MS = 2000
RAMP_STEPS = 30
RAMP_STEP_MS = 20


def ramp_motor(wheel, start, end):
    """Gradually change power over 600 ms to avoid a sudden command jump."""
    for step in range(1, RAMP_STEPS + 1):
        fraction = step / RAMP_STEPS
        power = start + (end - start) * fraction
        set_motor(wheel, power)
        time.sleep_ms(RAMP_STEP_MS)


def main():
    stop_all()
    print("WARNING: Motors may rotate. Verify hardware current limits first.")
    response = input("Type RUN only if approved and area is clear: ")
    if response != "RUN":
        print("Test cancelled; motors stopped.")
        return

    try:
        for wheel in WHEELS:
            for direction in (1, -1):
                for level in POWER_LEVELS:
                    power = direction * level
                    print("Wheel", wheel, "target power", power)
                    ramp_motor(wheel, 0.0, power)
                    time.sleep_ms(HOLD_MS)
                    ramp_motor(wheel, power, 0.0)
                    stop_all()
                    time.sleep_ms(500)
        print("Test sequence complete. Inspect driver temperature/current.")
    finally:
        stop_all()


if __name__ == "__main__":
    main()
