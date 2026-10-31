# Wizard Chess – Task 1 Motor Driver

**Status:** Implemented, not hardware-verified. Do not mark complete until tested.

## Platform
- Raspberry Pi Pico 2 W running MicroPython
- Two TB6612FNG dual motor drivers, using three total channels
- PWM frequency 20 kHz, duty cycle 0–65535
- STBY on each board tied to 3.3 V (per handoff)

## GPIO assignments

| Wheel | Driver channel | PWM | IN1 | IN2 |
|---|---|---|---|---|
| 1 | Driver 1 A | GP2 | GP3 | GP4 |
| 2 | Driver 1 B | GP5 | GP6 | GP7 |
| 3 | Driver 2 A | GP8 | GP9 | GP10 |

Other connections: driver VCC to 3.3 V logic; driver VM to appropriately rated motor supply; all grounds common. Never connect battery/5 V to a Pico GPIO. Follow the exact driver breakout's labels.

## Public API (`motor_driver.py`)
- `set_motor(wheel, power)` – wheel 1, 2, or 3; signed open-loop power -1.0 to +1.0. `0` coasts. Raises ValueError on invalid input.
- `stop_all()` – commands every wheel to coast immediately.
- `REVERSED[wheel] = True` – set per-wheel software polarity to fix motor orientation.

The `set_motor` API is non-blocking so the PID task can update it at ~50 Hz. Ramping is performed by the caller; `test_motors.py` demonstrates a gradual ramp.

## How to run in Thonny
1. Flash a Pico **2 W-compatible MicroPython** firmware; set Thonny's interpreter to MicroPython (Raspberry Pi Pico).
2. Put `motor_driver.py` and `test_motors.py` on the Pico, in the same root directory.
3. Confirm the motor/driver combination and safe current limits with Sanyam **before applying motor power**.
4. Lift the robot so wheels can freely rotate. If possible, use a current-limited bench supply and an accessible physical power disconnect.
5. Open `test_motors.py`, run it, and type `RUN` only if the hardware has been approved.
6. By default the sequence tests each wheel at ±30%, with 600 ms ramp-up, 2 seconds at target power, and 600 ms ramp-down. After current/driver suitability is confirmed, change `POWER_LEVELS` to `(0.30, 0.60, 1.00)` to test ±30%, ±60%, ±100% as the handoff requests.
7. Record observed direction and smoothness; measure current and check driver temperature between tests. If positive power drives a wheel backward, set its `REVERSED` flag to `True` and rerun.
8. Press Ctrl+C to interrupt; `finally` calls `stop_all()`, but software is **not** an emergency stop. Keep a physical disconnect nearby.

## Hardware warning
DFRobot's proposed FIT0186 motor lists a 7 A stall current. The TB6612FNG is about 1.2 A/channel average (with brief, limited peaks). Thus the proposed motor can overload the driver under startup/stall/load. Ramping alone does not guarantee protection. Validate the motor, supply, peak/stall current and driver selection with Sanyam; do not automatically run 100% tests.

## Acceptance checklist
- [ ] Each of the 3 wheels moves in both directions
- [ ] Positive power is the intended direction; software reverse flags verified
- [ ] 30%, 60%, 100% tests run for 2 s **after** safety approval
- [ ] Startup and slowdown are gradual during test
- [ ] Driver current/temperature remain within safe hardware limits
- [ ] `stop_all()` stops all motors (coast)
- [ ] Hardware tests documented and committed to the shared repo

## Integration note
This task controls electrical power, not wheel speed. The encoder/PID module will use `set_motor()` to adjust drive power; motion planning/odometry belongs to other tasks.
