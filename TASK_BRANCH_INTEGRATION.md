# Task 1, 4, and 5 integration notes

This branch combines the current Task 1 motor driver, Task 4 RFID prototype,
and Task 5 JSON/UDP link. The current GitHub repo has no Task 2 or Task 3
branch, so this is a software integration checkpoint, not a complete robot
motion stack.

## Interfaces and dependencies

| Task | Present interface | Still needed |
|---|---|---|
| 1: motors | `set_motor(wheel, power)` accepts wheels 1-3 and signed open-loop power from -1.0 to 1.0; `stop_all()` coasts all three. | Task 2 must provide encoder/speed control and call the motor API. PWM power is not measured wheel speed. |
| 4: RFID | `read_pair()` returns left/right `(uid, gain)` samples; position logic exposes `current_tile()`, `center_on_card()`, and bounded `search_for_card()`. | A real RC522 UID/anticollision callback and Task 3 motion adapter implementing blocking `move_relative(right_mm, forward_mm)`, `get_pose()`, `set_pose(x_mm, y_mm, heading)`, and `stop()`. Heading units and board origin/orientation must be agreed with Task 3. |
| 5: Wi-Fi | Laptop and Pico exchange protocol version 1 JSON over UDP. Commands carry a lowercase algebraic square and robot ID; the Pico calls `on_goto(tile)` and reports `moving`, then `arrived` or `lost`. | Task 3 must supply a blocking tile-to-motion callback and return literal `False` on movement failure. The new JSON path is not wired into the existing `PathPlanning/PythonServer.py` / ESP-01 byte protocol. |

Task 4 uses robot-relative millimeters: positive `right_mm` and `forward_mm`.
Its world pose uses `(x_mm, y_mm, heading)`. The existing task does not define
heading units or board axes, so no adapter should silently guess them. Task 5
uses squares `a1` through `h8`; the Task 3 callback owns square-to-coordinate
conversion.

The current laptop `PythonServer` and the Task 5 `LaptopLink` both bind UDP
port 12345 by default, but speak different formats: the existing path sends
`WIZ`-prefixed byte commands to ESP-01s, while Task 5 sends JSON directly to a
Pico 2 W. They cannot run together on the same laptop with their default
settings; use one path at a time until a server adapter or migration is
implemented.

## GPIO resource map

The new Task 1 and Task 4 maps are disjoint:

| Component | GPIOs |
|---|---|
| Task 1 TB6612 motor driver | GP2-GP10 |
| Task 4 shared RC522 SPI | GP18 (SCK), GP19 (MOSI), GP16 (MISO), GP20 (left CS), GP17 (right CS), GP22 (reset) |
| Task 5 Wi-Fi | Pico 2 W internal WLAN; no user GPIO assignment |

The legacy Arduino sketches in `ControlCode/` use a different L293D map:
motor outputs/enables GP13-GP16 and GP18-GP19, encoder inputs GP10 and GP20, UART GP0/GP1,
and button GP28. This conflicts with the new work if those sketches or their
wiring are used at the same time: Task 1 GP10 overlaps the old encoder input;
Task 4 GP16/18/19 overlaps old motor outputs/enables and GP20 overlaps an old
encoder. Do not deploy the legacy sketch and the new MicroPython task stack
simultaneously. Task 2/3 pin assignments are not present, so check them against
this map before adding their hardware.

These are pin-map findings from source and documentation, not confirmation of
the assembled robot's wiring. Verify board revision, motor driver, and actual
wiring with power disconnected before connecting both RFID readers and motors.

## Offline checks

From the repository root:

```sh
python3 ControlCode/motor_driver/simulate_motor_driver.py
python3 -m unittest discover -s tests -v
```

The motor simulator and RFID tests use fakes. `test_wifi_link.py` runs the
laptop and Pico protocol implementations over localhost UDP with a fake WLAN
and a stub Task 3 callback. These checks verify software message/callback
behavior only; they do not exercise an access point, RC522 readers, motors,
encoders, or real motion.

## Hardware smoke-test sequence

1. With power disconnected, identify the exact Pico 2 W, TB6612FNG, and RC522
   wiring. Keep the new map separate from the legacy sketch's pin map. Confirm
   the motor's startup/stall current is compatible with the selected driver;
   the Task 1 README flags a possible 7 A motor versus roughly 1.2 A/channel
   TB6612FNG mismatch. Do not proceed to motor power until that is resolved.
2. With motors disconnected, run the two-reader RFID bring-up. Verify reader
   versions, antenna alternation, UID callback, gain ordering, and repeatable
   detection at known offsets.
3. With the Pico and laptop on the same hotspot, verify `hello` registration,
   send a `goto` to a stub callback, and confirm `moving` then `arrived`. Return
   literal `False` and raise an exception in separate trials; both should
   report `lost`. Confirm `battery_low` reaches the laptop.
4. Only after current limits are approved, test one wheel at a time with wheels
   raised, a current-limited supply, a physical disconnect, and the test
   script's default 30% level. Verify direction and stop behavior. Do not
   raise the test to 60% or 100% until the motor/driver combination is cleared.
5. After Tasks 2/3 and their adapters are available, test stop/timeout behavior
   with wheels raised, then a low-speed move to a clear square. Verify the
   Task 4 coordinate/sign contract and Task 5 `arrived` only after the blocking
   move finishes. Record hardware results separately from offline test output.
