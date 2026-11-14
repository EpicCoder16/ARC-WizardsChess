# Task 3: kiwi drive and odometry

This module implements configurable three-wheel kinematics, encoder-based odometry and a blocking point-to-point controller. It is **simulation-tested only**. Real robot geometry, Task 2 encoder/PID implementation and calibration, Task 4 UID reading, calibration and hardware tests are still required. It is not a deployable robot program by itself.

## Frames and units

Body x is robot-right, body y is robot-forward. Board x/y are millimeters, heading is radians counterclockwise; heading zero aligns the robot's body axes to the board axes. Positive rotation is counterclockwise. Wheel angles give each wheel's location around the robot center in radians counterclockwise from body +x. The assumed drive direction is tangent counterclockwise to that radius: at angle 0, positive wheel travel drives body +y. Configure or invert each encoder and motor sign in the Task 2 adapter so measured and commanded positive travel agree. Wheel indices 1, 2, 3 must match those angles and radii.

`wheel_radii_mm` is the **distance from robot center to each wheel's contact point**, not wheel radius. Task 2 converts encoder counts to signed millimeters using the measured wheel diameter and counts per revolution. Confirm all geometry, coordinate origin, signs and units with the team. The actual wheel angles and distances have not been measured here.

The confirmed wheel diameter is **58 mm**, recorded in `config_example.py`. One wheel revolution travels `pi * 58`, approximately **182.2 mm**, before allowing for slip. Task 2 must use the actual encoder counts per wheel revolution and its chosen decoding mode when converting counts to distance. All remaining unmeasured geometry and motion limits are `None` placeholders; they must be replaced with measured or validated values before constructing a controller. The example configuration is not loaded automatically and cannot move the robot.

## Interface

Construct `KiwiDrive(wheels, wheel_angles_rad, wheel_radii_mm, max_wheel_speed_mm_s, max_move_mm, speed_mm_s, acceleration_mm_s2, tolerance_mm, timeout_ms, control_period_ms=20)`. All configuration is required; values in the tests are simulated examples, not real settings. Construction reads wheel distances but does not command motion.

- `wheel_speeds(vx, vy, omega)` converts body-frame velocities in mm/s and rad/s to wheel speeds in mm/s. `chassis_motion(values)` performs the inverse for either wheel speeds or wheel distances.
- `drive(vx, vy, omega)` sends wheel speed targets, scaling all together if needed to respect the configured wheel limit. It does not schedule the PID loop or stop automatically: the caller must service Task 2 and stop it.
- `get_pose()` updates odometry from signed cumulative wheel distances and returns `(x_mm, y_mm, heading_rad)`. Call frequently, including when external code drives the robot. `set_pose(x, y, heading)` corrects pose and rebases encoder readings without resetting the physical encoders.
- `move_to(x_mm, y_mm)` is a blocking, feedback-controlled board-frame move with a speed ramp, move-distance limit and timeout. It holds the starting heading with proportional encoder feedback (`heading_gain`, default 2.0 per second). It does not avoid obstacles. It returns `True` inside the configured position tolerance and stops on success or exception. Slippage, sensor errors, scheduling delays, motor lag and asymmetric acceleration can still cause overshoot or heading drift. Tune on the actual robot.
- `move_relative(right_mm, forward_mm)` converts a robot-frame offset into a board target and calls `move_to`; it supplies the movement API expected by Task 4. Configure position tolerance smaller than the smallest commanded RFID centering step, or those steps will be treated as already complete. The RFID offset direction/signs still need real testing.
- `stop()` zeros all three targets and calls the adapter's immediate stop, attempting all wheels even when a command fails. This is software only; use a physical disconnect as well.

`move_to` assumes the robot is initially stationary and Task 2's PID loop controls speed in both directions. Target updates are bounded by `max_wheel_speed_mm_s`, but `drive` is a low-level interface: callers must ramp their own commands. Normal translation commands ramp down to zero inside tolerance; faults stop immediately. Motor lag and wheel saturation can still change the physical acceleration. Hardware protection, watchdogs and an independent emergency stop remain necessary.

## Task 2 adapter contract

Provide an object with:

```
get_wheel_distance(wheel)        # 1..3, signed cumulative mm
set_wheel_speed(wheel, mm_per_s)  # signed target, non-blocking
service()                        # advance PID / sample encoders, non-blocking
stop()                           # immediately stop motor outputs and PID targets
```

`wheel_adapter.WheelAdapter(controller, stop_all)` supplies these methods for Task 2's `WheelSpeedControl`: it schedules `update(dt)` every 20 ms using actual elapsed time, resets PID state and zeroes targets on stop, and always calls Task 1's immediate `stop_all`. Call `service()` frequently for low-level drive use. The adapter requires `controller.pids` whose values expose `reset()`. No fake motor driver is installed in this module. Do not reset encoder counts behind the controller's back; rebase with `set_pose` if counts are reset. Ensure `get_wheel_distance` is coherent across three wheels and encoder counts cannot silently wrap or go stale. Configure a task-specific watchdog in the adapter: the time limit here cannot guarantee that motors stop if a blocking hardware call or the interpreter hangs.

The Task 4 RFID class accepts this object's `move_relative`, `get_pose`, `set_pose` and `stop` methods. The Task 5 `on_goto(tile)` callback still needs a **verified tile-to-coordinate map**, then can call `move_to`. Do not report arrival merely from an estimated pose if RFID confirmation is required; agree on the success condition with the team. The existing laptop waypoint planner uses an older protocol and is not connected automatically.

## Testing

On a PC from the repository root:

```
python -m unittest discover -s tests -v
```

Simulation checks known wheel vectors, kinematic round trips, pose transforms, 610 mm forward/sideways/diagonal moves, limits and a frozen-encoder timeout. The simulated wheels follow targets perfectly, so these tests do not establish real accuracy.

Before floor testing: measure geometry and wheel diameter, validate motor-driver current suitability, calibrate Task 2's encoders/PID and verify wheel direction with the robot lifted. Start at low speed with a clear area and accessible physical power disconnect. Confirm odometry and stopping on short moves first. Then, per handoff, tape a start mark, drive 610 mm five times each forward, sideways and diagonally, and record destination error; the acceptance target is approximately 30 mm. Test RFID integration separately. No real-hardware tests have been performed.


## Integration wiring

Task 3 now includes the scheduling adapter, heading hold, normal translation
command deceleration, RFID-confirmed route callback, and acceptance trial runner.
Task 2 PR #6 remains a separate draft dependency: this change does not copy or
silently approve it. Main can run Task 3's simulations now; physical operation
requires the Task 2 controller and real IRQ encoders from that work, calibrated
and configured before constructing Task 3. No motor program starts on import.

With the Task 2 files available on the Pico import path:

```python
from speed_control import WheelSpeedControl
from motor_driver import set_motor, stop_all
from ControlCode.kiwi_drive.wheel_adapter import WheelAdapter
from ControlCode.kiwi_drive.kiwi_drive import KiwiDrive

controller = WheelSpeedControl(set_motor)
# REQUIRED: replace controller.encoders[1..3] with calibrated EncoderAdapter
# instances backed by QuadratureIRQEncoder. Never use the simulation encoders
# for physical motion. Supply measured diameter/counts/reverse for each wheel.
wheels = WheelAdapter(controller, stop_all)
# cfg contains the measured geometry and validated limits from config_example.
# drive = KiwiDrive(wheels, **cfg)
```

On the Pico upload the `ControlCode/kiwi_drive` folder preserving its package
path (or import its files directly). Task 2 currently uses flat imports, so put
its module folder on the import path. Upload Task 1's `motor_driver.py` as well.
Verify encoder and motor positive directions agree before motion. Keep
`POSITION_TOLERANCE_MM` smaller than RFID's 2 mm centering step.

`navigation.tile_coordinates(tile, origin_x, origin_y, spacing_mm,
board_angle_rad)` offers an explicit board transform. a1's center is the
origin, file increases are +x and rank increases are +y before rotation.
Verify that convention against the installed board and RFID map; the handoff's
nominal spacing is 610 mm. `TileNavigator(drive, rfid, coordinates, route)` takes
an explicit square-to-(x,y) map and an obstacle-cleared `route(tile)` callback.
Pass `navigator.on_goto` to Task 5's `WifiLink`. It validates the whole route,
moves to every supplied waypoint, requires the expected RFID tile, centers
and corrects pose there, and only then returns True. Missing/wrong cards
return False; exceptions propagate to Task 5's lost-status handling. It stops
on every exit. The route provider must include intermediate crossed tiles
where corrections are required and must avoid occupied squares; Task 3 does
not invent routes or connect the legacy laptop planner automatically.

## Physical acceptance record

After calibration, explicitly call
`acceptance.run_trials(drive, measure_pose, prepare_trial, write_row)`.
It runs five 610 mm trials in each of forward, sideways and diagonal directions.
`prepare_trial(direction, trial)` lets the operator put the robot back on the
tape mark and returns the independently measured start (x,y,heading).
`measure_pose()` must return an independently measured endpoint (x,y), never
odometry. `write_row(dict)` saves each result immediately (for example CSV).
The runner records target, measured endpoint, Euclidean error and whether
error is <=30 mm, and always stops after each trial. Importing it does nothing.
Its own automated test uses simulation; no physical results have been recorded.

Task 3 is software-tested, pending measured geometry, Task 2 calibration and
all 15 physical acceptance measurements. RFID UID acquisition and real Wi-Fi
movement validation remain separate system dependencies. The existing LED
bench boot program stays available until an operator supplies those inputs.
