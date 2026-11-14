# Task 3: kiwi drive and odometry

This module implements configurable three-wheel kinematics, encoder-based odometry and a blocking point-to-point controller. It is **simulation-tested only**. Real robot geometry, Task 2's encoder/speed-control adapter, Task 4 UID reading, calibration and hardware tests are still required. It is not a deployable robot program by itself.

## Frames and units

Body x is robot-right, body y is robot-forward. Board x/y are millimeters, heading is radians counterclockwise; heading zero aligns the robot's body axes to the board axes. Positive rotation is counterclockwise. Wheel angles give each wheel's location around the robot center in radians counterclockwise from body +x. The assumed drive direction is tangent counterclockwise to that radius: at angle 0, positive wheel travel drives body +y. Configure or invert each encoder and motor sign in the Task 2 adapter so measured and commanded positive travel agree. Wheel indices 1, 2, 3 must match those angles and radii.

`wheel_radii_mm` is the **distance from robot center to each wheel's contact point**, not wheel radius. Task 2 converts encoder counts to signed millimeters using the measured wheel diameter and counts per revolution. Confirm all geometry, coordinate origin, signs and units with the team. The actual wheel angles and distances have not been measured here.

The confirmed wheel diameter is **58 mm**, recorded in `config_example.py`. One wheel revolution travels `pi * 58`, approximately **182.2 mm**, before allowing for slip. Task 2 must use the actual encoder counts per wheel revolution and its chosen decoding mode when converting counts to distance. All remaining unmeasured geometry and motion limits are `None` placeholders; they must be replaced with measured or validated values before constructing a controller. The example configuration is not loaded automatically and cannot move the robot.

## Interface

Construct `KiwiDrive(wheels, wheel_angles_rad, wheel_radii_mm, max_wheel_speed_mm_s, max_move_mm, speed_mm_s, acceleration_mm_s2, tolerance_mm, timeout_ms, control_period_ms=20)`. All configuration is required; values in the tests are simulated examples, not real settings. Construction reads wheel distances but does not command motion.

- `wheel_speeds(vx, vy, omega)` converts body-frame velocities in mm/s and rad/s to wheel speeds in mm/s. `chassis_motion(values)` performs the inverse for either wheel speeds or wheel distances.
- `drive(vx, vy, omega)` sends wheel speed targets, scaling all together if needed to respect the configured wheel limit. It does not schedule the PID loop or stop automatically: the caller must service Task 2 and stop it.
- `get_pose()` updates odometry from signed cumulative wheel distances and returns `(x_mm, y_mm, heading_rad)`. Call frequently, including when external code drives the robot. `set_pose(x, y, heading)` corrects pose and rebases encoder readings without resetting the physical encoders.
- `move_to(x_mm, y_mm)` is a blocking, feedback-controlled board-frame move with a speed ramp, move-distance limit and timeout. It commands zero rotation and updates heading from odometry; it does **not** actively correct heading error or avoid obstacles. It returns `True` inside the configured position tolerance and stops on success or exception. Slippage, sensor errors, scheduling delays, motor lag and asymmetric acceleration can still cause overshoot or heading drift. Tune on the actual robot.
- `move_relative(right_mm, forward_mm)` converts a robot-frame offset into a board target and calls `move_to`; it supplies the movement API expected by Task 4. Configure position tolerance smaller than the smallest commanded RFID centering step, or those steps will be treated as already complete. The RFID offset direction/signs still need real testing.
- `stop()` zeros all three targets and calls the adapter's immediate stop, attempting all wheels even when a command fails. This is software only; use a physical disconnect as well.

`move_to` assumes the robot is initially stationary and Task 2's PID loop controls speed in both directions. Target updates are bounded by `max_wheel_speed_mm_s`, but `drive` is a low-level interface: callers must ramp their own commands. Stopping at tolerance or after a fault is immediate and is not a controlled deceleration guarantee. Hardware protection, watchdogs and an independent emergency stop remain necessary.

## Task 2 adapter contract

Provide an object with:

```
get_wheel_distance(wheel)        # 1..3, signed cumulative mm
set_wheel_speed(wheel, mm_per_s)  # signed target, non-blocking
service()                        # advance PID / sample encoders, non-blocking
stop()                           # immediately stop motor outputs and PID targets
```

The handoff's three Task 2 functions do not yet include `service()` or `stop()`; the teams must decide how the 50 Hz PID loop is scheduled and implement these adapter methods. No fake motor driver is installed in this module. Do not reset encoder counts behind the controller's back; rebase with `set_pose` if counts are reset. Ensure `get_wheel_distance` is coherent across three wheels and encoder counts cannot silently wrap or go stale. Configure a task-specific watchdog in the adapter: the time limit here cannot guarantee that motors stop if a blocking hardware call or the interpreter hangs.

The Task 4 RFID class accepts this object's `move_relative`, `get_pose`, `set_pose` and `stop` methods. The Task 5 `on_goto(tile)` callback still needs a **verified tile-to-coordinate map**, then can call `move_to`. Do not report arrival merely from an estimated pose if RFID confirmation is required; agree on the success condition with the team. The existing laptop waypoint planner uses an older protocol and is not connected automatically.

## Testing

On a PC from the repository root:

```
python -m unittest discover -s tests -v
```

Simulation checks known wheel vectors, kinematic round trips, pose transforms, 610 mm forward/sideways/diagonal moves, limits and a frozen-encoder timeout. The simulated wheels follow targets perfectly, so these tests do not establish real accuracy.

Before floor testing: measure geometry and wheel diameter, validate motor-driver current suitability, calibrate Task 2's encoders/PID and verify wheel direction with the robot lifted. Start at low speed with a clear area and accessible physical power disconnect. Confirm odometry and stopping on short moves first. Then, per handoff, tape a start mark, drive 610 mm five times each forward, sideways and diagonally, and record destination error; the acceptance target is approximately 30 mm. Test RFID integration separately. No real-hardware tests have been performed.
