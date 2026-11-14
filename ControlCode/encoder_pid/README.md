
# Wizard Chess - Task 2: Encoder and PID Speed Control

## Overview

This module controls the speed of all three wheels on the
Wizard Chess robot.

Encoders measure how far each wheel rotates, and PID
controllers adjust motor power to reach the target speed.

Designed for the Raspberry Pi Pico 2 W using MicroPython.

## Features

- Three independent wheel encoders
- Forward and backward rotation tracking
- Encoder counts converted to distance in millimeters
- Wheel speed calculated in mm/s
- PID motor speed control
- 50 Hz control loop (20 ms intervals)
- Integration with Task 1's motor driver
- Offline testing without physical hardware

## Files

- encoder.py: Encoder counting and speed calculations
- pid_controller.py: PID control calculations
- simulate_pid.py: Simulated motor and PID testing
- plot_speed.py: Generates speed graphs
- speed_control.py: Controls all three wheels
- pico_encoder.py: GPIO interrupt encoder interface
- pico_speed_adapter.py: Connects GPIO encoders to PID
- control_loop.py: Schedules PID updates at 50 Hz
- test_task1_integration.py: Tests Task 1 compatibility
- speed_log.csv: Simulated speed measurements
- speed_plot.png: Graph of simulated speed

## Main Interface

The WheelSpeedControl class provides:

set_wheel_speed(wheel, mm_per_s)
- Sets the target speed for a wheel.

get_wheel_distance(wheel)
- Returns the wheel's signed distance in millimeters.

reset_counts()
- Resets encoder counts for all three wheels.

update(dt)
- Calculates PID output and updates motor power.
- Intended to run approximately every 20 ms.

Wheel numbers are 1, 2, and 3.

## Configuration

Temporary values used for simulation:

- Wheel diameter: 70 mm
- Encoder counts per wheel revolution: 700
- PID Kp: 0.005
- PID Ki: 0.04
- PID Kd: 0.0
- Control frequency: 50 Hz

These values are NOT calibrated for the real robot.

Actual GPIO encoder pins, counts per revolution,
wheel diameter, and PID gains must be confirmed.

## How to Run Offline Tests

From the repository root:

python3 ControlCode/encoder_pid/encoder.py

python3 ControlCode/encoder_pid/pid_controller.py

python3 ControlCode/encoder_pid/simulate_pid.py

python3 ControlCode/encoder_pid/speed_control.py

python3 ControlCode/encoder_pid/control_loop.py

python3 ControlCode/encoder_pid/pico_encoder.py

python3 ControlCode/encoder_pid/pico_speed_adapter.py

python3 ControlCode/encoder_pid/test_task1_integration.py

The Task 1 integration test loads the motor driver
directly from ControlCode/motor_driver/motor_driver.py.

Task 1 has been merged into main, so no separate
Task 1 Git branch is required.

To regenerate the graph, run plot_speed.py using
a Python environment with matplotlib installed.

## Offline Test Results

- Encoder counting simulation: PASS
- PID calculation tests: PASS
- PID speed simulation: PASS
- Three-wheel controller tests: PASS
- Task 1 motor driver integration: PASS
- Simulated 50 Hz scheduling: PASS
- Simulated GPIO encoder tests: PASS
- Pico encoder and PID adapter test: PASS

Reported simulation results:

- Target speed: 100 mm/s
- Average speed error: Within 5%
- Estimated settling time: 0.24 seconds
- Final smoothed speed variation: 3.14 mm/s

These are simulated results, not physical measurements.

## Hardware Testing Still Required

- Confirm encoder GPIO pins and 3.3 V compatibility
- Confirm wheel diameter and counts per revolution
- Verify counts by manually rotating each wheel
- Check for missed encoder transitions
- Test actual 50 Hz scheduling on the Pico
- Tune PID gains with real motors on foam
- Verify speed error stays within approximately 5%
- Verify settling time under 0.5 seconds
- Check for oscillation
- Confirm motor driver current compatibility

The interrupt-based encoder implementation must be
benchmarked on the Pico. PIO may be necessary if GPIO
interrupts cannot reliably capture every transition.

The current code is an offline-tested prototype.
Physical hardware validation is pending.
