"""Task 3 measurements and limits. Replace every None before robot operation.

This file is configuration only: it does not start or move the robot.
"""

WHEEL_DIAMETER_MM = 58.0  # confirmed wheel diameter
ENCODER_COUNTS_PER_WHEEL_REV = None  # measure with the actual encoder and decoding mode

# Each tuple is ordered wheel 1, 2, 3. Angles are in radians, counterclockwise
# from robot-right (+x). Radii are center-to-wheel distances, not wheel radii.
WHEEL_ANGLES_RAD = (None, None, None)
WHEEL_CENTER_DISTANCES_MM = (None, None, None)

# Determine and validate these limits on the actual robot.
MAX_WHEEL_SPEED_MM_S = None
MAX_MOVE_MM = None
MOVE_SPEED_MM_S = None
ACCELERATION_MM_S2 = None
POSITION_TOLERANCE_MM = None
MOVE_TIMEOUT_MS = None
CONTROL_PERIOD_MS = 20  # initial 50 Hz scheduling target; verify with Task 2
