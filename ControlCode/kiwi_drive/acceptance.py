"""Operator-run physical Task 3 acceptance trials; never runs on import."""
import math


def run_trials(motion, measure_pose, prepare_trial, write_row):
    """15 physical trials. Callbacks keep measurements independent of odometry.

    prepare_trial(direction, trial) must position the robot on the tape mark
    and return its measured starting (x, y, heading). measure_pose() returns
    independently measured endpoint (x, y). write_row receives a dict for CSV.
    Caller must explicitly invoke this function with a cleared test area.
    """
    results = []
    for name, right, forward in (('forward', 0, 610), ('sideways', 610, 0),
                                ('diagonal', 610/math.sqrt(2), 610/math.sqrt(2))):
        for trial in range(1, 6):
            try:
                x, y, heading = prepare_trial(name, trial)
                motion.set_pose(x, y, heading)
                tx = x + math.cos(heading)*right - math.sin(heading)*forward
                ty = y + math.sin(heading)*right + math.cos(heading)*forward
                if motion.move_to(tx, ty) is not True:
                    raise RuntimeError('move failed')
                mx, my = measure_pose()
                error = math.hypot(mx-tx, my-ty)
                row = dict(direction=name, trial=trial, target_x_mm=tx,
                           target_y_mm=ty, measured_x_mm=mx, measured_y_mm=my,
                           error_mm=error, passed=error <= 30)
                write_row(row)
                results.append(row)
            finally:
                motion.stop()
    return results
