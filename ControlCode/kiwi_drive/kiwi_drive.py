"""Configurable 3-wheel kiwi kinematics and feedback motion controller.

Body axes: x right, y forward, heading/omega counterclockwise in radians.
Wheel angles locate each wheel around the robot, CCW from +x; positive
wheel travel is tangential CCW. Actual wiring polarity belongs in Task 2.
"""
import math
import time


class MotionError(Exception):
    pass


def _finite(value):
    return isinstance(value, (int, float)) and value == value and abs(value) != float('inf')


def _inverse3(a):
    # Gauss-Jordan inversion also rejects degenerate wheel layouts.
    m = [list(a[i]) + [1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise ValueError('wheel geometry is singular')
        m[col], m[pivot] = m[pivot], m[col]
        scale = m[col][col]
        m[col] = [v / scale for v in m[col]]
        for row in range(3):
            if row != col:
                factor = m[row][col]
                m[row] = [m[row][j] - factor * m[col][j] for j in range(6)]
    return [row[3:] for row in m]


def _matvec(matrix, vector):
    return tuple(sum(row[j] * vector[j] for j in range(3)) for row in matrix)


class KiwiDrive:
    def __init__(self, wheels, wheel_angles_rad, wheel_radii_mm, max_wheel_speed_mm_s,
                 max_move_mm, speed_mm_s, acceleration_mm_s2, tolerance_mm,
                 timeout_ms, control_period_ms=20, clock=None):
        """All physical parameters are explicit; no robot measurements are assumed.

        wheels provides get_wheel_distance(1..3), set_wheel_speed(1..3, mm/s),
        service() for Task 2's controller, and stop() to immediately zero motor
        output. Distances are signed cumulative mm.
        The drive is inert until drive() or move_to() is called.
        """
        if len(wheel_angles_rad) != 3 or len(wheel_radii_mm) != 3:
            raise ValueError('three wheel angles and radii required')
        values = tuple(wheel_angles_rad) + tuple(wheel_radii_mm) + (
            max_wheel_speed_mm_s, max_move_mm, speed_mm_s, acceleration_mm_s2,
            tolerance_mm, timeout_ms, control_period_ms)
        if not all(_finite(v) for v in values):
            raise ValueError('all geometry and limits must be finite numbers')
        if any(r <= 0 for r in wheel_radii_mm) or min(max_wheel_speed_mm_s,
                max_move_mm, speed_mm_s, acceleration_mm_s2, tolerance_mm,
                timeout_ms, control_period_ms) <= 0:
            raise ValueError('radii and motion limits must be positive')
        self.wheels = wheels
        self.matrix = tuple((-math.sin(a), math.cos(a), r)
                            for a, r in zip(wheel_angles_rad, wheel_radii_mm))
        self.inverse = _inverse3(self.matrix)
        self.max_wheel_speed = max_wheel_speed_mm_s
        self.max_move = max_move_mm
        self.speed = speed_mm_s
        self.accel = acceleration_mm_s2
        self.tolerance = tolerance_mm
        self.timeout_ms = timeout_ms
        self.period_ms = int(control_period_ms)
        if self.period_ms <= 0:
            raise ValueError('control period must be at least one millisecond')
        self.clock = clock or _Clock()
        self._pose = (0.0, 0.0, 0.0)
        self._last_distances = self._read_distances()

    def _read_distances(self):
        values = tuple(self.wheels.get_wheel_distance(i) for i in (1, 2, 3))
        if not all(_finite(v) for v in values):
            raise MotionError('invalid wheel distance')
        return values

    def wheel_speeds(self, vx, vy, omega):
        """Robot-frame mm/s and rad/s to three signed wheel speeds in mm/s."""
        if not all(_finite(v) for v in (vx, vy, omega)):
            raise ValueError('motion must be finite')
        return _matvec(self.matrix, (vx, vy, omega))

    def chassis_motion(self, wheel_motion):
        """Wheel distances/speeds to body dx, dy, dtheta or vx, vy, omega."""
        if len(wheel_motion) != 3 or not all(_finite(v) for v in wheel_motion):
            raise ValueError('three finite wheel values required')
        return _matvec(self.inverse, wheel_motion)

    def drive(self, vx, vy, omega):
        speeds = self.wheel_speeds(vx, vy, omega)
        peak = max(abs(v) for v in speeds)
        if peak > self.max_wheel_speed:
            speeds = tuple(v * self.max_wheel_speed / peak for v in speeds)
        try:
            for wheel, speed in enumerate(speeds, 1):
                self.wheels.set_wheel_speed(wheel, speed)
        except Exception:
            self.stop()
            raise
        return speeds

    def stop(self):
        """Zero wheel targets and request an immediate hardware output stop."""
        error = None
        for wheel in (1, 2, 3):
            try:
                self.wheels.set_wheel_speed(wheel, 0)
            except Exception as exc:
                error = exc
        try:
            self.wheels.stop()
        except Exception as exc:
            error = exc
        if error is not None:
            raise error

    def update_pose(self):
        distances = self._read_distances()
        delta = tuple(distances[i] - self._last_distances[i] for i in range(3))
        dx, dy, turn = self.chassis_motion(delta)
        x, y, heading = self._pose
        # Integrate a constant body twist, including rotation during the sample.
        if abs(turn) < 1e-9:
            body_x, body_y = dx, dy
        else:
            s, c = math.sin(turn) / turn, (1 - math.cos(turn)) / turn
            body_x, body_y = s * dx - c * dy, c * dx + s * dy
        self._pose = (x + math.cos(heading) * body_x - math.sin(heading) * body_y,
                      y + math.sin(heading) * body_x + math.cos(heading) * body_y,
                      (heading + turn + math.pi) % (2 * math.pi) - math.pi)
        self._last_distances = distances
        return self._pose

    def get_pose(self):
        return self.update_pose()

    def set_pose(self, x, y, heading):
        """Apply an external correction and rebase the current encoder snapshot."""
        if not all(_finite(v) for v in (x, y, heading)):
            raise ValueError('pose must be finite')
        snapshot = self._read_distances()
        self._pose = (x, y, (heading + math.pi) % (2 * math.pi) - math.pi)
        self._last_distances = snapshot

    def move_to(self, x_mm, y_mm):
        """Blocking feedback move in board coordinates, with bounded duration.

        Call only when Task 2 service() is safe to run and robot area is clear.
        It commands zero rotation; actual heading is tracked but not corrected.
        """
        if not all(_finite(v) for v in (x_mm, y_mm)):
            raise ValueError('target must be finite')
        start = self.clock.now_ms()
        previous = start
        commanded_speed = 0.0
        try:
            x, y, _ = self.get_pose()
            if math.hypot(x_mm - x, y_mm - y) > self.max_move:
                raise MotionError('target exceeds configured move limit')
            while True:
                self.wheels.service()
                x, y, heading = self.get_pose()
                dx, dy = x_mm - x, y_mm - y
                remaining = math.hypot(dx, dy)
                if remaining <= self.tolerance:
                    return True
                now = self.clock.now_ms()
                if self.clock.diff_ms(now, start) >= self.timeout_ms:
                    raise MotionError('motion timed out')
                dt = max(0.0, self.clock.diff_ms(now, previous) / 1000.0)
                previous = now
                # Brake based on remaining distance and ramp up from zero.
                wanted = min(self.speed, math.sqrt(2 * self.accel * remaining))
                change = self.accel * dt
                commanded_speed = max(commanded_speed - change,
                                      min(wanted, commanded_speed + change))
                world_vx, world_vy = commanded_speed * dx / remaining, commanded_speed * dy / remaining
                body_vx = math.cos(heading) * world_vx + math.sin(heading) * world_vy
                body_vy = -math.sin(heading) * world_vx + math.cos(heading) * world_vy
                self.drive(body_vx, body_vy, 0.0)
                self.clock.sleep_ms(self.period_ms)
        finally:
            self.stop()

    def move_relative(self, right_mm, forward_mm):
        """Task 4 adapter: body-frame displacement with the same move limits."""
        if not all(_finite(v) for v in (right_mm, forward_mm)):
            raise ValueError('relative movement must be finite')
        x, y, heading = self.get_pose()
        return self.move_to(x + math.cos(heading) * right_mm - math.sin(heading) * forward_mm,
                            y + math.sin(heading) * right_mm + math.cos(heading) * forward_mm)


class _Clock:
    def now_ms(self):
        if hasattr(time, 'ticks_ms'):
            return time.ticks_ms()
        return int(time.monotonic() * 1000)

    def diff_ms(self, later, earlier):
        return time.ticks_diff(later, earlier) if hasattr(time, 'ticks_diff') else later - earlier

    def sleep_ms(self, delay):
        if hasattr(time, 'sleep_ms'):
            time.sleep_ms(delay)
        else:
            time.sleep(delay / 1000.0)
