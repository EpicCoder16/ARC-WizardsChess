import math
import unittest
from ControlCode.kiwi_drive.kiwi_drive import KiwiDrive, MotionError


class SimulatedWheels:
    def __init__(self, frozen=False):
        self.distances = [0.0] * 3
        self.targets = [0.0] * 3
        self.frozen = frozen
        self.stopped = False

    def get_wheel_distance(self, wheel):
        return self.distances[wheel - 1]

    def set_wheel_speed(self, wheel, speed):
        self.targets[wheel - 1] = speed

    def service(self):
        pass

    def stop(self):
        self.stopped = True
        self.targets = [0.0] * 3

    def advance(self, dt):
        if not self.frozen:
            self.distances = [d + s * dt for d, s in zip(self.distances, self.targets)]


class SimulatedClock:
    def __init__(self, wheels):
        self.t = 0
        self.wheels = wheels

    def now_ms(self):
        return self.t

    def diff_ms(self, a, b):
        return a - b

    def sleep_ms(self, ms):
        self.wheels.advance(ms / 1000.0)
        self.t += ms


def create(wheels, **overrides):
    options = dict(wheel_angles_rad=(0, 2 * math.pi / 3, 4 * math.pi / 3),
                   wheel_radii_mm=(100, 100, 100), max_wheel_speed_mm_s=150,
                   max_move_mm=1000, speed_mm_s=100, acceleration_mm_s2=100,
                   tolerance_mm=1, timeout_ms=15000,
                   clock=SimulatedClock(wheels))
    options.update(overrides)
    return KiwiDrive(wheels, **options)


class KiwiDriveTests(unittest.TestCase):
    def assertVectorAlmostEqual(self, left, right):
        for actual, expected in zip(left, right):
            self.assertAlmostEqual(actual, expected, places=7)

    def test_known_forward_and_rotation(self):
        drive = create(SimulatedWheels())
        self.assertVectorAlmostEqual(drive.wheel_speeds(0, 100, 0), (100, -50, -50))
        self.assertVectorAlmostEqual(drive.wheel_speeds(0, 0, 1), (100, 100, 100))

    def test_forward_and_inverse_kinematics_round_trip(self):
        drive = create(SimulatedWheels())
        for motion in ((100, 0, 0), (0, 100, 0), (30, -40, 0.5)):
            self.assertVectorAlmostEqual(drive.chassis_motion(drive.wheel_speeds(*motion)), motion)

    def test_odometry_accounts_for_heading_and_pose_rebase(self):
        wheels = SimulatedWheels()
        drive = create(wheels)
        drive.set_pose(10, 20, math.pi / 2)
        wheels.distances = list(drive.wheel_speeds(0, 100, 0))
        self.assertVectorAlmostEqual(drive.get_pose(), (-90, 20, math.pi / 2))
        drive.set_pose(3, 4, 0)
        self.assertVectorAlmostEqual(drive.get_pose(), (3, 4, 0))

    def test_arc_integration(self):
        wheels = SimulatedWheels()
        drive = create(wheels)
        wheels.distances = list(drive.wheel_speeds(100, 0, math.pi / 2))
        self.assertVectorAlmostEqual(drive.get_pose(), (200 / math.pi, 200 / math.pi, math.pi / 2))

    def test_moves_one_tile_in_three_directions(self):
        for target in ((610, 0), (0, 610), (610 / math.sqrt(2), 610 / math.sqrt(2))):
            with self.subTest(target=target):
                wheels = SimulatedWheels()
                drive = create(wheels)
                self.assertTrue(drive.move_to(*target))
                x, y, _ = drive.get_pose()
                self.assertLessEqual(math.hypot(x - target[0], y - target[1]), 1)
                self.assertTrue(wheels.stopped)

    def test_relative_motion_respects_robot_heading(self):
        wheels = SimulatedWheels()
        drive = create(wheels)
        drive.set_pose(0, 0, math.pi / 2)
        drive.move_relative(0, 20)
        x, y, _ = drive.get_pose()
        self.assertLessEqual(math.hypot(x + 20, y), 1)

    def test_speed_limit_scales_all_wheels(self):
        drive = create(SimulatedWheels())
        raw = drive.wheel_speeds(200, 70, 0.2)
        scaled = drive.drive(200, 70, 0.2)
        self.assertAlmostEqual(max(abs(v) for v in scaled), 150)
        self.assertVectorAlmostEqual(scaled, tuple(v * 150 / max(abs(s) for s in raw) for v in raw))

    def test_stall_times_out_and_stops(self):
        wheels = SimulatedWheels(frozen=True)
        drive = create(wheels, timeout_ms=300)
        with self.assertRaises(MotionError):
            drive.move_to(100, 0)
        self.assertTrue(wheels.stopped)
        self.assertEqual(wheels.targets, [0, 0, 0])

    def test_limits_and_singular_geometry(self):
        wheels = SimulatedWheels()
        drive = create(wheels, max_move_mm=10)
        with self.assertRaises(MotionError):
            drive.move_to(11, 0)
        self.assertTrue(wheels.stopped)
        with self.assertRaises(ValueError):
            create(wheels, wheel_angles_rad=(0, 0, 0))


if __name__ == '__main__':
    unittest.main()
