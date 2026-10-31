import unittest
import tempfile
import os
from RFID.position import RFIDPosition, CenteringError
from RFID.tile_map import register, save, load


class SimulatedRobot:
    def __init__(self, x=0, y=0, card=True):
        self.x, self.y, self.card = x, y, card
        self.pose_updates = []
        self.stops = 0

    def read_pair(self):
        if not self.card or abs(self.y) > 20 or abs(self.x) > 50:
            return None, None
        # Card is at (0,0), readers are 52 mm apart. Lower is closer.
        left = round(abs(self.x - 26) / 2)
        right = round(abs(self.x + 26) / 2)
        return (b'\x01\x02\x03\x04', left), ('01:02:03:04', right)

    def move_relative(self, right, forward):
        self.x += right
        self.y += forward

    def get_pose(self):
        return (111, 222, 1.2)

    def set_pose(self, x, y, heading):
        self.pose_updates.append((x, y, heading))

    def stop(self):
        self.stops += 1


class RFIDPositionTests(unittest.TestCase):
    def make_position(self, robot, **kwargs):
        return RFIDPosition(robot, robot, {'01020304': ('e4', 2745, 2135)}, **kwargs)

    def test_centers_from_varied_start_positions(self):
        for x, y in ((-30, 10), (30, -10), (0, 0), (8, 18)):
            with self.subTest(x=x, y=y):
                robot = SimulatedRobot(x, y)
                position = self.make_position(robot)
                self.assertEqual(position.center_on_card(), 'e4')
                self.assertLessEqual(abs(robot.x), 2)
                self.assertLessEqual(abs(robot.y), 2)
                self.assertEqual(robot.pose_updates, [(2745, 2135, 1.2)])
                self.assertEqual(robot.stops, 1)

    def test_missing_card_stops_without_pose_correction(self):
        robot = SimulatedRobot(card=False)
        with self.assertRaises(CenteringError):
            self.make_position(robot).center_on_card()
        self.assertEqual(robot.pose_updates, [])
        self.assertEqual(robot.stops, 1)

    def test_edge_limit_fails_closed(self):
        robot = SimulatedRobot()
        with self.assertRaises(CenteringError):
            self.make_position(robot, max_travel_mm=4).center_on_card()
        self.assertEqual(robot.pose_updates, [])
        self.assertEqual(robot.stops, 1)

    def test_search_is_bounded(self):
        robot = SimulatedRobot(card=False)
        self.assertIsNone(self.make_position(robot).search_for_card(radius_steps=3))
        self.assertLessEqual(max(abs(robot.x), abs(robot.y)), 6)
        self.assertEqual(robot.stops, 1)

    def test_unknown_card_is_not_a_tile(self):
        robot = SimulatedRobot()
        position = RFIDPosition(robot, robot, {})
        self.assertIsNone(position.current_tile())
        with self.assertRaises(CenteringError):
            position.center_on_card()
        self.assertEqual(robot.pose_updates, [])

    def test_register_and_reload_map_and_reject_duplicates(self):
        tiles = {}
        register(tiles, b'\x01\x02', 'e4', 10, 20)
        with self.assertRaises(ValueError):
            register(tiles, 'ABCD', 'e4', 10, 20)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'tiles.json')
            save(tiles, path)
            self.assertEqual(load(path), tiles)


if __name__ == '__main__':
    unittest.main()
