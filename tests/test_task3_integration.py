import math
import unittest
from ControlCode.kiwi_drive.wheel_adapter import WheelAdapter
from ControlCode.kiwi_drive.navigation import TileNavigator, tile_coordinates
from ControlCode.kiwi_drive.acceptance import run_trials
from test_kiwi_drive import SimulatedWheels, create


class Task3IntegrationTests(unittest.TestCase):
    def test_adapter_schedule_and_immediate_stop_after_fault(self):
        class PID:
            def reset(self):
                self.reset_called = True
        class Controller(SimulatedWheels):
            def __init__(self):
                super().__init__()
                self.pids = {i: PID() for i in (1, 2, 3)}
                self.updates = []
            def update(self, dt):
                self.updates.append(dt)
        controller = Controller()
        now = [0]
        class Clock:
            def now_ms(self): return now[0]
            def diff_ms(self, a, b): return a-b
        stopped = []
        adapter = WheelAdapter(controller, lambda: stopped.append(True), Clock())
        self.assertFalse(adapter.service())
        now[0] = 25
        self.assertTrue(adapter.service())
        self.assertEqual(controller.updates, [.025])
        adapter.set_wheel_speed(1, 100)
        adapter.stop()
        self.assertEqual(controller.targets, [0, 0, 0])
        self.assertTrue(all(p.reset_called for p in controller.pids.values()))
        controller.update = lambda dt: (_ for _ in ()).throw(OSError('motor fault'))
        now[0] += 20
        with self.assertRaises(OSError): adapter.service()
        self.assertEqual(len(stopped), 2)

    def test_navigation_requires_correct_rfid_at_every_waypoint(self):
        class Motion:
            def __init__(self): self.moves = []; self.stopped = False
            def move_to(self, *xy): self.moves.append(xy); return True
            def stop(self): self.stopped = True
        class RFID:
            def __init__(self): self.tiles = iter(['a2', 'a3']); self.tile = None
            def current_tile(self): self.tile = next(self.tiles); return self.tile
            def center_on_card(self): return self.tile
        motion = Motion()
        nav = TileNavigator(motion, RFID(), {'a2': (0,610), 'a3': (0,1220)}, lambda tile: ['a2',tile])
        self.assertTrue(nav.on_goto('a3'))
        self.assertEqual(motion.moves, [(0,610),(0,1220)])
        self.assertTrue(motion.stopped)
        nav.rfid.current_tile = lambda: 'h8'
        self.assertFalse(nav.on_goto('a3'))
        nav.route = lambda tile: ['missing', tile]
        before = len(motion.moves)
        with self.assertRaises(KeyError): nav.on_goto('a3')
        self.assertEqual(len(motion.moves), before)

    def test_rotated_board_map(self):
        x,y = tile_coordinates('b2',10,20,610,math.pi/2)
        self.assertAlmostEqual(x,-600)
        self.assertAlmostEqual(y,630)
        with self.assertRaises(ValueError): tile_coordinates('i1',0,0,610,0)

    def test_move_ramps_translation_to_zero_and_corrects_heading(self):
        wheels = SimulatedWheels()
        drive = create(wheels)
        commands = []
        original = drive.drive
        def record(vx,vy,omega):
            commands.append((vx,vy,omega))
            return original(vx,vy,omega)
        drive.drive = record
        drive.move_to(0,610)
        speeds = [math.hypot(vx,vy) for vx,vy,_ in commands]
        self.assertLessEqual(max(abs(b-a) for a,b in zip(speeds,speeds[1:])), 2.000001)
        self.assertAlmostEqual(speeds[-1],0)
        # Inject a heading disturbance during motion.
        commands.clear()
        service = wheels.service
        def disturb():
            if not commands: drive._pose = (0,610,.1)
            service()
        wheels.service = disturb
        drive.move_to(0,620)
        self.assertTrue(any(omega < 0 for _,_,omega in commands))

    def test_physical_trial_runner_records_fifteen_independent_measurements(self):
        wheels = SimulatedWheels()
        drive = create(wheels)
        rows = run_trials(drive, lambda: drive.get_pose()[:2],
                          lambda direction, trial: (0,0,0), lambda row: None)
        self.assertEqual(len(rows),15)
        self.assertTrue(all(row['passed'] for row in rows))
