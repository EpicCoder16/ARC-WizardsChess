"""Offline UDP loopback checks for the Task 5 laptop/Pico protocol."""
import json
import socket
import time
import unittest

from ControlCode.Task5_WiFiLink.laptop_link import LaptopLink
from ControlCode.Task5_WiFiLink.wifi_link import MAX_DATAGRAM_BYTES, WifiLink


class _ConnectedWLAN:
    def __init__(self, interface):
        self.interface = interface

    def active(self, enabled):
        self.enabled = enabled

    def isconnected(self):
        return True

    def connect(self, ssid, password):
        raise AssertionError("already connected")

    def ifconfig(self):
        return ("127.0.0.1", "255.0.0.0", "127.0.0.1", "127.0.0.1")

    def disconnect(self):
        pass


class _Network:
    STA_IF = 0
    WLAN = _ConnectedWLAN


def _unused_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class WifiLoopbackTests(unittest.TestCase):
    def setUp(self):
        self.port = _unused_port()
        self.robot_id = "white-king"
        self.moves = []
        self.statuses = []
        self.laptop = LaptopLink(
            host="127.0.0.1", port=self.port, on_status=self.statuses.append
        )
        self.pico = WifiLink(
            ssid="offline-test",
            password="offline-test",
            robot_id=self.robot_id,
            laptop_ip="127.0.0.1",
            port=self.port,
            local_port=0,
            on_goto=self._move_to_tile,
            network_module=_Network,
        )
        self.assertEqual(self.pico.connect(), "127.0.0.1")
        hello = self.laptop.receive_once(timeout=1.0)
        self.assertEqual(
            hello,
            {"type": "hello", "robot_id": self.robot_id, "protocol": 1},
        )
        # Consume the registration response explicitly so it cannot sit ahead
        # of the first test command in the Pico's UDP receive queue.
        self.pico.sock.setblocking(True)
        self.pico.sock.settimeout(1.0)
        response, _ = self.pico.sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
        self.pico.sock.setblocking(False)
        self.assertEqual(json.loads(response.decode("utf-8")), {
            "type": "registered", "robot_id": self.robot_id, "protocol": 1
        })

    def tearDown(self):
        self.pico.close()
        self.laptop.close()

    def _move_to_tile(self, tile):
        self.moves.append(tile)
        return None  # Task 3 contract: only literal False means failure.

    def _send_json_to_pico(self, message):
        address = self.laptop.devices[self.robot_id]
        self.laptop.sock.sendto(json.dumps(message).encode("utf-8"), address)

    def _poll_command(self, command_id, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            processed = self.pico.poll()
            if command_id in self.pico._command_status:
                return processed
            time.sleep(0.001)
        self.fail("Pico did not process command {!r}".format(command_id))

    def _poll_until_handled(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.pico.poll():
                return True
            time.sleep(0.001)
        self.fail("Pico did not handle the queued command")

    def _receive_status(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.laptop.receive_once(
                timeout=min(0.1, max(0.001, deadline - time.monotonic()))
            )
            if message is not None and message.get("type") == "status":
                return message
        return None

    def test_goto_callbacks_and_status_messages_round_trip(self):
        command_id = self.laptop.send_goto(self.robot_id, "f3")
        self.assertTrue(self._poll_command(command_id))
        moving = self._receive_status()
        arrived = self._receive_status()

        self.assertEqual(self.moves, ["f3"])
        self.assertEqual(
            (moving["type"], moving["robot_id"], moving["status"],
             moving["tile"], moving["command_id"]),
            ("status", self.robot_id, "moving", "f3", command_id),
        )
        self.assertEqual(arrived["status"], "arrived")
        self.assertEqual(arrived["command_id"], command_id)
        self.assertEqual(self.statuses, [moving, arrived])

        # Replaying the same UDP command returns the cached result and never
        # calls the motion adapter twice.
        self._send_json_to_pico({
            "robot_id": self.robot_id,
            "cmd": "goto",
            "tile": "f3",
            "command_id": command_id,
        })
        self.assertTrue(self._poll_until_handled())
        retry_status = self._receive_status()
        self.assertEqual(retry_status["status"], "arrived")
        self.assertEqual(self.moves, ["f3"])

        self.pico.send_status("battery_low", battery_pct=18)
        battery = self._receive_status()
        self.assertEqual(battery["status"], "battery_low")
        self.assertEqual(battery["battery_pct"], 18)

    def test_motion_failure_and_invalid_tile_report_lost(self):
        self.pico.on_goto = lambda tile: False
        command_id = self.laptop.send_goto(self.robot_id, "g4")
        self.assertFalse(self._poll_command(command_id))
        self.assertEqual(self._receive_status()["status"], "moving")
        lost = self._receive_status()
        self.assertEqual(lost["status"], "lost")
        self.assertEqual(lost["detail"], "motion_handler_failed")
        self.assertEqual(lost["command_id"], command_id)

        self._send_json_to_pico({
            "robot_id": self.robot_id,
            "cmd": "goto",
            "tile": "i9",
            "command_id": "invalid-square",
        })
        self.assertFalse(self._poll_command("invalid-square"))
        invalid = self._receive_status()
        self.assertEqual((invalid["status"], invalid["detail"]),
                         ("lost", "invalid_tile"))

        def fail_move(tile):
            raise RuntimeError("task3 timeout")

        self.pico.on_goto = fail_move
        command_id = self.laptop.send_goto(self.robot_id, "h8")
        self.assertFalse(self._poll_command(command_id))
        self._receive_status()  # moving
        failed = self._receive_status()
        self.assertEqual(failed["status"], "lost")
        self.assertEqual(failed["detail"], "task3 timeout")
        self.assertEqual(failed["command_id"], command_id)

    def test_missing_robot_id_malformed_and_oversized_packets_are_ignored(self):
        address = self.laptop.devices[self.robot_id]
        self._send_json_to_pico({
            "cmd": "goto", "tile": "f3", "command_id": "missing-target"
        })
        for _ in range(20):
            self.pico.poll()
            time.sleep(0.001)
        self.assertEqual(self.moves, [])
        self.assertIsNone(self._receive_status(timeout=0.05))

        self.laptop.sock.sendto(b"{not-json", address)
        for _ in range(20):
            self.pico.poll()
            time.sleep(0.001)
        self.assertEqual(self.moves, [])

        long_command_id = "x" * 129
        self._send_json_to_pico({
            "robot_id": self.robot_id,
            "cmd": "goto",
            "tile": "f3",
            "command_id": long_command_id,
        })
        for _ in range(20):
            self.pico.poll()
            time.sleep(0.001)
        self.assertEqual(self.moves, [])

        valid = json.dumps({
            "robot_id": self.robot_id,
            "cmd": "goto",
            "tile": "f3",
            "command_id": "oversized",
        }).encode("utf-8")
        payload = valid + b" " * (MAX_DATAGRAM_BYTES - len(valid)) + b"x"
        self.assertEqual(len(payload), MAX_DATAGRAM_BYTES + 1)
        self.laptop.sock.sendto(payload, address)
        for _ in range(20):
            self.pico.poll()
            time.sleep(0.001)
        self.assertEqual(self.moves, [])
        self.assertIsNone(self._receive_status(timeout=0.05))

    def test_laptop_rejects_oversized_and_unknown_protocol_hello(self):
        hello = json.dumps({
            "type": "hello", "robot_id": "other", "protocol": 1
        }).encode("utf-8")
        oversized = hello + b" " * (MAX_DATAGRAM_BYTES - len(hello)) + b"x"
        self.laptop.sock.sendto(oversized, ("127.0.0.1", self.port))
        self.assertIsNone(self.laptop.receive_once(timeout=1.0))
        self.assertNotIn("other", self.laptop.devices)

        incompatible = json.dumps({
            "type": "hello", "robot_id": "other", "protocol": 2
        }).encode("utf-8")
        self.laptop.sock.sendto(incompatible, ("127.0.0.1", self.port))
        self.assertIsNone(self.laptop.receive_once(timeout=1.0))
        self.assertNotIn("other", self.laptop.devices)

    def test_message_validation_rejects_invalid_outgoing_metadata(self):
        with self.assertRaises(ValueError):
            self.pico.send_status("arrived", tile="i9")
        with self.assertRaises(ValueError):
            self.pico.send_status("battery_low", battery_pct=101)
        with self.assertRaises(ValueError):
            self.pico.send_status("arrived", command_id=3)
        with self.assertRaises(ValueError):
            self.pico.send_status("arrived", command_id="x" * 129)
        with self.assertRaises(ValueError):
            self.laptop.send_goto("x" * 65, "f3")


if __name__ == "__main__":
    unittest.main()
