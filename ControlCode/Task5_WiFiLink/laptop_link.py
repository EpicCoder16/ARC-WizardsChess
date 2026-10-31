"""Laptop-side UDP partner for the Pico Task 5 Wi-Fi link."""
import json
import socket

STATUSES = ("arrived", "moving", "lost", "battery_low")
MAX_DATAGRAM_BYTES = 512


def _valid_tile(tile):
    return (isinstance(tile, str) and len(tile) == 2
            and "a" <= tile[0] <= "h" and "1" <= tile[1] <= "8")


class LaptopLink:
    """Discover robot IDs, send goto commands, and receive robot status."""

    def __init__(self, host="0.0.0.0", port=12345, on_status=None):
        if not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        self.port = port
        self.on_status = on_status
        self.devices = {}
        self.latest_status = {}
        self._command_counter = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))

    def _send(self, message, address):
        payload = json.dumps(message).encode("utf-8")
        if len(payload) > MAX_DATAGRAM_BYTES:
            raise ValueError("JSON message exceeds {} bytes".format(MAX_DATAGRAM_BYTES))
        self.sock.sendto(payload, address)

    def receive_once(self, timeout=0.5):
        """Receive one datagram; return its message or None on timeout/invalid data."""
        self.sock.settimeout(timeout)
        try:
            payload, address = self.sock.recvfrom(MAX_DATAGRAM_BYTES)
        except socket.timeout:
            return None
        try:
            message = json.loads(payload.decode("utf-8"))
        except (ValueError, UnicodeError):
            return None
        if not isinstance(message, dict):
            return None

        robot_id = message.get("robot_id")
        if not isinstance(robot_id, str) or not robot_id:
            return None
        if message.get("type") == "hello":
            self.devices[robot_id] = address
            self._send({"type": "registered", "robot_id": robot_id, "protocol": 1},
                       address)
            return message
        if message.get("type") != "status":
            return None
        if self.devices.get(robot_id) != address:
            return None
        if message.get("status") not in STATUSES:
            return None
        self.latest_status[robot_id] = message
        if self.on_status is not None:
            self.on_status(message)
        return message

    def send_goto(self, robot_id, tile):
        """Send goto to a registered robot and return the command ID."""
        if not _valid_tile(tile):
            raise ValueError("tile must be a square such as 'f3'")
        if robot_id not in self.devices:
            raise KeyError("robot {!r} has not announced itself".format(robot_id))
        self._command_counter += 1
        command_id = "{}-{}".format(robot_id, self._command_counter)
        self._send({
            "robot_id": robot_id,
            "cmd": "goto",
            "tile": tile,
            "command_id": command_id,
        }, self.devices[robot_id])
        return command_id

    def close(self):
        self.sock.close()
