"""Task 5 JSON/UDP link for a Raspberry Pi Pico 2 W."""
import errno
import json
import socket
import time

try:
    import network
except ImportError:
    network = None

STATUSES = ("arrived", "moving", "lost", "battery_low")
MAX_DATAGRAM_BYTES = 512
MAX_ROBOT_ID_CHARS = 64
MAX_COMMAND_ID_CHARS = 128
RECENT_COMMAND_LIMIT = 16


def _now_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _elapsed_ms(start):
    now = _now_ms()
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, start)
    return now - start


def _sleep_ms(delay):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(delay)
    else:
        time.sleep(delay / 1000.0)


def _would_block(error):
    code = error.args[0] if error.args else None
    again = getattr(errno, "EAGAIN", 11)
    would_block = getattr(errno, "EWOULDBLOCK", again)
    return code == again or code == would_block


def _valid_tile(tile):
    return (isinstance(tile, str) and len(tile) == 2
            and "a" <= tile[0] <= "h" and "1" <= tile[1] <= "8")


def _valid_battery_pct(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and 0 <= value <= 100)


class WifiLink:
    """Pico endpoint. on_goto(tile) must finish the move and return False on failure."""

    def __init__(self, ssid, password, robot_id, laptop_ip, port=12345,
                 on_goto=None, local_port=None, announce_interval_ms=2000,
                 socket_module=socket, network_module=network):
        if (not isinstance(robot_id, str) or not robot_id
                or len(robot_id) > MAX_ROBOT_ID_CHARS):
            raise ValueError("robot_id must contain 1 to {} characters".format(
                MAX_ROBOT_ID_CHARS))
        if not isinstance(laptop_ip, str) or not laptop_ip:
            raise ValueError("laptop_ip must be a non-empty IPv4 address")
        if not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if local_port is None:
            local_port = port
        if not isinstance(local_port, int) or not 0 <= local_port <= 65535:
            raise ValueError("local_port must be between 0 and 65535")
        if announce_interval_ms <= 0:
            raise ValueError("announce_interval_ms must be positive")

        self.ssid = ssid
        self.password = password
        self.robot_id = robot_id
        self.laptop_ip = laptop_ip
        self.port = port
        self.local_port = local_port
        self.on_goto = on_goto
        self.announce_interval_ms = announce_interval_ms
        self.socket_module = socket_module
        self.network_module = network_module
        self.wlan = None
        self.sock = None
        self._last_announce_ms = None
        self._recent_command_ids = []
        self._command_status = {}

    def connect(self, timeout_ms=15000):
        """Join Wi-Fi, open UDP, announce the robot, and return its IPv4 address."""
        if self.network_module is None:
            raise RuntimeError("MicroPython network module is required on the Pico")
        self.wlan = self.network_module.WLAN(self.network_module.STA_IF)
        self.wlan.active(True)
        if not self.wlan.isconnected():
            self.wlan.connect(self.ssid, self.password)
        started = _now_ms()
        while not self.wlan.isconnected():
            if _elapsed_ms(started) >= timeout_ms:
                raise OSError("Timed out connecting to Wi-Fi")
            _sleep_ms(100)

        self.sock = self.socket_module.socket(
            self.socket_module.AF_INET, self.socket_module.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.local_port))
        self.sock.setblocking(False)
        self._announce(force=True)
        return self.wlan.ifconfig()[0]

    def _send(self, message):
        if self.sock is None:
            raise RuntimeError("Call connect() before sending messages")
        payload = json.dumps(message).encode("utf-8")
        if len(payload) > MAX_DATAGRAM_BYTES:
            raise ValueError("JSON message exceeds {} bytes".format(MAX_DATAGRAM_BYTES))
        self.sock.sendto(payload, (self.laptop_ip, self.port))

    def _announce(self, force=False):
        now = _now_ms()
        if (force or self._last_announce_ms is None
                or _elapsed_ms(self._last_announce_ms) >= self.announce_interval_ms):
            self._send({"type": "hello", "robot_id": self.robot_id, "protocol": 1})
            self._last_announce_ms = now

    def _remember_status(self, command_id, status):
        if command_id is None:
            return
        if command_id not in self._command_status:
            self._recent_command_ids.append(command_id)
        self._command_status[command_id] = status
        while len(self._recent_command_ids) > RECENT_COMMAND_LIMIT:
            expired = self._recent_command_ids.pop(0)
            self._command_status.pop(expired, None)

    def send_status(self, status, command_id=None, tile=None, detail=None,
                    battery_pct=None):
        if status not in STATUSES:
            raise ValueError("status must be one of {}".format(", ".join(STATUSES)))
        if (command_id is not None
                and (not isinstance(command_id, str) or not command_id
                     or len(command_id) > MAX_COMMAND_ID_CHARS)):
            raise ValueError("command_id must contain 1 to {} characters".format(
                MAX_COMMAND_ID_CHARS))
        if tile is not None and not _valid_tile(tile):
            raise ValueError("tile must be a square such as 'f3'")
        if battery_pct is not None and not _valid_battery_pct(battery_pct):
            raise ValueError("battery_pct must be a number from 0 to 100")
        message = {"type": "status", "robot_id": self.robot_id, "status": status}
        if command_id is not None:
            message["command_id"] = command_id
        if tile is not None:
            message["tile"] = tile
        if detail is not None:
            message["detail"] = str(detail)[:120]
        if battery_pct is not None:
            message["battery_pct"] = battery_pct
        self._send(message)
        if command_id is not None:
            self._remember_status(command_id, status)

    def _handle_command(self, payload):
        try:
            message = json.loads(payload.decode("utf-8"))
        except (ValueError, UnicodeError):
            return False
        if not isinstance(message, dict):
            return False
        target_id = message.get("robot_id")
        if target_id != self.robot_id:
            return False
        if message.get("cmd") != "goto":
            return False

        command_id = message.get("command_id")
        if (command_id is not None
                and (not isinstance(command_id, str) or not command_id
                     or len(command_id) > MAX_COMMAND_ID_CHARS)):
            return False
        tile = message.get("tile")
        if not _valid_tile(tile):
            self.send_status("lost", command_id=command_id, detail="invalid_tile")
            return False

        # A retried UDP packet must not make the robot repeat the same recent move.
        if command_id is not None and command_id in self._command_status:
            self.send_status(self._command_status[command_id],
                             command_id=command_id, tile=tile)
            return True

        self.send_status("moving", command_id=command_id, tile=tile)
        if self.on_goto is None:
            self.send_status("lost", command_id=command_id, tile=tile,
                             detail="motion_handler_unavailable")
            return False
        try:
            result = self.on_goto(tile)
        except Exception as error:
            self.send_status("lost", command_id=command_id, tile=tile,
                             detail=error)
            return False
        if result is False:
            self.send_status("lost", command_id=command_id, tile=tile,
                             detail="motion_handler_failed")
            return False
        self.send_status("arrived", command_id=command_id, tile=tile)
        return True

    def poll(self):
        """Handle at most one datagram; returns True when a goto is processed."""
        if self.sock is None:
            raise RuntimeError("Call connect() before polling")
        self._announce()
        try:
            # Read one byte beyond the limit so oversized UDP packets cannot be
            # accepted after recvfrom silently truncates a valid JSON prefix.
            payload, address = self.sock.recvfrom(MAX_DATAGRAM_BYTES + 1)
        except OSError as error:
            if _would_block(error):
                return False
            raise
        if len(payload) > MAX_DATAGRAM_BYTES:
            return False
        if address[0] != self.laptop_ip:
            return False
        return self._handle_command(payload)

    def run_forever(self, idle_sleep_ms=20):
        while self.sock is not None:
            self.poll()
            _sleep_ms(idle_sleep_ms)

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        if self.wlan is not None:
            self.wlan.disconnect()
            self.wlan = None
