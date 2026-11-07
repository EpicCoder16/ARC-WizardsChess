# Task 5: Wi-Fi link

This module connects a Pico 2 W to the laptop over the same Wi-Fi network. It
uses short JSON UDP messages and replaces the older ESP-01 binary link for this
Pico 2 W path.

## Start here: Wi-Fi bench test (no motion software required)

This is the current runnable test path. It does not import the motor driver,
PID, Kiwi maths, or RFID code. On `goto`, it prints the target square and
lights the onboard LED for 0.25 seconds. Every announcement and status carries
`"mode":"bench"`. An `arrived` response in this mode is simulated and proves
only that the command/status exchange completed.

### Upload with Thonny

1. Use a **Raspberry Pi Pico 2 W** with the matching stable
   [MicroPython firmware](https://micropython.org/download/RPI_PICO2_W/).
   If firmware is needed, hold BOOTSEL while connecting USB, then copy the
   board's UF2 file onto the drive that appears. Select the Pico MicroPython
   interpreter and its USB port in Thonny.
2. Connect the laptop and Pico to the same 2.4 GHz hotspot. The handoff suggests
   a phone hotspot because campus Wi-Fi may block traffic between devices.
   Find the laptop's IPv4 address for that hotspot in its network settings.
3. Copy `wifi_config.example.py` locally to `wifi_config.py`. Fill in `SSID`,
   `PASSWORD`, `LAPTOP_IP`, and a unique `ROBOT_ID`. Leave `BENCH_RESULT` as
   `"success"`. The local credential file is ignored by Git.
4. In Thonny, save **these four files to the Pico's root**, keeping their names:
   `main.py`, `bench.py`, `wifi_link.py`, and your `wifi_config.py`.
   The example config and laptop files do not need to be uploaded. Do not copy
   the motor test scripts into `main.py`; this bench entry point controls only
   the onboard LED. Keep motor power disconnected during this test.
5. Run `main.py` or reboot. Thonny should show `Wi-Fi BENCH mode` and `Pico IP:`.
   It retries network connection errors every three seconds. Use Ctrl+C in the
   Thonny shell to stop the program and edit configuration.

### Run on the laptop

From the repository root, using Python 3 (no extra packages required):

```sh
python3 ControlCode/Task5_WiFiLink/laptop_test.py --robot-id white-king --tile f3
```

The command waits up to 15 seconds for registration, sends `goto`, prints
statuses, and exits successfully after `arrived`. Expect the Pico LED to flash,
`moving` then `arrived` with `mode: bench`, and `PASS` on the laptop. It retries
using the same command ID if a response is dropped; the cached result avoids
running the callback again. Use `--timeout 30` if more discovery time is needed.

For failure tests, change the Pico config to `BENCH_RESULT = "failure"` and
restart it, then run:

```sh
python3 ControlCode/Task5_WiFiLink/laptop_test.py --robot-id white-king --tile f3 --expect lost
```

Repeat with `BENCH_RESULT = "error"`; expect `lost` with
`bench_simulated_error`. Restore `success` afterward. These tests need no PID
or Kiwi maths. `battery_low` has a send API, but no battery sensor is sampled
by this bench firmware. RFID UID/tile-crossing telemetry is not connected yet.

If registration times out, check the configured laptop IPv4 address, hotspot
client isolation, and permission for Python to receive UDP through the laptop
firewall. Stop the old `PathPlanning/PythonServer.py`: it also uses UDP 12345
and speaks a different protocol. If changing ports with `--port`, change `PORT`
on the Pico to match. Give each Pico a unique ID. After a hotspot IP change,
update `LAPTOP_IP` and restart the Pico.

The bench runner reconnects after Wi-Fi errors, keeping its in-memory completed
command cache. The cache holds only the latest 16 commands and is lost on Pico
reset; this is not persistent exactly-once delivery. Do not use the bench
runner to control motors. Real motion requires a separate startup entry point
with Task 3's completed movement callback and the hardware checks below.

## Protocol

The Pico announces itself every few seconds:

```json
{"type":"hello","robot_id":"white-king","protocol":1}
```

The laptop addresses a robot by its unique `robot_id`:

```json
{"robot_id":"white-king","cmd":"goto","tile":"f3","command_id":"white-king-1"}
```

The Pico reports `moving`, then `arrived` after its movement callback
finishes. Invalid tiles or callback failures report `lost`. Battery monitoring
can call `send_status("battery_low", battery_pct=18)`.

All datagrams are limited to 512 bytes. The Pico ignores commands that do not
come from the configured laptop IP. Supply the Wi-Fi password at runtime; the
module contains no network credentials. Robot IDs are limited to 64
characters, command IDs to 128 characters, and only protocol version 1 hello
messages are registered by the laptop.

The laptop generates random command IDs so restarting it does not reuse the
Pico's recent command IDs. Passing an existing ID to `send_goto` retries that
same command. Reusing it with a different square is ignored. Terminal replies
are cached with their error details before transmission, so a failed send can
be recovered by retrying. UDP messages are not encrypted or authenticated;
use a trusted test network. The configured source IP and robot ID filter
misaddressed traffic but do not authenticate a sender.

## Pico interface for future real movement

Copy `wifi_link.py` to the Pico 2 W. Pass Task 3's blocking movement adapter
as `on_goto`. The callback receives algebraic notation such as `"f3"`; it
should convert that tile to Task 3 coordinates, finish the move, and return
`False` only when movement fails.

```python
from wifi_link import WifiLink

def move_to_tile(tile):
    # Connect this to Task 3's tile-to-coordinate and drive functions.
    return task3_move_to_tile(tile)

link = WifiLink(
    ssid="your-hotspot-name",
    password="your-hotspot-password",
    robot_id="white-king",
    laptop_ip="192.168.4.1",
    port=12345,
    on_goto=move_to_tile,
)
print("Pico IP:", link.connect())
link.run_forever()
```

If Task 3 is not connected yet, omit `on_goto` during initial link testing.
The module reports `moving` followed by `lost` with
`motion_handler_unavailable`, rather than claiming arrival.

## Laptop interface and test

Run this on the laptop while it is connected to the same hotspot. Use the
laptop's actual hotspot IP in the Pico configuration.

```python
from laptop_link import LaptopLink

link = LaptopLink(host="0.0.0.0", port=12345)
try:
    while "white-king" not in link.devices:
        print("waiting:", link.receive_once(timeout=1.0))

    command_id = link.send_goto("white-king", "f3")
    while True:
        message = link.receive_once(timeout=1.0)
        if message is not None:
            print(message)
        if (
            message is not None
            and message.get("command_id") == command_id
            and message.get("status") in ("arrived", "lost")
        ):
            break
finally:
    link.close()
```

For the acceptance test, connect `on_goto` to Task 3's real move routine,
send `goto` to a safe square such as `f3`, and confirm the laptop receives
`moving` followed by `arrived`. A callback returning `False` should produce
`lost`. If battery monitoring is available, send a `battery_low` status and
confirm it appears on the laptop. Physical movement still depends on Task 3 and
the robot hardware.
