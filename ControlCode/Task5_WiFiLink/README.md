# Task 5: Wi-Fi link

This module connects a Pico 2 W to the laptop over the same Wi-Fi network. It
uses short JSON UDP messages and replaces the older ESP-01 binary link for this
Pico 2 W path.

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

## Pico interface

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
