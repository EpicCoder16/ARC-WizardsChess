"""Run from the laptop: discover a Pico, send goto, and check the response."""
import argparse
import json
import time

try:
    from .laptop_link import LaptopLink
except ImportError:
    from laptop_link import LaptopLink


def check_command(link, robot_id, tile, expected="arrived", timeout=15,
                  retry_interval=1):
    """Bound discovery and command waits; retry with the same command ID."""
    deadline = time.monotonic() + timeout
    while robot_id not in link.devices:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("No hello from {!r}; check IP, hotspot and firewall".format(robot_id))
        message = link.receive_once(timeout=min(0.25, remaining))
        if message is not None:
            print(json.dumps(message), flush=True)

    command_id = link.send_goto(robot_id, tile)
    print("Sent goto", tile, "command_id", command_id, flush=True)
    deadline = time.monotonic() + timeout
    next_retry = time.monotonic() + retry_interval
    while time.monotonic() < deadline:
        message = link.receive_once(timeout=min(0.25, max(0.001, deadline - time.monotonic())))
        if message is not None:
            print(json.dumps(message), flush=True)
            if (message.get("robot_id") == robot_id
                    and message.get("command_id") == command_id
                    and message.get("status") in ("arrived", "lost")):
                if message.get("mode") == "bench":
                    print("BENCH result only: no physical movement was verified.", flush=True)
                if message["status"] != expected:
                    raise RuntimeError("Expected {}, received {}".format(expected, message["status"]))
                return message
        if time.monotonic() >= next_retry:
            link.send_goto(robot_id, tile, command_id=command_id)
            next_retry = time.monotonic() + retry_interval
    raise TimeoutError("No terminal status for command " + command_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", default="white-king")
    parser.add_argument("--tile", default="f3")
    parser.add_argument("--port", type=int, default=12345)
    parser.add_argument("--timeout", type=float, default=15)
    parser.add_argument("--expect", choices=("arrived", "lost"), default="arrived")
    args = parser.parse_args()
    if not 0 < args.timeout < float("inf"):
        parser.error("--timeout must be positive and finite")
    link = None
    try:
        link = LaptopLink(port=args.port)
        check_command(link, args.robot_id, args.tile, args.expect, args.timeout)
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        print("FAIL:", error, flush=True)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if link is not None:
            link.close()
    print("PASS: command/status exchange completed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
