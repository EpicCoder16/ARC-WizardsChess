"""MicroPython bench firmware. No motor, encoder, or RFID modules are loaded."""
import time

try:
    from .wifi_link import WifiLink
except ImportError:
    from wifi_link import WifiLink


class BenchMotion:
    def __init__(self, result="success", led=None):
        if result not in ("success", "failure", "error"):
            raise ValueError("BENCH_RESULT must be success, failure, or error")
        self.result = result
        self.led = led

    def __call__(self, tile):
        print("BENCH ONLY: received goto", tile, "- no physical movement")
        if self.led is not None:
            self.led.value(1)
        try:
            time.sleep(0.25)
            if self.result == "error":
                raise RuntimeError("bench_simulated_error")
            return self.result == "success"
        finally:
            if self.led is not None:
                self.led.value(0)


def run():
    import wifi_config as config
    from machine import Pin

    led = Pin("LED", Pin.OUT)
    motion = BenchMotion(getattr(config, "BENCH_RESULT", "success"), led)
    link = WifiLink(config.SSID, config.PASSWORD, config.ROBOT_ID,
                    config.LAPTOP_IP, port=getattr(config, "PORT", 12345),
                    on_goto=motion, mode="bench")
    print("Wi-Fi BENCH mode: arrived is simulated; motors are not controlled.")
    try:
        while True:
            try:
                print("Pico IP:", link.connect())
                link.run_forever()
            except OSError as error:
                print("Network error:", error, "- retrying in 3 seconds")
            finally:
                link.close()
                led.value(0)
            time.sleep(3)
    except KeyboardInterrupt:
        print("Bench test stopped")
    finally:
        link.close()
        led.value(0)
