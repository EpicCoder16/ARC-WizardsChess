"""Keep the new motor and RFID pin assignments disjoint in software."""
import importlib
import sys
import types
import unittest
from unittest.mock import patch


class _FakePin:
    OUT = 1

    def __init__(self, number, mode=None, value=0):
        self.number = number
        self.state = value

    def value(self, new_value=None):
        if new_value is not None:
            self.state = new_value
        return self.state


class _FakePWM:
    def __init__(self, pin):
        self.pin = pin

    def freq(self, value):
        pass

    def duty_u16(self, value):
        pass


class HardwarePinMapTests(unittest.TestCase):
    def test_task1_motor_pins_do_not_overlap_task4_rfid_pins(self):
        machine = types.ModuleType("machine")
        machine.Pin = _FakePin
        machine.PWM = _FakePWM
        machine.SPI = object
        with patch.dict(sys.modules, {"machine": machine}):
            motor = importlib.import_module("ControlCode.motor_driver.motor_driver")
            readers = importlib.import_module("RFID.pico_readers")

        motor_pins = {
            pin for pin_group in motor._PIN_MAP.values() for pin in pin_group
        }
        rfid_pins = set(readers.SPI_PINS) | {
            readers.LEFT_CS_PIN,
            readers.RIGHT_CS_PIN,
            readers.RESET_PIN,
        }
        self.assertEqual(motor_pins & rfid_pins, set())


if __name__ == "__main__":
    unittest.main()
