import importlib
import sys
import types
import unittest
from unittest.mock import patch


class FakeReader:
    active = None

    def __init__(self, name, gain):
        self.name, self.gain = name, gain

    def antenna(self, on):
        if on:
            if FakeReader.active is not None:
                raise AssertionError('two antennas active')
            FakeReader.active = self
        elif FakeReader.active is self:
            FakeReader.active = None

    def lowest_gain(self):
        self.antenna(True)
        try:
            return self.gain
        finally:
            self.antenna(False)


class ReaderAdapterTests(unittest.TestCase):
    def setUp(self):
        fake_machine = types.ModuleType('machine')
        fake_machine.Pin = object
        fake_machine.SPI = object
        with patch.dict(sys.modules, {'machine': fake_machine}):
            self.module = importlib.import_module('RFID.pico_readers')
        FakeReader.active = None
        self.pair = self.module.PicoReaders.__new__(self.module.PicoReaders)
        self.pair.left = FakeReader('left', 18)
        self.pair.right = FakeReader('right', 23)

    def test_pair_alternates_and_disables_antennas(self):
        calls = []
        def uid(reader):
            reader.antenna(True)
            calls.append(reader.name)
            return b'\x01\x02'
        self.pair.read_uid = uid
        self.assertEqual(self.pair.read_pair(), ((b'\x01\x02', 18), (b'\x01\x02', 23)))
        self.assertEqual(calls, ['left', 'right'])
        self.assertIsNone(FakeReader.active)

    def test_callback_failure_disables_antennas(self):
        def fail(reader):
            reader.antenna(True)
            raise OSError('failed UID read')
        self.pair.read_uid = fail
        with self.assertRaises(OSError):
            self.pair.read_pair()
        self.assertIsNone(FakeReader.active)


if __name__ == '__main__':
    unittest.main()
