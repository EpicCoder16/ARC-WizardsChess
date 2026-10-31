"""Pico 2 W reader adapter based on the supplied two_reader_test.py.

UID acquisition is deliberately injected: the supplied test detects a card
but does not implement anticollision or read a UID.
"""
from machine import Pin, SPI
import time

GAINS = ((0, 18), (1, 23), (4, 33), (5, 38), (6, 43), (7, 48))


class Reader:
    def __init__(self, spi, cs):
        self.spi = spi
        self.cs = Pin(cs, Pin.OUT, value=1)

    def w(self, reg, val):
        self.cs.value(0)
        try:
            self.spi.write(bytes(((reg << 1) & 0x7E, val)))
        finally:
            self.cs.value(1)

    def r(self, reg):
        self.cs.value(0)
        try:
            self.spi.write(bytes((((reg << 1) & 0x7E) | 0x80,)))
            return self.spi.read(1)[0]
        finally:
            self.cs.value(1)

    def init(self):
        self.w(0x01, 0x0F)
        time.sleep_ms(50)
        for reg, value in ((0x2A, 0x8D), (0x2B, 0x3E), (0x2D, 30),
                           (0x2C, 0), (0x15, 0x40), (0x11, 0x3D)):
            self.w(reg, value)
        self.antenna(False)

    def version(self):
        return self.r(0x37)

    def antenna(self, on):
        value = self.r(0x14)
        self.w(0x14, (value | 0x03) if on else (value & 0xFC))

    def set_gain(self, code):
        value = self.r(0x26)
        self.w(0x26, (value & 0x8F) | (code << 4))

    def sees_card(self):
        for reg, value in ((0x01, 0x00), (0x04, 0x7F), (0x0A, 0x80),
                           (0x09, 0x52), (0x0D, 0x07), (0x01, 0x0C), (0x0D, 0x87)):
            self.w(reg, value)
        start = time.ticks_ms()
        while True:
            irq = self.r(0x04)
            if irq & 0x30:
                break
            if irq & 0x01 or time.ticks_diff(time.ticks_ms(), start) > 30:
                return False
        return not (self.r(0x06) & 0x1B) and self.r(0x0A) == 2

    def lowest_gain(self):
        try:
            self.antenna(True)
            time.sleep_ms(5)
            for code, db in GAINS:
                self.set_gain(code)
                hits = 0
                for _ in range(3):
                    self.antenna(False)
                    time.sleep_ms(3)
                    self.antenna(True)
                    time.sleep_ms(5)
                    if self.sees_card():
                        hits += 1
                if hits == 3:
                    return db
            return None
        finally:
            self.antenna(False)


class PicoReaders:
    def __init__(self, read_uid):
        """read_uid(reader) must return a UID or None, managing that antenna.

        It must implement card anticollision and selection for the actual cards.
        Both antennas are switched off before and after each callback.
        """
        if not callable(read_uid):
            raise ValueError('a UID-reading callback is required')
        self.read_uid = read_uid
        self.spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
                       sck=Pin(18), mosi=Pin(19), miso=Pin(16))
        self.reset = Pin(22, Pin.OUT, value=1)
        self.left = Reader(self.spi, 20)
        self.right = Reader(self.spi, 17)
        self.reset.value(0)
        time.sleep_ms(10)
        self.reset.value(1)
        time.sleep_ms(50)
        for reader in (self.left, self.right):
            reader.init()
            if reader.version() in (0x00, 0xFF):
                raise OSError('RFID reader not found; check wiring')

    def _off(self):
        try:
            self.left.antenna(False)
        finally:
            self.right.antenna(False)

    def _sample(self, reader):
        self._off()
        gain = reader.lowest_gain()
        if gain is None:
            return None
        self._off()
        uid = self.read_uid(reader)
        if uid is None:
            raise OSError('card detected but UID could not be read')
        return (uid, gain)

    def read_pair(self):
        try:
            left = self._sample(self.left)
            right = self._sample(self.right)
            return left, right
        finally:
            self._off()
