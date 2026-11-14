"""Pico 2 W reader adapter based on the supplied two_reader_test.py.

Includes bounded MFRC522 transceive and default ISO14443-A UID selection.
"""
from machine import Pin, SPI
import time
from RFID.uid import read_uid as default_read_uid, UIDReadError

GAINS = ((0, 18), (1, 23), (4, 33), (5, 38), (6, 43), (7, 48))
SPI_BUS = 0
SPI_BAUDRATE = 1_000_000
SPI_PINS = (18, 19, 16)  # SCK, MOSI, MISO
LEFT_CS_PIN = 20
RIGHT_CS_PIN = 17
RESET_PIN = 22


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

    def transceive(self, data, tx_bits=0, timeout_ms=8):
        """Bounded 106 kbit/s exchange, with explicit collision/error rejection.

        CRC_A is checked in uid.py, not automatically by the chip. None means
        timer expiry/no reply. SPI faults propagate; cleanup still runs.
        """
        if not 1 <= len(data) <= 64 or tx_bits not in range(8) or timeout_ms <= 0:
            raise ValueError('invalid transceive arguments')
        try:
            self.w(0x01, 0x00)  # Idle
            self.w(0x04, 0x7f)  # clear IRQs
            self.w(0x0a, 0x80)  # flush FIFO
            self.w(0x12, 0x00)  # TxMode: 106 kbit/s, software CRC
            self.w(0x13, 0x00)  # RxMode: software CRC
            self.w(0x0e, self.r(0x0e) & 0x7f)  # clear ValuesAfterColl
            self.w(0x0d, tx_bits)
            for value in data:
                self.w(0x09, value)
            self.w(0x01, 0x0c)  # Transceive
            self.w(0x0d, 0x80 | tx_bits)  # StartSend
            start = time.ticks_ms()
            while True:
                irq = self.r(0x04)
                error = self.r(0x06)
                if error & 0xdf:  # collision, parity, protocol, CRC, overflow, thermal/write
                    raise UIDReadError('MFRC522 receive error 0x%02x' % error)
                if irq & 0x20:  # RxIRq, not merely IdleIRq
                    count = self.r(0x0a) & 0x7f
                    if not 1 <= count <= 64:
                        raise UIDReadError('invalid receive FIFO length')
                    last_bits = self.r(0x0c) & 7
                    result = bytes(self.r(0x09) for _ in range(count))
                    bits = (count - 1) * 8 + last_bits if last_bits else count * 8
                    return result, bits
                if irq & 1 or time.ticks_diff(time.ticks_ms(), start) >= timeout_ms:
                    return None
        finally:
            try:
                self.w(0x0d, tx_bits)  # clear StartSend
            finally:
                self.w(0x01, 0x00)

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
    def __init__(self, read_uid=None):
        """read_uid(reader) must return a UID or None, managing that antenna.

        Defaults to the built-in anticollision/selection implementation.
        A custom callback remains supported for alternative reader protocols.
        Both antennas are switched off before and after each callback.
        """
        if read_uid is None:
            read_uid = default_read_uid
        if not callable(read_uid):
            raise ValueError('a UID-reading callback is required')
        self.read_uid = read_uid
        self.spi = SPI(SPI_BUS, baudrate=SPI_BAUDRATE, polarity=0, phase=0,
                       sck=Pin(SPI_PINS[0]), mosi=Pin(SPI_PINS[1]),
                       miso=Pin(SPI_PINS[2]))
        self.reset = Pin(RESET_PIN, Pin.OUT, value=1)
        self.left = Reader(self.spi, LEFT_CS_PIN)
        self.right = Reader(self.spi, RIGHT_CS_PIN)
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

    def read_pair_fast(self, gain_code=7):
        """UID-only (left, right), bytes or None; no minimum-gain scan.

        Different UIDs are preserved so callers can reject disagreement.
        Errors propagate; both antennas are disabled even on callback failure.
        This synchronous scan is bounded per exchange, not a motor scheduler.
        """
        if gain_code not in tuple(code for code, _ in GAINS):
            raise ValueError('unsupported receiver gain')
        try:
            values = []
            for reader in (self.left, self.right):
                self._off()
                reader.set_gain(gain_code)
                values.append(self.read_uid(reader))
            return tuple(values)
        finally:
            self._off()
