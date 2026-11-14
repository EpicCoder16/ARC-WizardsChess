import unittest
from unittest.mock import patch
from RFID.uid import crc_a, read_uid, UIDReadError
import test_pico_readers as fixtures
from test_pico_readers import FakeReader


class ScriptedCard:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.active = False
        self.sent = []
    def antenna(self, on): self.active = on
    def transceive(self, data, tx_bits=0):
        assert self.active
        self.sent.append((data, tx_bits))
        result = next(self.replies)
        if isinstance(result, Exception): raise result
        return result


def block(data):
    return data + bytes((data[0]^data[1]^data[2]^data[3],)), 40


def sak(value):
    raw = bytes((value,))
    return raw + crc_a(raw), 24


class UIDTests(unittest.TestCase):
    def setUp(self):
        self.sleep = patch('RFID.uid.time.sleep_ms', create=True)
        self.sleep.start()
        self.addCleanup(self.sleep.stop)

    def test_crc_known_halt_vector(self):
        self.assertEqual(crc_a(b'\x50\x00'), b'\x57\xcd')

    def test_four_byte_uid_and_select_frame(self):
        card = ScriptedCard([(b'\x04\x00',16), block(b'\x01\x02\x03\x04'), sak(8)])
        self.assertEqual(read_uid(card),b'\x01\x02\x03\x04')
        self.assertEqual(card.sent[0], (b'\x52',7))
        frame = b'\x93\x70\x01\x02\x03\x04\x04'
        self.assertEqual(card.sent[2][0],frame+crc_a(frame))
        self.assertFalse(card.active)

    def test_seven_and_ten_byte_uids(self):
        for uid in (bytes(range(1,8)), bytes(range(1,11))):
            chunks = [b'\x88'+uid[:3]]
            if len(uid)==10: chunks.append(b'\x88'+uid[3:6])
            chunks.append(uid[-4:])
            replies = [(b'\x44\x00',16)]
            for i,chunk in enumerate(chunks): replies += [block(chunk), sak(4 if i<len(chunks)-1 else 0)]
            card = ScriptedCard(replies)
            self.assertEqual(read_uid(card),uid)
            self.assertFalse(card.active)

    def test_no_card_and_malformed_or_lost_card(self):
        card=ScriptedCard([None])
        self.assertIsNone(read_uid(card))
        cases = [ [(b'\x04',8)],
                  [(b'\x04\x00',16),None],
                  [(b'\x04\x00',16),(b'\x01\x02\x03\x04\x00',40)],
                  [(b'\x04\x00',16),block(b'\x01\x02\x03\x04'),None],
                  [(b'\x04\x00',16),block(b'\x01\x02\x03\x04'),(b'\x08\x00\x00',24)],
                  [(b'\x04\x00',16),block(b'\x01\x02\x03\x04'),sak(4)],
                  [UIDReadError('collision')], [OSError('SPI failure')]]
        for replies in cases:
            card=ScriptedCard(replies)
            with self.assertRaises(OSError): read_uid(card)
            self.assertFalse(card.active)


class FastReaderTests(unittest.TestCase):
    setUp = fixtures.ReaderAdapterTests.setUp
    def test_fast_reads_fixed_gain_without_scan_and_preserves_disagreement(self):
        gains=[]
        for reader in (self.pair.left,self.pair.right):
            reader.set_gain=lambda value,r=reader: gains.append((r.name,value))
            reader.lowest_gain=lambda: self.fail('fast path ran gain scan')
        def uid(reader):
            reader.antenna(True)
            return b'left' if reader.name=='left' else b'right'
        self.pair.read_uid=uid
        self.assertEqual(self.pair.read_pair_fast(),(b'left',b'right'))
        self.assertEqual(gains,[('left',7),('right',7)])
        self.assertIsNone(FakeReader.active)

    def test_fast_no_card_and_failure_cleanup(self):
        for reader in (self.pair.left,self.pair.right): reader.set_gain=lambda gain: None
        self.pair.read_uid=lambda reader: None
        self.assertEqual(self.pair.read_pair_fast(),(None,None))
        def fail(reader):
            reader.antenna(True)
            raise UIDReadError('collision')
        self.pair.read_uid=fail
        with self.assertRaises(UIDReadError): self.pair.read_pair_fast()
        self.assertIsNone(FakeReader.active)


class TransportTests(unittest.TestCase):
    setUp = fixtures.ReaderAdapterTests.setUp
    def transport(self, irq=0x20, error=0, count=2, last_bits=0):
        reader=self.module.Reader.__new__(self.module.Reader)
        writes=[]
        reader.w=lambda reg,value: writes.append((reg,value))
        fifo=iter([0x04,0x00])
        reader.r=lambda reg: next(fifo) if reg==9 else {4:irq,6:error,10:count,12:last_bits,14:0}.get(reg,0)
        return reader,writes

    def test_transport_fifo_and_cleanup(self):
        reader,writes=self.transport()
        with patch.object(self.module.time,'ticks_ms',return_value=0,create=True):
            self.assertEqual(reader.transceive(b'\x52',7),(b'\x04\x00',16))
        self.assertEqual(writes[-2:],[(13,7),(1,0)])

    def test_transport_errors_collision_overflow_and_timeouts(self):
        for error in (1,2,4,8,16,64,128):
            reader,writes=self.transport(error=error)
            with patch.object(self.module.time,'ticks_ms',return_value=0,create=True):
                with self.assertRaises(UIDReadError): reader.transceive(b'\x52',7)
            self.assertEqual(writes[-1],(1,0))
        reader,writes=self.transport(irq=1)
        with patch.object(self.module.time,'ticks_ms',return_value=0,create=True):
            self.assertIsNone(reader.transceive(b'\x52',7))
        reader,writes=self.transport(irq=0)
        with patch.object(self.module.time,'ticks_ms',side_effect=[0,9],create=True), patch.object(self.module.time,'ticks_diff',side_effect=lambda a,b:a-b,create=True):
            self.assertIsNone(reader.transceive(b'\x52',7))
        self.assertEqual(writes[-1],(1,0))


class BenchLoggerTests(unittest.TestCase):
    def test_logger_records_mismatch_and_error_and_disables_antennas(self):
        import io
        from contextlib import redirect_stdout
        from RFID.bench_uid import run
        class Readers:
            calls = 0
            off = 0
            def read_pair_fast(self):
                self.calls += 1
                if self.calls == 2:
                    raise UIDReadError('collision')
                return b'\x01\x02\x03\x04', b'\x05\x06\x07\x08'
            def _off(self): self.off += 1
        readers = Readers()
        output = io.StringIO()
        with patch('RFID.bench_uid.time.ticks_ms', return_value=0, create=True), patch('RFID.bench_uid.time.ticks_diff', return_value=0, create=True), redirect_stdout(output):
            run('01020304', 10, trials=2, readers=readers)
        self.assertEqual(readers.off,2)
        self.assertIn('01020304,05060708,,,False,0,',output.getvalue())
        self.assertIn('collision',output.getvalue())
