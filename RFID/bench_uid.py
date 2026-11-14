"""Motor-free Task 4b observation logger. Never runs on import."""
import time
from RFID.position import RFIDPosition


def run(expected_uid, offset_mm, trials=20, gain_scan=False, readers=None):
    """Print CSV to the Pico console. Offset is measured/provided by operator."""
    expected = RFIDPosition.uid_key(expected_uid)
    if not isinstance(trials, int) or trials < 1:
        raise ValueError('positive integer trial count required')
    if readers is None:
        from RFID.pico_readers import PicoReaders
        readers = PicoReaders()
    print('trial,offset_mm,left_uid,right_uid,left_gain_db,right_gain_db,match,elapsed_ms,error')
    for trial in range(1, trials + 1):
        start = time.ticks_ms()
        left = right = None
        lg = rg = ''
        error = ''
        try:
            if gain_scan:
                ls, rs = readers.read_pair()
                if ls: left, lg = ls
                if rs: right, rg = rs
            else:
                left, right = readers.read_pair_fast()
            left = RFIDPosition.uid_key(left) if left is not None else ''
            right = RFIDPosition.uid_key(right) if right is not None else ''
        except OSError as exc:
            error = str(exc).replace(',', ';').replace('\n', ' ')
        finally:
            readers._off()
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        match = not error and left == expected and right == expected
        print('%s,%s,%s,%s,%s,%s,%s,%s,%s' %
              (trial, offset_mm, left or '', right or '', lg, rg, match, elapsed, error))
