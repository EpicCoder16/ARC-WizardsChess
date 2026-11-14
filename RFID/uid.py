"""ISO14443-A UID selection for MFRC522; no card-memory authentication.

Transport: reader.transceive(data, tx_bits=0) -> (bytes, valid_bits) or None.
Collisions fail closed rather than choosing an arbitrary card from a tile.
"""
import time


class UIDReadError(OSError):
    pass


def crc_a(data):
    """ISO14443-A CRC, initial value 0x6363, low byte first."""
    crc = 0x6363
    for value in data:
        value ^= crc & 0xff
        value ^= (value << 4) & 0xff
        crc = ((crc >> 8) ^ (value << 8) ^ (value << 3) ^ (value >> 4)) & 0xffff
    return bytes((crc & 0xff, crc >> 8))


def read_uid(reader):
    """Return 4/7/10 UID bytes, None for no initial response, or UIDReadError.

    Operates only the supplied reader and always disables its antenna. Caller
    must disable the other reader first. WUPA also wakes a previously selected
    or halted card. Each scan starts with a field-off interval to reset cards.
    """
    try:
        reader.antenna(False)
        time.sleep_ms(3)
        reader.antenna(True)
        time.sleep_ms(5)
        response = reader.transceive(b'\x52', tx_bits=7)
        if response is None:
            return None
        atqa, bits = response
        if len(atqa) != 2 or bits != 16:
            raise UIDReadError('invalid ATQA')
        uid = bytearray()
        for command in (0x93, 0x95, 0x97):
            response = reader.transceive(bytes((command, 0x20)))
            if response is None:
                raise UIDReadError('card lost during anticollision')
            block, bits = response
            if len(block) != 5 or bits != 40:
                raise UIDReadError('invalid anticollision length')
            if block[0] ^ block[1] ^ block[2] ^ block[3] != block[4]:
                raise UIDReadError('UID BCC mismatch')
            frame = bytes((command, 0x70)) + block
            response = reader.transceive(frame + crc_a(frame))
            if response is None:
                raise UIDReadError('card lost during select')
            sak, bits = response
            if len(sak) != 3 or bits != 24 or crc_a(sak[:1]) != sak[1:]:
                raise UIDReadError('invalid SAK CRC or length')
            more = bool(sak[0] & 0x04)
            if more:
                if command == 0x97 or block[0] != 0x88:
                    raise UIDReadError('invalid UID cascade')
                uid.extend(block[1:4])
            else:
                if command != 0x97 and block[0] == 0x88:
                    raise UIDReadError('unexpected cascade tag')
                uid.extend(block[:4])
                return bytes(uid)
        raise UIDReadError('incomplete UID')
    finally:
        reader.antenna(False)
