"""Explicit UID-to-square registration and JSON persistence for MicroPython."""
import json
from RFID.position import RFIDPosition


def register(tiles, uid, square, x_mm, y_mm):
    uid = RFIDPosition.uid_key(uid)
    if (not isinstance(square, str) or len(square) != 2 or
            square[0] not in 'abcdefgh' or square[1] not in '12345678'):
        raise ValueError('square must be a1 through h8')
    if uid in tiles and tuple(tiles[uid]) != (square, x_mm, y_mm):
        raise ValueError('UID already registered; remove or review the old mapping')
    for other_uid, entry in tiles.items():
        if other_uid != uid and entry[0] == square:
            raise ValueError('square already registered to another UID')
    tiles[uid] = (square, x_mm, y_mm)


def save(tiles, path):
    # Write to a temporary file before replacing the previous mapping.
    import os
    temporary = path + '.tmp'
    with open(temporary, 'w') as stream:
        json.dump(tiles, stream)
    os.rename(temporary, path)


def load(path):
    with open(path) as stream:
        raw = json.load(stream)
    tiles = {}
    for uid, entry in raw.items():
        if len(entry) != 3:
            raise ValueError('invalid tile entry')
        register(tiles, uid, entry[0], entry[1], entry[2])
    return tiles
