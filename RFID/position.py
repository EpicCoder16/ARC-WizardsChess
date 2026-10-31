"""RFID centering logic. Hardware and motion are supplied by the integration layer."""

class CenteringError(Exception):
    pass


class RFIDPosition:
    def __init__(self, readers, motion, tiles, step_mm=2, max_travel_mm=100,
                 gain_tolerance=0, max_lateral_steps=40):
        # tiles maps normalized hexadecimal UID strings to (square, x_mm, y_mm).
        if step_mm <= 0 or max_travel_mm < step_mm or max_lateral_steps < 1:
            raise ValueError("invalid movement limits")
        if gain_tolerance < 0:
            raise ValueError("gain tolerance must be nonnegative")
        self.readers, self.motion = readers, motion
        self.tiles = {self.uid_key(uid): value for uid, value in tiles.items()}
        self.step = step_mm
        self.max_travel = max_travel_mm
        self.gain_tolerance = gain_tolerance
        self.max_lateral_steps = max_lateral_steps
        self._last_offset = None

    @staticmethod
    def uid_key(uid):
        if isinstance(uid, (bytes, bytearray)):
            return ''.join('%02X' % b for b in uid)
        if isinstance(uid, str):
            value = uid.replace(':', '').replace(' ', '').replace('-', '').upper()
            if not value or len(value) % 2 or any(c not in '0123456789ABCDEF' for c in value):
                raise ValueError('UID must be hexadecimal bytes')
            return value
        raise ValueError('UID must be bytes or a hexadecimal string')

    def _read(self):
        # read_pair() must turn off one antenna before enabling the other.
        # Each result is None or (uid, minimum_gain), and both antennas are off on return.
        left, right = self.readers.read_pair()
        found = []
        for sample in (left, right):
            found.append(None if sample is None else (self.uid_key(sample[0]), sample[1]))
        if found[0] and found[1] and found[0][0] != found[1][0]:
            raise CenteringError('readers see different cards')
        return found

    def _uid(self, samples):
        sample = samples[0] or samples[1]
        return sample[0] if sample else None

    def current_tile(self):
        uid = self._uid(self._read())
        entry = self.tiles.get(uid)
        return entry[0] if entry else None

    def card_offset(self):
        """Last measured forward offset and gain difference, or None before a scan.

        Lateral offset is a gain difference, not a calibrated distance, and is
        reported separately; longitudinal estimate is relative to the scan start.
        """
        return self._last_offset

    def _move(self, right, forward):
        # Blocking, relative robot-frame movement, with its own timeout/fault stop.
        self.motion.move_relative(right, forward)

    def _present(self, uid):
        seen = self._uid(self._read())
        if seen is not None and seen != uid:
            raise CenteringError('different card encountered during scan')
        return seen == uid

    def _lateral(self, uid):
        for _ in range(self.max_lateral_steps):
            left, right = self._read()
            if self._uid((left, right)) != uid:
                raise CenteringError('card lost during lateral centering')
            if not left or not right:
                raise CenteringError('both readers must see the card for lateral centering')
            diff = left[1] - right[1]
            if abs(diff) <= self.gain_tolerance:
                return diff
            # Lower gain means closer. Positive right motion moves toward R.
            self._move(-self.step if diff < 0 else self.step, 0)
        raise CenteringError('lateral centering exceeded step limit')

    def _edge(self, uid, direction):
        travelled = 0
        while travelled + self.step <= self.max_travel:
            self._move(0, direction * self.step)
            travelled += self.step
            if not self._present(uid):
                # Boundary lies between last present and first absent sample.
                return direction * (travelled - self.step / 2), direction * travelled
        raise CenteringError('card edge not found within travel limit')

    def center_on_card(self):
        """Center on a mapped card; return its square. Raises on uncertainty.

        set_pose is called only after successful centering, since detection alone
        does not establish the robot's exact coordinates.
        """
        self._last_offset = None
        try:
            uid = self._uid(self._read())
            if uid is None or uid not in self.tiles:
                raise CenteringError('no mapped card detected')
            gain_difference = self._lateral(uid)
            # Start from a confirmed detection. Scan to both edges and return to midpoint.
            positive_edge, positive_end = self._edge(uid, 1)
            self._move(0, -positive_end)
            if not self._present(uid):
                raise CenteringError('could not return to scan origin')
            negative_edge, negative_end = self._edge(uid, -1)
            midpoint = (positive_edge + negative_edge) / 2
            self._move(0, midpoint - negative_end)
            if not self._present(uid):
                raise CenteringError('card not detected at estimated midpoint')
            # Recheck lateral centering at the longitudinal midpoint.
            gain_difference = self._lateral(uid)
            square, x, y = self.tiles[uid]
            self._last_offset = {'forward_mm_from_scan_start': midpoint,
                                 'gain_difference': gain_difference}
            _, _, heading = self.motion.get_pose()
            self.motion.set_pose(x, y, heading)
            return square
        finally:
            self.motion.stop()

    def search_for_card(self, radius_steps=3):
        """Bounded square spiral; return mapped square or None. Motion stays in place."""
        if radius_steps < 0:
            raise ValueError('radius_steps must be nonnegative')
        try:
            tile = self.current_tile()
            if tile:
                return tile
            # Each ring has 8*r single-step movements, bounded by radius_steps.
            for radius in range(1, radius_steps + 1):
                self._move(self.step, 0)
                tile = self.current_tile()
                if tile:
                    return tile
                for dx, dy, count in ((0, 1, 2 * radius - 1), (-1, 0, 2 * radius),
                                      (0, -1, 2 * radius), (1, 0, 2 * radius)):
                    for _ in range(count):
                        self._move(dx * self.step, dy * self.step)
                        tile = self.current_tile()
                        if tile:
                            return tile
            return None
        finally:
            self.motion.stop()
