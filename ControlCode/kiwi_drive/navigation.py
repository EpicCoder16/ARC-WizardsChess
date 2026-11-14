"""Explicit board coordinates and RFID-confirmed Task 5 goto callback."""
try:
    from .kiwi_drive import MotionError, _finite
except ImportError:
    from kiwi_drive import MotionError, _finite


def tile_coordinates(tile, origin_x, origin_y, spacing_mm, board_angle_rad):
    """a1 center is origin; files +x, ranks +y before board rotation."""
    import math
    if (not isinstance(tile, str) or len(tile) != 2 or
            tile[0] not in 'abcdefgh' or tile[1] not in '12345678'):
        raise ValueError('tile must be a1 through h8')
    if not all(_finite(v) for v in (origin_x, origin_y, spacing_mm, board_angle_rad)) or spacing_mm <= 0:
        raise ValueError('finite board geometry and positive spacing required')
    x = (ord(tile[0]) - ord('a')) * spacing_mm
    y = (int(tile[1]) - 1) * spacing_mm
    c, s = math.cos(board_angle_rad), math.sin(board_angle_rad)
    return origin_x + c*x - s*y, origin_y + s*x + c*y


class TileNavigator:
    def __init__(self, motion, rfid, coordinates, route):
        """route(tile) supplies obstacle-cleared tile waypoints, ending at tile.

        coordinates is an explicitly verified square -> (x_mm, y_mm) map.
        RFID centering corrects odometry at every waypoint. No route is guessed.
        """
        self.motion, self.rfid = motion, rfid
        self.coordinates, self.route = coordinates, route

    def on_goto(self, tile):
        try:
            points = list(self.route(tile))
            if not points or points[-1] != tile:
                raise MotionError('route must end at requested tile')
            # Validate the entire route before moving.
            targets = []
            for point in points:
                xy = self.coordinates[point]
                if len(xy) != 2 or not all(_finite(v) for v in xy):
                    raise ValueError('invalid tile coordinates')
                targets.append(xy)
            for point, xy in zip(points, targets):
                if self.motion.move_to(*xy) is not True:
                    return False
                if self.rfid.current_tile() != point:
                    return False
                if self.rfid.center_on_card() != point:
                    return False
            return True
        finally:
            self.motion.stop()
