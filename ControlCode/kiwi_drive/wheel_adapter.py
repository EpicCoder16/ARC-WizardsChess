"""Schedule the Task 2 update(dt) API and provide immediate Task 1 stop."""
try:
    from .kiwi_drive import _Clock, _finite
except ImportError:
    from kiwi_drive import _Clock, _finite


class WheelAdapter:
    def __init__(self, controller, stop_all, clock=None, period_ms=20):
        if not callable(stop_all) or not _finite(period_ms) or period_ms < 1:
            raise ValueError('stop callback and positive control period required')
        self.controller = controller
        self.stop_all = stop_all
        self.clock = clock or _Clock()
        self.period_ms = period_ms
        self.previous = self.clock.now_ms()

    def get_wheel_distance(self, wheel):
        return self.controller.get_wheel_distance(wheel)

    def set_wheel_speed(self, wheel, speed):
        if not _finite(speed):
            raise ValueError('speed must be finite')
        self.controller.set_wheel_speed(wheel, speed)

    def service(self):
        now = self.clock.now_ms()
        elapsed = self.clock.diff_ms(now, self.previous)
        if elapsed >= self.period_ms:
            try:
                self.controller.update(elapsed / 1000.0)
            except BaseException:
                self.stop()
                raise
            self.previous = now
            return True
        return False

    def stop(self):
        # Always cut motor output even if a target update or PID reset fails.
        error = None
        try:
            for wheel in (1, 2, 3):
                try:
                    self.controller.set_wheel_speed(wheel, 0)
                except Exception as exc:
                    error = exc
            for pid in self.controller.pids.values():
                pid.reset()
        finally:
            self.stop_all()
            self.previous = self.clock.now_ms()
        if error is not None:
            raise error
