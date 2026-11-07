
class PIDController:
    def __init__(self, kp=0.005, ki=0.0, kd=0.0):
        # PID tuning values (temporary)
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.last_error = None

    def update(self, target_speed, actual_speed, dt):
        # dt is the time between updates in seconds
        if dt <= 0:
            raise ValueError("Time must be positive")

        # Stop when target speed is zero
        if target_speed == 0:
            self.reset()
            return 0.0

        # Difference between desired and actual speed
        error = target_speed - actual_speed

        # Proportional correction
        p = self.kp * error

        # Integral correction
        new_integral = self.integral + error * dt
        i = self.ki * new_integral

        # Derivative correction
        if self.last_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.last_error) / dt

        d = self.kd * derivative

        # Calculate motor power
        power = p + i + d

        # Limit power between -1.0 and 1.0
        power = max(-1.0, min(1.0, power))

        # Prevent integral windup when saturated
        if -1.0 < p + i + d < 1.0:
            self.integral = new_integral

        self.last_error = error

        return power

    def reset(self):
        self.integral = 0.0
        self.last_error = None


# Test without hardware
if __name__ == "__main__":
    pid = PIDController()

    print("Starting PID tests...")

    print("Target 100, actual 0:",
          round(pid.update(100, 0, 0.02), 3))

    print("Target 100, actual 60:",
          round(pid.update(100, 60, 0.02), 3))

    print("Target 100, actual 120:",
          round(pid.update(100, 120, 0.02), 3))

    print("Target 0, actual 0:",
          round(pid.update(0, 0, 0.02), 3))

    print("PID tests complete!")
