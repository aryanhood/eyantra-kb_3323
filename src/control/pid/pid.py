# src/control/pid/pid.py
class PIDController:
    """
    Canonical PID controller (algorithm only).
    No plant, no sensors, no actuators.
    """
    def __init__(
        self,
        kp, ki, kd,
        integ_min=-1e6, integ_max=1e6,
        deriv_alpha=None
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_d = 0.0

        self.integ_min = integ_min
        self.integ_max = integ_max
        self.deriv_alpha = deriv_alpha  # None → no filtering

    def update(self, error, dt):
        # Integral
        self.integrator += error * dt
        self.integrator = max(self.integ_min, min(self.integrator, self.integ_max))

        # Derivative
        raw_d = (error - self.prev_error) / dt if dt > 0 else 0.0
        if self.deriv_alpha is not None:
            d = self.deriv_alpha * self.prev_d + (1 - self.deriv_alpha) * raw_d
            self.prev_d = d
        else:
            d = raw_d

        self.prev_error = error

        return (
            self.kp * error
            + self.ki * self.integrator
            + self.kd * d
        )
