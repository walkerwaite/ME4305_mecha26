"""PI velocity feedback plus linear velocity feedforward, in volts.

u = K_P * (v_sp - vel) + integral_voltage + K_FF * v_sp.
K_P and K_FF: V/(rad/s); K_I: V/rad; dt: seconds.
K_FF is a velocity slope, not a static-friction/breakaway-voltage offset.
"""
from math import isfinite


class ClosedLoop:
    def __init__(self, V_min=-3.5, V_max=3.5):
        self.V_min, self.V_max = float(V_min), float(V_max)
        if not (isfinite(self.V_min) and isfinite(self.V_max)
                and self.V_min < 0 < self.V_max):
            raise ValueError("Voltage limits must bracket zero")
        self.reset()

    def reset(self):
        self.integral_voltage = 0.0

    def run(self, vel, K_P, v_sp, K_I=0.0, K_FF=0.0, dt=0.0):
        if not all(isfinite(x) for x in (vel, K_P, v_sp, K_I, K_FF, dt)):
            raise ValueError("Controller inputs must be finite")
        if min(K_P, K_I, K_FF, dt) < 0:
            raise ValueError("Gains and dt must be nonnegative")
        error = v_sp - vel
        if K_I == 0:
            self.integral_voltage = 0.0
        base = K_P * error + K_FF * v_sp
        old = self.integral_voltage
        candidate = min(self.V_max, max(self.V_min, old + K_I * error * dt))
        requested = base + candidate
        # Conditional integration: reject increments that push farther into
        # saturation; allow increments that unwind the existing integral.
        if not ((requested > self.V_max and candidate > old)
                or (requested < self.V_min and candidate < old)):
            self.integral_voltage = candidate
        return min(self.V_max, max(self.V_min, base + self.integral_voltage))
