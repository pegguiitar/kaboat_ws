"""Pure WASD mixing and ramping for the laptop keyboard teleop client."""

import math


MAX_THRUST_N = 118.6
FORWARD = 12.0 / MAX_THRUST_N
REVERSE = 6.0 / MAX_THRUST_N
TURN = 3.0 / MAX_THRUST_N


def command_for_keys(keys, forward=FORWARD, reverse=REVERSE, turn=TURN):
    """Return (linear, angular) with independent forward and turn keys."""
    keys = {key.lower() for key in keys}
    linear = (forward if 'w' in keys and 's' not in keys else
              -reverse if 's' in keys and 'w' not in keys else 0.0)
    angular = (turn if 'a' in keys and 'd' not in keys else
               -turn if 'd' in keys and 'a' not in keys else 0.0)
    return linear, angular


def approach(current, target, max_change):
    if not (math.isfinite(current) and math.isfinite(target)
            and math.isfinite(max_change) and max_change >= 0.0):
        raise ValueError('nonfinite teleop ramp input')
    return current + max(-max_change, min(max_change, target - current))


class SmoothAxes:
    def __init__(self, linear_rate=0.3, angular_rate=0.15):
        if linear_rate <= 0.0 or angular_rate <= 0.0:
            raise ValueError('ramp rates must be positive')
        self.linear_rate = float(linear_rate)
        self.angular_rate = float(angular_rate)
        self.linear = 0.0
        self.angular = 0.0

    def step(self, keys, dt):
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError('dt must be nonnegative and finite')
        target_linear, target_angular = command_for_keys(keys)
        self.linear = approach(
            self.linear, target_linear, self.linear_rate * dt)
        self.angular = approach(
            self.angular, target_angular, self.angular_rate * dt)
        return self.linear, self.angular

    def reset(self):
        self.linear = 0.0
        self.angular = 0.0
