"""Pure WASD mixing and ramping for the laptop keyboard teleop client."""

import math


# 정규화 추력 비율(0~1). 예전 값(전진 12/118.6, 후진 6/118.6, 회전 3/118.6)은
# 회전 2.5% 가 ESC 시동 구간 근처라 배가 돌지 않았다. 다른 수조 시험(직선·LOS 30%)
# 수준으로 올린다. 젯슨 thruster_driver 의 teleop_max_* 가 같은 값으로 다시 자른다.
FORWARD = 0.30
REVERSE = 0.15
TURN = 0.25


def command_for_keys(keys, forward=FORWARD, reverse=REVERSE, turn=TURN):
    """Return (linear, angular) for the pressed keys.

    A/D 는 **제자리 회전**이다: 전진 0, 좌우 모터를 반대로(좌 -turn, 우 +turn).
    A/D 가 눌려 있으면 W/S 보다 우선한다. A 와 D 를 같이 누르면 회전 없음.
    """
    keys = {key.lower() for key in keys}
    turn_dir = (1.0 if 'a' in keys and 'd' not in keys else
                -1.0 if 'd' in keys and 'a' not in keys else 0.0)
    if turn_dir:
        return 0.0, turn_dir * turn
    linear = (forward if 'w' in keys and 's' not in keys else
              -reverse if 's' in keys and 'w' not in keys else 0.0)
    return linear, 0.0


def approach(current, target, max_change):
    if not (math.isfinite(current) and math.isfinite(target)
            and math.isfinite(max_change) and max_change >= 0.0):
        raise ValueError('nonfinite teleop ramp input')
    return current + max(-max_change, min(max_change, target - current))


class SmoothAxes:
    # 0.5 초 안에 목표 출력에 도달 (출력을 올렸으므로 예전 0.3 / 0.15 보다 빠르게).
    def __init__(self, linear_rate=0.6, angular_rate=0.5):
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
