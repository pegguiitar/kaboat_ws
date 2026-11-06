"""노트북 키보드 텔레옵의 순수 로직 (ROS·GUI 없음).

속도는 -4 ~ +4 의 9단계다. 0 = 중립, +1~+4 = 전진, -1~-4 = 후진.
W 를 한 번 누를 때마다 1단계 올리고 S 는 1단계 내린다. 단계는 키를 떼도 유지된다.

A/D 는 누르고 있는 동안에만 좌우 추진기에 출력 차이를 준다(차동 조향).
  A(좌회전): 좌 = 속도 - 선회 폭, 우 = 속도 + 선회 폭
  D(우회전): 좌 = 속도 + 선회 폭, 우 = 속도 - 선회 폭
선회 폭은 젯슨 thrusters.yaml 의 teleop_turn_pwm_offset [µs] 이다.
예) 중립 + A → 좌 후진, 우 전진 = 제자리 좌회전. A/D 를 떼면 다시 좌우 같은 출력(직진).

노트북은 [속도 단계, 선회 방향] 을 보내고 젯슨이 PWM 으로 바꾼다.
"""

MAX_LEVEL = 4

# X11 은 키를 누르고 있으면 '떼기 → 누르기' 를 거의 동시에(수 ms) 반복해서 보낸다(자동반복).
# 떼기 뒤 이 시간 안에 같은 키가 다시 눌리면 자동반복으로 보고 무시한다 [s].
# 사람이 같은 키를 떼었다 다시 누르는 데는 이보다 오래 걸려 빠른 연타는 모두 센다.
AUTOREPEAT_GAP = 0.03


def clamp_level(value):
    return max(-MAX_LEVEL, min(MAX_LEVEL, int(value)))


def turn_from_keys(held):
    """+1 = 좌회전(A), -1 = 우회전(D), 0 = 직진. A 와 D 를 같이 누르면 0."""
    a, d = 'a' in held, 'd' in held
    return 1 if a and not d else -1 if d and not a else 0


def teleop_command(level, held):
    """현재 속도 단계와 누르고 있는 키 → (속도 단계, 선회 방향 +1/0/-1)."""
    return clamp_level(level), turn_from_keys(held)


class SpeedLevel:
    """W/S 로 오르내리는 전후진 속도 단계 (-4 ~ +4)."""

    def __init__(self):
        self.level = 0

    def up(self):
        self.level = clamp_level(self.level + 1)
        return self.level

    def down(self):
        self.level = clamp_level(self.level - 1)
        return self.level

    def stop(self):
        self.level = 0
        return self.level


class KeyTracker:
    """자동반복을 걸러 실제로 누른 순간과 누르고 있는 키를 구분한다.

    press() 는 새로 누른 키면 True 를 돌려준다(W/S 1단계, Space 1회 기록 등에 사용).
    떼기는 AUTOREPEAT_GAP 이 지나야 확정된다(poll). 그 사이 다시 눌리면 자동반복이다.
    """

    def __init__(self, gap=AUTOREPEAT_GAP):
        self.gap = float(gap)
        self.held = set()
        self._release_at = {}

    def press(self, key, now):
        key = key.lower()
        if key in self._release_at:
            released = self._release_at.pop(key)
            # 떼기 직후(gap 안) 다시 눌림 = 자동반복. 그보다 늦으면 실제로 다시 누른 것
            # (주기 poll 이 떼기를 확정하기 전에 다시 눌러도 연타로 센다).
            return now - released >= self.gap
        if key in self.held:                 # 떼기 없이 반복된 누르기
            return False
        self.held.add(key)
        return True

    def release(self, key, now):
        key = key.lower()
        if key in self.held:
            self._release_at[key] = now

    def poll(self, now):
        """떼기가 확정된 키를 held 에서 뺀다. 매 주기 호출."""
        for key, at in list(self._release_at.items()):
            if now - at >= self.gap:
                self.held.discard(key)
                del self._release_at[key]
        return self.held

    def clear(self):
        self.held.clear()
        self._release_at.clear()
