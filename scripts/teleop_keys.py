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

# 한글 입력 상태에서 같은 자리 키 (두벌식). 조종 창은 입력기를 끄지만(Tk useinputmethods 0)
# 입력기가 그래도 자모를 넘기는 환경을 위해 영문 키로 바꾼다.
HANGUL_TO_KEY = {
    'ㅂ': 'q', 'ㅈ': 'w', 'ㄷ': 'e', 'ㄱ': 'r', 'ㅅ': 't', 'ㅛ': 'y', 'ㅕ': 'u', 'ㅑ': 'i',
    'ㅐ': 'o', 'ㅔ': 'p', 'ㅁ': 'a', 'ㄴ': 's', 'ㅇ': 'd', 'ㄹ': 'f', 'ㅎ': 'g', 'ㅗ': 'h',
    'ㅓ': 'j', 'ㅏ': 'k', 'ㅣ': 'l', 'ㅋ': 'z', 'ㅌ': 'x', 'ㅊ': 'c', 'ㅍ': 'v', 'ㅠ': 'b',
    'ㅜ': 'n', 'ㅡ': 'm',
    'ㅃ': 'q', 'ㅉ': 'w', 'ㄸ': 'e', 'ㄲ': 'r', 'ㅆ': 't', 'ㅒ': 'o', 'ㅖ': 'p',  # Shift
}
# 같은 자모의 X11 keysym 이름 (event.keysym 이 'Hangul_Jieuj' 처럼 올 때)
_HANGUL_KEYSYM = {
    'pieub': 'ㅂ', 'jieuj': 'ㅈ', 'dikeud': 'ㄷ', 'kiyeog': 'ㄱ', 'sios': 'ㅅ', 'yo': 'ㅛ',
    'yeo': 'ㅕ', 'ya': 'ㅑ', 'ae': 'ㅐ', 'e': 'ㅔ', 'mieum': 'ㅁ', 'nieun': 'ㄴ', 'ieung': 'ㅇ',
    'rieul': 'ㄹ', 'hieuh': 'ㅎ', 'o': 'ㅗ', 'eo': 'ㅓ', 'a': 'ㅏ', 'i': 'ㅣ', 'khieuq': 'ㅋ',
    'tieut': 'ㅌ', 'cieuc': 'ㅊ', 'phieuf': 'ㅍ', 'yu': 'ㅠ', 'u': 'ㅜ', 'eu': 'ㅡ',
    'ssangpieub': 'ㅃ', 'ssangjieuj': 'ㅉ', 'ssangdikeud': 'ㄸ', 'ssangkiyeog': 'ㄲ',
    'ssangsios': 'ㅆ', 'yae': 'ㅒ', 'ye': 'ㅖ',
}
# 마지막 수단: 리눅스 X11(evdev) 물리 키 번호 → 영문 키
_X11_KEYCODE = {
    24: 'q', 25: 'w', 26: 'e', 27: 'r', 28: 't', 29: 'y', 30: 'u', 31: 'i', 32: 'o', 33: 'p',
    38: 'a', 39: 's', 40: 'd', 41: 'f', 42: 'g', 43: 'h', 44: 'j', 45: 'k', 46: 'l',
    52: 'z', 53: 'x', 54: 'c', 55: 'v', 56: 'b', 57: 'n', 58: 'm', 65: 'space', 9: 'escape',
}


def normalize_key(keysym, char='', keycode=None):
    """Tk 키 이벤트 → 영문 소문자 키 이름 ('w', 'space', 'escape' …). 한/영 상태와 무관.

    순서: 영문 keysym → 한글 문자(event.char) → 한글 keysym 이름 → 물리 키 번호.
    """
    key = str(keysym or '').lower()
    if (len(key) == 1 and 'a' <= key <= 'z') or key in ('space', 'escape'):
        return key
    if char in HANGUL_TO_KEY:
        return HANGUL_TO_KEY[char]
    if key.startswith('hangul_') and _HANGUL_KEYSYM.get(key[7:]) in HANGUL_TO_KEY:
        return HANGUL_TO_KEY[_HANGUL_KEYSYM[key[7:]]]
    if keycode in _X11_KEYCODE:
        return _X11_KEYCODE[keycode]
    return key

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
