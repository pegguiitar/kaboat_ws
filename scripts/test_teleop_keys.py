"""9단계 속도(W/S), A/D 선회 방향, 키 자동반복 걸러내기."""

import unittest

from teleop_keys import (
    AUTOREPEAT_GAP, MAX_LEVEL, KeyTracker, SpeedLevel, normalize_key, teleop_command)


class NormalizeKeyTest(unittest.TestCase):
    def test_english_keys_pass_through(self):
        self.assertEqual(normalize_key('w'), 'w')
        self.assertEqual(normalize_key('W'), 'w')                    # Caps Lock / Shift
        self.assertEqual(normalize_key('space'), 'space')
        self.assertEqual(normalize_key('Escape'), 'escape')

    def test_hangul_input_maps_to_same_physical_key(self):
        # 한글 입력 상태: 문자로 오든 keysym 이름으로 오든 같은 자리 영문 키
        for char, key in (('ㅈ', 'w'), ('ㅁ', 'a'), ('ㄴ', 's'), ('ㅇ', 'd'), ('ㅌ', 'x'),
                          ('ㄱ', 'r'), ('ㅂ', 'q'), ('ㅔ', 'p'), ('ㅐ', 'o'), ('ㅉ', 'w')):
            self.assertEqual(normalize_key('??', char), key)
        for keysym, key in (('Hangul_Jieuj', 'w'), ('Hangul_Mieum', 'a'),
                            ('Hangul_Nieun', 's'), ('Hangul_Ieung', 'd'),
                            ('Hangul_Tieut', 'x'), ('Hangul_SsangJieuj', 'w')):
            self.assertEqual(normalize_key(keysym, ''), key)

    def test_physical_keycode_is_the_last_resort(self):
        self.assertEqual(normalize_key('??', '', 25), 'w')            # X11 evdev W 자리
        self.assertEqual(normalize_key('??', '', 65), 'space')
        self.assertEqual(normalize_key('Hangul', '', 130), 'hangul')  # 한/영 키는 무시됨

    def test_press_and_release_agree_in_hangul_mode(self):
        keys = KeyTracker()
        self.assertTrue(keys.press(normalize_key('Hangul_Mieum'), 0.0))
        self.assertIn('a', keys.held)
        keys.release(normalize_key('Hangul_Mieum'), 0.1)
        self.assertNotIn('a', keys.poll(0.2))


class SpeedLevelTest(unittest.TestCase):
    def test_w_and_s_step_one_level_and_hold_within_minus4_plus4(self):
        speed = SpeedLevel()
        self.assertEqual(speed.level, 0)
        self.assertEqual([speed.up() for _ in range(6)], [1, 2, 3, 4, 4, 4])
        self.assertEqual(speed.level, 4)                     # 키를 떼도 유지
        self.assertEqual([speed.down() for _ in range(10)],
                         [3, 2, 1, 0, -1, -2, -3, -4, -4, -4])
        self.assertEqual(speed.stop(), 0)
        self.assertEqual(MAX_LEVEL, 4)


class TeleopCommandTest(unittest.TestCase):
    def test_no_turn_key_sends_the_speed_level_straight(self):
        for level in range(-4, 5):
            self.assertEqual(teleop_command(level, set()), (level, 0))

    def test_a_is_left_turn_and_d_is_right_turn_while_held(self):
        self.assertEqual(teleop_command(3, {'a'}), (3, 1))
        self.assertEqual(teleop_command(3, {'d'}), (3, -1))
        self.assertEqual(teleop_command(0, {'a'}), (0, 1))    # 중립 + A = 제자리 좌회전

    def test_releasing_or_both_turn_keys_go_straight(self):
        self.assertEqual(teleop_command(2, set()), (2, 0))
        self.assertEqual(teleop_command(1, {'a', 'd'}), (1, 0))


class KeyTrackerTest(unittest.TestCase):
    def test_autorepeat_counts_as_one_press_and_keeps_the_key_held(self):
        keys = KeyTracker()
        self.assertTrue(keys.press('w', 0.00))
        # X11 자동반복: 떼기 → 곧바로 누르기 를 반복
        for k in range(1, 10):
            t = 0.5 + 0.03 * k
            keys.release('w', t)
            self.assertFalse(keys.press('w', t + 0.001))
            self.assertIn('w', keys.poll(t + 0.01))

    def test_real_release_is_confirmed_after_the_gap(self):
        keys = KeyTracker()
        keys.press('a', 0.0)
        keys.release('a', 1.0)
        self.assertIn('a', keys.poll(1.0 + AUTOREPEAT_GAP / 2))
        self.assertNotIn('a', keys.poll(1.0 + AUTOREPEAT_GAP))
        self.assertTrue(keys.press('a', 2.0))                # 다시 누르면 새 입력

    def test_fast_taps_count_even_before_the_release_is_polled(self):
        keys = KeyTracker()
        presses = 0
        for k in range(3):                     # 누름 80 ms, 간격 150 ms, poll 없음
            t = 0.15 * k
            presses += keys.press('w', t)
            keys.release('w', t + 0.08)
        self.assertEqual(presses, 3)

    def test_keys_are_case_insensitive_and_clear_drops_all(self):
        keys = KeyTracker()
        self.assertTrue(keys.press('W', 0.0))
        self.assertFalse(keys.press('w', 0.1))
        keys.clear()
        self.assertEqual(keys.poll(1.0), set())
        self.assertTrue(keys.press('w', 1.0))


if __name__ == '__main__':
    unittest.main()
