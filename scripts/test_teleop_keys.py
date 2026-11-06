"""9단계 속도(W/S), A/D 차동 조향, 키 자동반복 걸러내기."""

import unittest

from teleop_keys import (
    AUTOREPEAT_GAP, MAX_LEVEL, KeyTracker, SpeedLevel, thruster_levels)


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


class ThrusterLevelsTest(unittest.TestCase):
    def test_no_turn_key_drives_both_sides_equally(self):
        for level in range(-4, 5):
            self.assertEqual(thruster_levels(level, set()), (level, level))

    def test_forward_3_with_a_is_left_2_right_4(self):
        self.assertEqual(thruster_levels(3, {'a'}), (2, 4))   # 요구사항 예시
        self.assertEqual(thruster_levels(3, {'d'}), (4, 2))

    def test_neutral_a_and_d_rotate_in_place(self):
        self.assertEqual(thruster_levels(0, {'a'}), (-1, 1))  # 좌 후진, 우 전진 = 제자리 좌회전
        self.assertEqual(thruster_levels(0, {'d'}), (1, -1))

    def test_releasing_a_returns_to_equal_output(self):
        self.assertEqual(thruster_levels(2, {'a'}), (1, 3))
        self.assertEqual(thruster_levels(2, set()), (2, 2))

    def test_both_turn_keys_cancel_and_sides_stay_within_range(self):
        self.assertEqual(thruster_levels(1, {'a', 'd'}), (1, 1))
        self.assertEqual(thruster_levels(4, {'a'}), (3, 4))
        self.assertEqual(thruster_levels(-4, {'d'}), (-3, -4))
        self.assertEqual(thruster_levels(-3, {'a'}), (-4, -2))


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
