"""Check WASD thrust signs, in-place A/D rotation and bounded input ramping."""

import unittest

from teleop_keys import FORWARD, REVERSE, TURN, SmoothAxes, command_for_keys


class TeleopKeysTest(unittest.TestCase):
    def test_raised_outputs(self):
        self.assertEqual((FORWARD, REVERSE, TURN), (0.30, 0.15, 0.25))

    def test_w_and_s_drive_straight(self):
        self.assertEqual(command_for_keys('w'), (FORWARD, 0.0))
        self.assertEqual(command_for_keys('s'), (-REVERSE, 0.0))

    def test_a_and_d_rotate_in_place_with_opposite_thrusters(self):
        for keys, sign in (('a', 1.0), ('d', -1.0)):
            with self.subTest(keys=keys):
                linear, angular = command_for_keys(keys)
                self.assertEqual((linear, angular), (0.0, sign * TURN))
                left, right = linear - angular, linear + angular  # thruster mixer
                self.assertEqual(left, -right)                     # 제자리 회전
                self.assertEqual(right > 0.0, keys == 'a')         # A = 좌회전(CCW)

    def test_a_or_d_take_priority_over_w_or_s(self):
        for keys, sign in (('wa', 1.0), ('wd', -1.0), ('sa', 1.0), ('sd', -1.0)):
            with self.subTest(keys=keys):
                self.assertEqual(command_for_keys(keys), (0.0, sign * TURN))

    def test_conflicting_keys_are_neutral_on_that_axis(self):
        self.assertEqual(command_for_keys('ws'), (0.0, 0.0))
        self.assertEqual(command_for_keys('ad'), (0.0, 0.0))
        self.assertEqual(command_for_keys('wad'), (FORWARD, 0.0))

    def test_turn_ramps_in_and_out_and_reaches_target_in_half_a_second(self):
        ramp = SmoothAxes()
        linear, angular = ramp.step('a', 0.05)
        self.assertEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 0.5 * 0.05)
        for _ in range(9):
            linear, angular = ramp.step('a', 0.05)
        self.assertAlmostEqual(angular, TURN)                  # 0.5 s 안에 도달
        linear2, angular2 = ramp.step('', 0.05)
        self.assertLess(angular2, angular)
        ramp.reset()
        self.assertEqual((ramp.linear, ramp.angular), (0.0, 0.0))

    def test_switching_from_forward_to_turn_stops_forward_first(self):
        ramp = SmoothAxes()
        for _ in range(20):
            ramp.step('w', 0.05)
        self.assertAlmostEqual(ramp.linear, FORWARD)
        for _ in range(20):
            linear, angular = ramp.step('wa', 0.05)
        self.assertEqual(linear, 0.0)                          # 1 s 뒤 전진 0, 제자리 회전
        self.assertAlmostEqual(angular, TURN)


if __name__ == '__main__':
    unittest.main()
