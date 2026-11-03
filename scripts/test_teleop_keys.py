"""Check combined WASD thrust signs and bounded input ramping."""

import unittest

from teleop_keys import FORWARD, REVERSE, TURN, SmoothAxes, command_for_keys


class TeleopKeysTest(unittest.TestCase):
    def test_combinations_drive_both_thrusters_in_requested_direction(self):
        cases = {
            'wa': (FORWARD, TURN),
            'wd': (FORWARD, -TURN),
            'sa': (-REVERSE, TURN),
            'sd': (-REVERSE, -TURN),
        }
        for keys, expected in cases.items():
            with self.subTest(keys=keys):
                linear, angular = command_for_keys(keys)
                self.assertEqual((linear, angular), expected)
                left, right = linear - angular, linear + angular
                if 'w' in keys:
                    self.assertGreater(left, 0.0)
                    self.assertGreater(right, 0.0)
                else:
                    self.assertLess(left, 0.0)
                    self.assertLess(right, 0.0)
                self.assertEqual(right > left, 'a' in keys)

    def test_conflicting_keys_are_neutral_on_that_axis(self):
        self.assertEqual(command_for_keys('ws'), (0.0, 0.0))
        self.assertEqual(command_for_keys('ad'), (0.0, 0.0))

    def test_reverse_turn_ramps_in_and_out(self):
        ramp = SmoothAxes(linear_rate=0.3, angular_rate=0.15)
        linear, angular = ramp.step('sa', 0.05)
        self.assertLess(linear, 0.0)
        self.assertGreater(angular, 0.0)
        self.assertLessEqual(abs(linear), 0.015 + 1e-9)
        self.assertLessEqual(abs(angular), 0.0075 + 1e-9)
        linear2, angular2 = ramp.step('', 0.05)
        self.assertLessEqual(abs(linear2), abs(linear))
        self.assertLessEqual(abs(angular2), abs(angular))
        ramp.reset()
        self.assertEqual((ramp.linear, ramp.angular), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
