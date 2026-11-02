"""Position and final-heading behavior of the reusable waypoint primitive."""

import math
import unittest

from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams, waypoint)


class TestWaypointController(unittest.TestCase):
    def setUp(self):
        self.ctrl = WaypointController(WaypointParams(settle_time=0.5))

    def test_waypoint_is_meters_and_degrees(self):
        target = waypoint(5.0, 2.5, 180.0)
        self.assertAlmostEqual(target.x, 5.0)
        self.assertAlmostEqual(target.yaw, math.pi)
        self.assertIsNone(waypoint(5.0, 2.5).yaw)
        with self.assertRaises(ValueError):
            waypoint(float('nan'), 2.5, 90.0)

    def test_thrust_ratio_is_bounded_and_scales_approach(self):
        for invalid in (-0.01, 1.01, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                waypoint(5.0, 2.5, thrust_ratio=invalid)
        self.ctrl.set_goal(waypoint(5.0, 2.5, None, 0.5))
        half = self.ctrl.step(2.0, 2.5, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(half.linear, self.ctrl.params.max_linear * 0.5)
        self.ctrl.set_goal(waypoint(5.0, 2.5, None, 0.0))
        stopped = self.ctrl.step(2.0, 2.5, 0.0, 0.0, 0.1)
        self.assertEqual((stopped.linear, stopped.angular), (0.0, 0.0))
        self.assertFalse(stopped.reached)

    def test_thrust_ratio_scales_heading_alignment(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5, 180.0, 0.5))
        half = self.ctrl.step(5.0, 2.5, 0.0, 0.0, 0.0)
        self.ctrl.set_goal(waypoint(5.0, 2.5, 180.0, 1.0))
        full = self.ctrl.step(5.0, 2.5, 0.0, 0.0, 0.1)
        self.assertEqual(half.phase, 'ALIGN')
        self.assertAlmostEqual(abs(half.angular), abs(full.angular) * 0.5)

    def test_approach_turns_toward_target_and_slows_near_it(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5, 180.0))
        far = self.ctrl.step(8.0, 2.5, math.pi, 0.0, 0.0)
        near = self.ctrl.step(5.6, 2.5, math.pi, 0.0, 0.1)
        self.assertEqual(far.phase, 'APPROACH')
        self.assertGreater(far.linear, near.linear)
        self.assertGreater(near.linear, 0.0)

    def test_target_behind_boat_does_not_drive_forward(self):
        self.ctrl.set_goal(waypoint(2.0, 2.5, 0.0))
        out = self.ctrl.step(5.0, 2.5, 0.0, 0.0, 0.0)
        self.assertEqual(out.linear, 0.0)
        self.assertNotEqual(out.angular, 0.0)

    def test_position_only_keeps_old_low_forward_turn(self):
        self.ctrl.set_goal(waypoint(2.0, 2.5))
        out = self.ctrl.step(5.0, 2.5, 0.0, 0.0, 0.0)
        self.assertEqual(out.phase, 'APPROACH')
        self.assertAlmostEqual(out.linear, self.ctrl.params.max_linear * 0.3)
        self.assertNotEqual(out.angular, 0.0)

    def test_position_only_arrives_without_heading_alignment(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5))
        first = self.ctrl.step(5.1, 2.5, math.pi, 0.0, 0.0)
        arrived = self.ctrl.step(5.1, 2.5, math.pi, 0.0, 0.5)
        self.assertEqual(first.phase, 'SETTLING')
        self.assertEqual(arrived.phase, 'REACHED')
        self.assertTrue(arrived.reached)
        self.assertEqual((arrived.linear, arrived.angular), (0.0, 0.0))

    def test_position_only_slow_radius_can_match_station_behavior(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5))
        default = self.ctrl.step(2.5, 2.5, 0.0, 0.0, 0.0)
        station = self.ctrl.step(2.5, 2.5, 0.0, 0.0, 0.1,
                                 slow_radius=5.0)
        self.assertAlmostEqual(station.linear, self.ctrl.params.max_linear * 0.5)
        self.assertGreater(default.linear, station.linear)

    def test_aligns_final_heading_and_then_stops(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5, 180.0))
        align = self.ctrl.step(5.1, 2.5, 0.0, 0.0, 0.0)
        self.assertEqual(align.phase, 'ALIGN')
        self.assertEqual(align.linear, 0.0)
        self.assertFalse(align.reached)
        settling = self.ctrl.step(5.1, 2.5, math.pi, 0.0, 1.0)
        arrived = self.ctrl.step(5.1, 2.5, math.pi, 0.0, 1.5)
        self.assertEqual(settling.phase, 'SETTLING')
        self.assertTrue(arrived.reached)
        self.assertEqual((arrived.linear, arrived.angular), (0.0, 0.0))

    def test_heading_wrap_and_drift_reset_arrival(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5, -179.0))
        near_wrap = self.ctrl.step(5.0, 2.5, math.radians(179), 0.0, 0.0)
        self.assertEqual(near_wrap.phase, 'SETTLING')
        self.assertTrue(self.ctrl.step(5.0, 2.5, math.radians(179),
                                       0.0, 0.6).reached)
        drift = self.ctrl.step(5.6, 2.5, math.radians(179), 0.0, 0.7)
        self.assertEqual(drift.phase, 'APPROACH')
        self.assertFalse(self.ctrl.step(5.0, 2.5, math.radians(179),
                                        0.0, 0.8).reached)

    def test_invalid_odom_cannot_finish_goal(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5, 0.0))
        self.assertEqual(self.ctrl.step(float('nan'), 2.5, 0.0,
                                        0.0, 0.0).phase, 'INVALID_ODOM')


if __name__ == '__main__':
    unittest.main()
