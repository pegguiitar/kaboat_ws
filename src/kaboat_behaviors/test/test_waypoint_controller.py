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

    def test_thrust_ratio_scales_heading_alignment_cap(self):
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
        # 도착 반경(0.35)의 2배(0.7)를 넘게 밀려나야 다시 접근한다
        drift = self.ctrl.step(5.8, 2.5, math.radians(179), 0.0, 0.7)
        self.assertEqual(drift.phase, 'APPROACH')
        self.assertFalse(self.ctrl.step(5.0, 2.5, math.radians(179),
                                        0.0, 0.8).reached)

    def test_hysteresis_keeps_aligning_after_small_drift(self):
        """경계를 조금 넘어도 APPROACH 로 돌아가지 않는다 (APPROACH↔ALIGN 맴돌기 방지)."""
        self.ctrl.set_goal(waypoint(5.0, 2.5, 180.0))
        self.assertEqual(self.ctrl.step(5.3, 2.5, 0.0, 0.0, 0.0).phase, 'ALIGN')
        for x in (5.4, 5.55, 5.69):                    # 0.35 < 거리 ≤ 0.7
            out = self.ctrl.step(x, 2.5, 0.5, 0.0, 0.1)
            self.assertEqual(out.phase, 'ALIGN')
            self.assertEqual(out.linear, 0.0)
            self.assertGreater(out.angular, 0.0)       # 목표 각도(180°) 쪽으로만 돈다
        self.assertEqual(self.ctrl.step(5.71, 2.5, 0.5, 0.0, 0.2).phase, 'APPROACH')

    def test_slows_down_more_inside_twice_the_tolerance(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5))
        p = self.ctrl.params
        out = self.ctrl.step(4.5, 2.5, 0.0, 0.0, 0.0)  # 거리 0.5 m (< 0.7)
        expected = p.max_linear * (0.5 / p.slow_radius) * (0.5 / 0.7)
        self.assertAlmostEqual(out.linear, expected)

    def test_settling_keeps_heading_pd_and_waits_for_rotation_to_stop(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5, 180.0))
        # 허용 오차 안이지만 아직 빠르게 돌고 있음 → 정착 아님, 감쇠 명령을 낸다
        spinning = self.ctrl.step(5.0, 2.5, math.radians(175), 0.3, 0.0)
        self.assertEqual(spinning.phase, 'ALIGN')
        self.assertLess(spinning.angular, 0.0)
        settling = self.ctrl.step(5.0, 2.5, math.radians(175), 0.0, 0.1)
        self.assertEqual(settling.phase, 'SETTLING')
        self.assertGreater(settling.angular, 0.0)      # 정착 중에도 남은 오차를 계속 줄임

    def test_zero_settle_yaw_rate_disables_rotation_check(self):
        ctrl = WaypointController(WaypointParams(settle_time=0.5, settle_yaw_rate=0.0))
        ctrl.set_goal(waypoint(5.0, 2.5, 180.0))
        # 빠르게 돌고 있어도 헤딩 오차만 허용 범위 안이면 정착으로 본다
        self.assertEqual(ctrl.step(5.0, 2.5, math.radians(175), 0.8, 0.0).phase,
                         'SETTLING')
        self.assertTrue(ctrl.step(5.0, 2.5, math.radians(175), 0.8, 0.6).reached)
        with self.assertRaises(ValueError):
            WaypointParams(settle_yaw_rate=-0.1)

    def test_thrust_ratio_caps_output_but_keeps_pd_gain(self):
        ctrl = WaypointController(WaypointParams(max_angular=1.0, yaw_kp=1.4, yaw_kd=0.16))
        small = []
        for ratio in (0.5, 1.0):
            ctrl.set_goal(waypoint(5.0, 2.5, 180.0, ratio))
            small.append(ctrl.step(5.0, 2.5, math.radians(170), 0.0, 0.0).angular)
        self.assertAlmostEqual(small[0], small[1])     # 작은 오차: 비율과 무관한 같은 PD 출력
        ctrl.set_goal(waypoint(5.0, 2.5, 180.0, 0.2))
        big = ctrl.step(5.0, 2.5, 0.0, 0.0, 0.0).angular
        self.assertAlmostEqual(abs(big), 0.2)          # 큰 오차: 상한 max_angular × 비율

    def test_invalid_odom_cannot_finish_goal(self):
        self.ctrl.set_goal(waypoint(5.0, 2.5, 0.0))
        self.assertEqual(self.ctrl.step(float('nan'), 2.5, 0.0,
                                        0.0, 0.0).phase, 'INVALID_ODOM')


if __name__ == '__main__':
    unittest.main()
