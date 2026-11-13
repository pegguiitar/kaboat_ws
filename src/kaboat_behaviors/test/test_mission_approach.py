"""mission_approach 도착 조건: 2 m 안, 선수각 ±30° 안, 회전 속도 조건 없음, 0.5 s 유지."""

import math

import pytest

from kaboat_behaviors.mission_approach import (
    APPROACH_HEADING_TOLERANCE_DEG, APPROACH_POSITION_TOLERANCE,
    APPROACH_SETTLE_YAW_RATE, approach_params)
from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams, waypoint)


def controller():
    base = WaypointParams()
    return WaypointController(approach_params(
        base, APPROACH_POSITION_TOLERANCE, APPROACH_HEADING_TOLERANCE_DEG,
        APPROACH_SETTLE_YAW_RATE))


def test_defaults_are_two_metres_thirty_degrees_no_yaw_rate_check():
    assert APPROACH_POSITION_TOLERANCE == 2.0
    assert APPROACH_HEADING_TOLERANCE_DEG == 30.0
    assert APPROACH_SETTLE_YAW_RATE == 0.0


def test_only_arrival_condition_changes():
    base = WaypointParams()
    p = approach_params(base, 1.0, 15.0, 0.0)
    assert (p.max_linear, p.max_angular, p.yaw_kp, p.yaw_kd, p.settle_time,
            p.exit_factor) == (base.max_linear, base.max_angular, base.yaw_kp,
                               base.yaw_kd, base.settle_time, base.exit_factor)
    assert WaypointParams().position_tolerance == 0.35   # 다른 behavior 기본값은 그대로


def test_reaches_within_two_metres_and_thirty_degrees_while_turning():
    ctrl = controller()
    ctrl.set_goal(waypoint(10.0, 0.0, 90.0))
    # 1.9 m 떨어짐, 선수각 29° 어긋남, 0.5 rad/s 로 돌고 있음
    first = ctrl.step(8.1, 0.0, math.radians(61), 0.5, 0.0)
    assert first.phase == 'SETTLING'
    assert ctrl.step(8.1, 0.0, math.radians(61), 0.5, 0.6).reached


def test_outside_two_metres_keeps_approaching():
    ctrl = controller()
    ctrl.set_goal(waypoint(10.0, 0.0, 90.0))
    assert ctrl.step(7.9, 0.0, math.radians(90), 0.0, 0.0).phase == 'APPROACH'


@pytest.mark.parametrize('heading_deg', [59.0, 121.0])
def test_heading_beyond_thirty_degrees_keeps_aligning(heading_deg):
    ctrl = controller()
    ctrl.set_goal(waypoint(10.0, 0.0, 90.0))
    step = ctrl.step(9.5, 0.0, math.radians(heading_deg), 0.0, 0.0)
    assert step.phase == 'ALIGN'
    assert not step.reached
