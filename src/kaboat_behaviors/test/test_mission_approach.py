"""mission_approach 도착 조건: 2 m 안, 선수각 ±30° 안, 회전 속도 조건 없음, 0.5 s 유지."""

import math

import pytest

from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from kaboat_behaviors.behavior_base import (
    DEFAULT_MAX_ANGULAR, DEFAULT_MAX_LINEAR, BehaviorBase)
from kaboat_behaviors.mission_approach import (
    APPROACH_HEADING_TOLERANCE_DEG, APPROACH_MAX_ANGULAR, APPROACH_MAX_LINEAR,
    APPROACH_POSITION_TOLERANCE, APPROACH_SETTLE_YAW_RATE, APPROACH_THRUST_RATIO,
    MissionApproach, approach_params)
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


# ── 출력 상한 = goto (2026-11-14) ─────────────────────────────

def test_caps_match_goto_forty_percent():
    # goto = outdoor_waypoint.launch.py 기본 max_linear·max_angular 1.0 × thrust_ratio 0.4
    assert APPROACH_MAX_LINEAR * APPROACH_THRUST_RATIO == pytest.approx(0.4)
    assert APPROACH_MAX_ANGULAR * APPROACH_THRUST_RATIO == pytest.approx(0.4)


def _yaw_q(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def approach_node(boat_xy, boat_yaw, goal_xy, goal_yaw):
    """MissionApproach.compute_cmd → BehaviorBase._tick 경로 (ROS 노드 없이)."""
    params = approach_params(
        WaypointParams(max_linear=DEFAULT_MAX_LINEAR, max_angular=0.31,
                       yaw_kp=1.4, yaw_kd=0.16),
        APPROACH_POSITION_TOLERANCE, APPROACH_HEADING_TOLERANCE_DEG,
        APPROACH_SETTLE_YAW_RATE, max_linear=APPROACH_MAX_LINEAR,
        max_angular=APPROACH_MAX_ANGULAR)
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x, odom.pose.pose.position.y = boat_xy
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = _yaw_q(boat_yaw)
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x, goal.pose.position.y = goal_xy
    goal.pose.orientation.z, goal.pose.orientation.w = _yaw_q(goal_yaw)
    sent = []
    node = SimpleNamespace(
        active=True, odom=odom, entry_goal=goal, motion_enabled=True,
        thrust_ratio=APPROACH_THRUST_RATIO,
        # 공용 behavior 스케일 (base 기본) — approach 는 이걸 쓰면 안 됨
        max_linear=DEFAULT_MAX_LINEAR, max_angular=DEFAULT_MAX_ANGULAR,
        _waypoint_controller=WaypointController(params),
        _full_scale=(params.max_linear, params.max_angular),
        _command_thrust_ratio=None, _command_full_scale=None,
        preview_pub=SimpleNamespace(publish=lambda _m: None),
        cmd_pub=SimpleNamespace(publish=sent.append),
        report_complete=lambda: None)
    node.waypoint_odom_ready = lambda: True
    node.waypoint = lambda *a, **k: BehaviorBase.waypoint(node, *a, **k)
    node.compute_cmd = lambda: MissionApproach.compute_cmd(node)
    return node, sent


def test_far_goal_ahead_drives_at_goto_forward_cap():
    node, sent = approach_node((0.0, 0.0), 0.0, (30.0, 0.0), 0.0)
    BehaviorBase._tick(node)
    assert sent[-1].linear.x == pytest.approx(0.4)        # 예전 0.0202


def test_goal_behind_turns_at_goto_turn_cap():
    node, sent = approach_node((0.0, 0.0), 0.0, (-30.0, 0.0), math.pi)
    BehaviorBase._tick(node)
    assert abs(sent[-1].angular.z) == pytest.approx(0.4)  # 예전 0.062
