"""공통 los()/waypoint() 가 behavior._tick 에서 어떤 출력 한계를 받는지 검사한다.

los() 는 los_path_test/bspline_track_test 에서 검증된 제어를 쓰므로 thrust_ratio 가
곧 출력 상한이다(전체 스케일 1.0). waypoint() 는 기존대로 behavior 의
max_linear/max_angular 가 전체 스케일이다.
"""
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from kaboat_behaviors.behavior_base import (
    DEFAULT_MAX_ANGULAR, DEFAULT_MAX_LINEAR, BehaviorBase)
from kaboat_behaviors.los_controller import LosFollower, bspline_path
from kaboat_behaviors.waypoint_controller import WaypointController, WaypointParams

CPS = [(8.5, 2.0), (7.0, 3.5), (5.0, 1.5), (3.0, 3.5), (1.5, 2.5)]


def make_state(x, y, yaw, compute):
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
    odom.pose.pose.orientation.z = math.sin(yaw / 2)
    odom.pose.pose.orientation.w = math.cos(yaw / 2)
    sent = []
    state = SimpleNamespace(
        state='t', STATE_NAME='t', active=True, odom=odom,
        max_linear=DEFAULT_MAX_LINEAR, max_angular=DEFAULT_MAX_ANGULAR,
        _los_follower=LosFollower.bspline_track(),
        _waypoint_controller=WaypointController(WaypointParams(
            max_linear=DEFAULT_MAX_LINEAR, max_angular=DEFAULT_MAX_ANGULAR,
            yaw_kp=0.1, yaw_kd=0.05)),
        _command_thrust_ratio=None, _command_full_scale=None,
        cmd_pub=SimpleNamespace(publish=sent.append))
    state.waypoint_odom_ready = lambda: True
    state.compute_cmd = lambda: compute(state)
    return state, sent


def tick(state):
    BehaviorBase._tick(state)


def test_los_output_limit_is_thrust_ratio_not_the_small_behavior_scale():
    path = bspline_path(CPS, spacing=0.05)
    # 경로에서 옆으로 0.3 m, 헤딩 20° 어긋난 상태 → 회전이 상한까지 간다
    p, q = path.samples[60], path.samples[61]
    tangent = math.atan2(q[1] - p[1], q[0] - p[0])
    x, y = p[0] - 0.3 * math.sin(tangent), p[1] + 0.3 * math.cos(tangent)
    state, sent = make_state(x, y, tangent + math.radians(20),
                             lambda s: BehaviorBase.los(s, path, 0.3)[0])
    tick(state)
    cmd = sent[-1]
    assert abs(cmd.angular.z) == pytest.approx(0.30)          # 예전엔 0.0076 으로 잘렸다
    assert 0.05 < cmd.linear.x <= 0.30
    assert cmd.linear.x > 3 * DEFAULT_MAX_LINEAR * 0.3


def test_los_ratio_is_the_cap_for_both_axes():
    path = bspline_path(CPS, spacing=0.05)
    p, q = path.samples[60], path.samples[61]
    tangent = math.atan2(q[1] - p[1], q[0] - p[0])
    for ratio in (0.1, 0.2, 0.3):
        state, sent = make_state(p[0], p[1], tangent + math.radians(40),
                                 lambda s, r=ratio: BehaviorBase.los(s, path, r)[0])
        tick(state)
        assert abs(sent[-1].angular.z) <= ratio + 1e-9
        assert 0.0 <= sent[-1].linear.x <= ratio + 1e-9


def test_waypoint_keeps_the_behavior_scale():
    state, sent = make_state(
        0.0, 0.0, 0.0, lambda s: BehaviorBase.waypoint(s, 6.0, 0.0, None, 0.5)[0])
    tick(state)
    cmd = sent[-1]
    assert 0.0 < cmd.linear.x <= DEFAULT_MAX_LINEAR * 0.5 + 1e-9
    assert abs(cmd.angular.z) <= DEFAULT_MAX_ANGULAR * 0.5 + 1e-9


def test_commands_without_waypoint_or_los_use_the_plain_behavior_limits():
    def wild(_state):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = 5.0, -5.0
        return cmd
    state, sent = make_state(0.0, 0.0, 0.0, wild)
    tick(state)
    assert sent[-1].linear.x == pytest.approx(DEFAULT_MAX_LINEAR)
    assert sent[-1].angular.z == pytest.approx(-DEFAULT_MAX_ANGULAR)


def test_scale_flag_resets_every_tick():
    path = bspline_path(CPS, spacing=0.05)
    calls = {'n': 0}

    def alternate(s):
        calls['n'] += 1
        if calls['n'] == 1:
            return BehaviorBase.los(s, path, 0.3)[0]
        cmd = Twist()
        cmd.linear.x = 5.0
        return cmd
    state, sent = make_state(8.5, 2.0, math.pi, alternate)
    tick(state)
    tick(state)
    assert sent[-1].linear.x == pytest.approx(DEFAULT_MAX_LINEAR)   # LOS 한계가 이월되지 않음


def test_avoid_output_uses_los_thrust_ratio_not_waypoint_scale(monkeypatch):
    """obstacle_planner 상한: 전진 fsm.cmd_full(0.23), 회전 avoid.max_angular_ratio(0.33).

    예전엔 waypoint 상한(전진 0.101·회전 0.025)에 잘려 경로를 못 따라갔다.
    """
    from kaboat_behaviors import obstacle_planner
    from kaboat_behaviors.avoid_fsm import FsmParams, StepOut

    monkeypatch.setattr(obstacle_planner, 'build_dri', lambda *a, **k: None)
    state, sent = make_state(0.0, 0.0, 0.0,
                             lambda s: obstacle_planner.ObstaclePlanner.compute_cmd(s))
    state.goal = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=10.0, y=0.0)))
    state.occupancy_grid = object()
    state.dri_params = None
    state.thrust_ratio = FsmParams().cmd_full
    state.turn_ratio = 0.33
    state.fsm = SimpleNamespace(step=lambda *a: StepOut(0.25, -0.8, 'FOLLOW', None))
    state.distance_to_goal = lambda: 10.0
    state._publish_debug = lambda *a: None
    tick(state)
    cmd = sent[-1]
    assert FsmParams().cmd_full == pytest.approx(0.23)
    assert cmd.linear.x == pytest.approx(0.23)                # 전진 0.25 → 0.23 으로 잘림
    assert cmd.angular.z == pytest.approx(-0.33)              # 회전은 0.33 까지
