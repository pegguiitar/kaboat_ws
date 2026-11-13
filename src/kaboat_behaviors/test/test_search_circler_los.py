"""The search behavior orbits the locked target buoy through the shared LOS."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import pytest

from kaboat_behaviors.search_circler import SearchCircler
from kaboat_behaviors.behavior_base import DEFAULT_MAX_LINEAR, WAYPOINT_MAX_ANGULAR
from kaboat_behaviors.search_orbit import SearchParams
from kaboat_behaviors.waypoint_controller import WaypointController, WaypointParams


def make_state(color, buoys, boat=(45.0, 70.0), yaw=0.0, search_center=(50.0, 70.0), **params):
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = 50.0
    goal.pose.position.y = 70.0
    odom = SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=boat[0], y=boat[1]),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))),
        twist=SimpleNamespace(twist=SimpleNamespace(angular=SimpleNamespace(z=0.0))))
    calls = []
    params.setdefault('lock_confirm_sec', 0.0)
    params.setdefault('look_sec', 0.0)
    state = SimpleNamespace(
        goal=goal, odom=odom, occupancy_grid=None, buoys=buoys,
        params=SearchParams(target_color=color, **params), fsm=None,
        search_center=search_center, _fallback_area=None, _hold=None,
        _circle_path=None, _circle_center=None, _last_status=None,
        _entry_path=None, _entry_points=None, _entry_done=False,
        _los_follower=SimpleNamespace(reset=lambda: None),
        _hold_ctrl=None, _command_thrust_ratio=None,
        max_linear=0.1, _command_full_scale=None, calls=calls,
        status_pub=SimpleNamespace(publish=lambda msg: None),
        completed=[])
    state.waypoint_odom_ready = lambda: True
    state.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None,
                                               warning=lambda *a, **k: None,
                                               error=lambda *a, **k: None)
    state.los = lambda path, ratio, turn: (calls.append((path, ratio, turn)) or Twist(), False)
    state.waypoint = lambda x, y, *a, **kw: (calls.append(('wp', x, y, *a, kw)) or Twist(), False)
    state.report_complete = lambda: state.completed.append(True)
    for name in ('_status', '_go_goal', '_area_xy', '_plan_entry', '_hold_pose'):
        setattr(state, name, (lambda n: lambda *a: getattr(SearchCircler, n)(state, *a))(name))
    state._hold_ctrl = WaypointController(WaypointParams(
        max_linear=state.params.hold_linear, max_angular=WAYPOINT_MAX_ANGULAR,
        yaw_kp=1.4, yaw_kd=0.16))
    return state


def buoy(color, x, y):
    return SimpleNamespace(color=color, position=SimpleNamespace(x=x, y=y))


def move(state, x, y):
    state.odom.pose.pose.position.x, state.odom.pose.pose.position.y = x, y


def test_orbit_path_centered_on_target_with_rule_direction():
    # 원 위에서 시작 — 진입 경로 없이 원을 바로
    state = make_state('red', [buoy('red', 52.0, 71.0), buoy('green', 50.0, 75.0)],
                       boat=(54.0, 71.0))
    SearchCircler.compute_cmd(state)
    SearchCircler.compute_cmd(state)
    path, ratio, turn = state.calls[0]
    assert state.calls[1][0] is path                 # 같은 원을 계속 추종
    assert (ratio, turn) == (0.25, 0.31)             # 공용 LOS SOTA
    assert np.all(path.curvatures < 0)               # 빨강 → 시계(우현)
    radii = np.hypot(path.samples[:, 0] - 52.0, path.samples[:, 1] - 71.0)
    assert np.allclose(radii, 2.0)


def test_black_orbits_counter_clockwise():
    state = make_state('black', [buoy('black', 52.0, 71.0)], boat=(54.0, 71.0))
    SearchCircler.compute_cmd(state)
    assert np.all(state.calls[0][0].curvatures > 0)  # 검정 → 반시계(좌현)


def test_far_lock_follows_tangent_entry_then_circle():
    state = make_state('red', [buoy('red', 52.0, 71.0)], boat=(45.0, 70.0))
    SearchCircler.compute_cmd(state)
    entry, ratio, turn = state.calls[0]
    assert not entry.closed and (ratio, turn) == (0.25, 0.31)
    assert np.allclose(entry.samples[0], [45.0, 70.0])
    end = entry.samples[-1]
    assert math.hypot(end[0] - 52.0, end[1] - 71.0) == pytest.approx(2.0, abs=1e-6)
    SearchCircler.compute_cmd(state)
    assert state.calls[1][0] is entry                # 진입 경로를 계속
    move(state, end[0], end[1])                      # 진입 끝 → 원으로
    SearchCircler.compute_cmd(state)
    SearchCircler.compute_cmd(state)
    assert state.calls[-1][0].closed


def test_no_tangent_entry_when_disabled():
    state = make_state('red', [buoy('red', 52.0, 71.0)], tangent_entry=False)
    SearchCircler.compute_cmd(state)
    assert state.calls[0][0].closed


def test_no_target_drives_to_area_center():
    state = make_state('green', [buoy('red', 52.0, 71.0)], search_center=(60.0, 75.0))
    SearchCircler.compute_cmd(state)
    assert state.calls[0][:3] == ('wp', 60.0, 75.0)


def test_area_falls_back_ahead_of_activation_pose():
    state = make_state('green', [], yaw=math.pi / 2, search_center=None, center_offset=7.0)
    SearchCircler.compute_cmd(state)
    _, x, y = state.calls[0][:3]
    assert (x, y) == (45.0 + 7.0 * math.cos(math.pi / 2), 77.0)


def test_look_holds_activation_pose_facing_area_with_hold_cap():
    state = make_state('red', [], search_center=(60.0, 80.0), look_sec=15.0, hold_linear=0.25)
    SearchCircler.compute_cmd(state)
    assert state._hold == (45.0, 70.0, math.degrees(math.atan2(10.0, 15.0)))
    assert state._command_full_scale == (0.25, WAYPOINT_MAX_ANGULAR)   # 공용 waypoint() 아님
    assert state.calls == []
    move(state, 43.0, 70.0)                          # 바람에 2 m 밀림 → 처음 자리로 되돌아감
    cmd = SearchCircler.compute_cmd(state)
    assert state._hold[:2] == (45.0, 70.0)
    assert DEFAULT_MAX_LINEAR < cmd.linear.x <= 0.25 + 1e-9


def test_scan_in_place_at_area_uses_effective_turn():
    state = make_state('green', [], boat=(50.0, 70.0))
    cmd = SearchCircler.compute_cmd(state)
    assert cmd.linear.x == 0.0 and cmd.angular.z == -0.15   # 초록 → 시계 방향으로 둘러봄
    assert state._command_full_scale == (0.1, 0.15)


def test_unset_colour_stops():
    state = make_state('', [buoy('red', 52.0, 71.0)])
    cmd = SearchCircler.compute_cmd(state)
    assert cmd == Twist() and state.calls == []
