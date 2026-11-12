"""The search behavior orbits the locked target buoy through the shared LOS."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

from kaboat_behaviors.search_circler import SearchCircler
from kaboat_behaviors.search_orbit import SearchParams


def make_state(color, buoys, boat=(45.0, 70.0)):
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = 50.0
    goal.pose.position.y = 70.0
    odom = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=boat[0], y=boat[1]))))
    calls = []
    state = SimpleNamespace(
        goal=goal, odom=odom, occupancy_grid=None, buoys=buoys,
        params=SearchParams(target_color=color, lock_confirm_sec=0.0), fsm=None,
        _circle_path=None, _circle_center=None, _last_status=None,
        _los_follower=SimpleNamespace(reset=lambda: None),
        max_linear=0.1, _command_full_scale=None, calls=calls,
        status_pub=SimpleNamespace(publish=lambda msg: None),
        completed=[])
    state.waypoint_odom_ready = lambda: True
    state.distance_to_goal = lambda: math.hypot(50.0 - state.odom.pose.pose.position.x,
                                                70.0 - state.odom.pose.pose.position.y)
    state.get_logger = lambda: SimpleNamespace(info=lambda *a, **k: None,
                                               error=lambda *a, **k: None)
    state.los = lambda path, ratio, turn: (calls.append((path, ratio, turn)) or Twist(), False)
    state.waypoint = lambda x, y, **kw: (calls.append(('wp', x, y)) or Twist(), False)
    state.report_complete = lambda: state.completed.append(True)
    state._status = lambda text: SearchCircler._status(state, text)
    state._go_goal = lambda goal_xy: SearchCircler._go_goal(state, goal_xy)
    return state


def buoy(color, x, y):
    return SimpleNamespace(color=color, position=SimpleNamespace(x=x, y=y))


def test_orbit_path_centered_on_target_with_rule_direction():
    state = make_state('red', [buoy('red', 52.0, 71.0), buoy('green', 50.0, 75.0)])
    SearchCircler.compute_cmd(state)
    SearchCircler.compute_cmd(state)
    path, ratio, turn = state.calls[0]
    assert state.calls[1][0] is path                 # 같은 원을 계속 추종
    assert (ratio, turn) == (0.25, 0.31)             # 공용 LOS SOTA
    assert np.all(path.curvatures < 0)               # 빨강 → 시계(우현)
    radii = np.hypot(path.samples[:, 0] - 52.0, path.samples[:, 1] - 71.0)
    assert np.allclose(radii, 2.0)


def test_black_orbits_counter_clockwise():
    state = make_state('black', [buoy('black', 52.0, 71.0)])
    SearchCircler.compute_cmd(state)
    assert np.all(state.calls[0][0].curvatures > 0)  # 검정 → 반시계(좌현)


def test_no_target_drives_to_goal():
    state = make_state('green', [buoy('red', 52.0, 71.0)])
    SearchCircler.compute_cmd(state)
    assert state.calls == [('wp', 50.0, 70.0)]


def test_scan_in_place_at_goal_uses_effective_turn():
    state = make_state('green', [], boat=(50.0, 70.0))
    cmd = SearchCircler.compute_cmd(state)
    assert cmd.linear.x == 0.0 and cmd.angular.z == -0.15   # 초록 → 시계 방향으로 둘러봄
    assert state._command_full_scale == (0.1, 0.15)


def test_unset_colour_stops():
    state = make_state('', [buoy('red', 52.0, 71.0)])
    cmd = SearchCircler.compute_cmd(state)
    assert cmd == Twist() and state.calls == []
