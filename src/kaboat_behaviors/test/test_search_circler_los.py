"""The search behavior actually routes colored turns through the shared LOS."""

from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

from kaboat_behaviors.search_circler import SearchCircler


class FakeTime:
    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=0)


def test_search_uses_stable_circle_path_and_keeps_orbiting_outside_entry_radius():
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x = 5.0
    goal.pose.position.y = 2.5
    calls = []
    state = SimpleNamespace(
        goal=goal, odom=None, occupancy_grid=None,
        _circle_since=None, _circle_path=None, _circle_key=None,
        max_angular=0.02, distance=0.5, color='green')
    state.waypoint_odom_ready = lambda: True
    state.distance_to_goal = lambda: state.distance
    state.get_clock = lambda: SimpleNamespace(now=FakeTime)
    state.visible_buoys = lambda color: [object()] if color == state.color else []
    state.los = lambda path, ratio: (calls.append((path, ratio)) or Twist(), False)
    state.report_complete = lambda: None

    SearchCircler.compute_cmd(state)
    state.distance = 1.2  # orbit radius exceeds the 0.8 m entry threshold
    SearchCircler.compute_cmd(state)
    assert calls[0][0] is calls[1][0]
    assert calls[0][1] == calls[1][1] == 0.3
    assert np.all(calls[0][0].curvatures > 0)  # green -> CCW

    state.color = 'red'
    SearchCircler.compute_cmd(state)
    assert calls[2][0] is not calls[1][0]
    assert np.all(calls[2][0].curvatures < 0)  # red -> CW
