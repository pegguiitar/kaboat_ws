"""search_circler 노드 — 탐색 구역 접근, 표식 색별 선회, 5초 선회 후 완료.

색별 원 경로의 방향과 재사용은 test_search_circler_los.py 가 따로 확인한다.
"""
import pytest
from kaboat_msgs.msg import Buoy, BuoyArray

from kaboat_behaviors.search_circler import (
    ARRIVE_RADIUS, SEARCH_TIME, SearchCircler)
from node_helpers import BehaviorHarness, is_stop


@pytest.fixture
def h(monkeypatch):
    harness = BehaviorHarness(SearchCircler, monkeypatch)
    harness.activate()
    harness.set_goal(5.0, 0.0)
    yield harness
    harness.close()


def _buoys(*items):
    msg = BuoyArray()
    for color, x, y in items:
        b = Buoy()
        b.color = color
        b.position.x = float(x)
        b.position.y = float(y)
        msg.buoys.append(b)
    return msg


@pytest.mark.check('A')
def test_approaches_search_area_when_far(h):
    h.set_pose(0.0, 0.0)
    cmd = h.tick()
    assert cmd.linear.x > 0.0
    assert h.node._circle_since is None


@pytest.mark.check('A')
def test_pivots_in_place_when_no_marker_is_visible(h):
    h.set_pose(5.0 - ARRIVE_RADIUS / 2, 0.0)
    cmd = h.tick()
    assert cmd.linear.x == 0.0
    assert cmd.angular.z == pytest.approx(h.node.max_angular * 0.5)


@pytest.mark.check('A')
def test_reports_complete_after_search_time(h):
    h.set_pose(5.0, 0.0)
    h.tick(int(round(SEARCH_TIME / 0.1)))
    assert h.completions == []
    h.tick(1)
    assert h.completions == ['search']


@pytest.mark.check('B')
def test_green_marker_starts_counter_clockwise_circle(h):
    h.node._on_buoys(_buoys(('green', 8.0, 0.0)))
    h.set_pose(5.0, 0.0)
    h.tick()
    assert h.node._circle_key == (5.0, 0.0, 'ccw')


@pytest.mark.check('B')
def test_both_colors_visible_falls_back_to_pivot(h):
    h.node._on_buoys(_buoys(('green', 8.0, 0.5), ('red', 8.0, -0.5)))
    h.set_pose(5.0, 0.0)
    cmd = h.tick()
    assert h.node._circle_path is None
    assert cmd.angular.z == pytest.approx(h.node.max_angular * 0.5)


@pytest.mark.check('B')
def test_leaving_two_metre_hold_restarts_approach(h):
    h.set_pose(5.0, 0.0)
    h.tick(10)
    assert h.node._circle_since is not None
    h.set_pose(5.0, 1.5)                # 0.8m 밖이지만 2m 안 — 선회 유지
    h.tick()
    assert h.node._circle_since is not None
    h.set_pose(5.0, 2.5)                # 2m 밖 — 접근부터 다시
    h.tick()
    assert h.node._circle_since is None


@pytest.mark.check('C')
def test_stale_odom_or_wrong_goal_frame_gives_stop(h):
    h.set_pose(5.0, 0.0)
    h.node._on_odom(h.pose)
    h.clock.t += 1.0
    assert is_stop(h.node.compute_cmd())
    h.set_goal(5.0, 0.0, frame='map')
    assert is_stop(h.tick())


@pytest.mark.check('D')
def test_completion_once_per_activation(h):
    h.set_pose(5.0, 0.0)
    h.tick(int(round(SEARCH_TIME / 0.1)) + 20)
    assert h.completions == ['search']
    assert SearchCircler.CMD_TOPIC == '/cmd/search'
