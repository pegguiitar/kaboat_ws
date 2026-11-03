"""obstacle_planner 노드 — 격자 → DRI → 회피 FSM 배선, 완료 판정, 디버그 발행.

DRI·B-spline·FSM 각각의 판단은 test_dri.py, test_bspline_planner.py, test_avoid_fsm.py
가 확인한다. 여기서는 노드가 그 셋을 올바른 좌표·시간으로 잇는지 본다.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid

from kaboat_behaviors.obstacle_planner import ObstaclePlanner
from node_helpers import BehaviorHarness, Spy, is_stop

RES, SIZE, ORIGIN = 0.2, 150, (-10.0, -15.0)


def _grid(obstacles=()):
    """obstacles: [(x, y, r)] 원형 장애물을 100 으로 칠한 OccupancyGrid."""
    data = np.zeros((SIZE, SIZE), dtype=np.int8)
    xs = ORIGIN[0] + (np.arange(SIZE) + 0.5) * RES
    ys = ORIGIN[1] + (np.arange(SIZE) + 0.5) * RES
    gx, gy = np.meshgrid(xs, ys)
    for x, y, r in obstacles:
        data[np.hypot(gx - x, gy - y) <= r] = 100
    msg = OccupancyGrid()
    msg.header.frame_id = 'odom'
    msg.info.resolution = RES
    msg.info.width = msg.info.height = SIZE
    msg.info.origin.position.x, msg.info.origin.position.y = ORIGIN
    msg.info.origin.orientation.w = 1.0
    msg.data = data.ravel().tolist()
    return msg


@pytest.fixture
def h(monkeypatch):
    harness = BehaviorHarness(ObstaclePlanner, monkeypatch)
    harness.node.plan_pub = Spy()
    harness.node.cps_pub = Spy()
    harness.node.dri_pub = Spy()
    harness.activate()
    harness.set_goal(12.0, 0.0)
    yield harness
    harness.close()


@pytest.mark.check('A')
def test_clear_water_acquires_plan_then_drives_forward(h):
    h.node._on_grid(_grid())
    h.set_pose(0.0, 0.0)
    first = h.tick()
    assert is_stop(first)                    # 첫 tick 은 정지 후 plan 확보
    assert h.node.fsm.plan is not None
    second = h.tick()
    assert second.linear.x > 0.0


@pytest.mark.check('A')
def test_obstacle_on_straight_line_bends_plan_sideways(h):
    h.node._on_grid(_grid([(5.0, 0.0, 0.6)]))
    h.set_pose(0.0, 0.0)
    h.tick(2)
    samples = np.asarray(h.node.fsm.plan.samples)
    near = samples[(samples[:, 0] > 4.0) & (samples[:, 0] < 6.0)]
    assert len(near) > 0
    assert np.min(np.abs(near[:, 1])) > 0.6   # 장애물 반지름 밖으로 지나간다


@pytest.mark.check('A')
def test_reports_complete_within_two_metres_of_goal(h):
    h.node._on_grid(_grid())
    h.set_pose(9.0, 0.0)
    h.tick()
    assert h.completions == []
    h.set_pose(10.1, 0.0)
    h.tick()
    assert h.completions == ['avoid']


@pytest.mark.check('B')
def test_dataclass_fields_are_ros_parameters(monkeypatch):
    h = BehaviorHarness(ObstaclePlanner, monkeypatch,
                        ['dri.occ_threshold:=50', 'fsm.lookahead:=3.0'])
    try:
        assert h.node.dri_params.occ_threshold == 50
        assert h.node.fsm_params.lookahead == 3.0
        assert h.node.fsm.fsm.lookahead == 3.0
    finally:
        h.close()


@pytest.mark.check('B')
def test_body_frame_twist_is_rotated_to_world_velocity(h, monkeypatch):
    seen = {}
    real_step = h.node.fsm.step

    def spy_step(dri, boat_xy, yaw, vel, yaw_rate, wp, now_t):
        seen['vel'] = vel
        return real_step(dri, boat_xy, yaw, vel, yaw_rate, wp, now_t)
    monkeypatch.setattr(h.node.fsm, 'step', spy_step)
    h.node._on_grid(_grid())
    h.set_pose(0.0, 0.0, yaw=math.pi / 2)
    h.pose.twist.twist.linear.x = 1.0           # 선체 전방 1 m/s = 월드 +y
    h.tick()
    assert seen['vel'] == pytest.approx((0.0, 1.0), abs=1e-9)


@pytest.mark.check('C')
@pytest.mark.parametrize('missing', ['grid', 'odom', 'goal'])
def test_missing_input_publishes_stop(monkeypatch, missing):
    h = BehaviorHarness(ObstaclePlanner, monkeypatch)
    try:
        h.activate()
        if missing != 'goal':
            h.set_goal(12.0, 0.0)
        if missing != 'grid':
            h.node._on_grid(_grid())
        if missing != 'odom':
            h.set_pose(0.0, 0.0)
        cmd = h.tick()
        assert cmd is not None and is_stop(cmd)   # None(침묵)이 아니라 정지 명령
    finally:
        h.close()


@pytest.mark.check('C')
def test_deactivation_drops_old_plan(h):
    h.node._on_grid(_grid())
    h.set_pose(0.0, 0.0)
    h.tick(2)
    assert h.node.fsm.plan is not None
    h.activate('done')
    assert h.node.fsm.plan is None and h.node.fsm.state == 'FOLLOW'


@pytest.mark.check('D')
def test_debug_topics_only_with_subscribers(h, monkeypatch):
    h.node._on_grid(_grid([(5.0, 0.0, 0.6)]))
    h.set_pose(0.0, 0.0)
    h.tick(2)
    assert h.node.plan_pub.msgs == [] and h.node.dri_pub.msgs == []
    monkeypatch.setattr(h.node, 'count_subscribers', lambda topic: 1)
    h.tick()
    path = h.node.plan_pub.msgs[-1]
    assert len(path.poses) == len(h.node.fsm.plan.samples)
    dri = h.node.dri_pub.msgs[-1]
    assert dri.info.width == SIZE and max(dri.data) > 0
    assert len(h.node.cps_pub.msgs[-1].markers) == 7     # waypoint 1 + 제어점 6
