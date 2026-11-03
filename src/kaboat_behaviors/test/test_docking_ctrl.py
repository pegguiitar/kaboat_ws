"""docking_ctrl 노드 — 입구·표식 좌표 검증, 진입 경로 발행, FSM·추력 배선.

FSM 단계별 판단은 test_docking_fsm.py, 경로 계산은 test_dock_target.py 가 확인한다.
여기서는 노드가 좌표를 언제 받아들이고 언제 버리는지, 그리고 명령으로 어떻게 잇는지 본다.
"""
import json
import math

import pytest
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from kaboat_behaviors.docking_ctrl import DockingCtrl
from kaboat_behaviors.docking_fsm import DockState
from node_helpers import BehaviorHarness, Spy, is_stop


def _make(monkeypatch, *params):
    h = BehaviorHarness(DockingCtrl, monkeypatch, params)
    h.node.waypoints_pub = Spy()
    h.activate()
    h.set_goal(10.0, 0.0)
    return h


@pytest.fixture
def h(monkeypatch):
    harness = _make(monkeypatch, 'dock.motion_enabled:=true')
    yield harness
    harness.close()


def _point(h, x, y, z=0.0, frame='base_link', age=0.0):
    msg = PointStamped()
    msg.header.frame_id = frame
    t = h.clock.t - age
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    msg.point.x, msg.point.y, msg.point.z = float(x), float(y), float(z)
    return msg


def _see_dock(h, marker=(6.0, 0.0), mouth=(3.0, 0.0), **kw):
    """같은 스탬프의 표식·입구 좌표를 넣는다 (dock_mark_detector 출력 흉내)."""
    h.node._on_marker(_point(h, *marker, **kw))
    h.node._on_entrance(_point(h, *mouth, **{k: v for k, v in kw.items() if k != 'z'},
                               z=kw.get('z', 0.0)))


def _last_path_x(h):
    return [p.pose.position.x for p in h.node.waypoints_pub.msgs[-1].poses]


@pytest.mark.check('A')
def test_approach_drives_to_staging_goal(h):
    h.set_pose(0.0, 0.0)
    cmd = h.tick()
    assert h.node.fsm.state == DockState.APPROACH
    assert cmd.linear.x > 0.0


@pytest.mark.check('A')
def test_fresh_marker_and_mouth_publish_three_waypoints(h):
    h.set_pose(0.0, 0.0)
    _see_dock(h)
    h.node._on_odom(h.pose)
    h.node.compute_cmd()
    assert _last_path_x(h) == pytest.approx([1.5, 3.0, 5.0])   # 대기점, 입구, 접안점


@pytest.mark.check('A')
def test_plan_at_staging_point_leaves_approach(h):
    h.set_pose(9.5, 0.0)     # 대기점(10, 0) 1.5m 이내
    for _ in range(10):
        _see_dock(h)
        h.tick()
    assert h.node.fsm.state in (DockState.PRE_ENTRY, DockState.ALIGN, DockState.TO_MOUTH)


@pytest.mark.check('B')
def test_mismatched_stamps_or_raised_mouth_give_no_plan(h):
    h.set_pose(0.0, 0.0)
    h.node._on_marker(_point(h, 6.0, 0.0))
    h.node._on_entrance(_point(h, 3.0, 0.0, age=0.1))
    h.node.compute_cmd()
    assert h.node.waypoints_pub.msgs[-1].poses == []
    _see_dock(h, z=0.2)     # 입구가 수면 높이(±5cm)를 벗어남
    h.node.compute_cmd()
    assert h.node.waypoints_pub.msgs[-1].poses == []


@pytest.mark.check('B')
def test_dock_params_are_exposed_as_ros_parameters(monkeypatch):
    h = _make(monkeypatch, 'dock.motion_enabled:=true', 'dock.berth_distance:=1.5')
    try:
        assert h.node.fsm.p.berth_distance == 1.5
        h.set_pose(0.0, 0.0)
        _see_dock(h)
        h.node._on_odom(h.pose)
        h.node.compute_cmd()
        assert _last_path_x(h)[-1] == pytest.approx(4.5)
    finally:
        h.close()


@pytest.mark.check('C')
@pytest.mark.parametrize('kw', [{'frame': 'camera_link'}, {'age': 1.0}])
def test_wrong_frame_or_old_points_are_rejected(h, kw):
    _see_dock(h, **kw)
    assert h.node.marker is None and h.node.entrance is None


@pytest.mark.check('C')
def test_invalid_detection_status_clears_points(h):
    _see_dock(h)
    assert h.node.marker is not None
    h.node._on_detection_status(String(data=json.dumps({'valid': False})))
    assert h.node.marker is None and h.node.entrance is None
    _see_dock(h)
    h.node._on_detection_status(String(data='not json'))
    assert h.node.marker is None


@pytest.mark.check('C')
def test_motion_disabled_or_stale_odom_gives_stop(monkeypatch):
    h = _make(monkeypatch)    # dock.motion_enabled 기본 false
    try:
        h.set_pose(0.0, 0.0)
        _see_dock(h)
        cmd = h.tick()
        assert is_stop(cmd)
        assert len(h.node.waypoints_pub.msgs[-1].poses) == 3   # 경로는 계속 발행
    finally:
        h.close()


@pytest.mark.check('C')
def test_stale_odom_gives_stop(h):
    h.set_pose(0.0, 0.0)
    h.node._on_odom(h.pose)
    h.clock.t += 1.0
    assert is_stop(h.node.compute_cmd())


@pytest.mark.check('D')
def test_reactivation_resets_fsm_and_points(h):
    h.set_pose(9.5, 0.0)
    for _ in range(5):
        _see_dock(h)
        h.tick()
    assert h.node.fsm.state != DockState.APPROACH
    h.activate('search')
    h.activate('dock')
    assert h.node.fsm.state == DockState.APPROACH
    assert h.node.marker is None
    assert DockingCtrl.CMD_TOPIC == '/cmd/dock'
