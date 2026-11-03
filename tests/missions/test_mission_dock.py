"""L2 Dock — 표식·입구 좌표 → dock_target → docking_fsm → docking_ctrl → 선체 모델.

dock_mark_detector(YOLO) 출력 대신, 실제 도킹 슬롯 기하에서 표식 중심과 입구 중앙을
선체 좌표(base_link)로 바꿔 넣는다. 카메라 시야(±43°, 12 m) 안에 있을 때만 보인다.

  대기점(미션 목표) (0, 0) → 입구 (3.5, 0) → 접안 (5.0, 0) [표식 (6.0, 0) 에서 1 m]
  → 정지 → 후진해 대기점 1.8 m 이내로 돌아오면 완료.
"""
import math

import pytest
from geometry_msgs.msg import PointStamped

from kaboat_behaviors.docking_ctrl import DockingCtrl
from kaboat_behaviors.docking_fsm import DockState
from tests.missions.harness import (
    LidarGrid, MissionSim, Scenario, close, make_node, save_figure, seeds, tank_walls)
from tests.sim.boat_model import BoatState
from tests.sim.fake_sensors import Segment, World

STAGING = (0.0, 0.0)
MOUTH = (3.5, 0.0)
MARKER = (6.0, 0.0)
BERTH = (5.0, 0.0)
SLOT_HALF = 0.6
SLOT = [Segment(MOUTH[0], SLOT_HALF, MARKER[0], SLOT_HALF),
        Segment(MOUTH[0], -SLOT_HALF, MARKER[0], -SLOT_HALF),
        Segment(MARKER[0], -SLOT_HALF, MARKER[0], SLOT_HALF)]
BOUNDS = (-6.0, -6.0, 9.0, 6.0)
WORLD = World(segments=SLOT + tank_walls(*BOUNDS), bounds=BOUNDS)
START = BoatState(x=-4.0, y=0.0)
HALF_FOV = math.radians(43.5)
MAX_RANGE = 12.0
TIME_LIMIT = 120.0
HULL_CLEARANCE = 0.25    # 선체 중심 ↔ 슬롯 벽 최소 거리 [m] (선체 반폭 ≈ 0.23 m)
BERTH_TOLERANCE = 0.3    # HOLD 중 접안점 오차 [m]


class DockCamera:
    """슬롯 기하 → /dock/marker_base, /dock/entrance_base (같은 스탬프)."""

    def __init__(self):
        self.alive = True

    def _body(self, s, x, y):
        dx, dy = x - s.x, y - s.y
        c, si = math.cos(s.yaw), math.sin(s.yaw)
        return c * dx + si * dy, -si * dx + c * dy

    def __call__(self, sim):
        if not self.alive:
            return
        s = sim.boat.state
        mx, my = self._body(s, *MARKER)
        if mx <= 0.0 or math.hypot(mx, my) > MAX_RANGE or abs(math.atan2(my, mx)) > HALF_FOV:
            return
        stamp = sim.clock_msg()
        for kind, (x, y) in (('marker', (mx, my)), ('entrance', self._body(s, *MOUTH))):
            msg = PointStamped()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = stamp
            msg.point.x, msg.point.y = float(x), float(y)
            getattr(sim.node, f'_on_{kind}')(msg)


def _run(monkeypatch, scenario, cut_camera_in=None):
    from tests.sim.ros_adapters import FakeClock
    clock = FakeClock()
    node = make_node(DockingCtrl, monkeypatch, clock, ['dock.motion_enabled:=true'])
    lidar = LidarGrid(WORLD)
    camera = DockCamera()
    try:
        sim = MissionSim(node, START, scenario, WORLD, hooks=[lidar, camera], clock=clock,
                         status_fn=lambda n: n.fsm.state.value)
        sim.activate(STAGING)
        if cut_camera_in is not None:
            sim.run(max_time=TIME_LIMIT, until=lambda s: s.node.fsm.state == cut_camera_in)
            sim.cut_at = sim.t
            camera.alive = False
            sim.run(max_time=sim.t + 10.0, until=lambda s: s.t >= s.cut_at + 10.0)
        else:
            sim.run(max_time=TIME_LIMIT + 30.0, after_complete=1.0,
                    until=lambda s: s.node.fsm.state == DockState.ABORT)
        return sim
    finally:
        lidar.close()
        close(node)


def _states(rec):
    out = []
    for s in rec.samples:
        if not out or out[-1] != s.status:
            out.append(s.status)
    return out


def _check_mission(sim):
    rec = sim.rec
    states = _states(rec)
    done = rec.first_event_time('complete')
    assert done is not None, f'완료 신호 없음 — 상태 {states}'
    assert done <= TIME_LIMIT, f'완료 {done:.1f}s > {TIME_LIMIT}s'
    hold = [s for s in rec.samples if s.status == 'HOLD']
    assert hold, f'HOLD 없음 — 상태 {states}'
    err = max(math.hypot(s.x - BERTH[0], s.y - BERTH[1]) for s in hold)
    assert err <= BERTH_TOLERANCE, f'접안 오차 {err:.2f}m'
    wall = rec.min_clearance(World(segments=SLOT))
    assert wall >= HULL_CLEARANCE, f'슬롯 벽과 {wall:.2f}m (선체 중심 기준)'
    assert rec.always_inside(WORLD)


@pytest.mark.check('S1')
def test_dock_nominal(monkeypatch):
    sim = _run(monkeypatch, Scenario.nominal())
    save_figure('L2_dock_S1', sim.rec, WORLD, STAGING)
    _check_mission(sim)


@pytest.mark.check('S2')
@pytest.mark.parametrize('seed', seeds())
def test_dock_disturbed(monkeypatch, seed):
    sim = _run(monkeypatch, Scenario.disturbed(seed))
    save_figure(f'L2_dock_S2_seed{seed}', sim.rec, WORLD, STAGING)
    _check_mission(sim)


@pytest.mark.check('S3')
def test_dock_marker_loss_aborts_and_stops(monkeypatch):
    sim = _run(monkeypatch, Scenario.failure(), cut_camera_in=DockState.PRE_ENTRY)
    assert sim.cut_at < TIME_LIMIT, '대기점 이동(PRE_ENTRY)까지 가지 못함'
    # mark_lost_timeout 0.7s + 제어 주기 여유
    assert sim.rec.stopped_after(sim.cut_at + 1.0), '표식 끊김 1초 뒤에도 추력 명령'
    assert sim.rec.samples[-1].status == 'ABORT'
    assert sim.rec.first_event_time('complete') is None
