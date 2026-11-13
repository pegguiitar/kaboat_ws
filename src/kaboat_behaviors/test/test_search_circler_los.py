"""The search behavior orbits the locked target buoy through the shared LOS."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import pytest

import kaboat_behaviors.search_circler as search_circler_mod
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
    params.setdefault('require_grid', False)   # 지도 규칙은 아래 따로 시험
    params.setdefault('shape_check', False)    # 모양 검사는 test_search_orbit 에서
    params.setdefault('black_needs_field', False)
    state = SimpleNamespace(
        goal=goal, odom=odom, occupancy_grid=None, buoys=buoys,
        params=SearchParams(target_color=color, **params), fsm=None,
        search_center=search_center, _warned_no_center=False, _hold=None, _grid_at=None,
        _shape_hist={}, _shape_grid_at=None,
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
    for name in ('_status', '_go_goal', '_area_xy', '_plan_entry', '_hold_pose', '_grid_ok',
                 '_update_shapes', '_shape_ok'):
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


def test_area_falls_back_to_search_end_without_center():
    # search_entry·search_center 없는 시뮬·mission_params 흐름 — 예전처럼 search_end 중심
    state = make_state('green', [buoy('green', 55.0, 70.0)], search_center=None)
    SearchCircler.compute_cmd(state)
    assert state.fsm.area == (50.0, 70.0)
    assert state.fsm.center == (55.0, 70.0)


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


# ---------- 점유지도 필수 (gate 와 같은 규칙) ----------

def grid_msg(frame='odom'):
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame))


@pytest.mark.parametrize('grid, age', [(None, None), (grid_msg(), 1.0), (grid_msg('map'), 0.0)])
def test_stops_without_fresh_odom_grid(grid, age):
    state = make_state('red', [buoy('red', 52.0, 71.0)], require_grid=True, grid_timeout_sec=0.7)
    state.occupancy_grid = grid
    state._grid_at = None if age is None else search_circler_mod.time.monotonic() - age
    cmd = SearchCircler.compute_cmd(state)
    assert cmd == Twist() and state.calls == [] and state.fsm is None
    assert '점유지도' in state._last_status


def test_runs_with_fresh_grid():
    state = make_state('red', [buoy('red', 52.0, 71.0)], boat=(54.0, 71.0), require_grid=True)
    state.occupancy_grid = grid_msg()
    state._grid_at = search_circler_mod.time.monotonic()
    SearchCircler.compute_cmd(state)
    assert state.calls and state.calls[0][0].closed


# ---------- LOOK 반발: 전진할 때만 ----------

def test_look_repulsion_only_while_moving(monkeypatch):
    seen = []
    monkeypatch.setattr(search_circler_mod, 'apply_repulsion',
                        lambda cmd, grid, odom, ignore=None: seen.append(cmd.linear.x) or cmd)
    state = make_state('red', [], search_center=(60.0, 70.0), look_sec=15.0)
    SearchCircler.compute_cmd(state)          # 제자리 — 선수각만 (이미 영역 쪽) → 반발 안 걸림
    assert seen == []
    move(state, 43.0, 70.0)                   # 2 m 밀림 → 제자리로 전진 → 반발
    SearchCircler.compute_cmd(state)
    assert len(seen) == 1 and seen[0] > 0.0


# ---------- 모양 판정 (지도 → 부표별 다수결) ----------

def real_grid(cells, size=100, res=0.2, origin=(40.0, 60.0)):
    data = [0] * (size * size)
    for i, j in cells:
        data[j * size + i] = 100
    return SimpleNamespace(header=SimpleNamespace(frame_id='odom'), data=data, info=SimpleNamespace(
        width=size, height=size, resolution=res,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1]))))


def idbuoy(color, x, y, bid):
    return SimpleNamespace(color=color, position=SimpleNamespace(x=x, y=y), id=bid)


def test_shape_votes_accumulate_per_new_grid():
    # (45, 70) 에 3×3 칸 부표, 5 m 떨어진 y = 75 에 5 m 벽 (x 42~47) — 두 번째 unknown 이 그 위
    buoy_cells = [(25 + di, 50 + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)]
    wall_cells = [(i, 75) for i in range(10, 35)]
    state = make_state('black', [idbuoy('unknown', 45.0, 70.0, 1),
                                 idbuoy('unknown', 45.0, 75.0, 2)],
                       boat=(44.0, 70.0), shape_check=True)
    state.occupancy_grid = real_grid(buoy_cells + wall_cells)
    for k in range(6):
        state._grid_at = 100.0 + k                     # 새 지도 메시지마다 한 번씩
        SearchCircler._update_shapes(state, (44.0, 70.0), (50.0, 70.0))
    SearchCircler._update_shapes(state, (44.0, 70.0), (50.0, 70.0))   # 같은 지도 — 안 셈
    assert len(state._shape_hist[1]) == 6
    assert SearchCircler._shape_ok(state, idbuoy('unknown', 45.0, 70.0, 1)) is True
    assert SearchCircler._shape_ok(state, idbuoy('unknown', 45.0, 75.0, 2)) is False
    assert SearchCircler._shape_ok(state, idbuoy('unknown', 0, 0, 9)) is None
