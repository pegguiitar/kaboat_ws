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
                 '_update_shapes', '_shape_ok', '_entry_pivot', '_go_exit'):
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


def test_entry_pivots_in_place_when_heading_far_off():
    # 진입 경로는 동쪽(+x)인데 배는 서쪽(180°)을 봄 → 전진 없이 제자리 회전, 맞으면 LOS 진입
    state = make_state('red', [buoy('red', 52.0, 71.0)], boat=(45.0, 70.0), yaw=math.pi)
    cmd = SearchCircler.compute_cmd(state)
    assert state.calls == [] and cmd.linear.x == 0.0 and abs(cmd.angular.z) > 0.1
    assert '제자리 회전' in state._last_status
    first = state._entry_path
    state.odom.pose.pose.orientation.z = math.sin(0.15 / 2)          # 선수 8.6° — 20° 안
    state.odom.pose.pose.orientation.w = math.cos(0.15 / 2)
    SearchCircler.compute_cmd(state)
    assert state.calls and state.calls[-1][0] is first                 # 같은 진입 경로를 LOS 로
    state.odom.pose.pose.orientation.z = 1.0                           # 다시 틀어져도 회전은 한 번만
    state.odom.pose.pose.orientation.w = 0.0
    SearchCircler.compute_cmd(state)
    assert state.calls[-1][0] is first


def test_entry_no_pivot_when_heading_close():
    state = make_state('red', [buoy('red', 52.0, 71.0)], boat=(45.0, 70.0), yaw=math.radians(40))
    SearchCircler.compute_cmd(state)
    assert state.calls and not state.calls[0][0].closed                # 바로 LOS 진입


def test_entry_corners_are_rounded_so_los_keeps_speed():
    # 2026-11-13 배치, 시작 자세에서 빨강(시계) — 검정을 비키는 경유점이 있는 진입.
    # 꺾은선이면 꼭짓점 κ 4.5 → LOS 최저 속도, 둥글게 하면 |κ| ≤ 1/orbit_radius 근처
    from kaboat_behaviors.search_circler import polyline_path
    from kaboat_behaviors.search_orbit import tangent_entry
    boat, red, black, green = (168.9, -9.53), (174.5, -1.2), (169.3, -5.9), (166.5, -0.2)
    pts = tangent_entry(boat, red, 2.0, 'cw', [black, green], clearance=1.5)
    assert len(pts) == 4                                          # 경유점 있음
    sharp = polyline_path(pts)
    round_ = polyline_path(pts, fillet_radius=2.0)
    assert np.max(np.abs(sharp.curvatures)) > 3.0
    assert np.max(np.abs(round_.curvatures)) <= 1.0 / 1.7 + 1e-9   # 접점 쪽 짧은 구간 때문에 반경 조금 줄어듦
    assert np.allclose(round_.samples[0], pts[0]) and np.allclose(round_.samples[-1], pts[-1])
    steps = np.linalg.norm(np.diff(round_.samples, axis=0), axis=1)
    assert steps.max() < 0.06                                      # 거의 등간격
    dmin = np.min(np.hypot(round_.samples[:, 0] - black[0], round_.samples[:, 1] - black[1]))
    assert dmin >= 1.5 - 1e-6                                      # 깎아도 검정 clearance 유지


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
    move(state, 42.0, 70.0)                          # 바람에 3 m 밀림 → 처음 자리로 되돌아감
    cmd = SearchCircler.compute_cmd(state)
    assert state._hold[:2] == (45.0, 70.0)
    assert DEFAULT_MAX_LINEAR < cmd.linear.x <= 0.25 + 1e-9


def test_look_overshoot_near_hold_keeps_heading_without_turning_back():
    # 2026-11-14 실주행: 활성화 자리를 1.1 m 지나쳐 서자 뒤쪽 자리로 돌아가려 제자리에서 약 1.3 바퀴 돌았다.
    # 2 m 안이면 위치는 놔두고 선수각(영역 쪽)만 — 이미 영역 쪽을 보면 전진·회전 거의 0
    area_dir = math.atan2(10.0, 15.0)
    state = make_state('red', [], search_center=(60.0, 80.0), look_sec=15.0, yaw=area_dir)
    SearchCircler.compute_cmd(state)
    ahead = (45.0 + 1.1 * math.cos(area_dir), 70.0 + 1.1 * math.sin(area_dir))
    move(state, *ahead)
    for _ in range(3):
        cmd = SearchCircler.compute_cmd(state)
        assert cmd.linear.x == 0.0 and abs(cmd.angular.z) < 0.05
    assert state._hold[:2] == (45.0, 70.0)           # 기준 자리는 그대로 (멀리 밀리면 되돌아감)


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
                        lambda cmd, grid, odom, ignore=None, occ_threshold=50: seen.append(cmd.linear.x) or cmd)
    state = make_state('red', [], search_center=(60.0, 70.0), look_sec=15.0)
    SearchCircler.compute_cmd(state)          # 제자리 — 선수각만 (이미 영역 쪽) → 반발 안 걸림
    assert seen == []
    move(state, 42.0, 70.0)                   # 3 m 밀림 → 제자리로 전진 → 반발
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
    # (45, 70) 에 3×3 칸 부표, 5 m 떨어진 y = 75 에 5 m 벽 (x 42~47) — 두 번째 unknown 이 그 위.
    # 배는 둘 다 4.3 m (모양은 배 3–6 m 안에서만 잰다)
    buoy_cells = [(25 + di, 50 + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)]
    wall_cells = [(i, 75) for i in range(10, 35)]
    boat = (41.5, 72.5)
    state = make_state('black', [idbuoy('unknown', 45.0, 70.0, 1),
                                 idbuoy('unknown', 45.0, 75.0, 2)],
                       boat=boat, shape_check=True)
    state.occupancy_grid = real_grid(buoy_cells + wall_cells)
    for k in range(6):
        state._grid_at = 100.0 + k                     # 새 지도 메시지마다 한 번씩
        SearchCircler._update_shapes(state, boat, (50.0, 70.0))
    SearchCircler._update_shapes(state, boat, (50.0, 70.0))   # 같은 지도 — 안 셈
    assert len(state._shape_hist[1]) == 6
    assert SearchCircler._shape_ok(state, idbuoy('unknown', 45.0, 70.0, 1)) is True
    assert SearchCircler._shape_ok(state, idbuoy('unknown', 45.0, 75.0, 2)) is False
    assert SearchCircler._shape_ok(state, idbuoy('unknown', 0, 0, 9)) is None


def test_no_shape_votes_when_boat_too_close():
    # 배 1 m — 덩어리가 배 옆에서 번지므로 안 잰다 (shape_min_range 3 m)
    buoy_cells = [(25 + di, 50 + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)]
    state = make_state('black', [idbuoy('unknown', 45.0, 70.0, 1)], boat=(44.0, 70.0),
                       shape_check=True)
    state.occupancy_grid = real_grid(buoy_cells)
    state._grid_at = 100.0
    SearchCircler._update_shapes(state, (44.0, 70.0), (50.0, 70.0))
    assert 1 not in state._shape_hist


# ---------- 완료 뒤: 출구 쪽까지 원을 이어 돈다 ----------

def test_done_keeps_circling_until_exit_side():
    # 출구 (40, 71) 는 부표 (52, 71) 서쪽. 배가 동쪽 원 위에서 끝나면 곧장 가면 부표를 가로지른다
    state = make_state('red', [buoy('red', 52.0, 71.0)], boat=(54.0, 71.0))
    state.goal.pose.position.x, state.goal.pose.position.y = 40.0, 71.0
    SearchCircler.compute_cmd(state)                 # 고정 → 원
    circle = state.calls[-1][0]
    assert circle.closed
    state.fsm.phase = 'DONE'
    SearchCircler.compute_cmd(state)
    assert state.calls[-1][0] is circle and state.completed     # 완료 신호는 바로, 원은 이어 감
    move(state, 50.0, 71.0)                          # 서쪽(출구 쪽)에 옴 → 출구 직선을 LOS 로
    SearchCircler.compute_cmd(state)
    exit_path, ratio, turn = state.calls[-1]
    assert not exit_path.closed and (ratio, turn) == (0.25, 0.31)   # exit_thrust·orbit_turn
    assert np.allclose(exit_path.samples[0], [50.0, 71.0])
    assert np.allclose(exit_path.samples[-1], [40.0, 71.0])
    move(state, 54.0, 71.0)                          # 한 번 벗어나면 다시 원으로 안 돌아감
    SearchCircler.compute_cmd(state)
    assert state.calls[-1][0] is exit_path


def test_exit_line_detours_around_other_buoy():
    # 빨강 (52, 71) 을 돌고 서쪽 출구 (40, 71) 로 — 직선 위 (46, 71.1) 에 초록. 경유점으로 1.5 m 넘게 비킨다
    green = (46.0, 71.1)
    state = make_state('red', [buoy('red', 52.0, 71.0), buoy('green', *green)], boat=(50.0, 71.0))
    state.goal.pose.position.x, state.goal.pose.position.y = 40.0, 71.0
    SearchCircler.compute_cmd(state)
    state.fsm.phase = 'DONE'
    state._exit_clear = True
    SearchCircler.compute_cmd(state)
    path = state.calls[-1][0]
    assert len(state._exit_points) == 3                              # 경유점 있음
    d = np.hypot(path.samples[:, 0] - green[0], path.samples[:, 1] - green[1])
    assert d.min() >= 1.5 - 1e-6
    assert np.allclose(path.samples[-1], [40.0, 71.0])


def test_exit_stops_inside_arrive_radius():
    state = make_state('red', [buoy('red', 52.0, 71.0)], boat=(54.0, 71.0))
    state.goal.pose.position.x, state.goal.pose.position.y = 40.0, 71.0
    SearchCircler.compute_cmd(state)
    state.fsm.phase = 'DONE'
    state._exit_clear = True
    move(state, 41.0, 71.0)                          # search_end 1 m — arrive_radius 1.5 안
    n = len(state.calls)
    cmd = SearchCircler.compute_cmd(state)
    assert cmd == Twist() and len(state.calls) == n


def test_search_node_uses_measured_speed_curve_and_can_turn_it_off():
    """search 노드만 LOS 속도 → 명령을 실물 곡선으로 (2026-11-14). search.measured_speed:=false 면 예전."""
    import rclpy
    for args, expected in (([], True), (['--ros-args', '-p', 'search.measured_speed:=false'], False)):
        rclpy.init(args=args)
        try:
            node = SearchCircler()
            try:
                assert node.params.measured_speed is expected
                assert node._los_follower.measured_speed is expected
                assert node._los_follower.ratio_limits_only is True
            finally:
                node.destroy_node()
        finally:
            rclpy.shutdown()
    assert SearchParams().measured_speed is True


# ---------- 반발 조향은 점유 75 이상 칸만 (배 지나간 자리 잡음 칸 거르기) ----------

def odom_at(x, y, yaw=0.0):
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))


def one_cell_grid(value, cell=(52, 51)):
    # 배 (45, 70) 앞 왼쪽 2.4 m 쯤 (x 50.5, y 70.3) 칸 하나 — real_grid 와 같은 0.2 m·원점 (40, 60)
    data = [0] * (100 * 100)
    data[cell[1] * 100 + cell[0]] = value
    return SimpleNamespace(header=SimpleNamespace(frame_id='odom'), data=data, info=SimpleNamespace(
        width=100, height=100, resolution=0.2,
        origin=SimpleNamespace(position=SimpleNamespace(x=40.0, y=60.0))))


@pytest.mark.parametrize('value, threshold, steered', [
    (65, 50, True),     # 공용 기본 50 — 잡음 한두 번 칸에도 꺾임
    (65, 75, False),    # 탐색 75 — 2026-11-14 실주행 지도의 부표 아닌 칸은 전부 ≤ 70
    (100, 75, True),    # 부표 몸통(대부분 100)은 그대로 비킨다
])
def test_repulsion_threshold(value, threshold, steered):
    from kaboat_behaviors.obstacle_avoidance_utils import apply_repulsion
    cmd = Twist()
    cmd.linear.x = 0.25
    out = apply_repulsion(cmd, one_cell_grid(value), odom_at(48.0, 70.0), occ_threshold=threshold)
    assert (out.angular.z < -0.01) is steered      # 칸이 왼쪽 앞 → 오른쪽으로


def test_search_passes_repulse_threshold_everywhere(monkeypatch):
    seen = []
    monkeypatch.setattr(search_circler_mod, 'apply_repulsion',
                        lambda cmd, grid, odom, ignore=None, occ_threshold=50:
                        seen.append(occ_threshold) or cmd)
    state = make_state('red', [buoy('red', 52.0, 71.0)], boat=(54.0, 71.0))
    SearchCircler.compute_cmd(state)                 # 선회
    state2 = make_state('green', [], boat=(40.0, 70.0))
    SearchCircler.compute_cmd(state2)                # SEEK — 영역 중심으로
    assert seen and set(seen) == {75}
