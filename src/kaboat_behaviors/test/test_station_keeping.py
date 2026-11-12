"""station(위치유지) 순수 계산 — 전용 지도·부표·중심·입구·가상 앵커·폐루프."""

import math

import numpy as np
import pytest

from kaboat_behaviors.behavior_base import DEFAULT_MAX_LINEAR, WAYPOINT_MAX_ANGULAR
from kaboat_behaviors.los_controller import BSPLINE_TRACK_YAW_KD, BSPLINE_TRACK_YAW_KP
from kaboat_behaviors.station_keeping import (
    StationFsm, StationMap, StationParams, anchor_command, choose_entry_gate,
    entry_target, estimate_center, find_buoys, track_gate)
from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams, waypoint)

CENTER = (10.0, 0.0)
SQUARE = [(8.5, -1.5), (8.5, 1.5), (11.5, -1.5), (11.5, 1.5)]
BUOY_RADIUS = 0.2
FOV = math.radians(60.0)            # 라이다는 앞만 본다 (±60°)
BEAMS = np.linspace(-FOV, FOV, 241)
RANGE_MAX = 20.0
SHORE = 15.0    # 부표에 안 맞은 빔은 15 m 밖 둑에 맞는다 (무반사 inf 빔은 지도가 안 씀)


def scan(lidar_xy, yaw, buoys, radius=BUOY_RADIUS):
    """원 기둥 부표에 대한 2D 레이캐스트 → ranges (부표에 안 맞으면 SHORE)."""
    ranges = np.full(BEAMS.size, SHORE)
    ox, oy = lidar_xy
    for k, a in enumerate(BEAMS):
        dx, dy = math.cos(yaw + a), math.sin(yaw + a)
        for bx, by in buoys:
            fx, fy = ox - bx, oy - by
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - radius * radius
            disc = b * b - c
            if disc < 0:
                continue
            t = -b - math.sqrt(disc)
            if 0.0 < t < ranges[k]:
                ranges[k] = t
    ranges[ranges > RANGE_MAX] = np.inf
    return ranges


def add(target, lidar_xy, yaw, buoys):
    args = (scan(lidar_xy, yaw, buoys), -FOV, BEAMS[1] - BEAMS[0], 0.05, RANGE_MAX,
            lidar_xy, yaw)
    if isinstance(target, StationMap):
        target.integrate_scan(*args)
    else:
        target.add_scan(*args)


# ── 전용 지도 ─────────────────────────────────────────

def test_map_finds_four_buoys_from_inside_square_rotating():
    m = StationMap(CENTER, StationParams())
    for k in range(36):                       # 중심에서 한 바퀴 돌며 앞만 본다
        add(m, CENTER, math.radians(10 * k), SQUARE)
    buoys = find_buoys(m)
    assert len(buoys) == 4
    for bx, by in SQUARE:
        assert min(math.hypot(b.x - bx, b.y - by) for b in buoys) < 0.25


def test_far_hits_beyond_trust_range_are_ignored():
    m = StationMap(CENTER, StationParams())
    for _ in range(5):
        add(m, (2.0, 0.0), 0.0, SQUARE)       # 가까운 두 부표 6.5 m — 신뢰 거리 밖
    assert find_buoys(m) == []


def _buoy_strength_after_misses(free_ratio, misses):
    """부표를 포화시킨 뒤, 부표가 파도에 내려가 빔이 그 위를 지나가는 스캔을 misses 번."""
    m = StationMap(CENTER, StationParams(free_ratio=free_ratio))
    for _ in range(10):
        add(m, CENTER, math.radians(135.0), SQUARE)       # (8.5, 1.5) 정면
    assert len(find_buoys(m)) == 1
    for _ in range(misses):
        add(m, CENTER, math.radians(135.0), [])
    return find_buoys(m)


def test_station_map_keeps_buoys_that_shared_decay_would_erase():
    # 공용 지도 감쇠(비율 1.0)면 20 스캔(2 s)에 지워지지만, station 전용(0.1)은 남는다
    assert _buoy_strength_after_misses(1.0, 20) == []
    assert len(_buoy_strength_after_misses(0.1, 20)) == 1
    assert len(_buoy_strength_after_misses(0.1, 120)) == 1   # 12 s 동안 안 보여도 기억


def test_buoys_outside_search_radius_are_ignored():
    m = StationMap(CENTER, StationParams())
    far = (CENTER[0] + 5.0, CENTER[1])         # 반경 4 m 밖
    for k in range(36):
        add(m, CENTER, math.radians(10 * k), SQUARE + [far])
    assert len(find_buoys(m)) == 4


# ── 중심 추정 ─────────────────────────────────────────

P = StationParams()


def test_center_from_four_three_and_diagonal():
    c, kind = estimate_center(SQUARE, (3.0, 0.0), P)
    assert kind == 'four' and c == pytest.approx(CENTER)
    c, kind = estimate_center(SQUARE[:3], (3.0, 0.0), P)
    assert kind == 'three' and c == pytest.approx(CENTER)
    c, kind = estimate_center([SQUARE[0], SQUARE[3]], (3.0, 0.0), P)
    assert kind == 'diagonal' and c == pytest.approx(CENTER)


def test_center_from_first_side_is_beyond_it_and_hint_keeps_side():
    near = SQUARE[:2]                          # x=8.5 두 개 (배는 x=3 에서 접근)
    c, kind = estimate_center(near, (3.0, 0.0), P)
    assert kind == 'side' and c == pytest.approx(CENTER)
    # 배가 안으로 들어가 변을 지난 뒤에도 직전 중심 쪽을 유지 (배 반대쪽으로 뒤집히지 않음)
    c, _ = estimate_center(near, (10.0, 0.0), P, hint_xy=CENTER)
    assert c == pytest.approx(CENTER)
    assert estimate_center([SQUARE[0]], (3.0, 0.0), P) is None
    assert estimate_center([(8.5, 0.0), (8.5, 0.5)], (3.0, 0.0), P) is None


# ── 입구 ─────────────────────────────────────────────

def test_entry_gate_is_nearest_side_and_entry_aligns_first():
    gate = choose_entry_gate(SQUARE, (3.0, 3.0), P)
    assert gate.midpoint == pytest.approx((8.5, 0.0))
    assert gate.normal == pytest.approx((1.0, 0.0))
    # 수직이등분선에서 3 m 옆 → 입구 앞 정렬점 먼저
    target, stage = entry_target((5.0, 3.0), gate, CENTER, P)
    assert stage == 'ALIGN_GATE' and target == pytest.approx((7.5, 0.0))
    # 이등분선 위면 바로 중심
    target, stage = entry_target((6.0, 0.2), gate, CENTER, P)
    assert stage == 'THROUGH' and target == pytest.approx(CENTER)
    # 대각선 쌍만 있으면 입구 없음
    assert choose_entry_gate([SQUARE[0], SQUARE[3]], (3.0, 0.0), P) is None


def test_gate_tracks_bobbing_buoys_and_keeps_inward_side():
    gate = choose_entry_gate(SQUARE[:2], (3.0, 0.0), P)
    moved = [(8.7, -1.4), (8.4, 1.6)]
    tracked = track_gate(gate, moved, P)
    assert tracked.a == pytest.approx(moved[0]) and tracked.b == pytest.approx(moved[1])
    assert tracked.normal[0] > 0.9
    assert track_gate(gate, [], P) == gate


# ── 가상 앵커 ─────────────────────────────────────────

def test_anchor_forward_reverse_turn_and_deadband():
    lin, ang, stage = anchor_command((9.0, 0.0), 0.0, 0.0, CENTER, 0.0, P)
    assert stage == 'RETURN' and 0 < lin <= P.max_forward and ang == pytest.approx(0.0)
    lin, ang, stage = anchor_command((10.6, 0.0), 0.0, 0.0, CENTER, 0.0, P)
    assert stage == 'BACK' and -P.max_reverse <= lin < 0
    lin, ang, stage = anchor_command((12.0, 0.0), 0.0, 0.0, CENTER, 0.0, P)
    assert stage == 'TURN' and lin == 0.0 and abs(ang) == pytest.approx(P.max_turn)
    lin, ang, stage = anchor_command((10.2, 0.0), 0.0, 0.0, CENTER, 0.0, P)
    assert (lin, ang, stage) == (0.0, 0.0, 'HOLD')
    lin, ang, stage = anchor_command((10.2, 0.0), 0.0, 0.0, CENTER, math.radians(30), P)
    assert stage == 'ALIGN' and lin == 0.0 and ang > 0


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        StationParams(free_ratio=1.5)
    with pytest.raises(ValueError):
        StationParams(min_side=3.5, side_length=3.0)


# ── 폐루프: 흔들리는 부표 + 앞만 보는 라이다 + 해류 ─────────────

WAYPOINT_PARAMS = WaypointParams(
    max_linear=DEFAULT_MAX_LINEAR, max_angular=WAYPOINT_MAX_ANGULAR,
    yaw_kp=BSPLINE_TRACK_YAW_KP, yaw_kd=BSPLINE_TRACK_YAW_KD)


def run_mission(start_xy, start_yaw, goal_xy, seed=0, seconds=180.0, current=(0.04, 0.03)):
    rng = np.random.default_rng(seed)
    p = StationParams()
    fsm = StationFsm(goal_xy, p)
    wp = WaypointController(WAYPOINT_PARAMS)
    x, y, yaw = start_xy[0], start_xy[1], start_yaw
    u = r = 0.0
    dt, tau = 0.1, 0.7
    min_clear = math.inf
    phases = []
    done_at = None
    for k in range(int(seconds / dt)):
        t = k * dt
        # 부표가 흔들림 (±0.15 m), 스캔마다 30% 확률로 각 부표가 수면 아래로 (빔이 지나감)
        bob = [(bx + 0.15 * math.sin(0.7 * t + i), by + 0.15 * math.cos(0.5 * t + 2 * i))
               for i, (bx, by) in enumerate(SQUARE)]
        seen = [b for b in bob if rng.random() > 0.3]
        lidar = (x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
        add(fsm, lidar, yaw, seen)
        d = fsm.step((x, y), yaw, r, t)
        if not phases or phases[-1] != d.phase:
            phases.append(d.phase)
        if d.completed_now:
            done_at = t
        if d.kind == 'waypoint':
            wp.set_goal(waypoint(d.target[0], d.target[1], None, d.thrust_ratio))
            s = wp.step(x, y, yaw, r, t, slow_radius=d.slow_radius)
            lin, ang = s.linear, s.angular
        else:
            lin, ang = d.linear, d.angular
        # 단순 선체: 명령 → 속도 1차 지연 (전진 1.0 → 3 m/s, 회전 1.0 → 2 rad/s)
        u += (3.0 * lin - u) * dt / tau
        r += (2.0 * ang - r) * dt / tau
        yaw += r * dt
        x += (u * math.cos(yaw) + current[0]) * dt
        y += (u * math.sin(yaw) + current[1]) * dt
        min_clear = min(min_clear, *(math.hypot(x - bx, y - by) for bx, by in bob))
        if done_at is not None and t > done_at + 10.0:
            break
    return phases, done_at, (x, y), min_clear, fsm


@pytest.mark.parametrize('start, yaw, goal, seed, current', [
    ((3.0, 0.0), 0.0, (10.0, 0.0), 0, (0.04, 0.03)),             # 정면
    ((4.0, 3.5), -0.5, (10.4, -0.5), 1, (0.04, 0.03)),           # 비스듬히 + 기록 오차 0.6 m
    ((10.0, -7.0), math.pi / 2, (9.6, 0.3), 2, (0.10, -0.06)),   # 다른 변 + 센 옆 해류
    ((16.5, 2.0), math.pi, (10.0, 0.5), 4, (-0.08, 0.10)),       # 반대편 변 + 센 해류
])
def test_closed_loop_enters_holds_and_completes(start, yaw, goal, seed, current):
    phases, done_at, final, min_clear, fsm = run_mission(start, yaw, goal, seed,
                                                         current=current)
    assert phases == ['SEARCH', 'ENTER', 'HOLD']
    assert done_at is not None and done_at < 60.0
    # 완료 후 10 s 더 유지해도 사각형 안 (해류에 밀려도 앵커가 붙잡는다)
    assert math.dist(final, CENTER) < P.hold_radius
    assert min_clear > 0.6                         # 부표와 부딪히지 않음 (선체 반폭 + 여유)
    assert len(find_buoys(fsm.map)) == 4
    assert fsm.center_kind == 'four'
    assert math.dist(fsm.center, CENTER) < 0.3


def test_fallback_to_goal_when_no_buoys_seen():
    p = StationParams()
    fsm = StationFsm(CENTER, p)
    d = fsm.step((9.5, 0.0), 0.3, 0.0, 0.0)
    assert d.phase == 'HOLD' and d.center_kind == 'goal'
    for k in range(1, 52):
        d = fsm.step((9.5, 0.0), 0.3, 0.0, k * 0.1)
        if d.completed_now:
            break
    assert d.completed_now and fsm.completed
    # 완료는 한 번만
    assert not fsm.step((9.5, 0.0), 0.3, 0.0, 6.0).completed_now
