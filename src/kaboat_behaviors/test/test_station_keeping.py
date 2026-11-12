"""station(위치유지) 순수 계산 — 전용 지도·부표·중심·입구·가상 앵커·폐루프."""

import math

import numpy as np
import pytest

from kaboat_behaviors.behavior_base import DEFAULT_MAX_LINEAR, WAYPOINT_MAX_ANGULAR
from kaboat_behaviors.los_controller import BSPLINE_TRACK_YAW_KD, BSPLINE_TRACK_YAW_KP
from kaboat_behaviors import station_keeping
from kaboat_behaviors.station_keeping import (
    Buoy, StationFsm, StationMap, StationParams, anchor_command, choose_entry_gate,
    entry_target, estimate_center, find_buoys, square_clearance, square_corners, track_gate)
from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams, waypoint)

CENTER = (10.0, 0.0)
SQUARE = [(8.5, -1.5), (8.5, 1.5), (11.5, -1.5), (11.5, 1.5)]
SQUARE_ORDERED = [(8.5, -1.5), (11.5, -1.5), (11.5, 1.5), (8.5, 1.5)]   # 둘레 순서
BUOY_RADIUS = 0.2
FOV = math.radians(60.0)            # 라이다는 앞만 본다 (±60°)
BEAMS = np.linspace(-FOV, FOV, 241)
RANGE_MAX = 20.0
SHORE = 15.0    # 부표에 안 맞은 빔은 15 m 밖 둑에 맞는다 (무반사 inf 빔은 지도가 안 씀)


def scan(lidar_xy, yaw, buoys, radius=BUOY_RADIUS, noise=0.0, rng=None):
    """원 기둥 부표에 대한 2D 레이캐스트 → ranges (부표에 안 맞으면 SHORE).

    noise: 빔 중 이 비율을 0.8~6 m 무작위 거리 반사(해수면 노이즈)로 바꾼다.
    """
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
    if noise > 0.0:
        flip = rng.random(BEAMS.size) < noise
        ranges[flip] = rng.uniform(0.8, 6.0, int(flip.sum()))
    return ranges


def add(target, lidar_xy, yaw, buoys, **scan_kw):
    args = (scan(lidar_xy, yaw, buoys, **scan_kw), -FOV, BEAMS[1] - BEAMS[0], 0.05, RANGE_MAX,
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


def _buoy_after_misses(free_ratio, misses, radius=BUOY_RADIUS):
    """물체를 포화시킨 뒤, 파도에 내려가 빔이 그 위를 지나가는 스캔을 misses 번."""
    m = StationMap(CENTER, StationParams(free_ratio=free_ratio))
    for _ in range(10):
        add(m, CENTER, math.radians(135.0), SQUARE, radius=radius)   # (8.5, 1.5) 정면
    found = find_buoys(m)
    for _ in range(misses):
        add(m, CENTER, math.radians(135.0), [])
    return found, find_buoys(m), m


def test_big_cluster_decays_at_ten_percent_of_shared_rate():
    found, _, m = _buoy_after_misses(0.1, 0)
    assert len(found) == 1 and m.big_mask().sum() >= StationParams().big_min_cells
    # 공용 감쇠(비율 1.0)면 40 스캔(4 s)에 지워지지만, 큰 덩어리(0.1)는 20 s 뒤에도 남는다
    assert _buoy_after_misses(1.0, 40)[1] == []
    assert len(_buoy_after_misses(0.1, 40)[1]) == 1
    assert len(_buoy_after_misses(0.1, 200)[1]) == 1


def test_small_cluster_decays_like_shared_map():
    # 작은 물체(반지름 3 cm → 1~3칸 = 노이즈 크기)는 큰 덩어리가 아니라 공용과 같이 깎인다
    occ = StationParams().occ_logodds
    _, _, m = _buoy_after_misses(0.1, 0, radius=0.03)
    assert 1 <= int((m.logodds >= occ).sum()) <= 3 and not m.big_mask().any()
    _, _, m = _buoy_after_misses(0.1, 40, radius=0.03)
    assert int((m.logodds >= occ).sum()) == 0


def test_sea_clutter_alone_never_becomes_a_buoy():
    rng = np.random.default_rng(7)
    m = StationMap(CENTER, StationParams())
    for k in range(300):                     # 30 s, 빔 10 % 가 무작위 반사, 배는 천천히 회전
        add(m, CENTER, math.radians(2 * k), [], noise=0.10, rng=rng)
        assert not m.big_mask().any()
    assert find_buoys(m) == []


def test_buoys_outside_search_radius_are_ignored():
    m = StationMap(CENTER, StationParams())
    far = (CENTER[0] + 5.0, CENTER[1])         # 반경 4 m 밖
    for k in range(36):
        add(m, CENTER, math.radians(10 * k), SQUARE + [far])
    assert len(find_buoys(m)) == 4
    assert len(find_buoys(m, near=CENTER)) == 4   # 추정 중심 3 m 반경으로도 5 m 밖은 제외


def test_square_drifted_off_record_keeps_all_four_buoys():
    # 사각형이 기록 지점에서 (2.3, 1.0) m 밀림 → 먼 모서리 (11.5, 1.5) 는 기록 지점에서 4.55 m.
    # 기록 지점 4 m 만 보면 3개뿐이고 출구를 엉뚱한 변으로 잡았다.
    record = (CENTER[0] - 2.3, CENTER[1] - 1.0)
    fsm = StationFsm(record, StationParams())
    for k in range(36):
        add(fsm, CENTER, math.radians(10 * k), SQUARE)
    assert len(find_buoys(fsm.map)) == 3
    fsm.step(CENTER, 0.0, 0.0, 0.0)               # 부표 3개 → 추정 중심 = 대각선 중점
    d = fsm.step(CENTER, 0.0, 0.0, 0.1)           # 추정 중심 3 m 안 → 네 번째도 후보
    assert len(d.buoys) == 4 and d.center_kind == 'four'
    for bx, by in SQUARE:
        assert min(math.hypot(b.x - bx, b.y - by) for b in d.buoys) < 0.25


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


# ── 사각형 (완료 판정 영역) ────────────────────────────

def _same_corners(got, want):
    return sorted(map(tuple, np.round(got, 6))) == sorted(map(tuple, np.round(want, 6)))


def test_square_corners_fill_missing_buoys():
    assert _same_corners(square_corners(SQUARE, CENTER, P), SQUARE)
    for missing in range(4):                    # 3개 → 빠진 꼭짓점을 평행사변형으로
        seen = [q for k, q in enumerate(SQUARE) if k != missing]
        c, _ = estimate_center(seen, (3.0, 0.0), P)
        assert _same_corners(square_corners(seen, c, P), SQUARE)
    side = SQUARE[:2]                           # 한 변 → 중심 쪽 맞은편 변
    assert _same_corners(square_corners(side, CENTER, P), SQUARE)
    diagonal = [SQUARE[0], SQUARE[3]]           # 대각선 → 90° 돌린 대각선
    assert _same_corners(square_corners(diagonal, CENTER, P), SQUARE)
    assert square_corners([SQUARE[0]], CENTER, P) is None
    assert square_corners(SQUARE, None, P) is None


def test_square_clearance_is_inward_distance_to_nearest_side():
    sq = square_corners(SQUARE, CENTER, P)
    assert square_clearance(CENTER, sq) == pytest.approx(1.5)
    assert square_clearance((11.0, 0.3), sq) == pytest.approx(0.5)
    assert square_clearance((12.0, 0.0), sq) == pytest.approx(-0.5)      # 밖
    tilted = [(10 + 1.5 * math.cos(a), 1.5 * math.sin(a))                # 45° 돌린 사각형
              for a in np.radians([45, 135, 225, 315])]
    assert square_clearance(CENTER, tilted) == pytest.approx(1.5 / math.sqrt(2))


@pytest.fixture
def exact_buoys(monkeypatch):
    """전용 지도 대신 부표 위치를 그대로 준다 (지도 추정의 수 cm 치우침 없이 판정만 본다)."""
    def use(points):
        monkeypatch.setattr(station_keeping, 'find_buoys',
                            lambda _map, _near=None: [Buoy(x, y, 1.0) for x, y in points])
    return use


def _hold_until_complete(fsm, boat, seconds=6.0):
    for k in range(int(seconds / 0.1) + 1):
        d = fsm.step(boat, 0.0, 0.0, k * 0.1)
        if d.completed_now:
            return d
    return d


def _fsm_holding():
    fsm = StationFsm(CENTER, StationParams())
    fsm.step(CENTER, 0.0, 0.0, 0.0)              # 중심에서 시작 → 바로 HOLD
    assert fsm.phase == 'HOLD'
    return fsm


def test_hold_completes_only_inside_square_with_margin(exact_buoys):
    exact_buoys(SQUARE)
    # 중심 0.95 m 옆이지만 변까지 0.55 m (< inside_margin 0.6) → 완료 아님 (예전 반경 1 m 이면 완료)
    fsm = _fsm_holding()
    d = _hold_until_complete(fsm, (10.95, 0.0))
    assert not d.inside and d.clearance == pytest.approx(0.55) and not fsm.completed
    # 중심에서 1.2 m(대각선 쪽)지만 두 변까지 0.65 m → 사각형 안이라 완료 (예전 반경 1 m 이면 실패)
    fsm = _fsm_holding()
    d = _hold_until_complete(fsm, (10.85, 0.85))
    assert d.inside and fsm.completed and _same_corners(d.square, SQUARE)


def test_hold_uses_square_completed_from_three_buoys(exact_buoys):
    exact_buoys(SQUARE[:3])                      # (11.5, 1.5) 안 보임
    fsm = _fsm_holding()
    d = _hold_until_complete(fsm, (10.95, 0.95))  # 안 보이는 모서리 쪽 — 두 변까지 0.55 m
    assert not d.inside and not fsm.completed
    d = _hold_until_complete(fsm, (10.3, 0.3))
    assert d.inside and fsm.completed


def test_hold_follows_square_that_drifted_off_recorded_center(exact_buoys):
    # 기록한 중심(goal)이 사각형 서쪽 변 밖에 있어도, 보이는 부표 사각형 안에서 완료한다
    exact_buoys(SQUARE)
    goal = (CENTER[0] - 1.8, CENTER[1])
    fsm = StationFsm(goal, StationParams())
    fsm.step(goal, 0.0, 0.0, 0.0)
    fsm.phase = 'HOLD'
    d = _hold_until_complete(fsm, goal)
    assert not d.inside and d.clearance < 0 and not fsm.completed   # 기록 중심 = 사각형 밖
    d = _hold_until_complete(fsm, CENTER)
    assert d.inside and fsm.completed
    assert fsm.center == pytest.approx(CENTER)


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


def test_anchor_speed_tracks_distance_and_brakes():
    # 목표 1 m 앞, 멈춰 있음 → 전진. 목표 속도(0.4 m/s)로 이미 가면 전진 줄임, 빠르면 후진 제동
    lin_stop, _, _ = anchor_command((9.0, 0.0), 0.0, 0.0, CENTER, None, P, speed=0.0)
    lin_ok, _, stage_ok = anchor_command((9.0, 0.0), 0.0, 0.0, CENTER, None, P, speed=0.4)
    lin_fast, _, stage_fast = anchor_command((9.0, 0.0), 0.0, 0.0, CENTER, None, P, speed=0.8)
    assert lin_stop > lin_ok > 0 and stage_ok == 'RETURN'
    assert -P.max_brake <= lin_fast < 0 and stage_fast == 'BRAKE'
    # 멀면 목표 속도 상한(max_speed) — 상한으로 가고 있으면 그대로 유지
    lin_far, _, _ = anchor_command((4.0, 0.0), 0.0, 0.0, CENTER, None, P, speed=P.max_speed)
    assert lin_far == pytest.approx(P.speed_ff * P.max_speed)
    # 불감대 안인데 아직 움직임 → 제동, 거의 멈췄으면 0
    lin, _, stage = anchor_command((10.2, 0.0), 0.0, 0.0, CENTER, 0.0, P, speed=0.4)
    assert lin < 0 and stage == 'BRAKE'
    assert anchor_command((10.2, 0.0), 0.0, 0.0, CENTER, 0.0, P, speed=0.05)[0] == 0.0
    # 목표를 지나쳐 뒤에 두고 돌아서는 중에도 남은 전진 속도를 제동 (예전엔 0 으로 미끄러짐)
    lin, ang, stage = anchor_command((12.0, 0.0), 0.0, 0.0, CENTER, 0.0, P, speed=0.5)
    assert stage == 'TURN' and lin < 0 and abs(ang) == pytest.approx(P.max_turn)
    # 뒤로 밀리는 중이면 앞으로 제동
    lin, _, stage = anchor_command((12.0, 0.0), 0.0, 0.0, CENTER, 0.0, P, speed=-0.3)
    assert stage == 'TURN' and lin > 0


def test_invalid_params_rejected():
    with pytest.raises(ValueError):
        StationParams(free_ratio=1.5)
    with pytest.raises(ValueError):
        StationParams(min_side=3.5, side_length=3.0)
    with pytest.raises(ValueError):
        StationParams(inside_margin=1.5)            # 한 변 3 m 의 절반 이상이면 들어갈 곳이 없음
    with pytest.raises(ValueError):
        StationParams(inside_margin=-0.1)


# ── 폐루프: 흔들리는 부표 + 앞만 보는 라이다 + 해류 ─────────────

WAYPOINT_PARAMS = WaypointParams(
    max_linear=DEFAULT_MAX_LINEAR, max_angular=WAYPOINT_MAX_ANGULAR,
    yaw_kp=BSPLINE_TRACK_YAW_KP, yaw_kd=BSPLINE_TRACK_YAW_KD)


def run_mission(start_xy, start_yaw, goal_xy, seed=0, seconds=180.0, current=(0.04, 0.03),
                noise=0.05, tau=0.7):
    rng = np.random.default_rng(seed)
    p = StationParams()
    fsm = StationFsm(goal_xy, p)
    wp = WaypointController(WAYPOINT_PARAMS)
    x, y, yaw = start_xy[0], start_xy[1], start_yaw
    u = r = 0.0
    dt = 0.1
    min_clear = math.inf
    hold_side = math.inf            # HOLD 동안 배 중심 ~ 사각형 변 최소 거리
    phases = []
    done_at = None
    for k in range(int(seconds / dt)):
        t = k * dt
        # 부표가 흔들림 (±0.15 m), 스캔마다 30% 확률로 각 부표가 수면 아래로 (빔이 지나감)
        bob = [(bx + 0.15 * math.sin(0.7 * t + i), by + 0.15 * math.cos(0.5 * t + 2 * i))
               for i, (bx, by) in enumerate(SQUARE)]
        seen = [b for b in bob if rng.random() > 0.3]
        lidar = (x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
        add(fsm, lidar, yaw, seen, noise=noise, rng=rng)
        d = fsm.step((x, y), yaw, r, t, speed=u)
        if d.phase == 'HOLD':
            hold_side = min(hold_side, square_clearance((x, y), SQUARE_ORDERED))
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
        r += (2.0 * ang - r) * dt / min(tau, 1.4)    # 회전 τ 실측 1.4 s
        yaw += r * dt
        x += (u * math.cos(yaw) + current[0]) * dt
        y += (u * math.sin(yaw) + current[1]) * dt
        min_clear = min(min_clear, *(math.hypot(x - bx, y - by) for bx, by in bob))
        if done_at is not None and t > done_at + 10.0:
            break
    return phases, done_at, (x, y), min_clear, fsm, hold_side


@pytest.mark.parametrize('start, yaw, goal, seed, current', [
    ((3.0, 0.0), 0.0, (10.0, 0.0), 0, (0.04, 0.03)),             # 정면
    ((4.0, 3.5), -0.5, (10.4, -0.5), 1, (0.04, 0.03)),           # 비스듬히 + 기록 오차 0.6 m
    ((10.0, -7.0), math.pi / 2, (9.6, 0.3), 2, (0.10, -0.06)),   # 다른 변 + 센 옆 해류
    ((16.5, 2.0), math.pi, (10.0, 0.5), 4, (-0.08, 0.10)),       # 반대편 변 + 센 해류
])
def test_closed_loop_enters_holds_and_completes(start, yaw, goal, seed, current):
    phases, done_at, final, min_clear, fsm, _ = run_mission(start, yaw, goal, seed,
                                                            current=current)
    assert phases == ['SEARCH', 'ENTER', 'HOLD']
    assert done_at is not None and done_at < 60.0
    # 완료 후 10 s 더 유지해도 사각형 안 (해류에 밀려도 앵커가 붙잡는다)
    assert square_clearance(final, SQUARE_ORDERED) >= P.inside_margin
    assert min_clear > 0.6                         # 부표와 부딪히지 않음 (선체 반폭 + 여유)
    buoys = find_buoys(fsm.map)
    assert len(buoys) == 4                         # 빔 5 % 노이즈에도 가짜 부표 없음
    assert all(min(math.hypot(b.x - bx, b.y - by) for bx, by in SQUARE) < 0.4
               for b in buoys)
    assert fsm.center_kind == 'four'
    assert math.dist(fsm.center, CENTER) < 0.3


@pytest.mark.parametrize('start, yaw, goal, seed, current', [
    ((3.0, 0.0), 0.0, (10.0, 0.0), 0, (0.04, 0.03)),
    ((10.0, -7.0), math.pi / 2, (9.6, 0.3), 2, (0.10, -0.06)),
])
def test_slow_hull_stays_off_the_sides(start, yaw, goal, seed, current):
    # 실측 속도 응답(τ ≈ 2.8 s, rec_bag) — 명령을 끊어도 오래 미끄러진다. 제동이 없던 예전
    # 앵커는 중심을 지나 변까지 0.1~0.3 m 로 밀려 5 s 유지를 계속 놓쳤다.
    phases, done_at, final, min_clear, _, hold_side = run_mission(
        start, yaw, goal, seed, current=current, tau=2.8)
    assert phases == ['SEARCH', 'ENTER', 'HOLD']
    assert done_at is not None and done_at < 90.0
    assert hold_side > 0.35                        # HOLD 내내 변에서 떨어져 있음
    assert square_clearance(final, SQUARE_ORDERED) >= P.inside_margin
    assert min_clear > 0.6


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
