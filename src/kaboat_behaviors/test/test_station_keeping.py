"""station(위치유지) 순수 계산 — 전용 지도·부표·중심·입구·가상 앵커·폐루프."""

import math

import numpy as np
import pytest

from kaboat_behaviors.behavior_base import DEFAULT_MAX_LINEAR, WAYPOINT_MAX_ANGULAR
from kaboat_behaviors.los_controller import BSPLINE_TRACK_YAW_KD, BSPLINE_TRACK_YAW_KP
from kaboat_behaviors import station_keeping
from kaboat_behaviors.station_keeping import (
    Buoy, StationFsm, StationMap, StationParams, anchor_command, choose_entry_gate,
    detour_target, entry_target, estimate_center, exit_waypoints, find_buoys, innermost_point,
    select_corners, square_clearance, square_corners, track_gate)
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


def scan(lidar_xy, yaw, buoys, radius=BUOY_RADIUS, noise=0.0, rng=None, beams=BEAMS):
    """원 기둥 부표에 대한 2D 레이캐스트 → ranges (부표에 안 맞으면 SHORE).

    noise: 빔 중 이 비율을 0.8~6 m 무작위 거리 반사(해수면 노이즈)로 바꾼다.
    """
    ranges = np.full(beams.size, SHORE)
    ox, oy = lidar_xy
    for k, a in enumerate(beams):
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
        flip = rng.random(beams.size) < noise
        ranges[flip] = rng.uniform(0.8, 6.0, int(flip.sum()))
    return ranges


def add(target, lidar_xy, yaw, buoys, beams=BEAMS, **scan_kw):
    args = (scan(lidar_xy, yaw, buoys, beams=beams, **scan_kw), float(beams[0]),
            float(beams[1] - beams[0]), 0.05, RANGE_MAX, lidar_xy, yaw)
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


def test_extra_object_off_the_square_is_not_a_corner():
    # 사각형 동쪽 5 m 밖 물체도 후보(15 m 안)지만, 묶음은 기록 지점에 가까운 네 꼭짓점
    m = StationMap(CENTER, StationParams())
    far = (CENTER[0] + 5.0, CENTER[1])
    for k in range(36):
        add(m, CENTER, math.radians(10 * k), SQUARE + [far])
    found = find_buoys(m)
    assert len(found) == 4
    assert all(min(math.hypot(b.x - x, b.y - y) for x, y in SQUARE) < 0.25 for b in found)


def test_square_drifted_off_record_keeps_all_four_buoys():
    # 사각형이 기록 지점에서 (2.3, 1.0) m 밀림 → 먼 모서리 (11.5, 1.5) 는 기록 지점에서 4.55 m.
    # 예전 4 m 반경은 3개뿐이라 출구를 엉뚱한 변으로 잡았다 — 이제 처음부터 네 개.
    record = (CENTER[0] - 2.3, CENTER[1] - 1.0)
    fsm = StationFsm(record, StationParams())
    for k in range(36):
        add(fsm, CENTER, math.radians(10 * k), SQUARE)
    d = fsm.step(CENTER, 0.0, 0.0, 0.0)
    assert len(d.buoys) == 4 and d.center_kind == 'four'
    for bx, by in SQUARE:
        assert min(math.hypot(b.x - bx, b.y - by) for b in d.buoys) < 0.25


# ── 중심 추정 ─────────────────────────────────────────

P = StationParams()


def test_center_is_the_innermost_point_of_the_corners():
    c, kind = estimate_center(SQUARE, (3.0, 0.0), P)
    assert kind == 'four' and c == pytest.approx(CENTER, abs=0.03)
    # 3개: 삼각형의 가장 안쪽 점 — 사각형 안이고 세 변에서 같은 거리
    c, kind = estimate_center(SQUARE[:3], (3.0, 0.0), P)
    tri = square_corners(SQUARE[:3])
    assert kind == 'three' and square_clearance(c, tri) == pytest.approx(
        innermost_point(SQUARE[:3])[1], abs=0.02)
    assert square_clearance(c, SQUARE_ORDERED) > 0.8
    # 2개는 변인지 대각선인지 모른다 — 대각선이어도 중점 근처라 사각형 안
    c, kind = estimate_center([SQUARE[0], SQUARE[3]], (3.0, 0.0), P)
    assert kind == 'pair' and square_clearance(c, SQUARE_ORDERED) > 0.0


def test_center_from_first_side_is_beyond_it_and_hint_keeps_side():
    near = SQUARE[:2]                          # x=8.5 두 개 (배는 x=3 에서 접근)
    c, kind = estimate_center(near, (3.0, 0.0), P)
    assert kind == 'pair' and c == pytest.approx(CENTER)   # 간격 3 m → 안쪽 min(1.5, 1.5)
    # 배가 안으로 들어가 변을 지난 뒤에도 직전 중심 쪽을 유지 (배 반대쪽으로 뒤집히지 않음)
    c, _ = estimate_center(near, (10.0, 0.0), P, hint_xy=CENTER)
    assert c == pytest.approx(CENTER)
    assert estimate_center([SQUARE[0]], (3.0, 0.0), P) is None
    assert estimate_center([(8.5, 0.0), (8.5, 0.5)], (3.0, 0.0), P) is None


# ── 사각형 (완료 판정 영역) ────────────────────────────

def _same_corners(got, want):
    return sorted(map(tuple, np.round(got, 6))) == sorted(map(tuple, np.round(want, 6)))


def test_square_is_only_what_was_seen():
    # 예전엔 3개면 평행사변형, 2개면 공칭 3 m 로 채웠다 — 비정형 사각형에서 틀린 영역이 돼
    # 거짓 완료가 났다 (dock2 2차). 이제 보이는 꼭짓점 다각형만, 3개면 삼각형(보수적)
    assert _same_corners(square_corners(SQUARE), SQUARE)
    for missing in range(4):
        seen = [q for k, q in enumerate(SQUARE) if k != missing]
        assert _same_corners(square_corners(seen), seen)
    assert square_corners(SQUARE[:2]) is None and square_corners([SQUARE[0]]) is None


def test_square_clearance_is_inward_distance_to_nearest_side():
    sq = square_corners(SQUARE)
    assert square_clearance(CENTER, sq) == pytest.approx(1.5)
    assert square_clearance((11.0, 0.3), sq) == pytest.approx(0.5)
    assert square_clearance((12.0, 0.0), sq) == pytest.approx(-0.5)      # 밖
    tilted = [(10 + 1.5 * math.cos(a), 1.5 * math.sin(a))                # 45° 돌린 사각형
              for a in np.radians([45, 135, 225, 315])]
    assert square_clearance(CENTER, tilted) == pytest.approx(1.5 / math.sqrt(2))


@pytest.fixture
def exact_buoys(monkeypatch):
    """전용 지도 대신 부표 후보를 그대로 준다 (지도 추정의 수 cm 치우침 없이 판정만 본다)."""
    def use(points):
        monkeypatch.setattr(station_keeping, 'candidate_buoys',
                            lambda *_args, **_kw: [Buoy(x, y, 1.0) for x, y in points])
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


def test_hold_with_three_buoys_waits_then_judges_inside_the_triangle(exact_buoys):
    # 3개면 three_corner_wait 동안 네 번째를 찾고(완료 판정 안 함), 그 뒤에야 삼각형으로 판정
    exact_buoys(SQUARE[:3])                      # (11.5, 1.5) 안 보임
    fsm = _fsm_holding()
    d = _hold_until_complete(fsm, (9.4, -0.6))   # 삼각형 세 변에서 0.85 m 이상이지만 아직 기다림
    assert not d.inside and not fsm.completed and d.stage == 'LOOK'
    t0 = P.three_corner_wait
    for k in range(80):
        d = fsm.step((10.3, 0.3), 0.0, 0.0, t0 + k * 0.1)   # 사각형 안이지만 삼각형 밖 (보수적)
    assert not d.inside and not fsm.completed
    for k in range(80):
        d = fsm.step((9.4, -0.6), 0.0, 0.0, t0 + 10.0 + k * 0.1)
    assert d.inside and fsm.completed


def test_two_buoys_never_hold_or_complete(exact_buoys):
    # dock2 2차: 서·남 두 큐브로 지은 사각형에서 거짓 '위치유지 달성'. 이제 2개면 HOLD 안 감
    exact_buoys(SQUARE[:2])
    fsm = StationFsm(CENTER, StationParams())
    for k in range(80):
        d = fsm.step(CENTER, 0.0, 0.0, k * 0.1)
    assert d.phase == 'ENTER' and d.center_kind == 'pair' and d.square is None
    assert not fsm.completed


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
    assert stage == 'ALIGN_GATE' and target == pytest.approx((8.5 - P.pre_gate_distance, 0.0))
    # 이등분선 위면 바로 중심
    target, stage = entry_target((6.0, 0.2), gate, CENTER, P)
    assert stage == 'THROUGH' and target == pytest.approx(CENTER)
    # 2개면 그 쌍이 입구 — 대각선이어도 그 사이는 사각형 안이라 지나도 된다
    assert choose_entry_gate([SQUARE[0], SQUARE[3]], (3.0, 0.0), P) is not None
    # 배가 이미 사각형 안이면 입구 없음 (중심으로 바로)
    assert choose_entry_gate(SQUARE, CENTER, P) is None


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
        StationParams(inside_margin=2.6)            # 공칭 변(5 m)의 절반 이상이면 들어갈 곳이 없음
    with pytest.raises(ValueError):
        StationParams(max_side=4.0, side_length=5.0)
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
    assert d.phase == 'SEARCH'                      # search_timeout 전엔 부표를 더 찾는다
    d = fsm.step((9.5, 0.0), 0.3, 0.0, p.search_timeout)
    assert d.phase == 'HOLD' and d.center_kind == 'goal'
    t0 = p.search_timeout
    for k in range(1, 52):
        d = fsm.step((9.5, 0.0), 0.3, 0.0, t0 + k * 0.1)
        if d.completed_now:
            break
    assert d.completed_now and fsm.completed
    # 완료는 한 번만
    assert not fsm.step((9.5, 0.0), 0.3, 0.0, t0 + 6.0).completed_now


# ── 부표 검출 노드 결과 (/detections/buoys) ─────────────────

def test_detected_buoys_fill_in_when_the_map_has_none_yet():
    # 조류에 밀린 사각형: 전용 지도(6 m 안)에는 아직 없지만 검출 노드가 15 m 밖에서 잡음
    fsm = StationFsm(CENTER, StationParams())
    far_boat = (CENTER[0] - 12.0, CENTER[1])
    d = fsm.step(far_boat, 0.0, 0.0, 0.0)
    assert d.buoys == [] and fsm.phase == 'SEARCH'
    near_side = [(8.5, -1.5, 0.9), (8.5, 1.5, 0.8)]           # 배 쪽 한 변
    d = fsm.step(far_boat, 0.0, 0.0, 0.1, detected=near_side)
    assert len(d.buoys) == 2 and all(b.source == 'detector' for b in d.buoys)
    assert fsm.phase == 'ENTER' and d.gate is not None       # 두 개로 입구 고정 → 진입


def test_map_buoys_come_first_and_duplicates_merge():
    m = StationMap(CENTER, StationParams())
    for k in range(36):
        add(m, CENTER, math.radians(10 * k), SQUARE)
    # 검출 노드가 같은 네 부표를 0.3 m 어긋나게 + 반경 밖 하나 더
    detected = [(x + 0.3, y, 0.99) for x, y in SQUARE] + [(CENTER[0] + 6.0, CENTER[1], 1.0)]
    found = find_buoys(m, detected=detected)
    assert len(found) == 4 and all(b.source == 'map' for b in found)


def test_detected_buoys_complete_a_partial_square():
    m = StationMap(CENTER, StationParams())
    for k in range(18):                                       # 앞쪽 두 부표만 지도에
        add(m, (CENTER[0] - 4.0, CENTER[1]), math.radians(-10 + 20 * (k % 2)), SQUARE[:2])
    detected = [(x, y, 0.9) for x, y in SQUARE[2:]]
    found = find_buoys(m, detected=detected)
    assert len(found) == 4
    assert sorted(b.source for b in found) == ['detector', 'detector', 'map', 'map']


def test_detected_buoys_can_be_turned_off():
    fsm = StationFsm(CENTER, StationParams(use_detected_buoys=False))
    d = fsm.step((CENTER[0] - 12.0, CENTER[1]), 0.0, 0.0, 0.0,
                 detected=[(8.5, -1.5, 0.9), (8.5, 1.5, 0.9)])
    assert d.buoys == [] and fsm.phase == 'SEARCH'


# ── 2026-11-13 dock2 bag 연습장: 비정형 사각형·기록 지점 밖·밀린 큐브·도크 구조물 ─────────

# 큐브 네 개 (bag 의 검출 노드 추적 평균, 서·북·동·남) — 변 3.9~6.3 m, 내각 60~152°
DOCK2 = [(152.6, -29.4), (157.05, -24.95), (161.0, -29.7), (157.2, -30.6)]
DOCK2_CENTER_REC = (155.16, -31.21)      # 기록된 station_center — 사각형 밖 1.1 m
DOCK2_END = (159.83, -33.02)             # station_end (출구)
DOCK2_OTHERS = [(160.7, -37.4), (164.7, -38.0), (159.8, -40.2)]   # 남동쪽 도크 쪽 물체 (검출 노드)
PUSHED_WEST = (153.55, -30.18)           # 배에 밀린 서쪽 큐브가 새로 잡힌 자리 (옛 자리와 1.2 m)


def _buoys(points, strength=1.0):
    return [Buoy(x, y, strength) for x, y in points]


def test_dock2_corners_are_the_four_cubes_even_with_clutter_and_a_wrong_record():
    cands = _buoys(DOCK2 + [PUSHED_WEST] + DOCK2_OTHERS)
    # 기록 지점이 2 m 씩 (2.8 m) 더 틀려도 (무게중심에서 5.9 m — 부표가 각각 2 m 밀려도 무게중심은
    # 기록에서 5.1 m 안) 같은 묶음. 그보다 많이 틀리면 옛·새 자리를 둘 다 넣은 묶음과 겨루게 된다
    # (far_free 5 m 넘으면 감점) — 기록은 그날 사각형 가운데에서
    for anchor in (DOCK2_CENTER_REC, (DOCK2_CENTER_REC[0] - 2.0, DOCK2_CENTER_REC[1] - 2.0)):
        got = select_corners(cands, anchor, P)
        assert len(got) == 4
        # 큐브마다 꼭짓점 하나 (서쪽은 옛 자리든 밀린 새 자리든)
        for cube in DOCK2:
            assert sum(1 for b in got if math.dist((b.x, b.y), cube) < 1.3) == 1
        assert not any((b.x, b.y) in DOCK2_OTHERS for b in got)
    # 옛·새 자리 둘이 같은 꼭짓점으로 두 번 뽑히지 않는다 (간격 1.2 m < min_side)
    got = select_corners(cands, DOCK2_CENTER_REC, P)
    assert not ({(b.x, b.y) for b in got} >= {DOCK2[0], PUSHED_WEST})


def test_dock2_center_is_inside_with_margin_and_record_is_outside():
    poly = square_corners(DOCK2)
    c, kind = estimate_center(DOCK2, (145.0, -27.0), P)
    assert kind == 'four' and square_clearance(c, poly) > 2.1        # 가장 안쪽 ≈ 2.2 m
    assert square_clearance(np.mean(DOCK2, axis=0), poly) < 1.9      # 평균은 1.8 m
    assert square_clearance(DOCK2_CENTER_REC, poly) < -1.0           # 기록 지점은 밖


def test_dock2_second_attempt_no_false_completion(exact_buoys):
    # 15:15:52~57: 서·남 두 큐브만으로 '위치유지 달성'. 그 자리는 실제 사각형 변에서 0.16 m
    exact_buoys([DOCK2[0], DOCK2[3]])
    fsm = StationFsm(DOCK2_CENTER_REC, StationParams())
    for k in range(60):
        d = fsm.step((154.5, -29.73), -1.8, 0.0, k * 0.1)
    assert d.phase != 'HOLD' and not fsm.completed and d.square is None
    exact_buoys(DOCK2)                                 # 북·동도 보이면 실제 사각형으로
    for k in range(60, 120):
        d = fsm.step((154.5, -29.73), -1.8, 0.0, k * 0.1)
    assert len(d.buoys) == 4 and d.center_kind == 'four'
    assert square_clearance((154.5, -29.73), d.square) < P.inside_margin   # 그 자리는 완료 아님
    assert math.dist(d.center, np.mean(DOCK2, axis=0)) < 0.6


def test_detour_goes_around_a_buoy_on_the_way_and_stops_short_of_a_buoy_target():
    boat, target = (145.5, -27.65), DOCK2_CENTER_REC
    west = DOCK2[0]                                    # dock2 1차: 기록 지점 쪽 길 위의 서쪽 큐브
    via = detour_target(boat, target, [west], P)
    assert via != pytest.approx(target)
    assert math.dist(via, west) >= P.avoid_clearance

    def seg_dist(p, a, b):
        p, a, b = (np.asarray(v, float) for v in (p, a, b))
        t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
        return float(np.linalg.norm(p - (a + t * (b - a))))
    assert seg_dist(west, boat, via) >= P.avoid_clearance * 0.99
    # 길에서 먼 부표·배 뒤 부표는 무시
    assert detour_target(boat, target, [(150.0, -20.0), (140.0, -26.0)], P) == pytest.approx(target)
    # 목표가 부표에 붙어 있으면 배 쪽 target_clearance 앞에서 멈춘다 (stop 을 주면 그 거리)
    stop = detour_target(boat, (152.8, -29.4), [west], P)
    assert math.dist(stop, west) == pytest.approx(P.target_clearance, abs=1e-6)
    stop = detour_target(boat, (152.8, -29.4), [west], P, stop=P.standoff)
    assert math.dist(stop, west) == pytest.approx(P.standoff, abs=1e-6)


def test_map_recenter_keeps_what_was_seen():
    m = StationMap(CENTER, StationParams())
    for k in range(36):
        add(m, CENTER, math.radians(10 * k), SQUARE)
    before = sorted((round(b.x, 1), round(b.y, 1)) for b in find_buoys(m))
    assert m.recenter((CENTER[0] + 3.04, CENTER[1] - 1.97))
    assert m.center == pytest.approx((CENTER[0] + 3.0, CENTER[1] - 2.0))
    after = sorted((round(b.x, 1), round(b.y, 1)) for b in find_buoys(m, near=CENTER))
    assert after == before
    assert not m.recenter(m.center)


def test_one_buoy_seen_at_the_record_heads_toward_it_slowly(exact_buoys):
    # 사각형이 통째로 밀려 기록 지점에 없고 큐브 하나만 보이면 그 방위로 standoff 앞까지
    exact_buoys([(CENTER[0] + 7.0, CENTER[1] + 2.0)])
    fsm = StationFsm(CENTER, StationParams())
    d = fsm.step(CENTER, 0.0, 0.0, 0.0)
    assert d.phase == 'SEARCH' and d.stage == 'TO_BUOY' and d.kind == 'waypoint'
    assert math.dist(d.target, (CENTER[0] + 7.0, CENTER[1] + 2.0)) == pytest.approx(P.standoff)
    far = fsm.step((CENTER[0] - 10.0, CENTER[1]), 0.0, 0.0, 0.1)   # 아직 멀면 기록 지점으로
    assert far.stage == 'TO_GOAL' and far.target == pytest.approx(CENTER)


def test_dock2_exit_goes_through_a_side_away_from_the_cubes():
    c, _ = estimate_center(DOCK2, (145.0, -27.0), P)
    route = exit_waypoints(_buoys(DOCK2), c, DOCK2_END, P)
    assert len(route) == 3 and route[-1] == DOCK2_END     # 변 안쪽 점 → 변 밖 경유점 → 출구

    def seg_dist(p, a, b):
        p, a, b = (np.asarray(v, float) for v in (p, a, b))
        t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
        return float(np.linalg.norm(p - (a + t * (b - a))))
    # 중심 → 변 안쪽 점 → 변 밖 경유점: 큐브에서 1.5 m 넘게 (dock2 2차는 남쪽 큐브 0.42 m)
    legs = [(c, route[0]), (route[0], route[1])]
    assert min(seg_dist(q, a, b) for q in DOCK2 for a, b in legs) > 1.5
    assert square_clearance(route[0], square_corners(DOCK2)) > 1.0      # 안쪽 점은 사각형 안


CUBE_RADIUS = 0.3        # 파란 큐브 한 변 약 0.5~0.6 m
# 실물 라이다: 선미 120° 가림(onboard_scan_filter 120~240°) → 앞 ±120° (위 FOV ±60° 보다 넓다)
WIDE_BEAMS = np.linspace(-math.radians(120.0), math.radians(120.0), 481)


def run_dock2(start_xy, start_yaw, record_xy, shift=(0.0, 0.0), seed=0, seconds=240.0,
              current=(0.03, -0.02), tau=2.8, detector=True):
    """dock2 큐브 배치(+shift 만큼 통째로 밀림)에서 폐루프. 검출 노드는 15 m 안 부표·도크 물체를
    85 % 확률로 낸다. 반환: (단계 순서, 완료 시각, 끝 위치, 큐브까지 최소 중심거리, fsm, 사각형)."""
    rng = np.random.default_rng(seed)
    p = StationParams()
    cubes = [(x + shift[0], y + shift[1]) for x, y in DOCK2]
    fsm = StationFsm(record_xy, p)
    wp = WaypointController(WAYPOINT_PARAMS)
    x, y, yaw = start_xy[0], start_xy[1], start_yaw
    u = r = 0.0
    dt = 0.1
    min_clear, phases, done_at = math.inf, [], None
    for k in range(int(seconds / dt)):
        t = k * dt
        bob = [(bx + 0.2 * math.sin(0.6 * t + i), by + 0.2 * math.cos(0.4 * t + 2 * i))
               for i, (bx, by) in enumerate(cubes)]
        seen = [b for b in bob if rng.random() > 0.2]
        lidar = (x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
        add(fsm, lidar, yaw, seen + DOCK2_OTHERS, beams=WIDE_BEAMS, radius=CUBE_RADIUS,
            noise=0.03, rng=rng)
        detected = None
        if detector:
            detected = [(bx + rng.normal(0, 0.1), by + rng.normal(0, 0.1), 0.9)
                        for bx, by in bob + DOCK2_OTHERS
                        if math.hypot(bx - x, by - y) < 15.0 and rng.random() < 0.85]
        d = fsm.step((x, y), yaw, r, t, speed=u, detected=detected)
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
        u += (3.0 * lin - u) * dt / tau
        r += (2.0 * ang - r) * dt / min(tau, 1.4)
        yaw += r * dt
        x += (u * math.cos(yaw) + current[0]) * dt
        y += (u * math.sin(yaw) + current[1]) * dt
        min_clear = min(min_clear, *(math.hypot(x - bx, y - by) for bx, by in bob))
        if done_at is not None and t > done_at + 10.0:
            break
    return phases, done_at, (x, y), min_clear, fsm, square_corners(cubes)


@pytest.mark.parametrize('start, yaw, shift, seed', [
    ((145.5, -27.65), math.radians(-38), (0.0, 0.0), 0),     # dock2 1차: 서쪽 큐브가 기록 지점 쪽 길 위
    ((155.15, -24.48), math.radians(-76), (0.0, 0.0), 1),    # dock2 2차: 북쪽에서 내려옴
    ((145.5, -27.65), math.radians(-38), (3.0, -2.0), 2),    # 사각형이 통째로 3.6 m 밀림
    ((145.5, -27.65), math.radians(-38), (-4.0, 4.0), 3),    # 반대로 5.7 m — 기록 지점이 사각형 밖 멀리
])
def test_dock2_closed_loop_finds_the_square_holds_and_never_touches_a_cube(start, yaw, shift, seed):
    phases, done_at, final, min_clear, fsm, poly = run_dock2(start, yaw, DOCK2_CENTER_REC,
                                                             shift=shift, seed=seed)
    # 검출 노드가 처음부터 꼭짓점 2개 이상을 보면 첫 틱에 바로 ENTER (SEARCH 는 안 남을 수 있다)
    assert phases[-1] == 'HOLD' and 'ENTER' in phases and done_at is not None
    assert square_clearance(final, poly) >= P.inside_margin      # 완료 뒤 10 s 도 안
    assert min_clear > CUBE_RADIUS + 0.6                          # 선체 반폭 + 여유로 큐브와 안 닿음
    assert fsm.center_kind == 'four'
    assert all(min(math.hypot(b.x - x, b.y - y) for x, y in poly) < 0.6 for b in fsm.corners)


@pytest.mark.parametrize('start, yaw, seed', [
    ((145.5, -27.65), math.radians(-38), 0),
    ((155.15, -24.48), math.radians(-76), 1),   # 북쪽 큐브가 처음부터 옆 — 삼각형(서·남·동)이 얇다
])
def test_dock2_map_only_still_completes_without_touching(start, yaw, seed):
    # 부표 검출 노드가 없을 때(전용 지도만). 얇은 삼각형이면 대각선 건너편을 목표로, 선수를 안 보이는
    # 꼭짓점 쪽으로 돌려 찾는다 (예전 판단이면 남쪽 큐브 0.4 m 옆에서 버텼다)
    phases, done_at, final, min_clear, fsm, poly = run_dock2(start, yaw, DOCK2_CENTER_REC,
                                                             seed=seed, detector=False)
    assert done_at is not None and fsm.center_kind == 'four'
    assert square_clearance(final, poly) >= P.inside_margin
    assert min_clear > CUBE_RADIUS + 0.6


def test_gate_buoys_count_as_obstacles_unless_the_path_goes_between_them():
    from kaboat_behaviors.station_keeping import EntryGate, passes_between
    gate = EntryGate((0.0, -2.0), (0.0, 2.0), (1.0, 0.0))
    assert passes_between((-3.0, 0.0), (3.0, 0.5), gate, P)          # 가운데로
    assert not passes_between((-3.0, 2.3), (3.0, 2.3), gate, P)      # 부표 바깥 연장선 위
    assert not passes_between((-3.0, 1.6), (3.0, 1.8), gate, P)      # 부표 0.3 m 옆
