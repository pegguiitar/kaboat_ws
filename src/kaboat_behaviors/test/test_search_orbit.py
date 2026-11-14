"""search_orbit 단독 테스트 — ROS 없이 돈다 (부표는 color·position 을 가진 수제 객체)."""
import math
from types import SimpleNamespace

import pytest

from kaboat_behaviors.search_orbit import SearchFsm, SearchParams, clear_point, tangent_entry

GOAL = (50.0, 70.0)


def buoy(color, x, y, bid=None):
    return SimpleNamespace(color=color, position=SimpleNamespace(x=x, y=y), id=bid)


def fsm(color='red', area=None, **kw):
    kw.setdefault('lock_confirm_sec', 0.0)   # 기존 시나리오는 바로 고정
    kw.setdefault('look_sec', 0.0)           # 기존 시나리오는 시작 둘러보기 없이
    kw.setdefault('black_needs_field', False)  # 기존 검정 시나리오는 빨강·초록 확인 없이
    return SearchFsm(SearchParams(target_color=color, **kw), GOAL, area)


def orbit(f, center, radius, direction, degrees, buoys=(), start_deg=0.0, now=0.0):
    """center 둘레 radius 원 위를 direction 으로 degrees 만큼 1° 씩 돈다."""
    sign = -1.0 if direction == 'cw' else 1.0
    for k in range(int(degrees) + 1):
        a = math.radians(start_deg + sign * k)
        f.update(list(buoys), (center[0] + radius * math.cos(a),
                               center[1] + radius * math.sin(a)), now)
    return f.phase


# ---------- 규정 방향 ----------

@pytest.mark.parametrize('color, direction', [('red', 'cw'), ('green', 'cw'), ('black', 'ccw')])
def test_turn_direction_follows_rule(color, direction):
    assert SearchParams(target_color=color).direction == direction


def test_unset_color_has_no_direction_and_fsm_refuses():
    assert SearchParams().direction is None
    with pytest.raises(ValueError):
        SearchFsm(SearchParams(), GOAL)


def test_unknown_color_rejected():
    with pytest.raises(ValueError):
        SearchParams(target_color='yellow')


# ---------- 후보 선택·고정 ----------

def test_seek_until_target_colour_seen():
    f = fsm('green')
    assert f.update([buoy('red', 52, 70)], (40, 70), 0.0) == 'SEEK'
    assert f.update([buoy('green', 53, 71)], (40, 70), 0.1) == 'ORBIT'
    assert f.center == (53.0, 71.0)


def test_candidate_outside_search_radius_ignored():
    f = fsm('red', target_search_radius=10.0)
    assert f.update([buoy('red', 65, 70)], (40, 70), 0.0) == 'SEEK'


def test_picks_candidate_nearest_goal():
    f = fsm('red')
    f.update([buoy('red', 57, 70), buoy('red', 51, 70)], (40, 70), 0.0)
    assert f.center == (51.0, 70.0)


def test_switches_to_much_closer_candidate_before_counting():
    # 멀리서 주황(빨강으로 나옴)을 먼저 골랐다가 목표점 옆 진짜 빨강이 보이면 바꾼다
    f = fsm('red')
    f.update([buoy('red', 58, 70)], (30, 70), 0.0)
    f.update([buoy('red', 58, 70), buoy('red', 50.5, 70)], (35, 70), 0.1)
    assert f.center == (50.5, 70.0) and f.switches == 1


def test_no_switch_after_counting_started():
    f = fsm('red')
    f.update([buoy('red', 58, 70)], (30, 70), 0.0)
    orbit(f, (58, 70), 2.0, 'cw', 30)
    assert f.counting
    f.update([buoy('red', 58, 70), buoy('red', 50.5, 70)], (60, 70), 0.1)
    assert f.center == pytest.approx((58.0, 70.0)) and f.switches == 0


def test_lock_survives_buoy_leaving_map():
    f = fsm('red')
    f.update([buoy('red', 51, 70)], (40, 70), 0.0)
    f.update([], (45, 70), 0.1)
    assert f.phase == 'ORBIT' and f.center == (51.0, 70.0)


def test_refine_moves_center_towards_reobserved_buoy():
    # 같은 검출 ID 의 부표가 다시 보이면 그쪽으로 보정 (ID 를 따라가므로 1 m 움직여도)
    f = fsm('red', refine_alpha=0.5)
    f.update([buoy('red', 51, 70, 3)], (40, 70), 0.0)
    f.update([buoy('red', 52, 70, 3)], (40, 70), 0.1)
    assert f.center == pytest.approx((51.5, 70.0))


def test_refine_without_id_only_within_relock_radius():
    f = fsm('red', refine_alpha=0.5, relock_radius=0.6)
    f.update([buoy('red', 51, 70)], (40, 70), 0.0)
    f.update([buoy('red', 51.4, 70)], (40, 70), 0.1)      # 0.4 m — 이어받음
    assert f.center == pytest.approx((51.2, 70.0))
    f.update([buoy('red', 52.4, 70)], (40, 70), 0.2)      # 1.2 m — 옆 부표, 안 끌려감
    assert f.center == pytest.approx((51.2, 70.0))


def test_refine_ignores_other_colour_and_far_buoys():
    f = fsm('red', relock_radius=1.5)                  # 위치로 이어받는 반경만 시험
    f.update([buoy('red', 51, 70)], (40, 70), 0.0)
    f.update([buoy('green', 51.5, 70), buoy('red', 54, 70)], (40, 70), 0.1)
    assert f.center == (51.0, 70.0)


# ---------- 선회 각도 ----------

@pytest.mark.parametrize('color, direction', [('red', 'cw'), ('black', 'ccw')])
def test_done_after_laps_in_rule_direction(color, direction):
    f = fsm(color, laps=1.0)
    f.update([buoy(color, 51, 70)], (40, 70), 0.0)
    assert orbit(f, (51, 70), 2.0, direction, 350) == 'ORBIT'
    assert orbit(f, (51, 70), 2.0, direction, 15, start_deg=(-350 if direction == 'cw'
                                                                else 350)) == 'DONE'


def test_wrong_direction_never_completes():
    f = fsm('red', laps=1.0)                     # 빨강 = 시계
    f.update([buoy('red', 51, 70)], (40, 70), 0.0)
    assert orbit(f, (51, 70), 2.0, 'ccw', 720) == 'ORBIT'
    assert f.swept < 0


def test_angle_outside_band_not_counted():
    f = fsm('red', laps=1.0, orbit_band=1.0)
    f.update([buoy('red', 51, 70)], (40, 70), 0.0)
    orbit(f, (51, 70), 5.0, 'cw', 360)           # 반경 5 m — 띠(2±1 m) 밖
    assert f.swept == 0.0 and not f.counting


# ---------- 못 찾을 때 ----------

def test_give_up_after_timeout_at_goal():
    f = fsm('red', give_up_sec=90.0)
    assert f.update([], (30, 70), 0.0) == 'SEEK'          # 아직 접근 중 — 시계 안 돎
    assert f.update([], (30, 70), 500.0) == 'SEEK'
    f.update([], GOAL, 500.0)                              # 도착
    assert f.update([], GOAL, 589.0) == 'SEEK'
    assert f.update([], GOAL, 591.0) == 'GIVE_UP'
    assert f.done


def test_done_is_sticky():
    f = fsm('red', laps=0.1)
    f.update([buoy('red', 51, 70)], (40, 70), 0.0)
    orbit(f, (51, 70), 2.0, 'cw', 60)
    assert f.phase == 'DONE'
    assert f.update([buoy('red', 51, 70)], (40, 70), 1.0) == 'DONE'


# ---------- 완료 뒤 갈 점 ----------

def test_exit_point_is_goal_when_goal_clear_of_buoy():
    f = fsm('red')
    f.update([buoy('red', 53, 70)], (40, 70), 0.0)
    assert f.exit_point((55, 70)) == GOAL


def test_exit_point_steps_off_buoy_recorded_as_goal():
    # search_end 를 부표 중심에 찍은 경우 (sim mission_params 가 그렇다) — 부표에서 배 쪽으로 1.5 m
    f = fsm('red', done_clearance=1.5)
    f.update([buoy('red', 50, 70)], (40, 70), 0.0)
    x, y = f.exit_point((50, 72))
    assert (x, y) == pytest.approx((50.0, 71.5))
    assert math.dist((x, y), GOAL) < 2.0          # mission_manager 전환 반경 안


@pytest.mark.parametrize('boat', [(52.0, 70.0), (48.0, 70.0), (50.0, 72.0), (50.0, 68.0)])
def test_exit_point_near_buoy_steps_toward_exit_wherever_boat_ends(boat):
    # search_end 가 부표 1.0 m 옆 — 배가 어느 쪽에서 끝나도 출구점은 출구 쪽, search_end 0.5 m.
    # 예전(배 쪽)은 배가 반대편이면 출구점–search_end 2.5 m 라 mission_manager 2 m 전환이 안 됐다
    f = fsm('red', done_clearance=1.5)
    f.update([buoy('red', 50.0, 69.0)], (40, 70), 0.0)               # GOAL (50, 70) 은 부표 북쪽 1 m
    e = f.exit_point(boat)
    assert e == pytest.approx((50.0, 70.5))
    assert math.dist(e, GOAL) + f.p.exit_arrive_radius < 2.0


def test_exit_clear_only_when_exit_line_leaves_buoy():
    from kaboat_behaviors.search_orbit import exit_clear
    c, exit_xy = (0.0, 0.0), (-20.0, 0.0)                            # 출구는 서쪽 멀리
    assert not exit_clear((2.0, 0.0), c, exit_xy, 75.0, 3.0)         # 동쪽 — 직선이 부표를 지남
    assert not exit_clear((0.0, 2.0), c, exit_xy, 75.0, 3.0)         # 북쪽 — 90°
    assert exit_clear((-1.0, 1.73), c, exit_xy, 75.0, 3.0)          # 북서 (60°)
    assert exit_clear((-2.0, 0.0), c, exit_xy, 75.0, 3.0)            # 서쪽
    assert exit_clear((5.0, 0.0), c, exit_xy, 75.0, 3.0)             # 이미 원 띠 밖


# ---------- 검정 = unknown (소거법) ----------

def test_black_target_picks_unknown_buoy():
    # 부표 검출은 검정을 색으로 안 낸다 — 빨강·초록 아닌 물체는 unknown
    f = fsm('black')
    f.update([buoy('red', 52, 70), buoy('green', 51, 71), buoy('unknown', 53, 69)], (40, 70), 0.0)
    assert f.phase == 'ORBIT' and f.center == (53.0, 69.0) and f.direction == 'ccw'


def test_red_target_ignores_unknown_buoy():
    f = fsm('red')
    assert f.update([buoy('unknown', 50.5, 70)], (40, 70), 0.0) == 'SEEK'


def test_black_target_refines_with_unknown():
    f = fsm('black', refine_alpha=0.5)
    f.update([buoy('unknown', 51, 70, 4)], (40, 70), 0.0)
    f.update([buoy('unknown', 52, 70, 4)], (40, 70), 0.1)
    assert f.center == pytest.approx((51.5, 70.0))


# ---------- 검출 노드 ID·색 안정성 ----------

def test_lock_follows_detector_id_not_neighbour():
    # 검정(unknown) 목표 id 5 를 고정 — 1.2 m 옆 초록 공(id 6)이 잠깐 unknown 이 돼도 끌려가지 않는다
    f = fsm('black', refine_alpha=0.5)
    f.update([buoy('unknown', 51.0, 70.0, 5), buoy('green', 52.2, 70.0, 6)], (40, 70), 0.0)
    for k in range(10):
        f.update([buoy('unknown', 51.0, 70.0, 5), buoy('unknown', 51.5, 70.0, 6)], (40, 70), 0.1 * k)
    assert f.center == pytest.approx((51.0, 70.0)) and f.locked_id == 5


def test_lock_released_when_target_turns_out_wrong_colour():
    # 검정 목표로 unknown(id 5) 을 잡았는데 카메라 근거가 쌓여 초록으로 확정 → 고정 풀고 다시 찾음
    f = fsm('black')
    f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.0)
    assert f.phase == 'ORBIT'
    f.update([buoy('green', 51.0, 70.0, 5), buoy('unknown', 53.0, 70.0, 7)], (40, 70), 0.1)
    assert f.unlocks == 1 and f.locked_id == 7 and f.center == (53.0, 70.0)


def test_colour_must_hold_before_locking():
    f = SearchFsm(SearchParams(target_color='black', lock_confirm_sec=1.0, look_sec=0.0,
                                 black_needs_field=False), GOAL)
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.0) == 'SEEK'
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.5) == 'SEEK'
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 1.1) == 'ORBIT'


def test_colour_flicker_restarts_confirm_clock():
    f = SearchFsm(SearchParams(target_color='black', lock_confirm_sec=1.0, look_sec=0.0,
                                 black_needs_field=False), GOAL)
    f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.0)
    f.update([buoy('green', 51.0, 70.0, 5)], (40, 70), 0.6)    # 색이 바뀜 — 시계 다시
    f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.8)
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 1.5) == 'SEEK'
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 1.9) == 'ORBIT'


# ---------- 탐색 영역 중심(area) ↔ 출구(goal) 분리 (2026-11-13) ----------
# 그날 기록: search_entry (168.7, -8.7, 71.9°), search_end (147.1, -2.3) — 탐색 부표는 시작점 앞.
END_0113 = (147.10, -2.31)
AREA_0113 = (170.87, -2.52)          # 부표 셋 가운데 (= search_entry 선수 7 m 앞과 0.4 m 차이)
BUOYS_0113 = [buoy('unknown', 169.3, -5.9, 3), buoy('green', 166.5, -0.2, 4),
              buoy('red', 174.5, -1.2, 5),
              buoy('unknown', 150.9, -3.8, 9)]   # 끝점 옆 다른 물체


def fsm_0113(color, **kw):
    kw.setdefault('lock_confirm_sec', 0.0)
    kw.setdefault('look_sec', 0.0)
    return SearchFsm(SearchParams(target_color=color, **kw), END_0113, AREA_0113)


def test_area_defaults_to_goal():
    f = fsm('red')
    assert f.area == GOAL


@pytest.mark.parametrize('color, expect', [('red', (174.5, -1.2)), ('green', (166.5, -0.2)),
                                           ('black', (169.3, -5.9))])
def test_candidates_come_from_area_not_exit(color, expect):
    # 예전(끝점 10 m 기준)엔 셋 다 후보 0개 — 끝점에서 20–28 m
    f = fsm_0113(color)
    assert f.update(BUOYS_0113, (168.7, -8.9), 0.0) == 'ORBIT'
    assert f.center == expect


def test_old_exit_anchor_found_nothing_in_0113_layout():
    f = SearchFsm(SearchParams(target_color='red', lock_confirm_sec=0.0, look_sec=0.0,
                               target_search_radius=10.0), END_0113)
    assert f.update(BUOYS_0113, (168.7, -8.9), 0.0) == 'SEEK'


def test_black_ignores_unknown_object_near_exit():
    # 끝점 옆 unknown(150.9, -3.8) 은 영역 중심에서 20 m — 검정 후보 아님
    f = fsm_0113('black')
    f.update([b for b in BUOYS_0113 if b.id == 9], (150.0, -3.0), 0.0)
    assert f.phase == 'SEEK'


def test_seek_goes_to_area_and_gives_up_there():
    f = fsm_0113('red', give_up_sec=90.0)
    assert f.update([], END_0113, 0.0) == 'SEEK'          # 출구에 있어도 영역 중심 도착 아님
    assert f.update([], END_0113, 500.0) == 'SEEK'
    f.update([], AREA_0113, 500.0)
    assert f.update([], AREA_0113, 591.0) == 'GIVE_UP'


def test_exit_point_is_still_search_end():
    f = fsm_0113('red')
    f.update(BUOYS_0113, (168.7, -8.9), 0.0)
    assert f.exit_point((172.0, -1.0)) == END_0113


# ---------- LOOK (시작 자세에서 둘러보기) ----------

def test_look_then_seek_when_nothing_seen():
    f = fsm_0113('red', look_sec=15.0)
    assert f.update([], (168.7, -8.9), 100.0) == 'LOOK'
    assert f.update([], (168.7, -8.9), 114.9) == 'LOOK'
    assert f.look_remaining(110.0) == pytest.approx(5.0)
    assert f.update([], (168.7, -8.9), 115.1) == 'SEEK'


def test_red_locks_during_look_as_soon_as_confirmed():
    f = fsm_0113('red', look_sec=15.0, lock_confirm_sec=1.0)
    red = [buoy('red', 174.5, -1.2, 5)]
    assert f.update(red, (168.7, -8.9), 100.0) == 'LOOK'
    assert f.update(red, (168.7, -8.9), 101.1) == 'ORBIT'


def test_black_waits_until_look_ends():
    # 빨강·초록도 색이 붙기 전 잠깐 unknown — LOOK 동안은 검정 후보로 안 고른다
    f = fsm_0113('black', look_sec=15.0, lock_confirm_sec=1.0)
    early = [buoy('unknown', 174.5, -1.2, 5), buoy('unknown', 169.3, -5.9, 3)]
    assert f.update(early, (168.7, -8.9), 100.0) == 'LOOK'
    settled = [buoy('red', 174.5, -1.2, 5), buoy('unknown', 169.3, -5.9, 3)]
    assert f.update(settled, (168.7, -8.9), 110.0) == 'LOOK'
    assert f.update(settled, (168.7, -8.9), 115.1) == 'ORBIT'
    assert f.center == (169.3, -5.9)


def test_black_can_lock_during_look_when_disabled():
    f = fsm_0113('black', look_sec=15.0, black_waits_look=False)
    seen = [buoy('unknown', 169.3, -5.9, 3), buoy('green', 166.5, -0.2, 4)]
    assert f.update(seen, (168.7, -8.9), 0.0) == 'ORBIT'


# ---------- 접선 진입 ----------

def _on_circle(p, c, r):
    return math.dist(p, c) == pytest.approx(r, abs=1e-9)


@pytest.mark.parametrize('direction', ['cw', 'ccw'])
def test_tangent_point_is_tangent_and_matches_direction(direction):
    c, r, boat = (10.0, 0.0), 2.0, (0.0, 0.0)
    pts = tangent_entry(boat, c, r, direction)
    assert len(pts) == 3 and pts[0] == boat
    t, lead = pts[1], pts[2]
    assert _on_circle(t, c, r) and _on_circle(lead, c, r)
    # 배→접점 직선이 접점 반지름과 수직 (접선)
    dot = (t[0] - boat[0]) * (t[0] - c[0]) + (t[1] - boat[1]) * (t[1] - c[1])
    assert dot == pytest.approx(0, abs=1e-9)
    # 접점 → lead 가 규정 방향 (ccw = 각도 증가)
    a_t = math.atan2(t[1] - c[1], t[0] - c[0])
    a_l = math.atan2(lead[1] - c[1], lead[0] - c[0])
    d = math.atan2(math.sin(a_l - a_t), math.cos(a_l - a_t))
    assert (d > 0) == (direction == 'ccw')


def test_cw_from_west_enters_on_north_side():
    # 서쪽에서 동쪽 원으로 — 시계 방향이면 원 북쪽(위)으로 들어가 동쪽으로 내려간다
    pts = tangent_entry((0.0, 0.0), (10.0, 0.0), 2.0, 'cw')
    assert pts[1][1] > 0


def test_no_entry_when_already_on_circle_band():
    assert tangent_entry((12.5, 0.0), (10.0, 0.0), 2.0, 'cw', skip_band=0.7) is None
    assert tangent_entry((10.5, 0.0), (10.0, 0.0), 2.0, 'cw') is None    # 원 안


def test_entry_detours_around_buoy_on_the_line():
    c, r, boat = (10.0, 0.0), 2.0, (0.0, 0.0)
    straight = tangent_entry(boat, c, r, 'cw')
    t = straight[1]
    mid = ((boat[0] + t[0]) / 2, (boat[1] + t[1]) / 2)      # 진입 직선 한가운데 부표
    pts = tangent_entry(boat, c, r, 'cw', [mid], clearance=1.5)
    assert len(pts) == 4
    via = pts[1]
    assert math.dist(via, mid) == pytest.approx(2.0)        # clearance + 0.5
    assert pts[2] == pytest.approx(t)


def test_entry_ignores_far_buoys():
    pts = tangent_entry((0.0, 0.0), (10.0, 0.0), 2.0, 'cw', [(5.0, -6.0)], clearance=1.5)
    assert len(pts) == 3


def test_0113_red_entry_from_start_pose_clears_black():
    # 시작 자세에서 빨강(시계)으로 — 3 m 앞 검정을 1.5 m 넘게 비켜 간다
    boat, red, black, green = (168.7, -8.9), (174.5, -1.2), (169.3, -5.9), (166.5, -0.2)
    pts = tangent_entry(boat, red, 2.0, 'cw', [black, green], clearance=1.5)
    for a, b in zip(pts[:-2], pts[1:-1]):
        n = 200
        dmin = min(math.dist((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n), black)
                   for k in range(n + 1))
        assert dmin >= 1.5 - 1e-6


# ---------- 모양 검사 (blob_shape) ----------

def _grid(cells, size=40):
    import numpy as np
    g = np.zeros((size, size), dtype=np.int16)
    for i, j in cells:
        g[j, i] = 100
    return g


def test_blob_shape_small_buoy_vs_long_wall():
    from kaboat_behaviors.search_orbit import blob_shape
    buoy_cells = [(20 + di, 20 + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)]
    s = blob_shape(_grid(buoy_cells), 0.2, (0.0, 0.0), (4.1, 4.1))
    assert s.cells == 9 and 0.4 < s.extent <= 0.8 and s.ring_cells == 0
    wall = [(i, 20) for i in range(5, 35)]                           # 6 m 벽
    w = blob_shape(_grid(wall), 0.2, (0.0, 0.0), (4.1, 4.1))
    assert w.extent > 5.0 and w.cells == 30


def test_blob_shape_counts_separate_neighbours_in_ring():
    from kaboat_behaviors.search_orbit import blob_shape
    cells = [(20, 20), (27, 20), (20, 27)]                          # 1.4 m 떨어진 칸 둘
    s = blob_shape(_grid(cells), 0.2, (0.0, 0.0), (4.1, 4.1))
    assert s.cells == 1 and s.ring_cells == 2


def test_blob_shape_none_outside_grid_or_empty():
    from kaboat_behaviors.search_orbit import blob_shape
    assert blob_shape(_grid([]), 0.2, (0.0, 0.0), (4.1, 4.1)) is None
    assert blob_shape(_grid([(20, 20)]), 0.2, (0.0, 0.0), (100.0, 4.1)) is None


# ---------- 검정: 모양 + 빨강·초록 근접 ----------

def _shape(table):
    return lambda b: table.get(b.id)


def test_black_needs_buoy_shape():
    f = fsm_0113('black')
    dock = buoy('unknown', 171.0, -4.0, 20)                          # 영역 안 unknown 구조물
    black = buoy('unknown', 169.3, -5.9, 3)
    green = buoy('green', 166.5, -0.2, 4)
    f.update([dock, black, green], (168.7, -8.9), 0.0, _shape({20: False, 3: None}))
    assert f.phase == 'SEEK'                                         # 검정 모양 아직 모름 → 안 고름
    f.update([dock, black, green], (168.7, -8.9), 0.1, _shape({20: False, 3: True}))
    assert f.center == (169.3, -5.9)


def test_red_green_only_drop_structures():
    f = fsm_0113('red')
    f.update([buoy('red', 174.5, -1.2, 5)], (168.7, -8.9), 0.0, _shape({5: None}))
    assert f.phase == 'ORBIT'                                        # 멀어서 모름 → 그래도 고름
    g = fsm_0113('red')
    g.update([buoy('red', 174.5, -1.2, 5)], (168.7, -8.9), 0.0, _shape({5: False}))
    assert g.phase == 'SEEK'


def test_unlock_when_locked_turns_out_structure():
    f = fsm_0113('red')
    red = buoy('red', 174.5, -1.2, 5)
    f.update([red], (168.7, -8.9), 0.0, _shape({5: None}))
    assert f.phase == 'ORBIT'
    f.update([red], (172.0, -3.0), 0.1, _shape({5: False}))
    assert f.center is None and f.unlocks == 1


def test_no_shape_unlock_once_circling():
    # 선회 각을 세기 시작하면 모양으로는 안 푼다 — 배가 부표 옆을 돌면 지도 덩어리가 번진다
    f = fsm_0113('red')
    red = buoy('red', 174.5, -1.2, 5)
    f.update([red], (176.5, -1.2), 0.0, _shape({5: None}))          # 원 위에서 고정
    f.update([red], (176.4, -1.8), 0.1, _shape({5: None}))
    assert f.counting
    f.update([red], (176.2, -2.3), 0.2, _shape({5: False}))
    assert f.center is not None and f.unlocks == 0
    green = buoy('green', 174.5, -1.2, 5)                             # 색이 바뀌면 여전히 푼다
    f.update([green], (176.0, -2.6), 0.3, _shape({5: False}))
    assert f.center is None and f.unlocks == 1


def _disk(cx, cy, r, res=0.2):
    k = int(r / res) + 2
    ci, cj = cx / res, cy / res
    return [(i, j) for j in range(int(cj) - k, int(cj) + k + 1) for i in range(int(ci) - k, int(ci) + k + 1)
            if math.hypot(i + 0.5 - ci, j + 0.5 - cj) * res <= r]


def test_blob_shape_excludes_neighbour_buoy_from_ring():
    from kaboat_behaviors.search_orbit import blob_shape
    a, b = (4.1, 4.1), (5.56, 4.1)                                   # 1.46 m 붙은 쌍
    g = _grid(_disk(*a, 0.45) + _disk(*b, 0.45))                     # 실측 덩어리 14–16 칸
    plain = blob_shape(g, 0.2, (0.0, 0.0), a)
    excl = blob_shape(g, 0.2, (0.0, 0.0), a, exclude=[b], exclude_radius=0.7)
    assert plain.ring_cells > 12                                     # 예전: 이웃이 고리 칸 → 구조물
    assert excl.ring_cells == 0 and excl.cells == plain.cells


def test_measure_shapes_pair_passes_but_wall_fragment_fails():
    import numpy as np
    from types import SimpleNamespace as NS
    from kaboat_behaviors.search_orbit import measure_shapes
    p = SearchParams(target_color='black')
    a, b = (4.1, 4.1), (5.56, 4.1)
    wall = [(i, 30) for i in range(5, 35)]                            # y = 6.1 m, 6 m 벽
    g = np.asarray(_grid(_disk(*a, 0.45) + _disk(*b, 0.45) + wall))
    buoys = [NS(id=1, position=NS(x=a[0], y=a[1])), NS(id=2, position=NS(x=b[0], y=b[1])),
             NS(id=3, position=NS(x=4.9, y=6.1))]                    # 벽 위 부표만 한 조각
    out = measure_shapes(g, 0.2, (0.0, 0.0), buoys, (4.8, 0.2), (4.8, 4.1), p)
    assert out == {1: True, 2: True, 3: False}
    # 배가 3 m 안이면 안 잰다
    assert measure_shapes(g, 0.2, (0.0, 0.0), buoys[:1], (4.1, 2.6), (4.8, 4.1), p) == {}


def test_measure_shapes_keeps_structure_neighbours_as_evidence():
    # 부표만 한 도크 조각(1 m) 옆 1.5 m 에 도크 벽(4 m). 벽 위에도 검출 "부표"가 하나 있어도
    # 그 덩어리는 벽 전체라 이웃 부표로 빼지 않는다 → 조각의 고리에 벽이 남아 구조물로 판정
    import numpy as np
    from types import SimpleNamespace as NS
    from kaboat_behaviors.search_orbit import measure_shapes
    p = SearchParams(target_color='black')
    piece = [(i, 20) for i in range(18, 23)] + [(i, 21) for i in range(18, 23)]   # 1.0 m × 0.4 m
    wall = [(i, j) for i in range(10, 30) for j in (27, 28)]                       # 1.4 m 위, 4 m
    g = np.asarray(_grid(piece + wall))
    buoys = [NS(id=1, position=NS(x=4.1, y=4.2)), NS(id=2, position=NS(x=4.1, y=5.6))]
    out = measure_shapes(g, 0.2, (0.0, 0.0), buoys, (4.1, 0.0), (4.1, 4.1), p)
    assert out[1] is False and out[2] is False


def test_black_ranked_by_nearest_red_green_not_area_center():
    # 영역 중심 기준이면 A(중심 3.8 m)를 고르겠지만, 확인된 초록 옆 B(초록 2.3 m)를 고른다
    f = fsm_0113('black')
    a = buoy('unknown', 172.5, -6.0, 30)
    b = buoy('unknown', 168.0, 1.5, 31)
    green = buoy('green', 166.5, -0.2, 4)
    f.update([a, b, green], (168.7, -8.9), 0.0, _shape({30: True, 31: True, 4: True}))
    assert f.center == (168.0, 1.5)


def test_black_beyond_field_gap_ignored():
    # 영역(12 m) 안이지만 빨강에서 10.4 m — 탐색 부표 무리 밖 물체
    f = fsm_0113('black', field_gap=10.0)
    lone = buoy('unknown', 178.0, -11.0, 32)
    green, red = buoy('green', 166.5, -0.2, 4), buoy('red', 174.5, -1.2, 5)
    f.update([lone, green, red], (168.7, -8.9), 0.0, _shape({32: True}))
    assert f.phase == 'SEEK'


def test_black_needs_red_or_green_seen_by_default():
    # 부표만 한 도크 조각은 모양으로 못 거른다 — 빨강·초록을 못 봤으면 검정을 안 고름
    unknowns = [buoy('unknown', 169.3, -5.9, 3), buoy('unknown', 175.0, -8.0, 7)]
    f = fsm_0113('black')
    f.update(unknowns, (168.7, -8.9), 0.0, _shape({3: True, 7: True}))
    assert f.phase == 'SEEK'
    g = fsm_0113('black', black_needs_field=False)                   # 끄면 영역 중심 거리로
    g.update(unknowns, (168.7, -8.9), 0.0, _shape({3: True, 7: True}))
    assert g.center == (169.3, -5.9)


# ---------- SEEK 목적지는 부표에서 비킨다 ----------

def test_clear_point_moves_off_buoys():
    assert clear_point((50, 70), [], 2.5) == (50.0, 70.0)
    assert clear_point((50, 70), [(60, 70)], 2.5) == (50.0, 70.0)          # 멀면 그대로
    p = clear_point((50, 70), [(50, 70)], 2.5, toward=(40, 70))           # 부표 위 → 배 쪽으로
    assert p == pytest.approx((47.5, 70.0))
    p = clear_point((50, 70), [(51, 70)], 2.5)                            # 옆 → 바깥으로
    assert p == pytest.approx((48.5, 70.0))
    p = clear_point((50, 70), [(51, 70), (48.5, 70.5)], 2.5)              # 둘 사이
    assert min(math.dist(p, o) for o in [(51, 70), (48.5, 70.5)]) >= 2.5 - 1e-6


def test_seek_arrives_at_cleared_point_and_still_gives_up():
    # 영역 중심(GOAL) 위에 목표 아닌 부표 — 그 자리 대신 2.5 m 비킨 점에서 둘러보고, 거기서 포기 시계가 돈다
    f = fsm('red', give_up_sec=90.0)
    other = [buoy('green', 50.0, 70.0)]
    assert f.update(other, (40, 70), 0.0) == 'SEEK'
    assert f.seek_xy == pytest.approx((47.5, 70.0))
    f.update(other, (47.4, 70.0), 10.0)                                   # 비킨 점 도착 (중심 2.6 m)
    assert f.update(other, (47.4, 70.0), 99.0) == 'SEEK'
    assert f.update(other, (47.4, 70.0), 101.0) == 'GIVE_UP'


def test_clear_point_near_buoy_goes_toward_boat():
    # 영역 중심이 부표에서 3 cm — p−o 방향(동쪽) 대신 배(남쪽) 쪽으로
    p = clear_point((50.03, 70.0), [(50.0, 70.0)], 2.5, toward=(50.0, 60.0))
    assert p == pytest.approx((50.0, 67.5))


def test_seek_point_stays_put_while_detection_jitters():
    f = fsm('red', give_up_sec=90.0)
    first = None
    for k, (dx, dy) in enumerate([(0.03, 0.0), (-0.04, 0.02), (0.0, -0.05), (0.05, 0.05)]):
        f.update([buoy('green', 50.0 + dx, 70.0 + dy)], (40.0, 70.0 + k * 0.5), float(k))
        first = first or f.seek_xy
        assert f.seek_xy == first                                     # 흔들려도 그대로
    assert math.dist(first, (50.0, 70.0)) == pytest.approx(2.5, abs=0.06)
    f.update([buoy('green', 50.0, 70.0), buoy('red', first[0], first[1] + 1.0, 9)], (40, 70), 5.0)
    assert f.seek_xy != first                                         # 새 부표가 그 점 옆 → 다시 비킴


def test_yaw_kd_default_and_range():
    # 2026-11-14: 탐색 LOS 선수각 D 0.16 → 0.6 (회피와 같은 값)
    assert SearchParams().yaw_kd == 0.6
    for bad in (-0.1, 2.5, float('nan')):
        with pytest.raises(ValueError):
            SearchParams(target_color='red', yaw_kd=bad)
