"""search_orbit 단독 테스트 — ROS 없이 돈다 (부표는 color·position 을 가진 수제 객체)."""
import math
from types import SimpleNamespace

import pytest

from kaboat_behaviors.search_orbit import SearchFsm, SearchParams

GOAL = (50.0, 70.0)


def buoy(color, x, y, bid=None):
    return SimpleNamespace(color=color, position=SimpleNamespace(x=x, y=y), id=bid)


def fsm(color='red', **kw):
    kw.setdefault('lock_confirm_sec', 0.0)   # 기존 시나리오는 바로 고정
    return SearchFsm(SearchParams(target_color=color, **kw), GOAL)


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
    # search_end 를 부표 위에 찍은 경우 (sim mission_params 가 그렇다) — 부표에서 배 쪽으로 1.5 m
    f = fsm('red', done_clearance=1.5)
    f.update([buoy('red', 50, 70)], (40, 70), 0.0)
    x, y = f.exit_point((50, 72))
    assert (x, y) == pytest.approx((50.0, 71.5))
    assert math.dist((x, y), GOAL) < 2.0          # mission_manager 전환 반경 안


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
    f = SearchFsm(SearchParams(target_color='black', lock_confirm_sec=1.0), GOAL)
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.0) == 'SEEK'
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.5) == 'SEEK'
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 1.1) == 'ORBIT'


def test_colour_flicker_restarts_confirm_clock():
    f = SearchFsm(SearchParams(target_color='black', lock_confirm_sec=1.0), GOAL)
    f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.0)
    f.update([buoy('green', 51.0, 70.0, 5)], (40, 70), 0.6)    # 색이 바뀜 — 시계 다시
    f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 0.8)
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 1.5) == 'SEEK'
    assert f.update([buoy('unknown', 51.0, 70.0, 5)], (40, 70), 1.9) == 'ORBIT'
