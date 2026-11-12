"""search_orbit 단독 테스트 — ROS 없이 돈다 (부표는 color·position 을 가진 수제 객체)."""
import math
from types import SimpleNamespace

import pytest

from kaboat_behaviors.search_orbit import SearchFsm, SearchParams

GOAL = (50.0, 70.0)


def buoy(color, x, y):
    return SimpleNamespace(color=color, position=SimpleNamespace(x=x, y=y))


def fsm(color='red', **kw):
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
    f = fsm('red', refine_alpha=0.5)
    f.update([buoy('red', 51, 70)], (40, 70), 0.0)
    f.update([buoy('red', 52, 70)], (40, 70), 0.1)
    assert f.center == pytest.approx((51.5, 70.0))


def test_refine_ignores_other_colour_and_far_buoys():
    f = fsm('red', relock_radius=1.5)
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
