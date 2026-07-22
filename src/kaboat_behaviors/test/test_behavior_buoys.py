"""behavior_base 부표맵 헬퍼 단독 테스트 — ROS 없이 돈다.

buoy_range_bearing/visible_buoys 는 self.odom·self.buoys 만 읽는 순수 계산이라,
rclpy.init 없이 같은 필드를 가진 수제 객체(덕타이핑, test_dri 와 같은 방식)로
unbound 메서드를 직접 호출해 검증한다. 이게 소비자 마이그레이션(월드 좌표
부표 → 배 기준 거리·방위)의 핵심이라 red+green 경로가 없는 sim 씬 대신 여기서
기하를 못박는다.
"""
import math
from types import SimpleNamespace

import pytest

from kaboat_behaviors.behavior_base import BehaviorBase


def fake_odom(x, y, yaw):
    return SimpleNamespace(
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y),
            orientation=SimpleNamespace(
                w=math.cos(yaw / 2), x=0.0, y=0.0, z=math.sin(yaw / 2)))))


def buoy(color, x, y):
    return SimpleNamespace(color=color, position=SimpleNamespace(x=x, y=y))


def rb(odom, b):
    return BehaviorBase.buoy_range_bearing(SimpleNamespace(odom=odom), b)


# ---------- buoy_range_bearing (월드 좌표 → 배 기준) ----------

def test_none_when_no_odom():
    assert BehaviorBase.buoy_range_bearing(SimpleNamespace(odom=None),
                                           buoy('red', 5, 0)) is None


def test_straight_ahead():
    dist, bearing = rb(fake_odom(0, 0, 0.0), buoy('red', 5, 0))
    assert dist == pytest.approx(5.0) and bearing == pytest.approx(0.0)


def test_left_is_positive_bearing():
    dist, bearing = rb(fake_odom(0, 0, 0.0), buoy('green', 0, 5))
    assert dist == pytest.approx(5.0) and bearing == pytest.approx(math.pi / 2)


def test_yaw_rotates_frame():
    # 뱃머리 +y(yaw90°)로 돌면, 월드 (0,5) 부표는 정면(방위 0)이 된다
    dist, bearing = rb(fake_odom(0, 0, math.pi / 2), buoy('red', 0, 5))
    assert dist == pytest.approx(5.0) and bearing == pytest.approx(0.0, abs=1e-9)


def test_boat_offset_translates():
    dist, bearing = rb(fake_odom(10, 80, 0.0), buoy('red', 15, 80))
    assert dist == pytest.approx(5.0) and bearing == pytest.approx(0.0)


def test_roundtrip_with_project_to_world():
    # project_to_world(정방향) ↔ buoy_range_bearing(역방향) 왕복 일치
    from kaboat_perception.buoy_tracker import project_to_world
    bx, by, yaw = 3.0, -2.0, 0.7
    wx, wy = project_to_world(0.4, 6.0, bx, by, yaw)
    dist, bearing = rb(fake_odom(bx, by, yaw), buoy('red', wx, wy))
    assert dist == pytest.approx(6.0) and bearing == pytest.approx(0.4)


# ---------- visible_buoys (거리·전방각 필터 + 정렬) ----------

def make_self(odom, buoys):
    s = SimpleNamespace(odom=odom, buoys=buoys)
    # visible_buoys 가 내부에서 self.buoy_range_bearing 을 부른다 — 실제 메서드 연결
    s.buoy_range_bearing = lambda b: BehaviorBase.buoy_range_bearing(s, b)
    return s


def test_visible_filters_by_color():
    s = make_self(fake_odom(0, 0, 0.0),
                  [buoy('red', 5, 0), buoy('green', 6, 0)])
    out = BehaviorBase.visible_buoys(s, 'red')
    assert len(out) == 1 and out[0][0].color == 'red'


def test_visible_drops_far_buoys():
    s = make_self(fake_odom(0, 0, 0.0),
                  [buoy('red', 5, 0), buoy('red', 100, 0)])
    out = BehaviorBase.visible_buoys(s, 'red', max_range=15.0)
    assert len(out) == 1 and out[0][1] == pytest.approx(5.0)


def test_visible_drops_behind_buoys():
    # 뒤(월드 -x, yaw0 기준 방위 ±π)는 전방 시야(±50°) 밖
    s = make_self(fake_odom(0, 0, 0.0),
                  [buoy('red', 5, 0), buoy('red', -5, 0)])
    out = BehaviorBase.visible_buoys(s, 'red')
    assert len(out) == 1 and out[0][2] == pytest.approx(0.0)


def test_visible_sorted_by_range():
    s = make_self(fake_odom(0, 0, 0.0),
                  [buoy('red', 9, 0), buoy('red', 4, 0), buoy('red', 6, 0)])
    dists = [t[1] for t in BehaviorBase.visible_buoys(s, 'red')]
    assert dists == sorted(dists)
