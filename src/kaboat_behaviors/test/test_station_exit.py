"""위치 유지 출구: station_center 를 중심으로 유지한 뒤 station_end(출구)로 나가서 완료."""

import math

import numpy as np
import pytest

from kaboat_behaviors.station_keeping import (
    Buoy, StationDecision, StationParams, exit_waypoints)

P = StationParams()
CENTER = (155.16, -31.21)


def _square(center, side=3.0, rot_deg=20.0):
    r = math.radians(rot_deg)
    out = []
    for dx, dy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        x, y = dx * side / 2, dy * side / 2
        out.append(Buoy(center[0] + x * math.cos(r) - y * math.sin(r),
                        center[1] + x * math.sin(r) + y * math.cos(r), 10.0))
    return out


def _seg_dist(p, a, b):
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * (b - a))))


@pytest.mark.parametrize('exit_bearing_deg', [0, 30, 45, 120, -100, 180])
def test_exit_goes_through_a_side_never_near_a_corner(exit_bearing_deg):
    buoys = _square(CENTER)
    b = math.radians(exit_bearing_deg)
    exit_xy = (CENTER[0] + 5 * math.cos(b), CENTER[1] + 5 * math.sin(b))
    route = exit_waypoints(buoys, CENTER, exit_xy, P)
    assert len(route) == 2 and route[-1] == exit_xy
    legs = [(CENTER, route[0]), (route[0], route[1])]
    for buoy in buoys:                      # 중심→밖 경유점→출구 내내 부표에서 1.2 m 넘게
        assert min(_seg_dist((buoy.x, buoy.y), a, c) for a, c in legs) > 1.2
    outside = math.dist(route[0], CENTER)
    assert outside == pytest.approx(1.5 + 1.5, abs=1e-6)   # 변 중점(1.5 m) + 1.5 m


def test_three_buoys_ignore_diagonal_and_two_buoys_use_their_side():
    buoys = _square(CENTER, rot_deg=0.0)[:3]           # (+,+) (−,+) (−,−) — 4번째 안 보임
    route = exit_waypoints(buoys, CENTER, (CENTER[0] - 5, CENTER[1]), P)
    assert route[0][0] == pytest.approx(CENTER[0] - 3.0)   # 서쪽 변(실제 변)으로
    two = _square(CENTER, rot_deg=0.0)[:2]              # 북쪽 변만
    route = exit_waypoints(two, CENTER, (CENTER[0], CENTER[1] + 5), P)
    assert route[0][1] == pytest.approx(CENTER[1] + 3.0)
    assert exit_waypoints([], CENTER, (1.0, 2.0), P) == [(1.0, 2.0)]


# ── 노드: 유지 끝 → 출구 경유점 → station_end 에서 완료 ──────────────────

def _node(tmp_path, with_center=True):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from kaboat_mission.mission_points import MissionPoint, PointStore
    from kaboat_behaviors.station_keeper import StationKeeper
    origin = (35.18487880, 128.56510100, 30.0)
    (tmp_path / 'o.yaml').write_text(f'origin_lat: {origin[0]}\norigin_lon: {origin[1]}\n'
                                     f'origin_alt: {origin[2]}\n')
    points = {'station_center': MissionPoint(*CENTER, 0.0)} if with_center else {}
    PointStore(origin, points).save(str(tmp_path / 'p.json'))
    rclpy.init(args=['--ros-args', '-p', f'points_file:={tmp_path / "p.json"}',
                     '-p', f'origin_file:={tmp_path / "o.yaml"}'])
    node = StationKeeper()
    node.done = []
    node.complete_pub.publish = node.done.append
    node._on_state(String(data='station'))
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x, goal.pose.position.y = CENTER[0] + 5.0, CENTER[1]   # station_end = 출구
    goal.pose.orientation.w = 1.0
    node._on_goal(goal)
    return node


def _at(node, x, y):
    from nav_msgs.msg import Odometry
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
    odom.pose.pose.orientation.w = 1.0
    node._on_odom(odom)
    return node.compute_cmd()


def test_node_holds_at_center_then_exits_before_reporting(tmp_path):
    import rclpy
    node = _node(tmp_path)
    try:
        _at(node, *CENTER)
        assert node.fsm.goal == pytest.approx(CENTER)       # FSM 중심 = station_center
        buoys = _square(CENTER, rot_deg=0.0)
        hold = StationDecision(phase='HOLD', kind='anchor', center=CENTER,
                               center_kind='4', buoys=buoys, completed_now=True)
        node.fsm.step = lambda *a: hold
        _at(node, *CENTER)                                   # 유지 5 s 달성
        assert node.done == []                               # 아직 완료 아님 — 출구로
        assert node._exit_targets[0] == pytest.approx((CENTER[0] + 3.0, CENTER[1]))
        hold.completed_now = False
        cmd = _at(node, CENTER[0] + 0.5, CENTER[1])
        assert cmd.linear.x > 0.0                            # 동쪽 변으로 전진
        _at(node, CENTER[0] + 3.0, CENTER[1])                # 사각형 밖 경유점
        assert node.done == []
        _at(node, CENTER[0] + 4.6, CENTER[1])                # station_end 0.4 m
        assert [m.data for m in node.done] == ['station']    # 출구에서 완료
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_without_station_center_old_behaviour(tmp_path):
    import rclpy
    node = _node(tmp_path, with_center=False)
    try:
        _at(node, CENTER[0] + 5.0, CENTER[1])
        assert node.fsm.goal == pytest.approx((CENTER[0] + 5.0, CENTER[1]))   # station_end 가 중심
        done = StationDecision(phase='HOLD', kind='anchor', center=node.fsm.goal,
                               buoys=[], completed_now=True)
        node.fsm.step = lambda *a: done
        _at(node, CENTER[0] + 5.0, CENTER[1])
        assert [m.data for m in node.done] == ['station'] and node._exit_targets is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
