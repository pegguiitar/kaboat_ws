"""dock_ctrl_v2 순서: YOLO 로 목표 슬롯 방위 고정 → 서행 → 고정 치수 ㄷ자 슬롯 고정 → 중심선과 평행 진입.

도크(odom 그대로): 뒷벽 y = 0, 물 쪽 +y, 슬롯 중심 x = -4, 0, 4, 폭 2.97, 깊이 1.94 (dock_slot.yaml 실측).
배는 y > 0 에서 -y(-90°) 쪽을 본다.
"""
import json
import math

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from kaboat_hardware.dock_ctrl_v2 import DockCtrlV2, front_clearance
from kaboat_hardware.dock_one_shot import wrap

WIDTH, DEPTH, PITCH, FINGER = 2.97, 1.94, 4.0, 1.03
FACING_WALL = -math.pi / 2


@pytest.fixture
def node():
    rclpy.init()
    n = DockCtrlV2(parameter_overrides=[
        Parameter('mission_mode', value=True), Parameter('drive_enabled', value=True),
        Parameter('target_class', value='circle')])
    n.sent, n.reports = [], []
    n.cmd.publish = n.sent.append
    n.status.publish = lambda msg: n.reports.append(json.loads(msg.data))
    n._on_mission_state(String(data='dock'))
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _odom(node, x, y, yaw=FACING_WALL, rate=0.0):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    msg.twist.twist.angular.z = float(rate)
    node._odom(msg)


def _scan(node, ahead_m=None):
    msg = LaserScan(angle_min=-math.pi, angle_increment=math.radians(1.0), range_min=.4, range_max=50.)
    msg.ranges = [float('inf')] * 360
    if ahead_m is not None:
        for i in range(176, 185):            # 앞쪽 ±4°
            msg.ranges[i] = float(ahead_m)
    node._scan(msg)


def _det(node, bearing_deg, name='circle', conf=.9):
    node._det(String(data=json.dumps({'sight_rays': {name: {
        'confidence': conf, 'pixel_uv': [320 - 4 * bearing_deg, 220],
        'bearing_left_deg': bearing_deg}}})))


def _comb():
    half = WIDTH / 2
    centres = [-PITCH, 0.0, PITCH]
    segs = [((c - half, 0.0), (c + half, 0.0)) for c in centres]
    fingers = [(centres[0] - half - FINGER, centres[0] - half)]
    fingers += [(a + half, b - half) for a, b in zip(centres, centres[1:])]
    fingers += [(centres[-1] + half, centres[-1] + half + FINGER)]
    for a, b in fingers:
        segs += [((a, 0.0), (a, DEPTH)), ((b, 0.0), (b, DEPTH)), ((a, DEPTH), (b, DEPTH))]
    return segs


def _cloud(boat, step_deg=.2, max_range=12.0):
    """boat 에서 빔을 쏴 가장 가까운 도크 면에 맞은 점 (가림 재현)."""
    angles = np.radians(np.arange(-180, 180, step_deg))
    d = np.c_[np.cos(angles), np.sin(angles)]
    best = np.full(len(angles), np.inf)
    o = np.asarray(boat, float)
    for (ax, ay), (bx, by) in _comb():
        e = np.array([bx - ax, by - ay])
        den = d[:, 0] * e[1] - d[:, 1] * e[0]
        ok = np.abs(den) > 1e-12
        rel = np.array([ax, ay]) - o
        t = np.where(ok, (rel[0] * e[1] - rel[1] * e[0]) / np.where(ok, den, 1), np.inf)
        u = np.where(ok, (rel[0] * d[:, 1] - rel[1] * d[:, 0]) / np.where(ok, den, 1), -1)
        hit = ok & (t > .05) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    keep = best < max_range
    return o + d[keep] * best[keep][:, None]


def _feed_slot(node, boat, n=6, t0=100.0, dt=.25, keep=None):
    """배 위치에서 본 도크 점을 0.25 s 간격으로 n 번 넣고 tick."""
    pts = _cloud(boat)
    if keep is not None:
        pts = pts[keep(pts)]
    for k in range(n):
        node._add_cloud(t0 + dt * k, pts)
        node._tick()


def _lock_bearing(node, x=0.0, y=10.0):
    _odom(node, x, y)
    _scan(node)
    for _ in range(node.bearing_n):
        _det(node, 1.0)
        node._tick()
    assert node.phase == 'SEEK'


def _odom_at(node, t, yaw, rate=0.0, x=0.0, y=10.0):
    """헤더 시각이 있는 odom (지연 보상 시험용)."""
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    msg.twist.twist.angular.z = float(rate)
    node._odom(msg)


def _det_at(node, bearing_deg, image_stamp, name='circle', conf=.9):
    node._det(String(data=json.dumps({'image_stamp_s': image_stamp, 'sight_rays': {name: {
        'confidence': conf, 'pixel_uv': [320 - 4 * bearing_deg, 220],
        'bearing_left_deg': bearing_deg}}})))


def test_scan_turns_toward_target_then_locks_bearing(node):
    _odom(node, 0.0, 10.0)
    _scan(node)
    _det(node, 15.0)                         # 목표가 왼쪽 15° (고정 표본 ±8° 밖) — 멈춘 채 왼쪽으로 돈다
    node._tick()
    assert node.phase == 'SCAN'
    assert node.sent[-1].linear.x == 0.0 and node.sent[-1].angular.z > 0.0
    _det(node, 15.0, name='square')          # 한 프레임 놓쳐도 마지막 세계 방위로 계속 돈다
    node._tick()
    assert node.sent[-1].linear.x == 0.0 and node.sent[-1].angular.z > 0.0
    node.scan_heading_received -= 10.0       # 목표를 오래 못 보면 멈춰 기다린다
    node._tick()
    assert node.sent[-1].linear.x == 0.0 and node.sent[-1].angular.z == 0.0
    for k in range(node.bearing_n):
        _det(node, 1.0)
        node._tick()
        assert node.phase == ('SEEK' if k == node.bearing_n - 1 else 'SCAN')
    assert node.locked_bearing == pytest.approx(FACING_WALL + math.radians(1.0))


def test_off_centre_or_scattered_frames_do_not_lock(node):
    _odom(node, 0.0, 10.0)
    _scan(node)
    for _ in range(10):
        _det(node, 12.0)                                # 가운데 ±8° 밖
        node._tick()
    assert node.phase == 'SCAN'
    for k in range(10):
        _det(node, 4.0 if k % 2 else -4.0)              # 세계 방위가 8° 흩어짐
        node._tick()
    assert node.phase == 'SCAN'


def test_turning_boat_locks_latency_compensated_bearing(node):
    """2026-11-13 bag: 도는 중에도 촬영 시각(0.21 s 전) 선수각으로 바꾸면 세계 방위가 맞다
    (받은 시각 선수각이면 10°/s × 0.21 s = 2.1° 틀림)."""
    target = math.radians(-102.0)
    rate = math.radians(10.0)
    _scan(node)
    t, yaw = 1000.0, math.radians(-112.0)
    for _ in range(25):                                  # 1 s 의 odom 이력 (25 Hz)
        _odom_at(node, t, yaw, rate)
        t += .04
        yaw += rate * .04
    for _ in range(node.bearing_n):
        image_t = t - .21                                # 0.21 s 전에 찍은 영상의 방위
        seen = node._yaw_at(image_t)
        _det_at(node, math.degrees(wrap(target - seen)), image_t)
        node._tick()
        for _ in range(4):
            _odom_at(node, t, yaw, rate)
            t += .04
            yaw += rate * .04
    assert node.phase == 'SEEK'
    assert node.locked_bearing == pytest.approx(target, abs=math.radians(.5))


def test_yaw_rate_gate_is_optional(node):
    node.lock_rate = math.radians(4.0)                   # 켜면 도는 중 프레임은 고정에 안 씀
    _odom(node, 0.0, 10.0, rate=math.radians(10.0))
    _scan(node)
    for _ in range(10):
        _det(node, 1.0)
        node._tick()
    assert node.phase == 'SCAN'


def test_scan_turn_brakes_near_centre(node):
    """예전 ±3° 데드밴드는 가운데서 명령 0 → 감쇠도 꺼져 20°/s 로 지나쳤다."""
    _odom(node, 0.0, 10.0, rate=math.radians(20.0))
    _scan(node)
    _det(node, 1.0)
    node._tick()
    assert node.sent[-1].angular.z < 0.0                 # 왼쪽으로 도는 중 → 반대로 제동


def test_seek_creeps_along_bearing_until_u_slot_locks(node):
    _lock_bearing(node)
    _odom(node, 0.0, 6.5)
    node._tick()
    assert node.sent[-1].linear.x == pytest.approx(node.seek_cmd, rel=.3)
    _feed_slot(node, (0.4, 6.5), n=6)                  # 6 m 밖 — 보여도 고정 안 함 (2026-11-13 bag)
    assert node.phase == 'SEEK' and not node.slot_samples
    assert 'closer than 6.0m' in node.reports[-1]['slot']['reason']
    _odom(node, 0.0, 5.5)
    _feed_slot(node, (0.4, 5.5), n=4, t0=110.0)        # 0.75 s — 아직 1 s 미만
    assert node.phase == 'SEEK'
    assert node.reports[-1]['slot']['found'] is True
    _feed_slot(node, (0.4, 5.5), n=2, t0=111.0)
    assert node.phase == 'TO_VIA'
    via, stop = node.route_enu[1], node.route_enu[2]
    # 경유점 = 입구(1.94) 밖 pre_entry_offset 1.5, 정지점 = 뒷벽 앞 berth_distance 1.0 (dock_slot.yaml)
    assert via.tolist() == pytest.approx([0.0, 3.44], abs=.05)
    assert stop.tolist() == pytest.approx([0.0, 1.0], abs=.05)
    assert node.final_heading == pytest.approx(FACING_WALL, abs=math.radians(1.0))


def test_finger_tips_alone_or_repeated_clouds_do_not_lock(node):
    _lock_bearing(node)
    _odom(node, 0.0, 8.0)
    _feed_slot(node, (0.0, 8.0), n=8, keep=lambda p: p[:, 1] > DEPTH - .05)   # 칸막이 끝만
    assert node.phase == 'SEEK' and not node.slot_samples
    assert 'waiting for U slot' in node.reports[-1]['detail']
    cloud = _cloud((0.0, 8.0))
    node._add_cloud(200.0, cloud)
    node._add_cloud(200.0, cloud)                       # 같은 스캔 시각 — 한 번만
    assert len(node.cloud) == 1


def test_too_close_lock_backs_off_before_fixing_route(node):
    _lock_bearing(node)
    _odom(node, 0.0, 3.6)                    # 경유점(3.44 m) + 여유 0.5 m 보다 가까움
    _feed_slot(node, (0.0, 3.6), n=5)
    assert node.phase == 'BACKOFF'
    node._tick()
    assert node.sent[-1].linear.x == -node.retreat_cmd
    _odom(node, 0.0, 4.0)
    node._tick()
    assert node.phase == 'TO_VIA' and node.route_enu is not None


def test_seek_holds_for_front_obstacle_stale_scan_and_travel_limit(node):
    _lock_bearing(node)
    _scan(node, ahead_m=1.5)
    node._tick()
    assert node.sent[-1].linear.x == 0.0 and 'ahead' in node.reports[-1]['detail']
    _scan(node)
    node.scan_received -= 10.0               # 스캔 끊김
    node._tick()
    assert node.sent[-1].linear.x == 0.0 and 'scan stale' in node.reports[-1]['detail']
    _scan(node)
    _odom(node, 0.0, 10.0 - node.seek_max_travel)
    node._tick()
    assert node.sent[-1].linear.x == 0.0 and node.phase == 'SEEK'
    assert 'no U slot' in node.reports[-1]['detail']


def test_front_clearance_ignores_spray_and_side_points():
    scan = LaserScan(angle_min=-math.pi, angle_increment=math.radians(1.0), range_min=.4, range_max=50.)
    scan.ranges = [float('inf')] * 360
    scan.ranges[180] = .6                    # 0.8 m 안 — 선체·물보라
    scan.ranges[181] = scan.ranges[182] = 1.0 # 두 점뿐
    assert front_clearance(scan, math.radians(25), 2.0) is None
    for i in (100, 101, 102, 103):           # 옆(-80°)
        scan.ranges[i] = 1.0
    assert front_clearance(scan, math.radians(25), 2.0) is None
    scan.ranges[183] = 1.2
    assert front_clearance(scan, math.radians(25), 2.0) == pytest.approx(1.0)


def test_seek_moves_only_while_target_centred(node):
    """SEEK: 목표가 선수 ±seek_center_deg 밖이면 전진 0 으로 선수만 맞춘다 (선회율 제약 없음)."""
    _lock_bearing(node)
    _odom(node, 0.0, 9.0, yaw=FACING_WALL - math.radians(15.0), rate=math.radians(15.0))
    node._tick()
    assert node.phase == 'SEEK'
    assert node.sent[-1].linear.x == 0.0 and node.sent[-1].angular.z > 0.0
    _odom(node, 0.0, 9.0, yaw=FACING_WALL, rate=math.radians(15.0))   # 가운데면 돌고 있어도 전진
    node._tick()
    assert node.sent[-1].linear.x == pytest.approx(node.seek_cmd)


def test_seek_follows_target_sight_and_ignores_jumps(node):
    _lock_bearing(node)                                  # FACING_WALL + 1°
    _odom(node, 0.0, 9.0)
    before = node.locked_bearing
    for _ in range(10):
        _det(node, 5.0)                                  # 목표가 왼쪽 5° 로 보임 → 방위 갱신
        node._tick()
    assert node.locked_bearing == pytest.approx(FACING_WALL + math.radians(5.0), abs=math.radians(.3))
    assert node.seek_line_point == pytest.approx([0.0, 9.0])
    settled = node.locked_bearing
    _det(node, -15.0)                                    # 한 번에 20° 튐 (오검출) → 버림
    node._tick()
    assert node.locked_bearing == pytest.approx(settled)
    assert before != settled
    for _ in range(3):
        _det(node, 5.0, name='square')                   # 목표 안 보임 → 마지막 방위 유지
        node._tick()
    assert node.locked_bearing == pytest.approx(settled)
