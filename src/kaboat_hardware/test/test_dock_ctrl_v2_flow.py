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
    node.scan_heading_received -= 10.0       # 목표를 오래 못 보면 다른 도형(왼쪽) 쪽으로 탐색 선회
    node._tick()
    assert node.sent[-1].linear.x == 0.0
    assert node.sent[-1].angular.z == pytest.approx(node.search_turn)
    assert 'searching left' in node.reports[-1]['detail']
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


def test_ctrl_markers_show_phase_sight_and_bearing_line(node):
    """RViz /dock/ctrl_markers: 단계 글자·전이 기록, 목표 시선, SEEK 고정 방위선·가운데 범위."""
    published = []
    node.ctrl_markers_pub.publish = published.append
    _lock_bearing(node)
    _det(node, 1.0)
    node._tick()
    markers = {m.id: m for m in published[-1].markers}
    assert markers[0].text.startswith('SEEK')
    assert [line.split()[-1] for line in markers[1].text.splitlines()][-2:] == ['SCAN', 'SEEK']
    assert {2, 4, 5} <= set(markers)                      # 시선, 고정 방위선, ±가운데
    line = markers[4].points
    heading = math.atan2(line[1].y - line[0].y, line[1].x - line[0].x)
    assert heading == pytest.approx(node.locked_bearing)


def _shapes(node, **bearings):
    """목표가 아닌 도형들만 보이는 검출 (도형=화면 방위°)."""
    node._det(String(data=json.dumps({'sight_rays': {
        name: {'confidence': .9, 'pixel_uv': [320 - 4 * b, 220], 'bearing_left_deg': b}
        for name, b in bearings.items()}})))


def test_search_far_pair_turns_past_the_group(node):
    """멀리서 사각·원이 오른쪽에 몰려 보이면 삼각은 원 너머 오른쪽 (왼쪽 후보 +15° 는 화면 안이라 아님)."""
    node.target = 'triangle'
    _odom(node, 0.0, 10.0)
    _scan(node)
    _shapes(node, square=-5.0, circle=-25.0)
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(-node.search_turn)
    _shapes(node, square=12.0, circle=-8.0)            # 돌면서 도형이 왼쪽으로 밀려도 방향 유지
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(-node.search_turn)
    _det(node, -20.0, name='triangle')                 # 목표가 보이면 그쪽으로 선수 맞추기
    node._tick()
    assert node.sent[-1].angular.z < 0 and 'turning to triangle' in node.reports[-1]['detail']
    assert node.search_dir is None


def test_search_close_pair_uses_spacing_not_mean(node):
    """가까우면 양쪽에 걸쳐 보임: 사각 +36°, 원 −3° — 평균(+16.5°)은 왼쪽이지만 간격(39°)으로 보면
    삼각은 −42° (오른쪽 화면 바로 밖, 반화각 40°), 왼쪽 후보 +75° 는 더 멀다."""
    node.target = 'triangle'
    _odom(node, 0.0, 5.5)
    _scan(node)
    _shapes(node, square=36.0, circle=-3.0)
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(-node.search_turn)
    assert node.sent[-1].linear.x == 0.0


def test_search_excludes_candidate_inside_real_fov(node):
    """45d5c46 검토 예시: 원 −2°·사각 +30° → 후보 +62° / −34°. 실제 반화각 40° 면 −34° 는 화면 안
    (보였을 것)이라 목표가 아니다 → 왼쪽. 예전 34.5° 가정이면 오른쪽으로 잘못 돌았다."""
    node.target = 'triangle'
    _odom(node, 0.0, 6.0)
    _scan(node)
    _shapes(node, square=30.0, circle=-2.0)
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(node.search_turn)


def test_search_uses_half_fov_from_detection_status(node):
    """검출 상태의 image_half_fov_deg 가 있으면 그것 — 좌우 30° 면 −34° 후보는 화면 밖 → 오른쪽."""
    node.target = 'triangle'
    _odom(node, 0.0, 6.0)
    _scan(node)
    node._det(String(data=json.dumps({'image_half_fov_deg': {'left': 30.0, 'right': 30.0}, 'sight_rays': {
        name: {'confidence': .9, 'pixel_uv': [320 - 4 * b, 220], 'bearing_left_deg': b}
        for name, b in (('square', 30.0), ('circle', -2.0))}})))
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(-node.search_turn)


def test_search_stops_after_two_turns_and_resumes_when_target_seen(node):
    _scan(node)
    yaw = 0.0
    for k in range(80):                                # 10°씩 돌며 800° — 720° 에서 멈춰야 함
        _odom(node, 0.0, 10.0, yaw=yaw)
        _shapes(node)
        node._tick()
        yaw = math.atan2(math.sin(yaw - math.radians(10)), math.cos(yaw - math.radians(10)))
        if node.search_exhausted:
            break
    assert node.search_exhausted and 690 <= math.degrees(node.search_turned) <= 740
    assert node.sent[-1].angular.z == 0.0 and node.sent[-1].linear.x == 0.0
    assert 'not found after 2 turns' in node.reports[-1]['detail']
    _odom(node, 0.0, 10.0, yaw=yaw)
    _shapes(node)
    node._tick()
    assert node.sent[-1].angular.z == 0.0               # 멈춘 채 기다림 (다시 돌지 않음)
    _det(node, 12.0)                                   # 목표가 보이면 정상 SCAN 으로
    node._tick()
    assert node.sent[-1].angular.z > 0.0 and not node.search_exhausted
    assert 'turning to circle' in node.reports[-1]['detail']


def test_search_clockwise_when_nothing_seen_then_follows_first_shapes(node):
    _odom(node, 0.0, 10.0)
    _scan(node)
    _shapes(node)                                      # YOLO 는 돌지만 아무 도형도 없음
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(-node.search_turn)     # 시계방향
    assert 'no shape in view' in node.reports[-1]['detail']
    _shapes(node, triangle=20.0)                       # 가운데 밖 왼쪽에 다른 도형 — 그쪽으로 바꿈
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(node.search_turn)
    _shapes(node, triangle=-20.0)                      # 같은 근거(1개)로는 다시 안 바꿈
    node._tick()
    assert node.sent[-1].angular.z == pytest.approx(node.search_turn)


def test_single_centred_shape_turns_opposite_to_its_offset(node):
    """도크 4 m 안: 원 하나만 가운데 근처. 오른쪽(−3°)으로 치우쳤으면 왼쪽 옆 도형이 화면 가장자리에
    더 가깝다 → 왼쪽으로. 후진은 하지 않는다 (라이다가 뒤를 못 봄)."""
    node.target = 'triangle'
    _odom(node, 0.0, 4.0)
    _scan(node)
    _shapes(node, circle=-3.0)
    node._tick()
    assert node.sent[-1].linear.x == 0.0 and node.sent[-1].angular.z == pytest.approx(node.search_turn)
    node._clear_search()
    _shapes(node, circle=3.0)                          # 왼쪽으로 치우쳤으면 오른쪽으로
    node._tick()
    assert node.sent[-1].linear.x == 0.0 and node.sent[-1].angular.z == pytest.approx(-node.search_turn)


def _turn_search(node, yaw, deg, step=10.0, **shapes):
    """deg 만큼 search 방향으로 돌며 tick (odom 선수각을 step° 씩 바꿈). 마지막 선수각 반환."""
    for _ in range(int(deg / step)):
        yaw += math.copysign(math.radians(step), node.sent[-1].angular.z if node.sent else -1)
        _odom(node, 0.0, 4.0, yaw=yaw)
        _shapes(node, **shapes)
        node._tick()
    return yaw


def test_single_shape_direction_reverses_after_90deg_without_new_shape(node):
    """끝 슬롯(사각)만 보이고 그쪽엔 더 없다 — 90° 돌아도 새 도형이 없으면 한 번 반대로."""
    node.target = 'triangle'
    _scan(node)
    yaw = FACING_WALL
    _odom(node, 0.0, 4.0, yaw=yaw)
    _shapes(node, square=-3.0)
    node._tick()
    assert node.sent[-1].angular.z > 0                  # 왼쪽부터
    yaw = _turn_search(node, yaw, 80, square=-40.0)
    assert node.sent[-1].angular.z > 0                  # 아직 90° 전
    yaw = _turn_search(node, yaw, 20)
    assert node.sent[-1].angular.z < 0 and node.search_reversed
    assert 'reversed' in node.reports[-1]['detail']
    _turn_search(node, yaw, 200)                       # 다시는 안 뒤집는다
    assert node.sent[-1].angular.z < 0


def test_pair_rule_skips_side_already_swept_empty(node):
    """끝 슬롯(사각)만 보여 왼쪽 90° 를 훑었는데 없었다 → 뒤집어 돌다 사각 +1°·원 −39° 가 보이면
    간격 후보 +41°(왼쪽, 더 가까움)는 이미 비어 있던 쪽이라 빼고 −79°(오른쪽)로. 시뮬 c_sqend_tri."""
    node.target = 'triangle'
    _scan(node)
    yaw = FACING_WALL
    _odom(node, 0.0, 4.0, yaw=yaw)
    _shapes(node, square=-3.0)
    node._tick()
    yaw = _turn_search(node, yaw, 100)
    assert node.search_reversed and node.search_empty_side == 1
    _shapes(node, square=1.0, circle=-39.0)
    node._tick()
    assert node.search_rank == 2 and node.sent[-1].angular.z < 0


def test_new_shape_before_90deg_switches_to_pair_rule_without_reversal(node):
    node.target = 'triangle'
    _scan(node)
    yaw = FACING_WALL
    _odom(node, 0.0, 4.0, yaw=yaw)
    _shapes(node, circle=-3.0)
    node._tick()
    assert node.sent[-1].angular.z > 0                  # 왼쪽: 사각이 들어올 것
    yaw = _turn_search(node, yaw, 30, circle=-30.0)
    _shapes(node, square=36.0, circle=-35.0)           # 사각이 들어옴 → 삼각은 원 너머 오른쪽 밖
    node._tick()
    assert node.search_rank == 2 and node.sent[-1].angular.z < 0 and not node.search_reversed


def test_search_stops_when_yolo_status_is_stale(node):
    _odom(node, 0.0, 10.0)
    _scan(node)
    _shapes(node, square=-25.0)
    node._tick()
    assert node.sent[-1].angular.z != 0.0
    node.det_received -= 10.0                          # 검출 노드가 멈춤 — 눈먼 회전·후진 안 함
    node._tick()
    assert node.sent[-1].angular.z == 0.0 and node.sent[-1].linear.x == 0.0
    assert node.reports[-1]['detail'] == 'YOLO stale'


def _oblique_setup(node, x=0.0, y=4.0, bearing=-math.pi / 4):
    """가운데 슬롯(x=0) 정면 y m 에서 오른쪽 옆 슬롯(x=4) 표지 쪽(−45°) 을 목표로 SEEK."""
    node.slot_pitch = PITCH                            # 시험 도크 간격 (실선 yaml 은 bag 추정 3.55)
    _lock_bearing(node)
    _odom(node, x, y, yaw=bearing)
    node.locked_bearing, node.seek_line_point = bearing, np.array([x, y])
    node._add_cloud(300.0, _cloud((x, y)))


def test_oblique_target_repositions_before_moving(node):
    """보이는 가운데 ㄷ자 법선과 목표 방위가 45° (>= 37°) — 다가가기 전에 바로 우회."""
    _oblique_setup(node)
    _scan(node)                                        # 앞은 비어 있음 (막힘 아님)
    node._tick()
    assert node.phase == 'REPOSITION' and node.sent[-1].linear.x == 0.0
    assert 'deg off the visible U slot axis' in node.reports[-1]['detail']
    rp = node.reposition
    assert rp['steps'] == -1
    assert rp['target'] == pytest.approx([4.0, 0.0], abs=.2)
    assert rp['waypoint'] == pytest.approx([4.0, node.reposition_distance], abs=.2)
    assert rp['heading'] == pytest.approx(FACING_WALL, abs=math.radians(3))


def test_target_slot_itself_visible_does_not_reposition(node):
    """보이는 ㄷ자가 바로 목표 슬롯이면(0칸) 비스듬해도 우회하지 않는다 — SEEK 가 그대로 잠근다."""
    _oblique_setup(node, x=2.6, y=4.0, bearing=math.atan2(-4.0, -2.6))   # 가운데 슬롯이 목표 (−57°)
    _scan(node)
    node._tick()
    assert node.phase in ('SEEK', 'TO_VIA', 'BACKOFF') and node.reposition is None


def test_blocked_fallback_repositions_when_angle_check_cannot_decide(node):
    """각도 판단이 안 될 때(여기선 끔) 예전처럼 앞이 3 s 막히면 우회. 우회는 한 번만."""
    _oblique_setup(node)
    node.reposition_angle = math.radians(89.0)         # 각도 판단 끔
    _scan(node, ahead_m=1.5)
    node._tick()
    assert node.phase == 'SEEK' and 'ahead — holding' in node.reports[-1]['detail']
    node.blocked_since -= 10.0
    node._tick()
    assert node.phase == 'REPOSITION'
    rp = node.reposition
    _scan(node)
    head = math.atan2(rp['waypoint'][1] - 4.0, rp['waypoint'][0])
    _odom(node, 0.0, 4.0, yaw=head)
    node._tick()
    node._tick()
    assert rp['stage'] == 'drive' and node.sent[-1].linear.x > 0.0
    node.cloud.clear()                                 # 목표 ㄷ자는 아직 못 봄 → 경유점까지
    _odom(node, *rp['waypoint'], yaw=head)
    node._tick()
    assert rp['stage'] == 'face' and node.sent[-1].linear.x == 0.0
    _odom(node, *rp['waypoint'], yaw=FACING_WALL)
    node._tick()
    assert node.phase == 'SCAN' and node.reposition_used
    assert 're-scanning' in node.reports[-1]['detail']


def test_target_u_locks_during_reposition_goes_straight_to_via(node):
    """우회 중 목표 ㄷ자(x=4)가 양쪽 면까지 보여 안정되면 경유점 전에 바로 TO_VIA."""
    _oblique_setup(node)
    _scan(node)
    node._tick()
    assert node.phase == 'REPOSITION'
    pos = (3.0, 4.6)                                   # 경유점 가는 길, 뒷벽 4.6 m (> 3.94)
    _odom(node, *pos, yaw=math.radians(40))
    for k in range(6):
        node._add_cloud(400.0 + .25 * k, _cloud(pos))
        node._tick()
        if node.phase != 'REPOSITION':
            break
    assert node.phase == 'TO_VIA' and node.reposition is None
    assert 'locked during reposition' in node.reports[-1]['detail']
    assert node.route_enu[2].tolist() == pytest.approx([4.0, 1.0], abs=.1)     # 목표 슬롯 정지점


def test_reposition_ignores_target_lock_closer_than_backoff_limit(node):
    """뒷벽 3.94 m 보다 가까이서 잠기면 받지 않는다 (BACKOFF 후진 방지) — 우회를 계속."""
    _oblique_setup(node)
    _scan(node)
    node._tick()
    pos = (3.6, 3.7)
    _odom(node, *pos, yaw=math.radians(40))
    for k in range(6):
        node._add_cloud(500.0 + .25 * k, _cloud(pos))
        node._tick()
    assert node.phase == 'REPOSITION'


def test_reposition_needs_a_complete_u_slot(node):
    """보이는 ㄷ자가 없으면 우회하지 않고 이유를 남긴 채 멈춰 기다린다."""
    _lock_bearing(node)
    _odom(node, 0.0, 4.0, yaw=-math.pi / 4)
    node.locked_bearing, node.seek_line_point = -math.pi / 4, np.array([0.0, 4.0])
    node._add_cloud(300.0, np.array([[1.0, 2.5], [1.1, 2.4], [1.2, 2.3]]))   # 칸막이 끝 몇 점뿐
    _scan(node, ahead_m=1.5)
    node._tick()
    node.blocked_since -= 10.0
    node._tick()
    assert node.phase == 'SEEK' and node.sent[-1].linear.x == 0.0
    assert 'no complete U slot in view' in node.reports[-1]['detail']
