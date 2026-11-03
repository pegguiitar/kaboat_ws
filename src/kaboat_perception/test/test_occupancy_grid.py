"""occupancy_grid — 라이다 log-odds 누적, rolling window, 기울기 게이트, 발행 형식."""
import math

import numpy as np
import pytest
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Header

from kaboat_perception.occupancy_grid import L_MAX, L_OCC, OccupancyGridNode

N_BEAMS = 720
INC = 2 * math.pi / N_BEAMS


class Spy:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture
def make_node():
    nodes = []

    def factory(*params):
        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        node = OccupancyGridNode()
        node.pub = Spy()
        nodes.append(node)
        return node
    yield factory
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _odom(x=0.0, y=0.0, yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def _scan(hits):
    """hits: [(선체 기준 방위 rad, 거리 m)] — 나머지 빔은 무반사(inf)."""
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_increment = INC
    msg.range_min, msg.range_max = 0.1, 25.0
    ranges = [math.inf] * N_BEAMS
    for bearing, r in hits:
        ranges[int(round((bearing + math.pi) / INC)) % N_BEAMS] = float(r)
    msg.ranges = ranges
    return msg


def _value(node, x, y):
    """마지막 발행 격자에서 월드 (x, y) 셀 값."""
    node._publish()
    g = node.pub.msgs[-1]
    ix = math.floor((x - g.info.origin.position.x) / g.info.resolution)
    iy = math.floor((y - g.info.origin.position.y) / g.info.resolution)
    if not (0 <= ix < g.info.width and 0 <= iy < g.info.height):
        return None
    return g.data[iy * g.info.width + ix]


def _prob(logodds):
    return int(round(100.0 / (1.0 + math.exp(-logodds))))


@pytest.mark.check('A')
def test_single_hit_marks_cell_and_clears_beam_path(make_node):
    node = make_node()
    node._on_odom(_odom())
    node._on_scan(_scan([(0.0, 3.0)]))
    assert _value(node, 3.1, 0.1) == _prob(L_OCC)       # 히트 1회 ≈ 69
    assert _value(node, 1.5, 0.1) < 50                   # 빔 경로는 free 쪽
    assert _value(node, 0.1, 3.1) == -1                  # 관측 안 된 칸


@pytest.mark.check('A')
def test_repeated_hits_saturate_above_dri_threshold(make_node):
    node = make_node()
    node._on_odom(_odom())
    for _ in range(8):
        node._on_scan(_scan([(0.0, 3.0)]))
    assert _value(node, 3.1, 0.1) == _prob(L_MAX)        # 상한 포화 ≈ 99
    assert _value(node, 3.1, 0.1) >= 95                  # DRI occ_threshold 95 이상


@pytest.mark.check('A')
def test_scan_is_rotated_by_boat_yaw(make_node):
    node = make_node()
    node._on_odom(_odom(yaw=math.pi / 2))                # 북쪽을 봄
    node._on_scan(_scan([(0.0, 3.0)]))                   # 선체 전방 3m = 월드 (0, 3)
    assert _value(node, 0.1, 3.1) == _prob(L_OCC)


@pytest.mark.check('B')
def test_neighbouring_beams_in_one_cell_count_once(make_node):
    node = make_node()
    node._on_odom(_odom())
    node._on_scan(_scan([(0.0, 3.05), (INC, 3.05), (-INC, 3.05)]))
    assert _value(node, 3.1, 0.0) == _prob(L_OCC)        # hit_dedup — 스캔당 셀당 1회


@pytest.mark.check('B')
def test_window_follows_boat_and_keeps_world_position(make_node):
    node = make_node()
    node._on_odom(_odom())
    for _ in range(5):
        node._on_scan(_scan([(0.0, 3.0)]))
    before = _value(node, 3.1, 0.1)
    node._on_odom(_odom(x=2.0, y=1.0))                   # 배가 움직여도
    assert _value(node, 3.1, 0.1) == before              # 장애물은 같은 월드 칸
    g = node.pub.msgs[-1]
    assert g.info.origin.position.x == pytest.approx(2.0 - 10.0, abs=0.2)
    node._on_odom(_odom(x=30.0))                         # window(20m) 밖으로
    assert _value(node, 3.1, 0.1) is None


@pytest.mark.check('B')
def test_resolution_and_size_parameters(make_node):
    node = make_node('resolution:=0.1', 'size:=10.0')
    node._on_odom(_odom())
    node._publish()
    g = node.pub.msgs[-1]
    assert (g.info.width, g.info.height, g.info.resolution) == (100, 100, pytest.approx(0.1))


@pytest.mark.check('C')
def test_tilted_scan_is_dropped(make_node):
    node = make_node()
    node._on_odom(_odom())
    imu = Imu()
    pitch = math.radians(5.0)
    imu.orientation.y, imu.orientation.w = math.sin(pitch / 2), math.cos(pitch / 2)
    node._on_imu(imu)
    node._on_scan(_scan([(0.0, 3.0)]))
    assert _value(node, 3.1, 0.1) == -1


@pytest.mark.check('B')
def test_tilt_gate_can_be_disabled_for_tank(make_node):
    off = make_node('enable_tilt_gate:=false')
    off._on_odom(_odom())
    off.tilted = True                                     # 게이트가 꺼지면 무시된다
    off._on_scan(_scan([(0.0, 3.0)]))
    assert _value(off, 3.1, 0.1) == _prob(L_OCC)


@pytest.mark.check('C')
def test_no_odom_ignores_scan_and_publishes_unknown(make_node):
    node = make_node()
    node._on_scan(_scan([(0.0, 3.0)]))
    node._publish()
    assert set(node.pub.msgs[-1].data) == {-1}


@pytest.mark.check('C')
def test_all_invalid_beams_change_nothing(make_node):
    node = make_node()
    node._on_odom(_odom())
    node._on_scan(_scan([]))
    assert not node.observed.any()


@pytest.mark.check('B')
def test_depth_points_only_inside_height_band_are_used(make_node):
    node = make_node('cam_stride:=1', 'cam_height:=0.5')
    node._on_odom(_odom())
    # 광학 프레임 (X 우측, Y 아래, Z 전방): 전방 3m, 수면 위 0.5m 와 2.5m
    in_band = (0.0, 0.0, 3.05)
    too_high = (0.0, -2.0, 4.05)
    cloud = pc2.create_cloud_xyz32(Header(frame_id='camera'), [in_band, too_high])
    node._on_points(cloud)
    assert _value(node, 3.1, 0.1) != -1
    assert _value(node, 4.1, 0.1) == -1


@pytest.mark.check('D')
def test_published_grid_is_odom_frame_snapped_to_cells(make_node):
    node = make_node()
    node._on_odom(_odom(x=1.23, y=-0.77))
    node._publish()
    g = node.pub.msgs[-1]
    assert g.header.frame_id == 'odom'
    assert len(g.data) == g.info.width * g.info.height == 100 * 100
    for v in (g.info.origin.position.x, g.info.origin.position.y):
        assert v / 0.2 == pytest.approx(round(v / 0.2))
    assert np.int8(max(g.data)) <= 100
