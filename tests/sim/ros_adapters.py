"""가짜 수역 데이터 → ROS 메시지 변환과 시험용 가짜 시계.

노드 시험은 spin 하지 않고 콜백을 직접 호출하므로, 시간도 시험이 직접 넘긴다.
behavior 는 time.monotonic() 과 node.get_clock() 두 가지를 섞어 쓰기 때문에
FakeClock.install() 이 둘을 같은 시뮬레이션 시간으로 맞춘다.
"""
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from .boat_model import BoatState
from .fake_sensors import ScanConfig


class FakeClock:
    """시뮬레이션 시간 [s]. advance() 로만 흐른다."""

    def __init__(self, start: float = 1000.0):
        # 0 에서 시작하면 'now - 2.0' 같은 오래된 시각 계산이 음수가 되어
        # 실제 실행과 다른 분기를 탈 수 있어서 여유를 두고 시작한다.
        self.now = float(start)

    def advance(self, dt: float) -> float:
        self.now += float(dt)
        return self.now

    def monotonic(self) -> float:
        return self.now

    def install(self, monkeypatch, *nodes):
        """time.monotonic 과 각 node.get_clock() 을 이 시계로 바꾼다 (시험 종료 시 복구)."""
        monkeypatch.setattr(time, 'monotonic', self.monotonic)
        for node in nodes:
            monkeypatch.setattr(node, 'get_clock', lambda: _RosClock(self))


class _RosClock:
    def __init__(self, clock: FakeClock):
        self._clock = clock

    def now(self) -> Time:
        return Time(nanoseconds=int(self._clock.now * 1e9))


def _yaw_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def odom_msg(state: BoatState, frame_id: str = 'odom') -> Odometry:
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = float(state.x)
    msg.pose.pose.position.y = float(state.y)
    q = _yaw_quaternion(state.yaw)
    o = msg.pose.pose.orientation
    o.x, o.y, o.z, o.w = q
    msg.twist.twist.linear.x = float(state.u)
    msg.twist.twist.linear.y = float(state.v)
    msg.twist.twist.angular.z = float(state.r)
    return msg


def pose_msg(x: float, y: float, yaw: float = 0.0, frame_id: str = 'odom') -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    o = msg.pose.orientation
    o.x, o.y, o.z, o.w = _yaw_quaternion(yaw)
    return msg


def scan_msg(ranges, cfg: ScanConfig = ScanConfig(), frame_id: str = 'base_link') -> LaserScan:
    msg = LaserScan()
    msg.header.frame_id = frame_id
    msg.angle_min = float(cfg.angle_min)
    msg.angle_increment = float(cfg.angle_increment)
    msg.angle_max = float(cfg.angle_min + cfg.angle_increment * (cfg.beams - 1))
    msg.range_min = float(cfg.range_min)
    msg.range_max = float(cfg.range_max)
    msg.ranges = [float(r) for r in ranges]
    return msg


def grid_msg(grid, frame_id: str = 'odom') -> OccupancyGrid:
    """fake_sensors.occupancy_grid() 결과 → nav_msgs/OccupancyGrid."""
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.info.resolution = float(grid.info.resolution)
    msg.info.width = int(grid.info.width)
    msg.info.height = int(grid.info.height)
    msg.info.origin.position.x = float(grid.info.origin.position.x)
    msg.info.origin.position.y = float(grid.info.origin.position.y)
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(v) for v in grid.data]
    return msg
