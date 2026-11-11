"""gq7_local_odom — GQ7 필터 위치(ECEF)를 우리 원점 기준 /odom 으로 바꿔 발행 (야외).

  /ekf/odometry_earth (GQ7 칼만필터, ECEF 위치 + ECEF 기준 자세)
        │  원점 파일 ~/kaboat_geo_origin.yaml (geo_origin /geo/set_origin 이 저장)
        ▼
  /odom (frame odom, child base_link) — x = 동쪽, y = 북쪽, z = 위 [m], 자세도 같은 축(ENU)

왜 직접 바꾸나: GQ7 드라이버의 고정 원점 기능(filter_relative_position_source 1)이 젯슨에
설치된 드라이버에서 원점을 맞게 받고도 수백 km 떨어진 /ekf/odometry_map 을 냈다
(2026-11-11 현장). 드라이버 자동 원점(처음 Full Nav 위치)은 켤 때마다 바뀐다. 그래서
드라이버의 원점 기능은 쓰지 않고, 이미 시험된 geo_local 변환으로 여기서 계산한다.

위치 출처 (position_source, real_sensors 야외 기본 filter):
  filter — GQ7 필터 위치(odometry_earth). 매끄럽고 25 Hz. 2026-11-11 현장: 드라이버 4.9.0·SBAS
           에서는 멈춘 배 위치가 수 m 흘렀지만, 4.10.0 + RTK Fixed 에서는 cm 단위로 안정.
  gnss   — GNSS1 원시 위치(/gps/fix, 2 Hz, SBAS ±1~2 m)를 안테나 오프셋만큼 선체 중심으로
           옮겨 쓴다. 선수각·회전속도는 GQ7 필터(듀얼 안테나 헤딩), 선속도는 GNSS 속도
           (/gnss_1/velocity, ENU)를 선체 기준으로 돌려 쓴다. GNSS 가 max_fix_age_sec 넘게
           끊기면 /odom 을 내지 않는다(behavior 들이 odom 타임아웃으로 멈추게).
           위치는 다음 GNSS 가 올 때까지(0.5 s) 그대로라 계단처럼 움직인다.

- 원점 파일이 바뀌면(mtime) 1초 안에 다시 읽어 바로 반영 — /geo/set_origin 뒤 GQ7 을
  재시작할 필요가 없다(재시작하면 필터가 다시 수렴하느라 몇 분 걸린다).
- 원점 파일이 없으면 auto_origin=true(기본) 일 때 처음 받은 위치를 원점으로 쓰고 경고한다
  (켤 때마다 바뀜 — rec 로 지점을 기록하기 전에 원점을 잡을 것).
- 속도(twist)는 GQ7 이 선체 기준으로 내므로 그대로, 위치·자세 공분산은 ENU 로 회전한다.
- GQ7 은 선체 중심에 있어(gq7_outdoor.yaml filter_lever_arm_offset 0) 위치 = base_link.
"""

import math
import os
import time

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus

from kaboat_hardware.geo_local import LocalFrame, ecef_to_llh
from kaboat_mission.mission_points import DEFAULT_ORIGIN_FILE, read_origin_file


# ── 순수 계산 (ROS 무의존) ──────────────────────────────
def matrix_to_quat(m):
    """3x3 회전행렬 → 쿼터니언 (x, y, z, w)."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s,
                (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s,
                (m[0][2] - m[2][0]) / s)
    s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s,
            (m[1][0] - m[0][1]) / s)


def quat_mul(a, b):
    """쿼터니언 곱 a ⊗ b, (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate_covariance6(rot, cov):
    """6x6 공분산(행 우선 36개)의 위치·회전 블록을 3x3 회전 rot 으로 돌린다: B C Bᵀ."""
    b = [[0.0] * 6 for _ in range(6)]
    for i in range(3):
        for j in range(3):
            b[i][j] = b[i + 3][j + 3] = rot[i][j]
    c = [list(cov[6 * i:6 * i + 6]) for i in range(6)]
    bc = [[sum(b[i][k] * c[k][j] for k in range(6)) for j in range(6)] for i in range(6)]
    out = [[sum(bc[i][k] * b[j][k] for k in range(6)) for j in range(6)] for i in range(6)]
    return [out[i][j] for i in range(6) for j in range(6)]


def earth_to_local(frame: LocalFrame, position_ecef, quat_ecef, covariance=None):
    """GQ7 earth(ECEF) 자세·위치 → 원점 기준 ENU.

    quat_ecef 는 선체 → ECEF 회전. 결과 자세는 선체 → ENU(원점의 동·북·위) 회전.
    경기장 크기(수백 m)에서는 원점과 배 위치의 ENU 축 차이가 0.01° 미만이라 무시한다.
    """
    rot = frame.ecef_rotation()
    enu = frame.ecef_to_enu(*position_ecef)
    quat = quat_mul(matrix_to_quat(rot), quat_ecef)
    norm = math.sqrt(sum(v * v for v in quat))
    quat = tuple(v / norm for v in quat)
    cov = None if covariance is None else rotate_covariance6(rot, covariance)
    return enu, quat, cov


def yaw_of(quat) -> float:
    """쿼터니언 (x, y, z, w) → ENU 선수각 [rad]."""
    x, y, z, w = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def antenna_to_center(antenna_enu, yaw: float, offset_flu):
    """안테나 ENU 위치 → 선체 중심. offset_flu = 선체 중심 → 안테나 (선수, 좌현) [m]."""
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = offset_flu
    return (antenna_enu[0] - (c * ox - s * oy),
            antenna_enu[1] - (s * ox + c * oy),
            antenna_enu[2])


def enu_velocity_to_body(v_enu, yaw: float):
    """ENU 속도 → 선체 기준 (선수, 좌현, 위) 속도. 롤·피치는 작다고 보고 선수각만 돌린다."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v_enu[0] + s * v_enu[1], -s * v_enu[0] + c * v_enu[1], v_enu[2])


# ── 노드 ─────────────────────────────────────────────
class Gq7LocalOdom(Node):
    def __init__(self, **node_kwargs):
        super().__init__('gq7_local_odom', **node_kwargs)
        self.declare_parameter('origin_file', DEFAULT_ORIGIN_FILE)
        self.declare_parameter('input_topic', '/ekf/odometry_earth')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        # 원점 파일이 없을 때 처음 받은 위치를 원점으로 쓸지 (켤 때마다 바뀜)
        self.declare_parameter('auto_origin', True)
        # 위치 출처: 'filter'(GQ7 필터) 또는 'gnss'(GNSS1 원시 + 필터 선수각) — 모듈 설명 참고
        self.declare_parameter('position_source', 'filter')
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('gnss_velocity_topic', '/gnss_1/velocity')
        # 선체 중심(base_link) → GNSS1 안테나 (선수, 좌현) [m]. gq7_outdoor.yaml 의
        # gnss1_antenna_offset [0, -0.4, 0](FRD: 우현 +) 과 같은 위치 = 좌현 0.4 m.
        self.declare_parameter('gnss_antenna_offset', [0.0, 0.4])
        self.declare_parameter('max_fix_age_sec', 1.0)
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.origin_file = os.path.expanduser(str(g('origin_file')))
        self.frame_id = str(g('frame_id'))
        self.child_frame_id = str(g('child_frame_id'))
        self.auto_origin = bool(g('auto_origin'))
        self.position_source = str(g('position_source')).strip().lower()
        if self.position_source not in ('filter', 'gnss'):
            raise ValueError("position_source must be 'filter' or 'gnss'")
        self.antenna_offset = tuple(float(v) for v in g('gnss_antenna_offset'))
        self.max_fix_age = float(g('max_fix_age_sec'))
        if len(self.antenna_offset) != 2 or not self.max_fix_age > 0:
            raise ValueError('gnss_antenna_offset must be [forward, left], max_fix_age_sec > 0')
        self.fix = None            # (monotonic, NavSatFix)
        self.gnss_vel = None       # (monotonic, (ve, vn, vu))

        self.frame: LocalFrame | None = None
        self.origin_source = ''
        self._origin_mtime = None
        self._warned_frame = False

        self.pub = self.create_publisher(Odometry, str(g('output_topic')), 10)
        self.create_subscription(Odometry, str(g('input_topic')), self._on_earth,
                                 qos_profile_sensor_data)
        if self.position_source == 'gnss':
            self.create_subscription(NavSatFix, str(g('gnss_topic')), self._on_fix,
                                     qos_profile_sensor_data)
            self.create_subscription(TwistWithCovarianceStamped, str(g('gnss_velocity_topic')),
                                     self._on_gnss_vel, qos_profile_sensor_data)
        self.get_logger().info(
            f'/odom 위치 출처 = {self.position_source}'
            + (' (GNSS1 원시 위치 + GQ7 선수각, 안테나 오프셋 '
               f'{self.antenna_offset} m 보정)' if self.position_source == 'gnss'
               else ' (GQ7 필터 위치)'))
        self.create_timer(1.0, self._check_origin_file)
        self._check_origin_file()
        if self.frame is None:
            self.get_logger().warn(
                f'원점 파일 없음 ({self.origin_file}) — '
                + ('처음 받은 GQ7 위치를 원점으로 씀(켤 때마다 바뀜). ' if self.auto_origin
                   else '원점이 생길 때까지 /odom 을 내지 않음. ')
                + "geo_origin 으로 `ros2 service call /geo/set_origin std_srvs/srv/Trigger '{}'`")

    def _set_origin(self, origin, source):
        old = self.frame
        self.frame = LocalFrame(*origin)
        self.origin_source = source
        msg = (f'/odom 원점 = ({origin[0]:.8f}, {origin[1]:.8f}, {origin[2]:.2f} m) [{source}]')
        if old is not None:
            self.get_logger().warn('원점이 바뀜 — ' + msg)
        else:
            self.get_logger().info(msg)

    def _check_origin_file(self):
        """원점 파일이 새로 생기거나 바뀌었으면 다시 읽는다."""
        try:
            mtime = os.path.getmtime(self.origin_file)
        except OSError:
            return            # 파일 없음 — 지금 원점 유지
        if mtime == self._origin_mtime:
            return
        self._origin_mtime = mtime
        try:
            origin = read_origin_file(self.origin_file)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'원점 파일을 읽지 못함 ({self.origin_file}): {exc}')
            return
        if origin is not None:
            self._set_origin(origin, f'file {self.origin_file}')

    def _on_fix(self, msg: NavSatFix):
        if (msg.status.status >= NavSatStatus.STATUS_FIX
                and all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude))):
            self.fix = (time.monotonic(), msg)

    def _on_gnss_vel(self, msg: TwistWithCovarianceStamped):
        v = msg.twist.twist.linear
        if all(math.isfinite(c) for c in (v.x, v.y, v.z)):
            self.gnss_vel = (time.monotonic(), (v.x, v.y, v.z))

    def _gnss_pose(self, quat, filter_cov):
        """GNSS1 위치를 선체 중심으로 옮긴 (enu, cov). GNSS 가 오래됐으면 None."""
        now = time.monotonic()
        if self.fix is None or now - self.fix[0] > self.max_fix_age:
            self.get_logger().warn(
                f'GNSS({self.max_fix_age:.1f} s 이내) 없음 — /odom 발행 멈춤',
                throttle_duration_sec=2.0)
            return None
        fix = self.fix[1]
        antenna = self.frame.to_enu(fix.latitude, fix.longitude, fix.altitude)
        enu = antenna_to_center(antenna, yaw_of(quat), self.antenna_offset)
        cov = list(filter_cov)
        if fix.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            for i in range(3):
                for j in range(3):
                    cov[6 * i + j] = fix.position_covariance[3 * i + j]
        return enu, cov

    def _on_earth(self, msg: Odometry):
        if msg.header.frame_id not in ('earth', '') and not self._warned_frame:
            self._warned_frame = True
            self.get_logger().warn(
                f"{msg.header.frame_id!r} 프레임 — GQ7 odometry_earth(ECEF, 'earth') 가 아닐 수 있음")
        p = msg.pose.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or (p.x, p.y, p.z) == (0.0, 0.0, 0.0):
            return            # 필터가 아직 위치를 못 냄
        if self.frame is None:
            if not self.auto_origin:
                return
            self._set_origin(ecef_to_llh(p.x, p.y, p.z), '처음 받은 GQ7 위치 (켤 때마다 바뀜)')

        q = msg.pose.pose.orientation
        enu, quat, cov = earth_to_local(
            self.frame, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), list(msg.pose.covariance))
        twist = msg.twist                  # GQ7 은 선체 기준 속도
        if self.position_source == 'gnss':
            pose = self._gnss_pose(quat, cov)
            if pose is None:
                return
            enu, cov = pose
            if self.gnss_vel is not None and time.monotonic() - self.gnss_vel[0] <= self.max_fix_age:
                twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = (
                    enu_velocity_to_body(self.gnss_vel[1], yaw_of(quat)))
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = enu
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = quat
        out.pose.covariance = cov
        out.twist = twist
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Gq7LocalOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
