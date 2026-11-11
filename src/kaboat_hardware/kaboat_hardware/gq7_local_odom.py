"""gq7_local_odom — GQ7 필터 위치(ECEF)를 우리 원점 기준 /odom 으로 바꿔 발행 (야외).

  /ekf/odometry_earth (GQ7 칼만필터, ECEF 위치 + ECEF 기준 자세)
        │  원점 파일 ~/kaboat_geo_origin.yaml (geo_origin /geo/set_origin 이 저장)
        ▼
  /odom (frame odom, child base_link) — x = 동쪽, y = 북쪽, z = 위 [m], 자세도 같은 축(ENU)

왜 직접 바꾸나: GQ7 드라이버의 고정 원점 기능(filter_relative_position_source 1)이 젯슨에
설치된 드라이버에서 원점을 맞게 받고도 수백 km 떨어진 /ekf/odometry_map 을 냈다
(2026-11-11 현장). 드라이버 자동 원점(처음 Full Nav 위치)은 켤 때마다 바뀐다. 그래서
드라이버의 원점 기능은 쓰지 않고, 이미 시험된 geo_local 변환으로 여기서 계산한다.

- 원점 파일이 바뀌면(mtime) 1초 안에 다시 읽어 바로 반영 — /geo/set_origin 뒤 GQ7 을
  재시작할 필요가 없다(재시작하면 필터가 다시 수렴하느라 몇 분 걸린다).
- 원점 파일이 없으면 auto_origin=true(기본) 일 때 처음 받은 위치를 원점으로 쓰고 경고한다
  (켤 때마다 바뀜 — rec 로 지점을 기록하기 전에 원점을 잡을 것).
- 속도(twist)는 GQ7 이 선체 기준으로 내므로 그대로, 위치·자세 공분산은 ENU 로 회전한다.
- GQ7 은 선체 중심에 있어(gq7_outdoor.yaml filter_lever_arm_offset 0) 위치 = base_link.
"""

import math
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.origin_file = os.path.expanduser(str(g('origin_file')))
        self.frame_id = str(g('frame_id'))
        self.child_frame_id = str(g('child_frame_id'))
        self.auto_origin = bool(g('auto_origin'))

        self.frame: LocalFrame | None = None
        self.origin_source = ''
        self._origin_mtime = None
        self._warned_frame = False

        self.pub = self.create_publisher(Odometry, str(g('output_topic')), 10)
        self.create_subscription(Odometry, str(g('input_topic')), self._on_earth,
                                 qos_profile_sensor_data)
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
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = enu
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = quat
        out.pose.covariance = cov
        out.twist = msg.twist           # GQ7 은 선체 기준 속도 — 그대로
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
