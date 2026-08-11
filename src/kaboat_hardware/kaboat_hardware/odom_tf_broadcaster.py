"""실물용 최소 TF 브로드캐스터 — RViz 시각화를 위한 odom→base_link→센서 체인.

왜 필요한가:
  자율주행 스택 자체는 TF를 안 쓴다. occupancy_grid 는 /odom 메시지에서
  직접 배 위치를 읽고, lidar_link 를 base_link 와 동일하다고 가정한다
  (occupancy_grid.py docstring 참고). 그래서 sim 에서는 Gazebo 가 TF 를
  발행해 RViz 가 그려졌을 뿐, 실물에는 TF 발행자가 하나도 없다
  (gq7.yaml 의 tf_mode: 0, robot_state_publisher 미실행).

  TF 가 없으면 RViz 는 Fixed Frame 을 해석하지 못해
  ``Fixed Frame [odom] does not exist`` 로 Map·LaserScan 을 통째로 못 그린다.
  이 노드는 그 최소 체인만 채운다 — 제어 경로에는 전혀 관여하지 않는다.

발행:
  odom → base_link   (동적) — /odom 메시지의 pose 를 그대로 TF 로 중계
  base_link → <lidar_frame>  (정적) — 기본 identity
  base_link → <camera_frame> (정적) — 기본 identity

  ⚠️ lidar 오프셋 기본값이 0 인 것은 occupancy_grid 의 "lidar == base_link"
  가정과 **일부러 일치시킨 것**이다. 실측 오프셋을 여기에만 넣으면 RViz 의
  스캔 위치와 격자가 어긋나 보인다 — 실물 장착 오프셋을 반영할 때는
  occupancy_grid 의 좌표 변환도 함께 고쳐야 한다.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _static_tf(parent: str, child: str, xyz, rpy, stamp) -> TransformStamped:
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = float(xyz[0])
    tf.transform.translation.y = float(xyz[1])
    tf.transform.translation.z = float(xyz[2])

    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    tf.transform.rotation.w = cr * cp * cy + sr * sp * sy
    tf.transform.rotation.x = sr * cp * cy - cr * sp * sy
    tf.transform.rotation.y = cr * sp * cy + sr * cp * sy
    tf.transform.rotation.z = cr * cp * sy - sr * sp * cy
    return tf


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'laser_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        # 기본 identity — occupancy_grid 의 lidar==base_link 가정과 일치 (docstring)
        self.declare_parameter('lidar_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('lidar_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('camera_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('camera_rpy', [0.0, 0.0, 0.0])
        # /odom 이 아직 없을 때(예: GQ7 EKF 미출력) 배를 원점에 고정해 두면
        # 정지 벤치 점검에서 스캔·격자를 바로 볼 수 있다.
        self.declare_parameter('publish_identity_until_odom', True)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.fallback = bool(
            self.get_parameter('publish_identity_until_odom').value)

        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        stamp = self.get_clock().now().to_msg()
        statics = [
            _static_tf(self.base_frame,
                       str(self.get_parameter('lidar_frame').value),
                       self.get_parameter('lidar_xyz').value,
                       self.get_parameter('lidar_rpy').value, stamp),
            _static_tf(self.base_frame,
                       str(self.get_parameter('camera_frame').value),
                       self.get_parameter('camera_xyz').value,
                       self.get_parameter('camera_rpy').value, stamp),
        ]
        self.static_broadcaster.sendTransform(statics)

        self.odom_seen = False
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_timer(0.1, self._fallback_tick)   # 10Hz

        self.get_logger().info(
            f'odom_tf_broadcaster 시작 — {self.odom_frame} → {self.base_frame} '
            f'(정적: {statics[0].child_frame_id}, {statics[1].child_frame_id})')

    def _on_odom(self, msg: Odometry):
        if not self.odom_seen:
            self.odom_seen = True
            self.get_logger().info(
                f"/odom 수신 시작 (frame_id='{msg.header.frame_id}') — "
                '실제 pose 로 TF 발행')

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(tf)

    def _fallback_tick(self):
        """/odom 이 아직 없으면 배를 원점에 둔 identity TF 를 대신 발행한다."""
        if self.odom_seen or not self.fallback:
            return
        stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(
            _static_tf(self.odom_frame, self.base_frame,
                       (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), stamp))


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
