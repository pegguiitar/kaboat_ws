"""occupancy_grid — 장애물 격자 지도 (상시 가동, ① 레이어)

입력   /scan (sensor_msgs/LaserScan)                  — 주 소스
       /camera/depth/points (sensor_msgs/PointCloud2) — 보조 융합 (z-band 필터)
       /imu/data (sensor_msgs/Imu)                    — 기울기 게이팅
       /odom (nav_msgs/Odometry)                      — rolling window 중심
출력   /occupancy_grid (nav_msgs/OccupancyGrid)

node_structure v5 의 occupancy_grid. 배 주변 rolling window 2D 격자에
장애물 점유 확률(log-odds)을 누적해, obstacle_planner(DRI/VFH)와 전
behavior 의 회피가 공유하는 지도를 만든다. v5 설계상 장애물 표현은
이 노드가 단독 담당한다 (obstacle_detector 의 /obstacles 는 골격 단계의
과도기 인터페이스 — 이 노드가 채워지면 대체 예정).

skeleton 구현 — **인터페이스만**:
  구독·발행·파라미터·frame 규약은 전부 잡혀 있고, 지도 내용은 전부
  unknown(-1) 인 빈 격자를 odom 중심 rolling window 로 1Hz 발행한다.
  RViz 에서 Map 디스플레이로 위치·크기 규약을 바로 확인할 수 있다.

TODO(팀):
  1) /scan 레이캐스팅 → log-odds 업데이트 (hit +l_occ, 빔 경로 -l_free)
  2) /camera/depth/points 융합 — z-band(수면 위 h_min~h_max) 필터 후 투영
  3) /imu/data 롤/피치가 임계 초과인 프레임은 업데이트 스킵 (기울기 게이팅
     — 배가 기울면 2D LiDAR 가 수면/하늘을 훑어 유령 장애물이 생긴다)
  4) rolling window 이동 시 기존 셀 재배치(shift) 및 노후 셀 감쇠(decay)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

        self.declare_parameter('resolution', 0.2)   # 셀 크기 [m]
        self.declare_parameter('size', 40.0)        # window 한 변 길이 [m]
        self.declare_parameter('publish_rate', 1.0)  # 발행 주기 [Hz]
        # TODO(팀): l_occ/l_free/decay, z-band(h_min/h_max), 기울기 임계 파라미터 추가

        self.resolution = float(self.get_parameter('resolution').value)
        self.size = float(self.get_parameter('size').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.cells = int(self.size / self.resolution)

        self.odom = None
        self.scan = None
        self.imu = None

        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        # TODO(팀): 2) 구현 시 /camera/depth/points 구독 활성화
        # self.create_subscription(PointCloud2, '/camera/depth/points', self._on_points, 5)

        self.pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 1)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'occupancy_grid 시작 (인터페이스 골격 — 빈 격자 발행) — '
            f'{self.cells}×{self.cells}셀 @ {self.resolution}m → /occupancy_grid')

    def _on_scan(self, msg: LaserScan):
        self.scan = msg
        # TODO(팀): 1) 여기서 레이캐스팅 → log-odds 누적

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_imu(self, msg: Imu):
        self.imu = msg
        # TODO(팀): 3) 롤/피치 임계 초과 시 업데이트 게이팅 플래그 설정

    def _publish(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        grid.info.resolution = self.resolution
        grid.info.width = self.cells
        grid.info.height = self.cells
        # rolling window — 배(odom) 를 중심에 두고 origin 을 좌하단으로
        if self.odom is not None:
            grid.info.origin.position.x = \
                self.odom.pose.pose.position.x - self.size / 2.0
            grid.info.origin.position.y = \
                self.odom.pose.pose.position.y - self.size / 2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [-1] * (self.cells * self.cells)   # TODO(팀): log-odds → 0~100
        self.pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
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
