"""occupancy_grid — 장애물 격자 지도 (상시 가동, ① 레이어)

입력   /scan (sensor_msgs/LaserScan)                  — 주 소스
       /camera/depth/points (sensor_msgs/PointCloud2) — 보조 융합 (z-band 필터)
       /imu/data (sensor_msgs/Imu)                    — 기울기 게이팅
       /odom (nav_msgs/Odometry)                      — rolling window 중심
출력   /occupancy_grid (nav_msgs/OccupancyGrid)

node_structure v5 의 occupancy_grid. 배 주변 rolling window 2D 격자에
장애물 점유 확률(log-odds)을 누적해, obstacle_planner(DRI/VFH)와 전
behavior 의 회피가 공유하는 지도를 만든다. v5 설계상 장애물 표현은
이 노드가 단독 담당한다 (구 obstacle_detector/ObstacleArray 방식은 골격
단계의 과도기 인터페이스였고, 이 노드가 LiDAR+뎁스캠으로 채워지면서
대체돼 삭제됐다).

핵심 동작 (요약):
  1) 스캔/포인트가 들어올 때마다 거리·방향 상관없이 현재 odom 기준 월드
     좌표를 계산해 self.log_odds(dict) 에 log-odds 로 계속 누적 저장한다.
  2) 배가 이동해도 dict 안의 값은 손대지 않는다 — 전부 월드 절대좌표에
     고정된 채 그대로 남는다 (그래서 shift 로 재배치할 필요가 없다).
  3) publish 시점(1Hz)마다, 그 순간 배 위치를 중심으로 한 size×size[m]
     범위 안의 값만 dict 에서 조회해 /occupancy_grid 로 발행한다.
  4) 같은 시점에 그 범위를 훌쩍 벗어난 옛 항목은 prune 으로 삭제해 dict
     가 무한정 커지지 않게 한다.
  즉 "장애물 정보는 세상 전체 기준으로 무제한 기억하되, 실제로 내보내는
  지도는 배 근처 size×size[m]만 잘라서 보여준다".

구현 — /scan 레이캐스팅(Bresenham)으로 log-odds 갱신:
  각 빔의 경로 셀은 L_FREE 로 감쇠, 유효 히트 끝점 셀은 L_OCC 로 누적.
  lidar와 base_link 는 동일 위치/자세로 가정(TF 미사용, behavior 쪽과 동일
  단순화) — 실물 전환 시 lidar 장착 오프셋 보정 필요.
  좌표는 월드 절대 격자(floor(x/resolution))에 dict 로 희소 저장하므로
  rolling window 재중심 시 배열 shift 가 필요 없다 — 대신 publish 마다
  현재 창에서 크게 벗어난 키를 정리(prune)해 메모리를 유계로 유지한다.
  IMU 롤/피치가 TILT_MAX 를 넘으면 그 스캔은 통째로 버린다(배가 기울면
  2D LiDAR 가 수면/하늘을 훑어 유령 장애물이 생기기 때문).

  뎁스캠(/camera/depth/points)은 LiDAR 를 대체하는 게 아니라 사각지대
  보완용 — LiDAR 수평 한 평면이 놓치는 물체를 z-band(수면 위 cam_z_min~
  cam_z_max) + 거리(CAM_RANGE_MIN~MAX) 필터로 걸러 같은 log_odds dict 에
  L_OCC_CAM 만큼 더한다. 자유공간 레이캐스팅은 안 함(점유 증거만 추가).
  두 센서가 같은 셀을 같이 보면 log-odds 가 더 누적돼 더 확신 있는(짙은)
  셀이 되는 식으로 자연히 "융합"된다 — 별도 융합 로직 없이 같은 누적
  장부를 공유할 뿐. 프레임당 90만+ 포인트라 cam_stride 로 다운샘플한다.
  거리 필터가 중요한 이유: sim 의 depth clip 이 0.1~100m 라 그 안은 전부
  "유효값"으로 나오지만, 실물 D455 는 ≈0.6~6m 밖에서 부정확 — 안 거르면
  카메라 근접 노이즈(배 자신의 구조물 포함)나 원거리 배경이 장애물로
  오인된다 (wamv_kaboat.xacro 의 depth_camera 센서 주석 참고).

TODO(팀):
  1) decay(노후 셀 확률의 점진적 감쇠) — 현재는 prune(완전 삭제)만 있음
  2) inverse sensor model 을 거리/각도 함수로 — 지금 L_OCC/L_FREE 는 거리·빔
     중심으로부터의 각도와 무관한 고정 상수(원래는 가까울수록/빔 중심에
     가까울수록 더 신뢰해야 함). 정확도보단 단순함을 택한 skeleton 근사.
     -> 일단 실물 테스트에서 log odds ratio는 상수로 하고 정확도가 안나오면 다른 알고리즘 모색
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2

L_OCC = 0.85              # 히트 셀 log-odds 증가량 (≈ logit(0.7))
L_FREE = -0.4             # 경로(자유공간) 셀 log-odds 감소량 (≈ logit(0.4))
# 둘 다 거리/빔각도 무관 고정값 — 진짜 inverse sensor model 이면 거리·빔중심
# 이탈각의 함수여야 하는데, 그러면 복잡해지니 skeleton 은 상수로 근사 (TODO 2)
L_OCC_CAM = 0.5            # 뎁스캠 히트 log-odds 증가량 — 실외 IR 뎁스는 LiDAR
                           # 보다 신뢰도가 낮다고 보고 L_OCC(0.85)보다 약하게 잡음
CAM_RANGE_MIN, CAM_RANGE_MAX = 0.6, 6.0  # 실물 D455 유효 뎁스 범위 [m]
# sim 의 clip(0.1~100m)은 그 밖도 "유효값"으로 내보내지만(wamv_kaboat.xacro
# 주석 참고) 실물은 그 범위 밖에서 부정확 — 여기서 걸러야 함 (안 그러면
# 카메라 근접 노이즈/배 자신의 구조물, 원거리 오탐이 전부 장애물로 잡힘)
L_MIN, L_MAX = -2.0, 3.5  # log-odds 클램프 (과적분 방지)
TILT_MAX = math.radians(15.0)  # 이 이상 기울면 해당 스캔 버림


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _roll_pitch(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    return roll, pitch


def _bresenham(x0, y0, x1, y1):
    """(x0,y0) → (x1,y1) 격자 셀 인덱스 목록 (끝점 포함)."""
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    cells = []
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return cells


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

        self.declare_parameter('resolution', 0.2)   # 셀 크기 [m]
        self.declare_parameter('size', 20.0)        # window 한 변 길이 [m]
        self.declare_parameter('publish_rate', 1.0)  # 발행 주기 [Hz]
        self.declare_parameter('cam_stride', 1000)   # 뎁스 포인트 다운샘플 간격
        self.declare_parameter('cam_height', 0.5)    # 카메라 장착 높이(수면 기준) [m]
        self.declare_parameter('cam_z_min', 0.1)     # z-band 하한 — 수면 근처 제외 [m]
        self.declare_parameter('cam_z_max', 2.0)     # z-band 상한 — 상부구조물/하늘 제외 [m]

        self.resolution = float(self.get_parameter('resolution').value)
        self.size = float(self.get_parameter('size').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.cells = int(self.size / self.resolution)
        self.cam_stride = int(self.get_parameter('cam_stride').value)
        self.cam_height = float(self.get_parameter('cam_height').value)
        self.cam_z_min = float(self.get_parameter('cam_z_min').value)
        self.cam_z_max = float(self.get_parameter('cam_z_max').value)

        self.odom = None
        self.tilted = False
        self.log_odds = {}  # (world_ix, world_iy) -> log-odds 값 (희소 저장)

        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(PointCloud2, '/camera/depth/points', self._on_points, 5)

        self.pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 1)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'occupancy_grid 시작 — {self.cells}×{self.cells}셀 @ {self.resolution}m → /occupancy_grid')

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_imu(self, msg: Imu):
        roll, pitch = _roll_pitch(msg.orientation)
        self.tilted = abs(roll) > TILT_MAX or abs(pitch) > TILT_MAX

    def _cell(self, x: float, y: float):
        return (math.floor(x / self.resolution), math.floor(y / self.resolution))
        # 연속좌표 -> 이산좌표 로 바꿔주는 함수
    def _on_scan(self, msg: LaserScan):
        if self.odom is None or self.tilted:
            return  # 기울면 이번 스캔은 통째로 버림 — 유령 장애물 방지

        bx = self.odom.pose.pose.position.x
        by = self.odom.pose.pose.position.y
        yaw = _yaw(self.odom.pose.pose.orientation)
        rx, ry = self._cell(bx, by)

        angle = msg.angle_min
        for r in msg.ranges:
            beam_angle = angle
            angle += msg.angle_increment
            if not (msg.range_min < r < msg.range_max and math.isfinite(r)):
                continue  # 무반사/무효 빔은 건너뜀 (skeleton 최소 구현)

            world_angle = yaw + beam_angle
            ex = bx + r * math.cos(world_angle)
            ey = by + r * math.sin(world_angle)
            ex_i, ey_i = self._cell(ex, ey)

            path = _bresenham(rx, ry, ex_i, ey_i)
            for cx, cy in path[:-1]:               # 경로(자유공간) — 끝점 제외
                l = self.log_odds.get((cx, cy), 0.0) + L_FREE
                self.log_odds[(cx, cy)] = max(L_MIN, min(L_MAX, l))
            l = self.log_odds.get((ex_i, ey_i), 0.0) + L_OCC  # 끝점(히트)
            self.log_odds[(ex_i, ey_i)] = max(L_MIN, min(L_MAX, l))
    # log odds 갱신을 위해 log likelihood ratio를 one-step마다 더해서 갱신

    def _on_points(self, msg: PointCloud2):
        """뎁스캠으로 LiDAR 사각지대 보완 — 점유 증거만 더함(자유공간 갱신 없음)."""
        if self.odom is None or self.tilted:
            return

        pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.shape[0] == 0:
            return
        pts = pts[::self.cam_stride]  # 프레임당 90만+ 포인트 — 다운샘플 필수

        # 카메라 광학 프레임(REP103: X우측,Y아래,Z전방) → 보트 로컬(전방,좌측,높이)
        # 카메라도 base_link 와 동일 위치/자세로 가정(TF 미사용, LiDAR 쪽과 동일 단순화)
        lx, ly, up = pts[:, 2], -pts[:, 0], -pts[:, 1]

        # sim 의 clip(0.1~100m)은 실물 유효범위 밖도 유효값으로 내보내므로
        # 거리(CAM_RANGE)와 z-band 를 여기서 같이 걸러야 함 — 안 그러면 배
        # 자신의 구조물(근접)이나 원거리 배경(오탐)이 장애물로 잡힘
        in_range = (lx > CAM_RANGE_MIN) & (lx < CAM_RANGE_MAX)
        world_z = self.cam_height + up
        band = in_range & (world_z > self.cam_z_min) & (world_z < self.cam_z_max)
        lx, ly = lx[band], ly[band]
        if lx.shape[0] == 0:
            return

        bx = self.odom.pose.pose.position.x
        by = self.odom.pose.pose.position.y
        yaw = _yaw(self.odom.pose.pose.orientation)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        wx = bx + lx * cos_y - ly * sin_y
        wy = by + lx * sin_y + ly * cos_y

        for x, y in zip(wx, wy):
            key = self._cell(x, y)
            l = self.log_odds.get(key, 0.0) + L_OCC_CAM
            self.log_odds[key] = max(L_MIN, min(L_MAX, l))

    def _prune(self, cx: int, cy: int):
        """현재 rolling window 밖으로 나간 셀은 버려 메모리를 유계로 유지."""
        margin = self.cells // 2 + 5
        stale = [k for k in self.log_odds
                 if abs(k[0] - cx) > margin or abs(k[1] - cy) > margin]
        for k in stale:
            del self.log_odds[k]

    def _publish(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'odom'
        grid.info.resolution = self.resolution
        grid.info.width = self.cells
        grid.info.height = self.cells
        grid.info.origin.orientation.w = 1.0

        if self.odom is None:
            grid.data = [-1] * (self.cells * self.cells)
            self.pub.publish(grid)
            return

        # rolling window — 배(odom) 를 중심에 두고 origin 을 좌하단으로
        bx = self.odom.pose.pose.position.x
        by = self.odom.pose.pose.position.y
        ox = bx - self.size / 2.0
        oy = by - self.size / 2.0
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy

        self._prune(*self._cell(bx, by))

        data = [-1] * (self.cells * self.cells)
        for j in range(self.cells):
            wy = oy + (j + 0.5) * self.resolution
            for i in range(self.cells):
                wx = ox + (i + 0.5) * self.resolution
                l = self.log_odds.get(self._cell(wx, wy))
                if l is not None:
                    p = 1.0 - 1.0 / (1.0 + math.exp(l))
                    data[j * self.cells + i] = int(round(p * 100))
        grid.data = data
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
