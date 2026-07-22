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

핵심 동작 (요약) — "global 격자 위를 미끄러지는 고정 배열(rolling grid)":
  1) 세상 전체를 global 정수 격자(floor(월드좌표/resolution))로 본다.
     저장소는 배의 global 셀 주위 cells×cells 블록만 담는 고정 numpy
     배열이고, anchor(배열 (0,0)셀의 global 인덱스)가 어느 블록을 보고
     있는지를 가리킨다.
  2) 배가 움직이면 anchor 를 배의 절대 위치에서 매번 새로 계산해
     (이동량 누적이 아님 — 반올림 오차 무누적), 이전 anchor 와의 차이만큼
     배열을 정수 셀 단위로 밀고(np.roll) 새로 노출된 가장자리만 unknown
     으로 초기화한다. 격자는 항상 월드 축 정렬 — heading 회전은 절대
     하지 않으므로 보간/누적 오차가 없다.
  3) 관측(스캔/포인트)은 sensor→world 변환 후 자기 global 셀에 박히고,
     배열 인덱스는 (global 셀 − anchor). window(ROI) 밖 셀은 쓰기 시점에
     그냥 버린다 — 별도 prune 이 필요 없다.
  4) publish(1Hz)는 배열을 통째로 확률(0~100)로 변환해 내보낸다. origin
     은 anchor×resolution (격자에 스냅된 값) — 발행 셀과 저장 셀이 1:1 로
     정확히 겹쳐 aliasing(±1셀 튐)이 없다.

구현 — /scan 레이캐스팅으로 log-odds 갱신 (numpy + cv2 벡터화):
  끝점 계산은 numpy 일괄, 빔 경로(free)는 cv2.polylines(C 래스터라이저)로
  마스크에 한 번에 그린다 — 파이썬 Bresenham 루프(최악 169ms/스캔) 대비
  수 ms. 경로 셀은 스캔당 1회 L_FREE 감쇠(같은 스캔의 이웃 빔은 독립 관측이
  아니므로 중복 감쇠 안 함), 유효 히트 끝점은 빔 수만큼 L_OCC 중복 누적 —
  실물 장애물은 근거리에서 셀당 여러 빔이 박혀 한 스캔에 포화(≈97)된다.
  히트가 window 밖인 빔도 경계까지의 경로는 free 로 갱신(cv2 클리핑).
  lidar와 base_link 는 동일 위치/자세로 가정(TF 미사용, behavior 쪽과 동일
  단순화) — 실물 전환 시 lidar 장착 오프셋 보정 필요.
  IMU 롤/피치가 TILT_MAX 를 넘으면 그 스캔은 통째로 버린다(배가 기울면
  2D LiDAR 가 수면/하늘을 훑어 유령 장애물이 생기기 때문).

  뎁스캠(/camera/depth/points)은 LiDAR 를 대체하는 게 아니라 사각지대
  보완용 — LiDAR 수평 한 평면이 놓치는 물체를 z-band(수면 위 cam_z_min~
  cam_z_max) + 거리(CAM_RANGE_MIN~MAX) 필터로 걸러 같은 격자에
  L_OCC_CAM 만큼 더한다. 자유공간 레이캐스팅은 안 함(점유 증거만 추가).
  두 센서가 같은 셀을 같이 보면 log-odds 가 더 누적돼 더 확신 있는(짙은)
  셀이 되는 식으로 자연히 "융합"된다 — 별도 융합 로직 없이 같은 누적
  장부를 공유할 뿐. 프레임당 90만+ 포인트라 cam_stride 로 다운샘플한다.
  거리 필터가 중요한 이유: sim 의 depth clip 이 0.1~100m 라 그 안은 전부
  "유효값"으로 나오지만, 실물 D455 는 ≈0.6~6m 밖에서 부정확 — 안 거르면
  카메라 근접 노이즈(배 자신의 구조물 포함)나 원거리 배경이 장애물로
  오인된다 (wamv_kaboat.xacro 의 depth_camera 센서 주석 참고).

TODO(팀):
  1) decay(노후 셀 확률의 점진적 감쇠) — 현재는 window 밖으로 나갈 때만
     소멸하고, window 안 셀은 새 관측 없인 안 낡음
  2) inverse sensor model 을 거리/각도 함수로 — 지금 L_OCC/L_FREE 는 거리·빔
     중심으로부터의 각도와 무관한 고정 상수(원래는 가까울수록/빔 중심에
     가까울수록 더 신뢰해야 함). 정확도보단 단순함을 택한 skeleton 근사.
     -> 일단 실물 테스트에서 log odds ratio는 상수로 하고 정확도가 안나오면 다른 알고리즘 모색
"""
import math

import cv2
import numpy as np
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
TILT_MAX = math.radians(10.0)  # 이 이상 기울면 해당 스캔 버림 (15→10: 파도 유령 근원 축소)


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


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

        self.declare_parameter('resolution', 0.2)   # 셀 크기 [m]
        self.declare_parameter('size', 20.0)        # window 한 변 길이 [m]
        self.declare_parameter('publish_rate', 10.0)  # 발행 주기 [Hz] — 스캔(10Hz)과 동율.
        # 변환이 0.07ms 라 부담 없고, 유령/신규 장애물이 소비자에게 보이는 지연을
        # 최대 1s → 0.1s 로 줄인다. 10Hz 넘게 올려도 스캔이 10Hz 라 무의미.
        self.declare_parameter('hit_dedup', True)    # 히트를 스캔당 셀당 1회로 캡
        # 같은 스캔의 이웃 빔은 독립 관측이 아니므로 중복 누적하면 증거 과대계상 —
        # 수면 반사 노이즈가 한 스캔에 검정(97)까지 튀는 원인이었다(베이즈 독립성 교정).
        # A/B 실측(파도 gain0.5, 45s): 노이즈 피크≥90 비율 78→14%, 수명 0.97→0.52s,
        # 부표 가시성 손실 없음. 성능도 최악조건 0.97→0.74ms/스캔으로 오히려 빨라짐
        # (np.add.at 산발 누적 → unique+인덱싱). False 로 두면 예전 중복 누적 방식.
        self.declare_parameter('cam_stride', 1000)   # 뎁스 포인트 다운샘플 간격
        self.declare_parameter('cam_height', 0.5)    # 카메라 장착 높이(수면 기준) [m]
        self.declare_parameter('cam_z_min', 0.1)     # z-band 하한 — 수면 근처 제외 [m]
        self.declare_parameter('cam_z_max', 2.0)     # z-band 상한 — 상부구조물/하늘 제외 [m]

        self.resolution = float(self.get_parameter('resolution').value)
        self.size = float(self.get_parameter('size').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.cells = int(self.size / self.resolution)
        self.hit_dedup = bool(self.get_parameter('hit_dedup').value)
        self.cam_stride = int(self.get_parameter('cam_stride').value)
        self.cam_height = float(self.get_parameter('cam_height').value)
        self.cam_z_min = float(self.get_parameter('cam_z_min').value)
        self.cam_z_max = float(self.get_parameter('cam_z_max').value)

        self.odom = None
        self.tilted = False
        # rolling grid — [iy, ix] 순서 (OccupancyGrid row-major 와 동일).
        # anchor = 배열 (0,0) 셀의 global 격자 인덱스 (ax, ay).
        self.anchor = None
        self.grid = np.zeros((self.cells, self.cells), dtype=np.float32)  # log-odds
        self.observed = np.zeros((self.cells, self.cells), dtype=bool)    # 관측 여부

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
        # 연속좌표 -> 이산좌표(global 격자 인덱스) 로 바꿔주는 함수

    def _recenter(self):
        """배의 절대 위치로 anchor 를 다시 잡고 배열을 정수 셀만큼 민다.

        anchor 는 이동량 누적이 아니라 매번 floor(절대위치/res) 에서 새로
        계산하므로 반올림 오차가 누적되지 않는다(항상 ≤1셀 양자화뿐).
        shift 는 정수 roll(복사)이라 값 손실/보간 오차가 없다.
        """
        bx_i, by_i = self._cell(self.odom.pose.pose.position.x,
                                self.odom.pose.pose.position.y)
        ax = bx_i - self.cells // 2
        ay = by_i - self.cells // 2
        if self.anchor is None:
            self.anchor = (ax, ay)
            return
        dx = ax - self.anchor[0]
        dy = ay - self.anchor[1]
        if dx == 0 and dy == 0:
            return
        if abs(dx) >= self.cells or abs(dy) >= self.cells:
            # 한 번에 window 를 통째로 벗어남 (텔레포트/리셋) — 전체 초기화
            self.grid.fill(0.0)
            self.observed.fill(False)
        else:
            self.grid = np.roll(self.grid, (-dy, -dx), axis=(0, 1))
            self.observed = np.roll(self.observed, (-dy, -dx), axis=(0, 1))
            # roll 은 반대편으로 감아 돌므로, 새로 노출된 가장자리를 unknown 으로
            if dx > 0:
                self.grid[:, -dx:] = 0.0
                self.observed[:, -dx:] = False
            elif dx < 0:
                self.grid[:, :-dx] = 0.0
                self.observed[:, :-dx] = False
            if dy > 0:
                self.grid[-dy:, :] = 0.0
                self.observed[-dy:, :] = False
            elif dy < 0:
                self.grid[:-dy, :] = 0.0
                self.observed[:-dy, :] = False
        self.anchor = (ax, ay)

    def _on_scan(self, msg: LaserScan):
        if self.odom is None or self.tilted:
            return  # 기울면 이번 스캔은 통째로 버림 — 유령 장애물 방지
        self._recenter()

        bx = self.odom.pose.pose.position.x
        by = self.odom.pose.pose.position.y
        yaw = _yaw(self.odom.pose.pose.orientation)
        ax, ay = self.anchor
        c = self.cells
        # 배의 배열 인덱스 — recenter 직후라 항상 중앙 근처
        rx = math.floor(bx / self.resolution) - ax
        ry = math.floor(by / self.resolution) - ay

        # 끝점 계산 — 빔 2000개를 numpy 로 일괄 처리 (파이썬 루프 제거)
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + msg.angle_increment * np.arange(ranges.shape[0])
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not valid.any():
            return  # 무반사/무효 빔뿐 (skeleton 최소 구현)
        wa = yaw + angles[valid]
        r = ranges[valid]
        ex = np.floor((bx + r * np.cos(wa)) / self.resolution).astype(np.int64) - ax
        ey = np.floor((by + r * np.sin(wa)) / self.resolution).astype(np.int64) - ay

        # free 경로 — 배→끝점 선분 전부를 cv2(C 래스터라이저)로 마스크에 한 번에
        # 그림 (파이썬 Bresenham 루프 169ms/스캔 → 수 ms). window 밖 끝점은 cv2
        # 가 경계에서 클리핑하므로 "경계까지 free" 정책 그대로.
        # 의미 변화 하나: 루프 시절엔 한 셀을 지나는 빔 수만큼 L_FREE 를 중복
        # 누적했지만, 마스크는 스캔당 셀당 1회만 깎는다 — 같은 스캔의 이웃 빔은
        # 독립 관측이 아니므로 과감쇠를 막는 쪽(장애물 보존, 안전 방향)을 택함.
        segs = np.empty((ex.shape[0], 2, 2), np.int32)
        segs[:, 0, 0] = rx
        segs[:, 0, 1] = ry
        segs[:, 1, 0] = ex
        segs[:, 1, 1] = ey
        free = np.zeros((c, c), np.uint8)
        cv2.polylines(free, segs, False, 1)

        # 히트 셀 — window 안 끝점만
        m = (ex >= 0) & (ex < c) & (ey >= 0) & (ey < c)
        hx, hy = ex[m], ey[m]
        free[hy, hx] = 0                     # 히트 셀은 free 갱신에서 제외 (기존과 동일)
        fm = free.astype(bool)
        self.grid[fm] += L_FREE
        if self.hit_dedup:
            # 스캔당 셀당 1회 — 같은 스캔의 이웃 빔은 독립 관측이 아님 (파라미터 주석 참고)
            uniq = np.unique(hy.astype(np.int64) * c + hx)
            self.grid[uniq // c, uniq % c] += L_OCC
        else:
            # 같은 셀에 박힌 빔 수만큼 중복 누적 — 실물 장애물이 한 스캔에 포화되는 근거
            np.add.at(self.grid, (hy, hx), L_OCC)
        np.clip(self.grid, L_MIN, L_MAX, out=self.grid)
        self.observed |= fm
        self.observed[hy, hx] = True
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

        self._recenter()
        bx = self.odom.pose.pose.position.x
        by = self.odom.pose.pose.position.y
        yaw = _yaw(self.odom.pose.pose.orientation)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        wx = bx + lx * cos_y - ly * sin_y
        wy = by + lx * sin_y + ly * cos_y

        gx = np.floor(wx / self.resolution).astype(np.int64) - self.anchor[0]
        gy = np.floor(wy / self.resolution).astype(np.int64) - self.anchor[1]
        m = (gx >= 0) & (gx < self.cells) & (gy >= 0) & (gy < self.cells)
        if not m.any():
            return
        np.add.at(self.grid, (gy[m], gx[m]), L_OCC_CAM)  # 중복 셀도 누적
        np.clip(self.grid, L_MIN, L_MAX, out=self.grid)
        self.observed[gy[m], gx[m]] = True

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

        self._recenter()
        # origin 은 anchor 를 그대로 미터로 — 격자에 스냅돼 있어 발행 셀과
        # 저장 셀이 1:1 로 겹친다 (배 위치는 중앙 셀 안, 오차 ≤1셀)
        grid.info.origin.position.x = self.anchor[0] * self.resolution
        grid.info.origin.position.y = self.anchor[1] * self.resolution

        p = 1.0 / (1.0 + np.exp(-self.grid))
        data = np.where(self.observed, np.rint(p * 100.0), -1.0)

        grid.data = data.astype(np.int8).ravel().tolist()
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
