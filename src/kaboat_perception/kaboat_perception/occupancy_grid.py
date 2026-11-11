"""occupancy_grid — 장애물 격자 지도 (상시 가동, ① 레이어)

입력   /scan (sensor_msgs/LaserScan)                  — 주 소스
       /camera/color/image_raw + /camera/depth/image_raw (정렬 뎁스) + /camera/camera_info
                                                      — 카메라 층 (cam_enabled, 색 장애물)
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
  실물 장애물은 근거리에서 셀당 여러 빔이 박혀 한 스캔에 포화(≈99)된다.
  히트가 window 밖인 빔도 경계까지의 경로는 free 로 갱신(cv2 클리핑).
  lidar와 base_link 는 동일 위치/자세로 가정(TF 미사용, behavior 쪽과 동일
  단순화) — 실물 전환 시 lidar 장착 오프셋 보정 필요.
  enable_tilt_gate=true 일 때 IMU 롤/피치가 TILT_MAX 를 넘으면 그 스캔은
  통째로 버린다(배가 기울면 2D LiDAR 가 수면/하늘을 훑을 수 있기 때문).
  실내 수조처럼 평형을 가정하는 시험은 이 게이트를 끌 수 있다.

  카메라 층(cam_enabled, 기본 끔): 라이다 수평 한 면이 놓치는 수면 위 낮은 물체
  (주황 붐 등)를 컬러로 고르고 정렬 뎁스로 위치를 재 별도 배열 cam_grid 에 쌓는다
  (camera_obstacles.py). 라이다 배열에 그냥 더하면 붐 위를 지나는 라이다 빔이 매
  스캔 그 칸을 free 로 깎아 지워 버리므로 층을 나눴다. cam_grid 는 카메라가 지금
  볼 수 있는 칸(시야·거리 안)에서만 색 점이 없을 때 감쇠하고, 시야 밖 칸은 그대로
  둔다(옆으로 지나간 붐을 잊지 않게). 발행 확률 = max(라이다, 카메라).
  한 프레임 오탐은 0.8(≈69) 이라 DRI 문턱 95 에 못 미치고, 같은 칸이 4프레임
  (5 Hz 에서 0.8 s) 이어져야 장애물이 된다.

TODO(팀):
  1) decay(노후 셀 확률의 점진적 감쇠) — 현재는 window 밖으로 나갈 때만
     소멸하고, window 안 셀은 새 관측 없인 안 낡음
  2) inverse sensor model 을 거리/각도 함수로 — 지금 L_OCC/L_FREE 는 거리·빔
     중심으로부터의 각도와 무관한 고정 상수(원래는 가까울수록/빔 중심에
     가까울수록 더 신뢰해야 함). 정확도보단 단순함을 택한 skeleton 근사.
     -> 일단 실물 테스트에서 log odds ratio는 상수로 하고 정확도가 안나오면 다른 알고리즘 모색
"""
import dataclasses
import math
import time

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from kaboat_perception.camera_obstacles import (
    CameraObstacleParams, color_image_rgb, depth_image_m, detect_points, visible_mask)

L_OCC = 0.80              # 히트 셀 log-odds 증가량 (≈ logit(0.7))
L_FREE = -0.6             # 경로(자유공간) 셀 log-odds 감소량 (≈ logit(0.4))
# 둘 다 거리/빔각도 무관 고정값 — 진짜 inverse sensor model 이면 거리·빔중심
# 이탈각의 함수여야 하는데, 그러면 복잡해지니 skeleton 은 상수로 근사 (TODO 2)
L_MIN, L_MAX = -4.5, 4.35  # log-odds 클램프. 상한 4.35(≈5히트)면 포화 확률 ≈99 —
# DRI 문턱 95 위로 free ~3.5스캔의 여유가 생겨 부표가 경계(95~97)에서 안 깜빡인다.
# 상한을 올려도 유령엔 영향 없다: 3히트 해수면 유령(93)은 애초에 상한 근처에 못 가
# 감쇄 속도가 그대로고, 상한까지 쌓이는 건 꾸준히 관측되는 실제 장애물뿐이다(예전 3.5는
# 포화 97이라 문턱에 바짝 붙어 free 레이 한둘에 깜빡였다).
TILT_MAX = math.radians(3.0)  # 이 이상 기울면 해당 스캔 버림


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


def _shift(arr, dx: int, dy: int, fill):
    """rolling window 를 (dx, dy) 셀 민다 — 새로 노출된 가장자리는 fill(unknown).

    한 번에 window 를 통째로 벗어나면(텔레포트/리셋) 전체를 fill 로 초기화.
    roll 은 정수 복사라 값 손실/보간 오차가 없다.
    """
    n_y, n_x = arr.shape
    if abs(dx) >= n_x or abs(dy) >= n_y:
        out = np.empty_like(arr)
        out.fill(fill)
        return out
    out = np.roll(arr, (-dy, -dx), axis=(0, 1))
    if dx > 0:
        out[:, -dx:] = fill
    elif dx < 0:
        out[:, :-dx] = fill
    if dy > 0:
        out[-dy:, :] = fill
    elif dy < 0:
        out[:-dy, :] = fill
    return out


class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

        self.declare_parameter('resolution', 0.2)   # 셀 크기 [m]
        self.declare_parameter('size', 15.0)        # window 한 변 길이 [m] (배 기준 전후좌우 ≈7.5)
        self.declare_parameter('publish_rate', 10.0)  # 발행 주기 [Hz] — 스캔(10Hz)과 동율.
        # 변환이 0.07ms 라 부담 없고, 유령/신규 장애물이 소비자에게 보이는 지연을
        # 최대 1s → 0.1s 로 줄인다. 10Hz 넘게 올려도 스캔이 10Hz 라 무의미.
        self.declare_parameter('enable_tilt_gate', True)  # 실내 수조 시험은 false
        self.declare_parameter('tilt_threshold_deg', 3.0)  # [deg] 기울기 게이트 임계
        self.declare_parameter('pitch_min_deg', -4.0)      # [deg] 피치 하한
        self.declare_parameter('pitch_max_deg', 2.0)       # [deg] 피치 상한 (1.0 → 2.0, 2026-11-06)
        self.declare_parameter('roll_min_deg', -3.0)       # [deg] 롤 하한
        self.declare_parameter('roll_max_deg', 3.0)        # [deg] 롤 상한
        # 선체 라이다(TG-50) 장착 위치 — base_link(선체 중심) 기준 [m].
        # 실측: 선체 중심보다 선수 쪽 0.4 m, 좌우 0 (2026-11-11. 예전 0.5 m 는 수조 외벽
        # 추적기 판 중점 기준). 0 이면 base_link 와 같다고 가정.
        # gate_follower 의 lidar_offset_x/y, launch 의 base_link→onboard_laser_frame TF 와 같은 값.
        self.declare_parameter('lidar_offset_x', 0.4)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('hit_dedup', True)    # 히트를 스캔당 셀당 1회로 캡
        # 같은 스캔의 이웃 빔은 독립 관측이 아니므로 중복 누적하면 증거 과대계상 —
        # 수면 반사 노이즈가 한 스캔에 검정(97)까지 튀는 원인이었다(베이즈 독립성 교정).
        # A/B 실측(파도 gain0.5, 45s): 노이즈 피크≥90 비율 78→14%, 수명 0.97→0.52s,
        # 부표 가시성 손실 없음. 성능도 최악조건 0.97→0.74ms/스캔으로 오히려 빨라짐
        # (np.add.at 산발 누적 → unique+인덱싱). False 로 두면 예전 중복 누적 방식.
        # 카메라 층 (camera_obstacles.py) — 기본 끔. 켜려면 d455.launch.py 와 cam_height 실측.
        self.declare_parameter('cam_enabled', False)
        self.declare_parameter('cam_rate_hz', 5.0)       # 처리 상한 (카메라 15 fps 중 일부만)
        self.declare_parameter('cam_min_points', 3)      # 한 프레임에 칸당 이만큼 색 점이 있어야 히트
        self.declare_parameter('cam_l_occ', 0.8)         # 히트 log-odds (라이다 L_OCC 와 같게)
        self.declare_parameter('cam_l_free', -0.4)       # 보이는데 색 점 없는 칸 감쇠
        self.declare_parameter('cam_color_topic', '/camera/color/image_raw')
        self.declare_parameter('cam_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('cam_info_topic', '/camera/camera_info')
        for field in dataclasses.fields(CameraObstacleParams):
            self.declare_parameter(f'cam_{field.name}', field.default)

        self.resolution = float(self.get_parameter('resolution').value)
        self.size = float(self.get_parameter('size').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.cells = int(self.size / self.resolution)
        self.hit_dedup = bool(self.get_parameter('hit_dedup').value)
        self.lidar_offset_x = float(self.get_parameter('lidar_offset_x').value)
        self.lidar_offset_y = float(self.get_parameter('lidar_offset_y').value)
        if not (math.isfinite(self.lidar_offset_x) and math.isfinite(self.lidar_offset_y)):
            raise ValueError('lidar_offset_x/y must be finite')
        self.enable_tilt_gate = bool(self.get_parameter('enable_tilt_gate').value)
        self.tilt_threshold_deg = float(self.get_parameter('tilt_threshold_deg').value)
        self.pitch_min_deg = float(self.get_parameter('pitch_min_deg').value)
        self.pitch_max_deg = float(self.get_parameter('pitch_max_deg').value)
        self.roll_min_deg = float(self.get_parameter('roll_min_deg').value)
        self.roll_max_deg = float(self.get_parameter('roll_max_deg').value)

        self.pitch_min_rad = math.radians(self.pitch_min_deg)
        self.pitch_max_rad = math.radians(self.pitch_max_deg)
        self.roll_min_rad = math.radians(self.roll_min_deg)
        self.roll_max_rad = math.radians(self.roll_max_deg)
        self.tilt_max = math.radians(self.tilt_threshold_deg)
        self.cam_enabled = bool(self.get_parameter('cam_enabled').value)
        self.cam_params = CameraObstacleParams(**{
            f.name: type(f.default)(self.get_parameter(f'cam_{f.name}').value)
            for f in dataclasses.fields(CameraObstacleParams)})
        self.cam_period = 1.0 / float(self.get_parameter('cam_rate_hz').value)
        self.cam_min_points = int(self.get_parameter('cam_min_points').value)
        self.cam_l_occ = float(self.get_parameter('cam_l_occ').value)
        self.cam_l_free = float(self.get_parameter('cam_l_free').value)

        self.odom = None
        self.tilted = False
        self.roll_pitch = None          # 최근 IMU 롤·피치 [rad] — 카메라 수평 보정
        self.cam_K = None               # (fx, fy, cx, cy)
        self.cam_width = None
        self.cam_last = 0.0
        # rolling grid — [iy, ix] 순서 (OccupancyGrid row-major 와 동일).
        # anchor = 배열 (0,0) 셀의 global 격자 인덱스 (ax, ay).
        self.anchor = None
        self.grid = np.zeros((self.cells, self.cells), dtype=np.float32)  # log-odds
        self.observed = np.zeros((self.cells, self.cells), dtype=bool)    # 관측 여부
        self.cam_grid = np.zeros((self.cells, self.cells), dtype=np.float32)  # 카메라 층 log-odds ≥ 0

        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        if self.enable_tilt_gate or self.cam_enabled:
            self.create_subscription(
                Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)

        self.pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 1)
        self.cam_points_pub = None
        if self.cam_enabled:
            g = lambda n: str(self.get_parameter(n).value)
            self.create_subscription(CameraInfo, g('cam_info_topic'), self._on_cam_info, 1)
            self._cam_subs = (
                Subscriber(self, Image, g('cam_color_topic'), qos_profile=qos_profile_sensor_data),
                Subscriber(self, Image, g('cam_depth_topic'), qos_profile=qos_profile_sensor_data))
            self._cam_sync = ApproximateTimeSynchronizer(self._cam_subs, 5, 0.05)
            self._cam_sync.registerCallback(self._on_camera)
            # 디버그: 카메라가 고른 색 장애물 점 (odom, z = 수면 위 높이). 구독자 있을 때만
            self.cam_points_pub = self.create_publisher(
                PointCloud2, '/occupancy_grid/cam_points', 1)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'occupancy_grid 시작 — {self.cells}×{self.cells}셀 @ {self.resolution}m '
            f'→ /occupancy_grid (기울기 차단: {self.enable_tilt_gate}, '
            f'pitch=[{self.pitch_min_deg:.1f}°, {self.pitch_max_deg:.1f}°], '
            f'roll=[{self.roll_min_deg:.1f}°, {self.roll_max_deg:.1f}°])')
        if self.cam_enabled:
            c = self.cam_params
            colors = [n for n in ('orange', 'white') if getattr(c, n)]
            self.get_logger().info(
                f'카메라 층 켜짐 — 색 {colors}, 카메라 수면 위 {c.height:.2f} m, '
                f'선체 중심 기준 ({c.offset_x:+.2f}, {c.offset_y:+.2f}) m, 숙임 '
                f'{c.mount_pitch_deg:.1f}°, 거리 {c.range_min:.1f}~{c.range_max:.1f} m, '
                f'{1.0 / self.cam_period:.0f} Hz')

    def _on_odom(self, msg: Odometry):
        self.odom = msg

    def _on_imu(self, msg: Imu):
        roll, pitch = _roll_pitch(msg.orientation)
        self.roll_pitch = (roll, pitch)
        self.tilted = not (self.pitch_min_rad <= pitch <= self.pitch_max_rad) or \
                      not (self.roll_min_rad <= roll <= self.roll_max_rad)

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
        self.grid = _shift(self.grid, dx, dy, 0.0)
        self.observed = _shift(self.observed, dx, dy, False)
        self.cam_grid = _shift(self.cam_grid, dx, dy, 0.0)
        self.anchor = (ax, ay)

    def _on_scan(self, msg: LaserScan):
        if self.odom is None or (self.enable_tilt_gate and self.tilted):
            return  # 기울면 이번 스캔은 통째로 버림 — 유령 장애물 방지
        self._recenter()

        yaw = _yaw(self.odom.pose.pose.orientation)
        # 빔의 원점은 base_link 가 아니라 선체 라이다 장착 위치다(선수 쪽 lidar_offset_x).
        # 이를 빼먹으면 선수 방향 장애물이 그만큼 배 쪽으로 당겨져 찍힌다(수조 실측 0.5 m).
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        bx = (self.odom.pose.pose.position.x
              + self.lidar_offset_x * cy_ - self.lidar_offset_y * sy_)
        by = (self.odom.pose.pose.position.y
              + self.lidar_offset_x * sy_ + self.lidar_offset_y * cy_)
        ax, ay = self.anchor
        c = self.cells
        # 라이다의 배열 인덱스 — recenter 직후라 항상 중앙 근처 (자유공간 빔의 시작점)
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

    def _on_cam_info(self, msg: CameraInfo):
        k = msg.k
        if k[0] > 0.0 and k[4] > 0.0:
            self.cam_K = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))
            self.cam_width = int(msg.width)

    def _on_camera(self, color: Image, depth: Image):
        """컬러 + 정렬 뎁스 → 색 장애물 칸을 카메라 층에 (camera_obstacles.py)."""
        now = time.monotonic()
        if now - self.cam_last < self.cam_period:
            return
        if (self.odom is None or self.cam_K is None or self.roll_pitch is None
                or (self.enable_tilt_gate and self.tilted)):
            return
        self.cam_last = now
        try:
            pts, _ = detect_points(color_image_rgb(color), depth_image_m(depth), self.cam_K,
                                   self.cam_params, *self.roll_pitch)
        except ValueError as exc:
            self.get_logger().warn(f'카메라 프레임 건너뜀: {exc}', throttle_duration_sec=5.0)
            return
        self._add_camera_points(pts)

    def _add_camera_points(self, pts):
        """수평 프레임 점(앞, 왼, 높이) → 월드 칸. 칸당 cam_min_points 이상이면 히트,
        카메라가 볼 수 있는데 히트가 없는 칸은 감쇠한다."""
        self._recenter()
        pose = self.odom.pose.pose
        bx, by = pose.position.x, pose.position.y
        yaw = _yaw(pose.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        wx = bx + pts[:, 0] * c - pts[:, 1] * s
        wy = by + pts[:, 0] * s + pts[:, 1] * c
        ax, ay = self.anchor
        n = self.cells
        gx = np.floor(wx / self.resolution).astype(np.int64) - ax
        gy = np.floor(wy / self.resolution).astype(np.int64) - ay
        inside = (gx >= 0) & (gx < n) & (gy >= 0) & (gy < n)
        hit = np.zeros((n, n), dtype=bool)
        if inside.any():
            flat, counts = np.unique(gy[inside] * n + gx[inside], return_counts=True)
            flat = flat[counts >= self.cam_min_points]
            hit[flat // n, flat % n] = True
        centers = (np.arange(n) + ax + 0.5) * self.resolution, (np.arange(n) + ay + 0.5) * self.resolution
        cx_w, cy_w = np.meshgrid(centers[0], centers[1])     # [iy, ix]
        seen = visible_mask(cx_w, cy_w, bx, by, yaw, self.cam_K, self.cam_width, self.cam_params)
        self.cam_grid[seen & ~hit] += self.cam_l_free
        self.cam_grid[hit] += self.cam_l_occ
        np.clip(self.cam_grid, 0.0, L_MAX, out=self.cam_grid)
        if self.cam_points_pub is not None and self.cam_points_pub.get_subscription_count() > 0:
            cloud = np.stack([wx, wy, pts[:, 2]], axis=1).astype(np.float32)[:3000]
            header = Header(stamp=self.odom.header.stamp, frame_id='odom')
            self.cam_points_pub.publish(pc2.create_cloud_xyz32(header, cloud))

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
        observed = self.observed
        if self.cam_enabled:
            cam = self.cam_grid > 0.0
            p = np.where(cam, np.maximum(p, 1.0 / (1.0 + np.exp(-self.cam_grid))), p)
            observed = observed | cam
        data = np.where(observed, np.rint(p * 100.0), -1.0)

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
