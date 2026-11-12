"""D455 장착 실측값 — 도킹 TF(라이다 → camera_link)와 카메라 층(occupancy_grid)이 같이 쓴다.

카메라를 옮기면 여기만 고친다 (dock_yolo·도킹 미션 런치·autonomy·camera_obstacles 가 읽음).
2026-11-12 현장 실측:
  - TG-50 회전 중심(스캔 면)은 선체 중심(base_link)보다 선수 0.4 m (odom_tf_broadcaster 와 같음)
  - D455 렌즈는 라이다보다 뒤 0.615 m, 위 0.635 m → 선체 중심 기준 뒤 0.215 m
  - 수평 장착(숙임 0°), 흘수선 위 0.775 m
launch 에서 가볍게 import 하도록 numpy·cv2 를 쓰지 않는다.
"""

import math

LIDAR_X = 0.4                                # base_link → 라이다 [m] (선수 +)
CAMERA_FROM_LIDAR = (-0.615, 0.0, 0.635)     # 라이다 → camera_link [m] (선수 +, 좌현 +, 위 +)
CAMERA_PITCH_DEG = 0.0                       # 장착 숙임 [deg] (+ = 아래)
CAMERA_HEIGHT_ABOVE_WATER = 0.775            # 흘수선 → 렌즈 [m]
CAMERA_X = round(LIDAR_X + CAMERA_FROM_LIDAR[0], 4)   # 선체 중심 기준 [m] (-0.215)
CAMERA_Y = CAMERA_FROM_LIDAR[1]


def camera_mount_tf_arguments(parent_frame='onboard_laser_frame', child_frame='camera_link'):
    """static_transform_publisher 인자 (라이다 frame → camera_link). frame 은 launch 치환식도 됨."""
    x, y, z = CAMERA_FROM_LIDAR
    return ['--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', '0', '--pitch', str(math.radians(CAMERA_PITCH_DEG)), '--yaw', '0',
            '--frame-id', parent_frame, '--child-frame-id', child_frame]
