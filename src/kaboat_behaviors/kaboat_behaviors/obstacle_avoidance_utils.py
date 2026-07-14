"""obstacle_avoidance_utils — 공통 회피-보정 함수 라이브러리 (노드 아님).

규정 5.2(7) "장애물 회피 상시 작동" 대응 — 각 behavior 는 자기 주행 명령을
만든 뒤 이 함수로 /occupancy_grid(v5 의 단일 장애물 표현)를 반영해 회피
보정한다. 5개 behavior 에 각자 구현하면 미묘하게 다른 회피 동작이 생기므로
반드시 이 모듈을 공유해서 쓸 것.

skeleton — 격자 점유 셀 기반 단순 반발(repulsion) 조향:
  occupancy_grid 에서 점유(occupied) 셀을 전방 부채꼴로 훑어 거리 반비례
  가중치로 반대쪽 조향을 더한다. **지금은 occupancy_grid 가 빈 격자(-1)라
  자연히 무동작** — occupancy_grid 노드가 log-odds 로 채워지면 코드 변경
  없이 바로 회피가 살아난다.

TODO(팀): VFH(valley 탐색) / DRI 기반으로 교체, 후진/정지 판단 추가
"""
import math

from geometry_msgs.msg import Twist

AVOID_RANGE = 6.0        # 이 거리 안의 점유 셀만 고려 [m]
AVOID_FOV = math.pi / 2  # 전방 ±45° 부채꼴만 고려
GAIN = 0.5               # 반발 조향 게인
OCC_THRESHOLD = 50       # 점유 확률[0~100] 이 값 이상인 셀만 장애물로 취급


def apply_repulsion(cmd: Twist, occupancy_grid, odom) -> Twist:
    """주행 명령 cmd 에 격자 기반 장애물 반발 조향을 더해 돌려준다.

    occupancy_grid  nav_msgs/OccupancyGrid (frame_id='odom', 점유확률 0~100/-1)
    odom            nav_msgs/Odometry (현재 위치/자세) — 선체 기준 변환에 필요
    """
    if odom is None or occupancy_grid is None:
        return cmd

    info = occupancy_grid.info
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    width = info.width

    q = odom.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    bx = odom.pose.pose.position.x
    by = odom.pose.pose.position.y

    steer = 0.0
    nearest = float('inf')
    for i, v in enumerate(occupancy_grid.data):
        if v < OCC_THRESHOLD:      # -1(미탐색) 또는 저확률 셀은 건너뜀
            continue
        # 셀 인덱스 → odom 월드 좌표 (셀 중심)
        wx = ox + (i % width + 0.5) * res
        wy = oy + (i // width + 0.5) * res
        # odom → 선체 기준 (전방 x, 좌측 y)
        dx, dy = wx - bx, wy - by
        lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        dist = math.hypot(lx, ly)
        bearing = math.atan2(ly, lx)
        if dist > AVOID_RANGE or abs(bearing) > AVOID_FOV / 2 or lx <= 0:
            continue

        nearest = min(nearest, dist)
        # 장애물이 좌측(+bearing)이면 우측(−z)으로 조향, 가까울수록 강하게
        weight = (AVOID_RANGE - dist) / AVOID_RANGE
        steer -= math.copysign(weight, bearing)

    cmd.angular.z += GAIN * steer
    # 코앞(1.5m 이내)이면 감속
    if nearest < 1.5:
        cmd.linear.x *= max(nearest / 1.5, 0.2)
    return cmd
