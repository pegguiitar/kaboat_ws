"""obstacle_avoidance_utils — 공통 회피-보정 함수 라이브러리 (노드 아님).

규정 5.2(7) "장애물 회피 상시 작동" 대응 — obstacle_detector 는 항상 켜져 있고,
각 behavior 는 자기 주행 명령을 만든 뒤 이 함수로 /obstacles 를 반영해
회피 보정한다. 5개 behavior 에 각자 구현하면 미묘하게 다른 회피 동작이
생기므로 반드시 이 모듈을 공유해서 쓸 것.

skeleton — 단순 반발(repulsion) 조향:
  전방 부채꼴 안의 장애물마다 거리 반비례 가중치로 반대쪽 조향을 더한다.

TODO(팀): VFH(valley 탐색) / DRI 기반으로 교체, 후진/정지 판단 추가
"""
import math

from geometry_msgs.msg import Twist

AVOID_RANGE = 6.0        # 이 거리 안의 장애물만 고려 [m]
AVOID_FOV = math.pi / 2  # 전방 ±45° 부채꼴만 고려
GAIN = 0.5               # 반발 조향 게인


def apply_repulsion(cmd: Twist, obstacles, odom, obstacles_frame: str = 'odom') -> Twist:
    """주행 명령 cmd 에 장애물 반발 조향을 더해 돌려준다.

    obstacles        kaboat_msgs/Obstacle 목록
    odom             nav_msgs/Odometry (현재 위치/자세) — frame 변환에 필요
    obstacles_frame  obstacles 좌표계 ('odom' 이 아니면 선체 기준으로 취급)
    """
    if odom is None or not obstacles:
        return cmd

    q = odom.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    bx = odom.pose.pose.position.x
    by = odom.pose.pose.position.y

    steer = 0.0
    nearest = float('inf')
    for ob in obstacles:
        if obstacles_frame == 'odom':
            # odom 좌표 → 선체 기준 (전방 x, 좌측 y)
            dx, dy = ob.position.x - bx, ob.position.y - by
            lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        else:
            lx, ly = ob.position.x, ob.position.y  # 이미 센서(선체) 기준

        dist = math.hypot(lx, ly) - ob.radius
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
