"""obstacle_avoidance_utils — 공통 회피-보정 함수 라이브러리 (노드 아님).

station/search 는 주행 명령에 이 함수를 적용하고, dock 은 APPROACH·
ACQUIRE(표식 탐색 중)·PRE_ENTRY 에서만 적용한다. gate/avoid 는 자체 DRI
판단을 사용하고 미션 시작 자세로 이동하는 approach 는 이 함수를 호출하지 않는다.

점유 격자 기반 단순 반발(repulsion) 조향:
  occupancy_grid 노드는 LiDAR hit/free 를 log-odds 로 누적해 관측 셀을
  확률(0~100), 미관측 셀을 -1 로 발행한다. 이 함수는 확률 50 이상인
  전방 ±45°·4m 이내 셀마다 거리 선형 가중치를 합산해 반대쪽으로 조향한다.
  가장 가까운 셀이 1.5m 미만이면 전진 명령을 줄인다.

이 함수 자체는 격자의 frame/stamp 를 검사하거나 정지·후진을 결정하지 않는다.
"""
import math

from geometry_msgs.msg import Twist

from .behavior_base import MAX_THRUST

AVOID_RANGE = 4.0        # 이 거리 안의 점유 셀만 고려 [m]
AVOID_FOV = math.pi / 2  # 전방 ±45° 부채꼴만 고려
GAIN = 30.0 / MAX_THRUST  # 반발 조향 게인 — steer 1 당 차동 30N (구 0.5 동등)
OCC_THRESHOLD = 50       # 점유 확률[0~100] 이 값 이상인 셀만 장애물로 취급


def apply_repulsion(cmd: Twist, occupancy_grid, odom, ignore=None,
                    occ_threshold: int = OCC_THRESHOLD) -> Twist:
    """주행 명령 cmd 에 격자 기반 장애물 반발 조향을 더해 돌려준다.

    occupancy_grid  nav_msgs/OccupancyGrid (frame_id='odom', 점유확률 0~100/-1)
    odom            nav_msgs/Odometry (현재 위치/자세) — 선체 기준 변환에 필요
    ignore          (x, y, r) odom — 이 원 안의 셀은 장애물로 안 본다
                    (탐색 선회 중 일부러 도는 목표 부표)
    occ_threshold   이 점유 확률 이상인 셀만 장애물 (기본 OCC_THRESHOLD 50). 탐색은 75 — 배 근처
                    잡음 히트 한두 번(60~70)으로 생긴 칸을 거른다 (search_orbit.SearchParams.repulse_occ_min)
    """
    # A reached waypoint or failed odom check must remain a stop command.
    if (cmd.linear.x == 0.0 and cmd.angular.z == 0.0
            or odom is None or occupancy_grid is None):
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
        if v < occ_threshold:      # -1(미탐색) 또는 저확률 셀은 건너뜀
            continue
        # 셀 인덱스 → odom 월드 좌표 (셀 중심)
        wx = ox + (i % width + 0.5) * res
        wy = oy + (i // width + 0.5) * res
        if ignore is not None and math.hypot(wx - ignore[0], wy - ignore[1]) <= ignore[2]:
            continue
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
