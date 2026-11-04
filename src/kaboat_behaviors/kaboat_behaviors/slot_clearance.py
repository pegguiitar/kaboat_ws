"""slot_clearance — 점유 격자에서 도킹 슬롯 옆벽까지의 간격을 재는 순수 라이브러리.

ROS 비의존. `dri.py`처럼 OccupancyGrid 메시지 **모양의 객체**(info.resolution/
width/height/origin.position, data)를 덕타이핑으로 읽는다.

DRI(위험도) 값이 아니라 점유 셀 자체의 위치를 쓴다. DRI는 가우시안 가중이라
미터 단위 간격이 아니고, 접안 위치에서는 뒷벽 때문에 선체 전면 값도 커진다.

선체 좌표계(전방 x, 좌측 y)에서 선체 길이 방향 창 [-back, +front] 안의 점유
셀 중, 선체 폭 밖(|y| >= 선체 반폭)에 있는 셀까지의 **선체 가장자리~벽 간격**을
좌/우 각각의 최솟값으로 돌려준다. 선체 발자국 안쪽의 셀은 선체 자체로 보고
무시한다. 뒷벽이 창에 들어오면 옆벽으로 오인하므로 호출자가 front 를
`뒷벽까지 거리 - 여유` 이하로 줄여 넘긴다.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SlotClearance:
    left: float | None     # 선체 좌측 가장자리~벽 최소 간격 [m], 관측 없으면 None
    right: float | None
    hull_width: float

    @property
    def both(self) -> bool:
        return self.left is not None and self.right is not None

    @property
    def min_gap(self) -> float | None:
        gaps = [g for g in (self.left, self.right) if g is not None]
        return min(gaps) if gaps else None

    @property
    def slot_width(self) -> float | None:
        """양쪽 벽이 모두 보일 때 벽-벽 폭 [m]."""
        if not self.both:
            return None
        return self.left + self.right + self.hull_width


def measure_slot(grid, boat_xy, boat_yaw: float, *, hull_width: float,
                 back: float, front: float, max_side: float = 3.0,
                 occ_threshold: int = 85) -> SlotClearance:
    """점유 격자에서 선체 좌/우 옆벽 간격을 잰다.

    grid        OccupancyGrid 모양 객체 (odom 프레임, 점유확률 0~100 / -1)
    boat_xy     odom 에서의 선체 중심 (x, y)
    back/front  선체 중심 기준 뒤/앞 창 길이 [m]
    max_side    선체 중심에서 이 거리 밖의 셀은 벽으로 보지 않는다 [m]
    """
    half = hull_width / 2.0
    res = float(grid.info.resolution)
    width, height = int(grid.info.width), int(grid.info.height)
    data = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    iy, ix = np.nonzero(data >= occ_threshold)
    if ix.size == 0 or front < -back:
        return SlotClearance(None, None, hull_width)

    wx = grid.info.origin.position.x + (ix + 0.5) * res
    wy = grid.info.origin.position.y + (iy + 0.5) * res
    dx, dy = wx - float(boat_xy[0]), wy - float(boat_xy[1])
    c, s = math.cos(boat_yaw), math.sin(boat_yaw)
    lx = dx * c + dy * s          # 선체 전방
    ly = -dx * s + dy * c         # 선체 좌측

    in_window = (lx >= -back) & (lx <= front) & (np.abs(ly) <= max_side)
    outside_hull = np.abs(ly) >= half
    keep = in_window & outside_hull
    left = ly[keep & (ly > 0)]
    right = -ly[keep & (ly < 0)]
    return SlotClearance(
        float(left.min() - half) if left.size else None,
        float(right.min() - half) if right.size else None,
        hull_width)
