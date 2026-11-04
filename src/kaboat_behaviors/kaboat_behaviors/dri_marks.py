"""dri_marks — DRI 위험도가 문턱을 넘는 구역의 중심을 찾는 순수 라이브러리.

ROS 비의존. RViz 에서 DRI 지도(검정=위험)만으로는 문턱을 넘었는지 한눈에 알기
어려워, 문턱을 넘는 **연결된 구역마다 중심 한 점**을 구해 빨간 X 로 표시하는 데
쓴다. 표시 전용이며 판단·계획에는 쓰지 않는다.

문턱은 DRI **원값**(0 ~ 약 3.4)과 비교한다. RViz 지도에 보내는 0~100 값은
`dri / viz_dri_scale * 100` 으로 접은 표시용 값이라 비교 대상이 아니다.
"""
from dataclasses import dataclass

import numpy as np
from scipy import ndimage


@dataclass(frozen=True)
class RiskRegion:
    x: float        # 구역 중심 world 좌표 [m]
    y: float
    cells: int      # 문턱을 넘는 셀 수
    peak: float     # 구역 안 최대 DRI


def high_risk_regions(dri, threshold: float, min_cells: int = 1,
                      limit: int = 100) -> list[RiskRegion]:
    """DRI > threshold 인 8-연결 구역마다 중심(셀 중심들의 평균)을 돌려준다.

    dri       dri.DriGrid (data [iy, ix], anchor, resolution)
    limit     표시 폭주 방지 — 최대 DRI 가 큰 순서로 이만큼만 남긴다.
    """
    data = np.asarray(dri.data)
    mask = data > threshold            # NaN 은 False
    if not mask.any():
        return []
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=bool))
    index = np.arange(1, count + 1)
    centers = ndimage.center_of_mass(mask, labels, index)    # (row, col) 소수 인덱스
    sizes = ndimage.sum(mask, labels, index)
    peaks = ndimage.maximum(data, labels, index)
    res = float(dri.resolution)
    ax, ay = dri.anchor
    regions = [
        RiskRegion((ax + col + 0.5) * res, (ay + row + 0.5) * res,
                   int(n), float(peak))
        for (row, col), n, peak in zip(centers, sizes, peaks) if n >= min_cells
    ]
    regions.sort(key=lambda r: r.peak, reverse=True)
    return regions[:max(0, int(limit))]


def x_segments(regions, size: float) -> list:
    """구역마다 X 모양을 이루는 두 대각선 ((x1, y1), (x2, y2)) 목록."""
    half = size / 2.0
    segments = []
    for r in regions:
        segments.append(((r.x - half, r.y - half), (r.x + half, r.y + half)))
        segments.append(((r.x - half, r.y + half), (r.x + half, r.y - half)))
    return segments
