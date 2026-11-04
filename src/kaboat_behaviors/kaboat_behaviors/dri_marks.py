"""dri_marks — DRI 위험도가 문턱을 넘는 격자 셀을 찾는 순수 라이브러리.

ROS 비의존. RViz 에서 DRI 지도(검정=위험)만으로는 문턱을 넘었는지 한눈에 알기
어려워, 문턱을 넘는 **셀마다**(격자 한 칸, 수조 시험 0.2 m) 그 셀 중심에 빨간 X 를
찍는 데 쓴다. 구역으로 묶어 평균점에 찍으면 벽처럼 이어진 구역에서는 평균점이
빈 공간에 놓이므로 셀 단위로 표시한다. 표시 전용이며 판단·계획에는 쓰지 않는다.

문턱은 DRI **원값**(0 ~ 약 3.4)과 비교한다. RViz 지도에 보내는 0~100 값은
`dri / viz_dri_scale * 100` 으로 접은 표시용 값이라 비교 대상이 아니다.
"""
from dataclasses import dataclass

import numpy as np

MAX_MARKED_CELLS = 5000     # 표시 폭주 방지 (15 m 창 = 75x75 = 5625 셀)


@dataclass(frozen=True)
class RiskCell:
    x: float        # 셀 중심 world 좌표 [m]
    y: float
    risk: float     # 그 셀의 DRI


def high_risk_cells(dri, threshold: float,
                    limit: int = MAX_MARKED_CELLS) -> list[RiskCell]:
    """DRI > threshold 인 셀마다 셀 중심 좌표. 위험이 큰 순서로 limit 개까지."""
    data = np.asarray(dri.data)
    iy, ix = np.nonzero(data > threshold)          # NaN 은 False
    if ix.size == 0:
        return []
    risk = data[iy, ix]
    order = np.argsort(-risk, kind='stable')[:max(0, int(limit))]
    res = float(dri.resolution)
    ax, ay = dri.anchor
    return [RiskCell((ax + ix[k] + 0.5) * res, (ay + iy[k] + 0.5) * res,
                     float(risk[k])) for k in order]


def x_segments(cells, size: float) -> list:
    """셀마다 X 모양을 이루는 두 대각선 ((x1, y1), (x2, y2)) 목록."""
    half = size / 2.0
    segments = []
    for c in cells:
        segments.append(((c.x - half, c.y - half), (c.x + half, c.y + half)))
        segments.append(((c.x - half, c.y + half), (c.x + half, c.y - half)))
    return segments
