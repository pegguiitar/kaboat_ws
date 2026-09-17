"""trajectory_visualization.py — 실시간 주행 궤적 기록 및 RViz 비교 시각화 유틸리티.

ROS 의존성이 없는 순수 궤적 이력 관리 클래스(BoundedTrajectoryHistory)와
RViz2 시각화용 Marker 생성 헬퍼 함수들을 제공합니다.
"""

from collections import deque
import math
from typing import Deque, Iterator, List, Optional, Sequence, Tuple


# ── 기본 시각화 설정 상수 ───────────────────────────────────────
DEFAULT_MAX_POINTS: int = 1500
DEFAULT_MIN_DISTANCE: float = 0.05  # 5cm 거리 기반 다운샘플링
TRAIL_NAMESPACE: str = "actual_trajectory"
TRAIL_MARKER_ID: int = 0
TRAIL_COLOR: Tuple[float, float, float, float] = (1.0, 0.45, 0.0, 0.95)  # 선명한 오렌지 (목표 녹색/청록색과 대비)
TRAIL_LINE_WIDTH: float = 0.04
GOAL_TOLERANCE_NAMESPACE: str = "goal_tolerance"
GOAL_TOLERANCE_MARKER_ID: int = 0
GOAL_TOLERANCE_COLOR: Tuple[float, float, float, float] = (1.0, 0.85, 0.1, 0.75)  # 노란색


class BoundedTrajectoryHistory:
    """ROS에 독립적인 고정 크기(Bounded) 실시간 주행 궤적 기록기.

    주요 동작:
      - 첫 번째 유효 좌표는 거리 조건 없이 즉시 기록합니다.
      - 이후 좌표는 이전 기록 지점과의 2D 유클리드 거리가 min_distance 이상일 때만 기록(다운샘플링).
      - 최대 용량(max_points) 도달 시 가장 오래된 지점을 자동으로 폐기(O(1))하여
        정점 유지(Station Keeping) 등 장시간 운용 시에도 메모리 상한을 보장합니다.
    """

    def __init__(self, max_points: int = DEFAULT_MAX_POINTS, min_distance: float = DEFAULT_MIN_DISTANCE):
        if max_points < 1:
            raise ValueError("max_points는 1 이상이어야 합니다.")
        if min_distance < 0.0:
            raise ValueError("min_distance는 0.0 이상이어야 합니다.")
        self.max_points = max_points
        self.min_distance = min_distance
        self._points: Deque[Tuple[float, float, float]] = deque(maxlen=max_points)

    def add_point(self, x: float, y: float, z: float = 0.04) -> bool:
        """좌표 추가를 시도합니다.

        기록 성공 시 True, min_distance 미만으로 무시된 경우 False를 반환합니다.
        """
        fx = float(x)
        fy = float(y)
        fz = float(z)

        if len(self._points) == 0:
            self._points.append((fx, fy, fz))
            return True

        last_x, last_y, _ = self._points[-1]
        dist = math.hypot(fx - last_x, fy - last_y)
        if dist >= self.min_distance:
            self._points.append((fx, fy, fz))
            return True

        return False

    def clear(self) -> None:
        """기록된 궤적을 모두 제거합니다."""
        self._points.clear()

    def reset(self) -> None:
        """clear()의 별칭."""
        self.clear()

    @property
    def points(self) -> List[Tuple[float, float, float]]:
        """기록된 점들의 리스트 복사본 반환."""
        return list(self._points)

    @property
    def last_point(self) -> Optional[Tuple[float, float, float]]:
        """가장 최근에 기록된 점 반환 (없으면 None)."""
        if self._points:
            return self._points[-1]
        return None

    def is_empty(self) -> bool:
        """기록된 점이 없는지 여부 반환."""
        return len(self._points) == 0

    def __len__(self) -> int:
        return len(self._points)

    def __iter__(self) -> Iterator[Tuple[float, float, float]]:
        return iter(self._points)

    def __getitem__(self, index: int) -> Tuple[float, float, float]:
        return self._points[index]


def create_trail_marker(
    points: Sequence[Tuple[float, float, float]],
    frame_id: str = "odom",
    stamp=None,
    ns: str = TRAIL_NAMESPACE,
    marker_id: int = TRAIL_MARKER_ID,
    color: Tuple[float, float, float, float] = TRAIL_COLOR,
    line_width: float = TRAIL_LINE_WIDTH,
):
    """실제 주행 궤적을 나타내는 visualization_msgs/Marker LINE_STRIP 메시지를 생성합니다."""
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point

    marker = Marker()
    if stamp is not None:
        marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = float(line_width)

    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(color[3])

    pts = []
    for pt in points:
        p = Point()
        p.x = float(pt[0])
        p.y = float(pt[1])
        p.z = float(pt[2]) if len(pt) > 2 else 0.04
        pts.append(p)

    # 점이 1개뿐인 경우에도 RViz에서 표시되도록 시작점을 복제
    if len(pts) == 1:
        pts = [pts[0], pts[0]]

    marker.points = pts
    return marker


def create_goal_tolerance_marker(
    center_x: float,
    center_y: float,
    radius: float,
    frame_id: str = "odom",
    stamp=None,
    ns: str = GOAL_TOLERANCE_NAMESPACE,
    marker_id: int = GOAL_TOLERANCE_MARKER_ID,
    color: Tuple[float, float, float, float] = GOAL_TOLERANCE_COLOR,
    line_width: float = 0.03,
    num_segments: int = 36,
    z: float = 0.03,
):
    """도착 판정 허용 오차 반경을 나타내는 원형 링(LINE_STRIP) 마커를 생성합니다."""
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point

    marker = Marker()
    if stamp is not None:
        marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = float(line_width)

    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(color[3])

    pts = []
    for i in range(num_segments + 1):
        theta = 2.0 * math.pi * (i % num_segments) / num_segments
        p = Point()
        p.x = float(center_x) + float(radius) * math.cos(theta)
        p.y = float(center_y) + float(radius) * math.sin(theta)
        p.z = float(z)
        pts.append(p)

    marker.points = pts
    return marker
