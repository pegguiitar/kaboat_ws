"""스캔의 직선형 횡단 판에서 수조 기준 선체 중심과 yaw를 추정한다."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Tuple

import numpy as np

from kaboat_hardware.pose_velocity import normalize_angle


@dataclass(frozen=True)
class PanelPose:
    center: np.ndarray
    start: np.ndarray
    end: np.ndarray
    yaw: float
    span: float
    line_rms: float
    point_count: int

    @property
    def x(self):
        return float(self.center[0])

    @property
    def y(self):
        return float(self.center[1])


def _yaw_mod_pi(yaw):
    """판만 보일 때 구분할 수 없는 180° 두 방향을 하나로 정규화한다."""
    return (yaw + math.pi / 2.0) % math.pi - math.pi / 2.0


def _align_yaw_to_previous(yaw, previous_yaw):
    first = normalize_angle(yaw)
    opposite = normalize_angle(first + math.pi)
    if abs(normalize_angle(opposite - previous_yaw)) < abs(
            normalize_angle(first - previous_yaw)):
        return opposite
    return first


def fit_panel_line(
    points,
    min_points: int,
    min_span: float,
    max_span: float,
    max_line_rms: float,
) -> Optional[PanelPose]:
    """한 연속 스캔 클러스터에 직선을 맞추고 횡단 판 후보를 반환한다."""
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < min_points:
        return None
    if not np.all(np.isfinite(pts)):
        return None

    mean = np.mean(pts, axis=0)
    _, _, axes = np.linalg.svd(pts - mean, full_matrices=False)
    tangent, normal = axes
    along = (pts - mean) @ tangent
    across = (pts - mean) @ normal
    span = float(np.max(along) - np.min(along))
    line_rms = float(np.sqrt(np.mean(across ** 2)))
    if not min_span <= span <= max_span or line_rms > max_line_rms:
        return None

    # 관측된 선분의 양 끝 중점을 사용한다. 판의 중점이 base_link 원점이라는
    # 장착 가정에 따라 이 점이 선체 위치가 된다.
    start = mean + tangent * np.min(along)
    end = mean + tangent * np.max(along)
    center = 0.5 * (start + end)

    # 판은 선체 좌우(+Y/-Y)를 잇는다. 선체 +X 헤딩은 판 기울기에서 90° 뺀 방향.
    line_angle = math.atan2(float(tangent[1]), float(tangent[0]))
    yaw = _yaw_mod_pi(line_angle - math.pi / 2.0)
    return PanelPose(
        center=center, start=start, end=end, yaw=yaw, span=span,
        line_rms=line_rms, point_count=len(pts))


def select_panel_line(
    clusters: Iterable[np.ndarray],
    expected_span: float,
    min_points: int,
    min_span: float,
    max_span: float,
    max_line_rms: float,
    previous_pose: Optional[Tuple[float, float, float]] = None,
    max_position_jump: float = math.inf,
    max_yaw_jump: float = math.pi,
) -> Optional[PanelPose]:
    """길이·직선성·이전 pose에 가장 잘 맞는 판을 선택한다."""
    best = None
    best_score = math.inf
    for points in clusters:
        panel = fit_panel_line(
            points, min_points, min_span, max_span, max_line_rms)
        if panel is None:
            continue

        yaw = panel.yaw
        score = abs(panel.span - expected_span) / expected_span
        score += panel.line_rms / max(max_line_rms, 1e-6)
        if previous_pose is not None:
            prev_x, prev_y, prev_yaw = previous_pose
            yaw = _align_yaw_to_previous(yaw, prev_yaw)
            position_jump = math.hypot(panel.x - prev_x, panel.y - prev_y)
            yaw_jump = abs(normalize_angle(yaw - prev_yaw))
            if (position_jump > max_position_jump
                    or yaw_jump > max_yaw_jump):
                continue
            score += position_jump / max(max_position_jump, 1e-6)
            score += yaw_jump / max(max_yaw_jump, 1e-6)

        if score < best_score:
            best_score = score
            best = PanelPose(
                center=panel.center, start=panel.start, end=panel.end,
                yaw=yaw, span=panel.span, line_rms=panel.line_rms,
                point_count=panel.point_count)
    return best
