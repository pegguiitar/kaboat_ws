"""선수/선미 LiDAR 표식으로부터 선체 2D pose를 계산하는 ROS 비의존 유틸리티."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

from kaboat_hardware.pose_velocity import normalize_angle


@dataclass(frozen=True)
class MarkerCandidate:
    center: np.ndarray
    diameter: float
    point_count: int


@dataclass(frozen=True)
class MarkerPairResult:
    bow: MarkerCandidate
    stern: MarkerCandidate
    x: float
    y: float
    yaw: float
    separation: float


def estimate_base_pose(
    bow_world: Sequence[float],
    stern_world: Sequence[float],
    bow_body: Sequence[float],
    stern_body: Sequence[float],
) -> Tuple[float, float, float]:
    """알려진 두 표식의 선체/월드 좌표로 base_link의 (x, y, yaw)를 계산한다."""
    bow_w = np.asarray(bow_world, dtype=float)
    stern_w = np.asarray(stern_world, dtype=float)
    bow_b = np.asarray(bow_body, dtype=float)
    stern_b = np.asarray(stern_body, dtype=float)

    world_delta = bow_w - stern_w
    body_delta = bow_b - stern_b
    if np.linalg.norm(world_delta) <= 1e-9 or np.linalg.norm(body_delta) <= 1e-9:
        raise ValueError('선수/선미 표식 간격은 0보다 커야 합니다')

    yaw = normalize_angle(
        math.atan2(world_delta[1], world_delta[0])
        - math.atan2(body_delta[1], body_delta[0]))
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])

    # 두 표식 각각으로 계산한 base_link 원점을 평균해 점 위치 노이즈를 줄인다.
    base_from_bow = bow_w - rotation @ bow_b
    base_from_stern = stern_w - rotation @ stern_b
    base = 0.5 * (base_from_bow + base_from_stern)
    return float(base[0]), float(base[1]), yaw


def select_marker_pair(
    candidates: Iterable[MarkerCandidate],
    bow_diameter_range: Tuple[float, float],
    stern_diameter_range: Tuple[float, float],
    bow_body: Sequence[float],
    stern_body: Sequence[float],
    separation_tolerance: float,
    previous_pose: Optional[Tuple[float, float, float]] = None,
    max_position_jump: float = math.inf,
    max_yaw_jump: float = math.pi,
) -> Optional[MarkerPairResult]:
    """직경과 설치 간격에 맞는 선수(얇음)/선미(두꺼움) 표식 쌍을 선택한다."""
    items = list(candidates)
    expected_separation = float(np.linalg.norm(
        np.asarray(bow_body, dtype=float) - np.asarray(stern_body, dtype=float)))
    if expected_separation <= 0.0:
        raise ValueError('설정된 선수/선미 표식 간격은 0보다 커야 합니다')

    best = None
    best_score = math.inf
    bow_min, bow_max = bow_diameter_range
    stern_min, stern_max = stern_diameter_range

    for bow_index, bow in enumerate(items):
        if not bow_min <= bow.diameter <= bow_max:
            continue
        for stern_index, stern in enumerate(items):
            if bow_index == stern_index:
                continue
            if not stern_min <= stern.diameter <= stern_max:
                continue

            separation = float(np.linalg.norm(bow.center - stern.center))
            separation_error = abs(separation - expected_separation)
            if separation_error > separation_tolerance:
                continue

            try:
                x, y, yaw = estimate_base_pose(
                    bow.center, stern.center, bow_body, stern_body)
            except ValueError:
                continue

            score = separation_error / max(separation_tolerance, 1e-6)
            if previous_pose is not None:
                prev_x, prev_y, prev_yaw = previous_pose
                position_jump = math.hypot(x - prev_x, y - prev_y)
                yaw_jump = abs(normalize_angle(yaw - prev_yaw))
                if position_jump > max_position_jump or yaw_jump > max_yaw_jump:
                    continue
                score += position_jump / max(max_position_jump, 1e-6)
                score += yaw_jump / max(max_yaw_jump, 1e-6)

            if score < best_score:
                best_score = score
                best = MarkerPairResult(
                    bow=bow, stern=stern, x=x, y=y, yaw=yaw,
                    separation=separation)

    return best
