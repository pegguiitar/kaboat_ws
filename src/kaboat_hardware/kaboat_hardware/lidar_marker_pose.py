"""좌현/우현 LiDAR 표식으로부터 선체 2D pose를 계산하는 ROS 비의존 유틸리티."""

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
    thin: MarkerCandidate
    thick: MarkerCandidate
    x: float
    y: float
    yaw: float
    separation: float


def laser_points_to_pool(ranges, angles, lidar_x, lidar_y, lidar_yaw):
    """레이저 극좌표를 수조 절대좌표로 변환한다 (각도는 rad)."""
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    world_angles = angles + lidar_yaw
    return np.column_stack((
        lidar_x + ranges * np.cos(world_angles),
        lidar_y + ranges * np.sin(world_angles),
    ))


def estimate_base_pose(
    thin_world: Sequence[float],
    thick_world: Sequence[float],
    thin_body: Sequence[float],
    thick_body: Sequence[float],
) -> Tuple[float, float, float]:
    """알려진 두 표식의 선체/월드 좌표로 base_link의 (x, y, yaw)를 계산한다."""
    thin_w = np.asarray(thin_world, dtype=float)
    thick_w = np.asarray(thick_world, dtype=float)
    thin_b = np.asarray(thin_body, dtype=float)
    thick_b = np.asarray(thick_body, dtype=float)

    world_delta = thin_w - thick_w
    body_delta = thin_b - thick_b
    if np.linalg.norm(world_delta) <= 1e-9 or np.linalg.norm(body_delta) <= 1e-9:
        raise ValueError('두 표식의 간격은 0보다 커야 합니다')

    yaw = normalize_angle(
        math.atan2(world_delta[1], world_delta[0])
        - math.atan2(body_delta[1], body_delta[0]))
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])

    # 두 표식 각각으로 계산한 base_link 원점을 평균해 점 위치 노이즈를 줄인다.
    base_from_thin = thin_w - rotation @ thin_b
    base_from_thick = thick_w - rotation @ thick_b
    base = 0.5 * (base_from_thin + base_from_thick)
    return float(base[0]), float(base[1]), yaw


def select_marker_pair(
    candidates: Iterable[MarkerCandidate],
    thin_diameter_range: Tuple[float, float],
    thick_diameter_range: Tuple[float, float],
    thin_body: Sequence[float],
    thick_body: Sequence[float],
    separation_tolerance: float,
    previous_pose: Optional[Tuple[float, float, float]] = None,
    max_position_jump: float = math.inf,
    max_yaw_jump: float = math.pi,
) -> Optional[MarkerPairResult]:
    """직경과 설치 간격에 맞는 좌현(얇음)/우현(두꺼움) 표식 쌍을 선택한다."""
    items = list(candidates)
    expected_separation = float(np.linalg.norm(
        np.asarray(thin_body, dtype=float) - np.asarray(thick_body, dtype=float)))
    if expected_separation <= 0.0:
        raise ValueError('설정된 두 표식의 간격은 0보다 커야 합니다')

    best = None
    best_score = math.inf
    thin_min, thin_max = thin_diameter_range
    thick_min, thick_max = thick_diameter_range

    for thin_index, thin in enumerate(items):
        if not thin_min <= thin.diameter <= thin_max:
            continue
        for thick_index, thick in enumerate(items):
            if thin_index == thick_index:
                continue
            if not thick_min <= thick.diameter <= thick_max:
                continue

            separation = float(np.linalg.norm(thin.center - thick.center))
            separation_error = abs(separation - expected_separation)
            if separation_error > separation_tolerance:
                continue

            try:
                x, y, yaw = estimate_base_pose(
                    thin.center, thick.center, thin_body, thick_body)
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
                    thin=thin, thick=thick, x=x, y=y, yaw=yaw,
                    separation=separation)

    return best
