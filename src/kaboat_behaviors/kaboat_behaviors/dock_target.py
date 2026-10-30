"""Build boat-local dock waypoints from one marker and its mouth point."""

import math

from .docking_fsm import DockPlan, DockTarget


def target_from_point(x: float, y: float, z: float,
                      *, max_range: float = 12.0,
                      min_forward_x: float = 0.0,
                      require_ground: bool = True) -> DockTarget | None:
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    if (require_ground and abs(z) > 0.05) or x <= min_forward_x:
        return None
    distance = math.hypot(x, y)
    if distance <= 0.0 or distance > max_range:
        return None
    return DockTarget(bearing=math.atan2(y, x), distance=distance)


# 기존 호출자와 테스트의 이름을 유지한다.
target_from_entrance = target_from_point


def plan_from_points(marker_x: float, marker_y: float,
                     mouth_x: float, mouth_y: float, *,
                     pre_entry_offset: float,
                     berth_distance: float,
                     max_range: float = 12.0) -> DockPlan | None:
    """W1 outside mouth, W2 mouth, W3 ``berth_distance`` from back wall.

    All coordinates are in current base_link. The vector marker -> mouth points
    outward, so the desired heading into the slot is its opposite direction.
    """
    values = (marker_x, marker_y, mouth_x, mouth_y,
              pre_entry_offset, berth_distance, max_range)
    if not all(math.isfinite(value) for value in values):
        return None
    if pre_entry_offset <= 0 or berth_distance <= 0 or max_range <= 0:
        return None
    dx, dy = mouth_x - marker_x, mouth_y - marker_y
    wall_to_mouth = math.hypot(dx, dy)
    if wall_to_mouth <= berth_distance + 0.2 or wall_to_mouth > 8.0:
        return None
    outward_x, outward_y = dx / wall_to_mouth, dy / wall_to_mouth
    # The selected back wall must face the boat, with marker still forward.
    if marker_x <= 0 or outward_x >= -0.15:
        return None

    pre_x = mouth_x + pre_entry_offset * outward_x
    pre_y = mouth_y + pre_entry_offset * outward_y
    berth_x = marker_x + berth_distance * outward_x
    berth_y = marker_y + berth_distance * outward_y
    targets = [DockTarget(math.atan2(y, x), math.hypot(x, y))
               for x, y in ((pre_x, pre_y), (mouth_x, mouth_y),
                            (berth_x, berth_y), (marker_x, marker_y))]
    # W1/W2/W3가 정확히 현재 선체 위치에 오면 거리 0이 정상이다.
    if targets[-1].distance <= 0 or any(
            target.distance > max_range for target in targets):
        return None
    axis_error = math.atan2(-outward_y, -outward_x)
    return DockPlan(*targets, axis_error)
