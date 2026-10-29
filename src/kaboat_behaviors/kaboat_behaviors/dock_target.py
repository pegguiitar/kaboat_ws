"""Validate a fresh boat-local dock entrance before it reaches the FSM."""

import math

from .docking_fsm import DockTarget


def target_from_entrance(x: float, y: float, z: float,
                         *, max_range: float = 12.0,
                         min_forward_x: float = 0.0) -> DockTarget | None:
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    if abs(z) > 0.05 or x <= min_forward_x:
        return None
    distance = math.hypot(x, y)
    if distance <= 0.0 or distance > max_range:
        return None
    return DockTarget(bearing=math.atan2(y, x), distance=distance)
