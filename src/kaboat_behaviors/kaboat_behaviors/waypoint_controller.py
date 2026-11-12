"""Shared odom-frame point-to-point controller with optional final heading.

``waypoint(x, y, angle=None, thrust_ratio=1.0)`` takes metres, optional
degrees and a 0..1 multiplier of the configured thrust limits. The caller
decides which command topic to publish. With a heading, arrival requires
both position and heading; without one, only position is checked.

2026-11-09 개선 (목표 각도에서 APPROACH ↔ ALIGN 을 오가며 맴돌던 문제):
① 히스테리시스 — 도착 반경(position_tolerance)에 한 번 들어오면 그 exit_factor 배를
   벗어나기 전까지는 다시 접근하지 않는다. 경계에서 접근(목표점 쪽 회전)과 정렬(목표 각도
   쪽 회전)이 반대로 돌리며 번갈아 켜지던 원인.
② 도착 반경 exit_factor 배 안에서 전진을 거리 비례로 더 줄여 관성으로 미끄러져 나가지 않게.
③ 정착(SETTLING) 중에도 헤딩 PD 를 유지하고, 회전 속도가 settle_yaw_rate 이하일 때만
   도착으로 본다(예전엔 명령 0 이라 남은 회전 관성으로 허용 범위를 넘어 ALIGN 으로 되돌아감).
⑤ thrust_ratio 는 출력 **상한**(max_linear·max_angular × ratio)에만 적용한다. 예전엔 PD 출력
   전체에 곱해서 비율이 낮으면 회전 게인까지 같이 작아졌다(LOS 에서 고친 것과 같은 문제).
"""

from dataclasses import dataclass
import math


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    angle_deg: float | None = None
    thrust_ratio: float = 1.0

    @property
    def yaw(self) -> float | None:
        return (None if self.angle_deg is None
                else _wrap(math.radians(self.angle_deg)))


def waypoint(x: float, y: float, angle: float | None = None,
             thrust_ratio: float = 1.0) -> Waypoint:
    """Create a target: X/Y metres, optional heading degrees, thrust 0..1.

    0 degrees faces +X, 90 degrees faces +Y, and positive rotation is CCW.
    Bounds are checked by the indoor-tank ROS node, not by this reusable API.
    """
    target_angle = None if angle is None else float(angle)
    thrust_ratio = float(thrust_ratio)
    values = (float(x), float(y))
    if not all(math.isfinite(value) for value in values) or (
            target_angle is not None and not math.isfinite(target_angle)):
        raise ValueError('waypoint x, y and angle must be finite')
    if not math.isfinite(thrust_ratio) or not 0.0 <= thrust_ratio <= 1.0:
        raise ValueError('thrust_ratio must be between 0 and 1')
    return Waypoint(*values, target_angle, thrust_ratio)


@dataclass(frozen=True)
class WaypointParams:
    max_linear: float = 12.0 / 118.6
    max_angular: float = 3.0 / 118.6
    yaw_kp: float = 18.0 / 118.6
    yaw_kd: float = 9.0 / 118.6
    slow_radius: float = 1.5
    position_tolerance: float = 0.35
    heading_tolerance_deg: float = 8.0
    settle_time: float = 0.5
    exit_factor: float = 2.0        # ① 도착 후 이 배수의 반경을 벗어나야 다시 접근
    settle_yaw_rate: float = 0.1    # ③ 도착 판정 시 회전 속도 상한 [rad/s], 0 = 조건 없음

    def __post_init__(self):
        values = tuple(vars(self).values())
        if not all(math.isfinite(value) for value in values):
            raise ValueError('waypoint controller parameters must be finite')
        if (not 0 < self.max_linear <= 1 or not 0 < self.max_angular <= 1
                or self.yaw_kp <= 0 or self.yaw_kd < 0
                or self.slow_radius <= 0 or self.position_tolerance <= 0
                or not 0 < self.heading_tolerance_deg < 180
                or self.settle_time < 0 or self.exit_factor < 1.0
                or self.settle_yaw_rate < 0):
            raise ValueError('invalid waypoint controller parameters')


@dataclass(frozen=True)
class WaypointStep:
    linear: float = 0.0
    angular: float = 0.0
    phase: str = 'WAIT_GOAL'
    reached: bool = False
    distance: float = math.inf
    heading_error: float = 0.0


class WaypointController:
    def __init__(self, params: WaypointParams | None = None):
        self.params = params or WaypointParams()
        self.target: Waypoint | None = None
        self._inside_since: float | None = None
        self._arrived = False          # ① 도착 반경 안에 들어온 뒤 exit 반경 전까지 유지

    def reset(self):
        self.target = None
        self._inside_since = None
        self._arrived = False

    def set_goal(self, target: Waypoint):
        if target != self.target:
            self.target = target
            self._inside_since = None
            self._arrived = False

    def reset_progress(self):
        """Require a fresh settling interval after a sensor/safety interruption."""
        self._inside_since = None

    def step(self, x: float, y: float, yaw: float, yaw_rate: float,
             now: float, *, slow_radius: float | None = None) -> WaypointStep:
        target = self.target
        if target is None:
            return WaypointStep()
        if not all(math.isfinite(v) for v in (x, y, yaw, yaw_rate, now)):
            self._inside_since = None
            return WaypointStep(phase='INVALID_ODOM')

        p = self.params
        radius = p.slow_radius if slow_radius is None else float(slow_radius)
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError('slow_radius must be positive and finite')
        dx, dy = target.x - x, target.y - y
        distance = math.hypot(dx, dy)
        exit_radius = p.position_tolerance * p.exit_factor
        if self._arrived and distance > exit_radius:
            self._arrived = False
        if not self._arrived and distance <= p.position_tolerance:
            self._arrived = True
        # ⑤ 비율은 출력 상한에만. PD 게인은 비율과 무관.
        linear_cap = p.max_linear * target.thrust_ratio
        angular_cap = p.max_angular * target.thrust_ratio

        if not self._arrived:
            self._inside_since = None
            bearing_error = _wrap(math.atan2(dy, dx) - yaw)
            angular = _clamp(
                p.yaw_kp * bearing_error - p.yaw_kd * yaw_rate, angular_cap)
            linear = linear_cap * min(distance / radius, 1.0)
            if target.yaw is None:
                # Preserve the former seek_goal approach for position-only
                # missions: low forward thrust while turning, never pivot.
                if abs(bearing_error) >= math.pi / 3:
                    linear = linear_cap * 0.3
            elif abs(bearing_error) >= math.pi / 2:
                linear = 0.0
            elif abs(bearing_error) >= math.pi / 3:
                linear *= 0.2
            # ② 도착 반경 exit_factor 배 안에서는 거리에 비례해 더 줄인다
            linear *= min(1.0, distance / exit_radius)
            return WaypointStep(linear, angular, 'APPROACH', False,
                                distance, bearing_error)

        if target.yaw is None:
            return self._settle(now, distance, 0.0, 0.0)

        final_error = _wrap(target.yaw - yaw)
        # ③ 정렬·정착 내내 헤딩 PD 유지 (정착 중 명령 0 이면 회전 관성으로 다시 벗어남)
        angular = _clamp(
            p.yaw_kp * final_error - p.yaw_kd * yaw_rate, angular_cap)
        if (abs(final_error) > math.radians(p.heading_tolerance_deg)
                or 0 < p.settle_yaw_rate < abs(yaw_rate)):
            self._inside_since = None
            return WaypointStep(0.0, angular, 'ALIGN', False,
                                distance, final_error)
        return self._settle(now, distance, final_error, angular)

    def _settle(self, now: float, distance: float,
                heading_error: float, angular: float) -> WaypointStep:
        if self._inside_since is None or now < self._inside_since:
            self._inside_since = now
        reached = now - self._inside_since >= self.params.settle_time
        return WaypointStep(angular=angular,
                            phase='REACHED' if reached else 'SETTLING',
                            reached=reached, distance=distance,
                            heading_error=heading_error)
