"""L2/L3 시험용 가짜 수역과 센서 — ROS 무의존.

수역(World)은 원형 장애물(부표·기둥)과 선분(벽·도킹 구조물)로만 표현한다.
여기서 만드는 값은 메시지 필드와 같은 모양의 순수 데이터이고,
ROS 메시지로의 변환은 ros_adapters.py 가 맡는다.
"""
from dataclasses import dataclass, field
import math
from types import SimpleNamespace

import numpy as np


@dataclass
class Circle:
    x: float
    y: float
    radius: float = 0.2   # [m] 부표 반지름


@dataclass
class Segment:
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass
class World:
    circles: list = field(default_factory=list)
    segments: list = field(default_factory=list)
    bounds: tuple | None = None   # (xmin, ymin, xmax, ymax) 수역 경계 — 안전 판정용

    def min_clearance(self, x: float, y: float) -> float:
        """점 (x, y) 에서 가장 가까운 장애물 표면까지 거리 [m]. 안전 항목 판정용."""
        best = math.inf
        for c in self.circles:
            best = min(best, math.hypot(x - c.x, y - c.y) - c.radius)
        for s in self.segments:
            best = min(best, _point_segment_distance(x, y, s))
        return best

    def inside_bounds(self, x: float, y: float) -> bool:
        if self.bounds is None:
            return True
        xmin, ymin, xmax, ymax = self.bounds
        return xmin <= x <= xmax and ymin <= y <= ymax


def _point_segment_distance(px, py, s: Segment) -> float:
    dx, dy = s.x1 - s.x0, s.y1 - s.y0
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0.0 else max(0.0, min(1.0, ((px - s.x0) * dx + (py - s.y0) * dy) / length2))
    return math.hypot(px - (s.x0 + t * dx), py - (s.y0 + t * dy))


@dataclass
class ScanConfig:
    """선체 라이다 설정. 기본값은 gate_follower 시험과 같은 360°/720빔."""
    beams: int = 720
    angle_min: float = -math.pi
    range_min: float = 0.1
    range_max: float = 25.0

    @property
    def angle_increment(self) -> float:
        return 2.0 * math.pi / self.beams


def scan_ranges(x: float, y: float, yaw: float, world: World,
                cfg: ScanConfig = ScanConfig()) -> np.ndarray:
    """선체 위치에서 world 를 광선 추적한 거리 배열. 맞지 않은 빔은 inf."""
    angles = yaw + cfg.angle_min + cfg.angle_increment * np.arange(cfg.beams)
    dx, dy = np.cos(angles), np.sin(angles)
    ranges = np.full(cfg.beams, np.inf)
    for c in world.circles:
        # |p + t·d − c|² = r² 의 가장 작은 양의 해
        ox, oy = x - c.x, y - c.y
        b = ox * dx + oy * dy
        disc = b * b - (ox * ox + oy * oy - c.radius ** 2)
        hit = disc >= 0.0
        t = -b - np.sqrt(np.where(hit, disc, 0.0))
        ok = hit & (t > 0.0)
        ranges = np.where(ok, np.minimum(ranges, t), ranges)
    for s in world.segments:
        ex, ey = s.x1 - s.x0, s.y1 - s.y0
        denom = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((s.x0 - x) * ey - (s.y0 - y) * ex) / denom
            u = ((s.x0 - x) * dy - (s.y0 - y) * dx) / denom
        ok = (np.abs(denom) > 1e-12) & (t > 0.0) & (u >= 0.0) & (u <= 1.0)
        ranges = np.where(ok, np.minimum(ranges, t), ranges)
    ranges[(ranges < cfg.range_min) | (ranges > cfg.range_max)] = np.inf
    return ranges


def occupancy_grid(world: World, origin=(-10.0, -10.0), resolution: float = 0.2,
                   size: int = 100, occupied: int = 100) -> SimpleNamespace:
    """world 를 칠한 OccupancyGrid 모양 객체 (info.*, data). 관측 안 된 칸은 0(free).

    build_dri()·gate_corridor 는 이 모양을 그대로 받는다. 실제 occupancy_grid
    노드를 거치는 경로는 L2 Avoid 시나리오에서 따로 확인한다.
    """
    data = np.zeros((size, size), dtype=np.int8)
    xs = origin[0] + (np.arange(size) + 0.5) * resolution
    ys = origin[1] + (np.arange(size) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    half_cell = resolution / 2.0
    for c in world.circles:
        data[np.hypot(gx - c.x, gy - c.y) <= c.radius + half_cell] = occupied
    for s in world.segments:
        dist = np.vectorize(lambda px, py: _point_segment_distance(px, py, s))(gx, gy)
        data[dist <= half_cell] = occupied
    info = SimpleNamespace(
        resolution=resolution, width=size, height=size,
        origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1])))
    return SimpleNamespace(info=info, data=data.ravel().tolist())
