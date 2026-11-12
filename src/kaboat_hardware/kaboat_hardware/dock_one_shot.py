"""GPS-local straight docking route, target-marker sight and target-wall lock checks.

All coordinates in this module are east/north metres, except explicit WGS84
latitude/longitude values.  No spline or ROS dependency is used here.
예전 SCAN(세 마커 대칭 + RANSAC 법선 세 개)은 세 도형이 다 보이는 거리에서 라이다가 거의 안
들어와(2026-11-12 bag) 뺐다 — 목표 도형 방위(target_sight)로 서행하다 ㄷ자 슬롯(dock_u_slot)이
안정되면(stable_wall_lock) 고정.
"""

import math
import numpy as np

CLASSES = ('circle', 'triangle', 'square')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalGeo:
    """WGS84 tangent-plane approximation around a mission-sized area."""

    def __init__(self, latitude, longitude):
        if not (-90 < latitude < 90 and -180 <= longitude <= 180):
            raise ValueError('invalid GPS origin')
        self.latitude, self.longitude = float(latitude), float(longitude)
        phi = math.radians(latitude)
        sin2 = math.sin(phi) ** 2
        a, e2 = 6378137.0, 6.69437999014e-3
        self.east_per_rad = a * math.cos(phi) / math.sqrt(1 - e2 * sin2)
        self.north_per_rad = a * (1 - e2) / (1 - e2 * sin2) ** 1.5

    def xy(self, latitude, longitude):
        if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
            raise ValueError('invalid GPS fix')
        return np.array((math.radians(longitude - self.longitude) * self.east_per_rad,
                         math.radians(latitude - self.latitude) * self.north_per_rad))

    def latlon(self, xy):
        x, y = map(float, xy)
        return (self.latitude + math.degrees(y / self.north_per_rad),
                self.longitude + math.degrees(x / self.east_per_rad))


def parse_marker_order(text):
    """'square,triangle,circle' → 왼쪽부터의 순서 tuple, '' → None (순서 안 봄)."""
    names = tuple(n.strip() for n in str(text or '').split(',') if n.strip())
    if not names:
        return None
    if sorted(names) != sorted(CLASSES):
        raise ValueError(f'marker_order must list {CLASSES} once each, got {text!r}')
    return names


def target_sight(status, target, *, order=None, min_confidence=.55):
    """YOLO 결과에서 목표 도형의 화면 방위 [rad, + = 왼쪽]. 세 슬롯 중 어디로 갈지는 이것만으로 정한다.

    order(화면 왼쪽부터의 도형 순서)를 주면, 같이 보이는 신뢰 마커들의 좌우 순서가 그와
    어긋나는 프레임은 버린다 — 화면 가장자리의 엉뚱한 도형 오검출 거르기.
    반환 (ok, bearing 또는 None, 이유).
    """
    rays = status.get('sight_rays', {}) if isinstance(status, dict) else {}
    ray = rays.get(target)
    if not isinstance(ray, dict) or ray.get('confidence', 0) < min_confidence:
        return False, None, f'waiting for confident {target} mask'
    try:
        bearing = math.radians(float(ray['bearing_left_deg']))
    except (KeyError, TypeError, ValueError):
        return False, None, f'{target} bearing unavailable'
    if not math.isfinite(bearing):
        return False, None, f'{target} bearing unavailable'
    if order is not None:
        try:
            seen = sorted((float(r['pixel_uv'][0]), name) for name, r in rays.items()
                          if name in order and isinstance(r, dict)
                          and r.get('confidence', 0) >= min_confidence)
        except (KeyError, IndexError, TypeError, ValueError):
            return False, None, 'marker pixel centres unavailable'
        names = [name for _, name in seen]
        if names != [name for name in order if name in names]:
            return False, None, f'marker order is not {"-".join(order)}'
    return True, bearing, 'ok'


def stable_bearing(bearings, *, min_samples=5, max_spread_deg=3.0):
    """최근 세계 방위들이 서로 max_spread 안이면 그 원형 평균 [rad]. 반환 (ok, 평균 또는 None, 이유)."""
    values = np.asarray(bearings, float)[-min_samples:]
    if len(values) < min_samples or not np.isfinite(values).all():
        return False, None, f'collecting target bearing {len(values)}/{min_samples}'
    mean = math.atan2(float(np.mean(np.sin(values))), float(np.mean(np.cos(values))))
    spread = max(abs(wrap(v - mean)) for v in values)
    if math.degrees(spread) > max_spread_deg:
        return False, None, f'target bearing unstable ({math.degrees(spread):.1f} deg)'
    return True, mean, 'ok'


def stable_wall_lock(samples, *, min_samples=5, window_s=1.5, max_goal_spread_m=.3,
                     max_normal_spread_deg=8.0, min_span_s=0.0):
    """목표 슬롯 벽이 연속 측정에서 같은 자리·같은 법선이면 진입 기하를 고정.

    samples: 시간순 (stamp_s, wall_xy, outward, approach_xy, goal_xy) — outward 는 벽에서
    물(배) 쪽 단위 법선, approach = 경유점, goal = 정지점 (odom).
    마지막 샘플에서 window_s 안의 샘플 전부를 본다: min_samples 개 이상이고 min_span_s 이상
    이어져야 한다(겹친 누적 스캔은 독립 측정이 아니므로 개수만으로는 부족 — 10 Hz 면 5 개가 0.4 s).
    반환 (ok, lock 또는 None, 이유). lock = wall·outward·via·stop·heading(벽 쪽, 법선과 평행).
    """
    samples = list(samples)
    if not samples:
        return False, None, f'collecting target wall 0/{min_samples}'
    recent = [s for s in samples if s[0] >= samples[-1][0] - window_s]
    if len(recent) < min_samples:
        return False, None, f'collecting target wall {len(recent)}/{min_samples}'
    if recent[-1][0] - recent[0][0] < min_span_s:
        return False, None, f'target wall seen for {recent[-1][0] - recent[0][0]:.1f}/{min_span_s:.1f}s'
    wall, outward, via, stop = (np.asarray([s[i] for s in recent], float) for i in range(1, 5))
    if not all(np.isfinite(a).all() and a.shape == (len(recent), 2)
               for a in (wall, outward, via, stop)):
        return False, None, 'invalid target wall sample'
    lengths = np.linalg.norm(outward, axis=1)
    if np.min(lengths) < .9:
        return False, None, 'invalid target wall normal'
    outward = outward / lengths[:, None]
    mean = outward.mean(axis=0)
    if np.linalg.norm(mean) < .95:
        return False, None, 'target wall normals disagree'
    mean /= np.linalg.norm(mean)
    spread = max(math.degrees(math.acos(float(np.clip(v @ mean, -1, 1)))) for v in outward)
    if spread > max_normal_spread_deg:
        return False, None, f'target wall normal unstable ({spread:.1f} deg)'
    stop_mean = stop.mean(axis=0)
    if float(np.max(np.linalg.norm(stop - stop_mean, axis=1))) > max_goal_spread_m:
        return False, None, 'target stop point unstable'
    return True, dict(wall=wall.mean(axis=0), outward=mean, via=via.mean(axis=0),
                      stop=stop_mean, heading=math.atan2(-mean[1], -mean[0])), 'ok'


def wall_distance(position, wall, outward):
    """위치가 벽 앞(물 쪽)으로 법선 방향 몇 m 떨어졌나."""
    return float((np.asarray(position, float) - np.asarray(wall, float)) @ np.asarray(outward, float))


def straight_samples(start, end, spacing=.1):
    start, end = np.asarray(start, float), np.asarray(end, float)
    if start.shape != (2,) or end.shape != (2,) or not np.isfinite([start, end]).all():
        raise ValueError('route points must be finite XY')
    distance = float(np.linalg.norm(end - start))
    if distance < .05:
        raise ValueError('route segment too short')
    n = max(2, math.ceil(distance / spacing) + 1)
    return np.linspace(start, end, n)


def fixed_route(start, via, stop, spacing=.1):
    """Compute the two straight LOS segments once; no replanning."""
    first = straight_samples(start, via, spacing)
    second = straight_samples(via, stop, spacing)
    vector = np.asarray(stop, float) - np.asarray(via, float)
    heading = math.atan2(vector[1], vector[0])
    return first, second, heading


def retreat_route(stop, heading, distance=3.25, spacing=.1):
    """Fix a straight reverse leg from the dock stop without turning the bow."""
    if not math.isfinite(heading) or not math.isfinite(distance) or distance <= 3.0:
        raise ValueError('retreat distance must exceed 3 m')
    direction = np.array([math.cos(heading), math.sin(heading)])
    exit_xy = np.asarray(stop, float) - distance * direction
    return straight_samples(stop, exit_xy, spacing), exit_xy


def retreat_error(position, stop, heading):
    """Signed metres reversed from stop and lateral displacement from its axis."""
    direction = np.array([math.cos(heading), math.sin(heading)])
    delta = np.asarray(position, float) - np.asarray(stop, float)
    return float(-delta @ direction), float(direction[0] * delta[1] - direction[1] * delta[0])
