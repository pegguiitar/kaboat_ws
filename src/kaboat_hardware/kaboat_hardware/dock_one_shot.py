"""GPS-local straight docking route and three-marker readiness checks.

All coordinates in this module are east/north metres, except explicit WGS84
latitude/longitude values.  No spline or ROS dependency is used here.
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


def symmetric_markers(status, *, center_fraction=.08, mirror_fraction=.10,
                      min_confidence=.55):
    """Three distinct YOLO masks, centre target, mirrored outer markers."""
    rays = status.get('sight_rays', {})
    width = status.get('input_info', {}).get('color', {}).get('width', 0)
    if not isinstance(width, (int, float)) or width <= 0:
        return False, 'camera width unavailable'
    if any(name not in rays or rays[name].get('confidence', 0) < min_confidence
           for name in CLASSES):
        return False, 'waiting for three confident marker masks'
    try:
        pixels = {name: float(rays[name]['pixel_uv'][0]) for name in CLASSES}
    except (KeyError, IndexError, TypeError, ValueError):
        return False, 'marker pixel centres unavailable'
    centre = width / 2
    if not (pixels['square'] < pixels['triangle'] < pixels['circle']):
        return False, 'marker order is not square-triangle-circle'
    if abs(pixels['triangle'] - centre) > center_fraction * width:
        return False, 'target marker is off camera centre'
    if abs((pixels['circle'] + pixels['square']) / 2 - centre) > mirror_fraction * width:
        return False, 'outer markers are not symmetric'
    return True, 'ok'


def aligned_normals(status, yaw, *, max_spread_deg=8.0, max_yaw_deg=8.0,
                    max_plane_offset_m=.25):
    """Require three coherent back-wall fits and bow toward their common normal."""
    fits = status.get('direct_normals', {})
    if any(name not in fits or not fits[name].get('valid') for name in CLASSES):
        return False, None, 'waiting for three direct RANSAC normals'
    vectors = np.asarray([fits[name]['normal_odom'] for name in CLASSES], float)
    if vectors.shape != (3, 2) or not np.isfinite(vectors).all():
        return False, None, 'invalid RANSAC normal'
    lengths = np.linalg.norm(vectors, axis=1)
    if np.min(lengths) < .9:
        return False, None, 'invalid RANSAC normal length'
    vectors = vectors / lengths[:, None]
    mean = np.mean(vectors, axis=0)
    if np.linalg.norm(mean) < .95:
        return False, None, 'RANSAC normals disagree'
    outward = mean / np.linalg.norm(mean)
    spread = max(math.degrees(math.acos(float(np.clip(v @ outward, -1, 1))))
                 for v in vectors)
    heading = math.atan2(-outward[1], -outward[0])
    if spread > max_spread_deg:
        return False, heading, 'RANSAC normals are not stable together'
    if all('wall_center_odom' in fits[name] for name in CLASSES):
        centers = np.asarray([fits[name]['wall_center_odom'] for name in CLASSES], float)
        if centers.shape != (3, 2) or not np.isfinite(centers).all():
            return False, heading, 'invalid RANSAC wall centres'
        planes = centers @ outward
        if float(np.ptp(planes)) > max_plane_offset_m:
            return False, heading, 'RANSAC back walls are not collinear'
    if abs(math.degrees(wrap(heading - yaw))) > max_yaw_deg:
        return False, heading, 'bow not parallel to wall normals'
    return True, heading, 'ok'


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
