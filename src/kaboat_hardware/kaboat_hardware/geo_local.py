"""WGS84 위도·경도 ↔ 고정 원점 기준 로컬 x/y [m] 변환 (ROS 무의존).

원점(위도0, 경도0)에 접하는 평면(ENU: 동·북·위)으로 정확히 변환한다(WGS84 → ECEF → ENU).
경기장 크기(수십~수백 m)에서는 평면 근사 오차가 mm 수준이다.

axis_yaw_deg 로 로컬 x 축 방향을 돌릴 수 있다. 0 이면 x = 동쪽, y = 북쪽(ENU 그대로).
예) 경기장 긴 방향이 동쪽에서 반시계로 30° 이면 axis_yaw_deg = 30 으로 두면 x 가
경기장 긴 방향이 된다. 로컬 좌표의 각도(yaw)도 이 x 축 기준이다.
sensor_check/geodesy.py 와 같은 수식이다.
"""

from dataclasses import dataclass
import math

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def check_lat_lon(lat: float, lon: float):
    if not (math.isfinite(lat) and -90.0 <= lat <= 90.0):
        raise ValueError('latitude must be finite and between -90 and 90 degrees')
    if not (math.isfinite(lon) and -180.0 <= lon <= 180.0):
        raise ValueError('longitude must be finite and between -180 and 180 degrees')


def _ecef(lat: float, lon: float, height: float):
    check_lat_lon(lat, lon)
    if not math.isfinite(height):
        raise ValueError('height must be finite')
    phi, lam = math.radians(lat), math.radians(lon)
    sin_phi, cos_phi = math.sin(phi), math.cos(phi)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)
    return ((radius + height) * cos_phi * math.cos(lam),
            (radius + height) * cos_phi * math.sin(lam),
            (radius * (1.0 - WGS84_E2) + height) * sin_phi)


def _ecef_to_llh(x: float, y: float, z: float):
    """ECEF → (위도, 경도, 타원체고). Bowring 반복 (수 회면 mm 이하로 수렴)."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    height = 0.0
    for _ in range(8):
        sin_lat = math.sin(lat)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        height = p / math.cos(lat) - radius
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * radius / (radius + height)))
    return math.degrees(lat), math.degrees(lon), height


class LocalFrame:
    """고정 원점 기준 로컬 x/y. x 축은 동쪽에서 반시계로 axis_yaw_deg 만큼 돌린 방향."""

    def __init__(self, lat0: float, lon0: float, height0: float = 0.0,
                 axis_yaw_deg: float = 0.0):
        if not math.isfinite(axis_yaw_deg):
            raise ValueError('axis_yaw_deg must be finite')
        self.origin = (float(lat0), float(lon0), float(height0))
        self.axis_yaw_deg = float(axis_yaw_deg)
        self._xyz0 = _ecef(*self.origin)
        phi, lam = math.radians(lat0), math.radians(lon0)
        self._sp, self._cp = math.sin(phi), math.cos(phi)
        self._sl, self._cl = math.sin(lam), math.cos(lam)
        a = math.radians(self.axis_yaw_deg)
        self._ca, self._sa = math.cos(a), math.sin(a)

    def to_enu(self, lat: float, lon: float, height: float | None = None):
        """위도·경도 → (동, 북, 위) [m]. 높이를 안 주면 원점 높이(2D 사용)."""
        xyz = _ecef(lat, lon, self.origin[2] if height is None else height)
        dx, dy, dz = (v - b for v, b in zip(xyz, self._xyz0))
        east = -self._sl * dx + self._cl * dy
        north = -self._sp * self._cl * dx - self._sp * self._sl * dy + self._cp * dz
        up = self._cp * self._cl * dx + self._cp * self._sl * dy + self._sp * dz
        return east, north, up

    def to_local(self, lat: float, lon: float, height: float | None = None):
        """위도·경도 → 로컬 (x, y) [m]."""
        east, north, _ = self.to_enu(lat, lon, height)
        return (east * self._ca + north * self._sa,
                -east * self._sa + north * self._ca)

    def to_llh(self, x: float, y: float, up: float = 0.0):
        """로컬 (x, y) [m] → (위도, 경도, 타원체고)."""
        east = x * self._ca - y * self._sa
        north = x * self._sa + y * self._ca
        dx = (-self._sl * east - self._sp * self._cl * north + self._cp * self._cl * up)
        dy = (self._cl * east - self._sp * self._sl * north + self._cp * self._sl * up)
        dz = self._cp * north + self._sp * up
        return _ecef_to_llh(self._xyz0[0] + dx, self._xyz0[1] + dy, self._xyz0[2] + dz)

    def yaw_to_local(self, enu_yaw: float) -> float:
        """ENU 각도(동쪽 0, 반시계 +) [rad] → 로컬 x 축 기준 각도 [rad]."""
        a = enu_yaw - math.radians(self.axis_yaw_deg)
        return math.atan2(math.sin(a), math.cos(a))


def horizontal_std(covariance):
    """NavSatFix ENU 공분산에서 가장 큰 수평 1σ 오차 [m]. 알 수 없으면 None."""
    if len(covariance) != 9 or not all(math.isfinite(v) for v in covariance):
        return None
    east_var, north_var = covariance[0], covariance[4]
    off = (covariance[1] + covariance[3]) / 2.0
    if east_var <= 0 or north_var <= 0 or east_var * north_var < off ** 2:
        return None
    return math.sqrt((east_var + north_var) / 2.0
                     + math.hypot((east_var - north_var) / 2.0, off))


@dataclass(frozen=True)
class GeoTarget:
    name: str
    lat: float
    lon: float


def parse_targets(entries) -> list[GeoTarget]:
    """['이름,위도,경도', ...] → GeoTarget 목록. 빈 문자열은 건너뛴다."""
    targets = []
    for raw in entries:
        text = str(raw).strip()
        if not text:
            continue
        parts = [p.strip() for p in text.split(',')]
        if len(parts) != 3 or not parts[0]:
            raise ValueError(f'target must be "name,lat,lon": {raw!r}')
        lat, lon = float(parts[1]), float(parts[2])
        check_lat_lon(lat, lon)
        targets.append(GeoTarget(parts[0], lat, lon))
    names = [t.name for t in targets]
    if len(names) != len(set(names)):
        raise ValueError('target names must be unique')
    return targets


def average_llh(samples):
    """[(위도, 경도, 높이), ...] 평균. 경기장 크기에서는 단순 평균으로 충분."""
    if not samples:
        raise ValueError('no samples to average')
    n = float(len(samples))
    return tuple(sum(s[i] for s in samples) / n for i in range(3))
