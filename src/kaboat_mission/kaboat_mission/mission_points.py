"""미션 지점 파일 — 현장에서 기록한 지점(x, y, 선수각)을 JSON 으로 저장·불러오기 (ROS 무의존).

point_recorder(kaboat_hardware)가 쓰고, mission_manager 가 읽어서 entry_waypoints
([x, y, angle_deg]) / waypoints ([x, y]) 로 쓴다.

파일 형식 (기본 ~/kaboat_mission_points.json):
{
  "version": 1,
  "frame_id": "odom",
  "origin": {"lat": 35.1234567, "lon": 129.0456789, "alt": 12.5},
  "points": {
    "gate_entry": {"x": 1.234, "y": -5.678, "yaw_deg": 90.0,
                   "lat": 35.12346789, "lon": 129.04566012,
                   "recorded_at": "2026-11-10T14:03:12"}
  }
}

- x, y: /odom 좌표 [m] (GQ7 ENU: x = 동쪽, y = 북쪽)
- yaw_deg: 선수각 [deg], 0 = +x, 반시계 + (mission_params.yaml 과 같은 규약)
- origin: 기록 당시 GQ7 고정 원점(~/kaboat_geo_origin.yaml). 없었으면 null.
  원점이 다르면 같은 x/y 가 다른 자리이므로, 한 파일에는 같은 원점의 지점만 담는다.
- lat, lon: 원점이 있을 때 x/y 를 위도·경도로 되돌린 값. 원점을 다시 잡아도
  새 원점 기준 x/y 를 다시 계산할 수 있게 보관한다.

사용 예 (mission_manager):
    store = PointStore.load('~/kaboat_mission_points.json')
    store.check_origin(current_origin)        # 원점이 바뀌었으면 ValueError
    entry = store.get('gate_entry').pose()    # [x, y, yaw_deg]
"""

from dataclasses import dataclass
import json
import math
import os
import re
import tempfile

DEFAULT_POINTS_FILE = '~/kaboat_mission_points.json'
FORMAT_VERSION = 1
FRAME_ID = 'odom'
# 같은 원점으로 볼 거리 [m]. 원점 파일은 GNSS 평균이라 다시 잡으면 이보다 크게 바뀐다.
ORIGIN_TOLERANCE_M = 0.5
NAME_RE = re.compile(r'^[\w\-]{1,64}$')   # \w 는 한글 포함


@dataclass(frozen=True)
class MissionPoint:
    x: float
    y: float
    yaw_deg: float
    lat: float | None = None
    lon: float | None = None
    recorded_at: str = ''

    def pose(self) -> list:
        """entry_waypoints 형식 [x, y, angle_deg]."""
        return [self.x, self.y, self.yaw_deg]

    def xy(self) -> list:
        """waypoints 형식 [x, y]."""
        return [self.x, self.y]


def check_name(name) -> str:
    name = str(name).strip()
    if not NAME_RE.match(name):
        raise ValueError(
            f'지점 이름 {name!r} 을 쓸 수 없음 — 영문·숫자·한글·_·- 1~64자')
    return name


def auto_name(existing) -> str:
    """아직 안 쓴 p1, p2, … 중 첫 번째."""
    n = 1
    while f'p{n}' in existing:
        n += 1
    return f'p{n}'


def origin_distance_m(a, b) -> float:
    """두 원점 (위도, 경도, …) 사이 수평 거리 [m] (경기장 크기면 평면 근사로 충분)."""
    lat = math.radians((a[0] + b[0]) / 2.0)
    dy = math.radians(b[0] - a[0]) * 6371000.0
    dx = math.radians(b[1] - a[1]) * 6371000.0 * math.cos(lat)
    return math.hypot(dx, dy)


def same_origin(a, b, tolerance_m: float = ORIGIN_TOLERANCE_M) -> bool:
    if a is None or b is None:
        return a is None and b is None
    return origin_distance_m(a, b) <= tolerance_m


def _describe(origin) -> str:
    return '없음' if origin is None else f'({origin[0]:.7f}, {origin[1]:.7f})'


def _finite(value, what) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{what} 가 유한한 수가 아님')
    return value


class PointStore:
    """이름 → MissionPoint. 바꿀 때는 새 PointStore 를 돌려준다(저장 실패 시 원본 유지)."""

    def __init__(self, origin=None, points=None):
        self.origin = None if origin is None else tuple(float(v) for v in origin)
        self.points = dict(points or {})

    # ── 읽기 ─────────────────────────────────────────────
    def get(self, name) -> MissionPoint:
        try:
            return self.points[name]
        except KeyError:
            known = ', '.join(self.points) or '없음'
            raise KeyError(f'지점 {name!r} 이 없음 (저장된 지점: {known})') from None

    def check_origin(self, current_origin, tolerance_m: float = ORIGIN_TOLERANCE_M):
        """지금 원점이 기록 당시 원점과 다르면 ValueError (지점이 없으면 통과)."""
        if self.points and not same_origin(self.origin, current_origin, tolerance_m):
            raise ValueError(
                f'원점이 다름 — 파일 {_describe(self.origin)}, 지금 {_describe(current_origin)}. '
                '이 파일의 x/y 는 지금 /odom 과 다른 자리다')

    # ── 쓰기 ─────────────────────────────────────────────
    def with_point(self, name, point: MissionPoint, origin):
        """name 에 point 를 넣은 새 저장소와 덮어쓴 이전 지점(없으면 None)."""
        name = check_name(name)
        self.check_origin(origin)
        points = dict(self.points)
        previous = points.get(name)
        points[name] = point
        return PointStore(origin if not self.points else self.origin, points), previous

    # ── JSON ─────────────────────────────────────────────
    def to_dict(self) -> dict:
        origin = None
        if self.origin is not None:
            origin = {'lat': self.origin[0], 'lon': self.origin[1],
                      'alt': self.origin[2] if len(self.origin) > 2 else 0.0}
        return {
            'version': FORMAT_VERSION,
            'frame_id': FRAME_ID,
            'origin': origin,
            'points': {
                name: {'x': p.x, 'y': p.y, 'yaw_deg': p.yaw_deg,
                       'lat': p.lat, 'lon': p.lon, 'recorded_at': p.recorded_at}
                for name, p in self.points.items()
            },
        }

    @classmethod
    def from_dict(cls, data) -> 'PointStore':
        if not isinstance(data, dict):
            raise ValueError('최상위가 JSON 객체가 아님')
        if data.get('version') != FORMAT_VERSION:
            raise ValueError(f"version {data.get('version')!r} 은 모름 (지원: {FORMAT_VERSION})")
        if data.get('frame_id') != FRAME_ID:
            raise ValueError(f"frame_id 가 {FRAME_ID!r} 가 아님: {data.get('frame_id')!r}")
        raw_origin = data.get('origin')
        origin = None
        if raw_origin is not None:
            origin = (_finite(raw_origin['lat'], 'origin.lat'),
                      _finite(raw_origin['lon'], 'origin.lon'),
                      _finite(raw_origin.get('alt', 0.0), 'origin.alt'))
        raw_points = data.get('points') or {}
        if not isinstance(raw_points, dict):
            raise ValueError('points 가 JSON 객체가 아님')
        points = {}
        for name, raw in raw_points.items():
            name = check_name(name)
            lat, lon = raw.get('lat'), raw.get('lon')
            points[name] = MissionPoint(
                x=_finite(raw['x'], f'{name}.x'),
                y=_finite(raw['y'], f'{name}.y'),
                yaw_deg=_finite(raw['yaw_deg'], f'{name}.yaw_deg'),
                lat=None if lat is None else _finite(lat, f'{name}.lat'),
                lon=None if lon is None else _finite(lon, f'{name}.lon'),
                recorded_at=str(raw.get('recorded_at', '')))
        return cls(origin, points)

    @classmethod
    def load(cls, path: str = DEFAULT_POINTS_FILE) -> 'PointStore':
        """파일이 없으면 빈 저장소. 내용이 잘못됐으면 ValueError (덮어쓰지 않도록)."""
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            return cls()
        with open(path, encoding='utf-8') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError as exc:
                raise ValueError(f'JSON 이 깨짐: {exc}') from exc
        try:
            return cls.from_dict(data)
        except (KeyError, TypeError, AttributeError) as exc:
            raise ValueError(f'형식이 잘못됨: {exc!r}') from exc

    def save(self, path: str = DEFAULT_POINTS_FILE):
        """다 쓴 뒤 이름을 바꿔서, 저장 도중 꺼져도 이전 파일이 남는다."""
        path = os.path.expanduser(path)
        directory = os.path.dirname(path) or '.'
        os.makedirs(directory, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix='.mission_points_', suffix='.json', dir=directory)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)
                f.write('\n')
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise
