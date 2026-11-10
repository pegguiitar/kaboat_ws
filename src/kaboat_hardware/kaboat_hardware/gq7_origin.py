"""저장된 고정 원점(geo_origin)을 GQ7 드라이버 설정에 넣는다 (ROS 런타임 무의존).

GQ7 의 /ekf/odometry_map 은 위도·경도가 아니라 '상대 위치 원점' 기준 x/y [m] 를 낸다.
그 원점을 장치에 고정 위도·경도로 지정하면(source 1) /odom 이 우리 원점 기준이 된다.

  filter_relative_position_config: true
  filter_relative_position_frame: 2      # 원점을 위도·경도(LLH)로 지정
  filter_relative_position_source: 1     # 아래 고정점 기준
  filter_relative_position_ref: [위도, 경도, 고도]

이 설정은 드라이버가 장치를 설정할 때(device_setup: true)만 적용된다. 드라이버 시작 시
한 번 쓰이므로, 원점을 바꾸면 센서 런치를 다시 켜야 한다.
"""

import math
import os
import tempfile

import yaml

from kaboat_hardware.geo_local import check_lat_lon

# 원점을 합친 GQ7 설정 파일 이름 (임시 디렉터리에 매번 덮어씀)
GENERATED_FILE = 'kaboat_gq7_origin.yaml'


def load_origin_file(path: str):
    """geo_origin 이 저장한 원점 파일 → (위도, 경도, 고도). 파일이 없으면 None."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    lat, lon = float(data['origin_lat']), float(data['origin_lon'])
    alt = float(data.get('origin_alt', 0.0))
    check_lat_lon(lat, lon)
    if not math.isfinite(alt):
        raise ValueError('origin_alt must be finite')
    return lat, lon, alt


def origin_params(lat: float, lon: float, alt: float) -> dict:
    """GQ7 드라이버의 고정 원점 파라미터."""
    return {
        'filter_relative_position_config': True,
        'filter_relative_position_frame': 2,
        'filter_relative_position_source': 1,
        'filter_relative_position_ref': [float(lat), float(lon), float(alt)],
    }


def params_with_origin(base: dict, origin) -> dict:
    """GQ7 설정(YAML dict)의 모든 ros__parameters 에 고정 원점을 덮어쓴 사본."""
    merged = {}
    found = False
    for node, section in base.items():
        section = dict(section or {})
        if isinstance(section.get('ros__parameters'), dict):
            params = dict(section['ros__parameters'])
            if not params.get('device_setup', False):
                raise ValueError(
                    'GQ7 설정이 device_setup: false 라 원점이 장치에 적용되지 않는다 '
                    '(야외용 gq7_outdoor.yaml 을 쓴다)')
            params.update(origin_params(*origin))
            section['ros__parameters'] = params
            found = True
        merged[node] = section
    if not found:
        raise ValueError('GQ7 설정에 ros__parameters 가 없다')
    return merged


def write_params_with_origin(base_path: str, origin, directory: str | None = None) -> str:
    """base_path 설정 + 원점을 합친 YAML 을 쓰고 그 경로를 돌려준다.

    런치를 켤 때마다 파일이 쌓이지 않도록 같은 이름(kaboat_gq7_origin.yaml)에 덮어쓴다.
    다 쓴 뒤 이름을 바꿔서 드라이버가 반쯤 쓴 파일을 읽는 일은 없다.
    """
    with open(os.path.expanduser(base_path), encoding='utf-8') as f:
        base = yaml.safe_load(f) or {}
    merged = params_with_origin(base, origin)
    directory = directory or tempfile.gettempdir()
    path = os.path.join(directory, GENERATED_FILE)
    fd, tmp = tempfile.mkstemp(prefix='.kaboat_gq7_origin_', suffix='.yaml', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            yaml.safe_dump(merged, f, allow_unicode=True, sort_keys=False)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path
