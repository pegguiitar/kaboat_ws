"""geo_origin 노드: 원점 설정·저장·불러오기, 배·목표 위치 변환."""

import os

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger
import yaml

from kaboat_hardware.geo_local import LocalFrame
from kaboat_hardware.geo_origin import GeoOrigin

LAT0, LON0 = 35.1234567, 129.0456789


def _fix(lat, lon, status=NavSatStatus.STATUS_FIX):
    m = NavSatFix()
    m.latitude, m.longitude, m.altitude = lat, lon, 5.0
    m.status.status = status
    return m


def _node(tmp_path, **params):
    overrides = [Parameter('origin_file', Parameter.Type.STRING,
                           str(tmp_path / 'origin.yaml'))]
    for name, value in params.items():
        overrides.append(Parameter(name, value=value))
    node = GeoOrigin(parameter_overrides=overrides)
    node.sent = {'position': [], 'target_local': []}
    node.position_pub.publish = node.sent['position'].append
    node.target_local_pub.publish = node.sent['target_local'].append
    return node


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_set_origin_averages_fixes_saves_and_reloads(ros, tmp_path):
    node = _node(tmp_path, origin_samples=4)
    try:
        assert node.status_text().startswith('WAIT_ORIGIN')
        resp = node._on_set_origin(Trigger.Request(), Trigger.Response())
        assert not resp.success                       # GNSS 없음
        node._on_fix(_fix(LAT0, LON0, NavSatStatus.STATUS_NO_FIX))   # no fix 는 무시
        for d in (-1e-6, 1e-6, -1e-6, 1e-6):           # 원점 주변 ±10 cm 흔들림
            node._on_fix(_fix(LAT0 + d, LON0))
        resp = node._on_set_origin(Trigger.Request(), Trigger.Response())
        assert resp.success, resp.message
        assert node.frame.origin[:2] == pytest.approx((LAT0, LON0), abs=1e-9)
        saved = yaml.safe_load(open(tmp_path / 'origin.yaml'))
        assert saved['origin_lat'] == pytest.approx(LAT0) and saved['samples'] == 4
    finally:
        node.destroy_node()
    again = _node(tmp_path)                           # 다음 실행: 파일에서 원점
    try:
        assert again.frame is not None and 'file' in again.origin_source
        assert again.frame.origin[:2] == pytest.approx((LAT0, LON0), abs=1e-9)
    finally:
        again.destroy_node()


def test_parameter_origin_wins_and_positions_are_published(ros, tmp_path):
    ref = LocalFrame(LAT0, LON0)
    lat, lon, _ = ref.to_llh(12.0, -3.5)
    node = _node(tmp_path, origin_lat=LAT0, origin_lon=LON0,
                 targets=['dock,%.9f,%.9f' % (lat, lon)])
    try:
        assert node.origin_source == 'parameter'
        node._on_fix(_fix(lat, lon))
        p = node.sent['position'][-1]
        assert p.header.frame_id == 'geo_local'
        assert (p.point.x, p.point.y) == pytest.approx((12.0, -3.5), abs=1e-3)
        assert node.status_text().startswith('OK')
        name, x, y = node.local_targets()[0]
        assert name == 'dock' and (x, y) == pytest.approx((12.0, -3.5), abs=1e-3)
        node._on_target_llh(_fix(lat, lon))
        t = node.sent['target_local'][-1]
        assert (t.point.x, t.point.y) == pytest.approx((12.0, -3.5), abs=1e-3)
        assert not os.path.exists(tmp_path / 'origin.yaml')   # 파라미터 원점은 저장 안 함
    finally:
        node.destroy_node()


def test_status_right_after_set_origin_does_not_crash(ros, tmp_path):
    """원점 설정 직후 다음 GNSS 전에 상태 타이머가 돌아도 죽지 않는다 (실제 런치에서 발견)."""
    node = _node(tmp_path, origin_samples=2)
    try:
        node._on_fix(_fix(LAT0, LON0))
        node._on_fix(_fix(LAT0, LON0))
        assert node._on_set_origin(Trigger.Request(), Trigger.Response()).success
        assert node.status_text().startswith('OK')
        node._publish_status()
    finally:
        node.destroy_node()


def test_set_origin_refuses_while_moving(ros, tmp_path):
    ref = LocalFrame(LAT0, LON0)
    node = _node(tmp_path, origin_samples=4)
    try:
        for east in (0.0, 2.0, 4.0, 6.0):                 # 6 m 이동 중
            lat, lon, _ = ref.to_llh(east, 0.0)
            node._on_fix(_fix(lat, lon))
        resp = node._on_set_origin(Trigger.Request(), Trigger.Response())
        assert not resp.success and '흩어짐' in resp.message
        assert node.frame is None
    finally:
        node.destroy_node()
