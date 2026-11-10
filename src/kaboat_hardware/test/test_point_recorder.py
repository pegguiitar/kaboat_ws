"""point_recorder 노드: 지금 /odom 위치를 이름 붙여 JSON 에 기록."""

import json
import math
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
import yaml

from kaboat_hardware.geo_local import LocalFrame
from kaboat_hardware.point_recorder import PointRecorder
from kaboat_msgs.srv import RecordPoint

ORIGIN = (35.1234567, 129.0456789, 12.5)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _odom(x, y, yaw_deg, frame='odom'):
    m = Odometry()
    m.header.frame_id = frame
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    half = math.radians(yaw_deg) / 2.0
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(half), math.cos(half)
    return m


def _write_origin(path, origin=ORIGIN):
    path.write_text(yaml.safe_dump(
        {'origin_lat': origin[0], 'origin_lon': origin[1], 'origin_alt': origin[2]}))


def _node(tmp_path, origin=True):
    if origin:
        _write_origin(tmp_path / 'origin.yaml')
    node = PointRecorder(parameter_overrides=[
        Parameter('points_file', value=str(tmp_path / 'points.json')),
        Parameter('origin_file', value=str(tmp_path / 'origin.yaml')),
    ])
    node.sent = []
    node.markers_pub.publish = node.sent.append
    return node


def _record(node, name=''):
    return node._on_record(RecordPoint.Request(name=name), RecordPoint.Response())


def test_records_current_pose_with_lat_lon_and_reloads(ros, tmp_path):
    node = _node(tmp_path)
    try:
        assert not _record(node, 'a').success                     # /odom 없음
        node._on_odom(_odom(12.3456, -7.8912, 90.0))
        resp = _record(node, 'gate_entry')
        assert resp.success, resp.message
        assert (resp.name, resp.x, resp.y) == ('gate_entry', 12.346, -7.891)
        assert resp.yaw_deg == pytest.approx(90.0)

        saved = json.load(open(tmp_path / 'points.json', encoding='utf-8'))
        point = saved['points']['gate_entry']
        assert point['yaw_deg'] == pytest.approx(90.0)
        assert saved['origin']['lat'] == pytest.approx(ORIGIN[0])
        # 저장한 위도·경도를 같은 원점으로 되돌리면 같은 x/y
        x, y = LocalFrame(*ORIGIN).to_local(point['lat'], point['lon'])
        assert (x, y) == pytest.approx((12.346, -7.891), abs=1e-3)

        auto = _record(node)                                      # 이름 없으면 p1
        assert auto.success and auto.name == 'p1'
        listed = node._on_list(Trigger.Request(), Trigger.Response())
        assert 'gate_entry' in listed.message and 'p1' in listed.message
        # 화살표 + 이름 2개씩, 앞에 DELETEALL
        assert len(node.sent[-1].markers) == 1 + 2 * 2
    finally:
        node.destroy_node()
    again = _node(tmp_path)                                       # 다시 켜도 이어서
    try:
        assert list(again.store.points) == ['gate_entry', 'p1']
    finally:
        again.destroy_node()


def test_overwrite_same_name_reports_previous(ros, tmp_path):
    node = _node(tmp_path)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0))
        assert _record(node, 'dock').success
        node._on_odom(_odom(3.0, 4.0, 180.0))
        resp = _record(node, 'dock')
        assert resp.success and '덮어씀' in resp.message
        assert node.store.get('dock').xy() == pytest.approx([3.0, 4.0])
        assert len(node.store.points) == 1
    finally:
        node.destroy_node()


def test_refuses_stale_or_wrong_frame_odom_and_bad_names(ros, tmp_path):
    node = _node(tmp_path)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0, frame='map'))
        assert 'frame_id' in _record(node, 'a').message
        node._on_odom(_odom(1.0, 2.0, 0.0))
        node.odom_at = time.monotonic() - 5.0
        assert not _record(node, 'a').success
        node._on_odom(_odom(1.0, 2.0, 0.0))
        assert not _record(node, 'a b').success
        assert not (tmp_path / 'points.json').exists()
    finally:
        node.destroy_node()


def test_refuses_to_mix_origins_and_warns_without_origin(ros, tmp_path):
    node = _node(tmp_path, origin=False)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0))
        resp = _record(node, 'a')
        assert resp.success and '원점 파일 없음' in resp.message
        assert node.store.get('a').lat is None
        _write_origin(tmp_path / 'origin.yaml')                    # 나중에 원점이 생기면
        resp = _record(node, 'b')
        assert not resp.success and '원점' in resp.message        # 섞지 않는다
        assert list(node.store.points) == ['a']
    finally:
        node.destroy_node()


def test_broken_file_is_not_overwritten(ros, tmp_path):
    (tmp_path / 'points.json').write_text('{broken')
    node = _node(tmp_path)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0))
        resp = _record(node, 'a')
        assert not resp.success and '읽지 못함' in resp.message
        assert (tmp_path / 'points.json').read_text() == '{broken'
    finally:
        node.destroy_node()
