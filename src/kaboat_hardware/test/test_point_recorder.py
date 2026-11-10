"""point_recorder 노드: 요청 토픽을 받으면 지금 /odom 위치를 이름 붙여 JSON 에 기록."""

import json
import math
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import String
import yaml

from kaboat_hardware.geo_local import LocalFrame
from kaboat_hardware.point_recorder import PointRecorder, parse_request

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
    node.markers = []
    node.results = []
    node.markers_pub.publish = node.markers.append
    node.result_pub.publish = lambda msg: node.results.append(json.loads(msg.data))
    return node


def _send(node, req_id='', name='', cmd='record'):
    node._on_request(String(data=json.dumps({'id': req_id, 'cmd': cmd, 'name': name})))
    return node.results[-1]


def test_parse_request_accepts_json_or_plain_name():
    assert parse_request('{"id": "a1", "cmd": "list"}') == {'id': 'a1', 'cmd': 'list', 'name': ''}
    assert parse_request(' gate_entry ') == {'id': '', 'cmd': 'record', 'name': 'gate_entry'}
    assert parse_request('') == {'id': '', 'cmd': 'record', 'name': ''}
    assert parse_request('123')['name'] == '123'                   # JSON 숫자도 이름으로


def test_records_current_pose_with_lat_lon_and_reloads(ros, tmp_path):
    node = _node(tmp_path)
    try:
        assert not _send(node, 'r0', 'a')['success']               # /odom 없음
        node._on_odom(_odom(12.3456, -7.8912, 90.0))
        res = _send(node, 'r1', 'gate_entry')
        assert res['success'], res['message']
        assert (res['id'], res['name'], res['x'], res['y']) == ('r1', 'gate_entry', 12.346, -7.891)
        assert res['yaw_deg'] == pytest.approx(90.0)

        saved = json.load(open(tmp_path / 'points.json', encoding='utf-8'))
        point = saved['points']['gate_entry']
        assert saved['origin']['lat'] == pytest.approx(ORIGIN[0])
        # 저장한 위도·경도를 같은 원점으로 되돌리면 같은 x/y
        x, y = LocalFrame(*ORIGIN).to_local(point['lat'], point['lon'])
        assert (x, y) == pytest.approx((12.346, -7.891), abs=1e-3)

        assert _send(node, 'r2')['name'] == 'p1'                   # 이름 없으면 p1
        node._on_request(String(data='dock'))                      # 직접 보낸 글자 = 이름
        assert node.results[-1]['success'] and 'dock' in node.store.points
        listed = _send(node, 'r3', cmd='list')
        assert listed['points']['gate_entry'] == pytest.approx([12.346, -7.891, 90.0])
        assert 'p1' in listed['message']
        assert len(node.markers[-1].markers) == 1 + 2 * 3          # DELETEALL + 화살표·이름
    finally:
        node.destroy_node()
    again = _node(tmp_path)                                        # 다시 켜도 이어서
    try:
        assert list(again.store.points) == ['gate_entry', 'p1', 'dock']
    finally:
        again.destroy_node()


def test_resent_request_is_answered_again_without_recording_twice(ros, tmp_path):
    node = _node(tmp_path)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0))
        first = _send(node, 'same')
        node._on_odom(_odom(9.0, 9.0, 0.0))
        again = _send(node, 'same')                                # 노트북 재전송
        assert again == first and first['name'] == 'p1'
        assert list(node.store.points) == ['p1']
        assert node.store.get('p1').xy() == pytest.approx([1.0, 2.0])
    finally:
        node.destroy_node()


def test_overwrite_same_name_reports_previous(ros, tmp_path):
    node = _node(tmp_path)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0))
        assert _send(node, 'a', 'dock')['success']
        node._on_odom(_odom(3.0, 4.0, 180.0))
        res = _send(node, 'b', 'dock')
        assert res['success'] and '덮어씀' in res['message']
        assert node.store.get('dock').xy() == pytest.approx([3.0, 4.0])
        assert len(node.store.points) == 1
    finally:
        node.destroy_node()


def test_refuses_stale_or_wrong_frame_odom_and_bad_names(ros, tmp_path):
    node = _node(tmp_path)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0, frame='map'))
        assert 'frame_id' in _send(node, 'a', 'x')['message']
        node._on_odom(_odom(1.0, 2.0, 0.0))
        node.odom_at = time.monotonic() - 5.0
        assert not _send(node, 'b', 'x')['success']
        node._on_odom(_odom(1.0, 2.0, 0.0))
        assert not _send(node, 'c', 'a b')['success']
        assert not _send(node, 'd', cmd='delete')['success']
        assert not (tmp_path / 'points.json').exists()
    finally:
        node.destroy_node()


def test_refuses_to_mix_origins_and_warns_without_origin(ros, tmp_path):
    node = _node(tmp_path, origin=False)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0))
        res = _send(node, 'a', 'a')
        assert res['success'] and '원점 파일 없음' in res['message']
        assert node.store.get('a').lat is None
        _write_origin(tmp_path / 'origin.yaml')                    # 나중에 원점이 생기면
        res = _send(node, 'b', 'b')
        assert not res['success'] and '원점' in res['message']     # 섞지 않는다
        assert list(node.store.points) == ['a']
    finally:
        node.destroy_node()


def test_broken_file_is_not_overwritten(ros, tmp_path):
    (tmp_path / 'points.json').write_text('{broken')
    node = _node(tmp_path)
    try:
        node._on_odom(_odom(1.0, 2.0, 0.0))
        res = _send(node, 'a', 'a')
        assert not res['success'] and '읽지 못함' in res['message']
        assert (tmp_path / 'points.json').read_text() == '{broken'
    finally:
        node.destroy_node()
