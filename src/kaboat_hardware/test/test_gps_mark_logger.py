"""Space GPS 기록: 파일 이름·머리줄, 즉시 디스크 반영, GPS 없을 때 기록 안 함,
우리 원점 기준 x·y (원점 없으면 비움, 원점 바뀌면 경고)."""

import csv
from datetime import datetime
import time

import pytest

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Empty

from kaboat_hardware.gps_mark_logger import (
    GpsMarkCsv, GpsMarkLogger, fix_problem, format_timestamp, log_path, origin_xy)

ORIGIN = (37.5665, 126.978, 10.0)


def _write_origin(path, lat, lon, alt=10.0):
    path.write_text(f'origin_lat: {lat}\norigin_lon: {lon}\norigin_alt: {alt}\n',
                    encoding='utf-8')
    return path


def _fix(lat=37.5665123, lon=126.9780456, status=NavSatStatus.STATUS_FIX):
    msg = NavSatFix()
    msg.latitude, msg.longitude = lat, lon
    msg.status.status = status
    return msg


def test_file_name_header_and_rows_are_on_disk_immediately(tmp_path):
    now = datetime(2026, 11, 6, 14, 3, 12, 345678)
    path = log_path(str(tmp_path), now)
    assert path.endswith('gps_log_20261106_140312.csv')
    log = GpsMarkCsv(path)
    log.append(format_timestamp(now), 37.5665123, 126.9780456, (1.23456, -7.8))
    log.append(format_timestamp(now), 37.5665123, 126.9780456)        # 원점 없음
    # 닫지 않은 상태에서 다른 리더로 읽어도 이미 기록돼 있어야 한다(비정상 종료 대비)
    with open(log.path, newline='') as f:
        rows = list(csv.reader(f))
    assert rows == [['timestamp', 'latitude', 'longitude', 'x', 'y'],
                    ['2026-11-06 14:03:12.345', '37.56651230', '126.97804560',
                     '1.235', '-7.800'],
                    ['2026-11-06 14:03:12.345', '37.56651230', '126.97804560', '', '']]
    log.close()


def test_origin_xy_is_east_north_metres_from_origin():
    """x = 동쪽, y = 북쪽 [m] — /odom·rec 와 같은 축."""
    assert origin_xy(None, 37.0, 127.0) is None
    x, y = origin_xy(ORIGIN, ORIGIN[0], ORIGIN[1])
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-6)
    x, y = origin_xy(ORIGIN, ORIGIN[0] + 0.001, ORIGIN[1])      # 북쪽 0.001°
    assert x == pytest.approx(0.0, abs=0.01) and y == pytest.approx(111.0, rel=0.01)
    x, y = origin_xy(ORIGIN, ORIGIN[0], ORIGIN[1] + 0.001)      # 동쪽 0.001°
    assert x == pytest.approx(88.2, rel=0.01) and y == pytest.approx(0.0, abs=0.01)


def test_same_second_start_does_not_overwrite(tmp_path):
    path = log_path(str(tmp_path), datetime(2026, 11, 6, 9, 0, 0))
    first, second = GpsMarkCsv(path), GpsMarkCsv(path)
    assert first.path != second.path and second.path.endswith('_2.csv')
    first.close()
    second.close()


def test_fix_problems():
    now = 100.0
    assert fix_problem(None, 0.0, now, 2.0) == 'GPS 수신 없음'
    assert '갱신 안 됨' in fix_problem(_fix(), now - 5.0, now, 2.0)
    assert 'no fix' in fix_problem(_fix(status=NavSatStatus.STATUS_NO_FIX), now, now, 2.0)
    assert '유효하지 않음' in fix_problem(_fix(lat=float('nan')), now, now, 2.0)
    assert fix_problem(_fix(), now - 0.5, now, 2.0) is None


def test_node_logs_on_mark_and_reports_failure_without_fix(tmp_path):
    rclpy.init()
    node = None
    try:
        node = GpsMarkLogger(parameter_overrides=[
            Parameter('log_dir', Parameter.Type.STRING, str(tmp_path)),
            Parameter('origin_file', Parameter.Type.STRING,
                      str(tmp_path / 'no_origin.yaml'))])
        events = []
        node.event_pub.publish = events.append

        node._on_mark(Empty())                          # GPS 아직 없음
        assert 'GPS 수신 없음' in events[-1].data
        assert node.log.rows == 0

        node._on_fix(_fix())
        node._on_mark(Empty())
        assert node.log.rows == 1 and 'GPS 기록 #1' in events[-1].data
        assert '원점 파일 없음' in events[-1].data          # x·y 는 비우고 기록은 함

        node.fix_at = time.monotonic() - 10.0           # 오래된 GPS
        node._on_mark(Empty())
        assert node.log.rows == 1 and '갱신 안 됨' in events[-1].data

        with open(node.log.path, newline='') as f:
            rows = list(csv.reader(f))
        assert node.log.path.startswith(str(tmp_path))
        assert len(rows) == 2 and rows[1][1:] == ['37.56651230', '126.97804560', '', '']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_node_writes_origin_xy_and_warns_when_origin_changes(tmp_path):
    origin_file = _write_origin(tmp_path / 'origin.yaml', ORIGIN[0], ORIGIN[1])
    rclpy.init()
    node = None
    try:
        node = GpsMarkLogger(parameter_overrides=[
            Parameter('log_dir', Parameter.Type.STRING, str(tmp_path)),
            Parameter('origin_file', Parameter.Type.STRING, str(origin_file))])
        events = []
        node.event_pub.publish = events.append

        node._on_fix(_fix(lat=ORIGIN[0] + 0.001, lon=ORIGIN[1]))   # 원점에서 북쪽 약 111 m
        node._on_mark(Empty())
        assert 'x +0.00 m, y +110.' in events[-1].data and '원점' not in events[-1].data

        # 원점을 다시 잡음(북쪽 약 11 m) — 기록은 하되 경고, 새 원점 기준 x·y
        _write_origin(origin_file, ORIGIN[0] + 0.0001, ORIGIN[1])
        node._on_mark(Empty())
        assert '원점이 바뀜' in events[-1].data

        # 원점 파일이 깨지면 x·y 를 비우고 기록
        origin_file.write_text('origin_lat: [\n', encoding='utf-8')
        node._on_mark(Empty())
        assert '원점 파일을 읽지 못해' in events[-1].data and node.log.rows == 3

        with open(node.log.path, newline='') as f:
            rows = list(csv.reader(f))
        assert float(rows[1][4]) == pytest.approx(110.95, abs=0.2)
        assert float(rows[2][4]) == pytest.approx(99.86, abs=0.2)
        assert rows[3][3:] == ['', '']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
