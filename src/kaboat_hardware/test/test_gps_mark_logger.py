"""Space GPS 기록: 파일 이름·머리줄, 즉시 디스크 반영, GPS 없을 때 기록 안 함."""

import csv
from datetime import datetime
import time

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Empty

from kaboat_hardware.gps_mark_logger import (
    GpsMarkCsv, GpsMarkLogger, fix_problem, format_timestamp, log_path)


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
    log.append(format_timestamp(now), 37.5665123, 126.9780456)
    # 닫지 않은 상태에서 다른 리더로 읽어도 이미 기록돼 있어야 한다(비정상 종료 대비)
    with open(log.path, newline='') as f:
        rows = list(csv.reader(f))
    assert rows == [['timestamp', 'latitude', 'longitude'],
                    ['2026-11-06 14:03:12.345', '37.56651230', '126.97804560']]
    log.close()


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
            Parameter('log_dir', Parameter.Type.STRING, str(tmp_path))])
        events = []
        node.event_pub.publish = events.append

        node._on_mark(Empty())                          # GPS 아직 없음
        assert 'GPS 수신 없음' in events[-1].data
        assert node.log.rows == 0

        node._on_fix(_fix())
        node._on_mark(Empty())
        assert node.log.rows == 1 and 'GPS 기록 #1' in events[-1].data

        node.fix_at = time.monotonic() - 10.0           # 오래된 GPS
        node._on_mark(Empty())
        assert node.log.rows == 1 and '갱신 안 됨' in events[-1].data

        with open(node.log.path, newline='') as f:
            rows = list(csv.reader(f))
        assert node.log.path.startswith(str(tmp_path))
        assert len(rows) == 2 and rows[1][1:] == ['37.56651230', '126.97804560']
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
