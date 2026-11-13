"""mission_manager 가 현장 기록 지점(JSON)을 읽어 미션 좌표를 덮어쓰는지."""

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter  # noqa: E402
import yaml  # noqa: E402

from kaboat_mission.mission_manager import DEFAULT_WAYPOINTS, MissionManager  # noqa: E402
from kaboat_mission.mission_points import MissionPoint, PointStore  # noqa: E402

ORIGIN = (35.1234567, 129.0456789, 12.5)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _files(tmp_path, origin=ORIGIN, recorded_origin=ORIGIN, **points):
    if origin is not None:
        (tmp_path / 'origin.yaml').write_text(yaml.safe_dump(
            {'origin_lat': origin[0], 'origin_lon': origin[1], 'origin_alt': origin[2]}))
    store = PointStore()
    for name, pose in points.items():
        store, _ = store.with_point(name, MissionPoint(*pose), recorded_origin)
    store.save(str(tmp_path / 'points.json'))


def _node(tmp_path, points_file=None, **params):
    overrides = [
        Parameter('points_file', value=str(tmp_path / 'points.json')
                  if points_file is None else points_file),
        Parameter('origin_file', value=str(tmp_path / 'origin.yaml')),
        Parameter('auto_start', value=False),
    ]
    overrides += [Parameter(k, value=v) for k, v in params.items()]
    return MissionManager(parameter_overrides=overrides)


def test_recorded_points_override_params_and_start_approach(ros, tmp_path):
    _files(tmp_path, gate_entry=(1.0, 2.0, 90.0), gate_end=(20.0, 2.5, 10.0),
           dock_entry=(4.0, 1.0, 180.0), dock_end=(5.0, 5.0, 0.0), p1=(9.0, 9.0, 0.0))
    node = _node(tmp_path)
    try:
        assert node.entry_waypoints['gate'] == [1.0, 2.0, 90.0]
        assert node.waypoints['gate'] == [20.0, 2.5]
        assert node.entry_ready['gate'] and node.entry_ready['dock']
        assert node.waypoints['dock'] == [5.0, 5.0]
        # 기록 없는 미션은 기존 파라미터 그대로
        assert node.waypoints['station'] == DEFAULT_WAYPOINTS['station']
        assert not node.entry_ready['station']
        node.advance()                                  # 첫 미션 gate → 접근 단계
        assert (node.phase, node.idx) == ('approach', 0)
    finally:
        node.destroy_node()


def test_missing_file_or_disabled_keeps_params(ros, tmp_path):
    node = _node(tmp_path)                              # points.json 없음
    try:
        assert node.waypoints == {k: list(v) for k, v in DEFAULT_WAYPOINTS.items()}
        assert not any(node.entry_ready.values())
    finally:
        node.destroy_node()
    _files(tmp_path, gate_entry=(1.0, 2.0, 90.0), gate_end=(20.0, 2.5, 0.0))
    for off in ('', 'none', 'None'):                     # 수조 런치·launch 인자: 읽지 않음
        node = _node(tmp_path, points_file=off)
        try:
            assert node.waypoints['gate'] == DEFAULT_WAYPOINTS['gate']
            assert not node.entry_ready['gate']
        finally:
            node.destroy_node()


def test_refuses_points_recorded_with_another_origin(ros, tmp_path):
    moved = (ORIGIN[0] + 1e-4, ORIGIN[1], ORIGIN[2])    # 약 11 m 다른 원점
    _files(tmp_path, origin=moved, gate_entry=(1.0, 2.0, 90.0), gate_end=(20.0, 2.5, 0.0))
    with pytest.raises(ValueError, match='원점'):
        _node(tmp_path)


def test_refuses_when_origin_file_is_gone(ros, tmp_path):
    _files(tmp_path, origin=None, gate_entry=(1.0, 2.0, 90.0), gate_end=(20.0, 2.5, 0.0))
    with pytest.raises(ValueError, match='원점'):
        _node(tmp_path)


def test_refuses_half_recorded_mission(ros, tmp_path):
    _files(tmp_path, gate_entry=(1.0, 2.0, 90.0))
    with pytest.raises(ValueError, match='gate_end'):
        _node(tmp_path)


def test_refuses_broken_points_file(ros, tmp_path):
    _files(tmp_path)
    (tmp_path / 'points.json').write_text('{broken')
    with pytest.raises(ValueError):
        _node(tmp_path)


def test_search_center_recorded_point_is_used(ros, tmp_path):
    _files(tmp_path, search_entry=(168.7, -8.73, 71.9), search_end=(147.1, -2.31, 0.0),
           search_center=(170.9, -2.5, 0.0))
    node = _node(tmp_path)
    try:
        assert node.search_area_center() == [170.9, -2.5]
        assert node.waypoints['search'] == [147.1, -2.31]        # 끝점은 출구 그대로
    finally:
        node.destroy_node()


def test_search_center_falls_back_ahead_of_search_entry(ros, tmp_path):
    _files(tmp_path, search_entry=(0.0, 0.0, 90.0), search_end=(-20.0, 0.0, 0.0))
    node = _node(tmp_path, search_center_offset=7.0)
    try:
        x, y = node.search_area_center()
        assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(7.0)
    finally:
        node.destroy_node()


def test_search_center_none_without_entry_or_record(ros, tmp_path):
    node = _node(tmp_path, points_file='none')
    try:
        assert node.search_area_center() is None
    finally:
        node.destroy_node()
