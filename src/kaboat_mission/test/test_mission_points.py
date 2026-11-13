"""미션 지점 JSON 저장소: 저장·불러오기, 이름, 원점 일치 검사."""

import json
import os

import pytest

from kaboat_mission.mission_points import (
    MissionPoint, PointStore, auto_name, check_name, mission_slot, mission_targets,
    missions_to_run, origin_distance_m, read_origin_file, same_origin)

ORIGIN = (35.1234567, 129.0456789, 12.5)


def test_save_and_load_round_trip(tmp_path):
    path = str(tmp_path / 'points.json')
    store, prev = PointStore().with_point(
        'gate_entry', MissionPoint(1.234, -5.678, 90.0, 35.1, 129.0, 't0'), ORIGIN)
    assert prev is None
    store, _ = store.with_point('부두', MissionPoint(10.0, 2.0, -45.0), ORIGIN)
    store.save(path)

    data = json.load(open(path, encoding='utf-8'))
    assert data['frame_id'] == 'odom' and data['origin']['lat'] == pytest.approx(ORIGIN[0])
    assert list(data['points']) == ['gate_entry', '부두']          # 기록 순서 유지

    loaded = PointStore.load(path)
    assert loaded.origin == pytest.approx(ORIGIN)
    assert loaded.get('gate_entry').pose() == pytest.approx([1.234, -5.678, 90.0])
    assert loaded.get('부두').xy() == pytest.approx([10.0, 2.0])
    assert loaded.get('부두').lat is None
    assert os.listdir(tmp_path) == ['points.json']                # 임시 파일 안 남음


def test_missing_file_is_empty_and_broken_file_is_an_error(tmp_path):
    assert PointStore.load(str(tmp_path / 'none.json')).points == {}
    bad = tmp_path / 'bad.json'
    bad.write_text('{"version": 1, "frame_id": "odom", "points": {"a": {"x": 1}}}')
    with pytest.raises(ValueError):
        PointStore.load(str(bad))
    bad.write_text('{not json')
    with pytest.raises(ValueError, match='JSON'):
        PointStore.load(str(bad))
    bad.write_text('{"version": 1, "frame_id": "map", "points": {}}')
    with pytest.raises(ValueError, match='frame_id'):
        PointStore.load(str(bad))


def test_overwrite_returns_previous_and_keeps_original_store():
    first, _ = PointStore().with_point('a', MissionPoint(1.0, 1.0, 0.0), ORIGIN)
    second, prev = first.with_point('a', MissionPoint(2.0, 2.0, 10.0), ORIGIN)
    assert prev.x == 1.0 and second.get('a').x == 2.0
    assert first.get('a').x == 1.0                                # 원본은 그대로


def test_origin_must_match_once_points_exist():
    store, _ = PointStore().with_point('a', MissionPoint(0.0, 0.0, 0.0), ORIGIN)
    near = (ORIGIN[0] + 2e-6, ORIGIN[1], ORIGIN[2])                # 약 0.2 m
    far = (ORIGIN[0] + 1e-4, ORIGIN[1], ORIGIN[2])                 # 약 11 m
    store.with_point('b', MissionPoint(1.0, 0.0, 0.0), near)
    with pytest.raises(ValueError, match='원점'):
        store.with_point('b', MissionPoint(1.0, 0.0, 0.0), far)
    with pytest.raises(ValueError, match='원점'):
        store.with_point('b', MissionPoint(1.0, 0.0, 0.0), None)
    store.check_origin(near)
    with pytest.raises(ValueError):
        store.check_origin(far)
    PointStore().check_origin(far)                                # 빈 파일은 통과
    assert same_origin(None, None) and not same_origin(None, ORIGIN)
    assert origin_distance_m(ORIGIN, far) == pytest.approx(11.1, abs=0.1)


def test_names():
    assert check_name('  gate-1_a ') == 'gate-1_a'
    assert check_name('도킹') == '도킹'
    for bad in ('', 'a b', 'a/b', 'x' * 65):
        with pytest.raises(ValueError):
            check_name(bad)
    assert auto_name({}) == 'p1'
    assert auto_name({'p1': 0, 'p3': 0}) == 'p2'


def test_unknown_name_lists_known_ones():
    store, _ = PointStore().with_point('a', MissionPoint(0.0, 0.0, 0.0), None)
    with pytest.raises(KeyError, match='저장된 지점: a'):
        store.get('b')


def test_read_origin_file(tmp_path):
    assert read_origin_file(str(tmp_path / 'none.yaml')) is None
    f = tmp_path / 'origin.yaml'
    f.write_text('origin_lat: 35.1234567\norigin_lon: 129.0456789\norigin_alt: 12.5\n')
    assert read_origin_file(str(f)) == pytest.approx(ORIGIN)
    for bad in ('origin_lat: 95.0\norigin_lon: 129.0\n', 'origin_lon: 129.0\n', '{broken'):
        f.write_text(bad)
        with pytest.raises(ValueError):
            read_origin_file(str(f))


def test_mission_slot_names():
    assert mission_slot('gate_entry') == ('gate', 'entry')
    assert mission_slot('search_end') == ('search', 'end')
    for other in ('p1', 'start', 'gate', 'gates_end', 'dock_entry2', 'Gate_entry'):
        assert mission_slot(other) is None


def _store(**points):
    store = PointStore()
    for name, pose in points.items():
        store, _ = store.with_point(name, MissionPoint(*pose), ORIGIN)
    return store


def test_mission_targets_pairs_entry_and_end():
    store = _store(gate_entry=(1.0, 2.0, 90.0), gate_end=(20.0, 2.5, 0.0),
                   dock_end=(5.0, 5.0, 45.0), dock_entry=(4.0, 1.0, 180.0), p1=(0, 0, 0))
    targets = mission_targets(store)
    assert list(targets) == ['gate', 'dock']                         # 미션 순서, p1 은 안 씀
    assert targets['gate'] == {'entry': [1.0, 2.0, 90.0], 'end': [20.0, 2.5]}
    assert targets['dock']['entry'] == [4.0, 1.0, 180.0]
    assert mission_targets(PointStore()) == {}


def test_dock_entry_alone_uses_sensor_generated_goal():
    targets = mission_targets(_store(dock_entry=(4.0, 1.0, 180.0)))
    assert targets['dock'] == {'entry': [4.0, 1.0, 180.0], 'end': [4.0, 1.0]}


def test_mission_targets_refuses_half_pair():
    with pytest.raises(ValueError, match='gate_end'):
        mission_targets(_store(gate_entry=(1.0, 2.0, 90.0)))
    with pytest.raises(ValueError, match='station_entry'):
        mission_targets(_store(station_end=(1.0, 2.0, 0.0)))


def test_missions_to_run_follows_mission_manager_order():
    """종합 스택이 띄울 미션 — start_state 부터 끝까지, single_mission 이면 그 하나."""
    every = ('gate', 'station', 'dock', 'search', 'avoid')
    assert missions_to_run() == every
    assert missions_to_run('', single_mission=True) == ('gate',)      # '' = gate 부터
    assert missions_to_run('station', single_mission=True) == ('station',)
    assert missions_to_run('dock') == ('dock', 'search', 'avoid')
    assert missions_to_run('avoid', single_mission=True) == ('avoid',)


def test_missions_to_run_refuses_unknown_start_state():
    # mission_manager 는 오류 뒤 gate 부터 시작한다 — 오타로 게이트가 움직이기 전에 막는다
    with pytest.raises(ValueError, match='staton'):
        missions_to_run('staton', single_mission=True)
