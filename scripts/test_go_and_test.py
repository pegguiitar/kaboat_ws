"""go_and_test.py 시작 조건·명령 구성 (ROS 없이)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import go_and_test as gt  # noqa: E402


def test_start_condition_needs_both_distance_and_heading():
    ok, dist, err = gt.start_condition((1.0, 1.0), 100.0, (0.0, 0.0), 90.0, 2.0, 30.0)
    assert ok and dist == pytest.approx(2 ** 0.5) and err == pytest.approx(10.0)
    assert not gt.start_condition((2.5, 0.0), 90.0, (0.0, 0.0), 90.0, 2.0, 30.0)[0]   # 멂
    assert not gt.start_condition((0.5, 0.0), 125.0, (0.0, 0.0), 90.0, 2.0, 30.0)[0]  # 틀어짐


def test_heading_error_wraps_around_180():
    ok, _, err = gt.start_condition((0.0, 0.0), -175.0, (0.0, 0.0), 175.0, 2.0, 30.0)
    assert ok and err == pytest.approx(10.0)


def test_reference_is_bearing_to_goal_for_avoid_tests():
    assert gt.reference_yaw((0.0, 0.0), 0.0, (0.0, 10.0)) == pytest.approx(90.0)
    assert gt.reference_yaw((0.0, 0.0), 37.0) == 37.0          # 목표 없으면 기록 선수각


def test_hold_requires_unbroken_time():
    h = gt.HoldTimer(0.5)
    assert not h.update(True, 10.0)
    assert not h.update(True, 10.4)
    assert not h.update(False, 10.45)                          # 끊기면 처음부터
    assert not h.update(True, 10.5)
    assert h.update(True, 11.0)


def test_tournament_plan_goes_to_entry_then_launches_tournament_test():
    plan = gt.build_plan('tournament', ['drive_enabled:=false'])
    assert plan.goto_cmd[-1] == 'point:=tournament_entry'
    assert plan.goal == 'tournament_end'
    assert plan.test_cmd[:4] == ['ros2', 'launch', 'kaboat_hardware',
                                 'outdoor_tournament_test.launch.py']
    assert plan.test_cmd[4:] == ['drive_enabled:=false', 'start_tolerance:=3.5',
                                 'start_heading_tolerance_deg:=50']   # 넘기는 동안 흐를 여유
    assert (plan.dist, plan.heading, plan.hold) == (3.0, 45.0, 0.5)   # 회피·토너먼트 3 m·±45°


def test_avoid_test_start_check_follows_the_condition_with_margin():
    plan = gt.build_plan('avoid', [], dist=4.0, heading=60.0)
    assert 'start_tolerance:=4.5' in plan.test_cmd
    assert 'start_heading_tolerance_deg:=65' in plan.test_cmd
    kept = gt.build_plan('avoid', ['start_tolerance:=6'], dist=4.0)
    assert [a for a in kept.test_cmd if a.startswith('start_tolerance')] == ['start_tolerance:=6']


def test_start_point_argument_moves_the_entry():
    plan = gt.build_plan('tournament', ['start_point:=t2_entry', 'goal_point:=t2_end'])
    assert plan.goto_cmd[-1] == 'point:=t2_entry' and plan.goal == 't2_end'


def test_entry_heading_missions_and_goto_args():
    gate = gt.build_plan('gate', [], goto_args=['thrust_ratio:=0.3'])
    assert gate.goal is None and (gate.dist, gate.heading) == (2.0, 30.0)
    assert gate.goto_cmd[-2:] == ['point:=gate_entry', 'thrust_ratio:=0.3']
    assert gate.test_cmd[3] == 'outdoor_gate_test.launch.py'
    dock = gt.build_plan('dock', ['target_class:=circle'])
    assert dock.test_cmd[0] == 'bash' and dock.test_cmd[1].endswith('scripts/run_dock_mission.sh')
    assert dock.test_cmd[-1] == 'target_class:=circle'


def test_station_and_search_skip_the_autonomy_approach():
    station = gt.build_plan('station')
    assert 'start_state:=station' in station.test_cmd
    assert 'entry_required:=false' in station.test_cmd       # goto 가 이미 세움 → 바로 시작
    search = gt.build_plan('search')
    assert any('search_target_color' in w for w in search.warnings)
    assert not gt.build_plan('search', ['search_target_color:=red']).warnings


def test_bad_inputs_are_rejected():
    with pytest.raises(ValueError):
        gt.build_plan('nope')
    with pytest.raises(ValueError):
        gt.build_plan('gate', dist=0.0)
    with pytest.raises(ValueError):
        gt.build_plan('gate', heading=200.0)
