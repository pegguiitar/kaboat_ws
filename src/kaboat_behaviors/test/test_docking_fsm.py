import math

import pytest

from kaboat_behaviors.docking_fsm import (
    DockingFsm, DockParams, DockState, DockTarget)


TARGET = DockTarget(bearing=0.0, distance=5.0)


def enter_align(fsm):
    fsm.step(0.0, 1.0, None)
    out = fsm.step(0.1, 1.0, TARGET)
    assert out.state == DockState.ALIGN


def enter_dock(fsm):
    enter_align(fsm)
    fsm.step(0.2, 1.0, TARGET)
    out = fsm.step(0.8, 1.0, TARGET)
    assert out.state == DockState.ENTER


def test_approach_uses_goal_and_repulsion():
    out = DockingFsm().step(0.0, 4.0, None)
    assert out.state == DockState.APPROACH
    assert out.seek_goal
    assert out.use_repulsion


def test_staging_arrival_starts_acquire_scan():
    out = DockingFsm().step(0.0, 1.5, None)
    assert out.state == DockState.ACQUIRE
    assert out.angular > 0.0
    assert out.event == 'APPROACH → ACQUIRE'


def test_acquire_scan_changes_direction():
    fsm = DockingFsm(DockParams(scan_half_period=1.0))
    fsm.step(0.0, 1.0, None)
    assert fsm.step(0.5, 1.0, None).angular > 0.0
    assert fsm.step(1.1, 1.0, None).angular < 0.0


def test_visible_target_enters_align():
    fsm = DockingFsm()
    enter_align(fsm)
    assert fsm.state == DockState.ALIGN


def test_align_requires_continuous_hold():
    fsm = DockingFsm()
    enter_align(fsm)
    fsm.step(0.2, 1.0, DockTarget(math.radians(4.0), 5.0))
    assert fsm.step(0.6, 1.0, TARGET).state == DockState.ALIGN
    assert fsm.step(0.8, 1.0, TARGET).state == DockState.ENTER


def test_alignment_loss_resets_hold_timer():
    fsm = DockingFsm()
    enter_align(fsm)
    fsm.step(0.2, 1.0, TARGET)
    fsm.step(0.5, 1.0, DockTarget(math.radians(8.0), 5.0))
    assert fsm.step(0.8, 1.0, TARGET).state == DockState.ALIGN
    assert fsm.step(1.1, 1.0, TARGET).state == DockState.ALIGN
    assert fsm.step(1.31, 1.0, TARGET).state == DockState.ENTER


def test_lost_target_returns_to_acquire():
    fsm = DockingFsm(DockParams(mark_lost_timeout=0.5))
    enter_align(fsm)
    assert fsm.step(0.2, 1.0, None).state == DockState.ALIGN
    out = fsm.step(0.71, 1.0, None)
    assert out.state == DockState.ACQUIRE


def test_enter_stops_at_berth():
    fsm = DockingFsm()
    enter_dock(fsm)
    out = fsm.step(0.9, 4.0, DockTarget(0.0, 1.0))
    assert out.state == DockState.HOLD
    assert out.linear == 0.0


def test_hold_then_reverse():
    fsm = DockingFsm(DockParams(hold_time=2.0))
    enter_dock(fsm)
    fsm.step(0.9, 4.0, DockTarget(0.0, 0.9))
    assert fsm.step(2.89, 4.0, None).state == DockState.HOLD
    out = fsm.step(2.9, 4.0, None)
    assert out.state == DockState.REVERSE
    assert out.linear < 0.0


def test_reverse_completes_only_near_staging_waypoint():
    fsm = DockingFsm(DockParams(hold_time=0.0, reverse_min_time=0.5))
    enter_dock(fsm)
    fsm.step(0.9, 4.0, DockTarget(0.0, 0.9))
    fsm.step(1.0, 4.0, None)
    assert fsm.step(1.6, 2.5, None).state == DockState.REVERSE
    out = fsm.step(1.7, 1.8, None)
    assert out.state == DockState.COMPLETE
    assert out.complete
    assert out.linear == 0.0


def test_reset_returns_to_approach():
    fsm = DockingFsm()
    enter_align(fsm)
    fsm.reset()
    assert fsm.state == DockState.APPROACH


@pytest.mark.parametrize('bearing, expected_sign', [
    (math.radians(20.0), 1.0),
    (math.radians(-20.0), -1.0),
])
def test_alignment_steers_toward_target(bearing, expected_sign):
    fsm = DockingFsm()
    fsm.step(0.0, 1.0, None)
    out = fsm.step(0.1, 1.0, DockTarget(bearing, 5.0))
    assert math.copysign(1.0, out.angular) == expected_sign
