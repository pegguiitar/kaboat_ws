import math
from dataclasses import replace

from kaboat_behaviors.docking_fsm import (
    DockPlan, DockingFsm, DockParams, DockState, DockTarget)


FAR = DockPlan(
    pre_entry=DockTarget(0.0, 2.0),
    mouth=DockTarget(0.0, 3.5),
    berth=DockTarget(0.0, 5.5),
    marker=DockTarget(0.0, 6.5),
    axis_error=0.0)
AT_PRE = replace(FAR, pre_entry=DockTarget(0.0, 0.4))
AT_MOUTH = replace(AT_PRE, mouth=DockTarget(0.0, 0.35))
AT_BERTH = replace(AT_MOUTH, berth=DockTarget(0.0, 0.0),
                    marker=DockTarget(0.0, 1.0))


def tick(fsm, now, staging=1.0, plan=FAR, reverse_distance=0.0):
    return fsm.step(now, staging, plan, reverse_distance)


def to_pre_entry(fsm):
    tick(fsm, 0.0, plan=None)
    assert tick(fsm, 0.1).state == DockState.PRE_ENTRY


def to_align(fsm):
    to_pre_entry(fsm)
    assert tick(fsm, 0.2, plan=AT_PRE).state == DockState.ALIGN


def to_mouth(fsm):
    to_align(fsm)
    tick(fsm, 0.3, plan=AT_PRE)
    assert tick(fsm, 0.8, plan=AT_PRE).state == DockState.TO_MOUTH


def to_berth(fsm):
    to_mouth(fsm)
    assert tick(fsm, 0.9, plan=AT_MOUTH).state == DockState.TO_BERTH


def to_verify(fsm):
    to_berth(fsm)
    assert tick(fsm, 1.0, staging=4.0,
                plan=AT_BERTH).state == DockState.BERTH_VERIFY


def to_hold(fsm):
    to_verify(fsm)
    tick(fsm, 1.1, plan=AT_BERTH)
    assert tick(fsm, 1.6, plan=AT_BERTH).state == DockState.HOLD


def test_approach_uses_global_staging_goal():
    out = tick(DockingFsm(), 0.0, staging=4.0, plan=None)
    assert out.state == DockState.APPROACH
    assert out.seek_goal and out.use_repulsion


def test_acquire_requires_plan_and_times_out():
    fsm = DockingFsm(DockParams(acquire_max_time=1.0))
    tick(fsm, 0.0, plan=None)
    assert tick(fsm, 0.5, plan=None).state == DockState.ACQUIRE
    out = tick(fsm, 1.0, plan=None)
    assert out.state == DockState.ABORT and out.angular == 0.0


def test_pre_entry_is_first_local_waypoint():
    fsm = DockingFsm()
    to_pre_entry(fsm)
    out = tick(fsm, 0.15)
    assert out.state == DockState.PRE_ENTRY
    assert out.linear > 0.0 and out.use_repulsion
    assert tick(fsm, 0.2, plan=AT_PRE).state == DockState.ALIGN


def test_pre_entry_does_not_drive_with_large_bearing_error():
    fsm = DockingFsm()
    to_pre_entry(fsm)
    off = replace(FAR, pre_entry=DockTarget(math.radians(25), 2.0))
    out = tick(fsm, 0.2, plan=off)
    assert out.linear == 0.0 and out.angular > 0.0


def test_align_requires_continuous_axis_hold():
    fsm = DockingFsm()
    to_align(fsm)
    off = replace(AT_PRE, axis_error=math.radians(8))
    assert tick(fsm, 0.3, plan=off).angular > 0.0
    tick(fsm, 0.5, plan=AT_PRE)
    assert tick(fsm, 0.8, plan=AT_PRE).state == DockState.ALIGN
    assert tick(fsm, 1.01, plan=AT_PRE).state == DockState.TO_MOUTH


def test_mouth_must_be_reached_before_berth():
    fsm = DockingFsm()
    to_mouth(fsm)
    out = tick(fsm, 0.85, plan=AT_PRE)
    assert out.state == DockState.TO_MOUTH and out.linear > 0.0
    assert tick(fsm, 0.9, plan=AT_MOUTH).state == DockState.TO_BERTH


def test_berth_uses_marker_distance_not_mouth_distance():
    fsm = DockingFsm()
    to_berth(fsm)
    out = tick(fsm, 0.95, plan=AT_MOUTH)
    assert out.state == DockState.TO_BERTH and out.linear > 0.0
    out = tick(fsm, 1.0, plan=AT_BERTH)
    assert out.state == DockState.BERTH_VERIFY and out.linear == 0.0


def test_berth_point_reached_without_one_meter_mark_aborts():
    fsm = DockingFsm()
    to_berth(fsm)
    inconsistent = replace(AT_MOUTH,
                           berth=DockTarget(0.0, 0.1),
                           marker=DockTarget(0.0, 1.2))
    out = tick(fsm, 1.0, plan=inconsistent)
    assert out.state == DockState.ABORT and out.linear == 0.0


def test_berth_requires_continuous_confirmation():
    fsm = DockingFsm()
    to_verify(fsm)
    tick(fsm, 1.1, plan=AT_BERTH)
    assert tick(fsm, 1.3, plan=None).state == DockState.BERTH_VERIFY
    assert tick(fsm, 1.5, plan=AT_BERTH).state == DockState.BERTH_VERIFY
    assert tick(fsm, 1.99, plan=AT_BERTH).state == DockState.BERTH_VERIFY
    assert tick(fsm, 2.0, plan=AT_BERTH).state == DockState.HOLD


def test_plan_loss_inside_slot_aborts_and_stays_stopped():
    fsm = DockingFsm(DockParams(mark_lost_timeout=0.5))
    to_berth(fsm)
    assert tick(fsm, 1.0, plan=None).linear == 0.0
    out = tick(fsm, 1.51, plan=None)
    assert out.state == DockState.ABORT
    assert out.linear == 0.0 and out.angular == 0.0
    assert tick(fsm, 2.0, plan=AT_BERTH).state == DockState.ABORT


def test_axis_error_inside_slot_aborts():
    fsm = DockingFsm()
    to_berth(fsm)
    off = replace(AT_MOUTH, axis_error=math.radians(15))
    out = tick(fsm, 1.0, plan=off)
    assert out.state == DockState.ABORT and out.linear == 0.0


def test_reverse_requires_time_distance_and_staging():
    fsm = DockingFsm(DockParams(hold_time=2.0,
                                reverse_min_time=0.5,
                                reverse_min_distance=2.5))
    to_hold(fsm)
    assert tick(fsm, 3.59, staging=4.0, plan=None).state == DockState.HOLD
    out = tick(fsm, 3.6, staging=4.0, plan=None)
    assert out.state == DockState.REVERSE and out.linear < 0.0
    assert tick(fsm, 3.9, staging=1.8, plan=None,
                reverse_distance=2.5).state == DockState.REVERSE
    assert tick(fsm, 4.1, staging=1.8, plan=None,
                reverse_distance=0.2).state == DockState.REVERSE
    out = tick(fsm, 4.2, staging=1.8, plan=None,
               reverse_distance=2.5)
    assert out.state == DockState.COMPLETE and out.complete


def test_reverse_timeout_aborts():
    fsm = DockingFsm(DockParams(hold_time=0.0, reverse_max_time=1.0))
    to_hold(fsm)
    tick(fsm, 1.7, staging=4.0, plan=None)
    out = tick(fsm, 2.7, staging=4.0, plan=None)
    assert out.state == DockState.ABORT and out.linear == 0.0
