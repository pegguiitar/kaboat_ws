import math
from dataclasses import replace

from kaboat_behaviors.docking_fsm import (
    DockPlan, DockingFsm, DockParams, DockState, DockTarget)
from kaboat_behaviors.slot_clearance import SlotClearance


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


# 폭 3.0m 슬롯, 선체 1.0m 가 중앙: 좌우 간격 1.0m
GOOD_SLOT = SlotClearance(1.0, 1.0, 1.0)


def tick(fsm, now, staging=1.0, plan=FAR, reverse_distance=0.0,
         clearance=GOOD_SLOT):
    return fsm.step(now, staging, plan, reverse_distance,
                    clearance=clearance)


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
    assert out.use_waypoint and out.use_repulsion


def test_acquire_requires_plan_and_times_out():
    fsm = DockingFsm(DockParams(acquire_max_time=1.0, acquire_retries=0))
    tick(fsm, 0.0, plan=None)
    assert tick(fsm, 0.5, plan=None).state == DockState.ACQUIRE
    out = tick(fsm, 1.0, plan=None)
    assert out.state == DockState.ABORT and out.angular == 0.0


def test_acquire_timeout_relocates_then_retries_once():
    fsm = DockingFsm(DockParams(acquire_max_time=1.0, acquire_retries=1))
    tick(fsm, 0.0, plan=None)                       # APPROACH → ACQUIRE
    out = tick(fsm, 1.0, plan=None)                 # 탐색 실패
    assert out.state == DockState.RELOCATE
    assert out.use_waypoint and out.use_repulsion and out.linear == 0.0
    out = fsm.step(2.0, 1.0, None, relocate_reached=False)
    assert out.state == DockState.RELOCATE and out.use_waypoint
    out = fsm.step(3.0, 1.0, None, relocate_reached=True)
    assert out.state == DockState.ACQUIRE and out.angular != 0.0
    # 탐색 시간이 도착 시점부터 다시 계산된다.
    assert tick(fsm, 3.5, plan=None).state == DockState.ACQUIRE
    # 재시도도 실패하면 더는 이동하지 않고 ABORT.
    out = tick(fsm, 4.0, plan=None)
    assert out.state == DockState.ABORT and not out.use_waypoint


def test_retry_acquire_still_proceeds_when_marker_found():
    fsm = DockingFsm(DockParams(acquire_max_time=1.0))
    tick(fsm, 0.0, plan=None)
    tick(fsm, 1.0, plan=None)
    fsm.step(2.0, 1.0, None, relocate_reached=True)
    assert tick(fsm, 2.1).state == DockState.PRE_ENTRY


def test_relocate_ignores_plan_and_times_out():
    fsm = DockingFsm(DockParams(acquire_max_time=1.0, relocate_max_time=5.0))
    tick(fsm, 0.0, plan=None)
    tick(fsm, 1.0, plan=None)                       # → RELOCATE
    assert tick(fsm, 2.0).state == DockState.RELOCATE   # 계획이 보여도 이동 계속
    out = tick(fsm, 6.0, plan=None)
    assert out.state == DockState.ABORT and out.linear == 0.0


def test_reset_restores_retry_budget():
    fsm = DockingFsm(DockParams(acquire_max_time=1.0))
    tick(fsm, 0.0, plan=None)
    tick(fsm, 1.0, plan=None)
    fsm.reset()
    tick(fsm, 10.0, plan=None)
    assert tick(fsm, 11.0, plan=None).state == DockState.RELOCATE


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




def test_slot_width_within_tolerance_is_accepted():
    fsm = DockingFsm()
    to_mouth(fsm)
    out = tick(fsm, 0.9, plan=AT_MOUTH,
               clearance=SlotClearance(0.8, 1.1, 1.0))   # 폭 2.9m
    assert out.state == DockState.TO_BERTH


def test_slot_check_can_be_disabled():
    fsm = DockingFsm(DockParams(slot_check_enabled=False))
    to_mouth(fsm)
    assert tick(fsm, 0.9, plan=AT_MOUTH,
                clearance=None).state == DockState.TO_BERTH



def test_berth_stops_then_aborts_when_walls_disappear():
    fsm = DockingFsm()
    to_berth(fsm)
    held = tick(fsm, 1.0, plan=FAR, clearance=None)
    assert held.state == DockState.TO_BERTH and held.linear == 0.0
    assert tick(fsm, 1.5, plan=FAR, clearance=None).state == DockState.TO_BERTH
    assert tick(fsm, 1.8, plan=FAR, clearance=None).state == DockState.ABORT


def test_berth_wall_flicker_does_not_abort():
    fsm = DockingFsm()
    to_berth(fsm)
    tick(fsm, 1.0, plan=FAR, clearance=None)
    assert tick(fsm, 1.2, plan=FAR).state == DockState.TO_BERTH   # 복구
    tick(fsm, 1.3, plan=FAR, clearance=None)
    assert tick(fsm, 1.9, plan=FAR).state == DockState.TO_BERTH   # 타이머 초기화


def at_mouth_with(fsm, clearance, now):
    return tick(fsm, now, plan=AT_MOUTH, clearance=clearance)


BAD_CLEARANCES = (None, SlotClearance(1.0, None, 1.0),
                  SlotClearance(0.4, 0.4, 1.0))   # 미관측 / 한쪽만 / 폭 1.8m


def test_slot_check_failure_waits_still_before_deciding():
    for clearance in BAD_CLEARANCES:
        fsm = DockingFsm()
        to_mouth(fsm)
        for now in (0.9, 1.5, 3.0):                 # 대기 2.5초 이내
            out = at_mouth_with(fsm, clearance, now)
            assert out.state == DockState.TO_MOUTH
            assert out.linear == 0.0 and out.angular == 0.0


def test_slot_check_recovers_during_wait_and_enters():
    fsm = DockingFsm()
    to_mouth(fsm)
    assert at_mouth_with(fsm, None, 0.9).state == DockState.TO_MOUTH
    assert at_mouth_with(fsm, None, 2.0).state == DockState.TO_MOUTH
    assert at_mouth_with(fsm, GOOD_SLOT, 2.5).state == DockState.TO_BERTH


def test_slot_check_failure_after_wait_relocates_to_retry_point():
    for clearance in BAD_CLEARANCES:
        fsm = DockingFsm()
        to_mouth(fsm)
        at_mouth_with(fsm, clearance, 0.9)
        out = at_mouth_with(fsm, clearance, 3.5)    # 0.9 + 2.5 초과
        assert out.state == DockState.RELOCATE
        assert out.use_waypoint and out.use_repulsion
        # 도착하면 탐색부터 다시 시작해 정상 진입까지 이어진다.
        assert fsm.step(4.0, 1.0, None, relocate_reached=True,
                        clearance=None).state == DockState.ACQUIRE
        assert tick(fsm, 4.1).state == DockState.PRE_ENTRY


def test_slot_check_failure_aborts_when_no_retry_left():
    fsm = DockingFsm(DockParams(acquire_retries=0))
    to_mouth(fsm)
    at_mouth_with(fsm, None, 0.9)
    out = at_mouth_with(fsm, None, 3.5)
    assert out.state == DockState.ABORT and '옆벽' in out.event


def test_slot_and_acquire_failures_share_one_retry():
    fsm = DockingFsm(DockParams(acquire_max_time=1.0))
    tick(fsm, 0.0, plan=None)
    assert tick(fsm, 1.0, plan=None).state == DockState.RELOCATE   # 재시도 소진
    fsm.step(2.0, 1.0, None, relocate_reached=True)
    assert tick(fsm, 2.1).state == DockState.PRE_ENTRY
    tick(fsm, 2.2, plan=AT_PRE)
    tick(fsm, 2.3, plan=AT_PRE)
    assert tick(fsm, 2.9, plan=AT_PRE).state == DockState.TO_MOUTH
    at_mouth_with(fsm, None, 3.0)
    out = at_mouth_with(fsm, None, 6.0)
    assert out.state == DockState.ABORT and '폭' not in out.event


def test_wait_timer_resets_if_boat_leaves_the_mouth():
    fsm = DockingFsm()
    to_mouth(fsm)
    at_mouth_with(fsm, None, 0.9)
    tick(fsm, 2.0, plan=AT_PRE, clearance=None)     # 입구 반경 밖으로 밀림
    at_mouth_with(fsm, None, 2.5)                   # 대기 새로 시작
    assert at_mouth_with(fsm, None, 4.0).state == DockState.TO_MOUTH
    assert at_mouth_with(fsm, None, 5.2).state == DockState.RELOCATE


def test_berth_slows_immediately_but_aborts_only_after_hold():
    fsm = DockingFsm()
    to_berth(fsm)
    centered = tick(fsm, 1.0, plan=FAR)
    near = SlotClearance(0.45, 1.55, 1.0)           # 0.35~0.6m: 감속만
    out = tick(fsm, 1.1, plan=FAR, clearance=near)
    assert out.state == DockState.TO_BERTH and 0 < out.linear < centered.linear
    close = SlotClearance(0.30, 1.70, 1.0)          # 0.35m 미만
    first = tick(fsm, 1.2, plan=FAR, clearance=close)
    assert first.state == DockState.TO_BERTH
    assert 0 < first.linear <= out.linear           # 확인 중 최저 속도
    assert tick(fsm, 1.4, plan=FAR, clearance=close).state == DockState.TO_BERTH
    final = tick(fsm, 1.55, plan=FAR, clearance=close)   # 0.35초 연속
    assert final.state == DockState.ABORT and '간격' in final.event
    assert final.linear == 0.0


def test_single_frame_close_wall_does_not_abort():
    fsm = DockingFsm()
    to_berth(fsm)
    close = SlotClearance(0.30, 1.70, 1.0)
    assert tick(fsm, 1.0, plan=FAR, clearance=close).state == DockState.TO_BERTH
    assert tick(fsm, 1.1, plan=FAR).state == DockState.TO_BERTH       # 정상 복귀
    tick(fsm, 1.2, plan=FAR, clearance=close)                          # 타이머 재시작
    assert tick(fsm, 1.4, plan=FAR, clearance=close).state == DockState.TO_BERTH
    assert tick(fsm, 1.6, plan=FAR, clearance=close).state == DockState.ABORT


def test_mouth_approach_aborts_when_off_center_persistently():
    fsm = DockingFsm()
    to_mouth(fsm)
    off = SlotClearance(0.2, 1.8, 1.0)
    assert tick(fsm, 0.85, plan=FAR, clearance=off).state == DockState.TO_MOUTH
    assert tick(fsm, 1.0, plan=FAR, clearance=off).state == DockState.TO_MOUTH
    assert tick(fsm, 1.2, plan=FAR, clearance=off).state == DockState.ABORT
