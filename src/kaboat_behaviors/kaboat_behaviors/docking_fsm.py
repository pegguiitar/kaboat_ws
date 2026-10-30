"""ROS 비의존 도킹 상태기계.

도킹 behavior의 판단 로직을 카메라/ROS 배선에서 분리한다.

  APPROACH → ACQUIRE → PRE_ENTRY → ALIGN → TO_MOUTH → TO_BERTH
  → BERTH_VERIFY → HOLD → REVERSE → COMPLETE.

APPROACH는 mission waypoint까지 기존 seek_goal 제어를 사용한다. 표식을
획득·정렬한 뒤 슬롯 안으로 저속 진입하고, 잠시 정지한 다음 같은 선수각으로
후진한다. REVERSE가 waypoint 반경 안에 도달해야 COMPLETE가 되므로
mission_manager의 별도 2m 전환 조건과 모순되지 않는다.
"""
from dataclasses import dataclass
from enum import Enum
import math


class DockState(str, Enum):
    APPROACH = 'APPROACH'
    ACQUIRE = 'ACQUIRE'
    PRE_ENTRY = 'PRE_ENTRY'
    ALIGN = 'ALIGN'
    TO_MOUTH = 'TO_MOUTH'
    TO_BERTH = 'TO_BERTH'
    BERTH_VERIFY = 'BERTH_VERIFY'
    HOLD = 'HOLD'
    REVERSE = 'REVERSE'
    COMPLETE = 'COMPLETE'
    ABORT = 'ABORT'


@dataclass(frozen=True)
class DockTarget:
    bearing: float
    distance: float


@dataclass(frozen=True)
class DockPlan:
    pre_entry: DockTarget
    mouth: DockTarget
    berth: DockTarget
    marker: DockTarget
    axis_error: float


@dataclass(frozen=True)
class DockParams:
    staging_radius: float = 1.5
    pre_entry_offset: float = 1.5
    pre_entry_radius: float = 0.4
    mouth_radius: float = 0.35
    berth_point_radius: float = 0.15
    align_tolerance: float = math.radians(5.0)
    enter_tolerance: float = math.radians(10.0)
    waypoint_bearing_limit: float = math.radians(20.0)
    align_hold: float = 0.5
    berth_distance: float = 1.0
    berth_verify_time: float = 0.5
    hold_time: float = 3.0
    reverse_goal_radius: float = 1.8
    reverse_min_time: float = 0.5
    reverse_min_distance: float = 2.5
    reverse_max_time: float = 30.0
    mark_lost_timeout: float = 0.7
    acquire_max_time: float = 15.0
    scan_half_period: float = 2.0
    scan_angular: float = 0.35
    pre_entry_speed: float = 0.20
    mouth_speed: float = 0.16
    berth_speed: float = 0.12
    reverse_speed: float = 0.25
    steer_kp: float = 2.0


@dataclass(frozen=True)
class DockOutput:
    state: DockState
    linear: float = 0.0    # behavior max_linear 대비 비율 [-1, 1]
    angular: float = 0.0   # behavior max_angular 대비 비율 [-1, 1]
    seek_goal: bool = False
    use_repulsion: bool = False
    complete: bool = False
    event: str | None = None


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class DockingFsm:
    def __init__(self, params: DockParams | None = None):
        self.p = params or DockParams()
        self.reset()

    def reset(self):
        self.state = DockState.APPROACH
        self._state_since = None
        self._aligned_since = None
        self._berth_valid_since = None
        self._lost_since = None

    def _transition(self, state: DockState, now: float) -> str:
        previous = self.state
        self.state = state
        self._state_since = now
        self._aligned_since = None
        self._berth_valid_since = None
        self._lost_since = None
        return f'{previous.value} → {state.value}'

    def _steer(self, bearing: float) -> float:
        return _clamp(self.p.steer_kp * bearing, -1.0, 1.0)

    def step(self, now: float, staging_distance: float,
             plan: DockPlan | None,
             reverse_distance: float = 0.0) -> DockOutput:
        """W1 대기점, W2 입구, W3 접안점을 순서대로 추종한다."""
        if self._state_since is None:
            self._state_since = now

        if self.state == DockState.APPROACH:
            if staging_distance <= self.p.staging_radius:
                event = self._transition(DockState.ACQUIRE, now)
                return DockOutput(
                    self.state, angular=self.p.scan_angular,
                    use_repulsion=True, event=event)
            return DockOutput(
                self.state, seek_goal=True, use_repulsion=True)

        if self.state == DockState.ACQUIRE:
            if plan is not None:
                if abs(plan.pre_entry.bearing) >= math.pi / 2:
                    event = self._transition(DockState.ABORT, now)
                    return DockOutput(self.state, event=event)
                event = self._transition(DockState.PRE_ENTRY, now)
                return DockOutput(
                    self.state, angular=self._steer(plan.pre_entry.bearing),
                    event=event)
            if now - self._state_since >= self.p.acquire_max_time:
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            phase = int((now - self._state_since) / self.p.scan_half_period)
            direction = 1.0 if phase % 2 == 0 else -1.0
            return DockOutput(
                self.state, angular=direction * self.p.scan_angular,
                use_repulsion=True)

        if self.state in (DockState.PRE_ENTRY, DockState.ALIGN,
                          DockState.TO_MOUTH, DockState.TO_BERTH,
                          DockState.BERTH_VERIFY):
            if plan is None:
                if self.state == DockState.BERTH_VERIFY:
                    self._berth_valid_since = None
                if self._lost_since is None:
                    self._lost_since = now
                if now - self._lost_since >= self.p.mark_lost_timeout:
                    event = self._transition(DockState.ABORT, now)
                    return DockOutput(self.state, event=event)
                return DockOutput(self.state)
            self._lost_since = None

        if self.state == DockState.PRE_ENTRY:
            assert plan is not None
            if plan.pre_entry.distance <= self.p.pre_entry_radius:
                event = self._transition(DockState.ALIGN, now)
                return DockOutput(self.state, event=event)
            bearing = plan.pre_entry.bearing
            linear = (self.p.pre_entry_speed *
                      min(plan.pre_entry.distance / 1.0, 1.0)
                      if abs(bearing) <= self.p.waypoint_bearing_limit else 0.0)
            return DockOutput(self.state, linear=linear,
                              angular=self._steer(bearing),
                              use_repulsion=True)

        if self.state == DockState.ALIGN:
            assert plan is not None
            if abs(plan.axis_error) <= self.p.align_tolerance:
                if self._aligned_since is None:
                    self._aligned_since = now
                elif now - self._aligned_since >= self.p.align_hold:
                    event = self._transition(DockState.TO_MOUTH, now)
                    return DockOutput(self.state, event=event)
            else:
                self._aligned_since = None
            return DockOutput(self.state,
                              angular=self._steer(plan.axis_error))

        if self.state == DockState.TO_MOUTH:
            assert plan is not None
            if (abs(plan.axis_error) > self.p.enter_tolerance
                    or abs(plan.mouth.bearing) > self.p.waypoint_bearing_limit):
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            if plan.mouth.distance <= self.p.mouth_radius:
                event = self._transition(DockState.TO_BERTH, now)
                return DockOutput(self.state, event=event)
            return DockOutput(
                self.state,
                linear=self.p.mouth_speed * min(plan.mouth.distance / 1.0, 1.0),
                angular=self._steer(
                    0.5 * plan.mouth.bearing + 0.5 * plan.axis_error))

        if self.state == DockState.TO_BERTH:
            assert plan is not None
            if (abs(plan.axis_error) > self.p.enter_tolerance
                    or abs(plan.marker.bearing) > self.p.enter_tolerance
                    or abs(plan.berth.bearing) > self.p.waypoint_bearing_limit):
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            if plan.marker.distance <= self.p.berth_distance:
                event = self._transition(DockState.BERTH_VERIFY, now)
                return DockOutput(self.state, event=event)
            if plan.berth.distance <= self.p.berth_point_radius:
                # W3에 도달했는데 표식 거리가 1m 밖이면 관측/기하가 모순된다.
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            return DockOutput(
                self.state,
                linear=self.p.berth_speed * min(plan.berth.distance / 1.0, 1.0),
                angular=self._steer(
                    0.5 * plan.berth.bearing + 0.5 * plan.axis_error))

        if self.state == DockState.BERTH_VERIFY:
            # 한 번 접안 후보가 되면 검증이 끝날 때까지 추진하지 않는다.
            # 거리/각도 관측이 불안정하면 재진입 대신 안전하게 정지한다.
            assert plan is not None
            if (plan.marker.distance > self.p.berth_distance
                    or abs(plan.marker.bearing) > self.p.enter_tolerance
                    or abs(plan.axis_error) > self.p.enter_tolerance):
                self._berth_valid_since = None
                if self._lost_since is None:
                    self._lost_since = now
                if now - self._lost_since >= self.p.mark_lost_timeout:
                    event = self._transition(DockState.ABORT, now)
                    return DockOutput(self.state, event=event)
                return DockOutput(self.state)
            self._lost_since = None
            if self._berth_valid_since is None:
                self._berth_valid_since = now
            if now - self._berth_valid_since >= self.p.berth_verify_time:
                event = self._transition(DockState.HOLD, now)
                return DockOutput(self.state, event=event)
            return DockOutput(self.state)

        if self.state == DockState.HOLD:
            if now - self._state_since >= self.p.hold_time:
                event = self._transition(DockState.REVERSE, now)
                return DockOutput(
                    self.state, linear=-self.p.reverse_speed, event=event)
            return DockOutput(self.state)

        if self.state == DockState.REVERSE:
            if now - self._state_since >= self.p.reverse_max_time:
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            enough_time = now - self._state_since >= self.p.reverse_min_time
            enough_distance = reverse_distance >= self.p.reverse_min_distance
            if (enough_time and enough_distance
                    and staging_distance <= self.p.reverse_goal_radius):
                event = self._transition(DockState.COMPLETE, now)
                return DockOutput(
                    self.state, complete=True, event=event)
            return DockOutput(
                self.state, linear=-self.p.reverse_speed)

        if self.state == DockState.ABORT:
            return DockOutput(self.state)
        return DockOutput(DockState.COMPLETE, complete=True)
