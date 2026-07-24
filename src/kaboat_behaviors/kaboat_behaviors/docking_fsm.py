"""ROS 비의존 도킹 상태기계.

도킹 behavior의 판단 로직을 카메라/ROS 배선에서 분리한다.

  APPROACH → ACQUIRE → ALIGN → ENTER → HOLD → REVERSE → COMPLETE

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
    ALIGN = 'ALIGN'
    ENTER = 'ENTER'
    HOLD = 'HOLD'
    REVERSE = 'REVERSE'
    COMPLETE = 'COMPLETE'


@dataclass(frozen=True)
class DockTarget:
    bearing: float
    distance: float


@dataclass(frozen=True)
class DockParams:
    staging_radius: float = 1.5
    align_tolerance: float = math.radians(5.0)
    enter_tolerance: float = math.radians(10.0)
    align_hold: float = 0.5
    berth_distance: float = 1.0
    hold_time: float = 3.0
    reverse_goal_radius: float = 1.8
    reverse_min_time: float = 0.5
    mark_lost_timeout: float = 0.7
    scan_half_period: float = 2.0
    scan_angular: float = 0.35
    align_speed: float = 0.18
    enter_speed: float = 0.28
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
        self._lost_since = None

    def _transition(self, state: DockState, now: float) -> str:
        previous = self.state
        self.state = state
        self._state_since = now
        self._aligned_since = None
        self._lost_since = None
        return f'{previous.value} → {state.value}'

    def _steer(self, bearing: float) -> float:
        return _clamp(self.p.steer_kp * bearing, -1.0, 1.0)

    def step(self, now: float, staging_distance: float,
             target: DockTarget | None) -> DockOutput:
        """현재 관측 하나를 반영해 이번 tick의 명령을 반환한다."""
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
            if target is not None and target.distance >= 0.0:
                event = self._transition(DockState.ALIGN, now)
                return DockOutput(
                    self.state, angular=self._steer(target.bearing),
                    event=event)
            phase = int((now - self._state_since) / self.p.scan_half_period)
            direction = 1.0 if phase % 2 == 0 else -1.0
            return DockOutput(
                self.state, angular=direction * self.p.scan_angular,
                use_repulsion=True)

        if self.state in (DockState.ALIGN, DockState.ENTER):
            if target is None or target.distance < 0.0:
                if self._lost_since is None:
                    self._lost_since = now
                if now - self._lost_since >= self.p.mark_lost_timeout:
                    event = self._transition(DockState.ACQUIRE, now)
                    return DockOutput(
                        self.state, angular=self.p.scan_angular,
                        use_repulsion=True, event=event)
                return DockOutput(self.state)
            self._lost_since = None

        if self.state == DockState.ALIGN:
            assert target is not None
            if abs(target.bearing) <= self.p.align_tolerance:
                if self._aligned_since is None:
                    self._aligned_since = now
                elif now - self._aligned_since >= self.p.align_hold:
                    event = self._transition(DockState.ENTER, now)
                    return DockOutput(
                        self.state, linear=self.p.enter_speed,
                        angular=self._steer(target.bearing), event=event)
            else:
                self._aligned_since = None
            linear = (self.p.align_speed
                      if abs(target.bearing) <= 4.0 * self.p.align_tolerance
                      else 0.0)
            return DockOutput(
                self.state, linear=linear,
                angular=self._steer(target.bearing))

        if self.state == DockState.ENTER:
            assert target is not None
            berthed = (
                target.distance <= self.p.berth_distance
                and abs(target.bearing) <= self.p.enter_tolerance
            )
            if berthed:
                event = self._transition(DockState.HOLD, now)
                return DockOutput(self.state, event=event)
            return DockOutput(
                self.state, linear=self.p.enter_speed,
                angular=self._steer(target.bearing))

        if self.state == DockState.HOLD:
            if now - self._state_since >= self.p.hold_time:
                event = self._transition(DockState.REVERSE, now)
                return DockOutput(
                    self.state, linear=-self.p.reverse_speed, event=event)
            return DockOutput(self.state)

        if self.state == DockState.REVERSE:
            enough_time = now - self._state_since >= self.p.reverse_min_time
            if enough_time and staging_distance <= self.p.reverse_goal_radius:
                event = self._transition(DockState.COMPLETE, now)
                return DockOutput(
                    self.state, complete=True, event=event)
            return DockOutput(
                self.state, linear=-self.p.reverse_speed)

        return DockOutput(
            DockState.COMPLETE, complete=True)
