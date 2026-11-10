"""ROS 비의존 도킹 상태기계.

도킹 behavior의 판단 로직을 카메라/ROS 배선에서 분리한다.

  APPROACH → ACQUIRE → PRE_ENTRY → ALIGN → TO_MOUTH → TO_BERTH
  → BERTH_VERIFY → HOLD → REVERSE → COMPLETE.

ACQUIRE가 acquire_max_time 안에 표식을 못 찾으면 acquire_retries 횟수만큼
RELOCATE(지정 재시도 waypoint로 이동) 후 ACQUIRE를 다시 시도하고, 횟수를
소진하면 ABORT한다.

슬롯 옆벽은 점유 격자에서 잰 SlotClearance(slot_clearance.py)로 본다.
TO_MOUTH→TO_BERTH 전환 시 양쪽 벽이 보이고 벽-벽 폭이 slot_width 와 맞아야
한다. 아니면 slot_wait_time 동안 정지해 기다리고, 그래도 안 되면 ACQUIRE 실패와
같은 재시도 횟수로 RELOCATE 하며 횟수가 없으면 ABORT 한다. 진입 중 선체-벽
간격이 줄면 감속(side_slow_gap)하고, side_stop_gap 아래가 side_stop_hold 동안
이어질 때만 ABORT 한다(한 프레임 오탐으로 도킹이 끝나지 않게).

APPROACH는 mission waypoint까지 공통 위치/헤딩 제어를 사용한다. 표식을
획득·정렬한 뒤 슬롯 안으로 저속 진입하고, 잠시 정지한 다음 같은 선수각으로
후진한다. REVERSE가 waypoint 반경 안에 도달해야 COMPLETE가 되므로
mission_manager의 별도 5m 전환 조건과 모순되지 않는다.
"""
from dataclasses import dataclass
from enum import Enum
import math

from .slot_clearance import SlotClearance


class DockState(str, Enum):
    APPROACH = 'APPROACH'
    ACQUIRE = 'ACQUIRE'
    RELOCATE = 'RELOCATE'
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
    acquire_retries: int = 1          # 탐색 실패 시 재시도 waypoint 이동 횟수
    relocate_max_time: float = 60.0   # 재시도 waypoint 도착 제한
    scan_half_period: float = 2.0
    scan_angular: float = 0.35
    pre_entry_speed: float = 0.20
    mouth_speed: float = 0.16
    berth_speed: float = 0.12
    reverse_speed: float = 0.25
    steer_kp: float = 2.0
    # 슬롯 옆벽 (점유 격자 기반). 실측으로 보정할 초기값이다.
    slot_check_enabled: bool = True
    slot_width: float = 3.0           # 벽-벽 폭 [m]
    slot_width_tolerance: float = 0.5
    hull_width: float = 1.0           # 선체 실제 폭 [m]; 1.0 은 임시값
    hull_length: float = 1.0          # 선체 실제 전장 [m]; 1.0 은 임시값
    slot_look_ahead: float = 1.5      # 입구에서 앞쪽 벽을 보는 최대 거리 [m]
    slot_back_wall_margin: float = 0.4   # 창 끝~뒷벽 여유 (뒷벽을 옆벽으로 오인 방지)
    side_slow_gap: float = 0.6        # 선체-벽 간격이 이보다 좁으면 감속 [m]
    side_stop_gap: float = 0.35       # 이보다 좁으면 ABORT [m]
    side_min_speed_scale: float = 0.3
    side_stop_hold: float = 0.3       # stop_gap 아래가 이 시간 이어져야 ABORT [s]
    slot_wait_time: float = 2.5       # 입구 확인 실패 시 정지 대기 [s]


@dataclass(frozen=True)
class DockOutput:
    state: DockState
    linear: float = 0.0    # behavior max_linear 대비 비율 [-1, 1]
    angular: float = 0.0   # behavior max_angular 대비 비율 [-1, 1]
    use_waypoint: bool = False
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
        self._retries_used = 0
        self._wall_lost_since = None
        self._side_stop_since = None
        self._slot_wait_since = None

    def _transition(self, state: DockState, now: float) -> str:
        previous = self.state
        self.state = state
        self._state_since = now
        self._aligned_since = None
        self._berth_valid_since = None
        self._lost_since = None
        self._wall_lost_since = None
        self._side_stop_since = None
        self._slot_wait_since = None
        return f'{previous.value} → {state.value}'

    def _retry_or_abort(self, now: float, reason: str) -> DockOutput:
        """재시도 횟수가 남았으면 RELOCATE, 없으면 ABORT (ACQUIRE 와 공유)."""
        if self._retries_used < self.p.acquire_retries:
            self._retries_used += 1
            event = self._transition(DockState.RELOCATE, now)
            return DockOutput(
                self.state, use_waypoint=True, use_repulsion=True,
                event=f'{event} (재시도 {self._retries_used}/'
                      f'{self.p.acquire_retries}: {reason})')
        event = self._transition(DockState.ABORT, now)
        return DockOutput(self.state, event=f'{event} ({reason})')

    def _slot_invalid_reason(self, clearance) -> str | None:
        """입구 도착 시 양쪽 벽이 보이고 폭이 설계값과 맞는지 확인."""
        if not self.p.slot_check_enabled:
            return None
        if clearance is None or not clearance.both:
            return '양쪽 옆벽 미관측'
        error = abs(clearance.slot_width - self.p.slot_width)
        if error > self.p.slot_width_tolerance:
            return (f'슬롯 폭 불일치 {clearance.slot_width:.2f}m '
                    f'(설계 {self.p.slot_width:.2f}m)')
        return None

    def _side_guard(self, clearance, now: float, required: bool):
        """(중단 사유 | None, 속도 배율, 정지 여부) — 옆벽 접근 보호."""
        if not self.p.slot_check_enabled:
            return None, 1.0, False
        if clearance is None or not clearance.both:
            self._side_stop_since = None
            if required:
                # 슬롯 안에서는 벽이 양쪽에 보여야 한다. 끊기면 멈추고 대기한다.
                if self._wall_lost_since is None:
                    self._wall_lost_since = now
                if now - self._wall_lost_since >= self.p.mark_lost_timeout:
                    return '옆벽 관측 상실', 0.0, True
                return None, 0.0, True
            gap = clearance.min_gap if clearance is not None else None
            if gap is None:
                return None, 1.0, False
        else:
            self._wall_lost_since = None
            gap = clearance.min_gap
        if gap < self.p.side_stop_gap:
            # 한 프레임 오탐으로 도킹이 끝나지 않도록 연속 확인 후 ABORT.
            # 확인 중에는 최저 속도로 감속한다.
            if self._side_stop_since is None:
                self._side_stop_since = now
            if now - self._side_stop_since >= self.p.side_stop_hold:
                return f'옆벽 간격 {gap:.2f}m', 0.0, True
            return None, self.p.side_min_speed_scale, False
        self._side_stop_since = None
        span = max(self.p.side_slow_gap - self.p.side_stop_gap, 1e-6)
        scale = _clamp((gap - self.p.side_stop_gap) / span,
                       self.p.side_min_speed_scale, 1.0)
        return None, scale, False

    def _steer(self, bearing: float) -> float:
        return _clamp(self.p.steer_kp * bearing, -1.0, 1.0)

    def step(self, now: float, staging_distance: float,
             plan: DockPlan | None,
             reverse_distance: float = 0.0,
             relocate_reached: bool = False,
             clearance: SlotClearance | None = None) -> DockOutput:
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
                self.state, use_waypoint=True, use_repulsion=True)

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
                return self._retry_or_abort(now, '표식 탐색 실패')
            phase = int((now - self._state_since) / self.p.scan_half_period)
            direction = 1.0 if phase % 2 == 0 else -1.0
            return DockOutput(
                self.state, angular=direction * self.p.scan_angular,
                use_repulsion=True)

        if self.state == DockState.RELOCATE:
            # 재시도 waypoint에 위치·선수각이 모두 맞으면 탐색을 다시 시작한다.
            if relocate_reached:
                event = self._transition(DockState.ACQUIRE, now)
                return DockOutput(
                    self.state, angular=self.p.scan_angular,
                    use_repulsion=True, event=event)
            if now - self._state_since >= self.p.relocate_max_time:
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            return DockOutput(
                self.state, use_waypoint=True, use_repulsion=True)

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
            reason, scale, _ = self._side_guard(clearance, now, False)
            if reason is not None:
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=f'{event} ({reason})')
            if plan.mouth.distance <= self.p.mouth_radius:
                reason = self._slot_invalid_reason(clearance)
                if reason is None:
                    event = self._transition(DockState.TO_BERTH, now)
                    return DockOutput(self.state, event=event)
                # 점유 누적 부족 등 일시적일 수 있으니 정지해서 기다려 본다.
                if self._slot_wait_since is None:
                    self._slot_wait_since = now
                if now - self._slot_wait_since < self.p.slot_wait_time:
                    return DockOutput(self.state)
                return self._retry_or_abort(now, reason)
            self._slot_wait_since = None
            return DockOutput(
                self.state,
                linear=(self.p.mouth_speed * scale
                        * min(plan.mouth.distance / 1.0, 1.0)),
                angular=self._steer(
                    0.5 * plan.mouth.bearing + 0.5 * plan.axis_error))

        if self.state == DockState.TO_BERTH:
            assert plan is not None
            if (abs(plan.axis_error) > self.p.enter_tolerance
                    or abs(plan.marker.bearing) > self.p.enter_tolerance
                    or abs(plan.berth.bearing) > self.p.waypoint_bearing_limit):
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            reason, scale, stop = self._side_guard(clearance, now, True)
            if reason is not None:
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=f'{event} ({reason})')
            if stop:
                return DockOutput(self.state)
            if plan.marker.distance <= self.p.berth_distance:
                event = self._transition(DockState.BERTH_VERIFY, now)
                return DockOutput(self.state, event=event)
            if plan.berth.distance <= self.p.berth_point_radius:
                # W3에 도달했는데 표식 거리가 1m 밖이면 관측/기하가 모순된다.
                event = self._transition(DockState.ABORT, now)
                return DockOutput(self.state, event=event)
            return DockOutput(
                self.state,
                linear=(self.p.berth_speed * scale
                        * min(plan.berth.distance / 1.0, 1.0)),
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
