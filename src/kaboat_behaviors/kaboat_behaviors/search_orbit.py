"""search_orbit — 탐색 미션 계산 (ROS 무의존). search_circler 노드는 입출력만 맡는다.

규정 5.2(6) (2026-11): 사전에 주어진 색의 부표를 중심으로 원을 그리며 선회한다.
  빨강·초록 → 우현선회(시계, cw), 검정(노랑에서 바뀜) → 좌현선회(반시계, ccw).
  목표 색은 대회 당일 공지 → search.target_color 로 넣는다.
  기록(3구간)은 "목표부표에서 원을 그리는 선회기동 수행이 완료되는 순간"까지.

단계
  SEEK     목표 색 부표를 아직 못 찾음 — 탐색 목표점(search_end)으로 가고, 도착하면
           제자리 선회로 둘러본다.
  ORBIT    목표 부표 고정 — 부표 중심 반경 orbit_radius 원을 LOS 로 추종한다. 원 띠
           (|거리 − orbit_radius| ≤ orbit_band) 안에 있을 때만 부표 둘레를 규정 방향으로
           돈 각도를 쌓고, laps 바퀴가 되면 DONE.
  DONE     완료 — 노드가 완료 신호 1회 + 탐색 목표점으로 (mission_manager 전환 = 완료 +
           search_end 2 m 안).
  GIVE_UP  SEEK 에서 목표점에 도착하고 give_up_sec 동안 못 찾음 — 완료 신호를 내고 다음
           미션으로 넘긴다 (탐색 점수는 없지만 15분 제한 안에 회피를 하러 간다).

목표 부표 고정: buoy_detector 부표맵은 확정 부표를 시야 밖에서도 유지하지만, 보여야 하는
  자리에서 놓치면 지우고 색 투표가 뒤집힐 수도 있다. 선회 도중 중심이 사라지거나 바뀌면
  안 되므로 고른 부표 위치는 여기서 기억한다. 같은 색 부표가 고정 위치 relock_radius
  안에 있으면 그쪽으로 조금씩(refine_alpha) 보정한다.
후보: 목표 색 부표 중 탐색 목표점에서 target_search_radius 안, 목표점에 가장 가까운 것.
  각도를 쌓기 시작하기 전에는 목표점에 switch_margin 이상 더 가까운 후보가 나오면 바꾼다
  — 경기장 주황 부표는 buoy_detector 에서 빨강으로 나오므로(BUOY_DETECTOR.md), 멀리서
  주황 부표를 먼저 고른 뒤 진짜 목표가 보이는 경우를 위한 것이다.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

TARGET_COLORS = ('red', 'green', 'black')
# 규정 5.2(6): 빨강·초록 우현(시계), 검정(구 노랑) 좌현(반시계)
TURN_DIRECTION = {'red': 'cw', 'green': 'cw', 'black': 'ccw'}


@dataclass(frozen=True)
class SearchParams:
    target_color: str = ''            # 대회 당일 공지 — red / green / black. '' 이면 미설정
    orbit_radius: float = 2.0         # 부표 중심 → 선회 원 반경 [m] (선체 1.1 m + 부표 반경 여유)
    orbit_band: float = 1.0           # 원에서 이만큼 안쪽·바깥에 있을 때만 선회 각도를 센다 [m]
    laps: float = 1.1                 # 완료 바퀴 수 — 한 바퀴 + 36° 여유 (원이 확실히 닫히게)
    target_search_radius: float = 10.0  # 탐색 목표점에서 이 안의 부표만 후보 [m]
    relock_radius: float = 1.5        # 고정 위치 이 안의 같은 색 부표 = 같은 부표 [m]
    refine_alpha: float = 0.2         # 다시 보일 때 고정 위치 보정 비율 (tick 당)
    recenter_distance: float = 0.3    # 고정 위치가 이만큼 바뀌면 원 경로를 다시 만든다 [m]
    switch_margin: float = 2.0        # 목표점에 이만큼 더 가까운 후보가 나오면 바꾼다 [m]
    arrive_radius: float = 1.5        # 부표 못 찾았을 때 목표점 도착 반경 → 제자리 둘러보기 [m]
    scan_turn: float = 0.15           # 제자리 둘러보기 회전 명령 (최대 추력 비율)
    give_up_sec: float = 90.0         # 목표점 도착 후 이만큼 못 찾으면 포기 [s]
    orbit_thrust: float = 0.25        # 선회 LOS 직진 (공용 LOS 직진 0.25)
    orbit_turn: float = 0.31          # 선회 LOS 회전 상한 (공용 LOS 회전 0.31)
    ignore_radius: float = 1.0        # 선회 중 회피 반발에서 뺄 목표 부표 둘레 반경 [m]

    def __post_init__(self):
        if self.target_color not in ('',) + TARGET_COLORS:
            raise ValueError(f"search.target_color must be one of {TARGET_COLORS} or ''")
        positive = (self.orbit_radius, self.orbit_band, self.laps, self.target_search_radius,
                    self.relock_radius, self.recenter_distance, self.arrive_radius,
                    self.give_up_sec, self.ignore_radius)
        if not all(math.isfinite(v) and v > 0.0 for v in positive):
            raise ValueError('search radii, laps and give_up_sec must be finite and > 0')
        unit = (self.refine_alpha, self.scan_turn, self.orbit_thrust, self.orbit_turn)
        if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in unit):
            raise ValueError('search refine_alpha / scan_turn / orbit_thrust / orbit_turn '
                             'must be in [0, 1]')
        if not (math.isfinite(self.switch_margin) and self.switch_margin >= 0.0):
            raise ValueError('search.switch_margin must be finite and >= 0')
        if self.ignore_radius >= self.orbit_radius:
            raise ValueError('search.ignore_radius must be smaller than orbit_radius')

    @property
    def direction(self) -> Optional[str]:
        return TURN_DIRECTION.get(self.target_color)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _xy(buoy) -> Tuple[float, float]:
    return float(buoy.position.x), float(buoy.position.y)


class SearchFsm:
    """탐색 단계 판정. 매 tick update() 를 부르고 phase/center 를 읽어 주행한다.

    buoys 는 color 와 position.x/.y 를 가진 객체 (kaboat_msgs/Buoy 또는 같은 모양).
    """

    def __init__(self, params: SearchParams, goal_xy):
        if params.direction is None:
            raise ValueError('search.target_color is not set')
        self.p = params
        self.goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.phase = 'SEEK'
        self.center: Optional[Tuple[float, float]] = None
        self.swept = 0.0          # 규정 방향으로 돈 각도 [rad]
        self.counting = False     # 한 번이라도 원 띠 안에서 각도를 셌는가
        self._prev_theta = None
        self._scan_since = None
        self.switches = 0

    @property
    def direction(self) -> str:
        return self.p.direction

    @property
    def done(self) -> bool:
        return self.phase in ('DONE', 'GIVE_UP')

    @property
    def target_laps(self) -> float:
        return self.swept / (2.0 * math.pi)

    def _dist_goal(self, xy) -> float:
        return math.hypot(xy[0] - self.goal[0], xy[1] - self.goal[1])

    def candidates(self, buoys):
        """목표 색이고 목표점 target_search_radius 안 — 목표점에 가까운 순."""
        out = []
        for b in buoys:
            if b.color != self.p.target_color:
                continue
            xy = _xy(b)
            if not all(math.isfinite(v) for v in xy):
                continue
            d = self._dist_goal(xy)
            if d <= self.p.target_search_radius:
                out.append((d, xy))
        out.sort(key=lambda t: t[0])
        return [xy for _, xy in out]

    def update(self, buoys, boat_xy, now: float) -> str:
        if self.done:
            return self.phase
        cands = self.candidates(buoys)
        if self.center is None:
            if cands:
                self._lock(cands[0])
            else:
                self._seek(boat_xy, now)
                return self.phase
        else:
            self._refine(buoys)
            if not self.counting and cands:
                best = cands[0]
                far_from_lock = math.dist(best, self.center) > self.p.relock_radius
                closer = (self._dist_goal(best) + self.p.switch_margin
                          < self._dist_goal(self.center))
                if far_from_lock and closer:
                    self._lock(best)
                    self.switches += 1
        self._sweep(boat_xy)
        if self.swept >= self.p.laps * 2.0 * math.pi:
            self.phase = 'DONE'
        return self.phase

    # ── 내부 ──────────────────────────────────────────────
    def _lock(self, xy):
        self.center = (float(xy[0]), float(xy[1]))
        self.phase = 'ORBIT'
        self.swept = 0.0
        self.counting = False
        self._prev_theta = None

    def _seek(self, boat_xy, now):
        if self._dist_goal(boat_xy) > self.p.arrive_radius:
            return   # 목표점 접근 중 — 포기 시계는 도착해야 돈다
        if self._scan_since is None:
            self._scan_since = now
        elif now - self._scan_since >= self.p.give_up_sec:
            self.phase = 'GIVE_UP'

    def _refine(self, buoys):
        cx, cy = self.center
        best, best_d = None, self.p.relock_radius
        for b in buoys:
            if b.color != self.p.target_color:
                continue
            xy = _xy(b)
            d = math.hypot(xy[0] - cx, xy[1] - cy)
            if d <= best_d:
                best, best_d = xy, d
        if best is not None:
            a = self.p.refine_alpha
            self.center = ((1.0 - a) * cx + a * best[0], (1.0 - a) * cy + a * best[1])

    def _sweep(self, boat_xy):
        dx, dy = boat_xy[0] - self.center[0], boat_xy[1] - self.center[1]
        if abs(math.hypot(dx, dy) - self.p.orbit_radius) > self.p.orbit_band:
            self._prev_theta = None   # 띠 밖 — 이 구간 각도는 안 센다
            return
        theta = math.atan2(dy, dx)
        if self._prev_theta is not None:
            d = _wrap(theta - self._prev_theta)
            # 시계(cw)는 각도가 줄어드는 쪽이 전진
            self.swept += -d if self.direction == 'cw' else d
        self._prev_theta = theta
        self.counting = True
