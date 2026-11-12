"""search_orbit — 탐색 미션 계산 (ROS 무의존). search_circler 노드는 입출력만 맡는다.

규정 5.2(6) (2026-11): 사전에 주어진 색의 부표를 중심으로 원을 그리며 선회한다.
  빨강·초록 → 우현선회(시계, cw), 검정(노랑에서 바뀜) → 좌현선회(반시계, ccw).
  목표 색은 대회 당일 공지 → search.target_color 로 넣는다.
  검정은 buoy_detector 가 색으로 판정하지 않는다 — 빨강·초록 아닌 부표 크기 물체는 'unknown'
  으로 나오므로(소거법, BUOY_DETECTOR.md), 검정 목표는 unknown 부표에서 고른다 (MATCH_COLORS).
  기록(3구간)은 "목표부표에서 원을 그리는 선회기동 수행이 완료되는 순간"까지.

단계
  SEEK     목표 색 부표를 아직 못 찾음 — 탐색 목표점(search_end)으로 가고, 도착하면
           제자리 선회로 둘러본다.
  ORBIT    목표 부표 고정 — 부표 중심 반경 orbit_radius 원을 LOS 로 추종한다. 원 띠
           (|거리 − orbit_radius| ≤ orbit_band) 안에 있을 때만 부표 둘레를 규정 방향으로
           돈 각도를 쌓고, laps 바퀴가 되면 DONE.
  DONE     완료 — 노드가 완료 신호 1회 + 탐색 목표점으로 (mission_manager 전환 = 완료 +
           search_end 2 m 안). search_end 가 목표 부표 위(done_clearance 안)에 찍혀 있으면
           부표에 들이받지 않게 부표에서 배 쪽으로 done_clearance 떨어진 점으로 간다
           (exit_point — 그래도 search_end 2 m 안이다).
  GIVE_UP  SEEK 에서 목표점에 도착하고 give_up_sec 동안 못 찾음 — 완료 신호를 내고 다음
           미션으로 넘긴다 (탐색 점수는 없지만 15분 제한 안에 회피를 하러 간다).

목표 부표 고정: 부표 검출 노드(buoy_lidar_detector)는 배에서 멀어진 부표를 지우고 색이 잠깐씩
  바뀔 수도 있다. 선회 도중 중심이 사라지거나 바뀌면 안 되므로 고른 부표 위치는 여기서 기억한다.
  고정할 때 검출 노드의 부표 ID 를 기억해 그 ID 로만 조금씩(refine_alpha) 보정하고, ID 가
  사라지면 relock_radius(0.6 m) 안 같은 색 부표를 이어받는다 — 1.2 m 붙은 초록·검정 쌍에서
  옆 부표로 끌려가지 않게. 고정한 부표가 목표 아닌 빨강·초록으로 분명히 바뀌면 고정을 풀고
  다시 찾는다. 목표 색이 lock_confirm_sec(1 s) 이어진 부표만 고른다.
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
# 목표 색 → 부표맵에서 받아들일 색. 검정 = 빨강·초록 아닌 물체(unknown).
MATCH_COLORS = {'red': ('red',), 'green': ('green',), 'black': ('unknown', 'black')}


@dataclass(frozen=True)
class SearchParams:
    target_color: str = ''            # 대회 당일 공지 — red / green / black. '' 이면 미설정
    orbit_radius: float = 2.0         # 부표 중심 → 선회 원 반경 [m] (선체 1.1 m + 부표 반경 여유)
    orbit_band: float = 1.0           # 원에서 이만큼 안쪽·바깥에 있을 때만 선회 각도를 센다 [m]
    laps: float = 1.1                 # 완료 바퀴 수 — 한 바퀴 + 36° 여유 (원이 확실히 닫히게)
    target_search_radius: float = 10.0  # 탐색 목표점에서 이 안의 부표만 후보 [m]
    relock_radius: float = 0.6        # 고정한 부표 ID 가 사라지면 이 안의 같은 색 부표를 이어받음 [m]
                                      # (1.2 m 붙은 초록·검정 쌍보다 작게 — 예전 1.5 는 옆 부표로 끌려갔다)
    lock_confirm_sec: float = 1.0     # 목표 색이 이만큼 이어진 부표만 고른다 — 검출 노드는 카메라
                                      # 근거가 적을 때 일단 unknown 으로 내서, 검정 목표가 색이 덜
                                      # 정해진 초록·빨강 부표를 잡지 않게
    refine_alpha: float = 0.2         # 다시 보일 때 고정 위치 보정 비율 (tick 당)
    recenter_distance: float = 0.3    # 고정 위치가 이만큼 바뀌면 원 경로를 다시 만든다 [m]
    switch_margin: float = 2.0        # 목표점에 이만큼 더 가까운 후보가 나오면 바꾼다 [m]
    arrive_radius: float = 1.5        # 부표 못 찾았을 때 목표점 도착 반경 → 제자리 둘러보기 [m]
    scan_turn: float = 0.15           # 제자리 둘러보기 회전 명령 (최대 추력 비율)
    give_up_sec: float = 90.0         # 목표점 도착 후 이만큼 못 찾으면 포기 [s]
    orbit_thrust: float = 0.25        # 선회 LOS 직진 (공용 LOS 직진 0.25)
    orbit_turn: float = 0.31          # 선회 LOS 회전 상한 (공용 LOS 회전 0.31)
    ignore_radius: float = 1.0        # 선회 중 회피 반발에서 뺄 목표 부표 둘레 반경 [m]
    done_clearance: float = 1.5       # 완료 뒤 목표 부표에 이보다 가까이 안 감 [m]
                                      # (mission_manager transition_radius 2.0 보다 작게)

    def __post_init__(self):
        if self.target_color not in ('',) + TARGET_COLORS:
            raise ValueError(f"search.target_color must be one of {TARGET_COLORS} or ''")
        positive = (self.orbit_radius, self.orbit_band, self.laps, self.target_search_radius,
                    self.relock_radius, self.recenter_distance, self.arrive_radius,
                    self.give_up_sec, self.ignore_radius, self.done_clearance)
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
        self.locked_id = None     # 고정한 부표의 검출 노드 ID (없으면 위치로만)
        self.unlocks = 0
        self._match_since = {}    # 부표 키 → 목표 색으로 처음 보인 시각

    @property
    def direction(self) -> str:
        return self.p.direction

    @property
    def match(self):
        return MATCH_COLORS[self.p.target_color]

    @property
    def done(self) -> bool:
        return self.phase in ('DONE', 'GIVE_UP')

    @property
    def target_laps(self) -> float:
        return self.swept / (2.0 * math.pi)

    def exit_point(self, boat_xy) -> Tuple[float, float]:
        """완료 뒤 갈 점 — 보통 탐색 목표점, 목표점이 부표 위면 부표에서 배 쪽으로 비킨 점."""
        if self.center is None or math.dist(self.goal, self.center) >= self.p.done_clearance:
            return self.goal
        cx, cy = self.center
        vx, vy = boat_xy[0] - cx, boat_xy[1] - cy
        n = math.hypot(vx, vy)
        if n < 1e-6:
            vx, vy, n = 1.0, 0.0, 1.0
        k = self.p.done_clearance / n
        return (cx + vx * k, cy + vy * k)

    def _dist_goal(self, xy) -> float:
        return math.hypot(xy[0] - self.goal[0], xy[1] - self.goal[1])

    def candidates(self, buoys, now: Optional[float] = None):
        """목표 색(MATCH_COLORS)이고 목표점 target_search_radius 안 — 목표점에 가까운 순
        [(xy, id)]. now 를 주면 목표 색이 lock_confirm_sec 이상 이어진 부표만."""
        out, seen = [], set()
        for b in buoys:
            if b.color not in self.match:
                continue
            xy = _xy(b)
            if not all(math.isfinite(v) for v in xy):
                continue
            d = self._dist_goal(xy)
            if d > self.p.target_search_radius:
                continue
            bid = getattr(b, 'id', None)
            key = bid if bid is not None else (round(xy[0] * 2), round(xy[1] * 2))
            seen.add(key)
            if now is not None:
                first = self._match_since.setdefault(key, now)
                if now - first < self.p.lock_confirm_sec:
                    continue
            out.append((d, xy, bid))
        if now is not None:
            self._match_since = {k: v for k, v in self._match_since.items() if k in seen}
        out.sort(key=lambda t: t[0])
        return [(xy, bid) for _, xy, bid in out]

    def update(self, buoys, boat_xy, now: float) -> str:
        if self.done:
            return self.phase
        cands = self.candidates(buoys, now)
        if self.center is not None and not self._refine(buoys):
            self._unlock()                 # 고정한 부표가 목표 아닌 색으로 바뀜 — 다시 찾기
        if self.center is None:
            if cands:
                self._lock(*cands[0])
            else:
                self._seek(boat_xy, now)
                return self.phase
        elif not self.counting and cands:
            best, bid = cands[0]
            far_from_lock = math.dist(best, self.center) > self.p.relock_radius
            closer = (self._dist_goal(best) + self.p.switch_margin
                      < self._dist_goal(self.center))
            if far_from_lock and closer:
                self._lock(best, bid)
                self.switches += 1
        self._sweep(boat_xy)
        if self.swept >= self.p.laps * 2.0 * math.pi:
            self.phase = 'DONE'
        return self.phase

    # ── 내부 ──────────────────────────────────────────────
    def _lock(self, xy, bid=None):
        self.center = (float(xy[0]), float(xy[1]))
        self.locked_id = bid
        self.phase = 'ORBIT'
        self.swept = 0.0
        self.counting = False
        self._prev_theta = None

    def _unlock(self):
        self.center = None
        self.locked_id = None
        self.phase = 'SEEK'
        self.swept = 0.0
        self.counting = False
        self._prev_theta = None
        self.unlocks += 1

    def _seek(self, boat_xy, now):
        if self._dist_goal(boat_xy) > self.p.arrive_radius:
            return   # 목표점 접근 중 — 포기 시계는 도착해야 돈다
        if self._scan_since is None:
            self._scan_since = now
        elif now - self._scan_since >= self.p.give_up_sec:
            self.phase = 'GIVE_UP'

    def _refine(self, buoys) -> bool:
        """고정 위치 보정. 고정한 ID 의 부표가 있으면 그것만 따라가고(색이 잠깐 바뀌어도),
        목표 아닌 빨강·초록으로 바뀌었으면 False (고정 풀기). ID 가 없으면 relock_radius 안
        같은 색 부표를 이어받는다."""
        cx, cy = self.center
        best = None
        if self.locked_id is not None:
            for b in buoys:
                if getattr(b, 'id', None) == self.locked_id:
                    if b.color in ('red', 'green') and b.color not in self.match:
                        return False
                    best = _xy(b)
                    break
        if best is None:
            best_d = self.p.relock_radius
            for b in buoys:
                if b.color not in self.match:
                    continue
                xy = _xy(b)
                d = math.hypot(xy[0] - cx, xy[1] - cy)
                if d <= best_d:
                    best, best_d = xy, d
                    self.locked_id = getattr(b, 'id', self.locked_id)
        if best is not None:
            a = self.p.refine_alpha
            self.center = ((1.0 - a) * cx + a * best[0], (1.0 - a) * cy + a * best[1])
        return True

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
