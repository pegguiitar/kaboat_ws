"""search_orbit — 탐색 미션 계산 (ROS 무의존). search_circler 노드는 입출력만 맡는다.

규정 5.2(6) (2026-11): 사전에 주어진 색의 부표를 중심으로 원을 그리며 선회한다.
  빨강·초록 → 우현선회(시계, cw), 검정(노랑에서 바뀜) → 좌현선회(반시계, ccw).
  목표 색은 대회 당일 공지 → search.target_color 로 넣는다.
  검정은 buoy_lidar_detector 가 색으로 판정하지 않는다 — 빨강·초록 아닌 부표 크기 물체는 'unknown'
  으로 나오므로(소거법, BUOY_DETECTOR.md), 검정 목표는 unknown 부표에서 고른다 (MATCH_COLORS).
  기록(3구간)은 "목표부표에서 원을 그리는 선회기동 수행이 완료되는 순간"까지.

두 기준점 (2026-11-13 분리)
  area     탐색 영역 중심 — 후보 부표는 여기서 target_search_radius(12 m) 안만, SEEK 는 여기로 간다.
           mission_manager 가 /mission/search_center 로 준다(rec search_center, 없으면
           search_entry 선수 방향 7 m 앞). 안 주면 goal 과 같다(예전 동작).
  goal     search_end — 완료 뒤 빠져나갈 출구만 (mission_manager 전환 = 완료 + 2 m 안).
  2026-11-13 bag: search_end 를 탐색 부표에서 22 m 떨어진 곳(다음 미션 쪽)에 기록해, 끝점 10 m
  안 후보가 끝까지 0개였다. 출구와 탐색 영역을 한 점으로 겸하지 않는다.

단계
  LOOK     활성화 직후 look_sec(15 s) 동안 그 자리를 지키며(노드가 위치·선수각 유지) 둘러본다.
           부표 트래커는 search 가 활성이 돼야 켜진다 — 같은 bag 을 search 시작부터 트래커를
           켜고 재생하면 시작 자세에서 검정 +0.8 s, 초록 +6.2 s, 빨강 +14.1 s 에 들어왔다.
           빨강·초록 목표는 확인되는 즉시 고정한다. 검정 목표(unknown)는 LOOK 이 끝날 때까지
           기다렸다 고른다 — 빨강·초록도 색이 붙기 전 잠깐 unknown 을 거치므로(같은 bag 빨강 1.3 s).
  SEEK     LOOK 이 끝나도 못 찾음 — 탐색 영역 중심(area)으로 가고, 도착하면 제자리 선회로 둘러본다.
  ORBIT    목표 부표 고정 — 부표 중심 반경 orbit_radius 원을 LOS 로 추종한다. 원 밖에서는 규정
           방향 접선으로 들어가는 진입 경로(tangent_entry)를 먼저 따라간다. 원 띠
           (|거리 − orbit_radius| ≤ orbit_band) 안에 있을 때만 부표 둘레를 규정 방향으로
           돈 각도를 쌓고, laps 바퀴가 되면 DONE.
  DONE     완료 — 노드가 완료 신호 1회. 배→출구 직선이 부표에서 멀어지는 쪽(exit_clear, 출구 원뿔
           exit_cone_deg)에 올 때까지 원을 이어 돈 뒤 출구(goal)로 — 반대편에서 바로 나가면 직선이
           방금 돈 부표를 스쳤다(2026-11-13 시뮬 0.9 m). mission_manager 전환 = 완료 + search_end 2 m 안.
           search_end 가 목표 부표 위(done_clearance 안)에 찍혀 있으면 부표에서 **출구 쪽**으로
           done_clearance 떨어진 점으로 간다 (exit_point — search_end 2 m 안).
  GIVE_UP  SEEK 에서 영역 중심에 도착하고 give_up_sec 동안 못 찾음 — 완료 신호를 내고 다음
           미션으로 넘긴다 (탐색 점수는 없지만 15분 제한 안에 회피를 하러 간다).

목표 부표 고정: 부표 검출 노드(buoy_lidar_detector)는 배에서 멀어진 부표를 지우고 색이 잠깐씩
  바뀔 수도 있다. 선회 도중 중심이 사라지거나 바뀌면 안 되므로 고른 부표 위치는 여기서 기억한다.
  고정할 때 검출 노드의 부표 ID 를 기억해 그 ID 로만 조금씩(refine_alpha) 보정하고, ID 가
  사라지면 relock_radius(0.6 m) 안 같은 색 부표를 이어받는다 — 1.2 m 붙은 초록·검정 쌍에서
  옆 부표로 끌려가지 않게. 고정한 부표가 목표 아닌 빨강·초록으로 분명히 바뀌면 고정을 풀고
  다시 찾는다. 목표 색이 lock_confirm_sec(1 s) 이어진 부표만 고른다.
모양 검사(measure_shapes): 배 3–6 m 안 부표의 점유지도 덩어리(긴 축·칸·1–2 m 고리)를 지도마다 재서
  다수결. 고리·덩어리에서 다른 검출 부표 자리는 뺀다(붙은 쌍). 선회 각을 세기 시작하면 모양으로는
  고정을 풀지 않는다.
후보: 목표 색 부표 중 탐색 영역 중심에서 target_search_radius 안, 중심에 가장 가까운 것.
  각도를 쌓기 시작하기 전에는 중심에 switch_margin 이상 더 가까운 후보가 나오면 바꾼다
  — 경기장 주황 부표는 부표 검출에서 빨강으로 나오므로(BUOY_DETECTOR.md), 멀리서
  주황 부표를 먼저 고른 뒤 진짜 목표가 보이는 경우를 위한 것이다.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

TARGET_COLORS = ('red', 'green', 'black')
# 규정 5.2(6): 빨강·초록 우현(시계), 검정(구 노랑) 좌현(반시계)
TURN_DIRECTION = {'red': 'cw', 'green': 'cw', 'black': 'ccw'}
# 목표 색 → 부표맵에서 받아들일 색. 검정 = 빨강·초록 아닌 물체(unknown).
MATCH_COLORS = {'red': ('red',), 'green': ('green',), 'black': ('unknown', 'black')}

XY = Tuple[float, float]


@dataclass(frozen=True)
class SearchParams:
    target_color: str = ''            # 대회 당일 공지 — red / green / black. '' 이면 미설정
    orbit_radius: float = 2.0         # 부표 중심 → 선회 원 반경 [m] (선체 1.1 m + 부표 반경 여유)
    orbit_band: float = 1.0           # 원에서 이만큼 안쪽·바깥에 있을 때만 선회 각도를 센다 [m]
    laps: float = 1.1                 # 완료 바퀴 수 — 한 바퀴 + 36° 여유 (원이 확실히 닫히게)
    target_search_radius: float = 12.0  # 탐색 영역 중심에서 이 안의 부표만 후보 [m]
                                      # (10 → 12, 2026-11-13: 영역 중심 기준. 같은 bag 에서 탐색 부표는
                                      #  중심 3.6–4.7 m, 다른 물체는 16.8 m 이상)
    relock_radius: float = 0.6        # 고정한 부표 ID 가 사라지면 이 안의 같은 색 부표를 이어받음 [m]
                                      # (1.2 m 붙은 초록·검정 쌍보다 작게 — 예전 1.5 는 옆 부표로 끌려갔다)
    lock_confirm_sec: float = 1.0     # 목표 색이 이만큼 이어진 부표만 고른다 — 검출 노드는 카메라
                                      # 근거가 적을 때 일단 unknown 으로 내서, 검정 목표가 색이 덜
                                      # 정해진 초록·빨강 부표를 잡지 않게
    refine_alpha: float = 0.2         # 다시 보일 때 고정 위치 보정 비율 (tick 당)
    recenter_distance: float = 0.3    # 고정 위치가 이만큼 바뀌면 원 경로를 다시 만든다 [m]
    switch_margin: float = 2.0        # 영역 중심에 이만큼 더 가까운 후보가 나오면 바꾼다 [m]
    arrive_radius: float = 1.5        # 부표 못 찾았을 때 영역 중심 도착 반경 → 제자리 둘러보기 [m]
    scan_turn: float = 0.15           # 제자리 둘러보기 회전 명령 (최대 추력 비율)
    give_up_sec: float = 90.0         # 영역 중심 도착 후 이만큼 못 찾으면 포기 [s]
    orbit_thrust: float = 0.25        # 선회·접선 진입 LOS 전진 상한 (공용 LOS 0.25) — 실물 0.75 m/s
    orbit_turn: float = 0.31          # 선회 LOS 회전 상한 (공용 LOS 회전 0.31)
    # LOS 속도 → 명령을 실물 곡선으로 (2026-11-14, los_controller.MEASURED_SPEED_*). 공용 변환은 상한 0.25 에서
    # 1.48 m/s(Gazebo 맞춤)를 가정해 감속 구간 명령이 실물보다 3~4 배 작았다 — 선수각 오차 60° 넘으면 0.032
    # (실물 0.16 m/s), 굽이 하한 0.018(0.10 m/s)로 조류 0.15 m/s 에 원으로 못 돌아오고 부표에 닿았다.
    # 폐루프 시뮬(scripts/e2e_sim, 탐색만, 조류 없음·남남서·동북동 0.15 m/s): 공용 변환 1 판씩 탐색 226 s·406 s·
    # 출구 못 감, 선회 중 부표 접촉 2 판 → 실물 곡선 2 판씩 141–178·158–233·132–282 s, 명령 0.03 미만 시간
    # 27~39 % → 1~5 %, 선회 중 접촉 0. 출구(선회 뒤 search_end 까지)는 여전히 71–170 s·접촉 3 번 (SEARCH_MISSION.md
    # §7.8). false = 예전 공용 변환.
    measured_speed: bool = True
    ignore_radius: float = 1.0        # 선회 중 회피 반발에서 뺄 목표 부표 둘레 반경 [m]
    repulse_occ_min: int = 75         # 탐색의 반발 조향은 점유 확률 이 이상 칸만 (공용 기본 50). 지도는 반사 없는
                                      # 빔으로 빈 공간을 안 지워, 배 근처 물보라 히트 한두 번(60~70)이 배 지나간 자리에
                                      # 남는다 — 2026-11-14 실주행 지도: 부표 아닌 점유 칸 38/38 이 ≤ 70, 부표 몸통은
                                      # 77 % 가 100. 같은 궤도를 다시 돌 때 이 칸에 밀려 원을 5–8 m 벗어났다 (시뮬 선회
                                      # tick 의 87–92 % 에 반발, 그 칸의 84–96 % 가 물체 없는 자리). 50 = 예전 동작
    done_clearance: float = 1.5       # 완료 뒤 목표 부표에 이보다 가까이 안 감 [m]
                                      # (mission_manager transition_radius 2.0 보다 작게)
    look_sec: float = 15.0            # 활성화 직후 제자리에서 둘러보는 시간 [s] (0 = 안 함)
    black_waits_look: bool = True     # 검정(unknown) 목표는 LOOK 이 끝난 뒤에 고른다
    hold_linear: float = 0.25         # LOOK 제자리 유지 전진 상한 — 바람 0.25 m/s 를 이기게
                                      # (behavior 기본 0.101 ≈ 0.35 m/s 로는 겨우 버팀)
    require_grid: bool = True         # /occupancy_grid 없거나 끊기면 정지 (gate 와 같은 규칙)
    grid_timeout_sec: float = 0.7     # 지도가 이보다 오래 안 오면 끊긴 것으로 [s] (gate 0.7 과 같음)
    # 모양 검사 (점유지도 덩어리) — 2026-11-13 bag 실측: 탐색·gate 부표 17개(누운 부표 포함)
    # 800 프레임 긴 축 ≤ 1.28 m·칸 ≤ 19·주변 ≤ 6, 도크 벽·줄부표는 2.6–9.5 m·칸 20–155.
    # 아래 값(여유 있게)이면 부표 100 % 통과, 긴 구조물은 칸 수로 걸러진다. 다만 부표만 한 도크
    # 조각(1.35 m·14 칸)은 모양으로 못 거른다 — 검정은 black_needs_field 로 무리 안인지 본다.
    shape_check: bool = True          # 검정 후보는 부표 모양 확인된 것만, 빨강·초록은 구조물이면 뺀다
    shape_max_extent: float = 2.0     # 덩어리 긴 축 [m] (누운 부표 옆면·이동 번짐·이웃 부표 여유)
    shape_max_cells: int = 25         # 덩어리 칸 수 (0.2 m 칸)
    shape_max_ring: int = 12          # 중심 1–2 m 고리 안 다른 점유 칸 수
    shape_max_range: float = 6.0      # 배에서 이 안의 부표만 모양을 잰다 [m] (먼 건 점이 성김)
    shape_min_range: float = 3.0      # 배에서 이보다 가까운 부표는 안 잰다 [m] — 2026-11-12 bag: 배가 2 m 옆을
                                      # 지나는 동안 초록 덩어리가 12 → 21 칸·긴 축 1.2 → 1.9 m 로 번졌다
    shape_exclude_radius: float = 0.7  # 고리·덩어리를 셀 때 다른 검출 부표 중심 이 안 칸은 뺀다 [m] — 1.46 m
                                      # 붙은 초록·검정 쌍이 서로의 1–2 m 고리에 들어가 둘 다 구조물로 판정됐다
    shape_min_evals: int = 5          # 판정에 필요한 최소 평가 수 (지도 메시지 단위)
    shape_pass_ratio: float = 0.6     # 최근 평가 중 이 비율 이상 통과하면 부표 모양
    field_gap: float = 10.0           # 검정 후보는 확인된 빨강·초록 중 가장 가까운 것까지 이 안 [m]
                                      # (같은 bag 탐색 부표 사이 6.2–8.1 m)
    black_needs_field: bool = True    # 검정은 영역 안 빨강·초록을 하나 이상 확인해야 고른다 — 부표만 한
                                      # 구조물(도크 조각·기둥)은 "탐색 부표 무리 안"으로만 거를 수 있어서.
                                      # 못 보면 계속 찾다가 give_up (도는 것보다 안전한 실패)
    tangent_entry: bool = True        # 선회 원에 규정 방향 접선으로 진입
    entry_lead: float = 0.8           # 접점에서 원을 따라 이만큼 더 간 점까지 진입 경로 [m]
    entry_clearance: float = 1.5      # 진입 직선이 다른 부표와 이보다 가까우면 비켜 가는 경유점 [m]
    entry_skip_band: float = 0.7      # 원에서 이만큼 안이면 진입 경로 없이 원을 바로 추종 [m]
    entry_pivot_deg: float = 60.0     # 진입 시작 때 경로 방향과 선수각 차이가 이보다 크면 제자리에서 먼저 돈다 [deg]
                                      # (180 = 끔). 2026-11-14 시뮬: approach 0.4 로 빨리 도착해 선수 0° 로 고정 →
                                      # 진입 120° 쪽으로 크게 돌다 반발 조향에 밀려 검정 반대편으로 돌아 중심 0.61 m
    exit_cone_deg: float = 75.0       # 완료 뒤, 부표→배 방향과 배→출구 방향 사이가 이 안일 때 원을 벗어난다 [deg]
                                      # (그 전에는 원을 이어 돈다 — 출구 직선이 방금 돈 부표를 스치지 않게)
    exit_thrust: float = 0.25         # 완료·포기 뒤 출구 직선 LOS 직진 상한 (선회와 같음, 실물 0.75 m/s). 예전 공용 waypoint()
                                      # 0.101 은 조류 0.25 m/s 에서 22 m 에 125 s, 0.15 m/s 에서 옆으로 밀려 0.43 m
    exit_arrive_radius: float = 0.4   # 출구점이 search_end 에서 비킨 점일 때 도착 반경 [m]
                                      # (done_clearance + 이것 < mission_manager 전환 반경 2.0)

    def __post_init__(self):
        if self.target_color not in ('',) + TARGET_COLORS:
            raise ValueError(f"search.target_color must be one of {TARGET_COLORS} or ''")
        positive = (self.orbit_radius, self.orbit_band, self.laps, self.target_search_radius,
                    self.relock_radius, self.recenter_distance, self.arrive_radius,
                    self.give_up_sec, self.ignore_radius, self.done_clearance, self.entry_lead)
        if not all(math.isfinite(v) and v > 0.0 for v in positive):
            raise ValueError('search radii, laps, give_up_sec and entry_lead '
                             'must be finite and > 0')
        unit = (self.refine_alpha, self.scan_turn, self.orbit_thrust, self.orbit_turn, self.exit_thrust,
                self.hold_linear)
        if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in unit):
            raise ValueError('search refine_alpha / scan_turn / orbit_thrust / orbit_turn / '
                             'hold_linear must be in [0, 1]')
        non_negative = (self.switch_margin, self.look_sec, self.entry_clearance,
                        self.entry_skip_band)
        if not all(math.isfinite(v) and v >= 0.0 for v in non_negative):
            raise ValueError('search switch_margin / look_sec / entry_clearance / '
                             'entry_skip_band must be finite and >= 0')
        if not (math.isfinite(self.grid_timeout_sec) and self.grid_timeout_sec > 0.0):
            raise ValueError('search.grid_timeout_sec must be finite and > 0')
        if not all(math.isfinite(v) and v > 0.0 for v in (
                self.shape_max_extent, self.shape_max_range, self.field_gap)):
            raise ValueError('search shape_max_extent / shape_max_range / field_gap must be > 0')
        if not (math.isfinite(self.shape_min_range) and 0.0 <= self.shape_min_range < self.shape_max_range
                and math.isfinite(self.shape_exclude_radius) and self.shape_exclude_radius >= 0.0):
            raise ValueError('search shape_min_range must be in [0, shape_max_range) and '
                             'shape_exclude_radius >= 0')
        if not (math.isfinite(self.entry_pivot_deg) and 20.0 < self.entry_pivot_deg <= 180.0):
            raise ValueError('search.entry_pivot_deg must be in (20, 180]')
        if not (math.isfinite(self.exit_cone_deg) and 0.0 < self.exit_cone_deg <= 90.0
                and math.isfinite(self.exit_arrive_radius) and self.exit_arrive_radius > 0.0):
            raise ValueError('search exit_cone_deg must be in (0, 90] and exit_arrive_radius > 0')
        if (self.shape_max_cells < 1 or self.shape_max_ring < 0 or self.shape_min_evals < 1
                or not 0.0 < self.shape_pass_ratio <= 1.0):
            raise ValueError('search shape_max_cells / shape_max_ring / shape_min_evals / '
                             'shape_pass_ratio out of range')
        if self.ignore_radius >= self.orbit_radius:
            raise ValueError('search.ignore_radius must be smaller than orbit_radius')
        if not 1 <= self.repulse_occ_min <= 100:
            raise ValueError('search.repulse_occ_min must be in [1, 100]')

    @property
    def direction(self) -> Optional[str]:
        return TURN_DIRECTION.get(self.target_color)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _xy(buoy) -> XY:
    return float(buoy.position.x), float(buoy.position.y)


def _segment_closest(p: XY, q: XY, o: XY) -> Tuple[XY, float]:
    """선분 p–q 위에서 o 에 가장 가까운 점과 그 거리."""
    vx, vy = q[0] - p[0], q[1] - p[1]
    n2 = vx * vx + vy * vy
    t = 0.0 if n2 < 1e-12 else max(0.0, min(1.0, ((o[0] - p[0]) * vx + (o[1] - p[1]) * vy) / n2))
    c = (p[0] + t * vx, p[1] + t * vy)
    return c, math.hypot(c[0] - o[0], c[1] - o[1])


def tangent_entry(boat: XY, center: XY, radius: float, direction: str,
                  obstacles: Sequence[XY] = (), *, lead: float = 0.8,
                  clearance: float = 1.5, skip_band: float = 0.7) -> Optional[List[XY]]:
    """배 → 선회 원 규정 방향 접선 진입 경로의 꼭짓점 [배, (경유점), 접점, 접점+lead].

    배에서 원에 그은 두 접선 중 접점에서 원을 direction(cw/ccw)으로 이어 돌 수 있는 쪽을 고른다
    — 원으로 곧장 들어가 반대로 꺾거나 원 안을 가로지르지 않는다. 마지막 점은 접점에서 원을
    따라 lead 만큼 더 간 점이라, 진입 경로 끝의 진행 방향이 이미 원의 접선 방향이다.
    배가 이미 원 띠(radius + skip_band) 안이면 None — 원을 바로 추종한다.
    배 → 접점 직선이 obstacles(목표 아닌 부표)와 clearance 안으로 지나가면 가장 가까운 부표
    바깥쪽으로 clearance + 0.5 m 비킨 경유점을 하나 넣는다.
    """
    if direction not in ('cw', 'ccw'):
        raise ValueError("direction must be 'cw' or 'ccw'")
    px, py = float(boat[0]), float(boat[1])
    cx, cy = float(center[0]), float(center[1])
    d = math.hypot(px - cx, py - cy)
    if d <= radius + skip_band:
        return None
    phi = math.atan2(py - cy, px - cx)
    alpha = math.acos(radius / d)
    sign = 1.0 if direction == 'ccw' else -1.0
    chosen = None
    for s in (1.0, -1.0):
        th = phi + s * alpha
        tx, ty = cx + radius * math.cos(th), cy + radius * math.sin(th)
        # 원을 direction 으로 돌 때 접점에서의 진행 방향
        ux, uy = -sign * math.sin(th), sign * math.cos(th)
        if (tx - px) * ux + (ty - py) * uy > 0.0:
            chosen = (th, tx, ty)
            break
    th, tx, ty = chosen
    th2 = th + sign * lead / radius
    lead_pt = (cx + radius * math.cos(th2), cy + radius * math.sin(th2))
    pts: List[XY] = [(px, py)]
    via = detour_via((px, py), (tx, ty), obstacles, clearance, away_from=(cx, cy))
    if via is not None:
        pts.append(via)
    pts += [(tx, ty), lead_pt]
    return pts


def detour_via(start: XY, end: XY, obstacles: Sequence[XY], clearance: float,
               away_from: Optional[XY] = None) -> Optional[XY]:
    """start → end 직선이 obstacles 중 하나와 clearance 안으로 지나가면, 가장 가까운 것 바깥쪽으로
    clearance + 0.5 m 비킨 경유점. 없으면 None. 부표가 직선 바로 위면 away_from 에서 먼 쪽 수직으로
    (접선 진입은 선회 원 중심, 출구는 방금 돈 부표)."""
    px, py = float(start[0]), float(start[1])
    tx, ty = float(end[0]), float(end[1])
    worst = None
    for o in obstacles:
        c, dist = _segment_closest((px, py), (tx, ty), (float(o[0]), float(o[1])))
        if dist < clearance and (worst is None or dist < worst[2]):
            worst = ((float(o[0]), float(o[1])), c, dist)
    if worst is None:
        return None
    (ox, oy), (qx, qy), dist = worst
    if dist > 1e-6:
        nx, ny = (qx - ox) / dist, (qy - oy) / dist
    else:   # 부표가 직선 위 — away_from 에서 먼 쪽 수직 방향으로
        sx, sy = tx - px, ty - py
        n = math.hypot(sx, sy)
        nx, ny = -sy / n, sx / n
        if away_from is not None:
            cx, cy = away_from
            plus = (ox + nx - cx) ** 2 + (oy + ny - cy) ** 2
            minus = (ox - nx - cx) ** 2 + (oy - ny - cy) ** 2
            if plus < minus:
                nx, ny = -nx, -ny
    k = clearance + 0.5
    return (ox + nx * k, oy + ny * k)


@dataclass(frozen=True)
class BlobShape:
    extent: float      # 덩어리 긴 축 길이 [m] (주성분 방향 투영 폭 + 칸 크기)
    cells: int         # 덩어리 칸 수
    ring_cells: int    # 덩어리 밖, 중심에서 ring 반경 사이 점유 칸 수 (주변 구조물)


def blob_shape(grid, resolution: float, origin: XY, xy: XY, *, occ_threshold: int = 50,
               seed_radius: float = 0.6, ring: Tuple[float, float] = (1.0, 2.0),
               max_cells: int = 800, exclude: Sequence[XY] = (),
               exclude_radius: float = 0.7) -> Optional[BlobShape]:
    """점유지도에서 xy 의 덩어리 모양. 지도 밖이거나 xy 근처(seed_radius)에 점유 칸이 없으면 None.

    grid: 2-D 배열 [행 = y, 열 = x] (OccupancyGrid.data 를 height×width 로), 값 0–100 / -1.
    덩어리 = seed 칸에서 8-이웃으로 이어진 점유 칸 (max_cells 에서 끊음 — 벽은 그 전에 이미 길다).
    exclude: 다른 부표 중심들 — 그 exclude_radius 안 칸은 빈 칸으로 본다 (덩어리·고리 둘 다).
    이웃 부표는 구조물 근거가 아니다 (2026-11-12 bag: 1.46 m 붙은 초록·검정이 서로의 고리 칸이 됐다).
    """
    import numpy as np
    g = np.asarray(grid)
    h, w = g.shape
    ci = (xy[0] - origin[0]) / resolution
    cj = (xy[1] - origin[1]) / resolution
    if not (0 <= ci < w and 0 <= cj < h):
        return None
    occ = g >= occ_threshold
    r = int(math.ceil(max(seed_radius, ring[1]) / resolution)) + 1
    i0, i1 = max(0, int(ci) - r), min(w, int(ci) + r + 1)
    j0, j1 = max(0, int(cj) - r), min(h, int(cj) + r + 1)
    if exclude and exclude_radius > 0.0:
        occ = occ.copy()
        k = int(math.ceil(exclude_radius / resolution)) + 1
        for ex, ey in exclude:
            ei = (ex - origin[0]) / resolution
            ej = (ey - origin[1]) / resolution
            for j in range(max(0, int(ej) - k), min(h, int(ej) + k + 1)):
                for i in range(max(0, int(ei) - k), min(w, int(ei) + k + 1)):
                    if math.hypot(i + 0.5 - ei, j + 0.5 - ej) * resolution <= exclude_radius:
                        occ[j, i] = False
    seeds = [(j, i) for j in range(j0, j1) for i in range(i0, i1)
             if occ[j, i] and math.hypot(i + 0.5 - ci, j + 0.5 - cj) * resolution <= seed_radius]
    if not seeds:
        return None
    seen = set(seeds)
    stack = list(seeds)
    while stack and len(seen) < max_cells:
        j, i = stack.pop()
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                nj, ni = j + dj, i + di
                if (0 <= nj < h and 0 <= ni < w and (nj, ni) not in seen and occ[nj, ni]):
                    seen.add((nj, ni))
                    stack.append((nj, ni))
    pts = np.array([(i + 0.5, j + 0.5) for j, i in seen], dtype=float) * resolution
    if len(pts) >= 2:
        c = pts - pts.mean(axis=0)
        _, vecs = np.linalg.eigh(c.T @ c)
        proj = c @ vecs[:, -1]
        extent = float(proj.max() - proj.min()) + resolution
    else:
        extent = resolution
    ring_cells = 0
    for j in range(j0, j1):
        for i in range(i0, i1):
            if occ[j, i] and (j, i) not in seen:
                d = math.hypot(i + 0.5 - ci, j + 0.5 - cj) * resolution
                if ring[0] <= d <= ring[1]:
                    ring_cells += 1
    return BlobShape(extent, len(seen), ring_cells)


def measure_shapes(grid, resolution: float, origin: XY, buoys, boat_xy: XY, area_xy: XY,
                   params: 'SearchParams') -> dict:
    """지도 한 장 → {부표 ID: 부표 모양인가(bool)} — 노드가 새 지도마다 한 번 불러 다수결에 쌓는다.

    배에서 shape_min_range–shape_max_range 사이, 영역 중심 target_search_radius + 1 m 안 부표만 잰다.
    고리·덩어리에서 **이웃 부표** 자리는 뺀다 — 이웃 = 0.9–3 m 안 다른 검출 부표 중 자기 덩어리가 부표
    크기(긴 축·칸 상한 안)인 것. 벽·도크에 붙은 조각이 부표로 검출돼도 그 덩어리는 벽 전체라 안 빠지고
    구조물 근거로 남는다. 0.9 m 안 것은 같은 물체의 중복일 수 있어 빼지 않는다 (검출기가 합치는 거리).
    """
    p = params
    pts = [(getattr(b, 'id', None), (float(b.position.x), float(b.position.y))) for b in buoys]
    pts = [(bid, xy) for bid, xy in pts if all(math.isfinite(v) for v in xy)]
    small = {}

    def buoy_sized(key, xy):
        if key not in small:
            s = blob_shape(grid, resolution, origin, xy)
            small[key] = (s is not None and s.extent <= p.shape_max_extent
                          and s.cells <= p.shape_max_cells)
        return small[key]

    out = {}
    for k, (bid, xy) in enumerate(pts):
        if bid is None:
            continue
        d = math.dist(xy, boat_xy)
        if (d > p.shape_max_range or d < p.shape_min_range
                or math.dist(xy, area_xy) > p.target_search_radius + 1.0):
            continue
        others = [o for j, (oid, o) in enumerate(pts)
                  if j != k and 0.9 < math.dist(o, xy) <= 3.0 and buoy_sized(j, o)]
        s = blob_shape(grid, resolution, origin, xy, exclude=others,
                       exclude_radius=p.shape_exclude_radius)
        if s is None:
            continue
        out[bid] = (s.extent <= p.shape_max_extent and s.cells <= p.shape_max_cells
                    and s.ring_cells <= p.shape_max_ring)
    return out


def exit_clear(boat_xy: XY, center: XY, target: XY, cone_deg: float, free_radius: float) -> bool:
    """완료 뒤 원을 벗어나도 되나 — 배→출구 직선이 부표에서 멀어지는 쪽이면 True.

    부표→배 방향과 배→출구 방향 사이 각이 cone_deg 안이면 직선이 부표에서 계속 멀어진다 (90° 미만이면
    가장 가까운 점이 지금 자리). 배가 이미 free_radius 밖이거나 출구가 코앞이면 True.
    """
    rx, ry = boat_xy[0] - center[0], boat_xy[1] - center[1]
    gx, gy = target[0] - boat_xy[0], target[1] - boat_xy[1]
    nr, ng = math.hypot(rx, ry), math.hypot(gx, gy)
    if nr >= free_radius or ng < 0.5 or nr < 1e-6:
        return True
    cos_a = (rx * gx + ry * gy) / (nr * ng)
    return cos_a >= math.cos(math.radians(cone_deg))


class SearchFsm:
    """탐색 단계 판정. 매 tick update() 를 부르고 phase/center 를 읽어 주행한다.

    buoys 는 color 와 position.x/.y 를 가진 객체 (kaboat_msgs/Buoy 또는 같은 모양).
    goal_xy = 출구(search_end), area_xy = 탐색 영역 중심 (None 이면 goal 과 같음 — 예전 동작).
    """

    def __init__(self, params: SearchParams, goal_xy, area_xy=None):
        if params.direction is None:
            raise ValueError('search.target_color is not set')
        self.p = params
        self.goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.area = self.goal if area_xy is None else (float(area_xy[0]), float(area_xy[1]))
        self.phase = 'LOOK' if params.look_sec > 0.0 else 'SEEK'
        self.center: Optional[XY] = None
        self.swept = 0.0          # 규정 방향으로 돈 각도 [rad]
        self.counting = False     # 한 번이라도 원 띠 안에서 각도를 셌는가
        self._prev_theta = None
        self._scan_since = None
        self._look_since = None
        self.switches = 0
        self.locked_id = None     # 고정한 부표의 검출 노드 ID (없으면 위치로만)
        self.unlocks = 0
        self._match_since = {}    # 부표 키 → 목표 색으로 처음 보인 시각
        self._refs: List[XY] = []  # 검정 순위 기준 (확인된 빨강·초록)

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

    def look_remaining(self, now: float) -> float:
        """LOOK 남은 시간 [s] (시작 전이면 look_sec)."""
        if self._look_since is None:
            return self.p.look_sec
        return max(0.0, self.p.look_sec - (now - self._look_since))

    def exit_point(self, boat_xy) -> XY:
        """완료 뒤 갈 점 — 보통 출구(goal). 출구가 부표 done_clearance 안이면 부표에서 **출구 쪽**으로
        done_clearance 비킨 점 (출구점–goal < done_clearance 라 노드가 exit_arrive_radius 로 다가가면
        mission_manager 2 m 전환 안이다). 예전엔 배 쪽으로 비켜, 배가 반대편에서 끝내면 출구점이
        goal 에서 최대 3 m 라 전환이 안 됐다. goal 이 부표 중심과 같으면 배 쪽."""
        if self.center is None or math.dist(self.goal, self.center) >= self.p.done_clearance:
            return self.goal
        cx, cy = self.center
        vx, vy = self.goal[0] - cx, self.goal[1] - cy
        n = math.hypot(vx, vy)
        if n < 1e-6:
            vx, vy = boat_xy[0] - cx, boat_xy[1] - cy
            n = math.hypot(vx, vy)
        if n < 1e-6:
            vx, vy, n = 1.0, 0.0, 1.0
        k = self.p.done_clearance / n
        return (cx + vx * k, cy + vy * k)

    def _dist_area(self, xy) -> float:
        return math.hypot(xy[0] - self.area[0], xy[1] - self.area[1])

    def _field_refs(self, buoys) -> List[XY]:
        """검정 목표의 기준 — 영역 안에서 색이 확인된 빨강·초록 부표 위치."""
        if self.p.target_color != 'black':
            return []
        refs = []
        for b in buoys:
            if b.color in ('red', 'green'):
                xy = _xy(b)
                if (all(math.isfinite(v) for v in xy)
                        and self._dist_area(xy) <= self.p.target_search_radius):
                    refs.append(xy)
        return refs

    def _score(self, xy, refs) -> float:
        """후보 순위 — 낮을수록 먼저. 검정은 확인된 빨강·초록 중 가장 가까운 것까지 거리(탐색 부표
        무리 안에 있나), 기준이 없거나 빨강·초록 목표면 영역 중심까지 거리."""
        if refs:
            return min(math.dist(xy, r) for r in refs)
        return self._dist_area(xy)

    def candidates(self, buoys, now: Optional[float] = None, shape_ok=None):
        """목표 색(MATCH_COLORS)이고 영역 중심 target_search_radius 안 — 순위(_score) 순
        [(xy, id)]. now 를 주면 목표 색이 lock_confirm_sec 이상 이어진 부표만.

        shape_ok(buoy) → True(부표 모양) / False(구조물) / None(아직 모름). 주면
          검정(unknown) 후보는 True 인 것만 — 다리 기둥·도크·방파제도 unknown 이라서,
          빨강·초록 후보는 False 인 것만 뺀다 (색이 있어 구조물일 일이 드물고, 멀면 모양을 못 잰다).
        검정은 확인된 빨강·초록이 있으면 그중 가장 가까운 것까지 field_gap 안이어야 하고,
        black_needs_field 면 하나도 없을 때 고르지 않는다.
        """
        refs = self._field_refs(buoys)
        out, seen = [], set()
        for b in buoys:
            if b.color not in self.match:
                continue
            xy = _xy(b)
            if not all(math.isfinite(v) for v in xy):
                continue
            if self._dist_area(xy) > self.p.target_search_radius:
                continue
            if shape_ok is not None:
                ok = shape_ok(b)
                if ok is False or (ok is None and b.color not in ('red', 'green')):
                    continue
            score = self._score(xy, refs)
            if refs and score > self.p.field_gap:
                continue
            if self.p.target_color == 'black' and self.p.black_needs_field and not refs:
                continue
            bid = getattr(b, 'id', None)
            key = bid if bid is not None else (round(xy[0] * 2), round(xy[1] * 2))
            seen.add(key)
            if now is not None:
                first = self._match_since.setdefault(key, now)
                if now - first < self.p.lock_confirm_sec:
                    continue
            out.append((score, xy, bid))
        if now is not None:
            self._match_since = {k: v for k, v in self._match_since.items() if k in seen}
        out.sort(key=lambda t: t[0])
        self._refs = refs
        return [(xy, bid) for _, xy, bid in out]

    def _looking(self, now: float) -> bool:
        if self.p.look_sec <= 0.0:
            return False
        if self._look_since is None:
            self._look_since = now
        return now - self._look_since < self.p.look_sec

    def update(self, buoys, boat_xy, now: float, shape_ok=None) -> str:
        """shape_ok: candidates() 참고. 고정한 부표가 구조물(False)로 판정되면 고정을 푼다 — 단 선회 각을
        세기 시작한 뒤(counting)에는 모양으로 풀지 않는다(색이 바뀔 때만). 선회 중 배가 부표 옆을 돌면
        지도 덩어리가 번져 진짜 목표가 구조물로 판정됐다(2026-11-12 bag → 포기)."""
        if self.done:
            return self.phase
        cands = self.candidates(buoys, now, shape_ok)
        if self.center is not None and not self._refine(buoys):
            self._unlock()                 # 고정한 부표가 목표 아닌 색으로 바뀜 — 다시 찾기
        elif (self.center is not None and shape_ok is not None and self.locked_id is not None
              and not self.counting):
            locked = next((b for b in buoys if getattr(b, 'id', None) == self.locked_id), None)
            if locked is not None and shape_ok(locked) is False:
                self._unlock()             # 가까이서 보니 구조물(도크·기둥 등) — 다시 찾기
                cands = [c for c in cands if c[1] != locked.id]
        if self.center is None:
            looking = self._looking(now)
            wait = looking and self.p.black_waits_look and self.p.target_color == 'black'
            if cands and not wait:
                self._lock(*cands[0])
            else:
                if looking:
                    self.phase = 'LOOK'
                    return self.phase
                self.phase = 'SEEK'
                self._seek(boat_xy, now)
                return self.phase
        elif not self.counting and cands:
            best, bid = cands[0]
            far_from_lock = math.dist(best, self.center) > self.p.relock_radius
            closer = (self._score(best, self._refs) + self.p.switch_margin
                      < self._score(self.center, self._refs))
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
        if self._dist_area(boat_xy) > self.p.arrive_radius:
            return   # 영역 중심 접근 중 — 포기 시계는 도착해야 돈다
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
