"""station_keeping — 위치유지(station) 미션 전용 계산 (ROS 무의존).

경기: 부표 4개가 꼭짓점인 사각형 안에 들어가 그 자리를 유지한다. 2026-11-13 dock2 bag 연습장:
파란 큐브 4개, 변 3.9~6.3 m·대각선 5.7/8.4 m·내각 60~152° 로 정사각형이 아니고, 조류·바람에
±0.3 m 흔들리며 배에 밀리면 1 m 넘게 움직인다. 사각형이 통째로 밀려 기록 지점(station_center)이
사각형 밖이 되기도 한다 (그날 1.1 m 밖) — 그래서 크기·모양을 정해 두지 않고 보이는 부표로 잡는다.
가정: 사각형 주변 6 m 안에는 다른 부표 크기 물체가 없다 (있으면 꼭짓점과 구별이 안 될 수 있다 —
멀리서 네 개를 다 보는 부표 검출 노드를 함께 쓰면 거의 안 생긴다).

1) StationMap — station 전용 log-odds 격자
   공용 /occupancy_grid 는 빔이 지나간 칸을 스캔마다 L_FREE(-0.2) 깎는다. 라이다가 앞만
   보는데 배가 사각형 안에서 흔들리면 빔이 옆·뒤 부표 위로 지나가며 부표를 지운다. 그래서
   station 은 /scan/level 로 자기 지도를 따로 쌓고, **큰 덩어리**(점유 칸이 big_min_cells
   = 4칸 이상 이어진 것 = 부표)만 감쇠에 free_ratio(0.1)를 곱해 오래 기억한다. 작은 덩어리
   (해수면 반사 등 노이즈, 보통 1~2칸)는 공용 지도와 똑같이 깎여 금방 사라진다.
   공용 지도와 다른 behavior 는 그대로다.
   격자는 미션 목표(station_end) 중심에 고정된 정사각형이다(구역이 고정이라 rolling 불필요).
2) 부표 후보·꼭짓점 (2026-11-13 dock2 bag 로 다시 짬)
   candidate_buoys — 전용 지도 덩어리 OR 부표 검출 노드(/detections/buoys, 15 m 까지 추적)를
   합친 후보. 기록 지점 buoy_search_radius(15 m) 안, 또는 찾은 사각형 중심 square_search_radius 안.
   (예전 4 m 반경은 북·동 큐브(6.0·6.5 m)를 버려 서·남 두 개로 사각형을 지어냈다.)
   select_corners — 후보 중 "스테이션처럼 생긴" 묶음 최대 4개. 모양: 서로 min_side 이상, 변
   max_side·대각선 max_diagonal 이하, 넓이 min_area 이상, 오목도 허용 (부표가 제각각 2 m 흔들리면
   다섯에 하나는 오목 — 무작위 배치 시뮬). 그중 group_score 최고: 꼭짓점 수가 먼저, 그다음
   무게중심이 기록 지점에 가깝고(far_free 넘으면 제곱 감점), 둘레가 짧고, 묶음 가까이 남는 물체가
   적을수록. 방위 하나가 아니라 묶음으로 고르므로 근처 도크 구조물(남쪽 큐브에서 7.8 m)은 빠진다.
   StationFsm 은 잡은 꼭짓점을 기억하고 track_radius 안의 후보로 따라가며 (라이다는 앞만 봐서 뒤로
   간 꼭짓점은 안 보인다), 점수가 switch_margin 넘게 나은 묶음이 나올 때만 바꾼다. 사각형이 지도
   중심에서 멀어지면 누적을 유지한 채 지도를 옮긴다.
3) estimate_center — 꼭짓점 수별 목표 중심 (자세한 건 함수 설명).
     4개: 꼭짓점 다각형(오목 포함)의 가장 안쪽 점 — 비정형 사각형에서 평균보다 여유가 크다
          (dock2: 평균 1.81 m, 가장 안쪽 2.20 m)
     3개: 가장 긴 변이 대각선이면 그 건너편 쪽, 아니면 삼각형의 가장 안쪽 점
     2개: 쌍 중점에서 안쪽으로 min(간격/2, pair_inset) — 변인지 대각선인지 거리로 못 가르므로
          (북–남 대각선 5.7 m < 서–북 변 6.3 m) 임시 목표만
   목표 중심이 아는 부표에서 target_clearance 안이면 그만큼 밖으로 옮긴다 (safe_point).
   square_corners — 3개 이상이면 그 다각형 그대로 (3개면 삼각형), 2개 이하면 None.
4) 입구 — 배 쪽 변(다각형 변 중 배가 바깥에 있고 폭 gate_min_width 이상인 가장 가까운 변, 2개면
   그 쌍)을 입구로 잡고
   입구 앞 정렬점을 거쳐 중심으로 들어간다. 지나기 전까지는 매 틱 다시 고른다. 부표 사이를
   지나므로 공용 waypoint()(전진 12 N)가 아니라 가상 앵커(전진 상한 max_forward)로 간다.
   detour_target — 가는 길이 아는 부표에서 avoid_clearance 안을 지나면 그 부표 옆으로 돌아가는
   경유점, 목표가 부표에 붙어 있으면 그 앞 정지점 (dock2 1차: 기록 지점으로 직진하다 서쪽 큐브를
   밀고 감, 2차: 출구로 가다 남쪽 큐브 0.42 m).
5) anchor_command — 가상 앵커. 선수각 PD + 불감대는 수조 station_keeping_test 그대로.
   전진은 속도 제어 (2026-11-13): 목표까지 거리에 비례한 목표 속도(min(max_speed,
   k_approach·거리))를 /odom 전진 속도로 따라가고, 너무 빠르면 후진 추력으로 제동한다.
   목표를 지나쳐 뒤에 두고 제자리 회전할 때·불감대 안에서도 남은 속도를 제동한다.
   예전엔 거리 1 m 안까지 전진 0.25 를 내고 제동이 없어 관성(τ≈2.8 s)으로 사각형 변까지
   미끄러졌다(시뮬: 변까지 0.1 m, 4회 중 2회 부표 접촉).
6) StationFsm — SEARCH → ENTER → HOLD. 배 중심이 부표 사각형 안(각 변에서 inside_margin
   안쪽)에 hold_time 연속 있으면 완료 1회 보고(다음 미션 전환은 mission_manager 몫). 완료
   뒤에도 전환 전까지 계속 유지. (2026-11-13 경기 조건 변경: 예전엔 추정 중심 hold_radius
   반경 안이었다.) HOLD 진입은 꼭짓점 3개 이상, 완료 판정은 4개일 때 — 3개뿐이면 HOLD 에서
   제자리 회전하며 네 번째를 찾고 three_corner_wait 지나서야 삼각형으로 판정. 2개로 지은 사각형은
   2.4 m 틀려 거짓 '위치유지 달성'이 났다 (dock2 2차). 2개면 임시 목표에서 제자리 회전(LOOK).
   꼭짓점이 2개 미만이면 기록 지점으로, 거기 와서도 없으면 보이는 후보 방위로 standoff 앞까지
   서행. search_timeout 동안 끝내 못 찾을 때만 station_end 를 중심으로 반경 hold_radius 판정 (예비).
"""

from dataclasses import dataclass, field
import itertools
import math

import numpy as np
from scipy import ndimage


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass(frozen=True)
class StationParams:
    # ── station 전용 지도 ──
    resolution: float = 0.1         # [m] 셀
    # [m] 지도 중심 ±9 m — dock2 사각형은 무게중심에서 꼭짓점 최대 4.4 m, 기록 지점에서는 6.5 m.
    # 사각형이 recenter_distance 넘게 멀어지면 지도를 옮긴다(누적 유지).
    map_half_size: float = 9.0
    recenter_distance: float = 2.5
    max_range: float = 6.0          # [m] 라이다 신뢰 거리. 이보다 먼 히트는 안 씀
    l_occ: float = 0.8              # 히트 log-odds (공용 occupancy_grid L_OCC 와 같음)
    l_free: float = -0.2            # free 감쇠 (공용 L_FREE 와 같음) — 작은 덩어리(노이즈)
    free_ratio: float = 0.1         # 큰 덩어리(부표)만 감쇠 × 이 비율 (−0.2 × 0.1 = −0.02/스캔)
    big_min_cells: int = 4          # 점유(≥ occ_logodds) 칸이 이만큼 이어지면 큰 덩어리.
    # 지름 0.4 m 부표는 0.1 m 격자에서 4~9칸(거리 1.5~5.5 m, 흔들림 포함 시뮬 실측),
    # 해수면 반사는 보통 1~2칸.
    l_min: float = -2.0             # 공용(-4.5)보다 높게: 흔들려 옆 칸으로 간 부표를 빨리 다시 잡음
    l_max: float = 8.0              # 공용 L_MAX 와 같음
    # ── 부표 ──
    occ_logodds: float = 2.0        # 이 이상인 칸만 부표 후보 (히트 3스캔)
    min_cluster_cells: int = 2
    max_cluster_extent: float = 1.2  # [m] 흔들림으로 번진 부표까지
    # [m] 기록 지점(station_center, 없으면 station_end) 기준 후보 반경 — 사각형이 통째로 밀려도
    # 잡게 검출 노드 추적 거리(15 m)만큼. 엉뚱한 물체는 select_corners 의 모양 조건으로 거른다.
    # (예전 4 m: dock2 에서 북·동 큐브(6.5·6.0 m)를 버렸다)
    buoy_search_radius: float = 15.0
    # [m] 사각형을 찾은 뒤 그 중심 기준 후보 반경 (dock2 중심 → 꼭짓점 4.4 m + 제각각 흔들림 2 m 등).
    # 위 반경과 합집합 — 부표가 밀려도 중심이 따라가며 네 개를 계속 잡는다.
    square_search_radius: float = 7.5
    merge_distance: float = 0.8     # [m] 이보다 가까운 덩어리는 같은 부표(약한 쪽 버림)
    # 부표 검출 노드(buoy_lidar_detector, /detections/buoys) 결과도 후보로 쓴다 (2026-11-13).
    # 전용 지도는 라이다 max_range(6 m) 안만 봐서, 조류에 밀린 사각형은 가까이 가야 잡혔다.
    # 검출 노드는 15 m 까지 부표를 만들고 추적한다 (dock2: 9~15 m 검출률 86~100 %). 지도 덩어리 우선.
    use_detected_buoys: bool = True
    max_candidates: int = 10        # 꼭짓점 묶음을 고를 후보 수 상한 (조합 계산량)
    # 묶음 점수 = 100·꼭짓점 수 − (무게중심 ~ 기록 지점) − w_far·(그 거리 − far_free)²
    #             − w_perimeter·둘레 − w_unexplained·(남는 물체 수) + w_keep·(잡아 둔 꼭짓점 수).
    # 둘레 감점: 근처 다른 물체(도크 구조물)를 넣은 묶음은 둘레가 길다. 잡아 둔 꼭짓점 가산은 작게만 —
    # 예전엔 최우선이라 첫 틱에 (검출이 한 번 빠져) 잘못 잡은 묶음을 끝까지 놓지 않았다 (무작위 배치 시뮬).
    # 지금 묶음보다 switch_margin 넘게 나은 묶음이 나오면 바꾼다 (흔들림 방지).
    w_perimeter: float = 1.0
    # 무게중심이 기록 지점에서 far_free 넘게 떨어진 묶음은 w_far·(초과 거리)² 감점 — 기록 지점은 사각형
    # 밖으로 1~3 m 치우칠 수 있지만(dock2 1.1 m) 8 m 떨어진 도크 물체 묶음은 꼭짓점 수로도 못 이기게.
    far_free: float = 5.0
    w_far: float = 10.0
    # 묶음 다각형 안이나 그 경계에서 unexplained_margin 안에 있는데 어느 꼭짓점과도 track_radius 안에
    # 겹치지 않는 후보(남는 물체)마다 w_unexplained 감점 — 밀린 큐브의 옛·새 자리(1.2 m)를 두
    # 꼭짓점으로 쓰고 진짜 큐브 하나(3.9 m 옆)를 빼먹은 묶음이 둘레가 짧아 이기던 것을 막는다.
    # 짧은 변 자체는 감점하지 않는다 — 진짜 두 부표가 1.4 m 로 붙은 배치에서 도크 물체 묶음이 이겨
    # 사각형 밖에서 완료했다 (무작위 배치 시뮬).
    w_unexplained: float = 30.0
    unexplained_margin: float = 5.5
    w_keep: float = 1.0
    switch_margin: float = 0.5
    # ── 사각형 모양 (2026-11-13 dock2 bag 연습장: 변 3.9~6.3 m, 대각선 5.7·8.4 m) ──
    # 각 부표가 지금 자리에서 반경 2 m 안에서 제각각 흔들려도 (무작위 배치 시뮬: 변 0.5~9.7 m,
    # 대각선 2.6~12.1 m, 다섯에 하나는 오목) 네 개를 한 사각형으로 잡도록 넓게.
    side_length: float = 5.0        # [m] 공칭 한 변 — 참고값
    # [m] 꼭짓점끼리 최소 간격 — 두 부표가 1.2~1.5 m 로 붙어도 네 꼭짓점으로 (1.6 이면 진짜 넷을 못 묶고
    # 근처 도크 물체를 넣은 넷이 이겨 사각형 밖에서 완료했다, 무작위 배치 시뮬). 같은 부표 중복은 merge_distance
    min_side: float = 1.0
    max_side: float = 10.0          # [m] 한 변 최대
    max_diagonal: float = 13.0      # [m] 대각선 최대 (2개만 보일 때 쌍 간격 상한)
    min_area: float = 4.0           # [m²] 네 꼭짓점 넓이 하한 (거의 일직선이면 사각형 아님, 3개는 절반)
    pair_inset: float = 1.5         # [m] 2개만 보일 때 임시 목표 = 쌍 중점에서 안쪽 이만큼 (간격/2 이하)
    # ── 입구 ──
    pre_gate_distance: float = 2.0  # [m] 입구 중점 앞 정렬점 (변이 길어 1.0 → 2.0, 제동 여유)
    # [m] 입구로 쓸 변의 최소 폭 — 선체 폭 0.9 + 양쪽 (큐브 0.3 + 여유 0.5). 더 좁은 변은 안 지나간다
    # (1.6 m 변을 입구로 잡아 두 부표를 번갈아 피하다 0.6 m 까지 붙음, 무작위 배치 시뮬)
    gate_min_width: float = 2.5
    gate_lateral_tol: float = 0.4   # [m] 입구 수직이등분선에서 이만큼 안이면 바로 중심으로
    track_radius: float = 1.5       # [m] 잡은 꼭짓점·입구 부표 재연결 반경 (dock2: 밀린 큐브 1.2 m)
    # ── 길 위 부표 피하기 ──
    avoid_clearance: float = 1.5    # [m] 가는 길이 아는 부표에서 이보다 가까우면 옆으로 돌아감
    # [m] 목표점(중심 등)이 부표에서 이보다 가까우면 그만큼 밖으로 옮긴다 — 선체 반폭 0.45 + 큐브 0.3 + 여유.
    # 오목한 사각형은 가장 안쪽 점도 들어온 꼭짓점에서 1.4 m 일 수 있어 avoid_clearance 와 따로 둔다.
    target_clearance: float = 1.2
    standoff: float = 3.0           # [m] 꼭짓점이 1개뿐일 때 그 부표 이만큼 앞까지만 (방위로 서행)
    # ── SEARCH 접근 (base waypoint(): 전진 상한 max_linear 12 N × search_ratio) ──
    search_ratio: float = 1.0
    search_slow_radius: float = 2.0
    # [s] 이 시간 동안 꼭짓점 2개를 못 모으면 station_end 를 중심으로 유지(예비, fallback_to_goal)
    search_timeout: float = 60.0
    # ── 유지·완료 ──
    # 완료 조건 (2026-11-13): 배 중심이 부표 사각형 각 변에서 inside_margin 이상 안쪽.
    # 0.6 m ≈ 선체 반길이 0.55 m — 선체가 어느 방향이어도 거의 다 사각형 안. 배 중심만
    # 안이면 되는 규정이면 0.
    inside_margin: float = 0.6
    hold_radius: float = 1.0        # [m] ENTER → HOLD 전환 반경, 부표를 못 찾았을 때 유지 판정
    hold_time: float = 5.0          # [s] 연속 유지 시간 → 완료 보고
    # [s] 완료 판정은 꼭짓점 4개일 때만 — HOLD 에서 이만큼 기다려도 3개뿐이면 그때 삼각형으로 판정
    # (안쪽으로 들어온 네 번째 꼭짓점을 모르면 삼각형이 실제 사각형보다 클 수 있다)
    three_corner_wait: float = 45.0
    fallback_to_goal: bool = True   # 부표를 못 찾으면 station_end 를 중심으로 유지
    # ── 출구 (2026-11-12): 중심(station_center)과 station_end 가 다르면 유지 후 출구로 나간다 ──
    exit_outside: float = 1.5       # [m] 출구 쪽 변 중점에서 사각형 밖으로 이만큼 나간 경유점
    exit_reach: float = 0.8         # [m] 경유점·출구 도착 판정
    # ── 가상 앵커 (선수각·불감대는 수조 station_keeping_test 값, 상한은 LOS 직진·회전 상한) ──
    pos_deadband: float = 0.5
    yaw_deadband_deg: float = 8.0
    max_forward: float = 0.25
    max_reverse: float = 0.08       # 뒤쪽 가까운 목표로 천천히 후진 (BACK)
    max_turn: float = 0.31
    min_forward: float = 0.04
    kp_pos: float = 0.25            # BACK 후진 = kp_pos × 거리 (max_reverse 까지)
    kp_yaw: float = 1.5
    kd_yaw: float = 0.15
    reverse_band: float = 0.3       # [m] 불감대 + 이 거리 안의 뒤쪽 목표는 후진으로
    # ── 속도 제어·제동 (2026-11-13) ──
    max_speed: float = 0.5          # [m/s] 목표 속도 상한 (예전 전진 0.25 ≈ 0.7~0.85 m/s)
    k_approach: float = 0.4         # [1/s] 목표 속도 = k_approach × 거리 → 1 m 에서 0.4 m/s
    speed_ff: float = 0.25          # 목표 속도 1 m/s 당 전진 명령 (실측 0.4 ≈ 1.0 m/s, 저속은 덜 듦)
    k_speed: float = 0.4            # 속도 오차 1 m/s 당 명령 — 빠르면 음수 = 후진 제동
    max_brake: float = 0.15         # 제동 후진 명령 상한 (약 1419 µs)
    stop_speed: float = 0.08        # [m/s] 이보다 느리면 멈춘 것으로 보고 제동 안 함

    def __post_init__(self):
        floats = [v for v in vars(self).values() if not isinstance(v, bool)]
        if not all(math.isfinite(float(v)) for v in floats):
            raise ValueError('station parameters must be finite')
        positive = (self.resolution, self.map_half_size, self.max_range, self.l_occ,
                    self.occ_logodds, self.max_cluster_extent, self.buoy_search_radius,
                    self.side_length, self.min_side, self.pre_gate_distance,
                    self.gate_lateral_tol, self.track_radius, self.search_slow_radius,
                    self.hold_radius, self.pos_deadband, self.yaw_deadband_deg,
                    self.square_search_radius, self.max_side, self.max_diagonal,
                    self.pair_inset, self.avoid_clearance, self.standoff, self.target_clearance,
                    self.gate_min_width,
                    self.recenter_distance,
                    self.max_forward, self.max_turn, self.kp_pos, self.kp_yaw,
                    self.exit_outside, self.exit_reach, self.max_speed, self.k_approach,
                    self.k_speed)
        if (min(positive) <= 0 or self.l_free > 0 or not 0.0 <= self.free_ratio <= 1.0
                or self.l_min >= 0 or self.l_max <= self.occ_logodds
                or self.min_cluster_cells < 1 or self.big_min_cells < 1 or self.hold_time < 0
                or not self.min_side < self.side_length <= self.max_side < self.max_diagonal
                or self.max_candidates < 4 or self.search_timeout < 0
                or self.three_corner_wait < 0 or self.min_area < 0
                or self.w_perimeter < 0 or self.w_keep < 0 or self.switch_margin < 0
                or self.far_free < 0 or self.w_far < 0 or self.w_unexplained < 0
                or self.unexplained_margin < 0
                or not 0.0 <= self.inside_margin < self.side_length / 2.0
                or not 0.0 <= self.max_reverse <= 1.0 or self.max_forward > 1.0
                or not 0.0 <= self.max_brake <= 1.0 or self.speed_ff < 0
                or self.stop_speed < 0
                or self.max_turn > 1.0 or self.kd_yaw < 0 or self.reverse_band < 0
                or not 0.0 < self.search_ratio <= 1.0):
            raise ValueError('invalid station parameters')


# ── 1) station 전용 지도 ──────────────────────────────

class StationMap:
    """목표 중심에 고정된 log-odds 격자. [iy, ix] 순서."""

    def __init__(self, center_xy, params: StationParams):
        self.p = params
        self.center = (float(center_xy[0]), float(center_xy[1]))
        self.n = int(round(2.0 * params.map_half_size / params.resolution))
        self.origin = (self.center[0] - params.map_half_size,
                       self.center[1] - params.map_half_size)
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)
        self.observed = np.zeros((self.n, self.n), dtype=bool)

    def _cells(self, xs, ys):
        res = self.p.resolution
        ix = np.floor((xs - self.origin[0]) / res).astype(np.int64)
        iy = np.floor((ys - self.origin[1]) / res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return np.unique(iy[inside] * self.n + ix[inside])

    def integrate_scan(self, ranges, angle_min, angle_increment, range_min, range_max,
                       lidar_xy, yaw):
        """스캔 1장을 누적한다. 히트는 셀당 1회 l_occ, 빔 경로는 셀당 1회 l_free.

        빔 경로 중 큰 덩어리(부표)에 속한 칸만 l_free × free_ratio 로 덜 깎는다.
        """
        p = self.p
        ranges = np.asarray(ranges, dtype=np.float64)
        if ranges.size == 0:
            return
        angles = angle_min + angle_increment * np.arange(ranges.size)
        valid = (np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max))
        if not valid.any():
            return
        r = ranges[valid]
        wa = yaw + angles[valid]
        cos_a, sin_a = np.cos(wa), np.sin(wa)
        lx, ly = float(lidar_xy[0]), float(lidar_xy[1])

        hit = r <= p.max_range
        hit_flat = self._cells(lx + r[hit] * cos_a[hit], ly + r[hit] * sin_a[hit])

        # 빔 경로: 반 셀 간격 표본. 히트 빔은 끝 한 셀 앞까지, 먼 빔은 신뢰 거리까지.
        step = 0.5 * p.resolution
        dist = np.arange(0.0, p.max_range, step)
        free_len = np.where(hit, r - p.resolution, p.max_range)
        mask = dist[None, :] < free_len[:, None]
        xs = (lx + dist[None, :] * cos_a[:, None])[mask]
        ys = (ly + dist[None, :] * sin_a[:, None])[mask]
        free_flat = np.setdiff1d(self._cells(xs, ys), hit_flat, assume_unique=True)

        flat = self.logodds.reshape(-1)
        decay = np.full(free_flat.size, p.l_free, dtype=np.float32)
        if free_flat.size and p.free_ratio < 1.0:
            decay[self.big_mask().reshape(-1)[free_flat]] *= p.free_ratio
        flat[free_flat] += decay
        flat[hit_flat] += p.l_occ
        np.clip(self.logodds, p.l_min, p.l_max, out=self.logodds)
        seen = self.observed.reshape(-1)
        seen[free_flat] = True
        seen[hit_flat] = True

    def big_mask(self) -> np.ndarray:
        """점유(≥ occ_logodds) 칸이 8방향으로 big_min_cells 이상 이어진 덩어리의 칸들."""
        p = self.p
        labels, count = ndimage.label(self.logodds >= p.occ_logodds,
                                      structure=np.ones((3, 3), dtype=np.uint8))
        if count == 0:
            return np.zeros_like(self.observed)
        big = np.bincount(labels.ravel()) >= p.big_min_cells
        big[0] = False
        return big[labels]

    def probability_percent(self) -> np.ndarray:
        """디버그 발행용 0~100 (미관측 -1), [iy, ix]."""
        prob = np.rint(100.0 / (1.0 + np.exp(-self.logodds)))
        return np.where(self.observed, prob, -1).astype(np.int8)

    def recenter(self, center_xy) -> bool:
        """지도 중심을 center_xy 쪽으로 옮긴다 (칸 단위, 겹치는 곳의 누적은 그대로). 옮겼으면 True.

        지도는 기록 지점에 고정이라 사각형이 통째로 밀리면 먼 꼭짓점이 지도 밖으로 빠졌다.
        """
        res = self.p.resolution
        sx = int(round((center_xy[0] - self.center[0]) / res))
        sy = int(round((center_xy[1] - self.center[1]) / res))
        if sx == 0 and sy == 0:
            return False
        n = self.n
        logodds = np.zeros_like(self.logodds)
        observed = np.zeros_like(self.observed)
        # 새 칸 (iy, ix) = 옛 칸 (iy + sy, ix + sx)
        dst_x, src_x = slice(max(0, -sx), min(n, n - sx)), slice(max(0, sx), min(n, n + sx))
        dst_y, src_y = slice(max(0, -sy), min(n, n - sy)), slice(max(0, sy), min(n, n + sy))
        if dst_x.start < dst_x.stop and dst_y.start < dst_y.stop:
            logodds[dst_y, dst_x] = self.logodds[src_y, src_x]
            observed[dst_y, dst_x] = self.observed[src_y, src_x]
        self.logodds, self.observed = logodds, observed
        self.center = (self.center[0] + sx * res, self.center[1] + sy * res)
        self.origin = (self.origin[0] + sx * res, self.origin[1] + sy * res)
        return True


# ── 2) 부표 후보·꼭짓점 ──────────────────────────────────

@dataclass(frozen=True)
class Buoy:
    x: float
    y: float
    strength: float   # 덩어리 log-odds 합 (검출 노드 부표는 존재 확률 0~1)
    source: str = 'map'   # 'map' = 전용 지도 덩어리, 'detector' = /detections/buoys


def map_clusters(station_map: StationMap) -> list[Buoy]:
    """전용 지도의 부표 크기 덩어리 전부 (강한 순)."""
    p = station_map.p
    weights = station_map.logodds
    labels, count = ndimage.label(weights >= p.occ_logodds,
                                  structure=np.ones((3, 3), dtype=np.uint8))
    out = []
    if count == 0:
        return out
    for label_id, slc in enumerate(ndimage.find_objects(labels), start=1):
        if slc is None:
            continue
        local = labels[slc] == label_id
        if int(local.sum()) < p.min_cluster_cells:
            continue
        if p.resolution * max(local.shape) > p.max_cluster_extent:
            continue
        iy, ix = np.nonzero(local)
        w = weights[slc][local].astype(np.float64)
        cx = slc[1].start + float(np.average(ix, weights=w)) + 0.5   # 가중 중심 [셀]
        cy = slc[0].start + float(np.average(iy, weights=w)) + 0.5
        out.append(Buoy(station_map.origin[0] + cx * p.resolution,
                        station_map.origin[1] + cy * p.resolution, float(w.sum())))
    out.sort(key=lambda b: b.strength, reverse=True)
    return out


def candidate_buoys(station_map: StationMap, area_center=None, near=None,
                    detected=None) -> list[Buoy]:
    """꼭짓점 후보: 전용 지도 덩어리 OR 부표 검출 노드 (x, y, 확률), 같은 부표는 하나로.

    후보 영역: area_center(기록 지점, 없으면 지도 중심) buoy_search_radius 안, 또는 near(찾은
    사각형 중심) square_search_radius 안. 지도 덩어리를 먼저, 검출 노드는 확률 순으로 —
    merge_distance 안이면 같은 부표로 보고 뒤의 것을 버린다. 최대 max_candidates 개.
    """
    p = station_map.p
    area = station_map.center if area_center is None else area_center

    def inside(x, y):
        return (math.hypot(x - area[0], y - area[1]) <= p.buoy_search_radius
                or (near is not None
                    and math.hypot(x - near[0], y - near[1]) <= p.square_search_radius))

    found = [b for b in map_clusters(station_map) if inside(b.x, b.y)]
    extra = [Buoy(float(x), float(y), float(conf), 'detector')
             for x, y, conf in (detected or ())
             if math.isfinite(x) and math.isfinite(y) and inside(x, y)]
    extra.sort(key=lambda b: b.strength, reverse=True)
    chosen: list[Buoy] = []
    for b in found + extra:
        if all(math.hypot(b.x - c.x, b.y - c.y) >= p.merge_distance for c in chosen):
            chosen.append(b)
        if len(chosen) == p.max_candidates:
            break
    return chosen


def order_polygon(points):
    """점들을 무게중심 둘레 각도 순(둘레 순서)으로."""
    pts = [np.asarray(q, dtype=float) for q in points]
    c = np.mean(pts, axis=0)
    return sorted(pts, key=lambda q: math.atan2(q[1] - c[1], q[0] - c[0]))


def _station_shaped(group, params: StationParams) -> bool:
    """꼭짓점 묶음이 스테이션 사각형(의 일부)처럼 생겼나 — 간격·변·대각선·넓이.

    볼록은 요구하지 않는다: 부표가 반경 2 m 안에서 제각각 흔들리면 다섯에 하나꼴로 한 꼭짓점이
    안쪽으로 들어와 오목해진다(무작위 배치 200개 중 38개). 그 꼭짓점을 버리면 삼각형이 실제
    사각형보다 커져 밖에서 완료했다. 거의 일직선(넓이 min_area 미만)만 버린다.
    """
    p = params
    pts = [np.asarray((b.x, b.y), dtype=float) for b in group]
    gaps = sorted(float(np.linalg.norm(a - b)) for a, b in itertools.combinations(pts, 2))
    if gaps[0] < p.min_side or gaps[-1] > p.max_diagonal:
        return False
    if len(pts) == 2:
        return True
    if len(pts) == 3:          # 세 변 중 하나는 대각선일 수 있다 → 짧은 두 변만 한 변 조건
        a, b, c = pts
        area = 0.5 * abs(float(np.cross(b - a, c - a)))
        return gaps[1] <= p.max_side and area >= p.min_area / 2.0
    q = order_polygon(pts)
    sides = [float(np.linalg.norm(q[(i + 1) % 4] - q[i])) for i in range(4)]
    if max(sides) > p.max_side:
        return False
    xs, ys = np.array([v[0] for v in q]), np.array([v[1] for v in q])
    area = 0.5 * abs(float(np.dot(xs, np.roll(ys, -1)) - np.dot(ys, np.roll(xs, -1))))
    return area >= p.min_area


def group_score(group, anchor_xy, params: StationParams, keep=(), candidates=()) -> float:
    """묶음 점수 (클수록 좋음) — StationParams.w_perimeter·w_unexplained 설명 참고."""
    p = params
    pts = [np.asarray((b.x, b.y), dtype=float) for b in group]
    if len(pts) >= 3:
        q = order_polygon(pts)
        perimeter = sum(float(np.linalg.norm(q[(i + 1) % len(q)] - q[i])) for i in range(len(q)))
    elif len(pts) == 2:
        perimeter = float(np.linalg.norm(pts[1] - pts[0]))
    else:
        perimeter = 0.0
    kept = sum(1 for b in group
               if any(math.hypot(b.x - k.x, b.y - k.y) <= p.track_radius for k in keep))
    center = np.mean(pts, axis=0)
    off = float(np.linalg.norm(center - np.asarray(anchor_xy, dtype=float)))
    others = [(c.x, c.y) for c in candidates
              if all(math.hypot(c.x - b.x, c.y - b.y) > p.track_radius for b in group)]
    unexplained = 0
    if others and len(pts) >= 2:
        poly = order_polygon(pts) if len(pts) >= 3 else pts
        unexplained = int(np.sum(_clearance_many(others, poly) >= -p.unexplained_margin))
    return (100.0 * len(pts) - off - p.w_far * max(0.0, off - p.far_free) ** 2
            - p.w_perimeter * perimeter + p.w_keep * kept - p.w_unexplained * unexplained)


def select_corners(candidates, anchor_xy, params: StationParams, keep=()) -> list[Buoy]:
    """후보 중 스테이션처럼 생긴 묶음 최대 4개 (없으면 가장 강한 하나, 후보가 없으면 []).

    점수(group_score): 꼭짓점 수가 먼저, 그다음 묶음 무게중심이 anchor_xy(기록 지점)에 가깝고
    둘레가 짧고 keep(이미 잡은 꼭짓점)과 겹칠수록. 방위 하나가 아니라 묶음으로 고르므로 사각형
    근처의 다른 물체(도크 구조물 등)는 간격 조건과 둘레 감점에서 밀린다.
    """
    p = params
    cands = list(candidates)[:p.max_candidates]
    if not cands:
        return []
    best = None
    for n in (4, 3, 2):
        for group in itertools.combinations(cands, n):
            if not _station_shaped(group, p):
                continue
            score = group_score(group, anchor_xy, p, keep, cands)
            if best is None or score > best[0]:
                best = (score, list(group))
        if best is not None:
            return best[1]
    return [cands[0]]


def find_buoys(station_map: StationMap, near=None, detected=None) -> list[Buoy]:
    """지도(+검출 노드) 후보에서 꼭짓점 최대 4개 — 기록 지점(지도 중심) 기준 한 번에 고를 때."""
    cands = candidate_buoys(station_map, None, near, detected)
    return select_corners(cands, near if near is not None else station_map.center,
                          station_map.p)


# ── 3) 목표 중심 ────────────────────────────────────

def innermost_point(polygon, step: float = 0.1):
    """다각형(오목도) 안에서 경계까지 거리가 가장 큰 점과 그 거리 (격자 탐색 후 세밀화)."""
    poly = order_polygon(polygon)
    lo, hi = np.min(poly, axis=0), np.max(poly, axis=0)
    best_xy, best = None, -math.inf
    for s in (step, step / 5.0):
        if best_xy is None:
            xs, ys = np.arange(lo[0], hi[0] + s, s), np.arange(lo[1], hi[1] + s, s)
        else:
            xs = np.arange(best_xy[0] - 5 * step, best_xy[0] + 5 * step, s)
            ys = np.arange(best_xy[1] - 5 * step, best_xy[1] + 5 * step, s)
        gx, gy = np.meshgrid(xs, ys)
        pts = np.stack([gx.ravel(), gy.ravel()], axis=1)
        clear = _clearance_many(pts, poly)
        k = int(np.argmax(clear))
        if clear[k] > best:
            best_xy, best = pts[k], float(clear[k])
    return (float(best_xy[0]), float(best_xy[1])), best


def _clearance_many(points, polygon) -> np.ndarray:
    """점들 → 다각형(둘레 순서, 볼록이 아니어도) 경계까지 거리. 안이면 +, 밖이면 −.

    부표가 흔들려 한 꼭짓점이 안쪽으로 들어온 사각형(오목)도 그대로 판정한다 — 볼록 가정으로
    자르면 들어온 꼭짓점 둘레가 '안'이 되어 실제 사각형 밖에서 완료했다 (무작위 배치 시뮬).
    """
    poly = np.asarray(polygon, dtype=float)
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    dist = np.full(len(pts), np.inf)
    inside = np.zeros(len(pts), dtype=bool)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        d = b - a
        length2 = float(d @ d)
        if length2 < 1e-12:
            continue
        t = np.clip(((pts - a) @ d) / length2, 0.0, 1.0)
        dist = np.minimum(dist, np.hypot(*(pts - (a + t[:, None] * d)).T))
        if a[1] != b[1]:                                  # +x 방향 반직선과 만나는 변 수 (짝홀)
            crosses = (a[1] > pts[:, 1]) != (b[1] > pts[:, 1])
            x_at = a[0] + (pts[:, 1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            inside ^= crosses & (pts[:, 0] < x_at)
    return np.where(inside, dist, -dist)


def estimate_center(points, boat_xy, params: StationParams, hint_xy=None):
    """꼭짓점 좌표 목록 → (목표 중심 (x, y), 종류) 또는 None.

    4개: 다각형의 가장 안쪽 점 — 'four'.
    3개: 삼각형의 가장 긴 변이 max_side 를 넘으면 그 변은 대각선 — 안 보이는 꼭짓점은 그 건너편
         이므로 대각선 중점에서 건너편으로 (반대각선 − 세 번째 꼭짓점 높이)/2 만큼 (dock2 서·남·동:
         남쪽이 안쪽으로 들어와 삼각형이 얇아, 삼각형 안쪽 점은 남쪽 큐브 0.6 m 옆이었다).
         아니면 삼각형의 가장 안쪽 점 — 'three'.
    2개: 쌍 중점에서 안쪽으로 min(간격/2, pair_inset) — 'pair'. 안쪽 = hint_xy(직전 중심)
         쪽, 없으면 배 반대쪽. 배가 두 부표를 잇는 선 위(pair_inset/2 안)라 어느 쪽인지 모르면
         중점. 변인지 대각선인지 거리로 못 가르므로 이 점은 임시 목표다.
    """
    pts = [np.asarray(pt, dtype=float) for pt in points][:4]
    if len(pts) == 4:
        center, _ = innermost_point(pts)
        return center, 'four'
    if len(pts) == 3:
        diagonal = _triangle_diagonal(pts, params)
        if diagonal is not None:
            mid, away, half, height = diagonal
            return tuple(mid + away * max(0.0, (half - height) / 2.0)), 'three'
        center, _ = innermost_point(pts)
        return center, 'three'
    if len(pts) == 2:
        a, b = pts
        gap = float(np.linalg.norm(b - a))
        if gap < params.min_side:
            return None
        mid = (a + b) / 2.0
        normal = np.array([-(b - a)[1], (b - a)[0]]) / gap
        if hint_xy is not None:
            side = float(normal @ (np.asarray(hint_xy, dtype=float) - mid))
        else:
            side = -float(normal @ (np.asarray(boat_xy, dtype=float) - mid))
            if abs(side) < params.pair_inset / 2.0:
                return tuple(mid), 'pair'         # 선 위 — 어느 쪽이 안인지 모름
        if side < 0.0:
            normal = -normal
        return tuple(mid + normal * min(gap / 2.0, params.pair_inset)), 'pair'
    return None


def _triangle_diagonal(pts, params: StationParams):
    """세 꼭짓점의 가장 긴 변이 max_side 보다 길면 (대각선 중점, 건너편 단위벡터, 반대각선,
    세 번째 꼭짓점 높이), 아니면 None."""
    pairs = [(0, 1), (0, 2), (1, 2)]
    i, j = max(pairs, key=lambda ij: float(np.linalg.norm(pts[ij[0]] - pts[ij[1]])))
    a, b, v = pts[i], pts[j], pts[3 - i - j]
    length = float(np.linalg.norm(b - a))
    if length <= params.max_side:
        return None
    mid = (a + b) / 2.0
    normal = np.array([-(b - a)[1], (b - a)[0]]) / length
    height = float(normal @ (v - a))
    away = -normal if height > 0.0 else normal
    return mid, away, length / 2.0, abs(height)


def square_corners(points, center=None, params: StationParams | None = None):
    """완료 판정 영역 = 보이는 꼭짓점 다각형 (둘레 순서). 3개면 삼각형, 2개 이하면 None.

    예전엔 3개면 평행사변형, 2개면 공칭 3 m 로 나머지를 채웠는데 비정형 사각형에서는 틀린
    영역이 된다(dock2 2차 거짓 완료). 삼각형은 사각형 안에 들어 있어 판정이 보수적이다.
    """
    pts = [np.asarray(pt, dtype=float) for pt in points][:4]
    if len(pts) < 3:
        return None
    return [tuple(map(float, q)) for q in order_polygon(pts)]


def square_clearance(boat_xy, corners) -> float:
    """배 중심에서 사각형(둘레 순서 꼭짓점) 경계까지 거리 [m]. 안이면 +, 밖이면 − (오목도)."""
    return float(_clearance_many([boat_xy], corners)[0])


# ── 4) 입구 ──────────────────────────────────────────

@dataclass(frozen=True)
class EntryGate:
    a: tuple
    b: tuple
    normal: tuple     # 사각형 안쪽을 향하는 단위벡터

    @property
    def midpoint(self):
        return ((self.a[0] + self.b[0]) / 2.0, (self.a[1] + self.b[1]) / 2.0)

    @property
    def width(self):
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])


def _gate(a, b, inward):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    d = b - a
    gap = float(np.linalg.norm(d))
    normal = np.array([-d[1], d[0]]) / gap
    if float(normal @ np.asarray(inward, dtype=float)) < 0.0:
        normal = -normal
    return EntryGate(tuple(a), tuple(b), tuple(normal))


def choose_entry_gate(points, boat_xy, params: StationParams):
    """입구 = 배가 바깥에 있는 꼭짓점 다각형 변(폭 gate_min_width 이상) 중 중점이 배에 가장 가까운 변.

    2개면 그 쌍(간격 min_side~max_diagonal — 대각선이어도 그 사이는 사각형 안이라 지나도 된다).
    3개(삼각형)는 max_side 를 넘는 변(대각선)은 뺀다. 배가 이미 다각형 안이거나 입구가 없으면 None.
    """
    p = params
    pts = [np.asarray(q, dtype=float) for q in points][:4]
    boat = np.asarray(boat_xy, dtype=float)
    if len(pts) < 2:
        return None
    if len(pts) == 2:
        a, b = pts
        gap = float(np.linalg.norm(b - a))
        if not max(p.min_side, p.gate_min_width) <= gap <= p.max_diagonal:
            return None
        return _gate(a, b, (a + b) / 2.0 - boat)
    poly = order_polygon(pts)
    centroid = np.mean(poly, axis=0)
    best = None
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        gap = float(np.linalg.norm(b - a))
        if gap < max(p.min_side, p.gate_min_width) or gap > p.max_side:
            continue
        mid = (a + b) / 2.0
        inward = np.array([-(b - a)[1], (b - a)[0]]) / gap
        if float(inward @ (centroid - mid)) < 0.0:
            inward = -inward
        if float(inward @ (boat - a)) >= 0.0:
            continue                          # 배가 이 변 안쪽 — 입구 아님
        d = float(np.linalg.norm(mid - boat))
        if best is None or d < best[0]:
            best = (d, a, b, inward)
    if best is None:
        return None
    _, a, b, inward = best
    return _gate(a, b, inward)


def track_gate(gate: EntryGate, points, params: StationParams) -> EntryGate:
    """고정한 입구의 두 부표를 이번 부표 목록에 다시 붙인다 (흔들림 추종). 못 찾으면 그대로."""
    pts = [np.asarray(pt, dtype=float) for pt in points]

    def nearest(ref, exclude):
        best = None
        for k, pt in enumerate(pts):
            if k == exclude:
                continue
            d = float(np.linalg.norm(pt - np.asarray(ref)))
            if d <= params.track_radius and (best is None or d < best[0]):
                best = (d, k)
        return best

    ma = nearest(gate.a, None)
    mb = nearest(gate.b, ma[1] if ma else None)
    a = pts[ma[1]] if ma else np.asarray(gate.a)
    b = pts[mb[1]] if mb else np.asarray(gate.b)
    if float(np.linalg.norm(b - a)) < params.min_side:
        return gate
    return _gate(a, b, gate.normal)


def passes_between(boat_xy, target_xy, gate: EntryGate, params: StationParams) -> bool:
    """boat → target 직선이 입구 두 부표 사이(각 부표에서 min(간격/2, 1 m) 넘게 안쪽)를 지나나.

    아니면 입구 부표도 길 위 장애물로 본다 — 배가 입구 연장선 위(부표 바깥)에서 THROUGH 로
    판정돼 그 부표 0.45 m 옆을 지난 시뮬 사례 (dock2 사각형 밀림, 전용 지도만).
    """
    a, b = np.asarray(gate.a, dtype=float), np.asarray(gate.b, dtype=float)
    p0, p1 = np.asarray(boat_xy, dtype=float), np.asarray(target_xy, dtype=float)
    d, e = p1 - p0, b - a
    denom = float(d[0] * e[1] - d[1] * e[0])
    if abs(denom) < 1e-9:
        return False
    w = a - p0
    t = float(w[0] * e[1] - w[1] * e[0]) / denom          # 배→목표 위 위치 (0~1 이면 구간 안)
    u = float(w[0] * d[1] - w[1] * d[0]) / denom          # 입구 a→b 위 위치
    width = float(np.linalg.norm(e))
    margin = min(width / 2.0, 1.0) / width
    return -0.05 <= t <= 1.05 and margin <= u <= 1.0 - margin


def entry_target(boat_xy, gate: EntryGate, center, params: StationParams):
    """입구 밖·수직이등분선에서 벗어나 있으면 입구 앞 정렬점, 아니면 중심 → (목표, 단계)."""
    n = np.asarray(gate.normal)
    t = np.array([-n[1], n[0]])
    mid = np.asarray(gate.midpoint)
    rel = np.asarray(boat_xy, dtype=float) - mid
    along, lateral = float(rel @ n), float(rel @ t)
    pre = mid - n * params.pre_gate_distance
    if (along >= -params.gate_lateral_tol or abs(lateral) <= params.gate_lateral_tol
            or float(np.linalg.norm(np.asarray(boat_xy, dtype=float) - pre))
            <= params.pos_deadband):
        return tuple(center), 'THROUGH'
    return tuple(pre), 'ALIGN_GATE'


def detour_target(boat_xy, target_xy, obstacles, params: StationParams, clearance=None,
                  stop=None):
    """boat → target 직선이 아는 부표에서 clearance 안을 지나면 그 부표 옆 경유점.

    목표가 부표에서 target_clearance(stop 을 주면 그것과 큰 쪽)보다 가까우면 배 쪽 그 거리 점으로 바꾼다
    (기록 지점이 큐브 옆일 때 — dock2 1차에서 서쪽 큐브를 밀고 감). 배 뒤·목표 너머 부표는
    무시. 매 틱 다시 계산하므로 지나가면 원래 목표로 돌아간다.
    """
    r = params.avoid_clearance if clearance is None else float(clearance)
    near = params.target_clearance if stop is None else max(params.target_clearance, float(stop))
    boat = np.asarray(boat_xy, dtype=float)
    target = np.asarray(target_xy, dtype=float)
    obs = [np.asarray(o, dtype=float) for o in obstacles]
    for o in obs:
        if float(np.linalg.norm(target - o)) < near:
            # 이미 더 가까우면 그 거리만큼 밀어내는 점 — '그 자리'로 두면 물살에 밀려 붙었다
            away = boat - o
            dist = float(np.linalg.norm(away))
            if dist < 1e-6:
                away, dist = target - o, float(np.linalg.norm(target - o)) or 1.0
            target = o + away / dist * near
            break
    d = target - boat
    length = float(np.linalg.norm(d))
    if length < 1e-6:
        return (float(target[0]), float(target[1]))
    u = d / length
    n = np.array([-u[1], u[0]])
    worst = None
    for o in obs:
        rel = o - boat
        along, lat = float(rel @ u), float(rel @ n)
        if (0.0 < along < length and abs(lat) < r
                and float(np.linalg.norm(o - target)) >= 0.999 * near):
            if worst is None or along < worst[0]:
                worst = (along, o, lat)
    if worst is None:
        return (float(target[0]), float(target[1]))
    _, o, lat = worst
    side = -1.0 if lat > 0.0 else 1.0           # 부표가 길 왼쪽이면 오른쪽으로 돌아감
    # 부표 옆 + 진행 방향으로 조금 앞 — 옆만 두면 부표가 배 바로 앞일 때 경유점이 배 자리와 겹쳐
    # 앵커 불감대 안에서 멈춰 버렸다 (무작위 배치 시뮬 교착)
    via = o + side * n * (1.2 * r) + u * (0.8 * r)
    return (float(via[0]), float(via[1]))


def safe_point(point_xy, obstacles, clearance: float):
    """점을 아는 부표들에서 clearance 밖으로 민다 (가장 가까운 것부터 몇 번). 유지·진입 목표용 —
    오목하거나 얇은 사각형은 가장 안쪽 점이 들어온 꼭짓점에서 0.7 m 일 때도 있었다."""
    q = np.asarray(point_xy, dtype=float)
    obs = [np.asarray(o, dtype=float) for o in obstacles]
    for _ in range(3):
        close = [(float(np.linalg.norm(q - o)), o) for o in obs]
        close = [(d, o) for d, o in close if d < clearance]
        if not close:
            break
        d, o = min(close, key=lambda c: c[0])
        away = q - o if d > 1e-6 else np.array([1.0, 0.0])
        q = o + away / float(np.linalg.norm(away)) * clearance
    return (float(q[0]), float(q[1]))


# ── 5) 가상 앵커 ──────────────────────────────────────

def brake_command(speed, params: StationParams) -> float:
    """남은 전진(후진) 속도 [m/s] 를 없애는 명령. 거의 멈췄으면 0."""
    p = params
    if abs(speed) <= p.stop_speed:
        return 0.0
    return max(-p.max_brake, min(p.max_forward, -p.k_speed * speed))


def anchor_command(boat_xy, yaw, yaw_rate, target_xy, hold_yaw, params: StationParams,
                   speed: float = 0.0):
    """→ (linear, angular, 단계). linear/angular 는 최대추력 비율, speed 는 /odom 전진 속도 [m/s].

    단계: RETURN(목표 쪽 전진) / BRAKE(목표 쪽인데 너무 빨라 후진 제동) / BACK(뒤 가까운
    목표로 후진) / TURN(뒤 먼 목표 — 제자리 회전 + 제동) / ALIGN·HOLD(불감대 안 + 제동).
    """
    p = params
    dx, dy = target_xy[0] - boat_xy[0], target_xy[1] - boat_xy[1]
    dist = math.hypot(dx, dy)
    rel = _wrap(math.atan2(dy, dx) - yaw)
    brake = brake_command(speed, p)

    def turn(err):
        return _clamp(p.kp_yaw * err - p.kd_yaw * yaw_rate, p.max_turn)

    if dist <= p.pos_deadband:
        stage = 'HOLD' if brake == 0.0 else 'BRAKE'
        if hold_yaw is None:
            return brake, 0.0, stage
        yaw_err = _wrap(hold_yaw - yaw)
        if abs(yaw_err) <= math.radians(p.yaw_deadband_deg):
            return brake, 0.0, stage
        return brake, turn(yaw_err), 'ALIGN'
    if abs(rel) <= math.pi / 2.0:
        # 목표 속도는 거리에 비례 — 다가갈수록 느려져 불감대에 약 k_approach·0.5 m/s 로 들어온다
        want = min(p.max_speed, p.k_approach * dist) * math.cos(rel)
        linear = p.speed_ff * want + p.k_speed * (want - speed)
        if 0.0 < linear < p.min_forward:
            linear = p.min_forward          # ESC 가 반응하는 최소 추력
        linear = max(-p.max_brake, min(p.max_forward, linear))
        return linear, turn(rel), 'RETURN' if linear >= 0.0 else 'BRAKE'
    if dist < p.pos_deadband + p.reverse_band and p.max_reverse > 0.0:
        rev = _wrap(rel - math.pi)
        back = -min(p.max_reverse, max(p.min_forward, p.kp_pos * dist)) * math.cos(rev)
        return min(back, brake), turn(rev), 'BACK'
    # 목표를 지나쳐 뒤에 둠 — 돌아서는 동안 관성으로 더 멀어지지 않게 제동
    return brake, turn(rel), 'TURN'


# ── 5b) 출구 ─────────────────────────────────────────

def exit_waypoints(buoys, center, exit_xy, params: StationParams):
    """유지가 끝난 중심 → 출구(station_end) 경유점 [(변 안쪽 점), 사각형 밖 경유점, 출구].

    출구 방향을 가장 잘 바라보는 변(이웃한 두 부표, 간격 min_side~max_side. 3개면 삼각형의
    가장 긴 변은 대각선일 수 있어 뺀다)의 가운데를 수직으로 지나 exit_outside 만큼 밖으로
    나간 뒤 출구로 간다 — 중심에서 출구로 곧장 가면 모서리 부표에 닿을 수 있다 (dock2 2차:
    남쪽 큐브 0.42 m). 쓸 만한 변이 없으면 [출구].
    """
    c = np.asarray(center, dtype=float)
    out_dir = np.asarray(exit_xy, dtype=float) - c
    if float(np.linalg.norm(out_dir)) < 1e-6:
        return [tuple(exit_xy)]
    out_dir /= np.linalg.norm(out_dir)
    pts = [np.asarray((b.x, b.y) if hasattr(b, 'x') else b, dtype=float) for b in buoys]
    pts.sort(key=lambda q: math.atan2(q[1] - c[1], q[0] - c[0]))
    best, best_dot = None, 0.0
    edges = []
    for i in range(len(pts)):
        if len(pts) < 2 or (len(pts) == 2 and i == 1):
            break
        a, b = pts[i], pts[(i + 1) % len(pts)]
        edges.append((float(np.linalg.norm(b - a)), a, b))
    if len(pts) == 3:
        edges.remove(max(edges, key=lambda e: e[0]))
    for gap, a, b in edges:
        if not params.min_side <= gap <= params.max_side:
            continue
        mid = (a + b) / 2.0
        normal = np.array([-(b - a)[1], (b - a)[0]]) / gap
        if float(normal @ (mid - c)) < 0.0:
            normal = -normal                       # 사각형 밖을 향하게
        dot = float(normal @ out_dir)
        if dot > best_dot:
            best, best_dot = (mid, normal), dot
    if best is None:
        return [tuple(exit_xy)]
    mid, normal = best
    route = []
    # 비정형 사각형은 중심이 그 변의 수직이등분선 위가 아니다 — 변 안쪽 점을 먼저 거쳐 변을
    # 수직으로 지나게 한다 (중심에서 밖 경유점으로 곧장 가면 dock2 남쪽 큐브 1.2 m 옆). 정사각형처럼
    # 그 점이 중심과 같으면 생략.
    inner = mid - normal * params.exit_outside
    if float(np.linalg.norm(inner - c)) > params.exit_reach:
        route.append(tuple(map(float, inner)))
    return route + [tuple(map(float, mid + normal * params.exit_outside)), tuple(exit_xy)]


# ── 6) 상태기계 ──────────────────────────────────────

@dataclass
class StationDecision:
    phase: str                         # SEARCH / ENTER / HOLD
    kind: str                          # 'waypoint' (base waypoint()) | 'anchor'
    target: tuple | None = None        # 이번 주행 목표점 (HOLD 는 None — 중심)
    thrust_ratio: float = 1.0
    slow_radius: float | None = None
    linear: float = 0.0                # anchor 명령
    angular: float = 0.0
    stage: str = ''                    # 세부 단계 (로그·상태)
    center: tuple | None = None
    center_kind: str | None = None
    inside: bool = False               # HOLD: 완료 조건(사각형 안)을 지금 만족하는지
    clearance: float | None = None     # HOLD: 사각형 변까지 최소 안쪽 거리 (사각형 없으면 None)
    hold_elapsed: float = 0.0
    completed_now: bool = False        # 이번 step 에서 완료 조건 처음 달성
    buoys: list = field(default_factory=list)
    gate: EntryGate | None = None
    square: list | None = None         # 사각형 꼭짓점 4개 (둘레 순서)


class StationFsm:
    """SEARCH(꼭짓점 2개 전) → ENTER(입구로 진입) → HOLD(가장 안쪽 점 유지)."""

    def __init__(self, goal_xy, params: StationParams | None = None):
        self.p = params or StationParams()
        self.goal = (float(goal_xy[0]), float(goal_xy[1]))
        self.map = StationMap(self.goal, self.p)
        self.phase = 'SEARCH'
        self.gate: EntryGate | None = None
        self.center = None
        self.center_kind = None
        self.square = None
        self.corners: list[Buoy] = []   # 잡은 꼭짓점 (안 보여도 기억 — 라이다는 앞만 본다)
        self.through = False
        self.hold_yaw = None
        self.inside_since = None
        self.completed = False
        self.started_at = None
        self.hold_started = 0.0

    def add_scan(self, ranges, angle_min, angle_increment, range_min, range_max,
                 lidar_xy, yaw):
        self.map.integrate_scan(ranges, angle_min, angle_increment, range_min, range_max,
                                lidar_xy, yaw)

    def _update_corners(self, cands) -> list[Buoy]:
        """잡은 꼭짓점을 이번 후보에 다시 붙이고(track_radius), 매 틱 더 나은 묶음이 있는지 본다.

        지금 묶음보다 switch_margin 넘게 점수가 높은 묶음이 나오면 바꾼다. 기준점은 항상 기록
        지점 — 직전 중심을 기준으로 하면 잘못 잡은 묶음이 스스로를 굳힌다.
        """
        p = self.p
        tracked, used = [], set()
        for c in self.corners:
            best = None
            for k, b in enumerate(cands):
                d = math.hypot(b.x - c.x, b.y - c.y)
                if k not in used and d <= p.track_radius and (best is None or d < best[0]):
                    best = (d, k)
            if best is None:
                tracked.append(c)                   # 안 보임 — 기억한 자리 그대로
            else:
                used.add(best[1])
                tracked.append(cands[best[1]])
        pool = (tracked + [b for k, b in enumerate(cands) if k not in used])[:p.max_candidates]
        group = select_corners(pool, self.goal, p, keep=tracked)
        if len(tracked) < 2 or not _station_shaped(tracked, p):
            return group if len(group) >= len(tracked) else tracked
        if (group_score(group, self.goal, p, tracked, pool)
                > group_score(tracked, self.goal, p, tracked, pool) + p.switch_margin):
            return group
        return tracked

    def step(self, boat_xy, yaw, yaw_rate, now, speed: float = 0.0,
             detected=None) -> StationDecision:
        """speed: /odom 전진 속도 [m/s] (선체 기준 twist.linear.x) — 가상 앵커 제동에 쓴다.
        detected: 부표 검출 노드 부표 (x, y, 확률) 목록 — use_detected_buoys 면 후보에 더한다."""
        p = self.p
        boat = (float(boat_xy[0]), float(boat_xy[1]))
        if self.started_at is None:
            self.started_at = now
        near = self.center if self.center_kind in ('four', 'three') else None
        cands = candidate_buoys(self.map, self.goal, near,
                                detected if p.use_detected_buoys else None)
        self.corners = self._update_corners(cands)
        pts = [(b.x, b.y) for b in self.corners] if len(self.corners) >= 2 else []
        if len(pts) >= 3:
            middle = np.mean(pts, axis=0)
            if math.dist(middle, self.map.center) > p.recenter_distance:
                self.map.recenter(middle)          # 사각형이 밀려도 지도가 따라간다

        if self.phase == 'HOLD' and self.center_kind == 'goal' and len(pts) >= 2:
            self.phase = 'ENTER'                   # 예비 유지 중 부표를 찾음 → 제대로 진입
        if self.through and self.gate is not None:
            self.gate = track_gate(self.gate, pts, p)
        elif self.phase != 'HOLD':
            self.gate = choose_entry_gate(pts, boat, p)   # 지나기 전까지 매 틱 배 쪽 변
        hint = self.center if self.center_kind in ('four', 'three', 'pair') else None
        estimate = estimate_center(pts, boat, p, hint_xy=hint)
        known = [(b.x, b.y) for b in self.corners] + [(b.x, b.y) for b in cands]
        if estimate is not None:
            # 목표 중심은 아는 부표에서 target_clearance 밖으로. 꼭짓점 4개면 가장 안쪽 점이 이미
            # 꼭짓점에서 가장 먼 점이라(밀면 반대편 변에 붙는다) 꼭짓점이 아닌 부표만 본다.
            others = known if estimate[1] != 'four' else [
                o for o in known if all(math.dist(o, c) > p.merge_distance for c in pts)]
            self.center = safe_point(estimate[0], others, p.target_clearance)
            self.center_kind = estimate[1]
        self.square = square_corners(pts)

        def decision(**kw):
            return StationDecision(phase=self.phase, center=self.center,
                                   center_kind=self.center_kind, buoys=list(self.corners),
                                   gate=self.gate, square=self.square, **kw)

        if self.phase == 'SEARCH':
            if len(pts) >= 2 and self.center is not None:
                self.phase = 'ENTER'
            elif (p.fallback_to_goal and now - self.started_at >= p.search_timeout
                  and math.dist(boat, self.goal) <= p.hold_radius):
                # 끝내 꼭짓점 2개를 못 모음 — station_end 를 중심으로 유지 (예비)
                self.center, self.center_kind = self.goal, 'goal'
                self._start_hold(yaw, now)
            else:
                # 기록 지점으로 서행. 와서도 사각형이 없으면 보이는 후보(기록 지점에 가장 가까운)
                # 방위로 standoff 앞까지 — 사각형이 통째로 밀려 기록 지점에 없을 때
                target, stage = self.goal, 'TO_GOAL'
                if cands and math.dist(boat, self.goal) <= 2.0 * p.hold_radius:
                    b = min(cands, key=lambda c: math.hypot(c.x - self.goal[0], c.y - self.goal[1]))
                    away = np.asarray(boat) - (b.x, b.y)
                    dist = float(np.linalg.norm(away))
                    if dist > p.standoff:
                        target = tuple(np.asarray((b.x, b.y)) + away / dist * p.standoff)
                    else:
                        target = boat
                    stage = 'TO_BUOY'
                # 기록 지점이 부표 옆이면 standoff 앞에서 멈춘다 (관성 τ≈2.8 s — 1.5 m 면 밀려 들어감)
                target = detour_target(boat, target, known, p, stop=p.standoff)
                return decision(kind='waypoint', target=target, thrust_ratio=p.search_ratio,
                                slow_radius=p.search_slow_radius, stage=stage)

        if self.phase == 'ENTER':
            # HOLD 진입은 꼭짓점 3개 이상일 때만 — 2개로 지은 중심은 임시 목표
            if len(pts) >= 3 and math.dist(boat, self.center) <= p.hold_radius:
                self._start_hold(yaw, now)
            else:
                target, stage, skip = self.center, 'DIRECT', ()
                if self.gate is not None:
                    if self.through:
                        stage = 'THROUGH'
                    else:
                        target, stage = entry_target(boat, self.gate, self.center, p)
                        self.through = stage == 'THROUGH'
                    if stage == 'THROUGH' and passes_between(boat, target, self.gate, p):
                        skip = (self.gate.a, self.gate.b)   # 입구 두 부표 사이로 지나간다
                obstacles = [o for o in known
                             if all(math.dist(o, s) > p.track_radius for s in skip)]
                target = detour_target(boat, target, obstacles, p)
                if len(pts) == 2 and math.dist(boat, self.center) <= p.pos_deadband:
                    # 꼭짓점 2개로 임시 목표에 왔다 — 제자리에서 천천히 돌며 나머지를 찾는다
                    # (라이다는 앞 240° 만 본다)
                    return decision(kind='anchor', target=target,
                                    linear=brake_command(speed, p),
                                    angular=0.5 * p.max_turn, stage='LOOK')
                # 부표 사이를 지나므로 해류에 밀리지 않게 가상 앵커(전진 상한 max_forward)로 간다
                linear, angular, _ = anchor_command(boat, yaw, yaw_rate, target, None, p,
                                                    speed=speed)
                return decision(kind='anchor', target=target, linear=linear,
                                angular=angular, stage=stage)

        # HOLD — 완료 조건: 배 중심이 꼭짓점 다각형 안 (각 변에서 inside_margin 안쪽, 3개면 삼각형).
        # 부표를 못 찾아 station_end 로 유지 중이면(예비) 반경 hold_radius. 그 밖엔 완료 없음.
        clearance = None
        if self.square is not None:
            clearance = square_clearance(boat, self.square)
            inside = clearance >= p.inside_margin
        else:
            inside = self.center_kind == 'goal' and math.dist(boat, self.center) <= p.hold_radius
        if len(pts) == 3 and now - self.hold_started < p.three_corner_wait:
            inside = False                     # 네 번째를 찾는 중 — 아직 삼각형으로 판정 안 함
        completed_now = False
        elapsed = 0.0
        if inside:
            if self.inside_since is None or now < self.inside_since:
                self.inside_since = now
            elapsed = now - self.inside_since
            if not self.completed and elapsed >= p.hold_time:
                self.completed = True
                completed_now = True
        else:
            self.inside_since = None
        steer = self.center
        if math.dist(boat, self.center) > p.hold_radius:
            steer = detour_target(boat, self.center, known, p)   # 밀려났다 돌아올 때 부표 옆으로
        linear, angular, stage = anchor_command(boat, yaw, yaw_rate, steer,
                                                self.hold_yaw, p, speed=speed)
        if len(pts) == 3 and stage in ('HOLD', 'ALIGN', 'BRAKE'):
            # 자리에서는 제자리에서 천천히 돌며 안 보이는 네 번째 꼭짓점을 찾는다 (라이다 앞 240°)
            angular, stage = 0.5 * p.max_turn, 'LOOK'
            self.hold_yaw = float(yaw)
        return decision(kind='anchor', linear=linear, angular=angular, stage=stage,
                        inside=inside, clearance=clearance, hold_elapsed=elapsed,
                        completed_now=completed_now)

    def _start_hold(self, yaw, now=0.0):
        self.phase = 'HOLD'
        self.hold_yaw = float(yaw)
        self.inside_since = None
        self.hold_started = now
