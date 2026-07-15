#  장애물 회피 로컬 플래너 (DRI + B-spline) — 작업 계획

`obstacle_planner` behavior 를 단순 반발 조향에서 DRI 격자 + B-spline 로컬
플래너로 교체하는 작업 계획. 설계 논의에서 정한 사항과 모듈 경계·인터페이스를
적어둔 문서.

> **이 문서의 용도**: 새 세션이 작업을 이어받을 때 읽는 인수인계 노트다.
> 여러 사람이 나눠 맡기 위한 계약서가 아니므로, 설계가 틀렸다고 판단되면
> **고쳐도 된다** — 대신 §7 변경 로그에 무엇을 왜 바꿨는지 남길 것.
> 아래 인터페이스는 "합의된 규약"이 아니라 "지금까지 정한 최선"이다.

> 관련 문서: [SKELETON.MD](SKELETON.MD) §4 `obstacle_planner` / §6 안전값.
> 완료 후 이 문서 내용은 SKELETON.MD 로 흡수하고 본 문서는 삭제 예정.

---

## 0. 요구사항

### 기능 요구사항 — 파이프라인 단계별

플래너는 매 tick(10Hz) 아래 파이프라인을 돈다. 각 단계가 "무엇을 보장해야
하는가"를 정의한다. 괄호 안 `Rn` 은 이후 절에서 참조하는 요구사항 ID.

```
/occupancy_grid ─▶ ① DRI 생성 ─▶ ② 경로 생성 ─┬▶ ③ 추종+검사 ─▶ /cmd/avoid
                                    ▲          │       │
                                    │ 재생성    │ 수리   │ 위반
                                    └──────────┴───────┘
                                    (안전 경로 불가 시 ⑤ ESCAPE)
```

**① DRI 생성** (`/occupancy_grid` → DriGrid). occupancy_grid(20×20m·0.2m·1Hz)의
점유 셀을 위험도 float 격자로 바꾼다. 점유 셀마다 **등방 가우시안**으로
위험을 퍼뜨리되, 배와 **가까울수록** / 배 **heading 전방에 있을수록** 더 크고
강하게 퍼뜨린다 (**R1**, 수식 §1.1). 미관측(-1) 셀은 기여 0 = **free 취급**
(**R8**) — 위험 취급하면 센서 범위 밖으로 영영 못 나간다.

**② 경로 생성** (DriGrid + 배 상태 + waypoint → Plan). **제어점 6개**(§1.3)로
clamped cubic B-spline 을 만든다 (**R2**). 장애물이 ROI 에 하나도 없으면
waypoint 직선 방향 제어점으로 폴백 (**R7**). 생성은 **항상 유효한 Plan 또는
None(안전 경로 불가)** 을 반환 — 프로토타입식 다층 emergency fallback 금지.

**③ 추종 + 검사 + 수리** (Plan → /cmd/avoid). 생성과 검사는 **분리** (**R3**):
추종 중에는 재생성하지 않고 10Hz 로 경로 전방 구간만 최신 DRI 로 검사한다.
검사 위반 시 **수리** (**R4**): 위반 구간의 arc 제어점을 자기 arc 위에서
방위각만 이동(반지름 고정), N회까지. 경로 → `/cmd/avoid`(Twist) 변환은
spline lookahead 점 PD 조향 + 곡률/전방 DRI 감속 (**R6**) — `/cmd/avoid` 는
경로 벡터가 아니라 **제어값(속도 명령)**.

**④ 재생성** (트리거 3종, OR — 먼저 걸리는 것 승). (a) `dist(현재, 계획시점 p0)
≥ r1 − margin` = 첫 기준 장애물 통과, (b) 수리 N회 실패, (c) 마지막 생성 후 T초
경과(주기 백스톱 — 표류·동적 장애물 대응) (**R5**).

**⑤ ESCAPE** (안전 경로 불가 시). generate 가 None(argmin 점도 DRI > `hard_max`),
**또는** 위반이 경로 초입(p0→p1)에서 나 수리·재생성이 모두 실패하면, 경로 추종을
포기하고 **후진 + 장애물 반대(저DRI)쪽 조향**으로 전환 (**R9**). 배의 관성이
장애물 쪽을 향하면 spline 시작 접선이 그 운동에 묶여(§1.3 p1) 어떤 경로도 안전하지
않으므로, 경로가 아니라 **운동을 먼저 바꾼다**. 후진으로 ‖v‖ 가 줄면
`p1=p0+v·t_look` 이 p0 로 수축해 다음 재생성이 접선 구속에서 풀린다(자기교정) —
p1 을 인위적으로 짧게 만들 필요 없음. ⚠️ §6 전복 위험 — 후진 linear.x 는
작게(≈−0.05), sim 에서 역추력 검증 후.

### 비기능 요구사항 / 제약
- **N1**: `behavior_base` 계약 유지 — active 일 때 10Hz `compute_cmd()` → Twist.
  cmd_mux 300ms 워치독이 있으므로 **재생성 중에도 발행이 끊기면 안 됨**
  (생성은 같은 tick 안에서 동기로 — 100×100 numpy 라 수 ms 면 충분).
- **N2**: §6 안전값 불변 — `YAW_KP=0.3`, `YAW_KD=0.15`, `max_linear=0.20`,
  `max_angular=0.05` 를 바꾸지 않는다. 조향 여유가 없으므로 **감속이 회피
  성능의 절반** — 속도 스케일링을 성실히 구현할 것.
- **N3**: dri / planner 는 **ROS 의존 없는 순수 라이브러리 모듈**(numpy 입출력)
  로 작성 — ROS 없이 단독 유닛테스트 가능해야 함. 노드(`obstacle_planner.py`)
  만 ROS 를 안다. ("노드가 test 파일을 import" 안티패턴 금지)
- **N4**: 좌표는 전부 odom(world) frame. DRI 격자는 occupancy_grid 와 같은
  anchor/resolution/크기 (셀 1:1 대응).
- **N5**: 모든 튜닝값은 ROS 파라미터로 노출 (§5 표).
- **N6**: scipy 1.8 (컨테이너 확인됨) — B-spline 은 `scipy.interpolate` 사용 가능.

---

## 1. 모듈 구성과 인터페이스

```
kaboat_behaviors/
  dri.py                ← [작업 1] 순수 라이브러리 (ROS msg → DriGrid)
  bspline_planner.py    ← [작업 2] 순수 라이브러리 (generate/check/repair)
  obstacle_planner.py   ← [작업 3] ROS 노드 (상태기계 + follow_path)
  avoid_viz.py          ← [작업 4] 디버그 발행 헬퍼 (노드 아님, planner 가 호출)
```

### 1.1 `dri.py`

```python
@dataclass
class DriGrid:
    data: np.ndarray        # (H, W) float32 — 위험도 ≥ 0, [iy, ix] (grid msg 와 동일)
    anchor: tuple[int, int] # (ax, ay) — 배열 (0,0) 셀의 global 격자 인덱스
    resolution: float       # [m/cell]

    def risk_at(self, x: float, y: float) -> float:
        """world 좌표의 DRI 값. window 밖이면 math.inf (계획이 창 밖으로 못 나가게)."""

def build_dri(grid: OccupancyGrid, boat_xy: tuple[float, float],
              boat_yaw: float, p: DriParams) -> DriGrid:
    """OccupancyGrid msg → DRI. 점유 셀(값 ≥ occ_threshold)마다 등방 가우시안:
       - 반지름 σ, 진폭 A 를 근접성·전방성으로 키움
       - unknown(-1) 셀은 기여 0 (free 취급, R8)"""
```

`OccupancyGrid` msg 파싱은 `build_dri` 안에서만. 이후 전 모듈은 `DriGrid` 만 안다.

**DRI 수식 (작업 1 구현 스펙).** 배 위치 `b`, heading `ψ`. 점유 셀 `c` 하나가
격자점 `x` 에 기여하는 위험을 **등방** 가우시안으로 정의한다.

- `d = c − b`, 거리 `ρ = ‖d‖`.
- 전방성 `f = max(0, cos(∠(d, heading))) ∈ [0,1]` (장애물이 선수 정면이면 1, 옆이면 0).
- 근접성 `g(ρ) = 1 / (1 + ρ/ρ₀) ∈ (0,1]` (`ρ₀` = `dist_falloff`, 가까울수록 1).

크기(σ)와 세기(A)를 근접·전방성으로 키운다 — **배와 가깝거나 정면일수록 더 크고
강하게** (R1):

```
σ(c) = sigma · (1 + k_dist·g(ρ)) · (1 + k_head·f)     # 원의 반지름
A(c) = (1 + a_dist·g(ρ)) · (1 + a_head·f)             # 원의 진하기

셀 c 가 x 에 주는 값:
  contrib(x; c) = A(c) · exp( −‖x−c‖² / (2σ(c)²) )

DRI(x) = max_c contrib(x; c)   # 최근접(=가장 위협적인) 장애물이 결정
```

- **max vs 합산(Σ)**: **max 채택** (§7-2 에서 초안의 합산안을 뒤집음). 충돌 안전은
  최근접 장애물이 결정하므로 합이 아니라 max 가 물리적으로 맞다. 합산은 통과 가능한
  3m 틈과 최근접 2m 인 밀집 지대를 오답으로 차단해, 부표 밭에서 배가 ESCAPE 로
  포기하게 만든다. max 는 DRI 를 `[0, (1+a_dist)(1+a_head)]` 로 유계로 만들어
  §1.3 비용 균형과 threshold 의 의미도 안정시킨다.
- **등방인 이유** (구 비등방안을 폐기한 근거, §7-1): 축을 배→장애물 방향으로 잡고
  `σ_along` 만 키우면 (i) 게인이 붙는 순간 종횡비가 6:1 을 넘어 장애물마다 배를
  향한 시가 모양이 생기고, 그게 합쳐져 **배에서 뻗어나가는 V자 줄기**가 20m 창을
  덮는다. (ii) 더 근본적으로, 배가 장애물을 얼마나 떨어져 지나갈지(berth)는 원호
  위 **방위각**으로 정해지는데 그 방향이 하필 고정축 `σ_cross` 라 **게인을 올려도
  berth 가 안 넓어졌다** — R1 의 "더 넓게"가 실제로는 달성되지 않는 수식이었다.
  등방이면 σ 가 곧 berth 라 게인이 의도대로 먹고, 게인을 아무리 키워도 원이 커질
  뿐 찌그러지지 않는다.
- **구현**: 각 점유 셀마다 `3σ` 로 잘라낸 국소 패치(반경 `3σ/res`)에 splat 해 더한다.
  등방 가우시안은 **분리 가능**하므로 `exp(−dx²/2σ²)` ⊗ `exp(−dy²/2σ²)` 외적으로
  계산한다 (회전축이 없어서 가능 — 비등방안 대비 1.5배 빠름).
- `sigma`, `dist_falloff`, `k_dist/k_head`, `a_dist/a_head` 는 전부 파라미터(§3).
  게인 4개를 0 으로 두면 크기·세기가 고정된 균일 가우시안으로 축약된다(디버그 기준선).

### 1.2 `bspline_planner.py`

```python
@dataclass
class Plan:
    cps: np.ndarray          # (6, 2) 제어점 world 좌표 [p0..p5]
    samples: np.ndarray      # (M, 2) spline 샘플 (호길이 순, 간격 ≈ 0.5·res)
    origin: tuple[float, float]  # 계획 시점 배 위치 (= p0) — 재생성 트리거 (a) 기준
    r1: float | None         # 첫 기준 장애물 arc 반지름. 장애물 없으면 None
    radii: list[float]       # p2~p5 의 arc 반지름 [r1, r2, r3, r_end] (수리용)
    bearings: list[float]    # p2~p5 의 origin 기준 방위각 (수리 = 이 값만 이동)
    created_t: float         # 생성 시각 [s] — 재생성 트리거 (c) 기준
    progress_idx: int = 0    # 배가 지나온 samples 진행 인덱스 — check 가 갱신(단조 전진)

def generate(dri: DriGrid, boat_xy, boat_yaw, boat_vel_xy,
             waypoint_xy, p: PlannerParams) -> Plan | None:
    """제어점 6개 생성 + spline 샘플링. 후보가 전혀 없으면 None (호출측이 정지/최저속)."""

def check(plan: Plan, dri: DriGrid, boat_xy, p: PlannerParams) -> int | None:
    """전방 구간만 DRI 평가. 첫 위반 샘플 인덱스, 없으면 None.
       진행 갱신: progress_idx = argmin dist(배, samples[progress_idx .. +window])
       — 단조 전진(뒤로 안 돌아감). 검사 구간 = [progress_idx, 끝] 전부
       (r_end 9m / 0.1m 간격 ≈ 90샘플이라 10Hz 부담 없음 — 먼 위반도 일찍
       발견해 일찍 수리하는 게 이득). 위반 = DRI > safe_threshold."""

def repair(plan: Plan, dri: DriGrid, violation_idx: int, p: PlannerParams) -> bool:
    """위반 샘플에 가장 가까운 arc 제어점(p2~p5)을 자기 arc 위에서 Δθ 이동
       (DRI 낮아지는 쪽). plan 을 제자리 수정 + spline 재샘플. 성공 True.
       p0/p1 은 수리 대상 아님(§1.3) — 위반이 p2 앞(초입)이라 책임질 arc CP 가
       없거나 arc 부채꼴 한계 도달 시 False → 호출측이 재생성/ESCAPE 로 간다."""
```

### 1.3 제어점 규칙 (확정안)

| CP | 정의 |
|---|---|
| p0 | 계획 시점 배 위치 |
| p1 | `p0 + max(‖v‖·t_look, p1_min_dist) · (v̂ 또는 ‖v‖≈0 이면 heading 단위벡터)`. **속도 관성을 정직하게 반영** — 인위적으로 짧게 안 함(R9 자기교정에 의존). `p1_min_dist` 는 정지(v≈0) 시 접선 방향을 주기 위한 하한일 뿐(캡 아님). 수리 불가(§1.2) |
| p2 | 반지름 `r1 = 기준 장애물의 실제 거리` arc 위 비용 최소점. 기준 장애물 선택은 아래 규칙 |
| p3, p4 | `r_k = r1 + k·(r_end − r1)/3` (k=1,2) arc 위 비용 최소점 — **균등 분배** (고정 offset 아님: r 순서 보장) |
| p5 | `r_end = grid 반경 − boundary_margin` (기본 ≈9m) arc 위 비용 최소점. 미관측 셀 허용 (R8) |

- **r1 기준 장애물 선택 — 후방 하드컷 + 방위 가중 유효거리** (순수 최근접 셀은
  방금 지나친 옆/뒤 장애물이 뽑혀 arc·재생성 트리거 (a)가 무의미해짐):

  ```
  각 점유 셀 c:  Δθ = |배 기준 상대방위(c) − 0|   (heading 기준)
    Δθ > bearing_max → 제외                       # 후방/측후방 하드컷
    ρ_eff = ρ · (1 + k_bear·(1 − cos Δθ))          # 옆일수록 거리 부풀림
  기준 장애물 = argmin ρ_eff,  r1 = 그 셀의 실제 거리 ρ
  ```

  같은 거리면 정면 우선, 정면이 멀면 가까운 측면이 뽑히는 연속 트레이드오프.
  `k_bear=0` 이면 순수 최근접으로 축약(디버그 기준선). 기준은 waypoint 방향이
  아니라 **heading** — 초입 경로는 p1 접선(=운동 방향)을 따르므로 "곧 협상할
  장애물"은 heading 기준이 맞고, waypoint 끌림은 arc 비용(`w_wp`)이 담당(역할 분리).
- **후보 탐색**: 각 arc 에서 waypoint 방위 중심 ±`search_fov/2` 부채꼴을
  `n_candidates` 개 등간격 샘플.
- **선택 비용**: `w_dri·DRI + w_wp·|방위 − waypoint방위| + w_prev·|방위 − 직전 CP 방위|`
  argmin — safe 후보가 없어도 최소비용 점이 뽑히므로 별도 fallback 층 불필요.
  단 argmin 점의 DRI 가 `hard_max` 초과면 generate 는 None.
- **장애물 없음(R7)**: r1=None, p2~p4 반지름은 `‖p1−p0‖` 과 `r_end` 사이 균등 분배,
  방위는 전부 waypoint 방향.
- **spline**: clamped cubic B-spline (양 끝점 통과), 6 CP, `scipy.interpolate.BSpline`.

### 1.4 `obstacle_planner.py` 상태기계 (10Hz tick 내부)

```
compute_cmd():
  1) DRI 갱신 — **매 tick** build_dri(캐시된 최신 grid msg, 지금 pose)  (§7-4)
  2) 재생성 판정 — plan 이 없거나:
       (a) r1 존재 && dist(now, plan.origin) ≥ r1 − regen_margin
       (c) now − plan.created_t ≥ regen_period
     → plan = generate(...)
  3) idx = check(plan, ...) 위반이면:
       repair 최대 repair_max 회 → 실패 시 (b) plan = generate(...)
  4) plan 이 None(generate 실패) 이면: return escape_cmd(dri, ...)  # R9
  5) return follow_path(plan)
```

`escape_cmd(dri)`: 전방 부채꼴에서 DRI 최소 방위를 찾아 그쪽으로 조향
(`angular.z = YAW_KP·err − YAW_KD·yaw_rate`) + **후진**(`linear.x ≈ −0.05`,
§6 전복 주의). 로그로 "ESCAPE 모드" 명시 — sim 에서 자주 뜨면 grid 확대/
`hard_max` 재조정 신호. 후진으로 ‖v‖↓ → 다음 tick 재생성이 접선 구속에서 풀림.

`follow_path(plan)`: `plan.progress_idx` 에서 호길이 `lookahead_dist` 전방
샘플을 임시 목표로 방위오차 계산 → `angular.z = YAW_KP·err − YAW_KD·yaw_rate`
(기존 검증 게인 그대로). 감속(속도 공식):

```
f_curv = 1 / (1 + k_curv·κ)      # κ = lookahead 창 내 최대 곡률(이웃 샘플 3점 근사)
                                  #   요레이트 한계로 못 도는 곡률 → 속도로 흡수 (N2)
f_dri  = clamp(1 − DRI_max/safe_threshold, 0.2, 1)   # 전방 창 최대 DRI 비례 감속
linear.x = max_linear · min(f_curv, f_dri)
단 |방위오차| > 60° 면 linear.x = 0.3·max_linear    # seek_goal 의 전복 방지 규칙 유지
```

### 1.5 `avoid_viz.py`

```python
def publish_debug(node, dri: DriGrid | None, plan: Plan | None, repaired: bool)
```
- `/avoid/dri` (OccupancyGrid — DRI 를 0~100 스케일, RViz 용)
- `/avoid/path` (nav_msgs/Path — spline 샘플. 수리된 경로는 다른 색 Marker 병행)
- `/avoid/cps` (MarkerArray — 제어점 6개 + arc 원호)

---

## 2. 작업 분할

작업 사이 의존성은 **이 문서의 인터페이스뿐**이라, 아무 순서로 집어도 되고
다른 작업의 구현이 없어도 각각 단독으로 진행·테스트할 수 있다.

```
[작업 1] dri.py ──────────────┐
[작업 2] bspline_planner.py ──┼──▶ [작업 5] 통합 + sim 튜닝
[작업 3] obstacle_planner.py ─┤
[작업 4] avoid_viz.py ────────┘
```

### 작업 1 — `dri.py` (복잡도 낮음) — ✅ 완료
- **입력물**: §1.1 인터페이스.
- **단독 테스트**: ROS 없이 — 합성 OccupancyGrid(점유 셀 1~3개 수제 배열)로
  (i) 피크가 장애물 셀 위, (ii) 퍼짐이 등방(게인을 켜도 원),
  (iii) 가까운/전방 장애물이 더 크고 더 강함, (iv) unknown 기여 0 을 assert.
- **완료 기준**: 위 테스트 통과 + 100×100 격자 build_dri < 10ms.
- **결과**: `test/test_dri.py` 15개 통과. 실제 sim 격자(점유 셀 60~73개)에서
  build_dri ≈ **2.6ms**. 합성 부하로는 200셀 4.0ms / 500셀 8.5ms / 1000셀 15.3ms —
  선형이고 셀당 numpy 호출 오버헤드가 병목이라, 장애물 밭에서 500셀을 넘기면
  반경 버킷별 일괄 벡터화가 다음 수단.
- **테스트 함정**: 등방성을 world 좌표로 검증하면 안 된다 — `risk_at` 이 셀
  중심으로 스냅하는데 `4.6/0.2 = 22.999...` 라 floor 가 ±1셀 어긋나 3% 오차가 난다.
  배열 인덱스로 상하좌우 n셀을 비교할 것 (`_ring` 헬퍼).

### 작업 2 — `bspline_planner.py` (복잡도 높음 — 최대 리스크)
- **입력물**: §1.2, §1.3. `DriGrid` 는 인터페이스만 필요 — **수제 numpy 배열로
  가짜 DriGrid 를 만들어 개발** (작업 1 완성 불필요).
- **단독 테스트**: ROS 없이 —
  (i) radii 단조증가 assert, (ii) 정지 상태 p1 퇴화 방지,
  (iii) 단순 장면(장애물 1개 정면)에서 생성 경로가 check 무위반,
  (iv) 장애물을 경로 위에 추가 → check 가 위반 검출 → repair 가 DRI 낮은
  쪽으로 CP 이동 → 재검사 통과, (v) 장애물 0개 → R7 직선 폴백,
  (vi) waypoint 가 r_end 안쪽/바깥쪽 둘 다 정상,
  (vii) 기준 장애물 선택 — 배 뒤 1m + 정면 5m 장면에서 정면 장애물이 뽑히고
  r1=5m (후방 하드컷), 정면 8m + 측면 60° 3m 장면에서 측면이 뽑힘 (유효거리).
- **완료 기준**: 위 테스트 통과. matplotlib 시각화 스크립트(스크래치, 커밋 안 함) 권장.

### 작업 3 — `obstacle_planner.py` (복잡도 중간)
- **입력물**: §1.4. 플래너가 없어도 됨 — **직선 Plan 을 돌려주는 stub** 으로 개발.
- **단독 테스트**: sim 에서 stub 직선 경로를 follow_path 가 안정 추종(발진 없음),
  재생성 트리거 (a)(c) 발동 로그 확인, cmd 발행 10Hz 유지(워치독 무정지).
- **완료 기준**: stub 로 avoid 구간 직선 주행 완주 + 전복 없음.

### 작업 4 — `avoid_viz.py` (복잡도 낮음)
- **입력물**: §1.5. Plan/DriGrid 는 수제 객체로 채워 발행 테스트 가능.
- **완료 기준**: RViz 에서 세 토픽 표시 확인.

### 작업 5 — 통합 + 튜닝 (불확실성 최대 — 소요 예측 불가, 반복 튜닝)
- stub → 실제 모듈 교체, avoid 미션(goal [17.5, 77.0] 장애물 밭) 완주.
- 튜닝 순서: DRI(σ, 게인) RViz 로 먼저 → threshold/비용 가중 → 감속 계수 →
  regen_period. **몇 분 이상 5개 미션 전체 완주로 판정** (§6 SKELETON 동일 이유).
- **완료 기준**: avoid 구간 접촉 없이 완주 ×3회 연속, 다른 미션 회귀 없음.

---

## 3. 파라미터 (전부 ROS param, 기본값은 초안)

| 파라미터 | 기본 | 의미 |
|---|---|---|
| `dri.occ_threshold` | **75** | 이 점유확률 이상 셀만 위험원. **= "LiDAR 히트 몇 번부터 장애물로 믿을까" 손잡이**(§7-3): 히트 1회→발행값 70, 2회→85, 3회→93 이므로 50 이면 **단발 노이즈도 즉시 장애물**, 75 면 2회, 90 이면 3회를 요구. 정상 상태 셀은 12/97 로 포화해 있어 이 값에 안 흔들린다 — 오직 노이즈 필터로만 작동 |
| `dri.sigma` | 1.0 m | 등방 가우시안 σ 기준값(스케일 전) = berth 폭 |
| `dri.dist_falloff` (ρ₀) | 4.0 m | 근접성 `g(ρ)=1/(1+ρ/ρ₀)` 감쇠 스케일 |
| `dri.k_dist / k_head` | 0.3 / 0.3 (튜닝) | σ 의 근접·전방성 확대 게인 |
| `dri.a_dist / a_head` | 1.0 / 1.0 (튜닝) | 진폭 A 의 근접·전방성 확대 게인 |
| `planner.t_look / p1_min_dist` | 5.0 s / 1.0 m | p1 규칙 |
| `planner.bearing_max / k_bear` | 100° / 0.5 | r1 기준 장애물 — 후방 하드컷 / 방위 가중 |
| `planner.boundary_margin` | 1.0 m | r_end = grid 반경 − 이 값 |
| `planner.search_fov / n_candidates` | 180° / 31 | arc 후보 탐색 |
| `planner.w_dri / w_wp / w_prev` | 튜닝 | 후보 선택 비용 가중 |
| `planner.safe_threshold / hard_max` | 튜닝 | 검사 위반 기준 / generate 포기 기준 |
| `planner.repair_max / repair_dtheta` | 3 / 6° | 수리 반복 한도 / 1회 이동각 |
| `planner.regen_margin` | 1.0 m | 트리거 (a) 여유 |
| `planner.regen_period` | 4.0 s | 트리거 (c) 주기 백스톱 |
| `follow.lookahead_dist` | 1.5 m | pure-pursuit 전방 거리 |
| `follow.k_curv` | 튜닝 | 곡률 감속 강도 (§1.4 f_curv) |
| `escape.reverse_speed` | 0.05 | R9 ESCAPE 후진 속도(§6 전복 주의, sim 검증 후) |

## 4. 리스크

1. **곡률 vs 조향한계**: max_angular=0.05 로는 spline 급곡률을 못 따라감 —
   1차 대응은 감속(N2), 그래도 부족하면 계획 단계 곡률 제약 추가(작업 5 에서 판단).
2. **작업 2 엣지케이스**가 전체 소요의 주 변수 — 위 단독 테스트 목록을 먼저
   작성하고 구현하는 걸 권장 (test-first).
3. **grid 20m 창이 좁다고 판명되면**: occupancy_grid `size` 파라미터만 키우면
   전 모듈이 자동 대응 (r_end 가 grid 반경 파생값이므로 코드 변경 없음).
4. ~~**DRI 크기가 장애물 밀도에 좌우됨**~~ — **해소됨**(§7-2, 합산→max). max 면
   `DRI ≤ (1+a_dist)(1+a_head)` 로 밀집도와 무관하게 유계라, 비용 균형도 threshold 도
   흔들리지 않는다. (합산 시절 sim 실측이 점유 셀 22개→16, 51개→61 로 출렁였다.)

---

## 7. 변경 로그

계획서 자체를 바꿨을 때 무엇을 왜 바꿨는지 남긴다 (새 세션이 "왜 이렇게 돼
있지?" 를 되묻지 않도록).

### 7-1. DRI 를 비등방(타원) → 등방(원) 으로 (작업 1 중)

**바꾼 것**: §1.1 수식에서 `σ_along`/`σ_cross` 2축을 단일 `σ` 로 통합.
게인 `k_dist`/`k_head` 가 이제 원의 반지름을 키운다. §3 표도 같이 수정.

**왜**: sim + RViz 로 실제 위험장을 그려보니 구 수식이 병리적이었다.

1. `σ_along` 만 키우고 `σ_cross` 를 고정한 탓에 종횡비가 거리·방위에 따라
   2.7:1 ~ **6.7:1** 로 벌어졌다 (2m 정면 장애물: σ_along 4.0m vs σ_cross 0.6m).
   각 장애물이 배를 향한 시가 모양이 되고, 그게 합쳐져 **배에서 뻗어나가는 V자
   줄기**가 20m 창을 통째로 덮었다 (3σ = 24m > 창 크기).
2. 더 근본적으로 — berth(배가 장애물을 얼마나 떨어져 지나가는가)는 원호 위
   **방위각**으로 정해지고 그 방향은 `d̂⊥` = 고정축 `σ_cross` 다. 즉 게인을 올려도
   **berth 가 전혀 안 넓어졌다.** R1 의 "가까울수록 더 넓게"가 수식상 달성 불가였고,
   `σ_along` 확대는 방사 방향 그림자만 늘렸다.

**덤**: 등방 가우시안은 분리 가능해서 1D 두 개의 외적으로 계산된다 → 1.5배 빨라짐
(500셀 12.5ms → 8.5ms 로 10ms 예산 안에 들어옴). 회전 좌표 변환이 통째로 사라짐.

**폐기된 대안**: 게인만 낮춰서(k≈0.3) 타원을 완화하는 안 — 종횡비가 줄 뿐 berth
문제(2번)가 그대로 남아서 버렸다.

### 7-2. 셀 결합을 합산(Σ) → max 로 (초안 §1.1 의 "합산 채택"을 뒤집음)

**바꾼 것**: `DRI(x) = Σ_c contrib` → `DRI(x) = max_c contrib`. 초안 §1.1 이 합산을
채택하며 든 근거("촘촘하면 위험이 쌓여 더 크게 우회한다", "위험장이 매끄러워 repair
gradient 가 안정적")는 **둘 다 기각**한다.

**왜 — 합산은 물리적으로 틀렸다.** 충돌 안전은 **최근접 장애물**이 결정한다. 배는
가장 가까운 것에 부딪히지 여러 장애물의 **합**에 부딪히지 않는다. 합산은 "하나에
가까움"과 "여럿에 적당히 가까움"을 뒤섞는다. σ=1.0 / threshold=0.5 (= 안전반경
R=1.18m) 기준 실측 판정:

| 상황 | 최근접 | 정답 | 합산 | max |
|---|---|---|---|---|
| 외딴 부표 0.6m 옆 | 0.60m | 차단 | 차단 | 차단 |
| 1.2m 틈 정중앙 | 0.60m | 차단 | 차단 | 차단 |
| **3.0m 틈 정중앙** | 1.50m | **통과** | **차단 ✗** | 통과 |
| **밀집: 부표 5개가 각 2.0m** | 2.00m | **통과** | **차단 ✗** | 통과 |
| 밀집: 부표 8개가 각 2.5m | 2.50m | 통과 | 통과 | 통과 |

합산은 통과 가능한 3m 틈과 최근접 2m 인 밀집 지대를 **오답으로 차단**한다. 양옆
장애물의 꼬리가 더해져서다. 그리고 이건 하필 **avoid 미션의 정확한 실패 모드** —
부표 밭에서 후보가 전부 차단되면 generate→None→**ESCAPE 로 배가 포기**한다.
설계 목적 그 자체인 상황에서 무너진다.

**기각한 초안 근거 2개**:
- *"촘촘하면 더 크게 우회"* — 의도는 좋으나 합산은 그걸 **위험장을 거짓으로 만들어서**
  달성한다. 통과 가능한 틈을 막는 대가라면 안 된다. 밀집 지대를 통째로 돌아갈지는
  위험장이 아니라 **플래너의 비용 트레이드오프**가 정할 일이다 (역할 분리).
- *"gradient 가 매끄러움"* — max 는 argmax 가 바뀌는 경계에서 미분불가(C0)지만,
  repair 는 CP 를 Δθ 씩 옮겨보는 **이산 탐색**이고 generate 는 후보 31개 argmin 이라
  해석적 gradient 를 안 쓴다. 무관한 걱정이었다.

**덤 — §4-4 가 통째로 사라진다.** max 면 `DRI ≤ max A = (1+a_dist)(1+a_head)` 로
**유계**다(밀집도 무관). 합산 시절의 "sim 실측 16~61 로 출렁여서 §1.3 비용 균형이
흔들린다"는 문제가 없어지고, `safe_threshold`/`hard_max` 도 절대적 의미를 갖는다.

**같이 검토하고 기각한 것 — noisy-OR / 베이즈 셀 결합.** `1 − Π(1−p_c) = 1 − exp(−S)`
는 유계·단조·밀도인식을 다 갖지만, (i) 애초에 **합산과 같은 오답**을 낸다(S 의 단조함수라
판정 순서가 합산과 동일), (ii) DRI 가 float32 라 **S≥18 에서 정확히 1.0 으로 포화**해
(float64 라도 S≥40) 밀집 지역의 순서가 뭉개진다. 실측 S 가 이미 61 이었다.
**단, 베이즈 자체는 다른 층에서 쓴다 — §7-3 참고 (시간축 필터링).**

### 7-3. 베이지안 시간축 필터는 DRI 층에 안 넣는다 — occupancy_grid 가 이미 함

초기 DRI 구상은 마지막 단계로 *"DRI 값을 선박 기준 ±10m rolling window 에 log-odds
로 bayesian update 한다 (순간적 센서 노이즈 상쇄)"* 를 뒀다. **이 층은 만들지 않는다.**
목적(노이즈 상쇄)은 타당하지만 **이미 한 층 위에서 달성돼 있고**, DRI 에 또 걸면 해롭다.

- **이미 있다**: `occupancy_grid.py` 가 정확히 그 구조다 — ±10m rolling window,
  log-odds 누적(`L_OCC=+0.85` / `L_FREE=−0.4`, `L_MIN/L_MAX` 클램프), sigmoid 발행.
  노이즈 상쇄는 거기서 끝난다. DRI 는 그 결과를 소비할 뿐.
- **DRI 는 필터링 대상이 아니다**: log-odds 갱신은 **정적인 양**을 추정할 때 성립한다.
  DRI 는 배 기준(`f`=heading 상대방위, `g`=거리)이라 **배가 돌거나 움직이면 정당하게
  변한다**. 그걸 시간 필터에 넣으면 "센서 노이즈"와 "배가 방금 선회함"을 구분 못 하고
  정당한 변화를 지연시킨다. 게다가 log-odds 는 확률(0~1)을 요구하는데 DRI 는 확률이 아니다.
- **"선박 기준" 격자도 함정**: 배와 함께 회전하는 격자에 값을 누적하면 매 프레임 보간
  오차가 쌓인다. `occupancy_grid.py` 가 굳이 **월드 축 정렬 + 정수 roll** 을 쓰는 이유가
  그것이다(그쪽 docstring 참고). 회전은 절대 하지 않는다.

**BFS decay 도 안 넣는다 — 이미 더 정확하게 하고 있다.** 초기 구상의 3번 항목
*"유효 셀에서 bfs decay 로 주변 셀 매핑, 여러 셀이 참조하면 최대값"* 은 **결합(max)과
중단조건(감쇠<임계=3σ 컷오프)이 지금 구현과 같고**, 거리 척도만 다르다: BFS 는 격자
그래프 거리(4-이웃=마름모, 8-이웃=팔각형 아티팩트), 지금은 **참 유클리드 거리(정확한 원)**.
BFS 의 유일한 실익은 장애물을 뚫지 않는 **측지 거리**인데, 부표 밭은 막힌 벽이 없어
유클리드와 같다. 벽·부두처럼 큰 폐색이 생기면 그때 재검토.

**대신 노이즈 손잡이는 `occ_threshold` 다.** 초기 구상의 1번 항목
*"관측점 개수가 임계 이상일 때만 유효 셀"* 은 여기에 그대로 대응한다 —
`occupancy_grid` 의 log-odds 특성상 발행값이 **히트 횟수의 함수**이기 때문:

| LiDAR 히트 | log-odds | 발행값 | |
|---|---|---|---|
| 1회 | 0.85 | **70** | `occ_threshold=50` 이면 **단발 노이즈도 즉시 장애물** |
| 2회 | 1.70 | **85** | `occ_threshold=75` → 2회 요구 (기본값으로 채택) |
| 3회 | 2.55 | **93** | `occ_threshold=90` → 3회 요구 |
| 4회+ | 3.40→3.5 | 97 | 포화 |

⚠️ 이전에 "발행값이 12/97 쌍봉이라 occ_threshold 는 20~80 어디 둬도 같은 둔감한
손잡이"라고 적었던 건 **틀렸다**. 그 관측은 **포화된 정상 상태**의 히스토그램이었고,
노이즈는 정상 상태가 아니라 **전이 구간(=70)** 에 산다. 정상 상태 셀이 이 값에 안
흔들리는 건 맞지만, 그래서 오히려 **순수한 노이즈 필터**로 쓸 수 있다는 뜻이다.

### 7-4. `build_dri` 를 격자 주기(1Hz) → tick 주기(10Hz) 로

**바꾼 것**: §1.4 의 `1) DRI 갱신 (grid msg 새로 왔을 때만 build_dri)` →
**매 tick** `build_dri(캐시된 최신 grid msg, 지금 pose)`.

**왜**: `DRI = f(격자, 배 pose)` 인데 초안은 **격자 주기에 배 pose 를 묶어버렸다.**
두 입력의 요구 주기가 다르다:
- **격자는 1Hz 로 충분** — 정적 부표 + **월드 앵커** 격자라 1초 전 격자와 지금 격자가
  **같은 정보**다. staleness 가 원리적으로 없다. (동적 장애물이면 달라지지만 경기장은 정적.)
- **배 pose 는 10Hz 필요** — `f`(heading 상대방위)·`g`(거리)가 배 pose 의 함수라
  배가 움직이면 **정당하게** 변한다.

**실측**(sim, §6 안전값 명령): `max_linear=0.20` → **1.48 m/s (초당 7.4셀)**,
`max_angular=0.05` → **초당 31.6°**(직진 중 요동은 56°/s 까지). ⚠️ `max_linear=0.20`
은 **m/s 가 아니다** — `twist2thrust` 가 `scale=60` 을 곱해 12N **추력**으로 바꾸므로
실제 속도는 항력이 정한다. 초당 수십 도를 도는데 1Hz 로 `f=cos(Δθ)` 를 갱신하면 완전히
딴 값을 쓴다. (정확한 수치는 배·튜닝에 따라 달라지지만 **자릿수**는 유지된다.)

**비용**: build_dri ≈ 2.6ms × 10Hz = **CPU 2.6%**. N1 이 요구하는 "같은 tick 안에서 동기"
안에 여유롭게 들어간다.
