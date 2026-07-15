#  장애물 회피 로컬 플래너 (DRI + B-spline) — 작업 계획

`obstacle_planner` behavior 를 단순 반발 조향에서 DRI 격자 + B-spline 로컬
플래너로 교체하는 작업 계획. 설계 논의에서 확정된 사항과, 작업을 나눠서
진행해도 충돌 없도록 **모듈 경계·인터페이스를 먼저 동결**한 문서.

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
점유 셀을 위험도 float 격자로 바꾼다. 점유 셀마다 **비등방(이중) 가우시안**으로
위험을 퍼뜨리되, 배와 **가까울수록** / 배 **heading 전방에 있을수록** 더 넓고
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

## 1. 모듈 구성과 인터페이스 (동결 — 여기 바꾸면 전원 합의)

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
    """OccupancyGrid msg → DRI. 점유 셀(값 ≥ occ_threshold)마다 비등방 가우시안:
       - 축: 배→장애물 방향 (σ_along) / 그 수직 (σ_cross)
       - 스케일: 거리 반비례(dist_gain) × 전방각 가중(heading_gain·cos 상대방위)
       - unknown(-1) 셀은 기여 0 (free 취급, R8)"""
```

`OccupancyGrid` msg 파싱은 `build_dri` 안에서만. 이후 전 모듈은 `DriGrid` 만 안다.

**DRI 수식 (작업 1 구현 스펙).** 배 위치 `b`, heading `ψ`. 점유 셀 `c` 하나가
격자점 `x` 에 기여하는 위험을 비등방 가우시안으로 정의한다.

- `d = c − b`, 거리 `ρ = ‖d‖`, 단위벡터 `d̂` (배→장애물 방향) 과 그 수직 `d̂⊥`.
- 전방성 `f = max(0, cos(∠(d, heading))) ∈ [0,1]` (장애물이 선수 정면이면 1, 옆이면 0).
- 근접성 `g(ρ) = 1 / (1 + ρ/ρ₀) ∈ (0,1]` (`ρ₀` = `dist_falloff`, 가까울수록 1).

퍼짐(σ)과 세기(A)를 근접·전방성으로 키운다 — **배와 가깝거나 정면일수록 더 넓고
강하게** (R1):

```
σ_along(c) = sigma_along · (1 + k_dist·g(ρ)) · (1 + k_head·f)   # 배→장애물 축으로 길게
σ_cross(c) = sigma_cross                                        # 수직축은 기준값 고정
A(c)       = (1 + a_dist·g(ρ)) · (1 + a_head·f)                 # 진폭 스케일

셀 c 가 x 에 주는 값 (c-로컬 좌표 u=(x−c)·d̂, w=(x−c)·d̂⊥):
  contrib(x; c) = A(c) · exp( −u²/(2σ_along²) − w²/(2σ_cross²) )

DRI(x) = Σ_c contrib(x; c)   # 모든 점유 셀 합산 (밀집 장애물밭 = 자연히 더 높은 위험)
```

- **합산(Σ) vs 최대(max)**: 합산 채택 — 위험장이 매끄러워 repair 의 "DRI 낮은 쪽"
  gradient 판정이 안정적이고, 장애물이 촘촘하면 위험이 쌓여 더 크게 우회한다.
- **구현**: 각 점유 셀마다 `3σ` 로 잘라낸 국소 패치(≈ σ/res 반경)에만 회전 가우시안을
  splat 해 더한다. 축이 셀마다 달라(각 셀의 `d̂`) 단일 분리 컨볼루션은 불가 —
  셀별 splat 이 맞고, 점유 셀 수십 개 × 소형 패치라 벡터화하면 수 ms.
- `sigma_along/cross`, `dist_falloff`, `k_dist/k_head`, `a_dist/a_head` 는 전부
  파라미터(§3). 게인을 0 으로 두면 등방·고정 가우시안으로 축약된다(디버그 기준선).

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
  1) DRI 갱신 (grid msg 새로 왔을 때만 build_dri)
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

## 2. 작업 분할 (독립 수행 가능하도록)

의존성은 **이 문서의 인터페이스뿐**이다. 서로의 구현을 기다릴 필요 없음.

```
[작업 1] dri.py ──────────────┐
[작업 2] bspline_planner.py ──┼──▶ [작업 5] 통합 + sim 튜닝
[작업 3] obstacle_planner.py ─┤
[작업 4] avoid_viz.py ────────┘
```

### 작업 1 — `dri.py` (복잡도 낮음)
- **입력물**: §1.1 인터페이스.
- **단독 테스트**: ROS 없이 — 합성 OccupancyGrid(점유 셀 1~3개 수제 배열)로
  (i) 피크가 장애물 셀 위, (ii) 배→장애물 축 방향으로 더 길게 퍼짐,
  (iii) 가까운/전방 장애물이 더 강함, (iv) unknown 기여 0 을 assert.
- **완료 기준**: 위 테스트 통과 + 100×100 격자 build_dri < 10ms.

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
| `dri.occ_threshold` | 50 | 이 점유확률 이상 셀만 위험원 |
| `dri.sigma_along / sigma_cross` | 1.2 / 0.6 m | 가우시안 축별 σ 기준값(스케일 전) |
| `dri.dist_falloff` (ρ₀) | 4.0 m | 근접성 `g(ρ)=1/(1+ρ/ρ₀)` 감쇠 스케일 |
| `dri.k_dist / k_head` | 튜닝 | σ_along 의 근접·전방성 확대 게인 |
| `dri.a_dist / a_head` | 튜닝 | 진폭 A 의 근접·전방성 확대 게인 |
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
