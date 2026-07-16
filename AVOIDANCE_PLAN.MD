# 장애물 회피 로컬 플래너 — 현황·인수인계

`obstacle_planner` behavior 를 단순 반발 조향에서 DRI 격자 + B-spline 로컬
플래너로 교체하는 작업의 **현재 상태 스냅샷**. 새 세션이 이어받을 때 읽는 문서다.

> **이 문서는 규정집이 아니다.** 세부 알고리즘의 원본(spec)은 코드와 테스트다
> (`kaboat_behaviors/dri.py`, `bspline_planner.py`, `test/`). 문서는 "무엇이
> 있고, 무엇이 정해졌고(+왜 한 줄), 무엇이 미결인지"만 담는다. 미결 사항은
> **착수 시점에 사용자와 확정**하고, 결과·이유는 코드 주석과 커밋 메시지에
> 남긴다 — 여기엔 요약 한 줄과 커밋 해시만 추가한다.
> (이전 버전은 세부를 미리 규정하는 계약서 형식이었고, 작업마다 뒤집혀서
> 변경 로그가 본문보다 무거워졌다 — git 히스토리 `ca7a0f4` 이전 참조.)

관련: [SKELETON.MD](SKELETON.MD) §4/§6. sim 검증 환경 운영 지식은 세션 메모리
`kaboat-sim-demo-launch` + 컨테이너 `/root/demo/*.sh`.

---

## 1. 전체 그림 (확정)

```
/occupancy_grid ─▶ ① DRI 생성 ─▶ ② 경로 생성 ─┬▶ ③ 추종+검사 ─▶ /cmd/avoid
     (최신 캐시)     (매 tick)        ▲          │       │
                                      │ 재생성    │ 수리   │ 위반
                                      └──────────┴───────┘
                                      (전진 반평면에 답 없음 → ⑤ ESCAPE)
```

- **① DRI** (`dri.py` ✅): 점유 셀 → 위험도 격자. 등방 가우시안, **max 결합**
  (최근접 장애물이 안전을 결정 — 유계 [0, maxA]), unknown = free. 배 pose 의
  함수라 **매 tick 재계산** (격자는 1Hz 캐시).
- **② 생성** (`bspline_planner.generate` ✅): 제어점 6개 — p0 배, p1 관성점,
  p2~p5 는 **‖p1‖↔r_end 균등 사다리 arc** 위 비용 argmin (arc 는 장애물과
  무관 — r1 은 재생성 트리거 전용) → clamped cubic B-spline. 후보는
  **waypoint 전진 반평면**(±90°)에만 — 뒤로 가는 우회는 ESCAPE 소관.
  최선 후보조차 hard_max 초과면 None.
- **③ 검사/수리** (`check`/`repair` ✅): 생성과 분리 — 추종 중 재생성 없이
  매 tick 최신 DRI 로 전방 샘플 검사. 위반 시 CP 방위 이동 수리(4CP×±Δθ
  8후보 전수, 전방 max DRI 엄격 감소만 수락).
- **④ 재생성** (작업 3): (a) 기준 장애물(r1) 통과 (b) 수리 실패 (c) 주기 백스톱.
- **⑤ ESCAPE** (작업 3): 후진 + 저DRI 조향. 후진 → surge↓ → p1 수축 →
  접선 구속이 풀림(자기교정).

**모듈 구조 (확정):**

```
kaboat_behaviors/
  dri.py              ✅ 순수 라이브러리 (msg → DriGrid)          test 17개
  bspline_planner.py  ✅ 순수 라이브러리 (generate/check/repair)  test 21개
  obstacle_planner.py ⬜ [작업 3] ROS 노드 — 상태기계 + follow_path + ESCAPE
  avoid_viz.py        ⬜ [작업 4] 디버그 발행 (RViz)
                      ⬜ [작업 5] 통합 + 5개 미션 회귀 튜닝
```

**불변 원칙** (바꾸려면 사용자와):
- 라이브러리는 ROS 무의존(순수 numpy) — 노드만 ROS 를 안다. 단독 pytest 가능.
- SKELETON §6 안전값 불변 (`max_linear=0.20` 등은 **추력**, 실측 1.48 m/s).
  조향 여유가 없으므로 **감속이 회피 성능의 절반**.
- cmd_mux 300ms 워치독 — 재생성 중에도 발행 유지 (동기 수 ms 라 문제없음).
- 좌표는 전부 odom frame. DRI 격자는 occupancy_grid 와 셀 1:1.
- 배의 진짜 운동 구속은 **yaw**(실측 ~31°/s) — 전후는 역추력 제동 자유,
  sway 는 무액추에이션. p1 규칙(§7-7)의 근거.

## 2. 현재 상태

| 항목 | 상태 |
|---|---|
| `dri.py` | 완료(97e7080~). build_dri 실측 2.6ms/100×100 |
| `bspline_planner.py` | 완료(8f449fb~873db0c). generate 0.27ms, repair 0.70ms |
| 테스트 | 38개 — 호스트(scipy 1.11)·컨테이너(scipy 1.8) 통과 |
| 생성기 sim 평가 | 완료 — 부표 밭(3m 격자) 관통 확인, 파라미터 6개 실측 확정 |
| sim 검증 스택 | 컨테이너 `/root/demo/`: `sim_avoid.sh`(회피 스폰 2.0,76.5),
  `goto_point.py`, occupancy_grid/rviz 스크립트. 부표 밭 = 월드 x 10~25 / y 72~81 |

인터페이스 요약 (원본은 코드 docstring):
```python
build_dri(grid_msg, boat_xy, yaw, DriParams) -> DriGrid   # .data/.anchor/.occ_xy
generate(dri, boat_xy, yaw, vel_xy, wp_xy, PlannerParams, now_t=0) -> Plan | None
check(plan, dri, boat_xy, p) -> int | None    # 첫 위반 샘플 idx, progress 갱신
repair(plan, dri, violation_idx, p) -> bool   # CP 방위 이동, 실패 시 False
```

## 3. 파라미터 전체 표 (튜닝 참조용 — 원본은 코드 기본값)

상태: ✅ sim 실측 확정 / 📝 초안(미검증) / 🔴 미결

| 파라미터 | 값 | 상태 | 의미 / 근거 한 줄 |
|---|---|---|---|
| `dri.occ_threshold` | 50 | ✅ | 점유 판정 하한 = "히트 몇 번부터 믿나". 원거리 부표는 셀당 1히트(발행 70)에 머물러 75면 투명해짐. 실선 노이즈 보고 재검토 |
| `dri.sigma` | 0.6 m | ✅ | 등방 σ 기준 = berth 폭. 1.0 은 3m 틈을 전부 "막힘"으로 만듦 |
| `dri.dist_falloff` | 4.0 m | 📝 | 근접성 g(ρ)=1/(1+ρ/ρ₀) 의 ρ₀ |
| `dri.k_dist / k_head` | 0.3 / 0.3 | 📝 | σ 근접·전방 확대 게인 |
| `dri.a_dist` | 1.0 | 📝 | 진폭 근접 게인 |
| `dri.a_head` | 0.3 | 🔴 | 진폭 전방 게인. 0.3 확정은 배가 반대를 보던 **오염된 관찰** — R1 표현력·f_dri 예고 감속을 위해 0.7 재검증 중이었음 (중단됨) |
| `planner.t_look` | 1.0 s | ✅ | p1 = surge·t_look. 5.0 은 §6 오독(첨점 재현으로 기각) |
| `planner.p1_min_dist` | 1.0 m | 📝 | 정지·후진 시 접선 방향용 하한 |
| `planner.bearing_max` | 100° | 📝 | r1 후보 후방 하드컷 |
| `planner.k_bear` | 1.5 | ✅ | r1 방위 가중 — 0.5 는 지나친 측면 부표가 r1 독점 |
| `planner.boundary_margin` | 1.0 m | 🔴 | r_end = 격자변 최소거리 − 이 값. 0.5 로 줄이면 r_end +0.5 (완충 절반) |
| `planner.min_horizon` | 3.0 m | 📝 | waypoint 코앞이어도 r_end 하한 |
| `planner.cp_min_gap` | 0.5 m | 📝 | r_end 퇴화 방지: r_end ≥ ‖p1‖+2·이 값 |
| `planner.search_fov` | 180° | ✅ | 후보 부채꼴 = waypoint 전진 반평면 (뒤는 ESCAPE 소관) |
| `planner.n_candidates` | 31 | 📝 | arc 당 후보 수 (= 6° 양자화 — w_prev 와 결합) |
| `planner.w_dri / w_wp / w_prev` | 1.0 / 1.0 / 0.5 | ✅ | 후보 비용. w_wp 0.3 은 관통이 우회에 짐, w_prev 0.5 는 6° 양자화 S 잔물결 억제 |
| `planner.safe_threshold` | 1.0 | 📝 | check 위반 기준. berth 결합: 거품 반경 = σ√(2ln(A/thr)) |
| `planner.hard_max` | 3.0 | 📝 | generate 포기(None→ESCAPE) 기준 |
| `planner.repair_max / repair_dtheta` | 3 / 6° | 📝 | tick 당 수리 한도 / 1회 이동각 |
| `planner.progress_window` | 2.0 m | 📝 | check 진행 갱신 전방 창 |
| `follow.lookahead / k_curv` | 1.5 m / — | ⬜ | 작업 3. 곡률 실측: 정상 회피 κ≈0.9~1.2, 전속 가용 0.37 → 감속 필수 |
| `regen.margin / period` | 1.0 m / 4.0 s | ⬜ | 작업 3. 트리거 (a) 여유 / (c) 백스톱 |
| `escape.reverse_speed` | 0.05 | ⬜ | 작업 3. §6 전복 주의 — sim 역추력 검증 후 |

## 4. 미결 + 다음 작업

**미결 (착수 전 사용자와 확정):**

1. **a_head 0.3 vs 0.7** — 재검증 절차: 배가 밭을 정면으로 본 상태에서
   관통 유지 + 정면/측면 경사 확인 (오염 없이, 한 번에 한 변수).
2. **boundary_margin 1.0 vs 0.5**.

**리스크 (채택된 설계의 전제 조건):**
- CP 사다리(균등 4분할)는 "사다리 간격 < DRI 거품 유효 반경"일 때만 모든
  장애물이 어떤 arc 에든 느껴진다 — 지금 간격 ≈0.9m 로 성립. **격자를
  키우면(r_end↑) 깨질 수 있음** → arc 개수(고정 4) 재검토.
- generate(greedy)+repair(국소)는 위험 능선 너머 다른 basin 으로 못 넘어간다
  (게이트+측방 장면 교착 실측, 8f449fb) — 에스컬레이션(수리 실패→재생성→
  ESCAPE)은 노드(작업 3) 소관. sim 에서 잦으면 세그먼트 비용 검토.

**다음 = 작업 3 `obstacle_planner.py`** — 착수 시 정할 것들 (답 미리 안 정함):
- 상태기계: tick 당 generate 횟수 제한, "초입 위반 → ESCAPE" 판정 기준,
  ESCAPE 진입·탈출 조건.
- follow_path: lookahead/감속 공식 상수, waypoint 도착 판정과 behavior 전환.
- 개발 순서: 직선 Plan stub → sim 추종 안정성(발진·전복 없음) → 실제 planner 연결.

그다음: 작업 4(avoid_viz — gen_viz 스크래치가 원형), 작업 5(통합: 5개 미션
회귀, 접촉 없이 완주 ×3).

## 5. 함정·교훈 (새 세션이 반복하지 말 것)

- **그려보기 전엔 모른다** — 초안 설계 오류(비등방 V자, 합산 차단, t_look 첨점,
  arc 뭉침)는 전부 그려본 뒤에야 드러났다. 수식을 확정하기 전에 스크래치
  matplotlib/RViz 로 실제 장면을 그려라.
- **상수는 실측에서** — t_look=5.0 사건: `max_linear=0.20` 을 m/s 로 오독
  (실제 1.48 m/s, §6). SKELETON §6 실측값을 먼저 확인.
- **한 번에 한 변수** — a_head 0.3 은 배 heading 이 뒤집힌 상태에서 내린
  오염된 결정이었다. 관찰 조건(배 방향, 속도)을 로그로 확인하고 바꿀 것.
- **알고리즘·설계 확정은 사용자와** — 혼자 정해 문서에 박지 말 것.
- sim 운영 함정(FastDDS SHM, RTF 0.25, pkill 자기매칭 등)은 메모리
  `kaboat-sim-demo-launch` 참조.

## 6. 변경 로그 (한 줄 + 커밋 — 상세는 커밋 메시지/git 히스토리)

- be67465~3f77c68 초안 작성기 (계약서 형식 — 폐기됨)
- 97e7080 작업 1 dri.py: 등방(비등방 V자 기각)·max 결합(합산 기각)·10Hz 재계산
- ca7a0f4 착수 전 기하 확정: t_look 1.0s, r_first 클램프, r_end 격자유도,
  fov=전진 반평면
- 8f449fb 작업 2 bspline_planner.py: repair 8후보 전수(최근접 교착 실측),
  threshold 1.0, DriGrid.occ_xy 추가, greedy 능선 한계 확인
- 250f155 생성기 sim 튜닝: sigma 0.6·w_wp 1.0·a_head 0.3·w_prev 0.5·
  k_bear 1.5·occ_threshold 50
- 873db0c p1 을 v̂ → heading+surge 로 (v_eps 삭제)
- 12f94d4 문서를 계약서 → 현황 스냅샷 형식으로 전면 재작성
- f0201a5~5709971 occupancy_grid 발행 10Hz + 스캔 dedup (별도 작업 — occ_threshold
  의미에 영향 가능, a_head 재검증 때 같이 볼 것)
- (이 커밋) **CP 사다리를 r1 에서 분리** — radii 항상 ‖p1‖↔r_end 균등 4분할,
  r1 은 재생성 트리거 (a) 전용. 지평선 장애물 때 arc 뭉침([7.9,8.2,8.6,8.9] →
  경로가 밭 입구에서 끝남) 실측이 동기. r_first 클램프·horizon-out 특례 삭제
