# 장애물 회피 로컬 플래너 — 현황·인수인계

`obstacle_planner` behavior 를 단순 반발 조향에서 DRI 격자 + B-spline 로컬
플래너로 교체하는 작업의 **현재 상태 스냅샷**. 새 세션이 이어받을 때 읽는 문서다.

> **이 문서는 규정집이 아니다.** 세부 알고리즘의 원본(spec)은 코드와 테스트다
> (`kaboat_behaviors/dri.py`, `bspline_planner.py`, `test/`). 문서는 "무엇이
> 있고, 무엇이 정해졌고(+왜 한 줄), 무엇이 미결인지"만 담는다. 미결 사항은
> **착수 시점에 사용자와 확정**하고, 결과·이유는 코드 주석과 커밋 메시지에
> 남긴다 — 여기엔 요약 한 줄과 커밋 해시만 추가한다.
> (이전 버전은 세부를 미리 규정하는 계약서 형식이었고, 작업마다 뒤집혀서
> 변경 로그가 본문보다 무거워졌다 — git 히스토리 `0e5e205` 이전 참조.)

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
  안전은 **하드 필터**(직전 CP→후보 선분 max DRI > threshold 면 제거),
  선호는 방위 비용만 — 거품(σ)이 이미 berth 를 인코딩하므로 비용에 DRI 를
  또 넣으면 이중 계산(CP 가 틈 대신 DRI=0 지대로 도망감, sim 실측).
  어떤 arc 든 생존자 0 이면 None.
- **③ 검사/수리** (`check`/`repair` ✅): 생성과 분리 — 추종 중 재생성 없이
  매 tick 최신 DRI 로 전방 샘플 검사. 위반 시 CP 방위 이동 수리(4CP×±Δθ
  8후보 전수, 전방 max DRI 엄격 감소만 수락).
- **④ 재생성** (작업 3): (a) 기준 장애물(r1) 통과 (b) 수리 실패 (c) 주기 백스톱.
- **⑤ ESCAPE** (작업 3): 후진 + 저DRI 조향. 후진 → surge↓ → p1 수축 →
  접선 구속이 풀림(자기교정).

**모듈 구조 (확정):**

```
kaboat_behaviors/
  dri.py              ✅ 순수 라이브러리 — 위험 표현 (msg → DriGrid)   test 17개
  bspline_planner.py  ✅ 순수 라이브러리 — 경로 기하 (generate/check/repair)
                         [작업 3 소폭 추가: curvature_ahead, r1 계산 공개화]
  avoid_fsm.py        ⬜ [작업 3] 순수 라이브러리 — 조종 정책: 상태기계
                         (FOLLOW/ESCAPE 사다리) + follow_cmd + escape_cmd.
                         rclpy 무의존(YAW 게인은 노드가 주입). 분리 근거는
                         전이·타이머 로직의 검증 루프 — 노드 메서드면 Node 가
                         rclpy.init 없이 생성 불가라 가짜 시계·합성 장면
                         루프(작업 1·2 의 설계 오류를 전부 잡은 방식)에서
                         제외된다. 실물에선 같은 분리가 rosbag tick 재생
                         디버깅·현장 회귀 테스트가 된다 (2026-07-17 확정.
                         ⚠️ 첫 확정 때 근거로 쓴 "호스트에 rclpy 없음"은
                         오류 — 호스트에 Jazzy 있음. 재검토 후 위 근거로 재확정)
  obstacle_planner.py ⬜ [작업 3] ROS 노드 — 구독/발행/파라미터/로깅 배선만
  avoid_viz.py        ⬜ [작업 4] 디버그 발행 (RViz)
                      ⬜ [작업 5] 통합 + 5개 미션 회귀 튜닝
```


## 2. 현재 상태

| 항목 | 상태 |
|---|---|
| `dri.py` | 완료(c0d1ef5~). build_dri 실측 2.6ms/100×100 |
| `bspline_planner.py` | 완료(0e5e205~0e5e205). generate 0.27ms, repair 0.70ms |
| 테스트 | 41개 — 호스트(scipy 1.11)·컨테이너(scipy 1.8) 통과 |
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
| `dri.sigma` | 0.6 m | ✅ | 등방 σ **상한**(코앞·정면) = berth 폭. 1.0 은 3m 틈을 전부 "막힘"으로 만듦 |
| `dri.dist_falloff` | 4.0 m | 📝 | 근접성 g(ρ)=1/(1+ρ/ρ₀) 의 ρ₀ |
| `dri.k_dist / k_head` | 0.3 / 0.3 | ✅ | σ 게인 = **감쇄 폭**(최대 정규화, 07-17): 코앞·정면 = sigma, 멀고 옆일수록 ÷(1+k)². 위로 확대하는 구식은 3m 틈(히트 초승달 감안 유효 2.4m)을 물리 폐쇄 — pinch 1.12 실측 |
| `dri.a_dist` | 1.0 | 📝 | 진폭 근접 게인. 유효 반경 = σ√(2ln(A/thr)) 라 진폭 경사만으로 반경 경사 발생 (R1) |
| `dri.a_head` | 0.7 | ✅ | 진폭 전방 게인. 0.3(1.3배)은 경사가 안 보임 + 오염된 관찰의 결정 — 0.7 은 σ 정규화 후 통로 개방과 양립 실측 |
| `planner.t_look` | 1.0 s | ✅ | p1 = surge·t_look. 5.0 은 §6 오독(첨점 재현으로 기각) |
| `planner.p1_min_dist` | 1.0 m | 📝 | 정지·후진 시 접선 방향용 하한 |
| `planner.bearing_max` | 100° | 📝 | r1 후보 후방 하드컷 |
| `planner.k_bear` | 1.5 | ✅ | r1 방위 가중 — 0.5 는 지나친 측면 부표가 r1 독점 |
| `planner.boundary_margin` | 1.0 m | 🔴 | r_end = 격자변 최소거리 − 이 값. 0.5 로 줄이면 r_end +0.5 (완충 절반) |
| `planner.min_horizon` | 3.0 m | 📝 | waypoint 코앞이어도 r_end 하한 |
| `planner.cp_min_gap` | 0.5 m | 📝 | r_end 퇴화 방지: r_end ≥ ‖p1‖+2·이 값 |
| `planner.search_fov` | 180° | ✅ | 후보 부채꼴 = waypoint 전진 반평면 (뒤는 ESCAPE 소관) |
| `planner.n_candidates` | 31 | 📝 | arc 당 후보 수 (= 6° 양자화 — w_prev 와 결합) |
| `planner.w_wp / w_prev` | 1.0 / 0.5 | ✅ | 후보 비용 = 방위 항 둘뿐 (DRI 항 없음 — 하드 필터로 이관). w_prev 는 6° 양자화 S 잔물결 억제. sim 하네스는 w_prev 0.8 시험 중 |
| `planner.safe_threshold` | 1.0 | ✅ | **유일한 안전 기준선** — 생성(선분 하드 필터)·검사(check 위반) 공용. 거품 반경 = σ√(2ln(A/thr)). 구 w_dri(소프트 벌점)·hard_max(포기선) 삭제 |
| `planner.gen_margin` | 0.85 | 📝 | 생성 필터는 thr×이 값 에서 자름 — 현(chord) 판정 vs 실제 스플라인의 수 cm 편차 흡수 (1.0 이면 경계 스침 1.013 실측) |
| `planner.repair_max / repair_dtheta` | 3 / 6° | 📝 | tick 당 수리 한도 / 1회 이동각 |
| `planner.progress_window` | 2.0 m | 📝 | check 진행 갱신 전방 창 |

작업 3(노드)의 파라미터(추종·재생성·ESCAPE 관련)는 **노드가 생기면 그때
정하고 여기 추가** — 미리 이름·기본값을 지어두지 않는다. 노드 설계에 쓸
실측 하나만 기록: 정상 회피 경로 κ ≈ 0.9~1.2 vs 전속 가용 κ ≈ 0.37 —
조향만으론 못 따라가므로 감속이 필수라는 근거.

## 4. 미결 + 다음 작업

**미결 (착수 전 사용자와 확정):**

1. **boundary_margin 1.0 vs 0.5** — sim 하네스는 0.5 시험 중 (전속 주행 시
   격자 경계 가짜 위반 없는지가 판정 기준). t_look 0.7·p1_min 0.5·w_prev 0.8
   도 하네스에서 시험 중 — 확정 시 코드 기본값 승격.

**리스크 (채택된 설계의 전제 조건):**
- CP 사다리(균등 4분할)는 "사다리 간격 < DRI 거품 유효 반경"일 때만 모든
  장애물이 어떤 arc 에든 느껴진다 — 지금 간격 ≈0.9m 로 성립. **격자를
  키우면(r_end↑) 깨질 수 있음** → arc 개수(고정 4) 재검토.
- generate(greedy)+repair(국소)는 위험 능선 너머 다른 basin 으로 못 넘어간다
  (게이트+측방 장면 교착 실측, 0e5e205) — 에스컬레이션(수리 실패→재생성→
  ESCAPE)은 노드(작업 3) 소관. sim 에서 잦으면 세그먼트 비용 검토.

**다음 = 작업 3 `obstacle_planner.py`** — 상태기계는 확정(2026-07-17 사용자
합의), 나머지는 착수 시:

```
상태 2개 — FOLLOW / ESCAPE. 모든 상태에서 매 tick cmd 발행 (cmd_mux 워치독).

FOLLOW (10Hz): DRI 재계산 → check(plan)
  ├─ plan 없음(활성화 직후 포함) → 정지 cmd + generate 시도,
  │    재시도 시간 상한 초과 → ESCAPE
  ├─ 통과 → 재생성 트리거 점검 (generate 는 tick 당 ≤1회):
  │    (a)  배 진행 반경 > 저장 r1        — 기준 장애물 통과
  │    (a') 현재 r1 < 저장 r1 − δ        — 기준 장애물 교체 (값 비교 한 줄)
  │    (c)  마지막 생성 후 백스톱 주기 경과
  │    → generate: 성공 시 교체, None 이면 기존 plan 유지(아직 check 통과 중)
  └─ 위반(idx) → repair(≤repair_max) → 실패 시 즉시 generate
       └─ None → 위반까지 거리 ≥ d_panic: 감속 + 다음 tick 재시도
                  거리 < d_panic 또는 누적 재시도 시간 초과: ESCAPE

ESCAPE: 후진 + 저DRI 조향, 매 tick generate 재시도
        → 최소 이탈 시간 경과 + N tick 연속 성공 시 FOLLOW 복귀
```

- 설계 근거: "수리 실패 횟수 세기"는 불필요 — 하드 필터라 generate 는
  성공=안전 / 실패=None 뿐이므로, 세야 할 것은 실패 횟수가 아니라 **위반까지
  남은 여유**(거리+시간상한 혼합). 기준 장애물 교체는 더 가깝거나 정면인 위협의
  등장이 반드시 r1 급락으로 나타나므로 셀 identity 추적 없이 비교 한 줄이면
  된다(반대 방향 교체는 (a)·(c)가 잡음 — 안전은 어차피 check 가 매 tick 보장).
  ESCAPE 즉시 복귀는 후진→p1 수축→generate 재성공→전진→재막힘 채터링 위험이
  있어 최소 이탈+연속 성공으로 이중 차단. **DRI threshold 완화 단은 v1 제외**
  — 지속 실패의 답은 ESCAPE 로 일원화, sim 교착 빈도 실측 후 재검토.
- **follow_path 확정(2026-07-17 사용자 합의): PD 조향 + 곡률 feedforward 감속.**
  조향은 기존 `seek_goal()` PD(YAW_KP/KD — §6 실측 안전 게인) 재사용, 목표점만
  경로 위 lookahead 점으로. 감속은 전방 창 max |κ(s)| (B-spline 도함수, 해석적)
  로 `v_cmd = clamp(ω_max/κ_max, v_min, v_max)` — 근거: κ_avail = ω/v 이고
  실측 ω_max 0.52 rad/s ÷ 전속 1.48 m/s = 0.35 ≈ 기록된 전속 가용 κ 0.37.
  PD 만으로는 안 됨(오차 발생 후 반응 + 전속에서 각속도 물리 부족 = §6 전복
  패턴). heading 오차 비례 감속 항은 v1 제외(한 번에 한 변수). v_cmd→추력
  변환은 비선형(0.20≈1.5 m/s, 중간 미실측) — 초기 비례 근사 후 sim 튜닝.
- d_panic·재시도 상한·백스톱 주기·δ·최소 이탈/연속 성공·lookahead 거리·전방
  창 폭·v_min 등 상수는 노드 구현 시 sim 실측으로 정하고 §3 표에 추가(§3 방침 그대로).
- **ESCAPE 조향 확정(2026-07-17 사용자 합의): 후방 우선 + 포위 시 전방위.**
  후방 반평면 방위 후보를 배→후보점 **선분 max DRI argmin** 으로 골라 후진
  조향(생성기의 선분 평가 재사용, 반경은 2~3m 급 단거리) — "무작정 후진"이
  아니라 저DRI 필드를 따라가되 탐색을 후방으로 제한하는 이유: ① 전방은 이미
  더 강한 시험(스플라인 완주+하드 필터)에서 탈락했는데 선분(단거리) 시험만
  통과하는 전방 주머니(입구만 깨끗한 dead-end)로 전진하면 갇힘 심화, ② 후방
  = 방금 지나온 물 = 실측 보증 free, ③ ESCAPE 의 목적은 항법이 아니라
  generate 전제 리셋(surge↓→p1 수축→접선 구속 해제). 후방 후보 **전원
  threshold 초과(포위)일 때만** 전방위로 확장해 전역 최소-위험 방향 선택.
  threshold 의 역할은 이 탐색 범위 스위치가 전부 — **방향 선정은 어느 범위에서든
  무조건 argmin, 후보를 기준치로 거르지 않는다**(ESCAPE 는 사다리 최후 단이라
  실패 불허: 하드 필터로 생존자 0 이 되면 낼 명령이 없고 받아줄 다음 단도 없음.
  하드 필터는 실패를 ESCAPE 가 받아주는 generate 만의 특권).
  ⚠️ 착수 시 확인: `a_head` 는 후방 장애물을 의도적으로 연하게 그림 —
  ESCAPE 에선 후방이 진행 방향이라 argmin 이 정후방으로 기울 수 있음
  (sim 후방 장애물 장면으로 검증).
- **waypoint 도착 확정(동일): planner 자체 판정 안 만듦 + lookahead 캡.**
  전환은 mission_manager(1m) 전담 — 이중 판정으로 경계값 불일치 만들지 않음.
  planner 는 waypoint 근접 시(진행 반경 > waypoint 거리) lookahead 점을
  waypoint 자체로 캡 — r_end 가 min_horizon(3m)으로 클램프돼 경로가
  waypoint 를 지나쳐 뻗는 문제만 차단. **이로써 작업 3 알고리즘 설계 완결** —
  남은 것은 상수(sim 실측)와 구현뿐.
- 개발 순서 (2026-07-17 세분화 — 각 단계는 sim 으로 판정 가능한 상태로 종료):
  1. **follow_path + 직선 stub**: `follow_cmd`(lookahead+캡·PD·곡률 감속) +
     `curvature_ahead`. stub = 장애물 없을 때 generate 의 직선 스플라인 그대로.
     판정: avoid 미션 완주, 발진·전복·워치독 걸림 없음. lookahead·v_min 실측.
  2. **FOLLOW 사다리**: `AvoidFsm.step` — check→repair→generate→긴급도 분기,
     트리거 (a)(a')(c), plan 없음 처리. 합성 장면 단위 테스트.
  3. **ESCAPE**: `escape_cmd`(후방 argmin·포위 확장) + 진입/탈출 전이.
     후방 주머니·완전 포위 장면 테스트 + a_head 후방 연화 편향 확인.
  4. **노드 통합 + sim 튜닝**: obstacle_planner.py 재작성(얇은 래퍼), 부표 밭
     관통(접촉 0). 상수 확정(d_panic·t_retry·t_backstop·δ·ESCAPE·lookahead
     계열)해 §3 표에 추가 + §4 미결 1(boundary_margin 등 하네스 시험분) 판정.
- ⚠️ 구현 시 주의: 기존 cmd_mux 는 300ms 동안 발행이 없으면 그 behavior 를
  무시한다(워치독) — 재생성 tick 에도 cmd 발행이 끊기면 안 됨. 속도 명령은
  SKELETON §6 안전값(전복 실측) 범위 안에서.

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

- 9499159~0e5e205 초안 작성기 (계약서 형식 — 폐기됨)
- c0d1ef5 작업 1 dri.py: 등방(비등방 V자 기각)·max 결합(합산 기각)·10Hz 재계산
- 0e5e205 착수 전 기하 확정: t_look 1.0s, r_first 클램프, r_end 격자유도,
  fov=전진 반평면
- 0e5e205 작업 2 bspline_planner.py: repair 8후보 전수(최근접 교착 실측),
  threshold 1.0, DriGrid.occ_xy 추가, greedy 능선 한계 확인
- 0e5e205 생성기 sim 튜닝: sigma 0.6·w_wp 1.0·a_head 0.3·w_prev 0.5·
  k_bear 1.5·occ_threshold 50
- 0e5e205 p1 을 v̂ → heading+surge 로 (v_eps 삭제)
- 0e5e205 문서를 계약서 → 현황 스냅샷 형식으로 전면 재작성
- b9f135e occupancy_grid 발행 10Hz + 스캔 dedup (별도 작업 — occ_threshold
  의미에 영향 가능, a_head 재검증 때 같이 볼 것)
- a22d0dc **CP 사다리를 r1 에서 분리** — radii 항상 ‖p1‖↔r_end 균등 4분할,
  r1 은 재생성 트리거 (a) 전용. 지평선 장애물 때 arc 뭉침([7.9,8.2,8.6,8.9] →
  경로가 밭 입구에서 끝남) 실측이 동기. r_first 클램프·horizon-out 특례 삭제
- (이 커밋) **작업 3 구현 계획 확정** (사용자 합의) — `avoid_fsm.py` 순수 모듈
  신설(조종 정책: 상태기계+follow_cmd+escape_cmd, 노드는 배선만) + 개발 순서
  4단계 세분화(직선 stub → FOLLOW 사다리 → ESCAPE → 노드 통합·상수 튜닝).
  분리 근거는 §1 모듈 구조 주석 참조 — 첫 확정의 "호스트에 rclpy 없음"은
  사실 오류(호스트 Jazzy 존재)로 판명, 사용자 재질문으로 원점 재검토 후
  전이 로직의 가짜 시계 테스트 루프 + 실물 bag 재생을 근거로 재확정
- 19ac8d4 **작업 3 알고리즘 설계 확정** (사용자 합의) — ① 상태기계
  FOLLOW/ESCAPE 2상태: 재생성 트리거 (a) r1 통과/(a') r1 급락=교체/(c) 백스톱,
  긴급도=거리+시간상한, ESCAPE 탈출=최소 이탈+연속 성공, threshold 완화 단은
  v1 제외 ② follow_path: PD 조향(seek_goal 재사용)+곡률 feedforward 감속
  v=ω_max/κ(실측 0.37=0.52/1.48 로 검증) ③ ESCAPE 조향: 후방 반평면 선분
  max DRI argmin 후진, 포위 시만 전방위(전방위 상시 argmin 은 주머니 함정으로
  기각) — 방향 선정은 무조건 argmin, threshold 는 범위 스위치 전용
  ④ waypoint: 자체 판정 없이 lookahead 캡(전환은 mission_manager 전담)
- 49dd341 **σ 게인을 확대 → 최대 정규화 감쇄로** (사용자 방향 지시) — 코앞·
  정면 = sigma 상한 고정, 멀고 옆일수록 축소. 구식(위로 확대)은 정면 중거리
  σ 를 0.88m 로 불려 3m 격자 틈을 물리적으로 닫았음(pinch 1.12 실측 — 필터가
  아니라 DRI 층에서 이미 폐쇄). σ 게인은 amp 게인과 중복이기도 했다: 유효
  반경 = σ√(2ln(A/thr)) 라 진하기 경사만으로 반경 경사가 생긴다. a_head 0.7
  복원(0.3 은 오염된 관찰의 결정). 밭 한복판 실측: 4 arc 전부 통로 관통 선택
- 966e8cf **후보 선택을 소프트 DRI 비용 → 하드 필터로** (사용자 설계) —
  직전 CP→후보 **선분** max DRI > threshold 면 제거, 생존자 중 방위 비용만
  argmin. 동기: threshold 이하 꼬리 위험도 벌점으로 작동해 CP 가 틈 대신
  DRI=0 지대로 도망감(p4 튕김 실측) = 거품 크기와 비용의 안전 이중 계산.
  w_dri·hard_max 삭제, threshold 가 생성·검사 공용 유일 안전선. gen_margin
  0.85 신설(chord↔스플라인 편차 흡수). 점 필터만으론 CP 사이 스플라인이
  거품을 가로지름(1.55 실측) → 선분 필터로 확장
