# 종합임무 end-to-end 시뮬 (Gazebo 없이, 실제 런치 그대로)

실제 `autonomy.launch.py`(run_autonomy 인자) + `occupancy_grid`(run_map) + `thruster_driver`(dummy)를
그대로 띄우고, 센서·모터 자리만 `sim_node.py` 가 대신한다. 실시간(wall clock)으로 돈다 — 한 판 최대 16분.

```
/cmd_vel → 실제 ThrusterMixer(슬루·불감대·워치독 0.3 s) → 좌우 PWM → Nomoto 배 모델 → /odom·TF
arena.json → /scan, /scan/level (가상 라이다), /camera/color/image_raw (부표 색 사각형), /imu/data, /gps/fix
도킹 표지 → /dock/detection_status, /dock/markers/<도형>/ray  (YOLO 출력만 흉내 — 노트북에 ultralytics 없음)
```

## 돌리기

```bash
# 한 판 (이름, ROS_DOMAIN_ID, seed, 시나리오 json, [런치 추가 인자…])
bash scripts/e2e_sim/run_one.sh n1 70 1 scripts/e2e_sim/scen/nominal.json
# 여러 판 동시에 (판당 CPU 약 1.7 코어). 절전에 들어가면 시계가 튀어 판이 무효 → 억제하고 돌린다
gnome-session-inhibit --inhibit idle:suspend bash scripts/e2e_sim/batch.sh 70 n1:nominal:1 n2:cur_ssw:2
python3 scripts/e2e_sim/analyze.py n1 n2        # 정답 채점 + path.png
python3 scripts/e2e_sim/ev.py n1 0 900          # 미션 매니저·도킹·탐색·접촉 시간표
```

경기장·배 모델 고르기 (기본은 11-13 경기장 `arena.json` + `nomoto.json`):

```bash
# 2026-11-14 실주행(run1_20261114_091623) 배치 — 탐색 목표 black·도킹 목표 triangle·실제 출발 자세
KABOAT_E2E_ARENA=$PWD/scripts/e2e_sim/arena_20261114.json bash scripts/e2e_sim/run_one.sh t1 70 1 scripts/e2e_sim/scen/nominal.json
# 예전 3 개 bag 맞춤 배로 비교
KABOAT_E2E_NOMOTO=$PWD/scripts/e2e_sim/nomoto_20261113.json bash scripts/e2e_sim/run_one.sh o1 71 1 scripts/e2e_sim/scen/nominal.json
```

탐색 목표 색·도킹 목표 도형은 경기장 파일의 `targets`(없으면 red·circle)로 정해 런치에 넘기고, 판 폴더에 쓴 경기장·배
모델(`arena.json`·`nomoto.json`)을 복사해 `analyze.py` 가 그 경기장·목표로 채점한다. 시나리오 경로는 상대 경로도 된다.

결과는 `$KABOAT_E2E_OUT`(기본 `/tmp/kaboat_e2e_runs`)`/<이름>/` — `trace.jsonl`(5 Hz 정답 자세·PWM·명령·state),
`events.jsonl`(state·상태 토픽·미션 매니저 로그·접촉), `launch.log`, `summary.json`, `path.png`.

- `run_one.sh` 는 `HOME` 을 판 폴더로 바꿔 `~/kaboat_mission_points.json`·`~/kaboat_geo_origin.yaml` 이 시뮬
  것을 가리키게 한다 (실제 홈 파일은 안 건드림, 런치 인자는 실물 기본값 그대로).
- 노트북 `install/kaboat_hardware` 가 옛 복사본이면 옛 도킹 코드가 돈다 — `run_one.sh` 가
  `PYTHONPATH` 앞에 `src/kaboat_hardware` 를 둔다.
- 런치 인자 `search_target_color`·`dock_target_class` 는 경기장 파일 `targets` (없으면 red·circle), `dock_device:=cpu`.

## 배 모델 (`nomoto.json`, `calib/fit_nomoto_run14.py`)

**2026-11-14 야외 전체 실행(run1_20261114_091623) 으로 다시 맞춤** — 자동 미션 구간(gate·station·search·avoid 와
그 접근, 도킹은 슬롯 벽 접촉이라 뺌)의 `/cmd_vel` → 실제 믹서로 좌우 PWM 재현 → `/odom` 속도. 후진 × 0.755 는
오늘 자료가 없어 11-13 도킹 bag 값 그대로. 예전 계수(11-12·11-13 bag, `nomoto_20261113.json`)와 비교:

| RMS 전진 m/s / 선회 rad/s | gate | station | search | avoid | 11-13 dock1 | 11-13 dock2 |
|---|---|---|---|---|---|---|
| 예전 (`nomoto_20261113.json`) | 0.11 / 0.06 | 0.15 / 0.06 | 0.11 / 0.09 | 0.13 / 0.07 | 0.21 / 0.10 | 0.09 / 0.06 |
| 지금 (`nomoto.json`) | 0.05 / 0.05 | 0.09 / 0.05 | 0.10 / 0.04 | 0.08 / 0.07 | 0.22 / 0.11 | 0.12 / 0.07 |

- 선회 Nomoto 1차: `T·ṙ + r = K(u)·sgn(δ)|δ|^q`, `K(u) = K0 + K1·|u|` — T 0.59 s, K0 2.62, K1 −1.54, q 1.04
  (예전 0.58·2.36·−1.19·1.13 — 제자리 회전이 예전보다 빠르다: search 제자리 둘러보기 명령 0.15 에 실측 0.37 rad/s, 예전 모델 0.27)
- 전진 1차: `τ·u̇ + u = a·|f̄|^pw` — a 1.79 m/s, pw 0.54, τ 0.84 s(가속)/1.98 s(감속), 후진 × 0.755
  (예전 2.15·0.76·0.81·2.49)
- 정상 속도 실측 (여러 bag, 직진 3 s 이상): f̄ 0.05 → 0.22 · 0.10 → 0.34 · 0.18 → 0.65 · 0.23 → 0.77 · 0.25 → 0.82 ·
  0.40 → 1.08 · 0.47 → 1.22 m/s. 0.18–0.25 는 오늘 bag 이 처음 준 자료(search LOS 속도대)로 예전 모델은 0.06–0.07 m/s
  느렸다. 지금 모델은 0.18–0.47 을 ±0.07 로 맞추는 대신 **0.10 아래는 빠르다** (0.10 → 0.52, 실측 0.34) — 도킹 저속
  접근은 예전 계수가 더 맞는다. 거듭제곱 하나로는 둘 다 못 맞춘다.
- `δ = (우 − 좌)/2`, `f̄ = (좌 + 우)/2`, 좌·우 = `(|PWM − 1500| − 25)/375`, 입력 지연 0.1 s
- 외란: 일정 물살(시나리오 `current`) + 돌풍 OU(σ 0.04 m/s) + 바람 요 외란 OU(σ 0.02 rad/s).
  11-13 bag 의 모터 중립 표류 속도 중앙값 0.15 m/s, 90 % 0.20 m/s (11-14 아침은 0.02 m/s 로 잔잔).
- 옆미끄럼·파도·피치는 없다.
- 예전 계수로 돌리려면 `KABOAT_E2E_NOMOTO=$PWD/scripts/e2e_sim/nomoto_20261113.json` (아래 돌리기).
- 맞춤에 쓰는 믹서 재현 PWM 은 bag `/diagnostics` 의 실제 PWM 과 자율 96–99 %·텔레옵 100 % 가 10 µs 안 (2026-11-14 확인).

`calib/` 스크립트는 다시 맞출 때만 쓴다: 빈 작업 폴더에서 `python3 calib/extract.py <bag 폴더> cache/<이름>`
(bag → `cache/<이름>.pkl`·스캔 npz) → `fit_prep.py [이름…]` → `fit_nomoto.py` → `cache/nomoto.json`
(11-14 처럼 한 bag 의 미션 구간으로: `fit_prep.py run14` → `fit_nomoto_run14.py` → `cache/nomoto_run14.json`).
이름 = dock1(dock_test1_20261113_100845)·dock2(dock2_20261113_151029)·run12(run_20261112_152344)·run14(run1_20261114_091623).
`detcal.py`(검출률)·`build_arena.py`(물체 군집)도 같은 `cache/` 를 읽는다.

## 경기장 (`arena.json`, `make_arena.py`, `calib/build_arena.py`)

2026-11-13 오전 dock_test1 bag 한 시점의 배치 (같은 날 오후 dock2 에서는 스테이션 부표가 약 4 m,
도킹장이 약 11 m 옮겨져 있어 섞지 않았다).

| 요소 | 출처 |
|---|---|
| 게이트 7쌍·스테이션 4개·탐색 부표 | dock_test1 라이다 점 군집 (탐색 부표 색은 그 bag `/detections/buoys`) |
| 도킹장 3칸 | 그 bag 에서 dock_ctrl_v2 가 고정한 ㄷ자(가운데 칸) + dock_slot.yaml 치수, 표지 순서는 YOLO 방위 |
| 회피 구역 | dock_test1 군집(바 7개·부표·경계 줄), 서쪽 출구는 virtual_walls.yaml·run_20261112 |
| 미션 지점 | bag `/mission_points/markers` (gate/station/dock/search/avoid entry·end, station_center) |
| 라이다 검출률 | `detcal.json` — 거리 구간별 스캔당 보일 확률·빔 반환율 (부표·바·도킹 벽·경계 부이) |

회피 구역 연결 장애물은 2026-11-14 사용자 지시로 **부표 높이의 바**: 검출률은 tournament1 bag 의 바
(4 m 안 스캔마다 100 %, 4~6 m 49 %, 6~7.5 m 36 %). 예전 낮은 연결물(4 m 안 73~77 %)은 `bar_cls: boom_low` 로 비교.
출발 위치(`start`)는 그 bag 에 없어 gate_entry 선수 방향 9 m 뒤로 **가정**했다 (11-14 경기장은 실제 출발 자세).

### 2026-11-14 경기장 (`arena_20261114.json`, `make_arena_20261114.py`)

run1_20261114_091623 실주행 bag 의 배치. 11-13 과 달라진 것: 게이트 가운데 쌍들, 스테이션 부표(약 1 m), **도킹장 약 8 m 북쪽**
(오늘 고정 칸 = 서쪽 triangle), **탐색 부표 전부**, 회피 구역 안쪽 바·부표. 회피 구역 경계·입구·출구는 11-13 그대로(오늘
라이다 점과 겹침). 미션 지점은 그 bag `/mission_points/markers`, 출발 자세는 AUTO 시작(78.8 s) odom (57.93, 8.24, 41.7°).

| 확인 | 결과 |
|---|---|
| 오늘 실제 경로(배 중심) → 새 물체 표면 | 대부분 0.4 m 이상. 북쪽 바 0.26 m·입구 대각선 0.29 m (실물이 스쳤거나 군집이 배 쪽으로 0.1 m 번짐) |
| 회피 구역 병목 통로폭 (`calib/passable.py`) | 1.81 m (11-13 배치 1.51 m) |
| 탐색 (171.6, −4.4) | 검정 부표 하나 — 실물 검출기는 초록으로 판정(몸통 H 60–64·S 37–43 이 초록 기준 H 55–89·S ≥ 40 에 걸림)해 search 가 검정을 못 찾고 포기. 시뮬 카메라는 이 오판을 재현하지 않음 |

실제 주행을 같은 채점으로 보려면 bag 캐시를 판 폴더 형식으로 바꿔 `analyze.py` 에 넣는다:
`python3 calib/bag_to_run.py cache/run14.pkl arena_20261114.json $KABOAT_E2E_OUT/real14 && python3 analyze.py real14`
(그날: gate 30 s 통과, station 42 s, dock triangle 111 s, search 150 s 포기, avoid 133 s 통과).

## 시나리오 (`scen/*.json`)

`current`[동, 북 m/s], `start`, `points`(미션 지점 덮어쓰기), `drop_points`, `remove`/`remove_prefix`(물체 빼기),
`marker_hidden`(YOLO 가 못 보는 도형), `cmd_drop`[t0, t1](모터 무반응), `bar_cls`, `detect_scale`, `shift`.
`*_x.json` = station_end 를 부표에서 3 m 동쪽 (162.8, −33.6) 으로 옮긴 것.

`odom_error` (RTK Float 흉내, 2026-11-14 run2): 정답 자세에 '목표 슬롯 뒷벽 중심 기준 회전 + 도크 법선 방향 이동'
오차를 씌워 `/odom`·TF·`/gps/fix` 로 낸다. 라이다·카메라는 정답에서 만들므로 odom 안에서 도크가 옮겨·돌아 보인다
(현장과 같은 구조). 시각은 `/mission/state=dock` 부터, `trace.jsonl` 에 `err_m`·`err_deg`.
- `decay`: `t0_s` 까지 (`shift_m`, `rot_deg`), 그 뒤 `dur_s` 에 걸쳐 0 — `dock_drift_run2.json` (2.2 m·12° → 45 s, run2 실측 모양)
- `ramp`: `t0_s` 부터 `rate_mps`·`rate_dps` 로 커짐(`max_m`·`max_deg` 까지) — `dock_drift_ramp_pos/neg.json`
  (run2 p90 0.07 m/s·0.3°/s 가 잠금 뒤 끝까지 계속되는 최악)
이 셋은 도킹장 앞 (167.5, −29.0) 출발이라 `start_state:=dock single_mission:=true` 로 도킹만 돌린다.
동시에 여러 판을 돌리면 노트북 CPU 가 바빠 TO_GOAL 선수가 수십 도 흔들렸다(혼자 돌리면 3–5°) — 제어 비교는 한 판씩.

## 한계

- YOLO·카메라 영상은 실제가 아니다 (표지 방위에 잡음을 넣은 검출기 출력, 부표는 색 사각형).
- 물체는 고정이다 (실제 부표·바는 흔들리고 밀린다). 접촉은 기록만 하고, 도킹장·바만 통과를 막는다.
- 판마다 난수·타이밍이 달라 같은 시나리오도 결과가 갈린다 — 한 판으로 단정하지 말 것.
