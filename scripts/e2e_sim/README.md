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

결과는 `$KABOAT_E2E_OUT`(기본 `/tmp/kaboat_e2e_runs`)`/<이름>/` — `trace.jsonl`(5 Hz 정답 자세·PWM·명령·state),
`events.jsonl`(state·상태 토픽·미션 매니저 로그·접촉), `launch.log`, `summary.json`, `path.png`.

- `run_one.sh` 는 `HOME` 을 판 폴더로 바꿔 `~/kaboat_mission_points.json`·`~/kaboat_geo_origin.yaml` 이 시뮬
  것을 가리키게 한다 (실제 홈 파일은 안 건드림, 런치 인자는 실물 기본값 그대로).
- 노트북 `install/kaboat_hardware` 가 옛 복사본이면 옛 도킹 코드가 돈다 — `run_one.sh` 가
  `PYTHONPATH` 앞에 `src/kaboat_hardware` 를 둔다.
- 런치 인자는 `search_target_color:=red dock_target_class:=circle dock_device:=cpu` 고정 (run_one.sh).

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
- 예전 계수로 돌리려면 `run_one.sh` 의 `--nomoto` 를 `nomoto_20261113.json` 으로.

`calib/` 스크립트는 다시 맞출 때만 쓴다: 빈 작업 폴더에서 `python3 calib/extract.py <bag 폴더> cache/<이름>`
(bag → `cache/<이름>.pkl`·스캔 npz) → `fit_prep.py [이름…]` → `fit_nomoto.py` → `cache/nomoto.json`
(11-14 처럼 한 bag 의 미션 구간으로: `fit_prep.py run14` → `fit_nomoto_run14.py` → `cache/nomoto_run14.json`).
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
출발 위치(`start`)는 bag 에 없어 gate_entry 선수 방향 9 m 뒤로 **가정**했다.

## 시나리오 (`scen/*.json`)

`current`[동, 북 m/s], `start`, `points`(미션 지점 덮어쓰기), `drop_points`, `remove`/`remove_prefix`(물체 빼기),
`marker_hidden`(YOLO 가 못 보는 도형), `cmd_drop`[t0, t1](모터 무반응), `bar_cls`, `detect_scale`, `shift`.
`*_x.json` = station_end 를 부표에서 3 m 동쪽 (162.8, −33.6) 으로 옮긴 것.

## 한계

- YOLO·카메라 영상은 실제가 아니다 (표지 방위에 잡음을 넣은 검출기 출력, 부표는 색 사각형).
- 물체는 고정이다 (실제 부표·바는 흔들리고 밀린다). 접촉은 기록만 하고, 도킹장·바만 통과를 막는다.
- 판마다 난수·타이밍이 달라 같은 시나리오도 결과가 갈린다 — 한 판으로 단정하지 말 것.
