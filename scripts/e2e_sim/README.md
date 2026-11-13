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

## 배 모델 (`nomoto.json`, `calib/fit_nomoto.py`)

dock_test1·dock2(2026-11-13)·run(2026-11-12) bag 의 `/cmd_vel`·`/teleop/levels` → 실제 믹서로 좌우 PWM 재현 →
`/odom` 속도에 맞춤 (구동 구간 2281 s, 오차 RMS 전진 0.27 m/s·선회 0.10 rad/s — 바람 표류 포함).

- 선회 Nomoto 1차: `T·ṙ + r = K(u)·sgn(δ)|δ|^q`, `K(u) = K0 + K1·|u|` — T 0.58 s, K0 2.36, K1 −1.19, q 1.13
- 전진 1차: `τ·u̇ + u = a·|f̄|^pw` — a 2.15 m/s, pw 0.76, τ 0.81 s(가속)/2.49 s(감속), 후진 × 0.75
- `δ = (우 − 좌)/2`, `f̄ = (좌 + 우)/2`, 좌·우 = `(|PWM − 1500| − 25)/375`, 입력 지연 0.1 s
- 외란: 일정 물살(시나리오 `current`) + 돌풍 OU(σ 0.04 m/s) + 바람 요 외란 OU(σ 0.02 rad/s).
  bag 의 모터 중립 표류 속도 중앙값 0.15 m/s, 90 % 0.20 m/s.
- 옆미끄럼·파도·피치는 없다. 명령 0.05 아래 저속 구간은 bag 자료가 적어 외삽이다.

`calib/` 스크립트는 다시 맞출 때만 쓴다: 빈 작업 폴더에서 `python3 calib/extract.py <bag 폴더> cache/<이름>`
(bag → `cache/<이름>.pkl`·스캔 npz) → `fit_prep.py` → `fit_nomoto.py` → `cache/nomoto.json`.
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
