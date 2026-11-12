# 사진 기반 경기장 1차 구현

기존 `competition`의 배(`wamv_kaboat.xacro`, 선체 길이 1.1 m)와
종합미션 월드(`perception_task.sdf`)를 사용한다. 종합미션의 회피 구역만
새 모델로 교체하며 항로추종 채널, 탐색 구역, 도킹 구조물은 유지한다.

## 반영한 형태

- 내부 흰색 둥근 부표 **6열 × 3개 = 18개**와 입·출구 흰 부표 4개:
  직경 **0.90 m**, 수면 위 높이 0.47 m. 이전 직경 0.64 m에서 확대했다.
- 빨강·주황색의 낮은 연결 장애물 8개: 폭 0.26 m, 전체 높이 0.22 m,
  수면 위 높이 **0.12 m**. 흰색 부표보다 낮다.
- 배치도 위·아래 방향의 연결 **5개**, 대각 연결 **3개**(경계 연결형 1개 포함).
  모든 연결 장애물이 적어도 하나의 흰 부표와 연결된다. 독립형은 없다.
- 긴 세로 장애물 하나는 양끝보다 안쪽에 흰 부표 2개가 붙고, 장애물 끝이
  두 부표 바깥으로 각각 0.8 m 연장된다(부표 중심 기준).
- 노란 소형 부표(직경 0.15 m, 간격 최대 0.8 m)를 줄지어 배치한 직사각형 경계.
  소형 부표들을 지름 0.018 m의 가는 연결 줄로 잇는다. 부표와 줄의 색은
  별도이며, 사진으로 줄 색을 확정할 수 없어 줄은 임시 회갈색이다.
- 직사각형 네 꼭짓점의 큰 주황색 부표: 직경 1.04 m, 수면 위 높이 0.9 m.
- 두 짧은 변(서쪽·동쪽)에 각각 흰 부표 2개로 표시한 입구·출구.
  중심 간격 4 m, 흰 부표 표면 사이 유효 폭은 3.10 m이다.
  입·출구 사이에는 연결 줄과 노란 소형 부표를 두지 않는다.
- 모든 장애물과 경계 줄에 시각 형상과 같은 충돌 형상 포함.

사진 `IMG_0547`·`0548`·`0549`·`0294`와 설명에서 형태·색상·연결 유형을
반영했다. **치수, 개수, 배치는 실측 복원이 아니라 초기 근사**다.
구역은 처음 20 × 13 m 로 배치했고, 2026-11-12 동서 길이만 현장 실측 **27 m** 로 늘렸다
(GPS 모서리 기록 25.8~25.9 m + 부표가 기록점보다 바깥이라 양끝 0.6 m씩. 남북 13 m·물체 크기는
그대로, 내부 부표·붐 x 위치는 ×1.35 로 비례 확장). 실제 배치도를 받으면
아래 JSON에서 조정한다. 흰색은 이 회피 구역에 적용하며 다른 미션의
빨강/초록 게이트와 탐색 부표 색은 그대로다.

## 좌표와 파도

새 모델 원점은 직사각형 남서쪽 모서리다. 모델 좌표에 `(5, 11)`을 더하면
종합미션 좌표, `(5, 71)`을 더하면 world/odom 좌표다.
구역의 world 범위는 x=5..32, y=71..84 m이다.

파도는 기존 VRX 시스템의 **gain=0.8, period=5.0초**로 설정했다.
이전 값은 1.7 / 4.0초였다. 중간 수준을 위한 초기 설정이며 gain은
파고(m)가 아니다. 시각적 파도와 배의 부력은 기존 파도 토픽을 사용한다.
실행 중 배의 롤·피치와 수면을 확인하며 조정해야 한다.

새 구역의 부표·연결 장애물은 **수면에 고정된 static 형상**이다.
배는 기존 부력/유체 모델을 유지하지만, 새 물체의 상하 움직임·유연한
연결·계류 동역학은 아직 구현하지 않았다. 수면 위 0.12 m의 낮은 물체이므로
수평 라이다가 지나칠 수 있고 파도에 일시 가려질 수 있다.
카메라 검출·회피 알고리즘의 변경과 실주행 검증은 별도다.

## 조정 및 확인

`src/kaboat_sim/config/photo_arena.json`에서 부표 좌표, 장애물 양끝, 치수,
색을 수정한 뒤 저장된 SDF를 다시 생성한다. 문자열 끝점은 흰색 부표 이름,
배열 끝점은 모델 로컬 `[x, y]`다. 장애물의 중간에 부표를 붙이려면 실제
양끝을 배열로 지정하고 `attached_buoys`에 연결된 흰 부표 이름들을 넣는다.
생성기는 모든 장애물의 흰 부표 연결을 요구하며 실제로 닿는지도 검사한다.
`gates`의 `lower`·`upper`는 짧은 변 위의 입·출구 흰 부표 이름을 가리킨다.
해당 두 부표 사이의 경계 줄과 소형 부표는 자동으로 생략한다.

```bash
python3 scripts/generate_photo_arena.py
python3 scripts/generate_photo_arena.py --check
python3 -m unittest discover -s src/kaboat_sim/test -p test_photo_arena.py -v
```

기존 Gazebo Garden/VRX 환경에서 실행:

```bash
cd /workspace/kaboat_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaboat_sim
source install/setup.bash
# 기존 배를 서쪽 입구 바깥에 배치하고 GUI로 확인
ros2 launch kaboat_sim simulation.launch.py \
  spawn_x:=2.0 spawn_y:=77.5 spawn_yaw:=0.0
```

GUI에서 회피 구역의 world `(15, 77.5)` 부근을 본다.
기본 spawn `(2, 63)`은 기존 항로추종 출발점이다.
`scripts/demo/sim_avoid.sh`도 입구 바깥에서 시작하도록 바꾸었다.
회피 데모의 목표 기본값은 world `(36, 77.5)`로 동쪽 출구 바깥 4 m이다.
입구 중심은 `(5, 77.5)`, 출구 중심은 `(32, 77.5)`이다.
경계 부표와 연결 줄은 실제 충돌 물체이며, 입·출구를 통해서만 통과한다.

검증 범위: 생성 파일 일치, 연결 끝점, 수면 기준 높이, 충돌·시각 형상 일치,
기존 월드 연결, 부표와 장애물 간 불필요한 겹침, 직경 1.3 m 원형 배 공간의
입구 바깥→구역 내부→출구 바깥 경로 존재를 검사한다.
경로 존재는 플래너 성공을 보장하지 않는다.

이 환경에서는 SDFormat 검사와 로컬 Gazebo Fortress 서버의 새 모델 단독
100스텝 로딩을 확인했다. 전체 Garden/VRX 배·파도·카메라 통합 실행은
아직 검증하지 않았다(로컬에 해당 시뮬레이션 이미지와 VRX 소스가 없음).

## Gazebo 없이 폐루프 회피 시험 (`scripts/arena_sim/avoid_arena_sim.py`)

이 경기장 구조(`photo_arena.json`)로 가상 라이다·배를 만들고 **실제 노드 클래스**
(`occupancy_grid` + 가상 벽, `real_obstacle_avoid_test` = `test_avoid_outdoor` 와 같은
`outdoor_avoid_test.yaml`)를 가짜 시계로 돌린다. 노트북에서 1회 수십 초.

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=src/kaboat_hardware:src/kaboat_behaviors:src/kaboat_perception:src/kaboat_mission:$PYTHONPATH
python3 scripts/arena_sim/avoid_arena_sim.py --scenario field --seed 0 --plot out.png
python3 scripts/arena_sim/avoid_arena_sim.py --scenario field --booms-visible --wind -0.1 0 -p dri.sigma:=0.5
```

- 라이다 스캔 면은 수면 위 0.14 m(현장 실측 환산) — 흰 부표·모서리 부표만 보이고 붐(0.12 m)·
  노란 소형 부표·줄은 안 보인다. `--booms-visible` 은 카메라 층이 붐을 완벽히 잡는다고 본 상한.
- `--scenario ideal`(항상 보임) / `field`(2026-11-12 게이트 녹화로 잰 거리별 검출률·배 근처 잡음·
  위치 잡음). 배 응답(명령→속도·회전)도 같은 녹화로 잰 값(`scripts/arena_sim/field_calib.json`).
- 선체 1.1 × 0.8 m 사각형이 부표·붐·줄과 겹치면 충돌로 센다(물리적으로 막지는 않음).
