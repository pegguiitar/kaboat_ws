# D455 + YOLO26 도킹 입구 ROS 통합

현재 가중치는 `YOLO_KABOAT/results/run_yolo26_1/dock_sequence_v3_yolo26n_best.pt`이다.
합성 test 보강 후 기존 모델의 box/mask mAP50–95는 각각 0.953/0.835였다.
이 수치는 실해상·D455 depth·Jetson 지연을 검증하지 않는다.

## 데이터 흐름

1. `d455.launch.py`가 RGB, **RGB에 정렬된 depth**, color camera_info를 발행한다.
2. `dock_mark_detector`는 `/detector/enable=true`일 때만 YOLO26n-seg를 돌린다.
   세 클래스는 `red_triangle`, `green_circle`, `blue_square`이며 각 클래스의
   최고 신뢰도 마스크 하나만 사용한다.
3. 마스크 안의 유효 depth 중앙값으로 광학계 3D 마커 좌표를 구하고, 표식
   평면의 법선을 추정한다. depth·법선 품질이 부족하면 입구를 발행하지 않는다.
4. 카메라 광학 프레임에서 `base_link`로 측정 시각의 TF를 적용한다. 마커의
   3D 좌표는 `/dock/marker_base`, 법선 방향으로 `wall_to_mouth_m` 이동한
   **z=0 선체 기준 입구 XY**는 `/dock/entrance_base`로 발행한다.
5. `docking_ctrl`은 입구 XY로 거리·방위를 계산해 기존 도킹 FSM에 전달한다.
   `/detections/dock_marks`는 호환/관측용이며 제어 입력으로 쓰지 않는다.

## 첫 배선 점검: 모터 비활성

ROS 2 Humble 환경에서 `kaboat_msgs`, `kaboat_perception`, `kaboat_behaviors`,
`kaboat_bringup`을 빌드한다. detector를 실행하는 **동일 Python 환경**에서
`cv2`, `cv_bridge`, `message_filters`, `tf2_geometry_msgs`, `ultralytics`,
해당 Jetson용 PyTorch 빌드가 import되는지 확인한다. 가중치 파일을 Jetson의
접근 가능한 절대 경로에 배치한다.

개발 PC의 사용자 설치 NumPy 2.2.6은 ROS Humble `cv_bridge`(NumPy 1.x로
빌드됨)와 호환되지 않았다. detector는 NumPy 2 환경에서 명시적으로 중단한다.
Jetson에서도 **ROS와 YOLO가 같은 Python에서 NumPy 1.x, OpenCV, PyTorch를
함께 불러올 수 있는지** 먼저 확인해야 한다. 이 PC에서는 Ultralytics와
ROS-compatible NumPy가 한 환경에 준비되지 않아 ROS 실시간 추론까지는
검증하지 못했다. 기존 `.pt` 모델의 일반 Python 추론과 ROS 패키지 빌드는
각각 확인했다.

카메라 드라이버를 켠 뒤 이미지 `header.frame_id`를 확인하고,
`base_link`까지 **실측 장착 위치/자세의 TF**가 있는지 확인한다. RealSense
자체 TF만으로는 선체 장착 TF가 생기지 않는다. 임의의 0 변환을 넣으면
입구 좌표가 틀리므로 사용하지 않는다. 실제 마커 벽에서 도킹 입구까지의
수평 거리도 측정해 `wall_to_mouth_m`에 넣는다.

```bash
ros2 launch kaboat_bringup d455.launch.py
ros2 topic echo --once /camera/color/image_raw --field header
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 launch kaboat_bringup autonomy.launch.py \
  dock_weights:=/absolute/path/dock_sequence_v3_yolo26n_best.pt \
  dock_target_class:=red_triangle dock_device:=cpu \
  wall_to_mouth_m:=2.5 dock_motion_enabled:=false
ros2 topic echo /dock/detection_status
ros2 topic echo /dock/marker_base
ros2 topic echo /dock/entrance_base
```

위 `camera_color_optical_frame`은 예시다. 실제 이미지 header의 frame_id로
바꾸고, `wall_to_mouth_m:=2.5`도 **현장 실측값으로 바꿔야 한다**. 기본값
2.5m는 가정치이므로 이대로 추력을 켜면 안 된다.

주의: `autonomy.launch.py`는 다른 mission behavior도 포함한다. 모터가 연결된
선체에서 첫 점검을 할 때는 이 전체 launch 대신 카메라와 detector만 띄우고,
`/detector/enable`을 수동으로 활성화해 좌표를 관찰한다. 상태 `valid=false`면
입구 좌표가 갱신되지 않으며 제어기에는 수신 신선도 0.35초와 이미지 시각
최대 지연 0.5초 제한이 있다. 별도 detector 실행 예:

```bash
ros2 run kaboat_perception dock_mark_detector --ros-args \
  -p weights:=/absolute/path/dock_sequence_v3_yolo26n_best.pt \
  -p target_class:=red_triangle -p device:=cpu -p wall_to_mouth_m:=2.5
ros2 topic pub --once --qos-durability transient_local \
  /detector/enable std_msgs/msg/Bool '{data: true}'
```

## 추력 활성화 전 확인

- 선체 정지 상태에서 세 표식의 클래스와 마스크, depth 중앙값, TF 방향을 확인.
- 목표 표식을 바꾸면 `dock_target_class`를 `green_circle` 또는
  `blue_square`로 지정하고 detector를 재시작.
- 거리별(약 2.5–8m) 입구 XY 오차, 법선 방향, `wall_to_mouth_m`을 실측 확인.
- `/dock/entrance_base`가 끊기거나 지연되면 FSM이 정지/재획득하는지 확인.
- 비상정지, 주변 구조물과의 간격, 저속 추력·후진 동작을 별도 시험.
- 그 후에만 `dock_motion_enabled:=true`로 바꾼다. 기본값은 **false**다.

기존 도킹 FSM의 `APPROACH`는 mission waypoint까지의 `seek_goal`을 그대로
사용한다. YOLO 입구 XY는 `ALIGN`/`ENTER`의 로컬 목표다. 전체 항로의
장애물 회피 planner를 도킹 입구까지 자동 연결한 상태는 아니므로,
staging waypoint와 진입 경로의 안전성은 별도로 검증해야 한다.
