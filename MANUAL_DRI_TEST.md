# 무선 조종 중 DRI 모니터링

배는 **별도 무선 조종기의 원격 조종 모드**로 움직인다. 이 테스트 런처는
`/scan`·`/odom`을 읽어 DRI를 계산할 뿐, 모터 명령을 발행하거나 자율운항
노드를 시작하지 않는다. DRI 화면은 관찰용이며 무선 조종 명령을 자동으로
막거나 바꾸지 않는다.

```text
선체 TG-50 /scan + 수조 위치 /odom
    → 젯슨 /occupancy_grid → 기존 build_dri() → /manual_dri/dri
    → Zenoh + Tailscale → 노트북 RViz
무선 조종기 → 배의 원격 조종 모드 (ROS 모니터링 경로와 분리)
```

## 젯슨

노트북의 외벽 라이다와 젯슨의 실내 `/odom` 준비는
[TANK_TESTS_GUIDE.md](TANK_TESTS_GUIDE.md)의 1~2절을 따른다. 젯슨에
선체 TG-50이 연결되어 있어 `/scan`도 나와야 한다. 젯슨과 노트북 양쪽 모두
**수정된** `scripts/zenoh_monitor_link.py`를 사용해야 `/manual_dri/dri`와
`/manual_dri/status`가 노트북으로 넘어간다. 브리지가 이미 실행 중이면
양쪽에서 종료 후 새 스크립트로 다시 시작한다. 허용 토픽 목록은 시작할 때
적용된다.

젯슨에서 최초 한 번 빌드한다. 아래 경로는 현재 가이드의 젯슨 클론
위치이며, 실제 설치 위치가 다르면 바꾼다.

```bash
cd ~/Desktop/kaboat_ws/repo
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to kaboat_hardware
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

센서와 `/odom`을 실행한 **뒤에**, 같은 환경의 새 젯슨 터미널에서:

```bash
ros2 topic info /scan
ros2 topic info /odom --verbose
ros2 launch kaboat_hardware manual_dri_test.launch.py
```

`/odom`의 발행자는 하나여야 한다. 이미 `/occupancy_grid`가 실행 중이면
`start_occupancy_grid:=false`를 붙여 중복 지도 발행을 피한다. 이미
`base_link → onboard_laser_frame`의 실제 TF를 발행 중이면
`publish_scan_tf:=false`를 붙여 중복 TF를 피한다. 기본 TF는 라이다와
`base_link`의 원점·방향이 같다는 기존 점유 지도 가정을 따른다.

```bash
ros2 topic echo /manual_dri/status
ros2 topic hz /manual_dri/dri
ros2 topic info /cmd_vel --verbose
```

`READY`가 나오면 DRI가 갱신 중이다. 무선 조종기로 움직이기 전에
`/cmd_vel`의 ROS 발행자 수가 **0**인지 확인하고, 배의 원격 조종 모드를
기존 하드웨어 절차대로 설정한다. 이 테스트 런처는 스러스터 드라이버도
시작하지 않는다. 원격 조종 모드에서 별도 모터 드라이버가 필요한
구성이라면 그 모드에 맞는 기존 절차로 따로 실행한다.

## 노트북

노트북에서는 기존 Tailscale·Zenoh 연결을 유지한다. 외벽 라이다 Docker를
사용하는 수조 시험이라면 [수조 가이드의 1절](TANK_TESTS_GUIDE.md)에 따라
컨테이너를 먼저 실행한다. 실행 중인 외벽 라이다를 재시작하지 않고 새 코드를
컨테이너에도 반영한다.

```bash
cd ~/Projects/KABOAT/kaboat_ws
xhost +si:localuser:root
docker exec kaboat_lidar printenv ROS_DOMAIN_ID  # 43 확인
docker exec -it kaboat_lidar bash -lc \
  'source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to kaboat_hardware'
```

노트북 **호스트의 다른 터미널**에서 브리지를 시작한다. 젯슨의 주소는
젯슨에서 `tailscale ip -4`로 확인해 입력한다.

```bash
cd ~/Projects/KABOAT/kaboat_ws
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0
printf '젯슨 Tailscale IP: '
read -r JETSON_TAILSCALE_IP
python3 scripts/zenoh_monitor_link.py laptop --jetson-ip "$JETSON_TAILSCALE_IP"
```

젯슨에서도 `ROS_DOMAIN_ID=42`로 `python3 scripts/zenoh_monitor_link.py jetson`을
실행한다. 두 브리지 터미널은 켜 둔다. 노트북의 **그래픽 데스크톱 터미널**에서
RViz를 연다.

```bash
docker exec -it -e DISPLAY="$DISPLAY" kaboat_lidar bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspace/kaboat_ws/install/setup.bash && ros2 launch kaboat_hardware manual_dri_rviz.launch.py'
```

RViz의 `Fixed Frame`은 `odom`이다. 빨간 점은 `/scan`, 회색 지도는
`/occupancy_grid`, 겹친 색상 지도는 `/manual_dri/dri`, 화살표는 `/odom`이다.
DRI는 `viz_dri_scale: 4.0`으로 0~100에 맞춰 **표시한 값**이며 원래 위험도
실수값 자체가 아니다. 조정값은
[`manual_dri_test.yaml`](src/kaboat_hardware/config/manual_dri_test.yaml)에 있다.

`WAIT_ODOM`, `WAIT_SCAN`, `WAIT_GRID`, `WAIT_GRID_OBSERVATIONS` 상태에서는
기존 DRI를 미관측(`-1`) 지도 값으로 지운다. 각각 실내 `/odom`, 선체 라이다와 기울기 필터,
점유 지도 노드, 유효한 지도 관측 여부를 확인한다. 노트북에 DRI 토픽이
없으면 양쪽 Zenoh 스크립트가 최신인지와 브리지 연결을 확인한다.
