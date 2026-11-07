#!/usr/bin/env bash
# rviz_tank 화면의 실제 궤적(주황)을 지운다. gate/avoid 종류에서는 직전 계획 경로(초록)도 함께 지운다.
# 노트북 호스트에서 실행하며, 라이다 컨테이너(kaboat_lidar)의 시각화 노드에 서비스를 호출한다.
set -uo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx kaboat_lidar; then
    echo 'kaboat_lidar 컨테이너가 실행 중이 아닙니다.' >&2
    exit 1
fi

out=$(docker exec kaboat_lidar bash -lc \
    'source /opt/ros/humble/setup.bash && source /workspace/kaboat_ws/install/setup.bash && export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0 && timeout 8 ros2 service call /tank_test/clear_trajectory std_srvs/srv/Trigger "{}"' 2>&1)

if grep -q 'success=True' <<<"$out"; then
    echo '궤적 삭제 완료'
else
    echo '궤적 삭제 실패: rviz_tank를 먼저 실행했는지 확인하세요.' >&2
    tail -3 <<<"$out" >&2
    exit 1
fi
