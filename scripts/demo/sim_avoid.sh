#!/bin/bash
# 사진 기반 회피 구역 서쪽 입구 바깥(world 2, 77.5)에 스폰.
# 짧은 변의 흰색 부표 두 개 사이로 진입해 동쪽 출구로 나간다.
source "$(dirname "${BASH_SOURCE[0]}")/_env.sh"

# 이전 sim이 비정상 종료되면서 남긴 odom publisher를 제거한다.
# pgrep/pkill 명령 자체가 검색 결과에 잡히지 않도록 [o] 패턴을 사용한다.
pkill -TERM -f '[o]dom_gps_noise.py' 2>/dev/null || true
for _ in {1..20}; do
  pgrep -f '[o]dom_gps_noise.py' >/dev/null || break
  sleep 0.1
done
# SIGTERM 뒤에도 남은 프로세스만 최종 정리한다.
pkill -KILL -f '[o]dom_gps_noise.py' 2>/dev/null || true

# sigma=0.25m인 OU 위치잡음 — 오차의 약 95%(2σ)가 50cm 이내.
exec ros2 launch kaboat_sim simulation.launch.py headless:=True spawn_x:=2.0 spawn_y:=77.5 odom_noise_sigma:=0.25 "$@"
