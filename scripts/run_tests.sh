#!/usr/bin/env bash
# pytest 전체(L1~L3)를 Docker 일회용 컨테이너에서 실행한다 — TEST_PLAN.md 참고.
#
#   scripts/run_tests.sh                 # 전체
#   scripts/run_tests.sh -m level_unit   # L1 만 (pytest 인자를 그대로 넘긴다)
#   KABOAT_TEST_FIGURES=1 scripts/run_tests.sh   # 그림도 저장 (test_output/figures)
#
# 저장소는 읽기 전용으로 붙이고, kaboat_msgs 만 컨테이너 안 /tmp 에서 빌드한다.
# 결과는 test_output/junit.xml 에 남는다.
# (src/ 안에서 실행하면 src/kaboat_msgs 폴더가 빌드된 메시지 패키지를 가려서
#  import 가 깨진다 — 그래서 저장소 루트를 작업 폴더로 쓴다.)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${KABOAT_TEST_IMAGE:-kaboat-lidar:humble}"
mkdir -p "$REPO/test_output"

FIG_ENV=()
if [[ -n "${KABOAT_TEST_FIGURES:-}" ]]; then
  FIG_ENV=(-e KABOAT_TEST_FIGURES=/out/figures)
fi

docker run --rm \
  -v "$REPO":/ws:ro \
  -v "$REPO/test_output":/out \
  -e PYTHONDONTWRITEBYTECODE=1 \
  "${FIG_ENV[@]}" \
  "$IMAGE" bash -c '
set -e
source /opt/ros/humble/setup.bash
mkdir -p /tmp/msgs && cd /tmp/msgs
colcon build --base-paths /ws/src/kaboat_msgs --packages-select kaboat_msgs > /tmp/msgs/build.log 2>&1 \
  || { cat /tmp/msgs/build.log; exit 1; }
source /tmp/msgs/install/setup.bash
export PYTHONPATH=/ws/src/kaboat_behaviors:/ws/src/kaboat_hardware:/ws/src/kaboat_perception:/ws/src/kaboat_mission:/ws/src/kaboat_control:$PYTHONPATH
cd /ws
python3 -m pytest -p no:cacheprovider -q -rfs --junitxml=/out/junit.xml \
  src/kaboat_behaviors/test src/kaboat_hardware/test src/kaboat_perception/test \
  src/kaboat_mission/test src/kaboat_control/test tests "$@"
' bash "$@"
