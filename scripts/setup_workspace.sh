#!/usr/bin/env bash
# 최초 1회 워크스페이스 소스 세팅. 컨테이너 안에서 실행.
#   docker exec -it kaboat_sim_container bash -c "cd /workspace/kaboat_ws && ./scripts/setup_workspace.sh"
# 또는 컨테이너 접속 후 워크스페이스 루트에서 직접 실행해도 된다.
#
# 하는 일:
#   1. vrx.repos 에 명시된 vrx / ros_gz 소스를 vcstool 로 받는다.
#   2. vrx 저장소에 들어있는 깨진 심링크(ball_shooter 텍스처)를 제거한다.
#      -- 이 파일은 vrx 자체의 알려진 결함으로, 제거하지 않으면
#         `colcon build --symlink-install` 이 실패한다. 예전에는 이 README
#         단계를 사람이 손으로 해야 해서 깜빡하기 쉬웠다 -- 이제 스크립트로 고정.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

echo "==> [1/2] vcs import (vrx, ros_gz)"
vcs import src < vrx.repos

BROKEN_SYMLINK="src/vrx/vrx_urdf/vrx_gazebo/models/ball_shooter/meshes/Glass_BaseColor.png"
if [ -L "$BROKEN_SYMLINK" ]; then
  echo "==> [2/2] Removing known-broken vrx symlink: $BROKEN_SYMLINK"
  rm -f "$BROKEN_SYMLINK"
else
  echo "==> [2/2] Broken symlink already absent, skipping."
fi

echo ""
echo "==> Done. Next steps:"
echo "    rosdep update"
echo "    rosdep install --from-paths src --ignore-src -r -y"
echo "    colcon build --symlink-install"
echo "    (GZ_VERSION=garden is already set by the Dockerfile)"
