# 데모 스크립트 공통 환경 — 각 스크립트가 source 한다.
# 컨테이너에서 워크스페이스는 /workspace/kaboat_ws 에 마운트되고,
# 이 폴더는 그 안의 scripts/demo/ 다 (docker-compose 의 ./scripts 마운트).
HERE="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
# GUI(sim GUI, RViz)용 — 호스트에서 `xhost +local:` 먼저 실행해야 한다.
# headless 라도 필요: GPU 라이다/뎁스 센서가 렌더링 씬을 만들기 때문에
# X 접속에 실패하면 gz 서버가 시작 직후 죽는다.
export DISPLAY=${DISPLAY:-:0} LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
