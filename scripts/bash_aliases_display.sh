# docker-compose 는 컨테이너 생성 시점의 호스트 $DISPLAY 값을 그대로 고정해서
# 넣는데, 그 뒤 호스트 디스플레이 세션 번호가 바뀌면(예: :0 -> :1) 컨테이너
# 안에는 옛날 값이 남아 Gazebo GUI 가 "could not connect to display" 로 죽는다.
# docker exec -it 로 새 셸을 열 때마다 실제로 살아있는 X11 소켓을 보고
# DISPLAY 를 매번 다시 맞춰서, 컨테이너를 재생성하거나 -e DISPLAY=$DISPLAY 를
# 손으로 넘길 필요가 없게 한다.
_live_display=$(ls /tmp/.X11-unix 2>/dev/null | grep -o '[0-9]*' | sort -n | tail -1)
if [ -n "$_live_display" ]; then
    export DISPLAY=":${_live_display}"
fi
unset _live_display
