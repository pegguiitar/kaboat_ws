#!/usr/bin/env bash
# ==============================================================================
# 젯슨: 센서·모터·지도·카메라를 한 번에 — tmux 세션 하나에 창 하나씩 (로그가 안 섞이게)
#
#   run_sensors_motors      # 켜고 그 화면으로 (이미 켜져 있으면 붙기만)
#   stop_sensors_motors     # 모든 창에 Ctrl+C → 다 꺼지면 세션 닫기
#
# 창 (Ctrl+B 다음 숫자로 이동):
#   0 gq7     run_gq7_rtk   GQ7 + RTK (/odom)
#   1 lidar   run_lidar     /imu/data 가 나온 뒤 켠다 (기울기 판단에 GQ7 IMU)
#   2 motor   run_motor     모터 + Space GPS 기록기
#   3 points  run_points    rec 지점 기록기
#   4 map     run_map       장애물 지도 (라이다만)
#   5 camera  run_camera    D455
# run_zenoh 는 따로 켠다.
#
# 각 창은 보통 터미널(bash)이고 ~/.bashrc 단축어를 그대로 입력해 준다 — 사람이 친 것과 같다.
# 그래서 한 창에서 Ctrl+C 로 끄면 로그가 남고, ↑ Enter 로 그것만 다시 켤 수 있다.
# tmux: Ctrl+B 숫자 = 창 이동, Ctrl+B w = 목록, Ctrl+B [ = 위로 스크롤(q 끝),
#       Ctrl+B d = 빠져나오기(계속 돎, 다시 보기 run_sensors_motors)
# ==============================================================================

set -uo pipefail

SESSION="${KABOAT_TMUX_SESSION:-boat}"
IMU_WAIT_SEC="${KABOAT_IMU_WAIT_SEC:-40}"
# 창 이름:명령 (순서 = 창 번호). lidar 는 IMU 를 기다린 뒤 따로 보낸다.
WINDOWS=(gq7:run_gq7_rtk lidar:run_lidar motor:run_motor points:run_points
         map:run_map camera:run_camera)

attach() {
    [ -t 0 ] && [ -t 1 ] || return 0       # 터미널이 아니면(스크립트에서 부를 때) 붙지 않음
    if [ -n "${TMUX:-}" ]; then
        tmux switch-client -t "$SESSION"
    else
        tmux attach -t "$SESSION"
    fi
}

send() {   # send 창이름 명령
    tmux send-keys -t "$SESSION:$1" "$2" Enter
}

wait_for_imu() {
    local i
    for ((i = 0; i < IMU_WAIT_SEC; i++)); do
        if ros2 topic list 2>/dev/null | grep -qx /imu/data; then
            echo "✓ /imu/data 확인 ($((i)) s)"
            return 0
        fi
        sleep 1
    done
    echo "⚠️  ${IMU_WAIT_SEC} s 안에 /imu/data 가 안 보임 — 라이다를 그냥 켭니다 (gq7 창 확인: Ctrl+B 0)"
    return 1
}

start() {
    if ! command -v tmux >/dev/null; then
        echo "❌ tmux 가 없습니다 — sudo apt install tmux" >&2
        exit 1
    fi
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "이미 켜져 있음 (세션 $SESSION) — 붙습니다. 끄려면 stop_sensors_motors"
        attach
        return 0
    fi
    if [ -z "${NTRIP_USER:-}" ] && [ ! -f "$HOME/.ntrip_account" ]; then
        echo "❌ NTRIP 계정 없음 — ~/.ntrip_account 에 export NTRIP_USER=… NTRIP_PASS=… (run_gq7_rtk 필요)" >&2
        exit 1
    fi

    local w name
    tmux new-session -d -s "$SESSION" -n "${WINDOWS[0]%%:*}" bash
    for w in "${WINDOWS[@]:1}"; do
        tmux new-window -d -t "$SESSION" -n "${w%%:*}" bash
    done
    for w in "${WINDOWS[@]}"; do
        name="${w%%:*}"
        [ "$name" = lidar ] && continue
        send "$name" "${w#*:}"
        echo "▶ $name: ${w#*:}"
    done
    send lidar "echo '/imu/data 를 기다리는 중 — 나오면 run_lidar 를 켭니다'"
    echo "… /imu/data 기다리는 중 (최대 ${IMU_WAIT_SEC} s)"
    wait_for_imu
    send lidar run_lidar
    echo "▶ lidar: run_lidar"
    echo "✅ 세션 $SESSION — Ctrl+B 숫자로 창 이동 (0 gq7 1 lidar 2 motor 3 points 4 map 5 camera)"
    attach
}

stop() {
    if ! tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "켜진 세션 없음 ($SESSION)"
        return 0
    fi
    local i w busy
    echo "⏹  모든 창에 Ctrl+C (카메라 → … → GQ7 순)"
    for ((i = ${#WINDOWS[@]} - 1; i >= 0; i--)); do
        tmux send-keys -t "$SESSION:${WINDOWS[i]%%:*}" C-c 2>/dev/null
    done
    for ((i = 0; i < 20; i++)); do          # 다 셸로 돌아올 때까지 (최대 10 s)
        busy=$(tmux list-panes -s -t "$SESSION" -F '#{pane_current_command}' 2>/dev/null \
               | grep -cvx -E 'bash|sh|zsh')
        [ "$busy" -eq 0 ] && break
        sleep 0.5
    done
    if [ "$busy" -ne 0 ]; then
        echo "⚠️  ${busy}개 창이 아직 안 꺼져 세션을 닫으며 강제 종료합니다"
    fi
    tmux kill-session -t "$SESSION"
    echo "✅ 세션 $SESSION 닫음"
}

case "${1:-start}" in
    start) start ;;
    stop) stop ;;
    *) echo "사용: $0 [start|stop]" >&2; exit 2 ;;
esac
