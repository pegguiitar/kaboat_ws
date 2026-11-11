#!/usr/bin/env python3
"""노트북 키보드 조종 창 — Zenoh 로 젯슨 thruster_driver 에 명령을 보낸다.

노트북(ROS 도메인 43)이 보내는 토픽. 젯슨(도메인 42)이 받아 ESP 로 시리얼 전송한다.
  /teleop/enable   (Bool)           노트북 조종 모드 진입·해제
  /teleop/levels   (Int8MultiArray) [속도 단계 -4~+4, 선회 +1(A)/0/-1(D)]. 20 Hz
  /teleop/gps_mark (Empty)          Space — 젯슨 gps_mark_logger 가 CSV 에 한 줄 기록
  /teleop/esp_mode (String)         P = computer_control_mode, O = rc_control_mode
받는 토픽: /teleop/status (젯슨 제어 모드), /teleop/event (ESP 모드 전송·GPS 기록 결과)

키 (조종 창을 클릭해 포커스를 준 상태에서, 한글 입력 상태여도 같은 자리 키로 동작)
  W / S    속도 단계 +1 / -1 (-4 ~ +4, 떼도 유지)
  A / D    누르고 있는 동안 좌 / 우 회전 (좌우 출력 차이, 중립이면 제자리 회전)
  X        즉시 0단계(중립)
  Space    현재 GPS 위치 기록
  P / O    ESP 를 컴퓨터 조종 / RC 조종 모드로
  R        조종 종료 후 자율주행 복귀,  Q·Esc·창 닫기  조종 종료(중립 유지)
창이 포커스를 잃거나 젯슨 상태가 끊기면 즉시 0단계로 돌아간다.
"""

import fcntl
import os
from pathlib import Path
import sys
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int8MultiArray, String

from teleop_keys import MAX_LEVEL, KeyTracker, SpeedLevel, normalize_key, teleop_command


DOMAIN = '43'
PERIOD_MS = 50
STATUS_MAX_AGE = 1.0
LOCK_PATH = Path('/tmp/kaboat_laptop_teleop.lock')
ESP_MODES = {'p': 'computer_control_mode', 'o': 'rc_control_mode'}
BG = '#17202a'


def level_bar(level):
    """-4 … +4 를 칸으로 그린 문자열. 현재 단계 칸만 채운다."""
    cells = []
    for k in range(-MAX_LEVEL, MAX_LEVEL + 1):
        mark = '■' if k == level else ('│' if k == 0 else '·')
        cells.append(mark)
    return ' '.join(cells)


class TeleopWindow:
    def __init__(self, node):
        self.node = node
        self.mode_pub = node.create_publisher(Bool, '/teleop/enable', 10)
        self.levels_pub = node.create_publisher(Int8MultiArray, '/teleop/levels', 10)
        self.gps_pub = node.create_publisher(Empty, '/teleop/gps_mark', 10)
        self.esp_pub = node.create_publisher(String, '/teleop/esp_mode', 10)
        node.create_subscription(String, '/teleop/status', self._on_status, 10)
        node.create_subscription(String, '/teleop/event', self._on_event, 10)

        self.root = tk.Tk()
        # 한글 입력기가 W/A/S/D 를 가로채 ㅈ/ㅁ/ㄴ/ㅇ 로 바꾸지 않게 이 창은 입력기를 안 쓴다.
        # (그래도 자모가 오면 normalize_key 가 같은 자리 영문 키로 바꾼다)
        try:
            self.root.tk.call('tk', 'useinputmethods', '0')
        except tk.TclError:
            pass
        self.root.title('KABOAT 노트북 키보드 조종')
        self.root.geometry('620x360')
        self.root.resizable(False, False)
        self.root.configure(bg=BG)
        self.status_label = tk.Label(
            self.root, text='젯슨 상태 대기 중', fg='#f8c471', bg=BG,
            font=('Sans', 15, 'bold'))
        self.status_label.pack(pady=(16, 6))
        self.level_label = tk.Label(
            self.root, text='', fg='white', bg=BG, font=('Monospace', 16, 'bold'))
        self.level_label.pack(pady=4)
        self.output_label = tk.Label(
            self.root, text='조종하려면 이 창을 클릭하세요', fg='white', bg=BG,
            font=('Sans', 12))
        self.output_label.pack(pady=4)
        self.event_label = tk.Label(
            self.root, text='', fg='#85c1e9', bg=BG, font=('Sans', 11))
        self.event_label.pack(pady=4)
        tk.Label(
            self.root,
            text='W/S 속도 단계 +1/-1 (-4~+4, 유지)   A/D 누르는 동안 좌/우 회전\n'
                 'X 즉시 중립   Space GPS 위치 기록   P 컴퓨터 조종 / O RC 조종 (ESP)\n'
                 'R 자율주행 복귀   Q·Esc·창 닫기 조종 종료(중립 유지)',
            fg='#d5dbdb', bg=BG, font=('Sans', 11), justify='center').pack(pady=10)

        self.keys = KeyTracker()
        self.speed = SpeedLevel()
        self.focused = False
        self.last_status_time = 0.0
        self.last_status = 'NO_STATUS'
        self.last_event = ''
        self.closing = None  # None, 'manual', or 'auto'

        self.root.bind('<KeyPress>', self._key_press)
        self.root.bind('<KeyRelease>', self._key_release)
        self.root.bind('<FocusIn>', self._focus_in)
        self.root.bind('<FocusOut>', self._focus_out)
        self.root.protocol('WM_DELETE_WINDOW', self._close_manual)
        self.root.after(PERIOD_MS, self._tick)

    # ── 젯슨에서 오는 것 ───────────────────────────────
    def _on_status(self, msg):
        self.last_status = msg.data
        self.last_status_time = time.monotonic()

    def _on_event(self, msg):
        self.last_event = msg.data

    def _ready(self, now):
        return (now - self.last_status_time <= STATUS_MAX_AGE
                and self.last_status in ('TELEOP', 'TELEOP_TIMEOUT'))

    # ── 키 입력 ────────────────────────────────────────
    def _neutral(self):
        self.speed.stop()
        self.keys.clear()
        self.levels_pub.publish(Int8MultiArray(data=[0, 0]))

    def _focus_in(self, event):
        self.focused = True

    def _focus_out(self, event):
        # 데드맨: 창을 벗어나면 단계도 0 으로 (다시 들어와도 예전 속도로 출발하지 않게)
        self.focused = False
        self._neutral()

    @staticmethod
    def _key(event):
        return normalize_key(event.keysym, getattr(event, 'char', ''),
                             getattr(event, 'keycode', None))

    def _key_press(self, event):
        key = self._key(event)
        if not self.keys.press(key, time.monotonic()):
            return                                  # 자동반복 — 한 번만 처리
        if key == 'r':
            self._close('auto')
        elif key in ('q', 'escape'):
            self._close('manual')
        elif key == 'x':
            self.speed.stop()
        elif key == 'w':
            if self._ready(time.monotonic()):
                self.speed.up()
        elif key == 's':
            if self._ready(time.monotonic()):
                self.speed.down()
        elif key == 'space':
            self.gps_pub.publish(Empty())
            self.last_event = 'GPS 위치 기록 요청 보냄…'
        elif key in ESP_MODES:
            self.esp_pub.publish(String(data=ESP_MODES[key]))
            self.last_event = f'ESP 모드 전송 요청: {ESP_MODES[key]}'

    def _key_release(self, event):
        self.keys.release(self._key(event), time.monotonic())

    def _close(self, how):
        if self.closing is None:
            self.closing = how
            self._neutral()
            self.root.after(500, self.root.destroy)

    def _close_manual(self):
        self._close('manual')

    # ── 20 Hz 주기 ─────────────────────────────────────
    def _tick(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            now = time.monotonic()
            held = self.keys.poll(now)
            self.mode_pub.publish(Bool(data=self.closing != 'auto'))

            ready = self._ready(now)
            if ready and self.focused and self.closing is None:
                level, turn = teleop_command(self.speed.level, held)
            else:
                self.speed.stop()
                level, turn = 0, 0
            self.levels_pub.publish(Int8MultiArray(data=[level, turn]))

            if now - self.last_status_time > STATUS_MAX_AGE:
                label, color = '젯슨 응답 없음 — 키보드 명령 차단', '#e74c3c'
            elif self.last_status == 'ESTOP':
                label, color = '비상 정지 활성', '#e74c3c'
            elif self.last_status == 'RC_OVERRIDE':
                label, color = '무선 조종기 우선', '#f8c471'
            elif ready:
                label, color = '노트북 조종 연결됨', '#58d68d'
            else:
                label, color = f'젯슨 모드: {self.last_status}', '#f8c471'
            self.status_label.configure(text=label, fg=color)
            self.level_label.configure(
                text=f'속도 단계 {self.speed.level:+d}    {level_bar(self.speed.level)}')
            turn_text = {1: '좌회전 (A)', -1: '우회전 (D)', 0: '직진'}[turn]
            focus = '' if self.focused else '   (창을 클릭하세요)'
            self.output_label.configure(
                text=f'보내는 명령: 속도 {level:+d}단계, {turn_text}{focus}')
            self.event_label.configure(text=self.last_event)
            self.root.after(PERIOD_MS, self._tick)
        except tk.TclError:
            # Closing the window while a timer is queued is normal.
            pass

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.levels_pub.publish(Int8MultiArray(data=[0, 0]))
            # Q/창 닫기는 젯슨 텔레옵 모드를 유지한 채 중립. R 만 자율주행으로 넘긴다.
            self.mode_pub.publish(Bool(data=self.closing != 'auto'))


def main():
    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN:
        raise SystemExit('노트북 호스트에서 ROS_DOMAIN_ID=43을 설정하세요.')
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        raise SystemExit('그래픽 데스크톱 터미널에서 실행하세요.')

    with LOCK_PATH.open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('노트북 텔레옵 창이 이미 실행 중입니다.')
        rclpy.init()
        node = Node('laptop_keyboard_teleop')
        try:
            TeleopWindow(node).run()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
