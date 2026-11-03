#!/usr/bin/env python3
"""Laptop WASD control through Zenoh, with focus/key-release deadman behavior.

ROS domain 43 publishes /teleop/enable and /teleop/cmd_vel. The Jetson driver
in domain 42 arbitrates those against RC and autonomous /cmd_vel commands.
"""

import fcntl
import os
from pathlib import Path
import sys
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from teleop_keys import SmoothAxes


DOMAIN = '43'
PERIOD_MS = 50
STATUS_MAX_AGE = 1.0
LOCK_PATH = Path('/tmp/kaboat_laptop_teleop.lock')


class TeleopWindow:
    def __init__(self, node):
        self.node = node
        self.mode_pub = node.create_publisher(Bool, '/teleop/enable', 10)
        self.cmd_pub = node.create_publisher(Twist, '/teleop/cmd_vel', 10)
        node.create_subscription(
            String, '/teleop/status', self._on_status, 10)

        self.root = tk.Tk()
        self.root.title('KABOAT 노트북 텔레옵')
        self.root.geometry('560x250')
        self.root.resizable(False, False)
        self.root.configure(bg='#17202a')
        self.status_label = tk.Label(
            self.root, text='젯슨 상태 대기 중', fg='#f8c471', bg='#17202a',
            font=('Sans', 15, 'bold'))
        self.status_label.pack(pady=(20, 10))
        self.keys_label = tk.Label(
            self.root, text='조종하려면 이 창을 클릭하세요', fg='white',
            bg='#17202a', font=('Sans', 12))
        self.keys_label.pack(pady=5)
        tk.Label(
            self.root,
            text='W 전진  S 후진  A 좌회전  D 우회전\n'
                 'W+A / W+D 전진 회전 · S+A / S+D 후진 회전\n'
                 'Space 정지 · R 자율주행 복귀 · Q 또는 창 닫기 중립 유지',
            fg='#d5dbdb', bg='#17202a', font=('Sans', 11),
            justify='center').pack(pady=12)

        self.pressed = set()
        self.space_held = False
        self.focused = False
        self.smoother = SmoothAxes()
        self.last_tick = time.monotonic()
        self.last_status_time = 0.0
        self.last_status = 'NO_STATUS'
        self.closing = None  # None, 'manual', or 'auto'

        self.root.bind('<KeyPress>', self._key_press)
        self.root.bind('<KeyRelease>', self._key_release)
        self.root.bind('<FocusIn>', self._focus_in)
        self.root.bind('<FocusOut>', self._focus_out)
        self.root.protocol('WM_DELETE_WINDOW', self._close_manual)
        self.root.after(PERIOD_MS, self._tick)

    def _on_status(self, msg):
        self.last_status = msg.data
        self.last_status_time = time.monotonic()
        if msg.data not in ('TELEOP', 'TELEOP_TIMEOUT'):
            self.pressed.clear()
            self.smoother.reset()

    def _focus_in(self, event):
        self.focused = True

    def _focus_out(self, event):
        self.focused = False
        self.pressed.clear()
        self.space_held = False
        self.smoother.reset()
        self.cmd_pub.publish(Twist())

    def _key_press(self, event):
        key = event.keysym.lower()
        if key == 'r':
            self._return_auto()
        elif key in ('q', 'escape'):
            self._close_manual()
        elif key == 'space':
            self.space_held = True
            self.pressed.clear()
            self.smoother.reset()
            self.cmd_pub.publish(Twist())
        elif key in ('w', 'a', 's', 'd') and not self.space_held:
            self.pressed.add(key)

    def _key_release(self, event):
        key = event.keysym.lower()
        if key == 'space':
            self.space_held = False
        else:
            self.pressed.discard(key)

    def _return_auto(self):
        if self.closing is None:
            self.closing = 'auto'
            self.pressed.clear()
            self.smoother.reset()
            self.root.after(500, self.root.destroy)

    def _close_manual(self):
        if self.closing is None:
            self.closing = 'manual'
            self.pressed.clear()
            self.smoother.reset()
            self.root.after(500, self.root.destroy)

    def _tick(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            now = time.monotonic()
            dt = min(0.1, max(0.0, now - self.last_tick))
            self.last_tick = now
            self.mode_pub.publish(Bool(data=self.closing != 'auto'))

            ready = (now - self.last_status_time <= STATUS_MAX_AGE
                     and self.last_status in ('TELEOP', 'TELEOP_TIMEOUT'))
            cmd = Twist()
            if (ready and self.focused and not self.space_held
                    and self.closing is None):
                cmd.linear.x, cmd.angular.z = self.smoother.step(self.pressed, dt)
            else:
                self.smoother.reset()
            self.cmd_pub.publish(cmd)

            if now - self.last_status_time > STATUS_MAX_AGE:
                label, color = '젯슨 응답 없음 — 키보드 명령 차단', '#e74c3c'
            elif self.last_status == 'ESTOP':
                label, color = '비상 정지 활성', '#e74c3c'
            elif self.last_status == 'RC_OVERRIDE':
                label, color = '무선 조종기 우선', '#f8c471'
            elif ready:
                label, color = '노트북 텔레옵 연결됨', '#58d68d'
            else:
                label, color = f'젯슨 모드: {self.last_status}', '#f8c471'
            self.status_label.configure(text=label, fg=color)
            keys = '+'.join(sorted(self.pressed)) if self.pressed else '중립'
            self.keys_label.configure(
                text=f'키: {keys}    전진: {cmd.linear.x:+.3f}    회전: {cmd.angular.z:+.3f}')
            self.root.after(PERIOD_MS, self._tick)
        except tk.TclError:
            # Closing the window while a timer is queued is normal.
            pass

    def run(self):
        try:
            self.root.mainloop()
        finally:
            self.cmd_pub.publish(Twist())
            # Q/window close keeps teleop armed and the Jetson neutral. R alone
            # explicitly returns control to the autonomous /cmd_vel stream.
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
