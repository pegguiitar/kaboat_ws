#!/usr/bin/env python3
"""keyboard_teleop — WASD keyboard controller for simulation.

Publishes geometry_msgs/Twist to /cmd_vel. Run this together with
twist2thrust.py, and do not run autonomy.launch.py at the same time unless you
intentionally want cmd_mux and this teleop node to compete for /cmd_vel.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP = """
WASD keyboard teleop

  w : forward
  s : reverse
  a : turn left
  d : turn right
  space / x : stop
  q : quit

Hold a key to keep commanding it. If no key is received for timeout_sec, this
node publishes stop automatically.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # 명령 = 최대추력 비율 (twist2thrust 정규화, 2026-07-17).
        # 기본값은 구 scale=60 시절 검증 동작과 추력 N 동일: 12N / 후진 6N / 차동 ±3N.
        MAX_THRUST = 118.6
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('max_linear', 12.0 / MAX_THRUST)
        self.declare_parameter('max_reverse', 6.0 / MAX_THRUST)
        self.declare_parameter('max_angular', 3.0 / MAX_THRUST)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('timeout_sec', 0.35)

        cmd_topic = self.get_parameter('cmd_topic').value
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_reverse = float(self.get_parameter('max_reverse').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.cmd = Twist()
        self.last_key_time = None
        self.quit_requested = False

        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'keyboard_teleop 시작 — {cmd_topic}, '
            f'linear={self.max_linear}, angular={self.max_angular}'
        )
        print(HELP, flush=True)

    def handle_key(self, key: str):
        self.last_key_time = self.get_clock().now()

        cmd = Twist()
        if key == 'w':
            cmd.linear.x = self.max_linear
        elif key == 's':
            cmd.linear.x = -self.max_reverse
        elif key == 'a':
            cmd.angular.z = self.max_angular
        elif key == 'd':
            cmd.angular.z = -self.max_angular
        elif key in (' ', 'x'):
            pass
        elif key == 'q':
            self.quit_requested = True
            pass
        else:
            return

        self.cmd = cmd
        self.pub.publish(self.cmd)

    def tick(self):
        if self.last_key_time is None:
            self.pub.publish(Twist())
            return

        elapsed = (self.get_clock().now() - self.last_key_time).nanoseconds * 1e-9
        if elapsed > self.timeout_sec:
            self.cmd = Twist()
        self.pub.publish(self.cmd)


def read_key(timeout: float = 0.02) -> str:
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return ''
    return sys.stdin.read(1)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not node.quit_requested:
            key = read_key()
            if key:
                node.handle_key(key.lower())
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
