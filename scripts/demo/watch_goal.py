"""주행 감시 — 2 sim초마다 위치·속도·자세를 찍고, 목표 도착(1.2m)이나
120초 초과에서 요약 후 종료한다. 전복(roll/pitch) 최대치도 같이 본다.

사용: watch_goal.py <goal_x> <goal_y>
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

GX, GY = float(sys.argv[1]), float(sys.argv[2])
ARRIVE, TIMEOUT = 1.2, 120.0


def rp(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class W(Node):
    def __init__(self):
        super().__init__('watch_goal')
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 20)
        self.t0 = self.tlast = None
        self.done = False
        self.mr = self.mp = self.vmax = 0.0

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = self.tlast = t
        p = m.pose.pose.position
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        r, pi_ = rp(m.pose.pose.orientation)
        self.mr, self.mp = max(self.mr, abs(r)), max(self.mp, abs(pi_))
        self.vmax = max(self.vmax, v)
        d = math.hypot(GX - p.x, GY - p.y)
        if t - self.tlast >= 2.0:
            self.tlast = t
            print(f't={t - self.t0:5.1f}s pos=({p.x:6.2f},{p.y:6.2f}) d={d:5.2f} '
                  f'v={v:4.2f} wz={m.twist.twist.angular.z:+.2f}', flush=True)
        if d < ARRIVE:
            print(f'ARRIVED t={t - self.t0:.1f}s vmax={self.vmax:.2f} '
                  f'max|roll|={self.mr:.3f} max|pitch|={self.mp:.3f}', flush=True)
            self.done = True
        elif t - self.t0 > TIMEOUT:
            print(f'TIMEOUT d={d:.2f} max|roll|={self.mr:.3f} max|pitch|={self.mp:.3f}',
                  flush=True)
            self.done = True


rclpy.init()
n = W()
while rclpy.ok() and not n.done:
    rclpy.spin_once(n, timeout_sec=1.0)
n.destroy_node()
rclpy.shutdown()
