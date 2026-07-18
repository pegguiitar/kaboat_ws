"""접촉 판정 — 50Hz 궤적과 점유 셀(≥50) 사이의 최소 거리를 잰다.

접촉 기준: 선체 반폭 ~0.3m + 여유 → min clearance > 0.5m 면 접촉 0 판정.
도착(<1.2m) 또는 sim 120s 에 요약 출력 후 종료.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid

GX, GY = float(sys.argv[1]), float(sys.argv[2])


class C(Node):
    def __init__(self):
        super().__init__('clearance_check')
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.on_grid, 1)
        self.occ = None          # (N,2) 점유 셀 중심 world 좌표
        self.t0 = None
        self.done = False
        self.min_d = float('inf')
        self.min_at = None
        self.n = 0

    def on_grid(self, g):
        d = np.asarray(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        iy, ix = np.nonzero(d >= 50)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        self.occ = np.stack([ox + (ix + 0.5) * res, oy + (iy + 0.5) * res], axis=1)

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        if self.done:
            return
        p = m.pose.pose.position
        self.n += 1
        if self.occ is not None and len(self.occ):
            d = float(np.min(np.hypot(self.occ[:, 0] - p.x, self.occ[:, 1] - p.y)))
            if d < self.min_d:
                self.min_d = d
                self.min_at = (round(p.x, 2), round(p.y, 2), round(t - self.t0, 1))
        if math.hypot(GX - p.x, GY - p.y) < 1.2 or t - self.t0 > 120.0:
            self.done = True
            occ_n = 0 if self.occ is None else len(self.occ)
            print(f'CLEARANCE min={self.min_d:.2f}m at{self.min_at} '
                  f'(traj {self.n}pts, occ {occ_n}cells) '
                  f'{"CONTACT-FREE" if self.min_d > 0.5 else "TOO CLOSE"}',
                  flush=True)


rclpy.init()
n = C()
while rclpy.ok() and not n.done:
    rclpy.spin_once(n, timeout_sec=1.0)
n.destroy_node()
rclpy.shutdown()
