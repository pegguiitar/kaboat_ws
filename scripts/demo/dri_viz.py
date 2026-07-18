#!/usr/bin/env python3
"""dri_viz — DRI 를 RViz 로 보는 스크래치 디버그 노드 (커밋 안 함).

AVOIDANCE.MD §3-1 대로 **매 tick(10Hz)** 재계산한다:
  DRI = f(격자, 배 pose) 인데 격자는 1Hz, pose 는 ~50Hz 다. 격자는 정적 부표 +
  월드 앵커라 낡지 않지만 pose 는 초당 30~56° 돌므로 f=cos(Δθ) 가 금방 딴 값이
  된다. 그래서 격자는 캐시해두고 최신 pose 로 매 tick 다시 만든다.
  (obstacle_planner 가 compute_cmd() 안에서 할 일을 여기서 미리 재현하는 셈)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry

from kaboat_behaviors.dri import build_dri, DriParams

TICK_HZ = 10.0


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriViz(Node):
    def __init__(self):
        super().__init__('dri_viz')
        for k, v in [('dri_scale', 4.0), ('sigma', 1.0), ('dist_falloff', 4.0),
                     ('k_dist', 0.3), ('k_head', 0.3), ('a_dist', 1.0), ('a_head', 1.0)]:
            self.declare_parameter(k, v)
        self.declare_parameter('occ_threshold', 75)

        self.odom = None
        self.grid = None          # 최신 격자 msg 캐시 — 1Hz 로만 갱신됨
        self.n_build = 0

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self._on_grid, 1)
        self.pub = self.create_publisher(OccupancyGrid, '/avoid/dri', 1)
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            f'dri_viz 시작 — /occupancy_grid(1Hz) + /odom -> /avoid/dri ({TICK_HZ:.0f}Hz 재계산)')

    def _on_odom(self, msg):
        self.odom = msg

    def _on_grid(self, msg):
        self.grid = msg           # 캐시만. 재계산은 _tick 이 한다.

    def _params(self):
        g = lambda k: self.get_parameter(k).value
        return DriParams(occ_threshold=int(g('occ_threshold')),
                         sigma=float(g('sigma')), dist_falloff=float(g('dist_falloff')),
                         k_dist=float(g('k_dist')), k_head=float(g('k_head')),
                         a_dist=float(g('a_dist')), a_head=float(g('a_head')))

    def _tick(self):
        if self.grid is None or self.odom is None:
            return
        p = self.odom.pose.pose.position
        yaw = _yaw(self.odom.pose.pose.orientation)
        par = self._params()
        dri = build_dri(self.grid, (p.x, p.y), yaw, par)   # 캐시된 격자 + 지금 pose

        scale = float(self.get_parameter('dri_scale').value)
        vals = np.clip(dri.data / scale * 100.0, 0.0, 100.0)
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.grid.header.frame_id
        out.info = self.grid.info      # anchor/resolution 동일 -> occupancy_grid 와 겹침
        out.data = np.rint(vals).astype(np.int8).ravel().tolist()
        self.pub.publish(out)

        self.n_build += 1
        occ = int((np.asarray(self.grid.data, dtype=np.int16) >= par.occ_threshold).sum())
        self.get_logger().info(
            f'DRI max={dri.data.max():.1f} occ_cells={occ} yaw={math.degrees(yaw):+.0f}deg '
            f'builds={self.n_build}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    n = DriViz()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
