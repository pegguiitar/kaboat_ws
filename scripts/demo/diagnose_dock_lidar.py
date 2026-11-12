#!/usr/bin/env python3
"""Summarize scan continuity and three independent dock-wall fits."""
import argparse
from collections import defaultdict
import json
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


CLASSES = ('triangle', 'circle', 'square')
LIDAR_IN_BASE = (0.1 * 1.1 / 4.9, 0.0, 1.83 * 1.1 / 4.9)


def rotated(vector, q):
    x, y, z = vector
    u = (q.x, q.y, q.z)
    cross = (u[1] * z - u[2] * y, u[2] * x - u[0] * z, u[0] * y - u[1] * x)
    cross2 = (u[1] * cross[2] - u[2] * cross[1],
              u[2] * cross[0] - u[0] * cross[2],
              u[0] * cross[1] - u[1] * cross[0])
    return tuple(v + 2 * (q.w * c + d) for v, c, d in zip(vector, cross, cross2))


def summary(values):
    if not values:
        return None
    return {'count': len(values), 'min': round(min(values), 3),
            'median': round(statistics.median(values), 3),
            'max': round(max(values), 3)}


class Monitor(Node):
    def __init__(self):
        super().__init__('dock_lidar_diagnostic')
        self.scans = []
        self.stamps = []
        self.sector_hits = []
        self.anomalies = []
        self.normal_angles = defaultdict(list)
        self.normal_spans = defaultdict(list)
        self.normal_valid = defaultdict(int)
        self.status_count = 0
        self.wall_x = defaultdict(list)
        self.lidar_z = []
        self.pose = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/dock/lidar/status', self.on_status, 10)
        self.create_subscription(MarkerArray, '/dock/lidar/direct_markers', self.on_markers,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom_ground_truth', self.on_odom,
                                 qos_profile_sensor_data)

    def on_odom(self, msg):
        self.pose = msg.pose.pose
        q = self.pose.orientation
        self.lidar_z.append(self.pose.position.z + rotated(LIDAR_IN_BASE, q)[2])

    def on_scan(self, msg):
        valid = [math.isfinite(r) and msg.range_min < r < msg.range_max for r in msg.ranges]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        count = sum(valid)
        sector = sum(ok and 3.0 < r < 14.0 and
                     abs(msg.angle_min + i * msg.angle_increment) < math.radians(45)
                     for i, (ok, r) in enumerate(zip(valid, msg.ranges)))
        self.scans.append(count)
        self.stamps.append(stamp)
        self.sector_hits.append(sector)
        if sector < 90:
            self.anomalies.append({'sim_time_s': round(stamp, 3), 'valid_beams': count,
                                   'dock_sector_hits': sector,
                                   'lidar_z_m': round(self.lidar_z[-1], 3) if self.lidar_z else None})

    def on_status(self, msg):
        data = json.loads(msg.data)
        self.status_count += 1
        for name in CLASSES:
            item = data.get('direct_normals', {}).get(name, {})
            if item.get('valid'):
                self.normal_valid[name] += 1
                nx, ny = item['normal_odom']
                self.normal_angles[name].append(math.degrees(math.atan2(abs(ny), -nx)))
                self.normal_spans[name].append(item['span_m'])

    def on_markers(self, msg):
        if self.pose is None:
            return
        q = self.pose.orientation
        for marker in msg.markers:
            if marker.action != Marker.ADD or marker.id % 4 != 2 or len(marker.points) < 2:
                continue
            index = marker.id // 4
            if index >= len(CLASSES):
                continue
            point = marker.points[len(marker.points) // 2]
            dx = rotated((point.x, point.y, point.z), q)[0]
            self.wall_x[CLASSES[index]].append(self.pose.position.x + dx)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor()
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        count = node.status_count
        intervals = [b - a for a, b in zip(node.stamps, node.stamps[1:]) if b > a]
        result = {
            'scan_frames': len(node.scans),
            'scan_sim_interval_s': summary(intervals),
            'valid_beams': summary(node.scans),
            'dock_sector_hits_3_to_14m': summary(node.sector_hits),
            'empty_dock_sector_frames': sum(v == 0 for v in node.sector_hits),
            'low_dock_sector_frames': node.anomalies[:30],
            'lidar_world_height_m': summary(node.lidar_z),
            'fit_status_frames': count,
            'direct_walls': {name: {
                'valid_frames': node.normal_valid[name],
                'valid_fraction': round(node.normal_valid[name] / count, 3) if count else None,
                'normal_error_from_minus_x_deg': summary(node.normal_angles[name]),
                'fit_span_m': summary(node.normal_spans[name]),
                'wall_world_x_m': summary(node.wall_x[name]),
            } for name in CLASSES},
        }
        print(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
