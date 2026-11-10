#!/usr/bin/env python3
"""Print live rates and compact payloads without flooding a terminal with pixels."""
import argparse
import json
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped, Twist
from kaboat_msgs.msg import MarkArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String

TYPES = {
    '/camera/color/image_raw': Image,
    '/camera/depth/image_raw': Image,
    '/camera/camera_info': CameraInfo,
    '/dock/debug_image': Image,
    '/detections/dock_marks': MarkArray,
    '/dock/marker_ground': PointStamped,
    '/dock/marker_ground_raw': PointStamped,
    '/dock/markers/triangle/ground': PointStamped,
    '/dock/markers/circle/ground': PointStamped,
    '/dock/markers/square/ground': PointStamped,
    '/dock/final_waypoint': PointStamped,
    '/dock/approach_point': PointStamped,
    '/dock/alignment_point': PointStamped,
    '/dock/stop_point': PointStamped,
    '/dock/waypoints': RosPath,
    '/dock/avoid/cmd_preview': Twist,
    '/cmd_vel': Twist,
    '/dock/avoid/goal_candidate': PointStamped,
    '/dock/avoid/goal_status': String,
    '/dock/avoid/status': String,
    '/dock/bspline_path': RosPath,
    '/dock/detection_status': String,
    '/dock/lidar/status': String,
}

for marker_class in ('triangle','circle','square'):
    for suffix in ('wall_point_base','wall_point','marker_base','marker_ground'):
        TYPES[f'/dock/lidar/{marker_class}/{suffix}']=PointStamped
    for suffix in ('normal_base','normal'):
        TYPES[f'/dock/lidar/{marker_class}/{suffix}']=Vector3Stamped


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('topic', choices=TYPES)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('monitor' + args.topic.replace('/', '_'))
    received = deque(maxlen=100)
    latest = None
    counters = deque(maxlen=100)
    image_streams = {'/camera/color/image_raw': 'color',
                     '/camera/depth/image_raw': 'depth',
                     '/camera/camera_info': 'camera_info',
                     '/dock/debug_image': 'debug'}
    counter_mode = args.topic in image_streams

    def receive(msg):
        nonlocal latest
        now = time.monotonic()
        if counter_mode:
            data = json.loads(msg.data)
            stream = image_streams[args.topic]
            count = (data.get('debug_count', 0) if stream == 'debug'
                     else data.get('input_counts', {}).get(stream, 0))
            if counters and count < counters[-1][1]:
                counters.clear()
                received.clear()
            if not counters or count != counters[-1][1]:
                counters.append((now, count))
                received.append(now)
            latest = (f'frames published={count}; limits: 320px / 5Hz (default)'
                      if stream == 'debug' else data.get('input_info', {}).get(stream))
            return
        received.append(now)
        if isinstance(msg, Image):
            latest = f'{msg.width}x{msg.height} {msg.encoding} frame={msg.header.frame_id}'
        elif isinstance(msg, CameraInfo):
            latest = f'{msg.width}x{msg.height} fx={msg.k[0]:.2f} fy={msg.k[4]:.2f}'
        elif isinstance(msg, PointStamped):
            latest = (f'{msg.header.frame_id} x={msg.point.x:.3f} y={msg.point.y:.3f} '
                      f'z={msg.point.z:.3f} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        elif isinstance(msg, Twist):
            latest=f'linear.x={msg.linear.x:.3f} angular.z={msg.angular.z:.3f} (preview)'
        elif isinstance(msg, Vector3Stamped):
            latest=f'{msg.header.frame_id} normal=({msg.vector.x:.3f}, {msg.vector.y:.3f}, {msg.vector.z:.3f})'
        elif isinstance(msg, MarkArray):
            latest = [(m.shape, round(m.confidence, 3), round(m.distance, 2)) for m in msg.marks]
        elif isinstance(msg, RosPath):
            latest = f'{len(msg.poses)} poses frame={msg.header.frame_id}'
            if msg.poses:
                p = msg.poses[-1].pose.position
                latest += f' endpoint=({p.x:.3f}, {p.y:.3f})'
        else:
            try:
                data = json.loads(msg.data)
            except ValueError:
                latest = msg.data
                return
            latest = {k: data[k] for k in ('ground_valid', 'valid', 'reason', 'target_class', 'goal_xy', 'candidate_xy', 'averaging', 'confidence', 'filter_state', 'markers', 'processing_ms') if k in data}

    node.create_subscription(String if counter_mode else TYPES[args.topic],
                             '/dock/detection_status' if counter_mode else args.topic,
                             receive, qos_profile_sensor_data)
    print(args.topic, '[pipeline counters; no image subscription]' if counter_mode else '', flush=True)
    try:
        while rclpy.ok():
            until = time.monotonic() + 1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            rate = ((len(received) - 1) / (received[-1] - received[0])
                    if len(received) > 1 and now - received[-1] < 2 else 0)
            if counter_mode:
                rate = ((counters[-1][1] - counters[0][1]) / (counters[-1][0] - counters[0][0])
                        if len(counters) > 1 and now - counters[-1][0] < 2 else 0)
            age = f'{now - received[-1]:.1f}s' if received else 'never'
            print(f'{rate:.1f} Hz | last received {age} ago | {latest}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
