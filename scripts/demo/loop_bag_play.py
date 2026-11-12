#!/usr/bin/env python3
"""bag 반복 재생 — 반복할 때마다 시각을 bag 길이만큼 앞으로 밀어서 다시 낸다.

`ros2 bag play --loop` 는 처음으로 돌아갈 때 시뮬 시간이 거꾸로 가서 Humble RViz 가
'Resetting RViz' 하다 죽고, 노드들도 시간 역행으로 상태를 버린다. 여기서는 /clock 과
메시지 header·TF 시각에 (반복 횟수 × bag 길이)를 더해 시간이 끊김 없이 앞으로만 간다.

사용: loop_bag_play.py <bag 폴더> [배속, 기본 1.0]
QoS: 센서 토픽(스캔·영상·camera_info·IMU)은 BEST_EFFORT, 나머지 RELIABLE,
     /tf_static 은 TRANSIENT_LOCAL 로 처음 한 번만.
"""
import sys
import time

import rclpy
import rosbag2_py
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message

SENSOR_TYPES = ('sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Image',
                'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/CameraInfo',
                'sensor_msgs/msg/Imu')


def read_all(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgs = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        msgs.append((ts, topic, deserialize_message(data, get_message(types[topic]))))
    msgs.sort(key=lambda m: m[0])
    return types, msgs


def shift(stamp, offset_ns):
    t = Time.from_msg(stamp).nanoseconds + offset_ns
    return Time(nanoseconds=t).to_msg()


def main():
    path = sys.argv[1]
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    rclpy.init()
    node = rclpy.create_node('loop_bag_play')
    types, msgs = read_all(path)
    t_first, t_last = msgs[0][0], msgs[-1][0]
    length = t_last - t_first + 100_000_000          # bag 길이 + 0.1 s
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {}
    for name, typ in types.items():
        qos = (latched if name == '/tf_static'
               else qos_profile_sensor_data if typ in SENSOR_TYPES else reliable)
        pubs[name] = node.create_publisher(get_message(typ), name, qos)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    node.get_logger().info(f'{path}: {len(msgs)} msgs, {length / 1e9:.1f} s 반복 (×{rate})')
    loop = 0
    try:
        while rclpy.ok():
            offset = loop * length
            wall0 = time.monotonic()
            for ts, topic, msg in msgs:
                if topic == '/tf_static' and loop > 0:
                    continue
                due = (ts - t_first) / 1e9 / rate
                lag = due - (time.monotonic() - wall0)
                if lag > 0:
                    time.sleep(lag)
                if hasattr(msg, 'header'):
                    msg.header.stamp = shift(msg.header.stamp, offset)
                if hasattr(msg, 'transforms'):
                    for tr in msg.transforms:
                        tr.header.stamp = shift(tr.header.stamp, offset)
                clock = Clock()
                clock.clock = Time(nanoseconds=ts + offset).to_msg()
                clock_pub.publish(clock)
                pubs[topic].publish(msg)
            loop += 1
            node.get_logger().info(f'반복 {loop} 끝 — 시각을 {length / 1e9:.1f} s 밀어 다시')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
