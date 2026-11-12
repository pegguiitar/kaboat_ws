"""Prepare/replay a sensor-only bag for live RViz + perception inspection.

Replays original sensor messages; no saved experiment wall hypotheses and no
command topics. Each loop deserializes original bytes before shifting stamps.
"""
import argparse
import datetime as dt
from pathlib import Path
import sqlite3
import time

import rclpy
import rosbag2_py
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

TOPICS = {'/scan', '/scan/level', '/odom', '/imu/data', '/camera/camera_info',
          '/camera/camera/color/image_raw/compressed', '/tf', '/tf_static'}


def make_slice(bag, output):
    tz = dt.timezone(dt.timedelta(hours=9))
    start = int(dt.datetime(2026, 11, 12, 15, 31, 5, tzinfo=tz).timestamp()*1e9)
    end = int(dt.datetime(2026, 11, 12, 15, 33, 10, tzinfo=tz).timestamp()*1e9)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=output, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    created, statics = set(), {}
    rows = []
    for file in sorted(Path(bag).glob('*.db3')):
        db = sqlite3.connect(file.as_uri()+'?mode=ro', uri=True)
        topics = {i: (n, t) for i, n, t in db.execute('select id,name,type from topics') if n in TOPICS}
        for name, typ in topics.values():
            if name not in created:
                writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
                created.add(name)
        static_ids = [i for i, (n, _) in topics.items() if n == '/tf_static']
        for tid in static_ids:
            for (raw,) in db.execute('select data from messages where topic_id=? and timestamp<=? order by timestamp', (tid, start)):
                for tr in deserialize_message(raw, TFMessage).transforms:
                    statics[tr.child_frame_id] = tr
        ids = [i for i, (n, _) in topics.items() if n != '/tf_static']
        sql = 'select timestamp,topic_id,data from messages where topic_id in ('+','.join('?' for _ in ids)+') and timestamp>=? and timestamp<=? order by timestamp,id'
        for ns, tid, raw in db.execute(sql, ids+[start, end]):
            rows.append((ns, topics[tid][0], bytes(raw)))
        db.close()
    rows.sort(key=lambda r:r[0])
    if statics:
        writer.write('/tf_static', serialize_message(TFMessage(transforms=list(statics.values()))), start)
    for ns, name, raw in rows:
        writer.write(name, raw, ns)
    del writer
    print(f'SENSOR_BAG_READY messages={len(rows)} duration=125s path={output}', flush=True)


def replay(bag, rate):
    rclpy.init()
    node = rclpy.create_node('live_sensor_bag_replay')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name:t.type for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    pubs = {}
    for topic, typ in types.items():
        qos = qos_profile_sensor_data if typ.startswith('sensor_msgs/') else QoSProfile(depth=30)
        if topic == '/tf_static':
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pubs[topic] = node.create_publisher(get_message(typ), topic, qos)
    clock = node.create_publisher(Clock, '/clock', 10)
    records = []
    while reader.has_next():
        name, raw, ns = reader.read_next()
        if name in pubs:
            records.append((ns, name, bytes(raw)))
    first, last = records[0][0], records[-1][0]
    duration = last-first+200_000_000
    loop = 0
    time.sleep(2.)
    try:
        while rclpy.ok():
            offset = loop*duration
            wall0 = time.monotonic()
            for ns, name, raw in records:
                if not rclpy.ok():
                    break
                lag = (ns-first)/1e9/rate-(time.monotonic()-wall0)
                if lag > 0: time.sleep(lag)
                if not rclpy.ok():
                    break
                msg = deserialize_message(raw, get_message(types[name]))
                if hasattr(msg, 'header'):
                    msg.header.stamp = Time(nanoseconds=Time.from_msg(msg.header.stamp).nanoseconds+offset).to_msg()
                if hasattr(msg, 'transforms'):
                    for tr in msg.transforms:
                        tr.header.stamp = Time(nanoseconds=Time.from_msg(tr.header.stamp).nanoseconds+offset).to_msg()
                clock.publish(Clock(clock=Time(nanoseconds=ns+offset).to_msg()))
                pubs[name].publish(msg)
            loop += 1
            print(f'LIVE_SENSOR_REPLAY loop={loop} rate={rate}', flush=True)
    except KeyboardInterrupt:
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['slice', 'play'])
    ap.add_argument('bag')
    ap.add_argument('--out')
    ap.add_argument('--rate', type=float, default=1.)
    a = ap.parse_args()
    make_slice(a.bag, a.out) if a.mode == 'slice' else replay(a.bag, a.rate)
