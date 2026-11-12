"""Causal rosbag replay comparing latest gate outputs and colour-free walls.

No executor is spun, and all GateFollower publishers are intercepted. No
commands leave this process. Reference gate coordinates are used for scoring
only, never passed to WallModel.
"""
import argparse
import datetime as dt
import gzip
import json
import math
from pathlib import Path
import sqlite3
import sys
import time

import numpy as np

from wall_model import WallModel

KST = dt.timezone(dt.timedelta(hours=9))
ROOT = Path(__file__).resolve().parents[3]


def messages(bag, names, start=None, end=None):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    for file in sorted(Path(bag).glob('*.db3')):
        db = sqlite3.connect(file.as_uri()+'?mode=ro', uri=True)
        topics = {i: (n, get_message(t)) for i, n, t in db.execute('select id,name,type from topics') if n in names}
        if not topics:
            continue
        sql = 'select timestamp,topic_id,data from messages where topic_id in ('+','.join('?' for _ in topics)+')'
        args = list(topics)
        if start is not None:
            sql += ' and timestamp >= ?'
            args.append(int(start*1e9))
        if end is not None:
            sql += ' and timestamp <= ?'
            args.append(int(end*1e9))
        sql += ' order by timestamp,id'
        for ns, tid, raw in db.execute(sql, args):
            name, typ = topics[tid]
            yield ns/1e9, name, deserialize_message(raw, typ)
        db.close()


class Quiet:
    def __getattr__(self, name):
        return lambda *a, **kw: None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--end', default='15:37:00')
    ap.add_argument('--discount', type=float, default=3.)
    args = ap.parse_args()
    import rclpy
    from std_msgs.msg import String
    from kaboat_behaviors.gate_corridor import (GateParams, grid_posts,
        select_gate_pair_from_posts, pair_observed)
    from kaboat_behaviors.gate_follower import GateFollower
    from kaboat_behaviors.behavior_base import yaw_from_quaternion
    from kaboat_perception.buoy_localize import lidar_cluster_points, lidar_points, BuoyGeometry
    from kaboat_perception.buoy_lidar_detector import BuoyLidarDetector

    now = [0.]
    real_monotonic = time.monotonic
    time.monotonic = lambda: now[0]
    rclpy.init(args=['--ros-args', '-r', 'gate_behavior:__node:=gate_follower', '--params-file',
                     str(ROOT/'src/kaboat_bringup/config/gate_params.yaml')])
    node = GateFollower()
    for name in ('cmd_pub', 'complete_pub', 'path_pub', 'pair_pub', 'status_pub', 'preview_pub'):
        getattr(node, name).publish = lambda msg: None
    node.get_logger = lambda: Quiet()
    # Debug marker construction does not affect baseline decisions.
    node._publish_debug = lambda: None
    detector = BuoyLidarDetector()
    detector.get_logger = lambda: Quiet()
    detector._wanted = lambda: True
    detector._publish_markers = lambda stamp: None
    detector._publish_debug = lambda: None
    detected = [[]]
    detector.pub.publish = lambda msg: detected.__setitem__(0, [(b.position.x, b.position.y) for b in msg.buoys])
    models = {}
    odom = grid = goal = None
    trial = -1
    snapshots, scans = [], []
    trials = []
    next_tick = None
    last_save = -1e9
    max_t = dt.datetime(2026, 11, 12, *map(int, args.end.split(':')), tzinfo=KST).timestamp()
    names = ['/odom', '/scan/level', '/occupancy_grid', '/mission/state', '/mission/goal',
             '/imu/data', '/camera/camera_info', '/camera/camera/color/image_raw/compressed']
    try:
        for t, name, msg in messages(args.bag, names, end=max_t):
            now[0] = t
            if name == '/odom':
                odom = msg
                node._on_odom(msg)
                detector._on_odom(msg)
            elif name == '/imu/data':
                detector._on_imu(msg)
            elif name == '/camera/camera_info':
                detector._on_info(msg)
            elif name == '/camera/camera/color/image_raw/compressed':
                detector._on_compressed(msg)
            elif name == '/mission/state':
                if msg.data == 'gate':
                    if trials:
                        trials[-1]['baseline_passed'] = list(node._passed_midpoints)
                    trial += 1
                    trials.append(dict(trial=trial+1, start=t))
                    node._on_state(String(data='idle'))
                    node._on_state(msg)
                    models = {source: {mode: WallModel(mode, args.discount)
                              for mode in ('heading', 'waypoint', 'adaptive')}
                              for source in ('grid', 'detector', 'detector_unseen')}
                    # Mimic a subscriber arriving on activation; no pre-mission map.
                    detector.tracker.reset()
                    detected[0] = []
                    next_tick = t
            elif name == '/mission/goal':
                goal = (msg.pose.position.x, msg.pose.position.y)
                node._on_goal(msg)
            elif name == '/scan/level':
                node._on_scan(msg)
                if trial >= 0:
                    detector._on_scan(msg)
                if odom is not None and trial >= 0:
                    p = odom.pose.pose
                    yaw = yaw_from_quaternion(p.orientation)
                    g = BuoyGeometry()
                    clusters = lidar_cluster_points(lidar_points(msg.ranges, msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, g), g, msg.angle_increment)
                    c, s = math.cos(yaw), math.sin(yaw)
                    xy = [[p.position.x+c*v[0]-s*v[1], p.position.y+s*v[0]+c*v[1]]
                          for cl, v, r in clusters if len(cl) >= 3 and 2*r >= .1
                          and 1.5 <= math.hypot(v[0]-.4, v[1]) <= 15]
                    scans.append(dict(t=t, trial=trial+1, xy=xy))
            elif name == '/occupancy_grid':
                grid = msg
                node._on_grid(msg)
                if odom is None or goal is None or trial < 0:
                    continue
                p = odom.pose.pose
                boat = (p.position.x, p.position.y)
                yaw = yaw_from_quaternion(p.orientation)
                posts = grid_posts(msg, GateParams())
                sources = dict(grid=posts, detector=detected[0],
                    detector_unseen=[(tr.x, tr.y) for tr in detector.tracker.confirmed()])
                results = {}
                for source, source_models in models.items():
                    points = sources[source]
                    seed = select_gate_pair_from_posts(points, boat, goal, [], GateParams())
                    if seed is not None and not pair_observed(seed, node._scan_hits, GateParams()):
                        seed = None
                    results[source] = {mode: model.update(points, boat, yaw, goal, t, seed=seed)
                                       for mode, model in source_models.items()}
                if t-last_save >= .19:
                    snapshots.append(dict(t=t, trial=trial+1, boat=list(boat), yaw=yaw, goal=list(goal),
                        posts=posts, sources=sources, walls=results, baseline_status=node._status_text,
                        baseline_passed=len(node._passed_midpoints),
                        baseline_path=node._path.samples[::4].tolist() if node._path else [],
                        baseline_pair=list(node._pair.midpoint) if node._pair else None))
                    last_save = t
            if next_tick is not None and t >= next_tick:
                node._tick()
                next_tick = t+.1
        if trials:
            trials[-1]['baseline_passed'] = list(node._passed_midpoints)
    finally:
        node.destroy_node()
        detector.destroy_node()
        rclpy.shutdown()
        time.monotonic = real_monotonic
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with gzip.open(out/'replay.json.gz', 'wt') as f:
        json.dump(dict(commit='fe75c12', discount=args.discount, snapshots=snapshots, scans=scans, trials=trials), f)
    print(json.dumps(dict(snapshots=len(snapshots), scan_frames=len(scans), trials=trials), indent=2), flush=True)


if __name__ == '__main__':
    main()
