"""Publish recorded experimental hypotheses to RViz. No command publishers."""
import argparse
import datetime as dt
import gzip
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[3]
KST = dt.timezone(dt.timedelta(hours=9))
LEFT, RIGHT = (1., .3, .75), (.2, .85, 1.)


class Preview(Node):
    def __init__(self, frames, reference, rate=1., loop=False, checkpoint=None):
        super().__init__('gate_wall_experiment_preview')
        self.frames, self.reference = frames, reference
        self.rate, self.loop, self.checkpoint = rate, loop, checkpoint
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/gate_wall_experiment/markers', qos)
        self.tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'world', 'odom'
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        self.wall_start, self.idx = time.monotonic(), 0
        self.create_timer(.1, self.tick)

    def marker(self, name, kind, color=(1., 1., 1.), width=.08, alpha=1., z=.1):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = name, 0
        m.type, m.action = kind, Marker.ADD
        m.pose.orientation.w = 1.
        m.pose.position.z = z
        m.scale.x = m.scale.y = m.scale.z = width
        m.color.r, m.color.g, m.color.b = color
        m.color.a = alpha
        return m

    def line(self, name, points, color, width=.08, dashed=False, alpha=1.):
        m = self.marker(name, Marker.LINE_LIST if dashed else Marker.LINE_STRIP, color, width, alpha)
        if dashed:
            points = [p for i in range(0, len(points)-1, 4) for p in points[i:i+2]]
        m.points = [Point(x=float(x), y=float(y), z=0.) for x, y in points]
        return m

    def tick(self):
        if self.checkpoint is not None:
            snap = self.frames[self.checkpoint]
        else:
            elapsed = (time.monotonic()-self.wall_start)*self.rate
            duration = self.frames[-1]['t']-self.frames[0]['t']
            target = self.frames[0]['t']+(elapsed % duration if self.loop and duration > 0 else elapsed)
            if self.idx >= len(self.frames)-1 or self.frames[self.idx]['t'] > target:
                self.idx = 0
            while self.idx+1 < len(self.frames) and self.frames[self.idx+1]['t'] <= target:
                self.idx += 1
            snap = self.frames[self.idx]
        arr = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for source, dashed in [('grid', False), ('detector', True)]:
            w = snap['walls'][source]['adaptive']
            for side, name in enumerate(('left', 'right')):
                if len(w[name]) >= 2:
                    arr.markers.append(self.line(source+'_'+name, w[name], (LEFT, RIGHT)[side],
                        width=.12 if source == 'grid' else .07, dashed=dashed,
                        alpha=1. if w['status'] == 'TRACK' else .35))
            if len(w['center']) >= 2:
                arr.markers.append(self.line(source+'_center', w['center'], (.4, 1., .45), .045, dashed, .6))
            pts = self.marker(source+'_posts', Marker.SPHERE_LIST,
                              (.75, .75, .75) if source == 'grid' else (1., .85, .2),
                              width=.18 if source == 'grid' else .25, alpha=.85)
            pts.points = [Point(x=float(x), y=float(y), z=.1) for x, y in snap['sources'][source]]
            arr.markers.append(pts)
        path = snap['baseline_path']
        if len(path) >= 2:
            arr.markers.append(self.line('competition_path', path, (1., 1., 1.), .10))
        ref = self.marker('reference_posts_evaluation_only', Marker.CYLINDER_LIST if hasattr(Marker, 'CYLINDER_LIST') else Marker.SPHERE_LIST,
                          (.4, .4, .4), .35, .45)
        ref.points = [Point(x=float(x), y=float(y), z=0.) for gate in self.reference for x, y in gate]
        arr.markers.append(ref)
        x, y = snap['boat']
        boat = self.marker('recorded_boat', Marker.ARROW, (.2, .45, 1.), .2)
        boat.scale.x, boat.scale.y, boat.scale.z = .25, .5, .4
        boat.points = [Point(x=x, y=y, z=.15),
                       Point(x=x+1.4*math.cos(snap['yaw']), y=y+1.4*math.sin(snap['yaw']), z=.15)]
        arr.markers.append(boat)
        stamp = dt.datetime.fromtimestamp(snap['t'], KST).strftime('%H:%M:%S.%f')[:10]
        text = self.marker('legend', Marker.TEXT_VIEW_FACING, (1., 1., 1.), .40)
        text.pose.position.x, text.pose.position.y = 120., -9.
        text.text = ('BAG REPLAY / EXPERIMENT ONLY\n'+stamp+'  Competition: '+str(snap['baseline_status'])
            +'\nGRID solid / DETECTOR dashed | left pink / right cyan'
            +'\nGRID '+snap['walls']['grid']['adaptive']['status']
            +'  DETECTOR '+snap['walls']['detector']['adaptive']['status'])
        arr.markers.append(text)
        self.pub.publish(arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('replay')
    ap.add_argument('--trial', type=int, default=4)
    ap.add_argument('--rate', type=float, default=1.)
    ap.add_argument('--loop', action='store_true')
    ap.add_argument('--at', help='Hold nearest recorded KST HH:MM:SS frame')
    a = ap.parse_args()
    data = json.load(gzip.open(a.replay, 'rt'))
    frames = [s for s in data['snapshots'] if s['trial'] == a.trial]
    del data
    reference = json.load(open(ROOT/'scripts/gate_sim/gates_20261112.json'))['gates']
    at = None
    if a.at:
        t = dt.datetime(2026, 11, 12, *map(int, a.at.split(':')), tzinfo=KST).timestamp()
        at = min(range(len(frames)), key=lambda k: abs(frames[k]['t']-t))
    rclpy.init()
    node = Preview(frames, reference, a.rate, a.loop, at)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
