"""Live sensor replay: two-wall path previews and unchanged gate comparison."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from kaboat_msgs.msg import BuoyArray
from kaboat_behaviors.gate_corridor import (GateParams, grid_posts,
    select_gate_pair_from_posts, pair_observed, scan_hits_world)
from wall_model import WallModel


class LiveWallPath(Node):
    def __init__(self):
        super().__init__('live_wall_path_preview')
        self.models = {k: WallModel('waypoint') for k in ('grid', 'detector')}
        self.odom = self.grid = self.hits = None
        self.detected = []
        self.goal = (131.569, -22.083)  # Recorded gate_end for this selected bag run.
        self.prev = None
        self.reset_pending = True
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/mission/state', latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/mission/goal', latched)
        self.paths = {k: self.create_publisher(Path, f'/wall_corridor/{k}/path', 1)
                      for k in self.models}
        self.markers = self.create_publisher(MarkerArray, '/wall_corridor/markers', 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(OccupancyGrid, '/occupancy_grid', self.on_grid, 2)
        self.create_subscription(BuoyArray, '/detections/buoys', self.on_buoys, 10)
        self.create_subscription(LaserScan, '/scan/level', self.on_scan, qos_profile_sensor_data)
        self.create_timer(.5, self.publish_mission)

    def publish_mission(self):
        self.state_pub.publish(String(data='idle' if self.reset_pending else 'gate'))
        self.reset_pending = False
        msg = PoseStamped()
        msg.header.frame_id = 'odom'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.orientation.w = 1.
        msg.pose.position.x, msg.pose.position.y = self.goal
        self.goal_pub.publish(msg)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        xy = np.array([p.x, p.y])
        if self.prev is not None and np.linalg.norm(xy-self.prev) > 5:
            self.models = {k: WallModel('waypoint') for k in self.models}
            self.state_pub.publish(String(data='idle'))
            self.reset_pending = True
        self.prev, self.odom = xy, msg

    def pose(self):
        p = self.odom.pose.pose
        q = p.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return (p.position.x, p.position.y), yaw

    def on_scan(self, msg):
        if self.odom:
            xy, yaw = self.pose()
            self.hits = scan_hits_world(msg, xy, yaw, (.4, 0.))

    def on_buoys(self, msg):
        self.detected = [(b.position.x, b.position.y) for b in msg.buoys]

    def on_grid(self, msg):
        self.grid = msg
        if self.odom is None or self.hits is None:
            return
        boat, yaw = self.pose()
        p = GateParams()
        arr = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; arr.markers.append(clear)
        statuses = []
        for source, points in [('grid', grid_posts(msg, p)), ('detector', self.detected)]:
            seed = select_gate_pair_from_posts(points, boat, self.goal, [], p)
            if seed is not None and not pair_observed(seed, self.hits, p): seed = None
            result = self.models[source].update(points, boat, yaw, self.goal,
                self.get_clock().now().nanoseconds/1e9, seed=seed)
            path = Path(); path.header.frame_id = 'odom'
            path.header.stamp = self.get_clock().now().to_msg()
            center = result['center']
            if center:
                u = np.linspace(-3., 6., len(center))
                # Display-only: include the initial line to the first observed pair.
                end = min(6., result.get('supported_end', result['boat_s'])-result['boat_s'])
                for (x, y), s in zip(center, u):
                    if -.1 <= s <= max(.8, end):
                        ps = PoseStamped(); ps.header = path.header; ps.pose.orientation.w = 1.
                        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                        ps.pose.position.z = .12
                        path.poses.append(ps)
                for bank, key in enumerate(('left', 'right')):
                    line = Marker(); line.header = path.header; line.ns = source+'_'+key
                    line.type, line.action = Marker.LINE_STRIP, Marker.ADD
                    line.pose.orientation.w = 1.; line.scale.x = .05
                    line.color.r, line.color.g, line.color.b = ((1., .3, .8) if bank == 0 else (.2, 1., .5))
                    line.color.a = .85 if result['status'] == 'TRACK' else .35
                    line.points = [Point(x=float(x), y=float(y), z=.12) for x, y in result[key]]
                    arr.markers.append(line)
            self.paths[source].publish(path)
            statuses.append(f'{source}: {result["status"]}')
        text = Marker(); text.header = msg.header; text.ns = 'legend'
        text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.orientation.w = 1.; text.pose.position.x = boat[0]
        text.pose.position.y = boat[1]+5.; text.pose.position.z = .2
        text.scale.z = .28; text.color.r = text.color.g = text.color.b = text.color.a = 1.
        text.text = 'LIVE BAG | white: competition | orange: buoy-wall | cyan: grid-wall\n'+' / '.join(statuses)
        arr.markers.append(text)
        self.markers.publish(arr)


def main():
    rclpy.init()
    n = LiveWallPath()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # A signal can close the context during a publisher callback.
        if rclpy.ok():
            raise
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__': main()
