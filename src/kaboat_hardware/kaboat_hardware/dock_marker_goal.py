"""Bridge LiDAR-localized marker and wall normal to the docking goal."""
import json
import math
import time

import rclpy
import numpy as np
from geometry_msgs.msg import PointStamped, Vector3Stamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_behaviors.dock_marker_goal import require_interior, TimedGoalAverage
from kaboat_behaviors.dock_entry_search import find_docking_entry
from kaboat_behaviors.bspline_planner import PlannerParams
from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_behaviors.behavior_base import yaw_from_quaternion

CLASSES = ('triangle', 'circle', 'square')


class DockMarkerGoal(Node):
    def __init__(self):
        super().__init__('dock_marker_goal')
        for name, default in {'target_class': 'triangle', 'offset_m': 1.0,
                              'marker_timeout_s': 0.7, 'dock_stop_distance_m': 1.0,
                              'approach_extra_m': 1.0, 'align_extra_m': 0.5,
                              'tank_bounds': [0., 0., 10., 5.],
                              'boundary_margin_m': 0.5, 'goal_average_window_s': 0.0,
                              'goal_max_std_m': 0.25,'adaptive_entry_enabled':False,
                              'entry_step_m':.15,'entry_search_radius_m':1.5,'approach_only_entry_search':False}.items():
            self.declare_parameter(name, default)
        self.adaptive_entry=bool(self.get_parameter('adaptive_entry_enabled').value)
        self.approach_only_entry_search=bool(self.get_parameter('approach_only_entry_search').value)
        self.entry_step=float(self.get_parameter('entry_step_m').value)
        self.entry_radius=float(self.get_parameter('entry_search_radius_m').value)
        if not all(math.isfinite(v) and v>0 for v in (self.entry_step,self.entry_radius)):
            raise ValueError('entry search step/radius must be positive')
        self.previous_entry_shift=None;self.grid=None;self.grid_received=None
        self.create_subscription(OccupancyGrid,'/occupancy_grid',self._on_grid,1)
        self.target = str(self.get_parameter('target_class').value)
        if self.target not in CLASSES:
            raise ValueError('target_class must be triangle, circle or square')
        self.offset = float(self.get_parameter('offset_m').value)
        self.timeout = float(self.get_parameter('marker_timeout_s').value)
        self.stop_distance=float(self.get_parameter('dock_stop_distance_m').value)
        self.approach_extra=float(self.get_parameter('approach_extra_m').value)
        self.align_extra=float(self.get_parameter('align_extra_m').value)
        if (not all(math.isfinite(v) for v in (self.stop_distance,self.approach_extra,self.align_extra))
                or self.stop_distance<=0 or not self.approach_extra>self.align_extra>0):
            raise ValueError('invalid docking waypoint distances')
        self.bounds = tuple(float(v) for v in self.get_parameter('tank_bounds').value)
        self.margin = float(self.get_parameter('boundary_margin_m').value)
        if (len(self.bounds) != 4 or not all(math.isfinite(v) for v in self.bounds)
                or self.bounds[2] <= self.bounds[0] or self.bounds[3] <= self.bounds[1]):
            raise ValueError('tank_bounds must be a finite rectangle')
        if not all(math.isfinite(v) for v in (self.offset, self.margin)):
            raise ValueError('goal geometry limits must be finite')
        if self.offset <= 0 or self.margin < 0:
            raise ValueError('invalid goal geometry limits')
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('marker_timeout_s must be positive')
        self.average = TimedGoalAverage(
            window_s=float(self.get_parameter('goal_average_window_s').value),
            max_std_m=float(self.get_parameter('goal_max_std_m').value))
        self.normal_average=TimedGoalAverage(
            window_s=float(self.get_parameter('goal_average_window_s').value),
            max_std_m=math.sin(math.radians(8)))
        self.waypoints_pub=self.create_publisher(Path,'/dock/waypoints',10)
        self.point_pubs={name:self.create_publisher(PointStamped,f'/dock/{name}_point',qos)
                         for name in ('approach','alignment','stop')}
        self.points = {}
        self.normals = {}
        self.lidar_status = {}
        self.lidar_fits = {}
        self.odom = None
        self.odom_received = None
        self.candidate_pub = self.create_publisher(PointStamped, '/dock/avoid/goal_candidate', qos)
        self.goal_pub = self.create_publisher(PointStamped, '/dock/avoid/goal', qos)
        self.status_pub = self.create_publisher(String, '/dock/avoid/goal_status', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/dock/avoid/goal_markers', qos)
        for name in CLASSES:
            self.create_subscription(PointStamped, f'/dock/lidar/{name}/wall_point',
                                     lambda m, name=name: self._on_marker(name, m), qos)
        for name in CLASSES:
            self.create_subscription(Vector3Stamped, f'/dock/lidar/{name}/normal',
                                     lambda m, name=name: self._on_normal(name, m), qos)
        self.create_subscription(String, '/dock/lidar/status', self._on_lidar_status, qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_timer(0.1, self._tick)

    def _on_grid(self,msg):
        self.grid=msg;self.grid_received=time.monotonic()

    def _on_odom(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()

    def _on_marker(self, name, msg):
        if (msg.header.frame_id != 'odom' or abs(msg.point.z) > 1e-6
                or not all(math.isfinite(v) for v in (msg.point.x, msg.point.y, msg.point.z))):
            self.points.pop(name, None)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        previous = self.points.get(name)
        if previous is not None:
            old = previous[0].header.stamp
            if stamp <= old.sec + old.nanosec * 1e-9:
                return
        self.points[name] = (msg, time.monotonic())

    def _on_normal(self, name, msg):
        vector = msg.vector
        length = math.hypot(vector.x, vector.y)
        if (msg.header.frame_id != 'odom' or abs(vector.z)>1e-6
                or not all(math.isfinite(v) for v in (vector.x,vector.y,vector.z))
                or abs(length-1)>1e-3):
            self.normals.pop(name,None)
            return
        previous=self.normals.get(name)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if previous is not None:
            old=previous[0].header.stamp
            if stamp<=old.sec+old.nanosec*1e-9:return
        self.normals[name]=(msg,time.monotonic())

    def _on_lidar_status(self,msg):
        try:
            self.lidar_status=json.loads(msg.data)
            for name,fit in self.lidar_status.get('markers',{}).items():
                if fit.get('reason')=='ok':self.lidar_fits[name]=(self.lidar_status.get('scan_stamp_s'),fit)
        except (ValueError,TypeError):self.lidar_status={}

    def _tick(self):
        now,ros_now=time.monotonic(),self.get_clock().now().nanoseconds*1e-9
        fresh={name:msg for name,(msg,received) in self.points.items()
               if now-received<=self.timeout and 0<msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
               and -.1<=ros_now-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)<=self.timeout}
        info=None;normal_info=None;candidate=None;waypoints=None;wall=None;reference_xy=None;geometry=False;entry_info=None;path_emitted=False
        try:
            if self.odom is None or now-self.odom_received>self.timeout or self.odom.header.frame_id!='odom':
                raise ValueError('waiting for fresh odom')
            if self.target not in fresh:raise ValueError('waiting for target lidar wall projection')
            normal_msg,received=self.normals.get(self.target,(None,0))
            if normal_msg is None or now-received>self.timeout:raise ValueError('waiting for fresh lidar wall normal')
            point=fresh[self.target]
            if normal_msg.header.stamp!=point.header.stamp:raise ValueError('lidar normal and marker measurement stamps differ')
            normal=np.array([normal_msg.vector.x,normal_msg.vector.y])
            wall=np.array([point.point.x,point.point.y]);candidate=wall+self.stop_distance*normal
            raw=PointStamped();raw.header=point.header;raw.point.x,raw.point.y=map(float,candidate)
            self.candidate_pub.publish(raw)
            stamp=point.header.stamp.sec+point.header.stamp.nanosec*1e-9
            mean_wall,info=self.average.update(wall,stamp)
            mean_normal,normal_info=self.normal_average.update(normal,stamp)
            geometry=True
            if not info['ready']:raise ValueError(info['reason'])
            if not normal_info['ready']:raise ValueError('wall normal averaging is unstable or collecting')
            norm=float(np.linalg.norm(mean_normal))
            if norm<.9:raise ValueError('wall normal directions disagree')
            normal=mean_normal/norm
            reference_xy=mean_wall
            distances=np.array([self.stop_distance+self.approach_extra,
                                self.stop_distance+self.align_extra,self.stop_distance])
            waypoints=mean_wall+distances[:,None]*normal
            heading=math.atan2(-normal[1],-normal[0])
            if self.adaptive_entry:
                if (self.grid is None or now-self.grid_received>.5 or self.grid.header.frame_id!='odom'):
                    raise ValueError('waiting for fresh occupancy grid for entry search')
                if (self.grid.info.resolution<=0 or len(self.grid.data)!=self.grid.info.width*self.grid.info.height):
                    raise ValueError('invalid occupancy grid for entry search')
                pose=self.odom.pose.pose;boat=(pose.position.x,pose.position.y)
                yaw=yaw_from_quaternion(pose.orientation);u=self.odom.twist.twist.linear
                velocity=(math.cos(yaw)*u.x-math.sin(yaw)*u.y,math.sin(yaw)*u.x+math.cos(yaw)*u.y)
                if not all(math.isfinite(v) for v in (*boat,yaw,*velocity)):
                    raise ValueError("invalid odometry for entry search")
                dri=build_dri(self.grid,boat,yaw,DriParams())
                waypoints,_,entry_info=find_docking_entry(dri,boat,yaw,velocity,waypoints,heading,
                    PlannerParams(finite_goal=True),step=self.entry_step,radius=self.entry_radius,
                    previous_shift=self.previous_entry_shift,bounds=self.bounds,margin=self.margin,
                    approach_only=self.approach_only_entry_search)
                if entry_info['safe_path']:self.previous_entry_shift=entry_info['shift_m']
            path=Path();path.header=point.header
            for name,xy in zip(('approach','alignment','stop'),waypoints):
                pose=PoseStamped();pose.header=path.header
                pose.pose.position.x,pose.pose.position.y=map(float,xy)
                pose.pose.orientation.z=math.sin(heading/2);pose.pose.orientation.w=math.cos(heading/2)
                path.poses.append(pose)
                m=PointStamped();m.header=path.header;m.point.x,m.point.y=map(float,xy)
                self.point_pubs[name].publish(m)
            self.waypoints_pub.publish(path)
            path_emitted=True
            goal=PointStamped();goal.header=path.header;goal.point.x,goal.point.y=map(float,waypoints[-1])
            self.goal_pub.publish(goal)
            boundary_error=None
            try:
                for xy in waypoints:require_interior(xy,self.bounds,self.margin)
            except ValueError as exc:boundary_error=str(exc)
            fit_stamp,fit=self.lidar_fits.get(self.target,(None,{}))
            if fit_stamp is None or abs(fit_stamp-stamp)>1e-5:fit={}
            path_ready=entry_info is None or entry_info['safe_path']
            status=dict(valid=boundary_error is None and path_ready,
                reason=(boundary_error+'; preview only, arming remains blocked' if boundary_error
                        else entry_info['reason'] if entry_info and not path_ready else 'ok'),
                geometry_valid=True,motion_bounds_valid=boundary_error is None,
                geometry_source='lidar_ransac',normal_source=fit.get('source'),normal_shared=fit.get('shared'),
                wall_xy=mean_wall.tolist(),outward=normal.tolist(),goal_xy=waypoints[-1].tolist(),
                approach_xy=waypoints[0].tolist(),alignment_xy=waypoints[1].tolist(),
                candidate_xy=candidate.tolist(),heading_rad=heading,
                stop_distance_m=self.stop_distance,wall_rms_m=fit.get('wall_rms_m'),wall_span_m=fit.get('wall_span_m'))
        except ValueError as exc:
            status=dict(valid=False,reason=str(exc),candidate_xy=candidate.tolist() if candidate is not None else None)
        if not path_emitted:
            clear=Path();clear.header.frame_id='odom';clear.header.stamp=self.get_clock().now().to_msg()
            self.waypoints_pub.publish(clear)
        if not geometry:
            self.average.reset();self.normal_average.reset();self.previous_entry_shift=None
        status.update(target_class=self.target,fresh_markers=sorted(fresh),averaging=info,
                      normal_averaging=normal_info,average_window_s=self.average.window,entry_search=entry_info)
        self.status_pub.publish(String(data=json.dumps(status)))
        array=MarkerArray();labels=[]
        colors=((0.,.8,1.),(1.,.5,0.),(1.,1.,0.))
        offsets=([float(np.linalg.norm(xy-reference_xy)) for xy in waypoints]
                 if waypoints is not None and reference_xy is not None else
                 (self.stop_distance+self.approach_extra,self.stop_distance+self.align_extra,self.stop_distance))
        for i,name in enumerate(('APPROACH','ALIGN','STOP')):
            m=Marker();m.header.frame_id='odom';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='dock_goal';m.id=i;m.type=Marker.SPHERE;m.action=Marker.ADD if waypoints is not None else Marker.DELETE
            m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=.2;m.color.a=1.;m.lifetime.sec=1
            m.color.r,m.color.g,m.color.b=colors[i] if status['valid'] else (1.,0.,0.)
            if waypoints is not None:m.pose.position.x,m.pose.position.y=map(float,waypoints[i])
            array.markers.append(m)
            label=Marker();label.header=m.header;label.ns=m.ns;label.id=i+3;label.type=Marker.TEXT_VIEW_FACING
            label.action=m.action;label.pose.orientation.w=1.;label.pose.position.x=m.pose.position.x
            label.pose.position.y=m.pose.position.y;label.pose.position.z=.3
            label.scale.z=.18;label.color=m.color;label.lifetime.sec=1
            label.text=f'{name} +{offsets[i]:.1f}m';labels.append(label)
        array.markers.extend(labels)
        for text in (False,True):
            m=Marker();m.header.frame_id='odom';m.header.stamp=self.get_clock().now().to_msg()
            m.ns='dock_goal';m.id=6+int(text);m.type=Marker.TEXT_VIEW_FACING if text else Marker.SPHERE
            m.action=Marker.ADD if reference_xy is not None else Marker.DELETE;m.pose.orientation.w=1.
            m.scale.x=m.scale.y=m.scale.z=.16;m.color.r=m.color.g=m.color.b=m.color.a=1.;m.lifetime.sec=1
            if reference_xy is not None:m.pose.position.x,m.pose.position.y=map(float,reference_xy)
            m.pose.position.z=.3 if text else 0.;m.text='WALL';array.markers.append(m)
        self.viz_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = DockMarkerGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
