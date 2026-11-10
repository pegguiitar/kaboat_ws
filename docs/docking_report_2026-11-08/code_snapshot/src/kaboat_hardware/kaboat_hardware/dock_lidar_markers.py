"""Camera sight rays + measured LiDAR lines -> depth-independent dock landmarks."""
from collections import deque
import json
import math
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Quaternion, Vector3Stamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray
from kaboat_behaviors.dock_lidar_wall import RayWall, RayWallParams, associate_walls
from kaboat_behaviors.slot_axis import scan_points

CLASSES=('triangle','circle','square')


def rotate(vector, q):
    v=np.asarray(vector,float);a=np.array([q.x,q.y,q.z])
    return v+2*np.cross(a,np.cross(a,v)+q.w*v)


def transform_point(xyz, tf):
    t=tf.translation
    return rotate(xyz,tf.rotation)+[t.x,t.y,t.z]


def transform_wall(wall, tf, inverse=False):
    q=tf.rotation
    if inverse:
        q=Quaternion(x=-q.x,y=-q.y,z=-q.z,w=q.w)
        center=rotate(np.array([*wall.center,0.])-[tf.translation.x,tf.translation.y,tf.translation.z],q)[:2]
    else:center=transform_point([*wall.center,0.],tf)[:2]
    tangent=rotate([*wall.tangent,0.],q)[:2];tangent/=np.linalg.norm(tangent)
    normal=rotate([*wall.normal,0.],q)[:2];normal/=np.linalg.norm(normal)
    return RayWall(tuple(center),tuple(tangent),tuple(normal),wall.low,wall.high,wall.rms,wall.points)


def stamp_s(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


class DockLidarMarkers(Node):
    def __init__(self):
        super().__init__('dock_lidar_markers')
        defaults={'marker_behind_wall_m':1.,'ray_scan_slop_s':.12,'input_timeout_s':.7,
                  'angular_window_deg':15.,'ransac_inlier_m':.06,'ransac_min_span_m':.65,
                  'ransac_min_points':12,'shared_extrapolation_m':2.,'base_frame':'base_link','target_class':'triangle',
                  'station_normal_hint_odom':[0.,0.], 'station_history_timeout_s':1.5}
        for k,v in defaults.items():self.declare_parameter(k,v)
        get=lambda k:self.get_parameter(k).value
        self.target=str(get('target_class'))
        if self.target not in CLASSES:raise ValueError('invalid target class')
        self.normal_hint=np.asarray(get('station_normal_hint_odom'),float)
        self.history_timeout=float(get('station_history_timeout_s'))
        if self.normal_hint.shape!=(2,) or not np.isfinite(self.normal_hint).all() or not math.isfinite(self.history_timeout) or self.history_timeout<=0:
            raise ValueError('invalid station wall prior')
        length=np.linalg.norm(self.normal_hint)
        self.normal_hint=None if length<1e-6 else self.normal_hint/length
        self.previous_world_wall=None;self.previous_wall_stamp=None
        self.behind=float(get('marker_behind_wall_m'));self.slop=float(get('ray_scan_slop_s'))
        self.timeout=float(get('input_timeout_s'));self.base=str(get('base_frame'))
        if not all(math.isfinite(v) and v>=0 for v in (self.behind,self.slop,self.timeout)) or not self.base:
            raise ValueError('invalid lidar marker settings')
        self.params=RayWallParams(angular_window=math.radians(float(get('angular_window_deg'))),
                                 inlier_distance=float(get('ransac_inlier_m')),
                                 min_span=float(get('ransac_min_span_m')),
                                 min_points=int(get('ransac_min_points')),
                                 shared_extrapolation=float(get('shared_extrapolation_m')))
        if (not 0<self.params.angular_window<math.pi/2 or self.params.inlier_distance<=0
                or self.params.min_span<=0 or self.params.min_points<3 or self.params.shared_extrapolation<0):
            raise ValueError('invalid RANSAC settings')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.rays={c:deque(maxlen=30) for c in CLASSES};self.scan=None;self.scans=deque(maxlen=20);self.last_scan=None;self.last_pair=None
        self.pubs={}
        self.alias_points={(c,frame):self.create_publisher(PointStamped,
            f'/dock/markers/{c}/'+('ground_base' if frame==self.base else 'ground'),qos)
            for c in CLASSES for frame in (self.base,'odom')}
        self.alias_viz={frame:self.create_publisher(MarkerArray,
            '/dock/markers_base' if frame==self.base else '/dock/markers_ground',qos)
            for frame in (self.base,'odom')}
        self.selected_pub=self.create_publisher(PointStamped,'/dock/marker_ground',qos)
        for c in CLASSES:
            self.create_subscription(PointStamped,f'/dock/markers/{c}/ray',lambda m,c=c:self._on_ray(c,m),qos)
            for suffix,kind in [('wall_point_base',PointStamped),('marker_base',PointStamped),
                                ('normal_base',Vector3Stamped),('wall_point',PointStamped),
                                ('marker_ground',PointStamped),('normal',Vector3Stamped)]:
                self.pubs[c,suffix]=self.create_publisher(kind,f'/dock/lidar/{c}/{suffix}',qos)
        self.create_subscription(LaserScan,'/scan',self._on_scan,qos)
        self.status_pub=self.create_publisher(String,'/dock/lidar/status',10)
        self.viz_pub=self.create_publisher(MarkerArray,'/dock/lidar/markers',qos)
        self.create_timer(.1,self._tick)

    def _on_ray(self,c,m):
        if (m.header.frame_id and stamp_s(m)>0
                and all(math.isfinite(v) for v in (m.point.x,m.point.y,m.point.z))
                and np.linalg.norm([m.point.x,m.point.y,m.point.z])>1e-6):
            if not self.rays[c] or stamp_s(m)>stamp_s(self.rays[c][-1]):self.rays[c].append(m)

    def _on_scan(self,m):
        if m.header.frame_id and stamp_s(m)>0:
            if self.scan is None or stamp_s(m)>stamp_s(self.scan):
                self.scan=m;self.scans.append(m)

    def _point(self,c,suffix,xyz,stamp,frame):
        m=PointStamped();m.header.stamp=stamp;m.header.frame_id=frame
        m.point.x,m.point.y,m.point.z=map(float,xyz);self.pubs[c,suffix].publish(m)

    def _normal(self,c,suffix,normal,stamp,frame):
        m=Vector3Stamped();m.header.stamp=stamp;m.header.frame_id=frame
        m.vector.x,m.vector.y,m.vector.z=map(float,normal);self.pubs[c,suffix].publish(m)

    def _publish_aliases(self, observations, stamp, world_tf):
        colors={'triangle':(1.,.1,.1),'circle':(.1,1.,.1),'square':(.1,.3,1.)}
        for frame in (self.base,'odom'):
            array=MarkerArray()
            for i,c in enumerate(CLASSES):
                ob=observations.get(c);xyz=None
                if ob is not None and (frame==self.base or world_tf is not None):
                    xy=np.asarray(ob.point)-self.behind*np.asarray(ob.wall.normal)
                    xyz=np.array([*xy,0.])
                    if frame=='odom':xyz=transform_point(xyz,world_tf);xyz[2]=0.
                    point=PointStamped();point.header.frame_id=frame;point.header.stamp=stamp
                    point.point.x,point.point.y,point.point.z=map(float,xyz)
                    self.alias_points[c,frame].publish(point)
                    if c==self.target and frame=='odom':self.selected_pub.publish(point)
                for text in (False,True):
                    m=Marker();m.header.frame_id=frame;m.header.stamp=stamp;m.ns='dock_markers';m.id=i*2+int(text)
                    m.action=Marker.ADD if xyz is not None else Marker.DELETE;m.pose.orientation.w=1.
                    m.type=Marker.TEXT_VIEW_FACING if text else Marker.SPHERE
                    m.scale.x=m.scale.y=m.scale.z=.18;m.lifetime.nanosec=500000000
                    m.color.r,m.color.g,m.color.b=(1.,0.,1.) if c==self.target else colors[c];m.color.a=1.
                    m.text=c+' (lidar)'
                    if xyz is not None:
                        m.pose.position.x,m.pose.position.y=map(float,xyz[:2]);m.pose.position.z=.25 if text else 0.
                    array.markers.append(m)
            self.alias_viz[frame].publish(array)

    def _publish_visuals(self,observations,stamp):
        array=MarkerArray();colors={'triangle':(1.,.1,.1),'circle':(.1,1.,.1),'square':(.1,.3,1.)}
        for i,c in enumerate(CLASSES):
            observation=observations.get(c)
            for j in range(4):
                m=Marker();m.header.frame_id=self.base;m.header.stamp=stamp;m.ns='dock_lidar';m.id=i*4+j
                m.action=Marker.ADD if observation else Marker.DELETE;m.pose.orientation.w=1.
                m.lifetime.nanosec=500000000;m.color.r,m.color.g,m.color.b=colors[c];m.color.a=1.
                if observation:
                    hit=np.asarray(observation.point);normal=np.asarray(observation.wall.normal)
                    if j==0:
                        m.type=Marker.SPHERE;m.scale.x=m.scale.y=m.scale.z=.18
                        m.pose.position.x,m.pose.position.y=map(float,hit)
                    elif j==1:
                        m.type=Marker.ARROW;m.scale.x=.06;m.scale.y=.12;m.scale.z=.16
                        m.points=[Point(x=float(x),y=float(y),z=.05) for x,y in [hit,hit+normal*.8]]
                    elif j==2:
                        wall=observation.wall;m.type=Marker.LINE_STRIP;m.scale.x=.045
                        center=np.asarray(wall.center);tangent=np.asarray(wall.tangent)
                        m.points=[Point(x=float(x),y=float(y),z=.03) for x,y in [center+tangent*wall.low,center+tangent*wall.high]]
                    else:
                        m.type=Marker.TEXT_VIEW_FACING;m.scale.z=.17
                        m.pose.position.x,m.pose.position.y=map(float,hit);m.pose.position.z=.3
                        m.text=f'{c}: '+('shared '+observation.source if observation.shared else 'direct')
                array.markers.append(m)
        self.viz_pub.publish(array)

    def _tick(self):
        started=time.perf_counter();now=self.get_clock().now().nanoseconds*1e-9
        # YOLO arrives after inference. Match scan acquisition time to the image,
        # rather than comparing the delayed image against the newest scan.
        fresh_rays=[history[-1] for history in self.rays.values()
                    if history and -.1<=now-stamp_s(history[-1])<=self.timeout]
        anchor=max((stamp_s(m) for m in fresh_rays),default=None)
        available=[m for m in self.scans if -.1<=now-stamp_s(m)<=self.timeout]
        scan=(min(available,key=lambda m:abs(stamp_s(m)-anchor))
              if available and anchor is not None else self.scan)
        observations={};details={};reason='ok';world_tf=None;selection={}
        stamp=self.get_clock().now().to_msg() if scan is None else scan.header.stamp
        try:
            if scan is None or not -.1<=now-stamp_s(scan)<=self.timeout:
                raise ValueError('waiting for fresh scan')
            if anchor is None:raise ValueError('waiting for fresh YOLO sight ray')
            pair=(stamp_s(scan),anchor)
            if self.last_pair==pair:return
            scan_time=Time.from_msg(stamp)
            lidar_tf=self.tf.lookup_transform(self.base,scan.header.frame_id,scan_time).transform
            local=scan_points(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max)
            # The scan may have a nonzero mounting rotation as well as translation.
            xyz=np.c_[local,np.zeros(len(local))]
            points=transform_point(xyz,lidar_tf)[:, :2]
            rays={}
            for c,history in self.rays.items():
                if not history:details[c]={'reason':'no_detection','detected':False,'localized':False};continue
                ray=min(history,key=lambda m:abs(stamp_s(m)-stamp_s(scan)))
                delta=stamp_s(ray)-stamp_s(scan)
                age=now-stamp_s(ray)
                if not -.1<=age<=self.timeout:
                    details[c]={'reason':'detection_stale','detected':False,'localized':False,'ray_age_s':age};continue
                if abs(delta)>self.slop:
                    details[c]={'reason':'ray_scan_time_mismatch','detected':True,'localized':False,'delta_s':delta,'ray_age_s':age};continue
                try:
                    tf=self.tf.lookup_transform_full(self.base,scan_time,ray.header.frame_id,
                        Time.from_msg(ray.header.stamp),'odom').transform
                    compensated=True
                except TransformException:
                    # Base-only visualization remains available without world odometry.
                    # A tighter sync bound limits uncompensated motion.
                    if abs(delta)>.035:
                        details[c]={'reason':'motion_compensation_tf_unavailable','detected':True,'localized':False,'delta_s':delta};continue
                    tf=self.tf.lookup_transform(self.base,ray.header.frame_id,Time.from_msg(ray.header.stamp)).transform
                    compensated=False
                origin=transform_point([0,0,0],tf)[:2]
                direction=rotate([ray.point.x,ray.point.y,ray.point.z],tf.rotation)[:2]
                rays[c]=(origin,direction)
                details[c]={'ray_stamp_s':stamp_s(ray),'delta_s':delta,'ray_age_s':age,
                            'detected':True,'localized':False,'motion_compensated':compensated}
            try:world_tf=self.tf.lookup_transform('odom',self.base,scan_time).transform
            except TransformException:pass
            normal_hint=None;previous=None
            if self.normal_hint is not None:
                if world_tf is None:raise ValueError('station orientation prior needs odom TF')
                q=world_tf.rotation
                inverse=Quaternion(x=-q.x,y=-q.y,z=-q.z,w=q.w)
                normal_hint=rotate([*self.normal_hint,0.],inverse)[:2]
            if (world_tf is not None and self.previous_world_wall is not None
                    and 0<=stamp_s(scan)-self.previous_wall_stamp<=self.history_timeout):
                previous=transform_wall(self.previous_world_wall,world_tf,inverse=True)
            observations=associate_walls(points,rays,self.params,normal_hint=normal_hint,
                                         previous_wall=previous,selection=selection)
            self.last_scan=stamp_s(scan)
            self.last_pair=pair
            if observations and world_tf is not None:
                self.previous_world_wall=transform_wall(next(iter(observations.values())).wall,world_tf)
                self.previous_wall_stamp=stamp_s(scan)
            for c in rays:
                if c not in observations:
                    details[c]['reason']='no_supported_lidar_line';continue
                ob=observations[c];normal=np.asarray(ob.wall.normal);hit=np.asarray(ob.point)
                marker=hit-self.behind*normal
                for suffix,xy in [('wall_point_base',hit),('marker_base',marker)]:self._point(c,suffix,[*xy,0],stamp,self.base)
                self._normal(c,'normal_base',[*normal,0],stamp,self.base)
                if world_tf is not None:
                    for suffix,xy in [('wall_point',hit),('marker_ground',marker)]:
                        value=transform_point([*xy,0],world_tf);value[2]=0;self._point(c,suffix,value,stamp,'odom')
                    value=rotate([*normal,0],world_tf.rotation);value[2]=0;value/=np.linalg.norm(value)
                    self._normal(c,'normal',value,stamp,'odom')
                details[c].update(reason='ok',localized=True,source=ob.source,shared=ob.shared,
                    wall_rms_m=ob.wall.rms,wall_span_m=ob.wall.span,wall_points=ob.wall.points,
                    wall_point_base=ob.point,normal_base=ob.wall.normal,world_valid=world_tf is not None)
            if not observations:reason=selection.get('reason','no supported marker wall')
        except (ValueError,TransformException) as exc:reason=str(exc)
        self._publish_aliases(observations,stamp,world_tf)
        self._publish_visuals(observations,stamp)
        self.status_pub.publish(String(data=json.dumps({'valid':bool(observations),'reason':reason,
            'scan_stamp_s':stamp_s(scan) if scan else None,'marker_behind_wall_m':self.behind,
            'markers':details,'station_selection':selection,'processing_ms':(time.perf_counter()-started)*1000})))


def main(args=None):
    rclpy.init(args=args);node=DockLidarMarkers()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
