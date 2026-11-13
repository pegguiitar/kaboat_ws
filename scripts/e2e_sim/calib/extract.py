"""bag → 작은 토픽만 pickle 캐시 (스캔은 float32 npz)."""
import sys, pickle, math, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
STR = ['/mission/state', '/mission/complete', '/gate/status', '/station/status', '/dock/status',
       '/waypoint/status', '/teleop/status', '/teleop/event', '/teleop/esp_mode', '/dock/detection_status',
       '/dock/lidar/status', '/mission_points/result', '/dock/one_shot_gps', '/gnss/rtk_status',
       '/dock/avoid/goal_status', '/search/status']
TW = ['/cmd_vel', '/cmd/approach', '/cmd/gate', '/cmd/dock', '/cmd/station', '/cmd/search', '/cmd/avoid']
OTHER = ['/odom', '/rosout', '/gps/fix', '/tf_static', '/teleop/levels', '/teleop/enable', '/detections/buoys',
         '/mission/goal', '/mission/entry_goal', '/waypoint/goal', '/dock/one_shot_path', '/imu/data',
         '/scan', '/scan/level', '/scan/raw', '/mission_points/markers', '/dock/slot_markers', '/gate/pairs',
         '/station/markers', '/search/markers', '/camera/camera_info', '/ekf/dual_antenna_heading',
         '/detector/enable', '/gnss_1/velocity']
want = set(STR + TW + OTHER)
info = rosbag2_py.Info().read_metadata(bag, 'sqlite3')
comp = info.compression_format
so = rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3')
co = rosbag2_py.ConverterOptions('', '')
if comp:
    r = rosbag2_py.SequentialCompressionReader()
else:
    r = rosbag2_py.SequentialReader()
r.open(so, co)
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want &= set(types)
r.set_filter(rosbag2_py.StorageFilter(topics=sorted(want)))
cls = {n: get_message(types[n]) for n in want}
D = {n: [] for n in want}
scans = {n: {'t': [], 'ts': [], 'r': [], 'meta': None} for n in ('/scan', '/scan/level', '/scan/raw') if n in want}
def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
n = 0
imu_k = 0
caminfo_done = False
while r.has_next():
    name, data, t = r.read_next()
    t *= 1e-9
    if name == '/camera/camera_info':
        if caminfo_done: continue
        caminfo_done = True
    if name == '/imu/data':
        imu_k += 1
        if imu_k % 5: continue
    m = deserialize_message(data, cls[name])
    n += 1
    if name in STR:
        D[name].append((t, m.data))
    elif name in TW:
        D[name].append((t, m.linear.x, m.angular.z))
    elif name == '/odom':
        p = m.pose.pose
        D[name].append((t, stamp(m.header), p.position.x, p.position.y, yaw(p.orientation),
                        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z, m.header.frame_id, m.child_frame_id))
    elif name == '/rosout':
        D[name].append((t, m.name, m.level, m.msg))
    elif name == '/gps/fix':
        D[name].append((t, m.latitude, m.longitude, m.altitude, m.status.status, m.position_covariance[0]))
    elif name == '/tf_static':
        for tr in m.transforms:
            q = tr.transform.rotation; v = tr.transform.translation
            D[name].append((tr.header.frame_id, tr.child_frame_id, v.x, v.y, v.z, q.x, q.y, q.z, q.w))
    elif name == '/teleop/levels':
        D[name].append((t, tuple(m.data)))
    elif name in ('/teleop/enable', '/detector/enable'):
        D[name].append((t, m.data))
    elif name == '/detections/buoys':
        D[name].append((t, [(b.id, b.color, b.position.x, b.position.y, b.confidence) for b in m.buoys]))
    elif name in ('/mission/goal', '/mission/entry_goal', '/waypoint/goal'):
        D[name].append((t, m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation), m.header.frame_id))
    elif name == '/dock/one_shot_path':
        D[name].append((t, m.header.frame_id, [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
    elif name == '/imu/data':
        q = m.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
        D[name].append((t, roll, pitch, yaw(q), m.angular_velocity.z, m.linear_acceleration.x))
    elif name in scans:
        s = scans[name]
        if s['meta'] is None:
            s['meta'] = dict(angle_min=m.angle_min, angle_max=m.angle_max, inc=m.angle_increment, rmin=m.range_min,
                             rmax=m.range_max, frame=m.header.frame_id, n=len(m.ranges), scan_time=m.scan_time,
                             time_increment=m.time_increment, has_int=len(m.intensities))
        if len(m.ranges) == s['meta']['n']:
            s['t'].append(t); s['ts'].append(stamp(m.header)); s['r'].append(np.asarray(m.ranges, np.float32))
    elif name in ('/mission_points/markers', '/dock/slot_markers', '/gate/pairs', '/station/markers', '/search/markers'):
        mk = []
        for k in m.markers:
            mk.append(dict(ns=k.ns, id=k.id, type=k.type, action=k.action, frame=k.header.frame_id,
                           pos=(k.pose.position.x, k.pose.position.y, k.pose.position.z), yaw=yaw(k.pose.orientation),
                           scale=(k.scale.x, k.scale.y, k.scale.z), text=k.text,
                           color=(k.color.r, k.color.g, k.color.b, k.color.a),
                           pts=[(p.x, p.y) for p in k.points]))
        D[name].append((t, mk))
    elif name == '/camera/camera_info':
        D[name].append(dict(w=m.width, h=m.height, k=list(m.k), d=list(m.d), model=m.distortion_model, frame=m.header.frame_id))
    elif name == '/ekf/dual_antenna_heading':
        D[name].append((t, yaw(m.pose.pose.orientation)))
    elif name == '/gnss_1/velocity':
        D[name].append((t, m.twist.twist.linear.x, m.twist.twist.linear.y))
for nme, s in scans.items():
    np.savez_compressed(out + nme.replace('/', '_') + '.npz', t=np.array(s['t']), ts=np.array(s['ts']),
                        r=np.array(s['r'], np.float32), **{k: v for k, v in s['meta'].items()} if s['meta'] else {})
    D.pop(nme, None)
pickle.dump(D, open(out + '.pkl', 'wb'))
print(bag, 'msgs', n, {k: len(v) for k, v in D.items()})
