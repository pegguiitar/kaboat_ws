import json
import copy
from rclpy.time import Time
from unittest.mock import Mock
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from kaboat_hardware.dock_lidar_markers import DockLidarMarkers


@pytest.fixture
def lidar_node():
    rclpy.init();node=DockLidarMarkers()
    node.tf=Mock()
    tf=TransformStamped();tf.transform.rotation.w=1.
    node.tf.lookup_transform.return_value=tf;node.tf.lookup_transform_full.return_value=tf
    node.sent={k:[] for k in node.pubs};node.states=[]
    for k,p in node.pubs.items():p.publish=node.sent[k].append
    node.status_pub.publish=lambda m:node.states.append(json.loads(m.data))
    yield node
    node.destroy_node();rclpy.shutdown()


def inputs(node, classes=('triangle',)):
    stamp=node.get_clock().now().to_msg()
    for c in classes:
        m=PointStamped();m.header.frame_id='base_link';m.header.stamp=copy.deepcopy(stamp);m.point.x=1.
        node._on_ray(c,m)
    scan=LaserScan();scan.header.frame_id='base_link';scan.header.stamp=copy.deepcopy(stamp)
    scan.angle_min=-.6;scan.angle_increment=.002;scan.range_min=.1;scan.range_max=12.
    angles=scan.angle_min+np.arange(600)*scan.angle_increment
    scan.ranges=(5/np.cos(angles)).astype(float).tolist();node._on_scan(scan)


def test_single_visible_marker_generates_lidar_points_and_normal_without_depth(lidar_node):
    n=lidar_node;inputs(n);n._tick()
    point=n.sent['triangle','wall_point_base'][-1]
    marker=n.sent['triangle','marker_ground'][-1]
    normal=n.sent['triangle','normal'][-1]
    assert point.point.x==pytest.approx(5,abs=.001)
    assert marker.point.x==pytest.approx(6,abs=.001)  # inferred 1m behind wall
    assert normal.vector.x==pytest.approx(-1,abs=.001)
    assert marker.header.stamp==point.header.stamp==normal.header.stamp
    assert point.header.frame_id=='base_link' and marker.header.frame_id=='odom'
    assert n.states[-1]['markers']['triangle']['reason']=='ok'


def test_lost_scan_never_republishes_old_measurement(lidar_node):
    n=lidar_node;inputs(n);n._tick();n.scan.header.stamp.sec-=2;n._tick()
    assert len(n.sent['triangle','wall_point_base'])==1
    assert not n.states[-1]['valid']


def test_lost_detection_stops_points_even_when_scan_is_fresh(lidar_node):
    n=lidar_node;inputs(n);n.rays['triangle'].clear();n._tick()
    assert not n.sent['triangle','wall_point_base']
    assert not n.states[-1]['valid']


def test_missing_world_tf_still_publishes_base_geometry(lidar_node):
    n=lidar_node;inputs(n)
    original=n.tf.lookup_transform.return_value
    n.tf.lookup_transform.side_effect=lambda target,*args:(_ for _ in ()).throw(TransformException('no odom')) if target=='odom' else original
    n._tick()
    assert n.sent['triangle','wall_point_base']
    assert not n.sent['triangle','wall_point']
    assert not n.states[-1]['markers']['triangle']['world_valid']


def test_delayed_yolo_uses_scan_at_capture_time_instead_of_latest(lidar_node):
    n=lidar_node;inputs(n)
    captured=n.get_clock().now().nanoseconds-250_000_000
    stamp=Time(nanoseconds=captured).to_msg()
    n.rays['triangle'][-1].header.stamp=stamp
    n.scans[0].header.stamp=copy.deepcopy(stamp)
    old=n.scans[0]
    newer=copy.deepcopy(old);newer.header.stamp=n.get_clock().now().to_msg()
    # A newer scan has different geometry: matching it would also change the range.
    newer.ranges=[r*1.2 for r in newer.ranges]
    n._on_scan(newer)
    n._tick()
    point=n.sent['triangle','wall_point_base'][-1]
    assert point.header.stamp==stamp
    assert point.point.x==pytest.approx(5,abs=.001)
    assert abs(n.states[-1]['markers']['triangle']['delta_s'])<.001


def test_same_scan_image_pair_is_not_republished_as_a_new_measurement(lidar_node):
    n=lidar_node;inputs(n);n._tick();n._tick()
    assert len(n.sent['triangle','wall_point_base'])==1


def test_missing_yolo_does_not_retimestamp_old_ray_with_new_scan(lidar_node):
    n=lidar_node;inputs(n)
    n.rays['triangle'][-1].header.stamp.sec-=2
    n._tick()
    assert not n.sent['triangle','wall_point_base']
    assert 'fresh YOLO' in n.states[-1]['reason']


def test_old_class_detection_is_not_reported_as_scan_sync_failure(lidar_node):
    n=lidar_node;inputs(n,('triangle','square'))
    n.rays['square'][-1].header.stamp.sec-=2
    n._tick()
    assert n.states[-1]['markers']['triangle']['localized']
    stale=n.states[-1]['markers']['square']
    assert stale['reason']=='detection_stale'
    assert not stale['detected'] and not stale['localized']
    assert not n.sent['square','wall_point_base']
