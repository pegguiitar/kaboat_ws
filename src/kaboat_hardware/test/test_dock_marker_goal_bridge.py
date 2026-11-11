import json
import copy
import pytest
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from kaboat_hardware.dock_marker_goal import DockMarkerGoal
from kaboat_behaviors.dock_marker_goal import TimedGoalAverage


@pytest.fixture
def bridge():
    rclpy.init()
    node = DockMarkerGoal()
    node.average = TimedGoalAverage(window_s=0)
    node.normal_average = TimedGoalAverage(window_s=0)
    node.goals, node.candidates, node.states, node.viz = [], [], [], []
    node.goal_pub.publish = node.goals.append
    node.candidate_pub.publish = node.candidates.append
    node.status_pub.publish = lambda m: node.states.append(json.loads(m.data))
    node.viz_pub.publish = node.viz.append
    odom = Odometry()
    odom.header.frame_id = 'odom'
    node._on_odom(odom)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def markers(node, x=-.2):
    for name, y in [('triangle', 2.5), ('circle', 1.), ('square', 4.)]:
        m = PointStamped()
        m.header.frame_id = 'odom'
        m.header.stamp = node.get_clock().now().to_msg()
        m.point.x, m.point.y = x, y
        node._on_marker(name, m)
        normal=Vector3Stamped();normal.header=copy.deepcopy(m.header);normal.vector.x=1.
        node._on_normal(name,normal)


def test_goal_is_exactly_one_metre_inward_with_original_stamp(bridge):
    markers(bridge)
    bridge._tick()
    assert bridge.states[-1]['valid']
    assert bridge.goals[-1].point.x == pytest.approx(.8)
    assert bridge.goals[-1].point.y == pytest.approx(2.5)
    assert bridge.goals[-1].point.z == 0.
    assert bridge.goals[-1].header == bridge.points['triangle'][0].header
    assert bridge.states[-1]['fresh_markers'] == ['circle', 'square', 'triangle']
    assert bridge.points['triangle'][0].point.x == -.2


def test_stale_selected_marker_stops_new_goal_publication(bridge):
    markers(bridge)
    bridge._tick()
    msg, received = bridge.points['triangle']
    bridge.points['triangle'] = msg, received - 2
    bridge._tick()
    assert not bridge.states[-1]['valid']
    assert len(bridge.goals) == 1


def test_outside_goal_is_published_for_preview_with_motion_bounds_invalid(bridge):
    markers(bridge, x=-1.)
    bridge._tick()
    assert not bridge.states[-1]['valid']
    assert bridge.states[-1]['geometry_valid']
    assert not bridge.states[-1]['motion_bounds_valid']
    assert bridge.goals[-1].point.x == pytest.approx(0.)
    assert bridge.candidates[-1].point.x == pytest.approx(0.)
    assert bridge.viz[-1].markers[2].color.r == 1.
    assert bridge.viz[-1].markers[2].color.g == 0.


def test_one_target_and_its_lidar_normal_are_sufficient(bridge):
    markers(bridge)
    bridge.points.pop('circle');bridge.points.pop('square')
    bridge._tick()
    assert bridge.states[-1]['valid']
    assert bridge.states[-1]['geometry_source']=='lidar_ransac'
    assert bridge.goals[-1].point.x==pytest.approx(.8)


def test_mismatched_normal_stamp_cannot_generate_goal(bridge):
    markers(bridge)
    bridge.normals['triangle'][0].header.stamp.sec-=1
    bridge._tick()
    assert not bridge.goals
    assert not bridge.states[-1]['valid']


def test_missing_or_stale_normal_cannot_generate_goal(bridge):
    markers(bridge)
    normal,received=bridge.normals['triangle'];bridge.normals['triangle']=(normal,received-2)
    bridge._tick()
    assert not bridge.goals


def test_wall_normal_creates_three_collinear_waypoints_and_stop_offset(bridge):
    paths=[];bridge.waypoints_pub.publish=paths.append
    markers(bridge,x=.2);bridge._tick()
    path=paths[-1]
    assert len(path.poses)==3 and path.header.frame_id=='odom'
    assert [p.pose.position.x for p in path.poses]==pytest.approx([2.2,1.7,1.2])
    assert [p.pose.position.y for p in path.poses]==pytest.approx([2.5]*3)
    assert abs(path.poses[-1].pose.orientation.z)==pytest.approx(1.)
    assert bridge.goals[-1].point.x==pytest.approx(1.2)


def test_waypoint_display_distinguishes_stages_and_marks_wall_reference(bridge):
    markers(bridge,x=.2);bridge._tick()
    items=bridge.viz[-1].markers
    assert len({(m.color.r,m.color.g,m.color.b) for m in items[:3]})==3
    labels=[m.text for m in items if m.type==m.TEXT_VIEW_FACING]
    assert 'APPROACH +2.0m' in labels and 'STOP +1.0m' in labels and 'WALL' in labels
    assert items[6].pose.position.x==pytest.approx(.2)


def test_lost_target_clears_waypoint_path_instead_of_leaving_old_line_visible(bridge):
    paths=[];bridge.waypoints_pub.publish=paths.append
    markers(bridge);bridge._tick();assert len(paths[-1].poses)==3
    bridge.points.clear();bridge._tick()
    assert not paths[-1].poses and paths[-1].header.frame_id=='odom'
