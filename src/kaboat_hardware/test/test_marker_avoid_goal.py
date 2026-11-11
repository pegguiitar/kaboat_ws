import time
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from kaboat_hardware.real_obstacle_avoid_test import RealObstacleAvoidTest


@pytest.fixture
def mission():
    rclpy.init()
    node = RealObstacleAvoidTest()
    node.goal_topic = '/dock/avoid/goal'
    node.require_front_obstacle = False
    node.planner_params.finite_goal = True
    node.commands = []
    node.cmd_pub.publish = node.commands.append
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.w = 0.0
    odom.pose.pose.orientation.z = 1.0
    node._on_odom(odom)
    grid = OccupancyGrid()
    grid.header.frame_id = 'odom'
    grid.info.resolution = .2
    grid.info.width = grid.info.height = 50
    grid.info.origin.position.x = grid.info.origin.position.y = -5.
    cells = np.zeros((50, 50), dtype=np.int8)
    cells[22:28, 10:14] = 99  # obstacle on the straight route to the marker goal
    grid.data = cells.ravel().tolist()
    node._on_grid(grid)
    scan = LaserScan(range_min=.1, range_max=10., ranges=[2.])
    node._on_scan(scan)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def goal(node, x=-4., y=0.):
    msg = PointStamped()
    msg.header.frame_id = 'odom'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.point.x, msg.point.y = x, y
    node._on_goal(msg)
    return msg


def test_external_marker_goal_plans_around_obstacle_and_latches_on_start(mission):
    node = mission
    assert 'waiting for marker' in node._input_error(time.monotonic())
    goal(node)
    node._tick()
    assert node.preview_goal == (-4., 0.)
    assert node.preview_plan is not None
    np.testing.assert_allclose(node.preview_plan.samples[-1], (-4., 0.))
    assert any(abs(y) > .2 for x, y in node.preview_plan.samples)
    assert node._arm()[0]
    assert node.goal == (-4., 0.)
    assert not node.drive_enabled and node.commands == []
    node.external_goal_received -= 1
    node._tick()
    assert not node.started
    assert node.commands == []


def test_large_goal_shift_disarms_instead_of_chasing_new_marker(mission):
    goal(mission)
    mission._tick()
    assert mission._arm()[0]
    goal(mission, -2., 1.)
    assert 'latched goal' in mission._input_error(time.monotonic())
    mission._tick()
    assert not mission.started


def test_invalid_stale_and_reordered_goal_samples_do_not_refresh(mission):
    message = goal(mission)
    received = mission.external_goal_received
    mission._on_goal(message)
    assert mission.external_goal_received == received
    message.header.stamp.sec -= 10
    mission._on_goal(message)
    assert mission.external_goal_received == received
    message.header.frame_id = 'camera_link'
    mission._on_goal(message)
    assert mission.external_goal is None


def test_final_waypoint_output_matches_the_active_bspline_endpoint(mission):
    from unittest.mock import Mock
    mission.final_goal_pub = Mock()
    goal(mission)
    mission._tick()
    message = mission.final_goal_pub.publish.call_args.args[0]
    assert message.header.frame_id == 'odom'
    np.testing.assert_allclose((message.point.x, message.point.y), mission.preview_plan.samples[-1])
    assert message.point.z == 0.
    assert mission._arm()[0]
    goal(mission, -3.9, .05)
    mission._tick()
    message = mission.final_goal_pub.publish.call_args.args[0]
    np.testing.assert_allclose((message.point.x, message.point.y), (-4., 0.))


def test_preview_goal_outside_boundary_cannot_arm(mission):
    mission.tank_bounds = (-5., -5., 5., 5.)
    mission.tank_enabled = True
    mission.tank_stop_margin = .5
    goal(mission, -4.8, 0.)
    mission._tick()
    assert mission.preview_goal == (-4.8, 0.)
    ok, reason = mission._arm()
    assert not ok and 'boundary' in reason
    assert mission.commands == []


def docking_input(node,points=None):
    import copy,math
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    points=points or [(-2.,0.),(-3.,0.),(-4.,0.)]
    message=goal(node,*points[-1])
    path=Path();path.header=copy.deepcopy(message.header)
    heading=math.atan2(points[-1][1]-points[-2][1],points[-1][0]-points[-2][0])
    for x,y in points:
        p=PoseStamped();p.header=copy.deepcopy(path.header);p.pose.position.x=x;p.pose.position.y=y
        p.pose.orientation.z=math.sin(heading/2);p.pose.orientation.w=math.cos(heading/2);path.poses.append(p)
    node.dock_waypoints_topic='/dock/waypoints'
    node._on_dock_waypoints(path)
    return path


def test_docking_preview_and_fsm_share_the_same_constrained_path_with_no_motor_output(mission):
    node=mission;node.grid.data=[0]*len(node.grid.data)
    docking_input(node)
    preview=[];node.preview_cmd_pub.publish=preview.append
    node._tick()
    assert node.preview_plan is not None
    np.testing.assert_allclose(node.preview_plan.samples[-1],(-4.,0.))
    np.testing.assert_allclose(node.preview_plan.cps[-1],(-2.,0.))
    assert any(m.linear.x>0 for m in preview)  # proposed control exists before arming
    assert node.commands==[]
    assert node._arm()[0]
    node._tick();node._tick()
    assert node.fsm.plan.docking_phase=='APPROACH'
    assert any(m.linear.x>0 for m in preview)
    assert node.commands==[]


def test_docking_normal_rotation_disarms_instead_of_replanning_into_side_wall(mission):
    node=mission;node.grid.data=[0]*len(node.grid.data)
    docking_input(node);node._tick();assert node._arm()[0]
    docking_input(node,[(-4.,-2.),(-4.,-1.),(-4.,0.)])
    assert 'normal changed' in node._input_error(time.monotonic())
    node._tick();assert not node.started and node.commands==[]


def test_final_docking_phase_is_capped_and_does_not_reverse_to_approach(mission):
    node=mission;node.grid.data=[0]*len(node.grid.data)
    node.odom.pose.pose.position.x=-3.2
    docking_input(node);node._tick()
    assert node.preview_plan.docking_phase=='FINAL'
    assert max(node.preview_plan.samples[:,0])<=-3.2+1e-8
    preview=[];node.preview_cmd_pub.publish=preview.append
    assert node._arm()[0]
    node._tick();node._tick()
    assert all(abs(m.linear.x)<=node.dock_final_cmd+1e-8 for m in preview)
    assert node.commands==[]


def test_no_plan_reports_approach_point_collision_separately_from_stop(mission):
    node=mission;node.grid.data=[0]*len(node.grid.data)
    docking_input(node)
    # The approach point is (-2,0); the final stop (-4,0) remains free.
    ix=int((-2-node.grid.info.origin.position.x)/node.grid.info.resolution)
    iy=int((0-node.grid.info.origin.position.y)/node.grid.info.resolution)
    node.grid.data[iy*node.grid.info.width+ix]=99
    node._tick()
    assert node.preview_plan is None
    assert 'APPROACH_OCCUPIED' in node.dock_plan_reason
    assert node.commands==[]


def test_dri_publication_checks_resolved_publisher_after_topic_remapping(mission):
    from unittest.mock import Mock
    node=mission;goal(node)
    node.count_subscribers=Mock(return_value=0)  # old names have no subscribers after remapping
    node.dri_pub.get_subscription_count=Mock(return_value=1)
    node.dri_marks_pub.get_subscription_count=Mock(return_value=1)
    node.dri_pub.publish=Mock();node.dri_marks_pub.publish=Mock()
    node._tick()
    node.dri_pub.publish.assert_called_once()
    node.dri_marks_pub.publish.assert_called_once()
    message=node.dri_pub.publish.call_args.args[0]
    assert message.header.frame_id=='odom'
    assert len(message.data)==message.info.width*message.info.height


def test_dri_remains_available_while_waiting_for_marker_goal(mission):
    from unittest.mock import Mock
    node=mission
    node.dri_pub.get_subscription_count=Mock(return_value=1)
    node.dri_pub.publish=Mock()
    node._tick()
    node.dri_pub.publish.assert_called_once()
    assert node.preview_plan is None and not node.started and node.commands==[]
