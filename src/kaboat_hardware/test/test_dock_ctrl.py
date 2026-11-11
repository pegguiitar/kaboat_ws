"""Dock adapter exercises the SAME avoidance controller, not a separate controller."""
import time
import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from kaboat_hardware.dock_ctrl import DockCtrl


@pytest.fixture
def control():
    rclpy.init();node=DockCtrl();node.drive_enabled=True;node.goal_topic='/dock/approach_point'
    node.require_goal_updates=False;node.require_front_obstacle=False;node.planner_params.finite_goal=True
    node.commands=[];node.cmd_pub.publish=node.commands.append
    odom=Odometry();odom.header.frame_id='odom';odom.pose.pose.orientation.w=0.;odom.pose.pose.orientation.z=1.
    node._on_odom(odom)
    grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.resolution=.2;grid.info.width=grid.info.height=100
    grid.info.origin.position.x=grid.info.origin.position.y=-10.;grid.data=[0]*10000;node._on_grid(grid)
    node._on_scan(LaserScan(range_min=.1,range_max=10.,ranges=[2.]))
    point=PointStamped();point.header.frame_id='odom';point.header.stamp=node.get_clock().now().to_msg();point.point.x=-6.
    node._on_goal(point);node._tick()
    yield node
    node.destroy_node();rclpy.shutdown()


def test_existing_avoidance_fsm_follows_entry_at_existing_output_limits(control):
    node=control
    assert node.max_linear_cmd==.25 and node.max_angular_cmd==.31
    assert node.preview_plan is not None and node.preview_goal==(-6.,0.)
    assert node._arm()[0]
    node._tick();node._tick()
    assert node.fsm.plan is not None
    assert any(m.linear.x>0 for m in node.commands)
    assert all(abs(m.linear.x)<=.25 and abs(m.angular.z)<=.31 for m in node.commands)


def test_latched_entry_does_not_need_new_yolo_goal_frames(control):
    node=control;assert node._arm()[0];node.external_goal_received-=3
    node._tick();node._tick()
    assert node.started and node.goal==(-6.,0.)


def test_original_sensor_watchdog_still_stops_control(control):
    node=control;assert node._arm()[0];node.scan_received-=1.;node._tick()
    assert not node.started
    assert node.commands[-1].linear.x==node.commands[-1].angular.z==0.


def stop_goal(node,x=-7.,y=0.):
    point=PointStamped();point.header.frame_id='odom'
    point.header.stamp=node.get_clock().now().to_msg()
    point.point.x=x;point.point.y=y;node._on_stop_goal(point)


def test_switches_at_three_metres_and_replaces_entry_plan(control):
    node=control;stop_goal(node);assert node._arm()[0]
    node._tick();old_plan=node.fsm.plan
    node.odom.pose.pose.position.x=-3.;node._tick()
    assert node.final_stage and node.goal==(-7.,0.)
    assert node.fsm.plan is not None and node.fsm.plan is not old_plan
    node._tick()
    assert 0<node.commands[-1].linear.x<=node.dock_final_cmd


def test_before_threshold_keeps_entry_goal(control):
    node=control;stop_goal(node);assert node._arm()[0]
    node.odom.pose.pose.position.x=-2.99;node._tick()
    assert not node.final_stage and node.goal==(-6.,0.)


def test_final_stage_tracks_updates_and_never_returns_to_entry(control):
    node=control;stop_goal(node);assert node._arm()[0]
    node.odom.pose.pose.position.x=-3.;node._tick()
    stop_goal(node,-7.2);node.odom.pose.pose.position.x=-2.;node._tick()
    assert node.final_stage and node.goal==(-7.2,0.)
    assert node.planned_stop_goal==(-7.2,0.)


def test_missing_stop_goal_at_switch_stops_instead_of_reaching_entry(control):
    node=control;assert node._arm()[0]
    node.odom.pose.pose.position.x=-3.;node._tick()
    assert not node.started
    assert node.commands[-1].linear.x==node.commands[-1].angular.z==0.


def test_final_goal_loss_stops_and_cannot_drive_on_old_goal(control):
    node=control;stop_goal(node);assert node._arm()[0]
    node.odom.pose.pose.position.x=-3.;node._tick();node.stop_received-=2.
    node._tick()
    assert not node.started
    assert node.commands[-1].linear.x==node.commands[-1].angular.z==0.


def test_final_goal_jump_stops(control):
    node=control;stop_goal(node);assert node._arm()[0]
    node.odom.pose.pose.position.x=-3.;node._tick()
    stop_goal(node,-8.);node._tick()
    assert not node.started and node.commands[-1].linear.x==0.


def test_reaching_entry_does_not_finish_but_final_goal_does(control):
    node=control;stop_goal(node);assert node._arm()[0]
    node.odom.pose.pose.position.x=-6.;node._tick()
    assert node.started and not node.finished and node.goal==(-7.,0.)
    node.odom.pose.pose.position.x=-6.8;node._tick()
    assert node.finished and not node.started
    assert node.commands[-1].linear.x==node.commands[-1].angular.z==0.


def test_invalid_stop_frame_and_old_measurement_are_rejected(control):
    node=control;stop_goal(node)
    previous=node.stop_goal
    node._on_stop_goal(previous)
    assert node.stop_goal is previous
    previous.header.frame_id='base_link';node._on_stop_goal(previous)
    assert node.stop_goal is None
