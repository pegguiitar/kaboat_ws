"""behavior 노드 시험 공통 도구 (시험 파일 아님 — test_ 접두사 없음).

노드를 spin 하지 않고 콜백과 compute_cmd() 를 직접 부른다. behavior 는
time.monotonic() 과 get_clock() 을 섞어 쓰므로 FakeClock 이 둘을 같은 시각으로 맞춘다.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from std_msgs.msg import String


class Spy:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)

    @property
    def data(self):
        return [m.data for m in self.msgs]


class FakeClock:
    def __init__(self, start=1000.0):
        self.t = float(start)

    def now(self):
        return Time(nanoseconds=int(self.t * 1e9))


def odom(x, y, yaw=0.0, frame='odom', yaw_rate=0.0):
    msg = Odometry()
    msg.header.frame_id = frame
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.twist.twist.angular.z = float(yaw_rate)
    return msg


def goal(x, y, frame='odom'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.w = 1.0
    return msg


class BehaviorHarness:
    """behavior 노드 하나를 만들고 시각·odom 을 함께 넘기며 tick 한다."""

    def __init__(self, cls, monkeypatch, params=()):
        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        self.clock = FakeClock()
        monkeypatch.setattr(time, 'monotonic', lambda: self.clock.t)
        self.node = cls()
        self.node.get_clock = lambda: self.clock
        self.node.cmd_pub = Spy()
        self.node.complete_pub = Spy()
        self.pose = None

    def activate(self, state=None):
        self.node._on_state(String(data=state or self.node.STATE_NAME))

    def set_goal(self, x, y, frame='odom'):
        self.node._on_goal(goal(x, y, frame))

    def set_pose(self, x, y, yaw=0.0, frame='odom'):
        self.pose = odom(x, y, yaw, frame)

    def tick(self, n=1, dt=0.1):
        """dt 만큼 시각을 진행하고 odom 을 새로 넣은 뒤 compute_cmd() 결과를 돌려준다."""
        cmd = None
        for _ in range(n):
            self.clock.t += dt
            if self.pose is not None:
                # obstacle_planner 는 odom 스탬프를 시간원으로 쓴다.
                self.pose.header.stamp = self.clock.now().to_msg()
                self.node._on_odom(self.pose)
            cmd = self.node.compute_cmd()
        return cmd

    @property
    def completions(self):
        return self.node.complete_pub.data

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def is_stop(cmd):
    return cmd.linear.x == 0.0 and cmd.angular.z == 0.0
