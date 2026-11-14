"""obstacle_planner 노드: /odom 최신만(큐 깊이 1)·회피 전용 D 게인 (2026-11-14)."""

import pytest
import rclpy
from rclpy.parameter import Parameter

from kaboat_behaviors.behavior_base import BehaviorBase
from kaboat_behaviors.los_controller import BSPLINE_TRACK_YAW_KD, BSPLINE_TRACK_YAW_KP
from kaboat_behaviors import obstacle_planner as op


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _odom_depth(node):
    subs = [s for s in node.subscriptions if s.topic_name == '/odom']
    assert len(subs) == 1
    return subs[0].qos_profile.depth


def test_avoid_node_takes_only_the_newest_odom(ros):
    node = op.ObstaclePlanner()
    try:
        assert _odom_depth(node) == 1              # 젯슨에서 깊이 10 큐에 쌓여 0.3–0.4 s 묵은 /odom 을 썼다
    finally:
        node.destroy_node()


def test_other_behaviors_keep_default_odom_queue(ros):
    assert BehaviorBase.ODOM_QUEUE_DEPTH == 10       # 게이트·스테이션·탐색 등은 그대로

    class Dummy(BehaviorBase):
        STATE_NAME = 'dummy_test'
        CMD_TOPIC = '/cmd/dummy_test'

        def compute_cmd(self):
            return None
    node = Dummy()
    try:
        assert _odom_depth(node) == 10
    finally:
        node.destroy_node()


def test_avoid_yaw_gains_default_and_override(ros):
    node = op.ObstaclePlanner()
    try:
        assert node.fsm.yaw_kp == BSPLINE_TRACK_YAW_KP          # P 는 공용 LOS 그대로
        assert node.fsm.yaw_kd == op.AVOID_YAW_KD == 0.6        # D 는 회피 전용
        assert BSPLINE_TRACK_YAW_KD == 0.16                     # 공용 LOS(게이트·탐색) D 는 안 바뀜
    finally:
        node.destroy_node()
    rclpy.shutdown(); rclpy.init()
    # 파라미터 덮어쓰기는 실제 노드 생성으로 — 되돌리기 avoid.yaw_kd:=0.16
    from rclpy.node import Node
    orig = Node.__init__

    def init_with(self, name, **kw):
        kw['parameter_overrides'] = [Parameter('avoid.yaw_kd', value=0.16)]
        orig(self, name, **kw)
    Node.__init__ = init_with
    try:
        node = op.ObstaclePlanner()
        assert node.fsm.yaw_kd == pytest.approx(0.16)
        node.destroy_node()
    finally:
        Node.__init__ = orig


def test_bad_yaw_kd_is_rejected(ros):
    from rclpy.node import Node
    orig = Node.__init__

    def init_with(self, name, **kw):
        kw['parameter_overrides'] = [Parameter('avoid.yaw_kd', value=-1.0)]
        orig(self, name, **kw)
    Node.__init__ = init_with
    try:
        with pytest.raises(ValueError):
            op.ObstaclePlanner()
    finally:
        Node.__init__ = orig
