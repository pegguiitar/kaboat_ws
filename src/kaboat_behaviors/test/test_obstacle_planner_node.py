"""obstacle_planner 노드: /odom 최신만(큐 깊이 1)·회피 전용 D 게인 (2026-11-14)·실물 추력 곡선 (2026-11-15)."""

import math

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


def test_latency_age_clamps_and_rejects_clock_mismatch():
    assert op.latency_age(100.74, 100.0, 1.0) == pytest.approx(0.74)
    assert op.latency_age(101.5, 100.0, 1.0) == pytest.approx(1.0)       # 최대 1 s
    assert op.latency_age(99.95, 100.0, 1.0) == 0.0                      # 살짝 미래 = 0
    assert op.latency_age(100.0, 0.0, 1.0) == 0.0                        # 스탬프 0 등 시계 어긋남 = 보정 안 함
    assert op.latency_age(98.0, 100.0, 1.0) == 0.0


def test_predict_pose_extrapolates_position_and_heading():
    x, y, yaw = op.predict_pose(1.0, 2.0, math.radians(170.0), (0.5, -0.2), math.radians(40.0), 0.5)
    assert (x, y) == pytest.approx((1.25, 1.9))
    assert math.degrees(yaw) == pytest.approx(-170.0)                    # 170 + 20 → −170 (wrap)
    assert op.predict_pose(1.0, 2.0, 0.3, (9.0, 9.0), 9.0, 0.0) == (1.0, 2.0, 0.3)


def test_avoid_latency_prediction_default_on(ros):
    node = op.ObstaclePlanner()
    try:
        assert node.predict_latency is True and node.predict_max_s == 1.0
    finally:
        node.destroy_node()


def test_avoid_uses_measured_speed_curve_by_default(ros):
    from kaboat_behaviors.avoid_fsm import FsmParams
    node = op.ObstaclePlanner()
    try:
        assert node.fsm_params.measured_speed is True and op.AVOID_MEASURED_SPEED is True
        assert node.fsm_params.cmd_full == FsmParams().cmd_full == 0.2   # 전진 상한 그대로
        assert FsmParams().measured_speed is False                       # 공용(토너먼트 시험) 기본은 그대로
    finally:
        node.destroy_node()
    rclpy.shutdown(); rclpy.init()
    from rclpy.node import Node
    orig = Node.__init__

    def init_with(self, name, **kw):
        kw['parameter_overrides'] = [Parameter('fsm.measured_speed', value=False)]
        orig(self, name, **kw)
    Node.__init__ = init_with
    try:
        node = op.ObstaclePlanner()
        assert node.fsm_params.measured_speed is False          # 되돌리기 avoid_measured_speed:=false
        node.destroy_node()
    finally:
        Node.__init__ = orig
