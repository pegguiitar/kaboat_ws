"""obstacle_planner — 장애물 회피 behavior ROS 노드 (AVOIDANCE.MD §2).

배선만, 로직 0: /occupancy_grid·/odom·/mission/goal → build_dri(매 tick) →
AvoidFsm.step → /cmd/avoid. 알고리즘은 전부 순수 라이브러리(dri /
bspline_planner / avoid_fsm)에 있고, 여기는 구독/발행/파라미터/로깅과
좌표 변환(odom twist body→world)뿐이다 — FSM 을 노드 밖에 두는 이유는
avoid_fsm.py docstring 참조.
"""
import dataclasses
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from .behavior_base import BehaviorBase, YAW_KP, YAW_KD, yaw_from_quaternion
from .avoid_fsm import AvoidFsm, FsmParams
from .bspline_planner import PlannerParams
from .dri import DriParams, build_dri


class ObstaclePlanner(BehaviorBase):
    STATE_NAME = 'avoid'
    CMD_TOPIC = '/cmd/avoid'

    def __init__(self):
        super().__init__()
        # 파라미터 원본은 코드 기본값(AVOIDANCE.MD §4 는 그 사본) — dataclass 필드를 그대로
        # ROS param 으로 노출해 sim 튜닝 때 런타임 오버라이드만 한다.
        # 각도 필드(bearing_max 등)는 radian 그대로다.
        self.dri_params = self._params_from('dri', DriParams)
        self.planner_params = self._params_from('planner', PlannerParams)
        self.fsm_params = self._params_from('fsm', FsmParams)
        self.fsm = AvoidFsm(self.planner_params, self.fsm_params,
                            yaw_kp=YAW_KP, yaw_kd=YAW_KD)
        # 디버그 출력 — RViz 와 향후 avoid_viz 가 구독. 로직 아님.
        # z 는 0.2 로 띄운다 — z=0 이면 RViz 에서 Map/DRI 레이어에 묻혀 안 보임.
        self.plan_pub = self.create_publisher(Path, '/avoid/plan', 1)
        self.cps_pub = self.create_publisher(MarkerArray, '/avoid/cps', 1)

    def _params_from(self, prefix: str, cls):
        defaults = cls()
        kw = {}
        for f in dataclasses.fields(cls):
            name = f'{prefix}.{f.name}'
            self.declare_parameter(name, getattr(defaults, f.name))
            kw[f.name] = self.get_parameter(name).value
        return cls(**kw)

    def _on_state(self, msg):
        was_active = self.active
        super()._on_state(msg)
        if was_active and not self.active:
            # 비활성→재활성 시 낡은 plan·상태 이월 금지 
            self.fsm.reset()

    def compute_cmd(self):
        cmd = Twist()
        if self.odom is None or self.goal is None or self.occupancy_grid is None:
            return cmd   # 데이터 없음 — 정지 명령 발행 (워치독 침묵 금지)

        pos = self.odom.pose.pose.position
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        tw = self.odom.twist.twist
        # /odom twist 는 body frame — generate 는 world 속도를 받으므로 회전.
        # (sim 하네스 gen_viz 와 동일한 변환 — 튜닝된 파라미터가 이 가정
        # 위의 실측값이다)
        c, s = math.cos(yaw), math.sin(yaw)
        vel = (c * tw.linear.x - s * tw.linear.y,
               s * tw.linear.x + c * tw.linear.y)
        wp = (self.goal.pose.position.x, self.goal.pose.position.y)
        # 시간원은 odom 스탬프 — sim 은 RTF<1 로 벽시계와 어긋나고(수십 분
        # 드리프트 실측), TF·센서가 전부 이 시간축을 탄다. 벽시계를 쓰면
        # ① viz 가 "No transform"(RViz 실측) ② FSM 주기(t_backstop 등)가
        # 물리 시간과 어긋남. 실물에선 odom 스탬프 = 벽시계라 동작 동일.
        now_t = (self.odom.header.stamp.sec
                 + self.odom.header.stamp.nanosec * 1e-9)

        dri = build_dri(self.occupancy_grid, (pos.x, pos.y), yaw, self.dri_params)
        out = self.fsm.step(dri, (pos.x, pos.y), yaw, vel, tw.angular.z,
                            wp, now_t)
        if out.event is not None:
            self.get_logger().info(f'[{out.state}] {out.event}',
                                   throttle_duration_sec=1.0)
        self._publish_plan(out.plan)
        cmd.linear.x = out.linear_x
        cmd.angular.z = out.angular_z
        return cmd

    def _publish_plan(self, plan):
        path = Path()
        # 스탬프는 odom 것 — TF 와 같은 시간축이어야 RViz 가 변환 가능
        path.header.stamp = self.odom.header.stamp
        path.header.frame_id = self.occupancy_grid.header.frame_id
        if plan is not None:
            for x, y in plan.samples:
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                ps.pose.position.z = 0.2
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
        self.plan_pub.publish(path)

        marks = MarkerArray()
        # waypoint — 초록 구 (plan 유무와 무관하게 항상 표시)
        wp = Marker()
        wp.header = path.header
        wp.ns, wp.id = 'wp', 0
        wp.type, wp.action = Marker.SPHERE, Marker.ADD
        wp.pose.position.x = self.goal.pose.position.x
        wp.pose.position.y = self.goal.pose.position.y
        wp.pose.position.z = 0.2
        wp.pose.orientation.w = 1.0
        wp.scale.x = wp.scale.y = wp.scale.z = 0.5
        wp.color.g, wp.color.a = 0.9, 1.0
        wp.color.r = 0.1
        marks.markers.append(wp)
        n_cps = 6 if plan is not None else 0
        for i in range(n_cps):
            m = Marker()
            m.header = path.header
            m.ns, m.id = 'cps', i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x = float(plan.cps[i, 0])
            m.pose.position.y = float(plan.cps[i, 1])
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            # p0/p1 = 파랑 (배·관성점), p2~p5 = 주황 (arc 선택점)
            if i < 2:
                m.color.r, m.color.g, m.color.b = 0.2, 0.6, 1.0
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.6, 0.1
            m.color.a = 1.0
            marks.markers.append(m)
        for i in range(n_cps, 6):   # plan 없을 때 잔상 삭제
            m = Marker()
            m.header = path.header
            m.ns, m.id, m.action = 'cps', i, Marker.DELETE
            marks.markers.append(m)
        self.cps_pub.publish(marks)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePlanner()
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
