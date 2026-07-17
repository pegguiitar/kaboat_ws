#!/usr/bin/env python3
"""twist2thrust — /cmd_vel(정규화 명령) → 좌우 추력[N] 변환 드라이버.

명령 의미(2026-07-17 정규화): linear.x / angular.z 는 **최대 추력 대비 비율**
[-1, 1] 이다. 이 노드가 자기 스러스터 스펙(max_thrust)을 곱해 N 으로 바꾼다.
실물 전환 시 이 노드만 실물 ESC 드라이버로 교체하면 상위 스택은 같은 명령
의미를 유지한다 — 컷 %(상한)·스펙(max_thrust)의 소유자는 드라이버 계층이다
(SKELETON §6 실물 전환 지도).

구 의미(scale=60 임의상수 × cmd)는 폐기 — cmd 0.2 = 12N 이던 검증 동작은
새 의미에서 cmd 12/118.6 = 12N 으로 동일하게 재현된다(상위 상한들도 같이
환산됨, behavior_base 참조).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TwistToThrust(Node):
    def __init__(self):
        super().__init__('twist2thrust_node')
        # 이 배의 스러스터 최대 추력 [N] — kaboat_hull.xacro 의 max_thrust_cmd
        # 계산식과 같은 값을 유지할 것 (선체 크기를 바꾸면 둘 다 갱신).
        self.declare_parameter('max_thrust', 118.6)
        self.max_thrust = float(self.get_parameter('max_thrust').value)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub_l = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_r = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

    def cb(self, msg):
        # 차동 구동: 직진 + 회전. linear/angular 합이 1 을 넘으면 스러스터가
        # 물리적으로 포화되므로 여기서 ±max_thrust 로 잘라 준다(실물 ESC 와 동일).
        left = (msg.linear.x - msg.angular.z) * self.max_thrust
        right = (msg.linear.x + msg.angular.z) * self.max_thrust

        left_thrust = Float64()
        right_thrust = Float64()
        left_thrust.data = max(-self.max_thrust, min(self.max_thrust, left))
        right_thrust.data = max(-self.max_thrust, min(self.max_thrust, right))

        self.pub_l.publish(left_thrust)
        self.pub_r.publish(right_thrust)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToThrust()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
