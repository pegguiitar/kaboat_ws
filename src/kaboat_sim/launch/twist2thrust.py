#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TwistToThrust(Node):
    def __init__(self):
        super().__init__('twist2thrust_node')
        # 키보드 명령(Twist)을 구독
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        
        # 💡 수정 1: 토픽 이름을 XACRO 및 Launch 파일과 완벽히 일치시킵니다.
        self.pub_l = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_r = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

    def cb(self, msg):
        # 모터 출력 스케일 — 선체 크기에 맞출 것.
        # 원본 4.9m WAM-V 시절 250.0 이었으나, target_length=1.1 축소 후에는
        # 최대추력이 118.6N 뿐이고 질량이 ~2kg 라 250 이면 전복된다 (실측).
        # scale=60 → cmd_vel 0.2 = 12N (직진 주행 테스트로 검증된 안전값).
        scale = 60.0
        
        # 차동 구동(Differential Drive) 공식: 직진 + 회전
        left_thrust = Float64()
        right_thrust = Float64()
        
        left_thrust.data = (msg.linear.x - msg.angular.z) * scale
        right_thrust.data = (msg.linear.x + msg.angular.z) * scale
        
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