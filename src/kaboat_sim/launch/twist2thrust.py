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
        # 💡 수정 2: 모터 출력 스케일 조절 (원래 1000.0 이었으나 배 사양에 맞게 조절 가능)
        scale = 250.0  
        
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