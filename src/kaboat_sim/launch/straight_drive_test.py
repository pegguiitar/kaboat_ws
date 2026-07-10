#!/usr/bin/env python3
"""straight_drive_test  (테스트용)

배가 실제로 직진 주행하는지 확인하기 위한 최소 테스트 노드.
좌/우 추력 토픽에 동일한 값을 발행하면 요(yaw) 토크가 상쇄되어 직진한다.

사용법 (시뮬레이션이 이미 재생 중인 상태에서, 별도 터미널에서):
    ros2 run kaboat_sim straight_drive_test.py
    ros2 run kaboat_sim straight_drive_test.py --ros-args -p thrust:=20.0 -p duration:=15.0

⚠️ 추력 값 주의 (선체 크기 축소 반영됨):
    kaboat_hull.xacro 의 target_length=1.1 로 축소된 지금 배는 최대추력이
    약 118.6N 밖에 안 되고(원본 4.9m WAM-V 대비 s^2 배 축소), 질량도 약 2kg
    으로 아주 가볍다. 최대치에 가까운 값을 주면 한두 스텝만에 극단적인
    가속(F=ma, m 이 작아 a 가 매우 커짐)이 걸려 이전에 겪었던 물리 발산
    크래시("ODE INTERNAL ERROR ... aabbBound")가 재현될 수 있다. 기본값은
    최대치의 10%대로 안전하게 잡아뒀다 — 배 크기를 바꾸면 max_thrust 파라미터도
    같이 확인/조정할 것 (kaboat_hull.xacro 의 max_thrust_cmd 계산식 참고).

파라미터:
    thrust      좌/우 동일 추력 [N], 기본 12.0 (max_thrust 의 약 10% — 안전한 시작값)
    max_thrust  이 배의 최대추력 [N] 참고값, 기본 118.6 (kaboat_hull.xacro 와 동일하게
                유지할 것). thrust 가 이 값의 80% 를 넘으면 경고를 띄운다.
    duration    주행 시간 [s], 기본 10.0. 0 이하로 주면 Ctrl+C 로 끌 때까지 계속 주행.
    rate_hz     추력 재발행 주기 [Hz], 기본 10.0.

종료(타임아웃 도달 또는 Ctrl+C) 시 추력을 0으로 발행하고 끝난다 — 배가
관성으로 계속 나아가다 서서히 멈춘다(그대로 물리 감쇠에 맡김).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class StraightDriveTest(Node):
    def __init__(self):
        super().__init__('straight_drive_test')

        self.declare_parameter('thrust', 12.0)
        self.declare_parameter('max_thrust', 118.6)  # kaboat_hull.xacro 의 max_thrust_cmd 와 동일하게 유지
        self.declare_parameter('duration', 10.0)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('left_topic', '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_topic', '/wamv/thrusters/right/thrust')

        self.thrust = float(self.get_parameter('thrust').value)
        max_thrust = float(self.get_parameter('max_thrust').value)
        self.duration = float(self.get_parameter('duration').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        left_topic = self.get_parameter('left_topic').value
        right_topic = self.get_parameter('right_topic').value
        self.pub_l = self.create_publisher(Float64, left_topic, 10)
        self.pub_r = self.create_publisher(Float64, right_topic, 10)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        if abs(self.thrust) > max_thrust:
            self.get_logger().error(
                f'thrust={self.thrust}N 이 max_thrust={max_thrust}N 을 초과합니다 — '
                f'추력 시스템이 자체적으로 클램프하겠지만, 배가 작고 가벼워서 이런 큰 값은 '
                f'물리 발산(크래시)을 유발할 수 있습니다. 낮춰서 다시 실행하세요.'
            )
        elif abs(self.thrust) > 0.8 * max_thrust:
            self.get_logger().warn(
                f'thrust={self.thrust}N 이 max_thrust({max_thrust}N)의 80% 를 초과합니다 — '
                f'배가 작고 가벼워서 급격한 가속으로 이어질 수 있습니다.'
            )

        self.get_logger().info(
            f'straight_drive_test 시작: thrust={self.thrust}N (max={max_thrust}N), '
            f'duration={self.duration if self.duration > 0 else "무제한(Ctrl+C로 종료)"}s'
        )

    def tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if self.duration > 0 and elapsed >= self.duration:
            self.get_logger().info('duration 도달 — 추력 0으로 정지 명령 후 종료')
            self._publish(0.0)
            self.timer.cancel()
            rclpy.shutdown()
            return
        self._publish(self.thrust)

    def _publish(self, value: float):
        msg_l = Float64()
        msg_r = Float64()
        msg_l.data = value
        msg_r.data = value  # 좌우 동일 → yaw 토크 0 → 직진
        self.pub_l.publish(msg_l)
        self.pub_r.publish(msg_r)


def main(args=None):
    rclpy.init(args=args)
    node = StraightDriveTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C 로 끊겨도 추력 0을 마지막으로 한 번 발행해 배를 멈춘다.
        try:
            node._publish(0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
