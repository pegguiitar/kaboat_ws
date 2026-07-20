#!/usr/bin/env python3
"""odom_gps_noise — ground truth(/odom_ground_truth)에 GPS 수준 위치오차를 얹어
/odom 으로 재발행하는 sim 전용 노드 (localization 노이즈 견딤 테스트용).

왜 이 노드가 있나 (설계 배경):
  실물은 순수 GNSS(F9P/GQ7) + 외부 IMU(EBIMU)를 robot_localization EKF 로
  융합해 /odom 을 만든다 — 그 파이프라인은 localization_ekf.launch.py 에
  보존돼 있다. 그러나 EKF 의 "튜닝"(공분산/프로세스노이즈)은 실제 센서의
  노이즈 특성에 묶여 있어, sim 의 근사 노이즈에 맞춰 튜닝해봐야 실물로
  이월되지 않는다. 그래서 sim 에서는 EKF 를 튜닝하는 대신 "위치추정 결과에
  약 50cm 오차가 있다"는 최종 효과만 직접 모델링해 자율 스택을 시험한다.
  (배선/좌표계는 ground truth 그대로라 datum·프레임 어긋남 문제도 없다.)

노이즈 모델 — 왜 white noise 가 아니라 correlated(OU) 인가:
  실제 GPS/융합 위치오차는 매 샘플 독립인 백색잡음이 아니라, 대기·멀티패스
  바이어스가 수 초에 걸쳐 "천천히 흔들리는" 상관 잡음이다. 매 tick 독립
  가우시안을 더하면 위치가 50Hz 로 ±50cm 튀어 플래너가 발작한다. 그래서
  Ornstein-Uhlenbeck 과정으로, 오차가 정상 분포(정상 stddev=sigma)를
  유지하면서 상관시간 tau 로 매끄럽게 드리프트하게 만든다:
      e[t+1] = a·e[t] + sqrt(1−a²)·sigma·N(0,1),   a = exp(−dt/tau)
  위치(x,y)에만 얹는다 — 방위/각속도는 IMU(EBIMU) 담당이고 GPS 오차와
  무관하므로 orientation·twist 는 ground truth 그대로 통과시킨다.

파라미터:
  sigma [m]  위치오차 정상 stddev. 기본 0.25 → 약 95%(2σ)가 0.5m 이내
             ("최대 50cm 정도" 가정). 1σ=0.5m 를 원하면 0.5 로.
  tau   [s]  오차 상관시간(드리프트 속도). 기본 5.0 — 클수록 더 느리게 흔들림.
  seed       난수 시드(재현용). <0 이면 매번 다름.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomGpsNoise(Node):
    def __init__(self):
        super().__init__('odom_gps_noise')
        self.declare_parameter('sigma', 0.25)
        self.declare_parameter('tau', 5.0)
        self.declare_parameter('seed', -1)
        self.sigma = float(self.get_parameter('sigma').value)
        self.tau = float(self.get_parameter('tau').value)
        seed = int(self.get_parameter('seed').value)
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.ex = 0.0        # 현재 x 오차 (OU 상태) — 0 에서 시작해 sigma 로 수렴
        self.ey = 0.0
        self.last_t = None

        self.create_subscription(Odometry, '/odom_ground_truth', self._on_odom, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.get_logger().info(
            f'odom_gps_noise 시작 — sigma={self.sigma}m tau={self.tau}s '
            f'(/odom_ground_truth → /odom + odom TF)')

    def _on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.02 if self.last_t is None else max(1e-3, t - self.last_t)
        self.last_t = t

        # OU 갱신 — 정상 stddev=sigma, 상관시간=tau
        a = math.exp(-dt / self.tau) if self.tau > 1e-6 else 0.0
        s = self.sigma * math.sqrt(max(0.0, 1.0 - a * a))
        self.ex = a * self.ex + s * float(self.rng.standard_normal())
        self.ey = a * self.ey + s * float(self.rng.standard_normal())

        gx = msg.pose.pose.position.x
        gy = msg.pose.pose.position.y
        # orientation·twist 은 그대로 통과(IMU 담당), 위치 x·y 에만 오차 주입
        msg.pose.pose.position.x = gx + self.ex
        msg.pose.pose.position.y = gy + self.ey
        self.pub.publish(msg)

        # odom → base_link TF 도 같은(노이즈 낀) 위치로 발행 — 실 로컬라이저처럼
        # topic 과 TF 를 일치시켜, TF 소비자(RViz 등)도 같은 추정 위치를 본다.
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = msg.header.frame_id or 'odom'
        tf.child_frame_id = msg.child_frame_id or 'wamv/base_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomGpsNoise()
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
