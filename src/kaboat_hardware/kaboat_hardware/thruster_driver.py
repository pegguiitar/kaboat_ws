"""thruster_driver — 실물 모터/ESC 제어 ROS 2 노드.

입력:
  /cmd_vel (geometry_msgs/msg/Twist)       — 정규화된 자율주행 추력 비율 [-1.0, 1.0]
  /rc/cmd_vel (geometry_msgs/msg/Twist)    — RC 수동 조종기 오버라이드 명령 (옵션)
  /emergency_stop (std_msgs/msg/Bool)      — 비상 정지 신호 (true 시 즉시 PWM 중립)

출력:
  하드웨어 PWM (시리얼/PCA9685/더미)
  /diagnostics (diagnostic_msgs/msg/DiagnosticArray) — 모터 동작 상태 진단
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from kaboat_hardware.thruster_backend import (
    ThrusterConfig, ThrusterMixer, BaseThrusterBackend,
    DummyBackend, SerialBackend, PCA9685Backend
)


class ThrusterDriver(Node):
    def __init__(self):
        super().__init__('thruster_driver')

        # ── 하드웨어 통신 파라미터 ───────────────────────
        self.declare_parameter('hardware_type', 'dummy')  # 'dummy' | 'serial' | 'pca9685'
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('pca9685_address', 0x40)
        self.declare_parameter('left_channel', 0)
        self.declare_parameter('right_channel', 1)

        # ── PWM 및 모터 특성 파라미터 ───────────────────
        self.declare_parameter('neutral_pwm', 1500)
        self.declare_parameter('max_forward_pwm', 1900)
        self.declare_parameter('max_reverse_pwm', 1100)
        self.declare_parameter('deadband_us', 25)
        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 1.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('max_slew_rate', 2.0)

        # ── 안전 및 워치독 파라미터 ─────────────────────
        self.declare_parameter('update_rate_hz', 50.0)    # ESC 신호 갱신 주기 [Hz]
        self.declare_parameter('timeout_sec', 0.3)        # /cmd_vel 타임아웃 [s]
        self.declare_parameter('enable_rc_override', True)
        self.declare_parameter('rc_timeout_sec', 0.5)

        hw_type = str(self.get_parameter('hardware_type').value).lower()
        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.enable_rc = bool(self.get_parameter('enable_rc_override').value)
        self.rc_timeout_sec = float(self.get_parameter('rc_timeout_sec').value)

        # 설정 및 믹서 생성
        self.config = ThrusterConfig(
            neutral_pwm=int(self.get_parameter('neutral_pwm').value),
            max_forward_pwm=int(self.get_parameter('max_forward_pwm').value),
            max_reverse_pwm=int(self.get_parameter('max_reverse_pwm').value),
            deadband_us=int(self.get_parameter('deadband_us').value),
            left_trim=float(self.get_parameter('left_trim').value),
            right_trim=float(self.get_parameter('right_trim').value),
            invert_left=bool(self.get_parameter('invert_left').value),
            invert_right=bool(self.get_parameter('invert_right').value),
            max_slew_rate=float(self.get_parameter('max_slew_rate').value),
        )
        self.mixer = ThrusterMixer(self.config)

        # 백엔드 초기화
        self.backend: BaseThrusterBackend = self._init_backend(hw_type)
        if not self.backend.open():
            self.get_logger().error(f"하드웨어 백엔드 '{hw_type}' 초기화 실패! 중립 PWM 유지.")

        # 상태 변수
        self.last_cmd_time: float = 0.0
        self.last_rc_time: float = 0.0
        self.current_cmd = Twist()
        self.current_rc_cmd = Twist()
        self.emergency_stopped = False
        self.active_mode = 'idle'   # 'auto', 'rc_override', 'estop', 'timeout', 'idle'
        self.last_left_pwm = self.config.neutral_pwm
        self.last_right_pwm = self.config.neutral_pwm

        # 구독자
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
        if self.enable_rc:
            self.create_subscription(Twist, '/rc/cmd_vel', self._on_rc_cmd, 10)

        # 진단 퍼블리셔
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # 50Hz 주기 제어 타이머
        timer_period = 1.0 / max(1.0, self.update_rate_hz)
        self.create_timer(timer_period, self._control_loop)
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"thruster_driver 시작 (백엔드={hw_type}, 중립={self.config.neutral_pwm}us, "
            f"전진상한={self.config.max_forward_pwm}us, 후진상한={self.config.max_reverse_pwm}us, "
            f"주기={self.update_rate_hz}Hz)")

    def _init_backend(self, hw_type: str) -> BaseThrusterBackend:
        if hw_type == 'serial':
            return SerialBackend(port=self.port, baudrate=self.baudrate)
        elif hw_type == 'pca9685':
            i2c_bus = int(self.get_parameter('i2c_bus').value)
            addr = int(self.get_parameter('pca9685_address').value)
            ch_l = int(self.get_parameter('left_channel').value)
            ch_r = int(self.get_parameter('right_channel').value)
            return PCA9685Backend(i2c_bus=i2c_bus, address=addr,
                                  left_channel=ch_l, right_channel=ch_r)
        else:
            return DummyBackend()

    def _on_cmd_vel(self, msg: Twist):
        self.current_cmd = msg
        self.last_cmd_time = time.monotonic()

    def _on_rc_cmd(self, msg: Twist):
        self.current_rc_cmd = msg
        self.last_rc_time = time.monotonic()

    def _on_estop(self, msg: Bool):
        if msg.data and not self.emergency_stopped:
            self.get_logger().warn('비상 정지(E-Stop) 발동! 모터 출력 즉각 차단.')
        elif not msg.data and self.emergency_stopped:
            self.get_logger().info('비상 정지(E-Stop) 해제.')
        self.emergency_stopped = msg.data

    def _control_loop(self):
        now = time.monotonic()

        # 1. E-Stop 상태 확인
        if self.emergency_stopped:
            self.active_mode = 'estop'
            target_linear = 0.0
            target_angular = 0.0
            self.mixer.reset()
        # 2. RC Manual Override 확인 (유효한 RC 명령이 오고 있는 경우 우선 적용)
        elif self.enable_rc and (now - self.last_rc_time <= self.rc_timeout_sec):
            self.active_mode = 'rc_override'
            target_linear = self.current_rc_cmd.linear.x
            target_angular = self.current_rc_cmd.angular.z
        # 3. 자율주행 명령 및 워치독 확인
        elif (now - self.last_cmd_time <= self.timeout_sec):
            self.active_mode = 'auto'
            target_linear = self.current_cmd.linear.x
            target_angular = self.current_cmd.angular.z
        else:
            self.active_mode = 'timeout'
            target_linear = 0.0
            target_angular = 0.0

        # 차동 믹싱 및 슬루 레이트 제한 적용
        left_pwm, right_pwm, _, _ = self.mixer.mix(target_linear, target_angular, now)

        # 하드웨어 백엔드로 PWM 전송
        self.backend.send_pwm(left_pwm, right_pwm)
        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm

    def _publish_diagnostics(self):
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'Thruster System'
        status.hardware_id = 'kaboat_thrusters'

        if self.emergency_stopped:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Emergency Stop Active'
        elif self.active_mode == 'timeout':
            status.level = DiagnosticStatus.WARN
            status.message = 'Watchdog Timeout (Neutral PWM)'
        elif self.active_mode == 'rc_override':
            status.level = DiagnosticStatus.OK
            status.message = 'RC Manual Override Active'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Autonomous Drive Active'

        status.values = [
            KeyValue(key='mode', value=self.active_mode),
            KeyValue(key='left_pwm', value=str(self.last_left_pwm)),
            KeyValue(key='right_pwm', value=str(self.last_right_pwm)),
            KeyValue(key='linear_cmd', value=f"{self.current_cmd.linear.x:.3f}"),
            KeyValue(key='angular_cmd', value=f"{self.current_cmd.angular.z:.3f}"),
        ]

        diag_arr.status.append(status)
        self.diag_pub.publish(diag_arr)

    def destroy_node(self):
        # 종료 시 모터 중립 안전 정지
        try:
            self.backend.send_pwm(self.config.neutral_pwm, self.config.neutral_pwm)
            self.backend.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterDriver()
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

