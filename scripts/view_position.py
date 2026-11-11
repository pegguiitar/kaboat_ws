#!/usr/bin/env python3
"""/odom 위치·선수각을 한 줄로 실시간 표시 (젯슨·노트북 공용, 단축어 view_position).

  x, y      /odom 좌표 [m] — 원점(~/kaboat_geo_origin.yaml) 기준, x = 동쪽, y = 북쪽
  선수각    /odom 기준 [deg] — 동쪽 0°, 반시계 + (주행 코드·rec 가 쓰는 값)
  나침반    같은 방향을 북쪽 0°, 시계 + 로 (휴대폰 나침반과 비교용)
  속도      선체 기준 수평 속력 [m/s]
  GNSS      /ekf/status 의 gnss_state (RTK Fixed / RTK Float / SBAS …).
            microstrain_inertial_msgs 가 없는 환경(노트북 등)에서는 표시하지 않는다.

/odom 이 1초 넘게 안 오면 빨간색으로 경고한다. Ctrl+C 로 끈다.
"""

import math
import sys
import time

STALE_SEC = 1.0
RED, YELLOW, GREEN, RESET = '\033[91m', '\033[93m', '\033[92m', '\033[0m'


def heading_deg(q) -> float:
    """쿼터니언 (x, y, z, w) → /odom 선수각 [deg] (동쪽 0°, 반시계 +)."""
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def compass_deg(heading: float) -> float:
    """/odom 선수각 → 나침반 방위 [deg] (북쪽 0°, 시계 +, 0~360)."""
    return (90.0 - heading) % 360.0


def gnss_color(state: str) -> str:
    if state == 'RTK Fixed':
        return GREEN
    if state == 'RTK Float':
        return YELLOW
    return RED


def format_line(x, y, heading, speed, age, gnss_state=None) -> str:
    """표시할 한 줄. age = 마지막 /odom 이후 초."""
    text = (f'x {x:+8.2f} m  y {y:+8.2f} m  |  선수각 {heading:+7.1f}°  '
            f'나침반 {compass_deg(heading):5.1f}°  |  속도 {speed:4.2f} m/s')
    if gnss_state is not None:
        text += f'  |  GNSS {gnss_color(gnss_state)}{gnss_state}{RESET}'
    if age > STALE_SEC:
        text = f'{RED}[/odom {age:.1f}초 끊김]{RESET} ' + text
    return text


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    try:
        from microstrain_inertial_msgs.msg import HumanReadableStatus
    except ImportError:
        HumanReadableStatus = None

    class PositionViewer(Node):
        def __init__(self):
            super().__init__('position_viewer')
            self.odom = None
            self.odom_at = None
            self.gnss_state = None
            self.create_subscription(Odometry, '/odom', self._on_odom, 10)
            if HumanReadableStatus is not None:
                self.create_subscription(HumanReadableStatus, '/ekf/status', self._on_status, 10)
            self.create_timer(0.2, self._show)          # 5 Hz 로 화면 갱신
            print('대기 중: /odom 수신을 기다립니다... (Ctrl+C 로 종료)', flush=True)

        def _on_odom(self, msg):
            self.odom = msg
            self.odom_at = time.monotonic()

        def _on_status(self, msg):
            self.gnss_state = msg.gnss_state

        def _show(self):
            if self.odom is None:
                return
            p = self.odom.pose.pose.position
            o = self.odom.pose.pose.orientation
            v = self.odom.twist.twist.linear
            line = format_line(p.x, p.y, heading_deg((o.x, o.y, o.z, o.w)),
                               math.hypot(v.x, v.y), time.monotonic() - self.odom_at,
                               self.gnss_state)
            sys.stdout.write('\r\033[K' + line)
            sys.stdout.flush()

    rclpy.init(args=args)
    node = PositionViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\n종료합니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
