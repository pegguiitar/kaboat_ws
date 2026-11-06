"""gps_mark_logger — 노트북 키보드 Space 를 누를 때마다 현재 GPS 위치를 CSV 에 한 줄 기록.

시작하면 log_dir 에 gps_log_YYYYMMDD_HHMMSS.csv 를 만들고 첫 줄에
`timestamp,latitude,longitude` 를 쓴다. /teleop/gps_mark(Empty) 를 받을 때마다
그 시각과 최신 /gps/fix 위도·경도를 한 줄 추가하고 곧바로 디스크에 반영(flush+fsync)
하므로, 프로그램이 비정상 종료돼도 그 전까지의 기록은 남는다.

GPS 가 없거나(no fix) 오래됐으면(max_fix_age_sec) 줄을 쓰지 않고 /teleop/event 로
실패를 알린다. 노트북 조종 창이 그 결과를 표시한다.
"""

import csv
from datetime import datetime
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Empty, String

HEADER = ('timestamp', 'latitude', 'longitude')


def log_path(log_dir: str, now: datetime) -> str:
    return os.path.join(os.path.expanduser(log_dir),
                        now.strftime('gps_log_%Y%m%d_%H%M%S.csv'))


def format_timestamp(now: datetime) -> str:
    return now.strftime('%Y-%m-%d %H:%M:%S.') + f'{now.microsecond // 1000:03d}'


class GpsMarkCsv:
    """한 줄 쓸 때마다 디스크에 반영하는 CSV 기록기 (ROS 무의존)."""

    def __init__(self, path: str):
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        # 같은 초에 두 번 시작해도 덮어쓰지 않도록 'x'(새 파일만) 로 연다
        base, ext = os.path.splitext(path)
        candidate, n = path, 1
        while True:
            try:
                self._file = open(candidate, 'x', newline='', encoding='utf-8')
                break
            except FileExistsError:
                n += 1
                candidate = f'{base}_{n}{ext}'
        self.path = candidate
        self._writer = csv.writer(self._file)
        self.rows = 0
        self._write(HEADER)

    def _write(self, row):
        self._writer.writerow(row)
        self._file.flush()
        os.fsync(self._file.fileno())

    def append(self, timestamp: str, latitude: float, longitude: float):
        self._write((timestamp, f'{latitude:.8f}', f'{longitude:.8f}'))
        self.rows += 1

    def close(self):
        if not self._file.closed:
            self._file.close()


def fix_problem(fix, received_at, now, max_age):
    """기록 못 하는 이유 문자열, 기록 가능하면 None."""
    if fix is None:
        return 'GPS 수신 없음'
    if now - received_at > max_age:
        return f'GPS 가 {now - received_at:.1f} s 동안 갱신 안 됨'
    if fix.status.status < NavSatStatus.STATUS_FIX:
        return 'GPS 위치 미확정(no fix)'
    if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
        return 'GPS 좌표가 유효하지 않음'
    return None


class GpsMarkLogger(Node):
    def __init__(self, **node_kwargs):
        super().__init__('gps_mark_logger', **node_kwargs)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('log_dir', '~/kaboat_gps_logs')
        self.declare_parameter('max_fix_age_sec', 2.0)
        self.max_age = float(self.get_parameter('max_fix_age_sec').value)
        if not math.isfinite(self.max_age) or self.max_age <= 0.0:
            raise ValueError('max_fix_age_sec must be positive')

        self.log = GpsMarkCsv(log_path(
            str(self.get_parameter('log_dir').value), datetime.now()))
        self.fix = None
        self.fix_at = 0.0

        gps_topic = str(self.get_parameter('gps_topic').value)
        self.create_subscription(NavSatFix, gps_topic, self._on_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(Empty, '/teleop/gps_mark', self._on_mark, 10)
        self.event_pub = self.create_publisher(String, '/teleop/event', 10)
        self.get_logger().info(
            f'GPS 기록 파일: {self.log.path} (Space 로 기록, GPS 토픽 {gps_topic})')

    def _on_fix(self, msg: NavSatFix):
        self.fix = msg
        self.fix_at = time.monotonic()

    def _on_mark(self, _msg: Empty):
        stamp = format_timestamp(datetime.now())
        problem = fix_problem(self.fix, self.fix_at, time.monotonic(), self.max_age)
        if problem:
            self.get_logger().warn(f'GPS 기록 실패: {problem}')
            self._event(f'GPS 기록 실패: {problem}')
            return
        try:
            self.log.append(stamp, self.fix.latitude, self.fix.longitude)
        except OSError as exc:
            self.get_logger().error(f'GPS 기록 파일 쓰기 실패: {exc}')
            self._event(f'GPS 기록 실패: 파일 쓰기 오류 {exc}')
            return
        text = (f'GPS 기록 #{self.log.rows}: {self.fix.latitude:.7f}, '
                f'{self.fix.longitude:.7f} ({stamp})')
        self.get_logger().info(text)
        self._event(text)

    def _event(self, text):
        self.event_pub.publish(String(data=text))

    def destroy_node(self):
        self.log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsMarkLogger()
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
