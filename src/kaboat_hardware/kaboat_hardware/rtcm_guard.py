"""rtcm_guard — NTRIP 보정(/rtcm)을 GQ7 드라이버(/rtcm/gq7)로 중계하며 AUX 쓰기 실패·RTK 상태를 감시.

run_gq7_rtk(real_sensors.launch.py rtk:=true)가 켠다. GQ7 은 RTK 보정(RTCM)을 AUX 포트로만 받는데,
AUX 연결이 망가지면 드라이버가 받은 RTCM 마다 'Failed to write RTCM to aux port' 를 찍는다
(초당 ~10줄). 2026-11-13 토너먼트 시험: 11:36 부터 끝까지 3,293줄(로그의 79 %)이 쏟아져 녹화가
밀렸고, 보정이 안 들어가 RTK Fixed → SBAS 로 떨어진 채 7번 시험을 다 했다 — 아무도 몰랐다.

이 노드:
- /rosout 에서 그 실패 로그를 세어 짧은 시간에 여러 번이면 **중계를 멈춘다** (어차피 GQ7 에 안 들어가는
  보정이라 손해 없음, 로그 폭주만 멈춤). probe_period 마다 하나씩 다시 넣어 보고, probe_wait 안에 실패가
  없으면 다시 중계한다 (USB 를 다시 꽂아 복구된 경우).
- 멈춘 동안 원인·해결을 report_period 마다 한 번 크게 알린다.
- /ekf/status 의 gnss_state 로 "보정은 들어가는데 RTK 가 아님", "NTRIP 보정이 안 들어옴" 도 알린다.
- /gnss/rtk_status (String) 로 지금 상태를 1 Hz 로 낸다 (녹화·원격 모니터용).
"""

import collections
from dataclasses import dataclass

FAIL_TEXT = 'Failed to write RTCM'
RTK_STATES = ('RTK Fixed', 'RTK Float')


@dataclass
class GuardParams:
    fail_limit: int = 5            # fail_window 안 실패 로그가 이만큼이면 중계를 멈춤
    fail_window: float = 5.0       # [s]
    probe_period: float = 10.0     # [s] 멈춘 동안 보정 하나씩 다시 넣어 보는 주기
    probe_wait: float = 2.0        # [s] 넣어 본 뒤 이 안에 실패 로그가 없으면 복구로 봄
    report_period: float = 30.0    # [s] 멈춘 동안 안내 주기
    rtk_grace: float = 120.0       # [s] 보정이 들어가는데 RTK 가 아닌 채 이만큼이면 경고
    rtcm_timeout: float = 10.0     # [s] NTRIP 보정이 이만큼 안 오면 경고
    warn_period: float = 60.0      # [s] RTK·보정 경고 반복 주기


class RtcmGuardLogic:
    """ROS 없는 판단부. 시각 t 는 단조 증가하는 초."""

    def __init__(self, p: GuardParams = GuardParams()):
        self.p = p
        self.forwarding = True
        self.failures = collections.deque()
        self.total_failures = 0
        self.dropped = 0
        self.next_probe = None
        self.probe_sent_at = None
        self.last_report = None
        self.gnss_state = None
        self.not_rtk_since = None
        self.last_rtcm = None
        self.started_at = None
        self.last_rtk_warn = None
        self.last_rtcm_warn = None

    # ── 입력 ───────────────────────────────────────
    def on_failure_log(self, t):
        """드라이버의 'Failed to write RTCM' 로그 한 줄. 알릴 문장 목록을 돌려준다."""
        self.total_failures += 1
        self.failures.append(t)
        while self.failures and t - self.failures[0] > self.p.fail_window:
            self.failures.popleft()
        if self.probe_sent_at is not None:          # 넣어 본 것도 실패 — 계속 멈춤
            self.probe_sent_at = None
            self.next_probe = t + self.p.probe_period
            return []
        if self.forwarding and len(self.failures) >= self.p.fail_limit:
            self.forwarding = False
            self.next_probe = t + self.p.probe_period
            self.last_report = t
            return [('error', self.paused_text())]
        return []

    def forward(self, t) -> bool:
        """NTRIP 보정 하나가 왔다 — GQ7 로 넘길지."""
        self.last_rtcm = t
        if self.started_at is None:
            self.started_at = t
        if self.forwarding:
            return True
        if self.probe_sent_at is None and t >= self.next_probe:
            self.probe_sent_at = t
            return True
        self.dropped += 1
        return False

    def on_gnss_state(self, state, t):
        self.gnss_state = state
        if state in RTK_STATES:
            self.not_rtk_since = None
        elif self.not_rtk_since is None:
            self.not_rtk_since = t

    # ── 주기 ───────────────────────────────────────
    def tick(self, t):
        """1 Hz 정도로 부른다. 알릴 문장 [(등급, 문장)] 를 돌려준다."""
        out = []
        if self.started_at is None:
            self.started_at = t
        if self.probe_sent_at is not None and t - self.probe_sent_at > self.p.probe_wait:
            self.probe_sent_at = None
            self.forwarding = True
            self.failures.clear()
            out.append(('info', f'GQ7 AUX 포트 쓰기 복구 — RTK 보정 중계 다시 시작 '
                                f'(멈춘 동안 버린 보정 {self.dropped}개)'))
            self.dropped = 0
        if not self.forwarding and t - self.last_report >= self.p.report_period:
            self.last_report = t
            out.append(('error', self.paused_text()))
        if self.forwarding:
            no_rtcm = (self.last_rtcm is None and t - self.started_at > self.p.rtcm_timeout) or (
                self.last_rtcm is not None and t - self.last_rtcm > self.p.rtcm_timeout)
            if no_rtcm and self._due('last_rtcm_warn', t):
                since = t - (self.last_rtcm if self.last_rtcm is not None else self.started_at)
                out.append(('warn', f'NTRIP 보정(/rtcm)이 {since:.0f} s 째 안 들어옴 — 인터넷 연결·'
                                    f'NTRIP 계정·ntrip_client 로그 확인 (지금 GNSS: {self.gnss_state})'))
            elif (not no_rtcm and self.not_rtk_since is not None
                  and t - self.not_rtk_since > self.p.rtk_grace and self._due('last_rtk_warn', t)):
                out.append(('warn', f'보정은 GQ7 에 들어가는데 {t - self.not_rtk_since:.0f} s 째 RTK 아님 '
                                    f'({self.gnss_state}) — 안테나 하늘 시야·기준국 거리 확인'))
        return out

    def _due(self, attr, t):
        last = getattr(self, attr)
        if last is None or t - last >= self.p.warn_period:
            setattr(self, attr, t)
            return True
        return False

    # ── 표시 ───────────────────────────────────────
    def paused_text(self):
        return (f"GQ7 AUX 포트에 RTK 보정을 못 씀 — 드라이버 '{FAIL_TEXT} to aux port' "
                f"{len(self.failures)}번/{self.p.fail_window:.0f} s (누적 {self.total_failures}) → "
                f"RTK 안 됨 (지금 GNSS: {self.gnss_state}). 해결: GQ7 USB 를 다시 꽂고 run_gq7_rtk 재시작 "
                f"(ls -l /dev/microstrain_aux, sudo fuser -v /dev/microstrain_aux 확인). 로그 폭주를 막으려고 "
                f"보정 중계를 멈춤 — {self.p.probe_period:.0f} s 마다 하나씩 다시 넣어 봄")

    def status(self):
        g = self.gnss_state or 'GNSS ?'
        if not self.forwarding:
            return f'{g} | AUX 쓰기 실패 — 보정 중계 멈춤'
        return f'{g} | 보정 중계 중'


def build_node():
    """rtcm_guard 노드 (rclpy.init 뒤에 부른다)."""
    import time
    from rcl_interfaces.msg import Log
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rtcm_msgs.msg import Message
    from std_msgs.msg import String

    class RtcmGuard(Node):
        def __init__(self):
            super().__init__('rtcm_guard')
            d = GuardParams()
            p = GuardParams(**{k: type(getattr(d, k))(self.declare_parameter(k, getattr(d, k)).value)
                               for k in d.__dataclass_fields__})
            self.logic = RtcmGuardLogic(p)
            self.pub = self.create_publisher(Message, '/rtcm/gq7', 100)
            self.status_pub = self.create_publisher(String, '/gnss/rtk_status', 10)
            self.create_subscription(Message, '/rtcm', self._on_rtcm, 100)
            self.create_subscription(Log, '/rosout', self._on_log,
                                     QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
            try:
                from microstrain_inertial_msgs.msg import HumanReadableStatus
                self.create_subscription(HumanReadableStatus, '/ekf/status', self._on_status, 10)
            except ImportError:
                self.get_logger().warn('microstrain_inertial_msgs 없음 — RTK 상태(gnss_state)는 안 봄')
            self.create_timer(1.0, self._tick)
            self.get_logger().info('RTK 보정 중계: /rtcm → /rtcm/gq7 (GQ7 AUX 쓰기 실패 감시)')

        def _say(self, items):
            # 등급마다 다른 줄에서 부른다 — rclpy 는 한 호출 위치에서 등급이 바뀌면 예외를 낸다
            # (회피 노드가 이것 때문에 ESCAPE 복귀 때 풀렸다, 2026-11-13)
            for level, text in items:
                if level == 'error':
                    self.get_logger().error(text)
                elif level == 'warn':
                    self.get_logger().warn(text)
                else:
                    self.get_logger().info(text)

        def _on_rtcm(self, msg):
            if self.logic.forward(time.monotonic()):
                self.pub.publish(msg)

        def _on_log(self, msg):
            if msg.name != self.get_name() and FAIL_TEXT in msg.msg:
                self._say(self.logic.on_failure_log(time.monotonic()))

        def _on_status(self, msg):
            self.logic.on_gnss_state(str(msg.gnss_state), time.monotonic())

        def _tick(self):
            self._say(self.logic.tick(time.monotonic()))
            self.status_pub.publish(String(data=self.logic.status()))

    return RtcmGuard()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = build_node()
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
