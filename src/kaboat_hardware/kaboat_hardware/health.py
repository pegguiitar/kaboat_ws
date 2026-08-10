"""ROS에 의존하지 않는 센서 수신 상태 계산."""

from dataclasses import dataclass


OK = 0
WARN = 1
ERROR = 2


@dataclass
class TopicTracker:
    """한 센서 토픽의 수신 횟수, 주기, 최신성을 추적한다."""

    enabled: bool
    topic: str
    min_rate_hz: float
    timeout_sec: float
    count: int = 0
    first_rx_sec: float = 0.0
    last_rx_sec: float = 0.0
    frame_id: str = ''

    def observe(self, now_sec: float, frame_id: str = '') -> None:
        if self.count == 0:
            self.first_rx_sec = now_sec
        self.count += 1
        self.last_rx_sec = now_sec
        self.frame_id = frame_id

    def rate_hz(self) -> float:
        span = self.last_rx_sec - self.first_rx_sec
        return (self.count - 1) / span if self.count > 1 and span > 0.0 else 0.0

    def evaluate(self, now_sec: float, node_start_sec: float,
                 startup_grace_sec: float):
        """(level, message, age, rate)를 반환한다."""
        if not self.enabled:
            return OK, 'disabled', 0.0, 0.0
        if self.count == 0:
            if now_sec - node_start_sec <= startup_grace_sec:
                return WARN, 'waiting for first message', 0.0, 0.0
            return ERROR, 'no messages', 0.0, 0.0

        age = max(0.0, now_sec - self.last_rx_sec)
        rate = self.rate_hz()
        if age > self.timeout_sec:
            return ERROR, 'stale', age, rate
        if (self.count >= 3 and self.min_rate_hz > 0.0
                and rate < self.min_rate_hz):
            return WARN, 'rate below minimum', age, rate
        if not self.frame_id:
            return WARN, 'empty frame_id', age, rate
        return OK, 'receiving', age, rate
