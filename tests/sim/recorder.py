"""L2/L3 시나리오 기록과 판정 지표 — ROS 무의존.

시나리오는 매 제어 step 마다 record() 를 부르고, 끝난 뒤 지표로 판정한다.
그림은 KABOAT_TEST_FIGURES 환경 변수가 있을 때만 저장한다
(평소 시험 실행에서 matplotlib 을 불러오지 않기 위해서다).
"""
from dataclasses import dataclass, field
import math
import os
from pathlib import Path

FIGURE_ENV = 'KABOAT_TEST_FIGURES'


@dataclass
class Sample:
    t: float
    x: float
    y: float
    yaw: float
    linear: float
    angular: float
    status: str = ''


@dataclass
class Recorder:
    samples: list = field(default_factory=list)
    events: list = field(default_factory=list)   # [(t, text)] — 완료 신호, 상태 전환 등

    def record(self, t, state, linear, angular, status=''):
        self.samples.append(Sample(t, state.x, state.y, state.yaw,
                                   float(linear), float(angular), str(status)))

    def event(self, t, text):
        self.events.append((t, str(text)))

    # ── 판정 지표 ──────────────────────────────────────
    def first_event_time(self, text):
        for t, e in self.events:
            if e == text:
                return t
        return None

    def min_clearance(self, world) -> float:
        return min((world.min_clearance(s.x, s.y) for s in self.samples), default=math.inf)

    def always_inside(self, world) -> bool:
        return all(world.inside_bounds(s.x, s.y) for s in self.samples)

    def final_distance_to(self, x, y) -> float:
        s = self.samples[-1]
        return math.hypot(s.x - x, s.y - y)

    def stopped_after(self, t0, eps=1e-9) -> bool:
        """t0 이후 모든 명령이 0 인지 (실패 주입 판정)."""
        later = [s for s in self.samples if s.t >= t0]
        return bool(later) and all(abs(s.linear) <= eps and abs(s.angular) <= eps for s in later)


def figure_dir():
    """그림 저장 폴더. 환경 변수가 없으면 None — 호출자는 그림을 건너뛴다."""
    path = os.environ.get(FIGURE_ENV)
    if not path:
        return None
    out = Path(path)
    out.mkdir(parents=True, exist_ok=True)
    return out
