"""buoy_tracker — 부표 랜드마크 시간 융합 (ROS 무의존 순수 모듈).

buoy_detector 노드가 프레임마다 만든 **월드프레임 검출 목록**을 받아,
같은 부표를 프레임 간 연결하고(데이터 결합) · 몇 프레임 연속 보인 것만
확정하고(N-of-M) · 한동안 안 보이면 지우는(prune) 지속 랜드마크 지도를
유지한다. 출력은 확정된 부표들의 전역좌표 목록.

이 층을 노드에서 떼어 순수 모듈로 둔 이유는 avoid_fsm 과 같다 — 트래커의
진짜 버그(언제 승격? 언제 삭제? 두 부표를 하나로 합치나?)는 전이·타이머
로직이라, rclpy·카메라 없이 **가짜 시계 + 합성 검출 스트림**으로만 제대로
테스트된다(test/test_buoy_tracker.py).

좌표 규약(REP-103): body frame x=전방, y=좌측(+), bearing 좌측(+).
월드프레임 = odom (buoy_detector 가 /odom pose 로 투영해 넘긴다).
"""
import math
from dataclasses import dataclass, field
from typing import List, Dict


def project_to_world(bearing: float, distance: float,
                     boat_x: float, boat_y: float, yaw: float) -> tuple:
    """(bearing[rad,좌+], distance[m]) 상대검출 → 월드 (x, y).

    배 기준 극좌표를 body 직교(x=전방, y=좌측)로 풀고, 배 yaw 로 회전 +
    배 위치로 평행이동한다. obstacle_planner 의 body→world twist 회전과
    같은 부호 규약(c*vx - s*vy, s*vx + c*vy).
    """
    xb = distance * math.cos(bearing)   # 전방
    yb = distance * math.sin(bearing)   # 좌측(+)
    c, s = math.cos(yaw), math.sin(yaw)
    return (boat_x + c * xb - s * yb,
            boat_y + s * xb + c * yb)


@dataclass
class Detection:
    """한 프레임의 부표 검출 1개 (이미 월드좌표로 투영됨)."""
    color: str
    x: float
    y: float
    confidence: float = 1.0


@dataclass
class TrackerParams:
    gate_radius: float = 1.5   # m — 검출↔트랙 결합 최대 거리. 부표 최소 간격보다
                               #     작아야 서로 다른 부표가 하나로 안 합쳐진다.
    confirm_hits: int = 3      # N — 이만큼 누적 관측돼야 "진짜"로 확정·발행
    drop_time: float = 3.0     # s — 마지막 관측 후 이 시간 지나면 트랙 삭제
    ema_alpha: float = 0.3     # 위치 평활: 새 관측 가중치(0=고정, 1=최신값만)


@dataclass
class Track:
    id: int
    x: float
    y: float
    hits: int
    last_seen: float
    confidence: float
    color_votes: Dict[str, int] = field(default_factory=dict)

    @property
    def color(self) -> str:
        """지금까지 가장 많이 관측된 색 (물결 반짝임 한두 프레임에 안 흔들리게)."""
        if not self.color_votes:
            return 'unknown'
        return max(self.color_votes.items(), key=lambda kv: kv[1])[0]

    def confirmed(self, confirm_hits: int) -> bool:
        return self.hits >= confirm_hits


class BuoyTracker:
    """월드프레임 부표 검출 스트림 → 확정 랜드마크 목록.

    update(detections, now) 를 프레임마다 부르면 되고, 반환값이 그 시점의
    확정 부표 목록이다. now 는 벽시계가 아니라 odom 스탬프[s](sim RTF<1 대응).
    """

    def __init__(self, params: TrackerParams = None):
        self.p = params or TrackerParams()
        self.tracks: List[Track] = []
        self._next_id = 0

    def update(self, detections: List[Detection], now: float) -> List[Track]:
        gate2 = self.p.gate_radius ** 2
        claimed = set()   # 이 프레임에 이미 매칭된 트랙 id (1검출:1트랙)

        for det in detections:
            best = None
            best_d2 = gate2
            for tr in self.tracks:
                if tr.id in claimed:
                    continue
                d2 = (tr.x - det.x) ** 2 + (tr.y - det.y) ** 2
                if d2 <= best_d2:
                    best_d2 = d2
                    best = tr
            if best is not None:
                self._reinforce(best, det, now)
                claimed.add(best.id)
            else:
                self._spawn(det, now)

        self._prune(now)
        return self.confirmed()

    def confirmed(self) -> List[Track]:
        return [tr for tr in self.tracks if tr.confirmed(self.p.confirm_hits)]

    # ── 내부 ──────────────────────────────────────────────────

    def _spawn(self, det: Detection, now: float):
        tr = Track(id=self._next_id, x=det.x, y=det.y, hits=1,
                   last_seen=now, confidence=det.confidence,
                   color_votes={det.color: 1})
        self._next_id += 1
        self.tracks.append(tr)

    def _reinforce(self, tr: Track, det: Detection, now: float):
        a = self.p.ema_alpha
        tr.x = (1.0 - a) * tr.x + a * det.x
        tr.y = (1.0 - a) * tr.y + a * det.y
        tr.hits += 1
        tr.last_seen = now
        tr.confidence = (1.0 - a) * tr.confidence + a * det.confidence
        tr.color_votes[det.color] = tr.color_votes.get(det.color, 0) + 1

    def _prune(self, now: float):
        self.tracks = [tr for tr in self.tracks
                       if now - tr.last_seen <= self.p.drop_time]
