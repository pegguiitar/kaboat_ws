"""buoy_tracker — 부표 랜드마크 시간 융합 (ROS 무의존 순수 모듈).

buoy_detector 노드가 프레임마다 만든 **월드프레임 검출 목록**을 받아,
같은 부표를 프레임 간 연결하고(데이터 결합) · 몇 프레임 연속 보인 것만
확정하고(N-of-M) · 지울 것만 지우는(prune) 지속 랜드마크 지도를 유지한다.
출력은 확정된 부표들의 전역좌표 목록.

삭제 조건 (2026-11-12 변경 — 예전엔 마지막 관측 후 drop_time 지나면 무조건 삭제):
  확정 부표는 "안 보인다"고 지우지 않는다. 배가 지나가거나 돌면 부표는 카메라 시야
  밖(옆·뒤)으로 나가는데, 탐색 선회·게이트처럼 그 부표를 계속 써야 하기 때문이다.
  **보여야 하는 자리인데 검출이 안 될 때만** 놓친 시간(missed)을 쌓아 drop_time 을
  넘으면 지운다. 보여야 하는 자리 = 카메라 전방 ±view_half_fov, 거리
  view_range_min ~ view_range_max(뎁스 경로 6 m), 더 가까운 다른 확정 부표에
  가리지 않음 (in_view). 배 pose 를 안 주면(update(..., boat_pose=None) — 뎁스가 없어
  검출을 못 믿는 프레임) 놓친 시간을 쌓지 않는다.
  미확정 트랙(물결·반짝임 노이즈)은 예전처럼 마지막 관측 후 drop_time 지나면 지운다.

이 층을 노드에서 떼어 순수 모듈로 둔 이유는 avoid_fsm 과 같다 — 트래커의
진짜 버그(언제 승격? 언제 삭제? 두 부표를 하나로 합치나?)는 전이·타이머
로직이라, rclpy·카메라 없이 **가짜 시계 + 합성 검출 스트림**으로만 제대로
테스트된다(test/test_buoy_tracker.py).

좌표 규약(REP-103): body frame x=전방, y=좌측(+), bearing 좌측(+).
월드프레임 = odom (buoy_detector 가 /odom pose 로 투영해 넘긴다).
"""
import math
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

# 프레임 간격 상한 [s] — 카메라가 한동안 끊겼다 돌아온 첫 프레임에 놓친 시간이 한 번에
# 쌓여 부표가 지워지지 않게.
MAX_MISS_STEP = 0.5


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
    drop_time: float = 3.0     # s — 확정: 보여야 하는데 놓친 시간이 이만큼이면 삭제
                               #     미확정: 마지막 관측 후 이만큼 지나면 삭제
    # "보여야 하는 자리" (확정 부표 삭제 판정) — D455 수평 화각 87° = ±43.5° 에 여유 5°
    view_half_fov: float = math.radians(38.0)   # rad — 선수 기준 좌우
    view_range_min: float = 1.5  # m — 이보다 가까우면 화면 아래로 빠질 수 있음
    view_range_max: float = 6.0  # m — 뎁스 경로 한계. 너머는 라이다(배가 기울면 끊김)라 안 믿음
    occlusion_radius: float = 0.5  # m — 더 가까운 확정 부표의 이 반경 뒤는 가려진 것으로 봄
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
    missed: float = 0.0        # s — 보여야 하는 자리에서 연속으로 놓친 시간

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

    update(detections, now, boat_pose) 를 프레임마다 부르면 되고, 반환값이 그
    시점의 확정 부표 목록이다. now 는 벽시계가 아니라 odom 스탬프[s](sim RTF<1 대응).
    boat_pose = (x, y, yaw) — 확정 부표를 지울지 판단할 시야. None 이면 안 지운다.
    """

    def __init__(self, params: TrackerParams = None):
        self.p = params or TrackerParams()
        self.tracks: List[Track] = []
        self._next_id = 0
        self._last_now: Optional[float] = None

    def update(self, detections: List[Detection], now: float,
               boat_pose: Optional[Tuple[float, float, float]] = None) -> List[Track]:
        step = 0.0 if self._last_now is None else min(max(now - self._last_now, 0.0),
                                                       MAX_MISS_STEP)
        self._last_now = now
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

        if boat_pose is not None:
            for tr in self.confirmed():
                if tr.id not in claimed and self.in_view(tr, boat_pose):
                    tr.missed += step
        self._prune(now)
        return self.confirmed()

    def confirmed(self) -> List[Track]:
        return [tr for tr in self.tracks if tr.confirmed(self.p.confirm_hits)]

    def in_view(self, tr: Track, boat_pose: Tuple[float, float, float]) -> bool:
        """tr 이 지금 카메라에 보여야 하는 자리인가 (시야·거리 안, 앞 부표에 안 가림)."""
        bx, by, yaw = boat_pose
        dist = math.hypot(tr.x - bx, tr.y - by)
        if not self.p.view_range_min <= dist <= self.p.view_range_max:
            return False
        bearing = math.atan2(tr.y - by, tr.x - bx)
        if abs(_wrap(bearing - yaw)) > self.p.view_half_fov:
            return False
        for other in self.confirmed():
            if other is tr:
                continue
            d = math.hypot(other.x - bx, other.y - by)
            if d >= dist or d <= 0.0:
                continue
            ob = math.atan2(other.y - by, other.x - bx)
            if abs(_wrap(ob - bearing)) <= math.atan2(self.p.occlusion_radius, d):
                return False   # 더 가까운 부표 뒤 — 못 보는 게 당연
        return True

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
        tr.missed = 0.0
        tr.confidence = (1.0 - a) * tr.confidence + a * det.confidence
        tr.color_votes[det.color] = tr.color_votes.get(det.color, 0) + 1

    def _prune(self, now: float):
        keep = []
        for tr in self.tracks:
            if tr.confirmed(self.p.confirm_hits):
                alive = tr.missed < self.p.drop_time
            else:
                alive = now - tr.last_seen <= self.p.drop_time
            if alive:
                keep.append(tr)
        self.tracks = keep


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))
