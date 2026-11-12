"""게이트 부표 기억 — 지도에서 몇 번 잡힌 부표는 odom 좌표에 고정해 두고 계속 쓴다.

라이다는 작은 게이트 부표를 앞 5 m 안팎에서만 꾸준히 잡고 5~7.5 m 는 가끔 잡는다
(2026-11-12 녹화: 5~7.5 m 히트 스캔 25 %, 7.5 m 넘으면 1 %). 지도(occupancy grid)
덩어리는 잡혔다 사라졌다 하므로, 같은 자리(match_radius 안)에서 confirm_hits 번 잡히면
확정해 기억한다 — 다음 게이트까지 미리 이어 경로를 만들 수 있게(gate_follower 의 게이트 사슬).

잘못 기억한 것(물보라·반사)은 지운다: 배가 recheck_range 안으로 다가가 그 자리를 볼 수
있는데 forget_sec 넘게 안 잡히면 지움. 멀리 있는 것은 볼 기회가 없으니 그대로 둔다.
확정 전인 것은 forget_sec 넘게 다시 안 잡히면 거리와 상관없이 지운다.
위치는 잡힐 때마다 지수 평균으로 고친다(닻줄에 매인 부표는 0.3~0.5 m 움직임).
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class Post:
    xy: np.ndarray
    hits: int
    first_seen: float
    last_seen: float
    confirmed: bool = False
    in_range_since: float | None = None   # recheck_range 안에 들어온 시각 (밖이면 None)


@dataclass
class PostMemory:
    match_radius: float = 0.7
    confirm_hits: int = 3
    forget_sec: float = 1.0
    recheck_range: float = 4.0
    prune_range: float = 15.0      # 배에서 이보다 먼 것은 버림 (지나온 게이트 등)
    position_alpha: float = 0.3
    posts: list = field(default_factory=list)

    def __post_init__(self):
        values = (self.match_radius, self.forget_sec, self.recheck_range,
                  self.prune_range, self.position_alpha)
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or self.confirm_hits < 1 or self.position_alpha > 1.0
                or self.recheck_range >= self.prune_range):
            raise ValueError('invalid gate post memory parameters')

    def reset(self):
        self.posts = []

    def update(self, observations, now, boat_xy):
        """이번 지도 부표 덩어리(odom xy 목록)로 기억을 고친다."""
        boat = np.asarray(boat_xy, dtype=float)
        matched = set()
        for obs in observations:
            obs = np.asarray(obs, dtype=float)
            best, best_d = None, self.match_radius
            for i, post in enumerate(self.posts):
                d = float(np.linalg.norm(post.xy - obs))
                if d <= best_d and i not in matched:
                    best, best_d = i, d
            if best is None:
                self.posts.append(Post(obs.copy(), 1, now, now))
                matched.add(len(self.posts) - 1)
                continue
            post = self.posts[best]
            post.xy = (1.0 - self.position_alpha) * post.xy + self.position_alpha * obs
            post.hits += 1
            post.last_seen = now
            post.confirmed = post.confirmed or post.hits >= self.confirm_hits
            matched.add(best)
        kept = []
        for post in self.posts:
            distance = float(np.linalg.norm(post.xy - boat))
            if distance > self.prune_range:
                continue
            if distance <= self.recheck_range:
                if post.in_range_since is None:
                    post.in_range_since = now
            else:
                post.in_range_since = None
            if not post.confirmed:
                if now - post.last_seen > self.forget_sec:
                    continue
            elif (post.in_range_since is not None
                  and now - max(post.last_seen, post.in_range_since) > self.forget_sec):
                continue                      # 볼 수 있는 거리에 들어온 뒤로 계속 안 잡힘
            kept.append(post)
        self.posts = self._merged(kept)

    def _merged(self, posts):
        """위치를 고치다 match_radius 안으로 겹친 두 기억은 하나로 (먼저 본 쪽에 합침)."""
        out = []
        for post in sorted(posts, key=lambda p: p.first_seen):
            twin = next((o for o in out
                         if np.linalg.norm(o.xy - post.xy) < 0.5 * self.match_radius), None)
            if twin is None:
                out.append(post)
                continue
            total = twin.hits + post.hits
            twin.xy = (twin.xy * twin.hits + post.xy * post.hits) / total
            twin.hits = total
            twin.last_seen = max(twin.last_seen, post.last_seen)
            twin.confirmed = twin.confirmed or post.confirmed
        return out

    def confirmed(self):
        """확정된 부표 [(x, y), …]."""
        return [(float(p.xy[0]), float(p.xy[1])) for p in self.posts if p.confirmed]

    def with_current(self, current):
        """지금 지도 덩어리 + 지금은 안 잡히는 확정 기억 부표 (쌍 고르기·사슬 후보).

        지금 잡힌 것은 지도 위치 그대로 쓴다 — 기억이 비어 있으면 예전과 똑같다.
        """
        out = [(float(c[0]), float(c[1])) for c in current]
        now_xy = np.asarray(out, dtype=float).reshape(-1, 2)
        for post in self.posts:
            if not post.confirmed:
                continue
            if len(now_xy) and np.min(np.linalg.norm(now_xy - post.xy, axis=1)) <= self.match_radius:
                continue
            out.append((float(post.xy[0]), float(post.xy[1])))
        return out
