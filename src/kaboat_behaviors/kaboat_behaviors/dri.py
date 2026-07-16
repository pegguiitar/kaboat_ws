"""dri — occupancy_grid 의 점유 셀을 위험도(DRI) 격자로 바꾸는 순수 라이브러리.

AVOIDANCE_PLAN.MD §1.1 의 구현. ROS 의존 없음(N3) — `build_dri` 는
OccupancyGrid **메시지 모양의 객체**를 덕타이핑으로 읽을 뿐이라, 유닛테스트는
같은 필드를 가진 수제 객체를 넘기면 된다. 메시지 파싱은 이 모듈에서 끝나고
이후 전 모듈(bspline_planner, obstacle_planner)은 `DriGrid` 만 안다.

위험도 정의 (R1):
  점유 셀 c 하나가 격자점 x 에 주는 위험을 **등방 가우시안**으로 놓고, 배와
  가깝거나(g) 선수 정면에 있을수록(f) 더 크고(σ) 진하게(A) 퍼뜨린다.

    DRI(x) = max_c A(c)·exp(−‖x−c‖² / 2σ(c)²)

  셀 결합은 **max** — 합산(Σ)이 아니다 (§7-2). 충돌 안전은 **최근접 장애물**이
  결정한다: 배는 가장 가까운 것에 부딪히지 여러 장애물의 합에 부딪히지 않는다.
  합산은 "하나에 가까움"과 "여럿에 적당히 가까움"을 뒤섞어, 통과 가능한 3m 틈과
  최근접 2m 인 밀집 지대를 **오답으로 차단**한다 — 부표 밭에서 전부 차단되어
  generate→None→ESCAPE 로 배가 포기하는, 정확히 avoid 미션의 실패 모드다.
  max 는 DRI 를 [0, max A] 로 **유계**로 만들어 §1.3 비용 균형도 안정시킨다.

  게인(k_*)을 아무리 키워도 **원이 커질 뿐 찌그러지지 않는다**. 이전 비등방
  버전은 배→장애물 축(σ_along)만 키우고 수직축(σ_cross)은 고정해서, 게인이
  붙는 순간 종횡비가 6:1 을 넘고 장애물마다 배를 향한 시가 모양이 생겼다 —
  합치면 배에서 뻗어나가는 V자 줄기가 지도를 덮었다. 게다가 배가 장애물을
  얼마나 떨어져 지나갈지(berth)는 원호 위 방위각으로 결정되는데 그 방향이
  하필 고정축(σ_cross)이라, 게인을 올려도 berth 가 넓어지지 않았다.
  등방이면 σ 가 곧 berth 라 게인이 의도대로 먹는다.

미관측(-1) 셀은 기여 0 = free 취급 (R8) — 위험 취급하면 센서 범위 밖으로 영영
못 나간다. `occ_threshold` 비교에서 -1 은 자연히 탈락한다.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class DriParams:
    """§3 파라미터 표의 dri.* — 노드가 ROS param 에서 채워 넘긴다."""
    # 이 점유확률[0~100] 이상 셀만 위험원. occupancy_grid 의 log-odds 특성상 발행값이
    # 히트 횟수의 함수다(1회→70, 2회→85, 3회→93). ⚠️ 원거리 부표는 스캔이 성겨
    # 셀당 1히트(=70)에 머물러 영영 포화되지 않으므로, 75(2회 요구)로 두면 실제
    # 부표가 DRI 에서 투명해진다 — sim 실측 후 50 으로 확정 (§7-6). 단발 노이즈까지
    # 장애물이 되는 대가는 실선 LiDAR 노이즈를 보고 재검토.
    occ_threshold: int = 50
    sigma: float = 0.6          # 등방 σ **상한**(코앞·정면일 때) [m] — 곧 berth 폭.
    #   1.0 은 3m 격자 부표 밭에서 틈 중앙 risk 를 threshold 위로 밀어 올려
    #   전부 "막힘" 판정 → 배가 밭을 통째로 우회했다 (§7-6).
    dist_falloff: float = 4.0   # 근접성 g(ρ)=1/(1+ρ/ρ₀) 의 ρ₀ [m]
    # σ 게인 — ⚠️ 확대가 아니라 **감쇄 폭**이다 (2026-07-17, 사용자 방향 지시):
    # σ 는 최대 정규화라 코앞·정면 = sigma 그대로, 멀고 옆일수록
    # sigma/((1+k_dist)(1+k_head)) 까지 줄어든다. 위로 부풀리는 구식은 정면
    # 중거리 σ 를 0.88m 로 키워 3m 격자 틈(히트 초승달 감안 유효 2.4m)을
    # 물리적으로 닫았다 — 실측 pinch 1.12 > threshold.
    k_dist: float = 0.3
    k_head: float = 0.3
    a_dist: float = 1.0         # 진폭 A 근접 확대 게인
    a_head: float = 0.7         # 진폭 A 전방성 확대 게인 — 0.3 은 정면/측면 경사가
    #   화면에서 안 보일 만큼 약했고(1.3배), 그 결정 자체가 배가 반대를 보던
    #   오염된 관찰이었다. 0.7(1.7배)은 k 정규화 후 통로 개방과 양립 (2026-07-17).
    # 게인 넷을 0 으로 두면 크기·세기가 고정된 균일 가우시안으로 축약(디버그 기준선).


@dataclass
class DriGrid:
    data: np.ndarray             # (H, W) float32 — 위험도 ≥ 0, [iy, ix]
    anchor: tuple                # (ax, ay) — 배열 (0,0) 셀의 global 격자 인덱스
    resolution: float            # [m/cell]
    # 점유 셀 중심 world 좌표 (K, 2). bspline_planner 의 기준 장애물(r1) 선택용 —
    # 위험도 배열만으론 "장애물이 어디 있나"를 복원할 수 없어서 함께 나른다.
    occ_xy: np.ndarray = None

    def risk_at(self, x: float, y: float) -> float:
        """world 좌표의 DRI 값. window 밖이면 inf (계획이 창 밖으로 못 나가게)."""
        ix = math.floor(x / self.resolution) - self.anchor[0]
        iy = math.floor(y / self.resolution) - self.anchor[1]
        h, w = self.data.shape
        if not (0 <= ix < w and 0 <= iy < h):
            return math.inf
        return float(self.data[iy, ix])

    def risk_at_many(self, xs, ys) -> np.ndarray:
        """risk_at 의 벡터화판 — check() 가 샘플 수십~수백 개를 한 번에 본다."""
        xs = np.asarray(xs, dtype=np.float64)
        ys = np.asarray(ys, dtype=np.float64)
        ix = np.floor(xs / self.resolution).astype(np.int64) - self.anchor[0]
        iy = np.floor(ys / self.resolution).astype(np.int64) - self.anchor[1]
        h, w = self.data.shape
        inside = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        out = np.full(xs.shape, np.inf, dtype=np.float64)
        out[inside] = self.data[iy[inside], ix[inside]]
        return out


def build_dri(grid, boat_xy, boat_yaw: float, p: DriParams) -> DriGrid:
    """OccupancyGrid msg → DriGrid (§1.1).

    grid 는 nav_msgs/OccupancyGrid 와 같은 필드(info.resolution/width/height,
    info.origin.position, data)를 가진 객체면 무엇이든 된다(N3).
    """
    res = float(grid.info.resolution)
    w, h = int(grid.info.width), int(grid.info.height)
    # origin 은 occupancy_grid 가 anchor×resolution 으로 격자에 스냅해 발행한다
    # (그쪽 _publish 참고) — 되돌릴 때 round 라야 부동소수 오차로 ±1셀 안 튄다.
    anchor = (int(round(grid.info.origin.position.x / res)),
              int(round(grid.info.origin.position.y / res)))

    data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
    dri = np.zeros((h, w), dtype=np.float32)

    # unknown(-1) 은 occ_threshold 비교에서 자연히 탈락 = free 취급 (R8)
    oy, ox = np.nonzero(data >= p.occ_threshold)
    if oy.size == 0:
        return DriGrid(data=dri, anchor=anchor, resolution=res,
                       occ_xy=np.empty((0, 2), dtype=np.float64))

    # 점유 셀 중심의 world 좌표
    cxs = (anchor[0] + ox + 0.5) * res
    cys = (anchor[1] + oy + 0.5) * res

    bx, by = float(boat_xy[0]), float(boat_xy[1])
    dxs, dys = cxs - bx, cys - by
    rhos = np.hypot(dxs, dys)

    # 전방성 f = max(0, cos∠(배→장애물, heading)) — 정면 1, 옆/뒤 0.
    # 배 위에 얹힌 셀(ρ≈0)은 방향이 없으므로 가장 위험한 쪽(정면)으로 본다.
    degenerate = rhos < 1e-6
    safe_rho = np.where(degenerate, 1.0, rhos)
    cos_h, sin_h = math.cos(boat_yaw), math.sin(boat_yaw)
    f = np.where(degenerate, 1.0,
                 np.maximum(0.0, (dxs * cos_h + dys * sin_h) / safe_rho))
    # 근접성 g(ρ) = 1/(1+ρ/ρ₀) — 가까울수록 1
    g = 1.0 / (1.0 + rhos / p.dist_falloff)

    # 등방 — 두 축에 같은 σ. 게인은 종횡비를 건드리지 않는다.
    # σ 는 최대 정규화(코앞·정면 = sigma)라 멀고 옆일수록 **줄어든다** —
    # 진하기(amp)는 위로 곱한다 (유효 반경 = σ√(2ln(amp/thr)) 라 amp 경사가
    # 반경 경사도 만든다 — R1 의 "가까울수록/정면일수록"은 양쪽에서 온다).
    norm = (1.0 + p.k_dist) * (1.0 + p.k_head)
    sig = p.sigma * (1.0 + p.k_dist * g) * (1.0 + p.k_head * f) / norm
    amp = (1.0 + p.a_dist * g) * (1.0 + p.a_head * f)

    inv = 1.0 / (2.0 * sig * sig)
    # 3σ 밖은 잘라낸다 — 등방이라 패치는 그냥 반경 3σ 의 정사각형.
    radii = np.ceil(3.0 * sig / res).astype(np.int64)

    for k in range(oy.size):
        r = int(radii[k])
        x0, x1 = max(0, ox[k] - r), min(w, ox[k] + r + 1)
        y0, y1 = max(0, oy[k] - r), min(h, oy[k] + r + 1)
        if x0 >= x1 or y0 >= y1:
            continue

        # 패치 격자점의 셀 c 기준 상대 world 좌표
        px = (anchor[0] + np.arange(x0, x1) + 0.5) * res - cxs[k]
        py = (anchor[1] + np.arange(y0, y1) + 0.5) * res - cys[k]

        # 등방 가우시안은 분리 가능 — exp(−(dx²+dy²)/2σ²) = exp(−dx²/2σ²)·exp(−dy²/2σ²).
        # 1D 두 개의 외적이라 2D exp 를 통째로 부르지 않는다 (회전축이 없어서 가능).
        ex = np.exp(-(px * px) * inv[k])
        ey = np.exp(-(py * py) * inv[k])
        # max 결합 — 겹치는 셀은 더하지 않고 더 위험한 쪽을 남긴다 (§7-2)
        patch = dri[y0:y1, x0:x1]
        np.maximum(patch, amp[k] * np.outer(ey, ex), out=patch)

    return DriGrid(data=dri, anchor=anchor, resolution=res,
                   occ_xy=np.stack([cxs, cys], axis=1))
