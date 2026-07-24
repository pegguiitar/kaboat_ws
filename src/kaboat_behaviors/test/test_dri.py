"""dri.py 단독 테스트 (AVOIDANCE.MD §3-1) — ROS 없이 돈다.

OccupancyGrid msg 는 덕타이핑이라 같은 필드를 가진 수제 객체로 대체한다.
"""
import math
import time

import numpy as np
import pytest

from kaboat_behaviors.dri import DriGrid, DriParams, build_dri

RES = 0.2
CELLS = 100  # 20m / 0.2m — 실제 occupancy_grid 와 동일


class FakeGrid:
    """nav_msgs/OccupancyGrid 최소 모양 — build_dri 가 읽는 필드만."""

    def __init__(self, data, anchor, res=RES):
        h, w = data.shape
        origin = type('O', (), {'position': type('P', (), {
            'x': anchor[0] * res, 'y': anchor[1] * res})()})()
        self.info = type('I', (), {
            'resolution': res, 'width': w, 'height': h, 'origin': origin})()
        self.data = data.ravel().tolist()


def make_grid(occupied_world, boat_xy=(0.0, 0.0), unknown_world=(), value=100):
    """배가 격자 중앙에 오도록 anchor 를 잡고, 준 world 좌표에 점유 셀을 박는다."""
    anchor = (math.floor(boat_xy[0] / RES) - CELLS // 2,
              math.floor(boat_xy[1] / RES) - CELLS // 2)
    data = np.zeros((CELLS, CELLS), dtype=np.int16)  # 0 = free 관측됨
    for wx, wy in occupied_world:
        ix = math.floor(wx / RES) - anchor[0]
        iy = math.floor(wy / RES) - anchor[1]
        data[iy, ix] = value
    for wx, wy in unknown_world:
        ix = math.floor(wx / RES) - anchor[0]
        iy = math.floor(wy / RES) - anchor[1]
        data[iy, ix] = -1
    return FakeGrid(data, anchor), anchor


def test_peak_is_on_the_obstacle_cell():
    """(i) 위험도 피크가 장애물 셀 위에 있다."""
    grid, anchor = make_grid([(3.0, 0.0)])
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams())

    peak_iy, peak_ix = np.unravel_index(np.argmax(dri.data), dri.data.shape)
    exp_ix = math.floor(3.0 / RES) - anchor[0]
    exp_iy = math.floor(0.0 / RES) - anchor[1]
    assert (peak_ix, peak_iy) == (exp_ix, exp_iy)


def _cell_of(anchor, wx, wy):
    """world 좌표의 배열 인덱스 (ix, iy)."""
    return (math.floor(wx / RES) - anchor[0], math.floor(wy / RES) - anchor[1])


def _ring(dri, ix, iy, n):
    """(ix,iy) 에서 상하좌우로 정확히 n 셀 떨어진 네 점의 DRI.

    world 좌표로 재면 부동소수 floor 때문에 ±1셀씩 어긋나 등방성 비교가
    깨진다(4.6/0.2 = 22.999... → 셀 22). 배열 인덱스로 재면 정확히 대칭.
    """
    return [float(dri.data[iy, ix + n]), float(dri.data[iy, ix - n]),
            float(dri.data[iy + n, ix]), float(dri.data[iy - n, ix])]


def test_spread_is_isotropic():
    """(ii) 퍼짐이 등방 — 셀에서 같은 거리면 방향과 무관하게 같은 값.

    비등방(타원) 버전이 배를 향한 시가 모양을 만들어 V자 줄기가 지도를 덮은 게
    등방으로 바꾼 이유다. 게인이 켜져 있어도 원이어야 한다.
    """
    grid, anchor = make_grid([(3.0, 0.0)])
    p = DriParams(k_dist=1.0, k_head=1.0)  # 게인 최대로 켜도 원
    dri = build_dri(grid, (0.0, 0.0), 0.0, p)

    ix, iy = _cell_of(anchor, 3.0, 0.0)
    vals = _ring(dri, ix, iy, 5)
    assert max(vals) == pytest.approx(min(vals), rel=1e-5)


def test_gain_shrinks_far_side_without_distorting_shape():
    """σ 게인은 **감쇄 폭**(최대 정규화) — 코앞·정면 = sigma 그대로, 멀거나
    옆일수록 원이 줄어든다 (2026-07-17: 위로 부풀리는 구식은 3m 격자 틈을
    물리적으로 닫았다). 그러면서도 여전히 원(등방 회귀 방지)."""
    grid, anchor = make_grid([(3.0, 0.0)])
    ix, iy = _cell_of(anchor, 3.0, 0.0)
    flat = build_dri(grid, (0.0, 0.0), 0.0, DriParams(k_dist=0.0, k_head=0.0))
    gained = build_dri(grid, (0.0, 0.0), 0.0, DriParams(k_dist=1.0, k_head=1.0))

    # 3m 정면(g<1)이라 게인이 붙으면 σ 가 상한보다 줄어든다 → 같은 거리에서 더 감쇠
    assert _ring(gained, ix, iy, 7)[0] < _ring(flat, ix, iy, 7)[0]
    # 그러면서도 여전히 원
    vals = _ring(gained, ix, iy, 7)
    assert max(vals) == pytest.approx(min(vals), rel=1e-5)


def test_gain_keeps_near_frontal_wider_than_far_side():
    """감쇄는 상대적 — 게인이 켜져 있으면 가깝고 정면인 원이 멀고 옆인 원보다
    넓다 (R1 의 '더 크게'는 이제 이 상대 경사 + 진폭 경사(유효 반경)로 달성)."""
    p = DriParams(k_dist=1.0, k_head=1.0, a_dist=0.0, a_head=0.0)  # 진폭 고정
    near, a1 = make_grid([(2.0, 0.0)])
    far, a2 = make_grid([(0.0, 8.0)])          # 측면(+y, heading 은 +x) 8m
    d_near = build_dri(near, (0.0, 0.0), 0.0, p)
    d_far = build_dri(far, (0.0, 0.0), 0.0, p)
    ring_near = _ring(d_near, *_cell_of(a1, 2.0, 0.0), 5)[0]
    ring_far = _ring(d_far, *_cell_of(a2, 0.0, 8.0), 5)[0]
    assert ring_near > ring_far


def test_closer_obstacle_is_stronger():
    """(iii-a) 같은 방위면 가까운 장애물이 더 강하다 (근접성 g)."""
    near, _ = make_grid([(2.0, 0.0)])
    far, _ = make_grid([(8.0, 0.0)])
    p = DriParams()
    near_peak = build_dri(near, (0.0, 0.0), 0.0, p).risk_at(2.0, 0.0)
    far_peak = build_dri(far, (0.0, 0.0), 0.0, p).risk_at(8.0, 0.0)
    assert near_peak > far_peak


def test_frontal_obstacle_is_stronger_than_beam():
    """(iii-b) 같은 거리면 선수 정면 장애물이 옆보다 더 강하다 (전방성 f)."""
    front, _ = make_grid([(3.0, 0.0)])   # heading=0 기준 정면
    beam, _ = make_grid([(0.0, 3.0)])    # 정확히 옆 (cos=0 → f=0)
    p = DriParams()
    front_peak = build_dri(front, (0.0, 0.0), 0.0, p).risk_at(3.0, 0.0)
    beam_peak = build_dri(beam, (0.0, 0.0), 0.0, p).risk_at(0.0, 3.0)
    assert front_peak > beam_peak


def test_unknown_cells_contribute_nothing():
    """(iv) 미관측(-1) 셀은 기여 0 — free 취급)."""
    grid, _ = make_grid([], unknown_world=[(3.0, 0.0), (3.2, 0.0), (3.0, 0.2)])
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams())
    assert np.all(dri.data == 0.0)


def test_below_threshold_cells_contribute_nothing():
    """occ_threshold 미만의 약한 점유확률 셀도 위험원이 아니다."""
    grid, _ = make_grid([(3.0, 0.0)], value=40)  # threshold 50 미만
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams(occ_threshold=50))
    assert np.all(dri.data == 0.0)


def test_default_threshold_requires_four_hits():
    """기본 occ_threshold=95 = "4히트는 봐야 믿는다" — tilt 로 3히트(93)까지 쌓인
    해수면 유령을 배제하고, 근거리에서 포화(97)하는 부표만 통과시킨다.
    (원거리 성긴 부표 포기가 이 결정의 대가.)"""
    three_hit, _ = make_grid([(3.0, 0.0)], value=93)  # 3히트 — 문턱 미달
    four_hit, _ = make_grid([(3.0, 0.0)], value=97)   # 4히트(포화) — 통과
    assert np.all(build_dri(three_hit, (0.0, 0.0), 0.0, DriParams()).data == 0.0)
    assert build_dri(four_hit, (0.0, 0.0), 0.0, DriParams()).data.max() > 0.0


def test_cells_combine_by_max_not_sum():
    """겹치는 장애물은 max 로 결합된다 (합산 아님, §3-1).

    합산이면 두 장애물 사이 지점이 각각보다 높아져, 통과 가능한 틈을 오답으로
    차단한다. 충돌은 최근접 장애물이 결정하므로 max 가 맞다.
    """
    one, _ = make_grid([(3.0, 0.0)])
    two, _ = make_grid([(3.0, 0.0), (3.4, 0.0)])
    p = DriParams()
    mid_one = build_dri(one, (0.0, 0.0), 0.0, p).risk_at(3.2, 0.0)
    mid_two = build_dri(two, (0.0, 0.0), 0.0, p).risk_at(3.2, 0.0)
    # 장애물을 하나 더 놔도 중간점 위험이 **오르지 않는다** (합산이면 올랐다)
    assert mid_two == pytest.approx(mid_one, rel=1e-5)


def test_max_keeps_dri_bounded_by_peak_amplitude():
    """max 라 DRI 는 밀집도와 무관하게 max A 로 유계 — §3-2 비용 균형이 안정된다."""
    p = DriParams()
    dense, _ = make_grid([(2.0 + 0.2 * i, 0.2 * j) for i in range(8) for j in range(8)])
    dri = build_dri(dense, (0.0, 0.0), 0.0, p)

    # A(c) = (1+a_dist·g)(1+a_head·f) ≤ (1+a_dist)(1+a_head)
    bound = (1.0 + p.a_dist) * (1.0 + p.a_head)
    assert dri.data.max() <= bound + 1e-5


def test_passable_gap_is_not_blocked_by_distant_neighbours():
    """통과 가능한 넓은 틈이 양옆 장애물의 꼬리 때문에 막히지 않는다 (합산의 오답).

    σ=1.0 기준 3m 틈 정중앙은 양쪽에서 1.5m — 합산이면 0.649 로 threshold(0.5)를
    넘어 차단됐지만, 실제로는 최근접이 1.5m > 안전반경 1.18m 라 통과 가능하다.
    """
    p = DriParams(sigma=1.0, k_dist=0.0, k_head=0.0, a_dist=0.0, a_head=0.0)
    lone, _ = make_grid([(0.0, 1.5)])
    both, _ = make_grid([(0.0, -1.5), (0.0, 1.5)])

    # 틈 한가운데. 정확한 등거리 지점을 world 좌표로 집으려 하면 셀 스냅에 걸리므로
    # (자기 자신만 보면 못 잡는 함정), "반대편 장애물을 추가해도 값이 그대로냐" 로 검증한다.
    mid_lone = build_dri(lone, (0.0, 0.0), 0.0, p).risk_at(0.1, 0.1)
    mid_both = build_dri(both, (0.0, 0.0), 0.0, p).risk_at(0.1, 0.1)

    assert mid_both == pytest.approx(mid_lone, rel=1e-6)  # 합산이면 ~2배로 뛴다
    assert mid_both < 0.5                # threshold 0.5 를 안 넘음 = 통과 판정


def test_zero_gains_reduce_to_uniform_gaussian():
    """게인 4개 0 → 크기·세기가 고정된 균일 가우시안 (디버그 기준선, §3-1)."""
    grid, _ = make_grid([(3.0, 0.0)])
    p = DriParams(sigma=0.6, k_dist=0.0, k_head=0.0, a_dist=0.0, a_head=0.0)
    dri = build_dri(grid, (0.0, 0.0), 0.0, p)

    c = (math.floor(3.0 / RES) + 0.5) * RES
    assert dri.risk_at(c, 0.1) == pytest.approx(1.0, rel=1e-3)  # 진폭 A=1
    # 1σ 지점은 exp(-1/2) ≈ 0.607
    assert dri.risk_at(c + 0.6, 0.1) == pytest.approx(math.exp(-0.5), rel=1e-2)


def test_closer_obstacle_gets_bigger_radius():
    """(iii-c) 크기 가중치 — 가까운 장애물이 더 **큰** 원 (등방 전환의 핵심 이득).

    비등방 버전에선 이 확대가 σ_along(방사축)에만 걸려서 배가 지나가는
    berth(방위 방향 폭)를 전혀 안 넓혔다. 등방이라 σ 가 곧 berth 다.
    """
    p = DriParams(k_dist=1.0, k_head=0.0, a_dist=0.0, a_head=0.0)  # 크기 게인만
    near, _ = make_grid([(2.0, 0.0)])
    far, _ = make_grid([(8.0, 0.0)])
    dn = build_dri(near, (0.0, 0.0), 0.0, p)
    df = build_dri(far, (0.0, 0.0), 0.0, p)

    # 진폭 게인을 껐으므로 피크는 둘 다 1 — 차이는 오직 퍼짐 폭
    cn = (math.floor(2.0 / RES) + 0.5) * RES
    cf = (math.floor(8.0 / RES) + 0.5) * RES
    assert dn.risk_at(cn, 0.1) == pytest.approx(df.risk_at(cf, 0.1), rel=1e-3)
    # 같은 거리만큼 옆으로(=berth 방향) 벗어났을 때 가까운 쪽이 덜 감쇠
    # sigma 기본값을 0.6→0.5로 줄인 뒤에도 두 3σ 패치 안에 드는 1m 지점에서 비교.
    assert dn.risk_at(cn, 1.1) > df.risk_at(cf, 1.1)


def test_risk_at_outside_window_is_inf():
    """window 밖은 inf — 계획이 창 밖으로 못 나간다."""
    grid, _ = make_grid([(3.0, 0.0)])
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams())
    assert dri.risk_at(50.0, 0.0) == math.inf
    assert dri.risk_at(0.0, -50.0) == math.inf


def test_risk_at_many_matches_risk_at():
    """벡터화판이 스칼라판과 같은 값(창 밖 inf 포함)."""
    grid, _ = make_grid([(3.0, 0.0), (2.0, 1.0)])
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams())
    pts = [(3.0, 0.0), (2.0, 1.0), (0.5, 0.5), (50.0, 0.0), (-11.0, 0.0)]
    got = dri.risk_at_many([x for x, _ in pts], [y for _, y in pts])
    exp = [dri.risk_at(x, y) for x, y in pts]
    assert np.allclose(got, exp, equal_nan=False)


def test_empty_grid_is_all_zero():
    """장애물 0개 → 전부 0 (R7 폴백은 플래너 쪽 책임)."""
    grid, _ = make_grid([])
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams())
    assert dri.data.shape == (CELLS, CELLS)
    assert np.all(dri.data == 0.0)


def test_grid_geometry_is_preserved():
    """DRI 격자는 occupancy_grid 와 anchor/resolution/크기가 1:1)."""
    # 원점에서 멀리 떨어진 배 — anchor 가 음수/비대칭이어도 셀 대응이 어긋나면 안 됨
    boat = (12.3, -45.6)
    grid, anchor = make_grid([(boat[0] + 3.0, boat[1])], boat_xy=boat)
    dri = build_dri(grid, boat, 0.0, DriParams())
    assert dri.anchor == anchor
    assert dri.resolution == RES
    assert dri.data.shape == (CELLS, CELLS)

    # 피크가 world 좌표 기준 제자리 — origin 왕복(÷res 후 ×res)이 어긋나지 않았다
    peak_iy, peak_ix = np.unravel_index(np.argmax(dri.data), dri.data.shape)
    assert (peak_ix + anchor[0], peak_iy + anchor[1]) == (
        math.floor((boat[0] + 3.0) / RES), math.floor(boat[1] / RES))


def test_build_dri_under_10ms():
    """완료 기준: 100×100 격자 build_dri < 10ms (N1 — 10Hz tick 안에서 동기 실행)."""
    rng = np.random.default_rng(0)
    # 장애물 밭을 가정한 부하 — 점유 셀 200개를 창 전체에 뿌린다
    pts = rng.uniform(-9.0, 9.0, size=(200, 2))
    grid, _ = make_grid([tuple(pt) for pt in pts])
    p = DriParams()

    build_dri(grid, (0.0, 0.0), 0.0, p)  # 워밍업 (numpy 첫 호출 오버헤드 제외)
    best = min(_time_build(grid, p) for _ in range(5))
    assert best < 0.010, f'build_dri 가 {best * 1e3:.1f}ms — 10ms 예산 초과'


def _time_build(grid, p):
    t0 = time.perf_counter()
    build_dri(grid, (0.0, 0.0), 0.0, p)
    return time.perf_counter() - t0
