"""buoy_pointmap 단독 테스트 — 원본 점 log-odds 지도 + 굳은 점 군집화."""
import math
from types import SimpleNamespace

import numpy as np

from kaboat_perception.buoy_evidence import LidarView
from kaboat_perception.buoy_pointmap import PointMapParams, PointMapTracker

P = PointMapParams()
INC = math.radians(0.177)


def arc(center, r=0.25, viewer=(0.0, 0.0), n=12):
    c = np.asarray(center, float)
    los = math.atan2(viewer[1] - c[1], viewer[0] - c[0])
    a = los + np.radians(np.linspace(-50, 50, n))
    return np.column_stack((c[0] + r * np.cos(a), c[1] + r * np.sin(a)))


def view(ranges_by_bearing=None, default=np.inf, x=0.0, y=0.0):
    a = -math.pi + INC * np.arange(int(2 * math.pi / INC))
    r = np.full(len(a), default)
    for (b0, b1), rr in (ranges_by_bearing or {}).items():
        r[(a >= b0) & (a <= b1)] = rr
    return LidarView(x, y, 0.0, -math.pi, INC, r, 0.1, 40.0)


def det(pts, color=None):
    m = pts.mean(axis=0)
    return (m[0], m[1], color, True, pts)


def run(tk, centers, t0=0.0, steps=8, color=None):
    for k in range(steps):
        tk.update([det(arc(c), color) for c in centers], t0 + 0.1 * k)


def test_steady_buoy_confirmed_and_centre_stays():
    tk = PointMapTracker(P)
    run(tk, [(8.0, 0.0)])
    conf = tk.confirmed()
    assert len(conf) == 1
    xs = []
    for k in range(10):                       # 점이 남아 있으니 같은 점이 다시 와도 중심이 그대로
        tk.update([det(arc((8.0, 0.0)))], 0.8 + 0.1 * k)
        xs.append(conf[0].x)
    assert max(xs) - min(xs) < 1e-6


def test_short_burst_not_confirmed():
    tk = PointMapTracker(P)
    run(tk, [(3.0, 0.0)], steps=3)            # 0.2 s 물보라
    assert tk.confirmed() == []


def test_points_kept_when_no_return_or_occluded():
    tk = PointMapTracker(P)
    run(tk, [(8.0, 0.0)])
    for k in range(20):                       # 반사 없음(검정 흡수) — 깎지 않음
        tk.update([], 1.0 + 0.1 * k, view())
    assert len(tk.confirmed()) == 1
    for k in range(20):                       # 앞에서 가림
        tk.update([], 3.0 + 0.1 * k, view({(-0.2, 0.2): 4.0}))
    assert len(tk.confirmed()) == 1


def test_points_freed_when_beam_passes_through():
    tk = PointMapTracker(P)
    run(tk, [(8.0, 0.0)])
    for k in range(40):                       # 빔이 부표 자리를 지나 12 m 에서 맞음
        tk.update([], 1.0 + 0.1 * k, view({(-0.2, 0.2): 12.0}))
    assert tk.confirmed() == [] and len(tk.P) == 0


def test_adjacent_pair_stays_two_buoys():
    tk = PointMapTracker(P)
    run(tk, [(8.0, 0.0), (8.0, 1.3)], steps=10)
    conf = tk.confirmed()
    assert len(conf) == 2
    assert abs(conf[0].y - conf[1].y) > 1.0


def test_new_scan_points_near_old_ones_reinforce_instead_of_adding():
    tk = PointMapTracker(P)
    run(tk, [(8.0, 0.0)], steps=3)
    n = len(tk.P)
    tk.update([det(arc((8.0, 0.0)) + 0.01)], 0.3)
    assert len(tk.P) == n


def test_colour_seen_before_buoy_forms_is_kept():
    tk = PointMapTracker(P)
    vote = SimpleNamespace(red=300, green=0, visible=500, usable=True, color='red')
    tk.update([det(arc((8.0, 0.0)), vote)], 0.0)     # 카메라가 첫 스캔에만 봄 — 아직 부표 아님
    run(tk, [(8.0, 0.0)], t0=0.1)
    assert tk.confirmed()[0].color == 'red'


def test_far_points_forgotten():
    tk = PointMapTracker(P)
    run(tk, [(8.0, 0.0)])
    tk.update([], 1.0, view(x=-12.0))                # 배가 20 m 떨어짐 — 점은 바로 지움
    assert len(tk.P) == 0
    tk.update([], 2.5, view(x=-12.0))                # 부표는 track_grace(1 s) 뒤 지움
    assert tk.tracks == []


def test_cell_mode_snaps_points_to_cell_centres():
    tk = PointMapTracker(PointMapParams(cell=0.1))
    run(tk, [(8.0, 0.0)])
    assert len(tk.confirmed()) == 1
    frac = tk.P / 0.1 - 0.5
    assert np.allclose(frac, np.round(frac), atol=1e-9)        # 모든 점이 칸 중심
    n = len(tk.P)
    tk.update([det(arc((8.0, 0.0)) + 0.02)], 0.8)            # 같은 칸 안으로 조금 움직여도 점이 안 늘어남
    assert len(tk.P) <= n + 2
