"""apply_repulsion — station/search/dock 가 공통으로 쓰는 격자 기반 반발 조향."""
import math
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist

from kaboat_behaviors.obstacle_avoidance_utils import (
    AVOID_RANGE, GAIN, OCC_THRESHOLD, apply_repulsion)

RES, SIZE, ORIGIN = 0.1, 200, (-10.0, -10.0)


def _grid(cells=(), value=100):
    """cells: [(x, y)] odom 좌표에 value 를 칠한 격자."""
    data = [0] * (SIZE * SIZE)
    for x, y in cells:
        ix = int(math.floor((x - ORIGIN[0]) / RES))
        iy = int(math.floor((y - ORIGIN[1]) / RES))
        data[iy * SIZE + ix] = value
    info = SimpleNamespace(resolution=RES, width=SIZE, height=SIZE,
                           origin=SimpleNamespace(position=SimpleNamespace(x=ORIGIN[0], y=ORIGIN[1])))
    return SimpleNamespace(info=info, data=data)


def _odom(x=0.0, y=0.0, yaw=0.0):
    pose = SimpleNamespace(
        position=SimpleNamespace(x=x, y=y),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    return SimpleNamespace(pose=SimpleNamespace(pose=pose))


def _cmd(linear=0.1, angular=0.0):
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    return cmd


def _cell_center(x, y):
    """칠한 셀의 중심 — 함수가 실제로 쓰는 좌표."""
    ix = math.floor((x - ORIGIN[0]) / RES)
    iy = math.floor((y - ORIGIN[1]) / RES)
    return ORIGIN[0] + (ix + 0.5) * RES, ORIGIN[1] + (iy + 0.5) * RES


@pytest.mark.check('A')
def test_left_obstacle_steers_right_with_linear_weight():
    cx, cy = _cell_center(2.0, 1.0)
    out = apply_repulsion(_cmd(), _grid([(2.0, 1.0)]), _odom())
    weight = (AVOID_RANGE - math.hypot(cx, cy)) / AVOID_RANGE
    assert out.angular.z == pytest.approx(-GAIN * weight)
    assert out.linear.x == pytest.approx(0.1)   # 1.5m 밖 — 감속 없음


@pytest.mark.check('A')
def test_right_obstacle_steers_left_and_symmetric_pair_cancels():
    right = apply_repulsion(_cmd(), _grid([(2.0, -1.05)]), _odom())
    assert right.angular.z > 0.0
    both = apply_repulsion(_cmd(), _grid([(2.05, 1.05), (2.05, -1.05)]), _odom())
    assert both.angular.z == pytest.approx(0.0, abs=1e-12)


@pytest.mark.check('A')
def test_close_obstacle_slows_forward_command():
    cx, cy = _cell_center(0.75, 0.3)
    out = apply_repulsion(_cmd(0.1), _grid([(0.75, 0.3)]), _odom())
    assert out.linear.x == pytest.approx(0.1 * math.hypot(cx, cy) / 1.5)
    very_close = apply_repulsion(_cmd(0.1), _grid([(0.12, 0.02)]), _odom())
    assert very_close.linear.x == pytest.approx(0.1 * 0.2)   # 하한 20%


@pytest.mark.check('B')
@pytest.mark.parametrize('cell', [
    (4.5, 0.0),     # 4m 밖
    (1.0, 1.5),     # 전방 ±45° 밖 (약 56°)
    (-2.0, 0.0),    # 뒤쪽
])
def test_cells_outside_range_or_fov_are_ignored(cell):
    out = apply_repulsion(_cmd(0.1, 0.01), _grid([cell]), _odom())
    assert out.angular.z == pytest.approx(0.01)
    assert out.linear.x == pytest.approx(0.1)


@pytest.mark.check('B')
def test_threshold_and_unknown_cells():
    below = apply_repulsion(_cmd(), _grid([(2.0, 1.0)], value=OCC_THRESHOLD - 1), _odom())
    unknown = apply_repulsion(_cmd(), _grid([(2.0, 1.0)], value=-1), _odom())
    at = apply_repulsion(_cmd(), _grid([(2.0, 1.0)], value=OCC_THRESHOLD), _odom())
    assert below.angular.z == 0.0 and unknown.angular.z == 0.0
    assert at.angular.z < 0.0


@pytest.mark.check('B')
def test_obstacle_is_evaluated_in_boat_frame():
    # 배가 북쪽(+90°)을 보고 있으면 월드 (−1, 2) 는 선체 기준 좌측 전방이다.
    out = apply_repulsion(_cmd(), _grid([(-1.0, 2.0)]), _odom(yaw=math.pi / 2))
    assert out.angular.z < 0.0
    moved = apply_repulsion(_cmd(), _grid([(6.0, 6.0)]), _odom(x=4.0, y=5.0))
    assert moved.angular.z < 0.0


@pytest.mark.check('C')
def test_stop_command_stays_stop():
    out = apply_repulsion(_cmd(0.0, 0.0), _grid([(1.0, 0.3)]), _odom())
    assert out.linear.x == 0.0 and out.angular.z == 0.0


@pytest.mark.check('C')
def test_missing_grid_or_odom_returns_command_unchanged():
    assert apply_repulsion(_cmd(0.1, 0.02), None, _odom()).angular.z == pytest.approx(0.02)
    assert apply_repulsion(_cmd(0.1, 0.02), _grid([(1.0, 0.3)]), None).angular.z == pytest.approx(0.02)
