"""DRI 문턱 초과 구역의 빨간 X 마커(ROS 메시지) 빌더 검사."""
from types import SimpleNamespace as NS

import numpy as np
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_behaviors.dri_marker import (
    MARK_Z, NAMESPACE, clear_dri_x_markers, dri_x_markers)

RES = 0.2


def dri_for(cells):
    data = np.full((100, 100), -1, np.int16)
    for x, y in cells:
        data[int((y + 10) / RES), int((x + 10) / RES)] = 97
    grid = NS(info=NS(resolution=RES, width=100, height=100,
                      origin=NS(position=NS(x=-10.0, y=-10.0))),
              data=data.ravel().tolist())
    return build_dri(grid, (0.0, 0.0), 0.0, DriParams())


def test_each_region_gets_a_red_x_made_of_two_diagonals():
    stamp = Time(sec=12, nanosec=5)
    array = dri_x_markers(dri_for([(3.0, -3.0), (3.0, 3.0)]), 0.85, 0.4,
                          'odom', stamp)
    (marker,) = array.markers
    assert marker.ns == NAMESPACE and marker.id == 0
    assert marker.type == Marker.LINE_LIST and marker.action == Marker.ADD
    assert marker.header.frame_id == 'odom' and marker.header.stamp == stamp
    assert (marker.color.r, marker.color.g, marker.color.b, marker.color.a) == (1.0, 0.0, 0.0, 1.0)
    assert marker.lifetime.sec == 1                    # 발행이 끊기면 X 도 사라진다
    assert len(marker.points) == 2 * 2 * 2             # 구역 2개 × 대각선 2개 × 끝점 2개
    assert all(p.z == MARK_Z for p in marker.points)


def test_x_is_centred_on_the_region_and_small():
    array = dri_x_markers(dri_for([(3.0, 0.0)]), 0.85, 0.4, 'odom', Time())
    points = array.markers[0].points
    cx = sum(p.x for p in points) / len(points)
    cy = sum(p.y for p in points) / len(points)
    assert abs(cx - 3.1) < 0.25 and abs(cy - 0.1) < 0.25
    assert max(p.x for p in points) - min(p.x for p in points) <= 0.4 + 1e-9


def test_no_region_over_threshold_clears_previous_marks():
    dri = dri_for([(3.0, 0.0)])
    array = dri_x_markers(dri, float(dri.data.max()) + 1.0, 0.4, 'odom', Time())
    (marker,) = array.markers
    assert marker.action == Marker.DELETEALL and marker.ns == NAMESPACE


def test_clear_markers_delete_the_same_namespace():
    (marker,) = clear_dri_x_markers('odom', Time(sec=3)).markers
    assert marker.action == Marker.DELETEALL
    assert marker.ns == NAMESPACE and marker.id == 0
    assert marker.header.frame_id == 'odom'
    (no_stamp,) = clear_dri_x_markers().markers
    assert no_stamp.header.frame_id == 'odom'
