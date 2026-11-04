"""dri_marker — DRI 문턱 초과 구역을 RViz 빨간 X 마커로 만드는 ROS 쪽 빌더.

셀 계산은 ROS 비의존인 dri_marks.py 에 있다. 여기는 그 결과를 하나의
LINE_LIST 마커(셀마다 대각선 두 개)로 묶을 뿐이다. 마커는 lifetime 1초라
발행이 끊기면(노드 정지·입력 손실) 낡은 X 가 RViz 에 남지 않는다.
z 는 0.25 로 띄운다 — 지도 레이어(z=0)에 묻히지 않게(obstacle_planner 의
경로 z=0.2 와 같은 이유).
"""
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .dri_marks import high_risk_cells, x_segments

NAMESPACE = 'dri_over_threshold'
MARK_Z = 0.25


def dri_x_markers(dri, threshold: float, size: float, frame_id: str,
                  stamp) -> MarkerArray:
    """문턱을 넘는 셀마다 셀 중심에 빨간 X. 없으면 이전 X 를 지운다."""
    regions = high_risk_cells(dri, threshold)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns, marker.id = NAMESPACE, 0
    if not regions:
        marker.action = Marker.DELETEALL
        return MarkerArray(markers=[marker])
    marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = max(0.01, size * 0.12)          # 선 굵기
    marker.color.r, marker.color.a = 1.0, 1.0        # 빨강
    marker.lifetime.sec = 1
    for start, end in x_segments(regions, size):
        marker.points.append(Point(x=start[0], y=start[1], z=MARK_Z))
        marker.points.append(Point(x=end[0], y=end[1], z=MARK_Z))
    return MarkerArray(markers=[marker])


def clear_dri_x_markers(frame_id: str = 'odom', stamp=None) -> MarkerArray:
    """입력 손실 등으로 DRI 를 못 낼 때 남아 있는 X 를 즉시 지운다."""
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns, marker.id = NAMESPACE, 0
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])
