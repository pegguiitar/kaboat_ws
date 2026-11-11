"""gq7_local_odom: GQ7 odometry_earth(ECEF) → 원점 기준 /odom (x=동, y=북, 자세 ENU)."""

import math
import os

import pytest

from kaboat_hardware.geo_local import LocalFrame, _ecef
from kaboat_hardware.gq7_local_odom import (
    earth_to_local, matrix_to_quat, quat_mul, rotate_covariance6)

ORIGIN = (35.18487138, 128.56510758, 33.43)       # 2026-11-11 현장 원점


def _yaw(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def _conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _earth_pose(frame, x, y, up, yaw_deg):
    """원점 기준 (x, y, up, ENU 선수각) → GQ7 odometry_earth 와 같은 ECEF 위치·자세."""
    pos = _ecef(*frame.to_llh(x, y, up))
    h = math.radians(yaw_deg) / 2.0
    q_enu = (0.0, 0.0, math.sin(h), math.cos(h))
    q_ecef = quat_mul(_conj(matrix_to_quat(frame.ecef_rotation())), q_enu)   # ENU → ECEF
    return pos, q_ecef


def test_quaternion_helpers():
    assert matrix_to_quat(((1, 0, 0), (0, 1, 0), (0, 0, 1))) == pytest.approx((0, 0, 0, 1))
    rz90 = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
    assert _yaw(matrix_to_quat(rz90)) == pytest.approx(90.0)
    for m in (((1, 0, 0), (0, -1, 0), (0, 0, -1)), ((-1, 0, 0), (0, 1, 0), (0, 0, -1)),
              ((-1, 0, 0), (0, -1, 0), (0, 0, 1))):                       # 180° 회전 3종
        q = matrix_to_quat(m)
        assert sum(v * v for v in q) == pytest.approx(1.0)
    assert quat_mul((0, 0, 0, 1), (0.1, 0.2, 0.3, 0.927)) == pytest.approx((0.1, 0.2, 0.3, 0.927))


@pytest.mark.parametrize('x,y,up,yaw', [
    (0.0, 0.0, 0.0, 0.0), (12.0, -7.0, 1.5, 30.0), (-150.0, 220.0, -2.0, -133.0),
    (400.0, 400.0, 0.0, 179.0)])
def test_earth_pose_comes_back_as_local_xy_and_heading(x, y, up, yaw):
    frame = LocalFrame(*ORIGIN)
    pos, q = _earth_pose(frame, x, y, up, yaw)
    enu, quat, _ = earth_to_local(frame, pos, q)
    assert enu == pytest.approx((x, y, up), abs=1e-3)
    assert _yaw(quat) == pytest.approx(yaw, abs=0.02)           # 400 m 에서도 축 차이 0.01° 미만
    assert sum(v * v for v in quat) == pytest.approx(1.0)


def test_covariance_is_rotated_into_enu():
    frame = LocalFrame(*ORIGIN)
    r = frame.ecef_rotation()
    rt = _transpose(r)
    enu_diag = [[4.0, 0, 0], [0, 1.0, 0], [0, 0, 9.0]]
    # ENU 대각 공분산을 ECEF 로 표현한 것 (위치·회전 블록 같은 값)
    ecef3 = [[sum(rt[i][k] * enu_diag[k][l] * r[l][j] for k in range(3) for l in range(3))
              for j in range(3)] for i in range(3)]
    cov = [0.0] * 36
    for i in range(3):
        for j in range(3):
            cov[6 * i + j] = cov[6 * (i + 3) + j + 3] = ecef3[i][j]
    out = rotate_covariance6(r, cov)
    for k, v in enumerate((4.0, 1.0, 9.0)):
        assert out[7 * k] == pytest.approx(v)                   # 위치 대각
        assert out[7 * (k + 3)] == pytest.approx(v)             # 회전 대각
    assert out[1] == pytest.approx(0.0, abs=1e-9) and out[6] == pytest.approx(0.0, abs=1e-9)


# ── 노드 ─────────────────────────────────────────────
rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
import yaml  # noqa: E402

from kaboat_hardware.gq7_local_odom import Gq7LocalOdom  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _write_origin(path, origin):
    path.write_text(yaml.safe_dump(
        {'origin_lat': origin[0], 'origin_lon': origin[1], 'origin_alt': origin[2]}))


def _node(tmp_path, **params):
    overrides = [Parameter('origin_file', value=str(tmp_path / 'origin.yaml'))]
    overrides += [Parameter(k, value=v) for k, v in params.items()]
    node = Gq7LocalOdom(parameter_overrides=overrides)
    node.sent = []
    node.pub.publish = node.sent.append
    return node


def _earth_msg(frame, x, y, yaw):
    pos, q = _earth_pose(frame, x, y, 0.0, yaw)
    m = Odometry()
    m.header.frame_id = 'earth'
    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = pos
    (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
     m.pose.pose.orientation.z, m.pose.pose.orientation.w) = q
    m.twist.twist.linear.x, m.twist.twist.angular.z = 0.8, 0.05
    return m


def test_publishes_odom_in_origin_frame_and_reloads_changed_origin(ros, tmp_path):
    _write_origin(tmp_path / 'origin.yaml', ORIGIN)
    node = _node(tmp_path)
    try:
        frame = LocalFrame(*ORIGIN)
        node._on_earth(_earth_msg(frame, 25.0, -10.0, -43.2))
        out = node.sent[-1]
        assert (out.header.frame_id, out.child_frame_id) == ('odom', 'base_link')
        p = out.pose.pose.position
        assert (p.x, p.y) == pytest.approx((25.0, -10.0), abs=1e-3)
        o = out.pose.pose.orientation
        assert _yaw((o.x, o.y, o.z, o.w)) == pytest.approx(-43.2, abs=0.02)
        assert (out.twist.twist.linear.x, out.twist.twist.angular.z) == (0.8, 0.05)

        # 원점을 새로 잡으면(파일 바뀜) GQ7 재시작 없이 바로 새 원점 기준
        moved = frame.to_llh(25.0, -10.0)
        _write_origin(tmp_path / 'origin.yaml', moved)
        os.utime(tmp_path / 'origin.yaml', (1e9, 1e9))
        node._check_origin_file()
        node._on_earth(_earth_msg(frame, 25.0, -10.0, -43.2))
        p = node.sent[-1].pose.pose.position
        assert (p.x, p.y) == pytest.approx((0.0, 0.0), abs=1e-3)
    finally:
        node.destroy_node()


def test_without_origin_file_first_position_or_nothing(ros, tmp_path):
    frame = LocalFrame(*ORIGIN)
    node = _node(tmp_path)                                    # auto_origin 기본 true
    try:
        node._on_earth(_earth_msg(frame, 30.0, 40.0, 0.0))
        p = node.sent[-1].pose.pose.position
        assert (p.x, p.y) == pytest.approx((0.0, 0.0), abs=1e-6)
        node._on_earth(_earth_msg(frame, 33.0, 44.0, 0.0))
        p = node.sent[-1].pose.pose.position
        assert (p.x, p.y) == pytest.approx((3.0, 4.0), abs=1e-3)
    finally:
        node.destroy_node()
    strict = _node(tmp_path, auto_origin=False)
    try:
        strict._on_earth(_earth_msg(frame, 30.0, 40.0, 0.0))
        assert strict.sent == []
    finally:
        strict.destroy_node()


def test_ignores_empty_filter_position(ros, tmp_path):
    _write_origin(tmp_path / 'origin.yaml', ORIGIN)
    node = _node(tmp_path)
    try:
        node._on_earth(Odometry())                            # 위치 0,0,0 = 아직 못 냄
        assert node.sent == []
    finally:
        node.destroy_node()
