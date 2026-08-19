from sensor_msgs.msg import CameraInfo

from kaboat_hardware.ceiling_apriltag_status import camera_info_is_calibrated


def _camera_info():
    msg = CameraInfo()
    msg.width = 1280
    msg.height = 720
    msg.k = [900.0, 0.0, 640.0,
             0.0, 900.0, 360.0,
             0.0, 0.0, 1.0]
    return msg


def test_valid_camera_info_is_calibrated():
    assert camera_info_is_calibrated(_camera_info())


def test_zero_intrinsics_are_rejected():
    msg = _camera_info()
    msg.k = [0.0] * 9
    assert not camera_info_is_calibrated(msg)


def test_non_finite_intrinsics_are_rejected():
    msg = _camera_info()
    msg.k[0] = float('nan')
    assert not camera_info_is_calibrated(msg)


def test_zero_resolution_is_rejected():
    msg = _camera_info()
    msg.width = 0
    assert not camera_info_is_calibrated(msg)
