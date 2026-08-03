"""Depth image helpers shared by camera-based perception nodes.

ROS depth images use two common encodings (REP-118):
  - 16UC1: unsigned millimetres (RealSense ROS default)
  - 32FC1: floating-point metres (Gazebo RGBD camera in this workspace)

Perception code keeps one internal contract: float32 metres.
"""

import numpy as np


MM_TO_M = 0.001


def depth_to_meters(depth_frame, encoding: str):
    """Convert a ROS depth image array to float32 metres.

    Zero and non-finite values are intentionally preserved; ``distance_at``
    filters them when sampling a detection.
    """
    enc = encoding.upper()
    frame = np.asarray(depth_frame)
    if enc == '16UC1':
        return frame.astype(np.float32) * MM_TO_M
    if enc == '32FC1':
        return frame.astype(np.float32, copy=False)
    raise ValueError(
        f'unsupported depth encoding {encoding!r}; expected 16UC1 or 32FC1')


def distance_at(depth_frame, cx: float, cy: float, win: int = 3) -> float:
    """Return the median metric depth around one image pixel, or -1 if invalid."""
    if depth_frame is None:
        return -1.0
    h, w = depth_frame.shape[:2]
    px, py = int(cx), int(cy)
    if not (0 <= px < w and 0 <= py < h):
        return -1.0
    x1, x2 = max(px - win, 0), min(px + win + 1, w)
    y1, y2 = max(py - win, 0), min(py + win + 1, h)
    roi = np.asarray(depth_frame[y1:y2, x1:x2], dtype=np.float32)
    valid = roi[np.isfinite(roi) & (roi > 0.0)]
    if valid.size == 0:
        return -1.0
    return float(np.median(valid))
