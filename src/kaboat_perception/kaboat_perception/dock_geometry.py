"""ROS-independent marker depth geometry in metres.

The marker's 3D position comes from the median depth inside the YOLO mask. The
dock axis / entrance is NOT estimated here: a small marker plane gives a noisy
normal, so the docking controller fits the back wall from the lidar scan.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerEstimate:
    point_optical: np.ndarray
    median_depth_m: float
    valid_pixels: int
    valid_fraction: float


def estimate_marker(mask, depth_m, fx, fy, cx, cy, *, min_depth=0.25,
                    max_depth=12.0, min_pixels=40,
                    distortion=()) -> MarkerEstimate | None:
    """Median valid depth and mask-centre ray as a 3D point; None if unreliable."""
    mask = np.asarray(mask, dtype=bool)
    depth_m = np.asarray(depth_m, dtype=np.float32)
    if mask.shape != depth_m.shape or fx <= 0 or fy <= 0:
        return None
    valid = mask & np.isfinite(depth_m) & (depth_m >= min_depth) & (depth_m <= max_depth)
    count = int(np.count_nonzero(valid))
    mask_count = int(np.count_nonzero(mask))
    if count < min_pixels or mask_count == 0:
        return None
    ys, xs = np.nonzero(valid)
    z_med = float(np.median(depth_m[ys, xs].astype(np.float64)))
    u, v = float(np.median(xs)), float(np.median(ys))
    distortion = np.asarray(distortion, dtype=np.float64)
    if distortion.size and np.any(distortion != 0):
        matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        x, y = cv2.undistortPoints(
            np.array([[[u, v]]], dtype=np.float64), matrix, distortion).reshape(2)
    else:
        x, y = (u - cx) / fx, (v - cy) / fy
    point = np.array([x * z_med, y * z_med, z_med])
    return MarkerEstimate(point, z_med, count, count / mask_count)
