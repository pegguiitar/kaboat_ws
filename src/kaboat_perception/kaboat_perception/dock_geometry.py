"""ROS-independent marker depth and dock-mouth geometry in metres."""

from dataclasses import dataclass

import cv2
import numpy as np

from .depth_utils import depth_to_meters


@dataclass(frozen=True)
class MarkerEstimate:
    point_optical: np.ndarray
    outward_normal_optical: np.ndarray
    median_depth_m: float
    valid_pixels: int
    valid_fraction: float
    plane_rms_m: float


def estimate_marker(mask, depth_m, fx, fy, cx, cy, *, min_depth=0.25,
                    max_depth=12.0, min_pixels=40, max_plane_rms=0.08,
                    sample_step=2, distortion=()) -> MarkerEstimate | None:
    """Median valid depth and visible-plane normal; return None if unreliable."""
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
    zs = depth_m[ys, xs].astype(np.float64)
    z_med = float(np.median(zs))
    u, v = float(np.median(xs)), float(np.median(ys))
    distortion = np.asarray(distortion, dtype=np.float64)

    def normalized(pixels):
        if distortion.size and np.any(distortion != 0):
            matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
                              dtype=np.float64)
            return cv2.undistortPoints(
                pixels.reshape(-1, 1, 2).astype(np.float64),
                matrix, distortion).reshape(-1, 2)
        return np.column_stack(((pixels[:, 0] - cx) / fx,
                                (pixels[:, 1] - cy) / fy))

    uv = normalized(np.array([[u, v]]))[0]
    point = np.array([uv[0] * z_med, uv[1] * z_med, z_med])
    near = np.abs(zs - z_med) <= max(0.08, 0.04 * z_med)
    pixels = np.column_stack((xs[near], ys[near]))
    uv_cloud = normalized(pixels)
    cloud = np.column_stack((uv_cloud[:, 0] * zs[near],
                             uv_cloud[:, 1] * zs[near], zs[near]))
    cloud = cloud[::max(1, sample_step)]
    if len(cloud) < max(12, min_pixels // max(1, sample_step)):
        return None
    center = np.median(cloud, axis=0)
    _, singular, vectors = np.linalg.svd(cloud - center, full_matrices=False)
    if singular[1] < 0.01:
        return None
    normal = vectors[-1]
    if np.dot(normal, -point) < 0:
        normal = -normal
    residual = np.abs((cloud - center) @ normal)
    rms = float(np.sqrt(np.mean(residual ** 2)))
    if rms > max_plane_rms:
        return None
    return MarkerEstimate(point, normal, z_med, count, count / mask_count, rms)


def project_entrance_xy(marker_base, normal_base, wall_to_mouth_m):
    """Move out from the marker wall, then flatten to boat-local z=0."""
    marker = np.asarray(marker_base, dtype=float)
    normal = np.asarray(normal_base, dtype=float)
    horizontal = normal[:2]
    length = float(np.linalg.norm(horizontal))
    if (length < 0.25 or not np.isfinite(length)
            or not np.all(np.isfinite(marker))
            or not np.isfinite(wall_to_mouth_m) or wall_to_mouth_m < 0):
        return None
    xy = marker[:2] + wall_to_mouth_m * horizontal / length
    return np.array([xy[0], xy[1], 0.0])
