"""ROS-independent marker depth geometry in metres.

The marker's 3D position comes from robust interior-mask depth statistics. The
dock axis / entrance is NOT estimated here: a small marker plane gives a noisy
normal, so the docking controller fits the back wall from the lidar scan.
"""

from dataclasses import dataclass
from collections import deque
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerEstimate:
    point_optical: np.ndarray
    median_depth_m: float
    valid_pixels: int
    valid_fraction: float
    depth_sigma_m: float = 0.0
    raw_depth_std_m: float = 0.0
    inlier_fraction: float = 1.0


def mask_sight_ray(mask, fx, fy, cx, cy, distortion=()):
    """Undistorted optical ray (Z=1) through mask median; depth is not an input."""
    mask=np.asarray(mask,dtype=bool)
    if (mask.ndim!=2 or not mask.any() or not all(math.isfinite(v) for v in (fx,fy,cx,cy))
            or min(fx,fy)<=0):
        raise ValueError('invalid sight ray geometry')
    ys,xs=np.nonzero(mask)
    uv=np.array([[[float(np.median(xs)),float(np.median(ys))]]],dtype=np.float64)
    matrix=np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype=np.float64)
    coefficients=np.asarray(distortion,dtype=np.float64)
    xy=cv2.undistortPoints(uv,matrix,coefficients).reshape(2)
    return np.array([*xy,1.0]), uv.reshape(2)


def estimate_marker(mask, depth_m, fx, fy, cx, cy, *, min_depth=0.25,
                    max_depth=12.0, min_pixels=40, distortion=(),
                    erosion_pixels=1, max_depth_sigma_m=0.25,
                    min_inlier_fraction=0.7, outlier_floor_m=0.06, diagnostics=None) -> MarkerEstimate | None:
    """Estimate a marker from its interior and reject incoherent depth surfaces.

    MAD removes isolated depth outliers; a spread limit also rejects broad or
    multimodal surfaces. The centre ray uses the mask, independent of depth holes.
    """
    if erosion_pixels < 0 or max_depth_sigma_m <= 0 or outlier_floor_m <= 0 or not 0 < min_inlier_fraction <= 1:
        raise ValueError('invalid depth quality thresholds')
    quality = diagnostics if diagnostics is not None else {}
    quality['reason'] = 'insufficient_depth'
    mask = np.asarray(mask, dtype=bool)
    depth_m = np.asarray(depth_m, dtype=np.float32)
    if mask.shape != depth_m.shape or fx <= 0 or fy <= 0:
        quality['reason'] = 'invalid_geometry'
        return None
    if erosion_pixels:
        kernel = np.ones((2 * erosion_pixels + 1,) * 2, np.uint8)
        core = cv2.erode(mask.astype(np.uint8), kernel).astype(bool)
    else:
        core = mask
    mask_count = int(np.count_nonzero(core))
    valid = core & np.isfinite(depth_m) & (depth_m >= min_depth) & (depth_m <= max_depth)
    count = int(np.count_nonzero(valid))
    quality.update(mask_pixels=mask_count, valid_pixels=count,
                   valid_fraction=count / mask_count if mask_count else 0.0)
    if count < min_pixels:
        return None
    values = depth_m[valid].astype(np.float64)
    median = float(np.median(values))
    sigma = float(1.4826 * np.median(np.abs(values - median)))
    inliers = np.abs(values - median) <= max(outlier_floor_m, 3.0 * sigma)
    fraction = float(np.mean(inliers))
    quality.update(raw_depth_m=median, raw_depth_std_m=float(np.std(values)),
                   depth_sigma_m=sigma, inlier_fraction=fraction)
    if sigma > max_depth_sigma_m:
        quality['reason'] = 'depth_spread_too_large'
        return None
    if np.count_nonzero(inliers) < min_pixels or fraction < min_inlier_fraction:
        quality['reason'] = 'too_many_depth_outliers'
        return None
    # Average only the robustly selected interior surface to reduce quantisation.
    z_est = float(np.mean(values[inliers]))
    ray, _ = mask_sight_ray(core, fx, fy, cx, cy, distortion)
    quality.update(reason='ok', depth_m=z_est)
    point = ray * z_est
    return MarkerEstimate(point, z_est, count, count / mask_count,
                          sigma, quality['raw_depth_std_m'], fraction)


class StationaryMarkerFilter:
    """Robust time filter for a stationary dock marker in odom coordinates.

    Filter after the measured-time TF so boat motion does not create range lag.
    Isolated innovations are rejected. A consistent new cluster is reacquired;
    stale state and non-increasing stamps cannot be published as new samples.
    """
    def __init__(self, tau_s=0.5, gate_m=0.35, reset_after_s=1.5,
                 warmup_samples=3, reacquire_samples=5, cluster_m=0.15):
        if not all(math.isfinite(x) for x in (tau_s, gate_m, reset_after_s, cluster_m)):
            raise ValueError('filter parameters must be finite')
        if tau_s <= 0 or gate_m <= 0 or reset_after_s <= 0:
            raise ValueError('filter time constants and gate must be positive')
        if warmup_samples < 1 or reacquire_samples < 1 or cluster_m <= 0:
            raise ValueError('filter sample counts and cluster radius must be positive')
        self.tau = tau_s
        self.gate = gate_m
        self.reset_after = reset_after_s
        self.warmup = warmup_samples
        self.reacquire = reacquire_samples
        self.cluster = cluster_m
        self.has_initialized = False
        self.epoch = 0
        self.reset()

    def reset(self):
        self.epoch += 1
        self.value = None
        self.last_stamp = None
        self.last_accepted = None
        self.samples = deque(maxlen=5)
        self.pending = []

    def update(self, xy, stamp):
        point = np.asarray(xy, dtype=float)
        if point.shape != (2,) or not np.isfinite(point).all() or not math.isfinite(stamp):
            return None, 'invalid_measurement'
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return None, 'non_increasing_stamp'
        if ((self.last_accepted is not None and stamp - self.last_accepted > self.reset_after)
                or (self.last_stamp is not None and stamp - self.last_stamp > self.reset_after)):
            self.reset()
        self.last_stamp = stamp
        if self.value is None:
            if self.pending and np.linalg.norm(point - np.median(self.pending, axis=0)) > self.cluster:
                self.pending.clear()
            self.pending.append(point)
            if len(self.pending) < self.warmup:
                return None, 'initializing'
            self.value = np.median(self.pending, axis=0)
            self.samples.extend(self.pending)
            self.pending.clear()
            self.last_accepted = stamp
            state = 'reinitialized' if self.has_initialized else 'initialized'
            self.has_initialized = True
            return self.value.copy(), state
        if np.linalg.norm(point - self.value) > self.gate:
            if self.pending and np.linalg.norm(point - np.median(self.pending, axis=0)) > self.cluster:
                self.pending.clear()
            self.pending.append(point)
            if len(self.pending) < self.reacquire:
                return None, 'innovation_rejected'
            self.value = np.median(self.pending, axis=0)
            self.samples.clear()
            self.samples.extend(self.pending)
            self.pending.clear()
            self.last_accepted = stamp
            self.epoch += 1
            return self.value.copy(), 'reacquired'
        self.pending.clear()
        self.samples.append(point)
        target = np.median(self.samples, axis=0)
        alpha = 1.0 - math.exp(-(stamp - self.last_accepted) / self.tau)
        self.value += alpha * (target - self.value)
        self.last_accepted = stamp
        return self.value.copy(), 'tracking'
