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


@dataclass(frozen=True)
class MarkerCloud:
    point_optical: np.ndarray
    points_optical: np.ndarray
    median_depth_m: float
    valid_pixels: int
    valid_fraction: float


@dataclass(frozen=True)
class WallEstimate:
    center_optical: np.ndarray
    outward_normal_optical: np.ndarray
    plane_rms_m: float
    marker_names: tuple[str, ...]
    inlier_fractions: dict[str, float]


def extract_marker_cloud(mask, depth_m, fx, fy, cx, cy, *, min_depth=0.25,
                    max_depth=12.0, min_pixels=40,
                    sample_step=2, distortion=(), erosion_pixels=0) -> MarkerCloud | None:
    """Extract one mask independently; no per-marker plane fit is required."""
    mask = np.asarray(mask, dtype=bool)
    depth_m = np.asarray(depth_m, dtype=np.float32)
    if (mask.shape != depth_m.shape or mask.ndim != 2
            or not np.all(np.isfinite([fx, fy, cx, cy])) or fx <= 0 or fy <= 0):
        return None
    if erosion_pixels > 0:
        size = 2 * int(erosion_pixels) + 1
        mask = cv2.erode(mask.astype(np.uint8), np.ones((size, size), np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
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
    return MarkerCloud(point, cloud, z_med, count, count / mask_count)


def estimate_marker(mask, depth_m, fx, fy, cx, cy, *, min_depth=0.25,
                    max_depth=12.0, min_pixels=40, max_plane_rms=0.08,
                    sample_step=2, distortion=()) -> MarkerEstimate | None:
    """Legacy single-marker estimator retained for callers and comparison."""
    extracted = extract_marker_cloud(
        mask, depth_m, fx, fy, cx, cy, min_depth=min_depth, max_depth=max_depth,
        min_pixels=min_pixels, sample_step=sample_step, distortion=distortion)
    if extracted is None:
        return None
    cloud = extracted.points_optical
    point = extracted.point_optical
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
    return MarkerEstimate(point, normal, extracted.median_depth_m,
                          extracted.valid_pixels, extracted.valid_fraction, rms)


def estimate_common_wall(clouds: dict[str, MarkerCloud], *,
                         inlier_distance=0.03, min_inlier_fraction=0.6,
                         max_plane_rms=0.08, max_points_per_marker=256,
                         iterations=128) -> WallEstimate | None:
    """Balanced robust plane fit without a vertical-wall/gravity constraint.

    With multiple available masks, at least two must support the plane.
    A single visible mask may stand alone, but collinear clouds never do.
    """
    if (not clouds or not np.isfinite(inlier_distance) or inlier_distance <= 0
            or not 0 < min_inlier_fraction <= 1 or max_points_per_marker < 3
            or iterations < 1 or not np.isfinite(max_plane_rms) or max_plane_rms <= 0):
        return None
    groups = {}
    for name, cloud in sorted(clouds.items()):
        points = np.asarray(cloud.points_optical, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            continue
        points = points[np.all(np.isfinite(points), axis=1)]
        if len(points) >= 12:
            groups[name] = points[np.linspace(
                0, len(points) - 1, min(len(points), max_points_per_marker), dtype=int)]
    if not groups:
        return None
    required = 1 if len(clouds) == 1 else 2

    def fit(parts):
        points = np.concatenate(parts)
        weights = np.concatenate([np.full(len(p), 1.0 / len(p)) for p in parts])
        weights /= weights.sum()
        center = np.sum(points * weights[:, None], axis=0)
        _, singular, vectors = np.linalg.svd(
            (points - center) * np.sqrt(weights[:, None]), full_matrices=False)
        # Require observable area, not just three nearly collinear centroids.
        if len(singular) < 3 or singular[1] < 0.005:
            return None
        return center, vectors[-1]

    def classify(center, normal):
        masks = {name: np.abs((p - center) @ normal) <= inlier_distance
                 for name, p in groups.items()}
        supported = tuple(name for name, mask in masks.items()
                          if mask.sum() >= 12 and mask.mean() >= min_inlier_fraction)
        return masks, supported

    candidates = []
    for parts in ([p] for p in groups.values()):
        result = fit(parts)
        if result is not None:
            candidates.append(result)
    result = fit(list(groups.values()))
    if result is not None:
        candidates.append(result)
    points = np.concatenate(list(groups.values()))
    probabilities = np.concatenate([np.full(len(p), 1 / len(p)) for p in groups.values()])
    probabilities /= probabilities.sum()
    rng = np.random.default_rng(0)
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False, p=probabilities)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length > 1e-8:
            candidates.append((a, normal / length))
    best = None
    best_score = None
    for center, normal in candidates:
        masks, supported = classify(center, normal)
        if len(supported) < required:
            continue
        score = (len(supported), sum(float(masks[n].mean()) for n in supported),
                 -sum(float(np.mean(np.abs((groups[n][masks[n]] - center) @ normal)))
                      for n in supported))
        if best_score is None or score > best_score:
            best_score, best = score, (masks, supported)
    if best is None:
        return None
    masks, supported = best
    # Refine with equal total weight per contributing marker, not per pixel.
    for _ in range(5):
        result = fit([groups[n][masks[n]] for n in supported])
        if result is None:
            return None
        center, normal = result
        updated, names = classify(center, normal)
        if len(names) < required:
            return None
        stable = names == supported and all(np.array_equal(updated[n], masks[n]) for n in names)
        masks, supported = updated, names
        if stable:
            break
    rms = float(np.sqrt(np.mean([
        np.mean(((groups[n][masks[n]] - center) @ normal) ** 2) for n in supported])))
    if rms > max_plane_rms:
        return None
    if np.dot(normal, -center) < 0:
        normal = -normal
    return WallEstimate(center, normal, rms, supported,
                        {n: float(m.mean()) for n, m in masks.items()})


def marker_on_wall(cloud: MarkerCloud, wall: WallEstimate) -> MarkerEstimate:
    """Keep the selected marker's location, projected onto the shared plane."""
    normal = wall.outward_normal_optical
    point = cloud.point_optical
    projected = point - np.dot(point - wall.center_optical, normal) * normal
    return MarkerEstimate(projected, normal, cloud.median_depth_m,
                          cloud.valid_pixels, cloud.valid_fraction, wall.plane_rms_m)


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
