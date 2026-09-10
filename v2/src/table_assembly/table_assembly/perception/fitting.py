"""Shapes from point clouds: the floor, boxes resting on it, and flat plates.

Every function here is given world points and hands back numbers. Nothing is
assumed about where anything is or how big it is; the only physical fact used
is that gravity points down, so the floor is level and the edge a board stands
on is horizontal.
"""

from __future__ import annotations

from collections import deque

import cv2
import numpy as np

from ..geometry import Box
from ..transforms import WORLD_Z

# 26-neighbourhood of a voxel, used to grow clusters.
_NEIGHBOURS = np.array(
    [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ],
    dtype=np.int64,
)


def cluster(points: np.ndarray, *, voxel: float = 0.012, min_points: int = 120) -> list[np.ndarray]:
    """Split a point cloud into one cluster per object, biggest first.

    Points are dropped into a voxel grid and neighbouring occupied voxels are
    flood-filled together. That is enough because the parts are set apart from
    each other, and it keeps the cost linear in the number of occupied voxels
    rather than quadratic in the number of points.
    """
    if len(points) == 0:
        return []

    keys = np.floor(points / voxel).astype(np.int64)
    voxels, point_voxel = np.unique(keys, axis=0, return_inverse=True)
    point_voxel = point_voxel.reshape(-1)
    lookup = {tuple(key): index for index, key in enumerate(voxels)}

    labels = np.full(len(voxels), -1, dtype=np.int64)
    next_label = 0
    for seed in range(len(voxels)):
        if labels[seed] != -1:
            continue
        labels[seed] = next_label
        queue = deque([seed])
        while queue:
            current = queue.popleft()
            for neighbour in voxels[current] + _NEIGHBOURS:
                index = lookup.get(tuple(neighbour))
                if index is not None and labels[index] == -1:
                    labels[index] = next_label
                    queue.append(index)
        next_label += 1

    per_point = labels[point_voxel]
    clusters = [points[per_point == label] for label in range(next_label)]
    clusters = [c for c in clusters if len(c) >= min_points]
    clusters.sort(key=lambda c: -len(c))
    return clusters


def floor_height(points: np.ndarray, *, bin_size: float = 0.002) -> float:
    """Height of the floor, from every grey point the camera saw.

    The floor is by far the biggest level surface in view, so the height that
    the most points share is the floor. The median of the points near that
    peak then refines it to well under a bin.
    """
    if len(points) < 100:
        raise ValueError("too few points to find the floor")
    z = points[:, 2]
    edges = np.arange(z.min(), z.max() + 2 * bin_size, bin_size)
    counts, edges = np.histogram(z, bins=edges)
    peak = edges[int(np.argmax(counts))] + bin_size / 2.0
    return float(np.median(z[np.abs(z - peak) < 0.01]))


def fit_resting_box(points: np.ndarray, floor_z: float, *, floor_margin: float = 0.004) -> Box | None:
    """Fit a box sitting square on the floor to one cluster of points.

    Height comes straight from the highest point above the floor, because the
    camera looks down on the object and always sees its top. Length and width
    come from the smallest rectangle enclosing the same points seen from above,
    which is exactly the object's footprint. This needs no iteration and no
    initial guess.

    The box's axes are: 0 the longer horizontal side, 1 the shorter one, 2 up.
    Returns ``None`` if the cluster is too thin to be anything.
    """
    above = points[points[:, 2] > floor_z + floor_margin]
    if len(above) < 30:
        return None

    height = float(above[:, 2].max() - floor_z)

    # OpenCV's convex hull maths is tuned for pixel-sized numbers. In metres
    # every coordinate here sits inside a 1.0 box, so work in millimetres.
    footprint = (above[:, :2] * 1000.0).astype(np.float32)
    (centre_x, centre_y), (side_a, side_b), angle_deg = cv2.minAreaRect(footprint)

    length, width = side_a / 1000.0, side_b / 1000.0
    yaw = np.deg2rad(angle_deg)
    if width > length:
        # Keep axis 0 along the longer side, turning the box a quarter turn to
        # match. Without this a box would change its reported orientation as
        # it rotated past 45 degrees.
        length, width = width, length
        yaw += np.pi / 2.0
    # A box looks the same after half a turn, so fold yaw into [-pi/2, pi/2).
    yaw = (yaw + np.pi / 2.0) % np.pi - np.pi / 2.0

    return Box.upright(
        (centre_x / 1000.0, centre_y / 1000.0, floor_z + height / 2.0),
        float(yaw),
        (length, width, height),
    )


def fit_plate(points: np.ndarray, *, bands: tuple[float, ...] = (0.006, 0.003, 0.002)) -> Box | None:
    """Fit a thin flat board, at any angle, to one cluster of points.

    The camera sees the board's broad front face and, from above, a strip of
    its top edge. The face is found first: a plane through the points, fitted
    by principal component analysis (the direction the points vary least in is
    the plane's normal). The edge strip sits behind the face and drags that
    first plane off true, so the fit is repeated using only the points close
    to the plane, with a tighter band each time, until only the face is left.

    With the face known, everything is measured in the face's own axes: the
    spread across it gives length and width, and how far the edge strip
    reaches behind it gives the thickness.

    The box's axes are: 0 along the board's horizontal edge (its length), 1 up
    the face (its width), 2 out of the front face, which is the face tilted
    upwards. Returns ``None`` for a cluster too small to fit.
    """
    if len(points) < 100:
        return None

    face = points
    for band in bands:
        centroid = face.mean(axis=0)
        _, _, axes = np.linalg.svd(face - centroid, full_matrices=False)
        normal = axes[2]
        face = points[np.abs((points - centroid) @ normal) < band]
        if len(face) < 50:
            return None

    centroid = face.mean(axis=0)
    _, _, axes = np.linalg.svd(face - centroid, full_matrices=False)
    normal = axes[2] if axes[2][2] >= 0.0 else -axes[2]

    # "Up the face" is the world vertical flattened onto the face. For a board
    # lying almost level that is undefined, so the face's own longest spread
    # stands in for it.
    up = WORLD_Z - float(WORLD_Z @ normal) * normal
    if np.linalg.norm(up) < 0.2:
        up = axes[0]
    up = up / np.linalg.norm(up)
    along = np.cross(up, normal)
    rotation = np.column_stack((along, up, normal))

    local = (points - centroid) @ rotation
    low, high = local.min(axis=0), local.max(axis=0)
    face_level = float(np.percentile(local[:, 2], 99.0))
    thickness = face_level - float(low[2])

    centre_local = np.array(
        [(low[0] + high[0]) / 2.0, (low[1] + high[1]) / 2.0, face_level - thickness / 2.0]
    )
    size = np.array([high[0] - low[0], high[1] - low[1], thickness])
    if size[1] > size[0]:
        # Axis 0 is the longer side. A quarter turn about the normal swaps the
        # two while keeping the frame right-handed.
        rotation = np.column_stack((up, -along, normal))
        size = size[[1, 0, 2]]
    return Box(centroid + (centre_local @ np.column_stack((along, up, normal)).T), rotation, size)


def surface_tilt(points: np.ndarray) -> float:
    """Angle between a surface's normal and the vertical, in radians."""
    centred = points - points.mean(axis=0)
    _, _, axes = np.linalg.svd(centred, full_matrices=False)
    return float(np.arccos(min(1.0, abs(float(axes[2][2])))))
