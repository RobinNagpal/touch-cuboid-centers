"""Turn RGB-D frames into cuboids.

The pipeline is four steps:

1. Mask the pixels that belong to a cuboid rather than to the table or the
   room. The cuboids are the only strongly coloured things in the cell, so a
   saturation threshold in HSV separates them.
2. Back-project those pixels through the camera intrinsics and the camera pose
   to get 3D points in the world frame.
3. Group the points into one cluster per cuboid.
4. Fit a box to each cluster.

Only numpy and OpenCV are used here, so the whole module can be tested with
synthetic point clouds and no simulator running.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import Cuboid

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


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole camera parameters, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_info(cls, msg) -> Intrinsics:
        k = msg.k
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


def object_mask(
    rgb: np.ndarray,
    depth: np.ndarray,
    *,
    min_saturation: int = 90,
    min_value: int = 50,
    max_range: float = 2.0,
) -> np.ndarray:
    """Pixels that show a cuboid.

    The table and the room are grey, the cuboids are painted in saturated
    colours, so saturation alone tells them apart. The depth check throws away
    pixels the camera did not get a range for and anything past the table.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    coloured = (hsv[:, :, 1] >= min_saturation) & (hsv[:, :, 2] >= min_value)
    ranged = np.isfinite(depth) & (depth > 0.0) & (depth < max_range)
    mask = coloured & ranged

    # Erode by a pixel: depth at an object's silhouette is a blend of the
    # object and whatever is behind it, and those mixed pixels land in mid-air.
    return cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)


def back_project(
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsics: Intrinsics,
    camera_to_world: np.ndarray,
) -> np.ndarray:
    """Masked depth pixels as an (N, 3) array of world points.

    ``camera_to_world`` is a 4x4 transform for the camera's *optical* frame,
    the ROS convention where z points along the view direction, x right and y
    down.
    """
    rows, cols = np.nonzero(mask)
    z = depth[rows, cols].astype(float)
    x = (cols - intrinsics.cx) * z / intrinsics.fx
    y = (rows - intrinsics.cy) * z / intrinsics.fy
    points = np.stack((x, y, z), axis=1)
    return points @ camera_to_world[:3, :3].T + camera_to_world[:3, 3]


def cluster(points: np.ndarray, *, voxel: float = 0.012, min_points: int = 120) -> list[np.ndarray]:
    """Split a point cloud into one cluster per object.

    Points are dropped into a voxel grid and neighbouring occupied voxels are
    flood-filled together. That is enough because the task assumes the cuboids
    are set apart from each other, and it keeps the cost linear in the number
    of occupied voxels rather than quadratic in the number of points.
    """
    if len(points) == 0:
        return []

    keys = np.floor(points / voxel).astype(np.int64)
    voxels, point_voxel = np.unique(keys, axis=0, return_inverse=True)
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


def fit_cuboid(points: np.ndarray, table_z: float, *, table_margin: float = 0.004) -> Cuboid | None:
    """Fit a box to one cluster of world points.

    Height comes straight from the highest point above the table, because the
    camera looks down on the box and always sees its top face. Length and
    width come from the smallest rectangle enclosing the same points seen from
    above, which is exactly the box's footprint.

    Returns ``None`` if the cluster is too thin to be a cuboid.
    """
    above = points[points[:, 2] > table_z + table_margin]
    if len(above) < 30:
        return None

    height = float(above[:, 2].max() - table_z)

    # OpenCV's convex hull maths is tuned for pixel-sized numbers. In metres
    # every coordinate here sits inside a 1.0 box, so work in millimetres.
    footprint = (above[:, :2] * 1000.0).astype(np.float32)
    (centre_x, centre_y), (side_a, side_b), angle_deg = cv2.minAreaRect(footprint)

    length, width = side_a / 1000.0, side_b / 1000.0
    yaw = np.deg2rad(angle_deg)
    if width > length:
        # Keep length as the longer horizontal side, and turn the box a quarter
        # turn so its local x axis still points along that side.
        length, width = width, length
        yaw += np.pi / 2.0

    # A box looks the same after half a turn, so fold yaw into [-pi/2, pi/2).
    yaw = (yaw + np.pi / 2.0) % np.pi - np.pi / 2.0

    return Cuboid(
        centre=np.array([centre_x / 1000.0, centre_y / 1000.0, table_z + height / 2.0]),
        yaw=float(yaw),
        size=np.array([length, width, height]),
    )


def find_cuboids(points: np.ndarray, table_z: float, **kwargs) -> list[Cuboid]:
    """Cluster a cloud and fit a box to every cluster."""
    found = (fit_cuboid(c, table_z) for c in cluster(points, **kwargs))
    return [box for box in found if box is not None]
