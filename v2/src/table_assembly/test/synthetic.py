"""Point clouds a camera would see, made from boxes whose true size is known.

The tests use these in place of the simulator. A box's surface is sampled on
a fine grid, keeping only the faces that look towards the camera, which is
what a depth camera returns: the near sides of things, never the far ones.
"""

from __future__ import annotations

import math

import numpy as np
from table_assembly.geometry import Box
from table_assembly.transforms import rotation_z


def visible_surface(box: Box, camera: np.ndarray, spacing: float = 0.002) -> np.ndarray:
    """Points on every face of ``box`` that faces ``camera``."""
    clouds = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        for sign in (1.0, -1.0):
            normal = box.axis(axis) * sign
            face_centre = box.centre + normal * box.size[axis] / 2.0
            if float(normal @ (camera - face_centre)) <= 0.0:
                continue
            us = np.arange(-box.size[others[0]] / 2, box.size[others[0]] / 2 + 1e-9, spacing)
            vs = np.arange(-box.size[others[1]] / 2, box.size[others[1]] / 2 + 1e-9, spacing)
            grid_u, grid_v = np.meshgrid(us, vs, indexing="ij")
            clouds.append(
                face_centre
                + grid_u.reshape(-1, 1) * box.axis(others[0])
                + grid_v.reshape(-1, 1) * box.axis(others[1])
            )
    return np.concatenate(clouds)


def floor_points(radius: float = 1.0, spacing: float = 0.01, z: float = 0.0) -> np.ndarray:
    """A square of level floor round the origin."""
    xs = np.arange(-radius, radius, spacing)
    grid_x, grid_y = np.meshgrid(xs, xs, indexing="ij")
    return np.stack([grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, z)], axis=1)


def spawned_box(spawned) -> Box:
    """A simulator box, as the same Box type the robot measures with."""
    return Box(spawned.centre, spawned.rotation, spawned.size)


def leaning_board(length, width, thickness, lean, yaw=0.3, centre=(0.5, 0.2, 0.1)):
    """A board standing on its long edge, leaning ``lean`` back from upright."""
    outward = rotation_z(yaw)[:, 0]
    along = rotation_z(yaw)[:, 1]
    up = outward * math.sin(lean) + np.array([0.0, 0.0, math.cos(lean)])
    back = np.cross(along, up)
    return Box(np.array(centre), np.column_stack((along, up, back)), (length, width, thickness))
