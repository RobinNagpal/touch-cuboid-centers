"""From camera pixels to points in the room.

Two kinds of pixel matter. Coloured ones are the parts: the legs and the table
top are painted in saturated colours and nothing else in the room is. Grey
ones are everything else — the floor, the stands, anything in the way — and are
kept too, because the floor has to be found and obstacles have to be avoided.

Only numpy and OpenCV, so this can be tested with no simulator running.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


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


def ranged_mask(depth: np.ndarray, max_range: float = 2.0) -> np.ndarray:
    """Pixels the camera got a trustworthy range for.

    Past two metres the floor is seen at such a shallow angle that one pixel
    covers centimetres of it, which is no use to anything here.
    """
    return np.isfinite(depth) & (depth > 0.0) & (depth < max_range)


def colour_masks(
    rgb: np.ndarray,
    depth: np.ndarray,
    *,
    min_saturation: int = 90,
    min_value: int = 50,
) -> tuple[np.ndarray, np.ndarray]:
    """Two masks: pixels that show a part, and pixels that show everything else.

    Pixels whose depth jumps away from their own kind around them are dropped
    from both; see ``flying_pixels``.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    coloured = (hsv[:, :, 1] >= min_saturation) & (hsv[:, :, 2] >= min_value)
    ranged = ranged_mask(depth)
    parts = coloured & ranged
    rest = ~coloured & ranged
    return parts & ~flying_pixels(depth, parts), rest & ~flying_pixels(depth, rest)


def flying_pixels(depth: np.ndarray, mask: np.ndarray, *, tolerance: float = 0.01, per_metre: float = 0.01):
    """Masked pixels whose depth disagrees with the masked pixels next to them.

    A depth camera looking at an edge can return a range somewhere between
    the object and whatever is behind it, and that pixel lands in mid-air,
    where it would stretch a measured part or invent an obstacle. Such a pixel
    stands out because its range differs from all of its neighbours'.

    Only neighbours inside the same mask are compared. A pixel on the edge of
    a leg has floor next to it, but the floor is not in the leg's mask, so the
    edge pixel is kept. That matters: simply shaving a pixel off every edge
    would make every part measure one to two pixels small, which is enough to
    upset the grip.

    The allowed disagreement grows with range, because a surface seen at a
    slant changes depth from one pixel to the next by more the further off it
    is.
    """
    kernel = np.ones((3, 3), np.uint8)
    nearest = cv2.erode(np.where(mask, depth, np.inf).astype(np.float32), kernel)
    furthest = cv2.dilate(np.where(mask, depth, -np.inf).astype(np.float32), kernel)
    allowed = tolerance + per_metre * np.nan_to_num(depth, nan=0.0, posinf=0.0)
    with np.errstate(invalid="ignore"):
        return mask & (((depth - nearest) > allowed) | ((furthest - depth) > allowed))


def back_project(
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsics: Intrinsics,
    camera_to_world: np.ndarray,
    *,
    stride: int = 1,
) -> np.ndarray:
    """Masked depth pixels as an (N, 3) array of world points.

    ``camera_to_world`` is a 4x4 transform for the camera's *optical* frame,
    the ROS convention where z points along the view direction, x right and y
    down. ``stride`` keeps one pixel in every ``stride`` along each image axis,
    for clouds where density matters less than speed.
    """
    if stride > 1:
        sparse = np.zeros_like(mask)
        sparse[::stride, ::stride] = mask[::stride, ::stride]
        mask = sparse
    rows, cols = np.nonzero(mask)
    z = depth[rows, cols].astype(float)
    x = (cols - intrinsics.cx) * z / intrinsics.fx
    y = (rows - intrinsics.cy) * z / intrinsics.fy
    points = np.stack((x, y, z), axis=1)
    return points @ camera_to_world[:3, :3].T + camera_to_world[:3, 3]
