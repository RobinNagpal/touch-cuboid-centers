"""A box in the room: the one shape everything here is described with.

Legs, the table top, the wall and the table that gets built are all boxes. A
box is a centre, a rotation and three side lengths, and the rotation is a full
3x3 matrix rather than a single yaw, because the table top starts off leaning
against a wall and a yaw cannot describe that.

Plain numpy, no ROS, so it can be tested on its own.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .transforms import frame, rotation_z


@dataclass(frozen=True)
class Box:
    """A box in the world frame.

    ``size[i]`` is the side length along ``rotation[:, i]``, the box's own i-th
    axis. What each axis means depends on what the box is, and the code that
    builds one says so.
    """

    centre: np.ndarray
    rotation: np.ndarray
    size: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "centre", np.asarray(self.centre, dtype=float))
        object.__setattr__(self, "rotation", np.asarray(self.rotation, dtype=float))
        object.__setattr__(self, "size", np.asarray(self.size, dtype=float))

    @classmethod
    def upright(cls, centre, yaw: float, size) -> Box:
        """A box sitting square on a level surface, turned ``yaw`` about the vertical."""
        return cls(centre, rotation_z(yaw), size)

    @property
    def pose(self) -> np.ndarray:
        """The 4x4 transform from the box's own frame to the world."""
        return frame(self.centre, self.rotation)

    def axis(self, index: int) -> np.ndarray:
        return self.rotation[:, index]

    def corners(self) -> np.ndarray:
        """The eight corners, as an (8, 3) array."""
        signs = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
        return self.centre + (signs * (self.size / 2.0)) @ self.rotation.T

    @property
    def bottom_z(self) -> float:
        return float(self.corners()[:, 2].min())

    @property
    def top_z(self) -> float:
        return float(self.corners()[:, 2].max())

    def footprint_distance(self, points_xy: np.ndarray, margin: float = 0.0) -> np.ndarray:
        """Distance from each 2D point to this box seen from above.

        Only makes sense for a box standing square, which is the only kind it is
        asked about: the table that is going to be built. Zero for a point
        inside the footprint.
        """
        heading = self.rotation[:2, 0] / np.linalg.norm(self.rotation[:2, 0])
        across = np.array([-heading[1], heading[0]])
        offset = np.asarray(points_xy, dtype=float) - self.centre[:2]
        along_x = np.abs(offset @ heading) - (self.size[0] / 2.0 + margin)
        along_y = np.abs(offset @ across) - (self.size[1] / 2.0 + margin)
        return np.hypot(np.maximum(along_x, 0.0), np.maximum(along_y, 0.0))


def describe(size: np.ndarray) -> str:
    """Side lengths in centimetres, for the log."""
    return " x ".join(f"{float(v) * 100:.1f}" for v in size) + " cm"
