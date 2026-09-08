"""Cuboid geometry: the six faces of a box, their areas, and where to touch.

A cuboid on a level table can only be rotated about the world z axis, so a box
is fully described by its centre, a single yaw angle, and three side lengths.
Everything in this module is plain numpy, with no ROS in it, so it can be
tested on its own.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Index of each side length inside ``Cuboid.size``.
LENGTH, WIDTH, HEIGHT = 0, 1, 2

_AXIS_NAMES = ("length", "width", "height")


@dataclass(frozen=True)
class Face:
    """One of the six faces of a cuboid, in world coordinates.

    ``name`` says which of the box's own axes the face looks along, so
    ``+length`` is the face whose outward normal is the box's +length axis.
    That face itself measures width by height.
    """

    name: str
    area: float
    centre: np.ndarray
    normal: np.ndarray


@dataclass(frozen=True)
class Cuboid:
    """A box resting flat on a table.

    ``size`` is (length, width, height) measured along the box's own x, y and
    z axes. By convention length is the longer of the two horizontal sides.
    """

    centre: np.ndarray
    yaw: float
    size: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "centre", np.asarray(self.centre, dtype=float))
        object.__setattr__(self, "size", np.asarray(self.size, dtype=float))

    @property
    def rotation(self) -> np.ndarray:
        """The 3x3 rotation that takes box axes to world axes."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    @property
    def top_z(self) -> float:
        return float(self.centre[2] + self.size[HEIGHT] / 2.0)

    def faces(self) -> tuple[Face, ...]:
        """All six faces, in world coordinates."""
        rotation = self.rotation
        result = []
        for axis in (LENGTH, WIDTH, HEIGHT):
            others = [i for i in (LENGTH, WIDTH, HEIGHT) if i != axis]
            area = float(self.size[others[0]] * self.size[others[1]])
            for sign, prefix in ((1.0, "+"), (-1.0, "-")):
                normal = rotation[:, axis] * sign
                centre = self.centre + normal * (self.size[axis] / 2.0)
                result.append(Face(f"{prefix}{_AXIS_NAMES[axis]}", area, centre, normal))
        return tuple(result)

    def distinct_face_areas(self) -> dict[str, float]:
        """The three areas a cuboid actually has, opposite faces being equal."""
        length, width, height = (float(v) for v in self.size)
        return {
            "length x width": length * width,
            "length x height": length * height,
            "width x height": width * height,
        }


def largest_touchable_face(cuboid: Cuboid, reach_from: np.ndarray) -> Face:
    """The biggest face the arm can actually reach.

    The face lying on the table is dropped: the arm cannot get under the box.
    Opposite faces always tie on area, so ties are broken by whichever face
    centre is closer to ``reach_from`` (the robot base), which picks the side
    the arm can approach without reaching across the box. Rounding the area
    before comparing stops millimetre-level measurement noise from deciding a
    tie for us.
    """
    reach_from = np.asarray(reach_from, dtype=float)
    touchable = [face for face in cuboid.faces() if face.normal[2] > -0.5]
    return min(
        touchable,
        key=lambda face: (
            -round(face.area, 6),
            float(np.linalg.norm(face.centre - reach_from)),
            face.name,
        ),
    )
