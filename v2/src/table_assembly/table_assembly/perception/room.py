"""What is in the room, worked out from nothing but what the camera saw.

The arm starts knowing only where it stands. From the pooled points of every
view it works out, in this order:

1. the floor, which every other height is measured from;
2. the parts, which are the coloured clusters, told apart by shape alone —
   the table top is the one broad thin plate, a leg is a stick;
3. the obstacles, which are the grey clusters standing up off the floor —
   standing on it, that is: a grey cluster floating above the floor is the
   arm seeing its own body.

No colour, size or position is looked up anywhere. A leg is a leg because it
is long and thin, not because it is red or because something said where it
would be.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..geometry import Box
from .fitting import cluster, fit_plate, fit_resting_box, floor_height

# A stick is at least this many times longer than it is thick.
STICK_RATIO = 2.0

# A plate is at most this thick compared with its shorter side.
PLATE_RATIO = 0.25

# Grey points this close to a part are that part's own dimly lit sides, not an
# obstacle standing next to it.
PART_HALO = 0.03

# Grey points lower than this above the floor are the floor.
FLOOR_BAND = 0.015

# An obstacle reaches down to within this of the floor. Anything grey that
# hangs in mid-air above it is the arm's own body caught at the edge of a
# picture, not something standing in the room.
GROUNDED = 0.05


@dataclass
class Room:
    floor_z: float
    top: Box | None = None
    legs: list[Box] = field(default_factory=list)
    obstacles: list[Box] = field(default_factory=list)
    unknown: list[Box] = field(default_factory=list)

    def everything(self) -> list[Box]:
        """Every box in the room, whatever it is."""
        return ([self.top] if self.top is not None else []) + self.legs + self.obstacles + self.unknown


def leg_length(leg: Box) -> float:
    return float(leg.size.max())


def leg_thickness(leg: Box) -> float:
    """The larger of a leg's two short sides, the one that decides if it fits the gripper."""
    return float(np.sort(leg.size)[1])


def is_standing(leg: Box) -> bool:
    return bool(leg.size[2] > max(leg.size[0], leg.size[1]))


def read_room(
    part_points: np.ndarray,
    other_points: np.ndarray,
    *,
    self_centre: np.ndarray,
    self_radius: float,
    floor_z: float | None = None,
) -> Room:
    """Floor, parts and obstacles from the pooled points of several views.

    The floor is found from the points unless ``floor_z`` is given. It is found
    once, from the wide survey where it fills most of every picture, and then
    handed back in for close-up views, where a part or the wall may fill more
    of the picture than the floor does.
    """
    part_points = _outside(part_points, self_centre, self_radius)
    other_points = _outside(other_points, self_centre, self_radius)

    if floor_z is None:
        floor_z = floor_height(other_points)
    room = Room(floor_z=floor_z)

    part_clusters = cluster(part_points)
    shapes = [(points, fit_plate(points), fit_resting_box(points, floor_z)) for points in part_clusters]
    shapes = [shape for shape in shapes if shape[2] is not None]

    # The table top is the broadest thin thing in the room. Everything else
    # that is coloured is either a leg or a part that fits neither shape.
    plates = [shape for shape in shapes if shape[1] is not None and _is_plate(shape[1])]
    top = max(plates, key=lambda shape: shape[1].size[0] * shape[1].size[1], default=None)
    for shape in shapes:
        _, plate, resting = shape
        if shape is top:
            room.top = plate
        elif _is_stick(resting):
            room.legs.append(resting)
        else:
            room.unknown.append(resting)

    raised = other_points[other_points[:, 2] > floor_z + FLOOR_BAND]
    raised = _away_from(raised, part_points, PART_HALO)
    for points in cluster(raised, voxel=0.015, min_points=150):
        if points[:, 2].min() > floor_z + GROUNDED:
            continue
        box = fit_resting_box(points, floor_z)
        if box is not None:
            room.obstacles.append(box)
    return room


def _is_plate(box: Box) -> bool:
    return bool(box.size[2] < PLATE_RATIO * box.size[1])


def _is_stick(box: Box) -> bool:
    sides = np.sort(box.size)
    return bool(sides[2] > STICK_RATIO * sides[1])


def _outside(points: np.ndarray, centre: np.ndarray, radius: float) -> np.ndarray:
    """Points further than ``radius`` from ``centre`` seen from above."""
    return points[np.linalg.norm(points[:, :2] - centre[:2], axis=1) > radius]


def _away_from(points: np.ndarray, others: np.ndarray, distance: float) -> np.ndarray:
    """Points not within about ``distance`` of any of ``others``.

    Done on a voxel grid rather than point by point: every voxel an ``other``
    falls in, and its neighbours, is marked, and points in marked voxels are
    dropped. That makes it approximate to within a voxel, which is plenty for
    telling a dim side of a part from a wall.
    """
    if len(points) == 0 or len(others) == 0:
        return points
    other_keys = np.unique(np.floor(others / distance).astype(np.int64), axis=0)
    offsets = np.array([(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)])
    marked = {tuple(key) for key in (other_keys[:, None, :] + offsets[None, :, :]).reshape(-1, 3)}
    keys, inverse = np.unique(np.floor(points / distance).astype(np.int64), axis=0, return_inverse=True)
    keep = np.array([tuple(key) not in marked for key in keys], dtype=bool)
    return points[keep[inverse.reshape(-1)]]
