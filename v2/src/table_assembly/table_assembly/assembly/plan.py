"""Planning the table: where each leg has to stand, and where the top goes.

The table is always built in the same place, ``SITE``. Everything else is
worked out from measurements: the top's size decides where the legs go, and
the legs' length decides how high the top ends up.

Plain numpy, no ROS, so it can be tested on its own.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry import Box

# Where the table is built, in the world frame: straight in front of the arm,
# close enough that the far legs are well inside its reach, and far enough out
# that it does not fold up tight at the top's near edge. At 55 cm, pulling the
# gripper back out from under the top folded the arm into itself. This is the
# arm's choice of where to work, not a fact about the room, so it is still
# looked at and has to be empty.
SITE = np.array([0.60, 0.0])

# How far in from the edges of the top a leg's outer faces stand. Flush with
# the edge looks right, but then a leg placed a few millimetres off would stick
# out from under the top, or have the top's edge land on it.
LEG_INSET = 0.012

# How much empty floor the table needs around it: room for the gripper and the
# fingers beside every leg.
SITE_CLEARANCE = 0.08


@dataclass(frozen=True)
class TablePlan:
    """The table that is going to be built."""

    table: Box  # the finished table: the top's footprint, from the floor to the top's upper face
    top: Box  # where the top goes, lying level on the legs
    leg_spots: tuple[np.ndarray, ...]  # where each leg stands, on the floor, furthest from the base first
    outward: np.ndarray  # horizontal unit vector from the arm's base towards the table


def table_footprint(top: Box) -> tuple[float, float]:
    """(side along the edge facing the arm, side reaching away from the arm).

    The top is picked up by a long edge and set down with that edge nearest
    the arm, so the table is as long across the arm's view as the top is long.
    """
    return float(top.size[0]), float(top.size[1])


def plan_table(
    top: Box,
    leg_length: float,
    leg_thickness: float,
    floor_z: float,
    centre_xy: np.ndarray,
    base: np.ndarray,
) -> TablePlan:
    """Where the top and the legs go, for a table centred at ``centre_xy``.

    The table is turned so its near edge faces the arm's base square on.
    """
    outward = np.array([centre_xy[0] - base[0], centre_xy[1] - base[1], 0.0])
    outward /= np.linalg.norm(outward)
    across = np.array([-outward[1], outward[0], 0.0])
    heading = math.atan2(across[1], across[0])

    length, depth = table_footprint(top)
    thickness = float(top.size[2])
    top_centre = np.array([centre_xy[0], centre_xy[1], floor_z + leg_length + thickness / 2.0])
    top_box = Box.upright(top_centre, heading, (length, depth, thickness))
    table = Box.upright(
        (centre_xy[0], centre_xy[1], floor_z + (leg_length + thickness) / 2.0),
        heading,
        (length, depth, leg_length + thickness),
    )

    half_along = length / 2.0 - LEG_INSET - leg_thickness / 2.0
    half_out = depth / 2.0 - LEG_INSET - leg_thickness / 2.0
    spots = [
        np.array([centre_xy[0], centre_xy[1], floor_z]) + across * a * half_along + outward * b * half_out
        for a in (-1.0, 1.0)
        for b in (-1.0, 1.0)
    ]
    # The far legs go in first. Standing a near leg up first would leave it
    # in the way of the gripper as it reaches past it to the far ones.
    spots.sort(key=lambda spot: -float(np.linalg.norm(spot[:2] - base[:2])))
    return TablePlan(table=table, top=top_box, leg_spots=tuple(spots), outward=outward)


def in_the_way(plan: TablePlan, things: list[Box], clearance: float = SITE_CLEARANCE) -> list[Box]:
    """Whatever the camera saw on or beside the patch of floor the table needs."""
    return [
        box
        for box in things
        if float(plan.table.footprint_distance(_footprint_samples(box)).min()) < clearance
    ]


def _footprint_samples(box: Box, step: float = 0.02) -> np.ndarray:
    """Points spread through a box, seen from above.

    Sampling the whole volume rather than the corners means a box at any
    angle is covered by the same code.
    """
    counts = [max(2, math.ceil(side / step) + 1) for side in box.size]
    grids = np.meshgrid(*[np.linspace(-s / 2.0, s / 2.0, n) for s, n in zip(box.size, counts, strict=True)])
    local = np.stack([g.ravel() for g in grids], axis=1)
    return (box.centre + local @ box.rotation.T)[:, :2]
