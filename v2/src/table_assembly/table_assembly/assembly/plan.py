"""Planning the table: where to build it and where each leg has to stand.

Everything is worked out from measurements. The top's size decides where the
legs go, the legs' length decides how high the top ends up, and the free floor
the camera saw decides where the whole thing is built.

Plain numpy, no ROS, so it can be tested on its own.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..geometry import Box

# How far in from the edges of the top a leg's outer faces stand. Flush with
# the edge looks right, but then a leg placed a few millimetres off would stick
# out from under the top, or have the top's edge land on it.
LEG_INSET = 0.012

# How much empty floor the table needs around it. The arm reaches in from the
# side nearest its base with the gripper held level, so the gripper body and
# wrist need space beside every leg.
SITE_CLEARANCE = 0.12


@dataclass(frozen=True)
class TablePlan:
    """The table that is going to be built."""

    table: Box  # the finished table: the top's footprint, from the floor to the top's upper face
    top: Box  # where the top goes, lying level on the legs
    leg_spots: tuple[np.ndarray, ...]  # where each leg stands, on the floor, furthest from the base first
    outward: np.ndarray  # horizontal unit vector from the arm's base towards the table


def gripped_axis(top: Box) -> tuple[int, float]:
    """Which of the top's two in-plane axes points up the slope, and its sign.

    The top is picked up by the edge that is highest, and that edge is at the
    positive or negative end of whichever in-plane axis leans most upwards.
    """
    index = 0 if abs(top.axis(0)[2]) > abs(top.axis(1)[2]) else 1
    return index, 1.0 if top.axis(index)[2] >= 0.0 else -1.0


def table_footprint(top: Box) -> tuple[float, float]:
    """(side along the edge facing the arm, side reaching away from the arm).

    The top is carried by one edge and set down with that edge nearest the
    arm, so the edge that was gripped becomes the near side of the table.
    """
    index, _ = gripped_axis(top)
    return float(top.size[1 - index]), float(top.size[index])


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


def choose_site(
    footprint: tuple[float, float],
    obstacles: list[Box],
    base: np.ndarray,
    *,
    radii: tuple[float, ...],
    preferred_radius: float,
    clearance: float = SITE_CLEARANCE,
) -> np.ndarray | None:
    """The centre of a patch of floor the table fits on, or ``None``.

    Candidate centres are tried on rings round the base at every 5 degrees.
    A candidate is kept only if nothing the camera saw is within ``clearance``
    of the table's footprint, and of those, the one straight in front of the
    arm at its most comfortable reach wins. That preference is the arm's, not
    knowledge of the room: every candidate still has to be proved empty.
    """
    samples = (
        np.concatenate([_footprint_samples(box) for box in obstacles]) if obstacles else np.zeros((0, 2))
    )
    length, depth = footprint

    best: tuple[float, float, np.ndarray] | None = None
    for radius in radii:
        for azimuth_deg in range(-180, 180, 5):
            azimuth = math.radians(azimuth_deg)
            centre = base[:2] + radius * np.array([math.cos(azimuth), math.sin(azimuth)])
            candidate = Box.upright(
                (centre[0], centre[1], 0.0), azimuth + math.pi / 2.0, (length, depth, 0.0)
            )
            if len(samples) and float(candidate.footprint_distance(samples).min()) < clearance:
                continue
            score = (abs(azimuth), abs(radius - preferred_radius))
            if best is None or score < best[:2]:
                best = (*score, centre)
    return None if best is None else best[2]


def _footprint_samples(box: Box, step: float = 0.02) -> np.ndarray:
    """Points spread through a box, seen from above.

    Sampling the whole volume rather than the corners means a box at any
    angle — the leaning top included — is covered by the same code.
    """
    counts = [max(2, math.ceil(side / step) + 1) for side in box.size]
    grids = np.meshgrid(*[np.linspace(-s / 2.0, s / 2.0, n) for s, n in zip(box.size, counts, strict=True)])
    local = np.stack([g.ravel() for g in grids], axis=1)
    return (box.centre + local @ box.rotation.T)[:, :2]
