"""Where the tool has to be to pick each part up and to put it down.

A part in the gripper is rigidly attached to the tool, so once it has been
picked up, the part and the tool move as one body. That is captured by the
*hold*: the tool's pose written in the part's own frame, worked out at the
moment of the grasp. Putting the part down anywhere is then one line — the
tool goes to the part's new pose times the hold — whatever the part is and
however it was picked up. Standing a leg on its spot and laying the top on the
legs are both just a new part pose.

All poses are 4x4 matrices. Plain numpy, no ROS.
"""

from __future__ import annotations

import math

import numpy as np

from ..arm.dimensions import FINGERTIP_OFFSET
from ..geometry import Box
from ..transforms import WORLD_Z, frame, rotation_z

# How far down a standing leg the fingertips reach. Deep enough that both rows
# of pads on each finger are on the leg, and the leg hangs straight down from
# them, so its weight pulls along the fingers and never tries to turn it.
LEG_GRIP_DEPTH = 0.045

# How deep the fingers reach over the top's edge. Deep enough that both rows of
# pads are on the board, because it is the spread between the rows that keeps
# a board held by one edge from tipping out of the fingers.
TOP_INSERTION = 0.050


def hold(part: Box, tool_pose: np.ndarray) -> np.ndarray:
    """The tool's pose in the part's own frame."""
    return np.linalg.inv(part.pose) @ tool_pose


def carried_tool_pose(held: np.ndarray, tool_rotation: np.ndarray, part_centre: np.ndarray) -> np.ndarray:
    """Where tool0 must be for a held part to sit at ``part_centre``.

    The tool's orientation is chosen by the caller; the part's orientation
    follows from it through the hold.
    """
    part_rotation = tool_rotation @ held[:3, :3].T
    return frame(part_centre, part_rotation) @ held


def leg_pick_poses(leg: Box) -> list[np.ndarray]:
    """Tool poses that grip a standing leg by its top end, from straight above.

    The fingers close across the leg, square to one of its sides. A square
    leg can be gripped across either pair of sides, with the tool turned
    either way round, so all four are offered.
    """
    down = -WORLD_Z
    position = np.array([leg.centre[0], leg.centre[1], leg.top_z - LEG_GRIP_DEPTH + FINGERTIP_OFFSET])
    poses = []
    for quarter in range(4):
        pinch = rotation_z(quarter * math.pi / 2.0) @ leg.axis(0)
        pinch = np.array([pinch[0], pinch[1], 0.0]) / np.linalg.norm(pinch[:2])
        poses.append(frame(position, np.column_stack((np.cross(pinch, down), pinch, down))))
    return poses


def leg_place_pose(
    held: np.ndarray, pick: np.ndarray, spot: np.ndarray, length: float, drop: float, base: np.ndarray
) -> np.ndarray:
    """The tool pose that stands a leg, held from above, upright on ``spot``.

    The tool still points straight down, turned about the vertical by as much
    as the arm's base turns between the leg and its spot. The wrist then ends
    up the same way round, seen from the arm, as when it picked the leg up.
    """
    turn = _azimuth(spot, base) - _azimuth(pick[:3, 3], base)
    return carried_tool_pose(held, rotation_z(turn) @ pick[:3, :3], spot + WORLD_Z * (length / 2.0 + drop))


def top_pick_poses(top: Box, base: np.ndarray) -> list[np.ndarray]:
    """Tool poses that grip a level top by the middle of its long edge nearest the arm.

    The tool is level and reaches in across that edge, away from the arm, with
    one finger above the board and one below it. Which finger is on top makes
    no difference to the grip, so both ways are offered.
    """
    across = np.array([top.axis(1)[0], top.axis(1)[1], 0.0])
    across /= np.linalg.norm(across)
    if float(across @ (top.centre - base)[:3]) < 0.0:
        across = -across
    edge = top.centre - across * float(top.size[1]) / 2.0
    position = edge - across * (FINGERTIP_OFFSET - TOP_INSERTION)
    return [level_tool_pose(position, across, pinch) for pinch in (WORLD_Z, -WORLD_Z)]


def level_tool_pose(position: np.ndarray, reach: np.ndarray, pinch: np.ndarray) -> np.ndarray:
    """A level tool at ``position``, reaching along ``reach``, its fingers closing along ``pinch``."""
    return frame(position, np.column_stack((np.cross(pinch, reach), pinch, reach)))


def level_top_pose(
    held: np.ndarray, centre: np.ndarray, outward: np.ndarray, pinch: np.ndarray
) -> np.ndarray:
    """The tool pose that holds the top level with its centre at ``centre``.

    The tool is level and reaches away from the arm, into the board, from its
    near edge. ``pinch`` is the way the fingers closed when it was picked up;
    keeping it means the board is carried round the right way up, rather than
    being flipped over on the way.
    """
    rotation = np.column_stack((np.cross(pinch, outward), pinch, outward))
    return carried_tool_pose(held, rotation, centre)


def carry_round(
    held: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
    height: float,
    base: np.ndarray,
    step: float = math.radians(5.0),
) -> list[np.ndarray]:
    """Tool poses that carry a held part from ``start`` round the base to above ``end``.

    ``start`` and ``end`` are tool poses that differ only by a turn about the
    vertical — a leg held upright from above, or a level top. The part goes up to
    ``height`` first, then round the arm's base on an arc at that height:
    distance from the base and bearing both change evenly, so the path never
    cuts in close to the base. On the way the tool turns evenly from the way
    it pointed at the start to the way it has to point at the end. None of
    that tips the part: it only ever turns about the vertical.
    """

    def polar(point):
        offset = point[:2] - base[:2]
        return float(np.linalg.norm(offset)), math.atan2(offset[1], offset[0])

    def wrap(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    (r0, a0) = polar((start @ np.linalg.inv(held))[:3, 3])
    (r1, a1) = polar((end @ np.linalg.inv(held))[:3, 3])
    turn = wrap(a1 - a0)
    relative = end[:3, :3] @ start[:3, :3].T
    spin = math.atan2(relative[1, 0], relative[0, 0])
    steps = max(1, math.ceil(max(abs(turn), abs(spin)) / step))

    poses = []
    for fraction in np.linspace(0.0, 1.0, steps + 1):
        radius, azimuth = r0 + (r1 - r0) * fraction, a0 + turn * fraction
        centre = np.array(
            [base[0] + radius * math.cos(azimuth), base[1] + radius * math.sin(azimuth), height]
        )
        poses.append(carried_tool_pose(held, rotation_z(spin * fraction) @ start[:3, :3], centre))
    return poses


def _azimuth(point: np.ndarray, base: np.ndarray) -> float:
    return math.atan2(point[1] - base[1], point[0] - base[0])


def shifted(pose: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """The same pose moved by ``offset``, in world coordinates."""
    moved = pose.copy()
    moved[:3, 3] = pose[:3, 3] + offset
    return moved


def backed_off(pose: np.ndarray, distance: float) -> np.ndarray:
    """The same pose moved back along the tool's own reach direction."""
    return shifted(pose, -pose[:3, 2] * distance)
