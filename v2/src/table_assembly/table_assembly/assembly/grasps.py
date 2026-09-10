"""Where the tool has to be to pick each part up and to put it down.

A part in the gripper is rigidly attached to the tool, so once it has been
picked up, the part and the tool move as one body. That is captured by the
*hold*: the tool's pose written in the part's own frame, worked out at the
moment of the grasp. Putting the part down anywhere is then one line — the
tool goes to the part's new pose times the hold — whatever the part is and
however it was picked up. Standing a leg up and laying the top flat are both
just a new part pose.

All poses are 4x4 matrices. Plain numpy, no ROS.
"""

from __future__ import annotations

import math

import numpy as np

from ..arm.dimensions import FINGERTIP_OFFSET
from ..geometry import Box
from ..transforms import WORLD_Z, frame, rotation_about, spin
from .plan import gripped_axis

# How far above the floor the fingertips stop when closing round a part lying
# on it. The fingers reach past the part, so without this they would be asked
# to close somewhere inside the floor.
FINGER_FLOOR_CLEARANCE = 0.005

# How deep the fingers reach over the top's edge. Deeper grips the board more
# firmly against tipping once it is level, but the fingers have to stay above
# whatever the board is leaning on.
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


def leg_pick_poses(leg: Box, floor_z: float) -> list[np.ndarray]:
    """Tool poses that grip a lying leg round its middle, from straight above.

    The middle, because that is where its weight is. Gripped anywhere else,
    the leg's weight twists it about the axis the fingers press along, and
    the only thing resisting that twist is friction spinning in the contact
    patch. In the simulator that is next to nothing, and a leg gripped a few
    centimetres off centre swings round in the fingers as soon as it is lifted.

    The fingers close across the leg, and the tool's x axis lies along it.
    Either way along works, so both are offered.
    """
    down = -WORLD_Z
    tool_z = floor_z + FINGER_FLOOR_CLEARANCE + FINGERTIP_OFFSET
    poses = []
    for end in (1.0, -1.0):
        along = leg.axis(0) * end
        rotation = np.column_stack((along, np.cross(down, along), down))
        poses.append(frame(np.array([leg.centre[0], leg.centre[1], tool_z]), rotation))
    return poses


def turned_upright(
    held: np.ndarray, tool_pose: np.ndarray, reach: np.ndarray | None = None, steps: int = 8
) -> list[np.ndarray]:
    """Tool poses that turn a lying leg upright where it is, about its own middle.

    The leg turns about its own centre, so it neither swings out sideways nor
    drops towards the floor, and it ends upright with the tool level and its
    x axis — the camera side — pointing up. The last pose is the leg upright.

    ``reach`` is which way the level tool should point at the end. Left out,
    the leg only tips up, and the tool ends pointing along where the leg lay.
    That can be a bad way for the tool to point: level and square across the
    arm's reach is where the arm's wrist runs out of ways to move, and a
    straight line into that pose stops just short of it. Asking for the tool to
    end pointing along the arm's reach instead swivels the leg as it tips.

    Doing the turn here, on its own and slowly, rather than as part of the
    carry, is what keeps the leg square in the fingers: a turn folded into one
    long planned move can be fast and far, and flings a part held only by
    friction.
    """
    part = tool_pose @ np.linalg.inv(held)
    if reach is None:
        pinch = tool_pose[:3, 1]
        sign = 1.0 if (rotation_about(pinch, math.pi / 2.0) @ tool_pose[:3, :3])[2, 0] > 0.0 else -1.0
        final_tool = rotation_about(pinch, sign * math.pi / 2.0) @ tool_pose[:3, :3]
    else:
        level = np.array([reach[0], reach[1], 0.0]) / np.linalg.norm(reach[:2])
        final_tool = np.column_stack((WORLD_Z, np.cross(level, WORLD_Z), level))

    # The whole turn is one rotation of the leg about its centre: from where it
    # is to where it ends, walked in equal steps about that rotation's axis.
    start = part[:3, :3]
    end = final_tool @ held[:3, :3].T
    relative = end @ start.T
    angle = math.acos(max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0)))
    if angle < 1e-9:
        return [tool_pose]
    axis = np.array(
        [relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0], relative[1, 0] - relative[0, 1]]
    )
    if np.linalg.norm(axis) < 1e-9:
        # Half a turn: the axis is the eigenvector of the rotation.
        values, vectors = np.linalg.eig(relative)
        axis = np.real(vectors[:, int(np.argmin(np.abs(values - 1.0)))])
    poses = []
    for fraction in np.linspace(0.0, 1.0, steps + 1)[1:]:
        rotation = rotation_about(axis, fraction * angle) @ start
        poses.append(frame(part[:3, 3], rotation) @ held)
    return poses


def upright_tool_pose(held: np.ndarray, centre: np.ndarray, reach: np.ndarray) -> np.ndarray:
    """The tool pose that holds a leg upright, centred at ``centre``.

    The tool is level, reaching along ``reach``, with its x axis — the camera
    side — straight up.
    """
    level = np.array([reach[0], reach[1], 0.0]) / np.linalg.norm(reach[:2])
    return carried_tool_pose(held, np.column_stack((WORLD_Z, np.cross(level, WORLD_Z), level)), centre)


def carry_upright(
    held: np.ndarray,
    start: np.ndarray,
    reach: np.ndarray,
    end: np.ndarray,
    height: float,
    base: np.ndarray,
    step: float = math.radians(5.0),
) -> list[np.ndarray]:
    """Tool poses that carry an upright leg from ``start`` to above ``end``.

    Up to ``height`` first, then round the arm's base on an arc at that
    height: distance from the base and bearing both change evenly, so the
    path never cuts across close to the base. On the way the tool swings
    round, from reaching along ``reach`` to reaching straight out from the
    base, which is how the leg is stood on its spot. All of that only turns
    the leg about its own upright axis: it stays upright the whole way, and
    with its weight along its own length there is nothing to twist it in the
    fingers, however far off its middle it was gripped.
    """

    def polar(point):
        offset = point[:2] - base[:2]
        return float(np.linalg.norm(offset)), math.atan2(offset[1], offset[0])

    def wrap(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    (r0, a0), (r1, a1) = polar(start), polar(end)
    turn = wrap(a1 - a0)
    bearing = wrap(math.atan2(reach[1], reach[0]) - a0)
    steps = max(1, math.ceil(max(abs(turn), abs(bearing)) / step))

    def at(fraction):
        radius, azimuth = r0 + (r1 - r0) * fraction, a0 + turn * fraction
        centre = np.array(
            [base[0] + radius * math.cos(azimuth), base[1] + radius * math.sin(azimuth), height]
        )
        heading = azimuth + bearing * (1.0 - fraction)
        return upright_tool_pose(held, centre, np.array([math.cos(heading), math.sin(heading), 0.0]))

    return [at(fraction) for fraction in np.linspace(0.0, 1.0, steps + 1)]


def standing_leg_poses(held: np.ndarray, spot: np.ndarray, length: float, drop: float, outward: np.ndarray):
    """Tool poses that stand a held leg upright on ``spot``, best first.

    With the leg upright the tool is level, halfway up the leg, with its x
    axis pointing straight up so the camera is above the gripper rather than
    below it near the floor. It can come in from any side. Coming in along ``outward``, from the
    arm's side, is tried first, then directions further and further round.
    """
    centre = spot + WORLD_Z * (length / 2.0 + drop)
    heading = math.atan2(outward[1], outward[0])
    offsets = sorted(range(-150, 181, 30), key=abs)
    poses = []
    for offset in offsets:
        angle = heading + math.radians(offset)
        reach = np.array([math.cos(angle), math.sin(angle), 0.0])
        rotation = np.column_stack((WORLD_Z, np.cross(reach, WORLD_Z), reach))
        poses.append(carried_tool_pose(held, rotation, centre))
    return poses


def top_pick_poses(top: Box) -> list[np.ndarray]:
    """Tool poses that grip the top by the middle of its highest edge.

    The tool reaches down the face of the board, with the fingers either side
    of it, so the board's weight hangs straight below the grip.
    """
    index, sign = gripped_axis(top)
    up = top.axis(index) * sign
    normal = top.axis(2)
    edge = top.centre + up * float(top.size[index]) / 2.0
    tool_z = -up
    rotation = np.column_stack((np.cross(normal, tool_z), normal, tool_z))
    position = edge - tool_z * (FINGERTIP_OFFSET - TOP_INSERTION)
    return [frame(position, rotation), frame(position, spin(rotation, math.pi))]


def level_top_poses(held: np.ndarray, centre: np.ndarray, outward: np.ndarray) -> list[np.ndarray]:
    """Tool poses that hold the top level with its centre at ``centre``.

    The tool is level and reaches away from the arm, into the board. The
    fingers close across the board's thickness, so they are one above the
    other; which of the two is on top makes no difference to the table, so
    both are offered.
    """
    poses = []
    for pinch in (WORLD_Z, -WORLD_Z):
        rotation = np.column_stack((np.cross(pinch, outward), pinch, outward))
        poses.append(carried_tool_pose(held, rotation, centre))
    return poses


def shifted(pose: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """The same pose moved by ``offset``, in world coordinates."""
    moved = pose.copy()
    moved[:3, 3] = pose[:3, 3] + offset
    return moved


def backed_off(pose: np.ndarray, distance: float) -> np.ndarray:
    """The same pose moved back along the tool's own reach direction."""
    return shifted(pose, -pose[:3, 2] * distance)
