"""Numbers that describe the work cell.

The world file, the cuboid spawner and the task all have to agree on where the
table is and which half of it is which, so those numbers live here and nowhere
else.
"""

from __future__ import annotations

import numpy as np

WORLD_FRAME = "world"

# The table. Its top is the plane everything else is measured against.
TABLE_TOP_Z = 0.75
TABLE_SIZE = (1.4, 1.2, 0.05)
TABLE_CENTRE_XY = (0.35, 0.0)

# The arm stands on the table at the near edge and reaches out along +x.
ROBOT_BASE = np.array([0.0, 0.0, TABLE_TOP_Z])

# Cuboids start in the pending zone and are moved to the done zone once they
# have been measured, so the two halves record what is left to do. Both are
# (x_min, x_max, y_min, y_max) rectangles on the table top.
PENDING_ZONE = (0.32, 0.56, -0.42, -0.14)
DONE_ZONE = (0.32, 0.56, 0.14, 0.42)

# Cuboid sides are drawn from this range.
CUBOID_MIN_SIDE = 0.040
CUBOID_MAX_SIDE = 0.090

# The gripper. A box can only be moved if one of its two horizontal sides fits
# between the fingers, so the spawner refuses to place a box that does not.
# The margin between the two is deliberate: a box almost as wide as the gripper
# opens has to be approached with its yaw measured almost exactly right, and a
# degree or two of error puts a corner in the way of a finger.
GRIPPER_MAX_OPENING = 0.080
MAX_GRASP_WIDTH = 0.065

# Where the camera sits relative to tool0, in the tool's own frame. It is off to
# one side so the fingers stay out of shot, which means pointing tool0 at
# something is not the same as pointing the camera at it.
CAMERA_OFFSET = np.array([0.085, 0.0, 0.015])

# Distances from tool0, measured along the tool's own z axis, which is the
# direction the gripper reaches in. See cuboid_cell/urdf/gripper.urdf.xacro.
FINGERTIP_OFFSET = 0.170
GRASP_OFFSET = 0.110

# How much air is left under the fingertips when the arm closes on a box
# standing on the table. The fingers reach past the box, so without this they
# would be asked to close somewhere inside the table top.
FINGER_TABLE_CLEARANCE = 0.015

# Heights the arm works at, measured from the table top.
SURVEY_HEIGHT = 0.42
APPROACH_HEIGHT = 0.16
LIFT_HEIGHT = 0.15

# How far back from a face the arm lines up before moving in to touch it.
TOUCH_STANDOFF = 0.06


def zone_centre(zone: tuple[float, float, float, float]) -> np.ndarray:
    """Middle of a zone, on the table top."""
    x_min, x_max, y_min, y_max = zone
    return np.array([(x_min + x_max) / 2.0, (y_min + y_max) / 2.0, TABLE_TOP_Z])


def zone_slots(zone: tuple[float, float, float, float], count: int) -> list[np.ndarray]:
    """``count`` resting places spread over a zone.

    Up to three boxes go in a line down the middle of the zone. Beyond that a
    single line would put them closer together than a box is wide, so they go
    in two columns instead.
    """
    x_min, x_max, y_min, y_max = zone
    columns = 1 if count <= 3 else 2
    rows = -(-count // columns)
    return [
        np.array(
            [
                _spread(x_min, x_max, i % columns, columns),
                _spread(y_min, y_max, i // columns, rows),
                TABLE_TOP_Z,
            ]
        )
        for i in range(count)
    ]


def _spread(low: float, high: float, index: int, count: int) -> float:
    """Position ``index`` of ``count`` spread evenly between two bounds."""
    if count == 1:
        return (low + high) / 2.0
    return low + (high - low) * index / (count - 1)
