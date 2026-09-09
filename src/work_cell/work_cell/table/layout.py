"""Where the table is, and which half of it is which.

The table top is the plane everything in the cell is measured against: the arm
stands on it, the cuboids rest on it, and every height elsewhere in the code is
written as a distance from it. That is why these numbers live in one place. The
world file, the cuboid spawner and the task all have to agree on them, and the
cheapest way to guarantee that is to give them one source.
"""

from __future__ import annotations

import numpy as np

WORLD_FRAME = "world"

# The table itself. Its top is at 75 cm, a normal bench height.
TABLE_TOP_Z = 0.75
TABLE_SIZE = (1.4, 1.2, 0.05)
TABLE_CENTRE_XY = (0.35, 0.0)

# The arm stands on the table at the near edge and reaches out along +x.
ROBOT_BASE = np.array([0.0, 0.0, TABLE_TOP_Z])

# The two halves of the table. Cuboids start in the pending zone and are moved
# to the done zone once they have been measured, so between them the two halves
# record what is left to do without anything having to remember it. Both are
# (x_min, x_max, y_min, y_max) rectangles on the table top.
PENDING_ZONE = (0.32, 0.56, -0.42, -0.14)
DONE_ZONE = (0.32, 0.56, 0.14, 0.42)


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
