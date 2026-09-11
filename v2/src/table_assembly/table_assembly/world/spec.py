"""The ranges the simulated room is drawn from.

This is the simulator's side of the project, and nothing the robot runs may
import it. Every size and position here is what the robot has to *find out*;
if the task code could read these numbers, measuring them would be theatre.
A test enforces that boundary.

Every run draws a new room from these ranges, so the robot never sees the
same table twice. A seed makes any one room repeatable.
"""

from __future__ import annotations

# The table top. It lies flat on two stands to the arm's left, its long side
# across the arm's line of sight, give or take a few degrees. Distances and
# angles are measured from the arm's base to the top's centre.
TOP_AZIMUTH_DEG = (80.0, 100.0)
TOP_DISTANCE = (0.55, 0.60)
TOP_TURN_DEG = (-8.0, 8.0)
TOP_LENGTH = (0.24, 0.30)
TOP_WIDTH = (0.16, 0.20)
TOP_THICKNESS = (0.016, 0.020)
# A light board: poplar plywood is about this. The arm carries it by one edge,
# held level, so every gram of it is a lever on the grip.
TOP_DENSITY = 400.0  # kg/m^3

# The two stands, one under each end of the top. The arm grips the top by the
# middle of its near edge with one finger under the board, so the middle has
# to be open underneath. They are high enough that the gripper, held level at
# the board with its fingers one above the other, stays clear of the floor.
STAND_HEIGHT = (0.08, 0.09)
STAND_WIDTH = 0.04  # along the top's length
# How far the stands stop short of the top's ends and edges.
STAND_INSET = 0.01

# The legs. All four are the same, as they would be in a flat-pack table, and
# they start standing on end, to the arm's right. The arm picks each one up by
# its top end from straight above.
LEG_COUNT = 4
LEG_LENGTH = (0.13, 0.16)
LEG_THICKNESS = (0.025, 0.035)
LEG_AZIMUTH_DEG = (-130.0, -60.0)
LEG_DISTANCE = (0.40, 0.60)
# The closest two legs may stand to each other, centre to centre. The robot
# tells parts apart by the gaps between them, and the open fingers reach about
# 5 cm to either side of the leg they are closing on, so the next leg has to
# be further off than that.
LEG_SPACING = 0.12
LEG_DENSITY = 500.0  # kg/m^3, about pine

# Saturated, well separated colours. The robot tells parts from the room by how
# colourful they are, so nothing here may be grey.
PALETTE = (
    (0.80, 0.12, 0.12),  # red
    (0.10, 0.65, 0.20),  # green
    (0.12, 0.28, 0.80),  # blue
    (0.85, 0.72, 0.08),  # yellow
    (0.72, 0.12, 0.66),  # magenta
    (0.06, 0.62, 0.70),  # cyan
)

# The room is grey on purpose, for the same reason.
STAND_COLOUR = (0.62, 0.62, 0.64)
