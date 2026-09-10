"""The ranges the simulated room is drawn from.

This is the simulator's side of the project, and nothing the robot runs may
import it. Every size and position here is what the robot has to *find out*;
if the task code could read these numbers, measuring them would be theatre.
A test enforces that boundary.

Every run draws a new room from these ranges, so the robot never sees the
same table twice. A seed makes any one room repeatable.
"""

from __future__ import annotations

# The wall the table top leans on. A low wall, deliberately: the top is
# picked up by its highest edge with a finger either side of it, and against a
# full-height wall there would be no room for the finger on the wall's side.
# Distances and angles are measured from the arm's base.
WALL_AZIMUTH_DEG = (65.0, 115.0)
WALL_DISTANCE = (0.46, 0.52)  # to the wall's front face
WALL_LENGTH = 0.45
WALL_THICKNESS = 0.10
WALL_HEIGHT = (0.06, 0.08)

# The table top. It stands on its long edge on the floor and leans back
# against the wall.
TOP_LENGTH = (0.24, 0.32)
TOP_WIDTH = (0.16, 0.20)
TOP_THICKNESS = (0.016, 0.020)
TOP_LEAN_DEG = (15.0, 22.0)
# How far along the wall from its middle the top may stand.
TOP_SLIDE = 0.05
# At least this much of the top, measured up its face, has to stand clear
# above the wall, so the fingers can close on the edge without touching it.
TOP_FREE_EDGE = 0.09
# A light board: poplar plywood is about this. The arm carries it by one edge,
# held level, so every gram of it is a lever on the grip.
TOP_DENSITY = 400.0  # kg/m^3

# The legs. All four are the same, as they would be in a flat-pack table.
# The arm stands a leg up holding it round the middle with the gripper level,
# so half a leg's length is how high the wrist is off the floor at that
# moment; much shorter than this and the wrist would scrape the floor.
LEG_COUNT = 4
LEG_LENGTH = (0.13, 0.16)
LEG_THICKNESS = (0.025, 0.035)
LEG_AZIMUTH_DEG = (-135.0, -45.0)
LEG_DISTANCE = (0.40, 0.62)
# The closest two legs may lie to each other. The robot tells parts apart by
# the gaps between them, so two legs touching would be seen as one part.
LEG_GAP = 0.04
LEG_DENSITY = 700.0  # kg/m^3, about oak

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
WALL_COLOUR = (0.62, 0.62, 0.64)
