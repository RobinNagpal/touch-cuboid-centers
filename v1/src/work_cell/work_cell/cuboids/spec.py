"""What a cuboid is.

The boxes are drawn fresh for every run, so nothing about any one of them is
fixed. What is fixed is the range they are drawn from, and that is here.
"""

from __future__ import annotations

# Cuboid sides are drawn from this range.
CUBOID_MIN_SIDE = 0.040
CUBOID_MAX_SIDE = 0.090

# About right for a block of oak. Heavy enough that touching a box does not
# send it skating, light enough that the gripper is never straining.
DENSITY = 700.0  # kg/m^3

# Centre to centre, so that two boxes never end up touching. The measuring step
# groups points that are near each other, so two boxes in contact would be
# measured as one large box.
MIN_SEPARATION = 0.13

# Saturated, well separated colours. The perception step tells a cuboid from
# the table by how colourful it is, so nothing here may be grey.
PALETTE = (
    (0.80, 0.12, 0.12),  # red
    (0.10, 0.65, 0.20),  # green
    (0.12, 0.28, 0.80),  # blue
    (0.85, 0.72, 0.08),  # yellow
    (0.72, 0.12, 0.66),  # magenta
    (0.06, 0.62, 0.70),  # cyan
)
