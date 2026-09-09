"""The arm's own measurements.

These are the numbers the code needs that are not in the robot model: how far
the fingertips reach past the flange, where the camera sits, and how high above
the table the arm does each part of its job. They are here rather than read out
of the URDF because they describe how the arm is *used*, not how it is built.

Anything changed here has a matching number in arm.urdf.xacro or
gripper.urdf.xacro, and the two have to move together.
"""

from __future__ import annotations

import numpy as np

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
# direction the gripper reaches in. See arm/gripper.urdf.xacro.
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
