"""What the arm knows about itself.

This is the only fixed knowledge the robot has. It knows where it is bolted
down and how its tooling is built. It does not know where the floor is, where
the stands are, or anything about the parts: those are measured with the
camera on every run.

Anything changed here has a matching number in arm.urdf.xacro or
gripper.urdf.xacro, and the two have to move together.
"""

from __future__ import annotations

import numpy as np

WORLD_FRAME = "world"

# Where the arm's base is bolted down. See arm.urdf.xacro.
BASE_POSITION = np.array([0.0, 0.0, 0.0])

# The arm's ready posture, joint by joint from the base: elbow up, forearm
# level and pointing out, tool pointing at the floor. The same as the "ready"
# state in the SRDF and the starting angles in arm.urdf.xacro.
READY_JOINTS = (0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)

# The arm's own footprint. Anything the camera sees inside this cylinder is the
# robot looking at itself, not something in the room.
SELF_RADIUS = 0.13
SELF_HEIGHT = 0.25

# The gripper. A part can only be picked up if the side the fingers close
# across fits between them with room to spare, because a part almost as wide
# as the gripper opens has to be approached with its yaw right to within a
# degree or two, or a corner catches a finger.
#
# The fingers travel 4 cm each, an 8 cm gap, but are never opened past 7.4:
# a finger sent right to its end stop has been seen to stay stuck there for
# good when told to close again, after the arm has been moving.
GRIPPER_MAX_OPENING = 0.074
MAX_GRASP_WIDTH = 0.065

# Where the camera sits relative to tool0, in the tool's own frame. It is off to
# one side so the fingers stay out of shot, which means pointing tool0 at
# something is not the same as pointing the camera at it.
CAMERA_OFFSET = np.array([0.085, 0.0, 0.015])

# Distances from tool0, measured along the tool's own z axis, which is the
# direction the gripper reaches in. See arm/gripper.urdf.xacro.
FINGERTIP_OFFSET = 0.170
GRASP_OFFSET = 0.110

# The survey: where the camera goes to look around the room before the arm
# knows what is in it. The camera is carried round the base on a circle and
# pointed down and outwards at the floor, so between them the views cover a
# ring of floor from about 0.35 m to 1.0 m out.
SURVEY_AZIMUTHS_DEG = (0, 45, 90, 135, 180, -135, -90, -45)
SURVEY_CAMERA_RADIUS = 0.30
SURVEY_CAMERA_HEIGHT = 0.60
SURVEY_LOOK_RADIUS = 0.65
