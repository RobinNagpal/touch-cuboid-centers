"""Small conversions between ROS poses and 4x4 matrices."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Pose, Quaternion

WORLD_X = np.array([1.0, 0.0, 0.0])
WORLD_Z = np.array([0.0, 0.0, 1.0])


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix for a unit quaternion."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(rotation: np.ndarray) -> Quaternion:
    """Unit quaternion for a rotation matrix, via the largest-diagonal branch.

    Picking the branch with the biggest denominator keeps the square root away
    from zero, which is what makes this numerically stable for any rotation.
    """
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(rotation[i, i] - rotation[j, j] - rotation[k, k] + 1.0) * 2.0
        components = [0.0, 0.0, 0.0]
        components[i] = 0.25 * s
        components[j] = (rotation[j, i] + rotation[i, j]) / s
        components[k] = (rotation[k, i] + rotation[i, k]) / s
        x, y, z = components
        w = (rotation[k, j] - rotation[j, k]) / s
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


def transform_to_matrix(transform) -> np.ndarray:
    """4x4 matrix for a geometry_msgs/Transform."""
    matrix = np.eye(4)
    rotation = transform.rotation
    matrix[:3, :3] = quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    matrix[:3, 3] = (transform.translation.x, transform.translation.y, transform.translation.z)
    return matrix


def make_pose(position: np.ndarray, rotation: np.ndarray) -> Pose:
    """geometry_msgs/Pose from a 3-vector and a 3x3 rotation."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    pose.orientation = matrix_to_quaternion(rotation)
    return pose


def frame(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """4x4 matrix from a position and a 3x3 rotation."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = position
    return matrix


def spin(rotation: np.ndarray, angle: float) -> np.ndarray:
    """The same tool pose, turned about the tool's own z axis."""
    c, s = math.cos(angle), math.sin(angle)
    return rotation @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def grasp_options(rotation: np.ndarray) -> list[np.ndarray]:
    """Both orientations that grip a box the same way.

    A parallel gripper is symmetric, so swapping which finger is on which side
    of the box makes no difference to the grip. If the arm cannot reach one of
    the two, it is worth asking for the other.
    """
    return [rotation, spin(rotation, math.pi)]


def view_options(rotation: np.ndarray, count: int = 6) -> list[np.ndarray]:
    """Every orientation that looks the same way.

    Rolling the camera about the direction it is looking turns the picture but
    does not change what is in it, because pixels are placed in the world using
    the pose the arm was really in. So the roll is free.
    """
    return _turns(rotation, count)


def release_options(rotation: np.ndarray, count: int = 8) -> list[np.ndarray]:
    """Every orientation that will set a held box down flat.

    Once the box is in the gripper, how far the wrist is turned about the
    vertical only decides which way round the box ends up on the table, and
    nothing cares about that: it gets measured again where it lands. So the
    whole circle is available, and offering all of it gives the planner room
    to find an arm configuration that reaches the far corners of the table.
    The orientation the box was picked up with is offered first, because using
    it means the wrist does not have to turn at all.
    """
    return _turns(rotation, count)


def _turns(rotation: np.ndarray, count: int) -> list[np.ndarray]:
    """``count`` turns about the tool z axis, smallest first.

    Smallest first because if the arm can do the job with a small twist of the
    wrist there is no reason to make a big one.
    """
    steps = sorted(range(count), key=lambda i: (abs(_wrap(2 * math.pi * i / count)), i))
    return [spin(rotation, 2 * math.pi * i / count) for i in steps]


def _wrap(angle: float) -> float:
    """An angle folded into -pi to pi, so that 350 degrees counts as -10."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def look_along(forward: np.ndarray, up_hint: np.ndarray | None = None) -> np.ndarray:
    """A tool orientation whose z axis points along ``forward``.

    The tool's z axis is the direction the gripper reaches in, so this is how a
    face normal turns into something the arm can be asked to hold. The spin
    about that axis is left free by the task, so it is pinned here.

    The hint is the world x axis rather than the world z axis on purpose. Most
    of the poses in this task look downwards, and a hint they are nearly
    parallel to has to be swapped for a different one, which makes two similar
    tool directions come out a quarter turn apart and the arm reconfigure its
    wrist between two viewpoints that are ten centimetres from each other.
    """
    z = np.asarray(forward, dtype=float)
    z = z / np.linalg.norm(z)
    up_hint = WORLD_X if up_hint is None else up_hint
    if abs(float(np.dot(z, up_hint))) > 0.95:
        up_hint = WORLD_Z
    x = np.cross(up_hint, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))
