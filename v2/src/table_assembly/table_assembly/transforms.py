"""Rotations and rigid transforms, as plain numpy.

No ROS in here, so the perception and planning code that leans on it can be
tested without a simulator. Turning these into ROS messages is messages.py's
job.
"""

from __future__ import annotations

import math

import numpy as np

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


def matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Unit quaternion (x, y, z, w) for a rotation matrix.

    Uses the largest-diagonal branch. Picking the branch with the biggest
    denominator keeps the square root away from zero, which is what makes this
    numerically stable for any rotation.
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
    return float(x), float(y), float(z), float(w)


def rpy_from_matrix(rotation: np.ndarray) -> tuple[float, float, float]:
    """Roll, pitch and yaw, in the fixed-axis order SDF and URDF use."""
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return roll, pitch, yaw


def rotation_z(angle: float) -> np.ndarray:
    """A turn about the world's vertical axis."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_about(axis: np.ndarray, angle: float) -> np.ndarray:
    """A turn of ``angle`` about ``axis``, by Rodrigues' formula."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    cross = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)


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


def view_options(rotation: np.ndarray, count: int = 6) -> list[np.ndarray]:
    """Every orientation that looks the same way, smallest wrist turn first.

    Rolling the camera about the direction it is looking turns the picture but
    does not change what is in it, because pixels are placed in the world using
    the pose the arm was really in. So the roll is free, and offering several
    gives the planner room to reach awkward viewpoints.
    """
    steps = sorted(range(count), key=lambda i: (abs(_wrap(2 * math.pi * i / count)), i))
    return [spin(rotation, 2 * math.pi * i / count) for i in steps]


def _wrap(angle: float) -> float:
    """An angle folded into -pi to pi, so that 350 degrees counts as -10."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def look_along(forward: np.ndarray, up_hint: np.ndarray | None = None) -> np.ndarray:
    """A tool orientation whose z axis points along ``forward``.

    The tool's z axis is the direction the gripper reaches in, and the camera
    looks the same way. The spin about that axis is left free by the caller,
    so it is pinned here.

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
