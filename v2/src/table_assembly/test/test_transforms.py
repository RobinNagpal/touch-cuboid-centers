import math

import numpy as np
import pytest
from table_assembly.transforms import (
    look_along,
    matrix_to_quaternion,
    quaternion_to_matrix,
    rotation_z,
    rpy_from_matrix,
)


def rotations():
    """A spread of rotations, including the ones with awkward diagonals."""
    yield np.eye(3)
    yield np.diag([1.0, -1.0, -1.0])
    yield np.diag([-1.0, 1.0, -1.0])
    yield np.diag([-1.0, -1.0, 1.0])
    yield look_along(np.array([0.0, 0.0, -1.0]))
    yield look_along(np.array([0.3, -0.4, -0.9]))
    yield look_along(np.array([1.0, 0.0, 0.0]))


@pytest.mark.parametrize("rotation", list(rotations()))
def test_a_rotation_survives_the_round_trip_through_a_quaternion(rotation):
    assert quaternion_to_matrix(*matrix_to_quaternion(rotation)) == pytest.approx(rotation, abs=1e-9)


@pytest.mark.parametrize("rotation", list(rotations()))
def test_roll_pitch_yaw_rebuild_the_rotation(rotation):
    roll, pitch, yaw = rpy_from_matrix(rotation)
    c, s = math.cos, math.sin
    rx = np.array([[1, 0, 0], [0, c(roll), -s(roll)], [0, s(roll), c(roll)]])
    ry = np.array([[c(pitch), 0, s(pitch)], [0, 1, 0], [-s(pitch), 0, c(pitch)]])
    assert rotation_z(yaw) @ ry @ rx == pytest.approx(rotation, abs=1e-9)


@pytest.mark.parametrize(
    "forward",
    [
        np.array([0.0, 0.0, -1.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.2, 0.3, -0.9]),
        np.array([-0.6, 0.1, -0.8]),
    ],
)
def test_look_along_points_the_tool_z_axis_where_it_was_told(forward):
    rotation = look_along(forward)
    assert rotation[:, 2] == pytest.approx(forward / np.linalg.norm(forward))
    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_two_nearby_downward_directions_give_nearby_orientations():
    # Viewpoints sit a few centimetres apart and look almost straight down. If
    # the orientations they ask for jump, the wrist has to unwind between two
    # pictures of the same thing.
    straight_down = look_along(np.array([0.0, 0.0, -1.0]))
    slightly_off = look_along(np.array([0.1, -0.08, -0.42]))
    assert straight_down[:, 0] @ slightly_off[:, 0] > 0.9
