import numpy as np
import pytest
from cuboid_cell.transforms import look_along, matrix_to_quaternion, quaternion_to_matrix


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
    q = matrix_to_quaternion(rotation)
    assert quaternion_to_matrix(q.x, q.y, q.z, q.w) == pytest.approx(rotation, abs=1e-9)


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


@pytest.mark.parametrize(
    "forward",
    [
        np.array([0.0, 0.0, -1.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.2, 0.3, -0.9]),
    ],
)
def test_look_along_returns_a_right_handed_frame(forward):
    rotation = look_along(forward)
    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_two_nearby_downward_directions_give_nearby_orientations():
    # Survey viewpoints sit a few centimetres apart and look almost straight
    # down. If the orientations they ask for jump, the wrist has to unwind
    # between two pictures of the same table.
    straight_down = look_along(np.array([0.0, 0.0, -1.0]))
    slightly_off = look_along(np.array([0.1, -0.08, -0.42]))
    assert straight_down[:, 0] @ slightly_off[:, 0] > 0.9
