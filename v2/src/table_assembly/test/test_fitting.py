"""The fitting functions, given clouds of boxes whose true size is known."""

import math

import numpy as np
import pytest
from synthetic import floor_points, leaning_board, visible_surface
from table_assembly.geometry import Box
from table_assembly.perception.fitting import cluster, fit_plate, fit_resting_box, floor_height, surface_tilt

ABOVE = np.array([0.0, 0.0, 0.8])


def test_the_floor_is_the_height_most_points_share():
    cloud = np.concatenate(
        [floor_points(z=0.012), visible_surface(Box.upright((0.3, 0, 0.1), 0, (0.2, 0.2, 0.2)), ABOVE)]
    )
    assert floor_height(cloud) == pytest.approx(0.012, abs=0.001)


@pytest.mark.parametrize("yaw", [0.0, 0.4, -0.9, math.pi / 4, 1.5])
@pytest.mark.parametrize("size", [(0.12, 0.03, 0.03), (0.14, 0.025, 0.025), (0.05, 0.05, 0.05)])
def test_a_box_lying_on_the_floor_is_measured(yaw, size):
    box = Box.upright((0.45, -0.3, size[2] / 2), yaw, size)
    fitted = fit_resting_box(visible_surface(box, ABOVE + np.array([0.2, -0.2, 0.0])), 0.0)
    assert np.sort(fitted.size[:2]) == pytest.approx(np.sort(size[:2]), abs=0.002)
    assert fitted.size[2] == pytest.approx(size[2], abs=0.002)
    assert fitted.centre[:2] == pytest.approx([0.45, -0.3], abs=0.002)


def test_a_standing_leg_is_measured_tall():
    leg = Box.upright((0.5, 0.0, 0.06), 0.3, (0.03, 0.03, 0.12))
    fitted = fit_resting_box(visible_surface(leg, np.array([0.2, 0.0, 0.5])), 0.0)
    assert fitted.size[2] == pytest.approx(0.12, abs=0.002)
    assert fitted.size[:2] == pytest.approx([0.03, 0.03], abs=0.002)


@pytest.mark.parametrize("lean_deg", [12.0, 18.0, 25.0])
@pytest.mark.parametrize("size", [(0.30, 0.18, 0.018), (0.24, 0.16, 0.016)])
def test_a_leaning_board_is_measured(lean_deg, size):
    board = leaning_board(*size, math.radians(lean_deg))
    # Seen from in front and above, as the arm does: the front face and the
    # top edge, nothing of the back.
    camera = board.centre - board.axis(2) * 0.3 + np.array([0.0, 0.0, 0.35])
    fitted = fit_plate(visible_surface(board, camera))
    assert fitted.size == pytest.approx(np.array(size), abs=0.0015)
    assert fitted.centre == pytest.approx(board.centre, abs=0.0015)
    # The fitted normal points out of the front face, which leans upwards.
    assert fitted.axis(2)[2] > 0
    assert abs(float(fitted.axis(2) @ board.axis(2))) == pytest.approx(1.0, abs=1e-4)


def test_a_level_board_can_be_fitted_too():
    board = Box.upright((0.5, 0.0, 0.14), 0.2, (0.28, 0.18, 0.018))
    fitted = fit_plate(visible_surface(board, np.array([0.3, 0.0, 0.6])))
    assert fitted.size[:2] == pytest.approx([0.28, 0.18], abs=0.002)
    assert fitted.axis(2)[2] == pytest.approx(1.0, abs=1e-3)


def test_separate_objects_come_back_as_separate_clusters():
    cloud = np.concatenate(
        [
            visible_surface(Box.upright((0.4, -0.3, 0.015), 0.2, (0.12, 0.03, 0.03)), ABOVE),
            visible_surface(Box.upright((0.5, -0.1, 0.015), -0.7, (0.12, 0.03, 0.03)), ABOVE),
        ]
    )
    assert len(cluster(cloud)) == 2


def test_a_level_surface_has_no_tilt_and_a_sloping_one_does():
    flat = floor_points(radius=0.1, spacing=0.005, z=0.2)
    assert surface_tilt(flat) == pytest.approx(0.0, abs=1e-6)
    sloped = flat.copy()
    sloped[:, 2] += sloped[:, 0] * math.tan(math.radians(4.0))
    assert math.degrees(surface_tilt(sloped)) == pytest.approx(4.0, abs=0.01)
