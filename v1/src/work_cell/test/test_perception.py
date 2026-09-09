"""Perception tests, run against synthetic point clouds.

No simulator is involved: a cuboid of a known size and yaw is turned into the
points a camera looking down on it would see, and the fit has to give the
numbers back.
"""

import math

import numpy as np
import pytest
from work_cell.cuboids.geometry import Cuboid
from work_cell.cuboids.perception import cluster, find_cuboids, fit_cuboid

TABLE_Z = 0.75


def visible_points(centre_xy, size, yaw, spacing=0.002):
    """Points on the top face and the two upright faces a camera would see.

    A camera above and to one side of a box sees its top and the two sides
    facing the camera. That is all this returns: no bottom, and no far sides.
    """
    length, width, height = size
    rotation = Cuboid(np.zeros(3), yaw, np.array(size)).rotation

    us = np.arange(-length / 2, length / 2 + spacing, spacing)
    vs = np.arange(-width / 2, width / 2 + spacing, spacing)
    ws = np.arange(0.0, height + spacing, spacing)

    local = []
    grid_u, grid_v = np.meshgrid(us, vs, indexing="ij")
    local.append(np.stack([grid_u.ravel(), grid_v.ravel(), np.full(grid_u.size, height)], axis=1))

    grid_u, grid_w = np.meshgrid(us, ws, indexing="ij")
    local.append(np.stack([grid_u.ravel(), np.full(grid_u.size, width / 2), grid_w.ravel()], axis=1))

    grid_v, grid_w = np.meshgrid(vs, ws, indexing="ij")
    local.append(np.stack([np.full(grid_v.size, length / 2), grid_v.ravel(), grid_w.ravel()], axis=1))

    points = np.concatenate(local) @ rotation.T
    points += np.array([centre_xy[0], centre_xy[1], TABLE_Z])
    return points


@pytest.mark.parametrize("yaw", [0.0, 0.3, -0.7, math.pi / 4, 1.4, -1.5])
@pytest.mark.parametrize("size", [(0.08, 0.05, 0.03), (0.09, 0.04, 0.07), (0.05, 0.05, 0.05)])
def test_the_fit_recovers_the_size_it_was_given(yaw, size):
    fitted = fit_cuboid(visible_points((0.42, -0.25), size, yaw), TABLE_Z)
    assert fitted.size == pytest.approx(np.array(size), abs=0.002)


def test_the_fit_recovers_the_centre_it_was_given():
    fitted = fit_cuboid(visible_points((0.42, -0.25), (0.08, 0.05, 0.03), 0.4), TABLE_Z)
    assert fitted.centre == pytest.approx([0.42, -0.25, TABLE_Z + 0.015], abs=0.002)


@pytest.mark.parametrize("yaw", [0.0, 0.3, -0.7, 1.2])
def test_the_fit_recovers_the_yaw_it_was_given(yaw):
    size = (0.08, 0.05, 0.03)
    fitted = fit_cuboid(visible_points((0.42, -0.25), size, yaw), TABLE_Z)
    # A box looks the same after half a turn, so compare the axes, not the angle.
    expected = Cuboid(np.zeros(3), yaw, np.array(size)).rotation[:, 0]
    assert abs(float(np.dot(fitted.rotation[:, 0], expected))) == pytest.approx(1.0, abs=1e-3)


def test_length_is_always_the_longer_horizontal_side():
    for yaw in (0.0, 0.9, -1.2):
        fitted = fit_cuboid(visible_points((0.4, 0.0), (0.04, 0.09, 0.05), yaw), TABLE_Z)
        assert fitted.size[0] >= fitted.size[1]


def test_boxes_set_apart_come_back_as_separate_clusters():
    cloud = np.concatenate(
        [
            visible_points((0.36, -0.34), (0.08, 0.05, 0.03), 0.2),
            visible_points((0.50, -0.18), (0.06, 0.06, 0.06), -0.5),
        ]
    )
    assert len(cluster(cloud)) == 2


def test_two_boxes_are_measured_independently():
    cloud = np.concatenate(
        [
            visible_points((0.36, -0.34), (0.08, 0.05, 0.03), 0.2),
            visible_points((0.50, -0.18), (0.06, 0.06, 0.06), -0.5),
        ]
    )
    sizes = sorted(tuple(np.round(box.size, 3)) for box in find_cuboids(cloud, TABLE_Z))
    assert sizes[0] == pytest.approx([0.06, 0.06, 0.06], abs=0.002)
    assert sizes[1] == pytest.approx([0.08, 0.05, 0.03], abs=0.002)


def test_a_cloud_with_nothing_above_the_table_fits_nothing():
    flat = np.random.default_rng(0).uniform(-0.2, 0.2, size=(500, 3))
    flat[:, 2] = TABLE_Z
    assert fit_cuboid(flat, TABLE_Z) is None
