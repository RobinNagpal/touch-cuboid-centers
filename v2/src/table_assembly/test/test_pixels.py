"""The pixel masks, on small made-up depth images."""

import numpy as np
from table_assembly.perception.pixels import Intrinsics, back_project, flying_pixels


def leg_on_floor():
    """A 20x20 depth image: floor at 0.60 m, a leg's top at 0.57 m in the middle."""
    depth = np.full((20, 20), 0.60)
    leg = np.zeros((20, 20), dtype=bool)
    leg[8:12, 4:16] = True
    depth[leg] = 0.57
    return depth, leg


def test_the_edges_of_a_part_are_kept():
    # Shaving the edge off every part is what makes parts measure small. The
    # floor next to the leg is not in the leg's mask, so it must not count.
    depth, leg = leg_on_floor()
    assert not flying_pixels(depth, leg).any()


def test_a_pixel_hanging_between_a_part_and_what_is_behind_it_is_dropped():
    # The top edge of the leaning board, seen with the floor far behind it.
    depth, board = leg_on_floor()
    depth[~board] = 1.2
    depth[8, 10] = 0.9  # a range half way between the two
    assert flying_pixels(depth, board)[8, 10]


def test_a_pixel_a_centimetre_off_is_left_alone():
    # Half way down a leg's side is still on the leg, near enough: dropping it
    # would cost more than keeping it.
    depth, leg = leg_on_floor()
    depth[8, 10] = 0.585
    assert not flying_pixels(depth, leg)[8, 10]


def test_a_surface_seen_at_a_slant_is_not_mistaken_for_flying_pixels():
    # Depth climbing 6 mm a pixel, as a floor does when seen from far off.
    depth = np.tile(np.linspace(1.5, 1.614, 20), (20, 1))
    assert not flying_pixels(depth, np.ones((20, 20), dtype=bool)).any()


def test_a_pixel_back_projects_to_where_the_camera_saw_it():
    depth = np.full((3, 3), 2.0)
    mask = np.zeros((3, 3), dtype=bool)
    mask[1, 2] = True
    camera = Intrinsics(fx=100.0, fy=100.0, cx=1.0, cy=1.0)
    lifted = np.eye(4)
    lifted[2, 3] = 1.0
    # One pixel right of centre, two metres out: 2 cm right, 2 m along z.
    assert np.allclose(back_project(depth, mask, camera, lifted), [[0.02, 0.0, 3.0]])
