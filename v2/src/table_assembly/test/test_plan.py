import math

import numpy as np
import pytest
from synthetic import leaning_board
from table_assembly.assembly.plan import LEG_INSET, choose_site, plan_table, table_footprint
from table_assembly.geometry import Box

BASE = np.zeros(3)
TOP = leaning_board(0.30, 0.18, 0.018, math.radians(18.0))


def test_the_table_is_as_long_as_the_top_edge_facing_the_arm():
    # The top stands on its long edge and is gripped by the opposite long
    # edge, which becomes the near side of the table.
    assert table_footprint(TOP) == pytest.approx((0.30, 0.18))


def test_every_leg_stands_under_the_top_just_in_from_its_corners():
    plan = plan_table(TOP, 0.12, 0.03, 0.0, np.array([0.55, 0.0]), BASE)
    assert len(plan.leg_spots) == 4
    for spot in plan.leg_spots:
        # In the table's own frame, every leg is inset from both edges.
        local = np.abs((spot - plan.top.centre) @ plan.top.rotation)
        assert local[0] == pytest.approx(0.15 - LEG_INSET - 0.015)
        assert local[1] == pytest.approx(0.09 - LEG_INSET - 0.015)
        assert spot[2] == 0.0


def test_the_far_legs_go_in_first():
    plan = plan_table(TOP, 0.12, 0.03, 0.0, np.array([0.55, 0.0]), BASE)
    distances = [np.linalg.norm(spot[:2]) for spot in plan.leg_spots]
    assert distances == sorted(distances, reverse=True)


def test_the_top_sits_on_the_legs_level_and_square_to_the_arm():
    plan = plan_table(TOP, 0.12, 0.03, 0.01, np.array([0.4, 0.4]), BASE)
    assert plan.top.centre[2] == pytest.approx(0.01 + 0.12 + 0.009)
    assert plan.top.axis(2) == pytest.approx([0, 0, 1])
    assert plan.outward == pytest.approx([math.sqrt(0.5), math.sqrt(0.5), 0])
    # The table's long side runs across the arm's line of sight.
    assert abs(float(plan.top.axis(0) @ plan.outward)) == pytest.approx(0.0, abs=1e-9)


def test_an_empty_floor_puts_the_table_straight_in_front_at_the_preferred_reach():
    centre = choose_site((0.3, 0.18), [], BASE, radii=(0.5, 0.55, 0.6), preferred_radius=0.55)
    assert centre == pytest.approx([0.55, 0.0])


def test_the_table_is_built_clear_of_whatever_is_on_the_floor():
    in_front = Box.upright((0.55, 0.0, 0.05), 0.0, (0.3, 0.3, 0.1))
    centre = choose_site((0.3, 0.18), [in_front], BASE, radii=(0.5, 0.55, 0.6), preferred_radius=0.55)
    assert centre is not None
    site = Box.upright(
        (centre[0], centre[1], 0.0), math.atan2(centre[1], centre[0]) + math.pi / 2, (0.3, 0.18, 0)
    )
    corners = in_front.corners()[:, :2]
    assert site.footprint_distance(corners).min() >= 0.12 - 1e-9


def test_no_room_anywhere_is_reported_rather_than_squeezed():
    ring = [
        Box.upright((0.55 * math.cos(a), 0.55 * math.sin(a), 0.05), a, (0.2, 0.2, 0.1))
        for a in np.radians(range(0, 360, 20))
    ]
    assert choose_site((0.3, 0.18), ring, BASE, radii=(0.5, 0.55, 0.6), preferred_radius=0.55) is None
