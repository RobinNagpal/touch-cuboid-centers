import math

import numpy as np
import pytest
from synthetic import spawned_box
from table_assembly.assembly.plan import LEG_INSET, SITE, in_the_way, plan_table, table_footprint
from table_assembly.geometry import Box
from table_assembly.world.spawn import random_room

BASE = np.zeros(3)
TOP = Box.upright((0.0, 0.55, 0.1), 0.1, (0.30, 0.18, 0.018))


def test_the_table_is_as_long_as_the_top_edge_facing_the_arm():
    # The top is gripped by a long edge, which becomes the near side of the table.
    assert table_footprint(TOP) == pytest.approx((0.30, 0.18))


def test_every_leg_stands_under_the_top_just_in_from_its_corners():
    plan = plan_table(TOP, 0.15, 0.03, 0.0, np.array([0.55, 0.0]), BASE)
    assert len(plan.leg_spots) == 4
    for spot in plan.leg_spots:
        # In the table's own frame, every leg is inset from both edges.
        local = np.abs((spot - plan.top.centre) @ plan.top.rotation)
        assert local[0] == pytest.approx(0.15 - LEG_INSET - 0.015)
        assert local[1] == pytest.approx(0.09 - LEG_INSET - 0.015)
        assert spot[2] == 0.0


def test_the_far_legs_go_in_first():
    plan = plan_table(TOP, 0.15, 0.03, 0.0, np.array([0.55, 0.0]), BASE)
    distances = [np.linalg.norm(spot[:2]) for spot in plan.leg_spots]
    assert distances == sorted(distances, reverse=True)


def test_the_top_sits_on_the_legs_level_and_square_to_the_arm():
    plan = plan_table(TOP, 0.15, 0.03, 0.01, np.array([0.4, 0.4]), BASE)
    assert plan.top.centre[2] == pytest.approx(0.01 + 0.15 + 0.009)
    assert plan.top.axis(2) == pytest.approx([0, 0, 1])
    assert plan.outward == pytest.approx([math.sqrt(0.5), math.sqrt(0.5), 0])
    # The table's long side runs across the arm's line of sight.
    assert abs(float(plan.top.axis(0) @ plan.outward)) == pytest.approx(0.0, abs=1e-9)


def test_something_on_the_spot_is_in_the_way_and_something_well_off_it_is_not():
    plan = plan_table(TOP, 0.15, 0.03, 0.0, SITE, BASE)
    on_it = Box.upright((SITE[0] + 0.05, SITE[1], 0.015), 0.4, (0.15, 0.03, 0.03))
    beside_it = Box.upright((SITE[0], SITE[1] - 0.20, 0.015), 0.0, (0.15, 0.03, 0.03))
    far_off = Box.upright((0.0, -0.5, 0.015), 0.0, (0.15, 0.03, 0.03))
    assert in_the_way(plan, [on_it, beside_it, far_off]) == [on_it, beside_it]


@pytest.mark.parametrize("seed", range(1, 31))
def test_every_room_leaves_the_building_spot_clear(seed):
    # The arm always builds in the same place and refuses to if anything is
    # there, so the simulator must never put anything there.
    room = random_room(seed)
    top = spawned_box(room.top)
    plan = plan_table(top, room.legs[0].size[2], room.legs[0].size[0], 0.0, SITE, BASE)
    assert not in_the_way(plan, [spawned_box(box) for box in room.boxes()])
