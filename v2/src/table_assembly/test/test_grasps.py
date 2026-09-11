"""Where the tool goes to pick parts up and put them down.

These check the geometry the arm relies on but cannot see: that a leg set
down really is upright on its spot, that the top laid down really is level,
and that the finger under the top misses the stands.
"""

import numpy as np
import pytest
from synthetic import spawned_box
from table_assembly.arm.dimensions import FINGERTIP_OFFSET
from table_assembly.assembly.grasps import (
    LEG_GRIP_DEPTH,
    TOP_INSERTION,
    carried_tool_pose,
    carry_round,
    hold,
    leg_pick_poses,
    leg_place_pose,
    level_top_pose,
    top_pick_poses,
)
from table_assembly.geometry import Box
from table_assembly.transforms import WORLD_Z, frame, rotation_z
from table_assembly.world.spawn import random_room

BASE = np.zeros(3)
LEG = Box.upright((0.1, -0.45, 0.076), 0.6, (0.03, 0.03, 0.15))
OUT = np.array([1.0, 0.0, 0.0])
SPOT = np.array([0.6, 0.1, 0.0])


def test_the_tool_goes_where_the_hold_says_for_any_pose_of_the_part():
    tool = frame(np.array([0.4, 0.1, 0.3]), rotation_z(0.7))
    part = Box(np.array([0.5, 0.0, 0.1]), rotation_z(-0.2), (0.1, 0.1, 0.1))
    held = hold(part, tool)
    # Put the part back where it was, with the tool as it was: same tool pose.
    assert carried_tool_pose(held, tool[:3, :3], part.centre) == pytest.approx(tool)


@pytest.mark.parametrize("pick", leg_pick_poses(LEG))
def test_a_standing_leg_is_gripped_from_above_by_its_top_end(pick):
    assert pick[:3, 2] == pytest.approx(-WORLD_Z)
    # The fingers close along the tool's y axis, square to a side of the leg.
    assert max(abs(float(pick[:3, 1] @ LEG.axis(i))) for i in (0, 1)) == pytest.approx(1.0)
    fingertips = pick[:3, 3] + pick[:3, 2] * FINGERTIP_OFFSET
    assert fingertips[:2] == pytest.approx(LEG.centre[:2])
    assert fingertips[2] == pytest.approx(LEG.top_z - LEG_GRIP_DEPTH)


@pytest.mark.parametrize("pick", leg_pick_poses(LEG))
def test_a_leg_set_down_stands_upright_on_its_spot(pick):
    held = hold(LEG, pick)
    place = leg_place_pose(held, pick, SPOT, 0.15, 0.003, BASE)
    leg = place @ np.linalg.inv(held)
    assert leg[:3, 2] == pytest.approx(WORLD_Z)
    assert leg[:3, 3] == pytest.approx(SPOT + WORLD_Z * 0.078)
    # Still pointing straight down, turned with the arm's base.
    assert place[:3, 2] == pytest.approx(-WORLD_Z)
    turn = np.arctan2(SPOT[1], SPOT[0]) - np.arctan2(pick[1, 3], pick[0, 3])
    assert place[:3, :3] == pytest.approx(rotation_z(turn) @ pick[:3, :3])


@pytest.mark.parametrize("pick", leg_pick_poses(LEG))
def test_a_leg_is_carried_upright_round_the_base_and_ends_over_its_spot(pick):
    held = hold(LEG, pick)
    place = leg_place_pose(held, pick, SPOT, 0.15, 0.003, BASE)
    poses = carry_round(held, pick, place, 0.3, BASE)
    for pose in poses:
        leg = pose @ np.linalg.inv(held)
        # Upright the whole way, so its weight never twists it in the fingers.
        assert leg[:3, 2] == pytest.approx(WORLD_Z)
        assert leg[2, 3] == pytest.approx(0.3)
        # Never cutting in close to the base.
        assert (
            np.linalg.norm(leg[:2, 3]) >= min(np.linalg.norm(LEG.centre[:2]), np.linalg.norm(SPOT[:2])) - 1e-9
        )
    # It starts straight above where it stood and ends straight above its spot.
    assert poses[0][:2, 3] == pytest.approx(pick[:2, 3])
    assert poses[-1][:2, 3] == pytest.approx(place[:2, 3])
    assert poses[-1][:3, :3] == pytest.approx(place[:3, :3])


TOP = Box.upright((0.05, 0.57, 0.094), 1.62, (0.28, 0.18, 0.018))


@pytest.mark.parametrize("pick", top_pick_poses(TOP, BASE))
def test_the_top_is_gripped_level_across_its_near_edge(pick):
    # Reaching away from the arm, straight in across the long edge.
    assert pick[:3, 2] == pytest.approx(TOP.axis(1) * np.sign(TOP.axis(1) @ TOP.centre))
    assert abs(float(pick[:3, 1] @ WORLD_Z)) == pytest.approx(1.0)
    fingertips = pick[:3, 3] + pick[:3, 2] * FINGERTIP_OFFSET
    near_edge = TOP.centre - pick[:3, 2] * 0.09
    assert fingertips == pytest.approx(near_edge + pick[:3, 2] * TOP_INSERTION)


@pytest.mark.parametrize("pick", top_pick_poses(TOP, BASE))
def test_the_top_laid_down_is_level_and_where_it_was_asked(pick):
    held = hold(TOP, pick)
    centre = np.array([0.55, 0.0, 0.18])
    place = level_top_pose(held, centre, OUT, pick[:3, 1])
    top_pose = place @ np.linalg.inv(held)
    # Level and the same way up as it was picked up.
    assert top_pose[:3, 2] == pytest.approx(WORLD_Z)
    assert top_pose[:3, 3] == pytest.approx(centre)
    # The tool reaches away from the arm into the board, from its near edge.
    assert place[:3, 2] == pytest.approx(OUT)
    assert place[0, 3] < centre[0] - 0.09


@pytest.mark.parametrize("pick", top_pick_poses(TOP, BASE))
def test_the_top_is_carried_level_round_the_base(pick):
    held = hold(TOP, pick)
    lifted = pick.copy()
    lifted[2, 3] += 0.08
    place = level_top_pose(held, np.array([0.55, 0.0, 0.18]), OUT, pick[:3, 1])
    poses = carry_round(held, lifted, place, 0.26, BASE)
    for pose in poses:
        top = pose @ np.linalg.inv(held)
        assert top[:3, 2] == pytest.approx(WORLD_Z)
        assert top[2, 3] == pytest.approx(0.26)
    assert poses[-1][:3, :3] == pytest.approx(place[:3, :3])


@pytest.mark.parametrize("seed", range(1, 21))
def test_the_finger_under_the_top_misses_the_stands_and_the_floor(seed):
    room = random_room(seed)
    top = spawned_box(room.top)
    for pick in top_pick_poses(top, BASE):
        # The lower finger runs in under the board from its near edge, 3 cm
        # wide, a centimetre and a half below it with the fingers open.
        tip = pick[:3, 3] + pick[:3, 2] * FINGERTIP_OFFSET
        for spawned in room.stands:
            stand = spawned_box(spawned)
            sideways = abs(float((stand.centre - tip) @ pick[:3, 0]))
            assert sideways - stand.size[0] / 2.0 > 0.015 + 0.01
        # And the gripper body, 11 cm tall across the way the fingers close,
        # stays above the floor.
        assert pick[2, 3] - 0.055 > 0.02
