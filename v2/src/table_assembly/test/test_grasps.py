"""Where the tool goes to pick parts up and put them down.

These check the geometry the arm relies on but cannot see: that a leg stood
up really is upright on its spot, that the top laid down
really is level, and that the finger on the wall's side of the top clears the
wall.
"""

import math

import numpy as np
import pytest
from synthetic import leaning_board, spawned_box
from table_assembly.arm.dimensions import FINGERTIP_OFFSET
from table_assembly.assembly.grasps import (
    FINGER_FLOOR_CLEARANCE,
    TOP_INSERTION,
    carried_tool_pose,
    carry_upright,
    hold,
    leg_pick_poses,
    level_top_poses,
    standing_leg_poses,
    top_pick_poses,
    turned_upright,
)
from table_assembly.geometry import Box
from table_assembly.transforms import WORLD_Z, frame, rotation_z
from table_assembly.world.spawn import random_room

LEG = Box.upright((0.4, -0.35, 0.015), 0.6, (0.14, 0.03, 0.03))
OUT = np.array([1.0, 0.0, 0.0])
SPOT = np.array([0.6, 0.1, 0.0])


def test_the_tool_goes_where_the_hold_says_for_any_pose_of_the_part():
    tool = frame(np.array([0.4, 0.1, 0.3]), rotation_z(0.7))
    part = Box(np.array([0.5, 0.0, 0.1]), rotation_z(-0.2), (0.1, 0.1, 0.1))
    held = hold(part, tool)
    # Put the part back where it was, with the tool as it was: same tool pose.
    assert carried_tool_pose(held, tool[:3, :3], part.centre) == pytest.approx(tool)


@pytest.mark.parametrize("pose", leg_pick_poses(LEG, 0.0))
def test_a_lying_leg_is_gripped_from_above_across_its_middle(pose):
    assert pose[:3, 2] == pytest.approx(-WORLD_Z)
    # The fingers close along the tool's y axis, which is the leg's short side.
    assert abs(float(pose[:3, 1] @ LEG.axis(1))) == pytest.approx(1.0)
    fingertips = pose[:3, 3] + pose[:3, 2] * FINGERTIP_OFFSET
    assert fingertips[2] == pytest.approx(FINGER_FLOOR_CLEARANCE)
    # Right over its centre of mass, so its weight cannot twist it in the grip.
    assert pose[:2, 3] == pytest.approx(LEG.centre[:2])


@pytest.mark.parametrize("pick", leg_pick_poses(LEG, 0.0))
def test_a_leg_stood_up_is_upright_on_its_spot(pick):
    held = hold(LEG, pick)
    for place in standing_leg_poses(held, SPOT, 0.14, 0.003, OUT):
        leg_pose = place @ np.linalg.inv(held)
        assert abs(float(leg_pose[2, 0])) == pytest.approx(1.0)
        assert leg_pose[:3, 3] == pytest.approx(SPOT + WORLD_Z * 0.073)
        # The tool is level, halfway up the leg, with the camera side up.
        assert place[2, 2] == pytest.approx(0.0, abs=1e-9)
        assert place[2, 3] == pytest.approx(0.073)
        assert place[:3, 0] == pytest.approx(WORLD_Z)


def test_standing_a_leg_up_from_the_arm_side_is_tried_first():
    held = hold(LEG, leg_pick_poses(LEG, 0.0)[0])
    first = standing_leg_poses(held, SPOT, 0.14, 0.003, OUT)[0]
    assert first[:3, 2] == pytest.approx(OUT)


TOP = leaning_board(0.30, 0.18, 0.018, math.radians(18.0), yaw=0.0, centre=(0.5, 0.0, 0.09))


@pytest.mark.parametrize("pick", top_pick_poses(TOP))
def test_the_top_is_gripped_on_its_upper_edge_across_its_thickness(pick):
    up = TOP.axis(1)
    assert pick[:3, 2] == pytest.approx(-up)
    assert abs(float(pick[:3, 1] @ TOP.axis(2))) == pytest.approx(1.0)
    fingertips = pick[:3, 3] + pick[:3, 2] * FINGERTIP_OFFSET
    edge = TOP.centre + up * 0.09
    assert fingertips == pytest.approx(edge - up * TOP_INSERTION)


@pytest.mark.parametrize("pick", top_pick_poses(TOP))
def test_the_top_laid_down_is_level_and_where_it_was_asked(pick):
    held = hold(TOP, pick)
    centre = np.array([0.55, 0.1, 0.14])
    for place in level_top_poses(held, centre, OUT):
        top_pose = place @ np.linalg.inv(held)
        assert abs(float(top_pose[2, 2])) == pytest.approx(1.0)
        assert top_pose[:3, 3] == pytest.approx(centre)
        # The tool reaches away from the arm into the board, from its near edge.
        assert place[:3, 2] == pytest.approx(OUT)
        assert place[0, 3] < centre[0] - 0.09


@pytest.mark.parametrize("seed", range(1, 21))
def test_the_finger_behind_the_top_clears_the_wall(seed):
    room = random_room(seed)
    top, wall = spawned_box(room.top), spawned_box(room.wall)
    # The fingertip on the wall's side, with the fingers open a centimetre
    # and a half either side of the board, and 12 mm thick.
    pick = top_pick_poses(top)[0]
    tip = pick[:3, 3] + pick[:3, 2] * FINGERTIP_OFFSET
    towards_wall = -top.axis(2) if float(top.axis(2) @ (wall.centre - top.centre)) < 0 else top.axis(2)
    finger_back = tip + towards_wall * (top.size[2] / 2 + 0.015 + 0.012)
    assert finger_back[2] > wall.top_z + 0.01


@pytest.mark.parametrize("reach", [None, np.array([1.0, 0.0, 0.0]), np.array([-0.6, 0.8, 0.0])])
@pytest.mark.parametrize("pick", leg_pick_poses(LEG, 0.0))
def test_a_leg_is_turned_upright_about_its_own_middle(pick, reach):
    held = hold(LEG, pick)
    lifted = pick.copy()
    lifted[2, 3] += 0.1
    turn = turned_upright(held, lifted, reach)
    start_centre = (lifted @ np.linalg.inv(held))[:3, 3]
    for pose in turn:
        # The leg's middle stays put all the way round.
        assert (pose @ np.linalg.inv(held))[:3, 3] == pytest.approx(start_centre)
    upright = turn[-1] @ np.linalg.inv(held)
    assert abs(float(upright[2, 0])) == pytest.approx(1.0)
    # And the camera side of the tool ends up on top, pointing where asked.
    assert turn[-1][:3, 0] == pytest.approx(WORLD_Z)
    if reach is not None:
        assert turn[-1][:3, 2] == pytest.approx(reach / np.linalg.norm(reach))


@pytest.mark.parametrize("pick", leg_pick_poses(LEG, 0.0))
def test_a_leg_is_carried_upright_round_the_base_and_ends_over_its_spot(pick):
    held = hold(LEG, pick)
    start = np.array([0.0, -0.45, 0.15])
    end = np.array([0.6, 0.1, 0.07])
    # It sets off with the gripper reaching back towards the base.
    poses = carry_upright(held, start, np.array([0.0, 1.0, 0.0]), end, 0.3, np.zeros(3))
    for pose in poses:
        leg = pose @ np.linalg.inv(held)
        # Upright the whole way, so its weight never twists it in the fingers.
        assert abs(float(leg[2, 0])) == pytest.approx(1.0)
        assert leg[2, 3] == pytest.approx(0.3)
        # Never cutting in close to the base.
        assert np.linalg.norm(leg[:2, 3]) >= 0.45 - 1e-9
    assert (poses[-1] @ np.linalg.inv(held))[:2, 3] == pytest.approx(end[:2])
    # And it arrives reaching straight out from the base, the way legs are stood up.
    assert poses[-1][:3, 2] == pytest.approx(np.array([0.6, 0.1, 0.0]) / np.linalg.norm([0.6, 0.1]))
