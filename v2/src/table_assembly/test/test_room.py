"""Reading a whole room, as the simulator would lay it out, from points alone.

The test is the only place the robot's measurements and the simulator's truth
meet: the room is drawn by the spawner, turned into the points a camera
circling the arm would see, and what the robot reads from those points is
compared with what the spawner put there.
"""

import math

import numpy as np
import pytest
from synthetic import floor_points, spawned_box, visible_surface
from table_assembly.perception.room import is_standing, leg_length, read_room
from table_assembly.world.spawn import random_room

BASE = np.zeros(3)
# Where the survey puts the camera, near enough: a ring round the base.
CAMERAS = [np.array([0.3 * math.cos(a), 0.3 * math.sin(a), 0.6]) for a in np.radians(range(0, 360, 45))]


def survey(room):
    """The parts' points and everything else's, as a survey would see them."""
    parts = []
    for spawned in (room.top, *room.legs):
        box = spawned_box(spawned)
        camera = min(CAMERAS, key=lambda c: np.linalg.norm(c - box.centre))
        parts.append(visible_surface(box, camera))
    others = [floor_points()]
    for spawned in room.stands:
        stand = spawned_box(spawned)
        others.append(visible_surface(stand, min(CAMERAS, key=lambda c: np.linalg.norm(c - stand.centre))))
    return np.concatenate(parts), np.concatenate(others)


@pytest.mark.parametrize("seed", [1, 2, 3, 7, 12])
def test_the_room_is_read_back_as_the_simulator_laid_it_out(seed):
    room = random_room(seed)
    seen = read_room(*survey(room), self_centre=BASE, self_radius=0.13)

    assert seen.floor_z == pytest.approx(0.0, abs=0.001)

    assert seen.top is not None
    assert seen.top.size == pytest.approx(np.array(room.top.size), abs=0.002)
    assert seen.top.centre == pytest.approx(room.top.centre, abs=0.002)

    assert len(seen.legs) == 4
    for leg in seen.legs:
        assert is_standing(leg)
        assert leg_length(leg) == pytest.approx(room.legs[0].size[2], abs=0.002)

    # Each stand is found as an obstacle: something grey standing up off the
    # floor. How tall is not asked: the top hides the upper part of it.
    assert len(seen.obstacles) == 2
    for spawned in room.stands:
        assert min(np.linalg.norm(box.centre[:2] - spawned.centre[:2]) for box in seen.obstacles) < 0.02
    assert not seen.unknown


def test_nothing_the_robot_sees_of_itself_counts():
    room = random_room(1)
    parts, others = survey(room)
    # A patch of grey right by the base, as the camera would see the arm's own base.
    own_base = np.array(
        [
            [0.05 * math.cos(a), 0.05 * math.sin(a), z]
            for a in np.linspace(0, 6, 200)
            for z in (0.05, 0.1, 0.15)
        ]
    )
    seen = read_room(parts, np.concatenate([others, own_base]), self_centre=BASE, self_radius=0.13)
    assert len(seen.obstacles) == 2
