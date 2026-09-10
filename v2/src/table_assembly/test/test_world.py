"""The simulated room, and the line between it and the robot.

The first tests check the spawner lays out a room that can physically stand
up. The last one checks the robot's code never reads the spawner's numbers,
which is the whole point of measuring.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest
from synthetic import spawned_box
from table_assembly.world import spec
from table_assembly.world.spawn import _gap, box_sdf, random_room

PACKAGE = Path(__file__).resolve().parents[1] / "table_assembly"


def test_the_same_seed_gives_the_same_room():
    a, b = random_room(5), random_room(5)
    assert a.top.size == b.top.size
    assert a.legs[2].centre == pytest.approx(b.legs[2].centre)


@pytest.mark.parametrize("seed", range(1, 31))
def test_the_top_rests_on_the_floor_and_against_the_wall(seed):
    room = random_room(seed)
    top, wall = spawned_box(room.top), spawned_box(room.wall)
    corners = top.corners()
    # The lowest corner is on the floor, give or take the millimetre of air.
    assert corners[:, 2].min() == pytest.approx(0.001, abs=1e-6)
    # The back face passes through the wall's top front corner.
    back_normal = top.axis(2)
    wall_corner_height = wall.top_z
    back_face_point = top.centre + back_normal * top.size[2] / 2
    outward = wall.axis(0)
    front_of_wall = wall.centre - outward * wall.size[0] / 2
    corner = front_of_wall.copy()
    corner[2] = wall_corner_height
    corner += np.array([0.0, 0.0, 0.001])
    assert float((corner - back_face_point) @ back_normal) == pytest.approx(0.0, abs=1e-6)
    assert spec.TOP_LEAN_DEG[0] <= math.degrees(room.lean) <= spec.TOP_LEAN_DEG[1]


@pytest.mark.parametrize("seed", range(1, 31))
def test_the_legs_never_touch(seed):
    legs = random_room(seed).legs
    for i, a in enumerate(legs):
        for b in legs[i + 1 :]:
            assert _gap(a, b) >= spec.LEG_GAP


def test_every_box_becomes_a_model():
    room = random_room(1)
    for box in room.boxes():
        sdf = box_sdf(box)
        assert f'<model name="{box.name}">' in sdf
        assert ("<static>true</static>" in sdf) == (box.density == 0.0)


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text())
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            found.add("." * node.level + (node.module or ""))
        elif isinstance(node, ast.Import):
            found.update(alias.name for alias in node.names)
    return found


@pytest.mark.parametrize(
    "path",
    [p for p in PACKAGE.rglob("*.py") if "world" not in p.relative_to(PACKAGE).parts],
    ids=lambda p: str(p.relative_to(PACKAGE)),
)
def test_the_robot_never_reads_the_simulators_numbers(path):
    """Nothing outside world/ may import from it.

    world/ is where the sizes and positions of everything in the room are
    decided. If the robot's code could read them, it would not need to
    measure anything.
    """
    for name in _imports(path):
        assert "world" not in name.split("."), f"{path.name} imports {name}"
