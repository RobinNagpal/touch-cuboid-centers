"""The simulated room, and the line between it and the robot.

The first tests check the spawner lays out a room that can physically stand
up. The last one checks the robot's code never reads the spawner's numbers,
which is the whole point of measuring.
"""

import ast
from pathlib import Path

import numpy as np
import pytest
from synthetic import spawned_box
from table_assembly.world import spec
from table_assembly.world.spawn import box_sdf, random_room

PACKAGE = Path(__file__).resolve().parents[1] / "table_assembly"


def test_the_same_seed_gives_the_same_room():
    a, b = random_room(5), random_room(5)
    assert a.top.size == b.top.size
    assert a.legs[2].centre == pytest.approx(b.legs[2].centre)


@pytest.mark.parametrize("seed", range(1, 31))
def test_the_top_lies_level_on_both_stands(seed):
    room = random_room(seed)
    top = spawned_box(room.top)
    assert top.axis(2) == pytest.approx([0.0, 0.0, 1.0])
    assert len(room.stands) == 2
    for spawned in room.stands:
        stand = spawned_box(spawned)
        assert stand.bottom_z == pytest.approx(0.0, abs=1e-9)
        # The top sits on it, give or take the millimetre of air.
        assert top.bottom_z == pytest.approx(stand.top_z + 0.001, abs=1e-9)
        # And the stand is under the top all round, so it cannot tip.
        assert top.footprint_distance(stand.corners()[:, :2]).max() == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("seed", range(1, 31))
def test_the_middle_of_the_top_is_open_underneath(seed):
    # The arm grips the top by the middle of its near edge with one finger
    # under the board, so the stands have to leave room there for a finger.
    room = random_room(seed)
    top = spawned_box(room.top)
    for spawned in room.stands:
        along = abs(float((spawned_box(spawned).centre - top.centre) @ top.axis(0)))
        assert along - spec.STAND_WIDTH / 2.0 > 0.05


@pytest.mark.parametrize("seed", range(1, 31))
def test_the_legs_stand_upright_and_well_apart(seed):
    legs = random_room(seed).legs
    for leg in legs:
        box = spawned_box(leg)
        assert box.bottom_z == pytest.approx(0.001, abs=1e-9)
        assert box.size[2] > box.size[0]
    for i, a in enumerate(legs):
        for b in legs[i + 1 :]:
            assert np.linalg.norm(a.centre[:2] - b.centre[:2]) >= spec.LEG_SPACING


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
