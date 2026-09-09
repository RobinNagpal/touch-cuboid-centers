"""Deciding what cuboids to put on the table, and where.

Nothing about any one cuboid is fixed. Its sides, colour, position, and which
way up it is sitting are all drawn fresh for every run, so the same code has to
cope with a different table each time. A seed makes any one of those tables
repeatable, which is what makes a failure worth reporting: the run that
produced it can be run again.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path

from ..arm.dimensions import MAX_GRASP_WIDTH
from ..table.layout import PENDING_ZONE, TABLE_TOP_Z
from .spec import (
    CUBOID_MAX_SIDE,
    CUBOID_MIN_SIDE,
    DENSITY,
    MIN_SEPARATION,
    PALETTE,
)

TEMPLATE = Path(__file__).parent / "cuboid.sdf"


@dataclass(frozen=True)
class SpawnedCuboid:
    """One cuboid, as the simulator will be told to create it."""

    name: str
    size: tuple[float, float, float]
    position: tuple[float, float, float]
    yaw: float
    colour: tuple[float, float, float]


def random_cuboids(count: int, seed: int) -> list[SpawnedCuboid]:
    """Lay out ``count`` cuboids in the pending zone."""
    rng = random.Random(seed)
    x_min, x_max, y_min, y_max = PENDING_ZONE

    placed: list[SpawnedCuboid] = []
    for index in range(count):
        size = _graspable_size(rng)
        position = _free_spot(rng, placed, x_min, x_max, y_min, y_max, size[2])
        placed.append(
            SpawnedCuboid(
                name=f"cuboid_{index}",
                size=size,
                position=position,
                yaw=rng.uniform(-math.pi / 2, math.pi / 2),
                colour=PALETTE[index % len(PALETTE)],
            )
        )
    return placed


def _graspable_size(rng: random.Random) -> tuple[float, float, float]:
    """Three sides, with a random one of them standing vertical.

    Which side ends up vertical is what decides whether the biggest face is
    the one on top or one of the sides, so it is drawn at random rather than
    always resting the box on its largest face.

    The one thing that is not left to chance is that the arm must be able to
    pick the box up, so a draw where both horizontal sides are too wide for
    the gripper is thrown away.
    """
    while True:
        sides = [rng.uniform(CUBOID_MIN_SIDE, CUBOID_MAX_SIDE) for _ in range(3)]
        rng.shuffle(sides)
        if min(sides[0], sides[1]) <= MAX_GRASP_WIDTH:
            return (sides[0], sides[1], sides[2])


def _free_spot(rng, placed, x_min, x_max, y_min, y_max, height) -> tuple[float, float, float]:
    """A resting place far enough from every box already put down."""
    for _ in range(500):
        x = rng.uniform(x_min, x_max)
        y = rng.uniform(y_min, y_max)
        if all(math.dist((x, y), box.position[:2]) >= MIN_SEPARATION for box in placed):
            return (x, y, TABLE_TOP_Z + height / 2)
    raise RuntimeError(
        f"could not fit {len(placed) + 1} cuboids in the pending zone without them "
        f"touching; try fewer, or widen PENDING_ZONE in table/layout.py"
    )


def cuboid_sdf(cuboid: SpawnedCuboid) -> str:
    """One cuboid as the simulator's own model format."""
    length, width, height = cuboid.size
    mass = DENSITY * length * width * height
    red, green, blue = cuboid.colour
    x, y, z = cuboid.position

    return (
        TEMPLATE.read_text()
        .split("-->\n", 1)[1]
        .format(
            name=cuboid.name,
            x=x,
            y=y,
            z=z,
            yaw=cuboid.yaw,
            mass=mass,
            # A solid box, about its own centre.
            ixx=mass * (width**2 + height**2) / 12.0,
            iyy=mass * (length**2 + height**2) / 12.0,
            izz=mass * (length**2 + width**2) / 12.0,
            length=length,
            width=width,
            height=height,
            red=red,
            green=green,
            blue=blue,
        )
    )
