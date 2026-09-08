"""Building the Gazebo world the run starts from.

The table and the lighting are fixed and live in ``worlds/cell.sdf``. The
cuboids are not: their sizes, colours, positions and which way up they are
sitting are drawn fresh for every run, so the same code has to cope with a
different table each time. A seed makes any one of those tables repeatable.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .cell import (
    CUBOID_MAX_SIDE,
    CUBOID_MIN_SIDE,
    MAX_GRASP_WIDTH,
    PENDING_ZONE,
    TABLE_TOP_Z,
)

# Saturated, well separated colours. The perception step tells a cuboid from
# the table by how colourful it is, so nothing here may be grey.
PALETTE = (
    ("red", (0.80, 0.12, 0.12)),
    ("green", (0.10, 0.65, 0.20)),
    ("blue", (0.12, 0.28, 0.80)),
    ("yellow", (0.85, 0.72, 0.08)),
    ("magenta", (0.72, 0.12, 0.66)),
    ("cyan", (0.06, 0.62, 0.70)),
)

DENSITY = 700.0  # kg/m^3, about right for a block of oak
MIN_SEPARATION = 0.13  # centre to centre, so the boxes never touch each other


@dataclass(frozen=True)
class SpawnedCuboid:
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
                colour=PALETTE[index % len(PALETTE)][1],
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
        f"touching; try fewer, or widen PENDING_ZONE in cell.py"
    )


def cuboid_sdf(cuboid: SpawnedCuboid) -> str:
    length, width, height = cuboid.size
    mass = DENSITY * length * width * height
    ixx = mass * (width**2 + height**2) / 12.0
    iyy = mass * (length**2 + height**2) / 12.0
    izz = mass * (length**2 + width**2) / 12.0
    red, green, blue = cuboid.colour
    x, y, z = cuboid.position

    return f"""
    <model name="{cuboid.name}">
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {cuboid.yaw:.4f}</pose>
      <link name="body">
        <inertial>
          <mass>{mass:.4f}</mass>
          <inertia>
            <ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>{length:.4f} {width:.4f} {height:.4f}</size></box></geometry>
          <surface>
            <friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>
            <contact><ode><kp>1e6</kp><kd>100</kd></ode></contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.4f} {width:.4f} {height:.4f}</size></box></geometry>
          <material>
            <ambient>{red:.3f} {green:.3f} {blue:.3f} 1</ambient>
            <diffuse>{red:.3f} {green:.3f} {blue:.3f} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def build_world(template: str, cuboids: list[SpawnedCuboid]) -> str:
    """Drop the cuboids into the world template."""
    marker = "<!-- CUBOIDS -->"
    if marker not in template:
        raise ValueError(f"the world template has no {marker} line to fill in")
    return template.replace(marker, "".join(cuboid_sdf(c) for c in cuboids))
